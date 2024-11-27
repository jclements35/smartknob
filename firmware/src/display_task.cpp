#if SK_DISPLAY
#include "display_task.h"
#include "semaphore_guard.h"
#include "util.h"

#include "font/roboto_light_60.h"

#include "display_task_arrays.cpp"
#include <cstdint>

static const uint8_t LEDC_CHANNEL_LCD_BACKLIGHT = 0;

DisplayTask::DisplayTask(const uint8_t task_core) : Task{"Display", 2048, 1, task_core} {
  knob_state_queue_ = xQueueCreate(1, sizeof(PB_SmartKnobState));
  assert(knob_state_queue_ != NULL);

  mutex_ = xSemaphoreCreateMutex();
  assert(mutex_ != NULL);
}

DisplayTask::~DisplayTask() {
  vQueueDelete(knob_state_queue_);
  vSemaphoreDelete(mutex_);
}

static void drawImage(TFT_eSprite& spr, const uint16_t *image, int width) {
  for (int i = 0; i < width*width; i++)
    spr.drawPixel(i % width, i/width, image[i]);
}

void DisplayTask::run() {
    tft_.begin();
    tft_.invertDisplay(1);
    tft_.setRotation(SK_DISPLAY_ROTATION);
    tft_.fillScreen(TFT_DARKGREEN);

    ledcSetup(LEDC_CHANNEL_LCD_BACKLIGHT, 5000, SK_BACKLIGHT_BIT_DEPTH);
    ledcAttachPin(PIN_LCD_BACKLIGHT, LEDC_CHANNEL_LCD_BACKLIGHT);
    ledcWrite(LEDC_CHANNEL_LCD_BACKLIGHT, (1 << SK_BACKLIGHT_BIT_DEPTH) - 1);

    spr_.setColorDepth(4);

    if (spr_.createSprite(TFT_WIDTH, TFT_HEIGHT) == nullptr) {
      log("ERROR: sprite allocation failed!");
      tft_.fillScreen(TFT_RED);
    } else {
      log("Sprite created!");
      tft_.fillScreen(TFT_BLACK);
    }
    spr_.setTextColor(0xFFFF, TFT_BLACK);
    
    PB_SmartKnobState state;

    while(1) {
        if (xQueueReceive(knob_state_queue_, &state, portMAX_DELAY) == pdFALSE) {
          continue;
        }

        if (strcmp(state.config.text,"Select Music Album") == 0){
          drawImage(spr_, image_music, arraySize);
        } else if (strcmp(state.config.text,"Control Music Volume") == 0){
          if (state.config.position == 0)
            drawImage(spr_, image_mute, arraySize);
          else if (state.config.position <= 3)
            drawImage(spr_, image_sound1, arraySize);
          else if (state.config.position <= 6)
            drawImage(spr_, image_sound2, arraySize);
          else if (state.config.position <= 9)
            drawImage(spr_, image_sound3, arraySize);
          else
            drawImage(spr_, image_sound4, arraySize);
        } else if (strcmp(state.config.text,"Navigate on Map") == 0){
          drawImage(spr_, image_location, arraySize);
        } else if (strcmp(state.config.text,"Social Media Task") == 0){
          drawImage(spr_, image_instagram, arraySize);
        }
    
        spr_.pushSprite((TFT_WIDTH-arraySize)/2, (TFT_WIDTH-arraySize)/2);

        {
          SemaphoreGuard lock(mutex_);
          ledcWrite(LEDC_CHANNEL_LCD_BACKLIGHT, brightness_);
        }
        delay(5);
    }
}

QueueHandle_t DisplayTask::getKnobStateQueue() {
  return knob_state_queue_;
}

void DisplayTask::setBrightness(uint16_t brightness) {
  SemaphoreGuard lock(mutex_);
  brightness_ = brightness >> (16 - SK_BACKLIGHT_BIT_DEPTH);
}

void DisplayTask::setLogger(Logger* logger) {
    logger_ = logger;
}

void DisplayTask::log(const char* msg) {
    if (logger_ != nullptr) {
        logger_->log(msg);
    }
}

#endif