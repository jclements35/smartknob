#include <Arduino.h>

#include "configuration.h"
#include "display_task.h"
#include "interface_task.h"
#include "motor_task.h"

Configuration config;

#if SK_DISPLAY
static DisplayTask display_task(0);
static DisplayTask* display_task_p = &display_task;
#else
static DisplayTask* display_task_p = nullptr;
#endif
static MotorTask motor_task(1, config);

InterfaceTask interface_task(0, motor_task, display_task_p);

void sensorTask(void* pvParameters){
  while (true) {
    // Read the analog and digital pins
    int analogValue1 = analogRead(PIN_JOYSTICK_XOUT);
    int analogValue2 = analogRead(PIN_JOYSTICK_YOUT);
    int digitalValue = digitalRead(PIN_JOYSTICK_BUTTON);

    // Output the values to the serial monitor (or handle via Protobuf)
    Serial.print("XOUT: ");
    Serial.print(analogValue1);
    Serial.print("\tYOUT: ");
    Serial.print(analogValue2);
    Serial.print("\tBUTTON: ");
    Serial.println(digitalValue);

    // Add a delay to prevent flooding the serial output
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Adjust delay as needed
  }
}

void initJoystickPins() {
    // Initialize analog pins as input
    pinMode(PIN_JOYSTICK_XOUT, INPUT);
    pinMode(PIN_JOYSTICK_YOUT, INPUT);

    // Initialize digital pin as input
    pinMode(PIN_JOYSTICK_BUTTON, INPUT_PULLUP);

    // Optionally, create a task or handler to read the sensor values
    // This could be done by creating a new task to handle reading the sensor data
    xTaskCreatePinnedToCore(sensorTask, "SensorTask", 2048, NULL, 1, NULL, 1);
}

void setup() {
  #if SK_DISPLAY
  display_task.setLogger(&interface_task);
  display_task.begin();

  // Connect display to motor_task's knob state feed
  motor_task.addListener(display_task.getKnobStateQueue());
  #endif

  interface_task.begin();

  config.setLogger(&interface_task);
  config.loadFromDisk();

  interface_task.setConfiguration(&config);

  motor_task.setLogger(&interface_task);
  motor_task.begin();

  initJoystickPins();

  // Free up the Arduino loop task
  vTaskDelete(NULL);
}

void loop() {
  // char buf[50];
  // static uint32_t last_stack_debug;
  // if (millis() - last_stack_debug > 1000) {
  //   interface_task.log("Stack high water:");
  //   snprintf(buf, sizeof(buf), "  main: %d", uxTaskGetStackHighWaterMark(NULL));
  //   interface_task.log(buf);
  //   #if SK_DISPLAY
  //     snprintf(buf, sizeof(buf), "  display: %d", uxTaskGetStackHighWaterMark(display_task.getHandle()));
  //     interface_task.log(buf);
  //   #endif
  //   snprintf(buf, sizeof(buf), "  motor: %d", uxTaskGetStackHighWaterMark(motor_task.getHandle()));
  //   interface_task.log(buf);
  //   snprintf(buf, sizeof(buf), "  interface: %d", uxTaskGetStackHighWaterMark(interface_task.getHandle()));
  //   interface_task.log(buf);
  //   snprintf(buf, sizeof(buf), "Heap -- free: %d, largest: %d", heap_caps_get_free_size(MALLOC_CAP_8BIT), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  //   interface_task.log(buf);
  //   last_stack_debug = millis();
  // }
}
