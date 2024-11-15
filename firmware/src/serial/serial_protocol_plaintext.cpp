#include "../proto_gen/smartknob.pb.h"

#include "serial_protocol_plaintext.h"

int lastJoystickSerial = millis();
static int joystickSamplingRate_ms_slow = 1000;
static int joystickSamplingRate_ms_fast = 100;
static float deadzoneDecimal = .5;

void SerialProtocolPlaintext::handleState(const PB_SmartKnobState& state) {
    bool taskChange = strcmp(latest_state_.config.text,state.config.text) != 0;
    bool tickChange = latest_state_.current_position != state.current_position;
    bool CWChange = state.current_position - latest_state_.current_position < 0;
    //bool joystickChange = (abs(state.XOUT) > deadzoneDecimal || abs(state.YOUT) > deadzoneDecimal);
    latest_state_ = state; //Reset
    bool isMap = strcmp(latest_state_.config.text, "Navigate on Map")  == 0 ||
                 strcmp(latest_state_.config.text, "Social Media Task") == 0;  

    if (taskChange){
        stream_.printf("%s\n",latest_state_.config.text);
    } else {
        if (tickChange) {
            if (CWChange){
                stream_.printf("CW\n");
            } else {
                stream_.printf("CCW\n");
            }
        }
        if (isMap){
            if (millis() - lastJoystickSerial > joystickSamplingRate_ms_fast){
                stream_.printf("%.3f,%.3f\n",latest_state_.XOUT,latest_state_.YOUT);
                lastJoystickSerial = millis();
            }
        } else {
            if (millis() - lastJoystickSerial > joystickSamplingRate_ms_slow){
                stream_.printf("%.3f,%.3f\n",latest_state_.XOUT,latest_state_.YOUT);
                lastJoystickSerial = millis();
            }
        }
    }
}

void SerialProtocolPlaintext::log(const char* msg) {
    stream_.print("LOG: ");
    stream_.println(msg);
}

void SerialProtocolPlaintext::loop() {
    while (stream_.available() > 0) {
        int b = stream_.read();
        if (b == 0) {
            if (protocol_change_callback_) {
                protocol_change_callback_(SERIAL_PROTOCOL_PROTO);
            }
            break;
        }
        if (b == ' ') {
            if (demo_config_change_callback_) {
                demo_config_change_callback_();
            }
        } else if (b == 'C') {
            motor_calibration_callback_();
        } else if (b == 'S') {
            if (strain_calibration_callback_) {
                strain_calibration_callback_();
            }
        } else if (b == 'J'){
            if (joystick_calibration_callback_){
                joystick_calibration_callback_();
            }
        }

    }
}



void SerialProtocolPlaintext::init(DemoConfigChangeCallback demo_config_change_callback, StrainCalibrationCallback strain_calibration_callback, JoystickCalibrationCallback joystick_calibration_callback) {
    demo_config_change_callback_ = demo_config_change_callback;
    strain_calibration_callback_ = strain_calibration_callback;
    joystick_calibration_callback_ = joystick_calibration_callback;
    stream_.println("SmartKnob starting!\n\nSerial mode: plaintext\n"
                    "Press 'C' at any time to calibrate motor/sensor.\n"
                    "Press 'S' at any time to calibrate strain sensors.\n"
                    "Press 'J' at any time to calibrate joystick/sensors.\n"
                    "Press <Space> to change haptic modes.\n");
}
