#include "../proto_gen/smartknob.pb.h"

#include "serial_protocol_plaintext.h"

bool numericControl = false; //Using XOUT, YOUT as commands vs. constants
bool continuousJoyCommands = false; //Only 1 press per joystick motion
bool waitUntilRecenter = false; //Logic for only 1 press per joystick motion

int debounceDelay;

void SerialProtocolPlaintext::updateJoystick(float XOUT, float YOUT){
    if (numericControl){
        if (abs(XOUT) > .5 || abs(YOUT) > .5)
            stream_.printf("%.3f,%.3f\n",XOUT,YOUT);
    } else {
        if (!waitUntilRecenter && (abs(XOUT) > .5 || abs(YOUT) > .5)){
            if (abs(XOUT) > abs(YOUT)){
                if (XOUT > 0){
                    stream_.print("Left\n");
                } else {
                    stream_.print("Right\n");
                }
            } else {
                if (YOUT > 0){
                    stream_.print("Up\n");
                } else {
                    stream_.print("Down\n"); 
                }
            }
            if (!continuousJoyCommands){
                waitUntilRecenter = true;
                debounceDelay = millis();
            }
        } else if (waitUntilRecenter && debounceDelay - millis() > 200 && abs(XOUT) < .5 && abs(YOUT) < .5){
            waitUntilRecenter = false;
        }
    }
}

void SerialProtocolPlaintext::handleState(const PB_SmartKnobState& state) {
    bool taskChange = strcmp(latest_state_.config.text,state.config.text) != 0;
    bool tickChange = latest_state_.current_position != state.current_position;
    bool CWChange = state.current_position - latest_state_.current_position < 0;

    latest_state_ = state; //Reset
    if (taskChange){
        stream_.printf("%s\n",latest_state_.config.text);
        if (strcmp(latest_state_.config.text,"Select Music Album") == 0){
            numericControl = false;
            continuousJoyCommands = false;
        } else if (strcmp(latest_state_.config.text,"Control Music Volume") == 0){
            numericControl = false;
            continuousJoyCommands = false;
        } else if (strcmp(latest_state_.config.text,"Navigate on Map") == 0){
            numericControl = true;
        } else if (strcmp(latest_state_.config.text,"Social Media Task") == 0){
            numericControl = false;
            continuousJoyCommands = true;
        }
    } else {
        if (tickChange) {
            if (CWChange){
                stream_.printf("CW\n");
            } else {
                stream_.printf("CCW\n");
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
