#include "../proto_gen/smartknob.pb.h"

#include "serial_protocol_plaintext.h"

int pressStart, pressInt = 0;

void SerialProtocolPlaintext::handleState(const PB_SmartKnobState& state) {
    bool tickChange = latest_state_.current_position != state.current_position;
    bool joystickChange = latest_state_.XOUT != state.XOUT || latest_state_.YOUT != state.YOUT;
    latest_state_ = state; //Reset
    
    if (tickChange) {
        if (state.current_position - latest_state_.current_position < 0){
            stream_.printf("CW\n");
        } else {
            stream_.printf("CCW\n");
        }
    }
    if (joystickChange){
        stream_.printf("%.3f,%.3f\n",latest_state_.XOUT,latest_state_.YOUT);
    }    
    /*
    bool substantial_change = (latest_state_.current_position != state.current_position)
        || (latest_state_.config.detent_strength_unit != state.config.detent_strength_unit)
        || (latest_state_.config.endstop_strength_unit != state.config.endstop_strength_unit)
        || (latest_state_.config.min_position != state.config.min_position)
        || (latest_state_.config.max_position != state.config.max_position);

    latest_state_ = state;
    if (substantial_change) {       
        stream_.printf("STATE: %d [%d, %d]  (detent strength: %0.2f, width: %0.0f deg, endstop strength: %0.2f)\n", 
            state.current_position,
            state.config.min_position,
            state.config.max_position,
            state.config.detent_strength_unit,
            degrees(state.config.position_width_radians),
            state.config.endstop_strength_unit);
    }
    */
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
