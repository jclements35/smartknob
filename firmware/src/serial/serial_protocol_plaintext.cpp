#include "../proto_gen/smartknob.pb.h"

#include "serial_protocol_plaintext.h"

int choosenStrengthInt = 0;
int choosenStepSizeInt = 0;

//#include "arduino.h"
//#include "HID-Project.h"

int leftRepeated = 0;
int rightRepeated = 0;

void SerialProtocolPlaintext::setup(){
    Serial.begin(921600);
    //Keyboard.begin();
}

void SerialProtocolPlaintext::handleState(const PB_SmartKnobState& state) {
    bool substantial_change = true;
    bool position_change = latest_state_.current_position != state.current_position;
    if (position_change) {
        if (state.current_position - latest_state_.current_position < 0){
            if(leftRepeated == 0) stream_.printf("Tick Check: %d\n",rightRepeated);
            stream_.printf("Left: %d\n",leftRepeated++);
            //Keyboard.write("A");
            rightRepeated = 0;
        } else {
            if(rightRepeated == 0) stream_.printf("Tick Check: %d\n",leftRepeated);
            stream_.printf("Right %d\n",rightRepeated++);
            //Keyboard.write("D");
            leftRepeated = 0;
        }
    }
    //Reset
    latest_state_ = state;
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
    /*
    stream_.print("LOG: ");
    stream_.println(msg);
    */
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
        } else if (b == 's'){

        } else if (b == 'd'){

        }

    }
}



void SerialProtocolPlaintext::init(DemoConfigChangeCallback demo_config_change_callback, StrainCalibrationCallback strain_calibration_callback) {
    demo_config_change_callback_ = demo_config_change_callback;
    strain_calibration_callback_ = strain_calibration_callback;
    stream_.println("SmartKnob starting!\n\nSerial mode: plaintext\nPress 'C' at any time to calibrate motor/sensor.\nPress 'S' at any time to calibrate strain sensors.\nPress <Space> to change haptic modes.\n");
    setup();
}
