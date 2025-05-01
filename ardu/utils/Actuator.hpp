#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "roverConstants.h"
#include <CytronMotorDriver.h> 

// TO DO: initalize into a known state!!
class Actuator {
    public:
        Actuator();

        /// @brief extend the actuator for some amount of time
        /// @param timeExtendMS time extend
        void extendForTimeMS(long timeExtendMS);

        /// @brief retract the actuator for some amount of time
        /// @param timeRetractMS time to retract
        void retractForTimeMS(long timeRetractMS);

        /// @brief uses button inputs on controller to extend or retract actuator, does not update state of robot, assumes visual confirmation of state
        /// @param extend button for extending 1: extend 0: do nothing
        /// @param retract button for retracting 1: retract 0: do nothing
        void remoteControl(bool extend, bool retract);

        /// @brief update state of actuator extending, retracting, or stopped
        void update();

        /// @brief stop the actuator
        void stop();

        /// @brief get the state of the actuator
        /// @return 0 = extending, 1 = extended, 2 = retracting, 3 = retracted, -1 = unknown
        int getState();

        /// @brief set the speed of retracting/extending by giving 
        /// @param duty percent duty cycle value 0-100 duty cycle (speed) value
        void setPWM(int dutyExtend, int dutyRetract);

        /// @brief get current PWM extend of actuator
        /// @return pwm
        int getPWMExtend();

        /// @brief get current PWM retract of actuator
        /// @return pwm
        int getPWMRetract();

        /// @brief check if the limit switch is activated
        /// @return true if limit switch is activated (pressed)
        bool isLimitSwitchActivated();

    private:
        int pwmExtend = 127;
        int pwmRetract = 255;
        bool isExtending = false;
        bool extended = false;
        bool isRetracting = false;
        bool retracted = false;
        long startTime = 0;
        long duration = 0;

        CytronMD actuatorMTR = CytronMD(PWM_DIR, ACT_MTR_PWM, ACT_MTR_DIR);
};

#endif


Actuator::Actuator() {
    actuatorMTR.setSpeed(0);
    pinMode(LM_SW_2, INPUT_PULLUP);  // Initialize limit switch with internal pullup
}

bool Actuator::isLimitSwitchActivated() {
    return digitalRead(LM_SW_2) == LOW;  // Active high due to pullup
}

void Actuator::remoteControl(bool extend, bool retract) {
    // dummy check
    if (extend && retract) {
        actuatorMTR.setSpeed(0);
        isExtending = false;
        isRetracting = false;
    }
    else if (extend && isLimitSwitchActivated()) {  // Only extend if limit switch not activated
        actuatorMTR.setSpeed(pwmExtend);
        isExtending = true;
        isRetracting = false;
    }
    else if (retract) {
        actuatorMTR.setSpeed(-pwmRetract);
        isExtending = false;
        isRetracting = true;
    }
    else {
        actuatorMTR.setSpeed(0);
        isExtending = false;
        isRetracting = false;
    }
}

void Actuator::extendForTimeMS(long timeExtendMS) {
    if (isLimitSwitchActivated()) {  // Don't extend if limit switch is activated
        stop();
        return;
    }
    
    isExtending = true;
    extended = false;  // Reset state
    retracted = false; // Ensure retracted flag is cleared
    duration = timeExtendMS;
    startTime = millis();
    actuatorMTR.setSpeed(pwmExtend);
}

void Actuator::retractForTimeMS(long timeRetractMS) {
    isRetracting = true;
    extended = false;
    retracted = false;
    duration = timeRetractMS;
    startTime = millis();
    actuatorMTR.setSpeed(-pwmRetract); // Assuming negative speed retracts
}

void Actuator::update() {
    long currentTime = millis();

    if (isExtending && (currentTime - startTime >= duration)) {
        // done extending
        actuatorMTR.setSpeed(0); // Stop actuator
        isExtending = false;
        extended = true;
    }

    if (isRetracting && (currentTime - startTime >= duration)) {
        // done retracting
        actuatorMTR.setSpeed(0); // Stop actuator
        isRetracting = false;
        retracted = true;
    }
}

int Actuator::getState() {
    if (isExtending) return 0;
    if (isLimitSwitchActivated()) return 1;
    if (isRetracting) return 2;
    if (retracted) return 3;
    return -1; 
}

void Actuator::stop() {
    isExtending = false;
    isRetracting = false;
    startTime = 0;
    duration = 0;
    actuatorMTR.setSpeed(0);
}

// TO DO: update to not set PWM below lowest functional voltage
void Actuator::setPWM(int dutyExtend, int dutyRetract) {
    pwmExtend = constrain(map(dutyExtend, 0, 100, 0, 255), 0, 255);
    pwmRetract = constrain(map(dutyRetract, 0, 100, 0, 255), 0, 255);
}

int Actuator::getPWMExtend() {
    return pwmExtend;
}

int Actuator::getPWMRetract() {
    return pwmRetract;
}