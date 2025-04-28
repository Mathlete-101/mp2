#ifndef FAN_H
#define FAN_H

#include "roverConstants.h"

class Fan {
    public:
        Fan();

        /// @brief Set the fan speed as a percentage (0-100)
        /// @param speedPercent Speed as a percentage (0-100)
        void setSpeed(int speedPercent);

        /// @brief Get the current fan speed percentage
        /// @return Current speed percentage (0-100)
        int getSpeed();

        /// @brief Stop the fan
        void stop();

    private:
        int currentSpeed = 50;  // Default speed of 50%
};

#endif

Fan::Fan() {
    pinMode(FAN_PWM, OUTPUT);
    analogWrite(FAN_PWM, map(currentSpeed, 0, 100, 0, 255));
}

void Fan::setSpeed(int speedPercent) {
    currentSpeed = constrain(speedPercent, 0, 100);
    analogWrite(FAN_PWM, map(currentSpeed, 0, 100, 0, 255));
}

int Fan::getSpeed() {
    return currentSpeed;
}

void Fan::stop() {
    currentSpeed = 0;
    analogWrite(FAN_PWM, 0);
} 