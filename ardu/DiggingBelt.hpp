#ifndef DIGGINGBELT_H
#define DIGGINGBELT_H

#include "roverConstants.h"
#include <CytronMotorDriver.h>
#include <Encoder.h>


class DiggingBelt {
    public:
    DiggingBelt();

    /// @brief set the speed of moving the dumping belt by giving 0-100 duty cycle (speed) value
    /// @param duty percent duty cycle value 0-100 duty cycle (speed) value
    void setPWM(int duty);

    /// @brief get pwm
    /// @return pwm
    int getPWM();

    /// @brief use remote control to run digging belt
    /// @param digBeltOn 1 = on, 0 = off
    void remoteControl(bool digBeltOn);

    /// @brief return the current speed of the dig belt revoluions per second
    /// @return  revolutions per second
    double getCurrentSpeedRPM();

    /// @brief turns on digging belt
    void start();

    /// @brief update speed of dig belt
    void update();

    /// @brief get state of digging
    /// @return 0 = off 1 = digging
    int getState();

    /// @brief stop the digging belt
    void stop();

    private:
    int pwm = 150;
    double currentSpeedRPM = 0;
    bool digging = false;


    CytronMD digBeltMTR = CytronMD(PWM_DIR, DB_MTR_PWM, DB_MTR_DIR);
    Encoder digBeltENC = Encoder(DB_ENC_A, DB_ENC_B);
};

#endif

DiggingBelt::DiggingBelt() {
    digBeltMTR.setSpeed(0);
    digBeltENC.readAndReset(); 
}

void DiggingBelt::setPWM(int duty) {
    pwm = constrain(map(duty, 0, 100, 0, 255), 0, 255);
}

int DiggingBelt::getPWM() {
    return pwm;
}

void DiggingBelt::remoteControl(bool digBeltOn) {
    if(digBeltOn) {
        digBeltMTR.setSpeed(pwm);
        digging = true;
    }
    else {
        digBeltMTR.setSpeed(0);
        digging = false;
    }
}

double DiggingBelt::getCurrentSpeedRPM() {
    return currentSpeedRPM;
}

void DiggingBelt::update() {
    static unsigned long lastUpdateTime = 0;
    static int lastEncoderCount = 0;

    if (digging) {
        unsigned long currentTime = millis();  // Get current time in milliseconds
        unsigned long dt = currentTime - lastUpdateTime; // Time difference in milliseconds

        if (dt >= SAMPLE_TIME_MS) { // Update every sample time 
            int currentEncoderCount = digBeltENC.read(); 
            int deltaCounts = currentEncoderCount - lastEncoderCount;

            currentSpeedRPM = (static_cast<double>(deltaCounts) / ENC_CNT_PER_REV_DIG) * (60000.0 / dt);

            lastEncoderCount = currentEncoderCount;
            lastUpdateTime = currentTime;
        }
    }
}

void DiggingBelt::start() {
    digBeltMTR.setSpeed(pwm);
    digging = true;
}

int DiggingBelt::getState() {
    if (!digging) return 0;
    else return 1;
}

void DiggingBelt::stop() {
    digBeltMTR.setSpeed(0);
    currentSpeedRPM = 0;
    digging = false;
}
