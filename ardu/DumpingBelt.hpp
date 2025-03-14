#ifndef DUMPINGBELT_H
#define DUMPINGBELT_H

#include "roverConstants.h"
#include <CytronMotorDriver.h>
#include <Encoder.h>

class DumpingBelt {
    public:
    DumpingBelt();

    /// @brief dump stuff on conveyor belt by rotating it half way
    void dump();

    /// @brief make more room for material on belt by rotating the belt slightly
    void rotateForLoading();

    /// @brief rotate the conveyor belt for a specified amount of time
    /// @param timeRotateMS time to rotate belt
    void rotateForLoadingMS(long timeRotateMS);

    /// @brief set the speed of moving the dumping belt by giving 0-100 duty cycle (speed) value
    /// @param duty percent duty cycle value 0-100 duty cycle (speed) value
    void setPWM(int duty);

    /// @brief get pwm
    /// @return pwm
    int getPWM();

    /// @brief gives the number of times rotateBeltForLoading() has been called (number of loads on belt)
    /// @return number of loads
    int getNumLoadsOnBelt();

    /// @brief checks if belt is full
    /// @return 1 = full, 0 = not full
    bool isFull();

    /// @brief use remote control of belt
    /// @param dumpBeltOn 1 = rotate 0 = nothing
    void remoteControl(bool dumpBeltOn);

    /// @brief get state of dumping belt
    /// @return 0 = rotating for loads, 1 = rotating for time, 2 = dumping, 3 = waiting (use getNumLoadsOnBelt() for loads)
    int getState();

    /// @brief update the state of dumping/rotating
    void update();

    /// @brief stop the conveyor belt
    void stop();

    private:
    int pwm = -150;                   // driving dump belt correct direction
    int numLoadsOnBelt = 0;
    bool full = false;
    bool rotatingForLoads = false;
    bool rotatingForTime = false;
    bool dumping = false;
    long startTime = 0;
    long duration = 0;

    CytronMD dumpBeltMTR = CytronMD(PWM_DIR, CNVB_MTR_PWM, CNVB_MTR_DIR);
    Encoder dumpBeltENC = Encoder(CNVB_ENC_A, CNVB_ENC_B);
};
#endif

DumpingBelt::DumpingBelt() {
    dumpBeltMTR.setSpeed(0);
    dumpBeltENC.readAndReset(); 
}

void DumpingBelt::dump() {
    dumping = true;
    dumpBeltENC.readAndReset();
    dumpBeltMTR.setSpeed(pwm);
}

void DumpingBelt::rotateForLoading() {
    rotatingForLoads = true;
    dumpBeltENC.readAndReset();
    dumpBeltMTR.setSpeed(pwm);
}

void DumpingBelt::rotateForLoadingMS(long timeRotateMS) {
    duration = timeRotateMS;
    startTime = millis();
    rotatingForTime = true;
    dumpBeltENC.readAndReset();
    dumpBeltMTR.setSpeed(pwm);
}

// TO DO: update to not set pwm below lowest functional voltage
void DumpingBelt::setPWM(int duty) {
    pwm = constrain(map(duty, 0, 100, 0, 255), 0, 255);
    pwm = -pwm;   // driving dump belt correct direction
}

int DumpingBelt::getPWM() {
    return abs(pwm);
}

int DumpingBelt::getNumLoadsOnBelt() {
    return numLoadsOnBelt;
}

void DumpingBelt::remoteControl(bool dumpBeltOn) {
    if (dumpBeltOn) {
        dumpBeltMTR.setSpeed(pwm);
        dumping = true;
    }
    else {
        dumpBeltMTR.setSpeed(0);
        dumping = false;
    }
}

bool DumpingBelt::isFull() {
    return full;
}

int DumpingBelt::getState() {
    if (rotatingForLoads) return 0;
    else if (rotatingForTime) return 1;
    else if (dumping) return 2;
    else return 3;
}

void DumpingBelt::update() {
    long currentCount = dumpBeltENC.read();

    if (dumping) {
        if (abs(currentCount) >= HALF_ROTATION_CNV_BELT) {
            dumpBeltMTR.setSpeed(0);
            dumping = false;
            numLoadsOnBelt = 0;
            full = false;
        }
        return;
    }
    if (rotatingForLoads) {
        if (abs(currentCount) >= INC_LOADING_ROTATION_CNV_BELT) {
            dumpBeltMTR.setSpeed(0);
            rotatingForLoads = false;
            numLoadsOnBelt = numLoadsOnBelt + 1;
            if (numLoadsOnBelt >= MAX_LOADS_CNV_BELT) {
                full = true;
            }
        }
        return;
    }
    // TO DO: track how far the belt rotated, inc loads, and determine if it dumped or not??
    if (rotatingForTime) {
        long currentTime = millis();
        if (currentTime - startTime >= duration) {
            dumpBeltMTR.setSpeed(0);
            rotatingForTime = false;
        }
        return;
    }
}

void DumpingBelt::stop() {
    dumpBeltMTR.setSpeed(0);
    startTime = 0;
    duration = 0;
    dumping = false;
    rotatingForLoads = false;
    rotatingForTime = false;
}

