#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "roverConstants.h"
#include <CytronMotorDriver.h>
#include <Encoder.h>

class DriveTrain {
    public:
    DriveTrain();


    /// @brief sets the speed of the wheels to carry out speed commands
    /// @param fwd_speed_mps forward speed
    /// @param angular_speed_rps turning speed
    void setSpeed(double fwd_speed_mps, double angular_speed_rps);

    /// @brief sets a speed for specified amount of time
    /// @param fwd_speed_mps speed forward/backward
    /// @param angular_speed_rps truning speed (clockwise is positive??)
    /// @param timeDrive duration of drive
    void setSpeedMS(double fwd_speed_mps, double angular_speed_rps, long timeDriveMS);

    /// @brief sets the speed of the wheels to carry out speed commands
    /// @param fwd_speed_mps forward speed
    /// @param angular_speed_rps turning speed
    void remoteControl(double fwd_speed_mps, double angular_speed_rps);

    /// @brief set Kp for wheel control
    /// @param newKp new Kp value
    void setKp(double newKp);

    /// @brief get current Kp value
    /// @return Kp
    double getKp();

    /// @brief set Ki for wheel control
    /// @param newKi new Ki value
    void setKi(double newKi);

    /// @brief get current Ki value
    /// @return Ki
    double getKi();

    /// @brief get set fwd speed
    /// @return set fwd speed
    double getSetFWDSpeedMPS();

    /// @brief get actual fwd speed
    /// @return actual fwd speed
    double getActualFWDSpeedMPS();

    /// @brief get set angular speed
    /// @return set angular speed
    double getSetANGSpeedRPS();

    /// @brief get actual angular speed
    /// @return actual angular speed
    double getActualANGSpeedRPS();

    /// @brief get state of drivetrain
    /// @return 0 = speedControl, 1 = timeControl, 2 = waiting
    int getState();

    /// @brief update the wheel speeds by running control system for speed or position
    void update();

    /// @brief stop motors, reset values for control system
    void stop();

    private:

    /// @brief calculate the wheel speed for each wheel based on given velocities
    void calculateWheelSpeed();

    /// @brief calculate the actual speeds of rover (fwd and angular)
    void calculateActualSpeed();

    /// @brief convert counts to meters
    /// @param enc_counts from wheel encoder
    /// @return meters traveled by wheel
    double countsToMeters(long enc_counts);

    int i;
    bool speedControl = false;
    bool timeControl = false;
    bool setZeroSpeedTimer = false;

    double desired_speed_mps = 0;
    double desired_angular_speed_rps = 0;
    double actual_speed_mps = 0;
    double actual_angular_speed_rps = 0;
    double Kp = 2.5;
    double Ki = 0.05;

    long duration;
    long startTime;

    long timeZeroSpeed = 0;

    long prev_RC_time = 0;
    
    
    // Orgainzed so index 0 = Front Right Motor, 1 = Front Left Motor, 2 = Back Right Motor, 3 = Back Left Motor
    
    double desired_wheel_speed[4] = {0};
    double actual_wheel_speed[4] = {0};
    long last_time_ms[4] = {0};
    long encCount[4] = {0};
    long pos_m[4] = {0};
    long prev_pos_m[4] = {0};
    double Voltage[4] = {0};
    double error[4] = {0};
    double integral_error[4] = {0};

    CytronMD motors[4] = {CytronMD(PWM_DIR, RIGHT_F_PWM, RIGHT_F_DIR), 
        CytronMD(PWM_DIR, LEFT_F_PWM, LEFT_F_DIR), 
        CytronMD(PWM_DIR, RIGHT_B_PWM, RIGHT_B_DIR), 
        CytronMD(PWM_DIR, LEFT_B_PWM, LEFT_B_DIR)};

    Encoder encoders[4] = {Encoder(ENCR_A_FR, ENCR_B_FR), 
        Encoder(ENCR_A_FL, ENCR_B_FL), 
        Encoder(ENCR_A_BR, ENCR_B_BR), 
        Encoder(ENCR_A_BL, ENCR_B_BL)};
};

#endif

DriveTrain::DriveTrain() {
    for (int i=0; i<4; i++) {
        motors[i].setSpeed(0);
        encoders[i].readAndReset();

        // init time
        last_time_ms[i] = millis();
    }
}

void DriveTrain::setSpeed(double fwd_speed_mps, double angular_speed_rps) {
     // Check if the speed and angular speed are unchanged
    if (fabs(fwd_speed_mps - desired_speed_mps) < 0.01 && fabs(angular_speed_rps - desired_angular_speed_rps) < 0.01) {
        return;
    }
    desired_speed_mps = fwd_speed_mps;
    desired_angular_speed_rps = angular_speed_rps;
    setZeroSpeedTimer = true;
    speedControl = true;
    timeControl = false;
    calculateWheelSpeed();
}

void DriveTrain::setSpeedMS(double fwd_speed_mps, double angular_speed_rps, long timeDriveMS) {
    desired_speed_mps = fwd_speed_mps;
    desired_angular_speed_rps = angular_speed_rps;
    setZeroSpeedTimer = true;
    speedControl = true;
    timeControl = true;
    duration = timeDriveMS;
    startTime = millis();
    calculateWheelSpeed();
}

// check if previous inputs are the same as current and last time called, if within a certain time frame continue, otherwise reinitialize
void DriveTrain::remoteControl(double fwd_speed_mps, double angular_speed_rps) {
     // Check if the speed and angular speed are unchanged
    if (fabs(fwd_speed_mps - desired_speed_mps) < 0.01 && fabs(angular_speed_rps - desired_angular_speed_rps) < 0.01) {
        return;
    }
    desired_speed_mps = fwd_speed_mps;
    desired_angular_speed_rps = angular_speed_rps;
    setZeroSpeedTimer = true;
    speedControl = true;
    timeControl = false;
    calculateWheelSpeed();
}


void DriveTrain::update() {
    long current_time_ms = millis();
    double pwm;

    if (current_time_ms - last_time_ms[0] >= SAMPLE_TIME_MS) {
        for (i=0; i<4; i++) {
            //current_time_ms = millis(); // update for every motor

            encCount[i] = encoders[i].read(); // get new encoder count

            pos_m[i] = countsToMeters(encCount[i]);

            actual_wheel_speed[i] = (pos_m[i] - prev_pos_m[i]) / ((current_time_ms - last_time_ms[i]) / 1000.0); // calculate speed rad/s
            prev_pos_m[i] = pos_m[i]; // update position

            // update error
            error[i] = desired_wheel_speed[i] - actual_wheel_speed[i];
            integral_error[i] += error[i] * ((current_time_ms - last_time_ms[i]) / 1000.0);

            Voltage[i] = Kp * error[i] + Ki * integral_error[i];

            // idk about this, adjust the voltage for deadzone
            if (Voltage[i] > 0) {
                Voltage[i] += WHEEL_UNUSABLE_V;
            }
            else if (Voltage[i] < 0) {
                Voltage[i] -= WHEEL_UNUSABLE_V;
            }

            // calc pwm value
            pwm = constrain(255 * Voltage[i] / BATTERY_VOLTAGE, -255, 255);

            // make right side motors spin same direction as left side, FL wheel pwr connector is backwards
            if (i == 0 || i == 2) {
                pwm = -pwm;
            }

            // update the motor speed
            motors[i].setSpeed(pwm);
            
            // update last time calculation occured
            last_time_ms[i] = current_time_ms;
        }
    } // if using time control, check if the time has passed and if so stop rover

    if (timeControl) {
        if (current_time_ms - startTime >= duration) {
            stop();
        }
    }
    // check if the desired speed is 0 for longer than 200ms, if so, clear error, make sure rover is stopped
    if (abs(desired_speed_mps) < 0.01 && abs(desired_angular_speed_rps) < 0.01) {
        if (setZeroSpeedTimer) {
            timeZeroSpeed = millis();
            setZeroSpeedTimer = false;
        }
        if (current_time_ms - timeZeroSpeed >= 200) {
            stop();
        }
    }
}

void DriveTrain::setKp(double newKp) {
    Kp = newKp;
}

double DriveTrain::getKp() {
    return Kp;
}

void DriveTrain::setKi(double newKi) {
    Ki = newKi;
}

double DriveTrain::getKi() {
    return Ki;
}

double DriveTrain::getSetFWDSpeedMPS() {
    return desired_speed_mps;
}

double DriveTrain::getActualFWDSpeedMPS() {
    calculateActualSpeed();
    return actual_speed_mps;
}

double DriveTrain::getSetANGSpeedRPS() {
    return desired_angular_speed_rps;
}

double DriveTrain::getActualANGSpeedRPS() {
    calculateActualSpeed();
    return actual_angular_speed_rps;
}

int DriveTrain::getState() {
    if (speedControl && !timeControl) return 0;
    else if (timeControl) return 1;
    else return 2;
}

void DriveTrain::stop() {
    speedControl = false;
    timeControl = false;
    desired_speed_mps = 0;
    desired_angular_speed_rps = 0;
    startTime = 0;
    duration = 0;
    for (i=0; i<4; i++) {
        desired_wheel_speed[i] = 0;
        actual_wheel_speed[i] = 0;
        Voltage[i] = 0;
        error[i] = 0;
        integral_error[i] = 0;
        motors[i].setSpeed(0);
    }
}

void DriveTrain::calculateWheelSpeed() {
  double v_R = desired_speed_mps - (TRACK_WIDTH_M / 2.0) * desired_angular_speed_rps;
  double v_L = desired_speed_mps + (TRACK_WIDTH_M / 2.0) * desired_angular_speed_rps;

  double omega_R = v_R / WHEEL_RADIUS_M;  // Convert m/s to rad/s
  double omega_L = v_L / WHEEL_RADIUS_M;  // Convert m/s to rad/s

   // Assign to array wheel in array
  desired_wheel_speed[0] = omega_R;
  desired_wheel_speed[1] = omega_L;
  desired_wheel_speed[2] = omega_R;
  desired_wheel_speed[3] = omega_L;
}

void DriveTrain::calculateActualSpeed() {
    double actual_v_R = (actual_wheel_speed[0] + actual_wheel_speed[2]) / 2.0 * WHEEL_RADIUS_M;  // Right side average speed (m/s)
    double actual_v_L = (actual_wheel_speed[1] + actual_wheel_speed[3]) / 2.0 * WHEEL_RADIUS_M;  // Left side average speed (m/s)

    actual_speed_mps = (actual_v_R + actual_v_L) / 2.0; // Linear speed (m/s)
    actual_angular_speed_rps = (actual_v_R - actual_v_L) / TRACK_WIDTH_M; // Angular speed (rad/s)
}


double DriveTrain::countsToMeters(long enc_counts) {
    double theta = 2 * PI * (double) enc_counts / ENC_CNT_PER_REV; // convert to radians
    return theta * WHEEL_RADIUS_M;  // convert to meters
}

