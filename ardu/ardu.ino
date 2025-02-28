// code that will not work for lunar reglith pit test #1 on 2/14/2025


#include <Encoder.h>
#include <CytronMotorDriver.h>
#include <ArduinoJson.h>


#define RIGHT_F_PWM 7
#define RIGHT_F_DIR 6   // spins opposite way, encoders inc with current forward
#define ENCR_A_FR 20
#define ENCR_B_FR 27


#define LEFT_F_PWM 9
#define LEFT_F_DIR 8
#define ENCR_A_FL 2
#define ENCR_B_FL 25

#define RIGHT_B_PWM 5
#define RIGHT_B_DIR 16   // spins opposite way, encoders inc with current forward
#define ENCR_A_BR 3
#define ENCR_B_BR 23

#define LEFT_B_PWM 12
#define LEFT_B_DIR 17
#define ENCR_A_BL 19
#define ENCR_B_BL 29

#define ACT_MTR_PWM 11        
#define ACT_MTR_DIR 15 

#define DB_MTR_PWM 10         
#define DB_MTR_DIR 14 

#define CNVB_MTR_PWM 4
#define CNVB_MTR_DIR 30

#define SAMPLE_TIME_MS 10

#define BATTERY_VOLTAGE 13.2
#define PI 3.141593

#define ENC_CNT_PER_REV 5281
#define WHEEL_RADIUS_M 0.1574 
#define TRACK_WIDTH_M   0.5476


long last_time_ms[4] = {0};
long current_time_ms;

unsigned long lastPrintTime = 0;


float actual_wheel_speed[4] = {0};
float desired_wheel_speed[4] = {0};

float desired_speed_mps = 0;
float desired_angular_speed_rps = 0;  // Angular speed in rad/s
int dpad_x = 0, dpad_y = 0;
bool buttons[4] = {0, 0, 0, 0};  // A, B, X, Y
float dump_belt_ctrl;
float dig_belt_ctrl;
float dig_actuator_ctrl;

float error[4] = {0};
float Voltage[4] = {0};


// Orgainzed so index 0 = Front Right Motor, 1 = Front Left Motor, 2 = Back Right Motor, 3 = Back Left Motor

CytronMD motors[4] = {CytronMD(PWM_DIR, RIGHT_F_PWM, RIGHT_F_DIR), 
                      CytronMD(PWM_DIR, LEFT_F_PWM, LEFT_F_DIR), 
                      CytronMD(PWM_DIR, RIGHT_B_PWM, RIGHT_B_DIR), 
                      CytronMD(PWM_DIR, LEFT_B_PWM, LEFT_B_DIR)};

Encoder encoders[4] = {Encoder(ENCR_A_FR, ENCR_B_FR), 
                      Encoder(ENCR_A_FL, ENCR_B_FL), 
                      Encoder(ENCR_A_BR, ENCR_B_BR), 
                      Encoder(ENCR_A_BL, ENCR_B_BL)};
                    
CytronMD actuator = CytronMD(PWM_DIR, ACT_MTR_PWM, ACT_MTR_DIR);
CytronMD digBelt = CytronMD(PWM_DIR, DB_MTR_PWM, DB_MTR_DIR);
CytronMD convBelt = CytronMD(PWM_DIR, CNVB_MTR_PWM, CNVB_MTR_DIR);

long encCount[4] = {0};

float pos_m[4] = {0};
float prev_pos_m[4] = {0};
float PWM;

// PID control variables
float Kp = 5;  // affects how much '+' increases speed, not rise time
float Ki = 0.05;
float integral_error[4] = {0};

int i;
bool print = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
        ;  // Wait for Serial connection
    }
  Serial.println("Arduino Ready to Receive Data...");
  
  for (i=0; i<4; i++) {
    last_time_ms[i] = millis();
  }
}

void loop() {
  
  
  static String jsonData = "";
    
    // Read incoming serial data
    while (Serial.available()) {
        char c = Serial.read();
//        Serial.print("Recieved chr: ");
//        Serial.println(c);
        if (c == '\n') {  // End of JSON message
            processJson(jsonData);
            jsonData = "";  // Clear buffer for next message
        } else {
            jsonData += c;
        }
    }
      
  // shutdown everything, hard stop when dpad_x = 1 (right cross on dpad pressed)
   if (dpad_x == 1) {
      hardStop();
    } 

  current_time_ms = millis();
  if (current_time_ms - last_time_ms[0] >= SAMPLE_TIME_MS) {
    for (i=0; i<4; i++) {
      current_time_ms = millis(); // update for every motor

      encCount[i] = encoders[i].read();   // read encoders
      // Serial.print(encCount[i]);

      // // make right side motors spin same direction as left side
      // if (i == 0 || i == 2) {
      //   encCount[i] = -encCount[i];
      // }

      pos_m[i] = countsToMeters(encCount[i]); // calculate the position in meters

      actual_wheel_speed[i] = (pos_m[i] - prev_pos_m[i]) / ((current_time_ms - last_time_ms[i]) / 1000.0);   // calculate speed in rad/s
      prev_pos_m[i] = pos_m[i];

      // calculate error
      error[i] = desired_wheel_speed[i] - actual_wheel_speed[i];
      integral_error[i] += error[i] * ((current_time_ms - last_time_ms[i]) / 1000.0);

      Voltage[i] = Kp * error[i] + Ki * integral_error[i];


      // set PWM for motors
      PWM = 255 * Voltage[i] / BATTERY_VOLTAGE;
      //Serial.println(PWM);

      // make right side motors spin same direction as left side
      if (i == 0 || i == 2) {
        PWM = -PWM;
      }
      
      motors[i].setSpeed(PWM);

      last_time_ms[i] = millis();

      // print time, voltage, and speed
      if (i == 0 && print) {
        // Serial.print(current_time_ms);
        // Serial.print("\t");
        // Serial.print(Voltage[i]);
        // Serial.print("\t");
        // Serial.print(actual_wheel_speed[i]);
        // Serial.print("\t");
        // Serial.print(desired_speed_mps);
      }
    }
  }
  // control actuator
  actuatorControl();
  // control digging belt
  digBeltControl();
  // control conveyor belt
  convBeltControl();

  // Print update every second
    if (millis() - lastPrintTime >= 5000) {
        printData();
        lastPrintTime = millis();
    }

}

// Convert encoder counts to meters
float countsToMeters(long enc_counts) {
    float theta = 2 * PI * (float) enc_counts / ENC_CNT_PER_REV; // Convert to radians
    return theta * WHEEL_RADIUS_M; // Convert to meters
}

void processJson(String jsonString) {
//    Serial.println("Processing json...");
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, jsonString);

    if (error) {
        Serial.print("JSON parse failed: ");
        Serial.println(error.c_str());
        Serial.println("Invalid JSON: " + jsonString);
        return;
    }

    // Extract data
    desired_speed_mps = doc["linearx_mps"];
    desired_angular_speed_rps = doc["angularz_rps"];
    dump_belt_ctrl = doc["dump_belt"];
    dig_belt_ctrl = doc["dig_belt"];
    dig_actuator_ctrl = doc["dig_actuator"];
    

    JsonObject buttonsJson = doc["buttons"];
    buttons[0] = buttonsJson["A"];
    buttons[1] = buttonsJson["B"];
    buttons[2] = buttonsJson["X"];
    buttons[3] = buttonsJson["Y"];

    JsonObject dpadJson = doc["dpad"];
    dpad_x = dpadJson["x"];
    dpad_y = dpadJson["y"];

    Kp = doc["Kp"];
    Ki = doc["Ki"];

    // update the wheel speeds
    calculateWheelSpeed();
}

void calculateWheelSpeed() {
  float v_R = desired_speed_mps - (TRACK_WIDTH_M / 2.0) * desired_angular_speed_rps;
  float v_L = desired_speed_mps + (TRACK_WIDTH_M / 2.0) * desired_angular_speed_rps;

  float omega_R = v_R / WHEEL_RADIUS_M;  // Convert m/s to rad/s
  float omega_L = v_L / WHEEL_RADIUS_M;  // Convert m/s to rad/s

   // Assign to global array
  desired_wheel_speed[0] = omega_R;
  desired_wheel_speed[1] = omega_L;
  desired_wheel_speed[2] = omega_R;
  desired_wheel_speed[3] = omega_L;
  // Serial.print("Right Speed: ");
  // Serial.print(omega_R);
  // Serial.print("\t Left Speed: ");
  // Serial.println(omega_L);
}

void actuatorControl() {
    static float PWM_Actuator = 150.0;

    // Control actuator based on button inputs
    if (dig_actuator_ctrl != 0) {
      // Manage PWM using DPAD
        PWM_Actuator = constrain(PWM_Actuator + (dig_actuator_ctrl * 0.1), 0.0, 255.0);
        actuator.setSpeed(PWM_Actuator);  // Extend
    } else if (buttons[1]) {
      // Manage PWM using DPAD
        PWM_Actuator = constrain(PWM_Actuator + (dig_actuator_ctrl * 0.1), 0.0, 255.0);
        actuator.setSpeed(-PWM_Actuator); // Retract
    } else {
        actuator.setSpeed(0);  // Stop if no buttons are pressed
    }
}

void digBeltControl() {
  
  static float PWM_DigBelt = 150.0;

   // X button pressed, no other buttons pressed, dig
  if (dig_belt_ctrl != 0) {
    // manage PWM 
    PWM_DigBelt = constrain(PWM_DigBelt + (dig_belt_ctrl * 0.1), 0.0, 255.0);
    digBelt.setSpeed(PWM_DigBelt);
  }
  // stop motor if not used
  else {
    digBelt.setSpeed(0);
  }
}

void convBeltControl() { //dump belt
  
  static float PWM_ConvBelt = 100.0;

  // Y button pressed, no other buttons pressed, dump
  if (dump_belt_ctrl != 0) {
     // manage PWM 
    PWM_ConvBelt = constrain(PWM_ConvBelt + (dump_belt_ctrl * 0.1), 0.0, 255.0);
    convBelt.setSpeed(PWM_ConvBelt);
  }
  // stop motor if not used
  else {
    convBelt.setSpeed(0);
  }
}

// shutdown everything, hard stop when dpad_x = 1 (right cross on dpad pressed)
void hardStop() {
  
  desired_speed_mps = 0;
  desired_angular_speed_rps = 0; 

  for (i=0; i<4; i++) {
    integral_error[i] = 0;
    desired_wheel_speed[i] = 0;
    motors[i].setSpeed(0);
  }

  convBelt.setSpeed(0);
  digBelt.setSpeed(0);
  actuator.setSpeed(0);
  
  // redundent, but whatever
  calculateWheelSpeed();

}



void printData() {
    Serial.print("Speed (m/s): ");
    Serial.print(desired_speed_mps);
    Serial.print("\tAngular Speed (rad/s): ");
    Serial.print(desired_angular_speed_rps);
    
    Serial.print("\tD-Pad X: ");
    Serial.print(dpad_x);
    Serial.print("\tD-Pad Y: ");
    Serial.print(dpad_y);

    Serial.print("\tButtons: A[");
    Serial.print(buttons[0]);
    Serial.print("] B[");
    Serial.print(buttons[1]);
    Serial.print("] X[");
    Serial.print(buttons[2]);
    Serial.print("] Y[");
    Serial.print(buttons[3]);
    Serial.print("]");

    Serial.print("\tKp: ");
    Serial.print(Kp);
    Serial.print("\tKi: ");
    Serial.println(Ki);
}

// void processCmd(char command) {

//   // max speed for each wheel is 0.412 m/s or 25 rpm
//   if (command == '+') {
//     desired_speed_mps += 0.1;
//   }
//   else if (command == '-') {
//     desired_speed_mps -= 0.1;
//   }
//   else if (command == 'p') {
//     print = !print;
//   }
//   else if (command == 'r') {
//       desired_angular_speed_rps += 0.1; // Increase rotation speed (counterclockwise) // maybe needs to increment faster for more response, has a huge turning radius rn
//     }
//   else if (command == 'l') {
//       desired_angular_speed_rps -= 0.1; // Decrease rotation speed (clockwise)
//   }
//   else {
//       desired_speed_mps = 0;
//       desired_angular_speed_rps = 0;
//       for (int i = 0; i < 4; i++) {
//           integral_error[i] = 0;
//       }
//     }
//     Serial.print("Desired Speed (m/s): ");
//     Serial.print(desired_speed_mps);
//     Serial.print("\t Angular Speed (rad/s): ");
//     Serial.println(desired_angular_speed_rps);
// }
