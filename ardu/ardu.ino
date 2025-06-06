#include "utils/Actuator.hpp"
#include "utils/DumpingBelt.hpp"
#include "utils/DiggingBelt.hpp"
#include "utils/DriveTrain.hpp"
#include "utils/Fan.hpp"
#include <ArduinoJson.h>

// use initalize function in sketch to confirm initialization
Actuator actuator;
DumpingBelt dumpBelt;
DiggingBelt digBelt;
DriveTrain driveTrain;
Fan fan;


double desired_speed_mps = 0.0;
double desired_angular_speed_rps = 0.0;  // Angular speed in rad/s
int dpad_x = 0, dpad_y = 0;
bool actuator_extend = false;
bool actuator_retract = false;
bool dig_belt = false;
bool dump_belt = false;
//bool buttons[4] = {0, 0, 0, 0};  // A, B, X, Y
float Kp = 5;  // affects how much '+' increases speed, not rise time
float Ki = 0.05;
int percentDutyCycle = 59; // default value


unsigned long lastPrintTime = 0;
bool printUpdate = false;

void setup() {
 // set up serial connection
  Serial.begin(115200);
  while (!Serial) {
        ;  // Wait for Serial connection
    }
  Serial.println("Arduino Ready to Receive Data...");
}

void loop() {

  // shutdown everything, hard stop when dpad_x = 1 (right cross on dpad pressed)
   if (dpad_x == 1) {
      hardStop();
    } 

  // check for incoming messages
  receiveData();

  

  // set the motor wheel speeds
  driveTrain.remoteControl(desired_speed_mps, desired_angular_speed_rps);

  // update the drive train control system
  driveTrain.update();

  // control actuator On XBox A = extend, B = retract
  actuator.remoteControl(actuator_extend, actuator_retract);

  // control the dumping belt, Y = on
  dumpBelt.remoteControl(dump_belt);

  // control the digging belt, X = on
  digBelt.remoteControl(dig_belt);
  digBelt.update();


  // Print update every 5 seconds
  if (millis() - lastPrintTime >= 5000 && printUpdate) {
      printData();
      lastPrintTime = millis();
  }
}

// check for and receive data
void receiveData() {
  static String jsonData = "";
  
  // Read incoming serial data
  while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n') {  // End of JSON message
          processJson(jsonData);
          jsonData = "";  // Clear buffer for next message
      } else {
          jsonData += c;
      }
  }
}

// process the incoming messages
void processJson(String jsonString) {
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, jsonString);

    if (error) {
        Serial.print("JSON parse failed: ");
        Serial.println(error.c_str());
        return;
    }

    // Execute the command
    executeCommand(jsonString);
}

void executeCommand(String command) {
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, command);

  // Extract data safely, only update values if they exist in the message
  if (doc.containsKey("linearx_mps")) {
    desired_speed_mps = doc["linearx_mps"];
  }
  if (doc.containsKey("angularz_rps")) {
    desired_angular_speed_rps = doc["angularz_rps"];
  }

  if (doc.containsKey("actuator_extend")) {
    actuator_extend = doc["actuator_extend"];
  }
  if (doc.containsKey("actuator_retract")) {
    actuator_retract = doc["actuator_retract"];
  }

  if (doc.containsKey("dig_belt")) {
    dig_belt = doc["dig_belt"];
  }
  if (doc.containsKey("dump_belt")) {
    dump_belt = doc["dump_belt"];
  }
    
  if (doc.containsKey("dpad")) {
    JsonObject dpadJson = doc["dpad"];
    if (dpadJson.containsKey("x")) {
      dpad_x = dpadJson["x"];
    }
    if (dpadJson.containsKey("y")) {
      dpad_y = dpadJson["y"];
    }
  }

  if (doc.containsKey("Kp")) {
    Kp = doc["Kp"];
    driveTrain.setKp(Kp);
  }

  if (doc.containsKey("Ki")) {
    Ki = doc["Ki"];
    driveTrain.setKi(Ki);
  }
  // split so that each motor gets a different value
  if (doc.containsKey("dutyA")) {
    JsonObject actuatorSpeedJson = doc["dutyA"];
    actuator.setPWM(actuatorSpeedJson["extend_speed"], actuatorSpeedJson["retract_speed"]);
  }
  if (doc.containsKey("dutyD")) {
    dumpBelt.setPWM(doc["dutyD"]);
  }
  if (doc.containsKey("dutyB")) {
    digBelt.setPWM(doc["dutyB"]);
  }
  if (doc.containsKey("fan_speed")) {
    fan.setSpeed(doc["fan_speed"]);
  }
}

void createUpdate(String updateRequest) {
  StaticJsonDocument<256> response;

// drive train updates
  JsonObject drive_train_response = response.createNestedObject("drive_train");

  drive_train_response["set_linearx_mps"] = driveTrain.getSetFWDSpeedMPS();
  drive_train_response["set_angularz_rps"] = driveTrain.getSetANGSpeedRPS();
  drive_train_response["actual_linearx_mps"] = driveTrain.getActualFWDSpeedMPS();
  drive_train_response["actual_angularz_rps"] = driveTrain.getActualANGSpeedRPS();
  drive_train_response["get_Kp"] = driveTrain.getKp();
  drive_train_response["get_Ki"] = driveTrain.getKi();
  drive_train_response["get_state"] = driveTrain.getState();
    
  // actuator updates
  JsonObject actuator_response = response.createNestedObject("actuator");

  actuator_response["get_state"] = actuator.getState();
  actuator_response["get_pwm_extend"] = actuator.getPWMExtend();
  actuator_response["get_pwm_retract"] = actuator.getPWMRetract();
  
  // digging belt updates
  JsonObject dig_belt_response = response.createNestedObject("dig_belt");

  dig_belt_response["get_state"] = digBelt.getState();
  dig_belt_response["get_pwm"] = digBelt.getPWM();
  dig_belt_response["get_speed"] = digBelt.getCurrentSpeedRPM();
  
  // dumping belt updates
  JsonObject dump_belt_response = response.createNestedObject("dump_belt");
  dump_belt_response["get_state"] = dumpBelt.getState();
  dump_belt_response["get_pwm"] = dumpBelt.getPWM();
  dump_belt_response["get_loads"] = dumpBelt.getNumLoadsOnBelt();
  dump_belt_response["get_full"] = dumpBelt.isFull();

  // fan updates
  JsonObject fan_response = response.createNestedObject("fan");
  fan_response["get_speed"] = fan.getSpeed();

  String updateResponse;
  serializeJson(response, updateResponse);

  // send the response
  Serial.println(updateResponse);
}

// print update on stuff happening 
// TO DO: print actual speed of wheels
void printData() {
    Serial.print("Speed (m/s): ");
    Serial.print(driveTrain.getSetFWDSpeedMPS());
    Serial.print("\tAngular Speed (rad/s): ");
    Serial.print(driveTrain.getSetANGSpeedRPS());
    Serial.print("\tActual Speed (m/s): ");
    Serial.print(driveTrain.getActualFWDSpeedMPS());
    Serial.print("\t Actual Angular Speed: ");
    Serial.println(driveTrain.getActualANGSpeedRPS());

    
    Serial.print("\tD-Pad X: ");
    Serial.print(dpad_x);
    Serial.print("\tD-Pad Y: ");
    Serial.print(dpad_y);

    Serial.print("\tActuator Extend:");
    Serial.print(actuator_extend);
    Serial.print(" Actuator Retract: ");
    Serial.print(actuator_retract);
    Serial.print(" Dig Belt: ");
    Serial.print(dig_belt);
    Serial.print(" Dump Belt: ");
    Serial.print(dump_belt);
   

    Serial.print("\tKp: ");
    Serial.print(Kp);
    Serial.print("\tKi: ");
    Serial.print(Ki);

    Serial.print("\tPWM (A (Ex, Re), DUMP, DIG): ");
    Serial.print(actuator.getPWMExtend());
    Serial.print(", ");
    Serial.print(actuator.getPWMRetract());
    Serial.print(", ");
    Serial.print(dumpBelt.getPWM());
    Serial.print(", ");
    Serial.println(digBelt.getPWM());
}

// turn everything off
void hardStop() {
  actuator.stop();
  dumpBelt.stop();
  digBelt.stop();
  driveTrain.stop();
}









