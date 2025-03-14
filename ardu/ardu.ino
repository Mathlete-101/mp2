#include "Actuator.hpp"
#include "DumpingBelt.hpp"
#include "DiggingBelt.hpp"
#include "DriveTrain.hpp"
#include <ArduinoJson.h>

// use initalize function in sketch to confirm initialization
Actuator actuator;
DumpingBelt dumpBelt;
DiggingBelt digBelt;
DriveTrain driveTrain;


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
    bool cmd;

    if (error) {
        Serial.print("JSON parse failed: ");
        Serial.println(error.c_str());
        return;
    }

    if (doc.containsKey("cmd")) {
      cmd = doc["cmd"];
      if (cmd) {
        executeCommand(jsonString);
      }
      else {
        createUpdate(jsonString);
      }
    }
    else {
      Serial.println("Command/Update not recognized");
    }
    
}

void executeCommand(String command) {
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, command);

  // Extract data safely
    
  desired_speed_mps = doc["linearx_mps"];
  desired_angular_speed_rps = doc["angularz_rps"];

  actuator_extend = doc["actuator_extend"];
  actuator_retract = doc["actuator_retract"];

  dig_belt = doc["dig_belt"];
  dump_belt = doc["dump_belt"];
    
  // JsonObject buttonsJson = doc["buttons"];
  // buttons[0] = buttonsJson["A"];
  // buttons[1] = buttonsJson["B"];
  // buttons[2] = buttonsJson["X"];
  // buttons[3] = buttonsJson["Y"];

    
  JsonObject dpadJson = doc["dpad"];
  dpad_x = dpadJson["x"];
  dpad_y = dpadJson["y"];

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
        percentDutyCycle = doc["dutyA"];
        actuator.setPWM(percentDutyCycle);
    }
    if (doc.containsKey("dutyD")) {
        percentDutyCycle = doc["dutyD"];
        dumpBelt.setPWM(percentDutyCycle);
    }
     if (doc.containsKey("dutyB")) {
        percentDutyCycle = doc["dutyB"];
        digBelt.setPWM(percentDutyCycle);
    }
}

void createUpdate(String updateRequest) {
  StaticJsonDocument<256> received;
  StaticJsonDocument<256> response;

  DeserializationError error = deserializeJson(received, updateRequest);

// drive train updates
  if (received.containsKey("drive_train")) {
    JsonObject drive_train_response = response.createNestedObject("drive_train");
    JsonObject drive_train_received = received["drive_train"];

    if (drive_train_received.containsKey("set_linearx_mps")) {
      drive_train_response["set_linearx_mps"] = driveTrain.getSetFWDSpeedMPS();
    }
    if (drive_train_received.containsKey("set_angularz_rps")) {
      drive_train_response["set_angularz_rps"] = driveTrain.getSetANGSpeedRPS();
    }
    if (drive_train_received.containsKey("actual_linearx_mps")) {
      drive_train_response["actual_linearx_mps"] = driveTrain.getActualFWDSpeedMPS();
    }
    if (drive_train_received.containsKey("actual_angularz_rps")) {
      drive_train_response["actual_angularz_rps"] = driveTrain.getActualANGSpeedRPS();
    }
    if (drive_train_received.containsKey("get_Kp")) {
      drive_train_response["get_Kp"] = driveTrain.getKp();
    }
    if (drive_train_received.containsKey("get_Ki")) {
      drive_train_response["get_Ki"] = driveTrain.getKi();
    }
    if (drive_train_received.containsKey("get_state")) {
      drive_train_response["get_state"] = driveTrain.getState();
    }
  }
  // actuator updates
  if (received.containsKey("actuator")) {
    JsonObject actuator_response = response.createNestedObject("actuator");
    JsonObject actuator_received = received["actuator"];

    if (actuator_received.containsKey("get_state")) {
      actuator_response["get_state"] = actuator.getState();
    }
    if (actuator_received.containsKey("get_pwm")) {
      actuator_response["get_pwm"] = actuator.getPWM();
    }
  }
  // digging belt updates
  if (received.containsKey("dig_belt")) {
    JsonObject dig_belt_response = response.createNestedObject("dig_belt");
    JsonObject dig_belt_received = received["dig_belt"];

    if (dig_belt_received.containsKey("get_state")) {
      dig_belt_response["get_state"] = digBelt.getState();
    }
    if (dig_belt_received.containsKey("get_pwm")) {
      dig_belt_response["get_pwm"] = digBelt.getPWM();
    }
    if (dig_belt_received.containsKey("get_speed")) {
      dig_belt_response["get_speed"] = digBelt.getCurrentSpeedRPM();
    }
  }
  // dumping belt updates
  if (received.containsKey("dump_belt")) {
    JsonObject dump_belt_response = response.createNestedObject("dump_belt");
    JsonObject dump_belt_received = received["dump_belt"];

    if (dump_belt_received.containsKey("get_state")) {
      dump_belt_response["get_state"] = dumpBelt.getState();
    }
    if (dump_belt_received.containsKey("get_pwm")) {
      dump_belt_response["get_pwm"] = dumpBelt.getPWM();
    }
    if (dump_belt_received.containsKey("get_loads")) {
      dump_belt_response["get_loads"] = dumpBelt.getNumLoadsOnBelt();
    }
    if (dump_belt_received.containsKey("get_full")) {
      dump_belt_response["get_full"] = dumpBelt.isFull();
    }
  }

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

    Serial.print("\tPWM (A, DUMP, DIG): ");
    Serial.print(actuator.getPWM());
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
