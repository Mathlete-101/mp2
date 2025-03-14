// currently tag is module #5
// The purpose of this code is to set the tag address and antenna delay to default.
// this tag will be used for calibrating the anchors.

#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"
#include <ArduinoJson.h>

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

// Set up beacon data struct
struct BeaconVars{
  float distSep_in = 18.0; // Distance between two anchors (in)
  float dist1_in; // Distance from anchor 1 to tag
  float dist2_in; // Distance from anchor 2 to tag

  float getDist1(){
    return dist1_in;
  }
  float getDist2(){
    return dist2_in;
  }
  float getDist3(){
    return distSep_in;
  }
  // float* getTagCoords(){
  //   // OUTPUT: Array of tag coordinates
  //   float C = acos(( (dist1_in*dist1_in) + (distSep_in*distSep_in) - (dist2_in*dist2_in) )/(2*dist1_in*distSep_in)); // Measurements produce nan. b+c > a, likely due to incorrect dist
  //   float coords[2];
  //   coords[0] = dist1_in * cos(C);
  //   coords[1] = dist2_in * sin(C);
  //   return coords;
  // }
  void printDist(){
    Serial.print("Dist1: ");
    Serial.print(dist1_in);
    Serial.print(", Dist2: ");
    Serial.println(dist2_in);
  }
};
BeaconVars Beacons;

// Global status vars
bool sendCoords = false;


// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin

// leftmost two bytes below will become the "short address"
char tag_addr[] = "7D:00:22:EA:82:60:3B:9C";

long int lastTime = 0;
int offset = 200; // ms


void setup()
{
  Serial.begin(115200);
  delay(1000);

  //init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

// start as tag, do not assign random short address
  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);

  while(!Serial){
    ; // Wait for Serial to open
  }
}

void loop() {
  DW1000Ranging.loop(); // Beacon magic code idk

  receiveData(); // Check for JSON over Serial
  
  if (sendCoords){ // Send JSON of tag coordinates to Serial every <offset> millis
    if (millis() > (lastTime + offset)){
        float dist1 = Beacons.getDist1();
        float dist2 = Beacons.getDist2();
        sendCoordsJson(dist1, dist2);
        lastTime += offset;
    }
  }
  else{
    if (millis() > (lastTime + offset)){
        Beacons.printDist();
        lastTime += offset;
    }
  }

}

void newRange(){
  int address = DW1000Ranging.getDistantDevice()->getShortAddress();
  float dist = DW1000Ranging.getDistantDevice()->getRange() * 39.37; // convert to in

  if (address == 1){
    Beacons.dist1_in = dist;
  }
  else{
    Beacons.dist2_in = dist;
  }

}

void newDevice(DW1000Device *device){
  Serial.print("Device added: ");
  int address = device->getShortAddress();
  Serial.println(address);
}

void inactiveDevice(DW1000Device *device){
  Serial.print("delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}

void sendCoordsJson(float x, float y){
  JsonDocument doc;
  String response;
  doc["Coords"][0] = x;
  doc["Coords"][1] = y;
  serializeJson(doc, response);
  Serial.println(response);
}

void receiveData(){
  while(Serial.available()){
    static String jsonData = "";
    char c = Serial.read();
    if (c == '\n'){
      processJson(jsonData);
      jsonData = "";
    }
    else{
      jsonData += c;
    }
  }
}

void processJson(String jsonString) {
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, jsonString);
    bool cmd;

    if (error) {
        Serial.print("JSON parse failed: ");
        Serial.println(error.c_str());
        return;
    }

    // "Calibrate" is a bool to request a calibration between the beacons. Ideally we can dynamimcally callibrate, WIP.
    if (doc.containsKey("Calibrate")){
      if(doc["Calibrate"]){
        calibrateBeacons();
      }

    }
    // "SendCoords" is a bool to initiate Serial communication with Mega for coords. Will send coords if true, not if false.
    if (doc.containsKey("SendCoords")){
      sendCoords = doc["SendCoords"];
    }
    // "jsonDelay" is an int value for time between messages to Serial, in ms
    if (doc.containsKey("jsonDelay")){
      offset = doc["jsonDelay"];
    }
    
}

void calibrateBeacons(){
  Serial.println("Calibrating....");

  // FIXME: Add calibration code here

  Serial.println("Done!");
}
