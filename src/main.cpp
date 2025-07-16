#include <Arduino.h>
#include <EEPROM.h>
#include "TrackDataStruct.h"
#include "TrackDataTable.h"

#define SLAVE_ID 0xB1
#define RE_Sensor 2
#define DE_Sensor 3

#define RE_Senshub 19
#define DE_Senshub 18

#define STOP_BYTE 0x50
#define HEARTBEAT_BYTE 0xBB
#define RESET_BYTE 0x1B5
#define CMD_REQUEST_DATA 0x01
#define CMD_SET_CONFIG   0x02

#define CONFIG_ADDR 0
#define SLAVE_ID_ADDR 50 // Where we store slave ID


struct HeartbeatHandler {
  unsigned long lastHeartbeatTime = 0;
  unsigned long heartbeatTimeout = 2000; // 2 second timeout
  bool connected = false;
  String heartbeatChar = "♥";

  void beat() { 
    lastHeartbeatTime = millis();

    // Change characters to show on the terminal
    if (heartbeatChar == "♥") {
      heartbeatChar = "♡";
    }
    else{
      heartbeatChar = "♥";
    }
}

  void connect() {
    beat();
    connected = true;
  }

  void disconnect() { connected = false; }

  bool isAlive() {
    return (millis() - lastHeartbeatTime) < heartbeatTimeout;
  }
};

byte dataToSend[3];
IntervalTimer timer;

int16_t velocityValue = 0;
uint32_t sensorReading = 0;

uint8_t CANframe[8];
int sendingFrequency = 200; // Frequency to send data to sensor hub in Hz

long trackData[2][2] = {{1188, 0}, {125856, 125516}};

enum CommandType
{
  STOP,
  CYCLIC_POS,
  CYCLIC_POS_VEL,
  SINGLE_POS,
  SINGLE_POS_VEL,
  STANDBY,
  UNKNOWN,
};

TrackData trackDataLeft[leftDataTableSize]; // we will only use the left side this year, but i added right side for future (you are welcome DH10 <3)
TrackData trackDataRight[rightDataTableSize];

uint8_t handshakeInitFrame[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t handshakeOrderFrame[5] = {1,2,3,4,5};
uint8_t handshakeEndFrame[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
HeartbeatHandler heartbeatHandler;

// Hardt variables
int trackId = 0;
float absPosition = 0; // Absolute position in meters
float prevAbsPosition = 0; // Absolute position in meters
bool validPosition = false;


void sendCommandToSensor(CommandType cmd);
void sendData();
void resetTeensy();
void doHandshake();
void findAbsPosition(float barcode, int &trackId, float prevAbsPosition, float velocity, 
                   const std::vector<TrackData> &tracks, float &absPosition);
void barcode_estimator(float barcode, int trackIdPrev, float prevAbsPosition, float velocity, const std::vector<TrackData> &tracks,
                       int &trackId, float &absPosition, bool &validPosition);
float calculate_position(float barcode, TrackData track);
bool check_range(float barcode, const TrackData &track);
bool check_valid(float barcode, const TrackData &track);

void setup() {
  // ---------------------- USB ---------------------
  Serial.begin(9600); // USB serial
  Serial.println("Booting up...");
  // ------------------------------------------------

  // ----------------- SENSOR INIT ----------------
  Serial1.begin(115200); // Serial 1 is for receiving from the sensor
  Serial6.begin(115200); // Serial 6 is for sending to the sensor
  Serial1.setTimeout(1); // Set timeout for reading from RS-422
  pinMode(DE_Sensor, OUTPUT); 
  pinMode(RE_Sensor, OUTPUT);
  Serial.println("Sensor initialized.");
  // ----------------------------------------------

  // --------------- SENSORHUB SETUP ---------------
  Serial5.begin(115200, SERIAL_8N1_RXINV); // Serial5 is for receiving from the sensor hub
  Serial4.begin(115200, SERIAL_8N1_RXINV); // Serial4 is for sending to the sensor hub
  pinMode(RE_Senshub, OUTPUT);
  pinMode(DE_Senshub, OUTPUT);
  digitalWrite(RE_Senshub, LOW); // Enable receiver mode (inverted)
  digitalWrite(DE_Senshub, HIGH); // Enable receiver mode (inverted)
  Serial.println("Sensor hub initialized.");
  timer.begin(sendData, 1e6/sendingFrequency); // Start the timer to send response every 1/10second
  // -----------------------------------------------
  
  // ----------------- SENSOR SETUP ----------------
  sendCommandToSensor(STOP); // Request: stop sending
  delay(100);
  sendCommandToSensor(CYCLIC_POS_VEL); // Request: send cyclic position and velocity

  Serial.println("Sensor setup complete.");
  // -----------------------------------------------

  // ----------------- FIRST SENSOR READING ----------------

  uint8_t received[9];
  // Wait until sensor sends values
  while (Serial1.available() < 9) {    
    Serial.println("Waiting for 1st sensor reading...");
    delay(100);
    sendCommandToSensor(STOP); // Request: stop sending
    delay(100);
    sendCommandToSensor(CYCLIC_POS_VEL); // Request: send cyclic position and velocity
  
  }
  Serial1.readBytes(received, 9);

  sensorReading = 0;
  velocityValue = 0;

  // Get position from bytes
  for (int i = 2; i < 6; i++) {
    sensorReading = (sensorReading << 8) | received[i];
  }

  barcode_estimator(sensorReading/1e5, -1, sensorReading/1e5, velocityValue, std::vector<TrackData>(rightDataTable, rightDataTable + rightDataTableSize), trackId, absPosition, validPosition);
  prevAbsPosition = sensorReading/1e5; // Set the previous barcode to the current position

  // -----------------------------------------------

  Serial.println("Setup complete.");
}

void loop() {
  // ----------------- GET SENSOR DATA ----------------
  uint8_t received[9];
  if (Serial1.available() >= 9)
  {

    Serial1.readBytes(received, 9);

    sensorReading = 0;
    velocityValue = 0;

    // Get position from bytes
    for (int i = 2; i < 6; i++)
    {
      sensorReading = (sensorReading << 8) | received[i]; // in 0.01 mm
    }

    // Get velocity from bytes
    for (int i = 6; i < 8; i++) {
      velocityValue = (velocityValue << 8) | received[i]; // in mm/s
    }

    // ----------------- DE-SCRAMBLE DATA ----------------
    findAbsPosition(sensorReading/1e5, trackId, prevAbsPosition, velocityValue/1e3, std::vector<TrackData>(rightDataTable, rightDataTable + rightDataTableSize), absPosition);
    
    // Update previous value if new one is correct
    if (validPosition) {
      prevAbsPosition = absPosition;
    }
    // ---------------------------------------------------
      
    // Populate CAN frame
    for (size_t i = 0; i < 4; i++) {
        CANframe[i] = ((uint32_t)(absPosition*1e5) >> (8 * (3 - i))) & 0xFF;  // Big-endian (MSB first)
    }
    CANframe[5] = (abs(velocityValue) >> 8);
    CANframe[6] = (abs(velocityValue) & 0xFF);


    // Print if value within bounds (120m is max in EHC)
    if (absPosition < 120){
      Serial.print("\r|| Received position: ");
      Serial.print(sensorReading/1e5, 5);
      Serial.print(" m. Received velocity: ");
      Serial.print(velocityValue/1e3, 5);
      Serial.print(" m/s   ||   ");
      
      Serial.print("Absolute position: ");
      Serial.print(absPosition, 5);
      Serial.print(" m   ||   ");
    } else {
      Serial.println("Out of bounds value (>120 m): " + String(absPosition));
    }
    // if (heartbeatHandler.connected) { Serial.print(heartbeatHandler.heartbeatChar + "   ||  "); }
  }
  // if (Serial5.available() > 0) {
  //     uint8_t id = Serial5.read(); // Read the ID byte
  // }

  // ---------------------------------------------------



  // ----------------- SENSOR HUB COMMUNICATION ----------------
  // if (Serial5.available() > 0) {
  //   Serial.println("Data received from sensor hub.");
  //   uint8_t id = Serial5.read(); // Read the ID byte
  //   switch (id) {
  //     case HEARTBEAT_BYTE:
  //       if (!heartbeatHandler.connected) {
  //         Serial.println("\n\n==================================================");
  //         Serial.println("Received heartbeat from Sensor Hub while being disconnected. Performing handshake...");
  //         doHandshake(); // Perform handshake with sensor hub. If not succesful, Teensy will reset
  //         heartbeatHandler.connect(); // Start heartbeat timer

  //       } else {
  //         heartbeatHandler.beat(); // Update heartbeat time
  //       }
  //     break;
  //     case STOP_BYTE:
  //       Serial.println("Stop byte received from Sensor Hub.");
  //       timer.end(); // Stop the timer
  //       break;
  //     case RESET_BYTE:
  //       Serial.println("Reset byte received from Sensor Hub. Resetting Teensy...");
  //       resetTeensy(); // Reset Teensy
  //       break;
  //     default:
  //       Serial.print("Unknown command received from Sensor Hub: 0x");
  //       Serial.println(id, HEX);
  //       break;
  //   }
  // }
}

void findAbsPosition(float barcode, int &trackId, float prevAbsPosition, float velocity, 
                   const std::vector<TrackData> &tracks, float &absPosition) {
  int trackIdPrev, trackIdNext, trackIdCurrent = trackId;

  if (trackId == 0) {
    trackIdPrev = 0;
    trackIdNext = 1;
  } else if (trackId == tracks.size() - 1) {
    trackIdPrev = trackId - 1;
    trackIdNext = trackId;
  } else {
    trackIdPrev = trackId - 1;
    trackIdNext = trackId + 1;
  }

  const TrackData &trackPrev = tracks[trackIdPrev];
  const TrackData &trackNext = tracks[trackIdNext];
  const TrackData &trackCurrent = tracks[trackIdCurrent];

  float distance_prev = (trackPrev.id != trackCurrent.id) ? 
    fabs(prevAbsPosition - trackPrev.actualEnd) : 5000;
  float distance_next = (trackNext.id != trackCurrent.id) ? +
    fabs(prevAbsPosition - trackNext.actualStart) : 5000;

  const TrackData *trackOther = (distance_prev <= distance_next) ? &trackPrev : &trackNext;
  
  validPosition = true; // used to update prevAbsPosition
  if (check_range(barcode, *trackOther)) {
    trackId = trackOther->id;
    absPosition = calculate_position(barcode, *trackOther);
  } else if (check_range(barcode, trackCurrent)) {
    trackId = trackCurrent.id;
    absPosition = calculate_position(barcode, trackCurrent);
  } else {
    trackId = trackCurrent.id;
    absPosition = prevAbsPosition + velocity * 200;
    validPosition = false;
  }
}

bool check_range(float barcode, const TrackData &track) {
  if (track.barcodeStart < track.barcodeEnd) {
    return barcode >= track.barcodeStart + 0.05 && barcode <= track.barcodeEnd - 0.05;
  } else {
    return barcode <= track.barcodeStart - 0.05 && barcode >= track.barcodeEnd + 0.05;
  }
}

float calculate_position(float barcode, TrackData track){

  if (track.barcodeStart <= track.barcodeEnd) { // Starboard
    // Linear interpolation to calculate the position based on the barcode
    return track.actualStart + (barcode - track.barcodeStart);
  } else {
    return track.actualStart + (track.barcodeStart - barcode); // Return 0 or some error value
  }
}

bool check_valid(float barcode, const TrackData &track) {
  if (track.barcodeStart < track.barcodeEnd) {
    return barcode >= track.barcodeStart + 0.2 && barcode <= track.barcodeEnd - 8.2;
  } else {
    return barcode <= track.barcodeStart - 9.2 && barcode >= track.barcodeEnd + 0.2;
  }
}

void barcode_estimator(float barcode, int trackIdPrev, float prevAbsPosition, float velocity, const std::vector<TrackData> &tracks,
                       int &trackId, float &absPosition, bool &validPosition) {
  
  trackId = trackIdPrev;
  absPosition = 0;
  validPosition = false;

  if (trackIdPrev == -1) {
    int index = 0;
    while (index < tracks.size()) {
      const TrackData &track = tracks[index];
      if (check_range(barcode, track)) {
        trackId = track.id;
        absPosition = calculate_position(barcode, track);
        findAbsPosition(barcode, trackId, prevAbsPosition, velocity, tracks, absPosition);
        break;
      }
      index++;
    }

    if (index >= tracks.size()) {
      trackId = -1;
      absPosition = -1;
    }
  }

  if (trackId != -1) {
    validPosition = check_valid(barcode, tracks[trackId]);
  }

}

void sendData() {
  CANframe[0] = SLAVE_ID; // id
  CANframe[7] = (CANframe[0] + CANframe[1] + CANframe[2] + CANframe[3] + CANframe[4] + CANframe[5] + CANframe[6]) % 256; // checksum

  Serial.print("Sent data to Sensor Hub: ");

  for (size_t i = 0; i < 8; i++)
  {
    Serial.print("0x");
    Serial.print(CANframe[i], HEX);
    Serial.print(" ");
  }

  Serial4.write(CANframe, 8);
  Serial4.flush();
}


void sendCommandToSensor(CommandType cmd)
{
  dataToSend[0] = 0xC0;

  switch (cmd)
  {
  case STOP:
    dataToSend[1] = 0xF3;
    dataToSend[2] = 0x33;
    break;
  case CYCLIC_POS:
    dataToSend[1] = 0xF2;
    dataToSend[2] = 0x32;
    break;
  case CYCLIC_POS_VEL:
    dataToSend[1] = 0xF9;
    dataToSend[2] = 0x39;
    break;
  case SINGLE_POS:
    dataToSend[1] = 0xF1;
    dataToSend[2] = 0x31;
    break;
  case SINGLE_POS_VEL:
    dataToSend[1] = 0xF8;
    dataToSend[2] = 0x38;
    break;
  case STANDBY:
    dataToSend[1] = 0xFD;
    dataToSend[2] = 0x3D;
    break;
  case UNKNOWN:
    Serial.println("Unknown command");
    return;
    break;
  default:
    Serial.println("Unknown command");
    return;
    break;
  }

  Serial.print("Sending command: ");
  Serial.println(cmd);

  digitalWrite(DE_Sensor, HIGH); // Enable driver mode
  digitalWrite(RE_Sensor, HIGH); // Disable receiver mode (inverted)

  Serial6.write(dataToSend, sizeof(dataToSend)); // Send data over RS-422
  Serial6.flush();                                // Wait for serial to finish sending

  Serial.println("Command sent!\n");

  digitalWrite(DE_Sensor, LOW); // Disable driver mode
  digitalWrite(RE_Sensor, LOW); // Enable receiver mode (inverted)
}

void resetTeensy(){
  // Reset the Teensy board
  SCB_AIRCR = 0x05FA0004;
  while (1);  
}

// This all is a bit overkill, only when you wanna update data without flashing. 
// I will leave it for now, but it could be skipped without many consequences
void writeTrackDataToEEPROM() {
  const uint8_t initByte = 0x42;

  if (EEPROM.read(0) == initByte) {
    Serial.println("Track data already in EEPROM.");
    return;
  }

  int address = 1; // Start writing after the init byte
  Serial.println("Writing init byte to EEPROM...");
  EEPROM.write(0, initByte); // Write init byte to indicate data is present
  Serial.println("Writing track data to EEPROM...");
  
  for (int i = 0; i < leftDataTableSize; i++) {
    EEPROM.put(address, trackDataLeft[i]);
    address += sizeof(TrackData);
  }

  for (int i = 0; i < rightDataTableSize; i++) {
    EEPROM.put(address, trackDataRight[i]);
    address += sizeof(TrackData);
  }

  Serial.println("Track data written to EEPROM successfully.");

}

void readTrackDataFromEEPROM() {
  const uint8_t initByte = 0x42;

  if (EEPROM.read(0) != initByte) {
    Serial.println("No track data in EEPROM.");
    return;
  }

  int address = 1; // Start reading after the init byte
  Serial.println("Reading track data from EEPROM...");

  for (int i = 0; i < leftDataTableSize; i++) {
    EEPROM.get(address, trackDataLeft[i]);
    address += sizeof(TrackData);
  }

  for (int i = 0; i < rightDataTableSize; i++) {
    EEPROM.get(address, trackDataRight[i]);
    address += sizeof(TrackData);
  }

  Serial.println("Track data read from EEPROM successfully.");
}

void doHandshake(){
  // Handshake description: 
    // 1. Receive ping from sensor hub (happens in main loop)
    // 2. Send init handshake frame [1,1,1,1,1] to sensor hub 
    // 3. Wait for order frame [1,2,3,4,5] from sensor hub 
    // 4. Send order frame [1,2,3,4,5] back to sensor hub 
    // 5. Wait for end frame [1,1,1,1,1] from sensor hub 

  // 2. Send init handshake frame to sensor hub 
  Serial4.write(handshakeEndFrame, sizeof(handshakeInitFrame));
  Serial4.flush();
  Serial.println("Handshake sent to Sensor Hub. Waiting for response... ");
  
  // 3. Wait for order frame from sensor hub, timeout after 200 ms (reset after timeout)
  unsigned long startTime = millis();
  uint8_t receivedHandshake[5];

  while (Serial5.available() < 5){
    if (Serial5.peek() == 0x50){
      Serial5.read(); // Read the stop byte
      Serial.println("Stop byte received. Stopping data stream.");
      return;
    }

    if (millis() - startTime > 500) { // Timeout after 500 ms
          Serial.println("No response received (timeout). Handshake failed." + String(millis() - startTime));
          Serial.println("Resetting Teensy...");
          delay(100);
          resetTeensy(); // Reset Teensy if no response is received
          return;
      }
  }   

  // Check if frame good (reset Teensy if not)
  Serial5.readBytes(receivedHandshake, 5);
  if (memcmp(receivedHandshake, handshakeOrderFrame, 5) != 0){
      Serial.print("Received the wrong frame from Sensor Hub: [ ");

      for (int i; i < 5; i++){
          Serial.print(receivedHandshake[i]);
          Serial.print(" ");
      }
      Serial.println("]. Handshake failed. Resetting Teensy...");
      delay(100);
      resetTeensy(); // Reset Teensy if the frame is not correct
      return;
  }

  // 4. Send order frame back
  Serial4.write(handshakeOrderFrame, sizeof(handshakeOrderFrame));
  Serial4.flush();
  Serial.println("Order frame sent back to Sensor Hub. Waiting for end frame...");

  // 5. Wait for end frame from sensor hub, timeout after 200 ms (reset after timeout)
  startTime = millis();
  while (Serial5.available() < 5){
      if (Serial5.peek() == 0x50){
          Serial5.read(); // Read the stop byte
          Serial.println("Stop byte received. Stopping data stream.");
          return;
      }
      if (millis() - startTime > 500) { // Timeout after 200 ms
          Serial.println("No end frame received (timeout). Handshake failed." + String(millis() - startTime));
          Serial.println("Resetting Teensy...");
          delay(100);
          resetTeensy(); // Reset Teensy if no response is received
          return;
      }
  }

  // Check if frame good (reset Teensy if not)
  Serial5.readBytes(receivedHandshake, 5);
  if (memcmp(receivedHandshake, handshakeEndFrame, 5) != 0){
      Serial.print("Received the wrong frame from Sensor Hub: [ ");

      for (int i; i < 5; i++){
          Serial.print(receivedHandshake[i]);
          Serial.print(" ");
      }
      Serial.println("]. Handshake failed. Resetting Teensy...");
      delay(100);
      resetTeensy(); // Reset Teensy if the frame is not correct
      return;
  }

  Serial.println("Handshake successful! Starting data stream.");
  timer.begin(sendData, 1e6/sendingFrequency);
  Serial.println("==================================================\n\n");
   
}