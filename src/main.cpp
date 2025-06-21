#include <Arduino.h>
#include <EEPROM.h>

#define RE_Sensor 2
#define DE_Sensor 3

#define RE_Senshub 19
#define DE_Senshub 18

#define STOP_BYTE 0x50
#define START_BYTE 0x80
#define CMD_REQUEST_DATA 0x01
#define CMD_SET_CONFIG   0x02

#define CONFIG_ADDR 0
#define SLAVE_ID_ADDR 50 // Where we store slave ID

uint8_t SLAVE_ID;
uint8_t configValue;
byte dataToSend[3];
IntervalTimer timer;

int16_t velocityValue = 0;
uint32_t positionValue = 0;
uint8_t velocityArray[2];
uint8_t positionArray[4];

uint8_t frame[8];
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

void sendCommandToSensor(CommandType cmd);
void sendData();

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
  
  // -------------------- EEPROM --------------------
  // EEPROM.write(SLAVE_ID_ADDR, 0xB1); // Initialize EEPROM
  // SLAVE_ID = EEPROM.read(SLAVE_ID_ADDR);
  // if (SLAVE_ID == 0xFF) {
  //   SLAVE_ID = 0xB1; // Default ID if unset
  //   EEPROM.write(SLAVE_ID_ADDR, SLAVE_ID);
  // }
  SLAVE_ID = 0xB1; // Default ID if unset
  configValue = EEPROM.read(CONFIG_ADDR);
  Serial.println("EEPROM initialized.");
  // ------------------------------------------------

  // ----------------- SENSOR SETUP ----------------
  sendCommandToSensor(STOP); // Request: stop sending
  delay(1000);
  sendCommandToSensor(CYCLIC_POS_VEL); // Request: send cyclic position and velocity
  digitalWrite(DE_Sensor, LOW); // Disable driver mode
  digitalWrite(RE_Sensor, LOW); // Enable receiver mode (inverted)

  Serial.println("Sensor setup complete.");
  // -----------------------------------------------

  Serial.println("Setup complete.");
}

void loop() {
  uint8_t received[9];

  // TODO: 9 bytes is for cyclic, 7 for non cyclic. fix
  if (Serial1.available() >= 9)
  {
    Serial1.readBytes(received, 9);

    positionValue = 0;
    velocityValue = 0;

    // Get position from bytes
    for (int i = 2; i < 6; i++)
    {
      positionValue = (positionValue << 8) | received[i];
      positionArray[i-2] = received[i];
    }

    velocityArray[0] = received[6];
    velocityArray[1] = received[7];

    // Get velocity from bytes
    for (int i = 6; i < 8; i++)
    {
      velocityValue = (velocityValue << 8) | received[i];
    }
    if (positionValue < 1188000){
      Serial.print("\r||   Position: ");
      Serial.print(positionValue);
      Serial.print(". Velocity: ");
      Serial.print(velocityValue);
      Serial.print("   ||   ");
      // Serial4.write(positionArray, 4); // Send data over RS-422
      for (size_t i = 0; i < 4; i++)
      {
        frame[i+1] = positionArray[i];
      }
      frame[5] = (abs(velocityValue) >> 8);
      frame[6] = (abs(velocityValue) & 0xFF);
    }
  }
  // if (Serial5.available() > 0) {
  //   Serial.println("Data received from sensor hub.");

  //   // Read the start byte
  //   if (Serial5.read() == START_BYTE) {
  //     Serial.println("Start byte received, sending data...");
  //     timer.end(); // Stop the timer
  //     timer.begin(sendData, 100000); // Start the timer to send response every 1/10second
  //   }
  //   if (Serial5.read() == STOP_BYTE) {
  //     Serial.println("Stop byte received, stopping data sending.");
  //     // Read the command byte
  //     timer.end(); // Stop the timer
  //   }
  // }
}

void sendData() {
  frame[0] = SLAVE_ID;
  frame[7] = (frame[0] + frame[1] + frame[2] + frame[3] + frame[4] + frame[5] + frame[6]) % 256;
  for (size_t i = 0; i < 8; i++)
  {
    Serial.print("0x");
    Serial.print(frame[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  Serial4.write(frame, 8);
  Serial4.flush();
}


// #include <Arduino.h>
// #include <FlexCAN_T4.h>
// #include <Flexcan_T4.h>

// #define RE_Sensor 2
// #define DE_Sensor 3

// #define RE_Temphub 19
// #define DE_Temphub 18


// bool REQUEST_CYCLIC = true;

// int msgLength = 9;

// long lastSendTime = 0;
// long requestInterval = 1000;
// long lastPrintTime = 0;
// long printInterval = 1000;

// long timeSinceLastByte = 0;
// long byteTime[9];

// long msgCounter = 0;
// long msgCounterReceived = 0;

// bool sendToSensorhub = false;

// uint32_t prevPosition = -1000;
// FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can2;
// CAN_message_t msg;

// float precisionRangeStart = 20;
// float precisionRangeEnd = 30;


// void printByteArrayBinary(uint8_t *arr, size_t length)
// {
//   Serial.print("{ ");
//   for (size_t i = 0; i < length; i++)
//   {
//     for (int bit = 7; bit >= 0; bit--)
//     {
//       Serial.print((arr[i] >> bit) & 1); // Extract and print each bit
//     }
//     if (i < length - 1)
//       Serial.print(", "); // Add comma between bytes
//   }
//   Serial.print(" }");
// }


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

// void sendDataToSensorHub(uint32_t pos)
// {
//   digitalWrite(DE_Temphub, HIGH); // Enable driver mode
//   digitalWrite(RE_Temphub, HIGH); // Disable receiver mode (inverted)

//   Serial4.write(pos); // Send data over RS-422
//   Serial4.flush();                 // Wait for serial to finish sending

//   digitalWrite(DE_Temphub, LOW); // Disable driver mode
//   digitalWrite(RE_Temphub, LOW); // Enable receiver mode (inverted)
// }

// void setup()
// {
//   Serial.begin(9600); // USB serial
//   delay(2000);
//   Serial.println("Starting");

//   // %%% SENSOR SETUP %%%
//   // RS-422 serial
//   // Possible baud rates: 4800, 9600, 19200, 38400, 57600, 115200
//   Serial1.begin(115200); // Serial 1 is for receiving from the sensor
//   Serial6.begin(115200); // Serial 6 is for sending to the sensor
//   Serial1.setTimeout(1); // Set timeout for reading from RS-422

//   pinMode(DE_Sensor, OUTPUT);
//   pinMode(RE_Sensor, OUTPUT);

//   delay(100); // Wait for the RS-422 to initialize

//   // Request: stop sending
//   sendCommandToSensor("STOP");
//   delay(1000);

//   // Flush RS-422 buffer
//   while (Serial1.available() > 0)
//   {
//     Serial1.read();
//   }

//   // Request: start cyclic sending (see webConfig for cycle time)
//   if (REQUEST_CYCLIC)
//   {
//     sendCommandToSensor("CYCLIC_POS_VEL");
//   }

//   delay(50);

//   // %%% TEMPHUB SETUP %%%
//   Serial5.begin(115200, SERIAL_8N1_RXINV); // Serial 5 is for receiving from the temp hub
//   Serial4.begin(115200, SERIAL_8N1_RXINV); // Serial4 is for sending to the temp hub
 
//   pinMode(DE_Temphub, OUTPUT);
//   pinMode(RE_Temphub, OUTPUT);

//   digitalWrite(DE_Temphub, HIGH); // Disable driver mode
//   digitalWrite(RE_Temphub, LOW); // Enable receiver mode (inverted)
// }

// void loop()
// {
//   uint8_t received[9];

//   if (!REQUEST_CYCLIC){
//     if (millis() - lastSendTime > requestInterval) {
//       Serial.println("\nRequesting data");
//       lastSendTime = millis();
//       sendCommandToSensor("SINGLE_POS_VEL");
//     }
//   }

//   // TODO: 9 bytes is for cyclic, 7 for non cyclic. fix
//   if (Serial1.available() >= 9)
//   {

//     Serial1.readBytes(received, 9);
//     msgCounter++;
    

//     positionValue = 0;
//     vel = 0;

//     // Get position from bytes
//     for (int i = 2; i < 6; i++)
//     {
//       positionValue = (positionValue << 8) | received[i];
//       positionArray[i-2] = received[i];
//     }

//     velocityArray[0] = received[6];
//     velocityArray[1] = received[7];

//     // Get velocity from bytes
//     for (int i = 6; i < 8; i++)
//     {
//       vel = (vel << 8) | received[i];
//     }
//     if (positionValue < 9e6){
//       Serial.print("\r||   Position: ");
//       Serial.print(positionValue);
//       Serial.print(". Velocity: ");
//       Serial.print(vel);
//       Serial.print("   ||");
//       // Serial4.write(positionArray, 4); // Send data over RS-422
//       for (size_t i = 0; i < 4; i++)
//       {
//         sendArray[i] = positionArray[i];
//         Serial.print(i);
//         Serial.print(": ");
//         Serial.print(sendArray[i]);
//         Serial.print(", ");
//       }
//       sendArray[4] = (abs(vel) >> 8);
//       sendArray[5] = (abs(vel) & 0xFF);
//       Serial.print("4");
//       Serial.print(": ");
//       Serial.print(sendArray[4]);
//       Serial.print(", ");
//       Serial.print("5");
//       Serial.print(": ");
//       Serial.print(sendArray[5]);
//       Serial4.write(sendArray, 6);
//       Serial4.flush(); // Wait for serial to finish sending
//     }
//   }


  
// }