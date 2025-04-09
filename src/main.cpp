#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Flexcan_T4.h>

#define RE_Sensor 31
#define DE_Sensor 32

#define RE_Temphub 26
#define DE_Temphub 27


bool REQUEST_CYCLIC = true;

int msgLength = 9;
byte dataToSend[3];
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

long lastSendTime = 0;
long requestInterval = 1000;
long lastPrintTime = 0;
long printInterval = 1000;

long timeSinceLastByte = 0;
long byteTime[9];

long msgCounter = 0;
long msgCounterReceived = 0;

bool sendToSensorhub = false;

uint32_t position = 0;
uint32_t prevPosition = -1000;
uint32_t vel = 0;

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
CAN_message_t msg;

float precisionRangeStart = 20;
float precisionRangeEnd = 30;


void canSniff(const CAN_message_t &msg)
{
  Serial.print("     ||  CAN MESSAGES:  {");
  Serial.print("CAN ID: ");
  Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for (uint8_t i = 0; i < msg.len; i++)
  {
    Serial.print(msg.buf[i]);
    Serial.print(" ");
  }
  Serial.print("}  ||");
}

void sendCAN(int ID, uint8_t len, uint8_t *data)
{
  CAN_message_t msg;
  msg.id = ID; // Set the CAN ID
  msg.len = len; // Set the length of the message
  for (int i = 0; i < len; i++)
  {
    msg.buf[i] = data[i]; // Copy the received data to the CAN message buffer
  }
  can2.write(msg); // Send the CAN message
}
float decodeTemperature(uint8_t encoded) {
  if (encoded & 0x80) {  // High precision mode
      return precisionRangeStart + (encoded & 0x7F) / 10.0;
  } else {  // Integer mode
      return (float)encoded;
  }
}

void printByteArrayBinary(uint8_t *arr, size_t length)
{
  Serial.print("{ ");
  for (size_t i = 0; i < length; i++)
  {
    for (int bit = 7; bit >= 0; bit--)
    {
      Serial.print((arr[i] >> bit) & 1); // Extract and print each bit
    }
    if (i < length - 1)
      Serial.print(", "); // Add comma between bytes
  }
  Serial.println(" }");
}

CommandType getCommandFromString(const String &command)
{
  if (command == "STOP")
  {
    return STOP;
  }
  if (command == "CYCLIC_POS")
  {
    return CYCLIC_POS;
  }
  if (command == "CYCLIC_POS_VEL")
  {
    return CYCLIC_POS_VEL;
  }
  if (command == "SINGLE_POS")
  {
    return SINGLE_POS;
  }
  if (command == "SINGLE_POS_VEL")
  {
    return SINGLE_POS_VEL;
  }
  if (command == "STANDBY")
  {
    return STANDBY;
  }
  return UNKNOWN;
}

void sendCommandToSensor(const String command)
{
  dataToSend[0] = 0xC0;

  CommandType cmd = getCommandFromString(command);

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
  Serial.println(command);

  digitalWrite(DE_Sensor, HIGH); // Enable driver mode
  digitalWrite(RE_Sensor, HIGH); // Disable receiver mode (inverted)
  // delay(5);  //allow sending to complete
  Serial7.write(dataToSend, sizeof(dataToSend)); // Send data over RS-422
  Serial7.flush();                                // Wait for serial to finish sending

  Serial.println("Command sent!\n");

  digitalWrite(DE_Sensor, LOW); // Disable driver mode
  digitalWrite(RE_Sensor, LOW); // Enable receiver mode (inverted)
}

void sendDataToSensorHub(uint32_t pos)
{
  digitalWrite(DE_Temphub, HIGH); // Enable driver mode
  digitalWrite(RE_Temphub, HIGH); // Disable receiver mode (inverted)

  Serial6.write(pos); // Send data over RS-422
  Serial6.flush();                 // Wait for serial to finish sending

  digitalWrite(DE_Temphub, LOW); // Disable driver mode
  digitalWrite(RE_Temphub, LOW); // Enable receiver mode (inverted)
}

void setup()
{
  Serial.begin(9600); // USB serial
  delay(2000);
  Serial.println("Starting");

  // %%% SENSOR SETUP %%%
  // RS-422 serial
  Serial7.begin(115200); // Possible baud rates: 4800, 9600, 19200, 38400, 57600, 115200
  Serial7.setTimeout(1); // Set timeout for reading from RS-422
  // Serial1.setTX(24);

  pinMode(DE_Sensor, OUTPUT);
  pinMode(RE_Sensor, OUTPUT);

  delay(100); // Wait for the RS-422 to initialize

  // Request: stop sending
  sendCommandToSensor("STOP");
  delay(1000);

  // Flush RS-422 buffer
  while (Serial7.available() > 0)
  {
    Serial7.read();
  }

  // Request: start cyclic sending (see webConfig for cycle time)
  if (REQUEST_CYCLIC)
  {
    sendCommandToSensor("CYCLIC_POS_VEL");
  }

  digitalWrite(DE_Sensor, LOW); // Disable driver mode
  digitalWrite(RE_Sensor, LOW); // Enable receiver mode (inverted)

  delay(50);

  // %%% TEMPHUB SETUP %%%
  Serial6.begin(9600); // Serial4 is the sensorhub
  Serial6.setTimeout(1); // Set timeout for reading from RS-422

  pinMode(DE_Temphub, OUTPUT);
  pinMode(RE_Temphub, OUTPUT);

  digitalWrite(DE_Temphub, LOW); // Disable driver mode
  digitalWrite(RE_Temphub, LOW); // Enable receiver mode (inverted)
 
  // % CAN SETUP %%%
  can2.begin();
  can2.setBaudRate(1000000);
}

void loop()
{
  uint8_t received[9];

  if (!REQUEST_CYCLIC){
    if (millis() - lastSendTime > requestInterval) {
      Serial.println("\nRequesting data");
      lastSendTime = millis();
      sendCommandToSensor("SINGLE_POS_VEL");
    }
  }

  // TODO: 9 bytes is for cyclic, 7 for non cyclic. fix
  if (Serial7.available() >= 9)
  {
    Serial7.readBytes(received, 9);
    msgCounter++;

    position = 0;
    vel = 0;

    // Get position from bytes
    for (int i = 2; i < 6; i++)
    {
      position = (position << 8) | received[i];
    }

    // Get velocity from bytes
    for (int i = 6; i < 8; i++)
    {
      vel = (vel << 8) | received[i];
    }

    sendToSensorhub = true;

    sendCAN(0x320, sizeof(received), received); // Send the CAN message
    Serial.print("\r||   Position: ");
    Serial.print(position);
    Serial.print(". Velocity: ");
    Serial.print(vel);
    Serial.print("   ||");

  }


  // if (millis() - lastPrintTime > printInterval)
  // {
  //   lastPrintTime = millis();
  //   Serial.print("||   Msgs: ");
  //   Serial.print(msgCounter);
  //   Serial.print(" in ");
  //   Serial.print(printInterval);
  //   Serial.println("ms   ||    ");
  //   msgCounter = 0;
  // }

  
  if (Serial6.available()>= 7) {
    uint8_t received_temp[7];
    Serial6.readBytes(received_temp, 7);

    sendCAN(0x33A, sizeof(received_temp), received_temp); // Send the CAN message

    Serial.print("       Received from temp hub: ");
    Serial.print("{ ");
    for (int i = 0; i < 7; i++)
    {
      Serial.print(decodeTemperature(received_temp[i]));
      Serial.print(" ");
    }
    Serial.print("}  ||");
  }

  if (can2.read(msg))
  {
    Serial.print("CAN message received: ");
    Serial.print("ID: ");
    Serial.print(msg.id, HEX);
    Serial.print(" Length: ");
    Serial.print(msg.len);
    Serial.print(" Data: ");
    for (int i = 0; i < msg.len; i++)
    {
      byte b = msg.buf[i];
      Serial.print(b, HEX);
      Serial.print(" (");
      for (int bit = 7; bit >= 0; bit--)
      {
        Serial.print((b >> bit) & 1);
      }
      Serial.print("); ");
    }
    Serial.println();
    
  }

  if (millis() - lastPrintTime > printInterval) {
    msg.id = 0x4B4; // Set the CAN ID
    msg.len = 8; // Set the length of the message
    msg.buf[0] = 10;
    msg.buf[1] = 20;
    msg.buf[2] = 30;
    msg.buf[3] = 1;
    msg.buf[4] = 40;
    msg.buf[5] = 50;
    msg.buf[6] = 60;
    msg.buf[7] = 70;

    can2.write(msg); // Send the CAN message
    lastPrintTime = millis();
    Serial.println("CAN message sent");
  }
}