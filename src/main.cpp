#include <Arduino.h>
#include <FlexCAN_T4.h>

#define RE 31
#define DE 32

bool REQUEST_CYCLIC = true;

int msgLength = 9;
byte dataToSend[3];
enum CommandType {
  STOP,
  CYCLIC_POS,
  CYCLIC_POS_VEL,
  SINGLE_POS,
  SINGLE_POS_VEL,
  STANDBY,
  UNKNOWN,
};

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
CAN_message_t msg;

long lastSendTime = 0;
long requestInterval = 1000;
long lastPrintTime = 0;
long printInterval = 1000;
long timeSinceLastByte = 0;
long byteTime[9];
long msgCounter = 0;
long msgCounterReceived = 0;

bool sendCAN = false;

uint32_t position = 0;
uint32_t prevPosition = 0;
uint32_t vel = 0;

void canSniff(const CAN_message_t &msg) {
    Serial.print("     ||  CAN MESSAGES:  {");
    Serial.print("CAN ID: "); Serial.print(msg.id, HEX);
    Serial.print(" Buffer: ");
    for ( uint8_t i = 0; i < msg.len; i++ ) {
      Serial.print(msg.buf[i]); Serial.print(" ");
    } Serial.print("}  ||");                                           
  }

  
void printByteArrayBinary(uint8_t *arr, size_t length) {
  Serial.print("{ ");
  for (size_t i = 0; i < length; i++) {
      for (int bit = 7; bit >= 0; bit--) {
          Serial.print((arr[i] >> bit) & 1);  // Extract and print each bit
      }
      if (i < length - 1) Serial.print(", ");  // Add comma between bytes
  }
  Serial.println(" }");
}

CommandType getCommandFromString(const String &command){
  if (command == "STOP"){return STOP;}
  if (command == "CYCLIC_POS"){return CYCLIC_POS;}
  if (command == "CYCLIC_POS_VEL"){return CYCLIC_POS_VEL;}
  if (command == "SINGLE_POS"){return SINGLE_POS;}
  if (command == "SINGLE_POS_VEL"){return SINGLE_POS_VEL;}
  if (command == "STANDBY"){return STANDBY;}
  return UNKNOWN;
}

void sendCommand(const String command){
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

  digitalWrite(DE, HIGH);  // Enable driver mode
  digitalWrite(RE, HIGH);  // Disable receiver mode (inverted)
  // delay(5);  //allow sending to complete
  Serial7.write(dataToSend, sizeof(dataToSend));  // Send data over RS-422
  Serial.flush(); // Wait for serial to finish sending

  Serial.println("Command sent!\n");

  digitalWrite(DE, LOW);  // Disable driver mode
  digitalWrite(RE, LOW);  // Enable receiver mode (inverted)
}

void setup() {
  Serial.begin(9600);   // USB serial
  delay(2000);
  Serial.println("Starting");
  // RS-422 serial
  Serial7.begin(115200);  // Possible baud rates: 4800, 9600, 19200, 38400, 57600, 115200
  Serial7.setTimeout(1);  // Set timeout for reading from RS-422

  pinMode(DE, OUTPUT);
  pinMode(RE, OUTPUT);

  delay(100);  // Wait for the RS-422 to initialize

  // Request: stop sending
  sendCommand("STOP");
  delay(1000);  

  // Flush RS-422 buffer
  while (Serial7.available() > 0) {
    Serial7.read();
  }

  // Request: start cyclic sending (see webConfig for cycle time)
  if (REQUEST_CYCLIC){
    sendCommand("CYCLIC_POS_VEL");
  }

  digitalWrite(DE, LOW);  // Disable driver mode
  digitalWrite(RE, LOW);  // Enable receiver mode (inverted)

  delay(50);
  // Setup CAN
  can2.begin();
  can2.setBaudRate(1000000);
  Serial.println("CAN2 initialized");
  
}

void loop() {
  uint8_t received[9];
  // sendCAN = true;

  // if (!REQUEST_CYCLIC){
  //   if (millis() - lastSendTime > requestInterval) {
  //     Serial.println("\nRequesting data");
  //     lastSendTime = millis();
  //     sendCommand("SINGLE_POS_VEL");
  //   }
  // }
  if (Serial7.available() >= 9){
    Serial7.readBytes(received, 9);
    msgCounter++;

    position = 0;
    vel = 0;

    // Get position from bytes
    for (int i = 2; i < 6; i++){
      position = (position<<8) | received[i];
    }

    // Get velocity from bytes
    for (int i = 6; i < 8; i++){
      vel = (vel<<8) | received[i];
    }

    sendCAN = true;

      // Serial.print("\r||   Position: ");
      // Serial.print(position);
      // Serial.print(". Velocity: ");
      // Serial.print(vel);
      // Serial.print("   ||    ");
  }
  // for (int i = 0; i < 9; i++){
  //   Serial.print(received[i]);
  //   Serial.print(", ");
  // }
  // printByteArrayBinary(received, 9);
  // if (Serial7.available() >= 9) {
  //   msgCounter++;
  //   position = 0; 
  //   vel = 0;

  //   // while (Serial7.available() > 0){
  //   //   uint8_t firstByte = Serial7.peek();
  //   //   if (firstByte == 80 || firstByte == 64) {break;}
  //   //   Serial7.read();  // Discard misaligned bytes
  //   // }
  //   timeSinceLastByte = millis();
  //   for (int i = 0; i <= 8; i++){
  //     received[i] = Serial7.read();
  //     Serial.print("");
  //     Serial.print(received[i]);
  //     Serial.print(", ");
  //     byteTime[i] = millis() - timeSinceLastByte;
  //     timeSinceLastByte = millis();

  //     if (i > 1 && i < 6){
  //       position = (position<<8) | received[i];
  //     }

  //     if (i > 5 && i < 8){
  //       vel = (vel<<8) | received[i];
  //     }

  //   }
  //   Serial.println();

  //   if (position != prevPosition){
  //     // Serial.println(prevPosition);
  //     prevPosition = position;
  //     sendCAN = true;
  //   }
    
  //   Serial7.flush();
  //   while (Serial7.available() > 0) {
  //     Serial7.read(); // Discard leftover bytes
  //   }
  //   printByteArrayBinary(received, 9);
  //   Serial.print("Byte times: ");
  //   for (int i = 0; i < 9; i++){
  //     Serial.print(byteTime[i]);
  //     Serial.print(", ");
  //   }
  //   Serial.print("Position: ");
  //   Serial.print(position);
  //   Serial.print(". Velocity: ");
  //   Serial.println(vel);
    
  //   Serial7.flush();
  // }


  if (can2.read(msg)) {
      canSniff(msg);
      msgCounterReceived++;
    }

  if (sendCAN) {
    lastSendTime = millis();
    CAN_message_t msg;
    
    msg.id = 0x320;
    msg.len = 6;
    
    // uint8_t received[9] = {0,1,2,3,4,5,6,7,8};
    // for (int i = 0; i < 6; i++) msg.buf[i] = received[i];
    msg.buf[0] = received[2];
    msg.buf[1] = received[3];
    msg.buf[2] = received[4];
    msg.buf[3] = received[5];
    msg.buf[4] = received[6];
    msg.buf[5] = received[7];
    
    can2.write(msg);
    sendCAN = false;
    msgCounter++;
  }

  if (millis() - lastPrintTime > printInterval) {
    lastPrintTime = millis();
    Serial.print("||   Msgs: ");
    Serial.print(msgCounter);
    Serial.print(" in ");
    Serial.print(printInterval);
    Serial.println("ms   ||    ");
    msgCounter = 0;
    
    Serial.print("||   Msgs received: ");
    Serial.print(msgCounterReceived);
    Serial.print(" in ");
    Serial.print(printInterval);
    Serial.println("ms   ||    ");
    msgCounterReceived = 0;

  }


}