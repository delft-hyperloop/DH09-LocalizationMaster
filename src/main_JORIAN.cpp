// #include <Arduino.h>
// #include <SPI.h>

// //Barcode Settings
// const int BAR_CLK_PLUS_PIN = 9;  // Pin for CLK+
// const int BAR_CLK_MINUS_PIN = 10; // Pin for CLK- (inverted CLK+)
// const int BAR_DATA_MINUS_PIN = 23; // Pin for Data (Needs 2/3 Voltage Divider!!!(5V -> 3.3V))
// const int BAR_CLOCK_FREQUENCY = 100000; // CLK Frequency to the barcode sensor
// const int BAR_BIT_NO = 25; // Number of bits to receive from the barcode sensor

// //Incremental Settings
// const int INC_DATA_PIN = 14; // Pin for Data (Needs 1/10 Voltage Divider!!!(24V -> 2.4V))

// //Teensy LED
// const int LED = 13;

// //Timer setup
// IntervalTimer clockTimer; // Initiate clocktimer
// volatile bool clockState = false; //Initiate clocktimer

// int inc_val; // Incremental value
// bool frame_sent = false; // Bool to check if a new clk frame is sent
// uint32_t bar_val = 0; // Barcode value

// //Convert gray encoded binary to binary
// uint32_t grayToBinary(uint32_t gray) {
//   uint32_t binary = gray; // Start with MSB (it stays the same)
//   for (int i = 1; i < 24; i++) { // Start from bit 1 to bit 23
//     binary ^= (gray >> i); // XOR each subsequent bit
//   }
//   return binary;
// }

// //Prints the rightmost 25 bits of a binary
// void printBinary(uint32_t binaryValue) {
//   for (int i = 24; i >= 0; i--) {
//     Serial.print((binaryValue >> i) & 1);  // Print each bit
//   }
//   Serial.println();  // Print a newline
// }

// //Toggles the state of the clock
// void toggleClock() {
//   clockState = !clockState; // Flips the state
//   digitalWriteFast(BAR_CLK_PLUS_PIN, clockState); // Writes the state to clk line
//   digitalWriteFast(BAR_CLK_MINUS_PIN, !clockState); // Inverted version
//   delayMicroseconds(((1.0 / BAR_CLOCK_FREQUENCY) / 2) * 1e6); // Waits according to SSI spec(found in leuze BPS 307i SM 100 operating instructions)
// }

// //Sends a full clk frame
// void sendFrame() {
//   clockState = true;
//   for(int i = 0; i < BAR_BIT_NO * 2 + 1; i++){
//     toggleClock();
//     if ((i > 0) && (i % 2) == 0){
//       bar_val |= (digitalReadFast(BAR_DATA_MINUS_PIN) & 1) << (BAR_BIT_NO - (int)(i / 2)); // Put the data in the same format as in Leuze config software
//     }
//   }
//   frame_sent = true;
//   digitalWrite(BAR_CLK_PLUS_PIN, HIGH); // Start clock pins low
//   digitalWrite(BAR_CLK_MINUS_PIN, LOW); // Start CLK- as the inverse
// }

// //One-time function
// void setup() {
//   Serial.begin(9600);
//   pinMode(BAR_CLK_PLUS_PIN, OUTPUT);
//   pinMode(BAR_CLK_MINUS_PIN, OUTPUT);
  
//   digitalWrite(BAR_CLK_PLUS_PIN, HIGH); // Start clock pins low
//   digitalWrite(BAR_CLK_MINUS_PIN, LOW); // Start CLK- as the inverse

//   float clockPeriodMicroseconds = ((1e6 / BAR_CLOCK_FREQUENCY) / 2) * (BAR_BIT_NO * 2 + 1) + 25;
//   clockTimer.begin(sendFrame, clockPeriodMicroseconds); // Sets up the timer to send a new frame asap after the last
// }

// //Main Loop
// void loop() {
//   inc_val = digitalRead(INC_DATA_PIN);
//   digitalWrite(LED, inc_val); //Write value to LED Teensy
//   if (frame_sent){
//     if (bar_val != 1){
//       float value = grayToBinary(bar_val >> 1) / 100.0;  // Divide by 100 as float
//       Serial.print(value, 2);  // Print with 2 decimal places
//       Serial.println(" cm");
//     }
//     else {
//       Serial.println("Barcodes not found...");
//     }
//     bar_val = 0;
//     frame_sent = false;
//   }
// }