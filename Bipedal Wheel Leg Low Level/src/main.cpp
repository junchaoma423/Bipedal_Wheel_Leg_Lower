#include <Arduino.h>
const int RTS_PIN5 = 22; // RTS pin for Serial5, port 1
const int RTS_PIN1 = 2;  // RTS pin for Serial1, port 2
const int RTS_PIN4 = 18; // RTS pin for Serial4, port 18
const int ledPin = 13;

bool prints = true; // Used to enable or disable prints. Enable prints if debugging from Serial monitor, and disable prints before connecting to the controller

const char SOM = 'A'; // Start-of-message indicator

byte operationMode = 10;  // 10 for closed-loop control

// int motorID = 0;  // Initialize motorID
// float torque = 0; // Initialize torque command
// float position = 0; // Initialize desired position
// float speed = 0;    // Initialize desired velocity
// float kp = 0;       // Initialize kp
// float kd = 0;       // Initialize kd

float torque[8] = {};
float position[8] = {};
float speed[8] = {};
float kp[8] = {};
float kd[8] = {};


// int16_t motorSpeed;
// int32_t motorPosition;
// float current_id;
// float torqueActual; // Initialize torque feedback
// float speedActual;  // Initialize speed feedback
// float positionActual; // Initialize position feedback
// float motorTemp;      // Initialize motor temperature
// float motorError;     // Initialize motor error

float current_id;
float motorTemp[8] = {};
float motorError[8] = {};
int16_t motorSpeed;
int32_t motorPosition;
float torqueActual[8] = {};
float speedActual[8] = {};
float positionActual[8] = {};

float torqueLimit = 30;
float speedLimit = 15;

float motorDataSerial[5];

void printMessage(byte* message, int length) {
  for (int i = 0; i < length; i++) {
    Serial.print("0x");
    if (message[i] < 0x10) {
      Serial.print("0");
    }
    Serial.print(message[i], HEX);
    Serial.print("  ");
  }
  Serial.println();
}

// Funcion to calculate the CRC32 value
uint32_t crc32_core(uint32_t* ptr, uint32_t len) {
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t CRC32 = 0xFFFFFFFF;
  const uint32_t dwPolynomial = 0x04c11db7;

  for (uint32_t i = 0; i < len; i++) {
    xbit = 1 << 31;
    data = ptr[i];

    for (uint32_t bits = 0; bits < 32; bits++) {
      if (CRC32 & 0x80000000) {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      } else {
        CRC32 <<= 1;
      }

      if (data & xbit) {
        CRC32 ^= dwPolynomial;
      }

      xbit >>= 1;
    }
  }
  return CRC32;
}

// Function to generate the 34-byte message
void generateMessage(HardwareSerial& port, float motorId, byte operationMode, float torque, float position, float speed, float kp, float kd, byte* message) {
  // Set fixed bytes
  message[0] = 0xFE;
  message[1] = 0xEE;
  message[3] = 0x00;
  message[4] = operationMode;
  message[5] = 0xFF;
  message[6] = 0x00;
  message[7] = 0x00;
  message[8] = 0x00;
  message[9] = 0x00;
  message[10] = 0x00;
  message[11] = 0x00;

  // Fill in the motor id (byte 3)
  int16_t id = static_cast<int16_t>(motorId);
  message[2] = id;

  // Fill in the torque command (bytes 13 and 14)
  int16_t tff = static_cast<int16_t> ((torque * 256) / 9.1);
  message[12] = tff & 0xFF;
  message[13] = tff >> 8;

  // Fill in the speed (bytes 15 and 16)
  int16_t wdes = static_cast<int16_t>((speed * 128) * 9.1);
  message[14] = wdes & 0xFF;
  message[15] = wdes >> 8;

  // Fill in the position (bytes 17 to 20)
  int32_t pdes = static_cast<int32_t>(position * (16384 / (2 * 3.1415826))) * 9.1;
  message[16] = pdes & 0xFF;
  message[17] = (pdes >> 8) & 0xFF;
  message[18] = (pdes >> 16) & 0xFF;
  message[19] = pdes >> 24;

  // Fill in the kp (bytes 21 and 22) // FLIPPED
  uint16_t kpScaled = static_cast<uint16_t>(kp * 2048);
  message[20] = kpScaled & 0xFF;
  message[21] = kpScaled >> 8;

  // Fill in the kd (bytes 23 and 24) // FLIPPED
  uint16_t kdScaled = static_cast<uint16_t>(kd * 1024);
  message[22] = kdScaled & 0xFF;
  message[23] = kdScaled >> 8;

  message[24] = 0x00;
  message[25] = 0x00;
  message[26] = 0x00;
  message[27] = 0x00;
  message[28] = 0x00;
  message[29] = 0x00;

  // Calculate CRC32 check bits (bytes 31 to 34)
  uint32_t crcData[7]; // We need 7 uint32_t values for CRC calculation (30 bytes)
  memcpy(crcData, message, 30); // Copy first 30 bytes for CRC calculation
  uint32_t crcValue = crc32_core(crcData, 7);
  message[30] = crcValue & 0xFF;
  message[31] = (crcValue >> 8) & 0xFF;
  message[32] = (crcValue >> 16) & 0xFF;
  message[33] = (crcValue >> 24) & 0xFF;
}

// Read data from the motor
void readMessage(HardwareSerial& port, byte* response, int responseLength, int timeout) {
  unsigned long startTime = millis();
  int bytesRead = 0;
  int index = 0;
  int port_number = 0;

  while (bytesRead < responseLength && millis() - startTime < timeout) {
    if (port.available()) {
      response[bytesRead] = port.read();
      bytesRead++;
    }
  }

  if (&port == &Serial5){
    port_number = 0;
  } else if (&port == &Serial1){
    port_number = 1;
  } else {
    port_number = 2;
  }

  // If the timeout is reached and the response is not fully received, print an error message
  if (bytesRead < responseLength) {
    if (prints) {
      Serial.println("\nTimeout: Response not fully received!");
    }
  } else {
    // Decode and print the relevant ata
    current_id = response[2];
    index = port_number * 3 + current_id;
    motorTemp[index] = response[6];
    motorError[index] = response[7];
    uint16_t motorTorque = (response[13] << 8) | response[12];
    if (motorTorque > 250) {
      motorTorque = 0;
    }
    motorSpeed = (response[15] << 8) | response[14];
    motorPosition = (response[33] << 24) | (response[32] << 16) | (response[31] << 8) | response[30];

    // Convert the scaled values back to the original values
    torqueActual[index] = static_cast<float>((motorTorque) / 256.0) * 9.1;
    speedActual[index] = static_cast<float>(motorSpeed) / 128.0 / 9.1;
    positionActual[index] = static_cast<float>(motorPosition) / (16384.0 / (2.0 * 3.1415926)) / 9.1;

    if (abs(torqueActual[index]) > torqueLimit || abs(speedActual[index]) > speedLimit) { // 
      byte message[34];
      float torqueRead = torqueActual[index];
      float speedRead = speedActual[index];
      // generateMessage(current_id, operationMode, 0, 0, 0, 0, 0, message);
      port.write(message, sizeof(message));
      while (1) {Serial.printf("ERROR OVER TORQUE OR OVERSPEED; STOPPING. Torque: %f, Speed: %f\n", torqueRead, speedRead);} //Indefinetely stop the code until restarting
    }

    // if (prints) {
    //   // Print the decoded data
    //   Serial.print("\nMotor Temperature: ");
    //   Serial.println(motorTemp);

    //   Serial.print("Motor Error: ");
    //   Serial.println(motorError);

    //   Serial.print("Motor Torque: ");
    //   Serial.println(torqueActual);

    //   Serial.print("Motor Speed: ");
    //   Serial.println(speedActual);

    //   Serial.print("Motor Position: ");
    //   Serial.println(positionActual);
    // }
  }
}

void readSerialData(){
  // Read serial data from serial port
}

void sendSerialData(){
  // Send serial data to serial port
}


void setup() {
  pinMode(ledPin, OUTPUT);  // Turn LED on and keep on to indicate the code is running
  digitalWrite(ledPin, HIGH);

  Serial5.begin(4500000, SERIAL_8N1); // Serial bus for motor communication (pins 20, 21)
  Serial5.transmitterEnable(RTS_PIN5); // Use pin 22 for RTS (read-to-send) indicator

  Serial1.begin(4500000, SERIAL_8N1); // Serial bus for motor communication (pins 0, 1)
  Serial1.transmitterEnable(RTS_PIN1); // Use pin 2 for RTS

  Serial4.begin(4500000, SERIAL_8N1); // Serial bus for motor communication (pins 16, 17)
  Serial4.transmitterEnable(RTS_PIN4);  // Use pin 18 for RTS

  Serial.begin(9600); // Serial bus for communication with the PC (microUSB header)
}

void loop() {
// Step 1: Read data from serial port

byte message0[34];
byte message1[34];
byte message2[34];
byte message3[34];
byte message4[34];
byte message5[34];
byte message6[34];
byte message7[34];

// Message generation
// For port 1
generateMessage(Serial5, 0, operationMode, torque[0], position[0], speed[0], kp[0], kd[0], message0);
generateMessage(Serial5, 1, operationMode, torque[1], position[1], speed[1], kp[1], kd[1], message1);
generateMessage(Serial5, 2, operationMode, torque[2], position[2], speed[2], kp[2], kd[2], message2);

// For port 2
generateMessage(Serial1, 0, operationMode, torque[3], position[3], speed[3], kp[3], kd[3], message3);
generateMessage(Serial1, 1, operationMode, torque[4], position[4], speed[4], kp[4], kd[4], message4);
generateMessage(Serial1, 2, operationMode, torque[5], position[5], speed[5], kp[5], kd[5], message5);

// For port 3
generateMessage(Serial4, 0, operationMode, torque[6], position[6], speed[6], kp[6], kd[6], message6);
generateMessage(Serial4, 1, operationMode, torque[7], position[7], speed[7], kp[7], kd[7], message7);

//Message sending
Serial5.write(message0, sizeof(message0));
Serial5.write(message1, sizeof(message1));
Serial5.write(message2, sizeof(message2));
Serial1.write(message3, sizeof(message3));
Serial1.write(message4, sizeof(message4));
Serial1.write(message5, sizeof(message5));
Serial4.write(message6, sizeof(message6));
Serial4.write(message7, sizeof(message7));

if (prints) {
  printMessage(message0, 34);
  printMessage(message1, 34);
  printMessage(message2, 34);
  printMessage(message3, 34);
  printMessage(message4, 34);
  printMessage(message5, 34);
  printMessage(message6, 34);
  printMessage(message7, 34);
}

//Message receiving
byte response0[78];
byte response1[78];
byte response2[78];
int responseLength = 78;
for (int i = 0; i < 3; i++){
  readMessage(Serial5, response0, responseLength, 100);
  readMessage(Serial1, response1, responseLength, 100);
  readMessage(Serial4, response2, responseLength, 100);
}

// Todo, figure out the response message thing

}

