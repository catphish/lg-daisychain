#include "CRC16.h"
#include "CRC.h"

#define FRM_TYPE_COMMAND (1<<7)
#define REQ_TYPE_BROADCAST_READ (6<<4)
#define ADDR_SIZE_8 0

void setup() {
  // Communication with PC for debugging, I assume you know what you're doing here
  Serial.begin(115200);
  // I'm not certain what baud rate is required. Default is likely 250,000
  Serial2.begin(250000);
  Serial.println("Hello!");
}

uint16_t crc_16_ibm(uint8_t *buf, uint16_t len);

void loop() {
  delay(1000);
  // Our command frame will be 6 bytes (init, address, data, data, crc, crc)
  uint8_t command[6];
  // Broadcast read (write with response) command. 2 data byts. See datasheet page 64.
  // We use a broadcast because we don't know or care about the device's address.
  command[0] = FRM_TYPE_COMMAND | REQ_TYPE_BROADCAST_READ | ADDR_SIZE_8 | 2;
  // Register address 0 is the 2 byte silicon version
  // Change this to 0x0A to get the device address (and change length below to 0)
  command[1] = 0;
  // First "data" byte is address of highest responder, we'll set this to 31, the highest possible address.
  command[2] = 31;
  // Second "data" byte is the number of bytes expected (less 1). We expect 2 bytes, so set 1 here.
  // Set this to zero if only 1 byte is expected
  command[3] = 1;
  // Calculate crc
  uint16_t crc = crc_16_ibm(command, 4);
  // Append CRC
  command[4] = crc >> 8;
  command[5] = crc & 0xff;
  // Send the command
  Serial2.write(command, 6);

  // Wait for a response, first byte is length (less 1)
  uint8_t length = Serial2.read() + 1;
  Serial.print("Received response. Length: ");
  Serial.print(length);
  Serial.print("\n");
  // Receive and print each byte reseived. This should probably be 08 06 but, any response would be a start.
  for(int n=0; n<length; n++) {
    uint8_t data = Serial2.read();
    Serial.print(" Data: ");
    Serial.print(data, HEX);
    Serial.print("\n");
  }
  // Ignore CRC
  Serial2.read();
  Serial2.read();
}

// Ths function is provided in the TI datasheet. It should just work.
uint16_t crc_16_ibm(uint8_t *buf, uint16_t len) {
  uint16_t crc = 0;
  uint16_t j;
  while (len--) {
    crc ^= *buf++;
    for (j = 0; j < 8; j++)
      crc = (crc >> 1) ^ ((crc & 1) ? 0xa001 : 0);
    }
  return crc;
}
