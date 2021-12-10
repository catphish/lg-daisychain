#include <CAN.h>

void setup() {
  Serial.begin(1000000);
  CAN.begin(500000);
  CAN.onReceive(onReceive);
}

// Call this a couple of times per second for continuous balancing
void balance(float voltage) {
  uint16_t v = voltage * 65535.0f / 5.0f;
  CAN.beginPacket(0x4F8);
  CAN.write(0x00); // Balance
  CAN.write(v >> 8);
  CAN.write(v);
  CAN.endPacket();
}

void loop() {
  delay(500);
  // This program is current set up to balance to 3.8v all the time for testing.

  // You probably want to delect the lowest cell voltage in the pack and set this
  // a couple of mV higher than the detected minimum.

  // You probably also only want to balance above a specified maximum voltage so
  // that you are only top balancing.

  balance(3.8);
}

// Calculate NTC temperature
float temperature(uint16_t adc) {
  float r = 0.0000000347363427499292f * adc * adc - 0.001025770762903f * adc + 2.68235340614337f;
  float t = log(r) * -30.5280964239816f + 95.6841501312447f;
  return t;
}

// Calculate ADC voltage
float voltage(uint16_t adc) {
  float v = adc * 5.0f / 65535.0f;
  return v;
}

// Called when CAN data is received
void onReceive(int packetSize) {
  if(CAN.packetRtr()) return;         // Ignore rtr packets
  if((CAN.packetId() & 0xfffffffe) != 0x4f0) return; // Ignore non-BMS packets

  uint8_t module = CAN.read();
  uint8_t cell = CAN.read();
  uint16_t data = (uint16_t)CAN.read() << 8 | CAN.read();
  if(module == 0xff) {
    Serial.print("M:");
    Serial.print(module);
    Serial.print(" C:");
    Serial.print(cell);
    Serial.println(" pack data");
  } else {
    if(cell < 16) {
      Serial.print("M:");
      Serial.print(module);
      Serial.print(" C:");
      Serial.print(cell);
      Serial.print(" ");
      Serial.print(voltage(data), 3);
      Serial.print("V\n");
    } else if(cell == 17) {
      Serial.print("M:");
      Serial.print(module);
      Serial.print(" NEG SIDE TEMP: ");
      Serial.print(temperature(data), 2);
      Serial.print("C\n");
    } else if(cell == 18) {
      Serial.print("M:");
      Serial.print(module);
      Serial.print(" POS SIDE TEMP: ");
      Serial.print(temperature(data), 2);
      Serial.print("C\n");
    } else if(cell == 0xff) {
      Serial.print("M:");
      Serial.print(module);
      Serial.print(" Balancing cells: ");
      Serial.print(data, HEX);
      Serial.print("\n");
    }
  }
}
