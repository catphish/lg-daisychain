#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "bms.pio.h"
#include <math.h>

// Min difference to enable balancing. 131 = 10mV
#define BALANCE_DIFF 131
// Min absolute voltage to enable balancing. 52428 = 4.0V, 53738 = 4.1V, 54525 = 4.16V
#define BALANCE_MIN 52428

// Define pins for RS485 transceiver
#define SERIAL_IN     28
#define SERIAL_MASTER 27
#define SERIAL_OUT    26

// Buffer for received data
uint8_t rx_data_buffer[128];

uint8_t module_count = 0;
uint8_t error_count = 0;
uint16_t balance_threshold = 0;
uint16_t cell_voltage[16][16];
uint16_t aux_voltage[16][8];
uint16_t balance_bitmap[16];

// Variables for balancing process
uint16_t max_voltage;
uint16_t min_voltage;

// Timer
uint32_t previous_measurement = 0;
uint32_t sleep_period; // Time until next loop execution
uint8_t rewake;        // Whether the modules need to be woken up on next execution

// PIO and state machine selection
PIO pio = pio0;
#define SM_TX 0
#define SM_RX 1
#define SM_SQ 2

// Calculate message CRC.
uint16_t crc16(uint8_t * message, uint8_t length) {
  uint16_t crc = 0;
  uint16_t j;
  while (length--) {
    crc ^= *message++;
    for (j=0; j<8; j++) {
      crc = (crc >> 1) ^ ((crc & 1) ? 0xa001 : 0);
    }
  }
  return crc;
}

// Deactivate the TX PIO and send a square wave to wake up the device
void wakeup() {
  // Disable TX PIO
  pio_sm_set_enabled(pio, SM_TX, false);
  // Wait for it to be disabled
  busy_wait_ms(10);
  // Loop for 200 x 10us = 2ms
  for(int n=0; n<200; n++) {
    pio_sm_set_pins(pio, SM_TX, (1<<SERIAL_MASTER) | (0<<SERIAL_OUT)); // Drive DO low (DE enabled)
    busy_wait_us(4);
    pio_sm_set_pins(pio, SM_TX, (1<<SERIAL_MASTER) | (1<<SERIAL_OUT)); // Drive DO high (DE enabled)
    busy_wait_us(4);
  }
  pio_sm_set_pins(pio, SM_TX, (1<<SERIAL_MASTER) | (0<<SERIAL_OUT)); // Drive DO low (DE enabled)
  busy_wait_us(4);
  // Disable DE, stop driving bus
  pio_sm_set_pins(pio, SM_TX, 0);
  // Re-enable TX PIO
  pio_sm_set_enabled(pio, SM_TX, true);
  // Give the modules time to reset
  busy_wait_ms(200);
}

// Send a command string
void send_command(uint8_t* command, uint8_t length) {
  // Append framing but to the first byte and send it
  pio_sm_put_blocking(pio, SM_TX, command[0] | 0x100);
  // Send remaining bytes
  for(int n=1; n<length; n++)
    pio_sm_put_blocking(pio, SM_TX, command[n]);
  // Calculate and send CRC16
  uint16_t crc = crc16(command, length);
  pio_sm_put_blocking(pio, SM_TX, crc & 0xFF);
  pio_sm_put_blocking(pio, SM_TX, crc >> 8);
  busy_wait_us(20); // Always insert a short pause after sending commands
}

// Receive data from PIO into a local buffer, size limit and timeout in microseconds specified
// It's probably unnecessary to do this with an interrupt because we know when we expect to receive data
uint16_t receive_data(uint8_t* buffer, uint16_t size, uint32_t timeout) {
  uint16_t rx_data_offset = 0;
  // Return immediately if size is zero
  if(size == 0) return 0;
  // Loop until timeout expires
  for(int n=0; n<timeout; n++) {
    // Check for data in input FIFO
    while(!pio_sm_is_rx_fifo_empty(pio, SM_RX)) {
      // Receive one byte
      buffer[rx_data_offset++] = pio_sm_get_blocking(pio, SM_RX);
      // Return full size if we've filled the string
      if(rx_data_offset == size) return size;
    }
    // Sleep 1 microsecond each loop
    busy_wait_us(1);
  }
  // Return partial length received
  return rx_data_offset;
}

// Configure all daisychained packs with sequential addresses
void configure() {
  // Fully Enable Differential Interfaces and Select Auto-Addressing Mode
  send_command((uint8_t[]){0xF2,0x10,0x10,0xE0}, 4);
  // Configure the bq76PL455A-Q1 device to use auto-addressing to select address
  send_command((uint8_t[]){0xF1,0x0E,0x10}, 3);
  // Configure the bq76PL455A-Q1 device to enter auto-address mode
  send_command((uint8_t[]){0xF1,0x0C,0x08}, 3);
  // Configure 16 devices with sequential addresses
  for(int n=0; n<16; n++) {
    send_command((uint8_t[]){0xF1,0x0A,n}, 3);
  }
  for(int n=0; n<16; n++) {
    // Attempt to read back the address from each device
    pio_sm_clear_fifos(pio, SM_RX);
    send_command((uint8_t[]){0x81,n,0x0A,0x00}, 4);
    uint16_t received = receive_data(rx_data_buffer, 4, 10000);
    // If we don't receive a response, assume there are no more modules
    // and return the number of modules successfully found.
    if(received != 4) {
      module_count = n;
      return;
    }
  }
  module_count = 16;
}

// Request that all packs simultaneously sample voltage
void sample_all() {
  // 0xFF 0xFF 0xFF - these 24 bits enable sampling of 16 cell voltages and
  //                  8 AUX channels. Some will contain temperature data.
  //                  16x oversampling.
  send_command((uint8_t[]){0xF6,0x02,0x00,0xFF,0xFF,0xFF,0x00,0x04}, 8);
  busy_wait_ms(10);
}

// Put all modules to sleep
void sleep_modules() {
  send_command((uint8_t[]){0xF1,0x0C,0x48}, 3);
  rewake = 1;
}

// Return 1 if all PCB temperature sensors on a module are above 1.0v
uint8_t pcb_below_temp(uint8_t module) {
  if(aux_voltage[module][3] < 13107) return 0;
  if(aux_voltage[module][4] < 13107) return 0;
  if(aux_voltage[module][5] < 13107) return 0;
  if(aux_voltage[module][6] < 13107) return 0;
  return 1;
}

float temperature(uint16_t adc) {
  float r = adc;
  r = r * r * 0.0000000347363427499292f - r * 0.001025770762903f + 2.68235340614337f;
  float t = logf(r) * -30.5280964239816f + 95.6841501312447f;
  return t;
}

int main()
{
  // Set system clock to 80MHz, this seems like a reasonable value for the 4MHz data
  set_sys_clock_khz(80000, true);
  // Configure STDIO for debugging
	stdio_init_all();
  // Load and initialize the TX PIO program
  uint offset_tx = pio_add_program(pio, &daisychain_tx_program);
  daisychain_tx_program_init(pio, SM_TX, offset_tx, SERIAL_OUT, SERIAL_MASTER);
  // Load and initialize the RX PIO program
  uint offset_rx = pio_add_program(pio, &daisychain_rx_program);
  daisychain_rx_program_init(pio, SM_RX, offset_rx, SERIAL_IN, SERIAL_MASTER);
  // Load and initialize 8MHz square wave generator
  uint offset_sq = pio_add_program(pio, &square_wave_program);
softreset:
  // Wake modules immediately
  sleep_period = 0;
  rewake = 1;

  // Main loop.
  while (1) {
    // Check is 1 second has elapsed since the last mesurement
    if(time_us_32() - previous_measurement >= sleep_period) {
      previous_measurement = time_us_32();
      // If we've been sleeping, wake the modules
      if(rewake) {
        rewake = 0;
        // Wake and reset up the modules
        wakeup();
        // Configure the module addresses
        configure();
      }

      // Set discharge timeout (1s, all modules)
      send_command((uint8_t[]){ 0xF1,0x13,(1<<4) | (1<<3) }, 3);
      // Disable discharge (all modules)
      send_command((uint8_t[]){ 0xF2,0x14,0,0 }, 4);
      // Set communication timeout (10s) (all modules)
      send_command((uint8_t[]){ 0xF1,0x28,(6<<4) }, 3);

      // Send a broadcast message to all modules in chain to simultaneously sample all cells
      sample_all();

      // Collect voltages and set balancing on up to 16 modules
      // We want to complete this loop as fast as possible because balancing must be disabled during measurement
      max_voltage = 0;
      min_voltage = 65535;
      for(int module = 0; module < module_count; module++) {
        // Clear the input FIFO just in case
        pio_sm_clear_fifos(pio, SM_RX);
        // Request sampled voltage data from module
        send_command((uint8_t[]){0x81,module,0x02,0x20}, 4);
        // Receive response data from PIO FIFO into CPU buffer - 51 bytes of data with 10ms timeout
        // 24 values * 2 bytes + length + 2 byte checksum = 51
        uint16_t received = receive_data(rx_data_buffer, 51, 10000);
        // TODO: check RX CRC here
        if(received == 51) {
          balance_bitmap[module] = 0;
          uint16_t max_v = 0;
          for(int cell=0; cell<16; cell++) {
            // nb. Cells are in reverse, cell 16 is reported first
            cell_voltage[module][cell] = rx_data_buffer[(15-cell)*2+1] << 8 | rx_data_buffer[(15-cell)*2+2];
            if(cell_voltage[module][cell] > max_voltage) max_voltage = cell_voltage[module][cell];
            if(cell_voltage[module][cell] < min_voltage) min_voltage = cell_voltage[module][cell];
            // Balancing
            if(pcb_below_temp(module)) // Don't balance if PCB is hot
              if(balance_threshold) // Don't balance unless threshold set
                if(cell_voltage[module][cell] > balance_threshold) // Compare cell voltage to threshold
                  if(cell_voltage[module][cell] > max_v) { // Only balance the highest voltage cell
                    balance_bitmap[module] = (1 << cell); // Only ever balance one cell
                    max_v = cell_voltage[module][cell];
                  }
          }
          for(int aux=0; aux<8; aux++) {
            aux_voltage[module][aux] = rx_data_buffer[(16+aux)*2+1] << 8 | rx_data_buffer[(16+aux)*2+2];
          }
          send_command((uint8_t[]){ 0x92,module,0x14,balance_bitmap[module] >> 8, balance_bitmap[module] }, 5);
        } else {
          error_count++;
          goto softreset;
        }
      }
      //printf("Measurement complete in %i ms\n", (time_us_32() - previous_measurement)/1000);

      // Balancing
      if(max_voltage > BALANCE_MIN) { // 4.16V
        if(max_voltage > min_voltage + BALANCE_DIFF) { // Min cell + 10mV
          // At least one cell is above 4.16V, lets balance!
          balance_threshold = min_voltage + BALANCE_DIFF; // Min cell + 10mV
          if(balance_threshold < BALANCE_MIN) balance_threshold = BALANCE_MIN; // No less than 4.16V
          float v = balance_threshold * 5.0f / 65535.0f; // Convert to decimal for display
          printf("BALANCING: ACTIVE %.3fV\n", v);
          sleep_period = 900000;
        } else {
          // Cells are balanced
          balance_threshold = 0;
          printf("BALANCING: BALANCED\n");
          sleep_period = 60000000;
          sleep_modules();
        }
      } else {
        balance_threshold = 0;
        printf("BALANCING: INACTIVE\n");
          sleep_period = 60000000;
          sleep_modules();
      }

      //printf("Balancing voltage: %f\n", balance_threshold * 5.0f / 65535.0f);

      // Loop through all modules again to process data
      for(int module = 0; module < module_count; module++) {
        // Send cell voltages
        for(int cell=0; cell<16; cell++) {
          float v = cell_voltage[module][cell] * 5.0f / 65535.0f;
          printf("%2.2i.%2.2i: %.3f\n", module, cell, v);
        }

        //printf("%i AUX1 %.3fV\n", module, aux_voltage[module][0] * 5.0f / 65535.0f);
        printf("%2.2i.TempN: %.1fC\n", module, temperature(aux_voltage[module][1]));
        printf("%2.2i.TempP: %.1fC\n", module, temperature(aux_voltage[module][2]));
        //printf("%i PCB Temp #1 %.3fV\n", module, aux_voltage[module][3] * 5.0f / 65535.0f);
        //printf("%i PCB Temp #2 %.3fV\n", module, aux_voltage[module][4] * 5.0f / 65535.0f);
        //printf("%i PCB Temp #3 %.3fV\n", module, aux_voltage[module][5] * 5.0f / 65535.0f);
        //printf("%i PCB Temp #4 %.3fV\n", module, aux_voltage[module][6] * 5.0f / 65535.0f);
        //printf("%i AUX8 %f\n", module, aux_voltage[module][7] * 5.0f / 65535.0f);
        //printf("%2.2i.BAL: %04x\n", module, balance_bitmap[module]);
      }
      //printf("Loop complete in %i ms\n", (time_us_32() - previous_measurement)/1000);
    }
  }
}
