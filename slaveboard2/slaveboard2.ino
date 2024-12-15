#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define TURBIDITY_SENSOR_PIN 0 // ADC0 (Pin A0) is used for turbidity sensor
#define TURBIDITY_THRESHOLD 500 //  threshold for dirty water

void setupADC() {
  // Configure ADC reference voltage to AVcc
  *(&ADMUX) = (1 << REFS0); // Set REFS0 bit for AVcc reference

  // Set ADC prescaler to 64 (for 16 MHz clock, ADC clock will be 250 kHz)
  *(&ADCSRA) |= (1 << ADPS2) | (1 << ADPS1); // Set ADPS2 and ADPS1 bits

  // Enable the ADC
  *(&ADCSRA) |= (1 << ADEN); // Set ADEN bit to enable ADC
}

uint16_t readADC(uint8_t channel) {
  // Select the ADC channel
  *(&ADMUX) = (*(&ADMUX) & 0xF0) | (channel & 0x0F); // Clear and set channel bits

  // Start the ADC conversion
  *(&ADCSRA) |= (1 << ADSC); // Set ADSC bit to start conversion

  // Wait for conversion to complete
  while (*(&ADCSRA) & (1 << ADSC)); // Wait until ADSC bit is cleared

  // Return the 10-bit ADC result
  return *((volatile uint16_t *)&ADC); // Read ADC data register
}

void setupI2C() {
  // Set the slave address for I2C
  *(&TWAR) = (0x02 << 1); // Shift address left and set in TWAR

  // Enable TWI (I2C)
  *(&TWCR) = (1 << TWEN) | (1 << TWINT); // Set TWEN and clear TWINT to initialize
}

void setup() {
  // Initialize components
  setupADC();
  setupI2C();

  // Enable global interrupts
  sei();
}

void loop() {
  // Enable I2C and clear interrupt flag
  *(&TWCR) = (1 << TWINT) | (1 << TWEN);

  // Check if data has been received from the master
  if ((*(&TWSR) & 0xF8) == 0x60) { // Check if own SLA+W has been received
    *(&TWCR) = (1 << TWINT) | (1 << TWEN); // Clear interrupt flag
    while (!(*(&TWCR) & (1 << TWINT))); // Wait for TWINT to be set

    uint8_t command = *(&TWDR); // Read received command

    if (command == 0) { // Master requests turbidity value
      uint16_t turbidityValue = readADC(TURBIDITY_SENSOR_PIN); // Read ADC value

      // Send high byte of turbidity value
      *(&TWDR) = (turbidityValue >> 8); // Load high byte into TWDR
      *(&TWCR) = (1 << TWINT) | (1 << TWEN); // Clear interrupt flag and transmit
      while (!(*(&TWCR) & (1 << TWINT))); // Wait for transmission to complete

      // Send low byte of turbidity value
      *(&TWDR) = (turbidityValue & 0xFF); // Load low byte into TWDR
      *(&TWCR) = (1 << TWINT) | (1 << TWEN); // Clear interrupt flag and transmit
      while (!(*(&TWCR) & (1 << TWINT))); // Wait for transmission to complete
    }
  }
}
