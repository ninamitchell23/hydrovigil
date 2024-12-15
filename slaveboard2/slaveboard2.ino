#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define TURBIDITY_SENSOR_PIN 0 // ADC0 (Pin A0) is used for turbidity sensor
#define TURBIDITY_THRESHOLD 500 // Example threshold for dirty water

void setupADC() {
  // Set reference voltage to AVcc with an external capacitor at the AREF pin
  ADMUX = (1 << REFS0);  // AVcc as the reference voltage
  
  // Set ADC prescaler to 64 (for 16 MHz clock, ADC clock will be 250 kHz)
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1); // Prescaler 64
  
  // Enable ADC
  ADCSRA |= (1 << ADEN);
}

uint16_t readADC(uint8_t channel) {
  // Select the ADC channel
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);  // Mask to select ADC channel

  // Start the conversion
  ADCSRA |= (1 << ADSC); // Start the conversion

  // Wait for the conversion to finish
  while (ADCSRA & (1 << ADSC)) {
    // Wait here until ADSC becomes 0
  }

  // Read the ADC result (10-bit value: 0-1023)
  uint16_t result = ADC;  // ADC result (high byte + low byte)
  return result;
}

void setupI2C() {
  // Set up the I2C hardware in slave mode
  TWAR = (0x02 << 1); // Set the slave address (0x02 for turbidity sensor)
  TWCR = (1 << TWEN) | (1 << TWINT); // Enable TWI (I2C) and clear interrupt flag
}

void setup() {
  // Initialize components
  setupADC();
  setupI2C();
  
  sei(); // Enable global interrupts
}

void loop() {
  // Wait for I2C requests
  TWCR = (1 << TWINT) | (1 << TWEN); // Enable I2C and clear interrupt flag

  // Handle I2C communication (respond to master request)
  if ((TWSR & 0xF8) == TW_SR_DATA_ACK) { // Wait for data from the master
    uint8_t command = TWDR;  // Read the byte sent by the master

    // If command is '0' (request turbidity sensor reading), send data back
    if (command == 0) {
      // Read the turbidity sensor value from ADC
      uint16_t turbidityValue = readADC(TURBIDITY_SENSOR_PIN);

      // Send high byte of turbidity value to master
      TWDR = (turbidityValue >> 8);
      TWCR = (1 << TWINT) | (1 << TWEN); // Acknowledge and send high byte
      while (!(TWCR & (1 << TWINT))); // Wait for ACK from master

      // Send low byte of turbidity value to master
      TWDR = (turbidityValue & 0xFF);
      TWCR = (1 << TWINT) | (1 << TWEN); // Acknowledge and send low byte
      while (!(TWCR & (1 << TWINT))); // Wait for ACK from master
    }
  }
}
