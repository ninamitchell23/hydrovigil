#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define BUZZER_SLAVE_ADDRESS 0x03
#define BUZZER_PIN PB0 // Buzzer connected to pin PB0 (Arduino digital pin 8)

// Setup the buzzer pin
void setupBuzzer() {
  // Set buzzer pin as output
  DDRB |= (1 << BUZZER_PIN); // Set PB0 as output
  PORTB &= ~(1 << BUZZER_PIN); // Ensure the buzzer is off initially
}

void setupI2C() {
  // Set up the I2C hardware in slave mode
  TWAR = (BUZZER_SLAVE_ADDRESS << 1); // Set the slave address
  TWCR = (1 << TWEN) | (1 << TWINT); // Enable TWI (I2C) and clear interrupt flag
}

void turnBuzzerOn() {
  PORTB |= (1 << BUZZER_PIN); // Set PB0 high to turn the buzzer on
}

void turnBuzzerOff() {
  PORTB &= ~(1 << BUZZER_PIN); // Set PB0 low to turn the buzzer off
}

void setup() {
  // Initialize components
  setupBuzzer();
  setupI2C();
  
  sei(); // Enable global interrupts
}

void loop() {
  // Wait for I2C requests
  TWCR = (1 << TWINT) | (1 << TWEN); // Enable I2C and clear interrupt flag

  // Handle I2C communication (respond to master request)
  if ((TWSR & 0xF8) == TW_SR_DATA_ACK) { // Wait for data from the master
    uint8_t buzzerStatus = TWDR; // Read the byte sent by the master

    // If the water is dirty (1), turn the buzzer on, otherwise turn it off
    if (buzzerStatus == 1) {
      turnBuzzerOn(); // Turn buzzer on if water is dirty
    } else {
      turnBuzzerOff(); // Turn buzzer off if water is clean
    }

    // Acknowledge that data has been received
    TWCR = (1 << TWINT) | (1 << TWEN); // Send ACK back to master
  }
}
