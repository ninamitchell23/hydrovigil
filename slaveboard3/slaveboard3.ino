#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define BUZZER_SLAVE_ADDRESS 0x03
#define BUZZER_PIN PB0 // Buzzer connected to pin PB0 (Arduino digital pin 8)

// Pointer definitions for register access
volatile uint8_t *ddrb = (volatile uint8_t *)0x24;   // Data Direction Register for Port B
volatile uint8_t *portb = (volatile uint8_t *)0x25;  // Port B Data Register
volatile uint8_t *twar = (volatile uint8_t *)0x22;  // TWI (I2C) Address Register
volatile uint8_t *twcr = (volatile uint8_t *)0x56;  // TWI Control Register
volatile uint8_t *twsr = (volatile uint8_t *)0x21;  // TWI Status Register
volatile uint8_t *twdr = (volatile uint8_t *)0x23;  // TWI Data Register

// Setup the buzzer pin
void setupBuzzer() {
  // Set buzzer pin as output
  *ddrb |= (1 << BUZZER_PIN);      // Set PB0 as output
  *portb &= ~(1 << BUZZER_PIN);    // Ensure the buzzer is off initially
}

void setupI2C() {
  // Set up the I2C hardware in slave mode
  *twar = (BUZZER_SLAVE_ADDRESS << 1); // Set the slave address
  *twcr = (1 << TWEN) | (1 << TWINT);  // Enable TWI (I2C) and clear interrupt flag
}

void turnBuzzerOn() {
  *portb |= (1 << BUZZER_PIN); // Set PB0 high to turn the buzzer on
}

void turnBuzzerOff() {
  *portb &= ~(1 << BUZZER_PIN); // Set PB0 low to turn the buzzer off
}

void setup() {
  // Initialize components
  setupBuzzer();
  setupI2C();

  sei(); // Enable global interrupts
}

void loop() {
  // Wait for I2C requests
  *twcr = (1 << TWINT) | (1 << TWEN); // Enable I2C and clear interrupt flag

  // Handle I2C communication (respond to master request)
  if ((*twsr & 0xF8) == 0x60) { // TWI has received own SLA+W
    while (!(*twcr & (1 << TWINT))) {
      // Wait for the data to be received
    }
    uint8_t buzzerStatus = *twdr; // Read the byte sent by the master

    // If the water is dirty (1), turn the buzzer on, otherwise turn it off
    if (buzzerStatus == 1) {
      turnBuzzerOn(); // Turn buzzer on if water is dirty
    } else {
      turnBuzzerOff(); // Turn buzzer off if water is clean
    }

    // Acknowledge that data has been received
    *twcr = (1 << TWINT) | (1 << TWEN); // Send ACK back to master
  }
}
