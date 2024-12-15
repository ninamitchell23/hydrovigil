#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define TURBIDITY_SENSOR_SLAVE_ADDRESS 0x02
#define TURBIDITY_SENSOR_PIN 1 // Pin A1 for turbidity sensor 

// Global variable to store turbidity data
volatile uint16_t turbidity = 0;

// Timer configuration to trigger every 1 second (assuming 16 MHz clock)
#define TIMER_PRESCALER 64
#define TIMER_COUNT (250) // To generate interrupt every 1 second

void setupI2C() {
  // Set up the I2C hardware in slave mode
  TWAR = (TURBIDITY_SENSOR_SLAVE_ADDRESS << 1); // Set the slave address
  TWCR = (1 << TWEN) | (1 << TWINT); // Enable TWI (I2C) and clear interrupt flag
}

void setupADC() {
  // Set the ADC to read from pin A1 (Turbidity sensor)
  ADMUX = (1 << MUX1); // Select ADC1 (pin A1)
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1); // Prescaler to divide clock by 64
  ADCSRA |= (1 << ADEN); // Enable the ADC
}

uint16_t readTurbidity() {
  // Start ADC conversion
  ADCSRA |= (1 << ADSC); // Start the conversion
  while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete
  return ADC;
}

void setupTimer() {
  // Set up Timer1 to generate interrupt every 1 second
  TCCR1B |= (1 << WGM12); // CTC mode
  TCCR1B |= (1 << CS01) | (1 << CS00); // Prescaler 64
  OCR1A = TIMER_COUNT; // Set compare value for 1-second interrupt
  TIMSK1 |= (1 << OCIE1A); // Enable Timer1 compare interrupt
}

ISR(TIMER1_COMPA_vect) {
  // Read turbidity every time the timer interrupt is triggered
  turbidity = readTurbidity(); // Get turbidity in ADC units
}

void setup() {
  // Initialize components
  setupI2C();
  setupADC();
  setupTimer();
  
  sei(); // Enable global interrupts
}

void loop() {
  // Wait for I2C requests
  TWCR = (1 << TWINT) | (1 << TWEN); // Enable I2C and clear interrupt flag

  // Handle I2C communication (respond to master request)
  if ((TWSR & 0xF8) == TW_SR_DATA_ACK) { // Wait for data from the master
    // Send the turbidity data to the master
    TWDR = (turbidity >> 8); // High byte of turbidity
    TWCR = (1 << TWINT) | (1 << TWEN); // Send high byte
    
    while (!(TWCR & (1 << TWINT))); // Wait for completion of high byte send

    TWDR = (turbidity & 0xFF); // Low byte of turbidity
    TWCR = (1 << TWINT) | (1 << TWEN); // Send low byte
    
    while (!(TWCR & (1 << TWINT))); // Wait for completion of low byte send
  }
}
