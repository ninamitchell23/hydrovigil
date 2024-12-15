#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define TEMP_SENSOR_SLAVE_ADDRESS 0x01
#define LM35_PIN 0 // Pin A0 for the LM35 sensor

// Global variable to store temperature data
volatile uint16_t temperatureC = 0;

// Timer configuration to trigger every 1 second (16 MHz clock)
#define TIMER_PRESCALER 64
#define TIMER_COUNT (250) // To generate interrupt every 1 second

//  direct register access
#define SFR_BIT_SET(reg, bit) (*(volatile uint8_t *)&reg |= (1 << bit))
#define SFR_BIT_CLEAR(reg, bit) (*(volatile uint8_t *)&reg &= ~(1 << bit))
#define SFR_WRITE(reg, value) (*(volatile uint8_t *)&reg = (value))
#define SFR_READ(reg) (*(volatile uint8_t *)&reg)

void setupI2C() {
  // Set up the I2C hardware in slave mode
  SFR_WRITE(TWAR, TEMP_SENSOR_SLAVE_ADDRESS << 1); // Set the slave address
  SFR_BIT_SET(TWCR, TWEN); // Enable TWI (I2C)
  SFR_BIT_SET(TWCR, TWINT); // Clear interrupt flag
}

void setupADC() {
  // Set the ADC to read from pin A0 (LM35)
  SFR_WRITE(ADMUX, (1 << REFS0) | (LM35_PIN & 0x0F)); // Use AVCC as reference and ADC0 as input
  SFR_BIT_SET(ADCSRA, ADPS2); // Set prescaler to 64
  SFR_BIT_SET(ADCSRA, ADPS1);
  SFR_BIT_SET(ADCSRA, ADEN); // Enable the ADC
}

uint16_t readTemperature() {
  // Start ADC conversion
  SFR_BIT_SET(ADCSRA, ADSC); // Start the conversion
  while (SFR_READ(ADCSRA) & (1 << ADSC)); // Wait for conversion to complete

  // Return the 10-bit ADC result (temperature in ADC units)
  uint16_t result = SFR_READ(ADCL);
  result |= (SFR_READ(ADCH) << 8);
  return result;
}

void setupTimer() {
  // Set up Timer1 to generate interrupt every 1 second
  SFR_BIT_SET(TCCR1B, WGM12); // CTC mode
  SFR_BIT_SET(TCCR1B, CS01); // Prescaler 64
  SFR_BIT_SET(TCCR1B, CS00);
  SFR_WRITE(OCR1A, TIMER_COUNT); // Set compare value for 1-second interrupt
  SFR_BIT_SET(TIMSK1, OCIE1A); // Enable Timer1 compare interrupt
}

ISR(TIMER1_COMPA_vect) {
  // Read temperature every time the timer interrupt is triggered
  temperatureC = readTemperature(); // Get temperature in ADC units
}

void setup() {
  // Initialize components
  setupI2C();
  setupADC();
  setupTimer();

  SFR_BIT_SET(SREG, 7); // Enable global interrupts
}

void loop() {
  // Wait for I2C requests
  SFR_BIT_SET(TWCR, TWINT); // Enable I2C and clear interrupt flag
  SFR_BIT_SET(TWCR, TWEN);

  // Handle I2C communication (respond to master request)
  if ((SFR_READ(TWSR) & 0xF8) == TW_SR_DATA_ACK) { // Wait for data from the master
    // Send the temperature data to the master
    SFR_WRITE(TWDR, (temperatureC >> 8)); // High byte of temperature
    SFR_BIT_SET(TWCR, TWINT); // Send high byte
    SFR_BIT_SET(TWCR, TWEN);

    while (!(SFR_READ(TWCR) & (1 << TWINT))); // Wait for completion of high byte send

    SFR_WRITE(TWDR, (temperatureC & 0xFF)); // Low byte of temperature
    SFR_BIT_SET(TWCR, TWINT); // Send low byte
    SFR_BIT_SET(TWCR, TWEN);

    while (!(SFR_READ(TWCR) & (1 << TWINT))); // Wait for completion of low byte send
  }
}
