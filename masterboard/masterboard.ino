#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define TEMP_SENSOR_SLAVE_ADDRESS 0x01
#define TURBIDITY_SENSOR_SLAVE_ADDRESS 0x02
#define BUZZER_SLAVE_ADDRESS 0x03
#define TURBIDITY_THRESHOLD 500
#define EEPROM_TEMPERATURE_ADDR 0 // EEPROM address for storing temperature
#define EEPROM_TURBIDITY_ADDR 2   // EEPROM address for storing turbidity

// LCD Pins
#define RS 12
#define EN 11
#define D4 4
#define D5 5
#define D6 3
#define D7 2

volatile uint16_t temperatureC = 0;
volatile uint16_t turbidity = 0;
unsigned long lastDisplayUpdate = 0;
const unsigned long displayInterval = 1000; // 1-second interval for LCD update

// Helper macros to manipulate registers using pointers
#define REG(addr) (*((volatile uint8_t *)(addr)))
#define SFR_BIT_SET(sfr, bit) (REG(sfr) |= (1 << (bit)))
#define SFR_BIT_CLEAR(sfr, bit) (REG(sfr) &= ~(1 << (bit)))
#define SFR_BIT_READ(sfr, bit) (REG(sfr) & (1 << (bit)))
#define SFR_WRITE(sfr, value) (REG(sfr) = (value))
#define SFR_READ(sfr) (REG(sfr))

void setupI2C() {
  SFR_WRITE(0xB8, 32); // Set TWBR to 32 for 100kHz (assuming 16MHz clock)
  SFR_BIT_CLEAR(0xB9, TWPS1); // Clear TWPS1
  SFR_BIT_CLEAR(0xB9, TWPS0); // Clear TWPS0
  SFR_BIT_SET(0xBC, TWEN);    // Enable TWI (I2C)
}

uint16_t requestData(uint8_t slaveAddress) {
  uint16_t data = 0;

  // Send START condition
  SFR_BIT_SET(0xBC, TWSTA);
  SFR_BIT_SET(0xBC, TWINT);
  SFR_BIT_SET(0xBC, TWEN);
  while (!SFR_BIT_READ(0xBC, TWINT));

  // Send Slave Address with Write bit
  SFR_WRITE(0xBB, (slaveAddress << 1) | TW_WRITE);
  SFR_BIT_SET(0xBC, TWINT);
  SFR_BIT_SET(0xBC, TWEN);
  while (!SFR_BIT_READ(0xBC, TWINT));

  // Send repeated START condition
  SFR_BIT_SET(0xBC, TWSTA);
  SFR_BIT_SET(0xBC, TWINT);
  SFR_BIT_SET(0xBC, TWEN);
  while (!SFR_BIT_READ(0xBC, TWINT));

  // Send Slave Address with Read bit
  SFR_WRITE(0xBB, (slaveAddress << 1) | TW_READ);
  SFR_BIT_SET(0xBC, TWINT);
  SFR_BIT_SET(0xBC, TWEN);
  while (!SFR_BIT_READ(0xBC, TWINT));

  // Read high byte of data
  SFR_BIT_SET(0xBC, TWEA);
  SFR_BIT_SET(0xBC, TWINT);
  SFR_BIT_SET(0xBC, TWEN);
  while (!SFR_BIT_READ(0xBC, TWINT));
  data = SFR_READ(0xBB) << 8;

  // Read low byte of data
  SFR_BIT_CLEAR(0xBC, TWEA);
  SFR_BIT_SET(0xBC, TWINT);
  SFR_BIT_SET(0xBC, TWEN);
  while (!SFR_BIT_READ(0xBC, TWINT));
  data |= SFR_READ(0xBB);

  // Send STOP condition
  SFR_BIT_SET(0xBC, TWSTO);
  SFR_BIT_SET(0xBC, TWINT);
  SFR_BIT_SET(0xBC, TWEN);

  return data;
}

void notifyBuzzer(bool dirty) {
  // Send START condition
  SFR_BIT_SET(0xBC, TWSTA);
  SFR_BIT_SET(0xBC, TWINT);
  SFR_BIT_SET(0xBC, TWEN);
  while (!SFR_BIT_READ(0xBC, TWINT));

  // Send Slave Address with Write bit
  SFR_WRITE(0xBB, (BUZZER_SLAVE_ADDRESS << 1) | TW_WRITE);
  SFR_BIT_SET(0xBC, TWINT);
  SFR_BIT_SET(0xBC, TWEN);
  while (!SFR_BIT_READ(0xBC, TWINT));

  // Send buzzer status (1 for dirty, 0 for clean)
  SFR_WRITE(0xBB, dirty ? 1 : 0);
  SFR_BIT_SET(0xBC, TWINT);
  SFR_BIT_SET(0xBC, TWEN);
  while (!SFR_BIT_READ(0xBC, TWINT));

  // Send STOP condition
  SFR_BIT_SET(0xBC, TWSTO);
  SFR_BIT_SET(0xBC, TWINT);
  SFR_BIT_SET(0xBC, TWEN);
}

void setupLCD() {
  // Assuming pre-configured library for LCD, no additional setup here
}

void setup() {
  setupI2C();
  setupLCD();
  sei(); // Enable global interrupts

  // Read the last saved temperature and turbidity from EEPROM
  temperatureC = (SFR_READ(0x3F + EEPROM_TEMPERATURE_ADDR) << 8) |
                 SFR_READ(0x3F + EEPROM_TEMPERATURE_ADDR + 1);
  turbidity = (SFR_READ(0x3F + EEPROM_TURBIDITY_ADDR) << 8) |
              SFR_READ(0x3F + EEPROM_TURBIDITY_ADDR + 1);
}

void loop() {
  unsigned long currentMillis = millis();

  // Check if it's time to update the display
  if (currentMillis - lastDisplayUpdate >= displayInterval) {
    lastDisplayUpdate = currentMillis;

    // Request data from Slave Boards
    temperatureC = requestData(TEMP_SENSOR_SLAVE_ADDRESS);
    turbidity = requestData(TURBIDITY_SENSOR_SLAVE_ADDRESS);

    // Store the current data to EEPROM
    SFR_WRITE(0x3F + EEPROM_TEMPERATURE_ADDR, (temperatureC >> 8) & 0xFF);
    SFR_WRITE(0x3F + EEPROM_TEMPERATURE_ADDR + 1, temperatureC & 0xFF);
    SFR_WRITE(0x3F + EEPROM_TURBIDITY_ADDR, (turbidity >> 8) & 0xFF);
    SFR_WRITE(0x3F + EEPROM_TURBIDITY_ADDR + 1, turbidity & 0xFF);

    // Update LCD display
    // Assuming LCD update functionality here
    if (turbidity > TURBIDITY_THRESHOLD) {
      notifyBuzzer(true); // Notify buzzer board
    } else {
      notifyBuzzer(false); // Notify buzzer board
    }
  }
}
