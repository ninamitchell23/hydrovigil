#include <LiquidCrystal.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include <EEPROM.h> 

#define TEMP_SENSOR_SLAVE_ADDRESS 0x01
#define TURBIDITY_SENSOR_SLAVE_ADDRESS 0x02
#define BUZZER_SLAVE_ADDRESS 0x03
#define TURBIDITY_THRESHOLD 500
#define EEPROM_TEMPERATURE_ADDR 0 // EEPROM address for storing temperature
#define EEPROM_TURBIDITY_ADDR 2   // EEPROM address for storing turbidity

// LCD Pins
const int rs = 12, en = 11, d4 = 4, d5 = 5, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

volatile uint16_t temperatureC = 0;
volatile uint16_t turbidity = 0;
unsigned long lastDisplayUpdate = 0;
const unsigned long displayInterval = 1000; // 1-second interval for LCD update

void setupI2C() {
  TWBR = 32;                      // Set I2C clock frequency to 100kHz (16MHz clock)
  TWSR &= ~((1 << TWPS1) | (1 << TWPS0)); // No prescaler
  TWCR = (1 << TWEN);             // Enable TWI
}

uint16_t requestData(uint8_t slaveAddress) {
  uint16_t data = 0;

  // Send START condition
  TWCR = (1 << TWSTA) | (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));

  // Send Slave Address with Write bit
  TWDR = (slaveAddress << 1) | TW_WRITE;
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));

  // Send repeated START condition
  TWCR = (1 << TWSTA) | (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));

  // Send Slave Address with Read bit
  TWDR = (slaveAddress << 1) | TW_READ;
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));

  // Read high byte of data
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
  while (!(TWCR & (1 << TWINT)));
  data = TWDR << 8;

  // Read low byte of data
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));
  data |= TWDR;

  // Send STOP condition
  TWCR = (1 << TWSTO) | (1 << TWINT) | (1 << TWEN);

  return data;
}

void notifyBuzzer(bool dirty) {
  // Send START condition
  TWCR = (1 << TWSTA) | (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));

  // Send Slave Address with Write bit
  TWDR = (BUZZER_SLAVE_ADDRESS << 1) | TW_WRITE;
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));

  // Send buzzer status (1 for dirty, 0 for clean)
  TWDR = dirty ? 1 : 0;
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));

  // Send STOP condition
  TWCR = (1 << TWSTO) | (1 << TWINT) | (1 << TWEN);
}

void setupLCD() {
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  delay(2000);
  lcd.clear();
}

void setup() {
  setupI2C();
  setupLCD();
  sei(); // Enable global interrupts

  // Read the last saved temperature and turbidity from EEPROM
  temperatureC = EEPROM.read(EEPROM_TEMPERATURE_ADDR) << 8 | EEPROM.read(EEPROM_TEMPERATURE_ADDR + 1);
  turbidity = EEPROM.read(EEPROM_TURBIDITY_ADDR) << 8 | EEPROM.read(EEPROM_TURBIDITY_ADDR + 1);
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
    EEPROM.write(EEPROM_TEMPERATURE_ADDR, (temperatureC >> 8) & 0xFF);
    EEPROM.write(EEPROM_TEMPERATURE_ADDR + 1, temperatureC & 0xFF);
    EEPROM.write(EEPROM_TURBIDITY_ADDR, (turbidity >> 8) & 0xFF);
    EEPROM.write(EEPROM_TURBIDITY_ADDR + 1, turbidity & 0xFF);

    // Update LCD display
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(temperatureC);
    lcd.print(" C");

    lcd.setCursor(0, 1);
    if (turbidity > TURBIDITY_THRESHOLD) {
      lcd.print("Water: Dirty");
      notifyBuzzer(true); // Notify buzzer board
    } else {
      lcd.print("Water: Clean");
      notifyBuzzer(false); // Notify buzzer board
    }
  }
}
