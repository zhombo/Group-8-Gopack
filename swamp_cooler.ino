//CPE 301
//Professor Bashira Akter Anima
//Group#8
//Members(Louis-Pierce,Pinky-Nguyen,Bella-Picasso-Kenedy,Alexus-Rowe)

#include <LiquidCrystal.h>
#include <dht.h>
#include <Stepper.h>
#include <Wire.h>
#include <DS1307RTC.h>

// Pin Definitions (Arduino Mega 2560)
#define GREEN_LED 13  // PB7
#define YELLOW_LED 12 // PB6
#define RED_LED 11    // PB5
#define BLUE_LED 10   // PB4
#define FAN_PIN 9     // PH6 (PWM-capable)
#define RESET_PIN 2   // PE4
#define STOP_PIN 3    // PE5
#define START_PIN 18  // PJ1 (INT3)
#define VENT_LEFT_BUTTON 42  // PL7
#define VENT_RIGHT_BUTTON 46 // PL3
#define DHT_PIN 22           // PA0
#define RS_PIN 4             // PE6
#define EN_PIN 5             // PG5
#define D4_PIN 6             // PH3
#define D5_PIN 7             // PH4
#define D6_PIN 8             // PH5
#define D7_PIN 9             // PH6
#define WATER_SENSOR A0      // PF0 (ADC0)

// Constants
const float TEMP_THRESHOLD_HIGH = 25.0; // Adjusted for realistic testing
const unsigned long LCD_UPDATE_INTERVAL = 60000;
const int STEPPER_STEPS_PER_REV = 200;  // Adjust based on your stepper motor
const int STEPPER_STEP = 10;            // Steps per button press
const int WATER_LEVEL_THRESHOLD = 200;  // Calibrate based on sensor
const unsigned long DEBOUNCE_DELAY = 50;

// Globals
LiquidCrystal lcd(RS_PIN, EN_PIN, D4_PIN, D5_PIN, D6_PIN, D7_PIN);
dht DHT;
Stepper ventStepper(STEPPER_STEPS_PER_REV, 44, 45, 46, 47); // PL5, PL4, PL3, PL2
String systemState = "DISABLED";
volatile bool startPressed = false;
volatile bool stopPressed = false;
float currentTemp = 18.0;
float currentHumidity = 0.0;
int currentWaterLevel = 0;
unsigned long lastUpdateMillis = 0;
int ventPosition = 0; // Stepper position in steps
unsigned long lastButtonPress = 0;

// Function Prototypes
void initializeSystem();
void handleStates();
void updateSensors();
void printToLCD();
void logEvent(String message);
void logStartTime();
void adjustVent();
void checkWaterLevel();
void enterDisabledState();
void enterIdleState();
void enterErrorState();
void enterRunningState();
void ISR_StartButton();
void ISR_StopButton();
void setupADC();
int readADC();
void initUART();
void sendChar(char c);
void serialPrint(String message);

// Register-Based I/O Functions
void setLEDs(bool green, bool yellow, bool red, bool blue) {
  PORTB = (green << PB7) | (yellow << PB6) | (red << PB5) | (blue << PB4);
}

void setFan(bool on) {
  if (on) {
    // Use PWM for fan (full speed)
    TCCR2A |= (1 << COM2A1); // Enable PWM on OC2A (pin 9)
    OCR2A = 255;             // Full speed
  } else {
    OCR2A = 0; // Stop fan
  }
}

bool readButton(uint8_t port, uint8_t pin) {
  if (port == 'E') return (PINE & (1 << pin)) ? HIGH : LOW;
  if (port == 'J') return (PINJ & (1 << pin)) ? HIGH : LOW;
  if (port == 'L') return (PINL & (1 << pin)) ? HIGH : LOW;
  return HIGH; // Default to HIGH (not pressed)
}

void setup() {
  initializeSystem();
}

void loop() {
  handleStates();
}

void initializeSystem() {
  // Initialize UART for logging
  initUART();

  // LED Pins Setup (PB4-PB7 as outputs)
  DDRB |= (1 << DDB7) | (1 << DDB6) | (1 << DDB5) | (1 << DDB4);

  // Fan Pin Setup (PH6 as output, PWM)
  DDRH |= (1 << DDH6);
  TCCR2A = (1 << WGM21) | (1 << WGM20); // Fast PWM
  TCCR2B = (1 << CS22);                 // Prescaler 64
  setFan(false);

  // Button Pins Setup (PE4, PE5, PJ1, PL7, PL3 as inputs with pull-ups)
  DDRE &= ~((1 << DDE4) | (1 << DDE5));
  PORTE |= (1 << PE4) | (1 << PE5);
  DDRJ &= ~(1 << DDJ1);
  PORTJ |= (1 << PJ1);
  DDRL &= ~((1 << DDL7) | (1 << DDL3));
  PORTL |= (1 << PL7) | (1 << PL3);

  // Attach Interrupts for Start and Stop Buttons
  EICRA |= (1 << ISC31) | (1 << ISC30); // Rising edge for INT3 (START_PIN)
  EICRA |= (1 << ISC51) | (1 << ISC50); // Rising edge for INT5 (STOP_PIN)
  EIMSK |= (1 << INT3) | (1 << INT5);

  // LCD Initialization
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Initializing...");

  // Initialize Stepper Motor
  ventStepper.setSpeed(60); // RPM, adjust as needed
  ventStepper.step(0);      // Hold at initial position

  // Initialize ADC
  setupADC();

  // Read initial sensor values
  updateSensors();

  // Start in Disabled State
  enterDisabledState();
}

void handleStates() {
  if (systemState == "DISABLED") {
    enterDisabledState();
  } else if (systemState == "IDLE") {
    enterIdleState();
  } else if (systemState == "ERROR") {
    enterErrorState();
  } else if (systemState == "RUNNING") {
    enterRunningState();
  }
}

void enterDisabledState() {
  setLEDs(0, 1, 0, 0); // Yellow ON
  setFan(false);

  lcd.clear();
  lcd.print("System Disabled");

  if (startPressed) {
    systemState = "IDLE";
    lcd.clear();
    lcd.print("System Enabled");
    logEvent("System Enabled");
    logStartTime();
    startPressed = false;
  }
}

void enterIdleState() {
  setLEDs(1, 0, 0, 0); // Green ON
  setFan(false);

  updateSensors();
  checkWaterLevel();
  printToLCD();
  adjustVent();

  if (currentTemp >= TEMP_THRESHOLD_HIGH) {
    systemState = "RUNNING";
    logEvent("Entering RUNNING state");
  }

  if (stopPressed) {
    systemState = "DISABLED";
    logEvent("System Stopped");
    stopPressed = false;
    startPressed = false;
  }
}

void enterErrorState() {
  setLEDs(0, 0, 1, 0); // Red ON
  setFan(false);

  updateSensors();

  lcd.clear();
  lcd.print("Water Level Low!");

  if (readButton('E', PE4) == LOW && currentWaterLevel > WATER_LEVEL_THRESHOLD && millis() - lastButtonPress > DEBOUNCE_DELAY) {
    lastButtonPress = millis();
    systemState = "IDLE";
    logEvent("Error Resolved");
  }
}

void enterRunningState() {
  setLEDs(0, 0, 0, 1); // Blue ON
  setFan(true);

  updateSensors();
  printToLCD();
  adjustVent();
  checkWaterLevel();

  if (currentTemp < TEMP_THRESHOLD_HIGH) {
    systemState = "IDLE";
    logEvent("Temperature Normal");
  }

  if (stopPressed) {
    systemState = "DISABLED";
    logEvent("System Stopped");
    stopPressed = false;
    startPressed = false;
  }
}

void updateSensors() {
  DHT.read11(DHT_PIN);
  currentTemp = DHT.temperature;
  currentHumidity = DHT.humidity;
  currentWaterLevel = readADC();

  // Print live sensor timings to UART
  serialPrint("Temp: " + String(currentTemp) + " C\tHum: " + String(currentHumidity) + " %\tWater: " + String(currentWaterLevel) + "\n");
}

void printToLCD() {
  if (systemState != "DISABLED" && millis() - lastUpdateMillis >= LCD_UPDATE_INTERVAL) {
    lastUpdateMillis = millis();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(currentTemp);
    lcd.print("C");

    lcd.setCursor(0, 1);
    lcd.print("Hum:");
    lcd.print(currentHumidity, 1);
    lcd.print("% W:");
    lcd.print(currentWaterLevel);
  }
}

void logEvent(String message) {
  tmElements_t tm;
  RTC.read(tm);
  String timestamp = String(tm.Hour) + ":" + String(tm.Minute) + ":" + String(tm.Second) + " " +
                    String(tm.Month) + "/" + String(tm.Day) + "/" + String(tmYearToCalendar(tm.Year)) + " - ";
  serialPrint(timestamp + message + "\n");
}

void logStartTime() {
  tmElements_t tm;
  RTC.read(tm);
  String timestamp = "System Start Time: " + String(tm.Hour) + ":" + String(tm.Minute) + ":" + String(tm.Second) + " " +
                    String(tm.Month) + "/" + String(tm.Day) + "/" + String(tmYearToCalendar(tm.Year)) + "\n";
  serialPrint(timestamp);
}

void adjustVent() {
  if (readButton('L', PL7) == LOW && millis() - lastButtonPress > DEBOUNCE_DELAY) {
    lastButtonPress = millis();
    ventPosition -= STEPPER_STEP;
    ventStepper.step(-STEPPER_STEP);
    logEvent("Vent Moved Left: " + String(ventPosition) + " steps");
  } else if (readButton('L', PL3) == LOW && millis() - lastButtonPress > DEBOUNCE_DELAY) {
    lastButtonPress = millis();
    ventPosition += STEPPER_STEP;
    ventStepper.step(STEPPER_STEP);
    logEvent("Vent Moved Right: " + String(ventPosition) + " steps");
  }
}

void checkWaterLevel() {
  if (currentWaterLevel < WATER_LEVEL_THRESHOLD) {
    systemState = "ERROR";
    logEvent("Water Level Low");
  }
}

void ISR_StartButton() {
  startPressed = true;
}

void ISR_StopButton() {
  stopPressed = true;
}

void setupADC() {
  ADMUX = (1 << REFS0);                  // Reference voltage AVcc
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Enable ADC, prescaler 64
}

int readADC() {
  ADMUX &= 0xF0; // Select ADC0 (A0)
  ADCSRA |= (1 << ADSC); // Start conversion
  while (ADCSRA & (1 << ADSC)); // Wait for completion
  return ADC;
}

void initUART() {
  UBRR0 = 103; // 9600 baud at 16 MHz
  UCSR0B = (1 << TXEN0); // Enable transmitter
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8-bit data
}

void sendChar(char c) {
  while (!(UCSR0A & (1 << UDRE0))); // Wait for empty buffer
  UDR0 = c; // Send character
}

void serialPrint(String message) {
  for (unsigned int i = 0; i < message.length(); i++) {
    sendChar(message[i]);
  }
}