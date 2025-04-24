#include <Stepper.h>
#include <DHT.h>
#include <RTClib.h>

// Pin Definitions

#define START_BUTTON_PIN 2
#define STOP_BUTTON_PIN 19
#define RESET_BUTTON_PIN 18
#define VENT_CONTROL_PIN 3
#define LED_YELLOW 32
#define LED_GREEN 30
#define LED_RED 34
#define LED_BLUE 36

#define DHT_PIN 7
#define FAN_MOTOR_PIN1 4
#define FAN_MOTOR_PIN2 7
#define POWER_PIN 6
#define WATER_LEVEL_PIN A0

// Stepper Motor Pins
#define speedPin 5
#define STEPPER_PIN3 24
#define STEPPER_PIN4 22
#define STEPPER_PIN1 28
#define STEPPER_PIN2 26


// Configuration Constants
#define VENT_POSITION_TOLERANCE 5
#define DHTTYPE DHT11
#define WATER_THRESHOLD 15
#define TEMP_LOW_THRESHOLD 10
#define TEMP_HIGH_THRESHOLD 20
#define STEPS_PER_REVOLUTION 2038

// Definition status
enum CoolerState {
  DISABLED,
  IDLE,
  RUNNING,
  ERROR
};

volatile bool ventControlRequested = false;

// Global Variables
CoolerState currentState = DISABLED;
DHT dht(DHT_PIN, DHTTYPE);
RTC_DS3231 rtc;
Stepper stepper(STEPS_PER_REVOLUTION, STEPPER_PIN1, STEPPER_PIN3, STEPPER_PIN2, STEPPER_PIN4);
int currentVentAngle = 0;

//  Start Button Interrupt Routine 
void startButtonISR() {
    Serial.println("Start Button Interrupt Triggered!"); 
    if (currentState == DISABLED) {
    currentState = IDLE;
    updateLEDs();
   }
}

// Stop Button Interrupt Service Routine
void stopButtonISR() {
  if (currentState == RUNNING) {
    // Turn off fan motor
    controlFanMotor(false);
    Serial.print("State Transition to DISABLED at: ");
    DateTime now = rtc.now();
    Serial.println(now.timestamp());
    currentState = DISABLED;
    updateLEDs();
  }
}

//  Reset Button Interrupt Service Routine 
void resetButtonISR() {
  if (currentState == ERROR || currentState == RUNNING) {
      currentState = IDLE;
      updateLEDs();
  }
}

void ventControlISR() {
  ventControlRequested = true;
}

void setup() {
  Serial.begin(9600);
  
  if (!rtc.begin()) {
    Serial.println("RTC failed");
    while (1);
  }
  rtc.begin();
  // DHT Sensor
  dht.begin();

  pinMode(speedPin, OUTPUT);
  
  // Stepper Motor
  stepper.setSpeed(60);

  // Pin Mode Setup
  pinMode (POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, LOW);

  // Fan Setup
  pinMode(FAN_MOTOR_PIN1, OUTPUT);
  pinMode(FAN_MOTOR_PIN2, OUTPUT);


  // LED Pins
  pinMode(LED_YELLOW, OUTPUT);

  pinMode(LED_GREEN, OUTPUT);

  pinMode(LED_RED, OUTPUT);

  pinMode(LED_BLUE, OUTPUT);
  
  // Button Interrupts
  pinMode(VENT_CONTROL_PIN, INPUT_PULLUP);

  pinMode(START_BUTTON_PIN, INPUT_PULLUP);

  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);

  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(START_BUTTON_PIN), startButtonISR, FALLING);

  attachInterrupt(digitalPinToInterrupt(STOP_BUTTON_PIN), stopButtonISR, FALLING);

  attachInterrupt(digitalPinToInterrupt(RESET_BUTTON_PIN), resetButtonISR, FALLING);

  attachInterrupt(digitalPinToInterrupt(VENT_CONTROL_PIN), ventControlISR, FALLING);
   
  
  // State Setup
  currentState = DISABLED;
  digitalWrite(LED_YELLOW, HIGH);  
  Serial.println("Setup Complete. Start Button on Pin 2");
  initializeVentControl();
}

void controlFanMotor(bool enable) {
  if (enable) {
    // Low/High to simulate motor rotation
    digitalWrite(FAN_MOTOR_PIN1, HIGH);
    digitalWrite(FAN_MOTOR_PIN2, LOW);
    analogWrite(speedPin, 255);

  } else {
    // Stop motor
    digitalWrite(FAN_MOTOR_PIN1, LOW);
    digitalWrite(FAN_MOTOR_PIN2, LOW);
  }
}

void initializeVentControl() {
  if (currentState == DISABLED) {
    int buttonState = digitalRead(VENT_CONTROL_PIN);
    if (buttonState == LOW) {
      stepper.setSpeed(10);
      stepper.step(STEPS_PER_REVOLUTION/3);
      delay(10);
    }
  }
}

void updateLEDs() {
  // Turn off LEDs 
  digitalWrite(LED_YELLOW, LOW);

  digitalWrite(LED_GREEN, LOW);

  digitalWrite(LED_RED, LOW);

  digitalWrite(LED_BLUE, LOW);
  
  // Turn on LED based on current state
  switch(currentState) {
    case DISABLED:
      digitalWrite(LED_YELLOW, HIGH);
      break;
    case IDLE:
      digitalWrite(LED_GREEN, HIGH);
      break;
    case RUNNING:
      digitalWrite(LED_BLUE, HIGH);
      break;
    case ERROR:
      digitalWrite(LED_RED, HIGH);
      break;
  }
}

unsigned long lastUpdateTime = 0;
unsigned long updateInterval = 60000;
unsigned long fanStartTime = 0;
const unsigned long fanRunTime = 60000;

void loop() {
   DateTime now = rtc.now();
   if (ventControlRequested && currentState != ERROR) {
   ventControlRequested = false; 
   stepper.setSpeed(10);
   stepper.step(STEPS_PER_REVOLUTION/3);
  }
  switch(currentState) {
    case DISABLED:
        
      break;
    
    case IDLE:
      // Check Water Level
      Serial.print("State Transition to IDLE at: ");
      Serial.println(now.timestamp());
      
      digitalWrite (POWER_PIN, HIGH);
      int waterLevel = analogRead(WATER_LEVEL_PIN);
      digitalWrite(POWER_PIN, LOW);
      
      if (waterLevel < WATER_THRESHOLD) {
        currentState = ERROR;
        Serial.print("State Transition to ERROR at: ");
        Serial.println(now.timestamp());
        updateLEDs();
        break;
      }
      
      // Monitor Temperature
      float temperature = dht.readTemperature();
      float humidity = dht.readHumidity();
      
      Serial.print("Temp: ");

      Serial.print(temperature);

      Serial.print("°C, Humidity: ");

      Serial.print(humidity);

      Serial.println("%");
      
      // Change temperature if it is high
      if (temperature > TEMP_HIGH_THRESHOLD) {
        
        Serial.print("State Transition to RUNNING at: ");
        Serial.println(now.timestamp());
        currentState = RUNNING;
        updateLEDs();
      }
    
  case RUNNING:
  Serial.println("Entering RUNNING state");
  controlFanMotor(true); // Turn on the fan
  fanStartTime = millis();

  // Check Water Level
  digitalWrite(POWER_PIN, HIGH);
  waterLevel = analogRead(WATER_LEVEL_PIN);
  digitalWrite(POWER_PIN, LOW);

  Serial.print("Water level is currently: ");

  Serial.println(waterLevel);

  if (waterLevel < WATER_THRESHOLD) {
    currentState = ERROR;
    Serial.println("State Transition to ERROR: Low Water Level");
    updateLEDs();
    break;
  }

    // Monitor Temperature
    temperature = dht.readTemperature();
    humidity = dht.readHumidity();

    Serial.print("Temp: ");

    Serial.print(temperature);

    Serial.print("°C, Humidity: ");

    Serial.print(humidity);

    Serial.println("%");
  
    if (millis() - fanStartTime >= fanRunTime) {
      Serial.println("Fan run time exceeded, checking statstics");
    }
    // Change if temperature drops below limit
    if (temperature <= TEMP_HIGH_THRESHOLD) {
      Serial.println("Temperature dropped, transitioning to IDLE");
      currentState = IDLE;
      controlFanMotor(false); 
      updateLEDs();
    } else {
       fanStartTime = millis();
    }
  break;
    case ERROR:
      // Turn off Fan Motor
      controlFanMotor(false);
      
      Serial.println("ERROR: Low Water Level");  
  }
  delay(5000);
}