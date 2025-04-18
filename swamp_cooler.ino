#include <Wire.h>
#include "RTClib.h"
#include <LiquidCrystal.h>
#include <DHT.h>
#include <Servo.h>

#define DECL_REG_U8(name, addr) volatile unsigned char *name = (volatile unsigned char *)addr;
#define DECL_REG_U16(name, addr) volatile unsigned int *name = (volatile unsigned int *)addr;

class SwampCooler;
class StateInterface;
class DisabledState;
class IdleState;
class RunningState;
class ErrorState;

int getWaterLevel();
int getTemperature();
int getHumidity();

void disableAll();
void setDisabledOutputs();
void setIdleOutputs();
void setRunningOutputs();
void setErrorOutputs();
void updateLCDStats();
void printRTCTime();
void adcInit();
unsigned int adcRead(unsigned char adc_channel_num);

//PORTA Registers
volatile unsigned char *porta = (unsigned char *)0x22;
volatile unsigned char *pina = (unsigned char *)0x20;
volatile unsigned char *ddra = (unsigned char *)0x21;

//Position of LEDs and motor on PORTA
const unsigned char PORTA_YELLOW = 1;
const unsigned char PORTA_GREEN = 3;
const unsigned char PORTA_BLUE = 5;
const unsigned char PORTA_MOTOR = 6;
const unsigned char PORTA_RED = 7;

//Analog read registers
DECL_REG_U8(myADCSRA, 0x7A);
DECL_REG_U8(myADCSRB, 0x7B);
DECL_REG_U8(myADMUX, 0x7C);
DECL_REG_U8(myDIDR0, 0x7E);
DECL_REG_U8(myADCL, 0x78);
DECL_REG_U8(myADCH, 0x79);

// interrupt registers
DECL_REG_U8(myEIMSK, 0x3D);
DECL_REG_U8(myEICRA, 0x69);

// Button registers
DECL_REG_U8(myDDRD, 0x2A);
DECL_REG_U8(myPORTD, 0x2B);

// The pin for the water sensor
const unsigned char WATER_SENSOR_PIN = 0;

// The pin for the disable button
const unsigned char BUTTON_PIN = 18;

// The pin for the dht sensor
const unsigned char DHT_PIN = 19;

// The pin for the servo
const unsigned char SERVO_PIN = 7;

// The yellow led pin
const unsigned char YELLOW_LED_PIN = 23;
// The green led pin
const unsigned char GREEN_LED_PIN = 25;
// The blue led pin
const unsigned char BLUE_LED_PIN = 27;
// The red led pin
const unsigned char RED_LED_PIN = 29;

// The motor pin
const unsigned char MOTOR_PIN = 28;

// The time between lcd updates in ms. The DHT sensor only updates around 1Hz. The LCD also cannot display constantly.
const unsigned long LCD_UPDATE_INTERVAL = 1000;

// The time between servo updates in ms.
const unsigned long SERVO_UPDATE_INTERVAL = 50;

// The low water analog reading limit
int lowWaterThreshold = 150;

// The upper limit on the temperature, in degrees Celcius
float tempHighThreshold = 23.0;

// The lower limit on the temperature, in degrees Celcius
float tempLowThreshold = 19.0;

// The lcd display for DHT sensor readings
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// The DHT Temperature and Humidity sensor
DHT dht(DHT_PIN, DHT11);

// Real time clock module
RTC_DS1307 rtc;

// The current Date and Time
DateTime now;

// The servo for vent control
Servo servo;

// Holds latest water level
unsigned int waterLevel = 0;

// The current servo angle.
unsigned int servoAngle = 90;

// Holds latest temperature reading. Is Nan before readings are taken.
float temperature = NAN;

// Holds latest humidity reading. Is Nan before readings are taken.
float humidity = NAN;

// Is true when the button has been pressed. Must be manually reset with `buttonPressed = false`.
volatile bool buttonPressed = false;

// The time of the last lcd update
unsigned long lastLCDUpdate = 0;

// The time of the last servo update
unsigned long lastServoUpdate = 0;

// An enum of every possible state of a `SwampCooler`
enum State
{
  Disabled = 0,
  Idle = 1,
  Running = 2,
  Error = 3,
};

class StateInterface
{
protected:
  SwampCooler *sc;

public:
  virtual void disable_enable() = 0;
  virtual void checkWater() = 0;
  virtual void checkTemp() = 0;
  virtual void updateLCD() = 0;
};

class DisabledState : public StateInterface
{
public:
  DisabledState(SwampCooler *s);
  void disable_enable();
  void checkWater();
  void checkTemp();
  void updateLCD();
};

class IdleState : public StateInterface
{
public:
  IdleState(SwampCooler *s);
  void disable_enable();
  void checkWater();
  void checkTemp();
  void updateLCD();
};

class RunningState : public StateInterface
{
public:
  RunningState(SwampCooler *s);
  void disable_enable();
  void checkWater();
  void checkTemp();
  void updateLCD();
};

class ErrorState : public StateInterface
{
public:
  ErrorState(SwampCooler *s);
  void disable_enable();
  void checkWater();
  void checkTemp();
  void updateLCD();
};

class SwampCooler
{
  StateInterface *currentstate;
  DisabledState disabled;
  IdleState idle;
  RunningState running;
  ErrorState error;

public:
  State state;

  SwampCooler();
  void update();
  void setDisabled();
  void setIdle();
  void setRunning();
  void setError();
};

//********************DisabledState Methods********************
DisabledState::DisabledState(SwampCooler *s)
{
  sc = s;
}

void DisabledState::disable_enable()
{
  sc->setIdle();
}

void DisabledState::checkWater()
{
}

void DisabledState::checkTemp()
{
}

// Updates the LCD for the disabled state
void DisabledState::updateLCD()
{
  // Do nothing. LCD was populated during state transition.
}

//********************IdleState Methods********************
IdleState::IdleState(SwampCooler *s)
{
  sc = s;
}

void IdleState::disable_enable()
{
  sc->setDisabled();
}

void IdleState::checkWater()
{
  if (getWaterLevel() < lowWaterThreshold)
    sc->setError();
}

void IdleState::checkTemp()
{
  // Update humidity stat
  getHumidity();

  if (getTemperature() > tempHighThreshold)
    sc->setRunning();
}

// Update LCD for the idle state
void IdleState::updateLCD()
{
  updateLCDStats();
}

//********************RunningState Methods********************
RunningState::RunningState(SwampCooler *s)
{
  sc = s;
}

void RunningState::disable_enable()
{
  sc->setDisabled();
}

void RunningState::checkWater()
{
  if (getWaterLevel() < lowWaterThreshold)
  {
    Serial.print("Motor off, Changed state to error on: ");
    printRTCTime();
    sc->setError();
  }
}

void RunningState::checkTemp()
{
  if (getTemperature() < tempLowThreshold)
  {
    Serial.print("Motor off, Changed state to idle on: ");
    printRTCTime();
    sc->setIdle();
  }
}

// Update LCD for the Running state
void RunningState::updateLCD()
{
  // Update humidity stat
  getHumidity();
  updateLCDStats();
}

//********************ErrorState Methods********************
ErrorState::ErrorState(SwampCooler *s)
{
  sc = s;
}

void ErrorState::disable_enable()
{
  sc->setDisabled();
}

void ErrorState::checkWater()
{
  if (getWaterLevel() > lowWaterThreshold)
    sc->setIdle();
}

void ErrorState::checkTemp()
{
}

// Update the lcd for the Error state
void ErrorState::updateLCD()
{
  // Do nothing. LCD was written to during state transition.
}

//********************SwampCooler steps********************

SwampCooler::SwampCooler() : disabled{this}, idle{this}, running{this}, error{this}
{
  setIdle();
}

void SwampCooler::update()
{
  if (buttonPressed)
  {
    currentstate->disable_enable();
    buttonPressed = false;
  }

  currentstate->checkWater();
  currentstate->checkTemp();

  unsigned long time = millis();
  if (time - lastLCDUpdate > LCD_UPDATE_INTERVAL)
  {
    currentstate->updateLCD();
    lastLCDUpdate = time;
  }

  if (time - lastServoUpdate > SERVO_UPDATE_INTERVAL)
  {
    unsigned int reading = adcRead(1);

    reading /= 6; // 1024 / 180 = 5.688, close enough. Avoid floats for speed.

    unsigned int diff = reading > servoAngle ? reading - servoAngle : servoAngle - reading;
    if (diff > 10)
    {
      servo.write(reading);
      servoAngle = reading;
    }

    lastServoUpdate = time;
  }
}

// the disabled state
void SwampCooler::setDisabled()
{
  setDisabledOutputs();
  state = State::Disabled;
  currentstate = &disabled;
  currentstate->updateLCD();
}

// the idle state
void SwampCooler::setIdle()
{
  state = State::Idle;
  setIdleOutputs();
  currentstate = &idle;
}

// the running state
void SwampCooler::setRunning()
{
  setRunningOutputs();
  state = State::Running;
  currentstate = &running;
}

// the error state
void SwampCooler::setError()
{
  setErrorOutputs();
  state = State::Error;
  currentstate = &error;
}

//***************OUTPUT FUNCTIONS****************
void disableAll()
{
  // Disable all LEDs and motor
  *porta &= 0b00010101;
}

void setDisabledOutputs()
{
  disableAll();

  // Set Yellow LED to high
  *porta |= (1 << PORTA_YELLOW);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Disabled");
}

void setIdleOutputs()
{
  disableAll();

  // Enable Green LED
  *porta |= (1 << PORTA_GREEN);
}

void setRunningOutputs()
{
  disableAll();
  Serial.print("Motor on, Changed state to running on: ");
  printRTCTime();

  // Enable blue LED
  *porta |= (1 << PORTA_BLUE);
  *porta |= (1 << PORTA_MOTOR);
}

void setErrorOutputs()
{
  disableAll();

  // Enable Red LED
  *porta |= (1 << PORTA_RED);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Water Low");
}

// Get the water level and cache it the result in `waterlevel`
int getWaterLevel()
{
  unsigned int reading = adcRead(0);
  unsigned int diff = waterLevel > reading ? waterLevel - reading : reading - waterLevel;
  if (diff > 5)
    waterLevel = reading;

  return waterLevel;
}

// Get the temperature and cache the result in `temperature`
int getTemperature()
{
  temperature = dht.readTemperature();
  return temperature;
}

// Get the temperature and cache the result in `humidity`
int getHumidity()
{
  humidity = dht.readHumidity();
  return humidity;
}

// Update the lcd with humidity and temperature stats
void updateLCDStats()
{
  lcd.clear();
  lcd.setCursor(0, 0);

  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print("\xDF"
            "C");

  lcd.setCursor(0, 1);

  lcd.print("Humidity: ");
  lcd.print(humidity);
  lcd.print("%");
}

// Print time using RTC
void printRTCTime()
{
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" at ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
}

// Init ADC
void adcInit()
{
  // 7: enable adc, 2-0: 128 prescaler
  *myADCSRA = 0b10000111;

  // reset channel
  *myADMUX = 0b01000000;
}

// Read ADC value from channel
unsigned int adcRead(unsigned char adc_channel)
{
  // Set Channel
  *myADMUX = (*myADMUX & 0b11111000) | (adc_channel & 0b00000111);

  // Set MUX5 to 0
  *myADCSRB &= ~0b00001000;

  // Start Reading
  *myADCSRA |= 0b01000000;

  // Wait
  while ((*myADCSRA & 0x40) != 0)
    ;

  unsigned int low = *myADCL;
  unsigned int high = *myADCH;

  return (high << 8) | low;
}

//************MAIN***************

SwampCooler swampcooler;

// The time in millis since arduino startup of the last button press. Used for debouncing.
volatile unsigned long lastButtonPressTime = 0;
// The time between button presses.
volatile unsigned long buttonPressDebounceThreshold = 200;

// Whether the last button event was high
volatile bool lastButtonWasHigh = false;

// ISR handler for button presses
void processButtonPressISR()
{
  unsigned long currentButtonPressTime = millis();

  if (currentButtonPressTime - lastButtonPressTime > buttonPressDebounceThreshold)
    buttonPressed = true;

  lastButtonPressTime = currentButtonPressTime;
}

ISR(INT3_vect)
{
  unsigned long currentButtonPressTime = millis();

  if (currentButtonPressTime - lastButtonPressTime > buttonPressDebounceThreshold)
    buttonPressed = true;

  lastButtonPressTime = currentButtonPressTime;
}

void setup()
{
  Serial.begin(9600);

  // initialize analog read
  adcInit();

  // initialize RTC module
  Wire.begin();
  rtc.begin();
  now = rtc.now();

  // configure button pin
  *myDDRD &= ~0b00001000;
  *myPORTD |= 0x01 << 3;

  // Set LED and motor pinmodes
  *ddra |= 0b11101010;

  // Setup button ISR int3
  *myEIMSK |= 0b00001000;
  // Falling edge for int3
  *myEICRA |= 0b10000000;

  //Initialize lcd
  lcd.begin(16, 2);

  //Initialize Temp/Humidity sensor
  dht.begin();

  //Initialize servo
  servo.attach(SERVO_PIN);
}

void loop()
{
  swampcooler.update();
  now = rtc.now(); //load current time from rtc into "now"
}