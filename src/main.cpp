#include "myArduino.h" // Contains our register-level versions of common Arduino functions
#include <Stepper.h> // Stepper library, provided by Adafruit, for stepper motor control
#include <RTClib.h> // RTC library, provided by Adafruit, for clock module
#include <SPI.h> // Needed so RTClib.h can work
#include <DHT.h> // DHT library, provided by Adafruit, for temp and humidity sensor
#include <Adafruit_Sensor.h> //Needed so DHT can work
#include <LiquidCrystal.h> // LCD display library

// ADC, Timer, UART, and other functions that are typically available in the Arduino library
// that we must recreate can be found in myArduino.h. They have been seperated for clarity within the main file.  

//Definitions and constraints
// Define variable to track state.
enum state{IDLE, RUNNING, DISABLED, ERROR};
state currentState = DISABLED;
state previousState = DISABLED;

//Define variables for switching states
float currentTemp = 0;
float currentWaterLevel = 0;
const int WATER_THRESHOLD = 110; //Level of our reservoir threshold
const float TEMP_THRESHOLD = 30.00; //DHT Sensor is slightly skewed

//Configure on/off button
const int START_DIGITAL_PIN = 18; //For attachInterrupt()
const int START_PIN = PD3;
volatile bool on = false;

// Define state LED GPIO constants
const int LED_IDLE = PB3; // D50
const int LED_RUNNING = PB2; // D51
const int LED_DISABLED = PB1; // D52
const int LED_ERROR = PB0; // D53

// Configure vent fan, vent fan control, and main fan
const int stepsPerRevolution = 2038;
Stepper vent = Stepper(stepsPerRevolution, 23, 27, 25, 29); // IN1 = 23, IN2 = 25, IN3 = 27, IN4 = 29
int currentStepperPos = 0, previousVentPos = 0;
const int VENT_POT_PIN = 0; // ADC channel 0
const int FAN_PIN = PC2; //D35

//Configure clock, temp and humidity sensor, water level sensor 
RTC_DS1307 clock;
const int DHT_PIN = 45;
DHT dht(DHT_PIN, DHT11);
const int WATER_LVL_PIN = 1; // ADC channel 1

//Configure LCD pins and variables for LCD delay
const int RS = 12, EN = 11, D4 = 5, D5 = 4, D6 = 3, D7 = 2;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
unsigned long previousMillis = 0, currentMillis = 0;
const unsigned long updateDisplayInterval = 60000; //60000 ms = 1 minute

// ==== FUNCTION PROTOTYPES ====
//Setup functions
void gpioInit();
void sensorsMotorsInit();

//State control functions
void toggleOn();
void checkState();
void updateLED();

//Vent control functions
void controlVent();
void printVentPosition(int newpos);

//Event reporting functions
void printCurrentTime();
void printStateChange();

//Display temperature functions
void displayTempAndHumidity();

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}

// Function Definitions
void gpioInit() {
    //Set all LEDs to output, then all to LOW
    DDRB |= (1 << LED_IDLE) | (1 << LED_RUNNING) | (1 << LED_DISABLED) | (1 << LED_DISABLED);
    PORTB &= ~((1 << LED_IDLE) | (1 << LED_RUNNING) | (1 << LED_DISABLED) | (1 << LED_ERROR));

    // Setup motor as output, initialize to LOW to disable fan
    DDRC |= (1 << FAN_PIN);
    PORTC &= ~(1 << FAN_PIN);

    // Setup on/off button as input w_pullup.
    DDRD &= ~(1 << START_PIN);
    PORTD |= (1 << START_PIN);
    attachInterrupt(digitalPinToInterrupt(START_DIGITAL_PIN), toggleOn, FALLING);
}
void sensorsMotorsInit() {
    // Begin RTC module
    if (!clock.begin()) {
        U0printString("Couldn't find clock");
        while (1);
    }

    // Begin DHT module
    dht.begin();

    // Water level sensor needs just simple ADC read, no setup needed

    // Setup vent speed
    vent.setSpeed(10);
}

void toggleOn() {
    on = !on;
}

void controlVent()
{
  int potValue = adc_read(VENT_POT_PIN);

  //map the range of potentiometer to range of steps
  int targetPosition = map(potVAlue, 0 , 1023, 0, stepsPerRevolution);

  //converting steps to degrees
  int targetDegrees = map(targetPosition, 0, stepsPerRevolution, 0, 360);
  int currentDegrees = map(currentStepperPos, 0, stepsPerRevolution, 0, 360);

  //check whether movement past threshold before moving vent
  if (abs(targetDegrees - currentDegrees) > 2)
  {
    int toMove = targetPosition - currentStepperPos;
    vent.step(toMove);
    currentStepperPos = targetPosition;

    //print vent position
    printVentPosition(targetDegrees);
  }

  currentStepperPos = targetPosition;

}

void updateLED()
{
  //set to low, then set state indicator to high
  PORTB &= ~((1 << LED_IDLE) |  (1 << LED_RUNNING) | ( 1 << LED_DISABLED) | (1 << LED_ERROR));

  switch(currentState) 
  {
    case IDLE: PORTB |= (1 << LED_IDLE); 
    break;

    case RUNNING: PORTB |= (1 << LED_RUNNING);
    break;

    case DISABLED: PORTB |= ( 1 << LED_DISABLED);
    break;

    case ERROR: PORTB |= (1 << LED_ERROR);
    break;
  }
}


void checkState() 
{
    //readTemperature and readTemperature(true) are goofy
    currentTemp = dht.readTemperature();
    currentWaterLevel = adc_read(1);

    //handling whether anything is on
    switch (currentState) {
      case IDLE: 
            if (!on) currentState = DISABLED;
            else if (currentWaterLevel <= WATER_THRESHOLD) currentState = ERROR;
            else if (TEMP_THRESHOLD < currentTemp) currentState = RUNNING;
            break;
      
      case RUNNING:
            if (!on) currentState = DISABLED;
            else if (currentWaterLevel <= WATER_THRESHOLD) currentState = ERROR;
            else if (TEMP_THRESHOLD < currentTemp) 
            {
              currentState = IDLE;
              //turns off fan
              PORTC &= ~(1 << FAN_PIN);
            }
            break;

      case DISABLED:
            if (on) currentState = IDLE;

      case ERROR:
            if (!on) currentState = DISABLED;
            else if (currentWaterLevel > WATER_THRESHOLD) currentState = IDLE;
            break;

      default:
      break;
    }
}