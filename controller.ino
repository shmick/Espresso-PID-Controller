/*
  Control an espresso machine boiler using a PID controller

  Code samples used from the following:
  PID Library: https://github.com/br3ttb/Arduino-PID-Library
  PID Lab: https://www.pdx.edu/nanogroup/sites/www.pdx.edu.nanogroup/files/2013_Arduino%20PID%20Lab_0.pdf
  Smoothing: https://www.arduino.cc/en/Tutorial/Smoothing

  Hardware:
  HeaterMeter v4.2 PCB: https://github.com/CapnBry/HeaterMeter/wiki/HeaterMeter-4.2-Hardware
  Solid State Relay: "25A SSR-25 DA"
*/

#include <PID_v1.h>

// After powering on, how many minutes until we force the boiler to power down
// Turning the machine off and on again will reset the timer
const int maxRunTime = 45;
int timeNowMins;

// HM Pins
// TC = ADC 5
// Blower/Relay = Digital Pin 3

#define ThermocouplePin 5
#define RelayPin 3

//Define the aggressive Tuning Parameters
const double aggKp = 40;
const double aggKi = 0.5;
const double aggKd = 100;

//Define the conservative Tuning Parameters
const double consKp = 4;
const double consKi = 0.01;
const double consKd = 3;

//Define the gap degrees to switch between aggressive and conservative mode
const int gapDeg = 15;

//Specify the links and initial tuning parameters
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
const int WindowSize = 5000;
unsigned long windowStartTime;

//Define the info needed for the temperature averaging
const int numReadings = 128;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

// Communication setup
const long serialPing = 500; //This determines how often we ping our loop
// Serial pingback interval in milliseconds
unsigned long now = 0; //This variable is used to keep track of time
// placehodler for current timestamp
unsigned long lastMessage = 0; //This keeps track of when our loop last spoke to serial
unsigned long lastMessage2 = 0; //This keeps track of when our loop last spoke to serial
// last message timestamp.

void setup()
{
  // Setup Serial
  Serial.begin(38400); //Start a serial session
  lastMessage = millis(); // timestamp

  // Set the Relay to output mode and ensure the relay if off
  pinMode(RelayPin, OUTPUT);
  digitalWrite(RelayPin, LOW);

  // PID settings
  windowStartTime = millis();
  Setpoint = 105;
  myPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
}

void loop()
{
  //Keep track of time
  now = millis();

  //get a non averaged reading
  int raw = analogRead(ThermocouplePin);

  // subtract the last reading:
  total = total - readings[readIndex];
  // Read the temps from the thermocouple
  readings[readIndex] = analogRead(ThermocouplePin);
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;

  // HeaterMeter board runs at 3.3v
  // AD8495 Thermocouple is approx 5 mV/°C
  float Vout = average * (3.3 / 1023.0);
  Input = (Vout) / 0.005;

  /* TEST ONLY
    if (now - lastMessage2 > 1000) {
      Input = Input + 0.5;
      lastMessage2 = now; //update the time stamp.
    }
  */

  // Switch to conservative mode when the temp gap narrows
  double gap = abs(Setpoint - Input); //distance away from setpoint
  if (gap < gapDeg)
  { //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
    //we're far from setpoint, use aggressive tuning parameters
    myPID.SetTunings(aggKp, aggKi, aggKd);
  }

  // Compute the PID values
  myPID.Compute();

  // Starts a new PWM cycle every WindowSize milliseconds
  if (now - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }

  // Calculate the number of running minutes
  timeNowMins = millis() / 60000.0;

  // If more than maxRunTime minutes has elapsed, turn the boiler off
  if ( timeNowMins >= maxRunTime )
  {
    digitalWrite(RelayPin, LOW);
    myPID.SetMode(MANUAL);
  }
  else
  {
    // Calculate the number of milliseconds that have passed in the current PWM cycle.
    // If that is less than the Output value the relay is turned ON
    // If that is greater than (or equal to) the Output value the relay is turned OFF.
    if ( Output > (now - windowStartTime) )
      digitalWrite(RelayPin, HIGH);
    else
      digitalWrite(RelayPin, LOW);
  }

  // Output some data to serial to see what's happening
  if (now - lastMessage > serialPing)
  {
    Serial.print("Setpoint: ");
    Serial.print(Setpoint);
    Serial.print(",");
    Serial.print(" Input: ");
    Serial.print(Input);
    Serial.print(",");
    Serial.print(" Output: ");
    Serial.print(Output);
    Serial.print(",");
    Serial.print(" Avg: ");
    Serial.print(average);
    Serial.print(",");
    Serial.print(" Raw: ");
    Serial.print(raw);
    Serial.print("\n");
    lastMessage = now; //update the time stamp.
  }
}
