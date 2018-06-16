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

// HM Pins
// TC = ADC 5
// Blower/Relay = Digital Pin 3
#define ThermocouplePin 5
#define RelayPin 3

// Board voltage 3.3v or 5v Arduino
// Best to measure between GND and VCC for most accurate
const double brdVolts = 3.30;

// After powering on, how many minutes until we force the boiler to power down
// Turning the machine off and on again will reset the timer
const int maxRunTime = 45;
int timeNowMins;

// Anything below this % gets full power
const int FullPwrPct = 85;
int SetpointPct;

// Define the setpoint and initial parameters
const double Setpoint = 105;
double Input, Output;

//Define the PID tuning Parameters
const double Kp = 8.0;
const double Ki = 0.15  ;
const double Kd = 1.0;

//Needed to display current values on serial output
double currKp;
double currKi;
double currKd;

// Using P_ON_M mode ( http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/ )
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_M, DIRECT);
const int WindowSize = 500;
unsigned long windowStartTime;

//Define the info needed for the temperature averaging
const int numReadings = 32;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

// Thermocouple variables
float Vtc;
//double RawInput;

// Corrected temperature readings for a K-type thermocouple
// Coefficient values for 0C - 500C / 0mV - 20.644mV
const double c0 = 0.000000E+00;
const double c1 = 2.508355E+01;
const double c2 = 7.860106E-02;
const double c3 = -2.503131E-01;
const double c4 = 8.315270E-02;
const double c5 = -1.228034E-02;
const double c6 = 9.804036E-04;
const double c7 = -4.413030E-05;
const double c8 = 1.057734E-06;
const double c9 = -1.052755E-08;

// Communication setup
const long serialPing = 500; //This determines how often we ping our loop
// Serial pingback interval in milliseconds
unsigned long now = 0; //This variable is used to keep track of time
// placehodler for current timestamp
unsigned long lastMessage = 0; //This keeps track of when our loop last spoke to serial

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

  float Vout = average * (brdVolts / 1023.0);

  // AD8495 Thermocouple is approx 5 mV/°C
  // Make a linear temp calc so we can compare is against the calculated temp
  // RawInput = (Vout) / 0.005;

  // To accommodate the nonlinear behavior of the thermocouple, each amplifier has a different gain
  // so that the 5 mV/°C is accurately maintained for a given temperature measurement range.
  // The AD8495 and AD8497 (K type) have an instrumentation amplifier with a gain of 122.4.
  Vtc = ((Vout * 1000) - 1.25) / 122.4;

  // Use the corrected temperature readings for a K-type thermocouple in the 0-500°C range
  Input = c0 +
          c1 * Vtc +
          c2 * pow(Vtc, 2) +
          c3 * pow(Vtc, 3) +
          c4 * pow(Vtc, 4) +
          c5 * pow(Vtc, 5) +
          c6 * pow(Vtc, 6) +
          c7 * pow(Vtc, 7) +
          c8 * pow(Vtc, 8) +
          c9 * pow(Vtc, 9);

  // Starts a new PWM cycle every WindowSize milliseconds
  if (now - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }

  // Calculate the number of running minutes
  timeNowMins = now / 60000.0;
  
  // If more than maxRunTime minutes has elapsed, turn the boiler off
  // and dont perform any other PID functions
  if ( timeNowMins >= maxRunTime )
  {
    digitalWrite(RelayPin, LOW);
    myPID.SetTunings(0, 0, 0);
    myPID.SetMode(MANUAL);
  }
  else
  {
    SetpointPct = Input / Setpoint * 100;
    // Don't control the SSR via PID until we're above FullPwrPct (80-90%)
    if (SetpointPct < FullPwrPct )
    {
      myPID.SetMode(MANUAL);
      digitalWrite(RelayPin, HIGH);
    }
    else
    {
      // Compute the PID values
      myPID.Compute();
      // Calculate the number of milliseconds that have passed in the current PWM cycle.
      // If that is less than the Output value the relay is turned ON
      // If that is greater than (or equal to) the Output value the relay is turned OFF.
      // To reduce relay "flickering" wait till output is at least 50 before turning on
      if ( (Output > (now - windowStartTime)) && ( Output > 50 ) )
        digitalWrite(RelayPin, HIGH);
      else
        digitalWrite(RelayPin, LOW);
    }
  }

  // Output some data to serial to see what's happening
  if (now - lastMessage > serialPing)
  {
    currKp = myPID.GetKp();
    currKi = myPID.GetKi();
    currKd = myPID.GetKd();

    Serial.print("Setpoint: ");
    Serial.print(Setpoint);
    Serial.print(",");
    Serial.print(" Input: ");
    Serial.print(Input);
    Serial.print(",");
    Serial.print(" Output: ");
    Serial.print(Output);
    Serial.print(",");
    Serial.print(" Kp: ");
    Serial.print(currKp);
    Serial.print(",");
    Serial.print(" Ki: ");
    Serial.print(currKi);
    Serial.print(",");
    Serial.print(" Kd: ");
    Serial.print(currKd);
    Serial.print("\n");
    lastMessage = now; //update the time stamp.
  }
}