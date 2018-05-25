/*
  Control an espresso machine boiler using a PID controller

  Code samples used from the following:
  PID Library: https://github.com/br3ttb/Arduino-PID-Library
  PID Lab: https://www.pdx.edu/nanogroup/sites/www.pdx.edu.nanogroup/files/2013_Arduino%20PID%20Lab_0.pdf

  Hardware:
  HeaterMeter v4.2 PCB: https://github.com/CapnBry/HeaterMeter/wiki/HeaterMeter-4.2-Hardware
  Solid State Relay: "25A SSR-25 DA"
*/

#include <PID_v1.h>

// HM Pins
// TC = ADC5 / AnalogInput 5
// Blower/Relay = Digital Pin 3

#define ThermocouplePin 5
#define RelayPin 3

// Tuning parameters
double Kp = 10; //Initial Proportional Gain
double Ki = 0.5; //Initial Integral Gain
double Kd = 0; //Initial Differential Gain

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive Tuning Parameters
double aggKp = 50;
double aggKi = 0.03;
double aggKd = 1;

//Define the conservative Tuning Parameters
double consKp = 3;
double consKi = 0.01;
double consKd = 3;

//Define the gap degrees to switch between aggressive and conservative mode
int gapdeg = 15;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
int WindowSize = 5000;
unsigned long windowStartTime;

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

  // Set the Relay to output mode and ensure the relay off
  pinMode(RelayPin, OUTPUT);
  digitalWrite(RelayPin, LOW);

  // PID settings
  windowStartTime = millis();
  Setpoint = 105;
  myPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  //Keep track of time
  now = millis();

  // Read the temps from the thermocouple
  int raw = analogRead(ThermocouplePin);
  float Vout = raw * (3.3 / 1023.0);
  Input = (Vout) / 0.005;
  // Input = (Vout - 1.25) / 0.005;

  /* TEST ONLY
    if (now - lastMessage2 > 1000) { //If it has been long enough give us some info on serial
      Input = Input + 0.5;
      lastMessage2 = now; //update the time stamp.
    }
  */

  // Switch to conservative mode when the temp gap narrows
  double gap = abs(Setpoint - Input); //distance away from setpoint
  if (gap < gapdeg)
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

  // Calculate the number of milliseconds that have passed in the current PWM cycle.
  // If that is less than the Output value the relay is turned ON.
  // If that is greater than (or equal to) the Output value the relay is turned OFF.
  if (Output > (now - windowStartTime))
    digitalWrite(RelayPin, HIGH);
  else
    digitalWrite(RelayPin, LOW);

  // Output some data to serial to see what's happening
  if (now - lastMessage > serialPing) { //If it has been long enough give us some info on serial
    Serial.print("Setpoint = ");
    Serial.print(Setpoint);
    Serial.print(" Input = ");
    Serial.print(Input);
    Serial.print(" Output = ");
    Serial.print(Output);
    Serial.print(" Raw = ");
    Serial.print(raw);
    Serial.print("\n");
    lastMessage = now; //update the time stamp.
  }
}
