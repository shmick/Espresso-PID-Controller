/*
Control an espresso machine boiler using a PID controller

Code samples used from the following:
PID Library: https://github.com/br3ttb/Arduino-PID-Library
PID Lab: https://www.pdx.edu/nanogroup/sites/www.pdx.edu.nanogroup/files/2013_Arduino%20PID%20Lab_0.pdf
Thermocouple polynomial coefficients: https://ez.analog.com/thread/51921

Hardware: 
HeaterMeter v4.2 PCB: https://github.com/CapnBry/HeaterMeter/wiki/HeaterMeter-4.2-Hardware
Solid State Relay: "25A SSR-25 DA"
*/

#include <PID_v1.h>

// HM Pins
// TC = ADC5 / AnalogInput 5
// Blower/Relay = Digital Pin 3

#define PIN_INPUT 5
#define RELAY_PIN 3

// Tuning parameters
double Kp = 2; //Initial Proportional Gain
double Ki = 0.1; //Initial Integral Gain
double Kd = 0; //Initial Differential Gain

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int WindowSize = 2000;
unsigned long windowStartTime;

// Communication setup
const long serialPing = 500; //This determines how often we ping our loop
// Serial pingback interval in milliseconds
unsigned long now = 0; //This variable is used to keep track of time
// placehodler for current timestamp
unsigned long lastMessage = 0; //This keeps track of when our loop last spoke to serial
// last message timestamp.

void setup()
{
  windowStartTime = millis();

  //initialize the variables we're linked to
  Setpoint = 90;

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

}

void loop()
{

  // TC code

  float Input;

  float emf;

  int raw = analogRead(PIN_INPUT);

  float Vout = raw * (5.0 / 1023.0);

  emf = ((Vout * 1000) - 1.25) / 122.4;

  Input = (25.08355 * emf) + (0.07860106 * pow(emf, 2)) - (0.2503131 * pow(emf, 3)) + (0.08315270 * pow(emf, 4)) - (0.01228034 * pow(emf, 5)) + (0.0009804036 * pow(emf, 6)) - (0.0000441303 * pow(emf, 7)) + (0.000001057734 * pow(emf, 8)) - (0.00000001052755 * pow(emf, 9));
  // TC code end

  // PID Routine
  myPID.Compute();

  /************************************************
     turn the output pin on/off based on pid output
   ************************************************/
  if (millis() - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if (Output < millis() - windowStartTime) digitalWrite(RELAY_PIN, HIGH);
  else digitalWrite(RELAY_PIN, LOW);

  // Serial output stuff here
  now = millis(); //Keep track of time
  if (now - lastMessage > serialPing) { //If it has been long enough give us some info on serial
    // this should execute less frequently
    // send a message back to the mother ship
    Serial.print("Setpoint = ");
    Serial.print(Setpoint);
    Serial.print(" Input = ");
    Serial.print(Input);
    Serial.print(" Output = ");
    Serial.print(Output);
    Serial.print("\n");
  }
  lastMessage = now; //update the time stamp. 
}
