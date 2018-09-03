/*
  Control an espresso machine boiler using a PID controller

  Code samples used from the following:
  PID Library: https://github.com/br3ttb/Arduino-PID-Library
  PID Lab: https://www.pdx.edu/nanogroup/sites/www.pdx.edu.nanogroup/files/2013_Arduino%20PID%20Lab_0.pdf
  Smoothing: https://www.arduino.cc/en/Tutorial/Smoothing

  Hardware:
  Solid State Relay: "25A SSR-25 DA"
*/
#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSerifBold18pt7b.h>
#include <Fonts/FreeSans9pt7b.h>

// *****************************************
// * Config options that you can customize *
// *****************************************

#define ThermocouplePin 1
#define RelayPin 2

// Board voltage 3.3v or 5v
// Best to measure between GND and VCC for most accurate readings
const double brdVolts = 5.0;

// Vref for AD8495 board (in millivolts)
// Set to 0 if your reference voltage is grounded
const int Vref = 1230;

// After powering on, how many minutes until we force the boiler to power down
// Turning the machine off and on again will reset the timer
const int maxRunTime = 45;

// Turn the display off after 90 minutes
//
const int maxDisplayMins = 90;

// Define the PID setpoint
double Setpoint = 105;

// Define the PID tuning Parameters
double Kp = 3.5;
double Ki = 0.1;
double Kd = 0.0;

// PWM Window in milliseconds
const int WindowSize = 1000;


// ***********************************************************
// * There should be no need to tweak many things below here *
// ***********************************************************

// Used for max time shutdown
int timeNowMins;

// Used with FullPwrPct for initial startup
int SetpointPct;

// PID variables
double Input, Output;

// Using P_ON_M mode ( http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/ )
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_M, DIRECT);

double PWMOutput;

// Default to being ON
bool operMode = true;

unsigned long windowStartTime;

// Define the info needed for the temperature averaging
const int numReadings = 32;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

// Thermocouple variables
float Vout;
float Vtc;
float Vbits;

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

unsigned long now = 0; //This variable is used to keep track of time

// Communication setup
const long serialPing = 1000; //This determines how often we ping our loop
// Serial pingback interval in milliseconds

// placehodler for current timestamp
unsigned long lastMessage = 0; //This keeps track of when our loop last spoke to serial

// OLED Display setup
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);
#if (SSD1306_LCDHEIGHT != 32)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif
long previousOLEDMillis = 0;            // will store last time OLED was updated
const int OLEDinterval = 250;           // interval at which to write new data to the OLED

// Set to true to enable serial port output
bool SerialOut = false;

void setup()
{
  if ( SerialOut == true )
  {
    //Setup Serial
    Serial.begin(38400); //Start a serial session
    lastMessage = now; // timestamp
  }

  // Set the Relay to output mode and ensure the relay if off
  pinMode(RelayPin, OUTPUT);
  digitalWrite(RelayPin, LOW);

  // PID settings
  windowStartTime = now;
  myPID.SetOutputLimits(0, 100);
  //myPID.SetOutputLimits(0, 500);
  myPID.SetSampleTime(50);

  // Do initial calc of brdVolts / 1023
  Vbits = brdVolts / 1023;

  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
  {
    readings[thisReading] = 0;
  }

  // Setup the OLED display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.display();

} // end of setup()

void readTemps(void)
{
  // subtract the last reading:
  total = total - readings[readIndex];
  // Read the temps from the thermocouple
  readings[readIndex] = analogRead(ThermocouplePin);
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings)
  {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;

  // This should match the output voltage on the Out pin of the AD8945
  Vout = average * Vbits;

  // Based on Analog Devices AN-1087
  // Convert the AD8495 output back to millivolts so we can perform the NIST calc
  Vtc = ((Vout * 1000) - Vref - 1.25) / 122.4;

  // Use the NIST corrected temperature readings for a K-type thermocouple in the 0-500°C range
  // https://srdata.nist.gov/its90/type_k/kcoefficients_inverse.html
  Input = c0
          + c1 * Vtc
          + c2 * pow(Vtc, 2)
          + c3 * pow(Vtc, 3)
          + c4 * pow(Vtc, 4)
          + c5 * pow(Vtc, 5)
          + c6 * pow(Vtc, 6)
          + c7 * pow(Vtc, 7)
          + c8 * pow(Vtc, 8)
          + c9 * pow(Vtc, 9);
}

void relayControl(void)
{
  // Calculate the number of running minutes
  timeNowMins = now / 60000.0;

  // If more than maxRunTime minutes has elapsed, turn the boiler off
  // and dont perform any other PID functions
  if ( timeNowMins >= maxRunTime )
  {
    digitalWrite(RelayPin, LOW);
    myPID.SetTunings(0, 0, 0);
    myPID.SetMode(MANUAL);
    operMode = false;
  } else {
    // Compute the PID values
    myPID.SetMode(AUTOMATIC);
    myPID.Compute();
  }

  PWMOutput = Output * (WindowSize / 100.00);
  // Starts a new PWM cycle every WindowSize milliseconds
  if (now - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }

  // Calculate the number of milliseconds that have passed in the current PWM cycle.
  // If that is less than the Output value the relay is turned ON
  // If that is greater than (or equal to) the Output value the relay is turned OFF.
  if (PWMOutput > (now - windowStartTime))
  {
    digitalWrite(RelayPin, HIGH);
  } else {
    digitalWrite(RelayPin, LOW);
  }
}

void displayStats(void)
{
  unsigned long currentOLEDMillis = now;

  if (currentOLEDMillis - previousOLEDMillis > OLEDinterval) {
    // save the last time you wrote to the OLED display
    previousOLEDMillis = currentOLEDMillis;

    // have to wipe the buffer before writing anything new
    display.clearDisplay();

    if ( operMode == true )
    {
      display.setFont(&FreeSans9pt7b);
      display.setCursor(0, 22);
      // Dont add a decimal place for 100 or 0
      if ( (Output >= 100) || (Output == 0) )
      {
        display.print(Output, 0);
      }
      else
      {
        display.print(Output, 1);
      }
      display.setFont(&FreeSerifBold18pt7b);
      display.setCursor(42, 28);
      display.print(Input, 1);
    }
    else if ( operMode == false )
    {
      // After maxDisplayMins minutes turn off the display
      if ( timeNowMins >= maxDisplayMins )
      {
        display.clearDisplay();
      }
      else
      {
        display.setFont(&FreeSerifBold18pt7b);
        display.setCursor(28, 28);
        display.print("OFF");
      }
    }
    // Do the needful!
    display.display();
  }
}

void displaySerial(void)
{
  // Output some data to serial to see what's happening
  if (now - lastMessage > serialPing)
  {
    if ( operMode == true )
    {
      Serial.print("Time: ");
      Serial.print(now / 1000);
      Serial.print(", ");
      Serial.print("SP: ");
      Serial.print(Setpoint, 0);
      Serial.print(", ");
      Serial.print("Input: ");
      Serial.print(Input, 1);
      Serial.print(", ");
      Serial.print("Output: ");
      Serial.print(Output, 1);
      Serial.print(", ");
      Serial.print("Avg: ");
      Serial.print(average);
      Serial.print(", ");
      Serial.print("Vout: ");
      Serial.print(Vout);
      Serial.print("\n");
    } else {
      Serial.print("Input: ");
      Serial.print(Input, 1);
      Serial.print(", ");
      Serial.print("Mode: off");
      Serial.print("\n");
    }
    lastMessage = now; //update the time stamp.
  }
}

void loop()
{
  //Keep track of time
  now = millis();

  readTemps();
  relayControl();
  displayStats();
  if ( SerialOut == true )
  {
    displaySerial();
  }

} // End of loop()
