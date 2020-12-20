/*
  Control an espresso machine boiler using a PID controller

  Code samples used from the following:
  PID Library: https://github.com/br3ttb/Arduino-PID-Library
  PID Lab: https://www.pdx.edu/nanogroup/sites/www.pdx.edu.nanogroup/files/2013_Arduino%20PID%20Lab_0.pdf
  Smoothing: https://www.arduino.cc/en/Tutorial/Smoothing
  Arduino <> Wemos D1 Mini pins: https://github.com/esp8266/Arduino/blob/master/variants/d1_mini/pins_arduino.h
  ESP8266 file uploading https://tttapa.github.io/ESP8266/Chap12%20-%20Uploading%20to%20Server.html
  Hardware:
  Wemos D1 Mini ( https://wiki.wemos.cc/products:d1:d1_mini )
  128 x 64 OLED Display
  AD8495 Thermocouple Amplifier
  Solid State Relay: "25A SSR-25 DA"
*/
#include <PID_v1.h>

// Needed for the OLED display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSerifBold18pt7b.h>
#include <Fonts/FreeSans9pt7b.h>

// Needed for ESP8266
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <FS.h>   //Include File System Headers
//const char* htmlfile = "/index.html";

// Needed for pushing new sketches over WiFi
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

//#include <ArduinoJson.h>

// *****************************************
// * Config options that you can customize *
// *****************************************

// Replace with your own WiFi network credentials
const char* ssid     = "shmick";
const char* password = "SixNineTwoSix";

#define ThermocouplePin 0 // Ardunio 0 = Wemos D1 Mini Pin A0
#define RelayPin 4 // Ardunio D4 = Wemos D1 Mini Pin D2

// Board voltage 3.3v or 5v for Arduino.
// Set to 3.2 for ESP8266 units due to the voltage divider on ADC0
// Best to measure between GND and VCC for most accurate readings
const double brdVolts = 3.2;

// Vref for AD8495 board (in millivolts)
// Set to 0 if your reference voltage is grounded
// Set to 1200 (1.2 volts) if using the Adafruit board
const int Vref = 0;

// After powering on, how many minutes until we force the boiler to power down
// Turning the machine off and on again will reset the timer
const int maxRunTime = 180;

// Turn the display off after 200 minutes
const int maxDisplayMins = 200;

// Define the PID setpoint
//double Setpoint = 105;
double Setpoint = 105.11;
//double Setpoint = 24;

// Define the PID tuning Parameters
//double Kp = 3.5; working ok on 2018-09-14
double Kp = 4.5;
double Ki = 0.125;
double Kd = 0.0;

// PWM Window in milliseconds
const int WindowSize = 5000;

// ESP8266 WiFi Details
//WiFiServer server(80);
ESP8266WebServer server(80);
// Variable to store the HTTP request
String header;

// Default to being ON
bool operMode = true;

// ***********************************************************
// * There should be no need to tweak many things below here *
// ***********************************************************

// PID variables
// Using P_ON_M mode ( http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/ )
double Input, Output;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_M, DIRECT);
double PWMOutput;
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
const float Vbits = brdVolts / 1023;;

// Corrected temperature readings for a K-type thermocouple
// https://srdata.nist.gov/its90/type_k/kcoefficients_inverse.html
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


// All timers reference the value of now
unsigned long now = 0; //This variable is used to keep track of time

// OLED display timer
const int OLEDinterval = 200;           // interval at which to write new data to the OLED
unsigned long previousOLEDMillis = now;            // will store last time OLED was updated

// Serial output timer
const int serialPing = 500; //This determines how often we ping our loop
unsigned long lastMessage = now; //This keeps track of when our loop last spoke to serial

// Web Server client timer
const int clientWait = 50;
unsigned long lastClient = now;

int runTimeMins;
long runTimeSecs;
unsigned long runTimeStart = now;

// OLED Display setup
#define OLED_SDA 14 // Arduino 14 = ESP8266 Pin 5
#define OLED_SCL 12 // Arduino 12 = ESP8266 Pin 6
#define OLED_RESET 16
#define OLED_I2C 0x3C
Adafruit_SSD1306 display(OLED_RESET);
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
// You will need to modify the Adafruit_SSD1306.h file
// Step 1: uncomment this line: #define SSD1306_128_64
// Step 2: add a comment to this line: #define SSD1306_128_32
#endif



void setup()
{
  Serial.begin(115200); //Start a serial session
  lastMessage = now; // timestamp

  // Set the Relay to output mode and ensure the relay is off
  pinMode(RelayPin, OUTPUT);
  digitalWrite(RelayPin, LOW);

  // PID settings
  windowStartTime = now;

  myPID.SetOutputLimits(0, 100);

  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
  {
    readings[thisReading] = 0;
  }

  // Enable I2C communication
  Wire.begin(OLED_SDA, OLED_SCL);

  // Setup the OLED display
  display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C);  // initialize with the I2C addr 0x3C (for the 128x32)
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.display();

  // Connect to Wi-Fi network with SSID and password
  WiFi.begin(ssid, password);
  int count = 1;
  while ((count <= 60) && (WiFi.status() != WL_CONNECTED)) {
    delay(500);
    count = count + 1;
  }

  SPIFFS.begin();

  server.on("/upload", HTTP_GET, []() {                 // if the client requests the upload page
    if (!handleFileRead("/upload.html"))                // send it if it exists
      server.send(404, "text/plain", "404: Not Found"); // otherwise, respond with a 404 (Not Found) error
  });

  server.on("/upload", HTTP_POST,                       // if the client posts to the upload page
  []() {
    server.send(200);
  },                          // Send status 200 (OK) to tell the client we are ready to receive
  handleFileUpload                                    // Receive and save the file
           );

  server.onNotFound([]() {                              // If the client requests any URI
    if (!handleFileRead(server.uri()))                  // send it if it exists
      server.send(404, "text/plain", "404: Not Found"); // otherwise, respond with a 404 (Not Found) error
  });

  server.on("/stats", handleStats); //Reads ADC function is called from out index.html
  server.on("/json", handleJSON);
  server.on("/set", HTTP_POST, handleSetvals);
  server.begin();

  ArduinoOTA.setHostname("Wemos D1 Mni - espresso");  // For OTA - change name here to help identify device.
  ArduinoOTA.begin();  // For OTA

  MDNS.begin("espresso");

} // end of setup()

void keepTime(void)
{
  //Keep track of time
  now = millis();
  runTimeSecs = (now - runTimeStart) / 1000;
  runTimeMins = (now - runTimeStart) / 60000;
}


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

  // If more than maxRunTime minutes has elapsed, turn the boiler off
  // and dont perform any other PID functions
  if ( (runTimeMins >= maxRunTime) || (operMode == false) )
  {
    digitalWrite(RelayPin, LOW);
    myPID.SetMode(MANUAL);
    Output = 0;
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
    digitalWrite(RelayPin, HIGH);  // Wemos BUILTIN_LED LOW = ON
  } else {
    digitalWrite(RelayPin, LOW); // Wemos BUILTIN_LED HIGH = OFF
  }
}


void displayOLED(void)
{
  unsigned long currentOLEDMillis = now;

  if (currentOLEDMillis - previousOLEDMillis > OLEDinterval) {
    // save the last time you wrote to the OLED display
    previousOLEDMillis = currentOLEDMillis;

    // have to wipe the buffer before writing anything new
    display.clearDisplay();

    if ( operMode == true )
    {
      // TOP HALF = Temp + Input Temp
      display.setFont(&FreeSans9pt7b);
      display.setCursor(0, 22);
      display.print("Temp");

      display.setFont(&FreeSerifBold18pt7b);
      display.setCursor(48, 26);
      display.print(Input, 1);

      // BOTTOM HALF = Output + Output Percent
      display.setFont(&FreeSans9pt7b);
      display.setCursor(0, 56);
      display.print("Output");

      display.setFont(&FreeSerifBold18pt7b);
      display.setCursor(60, 60);
      // Dont add a decimal place for 100 or 0
      if ( (Output >= 100) || (Output == 0) )
      {
        display.print(Output, 0);
      }
      else if ( Output < 10 )
      {
        display.print(Output, 2);
      }
      else
      {
        display.print(Output, 1);
      }
    }
    else if ( operMode == false )
    {
      // After maxDisplayMins minutes turn off the display
      if ( runTimeMins >= maxDisplayMins )
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
      Serial.println(runTimeSecs);
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


void handleStats() {
  String message = "<head> <meta http-equiv=\"refresh\" content=\"2\"> </head>\n";
  message +=  "<h1>\n";
  message += "Time: " + String(runTimeSecs) + "<br>\n";
  message += "Setpoint: " + String(Setpoint) + "<br>\n";
  message +=  "PID Input:  " + String(Input) + "<br>\n";
  message +=  "PID Output: " + String(Output) + "<br>\n";
  message +=  "Avg: " + String(average) + "<br>\n";
  message +=  "Vout: " + String(Vout);
  server.send(200, "text/html", message);
}

/*
  void handleJSON() {
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["Input"] = Input;
  root["Output"] = Output;
  root["ADC"] = average;
  root["Setpoint"] = Setpoint;
  root["Uptime"] = runTimeSecs;
  root["Vout"] = Vout;
  String json;
  root.prettyPrintTo(json);
  server.send(200, "application/json", json);
  }
*/

void handleJSON() {
  // Quick and dirty JSON output without the use of ArduinoJson
  String message = "{ ";
  message +=  "\"Time\":" + String(runTimeSecs) + ", ";
  message += "\"Setpoint\":" + String(Setpoint) + ", ";
  message +=  "\"Input\":" + String(Input) + ", ";
  message +=  "\"Output\":" + String(Output) + ", ";
  message +=  "\"ADC\":" + String(average) + ", ";
  message +=  "\"Vout\":" + String(Vout); // No comma on Last entry
  message += " }";
  server.send(200, "application/json", message);
}


void handleSetvals() {
  String message;

  String opmode_val = server.arg("opmode");
  String sp_val = server.arg("sp");

  if ( opmode_val == "off" ) {
    operMode = false;
    message += "opermode: " + opmode_val;
    message += "\n";
  } else if ( opmode_val == "on" ) {
    operMode = true;
    runTimeStart = now;
    message += "opermode: " + opmode_val;
    message += "\n";
  }

  if ( sp_val != NULL ) {
    double sp_int = sp_val.toFloat();
    if ( sp_int < 105 && sp_int > 0.1 ) {
      Setpoint = sp_int;
      message += "Setpoint: " + sp_val;
      message += "\n";
    } else {
      message += "sp: " + String(sp_val) + " is invalid\n";
    }
  }

  server.send(200, "text/plain", message);
}


String getContentType(String filename) { // convert the file extension to the MIME type
  if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  else if (filename.endsWith(".json")) return "application/json";
  return "text/plain";
}


bool handleFileRead(String path) { // send the right file to the client (if it exists)
  //Serial.println("handleFileRead: " + path);
  if (path.endsWith("/")) path += "index.html";         // If a folder is requested, send the index file
  String contentType = getContentType(path);            // Get the MIME type
  if (SPIFFS.exists(path)) {                            // If the file exists
    File file = SPIFFS.open(path, "r");                 // Open it
    size_t sent = server.streamFile(file, contentType); // And send it to the client
    file.close();                                       // Then close the file again
    return true;
  }
  Serial.println("\tFile Not Found");
  return false;                                         // If the file doesn't exist, return false
}


void handleFileUpload() { // upload a new file to the SPIFFS
  File fsUploadFile;              // a File object to temporarily store the received file
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    String filename = upload.filename;
    if (!filename.startsWith("/")) filename = "/" + filename;
    //Serial.print("handleFileUpload Name: "); Serial.println(filename);
    fsUploadFile = SPIFFS.open(filename, "w");            // Open the file for writing in SPIFFS (create if it doesn't exist)
    filename = String();
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize); // Write the received bytes to the file
  } else if (upload.status == UPLOAD_FILE_END) {
    if (fsUploadFile) {                                   // If the file was successfully created
      fsUploadFile.close();                               // Close the file again
      //Serial.print("handleFileUpload Size: "); Serial.println(upload.totalSize);
      server.sendHeader("Location", "/success.html");     // Redirect the client to the success page
      server.send(303);
    } else {
      server.send(500, "text/plain", "500: couldn't create file");
    }
  }
}


void loop()
{
  keepTime();
  readTemps();
  //delay(1);
  relayControl();
  //delay(1);
  displayOLED();
  delay(1);
  displaySerial();
  delay(1);
  server.handleClient();
  delay(1);
  ArduinoOTA.handle();
  //delay(1);
} // End of loop()
