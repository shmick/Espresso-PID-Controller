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
  128 x 64 OLED Display using Adafruit_SSD1306 library
  AD8495 Thermocouple Amplifier ( https://www.adafruit.com/product/1778 )
  ADS1115 16-Bit ADC using library from https://github.com/baruch/ADS1115
  Solid State Relay ( Crydom TD1225 )
*/

// Set to 1 if using the Adafruit TC board or similar boards with 1.25v reference
#define ADA_TC 1

#if ADA_TC == 1
const int ADSGAIN = 2;
const double Vref = 1.2362;
#else
const int ADSGAIN = 16;
const int Vref = 0;
#endif

// set to true for testing the code on the breadboard setup. This will set the hostname to xespresso
const bool breadboard = false;

// set to 0 if not using the OLED display
#define OLED_DISPLAY 1

#include <PID_v1.h>

// Needed for the I2C ports
#include <Wire.h>

#if OLED_DISPLAY == 1
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSerifBold18pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#endif

// Needed for ESP8266
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <FS.h>   //Include File System Headers

// Needed for pushing new sketches over WiFi
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// Needed for IotWebConf https://github.com/prampec/IotWebConf
#include <DNSServer.h>
#include <IotWebConf.h>

// Needed for ADS1115 ADC
#include <ADS1115.h>

// *****************************************
// * Config options that you can customize *
// *****************************************

//#define ThermocouplePin 0 // ** Not needed with ADS1115 board
#define RelayPin 4 // Ardunio D4 = Wemos D1 Mini Pin D2

// After powering on, how many minutes until we force the boiler to power down
// Turning the machine off and on again will reset the timer
const int maxRunTime = 180;

// Turn the display off after 200 minutes
const int maxDisplayMins = 200;

// Default to being ON
bool operMode = true;

// Define the PID setpoint
double Setpoint = 105;

// Define the PID tuning Parameters
//double Kp = 3.5; working ok on 2018-09-14
double Kp = 4.5;
double Ki = 0.125;
double Kd = 0.2;

// PWM Window in milliseconds
const int WindowSize = 5000;

// -- Initial name of the Thing. Used e.g. as SSID of the own Access Point.
const char thingName[] = "espresso";
// -- Initial password to connect to the Thing, when it creates an own Access Point.
const char wifiInitialApPassword[] = "espresso";



// ***********************************************************
// ***********************************************************
// * There should be no need to tweak many things below here *
// ***********************************************************
// ***********************************************************

// PID variables
// Using P_ON_M mode ( http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/ )
double Input, Output;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_M, DIRECT);
double PWMOutput;
uint32_t windowStartTime;

// Define the info needed for the temperature averaging
const int numReadings = 8;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

// Thermocouple variables
float Vout; // The voltage coming from the out pin on the TC amp
float Vtc;
// const float Vbits = brdVolts / 1023; // ** Not needed with ADS1115 board

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

DNSServer dnsServer;
WebServer server(80);
IotWebConf iotWebConf(thingName, &dnsServer, &server, wifiInitialApPassword);

// Variable to store the HTTP request
String header;

// All timers reference the value of now
uint32_t now = 0; //This variable is used to keep track of time

// OLED display timer
const int OLEDinterval = 250;           // interval at which to write new data to the OLED
uint32_t previousOLEDMillis = now;            // will store last time OLED was updated

// Serial output timer
const int serialPing = 500; //This determines how often we ping our loop
uint32_t lastMessage = now; //This keeps track of when our loop last spoke to serial

int runTimeMins;
long runTimeSecs;
uint32_t runTimeStart = now;

// Temp read interval
const int TempInterval = 5;
uint32_t currentTempMillis;
uint32_t previousTempMillis = now;

// Server tasks interval
const int serverInterval = 50;
uint32_t currentServerMillis;
uint32_t previousServerMillis = now;

// Setup I2C pins
#define ESP_SDA 14 // Arduino 14 = ESP8266 Pin 5
#define ESP_SCL 12 // Arduino 12 = ESP8266 Pin 6

#if OLED_DISPLAY == 1
// OLED Display setup
#define OLED_RESET 16
#define OLED_I2C 0x3C
Adafruit_SSD1306 display(OLED_RESET);
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h");
// You will need to modify the Adafruit_SSD1306.h file
// Step 1: uncomment this line: #define SSD1306_128_64
// Step 2: add a comment to this line: #define SSD1306_128_32
#endif
#endif

// ADS1115 ADC
int adcval;
ADS1115 adc;
float ADS_PGA;

uint32_t prevLoopMillis;
uint32_t numLoops = 0;
uint32_t currLoops = 0;


void keepTime(void)
{
  //Keep track of time
  now = millis();
  runTimeSecs = (now - runTimeStart) / 1000;
  runTimeMins = (now - runTimeStart) / 60000;
}


int readADC()
{
  static int read_triggered = 0;
  if (!read_triggered) {
    if (adc.trigger_sample() == 0)
      read_triggered = 1;
  } else {
    if (!adc.is_sample_in_progress()) {
      adcval = adc.read_sample();
      read_triggered = 0;
    }
  }
  return adcval;
}


void readTemps(void)
{
  currentTempMillis = now;
  if (currentTempMillis - previousTempMillis > TempInterval) {

    // subtract the last reading:
    total = total - readings[readIndex];
    // Read the temps from the thermocouple
    readings[readIndex] = readADC();
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
    Vout = average * ADS_PGA / 1000;

    // Based on Analog Devices AN-1087
    // Convert the AD8495 output back to millivolts so we can perform the NIST calc
    Vtc = ((Vout * 1000) - (Vref * 1000) - 1.25) / 122.4;

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

    previousTempMillis = currentTempMillis;
  }
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
  // If that is less than the Output value, the relay is turned ON
  // If that is greater than (or equal to) the Output value, the relay is turned OFF.
  if (PWMOutput > (now - windowStartTime))
  {
    digitalWrite(RelayPin, HIGH);  // Wemos BUILTIN_LED LOW = ON
  } else {
    digitalWrite(RelayPin, LOW); // Wemos BUILTIN_LED HIGH = OFF
  }
}


// Track how many loops per second are executed.
void trackloop() {
  if ( now - prevLoopMillis >= 1000) {
    currLoops = numLoops;
    numLoops = 0;
    prevLoopMillis = now;
  }
  numLoops++;
}

#if OLED_DISPLAY == 1
void displayOLED(void)
{
  uint32_t currentOLEDMillis = now;

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
      if ( Input >= 100 )
        display.print(Input, 1);
      else
        display.print(Input);

      // BOTTOM HALF = Output + Output Percent
      display.setFont(&FreeSans9pt7b);
      display.setCursor(0, 56);
      display.print("Output");

      display.setFont(&FreeSerifBold18pt7b);
      display.setCursor(60, 60);
      // Dont add a decimal place for 100 or 0
      if ( (Output >= 100.0) || (Output == 0.0) )
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
#endif

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

// ESP8266WebServer handler
void handleRoot()
{
  // -- Let IotWebConf test and handle captive portal requests.
  if (iotWebConf.handleCaptivePortal())
  {
    // -- Captive portal request were already served.
    return;
  }
  String s = "<!DOCTYPE html><html lang=\"en\"><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/>";
  s += "<title>IotWebConf 02 Status and Reset</title></head><body>Hello world!";
  s += "Go to <a href='config'>configure page</a> to change settings.";
  s += "</body></html>\n";

  server.send(200, "text/html", s);
}


// ESP8266WebServer handler
void handleStats() {
  String message = "<head> <meta http-equiv=\"refresh\" content=\"2\"> </head>\n";
  message +=  "<h1>\n";
  message += "Time: " + String(runTimeSecs) + "<br>\n";
  message += "Setpoint: " + String(Setpoint) + "<br>\n";
  message +=  "PID Input:  " + String(Input) + "<br>\n";
  message +=  "PID Output: " + String(Output) + "<br>\n";
  message +=  "Avg: " + String(average) + "<br>\n";
  message +=  "Vout: " + String(Vout) + "<br>\n";
  message +=  "Loops: " + String(currLoops) + "<br>\n";
  message +=  "OperMode: " + String(operMode);
  server.send(200, "text/html", message);
}


// ESP8266WebServer handler
void handleJSON() {
  // Quick and dirty JSON output without the use of ArduinoJson
  String comma = ", ";
  String message = "{ ";
  message +=  "\"Uptime\":" + String(now / 1000) + comma;
  message +=  "\"Runtime\":" + String(runTimeSecs) + comma;
  message += "\"Setpoint\":" + String(Setpoint) + comma;
  message +=  "\"Input\":" + String(Input) + comma;
  message +=  "\"Output\":" + String(Output) + comma;
  message +=  "\"ADC\":" + String(average) + comma;
  message +=  "\"Vout\":" + String(Vout) + comma;
  message +=  "\"Mode\":" + String(operMode) + comma;
  message +=  "\"Loops\":" + String(currLoops);
  message += " }";
  server.send(200, "application/json", message);
}


// ESP8266WebServer handler
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
    if ( sp_int <= 105.11 || sp_int > 0.1 ) {
      Setpoint = sp_int;
      message += "Setpoint: " + sp_val;
      message += "\n";
    } else {
      message += "sp: " + String(sp_val) + " is invalid\n";
    }
  }

  server.send(200, "text/plain", message);
}


// ESP8266WebServer handler
String getContentType(String filename) { // convert the file extension to the MIME type
  if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  else if (filename.endsWith(".json")) return "application/json";
  return "text/plain";
}


// ESP8266WebServer handler
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


// ESP8266WebServer handler
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


void esp8266Tasks() {
  currentServerMillis = now;
  if (currentServerMillis - previousServerMillis > serverInterval) {
    iotWebConf.doLoop();
    server.handleClient();
    ArduinoOTA.handle();
    previousServerMillis = currentServerMillis;
  }
}


void configADC(void)
{
  // Setup ADS1115
  adc.begin();
  adc.set_data_rate(ADS1115_DATA_RATE_860_SPS);
  adc.set_mode(ADS1115_MODE_SINGLE_SHOT);
  adc.set_mux(ADS1115_MUX_GND_AIN0);

  //  TWO_THIRDS // 2/3x gain +/- 6.144V  1 bit = 0.1875mV
  //  ONE        // 1x gain   +/- 4.096V  1 bit = 0.125mV
  //  TWO        // 2x gain   +/- 2.048V  1 bit = 0.0625mV
  //  FOUR       // 4x gain   +/- 1.024V  1 bit = 0.03125mV
  //  EIGHT      // 8x gain   +/- 0.512V  1 bit = 0.015625mV
  //  SIXTEEN    // 16x gain  +/- 0.256V  1 bit = 0.0078125mV

  if ( ADSGAIN == 23 ) {
    adc.set_pga(ADS1115_PGA_TWO_THIRDS);
    ADS_PGA = 0.1875;
  }
  else if ( ADSGAIN == 1 ) {
    adc.set_pga(ADS1115_PGA_ONE);
    ADS_PGA = 0.125;
  }
  else if ( ADSGAIN == 2 ) {
    adc.set_pga(ADS1115_PGA_TWO);
    ADS_PGA = 0.0625;
  }
  else if ( ADSGAIN == 4 ) {
    adc.set_pga(ADS1115_PGA_FOUR);
    ADS_PGA = 0.03125;
  }
  else if ( ADSGAIN == 8 ) {
    adc.set_pga(ADS1115_PGA_EIGHT);
    ADS_PGA = 0.015625;
  }
  else if ( ADSGAIN == 16 ) {
    adc.set_pga(ADS1115_PGA_SIXTEEN);
    ADS_PGA = 0.0078125;
  }
}


void wifiServer(void)
{
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

  server.on("/config", [] { iotWebConf.handleConfig(); });

  server.on("/", handleRoot);

  server.begin();
}

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
  myPID.SetSampleTime(100);

  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
  {
    readings[thisReading] = 0;
  }

  // Enable I2C communication
  Wire.setClock(400000L); // ESP8266 Only
  Wire.begin(ESP_SDA, ESP_SCL);


#if OLED_DISPLAY == 1
  // Setup the OLED display
  display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C);  // initialize with the I2C addr 0x3C (for the 128x32)
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.display();
#endif

  configADC();

  // -- Initializing the configuration.
  iotWebConf.init();

  //  WiFiManager wm;
  //  wm.setConfigPortalTimeout(180);
  //  wm.autoConnect();

  SPIFFS.begin();

  wifiServer();

  if ( breadboard == true ) {
    ArduinoOTA.setHostname("Wemos D1 Mini - breadboard");
    ArduinoOTA.begin();  // For OTA
    delay(10);
    MDNS.begin("xespresso");
  }
  else if ( breadboard == false ) {
    ArduinoOTA.setHostname("Wemos D1 Mini - espresso");
    ArduinoOTA.begin();  // For OTA
    delay(10);
    MDNS.begin("espresso");
  }


} // end of setup()


void loop()
{
  keepTime();
  readTemps();
  relayControl();
  trackloop();
#if OLED_DISPLAY == 1
  displayOLED();
#endif
  esp8266Tasks();
} // End of loop()
