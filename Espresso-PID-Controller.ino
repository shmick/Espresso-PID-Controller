/*
  Control an espresso machine boiler using a PID controller

  Code samples used from the following:
  PID Library: https://github.com/br3ttb/Arduino-PID-Library
  PID Lab: https://www.pdx.edu/nanogroup/sites/www.pdx.edu.nanogroup/files/2013_Arduino%20PID%20Lab_0.pdf
  Smoothing: https://www.arduino.cc/en/Tutorial/Smoothing

  Hardware:
  Wemos D1 Mini ( https://www.wemos.cc/en/latest/d1/d1_mini.html )
  AD8495 Thermocouple Amplifier ( https://www.adafruit.com/product/1778 )
  ADS1115 16-Bit ADC 
  Solid State Relay ( Crydom TD1225 )
  128 x 64 OLED (optional) Display using Adafruit_SSD1306 library
*/

#include <PID_v1.h>          // PID Library
#include <Wire.h>            // I2C Library
#include <MQTT.h>            // https://github.com/256dpi/arduino-mqtt
#include <ArduinoJson.h>     // https://arduinojson.org/
#include <IotWebConf.h>      // https://github.com/prampec/IotWebConf
#include <IotWebConfUsing.h> // https://github.com/prampec/IotWebConf
#include <ADS1115.h>         // https://github.com/baruch/ADS1115
#include <JC_Button.h>       // Debounce the steam switch https://github.com/JChristensen/JC_Button

// set to 1 if using the OLED display. Disabled by default.
#define OLED_DISPLAY 0

#if OLED_DISPLAY == 1
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSerifBold18pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#endif

// UpdateServer includes
#ifdef ESP8266
#include <ESP8266HTTPUpdateServer.h>
#elif defined(ESP32)
// For ESP32 IotWebConf provides a drop-in replacement for UpdateServer.
#include <IotWebConfESP32HTTPUpdateServer.h>
#endif

// *****************************************
// * Config options that you can customize *
// *****************************************

// -- Initial name of the Thing. Used e.g. as SSID of the own Access Point.
const char thingName[] = "espresso";             // IotWebConf thingName
const char wifiInitialApPassword[] = "espresso"; // IotWebConf Initial password to connect to the Thing
#define STRING_LEN 128                           // IotWebConf Parameters
#define CONFIG_VERSION "mqtt01"                  // IotWebConf config version for EEPROM

#define RelayPin D2 // This drives the Solid State Relay
#define SteamPin D4 // The steam switch connects to this + GND

bool operMode = true; // keep things off by default while we're testing

const int maxRunTime = 180;             // Don't keep the boiler hot after this many minutes
const int steamReset = 3;               // Flip steam on/off to reset opermode in less than 3 seconds
const int steamMaxMins = 10;            // Max number of minutes we can remain in steam mode
const int maxDisplayMins = 200;         // Turn the display off after 200 minutes
double Setpoint = 105;                  // This will be the default coffee setpoint
const double CoffeeSetpoint = Setpoint; // This will be the default coffee setpoint
const double SteamSetpoint = 125;       // This will be the default steam  setpoint
const int WindowSize = 5000;            // PID PWM Window in milliseconds

// Define the PID tuning Parameters
double Kp = 4.5;
double Ki = 0.125;
double Kd = 0.2;

// ***********************************************************
// ***********************************************************
// * There should be no need to tweak many things below here *
// ***********************************************************
// ***********************************************************

float Vout; // The voltage coming from the out pin on the TC amp
float Vtc;  // NIST pre-calc value

ADS1115 adc;           //
const int ADSGAIN = 2; // see configADC()
float ADS_PGA;         // ADC programmable gain amplifier value

const double Vref = 1.25; // The AD8495 board uses a 1.25v voltage regulator for Vref

// PID variables
// Using P_ON_M mode ( http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/ )
double Input, Output;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_M, DIRECT);
double PWMOutput;
unsigned long windowStartTime;

// Define the info needed for the temperature averaging
const int numReadings = 8;
int readings[numReadings]; // the readings from the analog input
int readIndex = 0;         // the index of the current reading
int total = 0;             // the running total
int average = 0;           // the average

// -- Method declarations.
// void handleRoot();
// void mqttMessageReceived(String &topic, String &payload);
// bool connectMqtt();
// bool connectMqttOptions();

// -- Callback methods.
// void wifiConnected();
// void configSaved();
// bool formValidator();

// IotWebConf Begin
DNSServer dnsServer;
WebServer server(80);
WiFiClient net;
MQTTClient mqttClient(256);

char mqttUserValue[STRING_LEN];
char mqttPassValue[STRING_LEN];
char mqttHostValue[STRING_LEN];
char mqttEnabledValue[STRING_LEN];

String mqttStatTopic;
String mqttCmndTopic;

#ifdef ESP8266
ESP8266HTTPUpdateServer httpUpdater;
#elif defined(ESP32)
HTTPUpdateServer httpUpdater;
#endif
IotWebConf iotWebConf(thingName, &dnsServer, &server, wifiInitialApPassword, CONFIG_VERSION);

IotWebConfParameterGroup mqttGroup = IotWebConfParameterGroup("mqtt_conf", "MQTT Configuration");
IotWebConfTextParameter mqttUserParam = IotWebConfTextParameter("MQTT User", "mqttUser", mqttUserValue, STRING_LEN);
IotWebConfPasswordParameter mqttPassParam = IotWebConfPasswordParameter("MQTT Pass", "mqttPass", mqttPassValue, STRING_LEN);
IotWebConfTextParameter mqttHostParam = IotWebConfTextParameter("MQTT Host", "mqttHost", mqttHostValue, STRING_LEN);
IotWebConfCheckboxParameter mqttEnabledParam = IotWebConfCheckboxParameter("MQTT Enabled", "mqttEnabledParam", mqttEnabledValue, STRING_LEN, false);
// IotWebConf End

bool needMqttConnect = false;
bool needReset = false;
unsigned long lastMqttConnectionAttempt = 0;

// Variable to store the HTTP request
String header;

char jsonresult[512];

// All timers reference the value of now
unsigned long now = millis(); //This variable is used to keep track of time

// OLED display timer
const int OLEDinterval = 250;           // interval at which to write new data to the OLED
unsigned long previousOLEDMillis = now; // will store last time OLED was updated

// Serial output timer
const int serialPing = 500;      //This determines how often we ping our loop
unsigned long lastMessage = now; //This keeps track of when our loop last spoke to serial

int runTimeMins;
long runTimeSecs;
unsigned long runTimeStart = now;

// Temp read interval
const int TempInterval = 50;
unsigned long previousTempMillis = now;

// Server tasks interval
const int serverInterval = 50;
unsigned long previousServerMillis = now;

// MQTT publish stats interval
const int mqttStatsOperInterval = 1000 * 5;
const int mqttStatsIdleInterval = 1000 * 20;
unsigned long previousMqttStatsMillis = now;

// Keep track of loops per second
unsigned long prevLoopMillis;
int numLoops = 0;
int currLoops = 0;

// Reset the operMode to true by toggling the steam switch for less than 3 secconds
bool steamMode = false;
bool steamTimer = false;
unsigned long steamTimeStart;
unsigned long steamTimeMillis;
Button steamsw(SteamPin, 100); // 100ms debounce for the steam switch

// Setup I2C pins
#define ESP_SCL D5
#define ESP_SDA D6

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

void keepTime(void)
{
  now = millis(); //Keep track of time
  runTimeSecs = (now - runTimeStart) / 1000;
  runTimeMins = (now - runTimeStart) / 60000;
}

void publishMqttStats(unsigned long delaytime = 0)
{
  static int mqttStatsInterval;

  if (operMode && mqttStatsInterval != mqttStatsOperInterval)
    mqttStatsInterval = mqttStatsOperInterval;
  else if (!operMode && mqttStatsInterval != mqttStatsIdleInterval)
    mqttStatsInterval = mqttStatsIdleInterval;

  // if called by enablePID, reset the previousMqttStatsMillis counter
  // to avoid a double publish within the mqttStatsInterval time
  if (delaytime > 0)
  {
    mqttClient.publish(mqttStatTopic, genJSON());
    previousMqttStatsMillis = delaytime;
  }

  if (mqttStatsInterval < now - previousMqttStatsMillis)
  {
    mqttClient.publish(mqttStatTopic, genJSON());
    previousMqttStatsMillis = now;
  }
}

int readADC()
{
  static float adcval;
  static int read_triggered = 0;
  if (!read_triggered)
  {
    if (adc.trigger_sample() == 0)
      read_triggered = 1;
  }
  else
  {
    if (!adc.is_sample_in_progress())
    {
      adcval = adc.read_sample();
      read_triggered = 0;
    }
  }
  return adcval;
}

double readTemps()
{
  static double calculatedInput;

  // Corrected temperature readings for a K-type thermocouple
  // https://srdata.nist.gov/its90/type_k/kcoefficients_inverse.html
  // Coefficient values for 0C - 500C / 0mV - 20.644mV
  static const double c0 = 0.000000E+00;
  static const double c1 = 2.508355E+01;
  static const double c2 = 7.860106E-02;
  static const double c3 = -2.503131E-01;
  static const double c4 = 8.315270E-02;
  static const double c5 = -1.228034E-02;
  static const double c6 = 9.804036E-04;
  static const double c7 = -4.413030E-05;
  static const double c8 = 1.057734E-06;
  static const double c9 = -1.052755E-08;

  if (TempInterval < now - previousTempMillis)
  {
    previousTempMillis = now;
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
      readIndex = 0; // ...wrap around to the beginning:

    // calculate the average:
    average = total / numReadings;

    // This should match the output voltage on the Out pin of the AD8945
    Vout = average * ADS_PGA / 1000;

    // Based on Analog Devices AN-1087
    // Convert the AD8495 output back to millivolts so we can perform the NIST calc
    Vtc = ((Vout * 1000) - (Vref * 1000) - 1.25) / 122.4;

    // Use the NIST corrected temperature readings for a K-type thermocouple in the 0-500°C range
    // https://srdata.nist.gov/its90/type_k/kcoefficients_inverse.html
    calculatedInput = c0 +
                      c1 * Vtc +
                      c2 * pow(Vtc, 2) +
                      c3 * pow(Vtc, 3) +
                      c4 * pow(Vtc, 4) +
                      c5 * pow(Vtc, 5) +
                      c6 * pow(Vtc, 6) +
                      c7 * pow(Vtc, 7) +
                      c8 * pow(Vtc, 8) +
                      c9 * pow(Vtc, 9);
  }
  return calculatedInput;
}

// unless true is specified, default to false
bool enablePID(bool enable = false)
{
  if (enable == false)
  {
    // de-activate the relay
    digitalWrite(RelayPin, LOW);
    // set PID mode to manual mode
    myPID.SetMode(MANUAL);
    // Force PID output to 0
    Output = 0;
    // set operation mode to false
    operMode = false;
  }
  else if (enable == true)
  {
    myPID.SetMode(AUTOMATIC);
    runTimeStart = now;
    operMode = true;
  }

  if (mqttEnabledValue)
    publishMqttStats(now);
}

void relayControl(void)
{
  Input = readTemps(); // Providde the PID loop with the current temperature

  if (runTimeMins >= maxRunTime && operMode) // If we've reached maxRunTime, disable the PID control
    enablePID(false);

  else if (!myPID.GetMode() && operMode) // Set the PID back to Automatic mode if operMode is true
    enablePID(true);

  if (operMode)
  {
    myPID.Compute();

    // Starts a new PWM cycle every WindowSize milliseconds
    if (WindowSize < now - windowStartTime)
      windowStartTime += WindowSize;

    // Calculate the number of milliseconds that have passed in the current PWM cycle.
    // If that is less than the Output value, the relay is turned ON
    // If that is greater than (or equal to) the Output value, the relay is turned OFF.
    PWMOutput = Output * (WindowSize / 100.00);
    if ((PWMOutput > 100) && (PWMOutput > (now - windowStartTime)))
      digitalWrite(RelayPin, HIGH);
    else
      digitalWrite(RelayPin, LOW);
  }
}

void steamSwitch()
{
  // read the debounced value of the steam switch
  steamsw.read();

  // if switch is on, set steamMode to true
  if (steamsw.isPressed() && !steamMode)
    steamMode = true;
  else if (!steamsw.isPressed() && steamMode)
    steamMode = false;

  // start the steamTimer when the steamMode is active
  if (steamMode && !steamTimer)
  {
    steamTimer = true;
    steamTimeStart = now;
  }
  else if (!steamMode && steamTimer)
  {
    steamTimer = false;
  }

  // If steamTimer is true, update steamTimeMillis
  if (steamTimer)
    steamTimeMillis = now - steamTimeStart;

  // if steamMode is now false, check to see if we should set operMode to true
  if (!steamMode && steamTimeMillis > 0)
  {
    if (steamTimeMillis / 1000 <= steamReset)
    {
      steamTimeMillis = 0;
      enablePID(true);
    }
  }

  // This must be the last if statement. It's a safety check to ensure
  // that we set enablePID to false if the steam switch has been on too long
  if (steamTimeMillis / 1000 / 60 >= steamMaxMins)
    enablePID(false);
  else if (steamMode && operMode && Setpoint != SteamSetpoint)
    Setpoint = SteamSetpoint;
  else if (!steamMode && operMode && Setpoint != CoffeeSetpoint)
    Setpoint = CoffeeSetpoint;
}

// Track how many loops per second are executed.
void trackloop()
{
  if (now - prevLoopMillis >= 1000)
  {
    currLoops = numLoops;
    numLoops = 0;
    prevLoopMillis = now;
  }
  numLoops++;
}

bool connectMqtt()
{
  if (1000 > now - lastMqttConnectionAttempt)
  {
    // Do not repeat within 1 sec.
    return false;
  }
  Serial.println("Connecting to MQTT server...");
  if (!connectMqttOptions())
  {
    lastMqttConnectionAttempt = now;
    return false;
  }
  Serial.println("Connected!");

  mqttClient.subscribe(mqttCmndTopic);
  publishMqttStats(now);

  return true;
}

bool connectMqttOptions()
{
  bool result;

  if (mqttUserValue[0] != '\0' && mqttPassValue[0] != '\0')
    result = mqttClient.connect(iotWebConf.getThingName(), mqttUserValue, mqttPassValue);
  else if (mqttUserValue[0] != '\0')
    result = mqttClient.connect(iotWebConf.getThingName(), mqttUserValue);
  else
    result = mqttClient.connect(iotWebConf.getThingName());

  return result;
}

void displaySerial(void)
{
  // Output some data to serial to see what's happening
  if (serialPing < now - lastMessage)
  {
    if (operMode)
    {
      Serial.print("Time: ");
      Serial.println(runTimeSecs);
    }
    else
    {
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
  s += "<title>Espresso PID Controller</title></head><body>Espresso PID Controller";
  s += "<br>\n";
  s += "Go to <a href='stats'>stats page</a> for controller stats.";
  s += "<br>\n";
  s += "<ul>";
  s += "<li>MQTT Host: ";
  s += mqttHostValue;
  s += "<li>MQTT Enabled: ";
  s += mqttEnabledParam.isChecked();
  s += "<li>OperMode: ";
  s += operMode;
  s += "</ul>";
  s += "Go to <a href='config'>configure page</a> to change values.";
  s += "</body></html>\n";

  server.send(200, "text/html", s);
}

void handleStats()
{
  String m = "<style> ul { list-style:none; padding-left:0; } </style>";
  m += "<head> <meta http-equiv=\"refresh\" content=\"2\"> </head>\n";
  m += "<h1>\n";
  m += "<ul>";
  m += "<li>Time: ";
  m += runTimeSecs;
  m += "<li>Setpoint: ";
  m += Setpoint;
  m += "<li>PID Input:  ";
  m += Input;
  m += "<li>PID Output: ";
  m += Output;
  m += "<li>Avg: ";
  m += average;
  m += "<li>Vout: ";
  m += Vout;
  m += "<li>Loops: ";
  m += currLoops;
  m += "<li>OperMode: ";
  m += operMode;
  m += "<li>SteamMode: ";
  m += steamMode;
  m += "</ul>";
  server.send(200, "text/html", m);
}

double round2(double value) // Round down a float to 2 decimal places
{
  return (int)(value * 100 + 0.5) / 100.0;
}

char *genJSON()
{
  DynamicJsonDocument doc(256);
  doc["Name"] = iotWebConf.getThingName();
  doc["Uptime"] = now / 1000;
  doc["Runtime"] = runTimeSecs;
  doc["Setpoint"] = Setpoint;
  doc["Input"] = round2(Input);
  doc["Output"] = round2(Output);
  doc["ADC"] = average;
  doc["Vout"] = round2(Vout);
  doc["operMode"] = operMode;
  doc["Loops"] = currLoops;
  doc["steamMode"] = steamMode;

  serializeJson(doc, jsonresult);
  return jsonresult;
}

void handleJSON()
{
  server.send(200, "application/json", String(genJSON()));
}

// ESP8266WebServer handler
void handleSetvals()
{
  String message;

  String opmode_val = server.arg("opmode");
  String sp_val = server.arg("sp");

  if (opmode_val == "off")
  {
    enablePID(false);
    message += "opermode: " + opmode_val;
    message += "\n";
  }
  else if (opmode_val == "on")
  {
    enablePID(true);
    message += "opermode: " + opmode_val;
    message += "\n";
  }

  if (sp_val != NULL)
  {
    double sp_int = sp_val.toFloat();
    if (sp_int <= 105.11 || sp_int > 0.1)
    {
      Setpoint = sp_int;
      message += "Setpoint: " + sp_val;
      message += "\n";
    }
    else
    {
      message += "sp: " + String(sp_val) + " is invalid\n";
    }
  }
  server.send(200, "text/plain", message);
}

void esp8266Tasks()
{
  if (serverInterval < now - previousServerMillis)
  {
    previousServerMillis = now;

    iotWebConf.doLoop();

    if (mqttEnabledValue)
    {
      mqttTasks();
      publishMqttStats();
    }

    if (needReset)
    {
      Serial.println("Rebooting after 1 second.");
      iotWebConf.delay(1000);
      ESP.restart();
    }
  }
}

void mqttTasks()
{
  mqttClient.loop();

  if (needMqttConnect)
  {
    if (connectMqtt())
      needMqttConnect = false;
  }
  else if ((iotWebConf.getState() == IOTWEBCONF_STATE_ONLINE) && (!mqttClient.connected()))
  {
    Serial.println("MQTT reconnect");
    connectMqtt();
  }
}

void configADC(void)
{
  // Setup ADS1115
  adc.begin();
  adc.set_data_rate(ADS1115_DATA_RATE_860_SPS);
  adc.set_mode(ADS1115_MODE_SINGLE_SHOT);
  adc.set_mux(ADS1115_MUX_GND_AIN0);

  switch (ADSGAIN)
  {
  case 23: // TWO_THIRDS // 2/3x gain +/- 6.144V  1 bit = 0.1875mV
    adc.set_pga(ADS1115_PGA_TWO_THIRDS);
    ADS_PGA = 0.1875;
    break;
  case 1: //  ONE        // 1x gain   +/- 4.096V  1 bit = 0.125mV
    adc.set_pga(ADS1115_PGA_ONE);
    ADS_PGA = 0.125;
    break;
  case 2: //  TWO        // 2x gain   +/- 2.048V  1 bit = 0.0625mV
    adc.set_pga(ADS1115_PGA_TWO);
    ADS_PGA = 0.0625;
    break;
  case 4: //  FOUR       // 4x gain   +/- 1.024V  1 bit = 0.03125mV
    adc.set_pga(ADS1115_PGA_FOUR);
    ADS_PGA = 0.03125;
    break;
  case 8: //  EIGHT      // 8x gain   +/- 0.512V  1 bit = 0.015625mV
    adc.set_pga(ADS1115_PGA_EIGHT);
    ADS_PGA = 0.015625;
    break;
  case 16: //  SIXTEEN    // 16x gain  +/- 0.256V  1 bit = 0.0078125mV
    adc.set_pga(ADS1115_PGA_SIXTEEN);
    ADS_PGA = 0.0078125;
    break;
  }
}

void iotWebConfSetup(void)
{
  // -- Define how to handle updateServer calls.
  iotWebConf.setupUpdateServer(
      [](const char *updatePath) { httpUpdater.setup(&server, updatePath); },
      [](const char *userName, char *password) { httpUpdater.updateCredentials(userName, password); });

  mqttGroup.addItem(&mqttUserParam);
  mqttGroup.addItem(&mqttPassParam);
  mqttGroup.addItem(&mqttHostParam);
  mqttGroup.addItem(&mqttEnabledParam);
  iotWebConf.addParameterGroup(&mqttGroup);

  iotWebConf.setConfigSavedCallback(&configSaved);
  iotWebConf.setFormValidator(&formValidator);
  iotWebConf.setWifiConnectionCallback(&wifiConnected);

  // -- Initializing the configuration.
  bool validConfig = iotWebConf.init();
  if (!validConfig)
  {
    mqttUserValue[0] = '\0';
    mqttPassValue[0] = '\0';
    mqttHostValue[0] = '\0';
    mqttEnabledValue[0] = '\0';
  }

  server.on("/", handleRoot);
  server.on("/stats", handleStats); //Reads ADC function is called from out index.html
  server.on("/json", handleJSON);
  server.on("/set", HTTP_POST, handleSetvals);
  server.on("/config", [] { iotWebConf.handleConfig(); });
  server.onNotFound([]() { iotWebConf.handleNotFound(); });

  if (mqttEnabledValue)
  {
    mqttStatTopic = "espresso/" + String(iotWebConf.getThingName()) + "/stat";
    mqttCmndTopic = "espresso/" + String(iotWebConf.getThingName()) + "/cmnd";

    mqttClient.begin(mqttHostValue, net);
    mqttClient.onMessage(mqttMessageReceived);
  }
}

void mqttMessageReceived(String &topic, String &payload)
{
  Serial.println("Incoming: " + topic + " - " + payload);

  // we should only be listening to input from the cmnd topic
  if (topic == mqttCmndTopic)
  {
    StaticJsonDocument<200> doc;
    deserializeJson(doc, payload);

    bool operModeVal = doc["operMode"];

    if (operModeVal)
      enablePID(operModeVal);
    else if (!operModeVal)
      enablePID(operModeVal);
  }
}

void wifiConnected()
{
  needMqttConnect = true;
}

void configSaved()
{
  Serial.println("Configuration was updated.");
  needReset = true;
}

bool formValidator()
{
  Serial.println("Validating form.");
  bool valid = true;

  int l = server.arg(mqttHostParam.getId()).length();
  if (l < 3)
  {
    mqttHostParam.errorMessage = "Please provide at least 3 characters!";
    valid = false;
  }
  return valid;
}

#if OLED_DISPLAY == 1
void displayOLED(void)
{
  if (OLEDinterval < now - previousOLEDMillis)
  {
    previousOLEDMillis = now;

    // save the last time you wrote to the OLED display

    // have to wipe the buffer before writing anything new
    display.clearDisplay();

    if (operMode)
    {
      // TOP HALF = Temp + Input Temp
      display.setFont(&FreeSans9pt7b);
      display.setCursor(0, 22);
      display.print("Temp");

      display.setFont(&FreeSerifBold18pt7b);
      display.setCursor(48, 26);
      if (Input >= 100)
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
      if ((Output >= 100.0) || (Output == 0.0))
      {
        display.print(Output, 0);
      }
      else if (Output < 10)
      {
        display.print(Output, 2);
      }
      else
      {
        display.print(Output, 1);
      }
    }
    else if (!operMode)
    {
      // After maxDisplayMins minutes turn off the display
      if (runTimeMins >= maxDisplayMins)
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

void setup()
{
  Serial.begin(115200); //Start a serial session
  lastMessage = now;    // timestamp

  // Set the Relay to output mode and ensure the relay is off
  pinMode(RelayPin, OUTPUT);
  digitalWrite(RelayPin, LOW);

  steamsw.begin(); // initialize the button object

  windowStartTime = now;         // PID Window
  myPID.SetOutputLimits(0, 100); // PID output 0 - 100
  myPID.SetSampleTime(50);       // PID Samples every 50ms

  // initialize all the readings to 0 for the temperature averaging
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
  {
    readings[thisReading] = 0;
  }

  // Enable I2C communication
  Wire.setClock(400000L); // ESP8266 Only
  Wire.begin(ESP_SDA, ESP_SCL);

  configADC();
  iotWebConfSetup();

#if OLED_DISPLAY == 1
  // Setup the OLED display
  display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C); // initialize with the I2C addr 0x3C (for the 128x32)
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.display();
#endif
} // END setup

void loop()
{
  keepTime();     // keep updating now()
  steamSwitch();  // watch the steam switch and react to changes
  relayControl(); // control the relay via the PID loop
  esp8266Tasks(); // perform IotWebConf and MQTT tasks
  trackloop();    // track how many loops/sec the processor is doing
#if OLED_DISPLAY == 1
  displayOLED();
#endif
} // END loop
