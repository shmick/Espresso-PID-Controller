## Espresso PID Controller
PID controller being used on a pre-2015 Gaggia Classic boiler

The slow responding and somewhat innacturate bimetal coffee thermostat is replaced with a K type thermocouple
connected to an Arduino controller that controls a Solid State Relay (SSR) to control power to the small boiler inside
the espresso machine.

### 2021-01-09 Update
* Properly detect mqttEnabled value and ensure an MQTT host is set in order to be set to true
* Update `CONFIG_VERSION` from `mqtt01` to `v001` to deal with possible corrupt EEPROM data during testing

### 2021-01-06 Update
* Force the SSR off if maxBoilerTemp is reached ( 140C )
* Using the steam switch to reset operMode now requires operMode to be false

### 2020-12-28 Update
* Update to IotWebConf 3.0
* Add MQTT config parameters to web portal
* Use ArduinoJson for reading and generating JSON
* Lots of code cleanup and comments added

### 2020-12-20 Update
* Code cleanup, leaving only the IotWebConf bits in place
* Enable IotWebConf httpUpdater for OTA updates
* Add steamSwitch routine for detecting when the steam switch is pressed. This requires unplugging 2 of the wires on the back of the steam switch and replacing them with wires that go back to the 2-pin Steam port on the PCB.

### 2020-12-06 Update
* Added PCB files
* ~~Code still needs to be updated to use the steam switch input~~

## Parts used
#### Wemos D1 Mini
#### AD8495 Thermocouple amp from [Adafruit](https://www.adafruit.com/product/1778)
#### K Type thermocouple from a QIDI 3D printer with M4 threads from [eBay](https://www.ebay.ca/itm/QIDI-TECHNOLOGY-high-quality-thermocouple-sensor-for-3d-printer-Screw-thread-M4/332233484894)
#### ADS1115 15bit ADC to provide better temperature resolution
#### Crydom Solid State Relay 
#### I2C 128x64 OLED display from eBay/Amazon ( not using the OLED for daily use )


***
## Documentation

## IotWebConf Info
* AP password = espresso

## MQTT
MQTT stat topic = `espresso/<thingName>/stat`

Example stat payload `{"Name":"thingName","Uptime":60,"Runtime":60,"Setpoint":105,"Input":105.0,"Output":5.08,"ADC":28457,"Vout":1.78,"operMode":true,"Loops":27734,"steamMode":false}`

MQTT command topic = `espresso/<thingName>/cmnd`

Right now the only command payload is `{"operMode":true|false}`

