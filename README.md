## Espresso PID Controller
PID controller being used on a pre-2015 Gaggia Classic boiler

The slow responding and somewhat innacturate bimetal coffee thermostat is replaced with a K type thermocouple
connected to an Arduino controller that controls a Solid State Relay (SSR) to control power to the small boiler inside
the espresso machine.

### 2022-04-20 Update
* Fix: MQTT reconnect changed from 2 to 15 to stop reconnect from blocking the rest of the loop

### See [CHANGELOG](CHANGELOG.md) for previous updates

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

* `{"operMode":true|false}`
* `{"steamMode":true|false}`
* `{"Setpoint":<number between 1 and 110>}`

