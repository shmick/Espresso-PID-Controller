## Espresso PID Controller
PID controller being used on a pre-2015 Gaggia Classic boiler

The slow responding and somewhat innacturate bimetal coffee thermostat is replaced with a K type thermocouple
connected to an Arduino controller that controls a Solid State Relay (SSR) to control power to the small boiler inside
the espresso machine.

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
