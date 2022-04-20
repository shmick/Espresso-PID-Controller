## Changelog for Espresso PID Controller

### 2022-04-20 Update
* Fix: MQTT reconnect changed from 2 to 15 to stop reconnect from blocking the rest of the loop

### 2022-04-17 Update
* Add ability to change Setpoint via MQTT `{"Setpoint":<number between 1 and 110>}` 
* Add ability to enable steamMode via MQTT `{"steamMode":true|false}`

### 2022-04-16 Update
* Fix: Small changes required to compile using IotWebConf v3.2.0

### 2021-01-13 Update
* Add MQTT Last Will and Testament. Topic = `espresso/<thingname>/avail` 
* Fixed a bug that caused steam switch reset to sometimes set operMode to true directly after setting it to false
* Minor code cleanup

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