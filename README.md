# ArduinoAquariumMonitor

See comments in .ino file for more detail.
See write up of the project here: https://github.com/IoTmaker/ArduinoAquariumMonitor

### Prerequises:

Create <user home>/Arduino/libraries/configuration/Configuration.h
  Add the following constants replacing the values with yours:
```
/************************* WiFi Access Point *********************************/
#define CONFIG_WLAN_SSID       "some ssid"
#define CONFIG_WLAN_PASS       "password"

/************************* Adafruit.io Setup *********************************/
#define CONFIG_AIO_SERVER      "io.adafruit.com"
#define CONFIG_AIO_SERVERPORT  1883                   // use 8883 for SSL
#define CONFIG_AIO_USERNAME    "username"
#define CONFIG_AIO_KEY         "some_key_9845jgahdfahdsfouhadufuvasiub"
```
