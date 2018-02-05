# ArduinoAquariumMonitor

Repo link: https://github.com/IoTmaker/ArduinoAquariumMonitor

See write up of the project here:

#### See comments in .ino file for more detail.
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


## Pro-tips:
**Debugging the ESP8266**
I ran into issues when writing to the TFT display causing the ESP8266 to throw an error.
The microcontrolller will actually print a stack trace to the console.
At the time I had no idea what part of the code had generated the issue or even the nature of the error.
Luckily, serveral resources are available to help. This repo ( https://github.com/me-no-dev/EspExceptionDecoder ) contains a debugging tool that will decode the crytic codes generated by in serial console into a human readable stack trace.  Follow the instructions in the repo to install it into the Arduino IDE. Be sure to recompile your sketch before using the Tool.

Additional info on debugging:

  - http://arduino-esp8266.readthedocs.io/en/latest/faq/a02-my-esp-crashes.html

  - http://arduino-esp8266.readthedocs.io/en/latest/Troubleshooting/debugging.html
