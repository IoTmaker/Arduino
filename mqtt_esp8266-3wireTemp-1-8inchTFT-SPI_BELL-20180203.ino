/***************************************************
 * TODO:
 * Add turn led on off and wire up led(serves as relay)
 * add light detector and rewrite logic to read from meter
 * Figure out why water temp sensor gives 0.00 reading every other sample
 * Implement controll of light
 * 
 * 
 *  ****************************************************/

/* ****************************************************
 *  - Adafruit Huzzah ESP8266 board or Feather
        https://www.adafruit.com/product/2471
        https://www.adafruit.com/products/2821
 *  - DS18B20 (waterproof) 1 wire temp sensor
 *  - DHT11 temperature and humidity sensor 
 *  - Adafruit 1.8 inch TFT display 
 *  - 4.7 KOhm resistor
 *  - Adafruit FTDI Serial TTL-232 USB Cable or equal to program Huzzah
 *  
 *  Project by Steven Bell. @cropcommand 
 *  Additional resources credited in comments below.  
 *  Project URL:
 *  https://www.hackster.io/thestevenbell/monitor-water-and-air-temperature-with-esp8266-and-mqtt-bb56da 
 *  Project Repo:
 *  https://github.com/IoTmaker/ArduinoAquariumMonitor
 *  ****************************************************/
 
 /***************************************************

  How To Set Up Hardware:
  https://learn.adafruit.com/home-automation-in-the-cloud-with-the-esp8266-and-adafruit-io/programming-the-modules
  https://learn.adafruit.com/remote-control-with-the-huzzah-plus-adafruit-io
  
  How To Set Up Software and library dependencies:
  - Must use ESP8266 Arduino Library.  Follow the instructions in the repo to set 
    up the Arduino IDE to be able to program the Huzzah:
    https://github.com/esp8266/Arduino
  - Basic example of DHT11 with MQTT to Adafruit.io:
    https://github.com/openhomeautomation/adafruit-io-esp8266/blob/master/esp8266_sensor_module/esp8266_sensor_module.ino
  - Adafruit MQTT Library : https://github.com/adafruit/Adafruit_MQTT_Library
  - Adafruit Unified Sensor Library: https://github.com/adafruit/Adafruit_Sensor
  - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
  - For the DS18B20 temp sensor:
    http://milesburton.com/Dallas_Temperature_Control_Library

  
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Tony DiCola for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 *  ****************************************************/
  
/* ****** OneWire DS18B20 Temperature Example**********
 http://www.pjrc.com/teensy/td_libs_OneWire.html

 The DallasTemperature library can do all this work for you!
 http://milesburton.com/Dallas_Temperature_Control_Library

 https://create.arduino.cc/projecthub/TheGadgetBoy/ds18b20-digital-temperature-sensor-and-arduino-9cc806

 *  ****************************************************/
 
/* *****************DHT11****************************
 * http://www.hobbyist.co.nz/?q=documentations/wiring-up-dht11-temp-humidity-sensor-to-your-arduino
 * 
 *  ****************************************************/

#include "ESP8266WiFi.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <DHT.h>
#include <DHT_U.h>
#include <OneWire.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>
#include <DHT.h>
// Contains Keys and access tokens.  Path: Arduino/libraries/configuration/Configuration.h
#include <Configuration.h>

/************************* WiFi Access Point *********************************/
#define WLAN_SSID       CONFIG_WLAN_SSID
#define WLAN_PASS       CONFIG_WLAN_PASS

/************************* Adafruit.io Setup *********************************/
#define AIO_SERVER      CONFIG_AIO_SERVER
#define AIO_SERVERPORT  CONFIG_AIO_SERVERPORT           // use 8883 for SSL
#define AIO_USERNAME    CONFIG_AIO_USERNAME
#define AIO_KEY         CONFIG_AIO_KEY

/************ Global State (you don't need to change this!) ******************/
// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure client;

const char MQTT_SERVER[]     = AIO_SERVER;
const char MQTT_USERNAME[]   = AIO_USERNAME;
const char MQTT_PASSWORD[]   = AIO_KEY;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, AIO_SERVERPORT, MQTT_USERNAME, MQTT_PASSWORD);

/******************** DHT TEMP & HUMIDITY SENSOR ****************************/
#define DHTPIN            2         // Pin which is connected to the DHT sensor.
#define DHTTYPE           DHT11     // DHT 11 
DHT_Unified dht(DHTPIN, DHTTYPE);

/********* DS18B20 (digital temperature sensor)  ****************************/
OneWire  ds(13);  // on pin 10 (a 4.7K resistor is necessary)

/********* TFT 1.8 inch ST7735 from Adafruit  ****************************/
// These pins work for the 1.8" TFT shield on Huzzah
#define TFT_CS    14
#define TFT_RST   15                 
#define TFT_DC    12
#define TFT_SCLK  4   // Note, overwites the default Huzzah i2c pins. set these to be whatever pins you like!
#define TFT_MOSI  5   // Note, overwites the default Huzzah i2c pins. set these to be whatever pins you like!
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

/****************************** Feeds ***************************************/

// Setup a feed called 'photocell' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
//const char PHOTOCELL_FEED[]  = AIO_USERNAME "/feeds/photocell";
//Adafruit_MQTT_Publish photocell = Adafruit_MQTT_Publish(&mqtt, PHOTOCELL_FEED);

const char TEMPERATURE_FEED[]  = AIO_USERNAME "/feeds/temperature";
Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, TEMPERATURE_FEED);

const char WATER_TEMPERATURE_FEED[]  = AIO_USERNAME "/feeds/water_temperature";
Adafruit_MQTT_Publish water_temperature = Adafruit_MQTT_Publish(&mqtt, WATER_TEMPERATURE_FEED);

const char HUMIDITY_FEED[]  = AIO_USERNAME "/feeds/humidity";
Adafruit_MQTT_Publish humidity = Adafruit_MQTT_Publish(&mqtt, HUMIDITY_FEED);

// Setup a feed called 'onoff' for subscribing to changes.
const char ONOFF_FEED[]  = AIO_USERNAME "/feeds/onoff";
Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, ONOFF_FEED);


/*************************** Sketch Code ************************************/

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();

void setup() {
  Serial.begin(115200);
  delay(10);

  initilizeTFTDisplay();

  connectToWifi();

  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&onoffbutton);
  dht.begin();

  getDHTSensorDetails();
}

void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  // this is our 'wait for incoming subscription packets' busy subloop
  Adafruit_MQTT_Subscribe *subscription;

  // Implement to controll light
//  while ((subscription = mqtt.readSubscription(1000))) {
//    if (subscription == &onoffbutton) {
//      Serial.print(F("Got: "));
//      Serial.println((char *)onoffbutton.lastread);
//    }
//  }

  float water_temp = readTemperatureFrom1WireWaterproofSensor();
  float air_temperature = readAirTemperatureFromDHT11(air_temperature);
  float air_humidity = readAirHumidityFromDHT11(air_humidity);

  Serial.print("--air_temperature_data is ");
  Serial.println(air_temperature);
  if (! temperature.publish(air_temperature)){
    Serial.println(F("Failed to publish air_temperature\n"));
  }
  else{
    Serial.println(F("Temperature published!\n"));
  }
    
  Serial.print("--water_temperature is ");
  Serial.println(water_temp);
  if(water_temp > 0){
      if (! water_temperature.publish(water_temp)){
        Serial.println(F("Failed to publish water_temperature\n"));
      }
    else{
      Serial.println(F("Water Temperature published!\n"));
    }
  }
   
  Serial.print("--humidity_data is ");
  Serial.println(air_humidity);
  if (! humidity.publish(air_humidity))
    Serial.println(F("Failed to publish humidity\n"));
  else
    Serial.println(F("Humidity published!\n"));
  
  // reset screen for next set of readings
  tft.fillScreen(ST7735_BLACK);
  displayWaterTemperature(water_temp);
  displayAirTemperature(air_temperature);
  displayHumidityTemperature(air_humidity);
  
//  Serial.print(F("\nSending photocell val "));

//  Serial.print("...");
//  if (! photocell.publish(photocell)) {
//    Serial.println(F("Failed\n"));
//  } else {
//    Serial.println(F("OK!\n"));
//  }

    // ping the server to keep the mqtt connection alive
  // NOT required if you are publishing once every KEEPALIVE seconds
//  keepAliveMQTT();

   delay(5000);
}

float readTemperatureFrom1WireWaterproofSensor(){
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius, fahrenheit;
  
  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
    return 0.00;
  }
  
  Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return 0.00;
  }
  Serial.println();
 
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return 0.00;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  delay(750);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  Serial.print("  Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
  Serial.print("Water Temperature = ");
  Serial.print(celsius);
  Serial.print(" Celsius, ");
  Serial.print(fahrenheit);
  Serial.println(" Fahrenheit");

  return celsius;
}


void getDHTSensorDetails(){
  Serial.println("DHTxx Unified Sensor Example");
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Temperature");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");  
  Serial.println("------------------------------------");
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Humidity");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");  
  Serial.println("------------------------------------");
  // Set delay between sensor readings based on sensor details.
  //  delayMS = sensor.min_delay / 1000;
}

float readAirTemperatureFromDHT11(float air_temperature){
   // Get temperature event and print its value.
  sensors_event_t event;  
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading temperature!");
  }
  else {
    Serial.print("DHT11, Air Temperature: ");
    Serial.print(event.temperature);
    Serial.println(" *C");
  }
 
  return event.temperature;

}

float readAirHumidityFromDHT11(float air_humidity){
   // Get temperature event and print its value.
  sensors_event_t event;  
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading humidity!");
  }
  else {
    Serial.print("DHT11, Humidity: ");
    Serial.print(event.relative_humidity);
    Serial.println("%");
  }
  
  return event.relative_humidity;
}

void displayWaterTemperature(float water_temp){
  tft.setCursor(0, 0);
  String color = "white";
  if(water_temp > 0){
    if (water_temp <= 23.3 && water_temp >= 22.2){
      color = "yellow";
    }else if (water_temp <= 22.2){
      color = "blue";
    }else if (water_temp >= 23.3 && water_temp <= 25.5){
      color = "green";
    }else if (water_temp >= 25.5 && water_temp <= 26.1){
      color = "yellow";  
    } else {
      color = "red";
    }
    displayText("H2O Temp", " C", color , water_temp);
  } else {
    //do nothing - getting a zero value every other reading from water temp sensor
  }
}

void displayAirTemperature(float air_temp){
  tft.setCursor(0, 50);
  String color = "white";
  if(air_temp > 0){
    if (air_temp <= 20 && air_temp >= 18.3){
      color = "yellow";
    }else if (air_temp <= 20){
      color = "blue";
    }else if (air_temp >= 20 && air_temp <= 22.2){
      color = "green";
    }else if (air_temp >= 22.2 && air_temp <= 23.3){
      color = "yellow";  
    } else {
      color = "red";
    }
    displayText("Air Temp", " C", color , air_temp);
  } else {
    //do nothing - getting a zero value every other reading from water temp sensor
  }
}

void displayHumidityTemperature(float air_humidity){
  tft.setCursor(0, 100);
  String color = "white";
  if(air_humidity > 0){
    if (air_humidity <= 40 && air_humidity >= 35){
      color = "yellow";
    }else if (air_humidity <= 40){
      color = "blue";
    }else if (air_humidity >= 40 && air_humidity <= 55){
      color = "green";
    }else if (air_humidity >= 55 && air_humidity <= 70){
      color = "yellow";  
    } else {
      color = "red";
    }
    displayText("Humidity", " %", color , air_humidity);
  } else {
    //do nothing - getting a zero value every other reading from water temp sensor
  }
 }
 
void displayText(String parameter, String unitOfMeasure, String color, float value){
// LIST OF AVAILABLE PREDEFINED COLORS IN LIBRARY
//#define  ST7735_BLACK   0x0000
//#define ST7735_BLUE    0x001F
//#define ST7735_RED     0xF800
//#define ST7735_GREEN   0x07E0
//#define ST7735_CYAN    0x07FF
//#define ST7735_MAGENTA 0xF81F
//#define ST7735_YELLOW  0xFFE0
//#define ST7735_WHITE   0xFFFF

  if (color == "red"){
    tft.setTextColor(ST7735_RED);
  }else if (color == "green"){
    tft.setTextColor(ST7735_GREEN);
  }else if (color == "yellow"){
     tft.setTextColor(ST7735_YELLOW);
  }else if (color == "blue"){
    tft.setTextColor(ST7735_BLUE);
  }else {
    tft.setTextColor(ST7735_WHITE);
  }
  tft.setTextSize(2);
  tft.println(parameter + unitOfMeasure);
  tft.setTextSize(3);
  tft.println(value);
}

void initilizeTFTDisplay(){
  // Use this initializer if you're using a 1.8" TFT
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
  Serial.println("TFT Initialized");
  tft.fillScreen(ST7735_BLACK);
}

void connectToWifi(){
  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
 
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: "); 
  Serial.println(WiFi.localIP());
}

void keepAliveMQTT(){
  //    if(! mqtt.ping(3)) {
  // reconnect to adafruit io
//    if(! mqtt.connected())
//    MQTT_connect();
//  }
  
//  if(! mqtt.ping()) {
//    mqtt.disconnect();
//  }
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}
