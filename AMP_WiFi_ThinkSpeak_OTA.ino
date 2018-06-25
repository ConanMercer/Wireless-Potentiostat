//#include<Ethernet.h>      // library to work with ESP-12E board, used to connect to a specified internet IP address and port
//#include<ESP8266WiFi.h>         // library to work with ESP-12E board
#include <Servo.h>              // Communicate with servo-valves
#include <HardwareSerial.h>     // To utilise all of ESP32 hardware UARTs
#include <Wire.h>               // Serial communicate with I2C / TWI devices
#include <Adafruit_ADS1015.h>   // Driver for ADS1115 16-bit ADC
#include <Adafruit_MCP4725.h>   // Driver for MCP4725 12-bit DAC
//For smart phone connection
#include "SPIFFS.h"
#include <WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
//For OTA updates
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "credentials.h"

Adafruit_ADS1115 ads(0x49);     // Construct the ADS1115 at address 0x49
Adafruit_MCP4725 dac;           // Construct the MCP4725

WiFiClient client;              // Initialize WiFi client library
char server[] = "api.thingspeak.com"; // ThingSpeak Server address

Servo servo1;                   // Define servo-valves
Servo servo2;
Servo servo3;

HardwareSerial PumpSerial(1);   // Define hardware UARTs

/* Collect data once every 1 second and post data to ThingSpeak channel once every 16 seconds */
unsigned long lastConnectionTime = 0;               // Track the last connection time
unsigned long previousUpdate = 0;                   // Track the last update time
const unsigned long postingInterval = 16L * 1000L;  // Post data every 16 seconds
const unsigned long updateInterval = 0.5L * 1000L;    // Update JSON once every 100ms

/* Multi core tasks */
TaskHandle_t Task1, Task2;
SemaphoreHandle_t baton;

int16_t adc0, adc1, adc2, adc3; // Signed integer type with width of 16 bits for ADC using four channels
uint32_t dac_value;           // Unsigned integer type with width of 32 bits for DAC values

char Voltage[2890];  // Variable for control of DAC output set to 0.4V

//flag for saving data
bool shouldSaveConfig = true;

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

char jsonBuffer[1500] = "[";    // Initialize the jsonBuffer to hold data
int val = 2890;  // Variable for control of DAC output set to 0.4V
int pos = 0;
float c1 = 0;  // Variable for storage of current reading channel 1
float c2 = 0;  // Variable for storage of current reading channel 2
float c3 = 0;  // Variable for storage of current reading channel 3
float c4 = 0;  // Variable for storage of current reading channel 4
float v = 0;  // Variable for storage of voltage applied
int n = 0;    // Variable for storage of potential scan direction
float A = 0.01527;  // Variable for storage of slope current conversion 200uA
float B = 200;      // Variable for storage of intersect current conversion 200uA
float A2 = 0.003814;  // Variable for storage of slope current conversion  50uA
float B2 = 50;      // Variable for storage of intersect current conversion 50uA
float M = 0.000504637;  // Variable for storage of voltage multiplier
float O = 1.01562452;   // Variable for storage of voltage offset
int scanRate[] = {100, 100, 1, 1, 1, 1}; // multiple scan rate values (mV/s)
int const count = 6;    //number of CV sweeps
long intervals[count];

byte pumpStop[] = {0x73, 0x74, 0x70, 0x0d, 0x0a}; //Pump stop
byte pump200ul[] = {0x72, 0x61, 0x74, 0x31, 0x30, 0x30, 0x75, 0x6d, 0x0d, 0x0a, 0x72, 0x75, 0x6e, 0x0d, 0x0a}; //Pump 200ul

void setup() {
  // Setup needed for OTA
  Serial.begin(115200);   // Initiate serial communication to ESP-12E at 112500 baud rate

  SPIFFS.begin (true);
  //read configuration from FS json
  Serial.println("mounting FS...");
  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          strcpy(Voltage, json["Voltage"]);
        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read
  WiFiManagerParameter custom_Voltage("Voltage", "Voltage", Voltage, 40);
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  //add all your parameters here
  wifiManager.addParameter(&custom_Voltage);

  if (!wifiManager.autoConnect("ESP32 Potentiostat", "G42")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.restart();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...)");

  //read updated parameters
  strcpy(Voltage, custom_Voltage.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["Voltage"] = Voltage;
    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }
    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)         Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)   Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR)     Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

    baton = xSemaphoreCreateMutex();

  /* Core 0 task */
  xTaskCreatePinnedToCore(
    core0Task,                /* task function. */
    "core0",                  /* name of task. */
    10000,                    /* stack size of task */
    NULL,                     /* parameter of the task */
    1,                        /* priority of the task */
    &Task1,                   /* task handle to keep track of created task */
    0);                       /* Core */

  delay(500);                 /* needed to start-up Core 1 task */

  /* Core 1 task */
  xTaskCreatePinnedToCore(
    core1Task,                /* task function. */
    "core1",                  /* name of task. */
    10000,                    /* stack size of task */
    NULL,                     /* parameter of the task */
    1,                        /* priority of the task */
    &Task2,                   /* task handle to keep track of created task */
    1);                       /* Core */

  //chamber1();
  //chamber2();
  //chamber3();
  chamber4();
  PumpSerial.begin(19200, SERIAL_8N1, 19, 18);
  PumpSerial.write(pump200ul, sizeof(pump200ul));
  /* Core */
  //serial.begin(19200);          // Opens pump serial communication at baud rate 19200
  Wire.begin(21, 22);             // I2C Pins on ESP32
  ads.begin();                    // Initiate ADS1115 ADC
  dac.begin(0x62);                // Initiate MCP4725 DAC
  Wire.setClock(400000L);         // Set i2c to 400 kHz
  ads.setGain(GAIN_TWOTHIRDS);    // 2/3x gain +/- 6.144V  1 bit = 0.1875mV
  //ads.setSPS(ADS1115_DR_860SPS);  // Set ADS1115 samples per second to 860 (effects scan rate speed)
  //ads.setGain(GAIN_ONE);        // Set ADS1115 gain to 1x to allow for +/- 4.096V 1 bit = 0.125mV
  dac.setVoltage(val, false);   // false = don't save to EEPROM, ser amp potential
}

/* this function will apply the potential window */
void core0Task( void * parameter )
{
  /* loop forever */
  for (;;) {

    if (millis() - previousUpdate >=  updateInterval) {
      updatesJson(jsonBuffer);
    }
  }
  /* delete a task when finish,
    this will never happen because this is infinity loop */
  vTaskDelete( NULL );
}

/* this function will read the current */
void core1Task( void * parameter )
{
  /* loop forever */
  for (;;) {
    current();
  }
  /* delete a task when finish,
    this will never happen because this is infinity loop */
  vTaskDelete( NULL );
}

void loop() {
}

void current()
{
  ArduinoOTA.handle();
  yield();
  delay(10);
  //delay(intervals[pos]);
  //adc0 = ads.readADC_SingleEnded(0);
  //adc1 = ads.readADC_SingleEnded(1);
  //adc2 = ads.readADC_SingleEnded(2);
  //adc3 = ads.readADC_SingleEnded(3);
  v = ((val * M) - O);
  //Serial.print(v, 4);
  //c1 = (-((A * (adc0)) - B)); // Current reading output in uA channel 1
  //c2 = (-((A * (adc1)) - B)); // Current reading output in uA channel 2
  //c3 = (-((A * (adc2)) - B)); // Current reading output in uA channel 3
  //c4 = (-((A * (adc3)) - B)); // Current reading output in uA channel 4
  //Serial.print(",");
  //Serial.println(c1, 3);
  //Serial.print(",");
  //Serial.print(c2, 3);
  //Serial.print(",");
  //Serial.print(c3, 3);
  //Serial.print(",");
  //Serial.println(c4, 3);
  //Serial.print(",");
  //Serial.print(scanRate[pos]);
  //Serial.print(",");
  //Serial.println(intervals[pos]);
}


void potential()
{
  if (millis() - previousUpdate >=  updateInterval) {
    updatesJson(jsonBuffer);
  }
}

void updatesJson(char* jsonBuffer) {
  v = ((val * M) - O);
  adc0 = ads.readADC_SingleEnded(0);
  //adc1 = ads.readADC_SingleEnded(1);
  //adc2 = ads.readADC_SingleEnded(2);
  //adc3 = ads.readADC_SingleEnded(3);
  c1 = (-((A * (adc0)) - B)); // Current reading output in uA channel 1
  //c2 = (-((A * (adc1)) - B)); // Current reading output in uA channel 2
  //c3 = (-((A * (adc2)) - B)); // Current reading output in uA channel 3
  //c4 = (-((A * (adc3)) - B)); // Current reading output in uA channel 4
  Serial.print(v, 4);
  Serial.print(",");
  Serial.println(c1, 3);
  // Format the jsonBuffer as noted above
  strcat(jsonBuffer, "{\"delta_t\":");
  unsigned long deltaT = (millis() - previousUpdate) / 1000;
  size_t lengthT = String(deltaT).length();
  char temp[8];
  String(deltaT).toCharArray(temp, lengthT + 1);
  strcat(jsonBuffer, temp);
  strcat(jsonBuffer, ",");
  strcat(jsonBuffer, "\"field1\":");
  lengthT = String(c1).length();
  String(v).toCharArray(temp, lengthT + 1);      //Data uploaded to ThinkSpeak channel field 1
  strcat(jsonBuffer, temp);
  strcat(jsonBuffer, ",");
  strcat(jsonBuffer, "\"field2\":");
  String(c1).toCharArray(temp, lengthT + 1);       //Data uploaded to ThinkSpeak channel field 2
  strcat(jsonBuffer, temp);
  strcat(jsonBuffer, "},");
  // If posting interval time has reached 16 seconds, update the ThingSpeak channel with data
  if (millis() - lastConnectionTime >=  postingInterval) {
    size_t len = strlen(jsonBuffer);
    jsonBuffer[len - 1] = ']';
    httpRequest(jsonBuffer);
  }
  previousUpdate = millis(); // Update the last update time
}

// Updates the ThingSpeakchannel with data
void httpRequest(char* jsonBuffer) {
  // Format the data buffer as noted above
  char data[1500] = "{\"write_api_key\":\"XVGV1FAKZQXS75P6\",\"updates\":"; //Replace YOUR-CHANNEL-WRITEAPIKEY with your ThingSpeak channel write API key
  strcat(data, jsonBuffer);
  strcat(data, "}");
  // Close any connection before sending a new request
  client.stop();
  String data_length = String(strlen(data) + 1); //Compute the data buffer length
  Serial.println(data);
  // POST data to ThingSpeak
  if (client.connect(server, 80)) {
    client.println("POST /channels/478528/bulk_update.json HTTP/1.1"); //Replace YOUR-CHANNEL-ID with your ThingSpeak channel ID
    client.println("Host: api.thingspeak.com");
    client.println("User-Agent: mw.doc.bulk-update (Arduino ESP8266)");
    client.println("Connection: close");
    client.println("Content-Type: application/json");
    client.println("Content-Length: " + data_length);
    client.println();
    client.println(data);
  }
  jsonBuffer[0] = '[';                      // Reinitialize the jsonBuffer for next batch of data
  jsonBuffer[1] = '\0';
  lastConnectionTime = millis();            // Update the last connection time
}

// Servo-valve control
void chamber1() {
  servo1.attach(17);
  servo2.attach(16);
  servo1.write(150);      // Turn 1st Servo right to 150 degrees (directs to chamber 1)
  servo2.write(70);       // Turn 2nd Servo left to 60 degrees (directs chamber 1 outlet to waste)
  delay(2000);
  servo1.detach();
  servo2.detach();
}
void chamber2() {
  servo1.attach(17);
  servo3.attach(4);
  servo1.write(40);       // Turn 1st Servo left to 40 degrees (directs to chamber 2)
  servo3.write(160);      // Turn 3rd Servo left to 160 degrees (directs chamber 2 outlet to detection)
  delay(2000);
  servo1.detach();
  servo3.detach();
}
void chamber3() {
  servo1.attach(17);
  servo2.attach(16);
  servo1.write(150);      // Turn 1st Servo right to 150 degrees (directs to chamber 1)
  servo2.write(180);      // Turn 2nd Servo left to 60 degrees (directs chamber 1 outlet to detection)
  delay(2000);
  servo1.detach();
  servo2.detach();
}
void chamber4() {
  servo1.attach(17);
  servo3.attach(4);
  servo1.write(40);       // Turn 1st Servo left to 40 degrees (directs to chamber 2)
  servo3.write(50);       // Turn 3rd Servo left to 50 degrees (directs chamber 2 outlet to waste)
  delay(2000);
  servo1.detach();
  servo3.detach();
}
