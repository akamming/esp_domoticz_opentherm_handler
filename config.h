// Use this file to store all application constants and default config values (can be overruled with webgui)

//application constants
#define CONFIGFILE  "/config.json"                // name of the config file on the SPIFFS image
const int MQTTTimeoutInMillis = 30 * 1000;              // if no command was sent in this period, the program will assume the MQTT client is no longer there
const int HTTPTimeoutInMillis = 10 * 1000;              // if no command was sent during this periode, the program will assume HTTP control stopped.
const int heartbeatTickInMillis = 1000;                 // has to be max 1000, Opentherm assumes a command is sent to opentherm at least once per second
const int MQTTDiscoveryHeartbeatInMillis = 10*60*1000;  // Send discovery messages every 10 minutes to make sure HA or Domoticz can use the devices after restart.
float ThermostatTemperatureCalibration=0;         // set to a differenct value to zero is DS18B20 give a too high or low reading
int httpport=80;                                  // port for http interface
String host = "domesphelper";                     // mdns hostname
String mqttautodiscoverytopic = "homeassistant";  // On what topic are the MQTT autodiscovery messages expected

// Default Config values
int inPin = 4;                                    // pin number for opentherm adapter connection, 2 for Arduino, 4 for ESP8266 (D2), 21 for ESP32
int outPin = 5;                                   // pin number for opentherm adapter connection, 3 for Arduino, 5 for ESP8266 (D1), 22 for ESP32
int OneWireBus = 14;                              // Data wire is connected to 14 pin on the OpenTherm Shield (Temperature sensor)
bool usemqtt = false;                              // Set to True if you want to use MQTT
bool mqttpersistence = false;                     // Set to True if you want persistence messages (true ensures devices are recognized right away after reboot of home automation tool, but then you cannot delete the devices anymore (will keep reappearing)
                                                  //             false causes devices only to work after discovery messages are sent (see MQTT descovery heatbeat)
String mqttserver = "";                           // MQTT Server Adress
int mqttport = 1883;                              // MQTT Server Port
bool usemqttauthentication = false;                // Use MQTT Authentication
String mqttuser = "";                             // MQTT Username
String mqttpass = "";                             // MQTT Password
String mqtttemptopic = "";                        // MQTT Topic which contains the current temperature
