// Use this file to store all of the private credentials 
// and connection details

const int inPin = 4;                                // pin number for opentherm adapter connection, 2 for Arduino, 4 for ESP8266 (D2), 21 for ESP32
const int outPin = 5;                               // pin number for opentherm adapter connection, 3 for Arduino, 5 for ESP8266 (D1), 22 for ESP32
const int OneWireBus = 14;                          // Data wire is connected to 14 pin on the OpenTherm Shield (Temperature sensor)
const int domoticzTimeoutInMillis = 30 * 1000;      // if no command was sent in this period, the thermostat will assume domoticz is nog longer there 
const int heartbeatTickInMillis = 1000;             // has to be max 1000, Opentherm assumes a command is sent to opentherm at least once per second
const int MQTTDiscoveryHeartbeatInMillis = 10*60*1000;  // Send discovery messages every 10 minutes to make sure HA or Domoticz can use the devices after restart.
const char host[] = "domesphelper";                 // mdns hostname
const bool usemqtt = false;                         // Set to True if you want to use MQTT
const bool mqttpersistence = false;                  // Set to True if you want persistence messages 
const char mqttserver[] = "ip adress";              // MQTT Server Adress
const int mqttport = 1883;                           // MQTT Server Port
const char mqttuser[] = "user";                     // MQTT Username
const char mqttpass[] = "pass";                     // MQTT Password
const char mqttautodiscoverytopic[] = "homeassistant";  // On what topic are the MQTT autodiscovery messages expected
const float ThermostatTemperatureCalibration=0;     // set to a differenct value to zero is DS18B20 give a too high or low reading
const int httpport=80;                              // port for http interface
