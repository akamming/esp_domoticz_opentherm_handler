// Use this file to store all application constants and default config values (can be overruled with webgui)

// device names for mqtt autodiscovery
const char Boiler_Temperature_Name[] = "Boiler_Temperature";   
const char DHW_Temperature_Name[] = "DHW_Temperature";   
const char Return_Temperature_Name[] = "Return_Temperature";   
const char Thermostat_Temperature_Name[] = "Thermostat_Temperature";   
const char Outside_Temperature_Name[] = "Outside_Temperature";   
const char FlameActive_Name[] = "FlameActive";   
const char FaultActive_Name[] = "FaultActive";   
const char DiagnosticActive_Name[] = "DiagnosticActive";   
const char CoolingActive_Name[] = "CoolingActive";   
const char CentralHeatingActive_Name[] = "CentralHeatingActive";   
const char HotWaterActive_Name[] = "HotWaterActive";  
const char EnableCooling_Name[] = "EnableCooling";   
const char EnableCentralHeating_Name[] = "EnableCentralHeating";   
const char EnableHotWater_Name[] = "EnableHotWater";   
const char Boiler_Setpoint_Name[] = "Boiler_Setpoint";
const char DHW_Setpoint_Name[] = "DHW_Setpoint";
const char Modulation_Name[] = "Modulation";
const char Pressure_Name[] = "Pressure";
const char FaultCode_Name[] = "Faultcode";
const char Climate_Name[] = "Climate";
const char Weather_Dependent_Mode_Name[] = "WeatherDependentMode";

//application constants
#define CONFIGFILE  "/config.json"                // name of the config file on the SPIFFS image
const int MQTTTimeoutInMillis = 15 * 1000;              // if no command was sent in this period, the program will assume the MQTT client is no longer there
const int HTTPTimeoutInMillis = 10 * 1000;              // if no command was sent during this periode, the program will assume HTTP control stopped.
const int heartbeatTickInMillis = 1000;                 // has to be max 1000, Opentherm assumes a command is sent to opentherm at least once per second
const int MQTTDiscoveryHeartbeatInMillis = 10*60*1000;  // Send discovery messages every 10 minutes to make sure HA or Domoticz can use the devices after restart.
const int ClimateHeartbeatInMillis = 1000;        // Interval to do calculate new PID values when in CLimate mode
float ThermostatTemperatureCalibration=0;         // set to a differenct value to zero is DS18B20 give a too high or low reading
int httpport=80;                                  // port for http interface
String host = "domesphelper";                     // mdns hostname
String mqttautodiscoverytopic = "homeassistant";  // On what topic are the MQTT autodiscovery messages expected
const int ConfigSaveDelay=3000;                   // Number of milliseconds delay for when the climate settings are changed. (to prevent saving for every clock on the thermostat device in some UI)

// Constants for SetpointModes
const int OFF = 0;
const int HEAT = 1;
const int COOL = 11;
const int AUTO = 21;

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

// Default values Climate Decice 
float climate_SetPoint = 20;                      // default setpoint value for climate device
String climate_Mode = "off";                      // Default mode for climate device
bool Weather_Dependent_Mode = false;              // Default setting for weather dependent mode

// PID parameters
float KP = 30; // Proportaional gain: This is the Multiplier of  the error (e.g. KP=30: 1 degree error will result in 30 degrees change of the pid value)
float KI = 0.01; // Integral Gain: This is the multiplier of the error (e.g. KI=0.01: a 1 degree difference for 10 seconds will result in 0.1 degree change of the integral error (KI*error*duration))
float KD = 2.5; // Derative Gain:  Correction per every Delta K per Hour (e.g. KD=2.5: if the temp rises with 1 K per Hour, the PID will be lowered with 2.5 degrees)

//Boiler settings
int MaxBoilerTemp = 50;                     // Max boiler temp when in climate mode
int MinBoilerTemp = 10;                     // Min Boiler temp when in climate mode
float minimumTempDifference=3;              // Minum tempdiffernce before heating or cooling switches on
int FrostProtectionSetPoint = 6;            // Automatically heat when in frostprotection and below this temperature
int BoilerTempAtPlus20 = 20;                // for calculating when in weather dependent mode
int BoilerTempAtMinus10 = 50;               // for calculating when in weather dependent mode
int Curvature=10;                           // 0=none, 10=small, 20=medium, 30=large, 40=Extra Large
int SwitchHeatingOffAt = 19;                // Automatic switch off when in weather dependent mode when outside temp too high
int ReferenceRoomCompensation = 3;          // In weather dependent mode: Correct with this number per degree celcius difference (air temperature - setpoint) 
