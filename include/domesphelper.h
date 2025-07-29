// Default configuration constants for main.cpp
const bool   DEFAULT_USEMQTT = false;
const bool   DEFAULT_USEMQTTAUTH = false;
const char*  DEFAULT_MQTTSERVER = "";
const int    DEFAULT_MQTTPORT = 1883;
const char*  DEFAULT_MQTTUSER = "";
const char*  DEFAULT_MQTTPASS = "";
const bool   DEFAULT_MQTTPERSISTENCE = false;
const char*  DEFAULT_MQTTTEMP_TOPIC = "";
const char*  DEFAULT_MQTTOUTSIDETEMP_TOPIC = "";
const int    DEFAULT_INPIN = 4;
const int    DEFAULT_OUTPIN = 5;
const int    DEFAULT_ONEWIREBUS = 14;
const bool   DEFAULT_DEBUG = true;
const int    DEFAULT_KP = 30;
const float  DEFAULT_KI = 0.01f;
const float  DEFAULT_KD = 2.5f;
const int    DEFAULT_MINBOILERTEMP = 10;
const int    DEFAULT_MAXBOILERTEMP = 50;
const int    DEFAULT_MINIMUMTEMPDIFFERENCE = 3;
const int    DEFAULT_FROSTPROTECTIONSETPOINT = 6;
const int    DEFAULT_BOILERTEMPATPLUS20 = 20;
const int    DEFAULT_BOILERTEMPATMINUS10 = 50;
const char*  DEFAULT_CURVATURE_STRING = "small";
const int    DEFAULT_SWITCHHEATINGOFFAT = 19;
const int    DEFAULT_REFERENCEROOMCOMPENSATION = 3;
const char*  DEFAULT_CLIMATE_MODE = "off";
const int    DEFAULT_CLIMATE_SETPOINT = 20;
const bool   DEFAULT_WEATHERDEPENDENTMODE = false;
const bool   DEFAULT_HOLIDAYMODE = false;
// Use this file to store all application constants and default config values (can be overruled with webgui)

// device names for mqtt autodiscovery
const char Boiler_Temperature_Name[] = "Boiler_Temperature";   
const char DHW_Temperature_Name[] = "DHW_Temperature";   
const char Return_Temperature_Name[] = "Return_Temperature";   
const char Thermostat_Temperature_Name[] = "Thermostat_Temperature";   
const char Outside_Temperature_Name[] = "Outside_Temperature";   
const char OT_Outside_Temperature_Name[] = "OT_Outside_Temperature";   
const char FlameActive_Name[] = "FlameActive";   
const char FaultActive_Name[] = "FaultActive";   
const char DiagnosticActive_Name[] = "DiagnosticActive";   
const char CoolingActive_Name[] = "CoolingActive";   
const char CentralHeatingActive_Name[] = "CentralHeatingActive";   
const char HotWaterActive_Name[] = "HotWaterActive";  
const char FrostProtectionActive_Name[] = "FrostProtectionActive";
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
const char Holiday_Mode_Name[] = "HolidayMode";
const char MinBoilerTemp_Name[] = "MinimumBoilerTemp";
const char MaxBoilerTemp_Name[] = "MaximumBoilerTemp";
const char MinimumTempDifference_Name[] = "MinimumTempDifference";              // Minum tempdiffernce before heating or cooling switches on
const char FrostProtectionSetPoint_Name[] = "FrostProtectionSetPoint";          // Automatically heat when in frostprotection and below this temperature
const char BoilerTempAtPlus20_Name[] = "BoilerTempAtPlus20";                    // for calculating when in weather dependent mode
const char BoilerTempAtMinus10_Name[] = "BoilerTempAtMinus10";                  // for calculating when in weather dependent mode
const char Curvature_Name[]="Curvature";                           // 0=none, 10=small, 20=medium, 30=large, 40=Extra Large
const char SwitchHeatingOffAt_Name[] = "SwitchHeatingOffAt";                    // Automatic switch off when in weather dependent mode when outside temp too high
const char ReferenceRoomCompensation_Name[] = "ReferenceRoomCompensation";           // In weather dependent mode: Correct with this number per degree celcius difference (air temperature - setpoint) 
const char KP_Name[] = "KP";
const char KI_Name[] = "KI";
const char KD_Name[] = "KD";
const char P_Name[] = "P";
const char I_Name[] = "I";
const char D_Name[] = "D";
const char MQTT_TempTopic_Name[] = "MQTTTemperatureTopic";
const char MQTT_OutsideTempTopic_Name[] = "MQTTOutsideTemperatureTopic";
const char Debug_Name[] = "DebugToMQTT";
const char Error_Name[] = "Error";

//application constants
#define CONFIGFILE  "/config.json"                // name of the config file on the SPIFFS image
const int MQTTConnectTimeoutInMillis = 3*1000;     // the time to reconnect if disconnect occurred 
const int MQTTTimeoutInMillis = 15 * 1000;              // if no command was sent in this period, the program will assume the MQTT client is no longer there
const int HTTPTimeoutInMillis = 10 * 1000;              // if no command was sent during this periode, the program will assume HTTP control stopped.
const int heartbeatTickInMillis = 1000;                 // has to be max 1000, Opentherm assumes a command is sent to opentherm at least once per second
const int MQTTDiscoveryHeartbeatInMillis = 10*60*1000;  // Send discovery messages every 10 minutes to make sure HA or Domoticz can use the devices after restart.
const int ClimateHeartbeatInMillis = 1000;        // Interval to do calculate new PID values when in CLimate mode
const int MQTTTemperatureTimeoutInMillis = 60*60*1000; // no climate mode calculations when temperature reading is older than this value
float ThermostatTemperatureCalibration=0;         // set to a differenct value to zero is DS18B20 give a too high or low reading
int httpport=80;                                  // port for http interface
String host = "domesphelper";                     // mdns hostname
String mqttautodiscoverytopic = "homeassistant";  // On what topic are the MQTT autodiscovery messages expected
const int ConfigSaveDelay=5000;                   // Number of milliseconds delay for when the climate settings are changed. (to prevent saving for every clock on the thermostat device in some UI)

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
String mqttoutsidetemptopic = "";                        // MQTT Topic which contains the current outside temperature
String domoticzoutputtopic="domoticz/out";        // Topic where to listen to for domoticz sensors

// Default values Climate Decice 
float climate_SetPoint = 20;                      // default setpoint value for climate device
String climate_Mode = "off";                      // Default mode for climate device
bool Weather_Dependent_Mode = false;              // Default setting for weather dependent mode
bool Holiday_Mode = false;                        // Default setting for holiday mode

// PID parameters
float KP = 30; // Proportional gain: This is the Multiplier of  the error (e.g. KP=30: 1 degree error will result in 30 degrees change of the pid value)
float KI = 0.01; // Integral Gain: This is the multiplier of the error (e.g. KI=0.01: a 1 degree difference for 10 seconds will result in 0.1 degree change of the integral error (KI*error*duration))
float KD = 2.5; // Derative Gain:  Correction per every Delta K per Hour (e.g. KD=2.5: if the temp rises with 1 K per Hour, the PID will be lowered with 2.5 degrees)

//Boiler settings
float MaxBoilerTemp = 50;                     // Max boiler temp when in climate mode
float MinBoilerTemp = 10;                     // Min Boiler temp when in climate mode
float minimumTempDifference=3;              // Minum tempdiffernce before heating or cooling switches on
float FrostProtectionSetPoint = 6;            // Automatically heat when in frostprotection and below this temperature
float BoilerTempAtPlus20 = 20;                // for calculating when in weather dependent mode
float BoilerTempAtMinus10 = 50;               // for calculating when in weather dependent mode
int Curvature=10;                           // 0=none, 10=small, 20=medium, 30=large, 40=Extra Large
float SwitchHeatingOffAt = 19;                // Automatic switch off when in weather dependent mode when outside temp too high
float ReferenceRoomCompensation = 3;          // In weather dependent mode: Correct with this number per degree celcius difference (air temperature - setpoint) 

// uploadform
const char HTTP_UPLOAD_FORM[] PROGMEM = 
    "<form method=\"post\" enctype=\"multipart/form-data\">"
    "<input type=\"file\" name=\"name\">"
    "<input class=\"button\" type=\"submit\" value=\"Upload\">"
    "</form><br/>"
    "<a href=\"/\">Back to main page</a>";


// Function prototypes

void Debug(String text);
void Error(String text);
void IRAM_ATTR handleInterrupt();
void SendHTTP(String command, String result);
void handleResetWifiCredentials();
void handleGetSensors();
void handleCommand();
void handleGetInfo();
void sendUploadForm();
void handleFileUpload();
void handleGetConfig();
void handleSaveConfig();
void handleRemoveConfig();
void handleReset();
bool endsWith(const char* what, const char* withwhat);
bool serveFile(const char url[]);
void handleNotFound();
String getCurvatureStringFromInt(int i);
int getCurvatureIntFromString(String value);
void readConfig();
void SaveConfig();
float getOutsideTemperature();
float getDHWFlowrate();
void CommunicateSetpoint(const char* setpointName, float setpointValue, float *mqttValue);
void CommunicateNumber(const char* numberName, float Value, float *mqttValue, float tolerance);
void CommunicateNumberSensor(const char* numberName, float Value, float *mqttValue, float tolerance);
void CommunicateText(const char* TextName, String Value, String *mqttValue);
void CommunicateSteeringVarsToMQTT();
void resetI();
void resetD();
void InitPID();
void UpdatePID(float setpoint, float temperature);
float GetBoilerSetpointFromOutsideTemperature(float CurrentInsideTemperature, float CurrentOutsideTemperature);
void handleClimateProgram();
void handleOpenTherm();
String CommandTopic(const char* DeviceName);
String SetpointCommandTopic(const char* DeviceName);
String NumberCommandTopic(const char* DeviceName);
void LogMQTT(const char* topic, const char* payloadstr, const char* length, const char* logtext);
bool HandleClimateMode(const char* mode);
const char* SetpointIntToString(int value);
void SetMQTTTemperature(float value);
void SetMQTTOutsideTemperature(float value);
bool HandleBoilerMode(const char* mode);
bool HandleDHWMode(const char* mode);
void DelayedSaveConfig();
bool HandleSwitch(bool *Switch, bool *mqtt_switch, const char* mode);
bool handleClimateSetpoint(float setpoint);
void MQTTcallback(char* topic, byte* payload, unsigned int length);
void SubScribeToDomoticz();
void reconnect();
void addDeviceToJson(JsonDocument *json);
void PublishMQTTDimmer(const char* uniquename);
void UpdateMQTTDimmer(const char* uniquename, bool Value, float Mod);
void PublishMQTTSwitch(const char* uniquename);
void UpdateMQTTSwitch(const char* uniquename, bool Value);
void PublishMQTTBinarySensor(const char* uniquename, const char* deviceclass);
void UpdateMQTTBinarySensor(const char* uniquename, bool Value);
void PublishMQTTTemperatureSensor(const char* uniquename);
void UpdateMQTTTemperatureSensor(const char* uniquename, float temperature);
void PublishMQTTNumberSensor(const char* uniquename);
void UpdateMQTTNumberSensor(const char* uniquename, float value);
void PublishMQTTPressureSensor(const char* uniquename);
void UpdateMQTTPressureSensor(const char* uniquename, float pressure);
void PublishMQTTPercentageSensor(const char* uniquename);
void UpdateMQTTPercentageSensor(const char* uniquename, float percentage);
void PublishMQTTFaultCodeSensor(const char* uniquename);
void UpdateMQTTFaultCodeSensor(const char* uniquename, unsigned char FaultCode);
void PublishMQTTSetpoint(const char* uniquename, int mintemp, int maxtemp, bool includeCooling);
void UpdateMQTTSetpointTemperature(const char* uniquename, float value);
void PublishMQTTNumber(const char* uniquename, int min, int max, float step, bool isSlider);
void UpdateMQTTNumber(const char* uniquename, float value);
void PublishMQTTText(const char* uniquename);
void UpdateMQTTText(const char* uniquename, const char* value);
void PublishMQTTCurvatureSelect(const char* uniquename);
void UpdateMQTTCurvatureSelect(const char* uniquename, int value);
void UpdateMQTTBoilerSetpointMode();
void UpdateMQTTSetpointMode(const char* uniquename, int value);
void UpdateMQTTSetpoint(const char* uniquename, float temperature);
void updateTime();
void PublishAllMQTTSensors();