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
const bool   DEFAULT_DEBUG = true;
const bool   DEFAULT_INFOTOMQTT = true;
const bool   DEFAULT_ENABLE_LOG_TO_SPIFFS = true;
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
const bool   DEFAULT_BIDIRECTIONAL_REFERENCEROOMCOMPENSATION = false;
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
const char Boiler_Setpoint_Temperature_Name[] = "Boiler_Setpoint_Temperature";   
const char FlameActive_Name[] = "FlameActive";   
const char FaultActive_Name[] = "FaultActive";   
const char DiagnosticActive_Name[] = "DiagnosticActive";   
const char CoolingActive_Name[] = "CoolingActive";   
const char CentralHeatingActive_Name[] = "CentralHeatingActive";   
const char HotWaterActive_Name[] = "HotWaterActive";  
const char FrostProtectionActive_Name[] = "FrostProtectionActive";
const char EnableHotWater_Name[] = "EnableHotWater";   
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
const char BidirectionalReferenceRoomCompensation_Name[] = "BidirectionalReferenceRoomCompensation"; // Enable bidirectional reference room compensation
const char KP_Name[] = "KP";
const char KI_Name[] = "KI";
const char KD_Name[] = "KD";
const char P_Name[] = "P";
const char I_Name[] = "I";
const char D_Name[] = "D";
const char MQTT_TempTopic_Name[] = "MQTTTemperatureTopic";
const char MQTT_OutsideTempTopic_Name[] = "MQTTOutsideTemperatureTopic";
const char Debug_Name[] = "DebugToMQTT";
const char Log_Name[] = "Log";
const char Info_Name[] = "InfoToMQTT";
const char EnableLogToSPIFFS_Name[] = "EnableLogToSPIFFS";
const char IP_Address_Name[] = "Device_IP_Address";
const char WiFi_RSSI_Name[] = "WiFi_RSSI";
const char Uptime_Name[] = "Uptime";
const char Reset_Device_Name[] = "Reset_Device";
const char Free_Heap_Name[] = "Free_Heap";
                    
//application constants
#define CONFIGFILE  "/config.json"                // name of the config file on the SPIFFS image
const unsigned long MQTTConnectTimeoutInMillis = 3UL*1000UL;     // the time to reconnect if disconnect occurred 
const unsigned long heartbeatTickInMillis = 1000UL;                 // has to be max 1000, Opentherm assumes a command is sent to opentherm at least once per second
const unsigned long MQTTDiscoveryHeartbeatInMillis = 60UL*60UL*1000UL;  // Send discovery messages every 10 minutes to make sure HA or Domoticz can use the devices after restart.
const unsigned long ClimateHeartbeatInMillis = 1000UL;        // Interval to do calculate new PID values when in CLimate mode
const unsigned long MQTTTemperatureTimeoutInMillis = 60UL*60UL*1000UL; // no climate mode calculations when temperature reading is older than this v
const unsigned long HysteresisDelayMillis = 10UL * 60UL * 1000UL; // Delay in milliseconds before turning off heating/cooling to prevent chattering
float ThermostatTemperatureCalibration=0;         // set to a differenct value to zero is DS18B20 give a too high or low reading
int httpport=80;                                  // port for http interface
String host = "domesphelper";                     // mdns hostname
String mqttautodiscoverytopic = "homeassistant";  // On what topic are the MQTT autodiscovery messages expected
const unsigned long ConfigSaveDelay=5000UL;                   // Number of milliseconds delay for when the climate settings are changed. (to prevent saving for every clock on the thermostat device in some UI)

// Constants for SetpointModes
const int OFF = 0;
const int HEAT = 1;
const int COOL = 11;
const int AUTO = 21;

// Default Config values
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
bool Weather_Dependent_Mode = false;                        // Default setting for weather dependent mode
bool Holiday_Mode = false;                        // Default setting for holiday mode
bool enableLogToSPIFFS = true;                          // Default setting for logging to SPIFFS

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
bool BidirectionalReferenceRoomCompensation = false; // If true, compensate in both directions; if false, only increase when room temp < setpoint

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
bool UpdateMQTTSwitch(const char* uniquename, bool& currentValue, bool newValue, bool force = false);
void PublishMQTTBinarySensor(const char* uniquename, const char* deviceclass);
void UpdateMQTTBinarySensor(const char* uniquename, bool& currentValue, bool newValue, bool force);
void PublishMQTTRSSISensor(const char* uniquename);
void UpdateMQTTRSSISensor(const char* uniquename, int& currentValue, int newValue, bool force = false);
void PublishMQTTUptimeSensor(const char* uniquename);
void UpdateMQTTUptimeSensor(const char* uniquename, unsigned long& currentValue, unsigned long newValue, bool force = false);
bool UpdateMQTTNumberSensor(const char* uniquename, float& currentValue, float newValue, float tolerance, bool force = false);
void PublishMQTTPercentageSensor(const char* uniquename);
void UpdateMQTTPercentageSensor(const char* uniquename, float& currentValue, float newValue, bool force);
void PublishMQTTFaultCodeSensor(const char* uniquename);
void UpdateMQTTFaultCodeSensor(const char* uniquename, unsigned char& currentValue, unsigned char newValue, bool force);
void PublishMQTTSetpoint(const char* uniquename, int mintemp, int maxtemp, bool includeCooling);
void UpdateMQTTSetpointTemperature(const char* uniquename, float value, bool force = false);
void PublishMQTTNumber(const char* uniquename, int min, int max, float step, bool isSlider);
bool UpdateMQTTNumber(const char* uniquename, float& currentValue, float newValue, float tolerance, bool force = false);
void PublishMQTTText(const char* uniquename);
void UpdateMQTTText(const char* uniquename, String& currentValue, String newValue, bool force = false);
void PublishMQTTTextSensor(const char* uniquename);
bool UpdateMQTTTextSensor(const char* uniquename, const char* value, bool force = false);
void PublishMQTTCurvatureSelect(const char* uniquename);
void UpdateMQTTCurvatureSelect(const char* uniquename, int value, bool force = false);
void UpdateMQTTSetpointMode(const char* uniquename, int value, bool force = false);
bool UpdateMQTTSetpoint(const char* uniquename, float& currentValue, float newValue, bool force = false);
void UpdateClimateSetpointMode(bool force = false);
void updateTime();
bool PublishAllMQTTSensors();
String getUptimeString();
unsigned long getUptimeSeconds();