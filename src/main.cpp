/*
Domoticz_openterm_handler, based on  OpenTherm Master Communication Example from Ihor Melnyk
By: Arnold Kamminga
Date: May 14th , 2021
Uses the WiFiManager from tzapu to configure wifi connection
Uses the OpenTherm library from ihormelyk to communicate with boiler
Open serial monitor at 115200 baud to see output.
Hardware Connections (OpenTherm Adapter (http://ihormelnyk.com/pages/OpenTherm) to Arduino/ESP8266):
-OT1/OT2 = Boiler X1/X2
-VCC = 5V or 3.3V
-GND = GND
-IN  = Arduino (3) / ESP8266 (5g) Input Pin
*/


#include <Arduino.h>              // duh...
#include <OpenTherm.h>            //https://github.com/ihormelnyk/opentherm_library
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <ESP8266WebServer.h>     //Local Webserver for receiving http commands
#include <ESP8266mDNS.h>          // To respond on hostname as well
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <OneWire.h>              // One Wire bus  by Jim Studt
#include <DallasTemperature.h>    // temperature sensors by mile burton
#include <ArduinoOTA.h>           // OTA updates
#include <ArduinoJson.h>          // make JSON payloads
#define MQTT_LIBRARY 0        // Set to 0 for PubSubClient, 1 for ArduinoMqttClient, 2 for AsyncMQTTClient
#if MQTT_LIBRARY == 0
#include <PubSubClient.h>
#elif MQTT_LIBRARY == 1
#include <ArduinoMqttClient.h>
#else
#include <ESPAsyncTCP.h>
#include <AsyncMqttClient.h>
#endif
#include "domesphelper.h"               // Set Configuration and default constants
#include <LittleFS.h>             // Filesystem
#include <NTPClient.h>            // for NTP Client
#include <WiFiUdp.h>              // is needed by NTP client
#include <vector>                 // for log buffer

#define NUMBEROFMEASUREMENTS 10 // number of measurements over which to average for outside temp
#define MQTT_QOS_CONFIG 1
#define MQTT_QOS_STATE 0



// MQTT Async connect parameters
String mqttClientId = "";

// Generic MQTT helper functions
WiFiClient espClient;
#if MQTT_LIBRARY == 0
PubSubClient client(espClient);
#elif MQTT_LIBRARY == 1
MqttClient client(espClient);
#else
AsyncMqttClient client;
#endif

bool mqttConnected() {
  #if MQTT_LIBRARY == 2 // async library
    return client.connected();
  #else // other libraries
    return client.connected();
  #endif
}

int mqttConnectError() {
  #if MQTT_LIBRARY == 0 // PubSubClient
    return client.state();
  #elif MQTT_LIBRARY == 1 // ArduinoMqttClient
    return client.connectError();
  #else // AsyncMQTTClient
    return client.connected() ? 0 : -1; // AsyncMQTTClient doesn't have a direct error code
  #endif
}

void mqttSubscribe(const char* topic, int qos = 0) {
  #if MQTT_LIBRARY == 0 // PubSubClient
    client.subscribe(topic);
  #elif MQTT_LIBRARY == 1 // ArduinoMqttClient
    client.subscribe(topic, qos);
  #else // AsyncMQTTClient
    client.subscribe(topic, qos);
  #endif
}

void mqttPoll() {
  #if MQTT_LIBRARY == 0 // PubSubClient
    client.loop();
  #elif MQTT_LIBRARY == 1 // ArduinoMqttClient
    client.poll();
  #endif
  // For AsyncMQTTClient, no loop needed
}

bool mqttPublish(const char* topic, const char* payload, bool retained, int qos = 0) {
  #if MQTT_LIBRARY == 0 // PubSubClient
    return client.publish(topic, payload, retained);
  #elif MQTT_LIBRARY == 1 // ArduinoMqttClient
    client.beginMessage(topic, retained, qos);
    client.print(payload);
    return client.endMessage() != 0;
  #else // AsyncMQTTClient
    return client.publish(topic, qos, retained, payload);
  #endif
}

void mqttSetConnectionTimeout(int timeout) {
  #if MQTT_LIBRARY == 1 // ArduinoMqttClient
    client.setConnectionTimeout(timeout);
  #elif MQTT_LIBRARY == 2 // AsyncMQTTClient
    // AsyncMQTTClient uses setKeepAlive
  #endif
}

void mqttSetKeepAliveInterval(int interval) {
  #if MQTT_LIBRARY == 0 // PubSubClient
    // PubSubClient uses setKeepAlive in connect
  #elif MQTT_LIBRARY == 1 // ArduinoMqttClient
    client.setKeepAliveInterval(interval);
  #else // AsyncMQTTClient
    client.setKeepAlive(interval);
  #endif
}

void mqttSetId(const char* clientId) {
  #if MQTT_LIBRARY == 0 // PubSubClient
    // PubSubClient sets ID in connect
  #elif MQTT_LIBRARY == 1 // ArduinoMqttClient
    client.setId(clientId);
  #else // AsyncMQTTClient
    client.setClientId(clientId);
  #endif
}

#if MQTT_LIBRARY != 0 // pubsubclient does not support mqttset username/password
void mqttSetUsernamePassword(const char* user, const char* pass) {
  #if MQTT_LIBRARY == 1 // ArduinoMqttClient
    client.setUsernamePassword(user, pass);
  #else // AsyncMQTTClient
    client.setCredentials(user, pass);
  #endif
}
#endif

bool mqttConnect(const char* server, int port) {
  #if MQTT_LIBRARY == 0 // PubSubClient
    client.setServer(server, port);
    client.setBufferSize(2048); // For longer discoverymessages
    String willTopic = host + "/status";
    bool connected;
    if (usemqttauthentication) {
      connected = client.connect(host.c_str(), mqttuser.c_str(), mqttpass.c_str(), willTopic.c_str(), 1, true, "offline");
    } else {
      connected = client.connect(host.c_str(), NULL, NULL, willTopic.c_str(), 1, true, "offline");
    }
    if (connected) {
      mqttPublish(willTopic.c_str(), "online", true, 1);
    }
    return connected;
  #elif MQTT_LIBRARY == 1 // ArduinoMqttClient
    return client.connect(server, port);
  #else // AsyncMQTTClient
    client.setServer(server, port);
    client.setWill(host.c_str(), 1, true, "offline");
    client.setClientId(mqttClientId.c_str());
    client.setKeepAlive(300);
    if (usemqttauthentication) {
      client.setCredentials(mqttuser.c_str(), mqttpass.c_str());
    }
    client.connect();
    return true; // Async connect started
  #endif
}

void mqttUnsubscribe(const char* topic) {
  #if MQTT_LIBRARY == 0 // PubSubClient
    client.unsubscribe(topic);
  #elif MQTT_LIBRARY == 1 // ArduinoMqttClient
    client.unsubscribe(topic);
  #else // AsyncMQTTClient
    client.unsubscribe(topic);
  #endif
}

void mqttOnMessage(void (*callback)(int)) {
  #if MQTT_LIBRARY == 1 // ArduinoMqttClient
    client.onMessage(callback);
  #endif
  // For AsyncMQTTClient, set in setup
}

#if MQTT_LIBRARY == 1 // ArduinoMqttClient
void onMessageCallback(int messageSize) {
  // Read topic and payload
  String topic = client.messageTopic();
  String payload = client.readString();

  // Convert to old format for compatibility (optional, if you don't want to adjust the rest of the code)
  char topicChar[topic.length() + 1];
  topic.toCharArray(topicChar, topic.length() + 1);
  char payloadChar[payload.length() + 1];
  payload.toCharArray(payloadChar, payload.length() + 1);

  // Call the old logic (adjust if necessary)
  MQTTcallback(topicChar, (byte*)payloadChar, payload.length());
}
#endif



// For PubSubClient callback
#if MQTT_LIBRARY == 0 // PubSubClient
void mqttSetCallback(void (*callback)(char*, byte*, unsigned int)) {
  client.setCallback(callback);
}
#endif



// vars to manage boiler
bool enableCentralHeating = false; // define this as false, so we can set it to true in the config
bool enableHotWater = true;
bool enableCooling = false;
float boiler_SetPoint = 0;
float dhw_SetPoint = 65;
float P=0;
float I=0;
float D=0;  

// Current Temp on Thermostat
float currentTemperature = 0;

// Current Outside Temp for program Logic
float outside_Temperature = 0;

// Current Temp on mqtt
float mqttTemperature = 99;
float mqttOutsideTemperature = 99;

// return values from boiler
float dhw_Temperature = 0;
float boiler_Temperature = 0;
float return_Temperature = 0;
float OT_outside_Temperature = 0;
float modulation = 0;
float pressure = 0;
float flowrate = 0;
OpenThermResponseStatus responseStatus;
bool CentralHeating = false;
bool HotWater = false;
bool Cooling = false;
bool Flame = false;
bool Fault = false;
bool Diagnostic = false;
unsigned char FaultCode=65;
bool FrostProtectionActive;

// uptime
unsigned long previousMillis = 0; // remember last millis value
int y = 0 ;
int d = 0;
int h = 0;
int m = 0;
int s = 0;
int ms = 0;

// reported vars to mqtt, make sure all are initialized on unexpected values, so they are sent the 1st time
float mqtt_boiler_Temperature=100;
float mqtt_dhw_Temperature=100;
float mqtt_return_Temperature=100;
float mqtt_outside_Temperature=100;
float mqtt_OT_outside_Temperature=100;
float mqtt_currentTemperature=100;
float mqtt_mqttTemperature=100;
float mqtt_boiler_setpoint=0;
bool mqtt_CentralHeating=true;
bool mqtt_HotWater=true;
bool mqtt_Cooling=true;
bool mqtt_Flame=true;
bool mqtt_Fault=true;
bool mqtt_Diagnostic=true;
bool mqtt_FrostProtectionActive=true;
bool mqtt_Weather_Dependent_Mode=true;
bool mqtt_Holiday_Mode=true;
float mqtt_modulation=101;
bool mqtt_enable_HotWater=false;
float mqtt_climate_setpoint=1;
float mqtt_dhw_setpoint=0;
float mqtt_pressure=3; 
unsigned char mqtt_FaultCode=65;
String mqtt_climate_Mode = "abcd"; 
float mqtt_minboilertemp=0;
float mqtt_maxboilertemp=0;
float mqtt_minimumTempDifference=99;              // Minum tempdiffernce before heating or cooling switches on
float mqtt_FrostProtectionSetPoint =99;            // Automatically heat when in frostprotection and below this temperature
float mqtt_BoilerTempAtPlus20 = 99;                // for calculating when in weather dependent mode
float mqtt_BoilerTempAtMinus10 = 99;               // for calculating when in weather dependent mode
float mqtt_Curvature=99;                           // 0=none, 10=small, 20=medium, 30=large, 40=Extra Large
float mqtt_SwitchHeatingOffAt = 99;                // Automatic switch off when in weather dependent mode when outside temp too high
float mqtt_ReferenceRoomCompensation = 99;          // In weather dependent mode: Correct with this number per degree celsius difference (air temperature - setpoint) 
float mqtt_kp=99;
float mqtt_ki=99;
float mqtt_kd=99;
float mqtt_p=99;
float mqtt_i=99;
float mqtt_d=99;
float mqtt_wifi_rssi=0;
unsigned long mqtt_uptime=0;
float mqtt_free_heap=0;
unsigned long last_heap_publish=0;
String mqtt_mqtttemptopic="xyzxyz";
String mqtt_mqttoutsidetemptopic="xyzxyz";
bool mqtt_debug=false;
bool mqtt_infotomqtt = true;
bool mqtt_enableLogToSPIFFS = true;
String currentIP = "";  // Keep track of the current IP to only update on change

// vars for program logic
int OpenThermCommandIndex = 0; // index of the OpenTherm command we are processing
const char compile_date[] = __DATE__ " " __TIME__;  // Make sure we can output compile date
unsigned long t_heartbeat=millis()-heartbeatTickInMillis; // last heartbeat timestamp, init on previous heartbeat, so processing start right away
unsigned long t_last_mqtt_discovery=millis()-MQTTDiscoveryHeartbeatInMillis; // last mqqt discovery timestamp
unsigned long t_last_climateheartbeat=0; // last climate heartbeat timestamp
unsigned long t_last_tempreceived=0; // Time when last MQTT temp was received
unsigned long t_last_outsidetempreceived=0; // Time when last MQTT temp was received
unsigned long t_save_config; // timestamp for delayed save
unsigned long t_last_mqtt_try_connect = millis()-MQTTConnectTimeoutInMillis; // the last time we tried to connect to mqtt server
unsigned long heatingOffConditionStart = 0; // Timestamp when heating off condition was first met
unsigned long coolingOffConditionStart = 0; // Timestamp when cooling off condition was first met
bool ClimateConfigSaved=true;
float insideTempAt[60];
float outsidetemp[NUMBEROFMEASUREMENTS];
int outsidetempcursor;
bool debug=false;
bool infotomqtt = DEFAULT_INFOTOMQTT;
bool mqttConnectionLostLogged = false;
bool wifiConnectionLostLogged = false;
bool insideTemperatureReceived = false;
bool outsideTemperatureReceived = false;
bool OTAUpdateInProgress = false;
int sensorIndex = 0;
bool isPublishingAllSensors = false;
bool firstPublishDone = false;
unsigned long resetButtonSubscribeTime = 0;

// object for uploading files
File fsUploadFile;

// objects to be used by program
ESP8266WebServer server(httpport);    //Web server object. Will be listening in port 80 (default for HTTP)
OneWire oneWire(14);                  // for OneWire Bus. Data wire is connected to 14 pin on the OpenTherm Shield (Temperature sensor)
DallasTemperature sensors(&oneWire);  // for the temp sensor on one wire bus
OpenTherm ot(4,5);                    // pin number for opentherm adapter connection, 2/3 for Arduino, 4/5 for ESP8266 (D2), 21/22 for ESP32
WiFiUDP ntpUDP;                       // for NTP client
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 3600, 24 * 60 * 60 * 1000); //  Set the timeserver (incl offset and interval)



void logToSPIFFS(String message) {
  // Check size of log0.txt
  if (LittleFS.exists("/log0.txt")) {
    File logFile = LittleFS.open("/log0.txt", "r");
    if (logFile) {
      size_t size = logFile.size();
      logFile.close();
      if (size >= 51200) { // 50KB
        // Rotate files
        for (int i = 9; i >= 0; i--) {
          String oldName = "/log" + String(i) + ".txt";
          String newName = "/log" + String(i + 1) + ".txt";
          if (LittleFS.exists(oldName)) {
            if (LittleFS.exists(newName)) {
              LittleFS.remove(newName);
            }
            LittleFS.rename(oldName, newName);
          }
        }
        // Delete log10.txt if exists
        if (LittleFS.exists("/log10.txt")) {
          LittleFS.remove("/log10.txt");
        }
      }
    }
  }

  // Check total log size and remove oldest if > 512KB
  size_t totalSize = 0;
  for (int i = 0; i <= 10; i++) {
    String fileName = "/log" + String(i) + ".txt";
    if (LittleFS.exists(fileName)) {
      File f = LittleFS.open(fileName, "r");
      if (f) {
        totalSize += f.size();
        f.close();
      }
    }
  }
  if (totalSize > 524288) { // 512KB
    // Remove oldest (log10.txt)
    if (LittleFS.exists("/log10.txt")) {
      LittleFS.remove("/log10.txt");
    }
  }

  // Append message to log0.txt
  File logFile = LittleFS.open("/log0.txt", "a");
  if (logFile) {
    logFile.println(message);
    logFile.close();
  }
}

void Log(String level, String text) {
  text = timeClient.getFormattedTime() + " " + level + ": " + text;
  if (enableLogToSPIFFS) logToSPIFFS(text);
  if (WiFi.status() == WL_CONNECTED && mqttConnected()) {
    UpdateMQTTTextSensor(Log_Name, text.c_str(), true);
  }
  Serial.println(text);
}

void Debug(String text) {
  if (debug) {
    Log("DEBUG", text);
  }
}

void Error(String text) {
  Log("ERROR", text);
}

void Info(String text) {
  if (infotomqtt) {
    Log("INFO", text);
  }
}

void feedWatchdog() {
  ESP.wdtFeed();  // Feed the hardware watchdog
}

void IRAM_ATTR handleInterrupt() {
    ot.handleInterrupt();
}

void SendHTTP(String command, String result) {
  JsonDocument json;

  // generate message

  // Generic
  json["InterfaceVersion"] = 3;
  json["Command"] = command;
  json["Result"] = result;
  json["CompileDate"] = compile_date;
  json["uptime"] = getUptimeString();

  // Add Opentherm Status
  if (responseStatus == OpenThermResponseStatus::SUCCESS) {
    json["OpenThermStatus"] = "OK";
  } else if (responseStatus == OpenThermResponseStatus::NONE) {
    json["OpenThermStatus"] = "OpenTherm is not initialized";
  } else if (responseStatus == OpenThermResponseStatus::INVALID) {
    json["OpenThermStatus"] = "Invalid response";
  } else if (responseStatus == OpenThermResponseStatus::TIMEOUT) {
    json["OpenThermStatus"] = "Response timeout, is the boiler connected?";
  } else {
    json["OpenThermStatus"] = "Unknown Status";
  }

  // Report if we are receiving commands
  json["ControlledBy"] = "Climate Mode";
  
  // Add MQTT Connection status 
  json["MQTTconnected"] = mqttConnected() ? "true" : "false";
  json["MQTTstate"] = mqttConnectError();

  // Add BoilerManagementVars
  json["EnableHotWater"] = enableHotWater;
  json["DHWSetpoint"] = dhw_SetPoint;
  json["climateSetpoint"] = climate_SetPoint;
  json["climateMode"] = climate_Mode;
  json["weatherDependentMode"] = Weather_Dependent_Mode;
  json["HolidayMode"] = Holiday_Mode;
  json["FrostProtectionActive"] = FrostProtectionActive;

  // Add PID 
  json["P"] = P;
  json["I"] = I;
  json["D"] = D;
  
  // Add BoilerStatus
  json["CentralHeating"] = CentralHeating;
  json["HotWater"] = HotWater;
  json["Cooling"] = Cooling;
  json["Flame"] = Flame;
  json["Fault"] = Fault;
  json["Diagnostic"] = Diagnostic;

   // Add boiler sensors
  json["BoilerTemperature"] = float(int(boiler_Temperature*100))/100;
  json["BoilerSetpoint"] = float(int(boiler_SetPoint*100))/100;
  json["DhwTemperature"] = float(int(dhw_Temperature*100))/100;    
  json["ReturnTemperature"] = float(int(return_Temperature*100))/100;
  json["OutsideTemperature"] = float(int(outside_Temperature*100))/100;
  json["OTOutsideTemperature"] = float(int(OT_outside_Temperature*100))/100;
  json["Modulation"] = modulation;
  json["Pressure"] = pressure;
  json["Flowrate"] = flowrate;
  json["FaultCode"] = FaultCode;

  // Add Temp Sensor value
  json["ThermostatTemperature"] = float(int((currentTemperature+ThermostatTemperatureCalibration)*100))/100;
  json["mqttTemperature"] = float(int(mqttTemperature*100))/100;

   // Send output
  size_t jsonLen = measureJson(json) + 1;
  std::unique_ptr<char[]> buf(new char[jsonLen]);
  serializeJson(json, buf.get(), jsonLen);
  server.send(200, "application/json", buf.get());       //Response to the HTTP request
}

void handleResetWifiCredentials() {
  Info("Resetting Wifi Credentials");
  WiFiManager wifiManager;
  wifiManager.resetSettings();
  SendHTTP("ResetWifiCredentials","OK");
  delay(500);

  Debug("Trigger watchdog to reset wemos");
  wdt_disable();
  wdt_enable(WDTO_15MS);
  while (1) {}
}



void handleGetSensors() {
  Info("Getting the sensors");
  SendHTTP("GetSensors","OK"); // Add actual sensor data here
}

// Generic handler for enabling or disabling a function via HTTP argument
void handleHTTPToggle(const String& argName, bool& targetVar, const String& enableText = "On") {
  if (server.arg(argName) != "") {
    if (server.arg(argName).equalsIgnoreCase(enableText)) {
      if (!targetVar) Debug("Enabling " + argName);
      targetVar = true;
    } else {
      if (targetVar) Debug("Disabling " + argName);
      targetVar = false;
    }
  }
}

void handleCommand() {
  Info("Handling Command: Number of args received: "+server.args());
  String Statustext="Unknown Command";

  // blink the LED, so we can see a command was sent
  digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off , to indicate we are executing a command

  // for Debugging purposes
  for (int i = 0; i < server.args(); i++) {
    Debug ("Argument "+String(i)+" -> "+server.argName(i)+": "+server.arg(i));
  } 

  // Set DHWTemp
  if (server.arg("DHWTemperature")!="") {
    Serial.println("Setting dhw temp to "+server.arg("DHWTemperature"));
    dhw_SetPoint=server.arg("DHWTemperature").toFloat();
    Statustext="OK";
  }

  // handle http toggle commands
  handleHTTPToggle("HotWater", enableHotWater);
  handleHTTPToggle("weatherDependentMode", Weather_Dependent_Mode);
  handleHTTPToggle("holidayMode", Holiday_Mode);

  Debug("Weather Dependent Mode: "+String(Weather_Dependent_Mode));

  // Set Climate Mode
  if (server.arg("climateMode")!="") {
    Statustext="OK";
    Serial.println("Setting climate mode to "+server.arg("climateMode"));
    HandleClimateMode(server.arg("climateMode").c_str());
  }

  // Set climate Setpoint Temp
  if (server.arg("climateSetpoint")!="") {
    Serial.println("Setting Climate Setpoint to "+server.arg("climateSetpoint"));
    handleClimateSetpoint(server.arg("climateSetpoint").toFloat());
    Statustext="OK";
  }


  digitalWrite(LED_BUILTIN, LOW);    // turn the LED on , to indicate we executed the command

  SendHTTP("HandleCommand",Statustext);
}



void handleGetInfo()
{
  Info("GetInfo");
  JsonDocument json;
  char buf[2048];
  json["heap"] = ESP.getFreeHeap();
  json["sketchsize"] = ESP.getSketchSize();
  json["sketchspace"] = ESP.getFreeSketchSpace();
  json["cpufrequency"] = ESP.getCpuFreqMHz();
  json["chipid"] = ESP.getChipId();
  json["sdkversion"] = ESP.getSdkVersion();
  json["bootversion"] = ESP.getBootVersion();
  json["bootmode"] = ESP.getBootMode();
  json["flashid"] = ESP.getFlashChipId();
  json["flashspeed"] = ESP.getFlashChipSpeed();
  json["flashsize"] = ESP.getFlashChipRealSize();
  json["resetreason"] = ESP.getResetReason();
  json["resetinfo"] = ESP.getResetInfo();
  json["freeheap"] = ESP.getFreeHeap();
  if (WiFi.status() == WL_CONNECTED && mqttConnected())
  {
    json["MQTTconnected"] = true;
  } else {
    json["MQTTconnected"] = false;
  }
  json["mqttstate"] = mqttConnectError();
  json["currentTime"] = timeClient.getFormattedTime();
  json["uptime"] = getUptimeString();
  json["compile_date"] = String(compile_date);
  #if MQTT_LIBRARY == 0
  json["mqttLibrary"] = "PubSubClient";
  #elif MQTT_LIBRARY == 1
  json["mqttLibrary"] = "ArduinoMqttClient";
  #else
  json["mqttLibrary"] = "AsyncMQTTClient";
  #endif
  
  serializeJson(json, buf, sizeof(buf)); 
  server.send(200, "application/json", buf);       //Response to the HTTP request
}

void sendUploadForm()
{
  Serial.println("sendUploadForm");

  server.send(200, "text/html; charset=UTF-8", (String("Use this form to upload files<BR /><BR />")+String(HTTP_UPLOAD_FORM)).c_str());       //Response to the HTTP request
}

void handleFileUpload(){ // upload a new file to the SPIFFS
  HTTPUpload& upload = server.upload();
  if(upload.status == UPLOAD_FILE_START){
    Debug("Upload file start");
    String filename = upload.filename;
    if(!filename.startsWith("/")) filename = "/"+filename;
    Serial.print("handleFileUpload Name: "); Serial.println(filename);
    Debug("HandleFileUpload Name "+filename);
    fsUploadFile = LittleFS.open(filename, "w");            // Open the file for writing in LittleFS (create if it doesn't exist)
    if (!fsUploadFile) {
      server.send(500, "text/plain", "500: couldn't create file");
    }
    filename = String();
  } else if(upload.status == UPLOAD_FILE_WRITE){
    Debug("Upload file write");
    if(fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize); // Write the received bytes to the file
  } else if(upload.status == UPLOAD_FILE_END){
    Debug("Upload file end");
    if(fsUploadFile) {                                    // If the file was successfully created
      fsUploadFile.close();                               // Close the file again
      Serial.print("handleFileUpload Size: "); Serial.println(upload.totalSize);
      // server.send(200, "text/plain", "File succesfully uploaded");
      server.send(200, "text/html; charset=UTF-8", (String(upload.filename)+
                                      String(" succesfully uploaded (")+ 
                                      String(upload.totalSize)+String(" bytes), do you want to upload another file?<BR /><BR />")+
                                      String(HTTP_UPLOAD_FORM)).c_str()); // send form to upload another file
    } else {
      server.send(500, "text/plain", "500: couldn't create file");
    }
  }
}

void handleGetConfig()
{
  Info("GetConfig");
  JsonDocument json;

  // First load config from file
  if (LittleFS.exists(CONFIGFILE)) {
    File configFile = LittleFS.open(CONFIGFILE, "r");
    if (configFile) {
      DeserializationError error = deserializeJson(json, configFile);
      configFile.close();
      if (error) {
        Serial.println("Failed to read config file");
        server.send(500, "application/json", "{\"error\":\"Failed to read config file\"}");
        return;
      }
    }
  }

  // Mask password if present
  if(json["mqttpass"].is<const char*>()) {
    json["mqttpass"] = "*****";
  }
  
  // Add runtime info
  json["heap"] = ESP.getFreeHeap();
  json["uptime"] = getUptimeString();

  // Send output with pretty formatting
  char buf[2048];
  serializeJsonPretty(json, buf);
  server.send(200, "application/json", buf);
}

void handleConfigJsonFile() {
  Info("Config JSON file requested");
  if (LittleFS.exists(CONFIGFILE)) {
    File configFile = LittleFS.open(CONFIGFILE, "r");
    if (configFile) {
      JsonDocument json;
      DeserializationError error = deserializeJson(json, configFile);
      configFile.close();
      if (!error) {
        // Mask the password
        json["mqttpass"] = "*****";
        char buf[2048];
        serializeJsonPretty(json, buf);
        server.send(200, "application/json", buf);
        return;
      }
    } 
  }
  server.send(404, "application/json", "{\"error\":\"config.json not found\"}");
}

void handleSaveConfig() {
  Info("handleSaveConfig");
  String Message;
  
  // for Debugging purposes
  Serial.println("Handling Command: Number of args received: "+server.args());
  for (int i = 0; i < server.args(); i++) {
    Serial.println ("Argument "+String(i)+" -> "+server.argName(i)+": "+server.arg(i));
  } 

  // try to deserialize
  JsonDocument json;
  DeserializationError error = deserializeJson(json, server.arg("plain"));
  if (error) {
    Message=String("Invalid JSON: ")+error.f_str();
    server.send(500, "text/plain", Message.c_str());
    return;
  } else {

    // mqtt config
    usemqtt=json["usemqtt"] | usemqtt;
    usemqttauthentication=json["usemqttauthentication"] | usemqttauthentication;
    mqttpersistence=json["mqttretained"];
    mqttport=json["mqttport"] | 1883;

    if (json["mqttserver"].is<const char*>() ) {
      mqttserver=json["mqttserver"].as<String>();
    }

    if (json["mqttuser"].is<const char*>() ) {
      mqttuser=json["mqttuser"].as<String>();
    }

    if (json["mqttpass"]!="*****") {
      mqttpass=json["mqttpass"].as<String>();
    }

    if (json["mqtttemptopic"].is<const char*>() ) {
      mqtttemptopic=json["mqtttemptopic"].as<String>();
    }
    
    if (json["mqttoutsidetemptopic"].is<const char*>() ) {
      mqttoutsidetemptopic=json["mqttoutsidetemptopic"].as<String>();
    }
    
    debug=json["debugtomqtt"] | true;

    infotomqtt=json["infotomqtt"] | true;

    // PID Settings
    KP = json["KP"] | KP;
    KI = json["KI"] | KI;
    KD = json["KD"] | KD;

    // Boiler Control Settings
    MinBoilerTemp = json["MinBoilerTemp"] | MinBoilerTemp;
    MaxBoilerTemp = json["MaxBoilerTemp"] | MaxBoilerTemp;
    minimumTempDifference = json["minimumTempDifference"] | minimumTempDifference;
    FrostProtectionSetPoint = json["FrostProtectionSetPoint"] | FrostProtectionSetPoint;
    BoilerTempAtPlus20 = json["BoilerTempAtPlus20"] | BoilerTempAtPlus20;
    BoilerTempAtMinus10 = json["BoilerTempAtMinus10"] | BoilerTempAtMinus10;  
    SwitchHeatingOffAt = json["SwitchHeatingOffAt"] | SwitchHeatingOffAt;
    ReferenceRoomCompensation = json["ReferenceRoomCompensation"] | ReferenceRoomCompensation;
    if (json["Curvature"].is<const char*>() ) {
      Curvature=getCurvatureIntFromString(json["Curvature"] | "small");
    }

    // persistent climate mode
    if (json["climateMode"].is<const char*>() ) {
      climate_Mode = json["climateMode"].as<String>();
    }
    climate_SetPoint = json["climateSetpoint"] | climate_SetPoint;
    Weather_Dependent_Mode = json["weatherDependentMode"] | Weather_Dependent_Mode;
    Holiday_Mode = json["holidayMode"] | Holiday_Mode;

    // Save and restart
    SaveConfig();
    server.send(200, "text/plain", "New Config Saved");
    delay(500); // wait for server send to finish
    ESP.restart(); // restart
  }
}

void handleRemoveConfig() {
  Info("handleRemoveConfig");
  
  if (LittleFS.exists(CONFIGFILE)) {
    Serial.println("Config file exists, removing configfile");
    LittleFS.remove(CONFIGFILE);
    server.send(200, "text/plain", "Config file removed");
    Debug("Configfile removed");
    delay(500); // wait for server send to finish
    ESP.restart(); // restart
  } else {
    server.send(200, "text/plain", "No config file present to remove");
  } 

  return;
}

void handleReset() {
  Info("handleReset");
  
  server.send(200, "text/plain", "Device Reset");
  delay(500); // wait for server send to finish
  ESP.restart(); // restart

  return;
}

bool endsWith(const char* what, const char* withwhat)
{
    int l1 = strlen(withwhat);
    int l2 = strlen(what);
    if (l1 > l2)
        return 0;

    return strcmp(withwhat, what + (l2 - l1)) == 0;
}


bool serveFile(const char url[])
{
  Serial.printf("serveFile(): %s\n\rE",url);

  char path[50];
  
  if (url[strlen(url)-1]=='/') {
    sprintf (path,"%sindex.html",url);
  } else {
    sprintf(path,"%s",url);
  }
  if (LittleFS.exists(path))
  {
    File file = LittleFS.open(path, "r");
    if (server.hasArg("download")) server.streamFile(file, "application/octet-stream");
    else if (endsWith(path,".htm") or endsWith(path,".html")) server.streamFile(file, "text/html; charset=UTF-8");
    else if (endsWith(path,".css") ) server.streamFile(file, "text/css");
    else if (endsWith(path,".png") ) server.streamFile(file, "image/png");
    else if (endsWith(path,".gif") ) server.streamFile(file, "image/gif");
    else if (endsWith(path,".jpg") ) server.streamFile(file, "image/jpeg");
    else if (endsWith(path,".ico") ) server.streamFile(file, "image/x-icon");
    else if (endsWith(path,".xml") ) server.streamFile(file, "text/xml");
    else if (endsWith(path,".pdf") ) server.streamFile(file, "application./x-pdf");
    else if (endsWith(path,".zip") ) server.streamFile(file, "application./x-zip");
    else if (endsWith(path,".gz") ) server.streamFile(file, "application./x-gzip");
    else if (endsWith(path,".json") ) server.streamFile(file, "application/json");
    else server.streamFile(file, "text/plain");
    file.close();
    return true;
  }
  return false;
}


void handleDir() {
  String html = "<html><head><title>SPIFFS Directory</title></head><body><h1>SPIFFS Files</h1><ul>";
  Dir dir = LittleFS.openDir("/");
  while (dir.next()) {
    String fileName = dir.fileName();
    size_t fileSize = dir.fileSize();
    html += "<li><a href=\"" + fileName + "\">" + fileName + "</a> (" + String(fileSize) + " bytes) <a href=\"/delete?file=" + fileName + "\">(delete)</a></li>";
  }
  html += "</ul><p><a href=\"/upload\">upload files</a> <a href=\"http://" + host + "/\">back to GUI</a></p></body></html>";
  server.send(200, "text/html", html);
}

void handleDelete() {
  if (server.hasArg("file")) {
    String fileName = server.arg("file");
    if (LittleFS.remove("/" + fileName)) {
      server.send(200, "text/html", "<p>File deleted. <a href=\"/dir\">Back to dir</a></p>");
    } else {
      server.send(200, "text/html", "<p>Failed to delete file. <a href=\"/dir\">Back to dir</a></p>");
    }
  } else {
    server.send(400, "text/plain", "Missing file parameter");
  }
}

void handleNotFound()
{
  Info("HandleNotFound: " + server.uri());
  // first, try to serve the requested file from flash
  if (!serveFile(server.uri().c_str()))
  {
    // create 404 message if no file was found for this URI
    String message = "File Not Found\n\nURI: "+server.uri() + "\nMethod: "+ ( server.method() == HTTP_GET ? "GET" : "POST" ) + "\nArguments "+server.args()+"<BR />";
    
    for (uint8_t i = 0; i < server.args(); i++)
    {
      message += server.argName(i)+"="+server.arg(i)+"<BR />";
    }

    message += "<BR /><a href=\"upload\">Click here to upload the gui files</a>";
    
    server.send(404, "text/html; charset=UTF-8", message.c_str());
  }
}

String getCurvatureStringFromInt(int i) 
{
  if (i==0) {
    return "none";
  } else if (i==10) {
    return "small";
  } else if (i==20) {
    return "medium";
  } else if (i==30) {
    return "large";
  } else if (i==40) {
    return "extralarge";
  } else {
    return "none";
  }
}

int getCurvatureIntFromString(String value) {
  if (value=="none") {
    return 0;
  } else if (value=="small") {
    return 10;
  } else if (value=="medium") {
    return 20;
  } else if (value=="large") {
    return 30;
  } else if (value=="extralarge") {
    return 40;
  } else {
    Debug("Unknown Curvature String: ["+value+"]");
    return 10; // default value
  }
}

void readConfig()
{
  if (LittleFS.exists(CONFIGFILE)) {
    //file exists, reading and loading
    Debug("reading config file");
    File configFile = LittleFS.open(CONFIGFILE, "r");
    if (configFile) {
      size_t size = configFile.size();
      std::unique_ptr<char[]> buf(new char[size]);
      configFile.readBytes(buf.get(), size);
      JsonDocument json;
      auto deserializeError = deserializeJson(json, buf.get());
      serializeJson(json, Serial);
      if (!deserializeError) {
        // mqtt config
        usemqtt = json["usemqtt"].is<bool>() ? json["usemqtt"].as<bool>() : DEFAULT_USEMQTT;
        usemqttauthentication = json["usemqttauthentication"].is<bool>() ? json["usemqttauthentication"].as<bool>() : DEFAULT_USEMQTTAUTH;
        mqttserver = json["mqttserver"].is<String>() ? json["mqttserver"].as<String>() : String(DEFAULT_MQTTSERVER);
        mqttport = json["mqttport"].is<int>() ? json["mqttport"].as<int>() : DEFAULT_MQTTPORT;
        mqttuser = json["mqttuser"].is<String>() ? json["mqttuser"].as<String>() : String(DEFAULT_MQTTUSER);
        mqttpass = json["mqttpass"].is<String>() ? json["mqttpass"].as<String>() : String(DEFAULT_MQTTPASS);
        mqttpersistence = json["mqttretained"].is<bool>() ? json["mqttretained"].as<bool>() : DEFAULT_MQTTPERSISTENCE;
        mqtttemptopic = json["mqtttemptopic"].is<String>() ? json["mqtttemptopic"].as<String>() : String(DEFAULT_MQTTTEMP_TOPIC);
        mqttoutsidetemptopic = json["mqttoutsidetemptopic"].is<String>() ? json["mqttoutsidetemptopic"].as<String>() : String(DEFAULT_MQTTOUTSIDETEMP_TOPIC);

        debug = json["debugtomqtt"].is<bool>() ? json["debugtomqtt"].as<bool>() : DEFAULT_DEBUG;

        infotomqtt = json["infotomqtt"].is<bool>() ? json["infotomqtt"].as<bool>() : DEFAULT_INFOTOMQTT;

        enableLogToSPIFFS = json["enableLogToSPIFFS"].is<bool>() ? json["enableLogToSPIFFS"].as<bool>() : DEFAULT_ENABLE_LOG_TO_SPIFFS;

        // PID Settings
        KP = json["KP"].is<int>() ? json["KP"].as<int>() : DEFAULT_KP;
        KI = json["KI"].is<float>() ? json["KI"].as<float>() : DEFAULT_KI;
        KD = json["KD"].is<float>() ? json["KD"].as<float>() : DEFAULT_KD;

        // Boiler Control Settings
        MinBoilerTemp = json["MinBoilerTemp"].is<int>() ? json["MinBoilerTemp"].as<int>() : DEFAULT_MINBOILERTEMP;
        MaxBoilerTemp = json["MaxBoilerTemp"].is<int>() ? json["MaxBoilerTemp"].as<int>() : DEFAULT_MAXBOILERTEMP;
        minimumTempDifference = json["minimumTempDifference"].is<int>() ? json["minimumTempDifference"].as<int>() : DEFAULT_MINIMUMTEMPDIFFERENCE;
        FrostProtectionSetPoint = json["FrostProtectionSetPoint"].is<int>() ? json["FrostProtectionSetPoint"].as<int>() : DEFAULT_FROSTPROTECTIONSETPOINT;
        BoilerTempAtPlus20 = json["BoilerTempAtPlus20"].is<int>() ? json["BoilerTempAtPlus20"].as<int>() : DEFAULT_BOILERTEMPATPLUS20;
        BoilerTempAtMinus10 = json["BoilerTempAtMinus10"].is<int>() ? json["BoilerTempAtMinus10"].as<int>() : DEFAULT_BOILERTEMPATMINUS10;
        Curvature = getCurvatureIntFromString(json["Curvature"].is<String>() ? json["Curvature"].as<String>() : String(DEFAULT_CURVATURE_STRING));
        SwitchHeatingOffAt = json["SwitchHeatingOffAt"].is<int>() ? json["SwitchHeatingOffAt"].as<int>() : DEFAULT_SWITCHHEATINGOFFAT;
        ReferenceRoomCompensation = json["ReferenceRoomCompensation"].is<int>() ? json["ReferenceRoomCompensation"].as<int>() : DEFAULT_REFERENCEROOMCOMPENSATION;

        // persistent climate mode
        climate_Mode = json["climateMode"].is<String>() ? json["climateMode"].as<String>() : String(DEFAULT_CLIMATE_MODE);
        if (climate_Mode.equals("null")) {
          climate_Mode = String(DEFAULT_CLIMATE_MODE);
        }
        climate_SetPoint = json["climateSetpoint"].is<int>() ? json["climateSetpoint"].as<int>() : DEFAULT_CLIMATE_SETPOINT;
        Weather_Dependent_Mode = json["weatherDependentMode"].is<bool>() ? json["weatherDependentMode"].as<bool>() : DEFAULT_WEATHERDEPENDENTMODE;
        Holiday_Mode = json["holidayMode"].is<bool>() ? json["holidayMode"].as<bool>() : DEFAULT_HOLIDAYMODE;
        configFile.close();
        return;
      }
      configFile.close();
    } else {
      Debug("failed to load json config");
    }
  }

  // If config file does not exist, initialize all variables with defaults
  usemqtt = DEFAULT_USEMQTT;
  usemqttauthentication = DEFAULT_USEMQTTAUTH;
  mqttserver = DEFAULT_MQTTSERVER;
  mqttport = DEFAULT_MQTTPORT;
  mqttuser = DEFAULT_MQTTUSER;
  mqttpass = DEFAULT_MQTTPASS;
  mqttpersistence = DEFAULT_MQTTPERSISTENCE;
  mqtttemptopic = DEFAULT_MQTTTEMP_TOPIC;
  mqttoutsidetemptopic = DEFAULT_MQTTOUTSIDETEMP_TOPIC;
  debug = DEFAULT_DEBUG;
  infotomqtt = DEFAULT_INFOTOMQTT;
  enableLogToSPIFFS = DEFAULT_ENABLE_LOG_TO_SPIFFS;
  KP = DEFAULT_KP;
  KI = DEFAULT_KI;
  KD = DEFAULT_KD;
  MinBoilerTemp = DEFAULT_MINBOILERTEMP;
  MaxBoilerTemp = DEFAULT_MAXBOILERTEMP;
  minimumTempDifference = DEFAULT_MINIMUMTEMPDIFFERENCE;
  FrostProtectionSetPoint = DEFAULT_FROSTPROTECTIONSETPOINT;
  BoilerTempAtPlus20 = DEFAULT_BOILERTEMPATPLUS20;
  BoilerTempAtMinus10 = DEFAULT_BOILERTEMPATMINUS10;
  Curvature = getCurvatureIntFromString(DEFAULT_CURVATURE_STRING);
  SwitchHeatingOffAt = DEFAULT_SWITCHHEATINGOFFAT;
  ReferenceRoomCompensation = DEFAULT_REFERENCEROOMCOMPENSATION;
  climate_Mode = DEFAULT_CLIMATE_MODE;
  climate_SetPoint = DEFAULT_CLIMATE_SETPOINT;
  Weather_Dependent_Mode = DEFAULT_WEATHERDEPENDENTMODE;
  Holiday_Mode = DEFAULT_HOLIDAYMODE;
  // Save the new config file with defaults
  SaveConfig();
}

void SaveConfig()
{
  JsonDocument json;
  Debug("SaveConfig()");

  // mqtt config
  json["usemqtt"] = usemqtt;
  json["usemqttauthentication"] = usemqttauthentication;
  json["mqttserver"] = mqttserver; 
  json["mqttport"] = mqttport;
  json["mqttuser"] = mqttuser;
  json["mqttpass"] = mqttpass;
  json["mqttretained"] = mqttpersistence;
  json["mqtttemptopic"] = mqtttemptopic;
  json["mqttoutsidetemptopic"] = mqttoutsidetemptopic;
  json["debugtomqtt"] = debug;
  json["infotomqtt"] = infotomqtt;
  json["enableLogToSPIFFS"] = enableLogToSPIFFS;
  
  // add/change the Climate settings
  json["climateMode"] = climate_Mode;
  json["climateSetpoint"] = climate_SetPoint;
  json["weatherDependentMode"] = Weather_Dependent_Mode;
  json["holidayMode"] = Holiday_Mode;

  // add boiler control settings
  json["MinBoilerTemp"] = MinBoilerTemp;
  json["MaxBoilerTemp"] = MaxBoilerTemp;
  json["minimumTempDifference"] = minimumTempDifference;
  json["FrostProtectionSetPoint"] = FrostProtectionSetPoint;
  json["BoilerTempAtPlus20"] = BoilerTempAtPlus20;
  json["BoilerTempAtMinus10"] = BoilerTempAtMinus10;
  json["Curvature"] = Curvature;
  json["SwitchHeatingOffAt"] = SwitchHeatingOffAt;
  json["ReferenceRoomCompensation"] = ReferenceRoomCompensation;
  json["KP"] = KP;
  json["KI"] = KI;
  json["KD"] = KD;

  // save the new file
  File configFile = LittleFS.open(CONFIGFILE, "w");
  if (!configFile) {
    Debug("unable to open "+String(CONFIGFILE));
  } else {
    // Save the file
    serializeJson(json, configFile);
    configFile.close();
    Debug("Configfile saved");
  }  

  ClimateConfigSaved=true;
  
}

// not defined  in opentherm lib, so declaring local
float getOutsideTemperature() {
  unsigned long response = ot.sendRequest(ot.buildRequest(OpenThermRequestType::READ, OpenThermMessageID::Toutside, 0));
  float temp = ot.isValidResponse(response) ? ot.getFloat(response) : 0;
  return round(temp * 10.0) / 10.0; // round to 1 decimal place
}

float getDHWFlowrate() {
  unsigned long response = ot.sendRequest(ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::DHWFlowRate, 0));
  float temp = ot.isValidResponse(response) ? ot.getFloat(response) : 0;
  return round(temp * 10.0) / 10.0; // round to 1 decimal place
}

void CommunicateSteeringVarsToMQTT() {
if (WiFi.status() == WL_CONNECTED && mqttConnected()) {
    // Climate Mode
    UpdateClimateSetpointMode();

    // Communicate the setpoints
    UpdateMQTTSetpoint(Climate_Name, mqtt_climate_setpoint, climate_SetPoint, false);
    UpdateMQTTSetpoint(DHW_Setpoint_Name, mqtt_dhw_setpoint, dhw_SetPoint, false);
    UpdateMQTTNumberSensor(Boiler_Setpoint_Temperature_Name, mqtt_boiler_setpoint, boiler_SetPoint, 0.09, false);

    // Communicate the steering vars
    UpdateMQTTNumber(MinBoilerTemp_Name, mqtt_minboilertemp, MinBoilerTemp, 0.5, false);
    UpdateMQTTNumber(MaxBoilerTemp_Name, mqtt_maxboilertemp, MaxBoilerTemp, 0.5, false);
    UpdateMQTTNumber(MinimumTempDifference_Name, mqtt_minimumTempDifference, minimumTempDifference, 0.5, false);
    UpdateMQTTNumber(FrostProtectionSetPoint_Name, mqtt_FrostProtectionSetPoint, FrostProtectionSetPoint, 0.5, false);
    UpdateMQTTNumber(BoilerTempAtPlus20_Name, mqtt_BoilerTempAtPlus20, BoilerTempAtPlus20, 0.5, false);
    UpdateMQTTNumber(BoilerTempAtMinus10_Name, mqtt_BoilerTempAtMinus10, BoilerTempAtMinus10, 0.5, false);
    UpdateMQTTNumber(SwitchHeatingOffAt_Name, mqtt_SwitchHeatingOffAt, SwitchHeatingOffAt, 0.5, false);
    UpdateMQTTNumber(ReferenceRoomCompensation_Name, mqtt_ReferenceRoomCompensation, ReferenceRoomCompensation, 0.5, false);
    UpdateMQTTNumber(KP_Name, mqtt_kp, KP, 0.01, false);
    UpdateMQTTNumber(KI_Name, mqtt_ki, KI, 0.01, false);
    UpdateMQTTNumber(KD_Name, mqtt_kd, KD, 0.01, false);
    UpdateMQTTNumberSensor(P_Name, mqtt_p, P, 0.01);
    UpdateMQTTNumberSensor(I_Name, mqtt_i, I, 0.01);
    UpdateMQTTNumberSensor(D_Name, mqtt_d, D, 0.01);
    UpdateMQTTNumberSensor(WiFi_RSSI_Name, mqtt_wifi_rssi, (float)WiFi.RSSI(), 2.0, false);
    UpdateMQTTText(MQTT_TempTopic_Name, mqtt_mqtttemptopic, mqtttemptopic);
    UpdateMQTTText(MQTT_OutsideTempTopic_Name, mqtt_mqttoutsidetemptopic, mqttoutsidetemptopic);
    if (mqtt_Curvature!=Curvature) {
      UpdateMQTTCurvatureSelect(Curvature_Name,Curvature);
      mqtt_Curvature=Curvature;
    }

    // The actual temperature for the climate device. Now the temp required from mqtt. But should be made switchable to other sources 
    if (mqttTemperature!=mqtt_mqttTemperature and insideTemperatureReceived) {
      UpdateMQTTSetpointTemperature(Climate_Name,mqttTemperature);
      mqtt_mqttTemperature=mqttTemperature;
    }

    //Enable HOt Water
    if (UpdateMQTTSwitch(EnableHotWater_Name, mqtt_enable_HotWater, enableHotWater, false)) {
      // Communicate to setpoint as well
      if (enableHotWater) {
        UpdateMQTTSetpointMode(DHW_Setpoint_Name,1);
      } else  {
        UpdateMQTTSetpointMode(DHW_Setpoint_Name,0);
      }
    }

    // Weather Dependent Mode
    UpdateMQTTSwitch(Weather_Dependent_Mode_Name, mqtt_Weather_Dependent_Mode, Weather_Dependent_Mode, false);

    // Holiday Mode
    UpdateMQTTSwitch(Holiday_Mode_Name, mqtt_Holiday_Mode, Holiday_Mode, false);

    // Debug
    UpdateMQTTSwitch(Debug_Name, mqtt_debug, debug, false);

    // Info
    UpdateMQTTSwitch(Info_Name, mqtt_infotomqtt, infotomqtt, false);

    // Enable Log to SPIFFS
    UpdateMQTTSwitch(EnableLogToSPIFFS_Name, mqtt_enableLogToSPIFFS, enableLogToSPIFFS, false);

    // Frost Protection Active
    UpdateMQTTBinarySensor(FrostProtectionActive_Name, mqtt_FrostProtectionActive, FrostProtectionActive, false);
    // Uptime Sensor
    static unsigned long last_uptime_update = 0;
    if (millis() - last_uptime_update > 5000) { // every 5 seconds
      UpdateMQTTTextSensor(Uptime_Name, getUptimeString().c_str(), false);
      last_uptime_update = millis();
    }
  }
}

void resetI() 
{
  float roomTemperature;
  if (mqtttemptopic.length()==0) { // use internal temp
    roomTemperature=currentTemperature; // use internal temp sensor
  } else {
    roomTemperature=mqttTemperature;
  }

  // Set I (P and D are calculated, so no need to initialize)
  I=roomTemperature;
}

void resetD()
{
  // Reset InsideTempAt array at default value (currenttemp)
  for (int i=0;i<60;i++) {
    insideTempAt[i]=I;
  }
}

void InitPID()
{ 
  // No need to reset P, will be calculated, so only reset I and the matrix for D
  resetI();
  resetD();
}

void UpdatePID(float setpoint,float temperature)
{
  int dt;
  int CurrentMinute;
  float error;
  float DeltaKPH;
  float oldI=I;
  float oldP=P;
  float oldD=D;

  // only update PID when Hotwater is not active
  if (!HotWater) {

    //sp = setpoint, pv=current value

    dt=ClimateHeartbeatInMillis/1000;
    if (dt>10) {
        dt=10;
    }
    
    // calculate the error (sp-pv)
    error = setpoint-temperature;

    // calculate the amount of rising/dropping temp
    CurrentMinute=(millis() % 60000 / 1000);
    DeltaKPH = (mqttTemperature-insideTempAt[(CurrentMinute+45)%60])*4;  // temp change the last 15 mins multiplied by 4 is the number of kelvin per hour the temp is currently dropping or rising
    
    // calculate the PID output
    P = KP * error;          // proportional contribution
    I = I + KI * error * dt; // integral contribution
    D = -KD*DeltaKPH;        // deritive contribution

    // correct during heating when setpoint goes down when setpoint already below current temperature (or the other way around when cooling) 
    if ((climate_Mode.equals("heat") and P+I+D<boiler_SetPoint and boiler_SetPoint<temperature) or
        (climate_Mode.equals("cool") and P+I+D>boiler_SetPoint and boiler_SetPoint>temperature)) 
    {
      I=oldI;
    } 

    // Correct PID if PID above or below min/max boiler temp
    if (P+I+D<MinBoilerTemp) {
      // setpoint too low, correcting to absolute min
      I=oldI;
      P=oldP;
      D=oldD;
      D=MinBoilerTemp-P-I;
    } else if (P+I+D>MaxBoilerTemp){
      //setpoint too high, correcting to absolute max
      I=oldI;
      P=oldP;
      D=oldD;
      D=MaxBoilerTemp-P-I;
    }
  }
  Info("PID Mode, PID=("+String(P)+","+String(I)+","+String(D)+"), total: "+String(P+I+D));
}

float GetBoilerSetpointFromOutsideTemperature(float CurrentInsideTemperature, float CurrentOutsideTemperature) 
{
  float MaxYDelta=BoilerTempAtMinus10-BoilerTempAtPlus20; // boilertemp at -10 minus boilertemp at +20
  float MaxXDelta=30;                                     // 20 - (-10)=30

  // curve Calculation based on sine curve
  float TargetTemperatureWithoutCurvature= (20-CurrentOutsideTemperature) / MaxXDelta * MaxYDelta + BoilerTempAtPlus20;
  float ExtraCurvature=sin(PI*(20-CurrentOutsideTemperature)/MaxXDelta)*Curvature*MaxYDelta/100;
  float TargetTemperature=ExtraCurvature+TargetTemperatureWithoutCurvature;

  //Apply reference room compensation
  if (ReferenceRoomCompensation>0 and CurrentInsideTemperature<climate_SetPoint) {
    TargetTemperature+=(climate_SetPoint-CurrentInsideTemperature)*ReferenceRoomCompensation;
  }

  // Make sure target temp remains within set boundaries
  if (TargetTemperature>MaxBoilerTemp) {
    TargetTemperature=MaxBoilerTemp;
  }
  if (TargetTemperature<MinBoilerTemp) {
    TargetTemperature=MinBoilerTemp;
  }

  return TargetTemperature;
}

void handleClimateProgram()
{
  float  roomTemperature;
  if (mqtttemptopic.length()==0) { // use internal temp
    roomTemperature=currentTemperature; // use internal temp sensor
  } else {
    roomTemperature=mqttTemperature;
  }

  if (climate_Mode.equals("off") or 
      Holiday_Mode==true or 
      (mqtttemptopic.length()!=0 and insideTemperatureReceived==true and millis()-t_last_tempreceived>MQTTTemperatureTimeoutInMillis) or 
      (mqtttemptopic.length()!=0 and insideTemperatureReceived==false) ){ 
    // Frost protection mode
    if (roomTemperature<FrostProtectionSetPoint) {
      // we need to act
      FrostProtectionActive=true;

      // Update PID
      if (millis()-ClimateHeartbeatInMillis>t_last_climateheartbeat) {
        // Check if we have to update the PID (only when boiler not in hotwater mode)
        if (!HotWater) {
          UpdatePID(FrostProtectionSetPoint,roomTemperature);
        }
        Debug("Frost protection: P = "+String(P)+", I="+String(I)+", D="+String(D));

        // reset timestamp
        t_last_climateheartbeat=millis();
      }

      // set boiler steering vars
      enableCentralHeating=true;
      boiler_SetPoint=P+I+D;
    } else {
      // no need to act, set the correct state
      FrostProtectionActive=false;
      enableCentralHeating=false;
    }

  } else {
    // Some kind of Climate mode

    // At every heartbeat: Set PID values
    if (millis()-ClimateHeartbeatInMillis>t_last_climateheartbeat) {
      // Check if we have to update the PID (only when boiler not in hotwater mode)
      if (!(HotWater or Weather_Dependent_Mode)) {
        UpdatePID(climate_SetPoint,roomTemperature);
      } else {
        Debug("Skipping PID update, Hotwater or Weather dependent mode active");
      }

      if (Weather_Dependent_Mode and !HotWater) {
        Info("Weather dependent mode, setting setpoint to "+String(GetBoilerSetpointFromOutsideTemperature(roomTemperature,outside_Temperature)));
      }

      // reset timestamp
      t_last_climateheartbeat=millis();
    }

    // Calculate BoilerSetpoint
    if (Weather_Dependent_Mode) {
      // calculate Setpoint based on outside temp
      boiler_SetPoint=GetBoilerSetpointFromOutsideTemperature(roomTemperature,outside_Temperature);
    } else {
      // set Setpoint to PID
      boiler_SetPoint=P+I+D;
    }

    // Enable heating and/or cooling with hysteresis to prevent chattering
    if (climate_Mode.equals("heat")) { // heating mode
      if (boiler_SetPoint > roomTemperature + minimumTempDifference) { // heating required
        enableCentralHeating = true; // Enable heating
        heatingOffConditionStart = 0; // Reset timer
      } else {
        if (heatingOffConditionStart == 0) { // Start timer
          heatingOffConditionStart = millis(); 
        }
        if (millis() - heatingOffConditionStart > HysteresisDelayMillis) { // Check if timer exceeded
          enableCentralHeating = false; 
        }
      }
      enableCooling = false; // cooling mode disabled
      coolingOffConditionStart = 0; // Reset cooling timer
    } else if (climate_Mode.equals("cool")) { // cooling mode
      if (boiler_SetPoint < roomTemperature - minimumTempDifference) { // cooling required
        enableCooling = true; // Enable cooling
        coolingOffConditionStart = 0; // Reset timer
      } else {
        if (coolingOffConditionStart == 0) { // Start timer
          coolingOffConditionStart = millis();
        }
        if (millis() - coolingOffConditionStart > HysteresisDelayMillis) { // Check if timer exceeded
          enableCooling = false;
        }
      }
      enableCentralHeating = false; // heating mode disabled
      heatingOffConditionStart = 0; // Reset heating timer
    } else if (climate_Mode.equals("auto")) { // auto mode
      if (boiler_SetPoint > roomTemperature + minimumTempDifference) { // heating required
        enableCentralHeating = true; // Enable heating
        enableCooling = false; // Disable cooling
        heatingOffConditionStart = 0; // Reset heating timer
        coolingOffConditionStart = 0; // Reset cooling timer
      } else if (boiler_SetPoint < roomTemperature - minimumTempDifference) { // cooling required
        enableCentralHeating = false; // Disable heating
        enableCooling = true; // Enable cooling
        heatingOffConditionStart = 0; // Reset heating timer
        coolingOffConditionStart = 0; // Reset cooling timer
      } else {
        // Neither heating nor cooling condition met
        if (heatingOffConditionStart == 0) { // Start heating timer
          heatingOffConditionStart = millis(); 
        }
        if (coolingOffConditionStart == 0) { // Start cooling timer
          coolingOffConditionStart = millis();
        }
        if (millis() - heatingOffConditionStart > HysteresisDelayMillis) { // Check if timer exceeded
          enableCentralHeating = false;
        }
        if (millis() - coolingOffConditionStart > HysteresisDelayMillis) { // Check if timer exceeded
          enableCooling = false;
        }
      }
    }
  }
}

void handleSetBoilerStatus() {
  // enable/disable heating, hotwater, heating and get status from opentherm connection and boiler (if it can be reached)
  Debug("ot.setBoilerStatus(CH="+String(enableCentralHeating)+", HW="+String(enableHotWater)+", Cooling="+String(enableCooling)+")");
  unsigned long response = ot.setBoilerStatus(enableCentralHeating, enableHotWater, enableCooling);
  responseStatus = ot.getLastResponseStatus();
  if (responseStatus == OpenThermResponseStatus::SUCCESS) {
    // Yes we have a connection, update statuses
    Serial.println("Central Heating: " + String(ot.isCentralHeatingActive(response) ? "on" : "off"));
    Serial.println("Hot Water: " + String(ot.isHotWaterActive(response) ? "on" : "off"));
    Serial.println("Cooling: " + String(ot.isCoolingActive(response) ? "on" : "off"));
    Serial.println("Flame: " + String(ot.isFlameOn(response) ? "on" : "off"));
    Flame=ot.isFlameOn(response);
    CentralHeating=ot.isCentralHeatingActive(response);
    HotWater=ot.isHotWaterActive(response);
    Cooling=ot.isCoolingActive(response);
    Fault=ot.isFault(response);
    Diagnostic=ot.isDiagnostic(response);

    // modulation is reported on the switches, so make sure we have modulation value as well
    modulation = ot.getModulation();

    // Check if we have to send to MQTT for steering vars
    if (WiFi.status() == WL_CONNECTED && mqttConnected()) {
      UpdateMQTTBinarySensor(FlameActive_Name, mqtt_Flame, Flame, false);
      UpdateMQTTBinarySensor(FaultActive_Name, mqtt_Fault, Fault, false);
      UpdateMQTTBinarySensor(DiagnosticActive_Name, mqtt_Diagnostic, Diagnostic, false);
      UpdateMQTTBinarySensor(CoolingActive_Name, mqtt_Cooling, Cooling, false);
      UpdateMQTTBinarySensor(CentralHeatingActive_Name, mqtt_CentralHeating, CentralHeating, false);
      UpdateMQTTBinarySensor(HotWaterActive_Name, mqtt_HotWater, HotWater, false);
      UpdateMQTTPercentageSensor(Modulation_Name, mqtt_modulation, modulation, false);
    }
    // Execute the next command in the next call
  } else if (responseStatus == OpenThermResponseStatus::NONE) {
      Debug("Opentherm Error: OpenTherm is not initialized");
  } else if (responseStatus == OpenThermResponseStatus::INVALID) {
      Debug("Opentherm Error: Invalid response " + String(response, HEX));
  } else if (responseStatus == OpenThermResponseStatus::TIMEOUT) {
      Debug("Opentherm Error: Response timeout");
  }  else {
      Debug("Opentherm Error: unknown error");
  }
}

void handleGetOutSideTemp(){
  Debug("getOutsideTemperature()");
  // Get the outside temperature from OpenTherm
  OT_outside_Temperature = getOutsideTemperature();

  // set the outside temperature to calculate (depending on the)
  if (mqttoutsidetemptopic.length()>0) {
    Debug("Using MQTT Outside Temperature: "+String(mqttOutsideTemperature));
    // If a topic is set, don't read attached sensor, use mqtt value and skip GetOutsideTemp command
    outside_Temperature=mqttOutsideTemperature;
  } else {
    // No topic set, so we are using the outside temp reported by OpenTherm
    outside_Temperature=OT_outside_Temperature;
  }

  // Check if we have to send to MQTT
  if (WiFi.status() == WL_CONNECTED && mqttConnected()) {
    UpdateMQTTNumberSensor(Outside_Temperature_Name, mqtt_outside_Temperature, outside_Temperature, 0.09, false);
    UpdateMQTTNumberSensor(OT_Outside_Temperature_Name, mqtt_OT_outside_Temperature, OT_outside_Temperature, 0.09, false);
  }
}

void handleSetBoilerTemperature()
{
  // Set the boiler temperature
  Debug("ot.setBoilerTemperature("+String(boiler_SetPoint)+")");
  ot.setBoilerTemperature(boiler_SetPoint);
}

void handleSetDHWSetpoint()
{
  // Set the DHW setpoint
  Debug("ot.setDHWSetpoint("+String(dhw_SetPoint)+")"); 
  ot.setDHWSetpoint(dhw_SetPoint);
}

void handleGetBoilerTemperature()
{
  // Get the boiler temperature
  Debug("ot.getBoilerTemperature()");
  boiler_Temperature = ot.getBoilerTemperature();

  // Check if we have to send to MQTT
  if (WiFi.status() == WL_CONNECTED && mqttConnected()) {
    UpdateMQTTNumberSensor(Boiler_Temperature_Name, mqtt_boiler_Temperature, boiler_Temperature, 0.09, false);
  }
}

void handleGetDHWTemperature()
{
  Debug("ot.getDHWTemperature()");
  // Get the DHW temperature
  dhw_Temperature = ot.getDHWTemperature();

  // Check if we have to send to MQTT
  if (WiFi.status() == WL_CONNECTED && mqttConnected()) {
    if (UpdateMQTTNumberSensor(DHW_Temperature_Name, mqtt_dhw_Temperature, dhw_Temperature, 0.09, false)) {
      UpdateMQTTSetpointTemperature(DHW_Setpoint_Name,dhw_Temperature, false);
    }
  }
}

void handleGetReturnTemperature()
{
  Debug("ot.getReturnTemperature()");
  // Get the return temperature
  return_Temperature = ot.getReturnTemperature();

  // Check if we have to send to MQTT
  if (WiFi.status() == WL_CONNECTED && mqttConnected()) {
    UpdateMQTTNumberSensor(Return_Temperature_Name, mqtt_return_Temperature, return_Temperature, 0.09, false);
  }
}

void handleGetPressure()
{
  Debug("ot.getPressure()");
  // Get the pressure from OpenTherm
  pressure = ot.getPressure();

  // Check if we have to send to MQTT
  if (WiFi.status() == WL_CONNECTED && mqttConnected()) {
    UpdateMQTTNumberSensor(Pressure_Name, mqtt_pressure, pressure, 0.009, false);
  }
}

void handleGetFlowRate()
{
  Debug("getDHWFlowrate()");
  // Get the DHW flow rate from OpenTherm
  flowrate = getDHWFlowrate();

  /** Check if we have to send to MQTT
  if (client.connected()) {
    float delta = mqtt_flowrate-flowrate;
    if (delta<-0.01 or delta>0.01){ // value changed
      UpdateMQTTFlowRateSensor(FlowRate_Name,flowrate);
      mqtt_flowrate=flowrate;
    }
  } */
}

void handleGetFaultCode()
{
  Debug("ot.getFault()");
  // Get the fault code from OpenTherm
  FaultCode = ot.getFault();

  // Check if we have to send to MQTT
  if (WiFi.status() == WL_CONNECTED && mqttConnected()) {
    UpdateMQTTFaultCodeSensor(FaultCode_Name, mqtt_FaultCode, FaultCode, false);
  }
}

void handleGetThermostatTemperature()
{
  Debug("getThermostatTemperature()");
  // Get the thermostat temperature from the DS18B20 sensor
  sensors.requestTemperatures(); // Send the command to get temperature readings 
  currentTemperature = sensors.getTempCByIndex(0);

  // Check if we have to send to MQTT
  if (WiFi.status() == WL_CONNECTED && mqttConnected()) {
    UpdateMQTTNumberSensor(Thermostat_Temperature_Name, mqtt_currentTemperature, currentTemperature, 0.09, false);
    if (mqtttemptopic.length()==0) { // temptopic empty, use internal temperature
      SetMQTTTemperature(currentTemperature);
    }
  }
}

void handleOpenTherm() {

  // Array of handler functions to choose from, local in the function
  typedef void (*OTHandlerFunc)();
  OTHandlerFunc otHandlers[] = {
    handleSetBoilerStatus,
    handleSetBoilerTemperature,
    handleSetDHWSetpoint,
    handleGetBoilerTemperature,
    handleGetDHWTemperature,
    handleGetReturnTemperature,
    handleGetOutSideTemp,
    handleGetPressure,
    handleGetFlowRate,
    handleGetFaultCode,
    handleGetThermostatTemperature
  };
  const int OTHandlerCount = sizeof(otHandlers) / sizeof(otHandlers[0]);

  // Dispatcher: execute the correct handler and increment the index
  otHandlers[OpenThermCommandIndex]();
  OpenThermCommandIndex = (OpenThermCommandIndex + 1) % OTHandlerCount;
}


String CommandTopic(const char* DeviceName){
  return String(host)+String("/light/")+String(DeviceName)+String("/set");
}

String SetpointCommandTopic(const char* DeviceName){
  return String(host)+"/climate/"+String(DeviceName)+"/cmd_temp";
}

String NumberCommandTopic(const char* DeviceName){
  return String(host)+"/number/"+String(DeviceName)+"/set";
}


void LogMQTT(const char* topic, const char* payloadstr, const char* length, const char* logtext) {
  Error("ERROR: Unknown Message ("+String(payloadstr)+") with length "+String(length)+" received on topic "+String(topic)+", with log "+String(logtext));
}

bool HandleClimateMode(const char* mode)
{
  bool CommandSucceeded=true;
  if (String(mode).equals(String(climate_Mode))) {
    Error("Climate mode unchanged, ignoring command");
  } else {
    // Init PID calculater
    InitPID();

    if (String(mode).equals("off") or String(mode).equals("heat") or String(mode).equals("cool") or String(mode).equals("auto")) {
      Info("Setting clime mode to "+String(mode));
      climate_Mode=mode;
      if (String(mode).equals("off")) {
        // when setting to off, also correct boiler setpoint to prevent unwanted heating
        boiler_SetPoint=0;
      }
      DelayedSaveConfig();
    } else {
      Error("Unknown payload for Climate mode command");
      // sendback current mode to requester, so trick program into making it think it has to communicate
      mqtt_climate_Mode="abcd"; // random value, so the program thinks it is changed next time it wants to communicate
      CommandSucceeded=false;
    }
  }
  return CommandSucceeded;
}

const char* SetpointIntToString(int value) {
  if (value==OFF) {
    return "off";
  } else if (value==HEAT) {
    return "heat";
  } else if (value==COOL) {
    return "cool";
  } else if (value==AUTO) {
    return "auto";
  } else {
    return "unknown";
  }
}

void SetMQTTTemperature(float value) {
  // handle newly found mqttTemperature
  t_last_tempreceived=millis();
  mqttTemperature=value; 

  // additional actions if first time received
  if (!insideTemperatureReceived) {
    Info("First temperature ("+String(value)+") received, ready for climate mode");
    insideTemperatureReceived=true; // Make sure we do this only once ;-)
    InitPID();
  }
}

void SetMQTTOutsideTemperature(float value) {
  // handle newly found mqttTemperature
  t_last_outsidetempreceived=millis();
  mqttOutsideTemperature=value; 

  // additional actions if first time received
  if (!outsideTemperatureReceived) {
    Info("First outside temperature ("+String(value)+") received, ready for weather dependent mode");
    outsideTemperatureReceived=true; // Make sure we do this only once ;-)

    // this is our first measurement, initialize the array
    for (int i=0;i<NUMBEROFMEASUREMENTS;i++) {
      outsidetemp[i]=outside_Temperature;
    }

  }
}

bool HandleDHWMode(const char* mode) 
{
  bool succeeded=true;
  if (String(mode).equals("off")) {
    enableHotWater=false;
  } else if (String(mode).equals("heat")) {
    enableHotWater=true;
  } else {
    Error("Unknown payload for dhw setpoint mode command");
    // sendback current mode to requester, so trick program into making it think it has to communicate
    if (enableHotWater) {
      mqtt_enable_HotWater=false;
    } else {
      mqtt_enable_HotWater=true;
    }
    succeeded=false;
  }
  return succeeded;
}

void DelayedSaveConfig() 
{
  t_save_config = millis()+ConfigSaveDelay; // timestamp for delayed save
  ClimateConfigSaved=false;
}

bool HandleSwitch(bool *Switch, bool *mqtt_switch, const char* mode) 
{
  bool succeeded=true;
  // we have a match
  if (String(mode).equals("ON")){
    *Switch=true;
  } else if (String(mode).equals("OFF")) {
    *Switch=false;
  } else {
    Error("unknown state for enabletoggle");
    // make sure we communicate back the current status
    if (*Switch) {
      *mqtt_switch=false;
    } else {
      *mqtt_switch=true;
    }
    succeeded=false;
  }
  return succeeded;
}

bool handleClimateSetpoint(float setpoint) {
    climate_SetPoint=setpoint;
    InitPID();
    DelayedSaveConfig();
  return true;
}

String extractRelevantStringFromPayload(const char* payload) {
  // Define the relevant keys
  const char* keys[] = { "value", "state", "temperature", "svalue", "svalue1" };
  const int keyCount = sizeof(keys) / sizeof(keys[0]);

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    // No JSON, return original payload
    return String(payload);
  }

  // Try to find relevant keys
  for (int i = 0; i < keyCount; i++) {
    if (doc[keys[i]].is<float>()) return String(doc[keys[i]].as<float>());
    if (doc[keys[i]].is<const char*>()) return String(doc[keys[i]].as<const char*>());
    if (doc[keys[i]].is<int>()) return String(doc[keys[i]].as<int>());
  }

  // No relevant key found, try root value
  if (doc.is<float>()) return String(doc.as<float>());
  if (doc.is<const char*>()) return String(doc.as<const char*>());
  if (doc.is<int>()) return String(doc.as<int>());

  // fallback: return entire JSON as string
  String result;
  serializeJson(doc, result);
  return result;
}

int getIdxFromPayload(const char* payload) {
  // Parse the JSON payload to extract the idx value
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    Error("Failed to parse JSON payload: " + String(error.c_str()));
    return -1; // Return -1 if parsing fails
  }
  
  // Check if idx exists and is an integer
  if (doc["idx"].is<int>()) {
    return doc["idx"].as<int>();
  }
  
  Error("No valid idx found in payload");
  return -1; // Return -1 if idx is not found or not an integer
}

// Handler for Domoticz device readings
void handleDomoticzOutputTopic(const String& value, const char* payloadstr) {
  int idx = getIdxFromPayload(payloadstr);
  Info("Received domoticz device reading: "+String(idx)+","+String(payloadstr));
  if (mqtttemptopic.equals(String(idx))) {
    SetMQTTTemperature(value.toFloat());
  }
  if (mqttoutsidetemptopic.equals(String(idx))) {
    SetMQTTOutsideTemperature(value.toFloat());
  }
}

// Generic handler to change an MQTT topic and resubscribe
bool handleMQTTTopicChange(String& topicVar, const String& newValue) {
  if (topicVar.length() > 0) {
    mqttUnsubscribe(topicVar.c_str()); // Unsubscribe from old topic
  }
  topicVar = newValue;
  mqttSubscribe(topicVar.c_str(), 0);    // Subscribe to new topic
  return true; // Config must be saved
}

void MQTTcallback(char* topic, byte* payload, unsigned int length) {
  // get vars from callback
  String topicstr=String(topic);
  char payloadstr[1024];
  strncpy(payloadstr,(char *)payload,length);
  payloadstr[length]='\0';

  String value = extractRelevantStringFromPayload(payloadstr);

  if (!topicstr.equals(domoticzoutputtopic)) { // prevent flooding debug log with updates from domoticzdevices
    Info("Received message on topic ["+topicstr+"], payload: ["+payloadstr+"]");
  }

  // Assume succesful command 
  bool CommandSucceeded=false;
  
  // Domoticz devices
  if (topicstr.equals(domoticzoutputtopic)) {
    handleDomoticzOutputTopic(value, payloadstr);

  // climate mode
  } else if (topicstr.equals(host+"/climate/"+String(Climate_Name)+"/mode/set")) {
    CommandSucceeded=HandleClimateMode(value.c_str());

  // Climate setpoint temperature command
  } else if (topicstr.equals(SetpointCommandTopic(Climate_Name))) {
    CommandSucceeded = handleClimateSetpoint(value.toFloat());

  // DHW Setpoint mode receive
  } else if (topicstr.equals(host+"/climate/"+String(DHW_Setpoint_Name)+"/mode/set")) {
    CommandSucceeded=HandleDHWMode(value.c_str());

  // Handle Enable Hotwater switch command
  } else if (topicstr.equals(CommandTopic(EnableHotWater_Name))) {
    CommandSucceeded=HandleSwitch(&enableHotWater, &mqtt_enable_HotWater, value.c_str());

  // Weather Dependent Mode command
  } else if (topicstr.equals(CommandTopic(Weather_Dependent_Mode_Name))) {
    CommandSucceeded=HandleSwitch(&Weather_Dependent_Mode, &mqtt_Weather_Dependent_Mode, value.c_str());
    if (CommandSucceeded) {
      InitPID();
    }

  // Holiday Mode command
  } else if (topicstr.equals(CommandTopic(Holiday_Mode_Name))) {
    CommandSucceeded=HandleSwitch(&Holiday_Mode, &mqtt_Holiday_Mode, value.c_str());
    if (CommandSucceeded) {
      InitPID();
    }

  // Handle debug switch command
  } else if (topicstr.equals(CommandTopic(Debug_Name))) {
    Info("Received debug command: "+value);
    CommandSucceeded=HandleSwitch(&debug, &mqtt_debug, value.c_str());
    Info("Debugging switched "+String(value));

  // Handle info switch command
  } else if (topicstr.equals(CommandTopic(Info_Name))) {
    Info("Received info command: "+value);
    CommandSucceeded=HandleSwitch(&infotomqtt, &mqtt_infotomqtt, value.c_str());
    Info("Info logging switched "+String(value));

  // Handle enable log to SPIFFS switch command
  } else if (topicstr.equals(CommandTopic(EnableLogToSPIFFS_Name))) {
    Info("Received enable log to SPIFFS command: "+value);
    CommandSucceeded=HandleSwitch(&enableLogToSPIFFS, &mqtt_enableLogToSPIFFS, value.c_str());
    Info("Log to SPIFFS switched "+String(value));

  // DHW Setpoint temperature commands
  } else if (topicstr.equals(SetpointCommandTopic(DHW_Setpoint_Name))) {
    dhw_SetPoint=String(payloadstr).toFloat();
    CommandSucceeded=true; // we handled the command

  // MQTT temperature received
  } else if (topicstr.equals(mqtttemptopic)) {
    SetMQTTTemperature(value.toFloat()); // Just try to convert

  // MQTT outsidetemperature received
  } else if (topicstr.equals(mqttoutsidetemptopic)) {
    SetMQTTOutsideTemperature(value.toFloat()); // Just try to convert

  // Boiler Vars
  } else if (topicstr.equals(NumberCommandTopic(MinBoilerTemp_Name))) {
    MinBoilerTemp=value.toFloat();
    CommandSucceeded=true;
  } else if (topicstr.equals(NumberCommandTopic(MaxBoilerTemp_Name))) {
    MaxBoilerTemp=value.toFloat();
    CommandSucceeded=true;
  } else if (topicstr.equals(NumberCommandTopic(MinimumTempDifference_Name))) {
    minimumTempDifference=value.toFloat();
    CommandSucceeded=true;
  } else if (topicstr.equals(NumberCommandTopic(FrostProtectionSetPoint_Name))) {
    FrostProtectionSetPoint=value.toFloat();
    CommandSucceeded=true;
  } else if (topicstr.equals(NumberCommandTopic(BoilerTempAtPlus20_Name))) {
    BoilerTempAtPlus20=value.toFloat();
    CommandSucceeded=true;
  } else if (topicstr.equals(NumberCommandTopic(BoilerTempAtMinus10_Name))) {
    BoilerTempAtMinus10=value.toFloat();
    CommandSucceeded=true;
  } else if (topicstr.equals(NumberCommandTopic(SwitchHeatingOffAt_Name))) {
    SwitchHeatingOffAt=value.toFloat();
    CommandSucceeded=true;
  } else if (topicstr.equals(NumberCommandTopic(ReferenceRoomCompensation_Name))) {
    ReferenceRoomCompensation=value.toFloat();
    CommandSucceeded=true;
  } else if (topicstr.equals(NumberCommandTopic(KP_Name))) {
    KP=value.toFloat();
    CommandSucceeded=true;
  } else if (topicstr.equals(NumberCommandTopic(KI_Name))) {
    KI=value.toFloat();
    CommandSucceeded=true;
  } else if (topicstr.equals(NumberCommandTopic(KD_Name))) {
    KD=String(payloadstr).toFloat();
    CommandSucceeded=true;
  } else if (topicstr.equals(String(host)+"/select/"+String(Curvature_Name)+"/set")) {
    Curvature=getCurvatureIntFromString(payloadstr);
    CommandSucceeded=true;
  } else if (topicstr.equals(String(host)+"/text/"+String(MQTT_TempTopic_Name)+"/set")) {
    CommandSucceeded = handleMQTTTopicChange(mqtttemptopic, value);
  }
  else if (topicstr.equals(String(host)+"/text/"+String(MQTT_OutsideTempTopic_Name)+"/set")) {
    Info("Setting outsideTempTopic to "+String(payloadstr));
    CommandSucceeded = handleMQTTTopicChange(mqttoutsidetemptopic, value);

  } else if (topicstr.equals(host+"/button/"+String(Reset_Device_Name)+"/set")) {
    if (value.equals("PRESS")) {
      if (millis() - resetButtonSubscribeTime < 5000) {
        Info("Ignoring possible retained reset button press message");
      } else {
        handleReset();
        CommandSucceeded = true;
      }
    }

  // Unknown topic (this code should never be reached, indicates programming logic error)  
  } else {
    LogMQTT(topicstr.c_str(),payloadstr,String(length).c_str(),"unknown topic");
  }
}

void SubScribeToDomoticz() {
  mqttSubscribe(domoticzoutputtopic.c_str(), 0);
}

void reconnect()
{
  if (usemqtt) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.print("WiFi not connected, can't connect to MQTT broker");
      return;
    }
    if (!mqttConnected()) {
      Serial.print("Attempting MQTT connection...");
      String clientId = "esp_ot_" + WiFi.macAddress();
      clientId.replace(":", "");
      mqttSetId(clientId.c_str());  // Set unique client ID based on MAC
      mqttSetKeepAliveInterval(300);  // Keep-alive in seconden
      mqttClientId = clientId;
      #if MQTT_LIBRARY != 0
      if (usemqttauthentication) {
        mqttSetUsernamePassword(mqttuser.c_str(), mqttpass.c_str());
      }
      #endif
      #if MQTT_LIBRARY == 2
      mqttConnect(mqttserver.c_str(), mqttport);
      #else
      bool mqttconnected = mqttConnect(mqttserver.c_str(), mqttport);
      if (mqttconnected) {
        sensorIndex = 0; // Reset sensor index for discovery
        PublishAllMQTTSensors();
        Info("Succesfully (re)connected, starting operations");
        Info("None, all OK");
        if (mqtttemptopic.toInt()>0 or mqttoutsidetemptopic.toInt()>0) { // apparently it is a domoticz idx. So listen to domoticz/out
          SubScribeToDomoticz();
        }  
      } else {
        Serial.print("failed, rc=");
        Serial.print(mqttConnectError());
      }
      #endif
    }
  }
}

void addDeviceToJson(JsonDocument *json) {
  JsonObject dev = (*json)["dev"].to<JsonObject>();
  String MAC = WiFi.macAddress();
  MAC.replace(":", "");
  dev["ids"] = MAC;
  dev["name"] = host;
  dev["sw"] = String(host)+"_"+String(__DATE__)+"_"+String(__TIME__);
  dev["mdl"] = "d1_mini";
  dev["mf"] = "espressif";
  dev["cu"] = "http://"+WiFi.localIP().toString()+"/";  
}

void PublishMQTTSwitch(const char* uniquename)
{
  Info("Publishing switch: " + String(uniquename));
  JsonDocument json;

  // Construct JSON config message
  json["name"] = uniquename;
  json["unique_id"] = host+"_"+uniquename;
  json["cmd_t"] = host+"/light/"+String(uniquename)+"/set";
  json["stat_t"] = host+"/light/"+String(uniquename)+"/state";
  json["availability_topic"] = "domesphelper/status";

  addDeviceToJson(&json);

  // Publish config message
  char buf[1024];
  serializeJson(json, buf);
  mqttPublish((String(mqttautodiscoverytopic)+"/light/"+host+"/"+String(uniquename)+"/config").c_str(), buf, mqttpersistence, MQTT_QOS_CONFIG);

  // subscribe if need to listen to commands
  mqttSubscribe((host+"/light/"+String(uniquename)+"/set").c_str(), 0);
}

bool UpdateMQTTSwitch(const char* uniquename, bool& currentValue, bool newValue, bool force)
{
  if (!force && (isPublishingAllSensors || !firstPublishDone)) return false;

  if (force || currentValue != newValue) {
    if (WiFi.status() == WL_CONNECTED && mqttConnected()) {
      // Serial.println("UpdateMQTTSwitch");
      // publish state message
      mqttPublish((host+"/light/"+String(uniquename)+"/state").c_str(),newValue?"ON":"OFF",mqttpersistence,MQTT_QOS_STATE);
    }
    currentValue = newValue;
    return true;
  }
  return false;
}

void PublishMQTTButton(const char* uniquename)
{
  Info("Publishing button: " + String(uniquename));
  JsonDocument json;

  // Construct JSON config message
  json["name"] = uniquename;
  json["unique_id"] = host+"_"+uniquename;
  json["cmd_t"] = host+"/button/"+String(uniquename)+"/set";
  json["payload_press"] = "PRESS";
  json["availability_topic"] = "domesphelper/status";

  addDeviceToJson(&json);

  // Publish config message
  char buf[1024];
  serializeJson(json, buf);
  mqttPublish((String(mqttautodiscoverytopic)+"/button/"+host+"/"+String(uniquename)+"/config").c_str(), buf, mqttpersistence, MQTT_QOS_CONFIG);

  // subscribe if need to listen to commands
  mqttSubscribe((host+"/button/"+String(uniquename)+"/set").c_str(), 0);

  // Record subscribe time for reset button to ignore retained messages
  if (String(uniquename).equals(String(Reset_Device_Name))) {
    resetButtonSubscribeTime = millis();
  }
}

void PublishMQTTBinarySensor(const char* uniquename, const char* deviceclass)
{
  Info("Publishing binary sensor: " + String(uniquename));
  JsonDocument json;

  // Construct JSON config message
  json["name"] = uniquename;
  json["unique_id"] = host+"_"+uniquename;
  json["stat_t"] = host+"/binary_sensor/"+String(uniquename)+"/state";
  json["payload_on"] = true;
  json["payload_off"] = false;
  json["val_tpl"] = "{{ value_json.value }}";
  if (!String(deviceclass).equals("None")) {
    json["device_class"] =deviceclass;
  }
  json["availability_topic"] = "domesphelper/status";

  addDeviceToJson(&json);

  // Publish config message
  char buf[1024];
  serializeJson(json, buf);
  mqttPublish((String(mqttautodiscoverytopic)+"/binary_sensor/"+host+"/"+String(uniquename)+"/config").c_str(), buf, mqttpersistence, MQTT_QOS_CONFIG);
}

void UpdateMQTTBinarySensor(const char* uniquename, bool& currentValue, bool newValue, bool force)
{
  if (!force && (isPublishingAllSensors || !firstPublishDone)) return;

  if (force || currentValue != newValue) {
    if (WiFi.status() == WL_CONNECTED && mqttConnected()) {
      Serial.println("UpdateBinarySensor");

       JsonDocument json;
       json["value"] = newValue;

       char message[512];
       serializeJson(json,message);

      // publish state message
      mqttPublish((host+"/binary_sensor/"+String(uniquename)+"/state").c_str(),message,mqttpersistence,MQTT_QOS_STATE);
    }
    currentValue = newValue;
  }
}

void PublishMQTTNumberSensor(const char* uniquename, const char* unit = "", const char* device_class = "")
{
  Info("Publishing number sensor: " + String(uniquename));
  JsonDocument json;

  // Create message
  json["state_topic"] = host+"/sensor/"+String(uniquename)+"/state";
  json["json_attributes_topic"] = host+"/sensor/"+String(uniquename)+"/state";
  json["value_template"] =  "{{ value_json.value }}";
  json["name"] = uniquename;
  json["unique_id"] = host+"_"+uniquename;
  json["availability_topic"] = "domesphelper/status";
  if (strlen(unit) > 0) {
    json["unit_of_measurement"] = unit;
  }
  if (strlen(device_class) > 0) {
    json["device_class"] = device_class;
  }

  addDeviceToJson(&json);

  // publish the Message
  char buf[1024];
  serializeJson(json, buf);
  mqttPublish((String(mqttautodiscoverytopic)+"/sensor/"+host+"/"+String(uniquename)+"/config").c_str(), buf, mqttpersistence, MQTT_QOS_CONFIG);
}



bool UpdateMQTTNumberSensor(const char* uniquename, float& currentValue, float newValue, float tolerance, bool force)
{
  if (!force && (isPublishingAllSensors || !firstPublishDone)) return false;

  if (force || fabs(currentValue - newValue) > tolerance) {
    if (WiFi.status() == WL_CONNECTED && mqttConnected()) {
      Serial.println("UpdateMQTTGenericSensor");
      JsonDocument json;

      // Create message
      char state[128];
      json["value"] =  float(int(newValue*100))/100;   // ensures round to 1 decimal behind the comma
      serializeJson(json, state);  // buf now contains the json 
      mqttPublish((host+"/sensor/"+String(uniquename)+"/state").c_str(),state,mqttpersistence,MQTT_QOS_STATE);
    }
    currentValue = newValue;
    return true;
  }
  return false;
}

void PublishMQTTPercentageSensor(const char* uniquename)
{
  Info("Publishing percentage sensor: " + String(uniquename));
  JsonDocument json;

  // Create message
  json["value_template"] =  "{{ value_json.value }}";
  json["device_class"] = "power_factor";
  json["unit_of_measurement"] = "%";
  json["state_topic"] = host+"/sensor/"+String(uniquename)+"/state";
  json["json_attributes_topic"] = host+"/sensor/"+String(uniquename)+"/state";
  json["name"] = uniquename;
  json["unique_id"] = host+"_"+uniquename;
  json["availability_topic"] = "domesphelper/status";

  addDeviceToJson(&json);

  // publish the Message
  char buf[1024];
  serializeJson(json, buf);
  mqttPublish((String(mqttautodiscoverytopic)+"/sensor/"+host+"/"+String(uniquename)+"/config").c_str(), buf, mqttpersistence, MQTT_QOS_CONFIG);
}

void UpdateMQTTPercentageSensor(const char* uniquename, float& currentValue, float newValue, bool force)
{
  if (!force && (isPublishingAllSensors || !firstPublishDone)) return;

  if (force || fabs(currentValue - newValue) > 0.01) {
    if (WiFi.status() == WL_CONNECTED && mqttConnected()) {
      Serial.println("UpdateMQTTPercentageSensor");
      JsonDocument json;

      // Create message
      char data[128];
      json["value"]=newValue;
      serializeJson(json,data);
     
      mqttPublish((host+"/sensor/"+String(uniquename)+"/state").c_str(),data,mqttpersistence,0);
    }
    currentValue = newValue;
  }
}

void PublishMQTTFaultCodeSensor(const char* uniquename)
{
  Info("Publishing fault code sensor: " + String(uniquename));
  JsonDocument json;

  // Create message
  json["value_template"] =  "{{ value_json.value }}";
  json["unit_of_measurement"] = "";
  json["state_topic"] = host+"/sensor/"+String(uniquename)+"/state";
  json["json_attributes_topic"] = host+"/sensor/"+String(uniquename)+"/state";
  json["name"] = uniquename;
  json["unique_id"] = host+"_"+uniquename;
  json["availability_topic"] = "domesphelper/status";

  addDeviceToJson(&json);

  // publish the Message
  char buf[1024];
  serializeJson(json, buf);
  mqttPublish((String(mqttautodiscoverytopic)+"/sensor/"+host+"/"+String(uniquename)+"/config").c_str(), buf, mqttpersistence, MQTT_QOS_CONFIG);
}

void UpdateMQTTFaultCodeSensor(const char* uniquename, unsigned char& currentValue, unsigned char newValue, bool force)
{
  if (!force && (isPublishingAllSensors || !firstPublishDone)) return;

  if (force || currentValue != newValue) {
    if (WiFi.status() == WL_CONNECTED && mqttConnected()) {
      Serial.println("UpdateMQTTFaultCodeSensor");
        // Create message
      char state[128];
      JsonDocument json;
      json["value"] = newValue;
      serializeJson(json, state);  // buf now contains the json

      mqttPublish((host+"/sensor/"+String(uniquename)+"/state").c_str(),state,mqttpersistence,0);
    }
    currentValue = newValue;
  }
}

void PublishMQTTSetpoint(const char* uniquename, int mintemp, int maxtemp, bool includeCooling)
{
  Info("Publishing setpoint: " + String(uniquename));
  JsonDocument json;

  // Create message
  json["min_temp"] = mintemp;
  json["max_temp"] = maxtemp;

  json["name"] = uniquename;
  json["unique_id"] = host+"_"+uniquename;
  json["temp_cmd_t"] = host+"/climate/"+String(uniquename)+"/cmd_temp";
  json["temp_stat_t"] = host+"/climate/"+String(uniquename)+"/state";
  json["temp_stat_tpl"] = "{{value_json.seltemp}}";
  json["temp_step"] = 0.5;
  json["temp_unit"] = "C";
  json["precision"] = 0.1;
  json["curr_temp_t"] = host+"/climate/"+String(uniquename)+"/Air_temperature";
  json["curr_temp_tpl"] = "{{ value_json.value }}";
  JsonArray modes = json["modes"].to<JsonArray>();
  modes.add("off");
  modes.add("heat");
  if (includeCooling) {
    modes.add("cool");
    modes.add("auto");
  }
  json["mode_stat_t"] = host+"/climate/"+String(uniquename)+"/mode";
  json["mode_cmd_t"] = host+"/climate/"+String(uniquename)+"/mode/set";
  json["mode_stat_tpl"] =  "{{ {"+String(OFF)+": \"off\", "+String(HEAT)+": \"heat\", "+String(COOL)+": \"cool\", "+String(AUTO)+": \"auto\"}[value_json.value] | default('off') }}";
  json["availability_topic"] = "domesphelper/status";

  addDeviceToJson(&json);

  // publish the Message
  char buf[2024];
  serializeJson(json, buf);
  mqttPublish((String(mqttautodiscoverytopic)+"/climate/"+host+"/"+String(uniquename)+"/config").c_str(), buf, mqttpersistence, MQTT_QOS_CONFIG);
  mqttSubscribe((host+"/climate/"+String(uniquename)+"/mode/set").c_str(), 0);
  mqttSubscribe((host+"/climate/"+String(uniquename)+"/cmd_temp").c_str(), 0);
}

void UpdateMQTTSetpointTemperature(const char* uniquename,float value, bool force)
{
  if (!force && (isPublishingAllSensors || !firstPublishDone)) return;

  if (WiFi.status() == WL_CONNECTED && mqttConnected()) {
    Serial.println("UpdateMQTTSetpointtemperature");
    JsonDocument json;

    mqttPublish((host+"/climate/"+String(uniquename)+"/Air_temperature").c_str(),("{ \"value\": "+String(value)+" }").c_str(),mqttpersistence,0);
  }
}

void PublishMQTTNumber(const char* uniquename, int min, int max, float step, bool isSlider)
{
  Info("Publishing number: " + String(uniquename));
  // Debug("PublishMQTTNumber");
  JsonDocument json;

  // Create message
  json["name"] = uniquename;
  json["unique_id"] = host+"_"+uniquename;
  json["stat_t"] = host+"/number/"+String(uniquename)+"/state";
  json["cmd_t"] = host+"/number/"+String(uniquename)+"/set";

  json["min"] = min;
  json["max"] = max;
  json["step"] = step;
  json["unit_of_measurement"] = "°C";
  if (isSlider) {
    json["mode"] = "slider"; 
  } else {
    json["mode"] = "box";
  }
  json["availability_topic"] = "domesphelper/status";

  addDeviceToJson(&json);

  // publish the Message
  char buf[1024];
  serializeJson(json, buf);
  mqttPublish((String(mqttautodiscoverytopic)+"/number/"+host+"/"+String(uniquename)+"/config").c_str(), buf, mqttpersistence, MQTT_QOS_CONFIG);
  mqttSubscribe((host+"/number/"+String(uniquename)+"/set").c_str(), 0);
  // Debug("Publish to "+String(mqttautodiscoverytopic)+"/number/"+host+"/"+String(uniquename)+"/config");
}

bool UpdateMQTTNumber(const char* uniquename, float& currentValue, float newValue, float tolerance, bool force)
{
  if (!force && (isPublishingAllSensors || !firstPublishDone)) return false;

  if (force || fabs(currentValue - newValue) > tolerance) {
    if (WiFi.status() == WL_CONNECTED && mqttConnected()) {
      Serial.println("UpdateMQTTNumber");
      mqttPublish((host+"/number/"+String(uniquename)+"/state").c_str(),String(newValue).c_str(),mqttpersistence,0);
    }
    currentValue = newValue;
    return true;
  }
  return false;
}

void PublishMQTTText(const char* uniquename)
{
  Info("Publishing text: " + String(uniquename));
  // Debug("PublishMQTTText");
  JsonDocument json;

  // Create message
  json["name"] = uniquename;
  json["unique_id"] = host+"_"+uniquename;
  json["stat_t"] = host+"/text/"+String(uniquename)+"/state";
  json["cmd_t"] = host+"/text/"+String(uniquename)+"/set";
  // json["stat_tpl"] = "{{value_json.value}}";
  json["availability_topic"] = "domesphelper/status";

  addDeviceToJson(&json);

  // publish the Message
  char buf[1024];
  serializeJson(json, buf);
  mqttPublish((String(mqttautodiscoverytopic)+"/text/"+host+"/"+String(uniquename)+"/config").c_str(), buf, mqttpersistence, MQTT_QOS_CONFIG);
  mqttSubscribe((host+"/text/"+String(uniquename)+"/set").c_str(), 0);
  // Debug("Publish to "+String(mqttautodiscoverytopic)+"/text/"+host+"/"+String(uniquename)+"/config");
}

void UpdateMQTTText(const char* uniquename, String& currentValue, String newValue, bool force)
{
  bool shouldPublish = force;
  if (!newValue.equals(currentValue)) {
    shouldPublish = true;
  }

  if (!force && (isPublishingAllSensors || !firstPublishDone)) return;

  if (shouldPublish) {
    if (WiFi.status() == WL_CONNECTED && mqttConnected()) {
      mqttPublish((host+"/text/"+String(uniquename)+"/state").c_str(), newValue.c_str(), mqttpersistence, 0);
    }
  }

  currentValue = newValue;
}

void PublishMQTTTextSensor(const char* uniquename)
{
  Info("Publishing text sensor: " + String(uniquename));
  JsonDocument json;

  // Construct JSON config message for a read-only sensor
  json["name"] = uniquename;
  json["unique_id"] = host+"_"+uniquename;
  json["stat_t"] = host+"/sensor/"+String(uniquename)+"/state";
  json["value_template"] = "{{ value_json.value }}";
  // No cmd_t, so not editable
  json["availability_topic"] = "domesphelper/status";

  addDeviceToJson(&json);

  // Publish config message
  char buf[1024];
  serializeJson(json, buf);
  mqttPublish((String(mqttautodiscoverytopic)+"/sensor/"+host+"/"+String(uniquename)+"/config").c_str(), buf, mqttpersistence, MQTT_QOS_CONFIG);
}

bool UpdateMQTTTextSensor(const char* uniquename, const char* value, bool force)
{
  if (!force && (isPublishingAllSensors || !firstPublishDone)) return false;

  if (WiFi.status() == WL_CONNECTED && mqttConnected()) {
    Serial.println("UpdateMQTTTextSensor");
    JsonDocument json;

    // Construct JSON state message
    json["value"] = value;

    char state[128];
    serializeJson(json, state);  // state now contains the json

    // Publish state message
    return mqttPublish((host+"/sensor/"+String(uniquename)+"/state").c_str(), state, mqttpersistence, 0);
  }
  return false;
}


void PublishMQTTCurvatureSelect(const char* uniquename)
{
  Info("Publishing curvature select: " + String(uniquename));
  // Debug("PublishMQTTRefRoomCurvatureSelect");
  JsonDocument json;

  // Create message
  json["name"] = uniquename;
  json["unique_id"] = host+"_"+uniquename;
  json["stat_t"] = host+"/select/"+String(uniquename)+"/state";
  json["cmd_t"] = host+"/select/"+String(uniquename)+"/set";
  json["platform"] = "select";
  json["val_tpl"] = "{{ value_json.curvature }}";

  JsonArray options = json["options"].to<JsonArray>();
  options.add("none");
  options.add("small");
  options.add("medium");
  options.add("large");
  options.add("extralarge");
  json["availability_topic"] = "domesphelper/status";

  addDeviceToJson(&json);

  // publish the Message
  char buf[1024];
  serializeJson(json, buf);
  mqttPublish((String(mqttautodiscoverytopic)+"/select/"+host+"/"+String(uniquename)+"/config").c_str(), buf, mqttpersistence, MQTT_QOS_CONFIG);
  mqttSubscribe((host+"/select/"+String(uniquename)+"/set").c_str(), 0);
}

void UpdateMQTTCurvatureSelect(const char* uniquename,int value, bool force)
{
  if (!force && (isPublishingAllSensors || !firstPublishDone)) return;

  if (WiFi.status() == WL_CONNECTED && mqttConnected()) {
    Serial.println("UpdateMQTTCurvatureSelect");
    JsonDocument json;

    // Construct JSON config message
    json["curvature"] = getCurvatureStringFromInt(value);

    char jsonstr[128];
    serializeJson(json, jsonstr);  // conf now contains the json

    mqttPublish((host+"/select/"+String(uniquename)+"/state").c_str(),jsonstr,mqttpersistence,0);
  }
}

void UpdateClimateSetpointMode(bool force)
{
  if (!force && (isPublishingAllSensors || !firstPublishDone)) return;

  if (force || !climate_Mode.equals(mqtt_climate_Mode)) {
    if (climate_Mode.equals("off")) {
      UpdateMQTTSetpointMode(Climate_Name, OFF, force);
    } else if (climate_Mode.equals("heat")) {
      UpdateMQTTSetpointMode(Climate_Name, HEAT, force);
    } else if (climate_Mode.equals("cool")) {
      UpdateMQTTSetpointMode(Climate_Name, COOL, force);
    } else if (climate_Mode.equals("auto")) {
      UpdateMQTTSetpointMode(Climate_Name, AUTO, force);
    }
    mqtt_climate_Mode = climate_Mode;
  }
}

void UpdateMQTTSetpointMode(const char* uniquename,int value, bool force)
{
  if (!force && (isPublishingAllSensors || !firstPublishDone)) return;

  if (WiFi.status() == WL_CONNECTED && mqttConnected()) {
    Serial.println("UpdateMQTTSetpointmode");
    JsonDocument json;

    // Construct JSON config message
    // json["value"] = value;

    // char jsonstr[128];
    // serializeJson(json, jsonstr);  // conf now contains the json

    mqttPublish((host+"/climate/"+String(uniquename)+"/mode").c_str(),("{ \"value\": "+String(value)+" }").c_str(),mqttpersistence,0);
  }
}


bool UpdateMQTTSetpoint(const char* uniquename, float& currentValue, float newValue, bool force)
{
  if (!force && (isPublishingAllSensors || !firstPublishDone)) return false;

  if (force || fabs(currentValue - newValue) > 0.01) {
    if (WiFi.status() == WL_CONNECTED && mqttConnected()) {
      Serial.println("UpdateMQTTSetpoint");
      JsonDocument json;

      // Construct JSON config message
      json["seltemp"] = newValue;

      char value[128];
      serializeJson(json, value);  // conf now contains the json

      mqttPublish((host+"/climate/"+String(uniquename)+"/state").c_str(),value,mqttpersistence,0);
    }
    currentValue = newValue;
    return true;
  }
  return false;
}

void updateTime() {
  ms+=(unsigned long)(millis()-previousMillis);
  if (ms>= 1000)
  {
    ms -= 1000;
    s++;
    if (s > 59)
    {
      s = 0;
      m++;
      if (m > 59)
      {
        m = 0;
        h++;
        if (h > 23) 
        {
          h = 0;
          d++;
          if (d>364)
          {
            y++;
          }
        }
      }
    }
  }
  previousMillis=millis();
}

String getUptimeString() {
  // Calculate total seconds (approximate, ignoring leap years)
  unsigned long total_seconds = (unsigned long)y * 365 * 24 * 3600 + (unsigned long)d * 24 * 3600 + (unsigned long)h * 3600 + (unsigned long)m * 60 + s;
  
  unsigned long days = total_seconds / 86400;
  unsigned long hours = (total_seconds % 86400) / 3600;
  unsigned long minutes = (total_seconds % 3600) / 60;
  unsigned long seconds = total_seconds % 60;
  
  String timeStr = "";
  if (hours < 10) timeStr += "0";
  timeStr += String(hours) + ":";
  if (minutes < 10) timeStr += "0";
  timeStr += String(minutes) + ":";
  if (seconds < 10) timeStr += "0";
  timeStr += String(seconds);
  
  if (days > 0) {
    return String(days) + " days, " + timeStr;
  } else {
    return timeStr;
  }
}

unsigned long getUptimeSeconds() {
  // Calculate total seconds (approximate, ignoring leap years)
  return (unsigned long)y * 365 * 24 * 3600 + (unsigned long)d * 24 * 3600 + (unsigned long)h * 3600 + (unsigned long)m * 60 + s;
}

void SubscribeToTempTopics() {
  if (mqtttemptopic.length()>0 && mqtttemptopic.toInt()==0) {
    Info("Subscribing to temperature topic: " + mqtttemptopic);
    mqttSubscribe(mqtttemptopic.c_str(), 0);
  }
  if (mqttoutsidetemptopic.length()>0 && mqttoutsidetemptopic.toInt()==0) {
    Info("Subscribing to outside temperature topic: " + mqttoutsidetemptopic);
    mqttSubscribe(mqttoutsidetemptopic.c_str(), 0);
  }
}

bool PublishAllMQTTSensors()
{
  isPublishingAllSensors = true;
  switch (sensorIndex) {
    case 0:
      SubscribeToTempTopics();
      break;
    case 1:
      PublishMQTTNumberSensor(Boiler_Temperature_Name, "°C", "temperature");
      break;
    case 2:
      UpdateMQTTNumberSensor(Boiler_Temperature_Name, mqtt_boiler_Temperature, boiler_Temperature, 0.09, true);
      break;
    case 3:
      PublishMQTTNumberSensor(DHW_Temperature_Name, "°C", "temperature");
      break;
    case 4:
      UpdateMQTTNumberSensor(DHW_Temperature_Name, mqtt_dhw_Temperature, dhw_Temperature, 0.09, true);
      break;
    case 5:
      PublishMQTTNumberSensor(Return_Temperature_Name, "°C", "temperature");
      break;
    case 6:
      UpdateMQTTNumberSensor(Return_Temperature_Name, mqtt_return_Temperature, return_Temperature, 0.09, true);
      break;
    case 7:
      PublishMQTTNumberSensor(Thermostat_Temperature_Name, "°C", "temperature");
      break;
    case 8:
      UpdateMQTTNumberSensor(Thermostat_Temperature_Name, mqtt_currentTemperature, currentTemperature, 0.09, true);
      break;
    case 9:
      PublishMQTTNumberSensor(Outside_Temperature_Name, "°C", "temperature");
      break;
    case 10:
      UpdateMQTTNumberSensor(Outside_Temperature_Name, mqtt_outside_Temperature, outside_Temperature, 0.09, true);
      break;
    case 11:
      PublishMQTTNumberSensor(OT_Outside_Temperature_Name, "°C", "temperature");
      break;
    case 12:
      UpdateMQTTNumberSensor(OT_Outside_Temperature_Name, mqtt_OT_outside_Temperature, OT_outside_Temperature, 0.09, true);
      break;
    case 13:
      PublishMQTTNumberSensor(Pressure_Name, "bar", "pressure");
      break;
    case 14:
      UpdateMQTTNumberSensor(Pressure_Name, mqtt_pressure, pressure, 0.009, true);
      break;
    case 15:
      PublishMQTTPercentageSensor(Modulation_Name);
      break;
    case 16:
      UpdateMQTTPercentageSensor(Modulation_Name, mqtt_modulation, modulation, true);
      break;
    case 17:
      PublishMQTTFaultCodeSensor(FaultCode_Name);
      break;
    case 18:
      UpdateMQTTFaultCodeSensor(FaultCode_Name, mqtt_FaultCode, FaultCode, true);
      break;
    case 19:
      PublishMQTTNumberSensor(P_Name);
      break;
    case 20:
      UpdateMQTTNumberSensor(P_Name, mqtt_p, P, 0.01, true);
      break;
    case 21:
      PublishMQTTNumberSensor(I_Name);
      break;
    case 22:
      UpdateMQTTNumberSensor(I_Name, mqtt_i, I, 0.01, true);
      break;
    case 23:
      PublishMQTTNumberSensor(D_Name);
      break;
    case 24:
      UpdateMQTTNumberSensor(D_Name, mqtt_d, D, 0.01, true);
      break;
    case 25:
      PublishMQTTBinarySensor(FlameActive_Name,"heat");
      break;
    case 26:
      UpdateMQTTBinarySensor(FlameActive_Name, mqtt_Flame, Flame, true);
      break;
    case 27:
      PublishMQTTBinarySensor(FaultActive_Name,"problem");
      break;
    case 28:
      UpdateMQTTBinarySensor(FaultActive_Name, mqtt_Fault, Fault, true);
      break;
    case 29:
      PublishMQTTBinarySensor(DiagnosticActive_Name,"problem");
      break;
    case 30:
      UpdateMQTTBinarySensor(DiagnosticActive_Name, mqtt_Diagnostic, Diagnostic, true);
      break;
    case 31:
      PublishMQTTBinarySensor(CoolingActive_Name,"heat");
      break;
    case 32:
      UpdateMQTTBinarySensor(CoolingActive_Name, mqtt_Cooling, Cooling, true);
      break;
    case 33:
      PublishMQTTBinarySensor(CentralHeatingActive_Name,"heat");
      break;
    case 34:
      UpdateMQTTBinarySensor(CentralHeatingActive_Name, mqtt_CentralHeating, CentralHeating, true);
      break;
    case 35:
      PublishMQTTBinarySensor(HotWaterActive_Name,"heat");
      break;
    case 36:
      UpdateMQTTBinarySensor(HotWaterActive_Name, mqtt_HotWater, HotWater, true);
      break;
    case 37:
      PublishMQTTBinarySensor(FrostProtectionActive_Name,"cold");
      break;
    case 38:
      UpdateMQTTBinarySensor(FrostProtectionActive_Name, mqtt_FrostProtectionActive, FrostProtectionActive, true);
      break;
    case 39:
      PublishMQTTSwitch(EnableHotWater_Name);
      break;
    case 40:
      UpdateMQTTSwitch(EnableHotWater_Name, mqtt_enable_HotWater, enableHotWater, true);
      break;
    case 41:
      PublishMQTTSwitch(Weather_Dependent_Mode_Name);
      break;
    case 42:
      UpdateMQTTSwitch(Weather_Dependent_Mode_Name, mqtt_Weather_Dependent_Mode, Weather_Dependent_Mode, true);
      break;
    case 43:
      PublishMQTTSwitch(Holiday_Mode_Name);
      break;
    case 44:
      UpdateMQTTSwitch(Holiday_Mode_Name, mqtt_Holiday_Mode, Holiday_Mode, true);
      break;
    case 45:
      PublishMQTTSwitch(Debug_Name);
      break;
    case 46:
      UpdateMQTTSwitch(Debug_Name, mqtt_debug, debug, true);
      break;
    case 47:
      PublishMQTTSwitch(Info_Name);
      break;
    case 48:
      UpdateMQTTSwitch(Info_Name, mqtt_infotomqtt, infotomqtt, true);
      break;
    case 49:
      PublishMQTTSwitch(EnableLogToSPIFFS_Name);
      break;
    case 50:
      UpdateMQTTSwitch(EnableLogToSPIFFS_Name, mqtt_enableLogToSPIFFS, enableLogToSPIFFS, true);
      break;
    case 51:
      PublishMQTTSetpoint(DHW_Setpoint_Name,10,90,false);
      break;
    case 52:
      UpdateMQTTSetpoint(DHW_Setpoint_Name, mqtt_dhw_setpoint, dhw_SetPoint, true);
      UpdateMQTTSetpointTemperature(DHW_Setpoint_Name, dhw_Temperature, true);
      UpdateMQTTSetpointMode(DHW_Setpoint_Name, enableHotWater ? 1 : 0, true);
      break;
    case 53:
      PublishMQTTSetpoint(Climate_Name,5,30,true);
      break;
    case 54:
      UpdateMQTTSetpoint(Climate_Name, mqtt_climate_setpoint, climate_SetPoint, true);
      UpdateMQTTSetpointTemperature(Climate_Name, mqttTemperature, true);
      UpdateClimateSetpointMode(true);
      break;
    case 55:
      PublishMQTTNumber(MinBoilerTemp_Name,10,50,0.5,true);
      break;
    case 56:
      UpdateMQTTNumber(MinBoilerTemp_Name, mqtt_minboilertemp, MinBoilerTemp, 0.5, true);
      break;
    case 57:
      PublishMQTTNumber(MaxBoilerTemp_Name,30,90,0.5,true);
      break;
    case 58:
      UpdateMQTTNumber(MaxBoilerTemp_Name, mqtt_maxboilertemp, MaxBoilerTemp, 0.5, true);
      break;
    case 59:
      PublishMQTTNumber(MinimumTempDifference_Name,0,20,0.5,true);
      break;
    case 60:
      UpdateMQTTNumber(MinimumTempDifference_Name, mqtt_minimumTempDifference, minimumTempDifference, 0.5, true);
      break;
    case 61:
      PublishMQTTNumber(FrostProtectionSetPoint_Name,0,90,0.5,true);
      break;
    case 62:
      UpdateMQTTNumber(FrostProtectionSetPoint_Name, mqtt_FrostProtectionSetPoint, FrostProtectionSetPoint, 0.5, true);
      break;
    case 63:
      PublishMQTTNumber(BoilerTempAtPlus20_Name,10,90,0.5,true);
      break;
    case 64:
      UpdateMQTTNumber(BoilerTempAtPlus20_Name, mqtt_BoilerTempAtPlus20, BoilerTempAtPlus20, 0.5, true);
      break;
    case 65:
      PublishMQTTNumber(BoilerTempAtMinus10_Name,10,90,0.5,true);
      break;
    case 66:
      UpdateMQTTNumber(BoilerTempAtMinus10_Name, mqtt_BoilerTempAtMinus10, BoilerTempAtMinus10, 0.5, true);
      break;
    case 67:
      PublishMQTTNumber(SwitchHeatingOffAt_Name,10,90,0.5,true);
      break;
    case 68:
      UpdateMQTTNumber(SwitchHeatingOffAt_Name, mqtt_SwitchHeatingOffAt, SwitchHeatingOffAt, 0.5, true);
      break;
    case 69:
      PublishMQTTNumber(ReferenceRoomCompensation_Name,0,30,0.5,true);
      break;
    case 70:
      UpdateMQTTNumber(ReferenceRoomCompensation_Name, mqtt_ReferenceRoomCompensation, ReferenceRoomCompensation, 0.5, true);
      break;
    case 71:
      PublishMQTTNumber(KP_Name,0,50,1,false);
      break;
    case 72:
      UpdateMQTTNumber(KP_Name, mqtt_kp, KP, 0.01, true);
      break;
    case 73:
      PublishMQTTNumber(KI_Name,0,1,0.01,false);
      break;
    case 74:
      UpdateMQTTNumber(KI_Name, mqtt_ki, KI, 0.01, true);
      break;
    case 75:
      PublishMQTTNumber(KD_Name,0,5,0.1,false);
      break;
    case 76:
      UpdateMQTTNumber(KD_Name, mqtt_kd, KD, 0.01, true);
      break;
    case 77:
      PublishMQTTCurvatureSelect(Curvature_Name);
      break;
    case 78:
      UpdateMQTTCurvatureSelect(Curvature_Name, Curvature, true);
      break;
    case 79:
      PublishMQTTText(MQTT_TempTopic_Name);
      break;
    case 80:
      UpdateMQTTText(MQTT_TempTopic_Name, mqtt_mqtttemptopic, mqtttemptopic, true);
      break;
    case 81:
      PublishMQTTText(MQTT_OutsideTempTopic_Name);
      break;
    case 82:
      UpdateMQTTText(MQTT_OutsideTempTopic_Name, mqtt_mqttoutsidetemptopic, mqttoutsidetemptopic, true);
      break;
    case 83:
      PublishMQTTTextSensor(Log_Name);
      break;
    case 84:
      // No update for Log_Name
      break;
    case 85:
      PublishMQTTTextSensor(IP_Address_Name);
      break;
    case 86:
      UpdateMQTTTextSensor(IP_Address_Name, currentIP.c_str(), true);
      break;
    case 87:
      PublishMQTTNumberSensor(WiFi_RSSI_Name, "dBm");
      break;
    case 88:
      UpdateMQTTNumberSensor(WiFi_RSSI_Name, mqtt_wifi_rssi, (float)WiFi.RSSI(), 2.0, true);
      break;
    case 89:
      PublishMQTTNumberSensor(Boiler_Setpoint_Temperature_Name, "°C", "temperature");
      break;
    case 90:
      UpdateMQTTNumberSensor(Boiler_Setpoint_Temperature_Name, mqtt_boiler_setpoint, boiler_SetPoint, 0.09, true);
      break;
    case 91:
      PublishMQTTButton(Reset_Device_Name);
      break;
    case 92:
      // No update for Reset_Device_Name
      break;
    case 93:
      PublishMQTTTextSensor(Uptime_Name);
      break;
    case 94:
      UpdateMQTTTextSensor(Uptime_Name, getUptimeString().c_str(), true);
      break;
    case 95:
      PublishMQTTNumberSensor(Free_Heap_Name, "B");
      break;
    case 96:
      UpdateMQTTNumberSensor(Free_Heap_Name, mqtt_free_heap, ESP.getFreeHeap(), 0.0, true);
      break;
    case 97:
      // Reset index and return true (done)
      sensorIndex = 0;
      Info("All MQTT sensors published and values sent");
      isPublishingAllSensors = false;
      firstPublishDone = true;
      return true;
  }

  // Increment sensor index for next call
  sensorIndex++;
  return false; // Not done yet
}


// The setup code
void setup()
{
  // start Serial port for logging purposes
  Serial.begin(115200);
  Serial.println("\nDomEspHelper, compile date "+String(compile_date));

  //read configuration from FS json
  if (LittleFS.begin()) {
    Serial.println("mounted file system");
    readConfig();
  } else {
    Serial.println("failed to mount FS");
  }
  //end read
  
  // Handle Wifi connection by wifi manager
  WiFiManager wifiManager;
  wifiManager.setHostname(host.c_str());
  wifiManager.setConnectTimeout(180);
  wifiManager.autoConnect("Thermostat");

  // start NTP client
  timeClient.begin();

  if (MDNS.begin(host.c_str())) {
    Serial.println("MDNS responder started");
  }
  // Log IP adress
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());  //Print the local IP to access the server

  // Register commands on webserver
  server.on("/ResetWifiCredentials", handleResetWifiCredentials);
  server.on("/GetSensors",handleGetSensors);
  server.on("/info", handleGetInfo);
  server.on("/getconfig", handleGetConfig);
  server.on("/saveconfig", handleSaveConfig);
  server.on("/removeconfig", handleRemoveConfig);
  server.on("/reset", handleReset);
  server.on("/command", handleCommand); 
  server.on("/upload", HTTP_GET, sendUploadForm);  
  server.on("/upload", HTTP_POST, [](){ server.send(200); }, handleFileUpload);
  server.on("/config.json", handleConfigJsonFile); // serve config.json from file, masking password
  server.on("/dir", handleDir);
  server.on("/delete", handleDelete);
  server.onNotFound(handleNotFound);

  // Initialize OTA
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Set hostname & PAssword for OTA
  ArduinoOTA.setHostname(host.c_str());
  ArduinoOTA.setPassword(host.c_str()); // Disable for data upload

  ArduinoOTA.onStart([]() {
    if (ArduinoOTA.getCommand() == U_FLASH) {
      Info("OTA: Start updating sketch...");
    } else { // U_FS
      Info("OTA: Start updating filesystem...");
    }
    OTAUpdateInProgress=true;
  });

  ArduinoOTA.onEnd([]() {
    Info("OTA Completed, restarting");
    OTAUpdateInProgress=false;
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Debug("OTA Progress: " + String(progress / (total / 100))+ "%");
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Error("OTA: Error["+String(error)+"]:");
    if (error == OTA_AUTH_ERROR) {
      Error("OTA: Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Error("OTA: Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Error("OTA: Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Error("OTA: Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Error("OTA: End Failed");
    }
    OTAUpdateInProgress=false;
  });
  
  ArduinoOTA.begin();
  
  //Start the server
  server.begin();   
  Serial.println("Opentherm Helper is waiting for commands");   

  // Enable watchdog timer (5 minutes timeout)
  ESP.wdtEnable(300000); // 5 minutes in milliseconds

  //Init DS18B20 sensor
  sensors.begin();

  // Init builtin led
  pinMode(LED_BUILTIN, OUTPUT);

  // Start OpenTherm
  ot.begin(handleInterrupt);

  // MQTT
  // MQTT.setServer(mqttserver.c_str(), mqttport); // server details verwijderd, gebeurt in connect
  // MQTT.setBufferSize(1024); // discovery messages are longer than default max buffersize(!) verwijderd, niet nodig

// Limit blocking times on sockets to reduce stalls during network issues
  espClient.setTimeout(500);   // WiFiClient read timeout in ms
  mqttSetConnectionTimeout(250);     // ArduinoMqttClient connection timeout in ms (halve seconde)
  mqttSetKeepAliveInterval(300);        // Keep-alive in seconden

  #if MQTT_LIBRARY == 0
    mqttSetCallback(MQTTcallback);
  #elif MQTT_LIBRARY == 1
    mqttOnMessage(onMessageCallback); // listen to callbacks
  #else
    // AsyncMQTTClient callbacks
    client.onConnect([](bool sessionPresent) {
      Info("AsyncMQTT connected");
      if (mqtttemptopic.toInt()>0 or mqttoutsidetemptopic.toInt()>0) { // apparently it is a domoticz idx. So listen to domoticz/out
        SubScribeToDomoticz();
      }
      sensorIndex = 0; // Reset sensor index for discovery
      PublishAllMQTTSensors();
      String willTopic = host + "/status";
      mqttPublish(willTopic.c_str(), "online", true, 1);
    });
    client.onDisconnect([](AsyncMqttClientDisconnectReason reason) {
      Error("AsyncMQTT disconnected");
    });
    client.onMessage([](char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
      // Call the common MQTT callback
      MQTTcallback(topic, (byte*)payload, (unsigned int)len);
    });
  #endif
}

// Loop Code
void loop()
{
  Debug("starting loop");

  // Check WiFi connection status and log changes
  if (WiFi.status() != WL_CONNECTED) {
    if (!wifiConnectionLostLogged) {
      Error("WiFi connection lost");
      wifiConnectionLostLogged = true;
    }
  } else {
    if (wifiConnectionLostLogged) {
      wifiConnectionLostLogged = false;
      Info("WiFi connection restored, RSSI: " + String(WiFi.RSSI()) + " dBm");
    }
  }

  Debug("WiFi status: " + String(WiFi.status() == WL_CONNECTED ? "connected" : "not connected") + ", RSSI: " + String(WiFi.RSSI()) + " dBm");

  // Update Timeclient
  if (WiFi.status() == WL_CONNECTED) {
    unsigned long startTime = millis();
    timeClient.update();
    unsigned long updateDuration = millis() - startTime;
    Debug("timeClient.update() took " + String(updateDuration) + " milliseconds");
  } else {
    Debug("WiFi not connected, skipping timeClient.update()");
  }
  
  // handle OTA
  if (WiFi.status() == WL_CONNECTED) {
    unsigned long startTime = millis();
    ArduinoOTA.handle();
    unsigned long handleDuration = millis() - startTime;
    Debug("ArduinoOTA.handle() took " + String(handleDuration) + " milliseconds");
  } else {
    Debug("WiFi not connected, skipping ArduinoOTA.handle()");
  }

  // update Uptime
  updateTime();

  // Check if we have to save the config
  if ((!ClimateConfigSaved) and (millis()>t_save_config)) {
    SaveConfig();
  }



  // don't do anything if we are doing if OTA upgrade is in progress
  if (!OTAUpdateInProgress) {
    if (millis()-t_heartbeat>heartbeatTickInMillis) {
      //reset tick time
      t_heartbeat=millis();

      // Feed the watchdog
      feedWatchdog();

      // (Re)connect MQTT
      if (WiFi.status() == WL_CONNECTED) {
        if (!mqttConnected()) {
          if (!mqttConnectionLostLogged) {
            Error("MQTT connection lost");
            mqttConnectionLostLogged = true;
          }
          // We are no connected, so try to reconnect
          if (millis()-t_last_mqtt_try_connect>MQTTConnectTimeoutInMillis) {
            reconnect();
            t_last_mqtt_try_connect=millis();
          }
        } else {
          if (mqttConnectionLostLogged) {
            mqttConnectionLostLogged = false;
          }
        }

        if (WiFi.status() == WL_CONNECTED && mqttConnected()) {
          String newIP = WiFi.localIP().toString();
          if (!currentIP.equals(newIP)) {
            currentIP=newIP;
            UpdateMQTTTextSensor(IP_Address_Name,currentIP.c_str());
            Info("IP address changed to "+currentIP);
          }
        }
        
        digitalWrite(LED_BUILTIN, LOW);    // switch the LED on , to indicate we have connection

        // handle MDNS
        unsigned long startTime = millis();
        MDNS.update();
        unsigned long mdnsDuration = millis() - startTime;
        Debug("MDNS.update() took " + String(mdnsDuration) + " milliseconds");
      } else {
        // WiFi niet verbonden, sla netwerkacties over
        Debug("WiFi not connected, skipping heartbeat network operations");
      }
    }

    if (WiFi.status() == WL_CONNECTED && mqttConnected()) {
      // we are connected, check if we have to resend discovery info
      if (millis()-t_last_mqtt_discovery>MQTTDiscoveryHeartbeatInMillis || sensorIndex > 0)
      {
        if (PublishAllMQTTSensors()) {
          t_last_mqtt_discovery = millis();
        }
      }
    }

    // Remember last hour of temperatures (PID calculation needs to be able to determine if temperature is rising or dropping)
    {
      if (insideTemperatureReceived) {
        int CurrentMinute=(millis() % 60000) / 1000;
        insideTempAt[CurrentMinute] = mqttTemperature;   
      }
    }

    // Handle CLimate program
    handleClimateProgram(); 
    
    // Check if we have to communicate steering vars
    CommunicateSteeringVarsToMQTT();

    // Publish free heap every minute
    if (millis() - last_heap_publish > 60000) {
      UpdateMQTTNumberSensor(Free_Heap_Name, mqtt_free_heap, ESP.getFreeHeap(), 0.0, false);
      last_heap_publish = millis();
    }

    // handle openthem commands
    handleOpenTherm();

    //Handle incoming webrequests
    server.handleClient();

    // handle MQTT
    if (WiFi.status() == WL_CONNECTED) {
      unsigned long startTime = millis();
      mqttPoll();
      unsigned long pollDuration = millis() - startTime;
      Debug("client.poll() took " + String(pollDuration) + " milliseconds");
    } else {
      Debug("WiFi not connected, skipping client.poll()");
    }

  }
}
