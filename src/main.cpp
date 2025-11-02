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
#include <PubSubClient.h>         // MQTT library
#include "domesphelper.h"               // Set Configuration and default constants
#include <LittleFS.h>             // Filesystem
#include <NTPClient.h>            // for NTP Client
#include <WiFiUdp.h>              // is needed by NTP client

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
bool mqtt_enable_CentralHeating=true;
bool mqtt_enable_Cooling=true;
float mqtt_boiler_setpoint=1;
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
float mqtt_ReferenceRoomCompensation = 99;          // In weather dependent mode: Correct with this number per degree celcius difference (air temperature - setpoint) 
float mqtt_kp=99;
float mqtt_ki=99;
float mqtt_kd=99;
float mqtt_p=99;
float mqtt_i=99;
float mqtt_d=99;
String mqtt_mqtttemptopic="xyzxyz";
String mqtt_mqttoutsidetemptopic="xyzxyz";
bool mqtt_debug=false;

// vars for program logic
int OpenThermCommandIndex = 0; // index of the OpenTherm command we are processing
const char compile_date[] = __DATE__ " " __TIME__;  // Make sure we can output compile date
unsigned long t_heartbeat=millis()-heartbeatTickInMillis; // last heartbeat timestamp, init on previous heartbeat, so processing start right away
unsigned long t_last_mqtt_command=millis()-MQTTTimeoutInMillis; // last MQTT command timestamp. init on previous timeout value, so processing start right away
unsigned long t_last_http_command=millis()-HTTPTimeoutInMillis; // last HTTP command timestamp. init on previous timeout value, so processing start right away
unsigned long t_last_mqtt_discovery=millis()-MQTTDiscoveryHeartbeatInMillis; // last mqqt discovery timestamp
unsigned long t_last_climateheartbeat=0; // last climate heartbeat timestamp
unsigned long t_last_tempreceived=0; // Time when last MQTT temp was received
unsigned long t_last_outsidetempreceived=0; // Time when last MQTT temp was received
unsigned long t_save_config; // timestamp for delayed save
unsigned long t_last_mqtt_try_connect = millis()-MQTTConnectTimeoutInMillis; // the last time we tried to connect to mqtt server
bool ClimateConfigSaved=true;
bool OTAUpdateInProgress=false;
bool insideTemperatureReceived=false;
bool outsideTemperatureReceived=false;
#define NUMBEROFMEASUREMENTS 10 // number of measurements over which to average for outside temp
float insideTempAt[60];
float outsidetemp[NUMBEROFMEASUREMENTS];
int outsidetempcursor;
bool controlledByHTTP = false;
bool debug=false;

// object for uploading files
File fsUploadFile;

// objects to be used by program
ESP8266WebServer server(httpport);    //Web server object. Will be listening in port 80 (default for HTTP)
OneWire oneWire(14);                  // for OneWire Bus. Data wire is connected to 14 pin on the OpenTherm Shield (Temperature sensor)
DallasTemperature sensors(&oneWire);  // for the temp sensor on one wire bus
OpenTherm ot(4,5);                    // pin number for opentherm adapter connection, 2/3 for Arduino, 4/5 for ESP8266 (D2), 21/22 for ESP32
WiFiClient espClient;                 // Needed for MQTT
PubSubClient MQTT(espClient);         // MQTT client
WiFiUDP ntpUDP;                       // for NTP client
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 3600, 60000); //  Set the timeserver (incl offset and timeout)

void Debug(String text) {
  if (debug) {
    if (MQTT.connected()) {
      UpdateMQTTText(Debug_Name,(timeClient.getFormattedTime()+" "+text).c_str());
    }
    Serial.println(text);
  }
}

void Error(String text) {
  if (MQTT.connected()) {
    UpdateMQTTText(Error_Name,(timeClient.getFormattedTime()+" "+text).c_str());
  }
  Serial.println(text);
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
  json["uptime"] =  String(y)+" years, "+String(d)+" days, "+String(h)+" hrs, "+String(m)+" mins & "+String(s)+" secs";

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
  if (not (climate_Mode.equals("off") or Holiday_Mode==true)) {
    json["ControlledBy"] = "Climate Mode";
  } else if (millis()-t_last_http_command<HTTPTimeoutInMillis) {
    json["ControlledBy"] = "HTTP";
  } else if (millis()-t_last_mqtt_command<MQTTTimeoutInMillis) {
    json["ControlledBy"] = "MQTT";
  } else {
    json["ControlledBy"] = "None";
  }
  
  // Add MQTT Connection status 
  json["MQTTconnected"] = MQTT.connected() ? "true" : "false";
  json["MQTTstate"] = MQTT.state();

  // Add BoilerManagementVars
  json["EnableCentralHeating"] = enableCentralHeating;
  json["EnableHotWater"] = enableHotWater;
  json["EnableCooling"] = enableCooling;
  json["BoilerSetpoint"] = float(int(boiler_SetPoint*100))/100;
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
  Debug("Resetting Wifi Credentials");
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
  Serial.println("Getting the sensors");
  SendHTTP("GetSensors","OK");
}

// Generieke handler voor het aan- of uitzetten van een functie via HTTP argument
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
  String Statustext="Unknown Command";

  // we received a command, so someone is comunicating
  t_last_http_command=millis();
  controlledByHTTP=true;

  // blink the LED, so we can see a command was sent
  digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off , to indicate we are executing a command

  // for Debugging purposes
  Debug("Handling Command: Number of args received: "+server.args());
  for (int i = 0; i < server.args(); i++) {
    Debug ("Argument "+String(i)+" -> "+server.argName(i)+": "+server.arg(i));
  } 

  // Set DHWTemp
  if (server.arg("DHWTemperature")!="") {
    Serial.println("Setting dhw temp to "+server.arg("DHWTemperature"));
    dhw_SetPoint=server.arg("DHWTemperature").toFloat();
    Statustext="OK";
  }

  // Set Boiler Temp
  if (server.arg("BoilerTemperature")!="") {
    Serial.println("Setting boiler temp to "+server.arg("BoilerTemperature"));
    boiler_SetPoint=server.arg("BoilerTemperature").toFloat();
    Statustext="OK";
  }

  // handle http toggle commands
  handleHTTPToggle("CentralHeating", enableCentralHeating);
  handleHTTPToggle("HotWater", enableHotWater);
  handleHTTPToggle("Cooling", enableCooling);
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
  Serial.println("GetInfo");
  JsonDocument json;
  char buf[512];
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
  if (MQTT.connected())
  {
    json["MQTTconnected"] = true;
  } else {
    json["MQTTconnected"] = false;
  }
  json["mqttstate"] = MQTT.state();
  json["currentTime"] = timeClient.getFormattedTime();
  json["uptime"] = String(y)+" years, "+String(d)+" days, "+String(h)+" hrs, "+String(m)+" ms, "+String(s)+" secs, "+String(ms)+" msec";
  json["compile_date"] = String(compile_date);
  
  serializeJson(json, buf); 
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
  Serial.println("GetConfig");
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
  json["uptime"] = String(y)+" years, "+String(d)+" days, "+String(h)+" hrs, "+String(m)+" ms, "+String(s)+" secs, "+String(ms)+" msec";

  // Send output with pretty formatting
  char buf[2048];
  serializeJsonPretty(json, buf);
  server.send(200, "application/json", buf);
}

void handleConfigJsonFile() {
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
  Serial.println("handleSaveConfig");
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
  Serial.println("handleRemoveConfig");
  
  if (LittleFS.exists(CONFIGFILE)) {
    Serial.println("Config file existst, removing configfile");
    LittleFS.remove(CONFIGFILE);
    server.send(200, "text/plain", "Config file removed");
    Debug("Configfile removed");
    delay(500); // wait for server send to finish
    ESP.restart(); // restart
  } else {
    server.send(200, "text/plain", "No confile file present to remove");
  } 

  return;
}

void handleReset() {
  Debug("handleReset");
  
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


void handleNotFound()
{
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

void CommunicateSetpoint(const char* setpointName,float setpointValue,float *mqttValue) {
  // Debug("CommunicateSetpoint("+String(setpointName)+","+String(setpointValue)+","+String(*mqttValue)+"), diff is "+String(setpointValue-*mqttValue));
  if (not (setpointValue-*mqttValue>-0.1 and setpointValue-*mqttValue<0.1)) { 
    UpdateMQTTSetpoint(setpointName,setpointValue);
    *mqttValue=setpointValue;
  }
}

void CommunicateNumber(const char* numberName,float Value,float *mqttValue, float tolerance) {
  // Debug("CommunicateNumber("+String(numberName)+","+String(Value)+","+String(*mqttValue)+","+String(tolerance)+"), where diff is "+String(Value-*mqttValue));
  if (not (Value-*mqttValue>-tolerance and Value-*mqttValue<tolerance)){ // value changed
    UpdateMQTTNumber(numberName,Value);
    *mqttValue=Value;
  }
}

void CommunicateNumberSensor(const char* numberName,float Value,float *mqttValue, float tolerance) {
  // Debug("CommunicateNumber("+String(numberName)+","+String(Value)+","+String(*mqttValue)+","+String(tolerance)+"), where diff is "+String(Value-*mqttValue));
  if (not (Value-*mqttValue>-tolerance and Value-*mqttValue<tolerance)){ // value changed
    UpdateMQTTNumberSensor(numberName,Value);
    *mqttValue=Value;
  }
}


void CommunicateText(const char* TextName,String Value,String *mqttValue) {
  if (not Value.equals(*mqttValue)){ // value changed
    UpdateMQTTText(TextName,Value.c_str());
    *mqttValue=Value;
  }
}


void CommunicateSteeringVarsToMQTT() {
if (MQTT.connected()) {
    // Climate Mode
    if (!climate_Mode.equals(mqtt_climate_Mode)){ // value changed
      if (climate_Mode.equals("off")) {
        UpdateMQTTSetpointMode(Climate_Name,OFF);
      } else if (climate_Mode.equals("heat")) {
        UpdateMQTTSetpointMode(Climate_Name,HEAT);
      } else if (climate_Mode.equals("cool")) {
        UpdateMQTTSetpointMode(Climate_Name,COOL);
      } else if (climate_Mode.equals("auto")) {
        UpdateMQTTSetpointMode(Climate_Name,AUTO);
      }
      mqtt_climate_Mode=climate_Mode;
    }

    // Communicate the setpoints
    CommunicateSetpoint(Climate_Name,climate_SetPoint,&mqtt_climate_setpoint);
    CommunicateSetpoint(Boiler_Setpoint_Name,boiler_SetPoint,&mqtt_boiler_setpoint);
    CommunicateSetpoint(DHW_Setpoint_Name,dhw_SetPoint,&mqtt_dhw_setpoint);

    // Communicatie the steering vars
    CommunicateNumber(MinBoilerTemp_Name,MinBoilerTemp,&mqtt_minboilertemp,0.5);
    CommunicateNumber(MaxBoilerTemp_Name,MaxBoilerTemp,&mqtt_maxboilertemp,0.5);
    CommunicateNumber(MinimumTempDifference_Name,minimumTempDifference,&mqtt_minimumTempDifference,0.5);
    CommunicateNumber(FrostProtectionSetPoint_Name,FrostProtectionSetPoint,&mqtt_FrostProtectionSetPoint,0.5);
    CommunicateNumber(BoilerTempAtPlus20_Name,BoilerTempAtPlus20,&mqtt_BoilerTempAtPlus20,0.5);
    CommunicateNumber(BoilerTempAtMinus10_Name,BoilerTempAtMinus10,&mqtt_BoilerTempAtMinus10,0.5);
    CommunicateNumber(SwitchHeatingOffAt_Name,SwitchHeatingOffAt,&mqtt_SwitchHeatingOffAt,0.5);
    CommunicateNumber(ReferenceRoomCompensation_Name,ReferenceRoomCompensation,&mqtt_ReferenceRoomCompensation,0.5);
    CommunicateNumber(KP_Name,KP,&mqtt_kp,0.01);
    CommunicateNumber(KI_Name,KI,&mqtt_ki,0.01);
    CommunicateNumber(KD_Name,KD,&mqtt_kd,0.01);
    CommunicateNumberSensor(P_Name,P,&mqtt_p,0.01);
    CommunicateNumberSensor(I_Name,I,&mqtt_i,0.01);
    CommunicateNumberSensor(D_Name,D,&mqtt_d,0.01);
    CommunicateText(MQTT_TempTopic_Name,mqtttemptopic,&mqtt_mqtttemptopic);
    CommunicateText(MQTT_OutsideTempTopic_Name,mqttoutsidetemptopic,&mqtt_mqttoutsidetemptopic);
    if (mqtt_Curvature!=Curvature) {
      UpdateMQTTCurvatureSelect(Curvature_Name,Curvature);
      mqtt_Curvature=Curvature;
    }

    // The actual temperature for the climate device. Now the temp required from mqtt. But should be made switchable to other sources 
    if (mqttTemperature!=mqtt_mqttTemperature and insideTemperatureReceived) {
      UpdateMQTTSetpointTemperature(Climate_Name,mqttTemperature);
      mqtt_mqttTemperature=mqttTemperature;
    }

    // Enable CEntral Heating
    if (enableCentralHeating!=mqtt_enable_CentralHeating) // value changed
    {
      UpdateMQTTSwitch(EnableCentralHeating_Name,enableCentralHeating);
      mqtt_enable_CentralHeating=enableCentralHeating;
      // Communicate to setpoint as well
      UpdateMQTTBoilerSetpointMode();
    }

    // EnableCooling
    if (enableCooling!=mqtt_enable_Cooling) // value changed
    {
      UpdateMQTTSwitch(EnableCooling_Name,enableCooling);
      mqtt_enable_Cooling=enableCooling;
      // Communicate to setpoint as well
      UpdateMQTTBoilerSetpointMode();
    }

    //Enable HOt Water
    if (enableHotWater!=mqtt_enable_HotWater) // value changed
    {
      UpdateMQTTSwitch(EnableHotWater_Name,enableHotWater);
      mqtt_enable_HotWater=enableHotWater;
      // Communicate to setpoint as well
      if (enableHotWater) {
        UpdateMQTTSetpointMode(DHW_Setpoint_Name,1);
      } else  {
        UpdateMQTTSetpointMode(DHW_Setpoint_Name,0);
      }
    }

    // Weather Dependent Mode
    if (Weather_Dependent_Mode!=mqtt_Weather_Dependent_Mode) // value changed
    {
      UpdateMQTTSwitch(Weather_Dependent_Mode_Name,Weather_Dependent_Mode);
      mqtt_Weather_Dependent_Mode=Weather_Dependent_Mode;
    }

    // Holiday Mode
    if (Holiday_Mode!=mqtt_Holiday_Mode) // value changed
    {
      UpdateMQTTSwitch(Holiday_Mode_Name,Holiday_Mode);
      mqtt_Holiday_Mode=Holiday_Mode;
    }

    // Debug
    if (debug!=mqtt_debug) // value changed
    {
      UpdateMQTTSwitch(Debug_Name,debug);
      mqtt_debug=debug;
    }

    // Frost Protection Active
    if (FrostProtectionActive!=mqtt_FrostProtectionActive) // value changed
    {
      UpdateMQTTBinarySensor(FrostProtectionActive_Name,FrostProtectionActive);
      mqtt_FrostProtectionActive=FrostProtectionActive;
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

    // calculate he amount of rising/dropping temp
    CurrentMinute=(millis() % 60000 / 1000);
    DeltaKPH = (mqttTemperature-insideTempAt[(CurrentMinute+45)%60])*4;  // tempchange the last 15 mins mutltiplied by 4 is the number of kelvin per hour the temp is currently dropping or rising
    
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
  Debug("PID Mode, PID=("+String(P)+","+String(I)+","+String(D)+"), total: "+String(P+I+D));
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
    if (millis()-t_last_mqtt_command>MQTTTimeoutInMillis and millis()-t_last_http_command>HTTPTimeoutInMillis) { // allow HTTP or MQTT commands when not in climate mode
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
      }
    } else {
      // no need to actie, set the correct state
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
      }

      if (Weather_Dependent_Mode and !HotWater) {
        Debug("Weather dependent mode, setting setpoint to "+String(GetBoilerSetpointFromOutsideTemperature(roomTemperature,outside_Temperature)));
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

    // Enable heating and/or cooling
    if (climate_Mode.equals("heat")) {
      if (boiler_SetPoint>roomTemperature+minimumTempDifference) {
        enableCentralHeating=true;
      } else {
        enableCentralHeating=false;
      }
      enableCooling=false;
    } else if (climate_Mode.equals("cool")) {
      enableCentralHeating=false;
      if (boiler_SetPoint<roomTemperature-minimumTempDifference) {
        enableCooling=true;
      } else {
        enableCooling=false;
      }
    } else if (climate_Mode.equals("auto")) {
      if (boiler_SetPoint>roomTemperature+minimumTempDifference) {
        enableCentralHeating=true;
        enableCooling=false;
      } else if (boiler_SetPoint<roomTemperature-minimumTempDifference) {
        enableCentralHeating=false;
        enableCooling=true;
      } else {
        enableCentralHeating=false;
        enableCooling=false;
      }
    }
  }
}

void handleSetBoilerStatus() {
  // enable/disable heating, hotwater, heating and get status from opentherm connection and boiler (if it can be reached)
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
    if (MQTT.connected()) {
      if (Flame!=mqtt_Flame){ // value changed
        UpdateMQTTBinarySensor(FlameActive_Name,Flame);
        mqtt_Flame=Flame;
      }
      if (Fault!=mqtt_Fault){ // value changed
        UpdateMQTTBinarySensor(FaultActive_Name,Fault);
        mqtt_Fault=Fault;
      }
      if (Diagnostic!=mqtt_Diagnostic){ // value changed
        UpdateMQTTBinarySensor(DiagnosticActive_Name,Diagnostic);
        mqtt_Diagnostic=Diagnostic;
      }
      if (Cooling!=mqtt_Cooling){ // value changed
        UpdateMQTTBinarySensor(CoolingActive_Name,Cooling);
        mqtt_Cooling=Cooling;
      }
      if (CentralHeating!=mqtt_CentralHeating){ // value changed
        UpdateMQTTBinarySensor(CentralHeatingActive_Name,CentralHeating);
        mqtt_CentralHeating=CentralHeating;
      }
      if (HotWater!=mqtt_HotWater){ // value changed
        UpdateMQTTBinarySensor(HotWaterActive_Name,HotWater);
        mqtt_HotWater=HotWater;
      }
      if (modulation!=mqtt_modulation){ // value changed
        UpdateMQTTPercentageSensor(Modulation_Name,modulation);
        mqtt_modulation=modulation; // remember mqtt modulation value
      }
    }
    // Execute the next command in the next call
  } else if (responseStatus == OpenThermResponseStatus::NONE) {
      Serial.println("Opentherm Error: OpenTherm is not initialized");
  } else if (responseStatus == OpenThermResponseStatus::INVALID) {
      Serial.println("Opentherm Error: Invalid response " + String(response, HEX));
  } else if (responseStatus == OpenThermResponseStatus::TIMEOUT) {
      Serial.println("Opentherm Error: Response timeout");
  }  else {
      Serial.println("Opentherm Error: unknown error");
  }
}

void handleGetOutSideTemp(){
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
  if (MQTT.connected()) {
    float delta = mqtt_outside_Temperature-outside_Temperature;
    if ((delta != 0) and outside_Temperature!=99){ // value changed
      UpdateMQTTTemperatureSensor(Outside_Temperature_Name,outside_Temperature);
      mqtt_outside_Temperature=outside_Temperature;
    }

    delta = mqtt_OT_outside_Temperature-OT_outside_Temperature;
    if ((delta != 0) and OT_outside_Temperature!=99){ // value changed
      UpdateMQTTTemperatureSensor(OT_Outside_Temperature_Name,OT_outside_Temperature);
      mqtt_OT_outside_Temperature=OT_outside_Temperature;
    }
  }
}

void handleSetBoilerTemperature()
{
  // Set the boiler temperature
  ot.setBoilerTemperature(boiler_SetPoint);
}

void handleSetDHWSetpoint()
{
  // Set the DHW setpoint
  ot.setDHWSetpoint(dhw_SetPoint);
}

void handleGetBoilerTemperature()
{
  // Get the boiler temperature
  boiler_Temperature = ot.getBoilerTemperature();

  // Check if we have to send to MQTT
  if (MQTT.connected()) {
    float delta = mqtt_boiler_Temperature-boiler_Temperature;
    if (delta<-0.09 or delta>0.09){ // value changed
      UpdateMQTTTemperatureSensor(Boiler_Temperature_Name,boiler_Temperature);
      UpdateMQTTSetpointTemperature(Boiler_Setpoint_Name,boiler_Temperature);
      mqtt_boiler_Temperature=boiler_Temperature;
    }
  }
}

void handleGetDHWTemperature()
{
  // Get the DHW temperature
  dhw_Temperature = ot.getDHWTemperature();

  // Check if we have to send to MQTT
  if (MQTT.connected()) {
    float delta = mqtt_dhw_Temperature-dhw_Temperature;
    if (delta<-0.09 or delta>0.09){ // value changed
      UpdateMQTTTemperatureSensor(DHW_Temperature_Name,dhw_Temperature);
      UpdateMQTTSetpointTemperature(DHW_Setpoint_Name,dhw_Temperature);
      mqtt_dhw_Temperature=dhw_Temperature;
    }
  }
}

void handleGetReturnTemperature()
{
  // Get the return temperature
  return_Temperature = ot.getReturnTemperature();

  // Check if we have to send to MQTT
  if (MQTT.connected()) {
    float delta = mqtt_return_Temperature-return_Temperature;
    if (delta<-0.09 or delta>0.09){ // value changed
      UpdateMQTTTemperatureSensor(Return_Temperature_Name,return_Temperature);
      mqtt_return_Temperature=return_Temperature;
    }
  }
}

void handleGetPressure()
{
  // Get the pressure from OpenTherm
  pressure = ot.getPressure();

  // Check if we have to send to MQTT
  if (MQTT.connected()) {
    float delta = mqtt_pressure-pressure;
    if (delta<-0.009 or delta>0.009){ // value changed
      UpdateMQTTPressureSensor(Pressure_Name,pressure);
      mqtt_pressure=pressure;
    }
  }
}

void handleGetFlowRate()
{
  // Get the DHW flow rate from OpenTherm
  flowrate = getDHWFlowrate();

  /** Check if we have to send to MQTT
  if (MQTT.connected()) {
    float delta = mqtt_flowrate-flowrate;
    if (delta<-0.01 or delta>0.01){ // value changed
      UpdateMQTTFlowRateSensor(FlowRate_Name,flowrate);
      mqtt_flowrate=flowrate;
    }
  } */
}

void handleGetFaultCode()
{
  // Get the fault code from OpenTherm
  FaultCode = ot.getFault();

  // Check if we have to send to MQTT
  if (MQTT.connected()) {
    if (FaultCode!=mqtt_FaultCode){ // value changed
      UpdateMQTTFaultCodeSensor(FaultCode_Name,FaultCode);
      mqtt_FaultCode=FaultCode;
    }
  }
}

void handleGetThermostatTemperature()
{
  // Get the thermostat temperature from the DS18B20 sensor
  sensors.requestTemperatures(); // Send the command to get temperature readings 
  currentTemperature = sensors.getTempCByIndex(0);

  // Check if we have to send to MQTT
  if (MQTT.connected()) {
    float delta = mqtt_currentTemperature-currentTemperature;
    if (delta<-0.09 or delta>0.09){ // value changed
      UpdateMQTTTemperatureSensor(Thermostat_Temperature_Name,currentTemperature);
      mqtt_currentTemperature=currentTemperature;
      if (mqtttemptopic.length()==0) { // temptopic empty, use internal temperature
        SetMQTTTemperature(currentTemperature);
      }
    }
  }
}

void handleOpenTherm() {

  // Array of  handler functions to choose from, lokaal in de functie
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

  // Dispatcher: voer de juiste handler uit en verhoog de index
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
    Debug("Climate mode unchanged, ignoring command");
  } else {
    // Init PID calculater
    InitPID();

    if (String(mode).equals("off") or String(mode).equals("heat") or String(mode).equals("cool") or String(mode).equals("auto")) {
      Debug("Setting clime mode to "+String(mode));
      climate_Mode=mode;
      DelayedSaveConfig();
    } else {
      Error("Unknown payload for Climate mode command");
      // sendback current mode to requester, so trick program into making it think it has to communicatie
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
    Debug("First temperature ("+String(value)+") received, ready for climate mode");
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
    Debug("First outside temperature ("+String(value)+") received, ready for weather dependent mode");
    outsideTemperatureReceived=true; // Make sure we do this only once ;-)

    // this is our first measurement, initialize the array
    for (int i=0;i<NUMBEROFMEASUREMENTS;i++) {
      outsidetemp[i]=outside_Temperature;
    }

  }
}

bool HandleBoilerMode(const char* mode) 
{
  bool succeeded=true;
  if (String(mode).equals("off")) {
    enableCentralHeating=false;
    enableCooling=false;
  } else if (String(mode).equals("heat")) {
    enableCentralHeating=true;
    enableCooling=false;
  } else if (String(mode).equals("cool")) {
    enableCentralHeating=false;
    enableCooling=true;
  } else if (String(mode).equals("auto")) {
    enableCentralHeating=true;
    enableCooling=true;
  } else {
    Error("Unknown payload for central boiler setpoint mode command");
    // sendback current mode to requester, so trick program into making it think it has to communicatie
    if (enableCentralHeating) {
      mqtt_enable_CentralHeating=false;
    } else {
      mqtt_enable_CentralHeating=true;
    }
    if (enableCooling) {
      mqtt_enable_Cooling=false;
    } else {
      mqtt_enable_Cooling=true;
    }
    succeeded=false;
  }
  return succeeded;
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
    // sendback current mode to requester, so trick program into making it think it has to communicatie
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
    Debug("unknown state for enabletoggle");
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
  // Definieer de relevante sleutels
  const char* keys[] = { "value", "state", "temperature", "svalue", "svalue1" };
  const int keyCount = sizeof(keys) / sizeof(keys[0]);

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    // Geen JSON, geef originele payload terug
    return String(payload);
  }

  // Probeer relevante keys te vinden
  for (int i = 0; i < keyCount; i++) {
    if (doc[keys[i]].is<float>()) return String(doc[keys[i]].as<float>());
    if (doc[keys[i]].is<const char*>()) return String(doc[keys[i]].as<const char*>());
    if (doc[keys[i]].is<int>()) return String(doc[keys[i]].as<int>());
  }

  // Geen relevante key gevonden, probeer root value
  if (doc.is<float>()) return String(doc.as<float>());
  if (doc.is<const char*>()) return String(doc.as<const char*>());
  if (doc.is<int>()) return String(doc.as<int>());

  // fallback: geef hele JSON als string
  String result;
  serializeJson(doc, result);
  return result;
}

int getIdxFromPayload(const char* payload) {
  // Parse the JSON payload to extract the idx value
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    Debug("Failed to parse JSON payload: " + String(error.c_str()));
    return -1; // Return -1 if parsing fails
  }
  
  // Check if idx exists and is an integer
  if (doc["idx"].is<int>()) {
    return doc["idx"].as<int>();
  }
  
  Debug("No valid idx found in payload");
  return -1; // Return -1 if idx is not found or not an integer
}

// Handler voor domoticz device readings
void handleDomoticzOutputTopic(const String& value, const char* payloadstr) {
  int idx = getIdxFromPayload(payloadstr);
  Debug("Received domoticz device reading: "+String(idx)+","+String(payloadstr));
  if (mqtttemptopic.equals(String(idx))) {
    SetMQTTTemperature(value.toFloat());
  }
  if (mqttoutsidetemptopic.equals(String(idx))) {
    SetMQTTOutsideTemperature(value.toFloat());
  }
}

// Genereieke handler om een MQTT topic te wijzigen en opnieuw te subscriben
bool handleMQTTTopicChange(String& topicVar, const String& newValue) {
  if (topicVar.length() > 0) {
    MQTT.unsubscribe(topicVar.c_str()); // Unsubscribe van oude topic
  }
  topicVar = newValue;
  MQTT.subscribe(topicVar.c_str());    // Subscribe op nieuwe topic
  return true; // Config moet worden opgeslagen
}

void MQTTcallback(char* topic, byte* payload, unsigned int length) {
  // get vars from callback
  String topicstr=String(topic);
  char payloadstr[1024];
  strncpy(payloadstr,(char *)payload,length);
  payloadstr[length]='\0';

  String value = extractRelevantStringFromPayload(payloadstr);

  if (!topicstr.equals(domoticzoutputtopic)) { // prevent flooding debug log with updates from domoticzdevices
    Debug("Received message on topic ["+topicstr+"], payload: ["+payloadstr+"]");
  }

  // Assume succesful command 
  bool CommandSucceeded=false;
  
  if (millis()-t_last_http_command>HTTPTimeoutInMillis) { // only execute mqtt commands if not commanded by http

    // Domoticz devices
    if (topicstr.equals(domoticzoutputtopic)) {
      handleDomoticzOutputTopic(value, payloadstr);

    // climate mode
    } else if (topicstr.equals(host+"/climate/"+String(Climate_Name)+"/mode/set")) {
      CommandSucceeded=HandleClimateMode(value.c_str());

    // Climate setpoint temperature command
    } else if (topicstr.equals(SetpointCommandTopic(Climate_Name))) {
      CommandSucceeded = handleClimateSetpoint(value.toFloat());

    // Boiler Setpoint mode receive
    } else if (topicstr.equals(host+"/climate/"+String(Boiler_Setpoint_Name)+"/mode/set")) {
      CommandSucceeded=HandleBoilerMode(value.c_str());

    // DHW Setpoint mode receive
    } else if (topicstr.equals(host+"/climate/"+String(DHW_Setpoint_Name)+"/mode/set")) {
      CommandSucceeded=HandleDHWMode(value.c_str());

    // Handle Enable Hotwater switch command
    } else if (topicstr.equals(CommandTopic(EnableHotWater_Name))) {
      CommandSucceeded=HandleSwitch(&enableHotWater, &mqtt_enable_HotWater, value.c_str());

    // Handle EnableCooling switch command
    } else if (topicstr.equals(CommandTopic(EnableCooling_Name))) {
      CommandSucceeded=HandleSwitch(&enableCooling, &mqtt_enable_Cooling, value.c_str());

    // Enable Central Heating switch command
    } else if (topicstr.equals(CommandTopic(EnableCentralHeating_Name))) {
      CommandSucceeded=HandleSwitch(&enableCentralHeating, &mqtt_enable_CentralHeating, value.c_str());

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
      Debug("Received debug command: "+value);
      CommandSucceeded=HandleSwitch(&debug, &mqtt_debug, value.c_str());
      Debug("Debugging switched "+String(value));

    // boiler setpoint command
    } else if (topicstr.equals(SetpointCommandTopic(Boiler_Setpoint_Name))) {
      boiler_SetPoint=String(payloadstr).toFloat();
      CommandSucceeded=true; // we handled the command

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
      Debug("Setting outsideTempTopic to "+String(payloadstr));
      CommandSucceeded = handleMQTTTopicChange(mqttoutsidetemptopic, value);

    // Unknown topic (this code should never be reached, indicates programming logic error)  
    } else {
      LogMQTT(topicstr.c_str(),payloadstr,String(length).c_str(),"unknown topic");
    }

    if (CommandSucceeded) {
      // we received a succesful mqtt command, so someone is communicating correctly
      DelayedSaveConfig(); // save the config
      t_last_mqtt_command=millis();
    }
  }
}

void SubScribeToDomoticz() {
  MQTT.subscribe(domoticzoutputtopic.c_str());
}

void reconnect()
{
  if (usemqtt) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.print("WiFi not connected, can't connect to MQTT broker");
      return;
    }
    if (!MQTT.connected()) {
      Serial.print("Attempting MQTT connection...");
      bool mqttconnected;
      if (usemqttauthentication) {
        mqttconnected = MQTT.connect(host.c_str(), mqttuser.c_str(), mqttpass.c_str());
      } else {
        mqttconnected = MQTT.connect(host.c_str());
      }
      if (mqttconnected) {
        PublishAllMQTTSensors();
        Debug("Succesfully (re)connected, starting operations");
        Error("None, all OK");
        if (mqtttemptopic.toInt()>0 or mqttoutsidetemptopic.toInt()>0) { // apparently it is a domoticz idx. So listen to domoticz/out
          SubScribeToDomoticz();
        }      
        if (timeClient.forceUpdate()) {
          Debug("Time was set");
        } else {
          Debug("Time was not set");
        }
      } else {
        Serial.print("failed, rc=");
        Serial.print(MQTT.state());
      }
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

void PublishMQTTDimmer(const char* uniquename)
{
  Serial.println("PublishMQTTDimmer");
  JsonDocument json;

  // Construct JSON config message
  json["name"] = uniquename;
  json["unique_id"] = host+"_"+uniquename;
  json["cmd_t"] = host+"/light/"+String(uniquename)+"/set";
  json["stat_t"] = host+"/light/"+String(uniquename)+"/state";
  json["schema"] = "json";
  json["brightness"] = true;

  addDeviceToJson(&json);

  char conf[512];
  serializeJson(json, conf);  // conf now contains the json

  // Publish config message
  MQTT.publish((String(mqttautodiscoverytopic)+"/light/"+host+"/"+String(uniquename)+"/config").c_str(),conf,mqttpersistence);

}

void UpdateMQTTDimmer(const char* uniquename, bool Value, float Mod)
{
  Serial.println("UpdateMQTTDimmer");
  JsonDocument json;

  // Construct JSON config message
  json["state"]=Value ? "ON" : "OFF";
  if (Value and Mod==0) { // Workaround for homekit not being able to show dimmer with value on and brightness 0
    json["brightness"]=3;     
  } else {
    json["brightness"]=int(Mod*255/100);
  }
  char state[128];
  serializeJson(json, state);  // state now contains the json

  // publish state message
  MQTT.publish((host+"/light/"+String(uniquename)+"/state").c_str(),state,mqttpersistence);
}

void PublishMQTTSwitch(const char* uniquename)
{
  Serial.println("PublishMQTTSwitch");
  JsonDocument json;

  // Construct JSON config message
  json["name"] = uniquename;
  json["unique_id"] = host+"_"+uniquename;
  json["cmd_t"] = host+"/light/"+String(uniquename)+"/set";
  json["stat_t"] = host+"/light/"+String(uniquename)+"/state";

  addDeviceToJson(&json);

  char conf[512];
  serializeJson(json, conf);  // conf now contains the json

  // Publish config message
  MQTT.publish((String(mqttautodiscoverytopic)+"/light/"+host+"/"+String(uniquename)+"/config").c_str(),conf,mqttpersistence);

  // subscribe if need to listen to commands
  MQTT.subscribe((host+"/light/"+String(uniquename)+"/set").c_str());
}

void UpdateMQTTSwitch(const char* uniquename, bool Value)
{
  // Serial.println("UpdateMQTTSwitch");
  // publish state message
  MQTT.publish((host+"/light/"+String(uniquename)+"/state").c_str(),Value?"ON":"OFF",mqttpersistence);
}

void PublishMQTTBinarySensor(const char* uniquename, const char* deviceclass)
{
  Serial.println("PublishBinarySensor");
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

  addDeviceToJson(&json);

  char conf[512];
  serializeJson(json, conf);  // conf now contains the json

  // Publish config message
  MQTT.publish((String(mqttautodiscoverytopic)+"/binary_sensor/"+host+"/"+String(uniquename)+"/config").c_str(),conf,mqttpersistence);
}

void UpdateMQTTBinarySensor(const char* uniquename, bool Value)
{
  Serial.println("UpdateBinarySensor");

   JsonDocument json;
   json["value"] = Value;

   char message[512];
   serializeJson(json,message);

  // publish state message
  MQTT.publish((host+"/binary_sensor/"+String(uniquename)+"/state").c_str(),message,mqttpersistence);
}

void PublishMQTTTemperatureSensor(const char* uniquename)
{
  Serial.println("PublishMQTTTemperatureSensor");
  JsonDocument json;

  // Create message
  char conf[768];
  json["value_template"] =  "{{ value_json.value }}";
  json["device_class"] = "temperature";
  json["unit_of_measurement"] = "°C";
  json["state_topic"] = host+"/sensor/"+String(uniquename)+"/state";
  json["json_attributes_topic"] = host+"/sensor/"+String(uniquename)+"/state";
  json["name"] = uniquename;
  json["unique_id"] = host+"_"+uniquename;

  addDeviceToJson(&json);

  serializeJson(json, conf);  // buf now contains the json 

  // publsh the Message
  MQTT.publish((String(mqttautodiscoverytopic)+"/sensor/"+host+"/"+String(uniquename)+"/config").c_str(),conf,mqttpersistence);
}

void UpdateMQTTTemperatureSensor(const char* uniquename, float temperature)
{
  Serial.println("UpdateMQTTTemperatureSensor");
  JsonDocument json;

  // Create message
  char state[128];
  json["value"] =  float(int(temperature*10))/10;   // ensures round to 1 decimal behind the comma
  serializeJson(json, state);  // buf now contains the json 
  MQTT.publish((host+"/sensor/"+String(uniquename)+"/state").c_str(),state,mqttpersistence);
}

void PublishMQTTNumberSensor(const char* uniquename)
{
  Serial.println("PublishMQTTGenericSensor");
  JsonDocument json;

  // Create message
  char conf[768];
  json["state_topic"] = host+"/sensor/"+String(uniquename)+"/state";
  json["json_attributes_topic"] = host+"/sensor/"+String(uniquename)+"/state";
  json["value_template"] =  "{{ value_json.value }}";
  json["name"] = uniquename;
  json["unique_id"] = host+"_"+uniquename;

  addDeviceToJson(&json);

  serializeJson(json, conf);  // buf now contains the json 

  // publsh the Message
  MQTT.publish((String(mqttautodiscoverytopic)+"/sensor/"+host+"/"+String(uniquename)+"/config").c_str(),conf,mqttpersistence);
}

void UpdateMQTTNumberSensor(const char* uniquename, float value)
{
  Serial.println("UpdateMQTTGenericSensor");
  JsonDocument json;

  // Create message
  char state[128];
  json["value"] =  float(int(value*100))/100;   // ensures round to 1 decimal behind the comma
  serializeJson(json, state);  // buf now contains the json 

  MQTT.publish((host+"/sensor/"+String(uniquename)+"/state").c_str(),state,mqttpersistence);
}

void PublishMQTTPressureSensor(const char* uniquename)
{
  Serial.println("PublishMQTTPressureSensor");
  JsonDocument json;

  // Create message
  char conf[512];
  json["value_template"] =  "{{ value_json.value }}";
  json["device_class"] = "pressure";
  json["unit_of_measurement"] = "bar";
  json["state_topic"] = host+"/sensor/"+String(uniquename)+"/state";
  json["json_attributes_topic"] = host+"/sensor/"+String(uniquename)+"/state";
  json["name"] = uniquename;
  json["unique_id"] = host+"_"+uniquename;

  addDeviceToJson(&json);

  serializeJson(json, conf);  // buf now contains the json 

  // publsh the Message
  MQTT.publish((String(mqttautodiscoverytopic)+"/sensor/"+host+"/"+String(uniquename)+"/config").c_str(),conf,mqttpersistence);
}

void UpdateMQTTPressureSensor(const char* uniquename, float pressure)
{
  Serial.println("UpdateMQTTPressureSensor");
  // Create message
  char state[128];
  JsonDocument json;
  json["value"] =  pressure;
  serializeJson(json, state);  // buf now contains the json 
  // char charVal[10];
  // dtostrf(pressure,4,1,charVal); 
  MQTT.publish((host+"/sensor/"+String(uniquename)+"/state").c_str(),state,mqttpersistence);
}


void PublishMQTTPercentageSensor(const char* uniquename)
{
  Serial.println("PublishMQTTPercentageSensor");
  JsonDocument json;

  // Create message
  char conf[512];
  json["value_template"] =  "{{ value_json.value }}";
  json["device_class"] = "power_factor";
  json["unit_of_measurement"] = "%";
  json["state_topic"] = host+"/sensor/"+String(uniquename)+"/state";
  json["json_attributes_topic"] = host+"/sensor/"+String(uniquename)+"/state";
  json["name"] = uniquename;
  json["unique_id"] = host+"_"+uniquename;

  addDeviceToJson(&json);

  serializeJson(json, conf);  // buf now contains the json 

  // publsh the Message
  MQTT.publish((String(mqttautodiscoverytopic)+"/sensor/"+host+"/"+String(uniquename)+"/config").c_str(),conf,mqttpersistence);
}

void UpdateMQTTPercentageSensor(const char* uniquename, float percentage)
{
  Serial.println("UpdateMQTTPercentageSensor");
  // char charVal[10];
  // dtostrf(percentage,4,1,charVal); 
   JsonDocument json;

  // Create message
  char data[128];
  json["value"]=percentage;
  serializeJson(json,data);
 
  MQTT.publish((host+"/sensor/"+String(uniquename)+"/state").c_str(),data,mqttpersistence);
}

void PublishMQTTFaultCodeSensor(const char* uniquename)
{
  Serial.println("PublishMQTTFaultCodeSensor");
  JsonDocument json;

  // Create message
  char conf[512];
  json["value_template"] =  "{{ value_json.value }}";
  json["unit_of_measurement"] = "";
  json["state_topic"] = host+"/sensor/"+String(uniquename)+"/state";
  json["json_attributes_topic"] = host+"/sensor/"+String(uniquename)+"/state";
  json["name"] = uniquename;
  json["unique_id"] = host+"_"+uniquename;

  addDeviceToJson(&json);

  serializeJson(json, conf);  // buf now contains the json 

  // publsh the Message
  MQTT.publish((String(mqttautodiscoverytopic)+"/sensor/"+host+"/"+String(uniquename)+"/config").c_str(),conf,mqttpersistence);
}

void UpdateMQTTFaultCodeSensor(const char* uniquename, unsigned char FaultCode)
{
  Serial.println("UpdateMQTTFaultCodeSensor");
    // Create message
  char state[128];
  JsonDocument json;
  json["value"] =  FaultCode;
  serializeJson(json, state);  // buf now contains the json 

  MQTT.publish((host+"/sensor/"+String(uniquename)+"/state").c_str(),state,mqttpersistence);
}

void PublishMQTTSetpoint(const char* uniquename, int mintemp, int maxtemp, bool includeCooling)
{
  Serial.println("PublishMQTTSetpoint");
  JsonDocument json;

  // Create message
  char conf[1024];
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

  addDeviceToJson(&json);

  serializeJson(json, conf);  // buf now contains the json 

  // publsh the Message
  MQTT.publish((String(mqttautodiscoverytopic)+"/climate/"+host+"/"+String(uniquename)+"/config").c_str(),conf,mqttpersistence);
  MQTT.subscribe((host+"/climate/"+String(uniquename)+"/mode/set").c_str());
  MQTT.subscribe((host+"/climate/"+String(uniquename)+"/cmd_temp").c_str());
}

void UpdateMQTTSetpointTemperature(const char* uniquename,float value)
{
  Serial.println("UpdateMQTTSetpointtemperature");
  JsonDocument json;

  // Construct JSON config message
  // json["value"] = value;

  // char jsonstr[128];
  // serializeJson(json, jsonstr);  // conf now contains the json

  MQTT.publish((host+"/climate/"+String(uniquename)+"/Air_temperature").c_str(),("{ \"value\": "+String(value)+" }").c_str(),mqttpersistence);
}

void PublishMQTTNumber(const char* uniquename, int min, int max, float step, bool isSlider)
{
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

  addDeviceToJson(&json);

  char conf[1024];
  serializeJson(json, conf);  // buf now contains the json 

  // publsh the Message
  MQTT.publish((String(mqttautodiscoverytopic)+"/number/"+host+"/"+String(uniquename)+"/config").c_str(),conf,mqttpersistence);
  MQTT.subscribe((host+"/number/"+String(uniquename)+"/set").c_str());
  // Debug("Publish to "+String(mqttautodiscoverytopic)+"/number/"+host+"/"+String(uniquename)+"/config");
}

void UpdateMQTTNumber(const char* uniquename,float value)
{
  Serial.println("UpdateMQTTSetpointtemperature");
  JsonDocument json;

  // Construct JSON config message
  // json["value"] = value;

  // char jsonstr[128];
  // serializeJson(json, jsonstr);  // conf now contains the json

  MQTT.publish((host+"/number/"+String(uniquename)+"/state").c_str(),String(value).c_str(),mqttpersistence);
}

void PublishMQTTText(const char* uniquename)
{
  // Debug("PublishMQTTText");
  JsonDocument json;

  // Create message
  json["name"] = uniquename;
  json["unique_id"] = host+"_"+uniquename;
  json["stat_t"] = host+"/text/"+String(uniquename)+"/state";
  json["cmd_t"] = host+"/text/"+String(uniquename)+"/set";
  // json["stat_tpl"] = "{{value_json.value}}";

  addDeviceToJson(&json);

  char conf[1024];
  serializeJson(json, conf);  // buf now contains the json 

  // publsh the Message
  MQTT.publish((String(mqttautodiscoverytopic)+"/text/"+host+"/"+String(uniquename)+"/config").c_str(),conf,mqttpersistence);
  MQTT.subscribe((host+"/text/"+String(uniquename)+"/set").c_str());
  // Debug("Publish to "+String(mqttautodiscoverytopic)+"/text/"+host+"/"+String(uniquename)+"/config");
}

void UpdateMQTTText(const char* uniquename,const char* value)
{
  // Debug("UpdateMQTTText");
  JsonDocument json;

  MQTT.publish((host+"/text/"+String(uniquename)+"/state").c_str(),value,mqttpersistence);
}


void PublishMQTTCurvatureSelect(const char* uniquename)
{
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

  addDeviceToJson(&json);

  char conf[1024];
  serializeJson(json, conf);  // buf now contains the json 

  // publsh the Message
  MQTT.publish((String(mqttautodiscoverytopic)+"/select/"+host+"/"+String(uniquename)+"/config").c_str(),conf,mqttpersistence);
  MQTT.subscribe((host+"/select/"+String(uniquename)+"/set").c_str());
}

void UpdateMQTTCurvatureSelect(const char* uniquename,int value)
{
  Serial.println("UpdateMQTTCurvatureSelect");
  JsonDocument json;

  // Construct JSON config message
  json["curvature"] = getCurvatureStringFromInt(value);

  char jsonstr[128];
  serializeJson(json, jsonstr);  // conf now contains the json

  MQTT.publish((host+"/select/"+String(uniquename)+"/state").c_str(),jsonstr,mqttpersistence);
}


void UpdateMQTTBoilerSetpointMode()
{
  if (enableCentralHeating) {
    if (enableCooling) {
      // cooling and heating, set to Auto
      UpdateMQTTSetpointMode(Boiler_Setpoint_Name,21);
    } else {
      // only Central heating, set to Heating
      UpdateMQTTSetpointMode(Boiler_Setpoint_Name,1);
    }
  } else  {
    if (enableCooling) {
      // only cooling, set to Cooing
      UpdateMQTTSetpointMode(Boiler_Setpoint_Name,11);
    } else {
      // nothing, set to Off
      UpdateMQTTSetpointMode(Boiler_Setpoint_Name,0);
    }
  }

}

void UpdateMQTTSetpointMode(const char* uniquename,int value)
{
  Serial.println("UpdateMQTTSetpointmode");
  JsonDocument json;

  // Construct JSON config message
  // json["value"] = value;

  // char jsonstr[128];
  // serializeJson(json, jsonstr);  // conf now contains the json

  MQTT.publish((host+"/climate/"+String(uniquename)+"/mode").c_str(),("{ \"value\": "+String(value)+" }").c_str(),mqttpersistence);
}


void UpdateMQTTSetpoint(const char* uniquename, float temperature)
{
  Serial.println("UpdateMQTTSetpoint");
  JsonDocument json;

  // Construct JSON config message
  json["seltemp"] = temperature;

  char value[128];
  serializeJson(json, value);  // conf now contains the json

  MQTT.publish((host+"/climate/"+String(uniquename)+"/state").c_str(),value,mqttpersistence);
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

void PublishAllMQTTSensors()
{
  Serial.println("PublishAllMQTTSensors()");

  // Sensors
  PublishMQTTTemperatureSensor(Boiler_Temperature_Name);
  PublishMQTTTemperatureSensor(DHW_Temperature_Name);
  PublishMQTTTemperatureSensor(Return_Temperature_Name);
  PublishMQTTTemperatureSensor(Thermostat_Temperature_Name);
  PublishMQTTTemperatureSensor(Outside_Temperature_Name);
  PublishMQTTTemperatureSensor(OT_Outside_Temperature_Name);
  PublishMQTTPressureSensor(Pressure_Name);
  PublishMQTTPercentageSensor(Modulation_Name);
  PublishMQTTFaultCodeSensor(FaultCode_Name);
  PublishMQTTNumberSensor(P_Name);
  PublishMQTTNumberSensor(I_Name);
  PublishMQTTNumberSensor(D_Name);
  

  // binary sesnors sensors telling state
  PublishMQTTBinarySensor(FlameActive_Name,"heat");
  PublishMQTTBinarySensor(FaultActive_Name,"problem");
  PublishMQTTBinarySensor(DiagnosticActive_Name,"problem");
  PublishMQTTBinarySensor(CoolingActive_Name,"heat");
  PublishMQTTBinarySensor(CentralHeatingActive_Name,"heat");
  PublishMQTTBinarySensor(HotWaterActive_Name,"heat");
  PublishMQTTBinarySensor(FrostProtectionActive_Name,"cold");

  // Switches to control the boiler
  PublishMQTTSwitch(EnableCentralHeating_Name);
  PublishMQTTSwitch(EnableCooling_Name);
  PublishMQTTSwitch(EnableHotWater_Name);
  PublishMQTTSwitch(Weather_Dependent_Mode_Name);
  PublishMQTTSwitch(Holiday_Mode_Name);
  PublishMQTTSwitch(Debug_Name);

  // Publish setpoints
  PublishMQTTSetpoint(Boiler_Setpoint_Name,10,90,true);
  PublishMQTTSetpoint(DHW_Setpoint_Name,10,90,false);
  PublishMQTTSetpoint(Climate_Name,5,30,true);

  // publish boiler parameters
  PublishMQTTNumber(MinBoilerTemp_Name,10,50,0.5,true);
  PublishMQTTNumber(MaxBoilerTemp_Name,30,90,0.5,true);
  PublishMQTTNumber(MinimumTempDifference_Name,0,20,0.5,true);
  PublishMQTTNumber(FrostProtectionSetPoint_Name,0,90,0.5,true);
  PublishMQTTNumber(BoilerTempAtPlus20_Name,10,90,0.5,true);
  PublishMQTTNumber(BoilerTempAtMinus10_Name,10,90,0.5,true);
  PublishMQTTNumber(SwitchHeatingOffAt_Name,10,90,0.5,true);
  PublishMQTTNumber(ReferenceRoomCompensation_Name,0,30,0.5,true);
  PublishMQTTNumber(KP_Name,0,50,1,false);
  PublishMQTTNumber(KI_Name,0,1,0.01,false);
  PublishMQTTNumber(KD_Name,0,5,0.1,false);
  PublishMQTTCurvatureSelect(Curvature_Name);
  PublishMQTTText(MQTT_TempTopic_Name);
  PublishMQTTText(MQTT_OutsideTempTopic_Name);
  PublishMQTTText(Debug_Name);
  PublishMQTTText(Error_Name);

  // Subscribe to temperature topic
  if (mqtttemptopic.length()>0 and mqtttemptopic.toInt()==0) {
    MQTT.subscribe(mqtttemptopic.c_str());
  }
  // Subscribe to outside temperature topic
  if (mqttoutsidetemptopic.length()>0 and mqttoutsidetemptopic.toInt()==0) {
    MQTT.subscribe(mqttoutsidetemptopic.c_str());
  }

  // reset the timer
  t_last_mqtt_discovery=millis();

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
  server.onNotFound(handleNotFound);

  // Initialize OTA
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Set hostname & PAssword for OTA
  ArduinoOTA.setHostname(host.c_str());
  ArduinoOTA.setPassword(host.c_str()); // Disable for data upload

  ArduinoOTA.onStart([]() {
    if (ArduinoOTA.getCommand() == U_FLASH) {
      Debug("OTA: Start updating sketch...");
    } else { // U_FS
      Debug("OTA: Start updating filesystem...");
    }
    OTAUpdateInProgress=true;
  });

  ArduinoOTA.onEnd([]() {
    Debug("OTA Completed, restarting");
    OTAUpdateInProgress=false;
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Debug("OTA Progress: " + String(progress / (total / 100))+ "%");
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Debug("OTA: Error["+String(error)+"]:");
    if (error == OTA_AUTH_ERROR) {
      Debug("OTA: Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Debug("OTA: Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Debug("OTA: Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Debug("OTA: Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Debug("OTA: End Failed");
    }
    OTAUpdateInProgress=false;
  });
  
  ArduinoOTA.begin();
  
  //Start the server
  server.begin();   
  Serial.println("Opentherm Helper is waiting for commands");   

  //Init DS18B20 sensor
  sensors.begin();

  // Init builtin led
  pinMode(LED_BUILTIN, OUTPUT);

  // Start OpenTherm
  ot.begin(handleInterrupt);

  // MQTT
  MQTT.setServer(mqttserver.c_str(), mqttport); // server details
  MQTT.setBufferSize(1024); // discovery messages are longer than default max buffersize(!)

// Limit blocking times on sockets to reduce stalls during network issues
  espClient.setTimeout(1000);   // WiFiClient read timeout in ms
  MQTT.setSocketTimeout(2);     // PubSubClient socket timeout in seconds (default ~15)
  MQTT.setKeepAlive(15);        // Shorter keepalive to detect dead connections faster

  MQTT.setCallback(MQTTcallback); // listen to callbacks
}

// Loop Code
void loop()
{
  // Update Timeclient
  timeClient.update();

  // handle OTA
  ArduinoOTA.handle();

  // update Uptime
  updateTime();

  // Check if we have to save the config
  if ((!ClimateConfigSaved) and (millis()>t_save_config)) {
    SaveConfig();
  }

  // check if controlling by http stopped
  if (controlledByHTTP and millis()-t_last_http_command>HTTPTimeoutInMillis) {
    controlledByHTTP=false;
    HandleClimateMode(climate_Mode.c_str()); // do whatever needs to be done to reset the thermostat to the correct climate mode
  }

  // don't do anything if we are doing if OTA upgrade is in progress
  if (!OTAUpdateInProgress) {
    if (millis()-t_heartbeat>heartbeatTickInMillis) {
      //reset tick time
      t_heartbeat=millis();

      // (Re)connect MQTT
      if (!MQTT.connected()) {
        // We are no connected, so try to reconnect
        if (millis()-t_last_mqtt_try_connect>MQTTConnectTimeoutInMillis) {
          reconnect();
          t_last_mqtt_try_connect=millis();
        }
      } else {
        // we are connected, check if we have to resend discovery info
        if (millis()-t_last_mqtt_discovery>MQTTDiscoveryHeartbeatInMillis)
        {
          PublishAllMQTTSensors();
        }
      }
      
      if (millis()-t_last_mqtt_command>MQTTTimeoutInMillis and millis()-t_last_http_command>HTTPTimeoutInMillis and climate_Mode.equals("off")) {
          // No commands given for too long a time, so do nothing
          Serial.print("."); // Just print a dot, so we can see the software in still running
          digitalWrite(LED_BUILTIN, HIGH);    // switch the LED off, to indicate we lost connection

          // Switch off Heating and Cooling since there is no one controlling it
          enableCentralHeating=false;
          enableCooling=false;
          boiler_SetPoint=10;
      } else {
          digitalWrite(LED_BUILTIN, LOW);    // switch the LED on , to indicate we have connection
      }

      // handle MDNS
      MDNS.update();
    }

    // Remember last hour of temperatures (PID calculation needs to be able te determine if temperature is rising or dropping)
    {
      if (insideTemperatureReceived) {
        int CurrentMinute=(millis() % 60000) / 1000;
        insideTempAt[CurrentMinute] = mqttTemperature;   
      }
    }

    // Handle CLimate program
    handleClimateProgram(); 
    
    // Check if we have to communicatie steering vars
    CommunicateSteeringVarsToMQTT();

    // handle openthem commands
    handleOpenTherm();

    //Handle incoming webrequests
    server.handleClient();

    // handle MQTT
    MQTT.loop();

  }
} 
