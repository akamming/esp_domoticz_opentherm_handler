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
#include "config.h"                 // Set Configuration
#include <LittleFS.h>               // Filesystem


// vars to manage boiler
bool enableCentralHeating = false;
bool enableHotWater = true;
bool enableCooling = false;
float boiler_SetPoint = 0;
float dhw_SetPoint = 65;
float P=0;
float I=0;
float D=20;  

// Current Temp on Thermostat
float currentTemperature = 0;

// Current Temp on mqtt
float mqttTemperature = 99;

// return values from boiler
float dhw_Temperature = 0;
float boiler_Temperature = 0;
float return_Temperature = 0;
float outside_Temperature = 0;
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


// ot actions (main will loop through these actions in this order)
enum OTCommand { SetBoilerStatus, 
                 SetBoilerTemp, 
                 SetDHWTemp, 
                 GetBoilerTemp,  
                 GetDHWTemp, 
                 GetReturnTemp, 
                 GetOutsideTemp, 
                 GetPressure, 
                 GetFlowRate,
                 GetFaultCode, 
                 GetThermostatTemp 
               } OpenThermCommand ;

// vars for program logic
const char compile_date[] = __DATE__ " " __TIME__;  // Make sure we can output compile date
unsigned long t_heartbeat=millis()-heartbeatTickInMillis; // last heartbeat timestamp, init on previous heartbeat, so processing start right away
unsigned long t_last_mqtt_command=millis()-MQTTTimeoutInMillis; // last MQTT command timestamp. init on previous timeout value, so processing start right away
unsigned long t_last_http_command=millis()-HTTPTimeoutInMillis; // last HTTP command timestamp. init on previous timeout value, so processing start right away
unsigned long t_last_mqtt_discovery=millis()-MQTTDiscoveryHeartbeatInMillis; // last mqqt discovery timestamp
unsigned long t_last_climateheartbeat=0; // last climate heartbeat timestamp
unsigned long t_save_config; // timestamp for delayed save
bool ClimateConfigSaved=true;
bool OTAUpdateInProgress=false;
bool temperatureReceived=false;
bool debug=true;
float insideTempAt[60];

// objects to be used by program
ESP8266WebServer server(httpport);   //Web server object. Will be listening in port 80 (default for HTTP)
OneWire oneWire(OneWireBus);  // for OneWire Bus
DallasTemperature sensors(&oneWire); // for the temp sensor on one wire bus
OpenTherm ot(inPin, outPin); // for communication with Opentherm adapter
WiFiClient espClient;  // Needed for MQTT
PubSubClient MQTT(espClient); // MQTT client

void Debug(String text) {
  if (debug) {
    if (MQTT.connected()) {
      MQTT.publish((String(host)+"/debug").c_str(),text.c_str(),mqttpersistence);
    }
    Serial.println(text);
  }
}

void Error(const char* text) {
  if (MQTT.connected()) {
    MQTT.publish((String(host)+"/error").c_str(),text,mqttpersistence);
  }
  Serial.println(text);
}

void IRAM_ATTR handleInterrupt() {
    ot.handleInterrupt();
}

void SendHTTP(String command, String result) {
    String message = "{\n  \"InterfaceVersion\":2,\n  \"Command\":\""+command+"\",\n  \"Result\":\""+result+"\""+getSensors()+"\n}";
    server.send(200, "application/json", message);       //Response to the HTTP request
}

void handleResetWifiCredentials() {
  Serial.println("Resetting Wifi Credentials");
  WiFiManager wifiManager;
  wifiManager.resetSettings();
  SendHTTP("ResetWifiCredentials","OK");
  delay(500);

  Serial.println("Trigger watchdog to reset wemos");
  wdt_disable();
  wdt_enable(WDTO_15MS);
  while (1) {}
}



void handleGetSensors() {
  Serial.println("Getting the sensors");
  SendHTTP("GetSensors","OK");
}

void handleCommand() {
  String Statustext="Unknown Command";

  // we received a command, so someone is comunicating
  t_last_http_command=millis();

  // blink the LED, so we can see a command was sent
  digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off , to indicate we are executing a command

  // for Debugging purposes
  Serial.println("Handling Command: Number of args received: "+server.args());
  for (int i = 0; i < server.args(); i++) {
    Serial.println ("Argument "+String(i)+" -> "+server.argName(i)+": "+server.arg(i));

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

  // Enable/Disable Cooling
  if (server.arg("Cooling")!="") {
    Statustext="OK";
    if (server.arg("Cooling").equalsIgnoreCase("On")) {
      Serial.println("Enabling Cooling");
      enableCooling= true;
    } else {
      Serial.println("Disabling Cooling");
      enableCooling=false;
    }
  }

  // Enable/Disable Central Heating
  if (server.arg("CentralHeating")!="") {
    Statustext="OK";
    if (server.arg("CentralHeating").equalsIgnoreCase("On")) {
      Serial.println("Enabling Central Heating");
      enableCentralHeating=true;
    } else {
      Serial.println("Disabling Central Heating");
      enableCentralHeating=false;
    }
  }

  // Enable/Disable HotWater
  if (server.arg("HotWater")!="") {
    Statustext="OK";
    if (server.arg("HotWater").equalsIgnoreCase("On")) {
      Serial.println("Enabling Domestic Hot Water");
      enableHotWater=true;
    } else {
      Serial.println("Disabling Domestic Hot Water");
      enableHotWater=false;
    }
  }

  digitalWrite(LED_BUILTIN, LOW);    // turn the LED on , to indicate we executed the command

  SendHTTP("SetDHWTemp",Statustext);
}

String getSensors() { //Handler

  String message;

  // Add Compile Date
  message = ",\n  \"CompileDate\": \"" + String(compile_date) + "\"";

  // Add uptime
  message += ",\n  \"uptime\": \"" + String(y)+" years, "+String(d)+" days, "+String(h)+" hrs, "+String(m)+" mins & "+String(s)+" secs"+"\"";

  // Add Opentherm Status
  message += ",\n  \"OpenThermStatus\":";
  if (responseStatus == OpenThermResponseStatus::SUCCESS) {
      message += "\"OK\"";

  } else if (responseStatus == OpenThermResponseStatus::NONE) {
      message += "\"OpenTherm is not initialized\"";
  } else if (responseStatus == OpenThermResponseStatus::INVALID) {
      message += "\"Invalid response\"";
  } else if (responseStatus == OpenThermResponseStatus::TIMEOUT) {
      message += "\"Response timeout, is the boiler connected?\"";
  } else {
      message += "\"Unknown Status\"";
  }

  // Report if we are receiving commands
  if (millis()-t_last_http_command<HTTPTimeoutInMillis) {
    message += ",\n  \"ControlledBy\": \"HTTP\""; // HTTP overrules MQTT so check HTTP 1st
  } else if (millis()-t_last_mqtt_command<MQTTTimeoutInMillis) {
    message += ",\n  \"ControlledBy\": \"MQTT\""; // then check if MQTT command was given
  } else {
    message += ",\n  \"ControlledBy\": \"None\""; // no commands given lately
  }
  
  // Add MQTT Connection status 
  message += ",\n  \"MQTTconnected\": " + String(MQTT.connected() ? "true" : "false");
  message += ",\n  \"MQTTstate\": " + String(MQTT.state());

  // Add BoilerManagementVars
  message += ",\n  \"EnableCentralHeating\": " + String(enableCentralHeating ? "\"on\"" : "\"off\"");
  message += ",\n  \"EnableHotWater\": " + String(enableHotWater ? "\"on\"" : "\"off\"");
  message += ",\n  \"EnableCooling\": " + String(enableCooling ? "\"on\"" : "\"off\"");
  message +=",\n  \"BoilerSetpoint\": " + (String(boiler_SetPoint));
  message +=",\n  \"DHWSetpoint\": " + (String(dhw_SetPoint));
  message +=",\n  \"climateSetpoint\": " + String(climate_SetPoint);
  message +=",\n  \"climateMode\": \"" + String(climate_Mode) + "\"";
  message +=",\n  \"weatherDependentMode\": " + String(Weather_Dependent_Mode ? "\"on\"" : "\"off\"");
  message +=",\n  \"HolidayMode\": " + String(Holiday_Mode ? "\"on\"" : "\"off\"");
  message +=",\n  \"FrostProtectionActive\": " + String(FrostProtectionActive ? "\"on\"" : "\"off\"");
  
  // Add BoilerStatus
  message += ",\n  \"CentralHeating\": " + String(CentralHeating ? "\"on\"" : "\"off\"");
  message += ",\n  \"HotWater\": " + String(HotWater ? "\"on\"" : "\"off\"");
  message += ",\n  \"Cooling\": " + String(Cooling ? "\"on\"" : "\"off\"");
  message += ",\n  \"Flame\": " + String(Flame ? "\"on\"" : "\"off\"");
  message += ",\n  \"Fault\": " + String(Fault ? "\"on\"" : "\"off\"");
  message += ",\n  \"Diagnostic\": " + String(Diagnostic ? "\"on\"" : "\"off\"");

   // Add boiler sensors
  message +=",\n  \"BoilerTemperature\": " + (String)boiler_Temperature;
  message +=",\n  \"DhwTemperature\": " + String(dhw_Temperature);    
  message +=",\n  \"ReturnTemperature\": " + String(return_Temperature);
  message +=",\n  \"OutsideTemperature\": " + String(outside_Temperature);
  message +=",\n  \"Modulation\": " + String(modulation);
  message +=",\n  \"Pressure\": " + String(pressure);
  message +=",\n  \"Flowrate\": " + String(flowrate);
  message +=",\n  \"FaultCode\": " + String(FaultCode);

  // Add Temp Sensor value
  message +=",\n  \"ThermostatTemperature\": " + (String(currentTemperature+ThermostatTemperatureCalibration));
  message +=",\n  \"mqttTemperature\": " + (String(mqttTemperature));

  return message;
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
  json["uptime"] = String(y)+" years, "+String(d)+" days, "+String(h)+" hrs, "+String(m)+" ms, "+String(s)+" secs, "+String(ms)+" msec";
  json["compile_date"] = String(compile_date);
  
  serializeJson(json, buf); 
  server.send(200, "application/json", buf);       //Response to the HTTP request
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

void handleGetConfig()
{
  Serial.println("GetConfig");
  JsonDocument json;

  // gpio config
  json["inpin"] = inPin;
  json["outpin"] = outPin;
  json["temppin"] = OneWireBus;

  // mqtt config
  json["usemqtt"] = usemqtt;
  json["mqttserver"] = mqttserver;
  json["mqttport"] = mqttport;
  json["usemqttauthentication"] = usemqttauthentication;
  json["mqtttemptopic"] = mqtttemptopic;
  json["mqttuser"] = mqttuser;
  json["mqttpass"] = "*****"; // This is the only not allowed password, password will only be saved if it is not 5 stars
  json["mqttretained"] = mqttpersistence;  

  // PID settings
  json["KP"] = KP;
  json["KI"] = KI;
  json["KD"] = KD; 

  // Boiler Control Settings
  json["MinBoilerTemp"] = MinBoilerTemp;
  json["MaxBoilerTemp"] = MaxBoilerTemp;
  json["minimumTempDifference"] = minimumTempDifference;
  json["FrostProtectionSetPoint"] = FrostProtectionSetPoint;
  json["BoilerTempAtPlus20"] = BoilerTempAtPlus20;
  json["BoilerTempAtMinus10"] = BoilerTempAtMinus10; 
  json["Curvature"] = getCurvatureStringFromInt(Curvature);
  json["CurvatureInt"] = Curvature;
  json["SwitchHeatingOffAt"] = SwitchHeatingOffAt;
  json["ReferenceRoomCompensation"] = ReferenceRoomCompensation;

  // Add some General status info 
  if (MQTT.connected())
  {
    json["MQTTconnected"] = true;
  } else {
    json["MQTTconnected"] = false;
  }
  json["MQTTState"] = MQTT.state();
  json["debugtomqtt"]=debug;

  json["heap"] = ESP.getFreeHeap();
  json["uptime"] = String(y)+" years, "+String(d)+" days, "+String(h)+" hrs, "+String(m)+" ms, "+String(s)+" secs, "+String(ms)+" msec";


  // Send output
  char buf[1024];
  serializeJson(json, buf); 
  server.send(200, "application/json", buf);       //Response to the HTTP request
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
    // if password was not changed: leave old password in file
    if (json["mqttpass"]=="*****") {
      json["mqttpass"]=mqttpass;
    }

    // add/change the Climate settings
    json["climateMode"] = climate_Mode;
    json["climateSetpoint"] = climate_SetPoint;
    json["weatherDependentMode"] = Weather_Dependent_Mode;
    json["holidayMode"] = Holiday_Mode;

    //save the custom parameters to FS
    Debug("saving config");
    File configFile = LittleFS.open(CONFIGFILE, "w");
    if (!configFile) {
      server.send(500, "text/plain", "failed to open config file for writing");
      return;
    } else {
      
      // Save the file!
      serializeJson(json, configFile);
      configFile.close();

      // dump to response as well
      // serializeJson(json, Message);
      // server.send(200, "text/plain", Message.c_str());
      server.send(200, "text/plain", "New Config Saved");
      delay(500); // wait for server send to finish
      ESP.restart(); // restart
      
      return;
      //end save
    }
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
  Serial.println("handleReset");
  
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
    else if (endsWith(path,".htm") or endsWith(path,".html")) server.streamFile(file, "text/html");
    else if (endsWith(path,".css") ) server.streamFile(file, "text/css");
    else if (endsWith(path,".png") ) server.streamFile(file, "image/png");
    else if (endsWith(path,".gif") ) server.streamFile(file, "image/gif");
    else if (endsWith(path,".jpg") ) server.streamFile(file, "image/jpeg");
    else if (endsWith(path,".ico") ) server.streamFile(file, "image/x-icon");
    else if (endsWith(path,".xml") ) server.streamFile(file, "text/xml");
    else if (endsWith(path,".pdf") ) server.streamFile(file, "application./x-pdf");
    else if (endsWith(path,".zip") ) server.streamFile(file, "application./x-zip");
    else if (endsWith(path,".gz") ) server.streamFile(file, "application./x-gzip");
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
    String message = "File Not Found\n\nURI: "+server.uri() + "\nMethod: "+ ( server.method() == HTTP_GET ? "GET" : "POST" ) + "\nArguments "+server.args()+"\n";
    
    for (uint8_t i = 0; i < server.args(); i++)
    {
      message += server.argName(i)+"="+server.arg(i)+"\n";
    }
    
    server.send(404, "text/plain", message.c_str());
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
    return 10;
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
      // Allocate a buffer to store contents of the file.
      std::unique_ptr<char[]> buf(new char[size]);

      configFile.readBytes(buf.get(), size);

      JsonDocument json;
      auto deserializeError = deserializeJson(json, buf.get());
      serializeJson(json, Serial);
      if ( ! deserializeError ) {
        // mqtt config
        usemqtt=json["usemqtt"] | false;
        usemqttauthentication=json["usemqttauthentication"] | false;
        mqttserver=json["mqttserver"].as<String>();
        mqttport=json["mqttport"] | 1883;
        mqttuser=json["mqttuser"].as<String>();
        mqttpass=json["mqttpass"].as<String>();
        mqttpersistence=json["mqttretained"];
        
        //device config
        inPin=json["inpin"] | 4;
        outPin=json["outpin"] | 5;
        OneWireBus=json["temppin"] | 14;
        mqtttemptopic=json["mqtttemptopic"].as<String>();
        debug=json["debugtomqtt"] | true;

        // PID Settings
        KP = json["KP"] | 30;
        KI = json["KI"] | 0.01;
        KD = json["KD"] | 2.5;

        // Boiler Control Settings
        MinBoilerTemp = json["MinBoilerTemp"] | 10;
        MaxBoilerTemp = json["MaxBoilerTemp"] | 50;
        minimumTempDifference = json["minimumTempDifference"] | 3;
        FrostProtectionSetPoint = json["FrostProtectionSetPoint"] | 6;
        BoilerTempAtPlus20 = json["BoilerTempAtPlus20"] | 20;
        BoilerTempAtMinus10 = json["BoilerTempAtMinus10"] | 50;  
        Curvature=getCurvatureIntFromString(json["Curvature"] | "small");
        SwitchHeatingOffAt = json["SwitchHeatingOffAt"] | 19;
        ReferenceRoomCompensation = json["ReferenceRoomCompensation"] | 3;

        // persistent climate mode
        climate_Mode = json["climateMode"].as<String>();
        if (climate_Mode.equals("null")) { //if not present, null will be the value, so change to off
          climate_Mode="off";
        }
        climate_SetPoint = json["climateSetpoint"] | 20;
        Weather_Dependent_Mode = json["weatherDependentMode"] | false;
        Holiday_Mode = json["holidayMode"] | false;

        // close the file
        configFile.close();
      }
    } else {
        Debug("failed to load json config");
    }
  }
}

void SaveConfig()
{
  JsonDocument json;

  // first: Try to load existing configfile into json structure (prevent overwriting settings, we just want to add/change the climate settings in the config)

  if (LittleFS.exists(CONFIGFILE)) {
    //file exists, reading and loading
    Debug("reading config file");
    File configFile = LittleFS.open(CONFIGFILE, "r");
    if (configFile) {
      Serial.println("opened config file");
      size_t size = configFile.size();
      // Allocate a buffer to store contents of the file.
      std::unique_ptr<char[]> buf(new char[size]);

      configFile.readBytes(buf.get(), size);

      auto deserializeError = deserializeJson(json, buf.get());
      if ( ! deserializeError ) {
        Debug("parsed json");
      } else {
        Debug("failed to load json config");
      }
      configFile.close();
    }
  }

  // add/change the Climate settings
  json["climateMode"] = climate_Mode;
  json["climateSetpoint"] = climate_SetPoint;
  json["weatherDependentMode"] = Weather_Dependent_Mode;
  json["holidayMode"] = Holiday_Mode;

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
  return ot.isValidResponse(response) ? ot.getFloat(response) : 0;
}

float getDHWFlowrate() {
  unsigned long response = ot.sendRequest(ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::DHWFlowRate, 0));
  return ot.isValidResponse(response) ? ot.getFloat(response) : 0;
}

void CommunicateSteeringVarsToMQTT()
{
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

    // Climate Setpoint
    if ((climate_SetPoint-mqtt_climate_setpoint)>=0.1 or (climate_SetPoint-mqtt_climate_setpoint)<=-0.1){ // value changed
      UpdateMQTTSetpoint(Climate_Name,climate_SetPoint);
      mqtt_climate_setpoint=climate_SetPoint;
    }

    // The actual temperature for the climate device. Now the temp required from mqtt. But should be made switchable to other sources 
    if (mqttTemperature!=mqtt_mqttTemperature and temperatureReceived) {
      UpdateMQTTSetpointTemperature(Climate_Name,mqttTemperature);
      mqtt_mqttTemperature=mqttTemperature;
    }

    // Climate Setpoint
    if ((boiler_SetPoint-mqtt_boiler_setpoint)>=0.1 or (boiler_SetPoint-mqtt_boiler_setpoint)<=-0.1){ // value changed
      UpdateMQTTSetpoint(Boiler_Setpoint_Name,boiler_SetPoint);
      mqtt_boiler_setpoint=boiler_SetPoint;
    }

    // DHW Setpoint
    if ((dhw_SetPoint-mqtt_dhw_setpoint)>=0.1 or (dhw_SetPoint-mqtt_dhw_setpoint)<=-0.1){ // value changed
      UpdateMQTTSetpoint(DHW_Setpoint_Name,dhw_SetPoint);
      mqtt_dhw_setpoint=dhw_SetPoint;
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

    // Frost Protection Active
    if (FrostProtectionActive!=mqtt_FrostProtectionActive) // value changed
    {
      UpdateMQTTSwitch(FrostProtectionActive_Name,FrostProtectionActive);
      mqtt_FrostProtectionActive=FrostProtectionActive;
    }
  }
}

void InitPID()
{
  I=mqttTemperature;
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

float GetBoilerSetpointFromOutsideTemperature(float CurrentInsideTemperature, float CurrentOutsideTemperature) 
{
  float MaxYDelta=BoilerTempAtMinus10-BoilerTempAtPlus20; // boilertemp at -10 minus boilertemp at +20
  float MaxXDelta=30;                                     // 20 - (-10)=30

  // curve Calculation based on sine curve
  float TargetTemperatureWithoutCurvature= (20-CurrentOutsideTemperature) / MaxXDelta * MaxYDelta + BoilerTempAtPlus20;
  float ExtraCurvature=sin(PI*(20-CurrentOutsideTemperature)/MaxXDelta)*Curvature*MaxYDelta/100;
  float TargetTemperature=ExtraCurvature+TargetTemperatureWithoutCurvature;

  //Apply reference room compensation
  if (ReferenceRoomCompensation>0) {
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
  float  roomTemperature = mqttTemperature;

  if (climate_Mode.equals("off") or Holiday_Mode==true) { 
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
      // no need to actie, set the correct state
      FrostProtectionActive=false;
    }

  } else {
    // Some kind of Climate mode

    // At every heartbeat: Set PID values
    if (millis()-ClimateHeartbeatInMillis>t_last_climateheartbeat) {
      // Check if we have to update the PID (only when boiler not in hotwater mode)
      if (!HotWater) {
        UpdatePID(climate_SetPoint,roomTemperature);
      }

      // reset timestamp
      t_last_climateheartbeat=millis();
    }

    // Calculate BoilerSetpoint
    if (Weather_Dependent_Mode) {
      // calculate Setpoint based on outside temp
      boiler_SetPoint=GetBoilerSetpointFromOutsideTemperature(mqttTemperature,outside_Temperature);
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

void handleOpenTherm()
{
  // Handle commands
  switch (OpenThermCommand)
  {
    case SetBoilerStatus:
    {
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
            UpdateMQTTSwitch(FlameActive_Name,Flame);
            mqtt_Flame=Flame;
          }
          if (Fault!=mqtt_Fault){ // value changed
            UpdateMQTTSwitch(FaultActive_Name,Fault);
            mqtt_Fault=Fault;
          }
          if (Diagnostic!=mqtt_Diagnostic){ // value changed
            UpdateMQTTSwitch(DiagnosticActive_Name,Diagnostic);
            mqtt_Diagnostic=Diagnostic;
          }
          if (Cooling!=mqtt_Cooling){ // value changed
            UpdateMQTTSwitch(CoolingActive_Name,Cooling);
            mqtt_Cooling=Cooling;
          }
          if (CentralHeating!=mqtt_CentralHeating or modulation!=mqtt_modulation){ // Update switch when on/off or modulation changed
            if (CentralHeating) {
              UpdateMQTTDimmer(CentralHeatingActive_Name,CentralHeating,modulation);
            } else {
              UpdateMQTTDimmer(CentralHeatingActive_Name,CentralHeating,0);
            }
            mqtt_CentralHeating=CentralHeating;
          }
          if (HotWater!=mqtt_HotWater or modulation!=mqtt_modulation){ // Update switch when on/off or modulation changed
            if (HotWater) {
              UpdateMQTTDimmer(HotWaterActive_Name,HotWater,modulation);
            } else {
              UpdateMQTTDimmer(HotWaterActive_Name,HotWater,0);
            }
            mqtt_HotWater=HotWater;
          }
          if (modulation!=mqtt_modulation){ // value changed
            UpdateMQTTPercentageSensor(Modulation_Name,modulation);
            mqtt_modulation=modulation; // remember mqtt modulation value
          }
        }
        // Execute the next command in the next call
        OpenThermCommand = SetBoilerTemp;
      } else if (responseStatus == OpenThermResponseStatus::NONE) {
          Serial.println("Opentherm Error: OpenTherm is not initialized");
      } else if (responseStatus == OpenThermResponseStatus::INVALID) {
          Serial.println("Opentherm Error: Invalid response " + String(response, HEX));
      } else if (responseStatus == OpenThermResponseStatus::TIMEOUT) {
          Serial.println("Opentherm Error: Response timeout");
      }  else {
          Serial.println("Opentherm Error: unknown error");
      }
      break;
    }
    
    case SetBoilerTemp:
    {
      // set setpoint
      ot.setBoilerTemperature(boiler_SetPoint);

      OpenThermCommand = SetDHWTemp;
      break;
    }

    case SetDHWTemp:
    {
      ot.setDHWSetpoint(dhw_SetPoint);

       OpenThermCommand = GetBoilerTemp;
      break;
    }

    case GetBoilerTemp:
    {
      boiler_Temperature = ot.getBoilerTemperature();

      // Check if we have to send to MQTT
      if (MQTT.connected()) {
        float delta = mqtt_boiler_Temperature-boiler_Temperature;
        if (delta<-0.1 or delta>0.1){ // value changed
          UpdateMQTTTemperatureSensor(Boiler_Temperature_Name,boiler_Temperature);
          UpdateMQTTSetpointTemperature(Boiler_Setpoint_Name,boiler_Temperature);
          mqtt_boiler_Temperature=boiler_Temperature;
        }
      }

      OpenThermCommand = GetDHWTemp;
      break;
    }

    case GetDHWTemp:
    {
      dhw_Temperature = ot.getDHWTemperature();

      // Check if we have to send to MQTT
      if (MQTT.connected()) {
        float delta = mqtt_dhw_Temperature-dhw_Temperature;
        if (delta<-0.1 or delta>0.1){ // value changed
          UpdateMQTTTemperatureSensor(DHW_Temperature_Name,dhw_Temperature);
          UpdateMQTTSetpointTemperature(DHW_Setpoint_Name,dhw_Temperature);
          mqtt_dhw_Temperature=dhw_Temperature;
        }
      }

      OpenThermCommand = GetReturnTemp;
      break;
    }
      
    case GetReturnTemp:
    {
      return_Temperature = ot.getReturnTemperature();

      // Check if we have to send to MQTT
      if (MQTT.connected()) {
        float delta = mqtt_return_Temperature-return_Temperature;
        if (delta<-0.1 or delta>0.1){ // value changed
          UpdateMQTTTemperatureSensor(Return_Temperature_Name,return_Temperature);
          mqtt_return_Temperature=return_Temperature;
        }
      }

      OpenThermCommand = GetOutsideTemp;
      break;
    }
      
    case GetOutsideTemp:
    {
      outside_Temperature = getOutsideTemperature();
      
      // Check if we have to send to MQTT
      if (MQTT.connected()) {
        float delta = mqtt_outside_Temperature-outside_Temperature;
        if (delta<-0.1 or delta>0.1){ // value changed
          UpdateMQTTTemperatureSensor(Outside_Temperature_Name,outside_Temperature);
          mqtt_outside_Temperature=outside_Temperature;
        }
      }

      OpenThermCommand = GetPressure;
      break;
    }
      
    case GetPressure: 
    {
      pressure = ot.getPressure();

      // Check if we have to send to MQTT
      if (MQTT.connected()) {
        float delta = mqtt_pressure-pressure;
        if (delta<-0.01 or delta>0.01){ // value changed
          UpdateMQTTPressureSensor(Pressure_Name,pressure);
          mqtt_pressure=pressure;
        }
      }

      OpenThermCommand = GetFlowRate;
      break;
    }

    case GetFlowRate: 
    {
      flowrate = getDHWFlowrate();

      /* Check if we have to send to MQTT
      if (MQTT.connected()) {
        float delta = mqtt_flowrate-flowrate;
        if (delta<-0.01 or delta>0.01){ // value changed
          UpdateMQTTPressureSensor(Pressure_Name,pressure);
          mqtt_pressure=pressure;
        }
      } */

      OpenThermCommand = GetFaultCode;
      break;
    }
 
    case GetFaultCode:
    {
      FaultCode = ot.getFault();

      // Check if we have to send to MQTT
      if (MQTT.connected()) {
        if (FaultCode!=mqtt_FaultCode){ // value changed
          UpdateMQTTFaultCodeSensor(FaultCode_Name,FaultCode);
          mqtt_FaultCode=FaultCode;
        }
      }

      
      OpenThermCommand=GetThermostatTemp;
      break;
    }

    case GetThermostatTemp:
    {
      sensors.requestTemperatures(); // Send the command to get temperature readings 
      currentTemperature = sensors.getTempCByIndex(0);

      // Check if we have to send to MQTT
      if (MQTT.connected()) {
        float delta = mqtt_currentTemperature-currentTemperature;
        if (delta<-0.1 or delta>0.1){ // value changed
          UpdateMQTTTemperatureSensor(Thermostat_Temperature_Name,currentTemperature);
          mqtt_currentTemperature=currentTemperature;
        }
      }

      OpenThermCommand=SetBoilerStatus;
      
      break;
    }

  }
}

String CommandTopic(const char* DeviceName){
  return String(host)+String("/light/")+String(DeviceName)+String("/set");
}

String SetpointCommandTopic(const char* DeviceName){
  return String(host)+"/climate/"+String(DeviceName)+"/cmd_temp";
}

void LogMQTT(const char* topic, const char* payloadstr, const char* length, const char* logtext) {
  MQTT.publish((String(host)+"/error").c_str(),("Message ("+String(payloadstr)+") with length "+String(length)+" received on topic "+String(topic)+", with log "+String(logtext)).c_str(),mqttpersistence);
}

bool HandleClimateMode(const char* mode)
{
  bool CommandSucceeded=true;
  // Init PID calculater
  InitPID();

  if (String(mode).equals("off") or String(mode).equals("heat") or String(mode).equals("cool") or String(mode).equals("auto")) {
  climate_Mode=mode;
  DelayedSaveConfig();

  } else {
    Error("Unknown payload for Climate mode command");
    // sendback current mode to requester, so trick program into making it think it has to communicatie
    mqtt_climate_Mode="abcd"; // random value, so the program thinks it is changed next time it wants to communicate
    CommandSucceeded=false;
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
  // Handle actions if first time received
  if (!temperatureReceived) {
    Debug("resetting insidetemp matrix to "+String(value));
    // Set  InsideTempAt array at default value (currenttemp)
    for (int i=0;i<60;i++) {
      insideTempAt[i]=value;
    }
    temperatureReceived=true; // Make sure we do this only once ;-)
  }
  // Set mqttTemperature
  mqttTemperature=value; 
  InitPID();
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

void MQTTcallback(char* topic, byte* payload, unsigned int length) {
  // get vars from callback
  String topicstr=String(topic);
  char payloadstr[256];
  strncpy(payloadstr,(char *)payload,length);
  payloadstr[length]='\0';
  Debug("Received command on topic ["+topicstr+"], content: ["+payloadstr+"]");

  // Assume succesful command 
  bool CommandSucceeded=true;
  
  if (millis()-t_last_http_command>HTTPTimeoutInMillis) { // only execute mqtt commands if not commanded by http
    // decode payload
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payloadstr);
  
    if (error) {
      // Might be a string, so handle...
      // Climate  mode receive
      if (topicstr.equals(host+"/climate/"+String(Climate_Name)+"/mode/set")) {
        CommandSucceeded = HandleClimateMode(payloadstr);
        
      // Boiler Setpoint mode receive
      } else if (topicstr.equals(host+"/climate/"+String(Boiler_Setpoint_Name)+"/mode/set")) {
          CommandSucceeded=HandleBoilerMode(payloadstr);
      // DHW Setpoint mode receive
      } else if (topicstr.equals(host+"/climate/"+String(DHW_Setpoint_Name)+"/mode/set")) {
          CommandSucceeded=HandleDHWMode(payloadstr);
      // Handle Enable Hotwater switch command
      } else if (topicstr.equals(CommandTopic(EnableHotWater_Name))) {
        // we have a match
        if (String(payloadstr).equals("ON")){
          enableHotWater=true;
        } else if (String(payloadstr).equals("OFF")) {
          enableHotWater=false;
        } else {
          LogMQTT(topicstr.c_str(),payloadstr,String(length).c_str(),"unknown state");
          CommandSucceeded=false;
        }
      // Handle EnableCooling switch command
      } else if (topicstr.equals(CommandTopic(EnableCooling_Name))) {
        // we have a match
        if (String(payloadstr).equals("ON")){
          enableCooling=true;
        } else if (String(payloadstr).equals("OFF")) {
          enableCooling=false;
        } else {
          LogMQTT(topicstr.c_str(),payloadstr,String(length).c_str(),"unknown state");
          CommandSucceeded=false;
        } 
      // Enable Central Heating switch command
      } else if (topicstr.equals(CommandTopic(EnableCentralHeating_Name))) {
        // we have a match
        if (String(payloadstr).equals("ON")){
          enableCentralHeating=true;
        } else if (String(payloadstr).equals("OFF")) {
          enableCentralHeating=false;
        } else {
          LogMQTT(topicstr.c_str(),payloadstr,String(length).c_str(),"unknown state");
          CommandSucceeded=false;
        }
      // Weather Dependent Mode command
      } else if (topicstr.equals(CommandTopic(Weather_Dependent_Mode_Name))) {
        // we have a match
        if (String(payloadstr).equals("ON")){
          Weather_Dependent_Mode=true;
          DelayedSaveConfig();
        } else if (String(payloadstr).equals("OFF")) {
          Weather_Dependent_Mode=false;
          InitPID();
          DelayedSaveConfig();
        } else {
          LogMQTT(topicstr.c_str(),payloadstr,String(length).c_str(),"unknown state");
          CommandSucceeded=false;
        }
      // Holiday Mode command
      } else if (topicstr.equals(CommandTopic(Holiday_Mode_Name))) {
        // we have a match
        if (String(payloadstr).equals("ON")){
          Holiday_Mode=true;
          DelayedSaveConfig();
        } else if (String(payloadstr).equals("OFF")) {
          Holiday_Mode=false;
          InitPID();
          DelayedSaveConfig();
        } else {
          LogMQTT(topicstr.c_str(),payloadstr,String(length).c_str(),"unknown state");
          CommandSucceeded=false;
        }
      } else {
        // apparently we needed to deserialize, so log the error
        LogMQTT(topicstr.c_str(),payloadstr,String(length).c_str(),"Command unknown");
        CommandSucceeded=false;
      }
    } else {
      // json was decoded

      // Handle Enable Hotwater switch command
      if (topicstr.equals(CommandTopic(EnableHotWater_Name))) {
        // we have a match
        if (String(doc["state"]).equals("ON")){
          enableHotWater=true;
        } else if (String(doc["state"]).equals("OFF")) {
          enableHotWater=false;
        } else {
          LogMQTT(topicstr.c_str(),payloadstr,String(length).c_str(),"unknown state");
          CommandSucceeded=false;
        }
      // Handle EnableCooling switch command
      } else if (topicstr.equals(CommandTopic(EnableCooling_Name))) {
        // we have a match
        if (String(doc["state"]).equals("ON")){
          enableCooling=true;
        } else if (String(doc["state"]).equals("OFF")) {
          enableCooling=false;
        } else {
          LogMQTT(topicstr.c_str(),payloadstr,String(length).c_str(),"unknown state");
          CommandSucceeded=false;
        } 
      // Enable Central Heating switch command
      } else if (topicstr.equals(CommandTopic(EnableCentralHeating_Name))) {
        // we have a match
        if (String(doc["state"]).equals("ON")){
          enableCentralHeating=true;
        } else if (String(doc["state"]).equals("OFF")) {
          enableCentralHeating=false;
        } else {
          LogMQTT(topicstr.c_str(),payloadstr,String(length).c_str(),"unknown state");
          CommandSucceeded=false;
        }
      // Weather Dependent mode on or off
      } else if (topicstr.equals(CommandTopic(Weather_Dependent_Mode_Name))) {
        // we have a match
        if (String(doc["state"]).equals("ON")){
          Weather_Dependent_Mode=true;
          DelayedSaveConfig();
        } else if (String(doc["state"]).equals("OFF")) {
          Weather_Dependent_Mode=false;
          InitPID();
          DelayedSaveConfig();
        } else {
          LogMQTT(topicstr.c_str(),payloadstr,String(length).c_str(),"unknown state");
          CommandSucceeded=false;
        }
      // Holiday mode on or off
      } else if (topicstr.equals(CommandTopic(Holiday_Mode_Name))) {
        // we have a match
        if (String(doc["state"]).equals("ON")){
          Holiday_Mode=true;
          DelayedSaveConfig();
        } else if (String(doc["state"]).equals("OFF")) {
          Holiday_Mode=false;
          InitPID();
          DelayedSaveConfig();
        } else {
          LogMQTT(topicstr.c_str(),payloadstr,String(length).c_str(),"unknown state");
          CommandSucceeded=false;
        }
      // Boiler Setpoint temperature command
      } else if (topicstr.equals(SetpointCommandTopic(Boiler_Setpoint_Name))) {
        boiler_SetPoint=String(payloadstr).toFloat();

      // Boiler Setpoint mode command
      } else if (topicstr.equals(host+"/climate/"+String(Boiler_Setpoint_Name)+"/mode/set")) {
        CommandSucceeded=HandleBoilerMode(SetpointIntToString(int(doc["value"])));

      // Climate Setpoint mode Commands
      } else if (topicstr.equals(host+"/climate/"+String(Climate_Name)+"/mode/set")) {
        CommandSucceeded = HandleClimateMode(SetpointIntToString(int(doc["value"])));

      // Climate setpoint temperature command
      } else if (topicstr.equals(SetpointCommandTopic(Climate_Name))) {
        climate_SetPoint=String(payloadstr).toFloat();
        DelayedSaveConfig();

      // DHW Setpoint commands
      } else if (topicstr.equals(SetpointCommandTopic(DHW_Setpoint_Name))) {
        dhw_SetPoint=String(payloadstr).toFloat();
      } else if (topicstr.equals(host+"/climate/"+String(DHW_Setpoint_Name)+"/mode/set")) {
        CommandSucceeded=HandleDHWMode(SetpointIntToString(int(doc["value"])));
        
      // MQTT temperature received
      } else if (topicstr.equals(mqtttemptopic)) {
        SetMQTTTemperature(doc["value"]);
      
      // Unknown command
      } else {
        LogMQTT(topicstr.c_str(),payloadstr,String(length).c_str(),"unknown topic");
        CommandSucceeded=false;
      }
    }
    if (CommandSucceeded) {
      // we received a succesful mqtt command, so someone is communicating correctly
      t_last_mqtt_command=millis();
    }
  }
}

void reconnect()
{
  if (usemqtt) {
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
      } else {
        Serial.print("failed, rc=");
        Serial.print(MQTT.state());
      }
    }
  }
}

void PublishMQTTDimmer(const char* uniquename)
{
  Serial.println("UpdateMQTTDimmer");
  JsonDocument json;

  // Construct JSON config message
  json["name"] = uniquename;
  json["unique_id"] = host+"_"+uniquename;
  json["cmd_t"] = host+"/light/"+String(uniquename)+"/set";
  json["stat_t"] = host+"/light/"+String(uniquename)+"/state";
  json["schema"] = "json";
  json["brightness"] = true;

  JsonObject dev = json["dev"].to<JsonObject>();
  String MAC = WiFi.macAddress();
  MAC.replace(":", "");
  dev["ids"] = MAC;
  dev["name"] = host;
  dev["sw"] = String(host)+"_"+String(__DATE__)+"_"+String(__TIME__);
  dev["mdl"] = "d1_mini";
  dev["mf"] = "espressif";

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

void PublishMQTTSwitch(const char* uniquename, bool controllable)
{
  Serial.println("PublishMQTTSwitch");
  JsonDocument json;

  // Construct JSON config message
  json["name"] = uniquename;
  json["unique_id"] = host+"_"+uniquename;
  json["cmd_t"] = host+"/light/"+String(uniquename)+"/set";
  json["stat_t"] = host+"/light/"+String(uniquename)+"/state";

  JsonObject dev = json["dev"].to<JsonObject>();
  String MAC = WiFi.macAddress();
  MAC.replace(":", "");
  dev["ids"] = MAC;
  dev["name"] = host;
  dev["sw"] = String(host)+"_"+String(__DATE__)+"_"+String(__TIME__);
  dev["mdl"] = "d1_mini";
  dev["mf"] = "espressif";


  char conf[512];
  serializeJson(json, conf);  // conf now contains the json

  // Publish config message
  MQTT.publish((String(mqttautodiscoverytopic)+"/light/"+host+"/"+String(uniquename)+"/config").c_str(),conf,mqttpersistence);

  // subscribe if need to listen to commands
  if (controllable) {
    MQTT.subscribe((host+"/light/"+String(uniquename)+"/set").c_str());
  }
}

void UpdateMQTTSwitch(const char* uniquename, bool Value)
{
  Serial.println("UpdateMQTTSwitch");
  // publish state message
  MQTT.publish((host+"/light/"+String(uniquename)+"/state").c_str(),Value?"ON":"OFF",mqttpersistence);
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

  JsonObject dev = json["dev"].to<JsonObject>();
  String MAC = WiFi.macAddress();
  MAC.replace(":", "");
  dev["ids"] = MAC;
  dev["name"] = host;
  dev["sw"] = String(host)+"_"+String(__DATE__)+"_"+String(__TIME__);
  dev["mdl"] = "d1_mini";
  dev["mf"] = "espressif";

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
  json["value"] =  temperature;
  serializeJson(json, state);  // buf now contains the json 
  // char charVal[10];
  // dtostrf(temperature,4,1,charVal); 
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

  JsonObject dev = json["dev"].to<JsonObject>();
  String MAC = WiFi.macAddress();
  MAC.replace(":", "");
  dev["ids"] = MAC;
  dev["name"] = host;
  dev["sw"] = String(host)+"_"+String(__DATE__)+"_"+String(__TIME__);
  dev["mdl"] = "d1_mini";
  dev["mf"] = "espressif";

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

  JsonObject dev = json["dev"].to<JsonObject>();
  String MAC = WiFi.macAddress();
  MAC.replace(":", "");
  dev["ids"] = MAC;
  dev["name"] = host;
  dev["sw"] = String(host)+"_"+String(__DATE__)+"_"+String(__TIME__);
  dev["mdl"] = "d1_mini";
  dev["mf"] = "espressif";

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
  // json["device_class"] = "None";
  json["unit_of_measurement"] = "";
  json["state_topic"] = host+"/sensor/"+String(uniquename)+"/state";
  json["json_attributes_topic"] = host+"/sensor/"+String(uniquename)+"/state";
  json["name"] = uniquename;
  json["unique_id"] = host+"_"+uniquename;

  JsonObject dev = json["dev"].to<JsonObject>();
  String MAC = WiFi.macAddress();
  MAC.replace(":", "");
  dev["ids"] = MAC;
  dev["name"] = host;
  dev["sw"] = String(host)+"_"+String(__DATE__)+"_"+String(__TIME__);
  dev["mdl"] = "d1_mini";
  dev["mf"] = "espressif";

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



void PublishMQTTSetpoint(const char* uniquename, int mintemp, int maxtemp, bool includecooling)
{
  Serial.println("PublishMQTTSetpoint");
  JsonDocument json;

  // Create message
  char conf[1024];
  json["min_temp"] = mintemp;
  json["max_temp"] = maxtemp;
  JsonArray modes = json["modes"].to<JsonArray>();
  modes.add("off");
  modes.add("heat");
  if (includecooling) {
    modes.add("cool");
    modes.add("auto");
  }

  json["name"] = uniquename;
  json["unique_id"] = host+"_"+uniquename;
  json["temp_cmd_t"] = host+"/climate/"+String(uniquename)+"/cmd_temp";
  json["temp_stat_t"] = host+"/climate/"+String(uniquename)+"/state";
  json["temp_stat_tpl"] = "{{value_json.seltemp}}";
  json["curr_temp_t"] = host+"/climate/"+String(uniquename)+"/Air_temperature";
  json["curr_temp_tpl"] = "{{ value_json.value }}";
  json["mode_stat_t"] = host+"/climate/"+String(uniquename)+"/mode";
  json["mode_cmd_t"] = host+"/climate/"+String(uniquename)+"/mode/set";
  json["mode_stat_tpl"] =  "{{ {"+String(OFF)+": \"off\", "+String(HEAT)+": \"heat\", "+String(COOL)+": \"cool\", "+String(AUTO)+": \"auto\"}[value_json.value] | default('off') }}";
  json["temp_step"] = 0.5;
  json["temp_unit"] = "C";
  json["precision"] = 0.1;

  JsonObject dev = json["dev"].to<JsonObject>();
  String MAC = WiFi.macAddress();
  MAC.replace(":", "");
  dev["ids"] = MAC;
  dev["name"] = host;
  dev["sw"] = String(host)+"_"+String(__DATE__)+"_"+String(__TIME__);
  dev["mdl"] = "d1_mini";
  dev["mf"] = "espressif";

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
  PublishMQTTPressureSensor(Pressure_Name);
  PublishMQTTPercentageSensor(Modulation_Name);
  PublishMQTTFaultCodeSensor(FaultCode_Name);

  // On/off sensors telling state
  PublishMQTTSwitch(FlameActive_Name,false);
  PublishMQTTSwitch(FaultActive_Name,false);
  PublishMQTTSwitch(DiagnosticActive_Name,false);
  PublishMQTTSwitch(CoolingActive_Name,false);
  PublishMQTTDimmer(CentralHeatingActive_Name);
  PublishMQTTDimmer(HotWaterActive_Name);
  PublishMQTTSwitch(FrostProtectionActive_Name,false);
  t_last_mqtt_discovery=millis();

  // Switches to control the boiler
  PublishMQTTSwitch(EnableCentralHeating_Name,true);
  PublishMQTTSwitch(EnableCooling_Name,true);
  PublishMQTTSwitch(EnableHotWater_Name,true);
  PublishMQTTSwitch(Weather_Dependent_Mode_Name,true);
  PublishMQTTSwitch(Holiday_Mode_Name,true);

  // Publish setpoints
  PublishMQTTSetpoint(Boiler_Setpoint_Name,10,90,true);
  PublishMQTTSetpoint(DHW_Setpoint_Name,10,90,false);
  PublishMQTTSetpoint(Climate_Name,5,30,true);

  // Subscribe to temperature topic
  if (mqtttemptopic.length()>0) {
    MQTT.subscribe(mqtttemptopic.c_str());
  }
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
  server.onNotFound(handleNotFound);

  // Initialize OTA
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(host.c_str());

  // No authentication by default
  // ArduinoOTA.setPassword(host.c_str()); // Disable for data upload

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    if (ArduinoOTA.getCommand() == U_FLASH) {
      Serial.println("OTA: Start updating sketch...");
    } else { // U_FS
      Serial.println("OTA: Start updating filesystem...");
    }
    OTAUpdateInProgress=true;
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
    OTAUpdateInProgress=false;
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA: Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("OTA: Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("OTA: Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("OTA: Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("OTA: Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("OTA: End Failed");
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
  MQTT.setCallback(MQTTcallback); // listen to callbacks
}

// Loop Code
void loop()
{
  // handle OTA
  ArduinoOTA.handle();

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

      // (Re)connect MQTT
      if (!MQTT.connected()) {
        // We are no connected, so try to reconnect
        reconnect();
      } else {
        // we are connected, check if we have to resend discovery info
        if (millis()-t_last_mqtt_discovery>MQTTDiscoveryHeartbeatInMillis)
        {
          PublishAllMQTTSensors();
        }
      }
      
      if (millis()-t_last_mqtt_command>MQTTTimeoutInMillis and millis()-t_last_http_command>HTTPTimeoutInMillis) {
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
      if (temperatureReceived) {
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
