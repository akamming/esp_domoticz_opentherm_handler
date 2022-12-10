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
-IN  = Arduino (3) / ESP8266 (5) Output Pin
-OUT = Arduino (2) / ESP8266 (4) Input Pin
*/


#include <Arduino.h>              // duh...
#include <OpenTherm.h>            //https://github.com/ihormelnyk/opentherm_library
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <ESP8266WebServer.h>     //Local Webserver for receiving http commands
#include <ESP8266mDNS.h>          // To respond on hostname as well
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <OneWire.h>              // One Wire bus
#include <DallasTemperature.h>    // temperature sensors
#include <ArduinoOTA.h>           // OTA updates
#include <ArduinoJson.h>          // make JSON payloads
#include <PubSubClient.h>         // MQTT library
#include "config.h"                 // Set Configuration

// vars to manage boiler
bool enableCentralHeating = false;
bool enableHotWater = true;
bool enableCooling = false;
float boiler_SetPoint = 0;
float dhw_SetPoint = 65;

// device names for mqtt autodiscovery
const char Boiler_Temperature_Name[] = "Boiler_Temperature";   
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

// Current Temp on Thermostat
float currentTemperature = 0;

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


// reported vars to mqtt
float mqtt_boiler_Temperature=0;
float mqtt_outside_Temperature=0;
float mqtt_currentTemperature=0;
bool mqtt_CentralHeating=true;
bool mqtt_HotWater=true;
bool mqtt_Cooling=true;
bool mqtt_Flame=true;
bool mqtt_Fault=true;
bool mqtt_Diagnostic=true;
float mqtt_modulation=100;
bool mqtt_enable_HotWater=false;
bool mqtt_enable_CentralHeating=true;
bool mqtt_enable_Cooling=true;
float mqtt_boiler_setpoint=1;
float mqtt_dhw_setpoint=0;

// ot actions (main will loop through these actions in this order)
enum OTCommand { SetBoilerStatus, 
                 SetBoilerTemp, 
                 SetDHWTemp, 
                 GetBoilerTemp,  
                 GetDHWTemp, 
                 GetReturnTemp, 
                 GetOutsideTemp, 
                 GetModulation, 
                 GetPressure, 
                 GetFlowRate, 
                 GetFaultCode, 
                 GetThermostatTemp 
               } OpenThermCommand ;

// vars for program logic
const char compile_date[] = __DATE__ " " __TIME__;  // Make sure we can output compile date
unsigned long t_heartbeat=millis()-heartbeatTickInMillis; // last heartbeat timestamp, init on previous heartbeat, so processing start right away
unsigned long t_last_command=millis()-domoticzTimeoutInMillis; // last domoticz command timestamp. init on previous timeout value, so processing start right away
unsigned long t_last_mqtt_discovery=millis()-MQTTDiscoveryHeartbeatInMillis; // last mqqt discovery timestamp
bool OTAUpdateInProgress=false;

// objects to be used by program
ESP8266WebServer server(httpport);   //Web server object. Will be listening in port 80 (default for HTTP)
OneWire oneWire(OneWireBus);  // for OneWire Bus
DallasTemperature sensors(&oneWire); // for the temp sensor on one wire bus
OpenTherm ot(inPin, outPin); // for communication with Opentherm adapter
WiFiClient espClient;  // Needed for MQTT
PubSubClient MQTT(espClient); // MQTT client


void ICACHE_RAM_ATTR handleInterrupt() {
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
  t_last_command=millis();
}

void handleCommand() {
  String Statustext="Unknown Command";

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
    t_last_command=millis();
  }

  // Set Boiler Temp
  if (server.arg("BoilerTemperature")!="") {
    Serial.println("Setting boiler temp to "+server.arg("BoilerTemperature"));
    boiler_SetPoint=server.arg("BoilerTemperature").toFloat();
    Statustext="OK";
    t_last_command=millis();
  }

  // Enable/Disable Cooling
  if (server.arg("Cooling")!="") {
    t_last_command=millis();
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
    t_last_command=millis();
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
    t_last_command=millis();
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
    message += ",\n  \"CompileDate\": \"" + String(compile_date) + "\"";
 
    // Add Status
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
    


    // Add BoilerManagementVars
    message += ",\n  \"EnableCentralHeating\": " + String(enableCentralHeating ? "\"on\"" : "\"off\"");
    message += ",\n  \"EnableHotWater\": " + String(enableHotWater ? "\"on\"" : "\"off\"");
    message += ",\n  \"EnableCooling\": " + String(enableCooling ? "\"on\"" : "\"off\"");
    message +=",\n  \"BoilerSetpoint\": " + (String(boiler_SetPoint));
    message +=",\n  \"DHWSetpoint\": " + (String(dhw_SetPoint));

    
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

    return message;
}

void handleGetInfo()
{
  Serial.println("GetInfo");
  StaticJsonDocument<512> json;
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

  long seconds=millis()/1000;
  int secs = seconds % 60;
  int mins = (seconds/60) % 60;
  int hrs = (seconds/3600) % 24;
  int days = (seconds/(3600*24)); 
  json["uptime"] = String(days)+" days, "+String(hrs)+" hours, "+String(mins)+" minutes, "+String(secs)+" seconds";

  serializeJson(json, buf); 
  server.send(200, "application/json", buf);       //Response to the HTTP request
}


void setup()
{
    // start Serial port
    Serial.begin(115200);
    Serial.println("\nDomEspHelper, compile date "+String(compile_date));

    // Handle Wifi connection by wifi manager
    WiFiManager wifiManager;
    wifiManager.setHostname(host);
    wifiManager.autoConnect("Thermostat");

    if (MDNS.begin(host)) {
      Serial.println("MDNS responder started");
    }
    // Log IP adress
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());  //Print the local IP to access the server

    // Register commands on webserver
    server.on("/ResetWifiCredentials", handleResetWifiCredentials);
    server.on("/GetSensors",handleGetSensors);
    server.on("/info", handleGetInfo);
    server.on("/command", handleCommand);   //Associate the handler function to the path

    // Initialize OTA
    // Port defaults to 8266
    // ArduinoOTA.setPort(8266);
  
    // Hostname defaults to esp8266-[ChipID]
    ArduinoOTA.setHostname(host);
  
    // No authentication by default
    ArduinoOTA.setPassword(host);
  
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
    MQTT.setServer(mqttserver, mqttport); // server details
    MQTT.setBufferSize(512); // discovery messages are longer than default max buffersize(!)
    MQTT.setCallback(MQTTcallback); // listen to callbacks
}

void handleOpenTherm()
{
  
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

          // Check if we have to send to MQTT
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
            if (enableCentralHeating!=mqtt_enable_CentralHeating) // value changed
            {
              UpdateMQTTSwitch(EnableCentralHeating_Name,enableCentralHeating);
              mqtt_enable_CentralHeating=enableCentralHeating;
            }
            if (enableCooling!=mqtt_enable_Cooling) // value changed
            {
              UpdateMQTTSwitch(EnableCooling_Name,enableCooling);
              mqtt_enable_Cooling=enableCooling;
            }
            if (enableHotWater!=mqtt_enable_HotWater) // value changed
            {
              UpdateMQTTSwitch(EnableHotWater_Name,enableHotWater);
              mqtt_enable_HotWater=enableHotWater;
            }
            if (Cooling!=mqtt_Cooling){ // value changed
              UpdateMQTTSwitch(CoolingActive_Name,Cooling);
              mqtt_Cooling=Cooling;
            }
            if (CentralHeating!=mqtt_CentralHeating){ // value changed
              if (CentralHeating) {
                UpdateMQTTDimmer(CentralHeatingActive_Name,CentralHeating,modulation);
              } else {
                UpdateMQTTDimmer(CentralHeatingActive_Name,CentralHeating,0);
              }
              mqtt_CentralHeating=CentralHeating;
            }
            if (HotWater!=mqtt_HotWater){ // value changed
              if (HotWater) {
                UpdateMQTTDimmer(HotWaterActive_Name,HotWater,modulation);
              } else {
                UpdateMQTTDimmer(HotWaterActive_Name,HotWater,0);
              }
              mqtt_HotWater=HotWater;
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

      // check if we have to send MQTT message
      if (MQTT.connected()) {
        if ((boiler_SetPoint-mqtt_boiler_setpoint)>=0.1 or (boiler_SetPoint-mqtt_boiler_setpoint)<=-0.1){ // value changed
          UpdateMQTTSetpoint(Boiler_Setpoint_Name,boiler_SetPoint);
          mqtt_boiler_setpoint=boiler_SetPoint;
        }
      }

      OpenThermCommand = SetDHWTemp;
      break;
    }

    case SetDHWTemp:
    {
      ot.setDHWSetpoint(dhw_SetPoint);

      // check if we have to send MQTT message
      if (MQTT.connected()) {
        if ((dhw_SetPoint-mqtt_dhw_setpoint)>=0.1 or (dhw_SetPoint-mqtt_dhw_setpoint)<=-0.1){ // value changed
          UpdateMQTTSetpoint(DHW_Setpoint_Name,dhw_SetPoint);
          mqtt_dhw_setpoint=dhw_SetPoint;
        }
      }

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
          mqtt_boiler_Temperature=boiler_Temperature;
        }
      }

      OpenThermCommand = GetDHWTemp;
      break;
    }

    case GetDHWTemp:
    {
      dhw_Temperature = ot.getDHWTemperature();
      OpenThermCommand = GetReturnTemp;
      break;
    }
      
    case GetReturnTemp:
    {
      return_Temperature = ot.getReturnTemperature();
      OpenThermCommand = GetOutsideTemp;
      break;
    }
      
    case GetOutsideTemp:
    {
      outside_Temperature = ot.getOutsideTemperature();
      
      // Check if we have to send to MQTT
      if (MQTT.connected()) {
        float delta = mqtt_outside_Temperature-outside_Temperature;
        if (delta<-0.1 or delta>0.1){ // value changed
          UpdateMQTTTemperatureSensor(Outside_Temperature_Name,outside_Temperature);
          mqtt_outside_Temperature=outside_Temperature;
        }
      }

      OpenThermCommand = GetModulation;
      break;
    }
      
    case GetModulation:
    {
      modulation = ot.getModulation();

      // Check if we have to send to MQTT
      if (MQTT.connected()) {
        if (modulation!=mqtt_modulation){ // value changed
          UpdateMQTTPercentageSensor(Modulation_Name,modulation);
          if (CentralHeating) {
            UpdateMQTTDimmer(CentralHeatingActive_Name,CentralHeating,modulation);
          }
          if (HotWater) {
            UpdateMQTTDimmer(HotWaterActive_Name,HotWater,modulation);
          }
          mqtt_modulation=modulation;
        }
      }
      
      OpenThermCommand = GetPressure;
      break;
    }
      
    case GetPressure: 
    {
      pressure = ot.getPressure();
      OpenThermCommand = GetFlowRate;
      break;
    }
 
    case GetFlowRate:
    {
      flowrate = ot.getDHWFlowrate();
      OpenThermCommand = GetFaultCode;
      break;
    }

    case GetFaultCode:
    {
      FaultCode = ot.getFault();
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
  return String(mqttautodiscoverytopic)+String("/light/")+String(DeviceName)+String("/set");
}

String SetpointCommandTopic(const char* DeviceName){
  return String(mqttautodiscoverytopic)+"/climate/"+String(DeviceName)+"/cmd_temp";
}

void MQTTcallback(char* topic, byte* payload, unsigned int length) {
  // get vars from callback
  String topicstr=String(topic);
  char payloadstr[256];
  strncpy(payloadstr,(char *)payload,length);
  payloadstr[length]='\0';

  // decode payload
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, payloadstr);


  if (error) {
    MQTT.publish("log/topic",topicstr.c_str());
    MQTT.publish("log/payload",payloadstr);
    MQTT.publish("log/length",String(length).c_str());
    MQTT.publish("log/error","Deserialisation failed");
  } else {
    if (topicstr.equals(CommandTopic(EnableHotWater_Name))) {
      // we have a match
      if (String(doc["state"]).equals("ON")){
        enableHotWater=true;
      } else if (String(doc["state"]).equals("OFF")) {
        enableHotWater=false;
      }
    } else if (topicstr.equals(CommandTopic(EnableCooling_Name))) {
      // we have a match
      if (String(doc["state"]).equals("ON")){
        enableCooling=true;
      } else if (String(doc["state"]).equals("OFF")) {
        enableCooling=false;
      } 
    } else if (topicstr.equals(CommandTopic(EnableCentralHeating_Name))) {
      // we have a match
      if (String(doc["state"]).equals("ON")){
        enableCentralHeating=true;
      } else if (String(doc["state"]).equals("OFF")) {
        enableCentralHeating=false;
      }
    } else if (topicstr.equals(SetpointCommandTopic(Boiler_Setpoint_Name))) {
      // we have a match, log what we see..
      boiler_SetPoint=String(payloadstr).toFloat();
    } else if (topicstr.equals(SetpointCommandTopic(DHW_Setpoint_Name))) {
      // we have a match, log what we see..
      dhw_SetPoint=String(payloadstr).toFloat();
    } else {
      MQTT.publish("log/topic",topicstr.c_str());
      MQTT.publish("log/payload",payloadstr);
      MQTT.publish("log/length",String(length).c_str());
      MQTT.publish("log/command","unknown topic");
    }
  }
}

void reconnect()
{
  if (usemqtt) {
    if (!MQTT.connected()) {
      Serial.print("Attempting MQTT connection...");
      if (MQTT.connect(host, mqttuser, mqttpass)) {
        Serial.println("Connect succeeded");
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
  StaticJsonDocument<512> json;

  // Construct JSON config message
  json["name"] = uniquename;
  json["unique_id"] = uniquename;
  json["cmd_t"] = String(mqttautodiscoverytopic)+"/light/"+String(uniquename)+"/set";
  json["stat_t"] = String(mqttautodiscoverytopic)+"/light/"+String(uniquename)+"/state";
  json["schema"] = "json";
  json["brightness"] = true;
  char conf[512];
  serializeJson(json, conf);  // conf now contains the json

  // Publish config message
  MQTT.publish((String(mqttautodiscoverytopic)+"/light/"+String(uniquename)+"/config").c_str(),conf,true);

}

void UpdateMQTTDimmer(const char* uniquename, bool Value, float Mod)
{
  Serial.println("UpdateMQTTDimmer");
  StaticJsonDocument<512> json;

  // Construct JSON config message
  json["state"]=Value ? "ON" : "OFF";
  json["brightness"]=int(Mod*255/100);
  char state[128];
  serializeJson(json, state);  // state now contains the json

  // publish state message
  MQTT.publish((String(mqttautodiscoverytopic)+"/light/"+String(uniquename)+"/state").c_str(),state,true);
}

void PublishMQTTSwitch(const char* uniquename, bool controllable)
{
  Serial.println("PublishMQTTSwitch");
  StaticJsonDocument<512> json;

  // Construct JSON config message
  json["name"] = uniquename;
  json["unique_id"] = uniquename;
  json["cmd_t"] = String(mqttautodiscoverytopic)+"/light/"+String(uniquename)+"/set";
  json["stat_t"] = String(mqttautodiscoverytopic)+"/light/"+String(uniquename)+"/state";
  char conf[512];
  serializeJson(json, conf);  // conf now contains the json

  // Publish config message
  MQTT.publish((String(mqttautodiscoverytopic)+"/light/"+String(uniquename)+"/config").c_str(),conf,true);

  // subscribe if need to listen to commands
  if (controllable) {
    MQTT.subscribe((String(mqttautodiscoverytopic)+"/light/"+String(uniquename)+"/set").c_str());
  }
}

void UpdateMQTTSwitch(const char* uniquename, bool Value)
{
  Serial.println("UpdateMQTTSwitch");
  // publish state message
  MQTT.publish((String(mqttautodiscoverytopic)+"/light/"+String(uniquename)+"/state").c_str(),Value?"ON":"OFF",true);
}

void PublishMQTTTemperatureSensor(const char* uniquename)
{
  Serial.println("PublishMQTTTemperatureSensor");
  StaticJsonDocument<512> json;

  // Create message
  char conf[512];
  json["value_template"] =  "{{ value_json.value }}";
  json["device_class"] = "temperature";
  json["unit_of_measurement"] = "°C";
  json["state_topic"] = String(mqttautodiscoverytopic)+"/sensor/"+String(uniquename)+"/state";
  json["json_attributes_topic"] = String(mqttautodiscoverytopic)+"/sensor/"+String(uniquename)+"/state";
  json["name"] = uniquename;
  json["unique_id"] = uniquename;
  serializeJson(json, conf);  // buf now contains the json 

  // publsh the Message
  MQTT.publish((String(mqttautodiscoverytopic)+"/sensor/"+String(uniquename)+"/config").c_str(),conf,true);
}

void UpdateMQTTTemperatureSensor(const char* uniquename, float temperature)
{
  Serial.println("UpdateMQTTTemperatureSensor");
  char charVal[10];
  dtostrf(temperature,4,1,charVal); 
  MQTT.publish((String(mqttautodiscoverytopic)+"/sensor/"+String(uniquename)+"/state").c_str(),charVal,true);
}

void PublishMQTTPercentageSensor(const char* uniquename)
{
  Serial.println("PublishMQTTPercentageSensor");
  StaticJsonDocument<512> json;

  // Create message
  char conf[512];
  json["value_template"] =  "{{ value_json.value }}";
  json["device_class"] = "None";
  json["unit_of_measurement"] = "%";
  json["state_topic"] = String(mqttautodiscoverytopic)+"/sensor/"+String(uniquename)+"/state";
  json["json_attributes_topic"] = String(mqttautodiscoverytopic)+"/sensor/"+String(uniquename)+"/state";
  json["name"] = uniquename;
  json["unique_id"] = uniquename;
  serializeJson(json, conf);  // buf now contains the json 

  // publsh the Message
  MQTT.publish((String(mqttautodiscoverytopic)+"/sensor/"+String(uniquename)+"/config").c_str(),conf,true);
}

void UpdateMQTTPercentageSensor(const char* uniquename, float percentage)
{
  Serial.println("UpdateMQTTPercentageSensor");
  char charVal[10];
  dtostrf(percentage,4,1,charVal); 
  MQTT.publish((String(mqttautodiscoverytopic)+"/sensor/"+String(uniquename)+"/state").c_str(),charVal,true);
}


void PublishMQTTSetpoint(const char* uniquename)
{
  Serial.println("PublishMQTTSetpoint");
  StaticJsonDocument<512> json;

  // Create message
  char conf[512];
  json["name"] = uniquename;
  json["unique_id"] = uniquename;
  json["temp_cmd_t"] = String(mqttautodiscoverytopic)+"/climate/"+String(uniquename)+"/cmd_temp";
  json["temp_stat_t"] = String(mqttautodiscoverytopic)+"/climate/"+String(uniquename)+"/state";
  json["temp_stat_tpl"] = "{{value_json.seltemp}}";
  serializeJson(json, conf);  // buf now contains the json 

  // publsh the Message
  MQTT.publish((String(mqttautodiscoverytopic)+"/climate/"+String(uniquename)+"/config").c_str(),conf,true);
  MQTT.subscribe((String(mqttautodiscoverytopic)+"/climate/"+String(uniquename)+"/cmd_temp").c_str());
}

void UpdateMQTTSetpoint(const char* uniquename, float temperature)
{
  Serial.println("UpdateMQTTSetpoint");
  char charVal[10];
  dtostrf(temperature,4,1,charVal); 
  MQTT.publish((String(mqttautodiscoverytopic)+"/climate/"+String(uniquename)+"/state").c_str(),charVal,true);
}




void PublishAllMQTTSensors()
{
  Serial.println("PublishAllMQTTSensors()");
  // Sensors
  PublishMQTTTemperatureSensor(Boiler_Temperature_Name);
  PublishMQTTTemperatureSensor(Thermostat_Temperature_Name);
  PublishMQTTTemperatureSensor(Outside_Temperature_Name);
  PublishMQTTPercentageSensor(Modulation_Name);

  // On/off sensors telling state
  PublishMQTTSwitch(FlameActive_Name,false);
  PublishMQTTSwitch(FaultActive_Name,false);
  PublishMQTTSwitch(DiagnosticActive_Name,false);
  PublishMQTTSwitch(CoolingActive_Name,false);
  PublishMQTTDimmer(CentralHeatingActive_Name);
  PublishMQTTDimmer(HotWaterActive_Name);
  t_last_mqtt_discovery=millis();

  // Switches to control the boiler
  PublishMQTTSwitch(EnableCentralHeating_Name,true);
  PublishMQTTSwitch(EnableCooling_Name,true);
  PublishMQTTSwitch(EnableHotWater_Name,true);

  // Publish setpoints
  PublishMQTTSetpoint(Boiler_Setpoint_Name);
  PublishMQTTSetpoint(DHW_Setpoint_Name);
}

void loop()
{
  // handle OTA
  ArduinoOTA.handle();

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
      
      if (millis()-t_last_command>domoticzTimeoutInMillis) {
          // Domoticz was not there for a long time, so do nothing
          Serial.print("."); // Just print a dot, so we can see the software in still running
          digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off, to indicate we lost connection

          // Switch off Heating since there is no domoticz to control it
          enableCentralHeating=false;
          boiler_SetPoint=0;
      } else {
          digitalWrite(LED_BUILTIN, LOW);    // turn the LED on , to indicate we have connection
      }

      // handle MDNS
      MDNS.update();
    }

    // handle openthem commands
    handleOpenTherm();

    //Handle incoming webrequests
    server.handleClient();

    // handle MQTT
    MQTT.loop();

  }
} 
