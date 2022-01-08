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
#include <ArduinoJson.h>

// constants
const int inPin = 4;                            // pin number for opentherm adapter connection, 2 for Arduino, 4 for ESP8266 (D2), 21 for ESP32
const int outPin = 5;                           // pin number for opentherm adapter connection, 3 for Arduino, 5 for ESP8266 (D1), 22 for ESP32
const int OneWireBus = 14;                      //Data wire is connected to 14 pin on the OpenTherm Shield
const int domoticzTimeoutInMillis = 30 * 1000;  // if no command was sent in this period, the thermostat will assume domoticz is nog longer there 
const int heartbeatTickInMillis = 1000;         // has to be max 1000, Opentherm assumes a command is sent to opentherm at least once per second
const String host = "domesphelper";             // mdns hostname
const float ThermostatTemperatureCalibration=0;  // set to a differenct value to zero is DS18B20 give a too high or low reading
const char compile_date[] = __DATE__ " " __TIME__;

enum OTCommand { SetBoilerStatus, SetBoilerTemp, SetDHWTemp, GetBoilerTemp,  GetDHWTemp, GetReturnTemp, GetOutsideTemp, GetModulation, GetPressure, GetFlowRate, GetFaultCode, GetThermostatTemp } OpenThermCommand ;

// vars to manage boiler
bool enableCentralHeating = false;
bool enableHotWater = true;
bool enableCooling = false;
float boiler_SetPoint = 0;
float dhw_SetPoint = 65;

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
  
// vars for program logic
unsigned long t_heartbeat=millis()-heartbeatTickInMillis; // last heartbeat timestamp
unsigned long t_last_command=millis()-domoticzTimeoutInMillis; // last domoticz command timestamp
bool OTAUpdateInProgress=false;

// objects to be used by program
ESP8266WebServer server(80);   //Web server object. Will be listening in port 80 (default for HTTP)
OneWire oneWire(OneWireBus);  // OneWire Bus
DallasTemperature sensors(&oneWire); // for the temp sensor 
OpenTherm ot(inPin, outPin);


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
    ArduinoOTA.setHostname(host.c_str());
  
    // No authentication by default
    ArduinoOTA.setPassword("domesphelper");
  
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
    
          // Communicate setpoints to Boiler
  
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
      OpenThermCommand = GetModulation;
      break;
    }
      
    case GetModulation:
    {
      modulation = ot.getModulation();
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
      OpenThermCommand=SetBoilerStatus;
      break;
    }

  }
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
  }
} 
