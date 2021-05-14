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
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic

// constants
const int inPin = 2;                            // pin number for opentherm adapter connection, for Arduino, 4 for ESP8266 (D2), 21 for ESP32
const int outPin = 3;                           // pin number for opentherm adapter connection, for Arduino, 5 for ESP8266 (D1), 22 for ESP32
const int domoticzTimeoutInMillis = 60 * 1000;  // if no command was send in this period, the thermostat will stop sending commands to opentherm
const int heartbeatTickInMillis = 1000;         // has to be max 1000, Opentherm assumes a command is send to opentherm at least once per second


// vars to manage boiler
bool enableCentralHeating = true;
bool enableHotWater = true;
bool enableCooling = false;
float boiler_SetPoint = 0;
float dhw_SetPoint = 0;

// return values from boiler
float dhw_Temperature = 0;
float boiler_Temperature = 0;
float return_Temperature = 0;
float modulation = 0;
float pressure = 0;
OpenThermResponseStatus responseStatus;
bool CentralHeating = false;
bool HotWater = false;
bool Cooling = false;
bool Flame = false;

// vars for program logic
unsigned long t_heartbeat=millis()-heartbeatTickInMillis; // last heartbeat timestamp
unsigned long t_last_command=millis()-domoticzTimeoutInMillis; // last domoticz command timestamp

// objects to be used by program
ESP8266WebServer server(80);   //Web server object. Will be listening in port 80 (default for HTTP)
OpenTherm ot(inPin, outPin);


void ICACHE_RAM_ATTR handleInterrupt() {
    ot.handleInterrupt();
}

void handleEnableHotWater() {
  Serial.println("Enabling hot water");
  t_last_command=millis();
  enableHotWater = true;
  SendHTTP("EnableHotWater","OK");
}

void handleDisableHotWater() {
  Serial.println("Disabling hot water");
  t_last_command=millis();
  enableHotWater = false;
  SendHTTP("DisableHotWater","OK");
}

void handleEnableCentralHeating() {
  Serial.println("Enabling Central Heating");
  t_last_command=millis();
  enableCentralHeating = true;
  SendHTTP("EnableCentralHeating","OK");
}

void handleDisableCentralHeating() {
  Serial.println("Disabling Central Heating");
  t_last_command=millis();
  enableCentralHeating = false;
  SendHTTP("DisableCentralHeating","OK");
}

void handleEnableCooling() {
  Serial.println("Enabling Cooling");
  t_last_command=millis();
  enableCooling= true;
  SendHTTP("EnableCooling","OK");
}

void handleDisableCooling() {
  Serial.println("Disabling Cooling");
  t_last_command=millis();
  enableCooling = false;
  SendHTTP("DisableCooling","OK");
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

void handleSetBoilerTemp() { //Handler

    String Statustext;

    if (server.arg("Temperature")== ""){     //Parameter not found
        Serial.println("Temperature Argument not found");
        Statustext="Temperature Argument not specified";
    }else{         
        Serial.println("Setting boiler temp to "+server.arg("Temperature"));
        boiler_SetPoint=server.arg("Temperature").toFloat();
        Statustext="OK";
        t_last_command=millis();
    }
    SendHTTP("SetBoilerTemp",Statustext);
}

void handleSetDHWTemp() { //Handler

    String Statustext;

    if (server.arg("Temperature")== ""){     //Parameter not found
        Serial.println("Temperature Argument not found");
        Statustext="Temperature Argument not specified";
    }else{         
        Serial.println("Setting dhw temp to "+server.arg("Temperature"));
        dhw_SetPoint=server.arg("Temperature").toFloat();
        Statustext="OK";
        t_last_command=millis();
    }
    SendHTTP("SetDHWTemp",Statustext);
}

void handleGetSensors() {
  SendHTTP("GetSensors","OK");
  t_last_command=millis();
}

void SendHTTP(String command, String result) {
    String message = "{\n  \"InterfaceVersion\":1,\n  \"Command\":\""+command+"\",\n  \"Result\":\""+result+"\""+getSensors()+"\n}";
    server.send(200, "text/plain", message);       //Response to the HTTP request
}

String getSensors() { //Handler

    String message;

    OpenThermResponseStatus responseStatus = ot.getLastResponseStatus();

    // Add Status
    message += ",\n  \"OpenThermStatus\":";
    if (responseStatus == OpenThermResponseStatus::SUCCESS) {
        message += "\"OK\"";
    }
    if (responseStatus == OpenThermResponseStatus::NONE) {
        message += "\"OpenTherm is not initialized\"";
    }
    else if (responseStatus == OpenThermResponseStatus::INVALID) {
        message += "\"Invalid response\"";
    }
    else if (responseStatus == OpenThermResponseStatus::TIMEOUT) {
        message += "\"Response timeout\"";
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

     // Add boiler sensors
    message +=",\n  \"BoilerTemperature\": " + (String)boiler_Temperature;
    message +=",\n  \"DhwTemperature\": " + (String(dhw_Temperature));    
    message +=",\n  \"ReturnTemperature\": " + (String(return_Temperature));
    message +=",\n  \"Modulation\": " + (String(modulation));
    message +=",\n  \"Pressure\": " + (String(pressure));

    return message;
}

void setup()
{
    // start Serial port
    Serial.begin(115200);
    Serial.println("\nStarted");

    // Handle Wifi connection by wifi manager
    WiFiManager wifiManager;
    wifiManager.autoConnect("Thermostat");

    // Log IP adress
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());  //Print the local IP to access the server

    // Register commands on webserver
    server.on("/SetBoilerTemp", handleSetBoilerTemp);
    server.on("/SetDHWTemp",handleSetDHWTemp);
    server.on("/EnableHotWater", handleEnableHotWater);
    server.on("/DisableHotWater", handleDisableHotWater);
    server.on("/EnableCentralHeating", handleEnableCentralHeating);
    server.on("/DisableCentralHeating", handleDisableCentralHeating);
    server.on("/EnableCooling", handleEnableCooling);
    server.on("/DisableCooling", handleDisableCooling);
    server.on("/ResetWifiCredentials", handleResetWifiCredentials);
    server.on("/GetSensors",handleGetSensors);

    //Start the server
    server.begin();   
    Serial.println("Opentherm Helper is waiting for commands");   

    // Start OpenTherm
    ot.begin(handleInterrupt);
}

void loop()
{
    if (millis()-t_heartbeat>heartbeatTickInMillis) {
      //reset tick time
      t_heartbeat=millis();
      if (millis()-t_last_command>domoticzTimeoutInMillis) {
          // Domoticz was not there for a long time, so do nothing
          Serial.print("."); // Just print a dot, so we can see the software in still running
      } else {
          // enable/disable heating, hotwater, heating and get status from opentherm connection and boiler (if it can be reached)
          unsigned long response = ot.setBoilerStatus(enableCentralHeating, enableHotWater, enableCooling);
          OpenThermResponseStatus responseStatus = ot.getLastResponseStatus();
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

              // read additional sensors         
              boiler_Temperature = ot.getBoilerTemperature();
              dhw_Temperature = ot.getDHWTemperature();
              return_Temperature = ot.getReturnTemperature();
              modulation = ot.getModulation();
              pressure = ot.getPressure();

              // Communicate setpoints to Boiler (if applicable)
              if (enableCentralHeating) {
                Serial.println("Sending boiler setpoint ("+String(boiler_SetPoint)+")");
                ot.setBoilerTemperature(boiler_SetPoint);
              }
              if (enableHotWater) {
                  Serial.println("Setting dhw setpoint ("+String(dhw_SetPoint)+")");
                  ot.setDHWSetpoint(dhw_SetPoint);
              }
          }
          if (responseStatus == OpenThermResponseStatus::NONE) {
              Serial.println("Opentherm Error: OpenTherm is not initialized");
          } else if (responseStatus == OpenThermResponseStatus::INVALID) {
              Serial.println("Opentherm Error: Invalid response " + String(response, HEX));
          } else if (responseStatus == OpenThermResponseStatus::TIMEOUT) {
              Serial.println("Opentherm Error: Response timeout");
          }  else {
              Serial.println("Opentherm Error: unknown error");
          }
      } 
      //Handle incoming webrequests
      server.handleClient();    
    }
} 
