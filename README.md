# esp_domoticz_opentherm_handler
custom made EPS8266 firmware for OpenTherm Adapter (http://ihormelnyk.com/opentherm_adapter). Connects to Home Assistant and Domoticz using MQTT autodiscovery, but can also be controled with HTTP or the builtin UI

## Functionality
Basically a HTTP and MQTT wrapper around https://github.com/ihormelnyk/opentherm_library:
- The firmware sets up a connection with the boiler
- And then boiler can be controlled using HTTP, MQTT or the builtin UI (http://domesphelper or http://IP_adress_of_device)
- Autodiscovery supported, so easy integration for Domoticz and/or Home Assistant
- Made to work perfectly for domoticz weather dependent heating plugin (https://github.com/akamming/Domoticz_Thermostate_Plugin)

### Supported HTTP commands:
Controlling the boiler commands:
- http://domesphelper/GetSensors will return a JSON formatted status of all sensors
- http://domesphelper/command allows you to control the boiler (response also gives JSON formatted status of alle sensors):
    - HotWater=<on|off>  will enable or disable DHT
    - CentralHeating=<on|off> will enable or disable Central Heating
    - Cooling=<on|off> will enable or disable heating
    - BoilerTemperature=<desired temperature> will set the setpoint for the boiler temperature
    - DHWTemperature=<desired temperature> will set the setpoint for the Hot Water temperature
    - e.g. http://domesphelper/command?Hotwater=on&BoilerTemperature=50 will enable hot water and set boiler temperature to 50. The other steering vars remain unchanged
NOTE: The command should be repeated every 10 seconds, otherwise the Boiler will switch off heating/cooling automatically

Managing the device (are used by the UI):
- http://domesphelper/info gives a lot of technical info on the device
- http://domesphelper/getconfig retrieves the configuration file (e.g. with MQTT settings)
- http://domesphelper/saveconfig stores the configuration file (e.g. with MQTT settings) and reboot
- http://domesphelper/removeconfig deleters the config
- http://domesphelper/ResetWifiCredentials will clear wifi credentials and reboots the device, making the wifimanager portal to re-appear so you can add new WIFI connection settings
- http://domesphelper/reset reboots the device

## support MQTT commands
The following device are created using MQTT autodiscovery in domoticz and home assistant
- An EnableHotWater device: Set to ON if your heating/cooling system should produce warm water when needed
- An EnableCooling device: Set to ON if your heating/cooling system start cooling when needed
- An EnableCentralHeating device: Set to ON if your heating/cooling system 
- A Boiler setpoint device: Set the temperature to which you want the heating/cooling system to heat or cool
- A HotWater Setpoint device: (if supported by your boiler): The to be temperature of the hot water reserve in your heating/cooling system
- Several sensors containing the state of the heating/cooling system

NOTE: The command should be repeated every 10 seconds, otherwise the Boiler will switch off heating/cooling automatically

## Installation
Installation is simple, but for newbies i esp8266 here's the detailed explanation:
- Connect your OpenTherm adapter to the Wemos D1 mini and the boiler, according to http://ihormelnyk.com/opentherm_adapter. 
- Connect your Wemos D1 to the USB of your laptop/pc
- A new com port should now be available on your system. If not, install CH340 drivers (https://www.wemos.cc/en/latest/ch340_driver.html) 
- Install Arduino IDE (https://www.arduino.cc/en/software)
- Install Arduino-esp8266fs-plugin (https://github.com/esp8266/arduino-esp8266fs-plugin)
- Within Arduino IDE, 
    - Add support of your ESP8266 board by adding https://arduino.esp8266.com/stable/package_esp8266com_index.json  in the Arduino board management settings (menu File, Settings, "more boardmanager urls")
    - Install the following libraries if they are not present on your system (menu Tools / Manage Libraries in Arduino IDE): 
        - WiFiManager by Tzapu
        - OpenTherm by Ihor Melnyk
        - ArduinoJson (be Benoit Blanchon)
        - DallasTemperature (by Miles Burton)
        - OneWire (by Paul Stoffregen)
        - PubSubClient (by Nick O'Leary)
    - do a "git clone https://github.com/akamming/esp_domoticz_opentherm_handler"
    - using arduino open the .ino file in the cloned dir
    - OPTIONAL: adjust config.h for your default config settings. 
    - Open the board manager (menu Tools / Board / Board Manager)
    - Search for ESP8266 and click install, then close
    - Select your Board type, "LOLIN(WEMOS) D1 R2 & mini" if you use a Wemos D1 but in principle any ESP8266 board should work (menu Tools/Board)
    - Select the com Port to which your wemos is connected (menu Tools / Port)
    - Upload the firmware to your Wemos (menu Sketch / Upload)
    - Upload the data to your Wemos (menu Tools / ESP8266 Sketch Data Upload)

 ## Configuration
 ### Wifi config 
 - For connecting to wifi:
      - connect with a device to WiFi Access Point "Thermostat"
      - browse to http://192.168.4.1
      - follow instructions to connect the ESP to correct WiFi network

 ### Other Settings & Controlling heating/cooling manually
- (After you've setup the wifi connection, see above) 
- Enter http://domesphelper.local, http://domesphelper or http://domesphelper.home  (whatever your local domain is in your network) in your browser  
- Configure at least MQTT settings if you want to be able to control using Domoticz or Home Assistant (in that case also enable MQTT autodiscovery in Domoticz and/or Home Assistant)
- or use this link to see the value of the sensors and/or control the device using this UI (in the latter: Hit the "take control button". Warning: MQTT commands won't work as long as this webpage is still active, so make sure to reset the take control switch or close the webpage again)  
  
