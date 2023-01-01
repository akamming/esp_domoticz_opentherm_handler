# esp_domoticz_opentherm_handler
custom made EPS8266 firmware for OpenTherm Adapter (http://ihormelnyk.com/opentherm_adapter) for domoticz weather dependent heating plugin (https://github.com/akamming/Domoticz_Thermostate_Plugin)

Please look at https://github.com/akamming/Domoticz_Thermostate_Plugin on how to use this firmware

## Functionality
Basically a HTTP wrapper around https://github.com/ihormelnyk/opentherm_library:
- The firmware sets up a connection with the boiler
- The firmware remembers the settings for "Heating enabled", "hot water enabled", "cooling enabled", "DHW setpoint" and "Boiler setpoint"
- And will send these settings to the boiler in a continuous loop
- The settings can be changed by giving HTTP commands
- In the reply of every command, the current value of the settings, along with several sensors in the boiler which can be read by the opentherm library are returned

### Installation
- Connect your OpenTherm adapter to the Wemos D1 mini, according to http://ihormelnyk.com/opentherm_adapter. 
- Connect your Wemos D1 to the USB of your laptop/pc
- A new com port should now be available on your system. If not, install CH340 drivers (just google on it and you will find them) 
- Install Arduino IDE (https://www.arduino.cc/en/software)
- Within IDE, 
    - Add the url https://arduino.esp8266.com/stable/package_esp8266com_index.json  in the settings (menu File, Settings, "more boardmanager urls")
    - Install the following libraries (menu Tools / Manage Libraries in Arduino IDE)
        -  WhareHauoraWiFiManager by Tzapu
        -  OpenTherm by Ihor Melnyk
    - do a "git clone https://github.com/akamming/esp_domoticz_opentherm_handler"
    - using arduino open the .ino file in the cloned dir
    - adjust config.h for your default config settings. 
    - Open the board manager (menu Tools / Board / Board Manager)
    - Search for ESP8266 and click install, then close
    - Select Board type "LOLIN(WEMOS) D1 R2 & mini (menu Tools/Board)
    - Select the com Port to which your wemos is connected (menu Tools / Port)
    - Upload the firmware to your Wemos (menu Sketch / Upload)

 ## Configuration
 ### Wifi config 
 - For connecting to wifi:
      - connect with a device to WiFi Access Point "Thermostat"
      - browse to http://192.168.4.1
      - follow instructions to connect the ESP to correct WiFi network
 - If you want to reset the wificredentials, either
      - make sure the wemos cannot connect to configured network, and the config portal will appear automatically
      - or force this by opening a browser and entering the URL: "http://domesphelper/ResetWifiCredentials" 

 ### Other Settings
- After Wifi connection: Enter http://domesphelper.local, http://domesphelper or http://domesphelper.home  (whatever your local domain is in your network) in your browser and configure your device. Set mqtt to enabled and enter your mqtt settings correctly to make it work with latest domoticz python plugin. 
- also enable mqtt autodiscovery on your domoticz or Home Assistant application and your devices will appear.

## Supported commands
- http://domesphelper/ResetWifiCredentials will clear wifi credentials and reset the device, making the wifimanager portal to re-appear
- http://domesphelper/GetSensors will give the status + all sensors
- http://domesphelper/command enables to give commands, where the http variables are the commands:
    - HotWater=<on|off>  will enable or disable DHT
    - CentralHeating=<on|off> will enable or disable Central Heating
    - Cooling=<on|off> will enable or disable heating
    - BoilerTemperature=<desired temperature> will set the setpoint for the boiler temperature
    - DHWTemperature=<desired temperature> will set the setpoint for the Hot Water temperature
  e.g. http://domesphelper/command?Hotwater=on&BoilerTemperature=50 will enable hot water and set boiler temperature to 50. The other steering vars remain unchanged
  
  
