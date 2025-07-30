Home Assistant Example:
![hadashboard](https://github.com/akamming/esp_domoticz_opentherm_handler/blob/master/ha%20dashboard%20example.png)
(see sample YAML on the bottom of this document)

## Functionality
Basically a HTTP and MQTT wrapper around https://github.com/ihormelnyk/opentherm_library:
- The firmware sets up a connection with the boiler
- And then boiler can be controlled using HTTP, MQTT or the builtin UI (http://domesphelper or http://IP_adress_of_device)
- Autodiscovery supported, so easy integration for Domoticz and/or Home Assistant
- Made to work perfectly for domoticz weather dependent heating plugin (https://github.com/akamming/Domoticz_Thermostate_Plugin), 
- As of release 1.0, the thermostat functionality is also included in the firmware itself, See thermostat mode  

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

### Thermostat mode
Additional 3 more devices :
- a Climate setpoint Device (when you want to use the firmware in thermostat mode)
- a Holiday Mode  switch: Prevents heating when in themostat mode (only frostprotection)
- a Weather Dependent Mde switch: When in thermostat mode decides wheter to use the PID regulator or the Weather Dependent mode, where the boiler setpoint is derived from the outside temperature 

## Installation

### Hardware
- Connect your OpenTherm adapter to the Wemos D1 mini and the boiler, according to http://ihormelnyk.com/opentherm_adapter.
- **Pin connections:**
  - OpenTherm adapter: Connect to **pins 4 and 5** on the Wemos D1 mini
  - Temperature sensor (DS18B20): Connect to **pin 14** on the Wemos D1 mini
- **Note:** If you need to use different pins, you must modify the pin definitions in the code before uploading the sketch.

### Firmware Upload (PlatformIO Recommended)

- Install [Visual Studio Code](https://code.visualstudio.com/)
- In VSCode, open the Extensions view (`Ctrl+Shift+X`), search for "PlatformIO IDE" and install it.
- Open this project folder in VSCode.
- Connect your Wemos D1 mini to your computer via USB.
- The **first time** you upload the firmware, use the COM port (USB connection):
    - Make sure the correct COM port is selected in PlatformIO (check the bottom bar or `platformio.ini`).
    - Click the "Upload" button in the PlatformIO toolbar, **or**
    - Open the VSCode terminal and run:
      ```
      pio run -e d1_mini -t upload
      ```
- After the initial upload, the device will connect to WiFi (see "First Time setup" below).
- For **subsequent updates**, you can use OTA (Over-the-Air) updates:
    - Edit `platformio.ini` if needed:
        - Set the correct IP address for OTA updates (`upload_port = <your_device_ip>`)
        - Set the OTA password if you use one (`upload_flags = --auth=<your_ota_password>`)
    - Click the "Upload" button in the PlatformIO toolbar, **or**
    - Run:
      ```
      pio run -e d1_mini -t upload
      ```
    - Make sure your device is connected to WiFi and reachable at the IP address you set.

**Supported hardware:**  
Only ESP8266-based devices are supported.  
If you use a different ESP8266 board (not Wemos D1 mini), you must change the `board` option in `platformio.ini` to match your hardware.  
See the official PlatformIO documentation for available board options:  
https://docs.platformio.org/en/latest/platforms/espressif8266.html#boards

For more information about configuring `platformio.ini`, see:  
https://docs.platformio.org/page/projectconf.html

## Uploading GUI files to the device

You can easily upload web interface files (HTML, CSS, JS, etc.) to your device using PlatformIO:

1. Place all files you want to upload in the `data` directory of your project.
2. In PlatformIO, use the **Upload File System Image** task (via the sidebar or via `Ctrl+Shift+P` > PlatformIO: Upload File System Image).
3. The files will be sent via OTA to the LittleFS filesystem on your device.

**Important on  uploading GUI files using the OTA uploader:**  
Uploading new firmware will overwrite your current configuration on the device.  
If you want to keep your configuration, you should first download your config file by navigating to [http://domesphelper.local/config.json](http://domesphelper.local/config.json) in your browser.  
Save this file in your project's `data` directory. Make sure you set the MQTT password again (this is masked for security reasons if you retrieve it from the device)  
When you upload new code (using PlatformIO), all files in the `data` directory will also be uploaded to the device, so your configuration will be restored

### First Time setup
For connecting to wifi:
- on your laptop/PC: connect to a WiFi Access Point called "Thermostat"
- browse to http://192.168.4.1
- follow instructions to connect the ESP to correct WiFi network
- After the ESP is connected, connect to your normal Wifi Access Point again 
- Enter http://domesphelper.local, http://domesphelper or http://domesphelper.home  (whatever your local domain is in your network) in your browser to connect to the device. 
- Optional (if you didn't use the OTA FS uploader) If all went correct you should now see a "File not found" error including some buttons to upload files (go to next step), on that page: Upload index.html, settings.html, index.css and favicon.ico

## Configuration
(After you've complete the installation, see above): 
- Enter http://domesphelper.local, http://domesphelper or http://domesphelper.home  (whatever your local domain is in your network) in your browser  
- See below for an explanation of the settings (make sure you configure the MQTT settings and a MQTT temperature topic if you want to be able to control the thermostat from domoticz and/or home assistant)
- you can also use this page to control the boiler or thermostat directly 
  
### MQTT settings 
- MQTT should be switched on
- MQTT temperature topics: should be either 
  - the MQTT State topic of your temperature sensor 
  - The IDX of your domoticz device (it will listen to domoticz/out. Note, this is not recommended, cause the device than has to check every domoticz device update on if it's the temperature sensor, which will make the device very slow if domoticz has many device updates!)
  - leave empty (for reference room temp it will use the internal dallastemperature sensor connected to the device, so the DS18B20 pin should be configured correctly. This is also not recommended, cause the temperature of the ESP influences the temperature sensor. For outside temperature it will use the outside temperature as reported by the boiler (if a outside temp sensor is attached))

About the mqtt temperature topic: If you want to use a domoticz sensor to be used, there are several options available, but this is the best way to do it:
- Install Mosquitto (or another MQTT broker) of you did not yet install a MQTT broker
- Setup both the "MQTT Client gateway with LAN interface" and the "MQTT Auto Discovery Client Gateway" in domoticz (only if these aren't configured yet) and for bth: Set the Adress, port (and optionally username/password) to match your MQTT server settings.   
- Set the Publish Topic setting for the MQTT Client Gateway  to "Index (with Retain)". 
- In the settings of this ESP firmware: Set the MQTT switch to on, and the MQTT temperature Topics to "domoticz/out/idx_of_your_temp_sensor

### Boiler settings:
- MinBoilerTemp & MaxBoilerTemp: The thermostat will limit the setpoint for the boiler between these 2 values
- MiminumTemp difference: Heating will only be switched on if the boiler setpoint if this amount above the reference room temperature
- Frostprotection setpoint: If thermostat switched off, heating will still be on if reference termperature below this point
- PID parameters: Leave to 30 (KP), 0.03 (KI) and 2.5(KD), unless you know what you are doing.  The software contains a regular PID controller (https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller) So with KP, KI and KD you can tweak the mechanism to your needs   

### Weather dependent mode
(weather dependent mode requires you have an external temp sensor attached to your boiler, supported by opentherm)
- BoilerTempAtPlus20 and BoilerTempAtMinus10: Basic data for the boiler firing line
- Curvature:  Can curve the boiler firing line (basically meaning the boiler setpoint will go up earlier)
- Switchheatingoffat: Disable the heating when the outside temp is above this value 
- Refroomcompensation: Can change the boiler setpoint if the reference room temperature is lower than the climate setpoint

## Controlling the Device
The device can be controlled either by the MQTT devices (see above) or by navigating to the homepage 

## Backup/Restore config

### Backing up the config
- Using a browser, navigate to http://domesphelper/config.json. 
- Copy and past the json and store it on a save position
- Alternatively you could write a script which does a curl or a wget command to get http://domesphelper/config.json

### Restoring the config

- Upload your saved config.json file on http://domesphelper/upload

## Home Assistant Configuration

You can make a very nice  heating control dashboard (see  example on top of this readme) like this when using home assistant:

### Howto
- Install the device and connect to MQTT as described above under Installation
- In Home Assistant:
  - add the MQTT integration
  - make sure your room temperature is available in MQTT, e.g. by using the automation below (paragraph sample Automation to export temperature to MQTT), but if the sensor is already on mqtt, you can ofcourse use it's native topic  
- Configure your dashboard using the dashboard editor (see sample YAML below for the dashboard on top of this page)
- ofcourse make sure in the settings of this firmware that the MQTT topic is the same as the one as your automation exports the room temperature to.. 

#### sample Automation to export temperature to MQTT

```
alias: <<name of your automation>>
description: >-
  <<description of your automation>> 
triggers:
  - trigger: state
    entity_id:
      - sensor.<<name of your room temperature sensor>>
conditions:
  - condition: template
    value_template: >-
      {{ states('<< name of your temperature sensor>>') not in ['unavailable', 'unknown', 'none'] }}
actions:
  - action: mqtt.publish
    metadata: {}
    data:
      topic: <<the topic you want to use for exporting the temperature>>
      payload: >-
        { "value": "{{states("sensor.<<name of your temperature sensor>>") }}" }
mode: single
```

#### Home Assistant Sample Dashboard Configuration YAML

```
views:
  - title: C.V.
    path: c-v
    cards:
      - square: false
        type: grid
        columns: 1
        cards:
          - type: thermostat
            entity: climate.domesphelper_climate
            features:
              - type: climate-hvac-modes
                hvac_modes:
                  - 'off'
                  - heat
                  - cool
                  - auto
          - type: entities
            entities:
              - entity: light.domesphelper_weatherdependentmode
                name: Weersfhankelijk stoken
                icon: mdi:heat-wave
              - entity: light.domesphelper_holidaymode
                icon: mdi:account-group
                name: Vakantiemodus
              - entity: >-
                  switch.werkkast_vloerverwarming_werkkast_vloerverwarming_switch
              - entity: sensor.domesphelper_outside_temperature
                name: Buitentemperatuur
      - type: entities
        entities:
          - entity: sensor.garage_pir_garage_garage_pir_garage_temperature_air
          - entity: climate.domesphelper_boiler_setpoint
            icon: mdi:heat-wave
            name: CV warmtevraag
          - entity: binary_sensor.domesphelper_flameactive
          - entity: binary_sensor.domesphelper_hotwateractive
          - entity: binary_sensor.domesphelper_centralheatingactive
          - entity: binary_sensor.domesphelper_diagnosticactive
          - entity: binary_sensor.domesphelper_frostprotectionactive
          - entity: sensor.domesphelper_modulation
          - entity: schedule.cv
          - entity: binary_sensor.domesphelper_faultactive
          - entity: sensor.domesphelper_faultcode
        title: CV Sensoren
      - type: grid
        square: false
        columns: 1
        cards:
          - type: entities
            entities:
              - entity: number.domesphelper_minimumboilertemp
              - entity: number.domesphelper_maximumboilertemp
              - entity: number.domesphelper_minimumtempdifference
              - entity: number.domesphelper_frostprotectionsetpoint
            title: Generieke Boiler Instellingen
          - type: entities
            entities:
              - number.domesphelper_kp
              - number.domesphelper_ki
              - number.domesphelper_kd
            title: PID parameters
          - type: entities
            entities:
              - number.domesphelper_boilertempatplus20
              - number.domesphelper_boilertempatminus10
              - select.domesphelper_curvature
              - number.domesphelper_switchheatingoffat
              - number.domesphelper_referenceroomcompensation
            title: Weersafhankelijke Stooklijn
```


