Home Assistant Example:
![hadashboard](https://github.com/akamming/esp_domoticz_opentherm_handler/blob/master/ha%20dashboard%20example.png)
(see sample YAML on the bottom of this document)

## Functionality
Basically a HTTP and MQTT wrapper around https://github.com/ihormelnyk/opentherm_library:
- The firmware sets up a connection with the boiler
- The boiler can be controlled using the builtin thermostat mode via HTTP, MQTT or the builtin UI (http://domesphelper or http://IP_adress_of_device)
- Autodiscovery supported, so easy integration for Domoticz and/or Home Assistant
- Made to work perfectly for domoticz weather dependent heating plugin (https://github.com/akamming/Domoticz_Thermostate_Plugin), 
- The thermostat functionality is included in the firmware itself, See thermostat mode  

### WiFi RSSI Monitoring
- The device monitors the WiFi signal strength (RSSI) and publishes it as a sensor to MQTT for Home Assistant/Domoticz integration.
- RSSI is logged every loop iteration and updated periodically via MQTT with dBm unit.
- Helps in diagnosing WiFi connectivity issues.

### NTP Time Synchronization
- The device synchronizes time using NTP (Network Time Protocol) for accurate logging and scheduling.
- Time is updated periodically and used for timestamped debug/info messages.
- NTP updates are performed only when WiFi is connected to avoid blocking.

### OTA (Over-the-Air) Updates
- Firmware can be updated wirelessly via OTA without needing physical USB connection.
- Configure the device IP and optional password in `platformio.ini` for secure updates.
- OTA updates preserve the filesystem (LittleFS) but may require re-uploading config if needed.

### MQTT Reliability Improvements
- Connection checks ensure MQTT functions only run when connected, preventing errors.
- Batch sensor publishing (one per heartbeat) prevents broker overload during discovery.
- Detailed logging for WiFi/MQTT connections, PID values, and sensor publishing.
- Keep-alive interval optimized (30 seconds) for stable connections.
- Subscriptions to temperature topics happen first during discovery.

### Frost Protection
- When thermostat mode is off or in holiday mode, heating activates if room temperature drops below the Frost Protection Setpoint.
- Prevents pipes from freezing by maintaining minimum temperature.
- Uses PID control for gradual heating.

### PID Controller
- Implements a Proportional-Integral-Derivative (PID) controller for precise temperature regulation.
- Parameters KP, KI, KD can be adjusted via MQTT or web interface.
- PID is used in thermostat mode for maintaining setpoint temperature.
- Includes safeguards to prevent overshooting min/max boiler temperatures.

### Weather Dependent Mode
- Adjusts boiler setpoint based on outside temperature for energy efficiency.
- Uses a linear curve between BoilerTempAtPlus20 and BoilerTempAtMinus10.
- Curvature parameter allows non-linear adjustments.
- Heating is disabled when outside temperature exceeds SwitchHeatingOffAt.
- Reference Room Compensation adjusts setpoint if room temperature deviates from climate setpoint.

### Holiday Mode
- Disables thermostat control while maintaining frost protection.
- Prevents unnecessary heating during absences.
- Can be controlled via MQTT switch or web interface.

### MQTT Autodiscovery
- Automatically creates devices in Home Assistant and Domoticz via MQTT discovery topics.
- Includes sensors, switches, numbers, selects, and climate devices.
- No manual configuration needed in HA/Domoticz after initial setup.
- Sensor publishing is batched (one sensor per heartbeat) to prevent broker overload, spreading discovery over multiple heartbeats.
- Devices include `availability_topic` for automatic online/offline detection in Home Assistant using MQTT will messages.

### Web Interface
- Built-in web UI accessible at device IP or mDNS hostname (http://domesphelper.local).
- Allows configuration of MQTT, boiler settings, PID parameters, etc.
- Supports uploading custom HTML/CSS/JS files to LittleFS filesystem.
- Provides real-time status and control of the boiler/thermostat.

### Configuration Management
- Configuration stored in JSON format on LittleFS.
- Can be backed up/downloaded via HTTP API.
- Settings include MQTT credentials, PID parameters, boiler limits, etc.
- Config is preserved across firmware updates if uploaded via OTA filesystem.

### Supported HTTP commands:
Controlling the thermostat and hot water:
- http://domesphelper/GetSensors will return a JSON formatted status of all sensors
- http://domesphelper/command allows you to control the thermostat and hot water (response also gives JSON formatted status of all sensors):
    - HotWater=<on|off> will enable or disable hot water production
    - DHWTemperature=<desired temperature> will set the setpoint for the hot water temperature
    - climateMode=<off|heat|cool|auto> will set the thermostat mode
    - climateSetpoint=<desired temperature> will set the target temperature for thermostat mode
    - weatherDependentMode=<on|off> will enable or disable weather dependent heating
    - holidayMode=<on|off> will enable or disable holiday mode (frost protection only)
    - e.g. http://domesphelper/command?HotWater=on&DHWTemperature=60&climateMode=heat&climateSetpoint=20 will enable hot water at 60°C and set thermostat to heat mode with setpoint 20°C

Managing the device (are used by the UI):
- http://domesphelper/info gives a lot of technical info on the device
- http://domesphelper/getconfig retrieves the configuration file (e.g. with MQTT settings)
- http://domesphelper/saveconfig stores the configuration file (e.g. with MQTT settings) and reboot
- http://domesphelper/removeconfig deletes the config
- http://domesphelper/ResetWifiCredentials will clear wifi credentials and reboots the device, making the wifimanager portal to re-appear so you can add new WIFI connection settings
- http://domesphelper/reset reboots the device

## support MQTT commands
The following devices are created using MQTT autodiscovery in Domoticz and Home Assistant for thermostat and hot water control:
- A Climate device: Controls the thermostat mode and setpoint
- An EnableHotWater device: Set to ON if your heating system should produce warm water when needed
- A HotWater Setpoint device: (if supported by your boiler) The desired temperature of the hot water reserve in your heating system
- A Holiday Mode switch: Prevents heating when in thermostat mode (only frost protection)
- A Weather Dependent Mode switch: When in thermostat mode, decides whether to use the PID regulator or the Weather Dependent mode, where the boiler setpoint is derived from the outside temperature
- Several sensors containing the state of the heating system
- WiFi RSSI sensor: Monitors signal strength in dBm
- Log sensor: Receives debug and info messages from the device
- IP Address sensor: Shows current device IP

### Thermostat mode
The device operates primarily in thermostat mode for controlling the boiler and hot water. Direct boiler commands have been removed to ensure safe and efficient operation through the thermostat logic.

Additional devices:
- A Climate setpoint Device (main control for thermostat mode)
- An EnableHotWater device and HotWater Setpoint device for hot water control
- A Holiday Mode switch: Prevents heating when in thermostat mode (only frost protection)
- A Weather Dependent Mode switch: When in thermostat mode, decides whether to use the PID regulator or the Weather Dependent mode, where the boiler setpoint is derived from the outside temperature

Thermostat mode includes:
- PID-based temperature control for precise regulation
- Frost protection when thermostat is off
- Weather dependent heating based on outside temperature
- Holiday mode for absence periods
- Automatic switching between heating/cooling/auto modes
- Hot water production control 

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
- MQTT authentication: Enable if your broker requires username/password
- MQTT persistence: Retain messages for HA/Domoticz
- Debug to MQTT: Send debug logs to MQTT
  - **Warning:** Enabling debug logging can make the application unstable due to the large number of log messages. It is only intended for debugging purposes and should not be used in normal operation.
- Info to MQTT: Send info logs to MQTT

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

### Climate settings:
- Climate Mode: Off, Heat, Cool, Auto
- Climate Setpoint: Target temperature for thermostat mode
- Weather Dependent Mode: Enable/disable weather-based setpoint adjustment
- Holiday Mode: Enable/disable holiday mode (frost protection only)


## Troubleshooting
- **WiFi Connection Issues**: Check RSSI sensor in HA/Domoticz. If low, move device closer to router or use WiFi extender. Connection losses are logged as Info messages.
- **MQTT Connection Problems**: Verify broker settings, credentials, and network connectivity. Check MQTT state in device info. Connection losses and restorations are logged with RSSI details.
- **Boiler Not Responding**: Ensure OpenTherm adapter is correctly connected to pins 4 and 5. Check OpenTherm status in device info.
- **OTA Upload Fails**: Ensure device is on same network, IP is correct in platformio.ini, and firewall allows connections.
- **Time Sync Issues**: Check NTP server accessibility and WiFi connection during sync attempts.
- **PID Tuning**: If temperature oscillates, adjust KI down or KD up. Monitor logs for PID values on every heartbeat.
- **Weather Dependent Mode**: Requires outside temperature sensor on boiler. Check OT Outside Temperature sensor.
- **MQTT Discovery Overload**: If broker disconnects during discovery, check logs for batch publishing status. Discovery spreads over ~25 minutes (one sensor per 30 seconds).

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

---

*This README was partially generated using AI.*


