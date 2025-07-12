# TODO
- logging of the loop when in climate mode (prevent settings dots, switching led, etc...)
- find out weird boiler mode when network maintenance (router was not available and heater started heating at 70)


# Use mqtt as input for external temperature (when in weather dependent mode)

## create outsideTemperatureTopic
- create global var outsideTemperatureTopic
- saveconfig
- loadconfig
- publishmqttstring
- create global var mqtt outside temperaturetopic
- process incoming values
- publishtomqtt if outsidetemperaturetopic changed
- show outsidetemperaturetopic in gui
- send new outsidetemperaturetopic from gui
- use mqttoutside temperature as outsidetemperature source if configured (instead of attached sensor)



