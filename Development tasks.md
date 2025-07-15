# TODO
- logging of the loop when in climate mode (prevent settings dots, switching led, etc...)
- find out weird boiler mode when network maintenance (router was not available and heater started heating at 70)


# Use mqtt as input for external temperature (when in weather dependent mode)

String mqtt_mqttoutsidetemptopic="xyzxyz"; --> contains the the last known value of incoming topic of the outside temperature
String mqttoutsidetemptopic = "";                        // MQTT Topic which contains the current outside temperature


## create outsideTemperatureTopic
- create global var outsideTemperatureTopic --> Done
- saveconfig --> Done
- loadconfig --> Done
- publishmqttstring (and subscribe to state topic) --> Done
- create global var mqtt outside temperaturetopic --> Done
- process incoming values --> Done
- HandleGetConfig/SaveConfig -->Done
- ReceiveOutsideTemperature --> Done
- publishtomqtt if outsidetemperaturetopic changed --> Done
- subscribto to state topic --> Done
- mqttcallback op state topic --> Done
- unsubscribe when topic removed --> Done
- send new outsidetemperaturetopic from gui --> Done
- use mqttoutside temperature as outsidetemperature source if configured (instead of attached sensor) --> Done
- show outsidetemperaturetopic in gui --> Done
- update documentation --> Done

- rename mqtt temperature to mqtt reference room temperature
- change mqtttemptopic to mqttinsidetemptopic



