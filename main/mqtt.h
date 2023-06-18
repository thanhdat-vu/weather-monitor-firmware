#ifndef __MQTT_H__
#define __MQTT_H__

void mqtt_app_start();
void mqtt_publish_data_to_topic(char *topic, const char *data);

#endif