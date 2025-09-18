
#include <stdio.h>
#include <mosquitto.h>

#include "mqtt.h"

// --- MQTT Default Configuration ---
const char *MQTT_BROKER_HOST = "broker.hivemq.com"; // Replace with your broker
const int MQTT_BROKER_PORT = 1883;
const char *MQTT_CLIENT_ID = "STT_Client";
const char *MQTT_USER_NAME = "sage";
const char *MQTT_PASSWORD = "Sage@2025!";
const char *MQTT_TOPIC_PARTITION_0 = "mppt/partition0";
const char *MQTT_TOPIC_PARTITION_1 = "mppt/partition1";
const char *MQTT_TOPIC_WEATHERSTATION = "weatherstation/data";

// --- MQTT Connect Callback ---
void mqtt_connect_callback(struct mosquitto *mosq, void *userdata, int result) {

    if (!result) {
        printf("Conexão ao broker MQTT bem-sucedida!\n");
    } else {
        fprintf(stderr, "Erro ao conectar ao broker: %s\n", mosquitto_connack_string(result));
    }
}

// --- Function to Initialize Mosquitto ---
int mqtt_init(struct mosquitto **mosq, char *clientID) {

    mosquitto_lib_init();
    *mosq = mosquitto_new(clientID, true, NULL);
    if (!*mosq) {
        fprintf(stderr, "mosquitto_new failed\n");
        mosquitto_lib_cleanup();
        return 1;
    }
    mosquitto_connect_callback_set(*mosq, mqtt_connect_callback);
    return 0; // Success
}
 
int InitMQTT (struct mosquitto **mosq, char *mqttServerAddr, char *clientID, int clientPort, char *mqtt_username, char *mqtt_password) {

    int rc; 

    // Initialize Mosquitto
    if (mqtt_init(mosq, clientID) != 0) {
        fprintf(stderr, "MQTT initialization failed\n");
        return 1;
    }

    rc = mosquitto_username_pw_set(*mosq, mqtt_username, mqtt_password);
    if (rc != MOSQ_ERR_SUCCESS) {
        printf("Mosquitto: Erro ao definir as credenciais: %s\n", mosquitto_strerror(rc));
        return 1;
    }

    // Connect to the broker
    rc = mosquitto_connect(*mosq, mqttServerAddr, clientPort, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
        printf("mosquitto_connect failed: %s\n", mosquitto_strerror(rc));
        mosquitto_destroy(*mosq);
        mosquitto_lib_cleanup();
        return 1;
    }

    return 0; // Success

} // InitMQTT

void CloseMQTT (struct mosquitto *mosq) {

    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();

} // CloseMQTT