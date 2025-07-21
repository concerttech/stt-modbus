/////////////////////////
#ifndef MQTT_HEADER_H
#define MQTT_HEADER_H

// --- MQTT Configuration ---
extern const char *MQTT_BROKER_HOST; // Replace with your broker
extern const int MQTT_BROKER_PORT;
extern const char *MQTT_CLIENT_ID;
extern const char *MQTT_TOPIC_PARTITION_0;
extern const char *MQTT_TOPIC_PARTITION_1;
extern const char *MQTT_TOPIC_WEATHERSTATION;

/* Protótipo das funções exportadas */
// Inicialização da conexão
int InitMQTT (struct mosquitto **mosq, char *mqttServerAddr, char *clientID, int clientPort);
// Fecha a conexão
void CloseMQTT (struct mosquitto *mosq);

#endif // MQTT_HEADER_H