/***********************************************************************************/
/**
 * @file main.cpp
 * @brief Main program body
 * 
 * This files inplements the main functionality of the modbus comunication protocol
 * for the STT project.
 * It opens the serial port, initialize the MQTT conection, comunicates with MPPT 
 * and weatherStatin devices and publish the data on the MQTT server.
 * 
*************************************************************************************/

#include <iostream>
#include <mosquitto.h>
#include <gpiod.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>

#include "modbus.h"
#include "mqtt.h"
#include "mppt.h"
#include "weatherst.h"

// Definições para comunicação serial
#define SERIAL_PORT 2           // Porta serial (1 para porta A e 2 para porta B)
#define BAUDRATE B9600          // Taxa de transmissão
#define PARITY 'N'              // Sem paridade
#define DATA_BITS 8             // 8 bits de dados
#define STOP_BITS 1             // 1 bit de parada

#define DEFAULT_UPDATE_NTERVAL  10 // 10 segundos

// Definição das portas de IO
/*----------Serial A------------*/
// SODIMM 133
#define SERIALA_DE_CHIP         3
#define SERIALA_DE_LINE         24
// SODIMM 206
#define SERIALA_H_F_CHIP        0
#define SERIALA_H_F_LINE        0
// SODIMM 208
#define SERIALA_TERM_TX_CHIP    0
#define SERIALA_TERM_TX_LINE    1
// SODIMM 210
#define SERIALA_TERM_RX_CHIP    0
#define SERIALA_TERM_RX_LINE    5
// SODIMM 212
#define SERIALA_SLR_CHIP        0
#define SERIALA_SLR_LINE        6
/*----------Serial B------------*/
// SODIMM 141
#define SERIALB_DE_CHIP         1
#define SERIALB_DE_LINE         7
// SODIMM 216
#define SERIALB_H_F_CHIP        0    
#define SERIALB_H_F_LINE        7
// SODIMM 218
#define SERIALB_TERM_TX_CHIP    0 
#define SERIALB_TERM_TX_LINE    8
// SODIMM 220
#define SERIALB_TERM_RX_CHIP    3 
#define SERIALB_TERM_RX_LINE    3
// SODIMM 222
#define SERIALB_SLR_CHIP        3
#define SERIALB_SLR_LINE        1

/*-------------------------------------------------*/
/**
  @brief Set the IO value

  OPen the chip, poen the linha, configure line as output
  set IO value. 

  @param chip_num gpiochip number
  @param line_num line number
  @param value Value to be set

  @return return the error code (0 -> NoErro)
*/
/*-------------------------------------------------*/
int Set_IO (char chip_num, char line_num, char value) {

    const char *chipStringName="gpiochip%d";
    char chipName[15];
    struct gpiod_chip *chip;
    struct gpiod_line *line;

    // Chip name
    sprintf (chipName, chipStringName, chip_num);

    // Open GPIO chip
    chip = gpiod_chip_open_by_name(chipName); 
    // Open the GPIO line
    line = gpiod_chip_get_line(chip, line_num);  
    // Open line for output
    gpiod_line_request_output(line, "GPIO Line", 0);
    // Set line value
    gpiod_line_set_value(line, value);
  
    // Realease Line and chip
    gpiod_line_release(line);
    gpiod_chip_close(chip);

    return 0;
    
} // Set_IO

/*-------------------------------------------------*/
/**
  @brief Config the IO pins for the serial RS 485/422

  Set the port as half-duplex, conect the terminator resitor.
 
  @return return the error code (0 -> NoErro)
*/
/*-------------------------------------------------*/
int ConfigIO(uint8_t porta) {

    if (porta == 1) {
        // Set as hal-duplex
        Set_IO (SERIALA_H_F_CHIP, SERIALA_H_F_LINE, 1);

        // Turn-on the terminator resistor 
        Set_IO (SERIALA_TERM_TX_CHIP, SERIALA_TERM_TX_LINE,1);
        Set_IO (SERIALA_TERM_RX_CHIP, SERIALA_TERM_RX_LINE,1);
        return 0;
    }
    else if (porta==2) {
        // Set as hal-duplex
        Set_IO (SERIALB_H_F_CHIP, SERIALB_H_F_LINE, 1);

        // Turn-on the terminator resistor 
        Set_IO (SERIALB_TERM_TX_CHIP, SERIALB_TERM_TX_LINE,1);
        Set_IO (SERIALB_TERM_RX_CHIP, SERIALB_TERM_RX_LINE,1);
        return 0;
    }
    
    return -1;
    
} // ConfigIO

/**
 *  @brief Publish the partionData0 in MQTT 
 *  @param mosq MQTT object
 *  @param Partition0Data Data to be published
 *  
 */
void publishPartition0Data(struct mosquitto *mosq, struct Partition0Data *data, char *mqttTopic) {

    char payload[1024]; // Ajuste o tamanho conforme necessário
    int payloadLength = 0;
    char topic[100];
    int i;

    // Constrói o tópico para a partição 0
    snprintf(topic, sizeof(topic), mqttTopic);

    // Prepara o payload com os dados da estrutura no formato "Nome:Valor"
    payloadLength = snprintf(payload, sizeof(payload),
        "{ "
        "\"MaxVoltage\":%u,"
        "\"RatedCurrent\":%u,"
        "\"RatedDischargeCurrent\":%u,"
        "\"ProductType\":%u,"
        "\"ProductSpecification\":\"%s\","
        "\"SoftwareVersion\":%u,"
        "\"HardwareVersion\":%u,"
        "\"ProductSerialNumber\":%u,"
        "\"DeviceAddress\":%u,"
        "\"Online\":%u"
        "}",
        data->MaxVoltage, 
        data->RatedCurrent, 
        data->RatedDischargeCurrent, 
        data->ProductType, 
        data->ProductSpecification,
        data->SoftwareVersion,
        data->HardwareVersion,
        data->ProductSerialNumber,
        data->DeviceAddress,
        data->onLine
    );

    if (payloadLength < 0) {
        fprintf(stderr, "Erro ao formatar payload para partição 0\n");
        return; // Não tenta publicar dados inválidos
    }

    // Publica os dados no tópico MQTT
    mosquitto_publish(mosq, NULL, topic, payloadLength, payload, 0, 0);
    printf("Publicado no tópico %s, payload: %s\n", topic, payload);

} // publishPartition0Data


/**
 *  @brief Publish the partionData1 in MQTT 
 *  @param mosq MQTT object
 *  @param Partition1Data Data to be published
 *  
 */
void publishPartition1Data(struct mosquitto *mosq, struct Partition1Data *data, char *mqttTopic) {
    char payload[2048]; // Ajuste o tamanho conforme necessário
    int payloadLength = 0;
    char topic[100];

    // Constrói o tópico para a partição 1
    snprintf(topic, sizeof(topic), mqttTopic);

    // Prepara o payload com os dados da estrutura no formato "Nome:Valor"
    payloadLength = snprintf(payload, sizeof(payload),
        "{ "
        "\"LoadState\":%u,"
        "\"ChargeState\":%u,"
        "\"AlarmeFailure\":%u," // Alarme_Failure (sem aspas, é número)
        "\"BatterySOC\":%u," // SOC é %d [cite: 69]
        "\"BatteryVoltage\":%.1f," // Multiplicador 0.1 [cite: 69]
        "\"ChargeCurrent\":%.2f," // Multiplicador 0.01 [cite: 69]
        "\"DeviceTemperature\":%d," // Temperaturas podem ser negativas (sem aspas) [cite: 69]
        "\"BatteryTemperature\":%d," // Temperaturas podem ser negativas (sem aspas) [cite: 69]
        "\"LoadVoltage\":%.1f," // Multiplicador 0.1 [cite: 70]
        "\"LoadCurrent\":%.2f," // Multiplicador 0.01 [cite: 72]
        "\"LoadPower\":%u," // Sem multiplicador [cite: 72]
        "\"SolarPanelVoltage\":%.1f," // Multiplicador 0.1 [cite: 72]
        "\"SolarPanelCurrent\":%.2f," // Multiplicador 0.01 [cite: 72]
        "\"ChargePower\":%u," // Sem multiplicador [cite: 72]
        "\"LoadOnOff\":%u," // Sem multiplicador [cite: 72]
        "\"MinBatteryVoltageToday\":%.1f," // Multiplicador 0.1 [cite: 75]
        "\"MaxBatteryVoltageToday\":%.1f," // Multiplicador 0.1 [cite: 75]
        "\"MaxChargeCurrentToday\":%.2f," // Multiplicador 0.01 [cite: 75]
        "\"MaxDischargeCurrentToday\":%.2f," // Multiplicador 0.01 [cite: 78]
        "\"MaxChargePowerToday\":%u," // Sem multiplicador [cite: 78]
        "\"MaxDischargePowerToday\":%u," // Sem multiplicador [cite: 81]
        "\"ChargeAmpereHourToday\":%u," // Sem multiplicador [cite: 81]
        "\"DischargeAmpereHourToday\":%u," // Sem multiplicador [cite: 81]
        "\"GeneratingCapacityToday\":%u," // Sem multiplicador (mas o protocolo diz /1000 para kWh) [cite: 81]
        "\"ElectricityConsumedToday\":%u," // Sem multiplicador (mas o protocolo diz /1000 para kWh) [cite: 81]
        "\"TotalOperatingDays\":%u," // Sem multiplicador [cite: 84]
        "\"TotalOverdischargetimes\":%u," // Sem multiplicador [cite: 84]
        "\"TotalBatteryChargeTimes\":%u," // Sem multiplicador [cite: 84]
        "\"TotalBatteryChargeAH\":%u," // Sem multiplicador (mas o protocolo mostra KAH) [cite: 88]
        "\"TotalBatteryDischargeAH\":%u," // Sem multiplicador (mas o protocolo mostra KAH) [cite: 88]
        "\"AccumulatedGeneratingCapacity\":%u," // Sem multiplicador (mas o protocolo mostra KWH) [cite: 88]
        "\"AccumulatedElectricityConsuption\":%u," // Sem multiplicador (mas o protocolo mostra KWH) [cite: 88]
        "\"DummyFailureAlarme\":%u," // Endereço 0x121 [cite: 195]
        "\"MaxBatteryTemperatureToday\":%u," // Endereço 0x123 [cite: 87]
        "\"MinBatteryTemperatureToday\":%u," // Endereço 0x124 [cite: 90]
        "\"TotalLoadOperationTime\":%u," // Endereço 0x125 [cite: 90]
        "\"Onduration\":%u," // Endereço 0x127 [cite: 90]
        "\"OffDuration\":%u," // Endereço 0x128 [cite: 90]
        "\"LightingIndex\":%u," // Endereço 0x129 [cite: 93]
        "\"EnergyConsumption\":%u," // Endereço 0x12A [cite: 93]
        "\"SystemHealthIndex\":%u," // Endereço 0x12B [cite: 93]
        "\"ChargeDurationOnDay\":%u," // Endereço 0x12C [cite: 93]
        "\"NightDuration\":%u," // Endereço 0x12D [cite: 93]
        "\"OnLine\":%u"
        "}",
        data->LoadState,
        data->ChargeState,
        data->Alarme_Failure,
        data->BatterySOC, // SOC %d
        data->BatteryVoltage / 10.0, // 0.1V [cite: 69]
        data->ChargeCurrent / 100.0, // 0.01A [cite: 69]
        data->DeviceTemperature, // %d
        data->BatteryTemperature, // %d
        data->LoadVoltage / 10.0, // 0.1V [cite: 70]
        data->LoadCurrent / 100.0, // 0.01A [cite: 72]
        data->LoadPower, // %u
        data->SolarPanelVoltage / 10.0, // 0.1V [cite: 72]
        data->SolarPanelCurrent / 100.0, // 0.01A [cite: 72]
        data->ChargePower, // %u
        data->LoadOnOff, // %u
        data->MinBatteryVoltageToday / 10.0, // 0.1V [cite: 75]
        data->MaxBatteryVoltageToday / 10.0, // 0.1V [cite: 75]
        data->MaxChargeCurrentToday / 100.0, // 0.01A [cite: 75]
        data->MaxDischargeCurrentToday / 100.0, // 0.01A [cite: 78]
        data->MaxChargePowerToday, // %u [cite: 78]
        data->MaxDischargePowerToday, // %u [cite: 81]
        data->ChargeAmpereHourToday, // %u [cite: 81]
        data->DischargeAmpereHourToday, // %u [cite: 81]
        data->GeneratingCapacityToday, // %u [cite: 81]
        data->ElectricityConsumedToday, // %u [cite: 81]
        data->TotalOperatingDays, // %u [cite: 84]
        data->TotalOverdischargetimes, // %u [cite: 84]
        data->TotalBatteryChargeTimes, // %u [cite: 84]
        data->TotalBatteryChargeAH, // %u [cite: 88]
        data->TotalBatteryDischargeAH, // %u [cite: 88]
        data->AccumulatedGeneratingCapacity, // %u [cite: 88]
        data->AccumulatedElectricityConsuption, // %u [cite: 88]
        data->DummyFailureAlarme, // %u [cite: 195]
        data->MaxBatteryTemperatureToday, // %u [cite: 87]
        data->MinBatteryTemperatureToday, // %u [cite: 90]
        data->TotalLoadOperationTime, // %u [cite: 90]
        data->Onduration, // %u [cite: 90]
        data->OffDuration, // %u [cite: 90]
        data->LightingIndex, // %u [cite: 93]
        data->EnergyConsumption, // %u [cite: 93]
        data->SystemHealthIndex, // %u [cite: 93]
        data->ChargeDurationOnDay, // %u [cite: 93]
        data->NightDuration, // %u [cite: 93]
        data->onLine
    );

    if (payloadLength < 0) {
        fprintf(stderr, "Erro ao formatar payload para partição 1\n");
        return; // Não tenta publicar dados inválidos
    }

    // Publica os dados no tópico MQTT
    mosquitto_publish(mosq, NULL, topic, payloadLength, payload, 0, 0);
    printf("Publicado no tópico %s, payload: %s\n", topic, payload);

} // publishPartition1Data

/**
 *  @brief Publish Wetather Station data in MQTT 
 *  @param mosq MQTT object
 *  @param WeatherStationData Data to be published
 *  
 */
void publishWeatherStationData(struct mosquitto *mosq, struct TWeatherStationData *data, char *mqttTopic) {
    char payload[2048]; // Ajuste o tamanho conforme necessário
    int payloadLength = 0;
    char topic[100];

    // Constrói o tópico para a partição 1
    snprintf(topic, sizeof(topic), mqttTopic);

    // Prepara o payload com os dados da estrutura no formato "Nome:Valor"
    payloadLength = snprintf(payload, sizeof(payload),
        "{ "
        "\"atmosphericPressure\":%.1f,"
        "\"airTemperature\":%.1f,"
        "\"relativeHumidity\":%.1f," 
        "\"accumulatedRainfall\":%.1f," 
        "\"windDirection\":%u," 
        "\"windSpeed\":%.1f," 
        "\"OnLine\":%u"
        /*
        uint16_t WindForce;
        uint16_t WindDirectionGrade;
        uint16_t Noise;
        uint16_t PM2_5;
        uint16_t PM2_10;
        uint32_t W20Lux;
        uint16_t W20Rediation;
        uint16_t TotalRadiation;
        */
        "}",
        data->AtmosphericPressure/10.0,
        data->Temperature/10.0,
        data->Humidity/10.0,
        data->Rainfall/10.0,
        data->WindDirectionAngle,
        data->WindSpeed/10.0,
        data->onLine
    );

    if (payloadLength < 0) {
        fprintf(stderr, "Erro ao formatar payload para weather station\n");
        return; // Não tenta publicar dados inválidos
    }

    // Publica os dados no tópico MQTT
    mosquitto_publish(mosq, NULL, topic, payloadLength, payload, 0, 0);
    printf("Publicado no tópico %s, payload: %s\n", topic, payload);

} // publishWeatherStationData


/****************************************************************/
/**
 * @brief The application entry point.
 *
 * 
 * @return Return the program error code.
****************************************************************/ 
int main() {

    int serialPortFD;
    int rc;
    int serialPortNumber;
    char *mqttServerAddr, *mqttPortStr, *updateIntervalStr;
    char *mqttClientId, *mqttTopicPartition0, *mqttTopicPartition1, *mqttTopicWeather;
    char *serialPortStr;
    struct mosquitto *mosq = NULL;
    int mqttPort;
    int updateInterval;

    fprintf (stdout,"======================\n");
    fprintf (stdout,"     STT  MODBUS\n");
    fprintf (stdout,"======================\n");
    
    
    // Lê os outros parâmetros de configuração
    serialPortStr = getenv("SERIAL_PORT");
    if (serialPortStr != NULL) serialPortNumber = atoi(serialPortStr);
    else serialPortNumber = SERIAL_PORT;        
    // Servidor MQTT
    mqttServerAddr = getenv("SERVIDOR_MQTT");
    if (mqttServerAddr == NULL) mqttServerAddr = (char *)MQTT_BROKER_HOST;
    // Porta MQTT
    mqttPortStr = getenv("PORT_MQTT");
    if (mqttPortStr == NULL) mqttPort = MQTT_BROKER_PORT;
    else mqttPort = atoi(mqttPortStr);
    // Client ID no MQTT
    mqttClientId = getenv("CLIENT_ID");
    if (mqttClientId == NULL) mqttClientId = (char *)MQTT_CLIENT_ID;
    // Tópico do MPPT partição 0
    mqttTopicPartition0 = getenv("TOPIC_PARTITION0");
    if (mqttTopicPartition0 == NULL) mqttTopicPartition0 = (char *)MQTT_TOPIC_PARTITION_0;
    // Tópico do MPPT partição 1
    mqttTopicPartition1 = getenv("TOPIC_PARTITION1");
    if (mqttTopicPartition1 == NULL) mqttTopicPartition1 = (char *)MQTT_TOPIC_PARTITION_1;
    // Tópico da Estação Meteorológica
    mqttTopicWeather = getenv("TOPIC_WEATHER");
    if (mqttTopicWeather == NULL) mqttTopicWeather = (char *)MQTT_TOPIC_WEATHERSTATION;
    // Intervalo de atualização
    updateIntervalStr = getenv("UPDATE_NTERVAL");
    if (updateIntervalStr == NULL) updateInterval = DEFAULT_UPDATE_NTERVAL;
    else updateInterval = atoi(updateIntervalStr);

    // Loga os parametros que o sistema irá utilizar.
    fprintf (stdout, "\nParâmetros do programa\n");
    fprintf (stdout, "Porta serial: %d\n", serialPortNumber);
    fprintf (stdout, "Servidor MQTT: %s\n", mqttServerAddr);
    fprintf (stdout, "Porta servidor MQTT: %d\n", mqttPort);
    fprintf (stdout, "MPPT Topic P0: %s\n", mqttTopicPartition0);
    fprintf (stdout, "MPPT Topic P1: %s\n", mqttTopicPartition1);
    fprintf (stdout, "Weather Station Topic: %s\n", mqttTopicWeather);
    fprintf (stdout, "Intervalo de atualização: %d\n", updateInterval);
    fprintf (stdout,"=================================\n");

    // Abre a porta serial
    serialPortFD = openSerialPort(serialPortNumber, BAUDRATE, PARITY, DATA_BITS, STOP_BITS);
    fprintf (stdout, "\nAbrindo porta serial: %d\n", serialPortNumber);
    if (serialPortFD == -1) {
        fprintf(stderr,"Falha ao abrir a porta serial.\n");
        return 1;
    }

    // Configura os pinos de IO da porta serial
    fprintf (stdout, "\nConfigurando pinos de IO da porta serial\n");
    if (ConfigIO(serialPortNumber) == -1) {
        fprintf(stderr, "Falha ao abrir configurar os pinos de IO.\n");
        return 1;
    }

    // Inicializa o MQTT
    fprintf (stdout, "\nInicializando MQTT: %s\n", mqttServerAddr);
    if (InitMQTT(&mosq, mqttServerAddr, mqttClientId, mqttPort) != 0) {
        fprintf(stderr, "Falha ao inicializar o MQTT.\n");
        closeSerialPort(serialPortFD);
        return 1;
    }

    fflush (stdout);
    // Loop do programa
    while (1) {

        // Le e publicaca dados da partição 0
        Partition0Data partition0Data;
        if (readPartition0Data(serialPortFD, &partition0Data)) {
            partition0Data.onLine = true;
            publishPartition0Data(mosq, &partition0Data, mqttTopicPartition0);
        }
        else {
            memset (&partition0Data,0,sizeof(Partition0Data));
            publishPartition0Data(mosq, &partition0Data, mqttTopicPartition0);
        }

        // Le e publicaca dados da partição 1
        Partition1Data partition1Data;
        if (readPartition1Data(serialPortFD, &partition1Data)) {
            partition1Data.onLine = true;
            publishPartition1Data(mosq, &partition1Data, mqttTopicPartition1);
        }
        else {
            memset (&partition1Data,0,sizeof(Partition1Data));
            publishPartition1Data(mosq, &partition1Data, mqttTopicPartition1);            
        }
        
        // Le e publica dados da estação meteorologica
        TWeatherStationData weatherStationData;
        if (readWeatherStData(serialPortFD, &weatherStationData)) {
            weatherStationData.onLine = true;
            publishWeatherStationData(mosq,&weatherStationData, mqttTopicWeather);
        }
        else {
            memset (&weatherStationData,0,sizeof(TWeatherStationData));
            publishWeatherStationData(mosq, &weatherStationData, mqttTopicWeather);            
        }


        // Process Mosquitto events
        rc = mosquitto_loop(mosq, -1, 1);
        if (rc != MOSQ_ERR_SUCCESS) {
            fprintf(stderr, "mosquitto_loop failed: %s\n", mosquitto_strerror(rc));
            return 1;
        }
    
        fflush (stdout);
        // Aguarda o próximo ciclo 
        sleep(updateInterval); 

    } // while

    // Fecha a porta serial
    closeSerialPort(serialPortFD);
    // Fecha a conexão MQTT
    CloseMQTT(mosq);

    return 0;

} // main


