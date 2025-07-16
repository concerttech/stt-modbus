/***********************************************************************************/
/**
 * @file weatherst.cpp
 * @brief Weather Station comunication module
 * 
 * This files inplements function to comunicate with the Weather Station device.
 * It uses the basic function from the modbus.cpp module to provide the higher
 * level comunication functions.
 * 
*************************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "modbus.h"
#include "weatherst.h"


// Efetua a leitura dos dados da estação meteorologica
bool readWeatherStData(int fd, TWeatherStationData *data) {
    uint8_t responseBuffer[256]; // Buffer para armazenar a resposta
    uint16_t responseLength;

    // Envia o comando para ler os registradores da estação meteorologica (endereços 0x01F4 a 0x0202)
    if (!sendReadRegistersCommand(fd, WEATHER_ST_ADDRESS, REG_WIND_SPEED, WEATHER_ST_DATA_SIZE)) { 
        printf("Erro ao enviar comando de leitura dos dados da estação meteorológica.\n");
        return false;
    }

    // Recebe a resposta
    if (!receiveResponse(fd, FUNCTION_CODE_READ_HOLDING_REGISTERS, WEATHER_ST_ADDRESS, responseBuffer, &responseLength, SIZE_CMD_READ_HOLDING_REGISTER)) {
        printf("Erro ao receber resposta da estação meteorológica.\n");
        return false;
    }

    if (responseLength < 5 + WEATHER_ST_DATA_SIZE * 2) { // Verifica se a resposta tem o tamanho esperado (1 byte addr + 1 byte func + 1 byte data length + 16 registros * 2 + 2 bytes crc)
        printf("Erro: Resposta incompleta para dados da estação meteorológica. Esperado: %d, Recebido: %d\n", 5 + WEATHER_ST_DATA_SIZE * 2, responseLength);
        return false;
    }

    // Extrai os dados da resposta e preenche a estrutura
    data->WindSpeed = (responseBuffer[3] << 8) | responseBuffer[4];
    data->WindForce = (responseBuffer[5] << 8) | responseBuffer[6];
    data->WindDirectionGrade = (responseBuffer[7] << 8) | responseBuffer[8];
    data->WindDirectionAngle = (responseBuffer[9] << 8) | responseBuffer[10];
    data->Humidity = (responseBuffer[11] << 8) | responseBuffer[12];
    data->Temperature = (responseBuffer[13] << 8) | responseBuffer[14];
    data->Noise = (responseBuffer[15] << 8) | responseBuffer[16];
    data->PM2_5 = (responseBuffer[17] << 8) | responseBuffer[18];
    data->PM2_10 = (responseBuffer[19] << 8) | responseBuffer[20];
    data->AtmosphericPressure = (responseBuffer[21] << 8) | responseBuffer[22];
    data->W20Lux = (responseBuffer[23] << 24) | (responseBuffer[24] << 16) | (responseBuffer[25] << 8) | responseBuffer[26];
    data->W20Rediation = (responseBuffer[27] << 8) | responseBuffer[28];
    data->Rainfall = (responseBuffer[29] << 8) | responseBuffer[30];
    data->TotalRadiation = (responseBuffer[31] << 8) | responseBuffer[32];

    return true;

} // readWeatherStData
