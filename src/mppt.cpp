/***********************************************************************************/
/**
 * @file mppt.cpp
 * @brief MPPT comunication module
 * 
 * This files inplements function to comunicate with the MPPT device.
 * It uses the basic function from the modbus.cpp module to provide the higher
 * level comunication functions.
 * 
*************************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "modbus.h"
#include "mppt.h"


// Função para ler a tensão máxima do sistema e a corrente de carga nominal
bool readMaxVoltageAndChargeCurrent(int fd, uint8_t *systemVoltage, uint8_t *chargeCurrent) {
    uint8_t responseBuffer[MAX_BUFFER_SIZE];
    uint16_t responseLength;

    if (!sendReadRegistersCommand(fd, MPPT_ADDRESS, REG_MAX_VOLTAGE_CHARGE_CURRENT, 1)) {
        printf("Erro ao enviar comando de leitura.\n");
        return false;
    }

    if (!receiveResponse(fd, FUNCTION_CODE_READ_HOLDING_REGISTERS, MPPT_ADDRESS, responseBuffer, &responseLength, SIZE_CMD_READ_HOLDING_REGISTER)) {
        printf("Erro ao receber resposta.\n");
        return false;
    }
    if (responseLength < 7) {
        printf("Erro: Resposta inesperada, tamanho da resposta menor que o esperado.\n");
        return false;
    }

    // Extrai os dados da resposta
    *systemVoltage = (responseBuffer[3] >> 4) & 0x0F;
    *chargeCurrent = responseBuffer[4] & 0x0F;

    return true;
}

// Função para ler o SOC da bateria
bool readBatterySOC(int fd, uint8_t *batterySOC) {
    uint8_t responseBuffer[MAX_BUFFER_SIZE];
    uint16_t responseLength;

    if (!sendReadRegistersCommand(fd, MPPT_ADDRESS, REG_BATTERY_CAPACITY_SOC, 1)) {
        printf("Erro ao enviar comando de leitura do SOC da bateria.\n");
        return false;
    }

    if (!receiveResponse(fd, FUNCTION_CODE_READ_HOLDING_REGISTERS, MPPT_ADDRESS, responseBuffer, &responseLength, SIZE_CMD_READ_HOLDING_REGISTER)) {
        printf("Erro ao receber resposta.\n");
        return false;
    }
    if (responseLength < 7) {
        printf("Erro: Resposta inesperada, tamanho da resposta menor que o esperado.\n");
        return false;
    }

    // Extrai o SOC da resposta
    *batterySOC = responseBuffer[3];

    return true;
}

// Função para ler o estado da carga e o estado do carregamento
bool readLoadAndChargeState(int fd, uint8_t *loadState, uint8_t *chargeState) {
    uint8_t responseBuffer[MAX_BUFFER_SIZE];
    uint16_t responseLength;

    if (!sendReadRegistersCommand(fd, MPPT_ADDRESS, REG_LOAD_STATE_CHARGE_STATE, 1)) {
        printf("Erro ao enviar comando de leitura.\n");
        return false;
    }

    if (!receiveResponse(fd, FUNCTION_CODE_READ_HOLDING_REGISTERS, MPPT_ADDRESS, responseBuffer, &responseLength,SIZE_CMD_READ_HOLDING_REGISTER)) {
        printf("Erro ao receber resposta.\n");
        return false;
    }
    if (responseLength < 7) {
        printf("Erro: Resposta inesperada, tamanho da resposta menor que o esperado.\n");
        return false;
    }

    // Extrai os dados da resposta
    *loadState = (responseBuffer[3] >> 7) & 0x01;
    *chargeState = responseBuffer[4];

    return true;
}

// Função para alterar o endereço do dispositivo
bool changeDeviceAddress(int fd, uint8_t newAddress) {
    uint8_t responseBuffer[MAX_BUFFER_SIZE];
    uint16_t responseLength;
    if (!sendWriteSingleRegisterCommand(fd, MPPT_ADDRESS, REG_DEVICE_ADDRESS, newAddress)) {
        printf("Erro ao enviar comando de escrita para alterar endereço.\n");
        return false;
    }
    if (!receiveResponse(fd, FUNCTION_CODE_WRITE_SINGLE_REGISTER, MPPT_ADDRESS, responseBuffer, &responseLength,SIZE_CMD_WRITE_SINGLE_REGISTER)) {
        printf("Erro ao receber resposta ao alterar endereço.\n");
        return false;
    }
    if (responseLength < 8) {
        printf("Erro: Resposta inesperada, tamanho da resposta menor que o esperado.\n");
        return false;
    }
    if (responseBuffer[0] != newAddress) {
        printf("Erro: Endereço não alterado.\n");
        return false;
    }
    return true;
}

// Efetua a leitura da partição 0
bool readPartition0Data(int fd, Partition0Data *data) {
    uint8_t responseBuffer[256]; // Buffer para armazenar a resposta
    uint16_t responseLength;
    
    // Envia o comando para ler os registradores da partição 0 (endereços 0x000A a 0x001A)
    if (!sendReadRegistersCommand(fd, MPPT_ADDRESS, REG_MAX_VOLTAGE_CHARGE_CURRENT, PARTITION_0_SIZE)) {
        // Use REG_MAX_VOLTAGE_CHARGE_CURRENT (0x000A) como endereço inicial
        printf("Erro ao enviar comando de leitura para a partição 0.\n");
        return false;
    }

    // Recebe a resposta
    if (!receiveResponse(fd, FUNCTION_CODE_READ_HOLDING_REGISTERS, MPPT_ADDRESS, responseBuffer, &responseLength,SIZE_CMD_READ_HOLDING_REGISTER)) {
        printf("Erro ao receber resposta da partição 0.\n");
        return false;
    }

    if (responseLength < 5 + PARTITION_0_SIZE*2) {
        printf("Erro: Resposta incompleta para a partição 0.\n");
        return false;
    }

    // Extrai os dados da resposta e preenche a estrutura
    data->MaxVoltage = responseBuffer[3];
    data->RatedCurrent = responseBuffer[4];
    data->RatedDischargeCurrent = responseBuffer[5];
    data->ProductType =  responseBuffer[6];

    // Copia os 16 bytes de ProductSpecification
    memcpy(data->ProductSpecification, &responseBuffer[7], 16);
    data->ProductSpecification[16] = 0;

    data->SoftwareVersion = (responseBuffer[23] << 24) | (responseBuffer[24] << 16) | (responseBuffer[25] << 8) | responseBuffer[26];
    data->HardwareVersion = (responseBuffer[27] << 24) | (responseBuffer[28] << 16) | (responseBuffer[29] << 8) | responseBuffer[30];
    data->ProductSerialNumber = (responseBuffer[31] << 24) | (responseBuffer[32] << 16) | (responseBuffer[33] << 8) | responseBuffer[34];
    data->DeviceAddress = (responseBuffer[35] << 8) | responseBuffer[36];

    return true;
} // readPartition0Data

// Efetua a leitura da partição 1
bool readPartition1Data(int fd, Partition1Data *data) {
    uint8_t responseBuffer[256]; // Buffer para armazenar a resposta
    uint16_t responseLength;

    // Envia o comando para ler os registradores da partição 1 (endereços 0x00FD a 0x012D)
    if (!sendReadRegistersCommand(fd, MPPT_ADDRESS, REG_LOAD_STATE_CHARGE_STATE, PARTITION_1_SIZE)) { 
        printf("Erro ao enviar comando de leitura para a partição 1.\n");
        return false;
    }

    // Recebe a resposta
    if (!receiveResponse(fd, FUNCTION_CODE_READ_HOLDING_REGISTERS, MPPT_ADDRESS, responseBuffer, &responseLength, SIZE_CMD_READ_HOLDING_REGISTER)) {
        printf("Erro ao receber resposta da partição 1.\n");
        return false;
    }

    if (responseLength < 5 + PARTITION_1_SIZE * 2) { // Verifica se a resposta tem o tamanho esperado (1 byte addr + 1 byte func + 1 byte data length + 49 registros * 2 + 2 bytes crc)
        printf("Erro: Resposta incompleta para a partição 1. Esperado: %d, Recebido: %d\n", 5 + PARTITION_1_SIZE * 2, responseLength);
        return false;
    }

    // Extrai os dados da resposta e preenche a estrutura
    data->LoadState = responseBuffer[3];
    data->ChargeState = responseBuffer[4];
    data->Alarme_Failure = (responseBuffer[5] << 24) | (responseBuffer[6] << 16) | (responseBuffer[7] << 8) | responseBuffer[8];  // Correção: Falha e Alarme
    data->BatterySOC = (responseBuffer[9] << 8) | responseBuffer[10];
    data->BatteryVoltage = (responseBuffer[11] << 8) | responseBuffer[12];
    data->ChargeCurrent = (responseBuffer[13] << 8) | responseBuffer[14];
    data->DeviceTemperature = responseBuffer[15]; 
    data->BatteryTemperature = responseBuffer[16];
    data->LoadVoltage = (responseBuffer[17] << 8) | responseBuffer[18];
    data->LoadCurrent = (responseBuffer[19] << 8) | responseBuffer[20];
    data->LoadPower = (responseBuffer[21] << 8) | responseBuffer[22];
    data->SolarPanelVoltage = (responseBuffer[23] << 8) | responseBuffer[24];
    data->SolarPanelCurrent = (responseBuffer[25] << 8) | responseBuffer[26];
    data->ChargePower = (responseBuffer[27] << 8) | responseBuffer[28];
    data->LoadOnOff = (responseBuffer[29] <<8) | responseBuffer[30];
    data->MinBatteryVoltageToday = (responseBuffer[31] << 8) | responseBuffer[32];
    data->MaxBatteryVoltageToday = (responseBuffer[33] << 8) | responseBuffer[34];
    data->MaxChargeCurrentToday = (responseBuffer[35] << 8) | responseBuffer[36];  
    data->MaxDischargeCurrentToday = (responseBuffer[37] << 8) | responseBuffer[38];
    data->MaxChargePowerToday = (responseBuffer[39] << 8) | responseBuffer[40];
    data->MaxDischargePowerToday = (responseBuffer[41] << 8) | responseBuffer[42];
    data->ChargeAmpereHourToday = (responseBuffer[43] << 8) | responseBuffer[44];
    data->DischargeAmpereHourToday = (responseBuffer[45] << 8) | responseBuffer[46];   
    data->GeneratingCapacityToday =  (responseBuffer[47] << 8) | responseBuffer[48];
    data->ElectricityConsumedToday = (responseBuffer[49] << 8) | responseBuffer[50];
    data->TotalOperatingDays = (responseBuffer[51] << 8) | responseBuffer[52];
    data->TotalOverdischargetimes = (responseBuffer[53] << 8) | responseBuffer[54];
    data->TotalBatteryChargeTimes = (responseBuffer[55] << 8) | responseBuffer[56];
    data->TotalBatteryChargeAH = (responseBuffer[57] << 24) | (responseBuffer[58] << 16) | (responseBuffer[59] << 8) | responseBuffer[60];
    data->TotalBatteryDischargeAH = (responseBuffer[61] << 24) | (responseBuffer[62] << 16) | (responseBuffer[63] << 8) | responseBuffer[64];
    data->AccumulatedGeneratingCapacity = (responseBuffer[65] << 24) | (responseBuffer[66] << 16) | (responseBuffer[67] << 8) | responseBuffer[68];
    data->AccumulatedElectricityConsuption = (responseBuffer[69] << 24) | (responseBuffer[70] << 16) | (responseBuffer[71] << 8) | responseBuffer[72];
    // salta os 2 endereços (73 e 74) - Load state e charge state ja foram lidos em 0xFD
    // salta os 4 endereços (75, 76, 77 e 78) de indicação de falha e alarme que já foram lidos no endereço 5a 8
    data->MaxBatteryTemperatureToday = (responseBuffer[79] << 8) | responseBuffer[80];
    data->MinBatteryTemperatureToday = (responseBuffer[81] << 8) | responseBuffer[82];
    data->TotalLoadOperationTime = (responseBuffer[83] << 24) | (responseBuffer[84] << 16) | (responseBuffer[85] << 8) | responseBuffer[86];
    data->Onduration = (responseBuffer[87] << 8) | responseBuffer[88];
    data->OffDuration = (responseBuffer[89] << 8) | responseBuffer[90];
    data->LightingIndex = (responseBuffer[91] << 8) | responseBuffer[92];
    data->EnergyConsumption = (responseBuffer[93] << 8) | responseBuffer[94];
    data->SystemHealthIndex = (responseBuffer[95] << 8) | responseBuffer[96];
    data->ChargeDurationOnDay = (responseBuffer[97] << 8) | responseBuffer[98];
    data->NightDuration = (responseBuffer[99] << 8) | responseBuffer[100];

    return true;

} // readPartition1Data
