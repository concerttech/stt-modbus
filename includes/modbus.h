#ifndef MODBUS_REGISTERS_H
#define MODBUS_REGISTERS_H

// Definições para o protocolo MODBUS
#define MAX_BUFFER_SIZE 256 // Tamanho máximo do buffer de recepção
#define FUNCTION_CODE_READ_HOLDING_REGISTERS 0x03
#define FUNCTION_CODE_WRITE_SINGLE_REGISTER 0x06
#define FUNCTION_CODE_WRITE_MULTIPLE_REGISTERS 0x10

#define SIZE_CMD_READ_HOLDING_REGISTER 8
#define SIZE_CMD_WRITE_SINGLE_REGISTER 8

// Definição dos endereços dos registradores
// P0 - Informações do Produto
#define PARTITION_0_SIZE    17
#define REG_MAX_VOLTAGE_CHARGE_CURRENT    0x000A  // Tensão Máxima do Sistema e Corrente de Carga Nominal
#define REG_RATED_DISCHARGE_CURRENT_PRODUCT_TYPE 0x000B //Corrente de Descarga Nominal e Tipo de Produto
#define REG_SOFTWARE_VERSION            0x0014  // Versão do Software
#define REG_HARDWARE_VERSION            0x0016  // Versão do Hardware
#define REG_PRODUCT_SERIAL_NUMBER         0x0018  // Número de Série do Produto
#define REG_DEVICE_ADDRESS              0x001A  // Endereço do Dispositivo

// P1 - Dados em Tempo Real do Controlador
#define PARTITION_1_SIZE 49
#define REG_LOAD_STATE_CHARGE_STATE       0x00FD  // Estado da Carga e Estado do Carregamento
#define REG_CONTROLLER_FAULT_ALARM        0x00FE  // Falha do Controlador e Alarme
#define REG_BATTERY_CAPACITY_SOC          0x0100  // Capacidade da Bateria SOC
#define REG_BATTERY_VOLTAGE             0x0101  // Tensão da Bateria
#define REG_CHARGE_CURRENT              0x0102  // Corrente de Carga
#define REG_DEVICE_AND_BATTERY_TEMPERATURE  0x0103  // Temperatura do Dispositivo (Controlador) e da Bateria
#define REG_DC_LOAD_VOLTAGE              0x0104  // Tensão da Carga DC
#define REG_DC_LOAD_CURRENT              0x0105  // Corrente da Carga DC
#define REG_DC_LOAD_POWER                0x0106  // Potência da Carga DC
#define REG_SOLAR_PANEL_VOLTAGE           0x0107  // Tensão do Painel Solar
#define REG_SOLAR_PANEL_CURRENT           0x0108  // Corrente do Painel Solar
#define REG_CHARGE_POWER                0x0109  // Potência de Carga
#define REG_DC_LOAD_ON_OFF              0x010A  // Comando Liga/Desliga da Carga DC
#define REG_MIN_BATTERY_VOLTAGE_TODAY     0x010B  // Tensão Mínima da Bateria no Dia
#define REG_MAX_BATTERY_VOLTAGE_TODAY     0x010C  // Tensão Máxima da Bateria no Dia
#define REG_MIN_CHARGE_CURRENT_TODAY      0x010D  // Corrente Mínima de Carga no Dia
#define REG_MAX_CHARGE_CURRENT_TODAY      0x010E  // Corrente Máxima de Carga no Dia
#define REG_MAX_CHARGE_POWER_TODAY        0x010F  // Potência Máxima de Carga no Dia
#define REG_MAX_DISCHARGE_POWER_TODAY     0x0110  // Potência Máxima de Descarga no Dia
#define REG_DISCHARGE_AMPERE_HOUR_TODAY   0x0111  // Ampere-hora de Descarga no Dia
#define REG_GENERATING_CAPACITY_TODAY     0x0113  // Capacidade de Geração no Dia
#define REG_ELECTRICITY_CONSUMED_TODAY    0x0115  // Eletricidade Consumida no Dia
#define REG_TOTAL_BATTERY_CHARGE_TIMES    0x0117  // Total de Vezes de Carga da Bateria
#define REG_TOTAL_BATTERY_CHARGE_AH       0x0118  // Total de Ampere-hora de Carga da Bateria
#define REG_TOTAL_BATTERY_DISCHARGE_AH    0x011A  // Total de Ampere-hora de Descarga da Bateria
#define REG_TOTAL_GENERATING_CAPACITY     0x011C  // Capacidade Total de Geração
#define REG_MIN_BATTERY_TEMPERATURE_TODAY 0x0124 //Temperatura Mínima da Bateria no Dia
#define REG_TOTAL_LOAD_OPERATION_TIME     0x0125  // Tempo Total de Operação da Carga
#define REG_LAST_LOAD_TURN_ON_TIME       0x0127  // Último Tempo de Ligação da Carga
#define REG_LAST_LOAD_TURN_OFF_TIME      0x0128  // Último Tempo de Desligamento da Carga
#define REG_LIGHTING_INDEX              0x0129  // Índice de Iluminação
#define REG_ENERGY_CONSUMPTION            0x012A // Consumo de Energia

// P3 - Comandos de Controle do Dispositivo
#define REG_TURN_ON_OFF_LOAD             0xDF00  // Ligar/Desligar Carga
#define REG_CONSTANT_CURRENT_DIMMING      0xDF01  // Ajuste de Escurecimento de Corrente Constante
#define REG_VOLTAGE_CONTROL_ON_OFF_LOAD   0xDF02  // Ligar/Desligar Carga com Controle de Tensão
#define REG_CLEAR_CURRENT_ALARM         0xDF03  // Limpar Alarme de Corrente
#define REG_CLEAR_STATISTICS            0xDF04  // Limpar Estatísticas
#define REG_CLEAR_HISTORICAL_RECORD       0xDF05  // Limpar Registro Histórico
#define REG_RESTORE_FACTORY_SETTINGS      0xDF06  // Restaurar Configurações de Fábrica
#define REG_ACTIVATE_SLEEP_MODE           0xDF07  // Ativar Modo de Espera
#define REG_DEACTIVATE_SLEEP_MODE         0xDF08  // Desativar Modo de Espera
#define REG_SET_SYSTEM_TIME             0xDF09  // Configurar Hora do Sistema
#define REG_MANUAL_ON_POWER             0xDF0A  // Potência Manual Ligada
#define REG_MANUAL_ON_DURATION          0xDF0B  // Duração Manual Ligada
#define REG_MANUAL_OFF_DURATION         0xDF0C  // Duração Manual Desligada
#define REG_COMM_MODULE_POWER_OFF       0xDF0D  // Desligar Módulo de Comunicação

// P5 - Configuração dos Parâmetros da Bateria
#define REG_PV_CHARGE_SYSTEM_VOLTAGE      0xE002  // Tensão do Sistema Fotovoltaico de Carga
#define REG_BATTERY_SYSTEM_VOLTAGE        0xE003  // Tensão do Sistema da Bateria
#define REG_BATTERY_TYPE                  0xE004  // Tipo de Bateria
#define REG_BATTERY_CAPACITY              0xE005  // Capacidade da Bateria
#define REG_HIGH_VOLTAGE_DISCONNECT       0xE006  // Desconexão por Alta Tensão
#define REG_CHARGING_LIMIT_VOLTAGE        0xE007  // Tensão Limite de Carregamento
#define REG_OVERVOLTAGE_DISCONNECT        0xE008  // Desconexão por Sobretensão
#define REG_EQUALIZATION_CHARGING_VOLTAGE 0xE009  // Tensão de Carregamento de Equalização
#define REG_BOOST_CHARGING_VOLTAGE        0xE00A  // Tensão de Carregamento Boost
#define REG_BOOST_RETURN_VOLTAGE          0xE00B  // Tensão de Retorno do Boost
#define REG_OVERDISCHARGE_VOLTAGE         0xE00C  // Tensão de Sobredescarga
#define REG_UNDERVOLTAGE_WARNING          0xE00D  // Tensão de Aviso de Subtensão
#define REG_OVERDISCHARGE_RECOVERY        0xE00E  // Recuperação de Sobredescarga
#define REG_CHARGING_FLOAT_VOLTAGE        0xE00F  // Tensão de Flutuação de Carregamento
#define REG_FLOAT_RETURN_VOLTAGE          0xE010  // Tensão de Retorno da Flutuação
#define REG_DISCHARGE_LIMIT_VOLTAGE       0xE011  // Tensão Limite de Descarga
#define REG_BATTERY_RATED_VOLTAGE         0xE012  // Tensão Nominal da Bateria
#define REG_TEMPERATURE_COMPENSATION_COEFFICIENT 0xE013 // Coeficiente de Compensação de Temperatura
#define REG_CHARGING_CURRENT_LIMIT        0xE014  // Limite de Corrente de Carga
#define REG_DISCHARGE_CURRENT_LIMIT       0xE015  // Limite de Corrente de Descarga
#define REG_MIN_CHARGING_TEMPERATURE      0xE016  // Temperatura Mínima de Carregamento da Bateria
#define REG_MAX_CHARGING_TEMPERATURE      0xE017  // Temperatura Máxima de Carregamento da Bateria
#define REG_MIN_DISCHARGING_TEMPERATURE   0xE018  // Temperatura Mínima de Descarga da Bateria
#define REG_MAX_DISCHARGING_TEMPERATURE   0xE019  // Temperatura Máxima de Descarga da Bateria
#define REG_EQUALIZATION_CHARGING_TIME    0xE01A  // Tempo de Carregamento de Equalização
#define REG_BOOST_CHARGING_TIME           0xE01B  // Tempo de Carregamento Boost
#define REG_FLOAT_CHARGING_TIME           0xE01C  // Tempo de Carregamento Flutuante
#define REG_BAT_SWITCH_VOLTAGE            0xE01E  // Tensão de Comutação da Bateria

// P7 - Configuração dos Parâmetros da Carga da Lâmpada de Rua
#define REG_LOAD_CURRENT_SETTING          0xE08D  // Configuração de Corrente de Carga
#define REG_INTELLIGENT_POWER_MODE        0xE08E  // Modo de Potência Inteligente
#define REG_LIGHT_CONTROL_DELAY_TIME      0xE08F  // Duração do Atraso do Controle de Luz
#define REG_SENSING_DELAY_TIME            0xE090  // Tempo de Atraso de Sensoriamento
#define REG_SENSING_DISTANCE              0xE091  // Distância de Sensoriamento
#define REG_FIRST_SECTION_ON_DURATION     0xE092  // Duração Ligada da Primeira Seção
#define REG_FIRST_SECTION_ATTENDED_POWER  0xE093  // Potência Atendida da Primeira Seção
#define REG_FIRST_SECTION_UNATTENDED_POWER 0xE094  // Potência Não Atendida da Primeira Seção
#define REG_SECOND_SECTION_ON_DURATION    0xE095  // Duração Ligada da Segunda Seção
#define REG_SECOND_SECTION_ATTENDED_POWER 0xE096  // Potência Atendida da Segunda Seção
#define REG_SECOND_SECTION_UNATTENDED_POWER 0xE097 // Potência Não Atendida da Segunda Seção
#define REG_THIRD_SECTION_ON_DURATION      0xE098  // Duração Ligada da Terceira Seção
#define REG_THIRD_SECTION_ATTENDED_POWER   0xE099  // Potência Atendida da Terceira Seção
#define REG_THIRD_SECTION_UNATTENDED_POWER  0xE09A // Potência Não Atendida da Terceira Seção
#define REG_FOURTH_SECTION_ON_DURATION     0xE09B  // Duração Ligada da Quarta Seção
#define REG_FOURTH_SECTION_ATTENDED_POWER  0xE09C  // Potência Atendida da Quarta Seção
#define REG_FOURTH_SECTION_UNATTENDED_POWER 0xE09D // Potência Não Atendida da Quarta Seção
#define REG_FIFTH_SECTION_ON_DURATION      0xE09E  // Duração Ligada da Quinta Seção
#define REG_FIFTH_SECTION_ATTENDED_POWER   0xE09F  // Potência Atendida da Quinta Seção
#define REG_FIFTH_SECTION_UNATTENDED_POWER  0xE0A0 // Potência Não Atendida da Quinta Seção
#define REG_SIXTH_SECTION_ON_DURATION      0xE0A1  // Duração Ligada da Sexta Seção
#define REG_SIXTH_SECTION_ATTENDED_POWER   0xE0A2  // Potência Atendida da Sexta Seção
#define REG_SIXTH_SECTION_UNATTENDED_POWER  0xE0A3 // Potência Não Atendida da Sexta Seção
#define REG_SEVENTH_SECTION_ON_DURATION    0xE0A4  // Duração Ligada da Sétima Seção
#define REG_SEVENTH_SECTION_ATTENDED_POWER  0xE0A5  // Potência Atendida da Sétima Seção
#define REG_SEVENTH_SECTION_UNATTENDED_POWER 0xE0A6 // Potência Não Atendida da Sétima Seção
#define REG_EIGHTH_SECTION_ON_DURATION     0xE0A7  // Duração Ligada da Oitava Seção
#define REG_EIGHTH_SECTION_ATTENDED_POWER  0xE0A8  // Potência Atendida da Oitava Seção
#define REG_EIGHTH_SECTION_UNATTENDED_POWER 0xE0A9 // Potência Não Atendida da Oitava Seção
#define REG_NINTH_SECTION_ON_DURATION      0xE0AA  // Duração Ligada da Nona Seção
#define REG_NINTH_SECTION_ATTENDED_POWER   0xE0AB  // Potência Atendida da Nona Seção
#define REG_NINTH_SECTION_UNATTENDED_POWER  0xE0AC // Potência Não Atendida da Nona Seção
#define REG_MORNING_LIGHT_ON_DURATION    0xE0AD  // Duração da Luz Matinal Ligada
#define REG_MORNING_LIGHT_ATTENDED_POWER 0xE0AE  // Potência Atendida da Luz Matinal
#define REG_USER_DEFINE_START_VOLTAGE    0xE0AF  // Tensão de Início Definida pelo Usuário


/*-----------Funções exportadas--------------*/
int openSerialPort(char port, int baudrate, char parity, int dataBits, int stopBits);
void closeSerialPort(int fd);
bool sendReadRegistersCommand(int fd, uint8_t deviceAddress, uint16_t startAddress, uint16_t numRegisters);
bool receiveResponse(int fd, uint8_t expectedFunctionCode, uint8_t expectedDeviceAddress, uint8_t *buffer, uint16_t *length, uint8_t CmdSize);
bool sendWriteSingleRegisterCommand(int fd, uint8_t deviceAddress, uint16_t registerAddress, uint16_t value);

#endif // MODBUS_REGISTERS_H
