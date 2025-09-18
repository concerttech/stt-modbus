#include <stdio.h>
#include <stdint.h>
//#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <linux/serial.h>

#include "modbus.h"

#define SERIAL_PORT_STR "/dev/verdin-uart%d"

// Função para calcular o CRC16 (Modbus RTU)
uint16_t calculateCRC16(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
} // calculateCRC16

// Função para abrir a porta serial (Linux)
int openSerialPort(char port, int baudrate, char parity, int dataBits, int stopBits) {
    char msg[32];
    char portStr[32];
    int fd;

    // Cria o /dev da porta
    sprintf (portStr, SERIAL_PORT_STR, port);

    fd = open(portStr, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1) {
        sprintf(msg, "Erro ao abrir a porta %s", port);
        perror(msg);
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("Erro ao obter atributos da porta serial");
        close(fd);
        return -1;
    }

    // Configura os atributos da porta serial
    tty.c_iflag = IGNPAR;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ICANON;

    // Configura a velocidade de transmissão
    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);

    // Configura os bits de dados
    tty.c_cflag = (tty.c_cflag & ~CSIZE);
    switch (dataBits) {
        case 5:
            tty.c_cflag |= CS5;
            break;
        case 6:
            tty.c_cflag |= CS6;
            break;
        case 7:
            tty.c_cflag |= CS7;
            break;
        case 8:
        default:
            tty.c_cflag |= CS8;
            break;
    }
    // Configura a paridade
    if (parity == 'E') {
        tty.c_cflag |= PARENB;
        tty.c_cflag &= ~PARODD;
    } else if (parity == 'O') {
        tty.c_cflag |= PARENB | PARODD;
    } else {
        tty.c_cflag &= ~PARENB;
    }
    // Configura o stop bit
    if (stopBits == 2) {
        tty.c_cflag |= CSTOPB;
    } else {
        tty.c_cflag &= ~CSTOPB;
    }

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("Erro ao configurar atributos da porta serial");
        close(fd);
        return -1;
    }

    /* Enable RS485 mode: */
    struct serial_rs485 rs485conf;
    memset(&rs485conf, 0, sizeof(rs485conf));
    rs485conf.flags |= SER_RS485_ENABLED;
    rs485conf.flags |= SER_RS485_RTS_ON_SEND;
    rs485conf.flags &= ~(SER_RS485_RTS_AFTER_SEND);

    if (ioctl(fd, TIOCSRS485, &rs485conf) < 0) {
        perror("Erro ao definir RS485 mode");
        close(fd);
        return -1;
    }

    return fd;
}

// Função para fechar a porta serial (Linux)
void closeSerialPort(int fd) {
    close(fd);
    printf("Porta serial fechada.\n");
}

// Função para enviar dados pela porta serial (Linux)
int sendData(int fd, const uint8_t *data, uint16_t length) {
    ssize_t bytesSent = write(fd, data, length);
    if (bytesSent == -1) {
        perror("Erro ao enviar dados");
        return -1;
    }
    printf("Enviando dados: ");
    for (uint16_t i = 0; i < length; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
    return (int)bytesSent;
}

// Função para receber dados da porta serial com timeout entre bytes usando select()
int receiveDataWithTimeout(int fd, uint8_t *buffer, uint16_t maxLength, long timeout_ms) {
    fd_set readfds;
    struct timeval tv;
    int ret;
    int bytesRead = 0;
   
    FD_ZERO(&readfds);
    FD_SET(fd, &readfds);

    printf("Dados recebidos: ");
    int i = 0;

    //###
    // usleep(50000);

    while (bytesRead < maxLength) {
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;

        ret = select(fd + 1, &readfds, NULL, NULL, &tv);
        if (ret == -1) {
            perror("Erro em select()");
            return -1;
        } else if (ret == 0) {
            break; // Timeout entre bytes
        } else {
            ssize_t n = read(fd, buffer + bytesRead, maxLength - bytesRead);
            if (n == -1) {
                perror("Erro ao ler da porta serial");
                return -1;
            }
            if (n == 0)
                continue;

            bytesRead += n;            
            for (; i < bytesRead; i++) {
                printf("%02X ", buffer[i]);
            }
    
        }
    }
    printf("\n");
    return bytesRead;
}

// Função para enviar comando de leitura de registrador(es)
bool sendReadRegistersCommand(int fd, uint8_t deviceAddress, uint16_t startAddress, uint16_t numRegisters) {
    
    uint8_t buffer[8];
    uint16_t crc;

    buffer[0] = deviceAddress;
    buffer[1] = FUNCTION_CODE_READ_HOLDING_REGISTERS;
    buffer[2] = (startAddress >> 8) & 0xFF;
    buffer[3] = startAddress & 0xFF;
    buffer[4] = (numRegisters >> 8) & 0xFF;
    buffer[5] = numRegisters & 0xFF;

    crc = calculateCRC16(buffer, 6);
    buffer[6] = crc & 0xFF;
    buffer[7] = (crc >> 8) & 0xFF;

    return sendData(fd, buffer, 8) == 8;

} // sendReadRegistersCommand

// Função para enviar comando de escrita em um único registrador
bool sendWriteSingleRegisterCommand(int fd, uint8_t deviceAddress, uint16_t registerAddress, uint16_t value) {
    
    uint8_t buffer[8];
    uint16_t crc;

    buffer[0] = deviceAddress;
    buffer[1] = FUNCTION_CODE_WRITE_SINGLE_REGISTER;
    buffer[2] = (registerAddress >> 8) & 0xFF;
    buffer[3] = registerAddress & 0xFF;
    buffer[4] = (value >> 8) & 0xFF;
    buffer[5] = value & 0xFF;

    crc = calculateCRC16(buffer, 6);
    buffer[6] = crc & 0xFF;
    buffer[7] = (crc >> 8) & 0xFF;

    return sendData(fd, buffer, 8) == 8;

} // sendWriteSingleRegisterCommand

// Função para enviar comando de escrita em múltiplos registradores
bool sendWriteMultipleRegistersCommand(int fd, uint8_t deviceAddress, uint16_t startAddress, uint16_t numRegisters, const uint16_t *values, uint16_t numBytes) {
    
    uint16_t crc;
    uint16_t commandLength = 9 + numBytes;
    uint8_t buffer[commandLength];

    buffer[0] = deviceAddress;
    buffer[1] = FUNCTION_CODE_WRITE_MULTIPLE_REGISTERS;
    buffer[2] = (startAddress >> 8) & 0xFF;
    buffer[3] = startAddress & 0xFF;
    buffer[4] = (numRegisters >> 8) & 0xFF;
    buffer[5] = numRegisters & 0xFF;
    buffer[6] = numBytes;

    for (uint16_t i = 0; i < numRegisters; i++) {
        buffer[7 + (i * 2)] = (values[i] >> 8) & 0xFF;
        buffer[7 + (i * 2) + 1] = values[i] & 0xFF;
    }
    crc = calculateCRC16(buffer, commandLength - 2);
    buffer[commandLength - 2] = crc & 0xFF;
    buffer[commandLength - 1] = (crc >> 8) & 0xFF;

    return sendData(fd, buffer, commandLength) == commandLength;

} // sendWriteMultipleRegistersCommand

// Função para receber resposta e verificar erros
bool receiveResponse(int fd, uint8_t expectedFunctionCode, uint8_t expectedDeviceAddress, uint8_t *buffer, uint16_t *length, uint8_t CmdSize) {

    uint16_t receivedCRC, calculatedCRC;
    int numBytesRead = receiveDataWithTimeout(fd, buffer, MAX_BUFFER_SIZE, 100);

    // subtrai o tamanho do comando (echo)
    numBytesRead -= CmdSize;
    
    if (numBytesRead <= 0) {
        printf("Erro: Nenhuma resposta recebida ou erro na recepção.\n");
        return false;
    }
    memcpy (buffer, buffer+CmdSize, numBytesRead);

    *length = numBytesRead;
    if (numBytesRead < 5) {
        printf("Erro: Resposta muito curta.\n");
        return false;
    }

    // Verificação do endereço do dispositivo
    if (buffer[0] != expectedDeviceAddress) {
        printf("Erro: Endereço do dispositivo incorreto na resposta (Esperado: %02X, Recebido: %02X)\n", expectedDeviceAddress, buffer[0]);
        return false;
    }

    // Verificação do código de função
    if (buffer[1] == (expectedFunctionCode | 0x80)) {
        printf("Erro MODBUS: Código de função %02X com exceção %02X\n", expectedFunctionCode, buffer[2]);
        // Adicione aqui lógica para tratar códigos de exceção MODBUS
        return false;
    } else if (buffer[1] != expectedFunctionCode) {
        printf("Erro: Código de função incorreto na resposta (Esperado: %02X, Recebido: %02X)\n", expectedFunctionCode, buffer[1]);
        return false;
    }

    // Verificação do CRC
    receivedCRC = (buffer[numBytesRead - 1] << 8) | buffer[numBytesRead - 2];
    calculatedCRC = calculateCRC16(buffer, numBytesRead - 2);
    if (receivedCRC != calculatedCRC) {
        printf("Erro: Falha na verificação de CRC (Esperado: %04X, Recebido: %04X)\n", calculatedCRC, receivedCRC);
        return false;
    }

    return true;
} // receiveResponse





