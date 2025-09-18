# Projeto STT-MODBUS
Esse projeto implementa a funcionalidade ModBus na CPU-100. O protocolo é utilizado para comunicar com o
controlador de carga e a estação meteorológica. O dados lidos são publicados no servidor MQTT

## Principais funcioalidades
O software executa a leitura dos dados do controlador de carga MPPT, lendo a partição 0 e a partição 1. 
Os dados lidos são publicados no servidor MQTT. 
Os dados da estação meteorológica são lidos e publicados no servidor MQTT.
No caso de falha de comunicação um tópico zerado com a informação OnLine=0 é publicado
O software entre em um sleep antes de reiniciar o processo.

### ModBus
A comunicação MODBUS segue o padrão RTU. 
As seguintes funções do ModBus são implementadas
- Read Registers = 0x03
- Write Single Register = 0x06
- Write Multiple Registers = 0x10

O cálculo do CRC utiliza o CRC-16 com o polinômio 0xA001

### MQTT
A conexão com o MQTT é feita utilizando a biblioteca "mosquitto"
As principais funções utilizadas são: 
- mosquitto_connect: efetua a conexão ao servidor MQTT
- mosquitto_publish: publica um tópico no servidor MQTT

## Parâmetros
O programa é parametrizável utilizando variáveis de ambiente. 
As seguintes variáveis de ambiente são utilizadas:
- SERIAL_PORT: Número da porta serial que será utilizada (/dev/verdin-uartX, onde X é o núemro da porta)
- SERVIDOR_MQTT: Endereço do servidor MQQT
- PORT_MQTT: Porta do servidor MQTT
- MQTT_USER: Usuário no servidor MQTT
- MQTT_PASS: Senha no servidor MQTT
- CLIENT_ID: Identifição do cliente no servidor MQTT
- TOPIC_PARTITION0: Tópico relativo a partição 0 do controlador de carga (MPPT)
- TOPIC_PARTITION1: Tópico relativo a partição 1 do controlador de carga (MPPT)
- TOPIC_WEATHER: Tópico relativo aos dados da Estação Meteorológica
- HAS_WEATHERSTATION: Indica se tem ou não estção meteorológica
- UPDATE_NTERVAL: Intervalo entre leituras
