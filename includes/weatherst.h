#ifndef WEATHER_ST_HEADER
#define WEATHER_ST_HEADER

#define WEATHER_ST_ADDRESS 0x02

/* Wheather station registers */
#define WEATHER_ST_DATA_SIZE 0x0F

#define REG_WIND_SPEED       0x01F4  // Wind Speed *10
#define REG_WIND_FORCE       0x01F5  // Wind Force
#define REG_WIND_DIR_GRADE   0x01F6  // Wind direction grade 0-7
#define REG_WIND_DIR_ANGLE   0x01F7  // Wind direction angle
#define REG_HUMIDITY         0x01F8  // Umidade *10
#define REG_TEMPERATURE      0x01F9  // Temperatura *10
#define REG_NOISE            0x01FA  // Noise value *10
#define REG_PM2_5            0x01FB  // PM2.5 value
#define REG_PM10             0x01FC  // PM10 value
#define REG_PRESSURE         0x01FD  // Atmospheric pressure *10
#define REG_W20_LUX          0x01FE  // 20W Lux value
#define REG_20W_REDIATION    0x0200  // 20W radiation Value
#define REG_RAIN_FALL        0x0201  // Rain fall value *10
#define REG_TOTAL_RADIATION  0x0202  // Total radiation value

// Estruturas de dados para armazernas informação da estação meteorologica
struct TWeatherStationData {
    uint16_t WindSpeed;
    uint16_t WindForce;
    uint16_t WindDirectionGrade;
    uint16_t WindDirectionAngle;
    uint16_t Humidity;
    uint16_t Temperature;
    uint16_t Noise;
    uint16_t PM2_5;
    uint16_t PM2_10;
    uint16_t AtmosphericPressure;
    uint32_t W20Lux;
    uint16_t W20Rediation;
    uint16_t Rainfall;
    uint16_t TotalRadiation;
}; // WeatherStationData


bool readWeatherStData(int fd, TWeatherStationData *data);


#endif