#ifndef MPPT_HEADER
#define MPPT_HEADER

#define MPPT_ADDRESS 0x01

// Estruturas de dados para armazernas informação das partiçoes 
struct Partition0Data {
    uint8_t MaxVoltage;
    uint8_t RatedCurrent;
    uint8_t RatedDischargeCurrent;
    uint8_t ProductType;
    uint8_t ProductSpecification[17];
    uint32_t SoftwareVersion;
    uint32_t HardwareVersion;
    uint32_t ProductSerialNumber;
    uint16_t DeviceAddress;
    uint8_t  onLine;
}; // Partition0Data

struct Partition1Data {
    uint8_t LoadState;
    uint8_t ChargeState;
    uint32_t Alarme_Failure;
    uint16_t BatterySOC;
    uint16_t BatteryVoltage;
    uint16_t ChargeCurrent;
    uint8_t DeviceTemperature;
    uint8_t BatteryTemperature;
    uint16_t LoadVoltage;
    uint16_t LoadCurrent;
    uint16_t LoadPower;
    uint16_t SolarPanelVoltage;
    uint16_t SolarPanelCurrent;
    uint16_t ChargePower;
    uint16_t LoadOnOff;
    uint16_t MinBatteryVoltageToday;
    uint16_t MaxBatteryVoltageToday;
    uint16_t MaxChargeCurrentToday;
    uint16_t MaxDischargeCurrentToday;   
    uint16_t MaxChargePowerToday;
    uint16_t MaxDischargePowerToday;
    uint16_t ChargeAmpereHourToday;
    uint16_t DischargeAmpereHourToday;
    uint16_t GeneratingCapacityToday;
    uint16_t ElectricityConsumedToday;
    uint16_t TotalOperatingDays;
    uint16_t TotalOverdischargetimes;
    uint16_t TotalBatteryChargeTimes;
    uint32_t TotalBatteryChargeAH;
    uint32_t TotalBatteryDischargeAH;
    uint32_t AccumulatedGeneratingCapacity;
    uint32_t AccumulatedElectricityConsuption;    
    uint32_t DummyFailureAlarme; 
    uint16_t MaxBatteryTemperatureToday;
    uint16_t MinBatteryTemperatureToday;
    uint32_t TotalLoadOperationTime;
    uint16_t Onduration;
    uint16_t OffDuration;
    uint16_t LightingIndex;
    uint16_t EnergyConsumption;
    uint16_t SystemHealthIndex;
    uint16_t ChargeDurationOnDay;
    uint16_t NightDuration;
    uint8_t  onLine;
}; // Partition1Data


bool readPartition0Data(int fd, Partition0Data *data);
bool readPartition1Data(int fd, Partition1Data *data);

#endif
