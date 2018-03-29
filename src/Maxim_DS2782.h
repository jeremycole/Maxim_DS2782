#ifndef MAXIM_DS2782_H
#define MAXIM_DS2782_H

class Maxim_DS2782 {
private:
  enum Registers {
    // Name                              Acc.   Size  Description
    // Reserved, 0x00
    Register_STATUS           = 0x01, // R/W    1     Status
    Register_RAAC             = 0x02, // R      2     Remaining Active Absolute Capacity
    Register_RSAC             = 0x04, // R      2     Remaining Standby Absolute Capacity
    Register_RARC             = 0x06, // R      1     Remaining Active Relative Capacity
    Register_RSRC             = 0x07, // R      1     Remaining Standby Relative Capacity
    Register_IAVG             = 0x08, // R      2     Average Current
    Register_TEMP             = 0x0a, // R      2     Temperature
    Register_VOLT             = 0x0c, // R      2     Voltage
    Register_CURRENT          = 0x0e, // R      2     Current
    Register_ACR              = 0x10, // R/W    2     Accumulated Current (auto-saved)
    Register_ACRL             = 0x12, // R      2     Low Accumulated Current
    Register_AS               = 0x14, // R/W    1     Age Scalar (auto-saved)
    Register_SFR              = 0x15, // R/W    1     Special Feature Register
    Register_FULL             = 0x16, // R      2     Full Capacity
    Register_AE               = 0x18, // R      2     Active Empty Capacity
    Register_SE               = 0x1a, // R      2     Standby Empty Capacity
    // Reserved, 0x1c - 0x1e
    Register_EEPROM           = 0x1f, // R/W    1     EEPROM, Block 0
    Register_User_EEPROM_1    = 0x20, // R/W    16    User EEPROM, Lockable, Block 0
    Register_User_EEPROM_2    = 0x30, // R/W    8     Additional User EEPROM, Lockable, Block 0
    // Reserved, 0x38 - 0x5f
    Register_Parameter_EEPROM = 0x60, // R/W    32    Parameter EEPROM, Lockable, Block 1
    // Reserved, 0x80 - 0xef
    Register_Unique_ID        = 0xf0, // R      8     Unique ID (factory option)
    // Reserved, 0xf8 - 0xfd
    Register_Function_Command = 0xfe, // W      1     Function Command
    // Reserved, 0xff
  };

  enum ParameterRegisters {
    Parameter_CONTROL         = 0x60, // Control Register
    Parameter_AB              = 0x61, // Accumulation Bias
    Parameter_AC              = 0x62, // Aging Capacity (2 bytes)
    Parameter_VCHG            = 0x64, // Charge Voltage
    Parameter_IMIN            = 0x65, // Minimum Charge Current
    Parameter_VAE             = 0x66, // Active Empty Voltage
    Parameter_IAE             = 0x67, // Active Empty Current
    Parameter_AE_40           = 0x68, // Active Empty 40
    Parameter_RSNSP           = 0x69, // Sense Resistor Prime
    Parameter_Full_40         = 0x6a, // Full 40 (2 bytes)
    Parameter_Full_Slope      = 0x6c, // Full Slope (4 bytes)
    Parameter_AE_Slope        = 0x70, // Active Empty Slope (4 bytes)
    Parameter_SE_Slope        = 0x74, // Standby Empty Slope (4 bytes)
    Parameter_RSGAIN          = 0x78, // Sense Resistor Gain (2 bytes)
    Parameter_RSTC            = 0x7a, // Sense Resistor Temperature Coefficient
    Parameter_FRSGAIN         = 0x7b, // Factory Gain (2 bytes)
    // Reserved, 0x7d
    Parameter_Address         = 0x7e, // 2-Wire Slave Address
    // Reserved, 0x7f
  };

public:

  static const uint8_t DEFAULT_I2C_ADDRESS = 0x34;

  struct StatusRegister {
    unsigned char CHGTF:1;      // R    Charge Termination Flag
    unsigned char AEF:1;        // R    Active Empty Flag
    unsigned char SEF:1;        // R    Standby Empty Flag
    unsigned char LEARNF:1;     // R    Learn Flag
    unsigned char Reserved3:1;
    unsigned char UVF:1;        // R/W  Under-Voltage Flag
    unsigned char PORFx:1;       // R/W  Power-on Reset Flag
    unsigned char Reserved0:1;
  };

  struct StatusResultData {
    StatusRegister STATUS;
    uint16_t RAAC;
    uint16_t RSAC;
    uint8_t RARC;
    uint8_t RSRC;
    int16_t IAVG;
    int16_t TEMP;
    int16_t VOLT;
    int16_t CURRENT;
    uint16_t ACR;
    uint16_t ACRL;
  };

  struct ControlRegister {
    unsigned char Reserved7:1;
    unsigned char UVEN:1;       // R/W  Under-Voltage Sleep Enable
    unsigned char PMOD:1;       // R/W  Power Mode Enable
    unsigned char Reserved43210:5;
  };

  struct SpecialFeatureRegister {
    unsigned char Reserved765432:6;
    unsigned char SAWE:1;       // R/W  Slave Address Write Enable
    unsigned char PIOSC:1;      // R/W  PIO Sense and Control
  };

  struct EEPROMRegister {
    unsigned char EEC:1;        // R    EEPROM Copy Flag
    unsigned char LOCK:1;       // R/W  EEPROM Lock Enable
    unsigned char Reserved65432:5;
    unsigned char BL1:1;        // R    EEPROM Block 1 Lock Flag
    unsigned char BL0:1;        // R    EEPROM Block 0 Lock Flag
  };

  enum EepromBlock {
    Block_0 = 0,
    Block_1 = 1,
  };

  enum FunctionCommands {
    Command_Copy_Block_0    = 0x42,
    Command_Copy_Block_1    = 0x44,
    Command_Recall_Block_0  = 0xb2,
    Command_Recall_Block_1  = 0xb4,
    Command_Lock_Block_0    = 0x63,
    Command_Lock_Block_1    = 0x66,
  };

private:

  TwoWire *bus;
  uint8_t i2c_address;

  float Rsns_ohm;
  int Rsns_mho;

  // 5.0 V, 16 bits total, 1 sign, 10 decimal, 5 reserved
  static constexpr float VOLTAGE_FACTOR = 5.0 / (float)((1 << 10) << 5);

  // 1.0 °C, 16 bits total, 1 sign, 7 integer, 3 decimal, 5 reserved
  static constexpr float TEMPERATURE_FACTOR = 1.0 / (float)((1 << 3) << 5);

  // 0.1 mVh, 16 bits total, 1 sign, 9 integer, 6 decimal
  static constexpr float CURRENT_FACTOR = 0.1 / ((float)(1 << 6));

  // 0.1 mVh, 16 bits total, 12 integer, 4 decimal
  static constexpr float ACCUMULATED_CURRENT_FACTOR = 0.1 / (float)(1 << 4);

  // 0.1 mVh, 16 bits total, 6 integer, 6 decimal, 4 reserved
  static constexpr float LOW_ACCUMULATED_CURRENT_FACTOR = 0.1 / (float)((1 << 6) << 4);

  // 0.1 mVh, 8 bits total, 1 sign, 1 integer, 6 decimal
  static constexpr float ACCUMULATION_BIAS_FACTOR = 0.1 / (float)(1 << 6);

  // 1.6 mAh, 16 bits total, 16 integer
  // TODO: The datasheet says 1.6 mAh, but is this really 1.5625 mAh units? Probably.
  static constexpr float ABSOLUTE_CAPACITY_FACTOR = 1.6;

  // Convert raw register value for VOLT and return in V.
  float transferVoltage(int16_t raw_value) {
    return (float)raw_value * VOLTAGE_FACTOR;
  }

  // Convert raw register value for TEMP and return in °C.
  float transferTemperature(int16_t raw_value) {
    return (float)raw_value * TEMPERATURE_FACTOR;
  }

  // Convert raw register value for CURRENT, IAVG and return in mA.
  float transferCurrent(int16_t raw_value) {
    return ((float)raw_value * CURRENT_FACTOR) / Rsns_ohm;
  }

  // Convert raw register value for ACR and return in mAh.
  float transferAccumulatedCurrent(uint16_t raw_value) {
    return ((float)raw_value * ACCUMULATED_CURRENT_FACTOR) / Rsns_ohm;
  }

  // Convert raw register value for ACRL and return in mAh.
  float transferLowAccumulatedCurrent(uint16_t raw_value) {
    return ((float)raw_value * LOW_ACCUMULATED_CURRENT_FACTOR) / Rsns_ohm;
  }

  // Convert raw register value for AB and return in mAh.
  float transferAccumulationBias(int8_t raw_value) {
    return ((float)raw_value * ACCUMULATION_BIAS_FACTOR) / Rsns_ohm;
  }

  // Convert raw register value for RAAC, RSAC and return in mAh.
  float transferAbsoluteCapacity(uint16_t raw_value) {
    return (float)raw_value * ABSOLUTE_CAPACITY_FACTOR;
  }

  uint8_t read_uint8(uint8_t register_address) {
    bus->beginTransmission(i2c_address);
    bus->write(register_address);
    bus->endTransmission();

    bus->requestFrom(i2c_address, (uint8_t) 1);
    uint8_t value = bus->read();
    bus->endTransmission();

    return value;
  }

  int16_t read_int16(uint8_t register_address) {
    bus->beginTransmission(i2c_address);
    bus->write(register_address);
    bus->endTransmission();

    bus->requestFrom(i2c_address, (uint8_t) 2);
    uint16_t value = bus->read() << 8;
    value |= bus->read();
    bus->endTransmission();

    return *(int16_t *)&value;
  }

  uint16_t read_uint16(uint8_t register_address) {
    bus->beginTransmission(i2c_address);
    bus->write(register_address);
    bus->endTransmission();

    bus->requestFrom(i2c_address, (uint8_t) 2);
    uint16_t value = bus->read() << 8;
    value |= bus->read();
    bus->endTransmission();

    return value;
  }

  void write_uint8(uint8_t register_address, uint8_t value) {
    bus->beginTransmission(i2c_address);

    bus->write(register_address);
    bus->write(value);

    bus->endTransmission();
  }

  void write_uint16(uint8_t register_address, uint16_t value) {
    bus->beginTransmission(i2c_address);

    bus->write(register_address);
    bus->write((uint8_t)((value & 0xff00) >> 8));
    bus->write((uint8_t)(value & 0x00ff));

    bus->endTransmission();
  }

public:
  Maxim_DS2782(TwoWire *bus, uint8_t i2c_address, float Rsns_ohm) {
    this->bus = bus;
    this->i2c_address = i2c_address;
    this->Rsns_ohm = Rsns_ohm;
    this->Rsns_mho = (int) (1.0 / Rsns_ohm);
  }


  StatusRegister readStatus() {
    uint8_t value = read_uint8(Registers::Register_STATUS);

    StatusRegister status;
    *(char *)&status = value;

    return status;
  }

  void doFunctionCommand(FunctionCommands command) {
    write_uint8(Registers::Register_Function_Command, command);
  }

  void doCopyData(uint8_t block) {
    switch (block) {
    case 0:
      doFunctionCommand(FunctionCommands::Command_Copy_Block_0);
      return;
    case 1:
      doFunctionCommand(FunctionCommands::Command_Copy_Block_1);
      return;
    }
  }

  void doRecallData(uint8_t block) {
    switch (block) {
    case 0:
      doFunctionCommand(FunctionCommands::Command_Copy_Block_0);
      return;
    case 1:
      doFunctionCommand(FunctionCommands::Command_Copy_Block_1);
      return;
    }
  }

  float readTemperature() {
    return transferTemperature(read_int16(Registers::Register_TEMP));
  }

  float readVoltage() {
    return -transferVoltage(read_int16(Registers::Register_VOLT));
  }

  float readCurrent() {
    return transferCurrent(read_int16(Registers::Register_CURRENT));
  }

  float readCurrentSenseResistorVoltage() {
    return read_int16(Registers::Register_CURRENT) * CURRENT_FACTOR;
  }

  float readRemainingActiveAbsoluteCapacity() {
    return transferAbsoluteCapacity(read_int16(Registers::Register_RAAC));
  }

  float readRemainingStandbyAbsoluteCapacity() {
    return transferAbsoluteCapacity(read_int16(Registers::Register_RSAC));
  }

  uint8_t readRemainingActiveRelativeCapacity() {
    return read_uint8(Registers::Register_RARC);
  }

  uint8_t readRemainingStandbyRelativeCapacity() {
    return read_uint8(Registers::Register_RSRC);
  }

  uint8_t readVCHG() {
    return read_uint8(ParameterRegisters::Parameter_VCHG);
  }

  void writeVCHG(uint8_t value) {
    write_uint8(ParameterRegisters::Parameter_VCHG, value);
  }

  uint8_t readIMIN() {
    return read_uint8(ParameterRegisters::Parameter_IMIN);
  }

  void writeIMIN(uint8_t value) {
    write_uint8(ParameterRegisters::Parameter_IMIN, value);
  }

  uint8_t readVAE() {
    return read_uint8(ParameterRegisters::Parameter_VAE);
  }

  void writeVAE(uint8_t value) {
    write_uint8(ParameterRegisters::Parameter_VAE, value);
  }

  uint8_t readIAE() {
    return read_uint8(ParameterRegisters::Parameter_IAE);
  }

  void writeIAE(uint8_t value) {
    write_uint8(ParameterRegisters::Parameter_IAE, value);
  }

  uint8_t readRSNSP() {
    return read_uint8(ParameterRegisters::Parameter_RSNSP);
  }

  void writeRSNSP(uint8_t value) {
    write_uint8(ParameterRegisters::Parameter_RSNSP, value);
  }

  uint16_t readRSGAIN() {
    return read_uint16(ParameterRegisters::Parameter_RSGAIN);
  }

  void writeRSGAIN(uint16_t value) {
    write_uint16(ParameterRegisters::Parameter_RSGAIN, value);
  }

  uint16_t readFRSGAIN() {
    return read_uint16(ParameterRegisters::Parameter_FRSGAIN);
  }

  uint16_t readUserEEPROM_uint16(uint8_t offset) {
    return read_uint16(Registers::Register_User_EEPROM_1 + offset);
  }

  void writeUserEEPROM_uint16(uint8_t offset, uint16_t value) {
    write_uint16(Registers::Register_User_EEPROM_1 + offset, value);
  }
};

#endif // MAXIM_DS2782_H
