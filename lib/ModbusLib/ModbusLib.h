/////////////////////////////////////////////////////////////
// ModbusLib.h
// Modbus RTU library declarations for Waveshare ESP32 boards.
/////////////////////////////////////////////////////////////

#ifndef MODBUSLIB_H
#define MODBUSLIB_H

#include <Arduino.h>
#include <HardwareSerial.h>

#define MODBUS_TX_PIN 17
#define MODBUS_RX_PIN 18
#define MODBUS_RTS_PIN 21

enum class MB_Result : uint8_t {
    Ok,
    NotInitialized,
    InvalidArgument,
    Timeout,
    InvalidResponse,
    ModbusException,
    CommunicationError
};

enum class MB_Unit : uint8_t {
    Counts,
    Degrees,
    Radians,
    Rotations
};

struct MB_Telemetry {
    int32_t positionCounts;   // Position relative to the position at MB_Connect().
    float speedRpm;
    float currentAmps;
    uint16_t commandStatus;   // Register 0x0023.
    uint16_t faultCode;       // Register 0x0033.
    float pwmPercent;
    float busVoltageVolts;
};

// High-level functions.
// Simple use: MB_Connect(); MB_MoveTo(10);  // 10 degrees from connection position.
// A null address list uses the MB_Init() address. With an explicit list,
// relative=true moves each driver by target from its current position;
// relative=false sends each driver to the same absolute target coordinate.
MB_Result MB_Connect();
MB_Result MB_Disconnect();
MB_Result MB_SetupDriver(const uint8_t* addresses = nullptr,
                    size_t addressCount = 0);
MB_Result MB_MoveTo(float target,
                    MB_Unit unit = MB_Unit::Degrees,
                    const uint8_t* addresses = nullptr,
                    size_t addressCount = 0,
                    bool relative = true);
MB_Result MB_Stop(const uint8_t* addresses = nullptr, size_t addressCount = 0);
MB_Result MB_ReadPosition(float* position, MB_Unit unit = MB_Unit::Degrees);
MB_Result MB_ReadTelemetry(MB_Telemetry* telemetry);
MB_Result MB_SetPulsesPerRevolution(float pulsesPerRevolution);
MB_Result MB_SetDefaultMotion(float speed,
                              float acceleration,
                              MB_Unit unit = MB_Unit::Degrees);

// Mid-level AQMD3605BLS-B2 driver commands. Write calls accept an optional
// address list; nullptr/0 selects the MB_Init() default address.
MB_Result MB_SetSpeedLimit(float speed,
                           MB_Unit unit = MB_Unit::Degrees,
                           const uint8_t* addresses = nullptr,
                           size_t addressCount = 0);
MB_Result MB_SetAcceleration(float acceleration,
                             MB_Unit unit = MB_Unit::Degrees,
                             const uint8_t* addresses = nullptr,
                             size_t addressCount = 0);
MB_Result MB_ApplyMotionLimits(float speed,
                               float acceleration,
                               MB_Unit unit = MB_Unit::Degrees,
                               const uint8_t* addresses = nullptr,
                               size_t addressCount = 0);
MB_Result MB_ReadDriverStatus(uint16_t* status);
MB_Result MB_ReadFaultCode(uint16_t* faultCode);
MB_Result MB_SetAlarm(bool enabled,
                      const uint8_t* addresses = nullptr,
                      size_t addressCount = 0);

// Low-level Modbus RTU access.
MB_Result MB_Init(HardwareSerial& serial,
                  uint8_t slaveAddress,
                  uint32_t baudRate = 9600,
                  uint32_t serialConfig = SERIAL_8E1,
                  int8_t driverEnablePin = MODBUS_RTS_PIN);
int32_t MB_Read(uint8_t address, uint16_t registerAddress);
MB_Result MB_Write(uint8_t address, uint16_t registerAddress, uint16_t data);
MB_Result MB_ReadRegisters(uint8_t address,
                           uint16_t firstRegister,
                           uint16_t count,
                           uint16_t* data);

#endif // MODBUSLIB_H
