#include <Arduino.h>
#include <HardwareSerial.h>
#include <ModbusLib.h>
#include <math.h>

namespace {

constexpr uint32_t MB_RESPONSE_TIMEOUT_MS = 500;
constexpr uint8_t MB_DEFAULT_ADDRESS = 0x01;
constexpr uint32_t MB_DEFAULT_BAUD = 9600;
constexpr uint8_t MOTOR_POLES = 8;
constexpr float DEFAULT_PULSES_PER_REVOLUTION = 24.0f;
constexpr float DEFAULT_SPEED_DEGREES_PER_SECOND = 30.0f;
constexpr float DEFAULT_ACCEL_DEGREES_PER_SECOND_SQUARED = 60.0f;
constexpr double PI_VALUE = 3.14159265358979323846;

HardwareSerial* g_serial = nullptr;
int8_t g_driverEnablePin = -1;
uint8_t g_defaultAddress = MB_DEFAULT_ADDRESS;
bool g_connected = false;
int32_t g_rawZero = 0;
float g_pulsesPerRevolution = DEFAULT_PULSES_PER_REVOLUTION;
float g_defaultSpeed = DEFAULT_SPEED_DEGREES_PER_SECOND;
float g_defaultAcceleration = DEFAULT_ACCEL_DEGREES_PER_SECOND_SQUARED;
MB_Unit g_defaultMotionUnit = MB_Unit::Degrees;

uint16_t calculateCRC(const uint8_t* bytes, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= bytes[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x0001) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
        }
    }
    return crc;
}

bool readExact(uint8_t* buffer, size_t length) {
    const uint32_t start = millis();
    size_t received = 0;
    while (received < length && (millis() - start) < MB_RESPONSE_TIMEOUT_MS) {
        while (g_serial->available() > 0 && received < length) {
            const int value = g_serial->read();
            if (value >= 0) {
                buffer[received++] = static_cast<uint8_t>(value);
            }
        }
        if (received < length) {
            delay(1);
        }
    }
    return received == length;
}

void clearInputBuffer() {
    while (g_serial->available() > 0) {
        g_serial->read();
    }
}

MB_Result sendRequest(const uint8_t* request, size_t length) {
    if (g_serial == nullptr) {
        return MB_Result::NotInitialized;
    }

    clearInputBuffer();
    if (g_driverEnablePin >= 0) {
        digitalWrite(g_driverEnablePin, HIGH);
    }

    const size_t written = g_serial->write(request, length);
    g_serial->flush();

    if (g_driverEnablePin >= 0) {
        digitalWrite(g_driverEnablePin, LOW);
    }

    return written == length ? MB_Result::Ok : MB_Result::CommunicationError;
}

void appendCRC(uint8_t* frame, size_t dataLength) {
    const uint16_t crc = calculateCRC(frame, dataLength);
    frame[dataLength] = static_cast<uint8_t>(crc & 0xFF);
    frame[dataLength + 1] = static_cast<uint8_t>(crc >> 8);
}

bool hasValidCRC(const uint8_t* frame, size_t length) {
    if (length < 4) {
        return false;
    }
    const uint16_t actual = static_cast<uint16_t>(frame[length - 2]) |
                            (static_cast<uint16_t>(frame[length - 1]) << 8);
    return calculateCRC(frame, length - 2) == actual;
}

bool validAddress(uint8_t address) {
    return address >= 1 && address <= 247;
}

double radiansPerUnit(MB_Unit unit) {
    switch (unit) {
        case MB_Unit::Counts:
            return 2.0 * PI_VALUE / g_pulsesPerRevolution;
        case MB_Unit::Degrees:
            return PI_VALUE / 180.0;
        case MB_Unit::Radians:
            return 1.0;
        case MB_Unit::Rotations:
            return 2.0 * PI_VALUE;
        default:
            return 0.0;
    }
}

bool valueToCounts(float value, MB_Unit unit, int64_t* counts) {
    if (counts == nullptr || !isfinite(value)) {
        return false;
    }
    if (unit == MB_Unit::Counts) {
        *counts = static_cast<int64_t>(llround(value));
        return true;
    }

    const double unitRadians = radiansPerUnit(unit);
    if (unitRadians <= 0.0 || g_pulsesPerRevolution <= 0.0f) {
        return false;
    }
    const double turns = static_cast<double>(value) * unitRadians / (2.0 * PI_VALUE);
    *counts = static_cast<int64_t>(llround(turns * g_pulsesPerRevolution));
    return true;
}

float countsToValue(int32_t counts, MB_Unit unit) {
    if (unit == MB_Unit::Counts) {
        return static_cast<float>(counts);
    }

    const double unitRadians = radiansPerUnit(unit);
    if (unitRadians <= 0.0 || g_pulsesPerRevolution <= 0.0f) {
        return NAN;
    }
    const double turns = static_cast<double>(counts) / g_pulsesPerRevolution;
    return static_cast<float>(turns * (2.0 * PI_VALUE) / unitRadians);
}

double valueToRadiansPerSecond(float value, MB_Unit unit) {
    return static_cast<double>(value) * radiansPerUnit(unit);
}

bool speedToRegister(float speed, MB_Unit unit, uint16_t* raw) {
    if (raw == nullptr || !isfinite(speed) || speed <= 0.0f) {
        return false;
    }

    const double radiansPerSecond = valueToRadiansPerSecond(speed, unit);
    const double rpm = radiansPerSecond * 60.0 / (2.0 * PI_VALUE);
    const double commutationHz =
        radiansPerSecond * MOTOR_POLES / (20.0 * 2.0 * PI_VALUE) * 60.0;
    const long scaled = lround(commutationHz * 10.0);

    if (!isfinite(rpm) || rpm > 3000.0 || scaled < 1 || scaled > 32767) {
        return false;
    }
    *raw = static_cast<uint16_t>(scaled);
    return true;
}

bool accelerationToRegister(float acceleration, MB_Unit unit, uint16_t* raw) {
    if (raw == nullptr || !isfinite(acceleration) || acceleration <= 0.0f) {
        return false;
    }

    const double radiansPerSecondSquared =
        valueToRadiansPerSecond(acceleration, unit);
    const double commutationHz =
        radiansPerSecondSquared * MOTOR_POLES / (20.0 * 2.0 * PI_VALUE) * 60.0;
    const long scaled = lround(commutationHz * 10.0);

    if (!isfinite(commutationHz) || scaled < 1 || scaled > 65535) {
        return false;
    }
    *raw = static_cast<uint16_t>(scaled);
    return true;
}

bool validTargetList(const uint8_t* addresses, size_t count) {
    if (addresses == nullptr) return count == 0;
    if (count == 0 || count > 247) return false;
    for (size_t i = 0; i < count; ++i) {
        if (!validAddress(addresses[i])) return false;
        for (size_t j = 0; j < i; ++j) {
            if (addresses[i] == addresses[j]) return false;
        }
    }
    return true;
}

size_t resolvedTargetCount(const uint8_t* addresses, size_t count) {
    return addresses == nullptr ? 1 : count;
}

uint8_t resolvedTargetAddress(const uint8_t* addresses, size_t index) {
    return addresses == nullptr ? g_defaultAddress : addresses[index];
}

MB_Result firstError(MB_Result current, MB_Result next) {
    return current == MB_Result::Ok ? next : current;
}

MB_Result readPositionCounts(uint8_t address, int32_t* position) {
    if (position == nullptr) {
        return MB_Result::InvalidArgument;
    }

    uint16_t words[2] = {};
    const MB_Result result =
        MB_ReadRegisters(address, 0x0024, 2, words);
    if (result != MB_Result::Ok) {
        return result;
    }

    const uint32_t bits = (static_cast<uint32_t>(words[0]) << 16) | words[1];
    const int64_t signedValue = (bits & 0x80000000UL)
                                    ? static_cast<int64_t>(bits) - 0x100000000LL
                                    : static_cast<int64_t>(bits);
    *position = static_cast<int32_t>(signedValue);
    return MB_Result::Ok;
}

int32_t signedRegister(uint16_t value) {
    return (value & 0x8000) ? static_cast<int32_t>(value) - 0x10000
                            : static_cast<int32_t>(value);
}

MB_Result writeAcceleration(uint8_t address, uint16_t raw) {
    MB_Result result = MB_Write(address, 0x0062, raw);
    if (result != MB_Result::Ok) {
        return result;
    }

    // 0x0052 is the live acceleration override; 0x0062 is its configured ceiling.
    result = MB_Write(address, 0x0052, raw);
    if (result != MB_Result::Ok) {
        return result;
    }

    uint16_t confirmed = 0;
    result = MB_ReadRegisters(address, 0x0052, 1, &confirmed);
    if (result != MB_Result::Ok) {
        return result;
    }
    return confirmed == raw ? MB_Result::Ok : MB_Result::InvalidResponse;
}

} // namespace

MB_Result MB_Init(HardwareSerial& serial,
                  uint8_t slaveAddress,
                  uint32_t baudRate,
                  uint32_t serialConfig,
                  int8_t driverEnablePin) {
    if (!validAddress(slaveAddress) || baudRate == 0) {
        return MB_Result::InvalidArgument;
    }

    g_serial = &serial;
    g_defaultAddress = slaveAddress;
    g_driverEnablePin = driverEnablePin;
    g_connected = false;

    if (g_driverEnablePin >= 0) {
        pinMode(g_driverEnablePin, OUTPUT);
        digitalWrite(g_driverEnablePin, LOW);
    }

    g_serial->begin(baudRate, serialConfig, MODBUS_RX_PIN, MODBUS_TX_PIN);
    g_serial->setTimeout(MB_RESPONSE_TIMEOUT_MS);
    return MB_Result::Ok;
}

MB_Result MB_Connect() {
    if (g_connected) {
        return MB_Result::Ok;
    }

    if (g_serial == nullptr) {
        const MB_Result initResult =
            MB_Init(Serial1, MB_DEFAULT_ADDRESS, MB_DEFAULT_BAUD, SERIAL_8E1,
                    MODBUS_RTS_PIN);
        if (initResult != MB_Result::Ok) {
            return initResult;
        }
    }

    uint16_t deviceId = 0;
    MB_Result result = MB_ReadRegisters(g_defaultAddress, 0x0000, 1, &deviceId);
    if (result != MB_Result::Ok) {
        return result;
    }

    result = readPositionCounts(g_defaultAddress, &g_rawZero);
    if (result != MB_Result::Ok) {
        return result;
    }

    g_connected = true;
    return MB_Result::Ok;
}

MB_Result MB_Disconnect() {
    if (g_serial == nullptr) {
        return MB_Result::NotInitialized;
    }

    MB_Result result = MB_Result::Ok;
    if (g_connected) {
        result = MB_Write(g_defaultAddress, 0x0040, 0);
    }

    g_serial->end();
    g_serial = nullptr;
    g_driverEnablePin = -1;
    g_connected = false;
    return result;
}
MB_Result MB_SetupDriver(const uint8_t* addresses, size_t addressCount) {
    if (!g_connected) {
        return MB_Result::NotInitialized;
    }
    if (!validTargetList(addresses, addressCount)) {
        return MB_Result::InvalidArgument;
    }
    
    MB_Result overall = MB_Result::Ok;
    const size_t count = resolvedTargetCount(addresses, addressCount);
     for (size_t i = 0; i < count; ++i) {
        const uint8_t address = resolvedTargetAddress(addresses, i);
        MB_Result result = MB_Result::Ok;
        
        if (result == MB_Result::Ok) result = MB_Write(address, 0x0070, 0x1); // Enable Time-Position closed-loop control
        if (result == MB_Result::Ok) result = MB_Write(address, 0x0071, 0x0); // Set Allowed Error to 0 
        if (result == MB_Result::Ok) result = MB_Write(address, 0x0072, 0x1); // Enable Overshoot correction
        if (result == MB_Result::Ok) result = MB_Write(address, 0x0073, 0x8); // Set 8 Poles for the motor
        if (result == MB_Result::Ok) result = MB_Write(address, 0x0074, 0xA); // Set Reduction Ratio to 1:1
        if (result == MB_Result::Ok) result = MB_Write(address, 0x0093, 0x2); // Enable Position Closed-Loop Control
        if (result == MB_Result::Ok) result = MB_Write(address, 0x00b2, 0x1000); // Maximum Speed forwards is vMax
        if (result == MB_Result::Ok) result = MB_Write(address, 0x00b3, 0x1000); // Maximum Speed backwards is vMax

        overall = firstError(overall, result);
    }
    return overall;
    

}
MB_Result MB_MoveTo(float target, MB_Unit unit,
                    const uint8_t* addresses, size_t addressCount,
                    bool relative) {
    if (!g_connected) {
        return MB_Result::NotInitialized;
    }
    if (!validTargetList(addresses, addressCount)) {
        return MB_Result::InvalidArgument;
    }

    int64_t requestedCounts = 0;
    if (!valueToCounts(target, unit, &requestedCounts)) {
        return MB_Result::InvalidArgument;
    }

    uint16_t speedRaw = 0;
    uint16_t accelerationRaw = 0;
    if (!speedToRegister(g_defaultSpeed, g_defaultMotionUnit, &speedRaw) ||
        !accelerationToRegister(g_defaultAcceleration, g_defaultMotionUnit,
                                &accelerationRaw)) {
        return MB_Result::InvalidArgument;
    }

    MB_Result overall = MB_Result::Ok;
    const size_t count = resolvedTargetCount(addresses, addressCount);
    for (size_t i = 0; i < count; ++i) {
        const uint8_t address = resolvedTargetAddress(addresses, i);
        int64_t targetCounts = requestedCounts;
        MB_Result result = MB_Result::Ok;

        if (relative) {
            if (addresses == nullptr) {
                targetCounts += g_rawZero;
            } else {
                int32_t currentCounts = 0;
                result = readPositionCounts(address, &currentCounts);
                if (result == MB_Result::Ok) targetCounts += currentCounts;
            }
        }
        if (result == MB_Result::Ok &&
            (targetCounts < -2147483648LL || targetCounts > 2147483647LL)) {
            result = MB_Result::InvalidArgument;
        }
        if (result == MB_Result::Ok) result = MB_Write(address, 0x0066, speedRaw);
        if (result == MB_Result::Ok) result = writeAcceleration(address, accelerationRaw);
        if (result == MB_Result::Ok) result = MB_Write(address, 0x0044, speedRaw);
        if (result == MB_Result::Ok) result = MB_Write(address, 0x0045, 0);
        if (result == MB_Result::Ok) {
            const uint32_t bits = static_cast<uint32_t>(static_cast<int32_t>(targetCounts));
            result = MB_Write(address, 0x0046, static_cast<uint16_t>(bits >> 16));
            if (result == MB_Result::Ok) {
                result = MB_Write(address, 0x0047, static_cast<uint16_t>(bits & 0xFFFF));
            }
        }
        overall = firstError(overall, result);
    }
    return overall;
}

MB_Result MB_Stop(const uint8_t* addresses, size_t addressCount) {
    if (!g_connected) {
        return MB_Result::NotInitialized;
    }
    if (!validTargetList(addresses, addressCount)) return MB_Result::InvalidArgument;
    MB_Result overall = MB_Result::Ok;
    const size_t count = resolvedTargetCount(addresses, addressCount);
    for (size_t i = 0; i < count; ++i) {
        overall = firstError(overall,
            MB_Write(resolvedTargetAddress(addresses, i), 0x0040, 0));
    }
    return overall;
}

MB_Result MB_ReadPosition(float* position, MB_Unit unit) {
    if (!g_connected) {
        return MB_Result::NotInitialized;
    }
    if (position == nullptr) {
        return MB_Result::InvalidArgument;
    }

    int32_t rawPosition = 0;
    const MB_Result result = readPositionCounts(g_defaultAddress, &rawPosition);
    if (result != MB_Result::Ok) {
        return result;
    }
    *position = countsToValue(rawPosition - g_rawZero, unit);
    return isfinite(*position) ? MB_Result::Ok : MB_Result::InvalidArgument;
}

MB_Result MB_ReadTelemetry(MB_Telemetry* telemetry) {
    if (!g_connected) {
        return MB_Result::NotInitialized;
    }
    if (telemetry == nullptr) {
        return MB_Result::InvalidArgument;
    }

    uint16_t registers[25] = {};
    const MB_Result result =
        MB_ReadRegisters(g_defaultAddress, 0x0020, 25, registers);
    if (result != MB_Result::Ok) {
        return result;
    }

    const uint32_t positionBits =
        (static_cast<uint32_t>(registers[4]) << 16) | registers[5];
    const int64_t signedPosition =
        (positionBits & 0x80000000UL)
            ? static_cast<int64_t>(positionBits) - 0x100000000LL
            : static_cast<int64_t>(positionBits);
    telemetry->positionCounts =
        static_cast<int32_t>(signedPosition - g_rawZero);

    const int32_t hallRaw = signedRegister(registers[2]);
    const uint16_t speedRaw = registers[20];
    const uint16_t speedScale = registers[21];
    double rpm = speedRaw * (speedScale == 1 ? 10.0 : 1.0);

    if (speedRaw == 0xFFFF) {
        const double hallHz = hallRaw * (speedScale == 1 ? 1.0 : 0.1);
        rpm = hallHz / MOTOR_POLES * 20.0;
    }
    if (hallRaw < 0) {
        rpm = -fabs(rpm);
    }

    telemetry->speedRpm = static_cast<float>(rpm);
    telemetry->currentAmps = registers[1] * 0.01f;
    telemetry->commandStatus = registers[3];
    telemetry->faultCode = registers[19];
    telemetry->pwmPercent = registers[0] * 0.1f;
    telemetry->busVoltageVolts = registers[24] * 0.1f;
    return MB_Result::Ok;
}

MB_Result MB_SetPulsesPerRevolution(float pulsesPerRevolution) {
    if (!isfinite(pulsesPerRevolution) || pulsesPerRevolution <= 0.0f) {
        return MB_Result::InvalidArgument;
    }
    g_pulsesPerRevolution = pulsesPerRevolution;
    return MB_Result::Ok;
}

MB_Result MB_SetDefaultMotion(float speed,
                              float acceleration,
                              MB_Unit unit) {
    uint16_t speedRaw = 0;
    uint16_t accelerationRaw = 0;
    if (!speedToRegister(speed, unit, &speedRaw) ||
        !accelerationToRegister(acceleration, unit, &accelerationRaw)) {
        return MB_Result::InvalidArgument;
    }

    g_defaultSpeed = speed;
    g_defaultAcceleration = acceleration;
    g_defaultMotionUnit = unit;
    return MB_Result::Ok;
}

MB_Result MB_SetSpeedLimit(float speed, MB_Unit unit,
                           const uint8_t* addresses, size_t addressCount) {
    if (!g_connected) {
        return MB_Result::NotInitialized;
    }
    if (!validTargetList(addresses, addressCount)) return MB_Result::InvalidArgument;

    uint16_t raw = 0;
    if (!speedToRegister(speed, unit, &raw)) {
        return MB_Result::InvalidArgument;
    }
    MB_Result overall = MB_Result::Ok;
    const size_t count = resolvedTargetCount(addresses, addressCount);
    for (size_t i = 0; i < count; ++i) {
        overall = firstError(overall,
            MB_Write(resolvedTargetAddress(addresses, i), 0x0066, raw));
    }
    return overall;
}

MB_Result MB_SetAcceleration(float acceleration, MB_Unit unit,
                             const uint8_t* addresses, size_t addressCount) {
    if (!g_connected) {
        return MB_Result::NotInitialized;
    }
    if (!validTargetList(addresses, addressCount)) return MB_Result::InvalidArgument;

    uint16_t raw = 0;
    if (!accelerationToRegister(acceleration, unit, &raw)) {
        return MB_Result::InvalidArgument;
    }
    MB_Result overall = MB_Result::Ok;
    const size_t count = resolvedTargetCount(addresses, addressCount);
    for (size_t i = 0; i < count; ++i) {
        overall = firstError(overall,
            writeAcceleration(resolvedTargetAddress(addresses, i), raw));
    }
    return overall;
}

MB_Result MB_ApplyMotionLimits(float speed,
                               float acceleration,
                               MB_Unit unit,
                               const uint8_t* addresses, size_t addressCount) {
    if (!g_connected) {
        return MB_Result::NotInitialized;
    }
    if (!validTargetList(addresses, addressCount)) return MB_Result::InvalidArgument;

    uint16_t speedRaw = 0;
    uint16_t accelerationRaw = 0;
    if (!speedToRegister(speed, unit, &speedRaw) ||
        !accelerationToRegister(acceleration, unit, &accelerationRaw)) {
        return MB_Result::InvalidArgument;
    }

    MB_Result overall = MB_Result::Ok;
    const size_t count = resolvedTargetCount(addresses, addressCount);
    for (size_t i = 0; i < count; ++i) {
        const uint8_t address = resolvedTargetAddress(addresses, i);
        MB_Result result = MB_Write(address, 0x0066, speedRaw);
        if (result == MB_Result::Ok) {
            result = writeAcceleration(address, accelerationRaw);
        }
        overall = firstError(overall, result);
    }
    return overall;
}

MB_Result MB_SetAlarm(bool enabled, const uint8_t* addresses,
                      size_t addressCount) {
    if (!g_connected) {
        return MB_Result::NotInitialized;
    }
    if (!validTargetList(addresses, addressCount)) return MB_Result::InvalidArgument;

    const uint16_t expectedValue = enabled ? 0x0000 : 0x0001;
    MB_Result overall = MB_Result::Ok;
    const size_t count = resolvedTargetCount(addresses, addressCount);
    for (size_t i = 0; i < count; ++i) {
        const uint8_t address = resolvedTargetAddress(addresses, i);
        MB_Result result = MB_Write(address, 0x0099, expectedValue);
        uint16_t actualValue = 0;
        if (result == MB_Result::Ok) {
            result = MB_ReadRegisters(address, 0x0099, 1, &actualValue);
        }
        if (result == MB_Result::Ok && actualValue != expectedValue) {
            result = MB_Result::InvalidResponse;
        }
        overall = firstError(overall, result);
    }
    return overall;
}
MB_Result MB_ReadDriverStatus(uint16_t* status) {
    if (!g_connected) {
        return MB_Result::NotInitialized;
    }
    if (status == nullptr) {
        return MB_Result::InvalidArgument;
    }
    return MB_ReadRegisters(g_defaultAddress, 0x0023, 1, status);
}

MB_Result MB_ReadFaultCode(uint16_t* faultCode) {
    if (!g_connected) {
        return MB_Result::NotInitialized;
    }
    if (faultCode == nullptr) {
        return MB_Result::InvalidArgument;
    }
    return MB_ReadRegisters(g_defaultAddress, 0x0033, 1, faultCode);
}

int32_t MB_Read(uint8_t address, uint16_t registerAddress) {
    uint16_t value = 0;
    const MB_Result result = MB_ReadRegisters(address, registerAddress, 1, &value);
    return result == MB_Result::Ok ? static_cast<int32_t>(value) : -1;
}

MB_Result MB_Write(uint8_t address, uint16_t registerAddress, uint16_t data) {
    if (g_serial == nullptr) {
        return MB_Result::NotInitialized;
    }
    if (!validAddress(address)) {
        return MB_Result::InvalidArgument;
    }

    uint8_t request[8] = {
        address,
        0x06,
        static_cast<uint8_t>(registerAddress >> 8),
        static_cast<uint8_t>(registerAddress & 0xFF),
        static_cast<uint8_t>(data >> 8),
        static_cast<uint8_t>(data & 0xFF),
        0,
        0
    };
    appendCRC(request, 6);

    MB_Result result = sendRequest(request, sizeof(request));
    if (result != MB_Result::Ok) {
        return result;
    }

    uint8_t response[8] = {};
    if (!readExact(response, 2)) {
        return MB_Result::Timeout;
    }

    if (response[1] == static_cast<uint8_t>(0x06 | 0x80)) {
        if (!readExact(response + 2, 3)) {
            return MB_Result::Timeout;
        }
        return hasValidCRC(response, 5) && response[0] == address
                   ? MB_Result::ModbusException
                   : MB_Result::InvalidResponse;
    }

    if (response[1] != 0x06 || !readExact(response + 2, 6)) {
        return response[1] == 0x06 ? MB_Result::Timeout : MB_Result::InvalidResponse;
    }
    if (response[0] != address || !hasValidCRC(response, sizeof(response))) {
        return MB_Result::InvalidResponse;
    }

    for (size_t i = 0; i < 6; ++i) {
        if (response[i] != request[i]) {
            return MB_Result::InvalidResponse;
        }
    }

    return MB_Result::Ok;
}

MB_Result MB_ReadRegisters(uint8_t address,
                           uint16_t firstRegister,
                           uint16_t count,
                           uint16_t* data) {
    if (g_serial == nullptr) {
        return MB_Result::NotInitialized;
    }
    if (!validAddress(address) || count == 0 || count > 125 || data == nullptr ||
        static_cast<uint32_t>(firstRegister) + count > 0x10000UL) {
        return MB_Result::InvalidArgument;
    }

    uint8_t request[8] = {
        address,
        0x03,
        static_cast<uint8_t>(firstRegister >> 8),
        static_cast<uint8_t>(firstRegister & 0xFF),
        static_cast<uint8_t>(count >> 8),
        static_cast<uint8_t>(count & 0xFF),
        0,
        0
    };
    appendCRC(request, 6);

    MB_Result result = sendRequest(request, sizeof(request));
    if (result != MB_Result::Ok) {
        return result;
    }

    uint8_t response[5 + 125 * 2] = {};
    if (!readExact(response, 3)) {
        return MB_Result::Timeout;
    }
    if (response[0] != address) {
        return MB_Result::InvalidResponse;
    }

    if (response[1] == static_cast<uint8_t>(0x03 | 0x80)) {
        if (!readExact(response + 3, 2)) {
            return MB_Result::Timeout;
        }
        return hasValidCRC(response, 5) ? MB_Result::ModbusException
                                        : MB_Result::InvalidResponse;
    }
    if (response[1] != 0x03 || response[2] != count * 2) {
        return MB_Result::InvalidResponse;
    }

    const size_t remaining = static_cast<size_t>(response[2]) + 2;
    if (!readExact(response + 3, remaining)) {
        return MB_Result::Timeout;
    }

    const size_t responseLength = 3 + remaining;
    if (!hasValidCRC(response, responseLength)) {
        return MB_Result::InvalidResponse;
    }

    for (uint16_t i = 0; i < count; ++i) {
        const size_t index = 3 + static_cast<size_t>(i) * 2;
        data[i] = (static_cast<uint16_t>(response[index]) << 8) |
                  static_cast<uint16_t>(response[index + 1]);
    }

    return MB_Result::Ok;
}
