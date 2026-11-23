#include <Arduino.h>
#include "ModbusLib.h"

namespace {

constexpr uint8_t DRIVER_ADDRESS = 0x01;
constexpr uint16_t ID_REGISTER = 0x0000;
constexpr float TEST_SPEED_DEGREES_PER_SECOND = 18000.0f;
constexpr float TEST_ACCEL_DEGREES_PER_SECOND_SQUARED = 98302.5f;
constexpr float TEST_TARGET_DEGREES = 9000.0f;

bool allPassed = true;
bool serialInitialized = false;
bool highLevelConnected = false;
bool settingsSaved = false;

uint16_t originalSpeedLimit = 0;
uint16_t originalAccelerationLimit = 0;
uint16_t originalLiveAcceleration = 0;

const char* resultName(MB_Result result) {
    switch (result) {
        case MB_Result::Ok: return "Ok";
        case MB_Result::NotInitialized: return "NotInitialized";
        case MB_Result::InvalidArgument: return "InvalidArgument";
        case MB_Result::Timeout: return "Timeout";
        case MB_Result::InvalidResponse: return "InvalidResponse";
        case MB_Result::ModbusException: return "ModbusException";
        case MB_Result::CommunicationError: return "CommunicationError";
        default: return "Unknown";
    }
}

void reportCheck(const char* name, bool passed, const char* detail = nullptr) {
    Serial.printf("[%s] %s", passed ? "PASS" : "FAIL", name);
    if (detail != nullptr && detail[0] != '\0') {
        Serial.printf(" -- %s", detail);
    }
    Serial.println();
    if (!passed) {
        allPassed = false;
    }
}

void reportResult(const char* name, MB_Result result) {
    char detail[64];
    snprintf(detail, sizeof(detail), "MB_Result=%s (%u)",
             resultName(result), static_cast<unsigned>(result));
    reportCheck(name, result == MB_Result::Ok, detail);
}

MB_Result readSingle(uint16_t reg, uint16_t* value) {
    return MB_ReadRegisters(DRIVER_ADDRESS, reg, 1, value);
}

void restoreDriverSettings() {
    if (!settingsSaved) {
        return;
    }

    Serial.println();
    Serial.println("Restoring the original motion-limit registers...");
    reportResult("Restore register 0x0066",
                 MB_Write(DRIVER_ADDRESS, 0x0066, originalSpeedLimit));
    reportResult("Restore register 0x0062",
                 MB_Write(DRIVER_ADDRESS, 0x0062, originalAccelerationLimit));
    reportResult("Restore register 0x0052",
                 MB_Write(DRIVER_ADDRESS, 0x0052, originalLiveAcceleration));
}

void finishTest() {
    if (highLevelConnected) {
        reportResult("MB_Stop (cleanup)", MB_Stop());
    }

    restoreDriverSettings();

    if (serialInitialized) {
        reportResult("MB_Disconnect", MB_Disconnect());
        serialInitialized = false;
        highLevelConnected = false;

        const bool rejectedAfterDisconnect = MB_Read(DRIVER_ADDRESS, ID_REGISTER) == -1;
        reportCheck("MB_Read after disconnect is rejected", rejectedAfterDisconnect);
    }

    Serial.println();
    Serial.println(allPassed
                       ? "========== ALL TESTS PASSED =========="
                       : "========== SOME TESTS FAILED =========");
    Serial.println("Copy this complete Serial Monitor output when asking for help.");
}

bool saveOriginalSettings() {
    MB_Result result = readSingle(0x0066, &originalSpeedLimit);
    reportResult("Read original speed limit 0x0066", result);
    if (result != MB_Result::Ok) {
        return false;
    }

    result = readSingle(0x0062, &originalAccelerationLimit);
    reportResult("Read original acceleration limit 0x0062", result);
    if (result != MB_Result::Ok) {
        return false;
    }

    result = readSingle(0x0052, &originalLiveAcceleration);
    reportResult("Read original live acceleration 0x0052", result);
    if (result != MB_Result::Ok) {
        return false;
    }

    settingsSaved = true;
    return true;
}

void runTests() {
    Serial.println();
    Serial.println("1) Low-level communication");

    const int32_t deviceId = MB_Read(DRIVER_ADDRESS, ID_REGISTER);
    reportCheck("MB_Read register 0x0000",
                deviceId >= 0,
                deviceId >= 0 ? "slave 0x01 answered" : "read failed");
    if (deviceId >= 0) {
        Serial.printf("    Device ID: 0x%04lX (%ld)\n",
                      static_cast<unsigned long>(deviceId),
                      static_cast<long>(deviceId));
    }

    uint16_t idFromBlockRead = 0;
    MB_Result result = readSingle(ID_REGISTER, &idFromBlockRead);
    reportResult("MB_ReadRegisters register 0x0000", result);
    if (result == MB_Result::Ok) {
        Serial.printf("    Device ID: 0x%04X (%u)\n",
                      idFromBlockRead, idFromBlockRead);
    }

    if (deviceId < 0 || result != MB_Result::Ok) {
        return;
    }

    Serial.println();
    Serial.println("2) Low-level write (writes back the current speed value)");
    if (!saveOriginalSettings()) {
        return;
    }

    result = MB_Write(DRIVER_ADDRESS, 0x0066, originalSpeedLimit);
    reportResult("MB_Write register 0x0066", result);
    if (result == MB_Result::Ok) {
        const int32_t readBack = MB_Read(DRIVER_ADDRESS, 0x0066);
        const bool matches = readBack == originalSpeedLimit;
        reportCheck("Read back written value",
                    matches,
                    matches ? "value matches" : "value did not match");
    }

    Serial.println();
    Serial.println("3) High-level connection and read functions");
    result = MB_Connect();
    reportResult("MB_Connect", result);
    if (result != MB_Result::Ok) {
        return;
    }
    highLevelConnected = true;

    reportResult("MB_SetPulsesPerRevolution(24)",
                 MB_SetPulsesPerRevolution(24.0f));
    reportResult("MB_SetDefaultMotion(18000, 98302.5)",
                 MB_SetDefaultMotion(TEST_SPEED_DEGREES_PER_SECOND,
                                     TEST_ACCEL_DEGREES_PER_SECOND_SQUARED));

    float positionDegrees = 0.0f;
    result = MB_ReadPosition(&positionDegrees);
    reportResult("MB_ReadPosition", result);
    if (result == MB_Result::Ok) {
        Serial.printf("    Position: %.3f degrees from MB_Connect()\n",
                      positionDegrees);
    }

    MB_Telemetry telemetry = {};
    result = MB_ReadTelemetry(&telemetry);
    reportResult("MB_ReadTelemetry", result);
    if (result == MB_Result::Ok) {
        Serial.printf("    Position: %ld counts, speed: %.2f RPM\n",
                      static_cast<long>(telemetry.positionCounts),
                      telemetry.speedRpm);
        Serial.printf("    Current: %.2f A, PWM: %.1f%%, bus: %.1f V\n",
                      telemetry.currentAmps, telemetry.pwmPercent,
                      telemetry.busVoltageVolts);
        Serial.printf("    Command status: 0x%04X, fault: 0x%04X\n",
                      telemetry.commandStatus, telemetry.faultCode);
    }

    uint16_t status = 0;
    result = MB_ReadDriverStatus(&status);
    reportResult("MB_ReadDriverStatus", result);
    if (result == MB_Result::Ok) {
        Serial.printf("    Driver status: 0x%04X (%u)\n", status, status);
    }

    uint16_t faultCode = 0;
    result = MB_ReadFaultCode(&faultCode);
    reportResult("MB_ReadFaultCode", result);
    if (result == MB_Result::Ok) {
        Serial.printf("    Fault code: 0x%04X (%u)\n", faultCode, faultCode);
    }

    Serial.println();
    Serial.println("4) Mid-level motion-limit functions");
    reportResult("MB_SetSpeedLimit(18000 degrees/s)",
                 MB_SetSpeedLimit(TEST_SPEED_DEGREES_PER_SECOND));
    const int32_t speedRaw = MB_Read(DRIVER_ADDRESS, 0x0066);
    reportCheck("Speed-limit register read-back",
                speedRaw == 12000,
                speedRaw == 12000 ? "raw value 12000" : "expected raw value 12000");

    reportResult("MB_SetAcceleration(98302.5 degrees/s^2)",
                 MB_SetAcceleration(TEST_ACCEL_DEGREES_PER_SECOND_SQUARED));
    const int32_t accelerationRaw = MB_Read(DRIVER_ADDRESS, 0x0052);
    reportCheck("Live-acceleration register read-back",
                accelerationRaw == 65535,
                accelerationRaw == 65535 ? "raw value 65535" : "expected raw value 65535");

    reportResult("MB_ApplyMotionLimits(18000, 98302.5)",
                 MB_ApplyMotionLimits(TEST_SPEED_DEGREES_PER_SECOND,
                                      TEST_ACCEL_DEGREES_PER_SECOND_SQUARED));

    Serial.println();
    Serial.println("5) High-level motion test");
    Serial.println("    Sends 9000 degrees from the connect position (600 counts at 24 counts/revolution).");

    float positionBeforeCounts = 0.0f;
    result = MB_ReadPosition(&positionBeforeCounts, MB_Unit::Counts);
    reportResult("Read starting position in counts", result);

    uint16_t startWords[2] = {};
    MB_Result startRawResult = MB_ReadRegisters(DRIVER_ADDRESS, 0x0024, 2, startWords);
    reportResult("Read absolute starting position", startRawResult);

    if (result == MB_Result::Ok && startRawResult == MB_Result::Ok) {
        const int32_t expectedDeltaCounts = static_cast<int32_t>(
            (TEST_TARGET_DEGREES * 24.0f / 360.0f) + 0.5f);
        const uint32_t startBits =
            (static_cast<uint32_t>(startWords[0]) << 16) | startWords[1];
        const int64_t startAbsoluteCounts =
            (startBits & 0x80000000UL)
                ? static_cast<int64_t>(startBits) - 0x100000000LL
                : static_cast<int64_t>(startBits);
        const uint32_t expectedTargetBits =
            static_cast<uint32_t>(startAbsoluteCounts + expectedDeltaCounts);

        result = MB_MoveTo(TEST_TARGET_DEGREES);
        reportResult("MB_MoveTo(9000)", result);

        if (result == MB_Result::Ok) {
            uint16_t commandRegisters[4] = {};
            const MB_Result commandReadResult =
                MB_ReadRegisters(DRIVER_ADDRESS, 0x0044, 4, commandRegisters);
            reportResult("Read move-command registers 0x0044-0x0047",
                         commandReadResult);

            if (commandReadResult == MB_Result::Ok) {
                const uint32_t actualTargetBits =
                    (static_cast<uint32_t>(commandRegisters[2]) << 16) |
                    commandRegisters[3];
                Serial.printf("    Speed raw: %u, target raw: 0x%08lX, expected: 0x%08lX\n",
                              commandRegisters[0],
                              static_cast<unsigned long>(actualTargetBits),
                              static_cast<unsigned long>(expectedTargetBits));
                reportCheck("MB_MoveTo wrote the expected position registers",
                            actualTargetBits == expectedTargetBits);
            }

            bool targetReached = false;
            float positionAfterCounts = positionBeforeCounts;
            MB_Result positionReadResult = MB_Result::Ok;
            const uint32_t moveStartMs = millis();

            while ((millis() - moveStartMs) < 8000) {
                positionReadResult =
                    MB_ReadPosition(&positionAfterCounts, MB_Unit::Counts);
                if (positionReadResult != MB_Result::Ok ||
                    positionAfterCounts >= positionBeforeCounts + expectedDeltaCounts) {
                    targetReached =
                        positionReadResult == MB_Result::Ok &&
                        positionAfterCounts >= positionBeforeCounts + expectedDeltaCounts;
                    break;
                }
                delay(100);
            }

            reportResult("MB_ReadPosition during motion check", positionReadResult);
            Serial.printf("    Position counts: before=%.0f, after=%.0f, target delta=%ld\n",
                          positionBeforeCounts, positionAfterCounts,
                          static_cast<long>(expectedDeltaCounts));
            reportCheck("MB_MoveTo target was reached", targetReached,
                        targetReached ? "position count changed as requested"
                                      : "command was acknowledged, but position did not reach target");
        }
    }
}

} // namespace

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

    Serial.println("Waveshare / AQMD Modbus library function test");
    Serial.println("Serial Monitor: COM8 at 115200 baud");
    Serial.println("Driver: slave 0x01, 9600 baud, 8E1");
    Serial.println("WARNING: This commands 9000 degrees (25 turns) at maximum speed (3000 rpm) and acceleration.");
    Serial.println("Motion registers 0x0066, 0x0062 and 0x0052 are restored afterward.");

    MB_Result result = MB_Init(Serial1, DRIVER_ADDRESS, 9600, SERIAL_8E1,
                               MODBUS_RTS_PIN);
    reportResult("MB_Init", result);
    if (result == MB_Result::Ok) {
        serialInitialized = true;
        runTests();
    }

    finishTest();
}

void loop() {
    // The complete test runs once after reset.
}
