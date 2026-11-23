# ModbusLib

Arduino/PlatformIO-library voor Modbus RTU-aansturing van de AQMD3605BLS-B2-driver met een Waveshare ESP32-S3-POE-ETH-8DI-8DO-module.

De API is gegroepeerd in high-level beweging en uitlezen, mid-level driverinstellingen en low-level Modbus RTU-toegang. De huidige functies gebruiken het prefix `MB_`.

## Vereisten

- Waveshare ESP32-S3-POE-ETH-8DI-8DO (of compatibele ESP32-S3)
- AQMD3605BLS-B2-driver via Modbus RTU
- Arduino-framework met PlatformIO
- Standaardinstellingen: slave-adres `0x01`, `9600` baud, `8E1`
- UART-pinnen: RX GPIO 18, TX GPIO 17 en driver-enable/RTS GPIO 21

De pinnen staan in `ModbusLib.h`. Controleer bedrading en seriële instellingen als jouw opstelling hiervan afwijkt.

## Installeren in PlatformIO

Plaats de map `ModbusLib` in de `lib`-map van je PlatformIO-project en voeg toe aan `platformio.ini`:

```ini
lib_deps =
    .\lib\ModbusLib
```

Gebruik in je sketch:

```cpp
#include <Arduino.h>
#include "ModbusLib.h"
```

## Snel aan de slag

`MB_Connect()` initialiseert de standaardverbinding automatisch als `MB_Init()` nog niet is aangeroepen. De huidige positie wordt bij verbinden het nulpunt voor `MB_MoveTo()`.

```cpp
#include <Arduino.h>
#include "ModbusLib.h"

void setup() {
    Serial.begin(115200);

    if (MB_Connect() != MB_Result::Ok) {
        Serial.println("Verbinding met driver mislukt");
        return;
    }

    // 90 graden vanaf de positie bij MB_Connect().
    if (MB_MoveTo(90.0f) != MB_Result::Ok) {
        Serial.println("Bewegingsopdracht mislukt");
    }
}

void loop() {}
```

Standaard gebruikt de library 30 graden/s en 60 graden/s². Stel desgewenst andere waarden in vóór het bewegen:

```cpp
MB_SetDefaultMotion(90.0f, 180.0f, MB_Unit::Degrees);
MB_MoveTo(90.0f);
```

## API-overzicht

### High-level

| Functie | Gebruik |
|---|---|
| `MB_Connect()` | Initialiseert zo nodig de UART, controleert de driver en stelt het huidige punt als nulpunt in. |
| `MB_Disconnect()` | Stopt de driver, beëindigt de UART en verbreekt de verbinding. |
| `MB_MoveTo(target, unit, addresses, addressCount, relative)` | Zonder lijst beweegt de standaarddriver relatief aan het nulpunt van `MB_Connect()`. Met een lijst kiest `relative` tussen dezelfde afstand vanaf ieders huidige positie (`true`) en dezelfde absolute doelcoördinaat (`false`). |
| `MB_Stop(addresses, addressCount)` | Stopt de standaarddriver of de opgegeven adressen. |
| `MB_ReadPosition(&position, unit)` | Leest positie relatief aan het verbindingsnulpunt. |
| `MB_ReadTelemetry(&telemetry)` | Leest positie, snelheid, stroom, status, foutcode, PWM en busspanning. |
| `MB_SetPulsesPerRevolution(value)` | Pulsen/tellingen per omwenteling voor eenheidsconversies. Standaard: 24. |
| `MB_SetDefaultMotion(speed, acceleration, unit)` | Slaat snelheid en acceleratie voor volgende bewegingen op. |

Telemetrie lezen:

```cpp
MB_Telemetry telemetry{};
if (MB_ReadTelemetry(&telemetry) == MB_Result::Ok) {
    Serial.printf("Positie: %ld counts, snelheid: %.2f RPM\n",
                  static_cast<long>(telemetry.positionCounts),
                  telemetry.speedRpm);
    Serial.printf("Stroom: %.2f A, bus: %.1f V\n",
                  telemetry.currentAmps, telemetry.busVoltageVolts);
}
```

### Mid-level

Deze functies vereisen een succesvolle `MB_Connect()`.

| Functie | Gebruik |
|---|---|
| `MB_SetSpeedLimit(speed, unit, addresses, addressCount)` | Stelt snelheidslimiet in voor de standaarddriver of de opgegeven adressen. |
| `MB_SetAcceleration(acceleration, unit, addresses, addressCount)` | Stelt acceleratielimiet en live-acceleratie in voor de standaarddriver of de opgegeven adressen. |
| `MB_ApplyMotionLimits(speed, acceleration, unit, addresses, addressCount)` | Stelt snelheid en acceleratie in één aanroep in voor de standaarddriver of de opgegeven adressen. |
| `MB_ReadDriverStatus(&status)` | Leest status uit register `0x0023`. |
| `MB_ReadFaultCode(&faultCode)` | Leest foutcode uit register `0x0033`. |
| `MB_SetAlarm(enabled, addresses, addressCount)` | Schakelt alarm voor de standaarddriver of opgegeven adressen en controleert de waarde. |

`MB_SetAlarm(true)` schrijft `0x0000` naar register `0x0099` (alarm aan). `MB_SetAlarm(false)` schrijft `0x0001` (alarm uit).

### Opdrachten naar meerdere drivers

Schrijf- en stopfuncties accepteren optioneel een array met slave-adressen en het aantal elementen. Als je de lijst weglaat, wordt het standaardadres uit `MB_Init()` gebruikt. Adressen in de lijst moeten uniek zijn en binnen `0x01`-`0xF7` vallen.

```cpp
const uint8_t motors[] = {0x01, 0x02, 0x03};

MB_SetAlarm(false, motors, 3);  // alarm uit op alle drie
MB_SetSpeedLimit(90.0f, MB_Unit::Degrees, motors, 3);
MB_ApplyMotionLimits(90.0f, 180.0f, MB_Unit::Degrees, motors, 3);
MB_MoveTo(90.0f, MB_Unit::Degrees, motors, 3, true);  // elk 90 graden vanaf huidige positie
MB_MoveTo(0.0f, MB_Unit::Degrees, motors, 3, false); // allemaal naar absolute positie 0
MB_Stop(motors, 3);
```

De library stuurt losse unicast-berichten naar elk adres. Leesfuncties blijven het standaardadres gebruiken. Bij een groepsopdracht probeert de library ieder adres te bereiken en geeft daarna de eerste foutcode terug; een fout kan dus betekenen dat sommige drivers de opdracht al wel hebben uitgevoerd.

### Eenheden en limieten

Bewegingsfuncties accepteren `MB_Unit::Counts`, `Degrees`, `Radians` of `Rotations`. Standaard is graden. Bij `Counts` is de waarde rechtstreeks een aantal pulsen/tellingen. Stel met `MB_SetPulsesPerRevolution()` de juiste waarde voor jouw motorconfiguratie in.

Snelheid en acceleratie moeten positief zijn. De snelheidsconversie is begrensd op maximaal 3000 RPM; de bruikbare limieten hangen ook af van de driverconfiguratie.

### Low-level Modbus RTU

Gebruik deze functies voor directe toegang tot slave en registers. Adressen en registernummers geef je in hex op.

```cpp
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
```

Voorbeeld: foutcode-register `0x0033` lezen van slave `0x01`:

```cpp
MB_Init(Serial1, 0x01, 9600, SERIAL_8E1, MODBUS_RTS_PIN);

uint16_t fault = 0;
if (MB_ReadRegisters(0x01, 0x0033, 1, &fault) == MB_Result::Ok) {
    Serial.printf("Foutcode: 0x%04X\n", fault);
}
```

`MB_Read()` geeft de registerwaarde of `-1` bij een fout. De overige low-level functies geven een `MB_Result` terug. Gebruik de automatische `MB_Connect()`-initialisatie óf roep zelf `MB_Init()` aan.

## Resultaatcodes

| Waarde | Betekenis |
|---|---|
| `MB_Result::Ok` | Bewerking gelukt |
| `MB_Result::NotInitialized` | Vereiste communicatie/verbinding ontbreekt |
| `MB_Result::InvalidArgument` | Ongeldig argument of ongeldige waarde |
| `MB_Result::Timeout` | Geen geldig antwoord binnen de wachttijd |
| `MB_Result::InvalidResponse` | Antwoord of teruggelezen waarde is ongeldig |
| `MB_Result::ModbusException` | Slave gaf een Modbus-exception terug |
| `MB_Result::CommunicationError` | Andere communicatie- of protocolfout |

## Testsketch

`src/main.cpp` test de communicatie en meerdere functies en meldt `PASS` of `FAIL` via de seriële monitor. Dit project is ingesteld op COM8 en 115200 baud.

**Let op:** de huidige testsketch geeft een bewegingsopdracht van 9000 graden (25 omwentelingen) met hoge snelheid en acceleratie. Controleer de mechanische opstelling en pas de testwaarden aan voordat je deze sketch uitvoert. De test probeert de oorspronkelijke motion-limit-registers daarna terug te zetten.

## Registeroverzicht

| Register | Doel |
|---|---|
| `0x0000` | Driver-ID/verbindingstest |
| `0x0023` | Driverstatus |
| `0x0024`–`0x0025` | Positieteller (32-bit) |
| `0x0033` | Foutcode |
| `0x0040` | Stopopdracht |
| `0x0044`–`0x0047` | Snelheid en doelpositie |
| `0x0052` | Live acceleratie |
| `0x0062` | Acceleratielimiet |
| `0x0066` | Snelheidslimiet |
| `0x0099` | Alarm: `0` aan, `1` uit |

## Bestanden

- `ModbusLib.h`: publieke API en datatypes
- `ModbusLib.cpp`: Modbus RTU-protocol en implementatie
