/* Copyright (c) 2021, Martin Lück
**
** Permission to use, copy, modify, and/or distribute this software for
** any purpose with or without fee is hereby granted, provided that the
** above copyright notice and this permission notice appear in all copies.
**
** THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
** WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR
** BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES
** OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
** WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
** ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
** SOFTWARE.
*/

#define _DEBUG
#define HUMIDITY_SENSOR DHT22

// Schaltet den entsprechenden Ausgang nach dem Einlesen des Sensors auf low,
// um Strom zu sparen.
#define TOGGLE_POWER_DHTxxPIN
#define TOGGLE_POWER_DS1820PIN
#define TOGGLE_POWER_BMP085PIN

#define SEND_COUNTS 3
#define USE_ISR_READING
//#define _CALIBRATE

// Wenn nicht definiert, wird anstatt einer Reflexlichtschranke ein Hallsensor verwendet...
#define USE_IR_SENSOR_FOR_POWER_MEASUREMENT
#define USE_IR_SENSOR_FOR_GAS_MEASUREMENT
// #define USE_IR_SENSOR_FOR_WATER_MEASUREMENT

#define POWER_METER_SENSOR_VALUE_LOW 0
#define GAS_METER_SENSOR_VALUE_LOW 0
#define WATER_METER_SENSOR_VALUE_LOW 0

#if (0)
// Abgelesene Zählerstände vom 21.05.2017
// #define SET_INITIAL_START_VALUES
  #define INITIAL_TOTAL_RAIN_MM   0.0; // 44.0
  #define INITIAL_TOTAL_POWER_WH  ((3937.6) * 1000.0)
  #define INITIAL_TOTAL_WATER_L   (175.6 * 1000.0)
  #define INITIAL_TOTAL_GAS_L     (5696.2 * 1000.0)
#else
  // #define SET_INITIAL_START_VALUES
  #define INITIAL_TOTAL_RAIN_MM   0.0; // 44.0
  #define INITIAL_TOTAL_POWER_WH  0.0
  #define INITIAL_TOTAL_WATER_L   0.0
  #define INITIAL_TOTAL_GAS_L     0.0
#endif

#include <RH_ASK.h>
#include <SPI.h>
#include <EEPROM.h>
#include <LowPower.h>
#include "PCF8583.h"
#include "DHT.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <DallasTemperature.h>
#include <MsTimer2.h>

#ifdef _CALIBRATE
  #define _DEBUG
#endif

const int switchPin         = 2;
const int ds1820Pin         = 3;
const int powerDHTxxPin     = 4;
const int powerDS1820Pin    = 5;
const int powerBMP085Pin    = 6;
const int txPin             = 10;
const int DHTxxPin          = 11;
const int ledPin            = LED_BUILTIN;

const int analogInPinPowerMeter = A0;
const int analogInPinWaterMeter = A2;
const int analogInPinGasMeter   = A1;

const int ledIRPinPowerMeter    = 12;
const int ledIRPinWaterMeter    = A3;
const int ledIRPinGasMeter      = 9;
const int PCF8583Addr = 0xA0;
const int rxPin       = A6;       // Wird nur vom Receiver genutzt
const int pttPin      = A7;       //   "

// Gemessene eliptische Fläche des Regensensors im mm²
// #define A_RAIN_SENSOR 7952.2 // Original Regensensor
#define A_RAIN_SENSOR 36305  // Regensensor mit Auffangtrichter

// Gemessen: 4.26 ml/tick
#define ML_PER_TICK 4.26

// mm Regen pro tick ==> 0.117 mm
#define MM_PER_TICK ((1000.0 * ML_PER_TICK) / A_RAIN_SENSOR)

// Umdrehungen pro kWh
const float REV_PER_KWH = 75.0;

  // Liter pro Umdrehung
  const float WATER_LITER_PER_REV = 1.0;

// Liter pro Umdrehung
const float GAS_LITER_PER_REV = 10.0;

// Power-Meter kalibrierungs Werte des Stromzählers
const int   nPowerTriggerLevelHigh = 180;
const int   nPowerTriggerLevelLow = 150;

#ifdef USE_IR_SENSOR_FOR_WATER_MEASUREMENT
  // Water-Meter kalibrierungs Werte des Wasserzählers
  const int   nWaterTriggerLevelHigh = 150;
  const int   nWaterTriggerLevelLow = 70;
#else
  // Water-Meter kalibrierungs Werte des Wasserzählers
  // Messwert schwankt zwischen 546...583
  const int   nWaterTriggerLevelHigh = 570;
  const int   nWaterTriggerLevelLow = 560;
#endif
// Gas-Meter kalibrierungs Werte des Gaszählers
const int   nGasTriggerLevelHigh = 350;
const int   nGasTriggerLevelLow = 220;

#ifdef _DEBUG
  #define TRACE(sMsg) Serial.print(sMsg);
#else
  #define TRACE(sMsg)
#endif

const uint32_t MAGIC_HEADER = 0x11071964;
const int INVALID = 0x03ff;

typedef enum
{
  MsgInvalid          = 0,
  MsgPairing          = 1,
  MsgTemperature      = 2,
  MsgHumidity         = 3,
  MsgPressure         = 4,
  MsgWeatherCompact   = 5,
  MsgTotalRainMM 	    = 6,
  MsgPowerMeterCurrent= 7,
  MsgPowerMeterTotal  = 8,
  MsgTotalWater       = 9,
  MsgCurrentWaterFlow = 10,
  MsgTotalGas         = 11,
  MsgCurrentGasFlow   = 12,
  MsgWindSpeedCounter = 13

}eMessage;

struct DeviceSettings
{
  // Magic Header zum Erkennen eines gültigen Eintrages.
  uint32_t  ulMagicHeader = 0;

  // Seriennummer des Transmitters.
  uint16_t  u16DeviceID = 0;

  // Gesamtenergiezähler
  float fTotalPowerWh = 0.0;

  // Anzahl der Kippbwegungen der Regenwippe
  volatile bool bDetectRainSensor = false;
  volatile float fTotalRainMM = 0.0;

  // Gesamtwasserzähler
  float fTotalWaterL = 0.0;

  // Gesamtgaszähler
  float fTotalGasL = 0.0;
};

struct DataPacket
{
  uint16_t    u16DeviceID = 0;
  uint8_t     u8MsgID = 0;
  eMessage    eMsg = eMessage::MsgInvalid;
  uint32_t    u32Payload = 0;
};

volatile bool bRainCounterChanged = false;
uint32_t u32Tickcount = 0;
DeviceSettings deviceSettings;

// Luftfeuchte und Temperatur Sensor
DHT dht(DHTxxPin, HUMIDITY_SENSOR);

// Luftdruck, Luftfeuchte, Temperatur, Regen Sensor, Power Meter, Water Meter, Wind Speed
Adafruit_BMP085_Unified BMPSensor = Adafruit_BMP085_Unified(10085);
bool bDetectBMP085 = false;
bool bDetectDHTxx = false;
bool bDetectDS1820 = false;
bool bDetectPowerMeter = false;
bool bDetectWaterMeter = false;
bool bDetectGasMeter = false;
bool bDetectPCF8583 = false;

// DS1820 OneWire Temperatursensor
DeviceAddress DS1820DeviceAdr;
OneWire oneWire(ds1820Pin);
DallasTemperature ds1820(&oneWire);

// declare an instance of the library for IC at address 0xA0
// (A0 pin connected to ground)
PCF8583 pcf8583(PCF8583Addr);

// 433MHz Sende-Objekt
RH_ASK ASK_driver(2000, rxPin, txPin, pttPin, false);

// Funktionsprototypen
void loop_weather_station();
void loop_power_water_gas_meter();

void ReadPowerWaterGasSensors(int& nPowerMeterSensorDiff, int& nWaterMeterSensorDiff, int& nGasMeterSensorDiff);

bool  DetectPowerMeterTrigger(int nSensorDiff);
bool  DetectWaterMeterTrigger(int nSensorDiff);
bool  DetectGasMeterTrigger(int nSensorDiff);

DeviceSettings ReadDeviceSettingsFromEEProm();
bool WriteDeviceSettingsToEEProm(const DeviceSettings& deviceSettings);
void ClearEEProm();

void SendMessage(eMessage eMsg, uint8_t u8MsgID, uint32_t u32Payload);
void SendRepeatedMessage(eMessage eMsg, uint32_t u32Payload, uint32_t& u32Tickcount);
void BlinkLED(int nDelay);

uint32_t FloatToUint32(float fInputValue);
float Uint32ToFloat(uint32_t u32InputValue);
