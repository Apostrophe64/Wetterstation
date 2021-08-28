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

#ifndef __433MHZ_RECEIVER_H__
#define __433MHZ_RECEIVER_H__

// Wenn keine Netzwerkdienste gewünscht sind, kann Speicher gespart werden.
// #define ENABLE_NETWORK

//#define _DEBUG
//#define USE_2004_LCD
// #define USE_WINTEK // LCD von Wintek
#define USE_WINTEK_I2C

#ifdef ENABLE_NETWORK
  #define SSID F("")
  #define PASSWORD F("")
  const int SERVER_PORT = 80;

  // Lokaler Dienst auf Raspberry Pi
  #define EMON_CMS_SERVER F("emonpi.fritz.box")
  #define EMON_WRITE_API_KEY F("ad12dd19a15fc95c1ede23b62a38558d")
  const int EMON_CMS_PORT = 80;
  
  #define EMON_REQUEST F("GET /emoncms/input/post.json?node=")
  #define HTTP_HEADER F("Host: emonpi.fritz.box\r\nUser-Agent: Mozilla\r\nAccept-Language: en-US,en;q=0.5\r\nAccept-Encoding: gzip, deflate\r\nConnection: close\r\n\r\n\r\n")
#endif

#include <avr/pgmspace.h>
#include <string_tables.h>
#include <RH_ASK.h>
#include <SPI.h>
#include <EEPROM.h>

#ifdef USE_2004_LCD
  #include <LiquidCrystal_I2C.h>
#endif

#ifdef USE_WINTEK
  #include <Wintek2704.h>
#endif

#ifdef USE_WINTEK_I2C
  #include <Wintek2704_MCP23017.h>
#endif

#ifdef ENABLE_NETWORK
  #include <ESP8266.h>
#endif

// Pinbelegung 433MHz-Rx, Taster, Serial RX/TX
const int switchPin = 10;
const int rxPin = 11;
const int txPin = A2;   // Wird nur im Transmitter gebraucht.
const int pttPin = A3;  //   "

#ifdef _DEBUG
  const int serialTX = A0;
  const int serialRX = A1;
#endif

#ifdef AVR_MEGA2560
  #ifdef _DEBUG
    // #define TRACE(sMsg) Serial.print(sMsg);
    #define TRACE(sMsg) Serial1.print(sMsg);
  #else
    #define TRACE(sMsg)
  #endif
#else
  #ifdef ARDUINO_AVR_NANO
    #ifdef _DEBUG
      #include <SoftwareSerial.h>
      // SoftwareSerial mySerial(serialRX, serialTX); // RX, TX
      //#define TRACE(sMsg) mySerial.print(sMsg);
      #define TRACE(sMsg) Serial.print(sMsg);
    #else
      #define TRACE(sMsg)
    #endif
  #endif
#endif

#ifdef ENABLE_NETWORK
  #define HTML_SPACE      F("&nbsp&nbsp&nbsp")
  #define HTML_ARROW_UP   F("&#8593")
  #define HTML_ARROW_DOWN F("&#8595")

  bool bWifiConected = false;
  bool bHasWifi = false;
#endif

const uint32_t MAGIC_HEADER = 0x11071964;
const int INVALID = 0x03ff;

// Wenn nach dieser Zeit keine Sensor Daten mehr eintreffen, wird
// "No data" ausgegeben, damit ersichtlich wird, das die Funkverbindung
// unterbrochen ist
const uint32_t RECEIVE_TIMEOUT = (20L*60L);

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

struct KeyStore
{
  // Magic Header zum Erkennen eines gültigen Eintrages.
  uint32_t  ulMagicHeader = 0;

  // Seriennummer des Transmitters.
  uint16_t  u16DeviceID = 0;
};

struct DataPacket
{
  uint16_t    u16DeviceID = 0;
  uint8_t     u8MsgID = 0;
  eMessage    eMsg = eMessage::MsgInvalid;
  uint32_t    u32Payload = 0;
};

// LCD Objekt anlegen
#ifdef USE_WINTEK
  const int LCD_COLS = 27;
  const int LCD_ROWS = 4;
  wintec2704 lcd;
#endif
#ifdef USE_WINTEK_I2C
  const int LCD_COLS = 27;
  const int LCD_ROWS = 4;
  Wintec2704_I2C lcd(0x20);
#endif
#ifdef USE_2004_LCD
  const int LCD_COLS = 20;
  const int LCD_ROWS = 4;
  LiquidCrystal_I2C lcd(0x3f, LCD_COLS, LCD_ROWS);
#endif

#ifdef ENABLE_NETWORK
  #ifdef AVR_MEGA2560
    // ESP8266 wifi(Serial1);
    ESP8266 wifi(Serial);
  #else
    #ifdef ARDUINO_AVR_NANO
      ESP8266 wifi(Serial);
    #endif
  #endif
#endif

RH_ASK ASK_driver(2000, rxPin, txPin, pttPin, false);

// Funktionsprototypen
KeyStore ReadKeyStoreFromEEProm(uint8_t u8Transmitter);
bool WriteKeyStoreToEEProm(uint8_t u8Transmitter, const KeyStore& keyStore);
void ClearEEProm();
void PairDevice(uint8_t u8Transmitter, uint16_t u16DeviceID);

bool ReceiveDataPacket(DataPacket& dataPacket);
void StoreRainCounterData(uint8_t u8Transmitter, float fTotalRainMM);

void StoreWindSpeedCounterData(uint8_t u8Transmitter, float fWindSpeedCounter);

void ShowResults();
void ShowSensorDataOnLCD(uint8_t u8Transmitter, SensorType sensorType, uint8_t& u8Row);

int8_t GetNextPage(uint8_t u8Transmitter, int8_t i8StartPage);
bool HasPowerMeterData(uint8_t u8Transmitter);
bool HasWaterMeterData(uint8_t u8Transmitter);
bool HasGasMeterData(uint8_t u8Transmitter);

#ifdef ENABLE_NETWORK
  void SendResultsToCloud();
  bool sendSensorDataToClient(uint8_t mux_id);
  void SendHtmlEntry(uint8_t mux_id, uint8_t u8Transmitter, SensorType SensorType);

  bool SendToEmonCms(uint8_t u8Transmitter);
  bool SendJsonEntry(uint8_t mux_id, uint8_t u8Transmitter, SensorType SensorType, uint8_t& nJsonDataCounter);

  bool sendToClient(uint8_t mux_id, const String& sMsg);
  bool setupWifi();
  void handleServer();
#endif

uint32_t GetTimeSpanMillis(uint32_t u32StartTime);
uint16_t GetTimeSpanSeconds(uint16_t u16StartTime);
String GetTransmitterName(uint8_t u8Transmitter, int8_t i8Page);

//uint32_t FloatToUint32(float fInputValue);
float Uint32ToFloat(uint32_t u32InputValue);

#endif // __433MHZ_RECEIVER_H__
