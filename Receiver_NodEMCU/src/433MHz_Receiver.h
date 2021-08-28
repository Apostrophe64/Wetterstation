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

#define _DEBUG

#include <SPI.h>
#include <TFT_eSPI.h>

//#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <pgmspace.h>
#include <string_tables.h>
#include <RH_ASK.h>
#include <EEPROM.h>
#include <TimeLib.h>

const char* MY_SSID = "";
const char* MY_PASSWORD = "";
const int SERVER_PORT = 80;

IPAddress ip(192,168,0,33); //Feste IP des neuen Servers, frei wählbar
IPAddress gateway(192,168,0,1); //Gatway (IP Router eintragen)
IPAddress subnet(255,255,255,0); //Subnet Maske eintragen

ESP8266WebServer server(SERVER_PORT);

uint32_t u32UnixTime = 0; // NTP Time

// Lokaler Dienst auf Raspberry Pi
// const char* EMON_CMS_SERVER = "emonpi.fritz.box";
const char* EMON_CMS_SERVER = "192.168.0.23";
const char* EMON_WRITE_API_KEY = ""; // Hier muss der API Key von EmonCMS hin
const int EMON_CMS_PORT = 80;
#define EMON_REQUEST F("GET /emoncms/input/post.json?node=")
#define HTTP_HEADER F("Host: 192.168.0.23\r\nUser-Agent: Mozilla\r\nAccept-Language: en-US,en;q=0.5\r\nAccept-Encoding: gzip, deflate\r\nConnection: close\r\n\r\n\r\n")
#endif

const char* SMTP_SERVER = "192.168.0.1";
const int   SMTP_PORT = 25;

// Pinbelegung 433MHz-Rx, Taster, Serial RX/TX
const int txPin     = 10; // Wird nur im 433MHz Transmitter gebraucht.
const int pttPin    = 10; //   "
#ifdef ESP01
  const int rxPin     = 0; // 433 MHz Empänger Daten
  const int switchPin = 2; // Clear EEprom
#else
  const int rxPin     = D0; // 433 MHz Empänger Daten
  const int switchPin = D4; // Clear EEprom
  const int backGndPin= D1;
#endif
/*
// Pinbelegung des 2.2'' TFT
const int TFT_CS    = 10;   // GPIO 16 interne LED kein Port nötig, da TFT CS an GND
const int TFT_RST   = 10;   // D12 GPIO10 //kein Port nötig, da Reset an Reset angeschlossen
const int TFT_DC    = D3;
const int TFT_MOSI  = D7;   // GPIO 13
const int TFT_CLK   = D5;   // GPIO 14
const int TFT_MISO  = D6;   // GPIO 12
*/

//Falls die Anzeige gedreht wird
#define ROTATION_NO 0
#define ROTATION_90 1
#define ROTATION_180 2
#define ROTATION_270 3

#define RGB(r,g,b) ((r<<10) | (g<<5) | b )

#define LED_COLOR TFT_BLUE

TFT_eSPI tft = TFT_eSPI(); // Invoke custom library

#ifdef _DEBUG
  #define TRACE(sMsg) Serial.print(sMsg);
#else
  #define TRACE(sMsg)
#endif

#define HTML_SPACE      F("&nbsp&nbsp&nbsp")
#define HTML_ARROW_UP   F("&#8593")
#define HTML_ARROW_DOWN F("&#8595")


#define GFXFF 1
#define GLCD  1
#define FONT2 2
#define FONT4 4
#define FONT6 6
#define FONT7 7
#define FONT8 8

#define TRANSPARENT (uint16_t)0xff

bool bWifiConected = false;

const uint32_t MAGIC_HEADER = 0x11071964;
const int INVALID = 0x03ff;

// Wenn nach dieser Zeit keine Sensor Daten mehr eintreffen, wird
// "No data" ausgegeben, damit ersichtlich wird, das die Funkverbindung
// unterbrochen ist
const uint32_t RECEIVE_TIMEOUT = (20L*60L);

struct KeyStore
{
  // Magic Header zum Erkennen eines gültigen Eintrages.
  uint32_t  ulMagicHeader = 0;

  // Seriennummer des Transmitters.
  uint16_t  u16DeviceID = 0;

  // Für Transmitter mit Regenmesser wird der laufende Monat und
  // die gsammt Regenmenge zu Beginn dieses laufenden Monats gesichert,
  // um den Niederschlag des laufenden Monats anzeigen zu können.
  uint8_t u8Month = 0;
  float   fTotalRainMM = 0.0;
};

enum eMessage
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
} __attribute__ ((packed)); // Verkürzt sizeof(eMessage) von 4 Bytes auf 1 Byte!

struct DataPacket
{
  uint16_t    u16DeviceID = 0;
  uint8_t     u8MsgID = 0;
  eMessage    eMsg = eMessage::MsgInvalid;
  uint8_t     u8Padding = 0; // Das Enum im ESP8266 ist ein ShortEnum (1Byte)
  uint32_t    u32Payload = 0;
}__attribute__ ((packed, aligned(1)));

RH_ASK ASK_driver(2000, rxPin, txPin, pttPin, false);

uint32_t u32GoodMsg = 0;

// Funktionsprototypen
KeyStore ReadKeyStoreFromEEProm(uint8_t u8Transmitter);
bool WriteKeyStoreToEEProm(uint8_t u8Transmitter, const KeyStore& keyStore);
void ClearEEProm();
void PairDevice(uint8_t u8Transmitter, uint16_t u16DeviceID);

bool ReceiveDataPacket(DataPacket& dataPacket);
void StoreRainCounterData(uint8_t u8Transmitter, float fTotalRainMM);
void StoreWindSpeedCounterData(uint8_t u8Transmitter, float fWindSpeedCounter);

bool ShowResultsGFX();
bool ShowSensorDataOnGFX(uint8_t u8Transmitter, SensorType sensorType);

bool HasPowerMeterData(uint8_t u8Transmitter);
bool HasWaterMeterData(uint8_t u8Transmitter);
bool HasGasMeterData(uint8_t u8Transmitter);
bool CheckAllTransmittersForSendingData();

bool setupWifi();
void SendResultsToCloud();
bool SendToEmonCms(uint8_t u8Transmitter);
String CreateJsonEntry(uint8_t u8Transmitter, SensorType SensorType, uint8_t& nJsonDataCounter);
bool sendHttpRequest(WiFiClient& client, const String& sMsg);
String ReadAnswer(WiFiClient& client);
bool SendSMTP(uint8_t u8Transmitter);

void handleRoot();
void handleTemperature();
String createWebPage();
String CreateHtmlEntry(uint8_t u8Transmitter, SensorType sensorType);

uint32_t GetTimeSpanMillis(uint32_t u32StartTime);
uint32_t GetTimeSpanSeconds(uint32_t u32StartTime);
String GetTransmitterName(uint8_t u8Transmitter, int nPage);

//uint32_t FloatToUint32(float fInputValue);
float Uint32ToFloat(uint32_t u32InputValue);

void tft_ShowFrames(bool bClearScreen=true);
void tft_ShowTimeDate();
void tft_ReceiveFlag(uint8_t u8Transmitter);
void tft_ShowTitle(uint8_t u8Transmitter, SensorType sensorType);
void tft_ShowSensorValue(uint8_t u8Transmitter, SensorType sensorType, float fValue);
void tft_Disable7SegmentDisplay();
void tft_Print(int16_t x, int16_t y, uint8_t s, bool bLeft, const char* pText, uint16_t colorText, uint16_t colorBack=TFT_BLACK);
void tft_Print(int16_t x, int16_t y, uint8_t s, bool bLeft, const __FlashStringHelper* pText, uint16_t colorText, uint16_t colorBack=TFT_BLACK);
#endif // __433MHZ_RECEIVER_H__
