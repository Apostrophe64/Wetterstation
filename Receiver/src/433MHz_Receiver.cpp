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

#include "sensorData.h"
#include "433MHz_Receiver.h"
#include <avr/pgmspace.h>

extern std::vector<SensorData> m_SensorDatas;
uint8_t u8MsgIDs[MAX_TRANSMITTER] = {0};

int freeRam()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

///////////////////////////////////////////////////////////////////////////////
void setup()
{
#ifdef AVR_MEGA2560
  Serial.begin(115200);
  #ifdef _DEBUG
    Serial1.begin(115200);
  #endif
#else
  #ifdef ARDUINO_AVR_NANO
    Serial.begin(115200);
  #endif
  #ifdef _DEBUG
    // mySerial.begin(115200);
      Serial.begin(115200);
  #endif
#endif

  TRACE(F("Starting...\n"));
#ifdef _DEBUG
  std::vector<SensorData> SensDatas;
  TRACE(F("sizeof(SensDatas)="));
  TRACE((int)sizeof(SensDatas));
  TRACE("\n");

  std::vector<uint16_t> u16Values;
  TRACE(F("sizeof(u16Values)="));
  TRACE((int)sizeof(u16Values));
  TRACE("\n");
#endif
  // Clear EEProm Switch
  pinMode(switchPin, INPUT);
  digitalWrite(switchPin, HIGH); // Internen Pullup einschalten

  // initialize the LCD
  lcd.init();
  lcd.clear();
  lcd.backlight();

// Die Pfeile sind nur notwendig, wenn mehrere Sensorwerte gesammelt werden.
#if MAX_SENSOR_DATAS > 1
  // Trend-Zeichen definieren (Pfeil rauf, Pfeil runter)
  #define CHAR_ARROW_UP 0
  #define CHAR_ARROW_DOWN 1

  const static  char ArrowUp[]   PROGMEM = {0,4,14,21,4,4,4,0};
  const static  char ArrowDown[] PROGMEM = {0,4,4,4,21,14,4,0};

  lcd.createChar_P((unsigned char)CHAR_ARROW_UP, ArrowUp);
  lcd.createChar_P((unsigned char)CHAR_ARROW_DOWN, ArrowDown);
#endif

  #define CHAR_CUBIC 2
  const static char Cubic[]     PROGMEM = {12,2,12,2,12,0,0,0}; 
  lcd.createChar_P((unsigned char)CHAR_CUBIC, Cubic);

#ifdef USE_2004_LCD
  #define CHAR_SUM 3
  const static char Summe[]     PROGMEM = {31,16,8,4,8,16,31,0};
  lcd.createChar_P((unsigned char)CHAR_SUM, Summe);
#endif

#if (0)
while (true)
{
  for (int16_t nI = 0; nI < 11; nI++)
  {
    /*
    char buffer[30];
    strcpy_P(buffer, (char*)pgm_read_word(&g_sSensorName[nI])); // Necessary casts and dereferencing, just copy.
    lcd.print(buffer);
    lcd.print(F(" 12.2"));
    strcpy_P(buffer, (char*)pgm_read_word(&g_sLCDUnits[nI])); // Necessary casts and dereferencing, just copy.
    lcd.print(buffer);
    delay( 500 );
    */
    lcd.print(PSTR(g_sSensorName[nI]));
    lcd.print(F(" 12.2"));
    lcd.print(PSTR(g_sLCDUnits[nI]));
    delay(1000);
    lcd.clear();
  }
}
#endif
  // Wenn während des Resets 10 Sekunden der Taster betätigt wird
  // werden alle Keystore Einträge aus dem EEProm gelöscht.
  uint32_t u32TC = millis();
  int8_t i8Counter = 10;
  while (digitalRead(switchPin) == LOW)
  {
    lcd.setCursor(0,0);
    lcd.print(F("Clear EEProm in: "));
    lcd.print((int)i8Counter);
    lcd.print(F("s "));

    if (GetTimeSpanMillis(u32TC) > 1000)
    {
      u32TC = millis();
      i8Counter--;
      if (i8Counter == 0)
      {
        lcd.clear();
        lcd.print(F("Clearing EEProm..."));
        ClearEEProm();
        delay(2000);
        break;
      }
    }
  }

  ASK_driver.init();

  lcd.setCursor(0,0);
  lcd.print(F("To pair the device, "));
  lcd.setCursor(0,1);
  lcd.print(F("reset the transmit- "));
  lcd.setCursor(0,2);
  lcd.print(F("ter now.            "));

  // Warte 7 Sekunden auf die Pairing Message des Transmitters...
  KeyStore keyStore;
  DataPacket dataPacket;
  u32TC = millis();
  do
  {
    if (ReceiveDataPacket(dataPacket))
    {
        // Ist es eine Pairing Anforderung?
        if (dataPacket.eMsg == eMessage::MsgPairing)
        {
          // TRACE(F("Receiving pairing msg\n"));

          // Suche im Keystore anhand der Seriennummer, ob dieser Transmitter
          // schon bekannt ist. Wenn ja, dann setzen den Rollingcode neu
          for (uint8_t u8Transmitter = 0; u8Transmitter < MAX_TRANSMITTER; u8Transmitter++)
          {
            keyStore = ReadKeyStoreFromEEProm(u8Transmitter);
            if (keyStore.ulMagicHeader == MAGIC_HEADER)
            {
                if (keyStore.u16DeviceID == dataPacket.u16DeviceID)
                {
                  lcd.clear();
                  lcd.print(F("Repairing success"));
                  lcd.setCursor(0,1);
                  lcd.print(F("Transmitter: "));
                  lcd.print(u8Transmitter + 1);

                  delay(2000);
                  break;
                }
            }
            else // Empty Entry.
            {
              // Die erhaltenen Seriennummer sichern.
              keyStore.ulMagicHeader  = MAGIC_HEADER;
              keyStore.u16DeviceID    = dataPacket.u16DeviceID;

              // Neuen KeyStore Eintrag im EEProm sichern.
              if (WriteKeyStoreToEEProm(u8Transmitter, keyStore))
              {
                lcd.clear();
                lcd.print(F("New pairing success"));
                lcd.setCursor(0,1);
                lcd.print(F("Transmitter: "));
                lcd.print(u8Transmitter + 1);

                delay(2000);
              }
              break;
            }
          }
          break;
        }
     }
  }while ((keyStore.ulMagicHeader != MAGIC_HEADER) && (GetTimeSpanMillis(u32TC) < 7000));

#ifdef ENABLE_NETWORK
  lcd.clear();
  setupWifi();
  delay(2000);
#endif

  lcd.clear();
  lcd.print(F("Receiver ready"));

  delay(1000);
}

///////////////////////////////////////////////////////////////////////////////
void loop()
{
#ifdef ENABLE_NETWORK
  static uint32_t u32LastSendToCloudTC = millis();
#endif
  static uint32_t u32LastShowSensorDataTC = millis();

  KeyStore keyStore;
  DataPacket dataPacket;

  uint8_t u8Transmitter = 0;
  bool bFound = false;

  if (ReceiveDataPacket(dataPacket))
  {
    digitalWrite(LED_BUILTIN, HIGH);
#ifdef _DEBUG
      TRACE(F("(Msg="));
      TRACE(dataPacket.eMsg);
      TRACE(F(" MsgID="));
      TRACE(dataPacket.u8MsgID);
      TRACE(F(" DeviceID="));
      TRACE(dataPacket.u16DeviceID);
      TRACE(F(" Payload="));
      TRACE(dataPacket.u32Payload);
      TRACE(")\n");
#endif
    // Suche Anhand der DeviceID die passende Transmitternummer aus dem KeyStore
    for (uint8_t u8I = 0; u8I < MAX_TRANSMITTER; u8I++)
    {
      keyStore = ReadKeyStoreFromEEProm(u8I);
#ifdef _DEBUG     
      TRACE(u8I);
      TRACE(F(" Key-DeviceID="));
      TRACE(keyStore.u16DeviceID);
      TRACE("\n");
#endif      
      if ((keyStore.ulMagicHeader == MAGIC_HEADER) && (dataPacket.u16DeviceID == keyStore.u16DeviceID))
      {
        u8Transmitter = u8I;
        bFound = true;
        break;
      }
    }
    if (bFound)
    {
#ifdef ENABLE_NETWORK      
      // Nach dem erfolgreichen Empfang einer Nachricht, erstmal
      // SendResultsToCloud() nicht aufrufen, da evtl.
      // noch weitere Daten eintreffen.
      u32LastSendToCloudTC = millis();
#endif

#ifdef _DEBUG
      TRACE(F("(Msg="));
      TRACE(dataPacket.eMsg);
      TRACE(F(" MsgID="));
      TRACE(dataPacket.u8MsgID);
      TRACE(F(" Transmitter="));
      TRACE(u8Transmitter+1);
      TRACE(F(" Payload="));
      TRACE(dataPacket.u32Payload);
      TRACE(")\n");
#endif

      // Anzeigen, das etwas empfangen wurde.
      lcd.setCursor(LCD_COLS-3,0);
      lcd.print("(");
      lcd.print(u8Transmitter+1);
      lcd.print(")");

      // Neue Nachricht, oder nur eine Wiederholung der vorherigen Nachricht
      // des Transmitters.
      if (dataPacket.u8MsgID != u8MsgIDs[u8Transmitter])
      {
        u8MsgIDs[u8Transmitter] = dataPacket.u8MsgID;
        switch ((int)dataPacket.eMsg)
        {
          case eMessage::MsgWeatherCompact:
            /////////////////////////////////////////////////////////////////
            // Jeder Messwert wird mit 10Bit übertragen
            // t = -30.0 ... +70.0 °C -> T = (t + 30) * 10  -> 0 <= T <= 1000
            // h = 0.0 ... 100.0 %    -> H = h * 10         -> 0 <= H <= 1000
            // p = 963 ... 1063 hPa   -> P = (p - 963) * 10 -> 0 <= P <= 1000
            // Ein Wert von 0x3FF (1023) für T, H, oder P bedeutet 'ungültig'
            //3322 2222 2222 1111 1111 1100 0000 0000
            //1098 7654 3210 9876 5432 1098 7654 3210
            //xxTT TTTT TTTT HHHH HHHH HHPP PPPP PPPP
            /////////////////////////////////////////////////////////////////
            {
              int16_t nTemperature = (int16_t)(dataPacket.u32Payload >> 20) & 0x03ff;
              int16_t nHumidity    = (int16_t)(dataPacket.u32Payload >> 10) & 0x03ff;
              int16_t nPressure    = (int16_t)(dataPacket.u32Payload >> 00) & 0x03ff;

              if (nTemperature != 0x03ff) // Valid?
                StoreSensorData(u8Transmitter, eTemperatur, (float)(nTemperature - 300)/10.0);

              if (nHumidity != 0x03ff) // Valid?
                StoreSensorData(u8Transmitter, eHumidity, (float)(nHumidity)/10.0);

              if (nPressure != 0x03ff) // Valid?
                StoreSensorData(u8Transmitter, ePressure, (float)(nPressure + 9630)/10.0);
            }
            break;

          case eMessage::MsgTemperature:
              StoreSensorData(u8Transmitter, eTemperatur, (float)(dataPacket.u32Payload)/10.0);
              break;

          case eMessage::MsgHumidity:
              StoreSensorData(u8Transmitter, eHumidity, (float)(dataPacket.u32Payload)/10.0);
              break;

          case eMessage::MsgPressure:
              StoreSensorData(u8Transmitter, ePressure, (float)(dataPacket.u32Payload)/10.0);
              break;

          case eMessage::MsgTotalRainMM:
              StoreRainCounterData(u8Transmitter, Uint32ToFloat(dataPacket.u32Payload));
              break;

          case eMessage::MsgPowerMeterCurrent:
              StoreSensorData(u8Transmitter, ePowerCurrent, Uint32ToFloat(dataPacket.u32Payload));
              break;

          case eMessage::MsgPowerMeterTotal:
              StoreSensorData(u8Transmitter, ePowerTotal, Uint32ToFloat(dataPacket.u32Payload) / 1000.0);
              break;

          case eMessage::MsgCurrentWaterFlow:
              StoreSensorData(u8Transmitter, eWaterCurrent, Uint32ToFloat(dataPacket.u32Payload));
              break;

          case eMessage::MsgTotalWater:
              StoreSensorData(u8Transmitter, eWaterTotal, Uint32ToFloat(dataPacket.u32Payload) / 1000.0);
              break;

          case eMessage::MsgTotalGas:
              StoreSensorData(u8Transmitter, eGasTotal, Uint32ToFloat(dataPacket.u32Payload) / 1000.0);
              break;

          case eMessage::MsgCurrentGasFlow:
              StoreSensorData(u8Transmitter, eGasCurrent, Uint32ToFloat(dataPacket.u32Payload));
              break;

          case eMessage::MsgWindSpeedCounter:
              StoreWindSpeedCounterData(u8Transmitter, Uint32ToFloat(dataPacket.u32Payload));
              break;
          }
      }
    }
    else
    {
      TRACE(F("Can't find keytore entry\n"));
    }
  }

  digitalWrite(LED_BUILTIN, LOW);

  if (GetTimeSpanMillis(u32LastShowSensorDataTC) > 5000)
  {
    ShowResults();
    u32LastShowSensorDataTC = millis();
  }

#ifdef ENABLE_NETWORK
  if (GetTimeSpanMillis(u32LastSendToCloudTC) > 10000)
  {
    u32LastSendToCloudTC = millis();
    SendResultsToCloud();
  }

  handleServer();
#endif
}

///////////////////////////////////////////////////////////////////////////////
bool ReceiveDataPacket(DataPacket& dataPacket)
{
  bool bResult = false;

  uint8_t u8Buffer[RH_ASK_MAX_MESSAGE_LEN] = {0};
  uint8_t u8BufferLen = RH_ASK_MAX_MESSAGE_LEN;

  if (ASK_driver.recv(&u8Buffer[0], &u8BufferLen))
  {
    if (u8BufferLen == sizeof(DataPacket))
    {
      memcpy(&dataPacket, &u8Buffer[0], u8BufferLen);
      lcd.backlight();
      bResult = true;
    }
  }

  return bResult;
}

///////////////////////////////////////////////////////////////////////////////
void StoreRainCounterData(uint8_t u8Transmitter, float fTotalRainMM)
{
  SensorData* pSensorData = GetSensorData(u8Transmitter, eRainTotal);

	// Es müssen mindestens zwei Rainconter Messungen vorhanden sein,
  // um eine Abschätzung über die Zeit zu berechnen.
	if (pSensorData && pSensorData->GetTickCount() != 0)
	{
    // Regendifferenz seit der letzten Messung. abs() wegen der Rundungsfehler
    float fRainDiffMM = fTotalRainMM - pSensorData->GetSensorValue();

		// Ticks pro Minute berechnen...
		uint16_t u16TimeDiff = GetTimeSpanSeconds(pSensorData->GetTickCount());
    if (u16TimeDiff > 0)
    {
      float fRainMMperHour = 3600.0 * fRainDiffMM / (float)u16TimeDiff;
      StoreSensorData(u8Transmitter, eRainCurrent, fRainMMperHour);
    }
  }
  StoreSensorData(u8Transmitter, eRainTotal, fTotalRainMM);
}

///////////////////////////////////////////////////////////////////////////////
void StoreWindSpeedCounterData(uint8_t u8Transmitter, float fWindSpeedCounter)
{
  static float fPrevWindSpeedCounter = 0;

  float fWindSpeedkmh = 0.0;

  SensorData* pSensorData = GetSensorData(u8Transmitter, eWindSpeed);

  // Es müssen mindestens zwei WindSpeed counter Messungen vorhanden sein,
  // um eine Abschätzung über die Zeit zu berechnen.
	if (pSensorData && pSensorData->GetTickCount() != 0)
	{
    float fCounterDiff = fWindSpeedCounter - fPrevWindSpeedCounter;
    if (fCounterDiff >= 0)
    {
      // Ticks pro Minute berechnen...
  		uint16_t u16TimeDiff = GetTimeSpanSeconds(pSensorData->GetTickCount());
      if (u16TimeDiff > 0)
      {
        fWindSpeedkmh = 2.4 * fCounterDiff / (float)u16TimeDiff;
      }
    }
  }
  StoreSensorData(u8Transmitter, eWindSpeed, fWindSpeedkmh);
  fPrevWindSpeedCounter = fWindSpeedCounter;
}

///////////////////////////////////////////////////////////////////////////////
void ShowResults()
{
  static uint8_t u8Transmitter = 0;
  static int8_t i8Page = -1;

  // Wenn ein Sensor mindestens eine Wert enthält, soll das Display zunächst gelöscht werden
  if (HasAnySensorData(u8Transmitter))
  {
      // Anzeige der Sensornummer, zu dem die Messdaten gehören
      uint8_t u8Row = 0;
      i8Page = GetNextPage(u8Transmitter, i8Page);
#if (0)
      Serial.print("\nRam = ");
      Serial.print(freeRam());
      Serial.print(" Bytes\n");
#endif
      lcd.clear();
      lcd.setCursor(0, u8Row++);
      lcd.print(F("Sender:"));
      lcd.print(u8Transmitter + 1);
      if (i8Page != -1)
      {
        lcd.print(F("/"));
        lcd.print(i8Page + 1);
      }
      lcd.print(GetTransmitterName(u8Transmitter, i8Page).c_str());
#if (0)
      lcd.setCursor(0,0);
      lcd.print(F("Ram="));
      lcd.print(freeRam());
      lcd.print(" ");
#endif
      // Anzeige der gemessenen Temperatur
      ShowSensorDataOnLCD(u8Transmitter, eTemperatur, u8Row);

      // Anzeige der gemessenen Luftfeuchtigkeit
      ShowSensorDataOnLCD(u8Transmitter, eHumidity, u8Row);

      // Anzeige des gemessenen Luftdrucks
      ShowSensorDataOnLCD(u8Transmitter, ePressure, u8Row);

      // Anzeige der aktuellen Regenmenge hochgerechnet auf mm/h
      ShowSensorDataOnLCD(u8Transmitter, eRainCurrent, u8Row);

      // Anzeige der gemessenen Regenmenge
      ShowSensorDataOnLCD(u8Transmitter, eRainTotal, u8Row);

      // Die Anzeige der Verbrauchsdaten (Strom, Wasser und Gas) verteilen sich auf drei einzelne Seiten
      if (i8Page == 0)
      {
        // Anzeige des aktuellen Stromverbrauchs
        ShowSensorDataOnLCD(u8Transmitter, ePowerCurrent, u8Row);

        // Anzeige des Gesamtstromverbrauchs
        ShowSensorDataOnLCD(u8Transmitter, ePowerTotal, u8Row);
      }

      // Anzeige des aktuellen Wasserdurchlaufs in  l/minute
      if (i8Page == 1)
      {
        // Anzeige des aktuellen Wasserverbrauchs
        ShowSensorDataOnLCD(u8Transmitter, eWaterCurrent, u8Row);

        // Anzeige des Gesamtwasserverbrauchs
        ShowSensorDataOnLCD(u8Transmitter, eWaterTotal, u8Row);
      }

      if (i8Page == 2)
      {
        // Anzeige des aktuellen Gasverbrauchs
        ShowSensorDataOnLCD(u8Transmitter, eGasCurrent, u8Row);

        // Anzeige des Gesamtgasverbrauchs
        ShowSensorDataOnLCD(u8Transmitter, eGasTotal, u8Row);
      }

      // Anzeige der gemessenen Windgeschwindigkeit
      ShowSensorDataOnLCD(u8Transmitter, eWindSpeed, u8Row);
  }

  if (GetNextPage(u8Transmitter, i8Page) == -1)
  {
    i8Page = -1;

    // Schaut im KeyStore nach, wieviele Transmitter eingelehrnt sind.
    u8Transmitter++;
    KeyStore keyStore = ReadKeyStoreFromEEProm(u8Transmitter);
    if (keyStore.ulMagicHeader != MAGIC_HEADER)
      u8Transmitter = 0;
  }
}

///////////////////////////////////////////////////////////////////////////////
void ShowSensorDataOnLCD(uint8_t u8Transmitter, SensorType sensorType, uint8_t& u8Row)
{
  // Anzeige der gemessenen Sensordaten
  SensorData* pSensorData = GetSensorData(u8Transmitter, sensorType);
  if (pSensorData && pSensorData->GetSize() > 0)
  {
    lcd.setCursor(0, u8Row);
    // lcd.printPGM(g_sSensorName[sensorType]);
    char buffer[17] = {0}; // ACHTUNG: Der längste String ist 16 Bytes!
    strcpy_P(buffer, (char*)pgm_read_word(&g_sSensorName[sensorType])); // Necessary casts and dereferencing, just copy.
    lcd.print(buffer);

    if (GetTimeSpanSeconds(pSensorData->GetTickCount()) < RECEIVE_TIMEOUT)
    {
#if MAX_SENSOR_DATAS > 1
        lcd.print(pSensorData->GetArithmeticMean(), 1);
#else
        lcd.print(pSensorData->GetSensorValue(), 1);
#endif
      // lcd.printPGM(g_sLCDUnits[sensorType]);
      strcpy_P(buffer, (char*)pgm_read_word(&g_sLCDUnits[sensorType])); // Necessary casts and dereferencing, just copy.
      lcd.print(buffer);

#if MAX_SENSOR_DATAS > 1
        // Bei Sensoren mit mehreren Messwerten, eine Trendanzeige darstellen.
        if (pSensorData->GetSize() > 1)
        {
          lcd.setCursor(LCD_COLS-1, u8Row);
          float fTemperaturTrend= pSensorData->LinearRegression();
          if (fTemperaturTrend >= 0.02)
              lcd.write(CHAR_ARROW_UP);
          else if (fTemperaturTrend <= -0.02)
            lcd.write(CHAR_ARROW_DOWN);
        }
#endif
    }
    else
    {
      lcd.print(F("Keine Daten"));
    }
    u8Row++;
  }
}

#ifdef ENABLE_NETWORK
///////////////////////////////////////////////////////////////////////////////
bool setupWifi()
{
  if (bWifiConected)
  {
    bWifiConected = false;
    lcd.clear();
    lcd.print(F("Resetting Connection"));
    delay(1000);
    wifi.stopTCPServer();
    wifi.disableMUX();
    wifi.leaveAP();
  }

  lcd.clear();
  lcd.print(F("Wifi Connecting..."));
  //TRACE(F("Wifi Connecting...\n"));

  if (!wifi.restart())
  {
      // TRACE(F("Can't reset ESP8266\n"));
  }

  if (wifi.setOprToStation())
  {
      if (wifi.joinAP(SSID, PASSWORD))
      {
        //  TRACE(F("Wifi connected\n"));
        //  TRACE(SSID);
        //  TRACE("\n");
          bHasWifi = true;
          lcd.setCursor(0, 0);
          lcd.print(F("Wifi connected to    "));
          lcd.setCursor(0, 1);
          lcd.print(SSID);

          if (wifi.enableMUX())
          {
              if (wifi.startTCPServer(SERVER_PORT))
              {
                //TRACE(F("Tcp server started\n"));
                lcd.setCursor(0, 2);
                lcd.print(F("Webserver started"));
                if (wifi.setTCPServerTimeout(5))
                {
                    bWifiConected = true;
                }
              }
              else
              {
                //TRACE(F("Tcp server start failed\r\n"));
                lcd.setCursor(0, 2);
                lcd.print(F("Can't start Server"));
              }
          }
          else
          {
              //TRACE(F("enableMUX failed\n"));
          }
      }
      else
      {
          //TRACE(F("joinAP failed\n"));
          lcd.setCursor(0, 0);
          lcd.print(F("Network failure     "));
      }
  }
  else
  {
    //TRACE(F("setOprToStation failed\n"));
  }
  if (bWifiConected)
  {
      // TRACE(F("Wifi setup complete\r\n"));
  }
    return bWifiConected;
}

///////////////////////////////////////////////////////////////////////////////
void handleServer()
{
  if (bHasWifi)
  {
    uint8_t buffer[8] = {0};
    uint8_t mux_id;

    uint32_t len = wifi.recv(&mux_id, buffer, sizeof(buffer), 100);
    if (len > 0)
    {
      if (strncmp_P((const char*)buffer, PSTR("GET / "), 6) == 0)
        sendSensorDataToClient(mux_id);

      wifi.releaseTCP(mux_id);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
bool sendSensorDataToClient(uint8_t mux_id)
{
  // send a standard http response header
  sendToClient(mux_id, F("HTTP/1.1 200 OK\n"));
  sendToClient(mux_id, F("Content-Type: text/html\n"));
  sendToClient(mux_id, F("Connection: close\n"));   // the connection will be closed after completion of the response
  sendToClient(mux_id, F("Refresh: 45\n"));         // refresh the page automatically every 10 sec
  sendToClient(mux_id, "\n");

  // Und nun der Body....
  sendToClient(mux_id, F("<!DOCTYPE HTML>\n"));
  sendToClient(mux_id, F("<html>\n"));
  sendToClient(mux_id, F("<h1>Meine Wetterstation</h1>"));
  sendToClient(mux_id, F("<hr />\n"));

  // Anhand der Messdaten dynamisch die Html Seite aufbauen.
  String s;
  for (uint8_t u8Transmitter = 0; u8Transmitter < MAX_TRANSMITTER; u8Transmitter++)
  {
    // Wenn mindestens ein Messwert vorhaden ist "Sensor:" ausgeben
    if (HasAnySensorData(u8Transmitter))
    {
      s  = String(F("<h2>Sensor: ")) + String(u8Transmitter+1) + GetTransmitterName(u8Transmitter, -1) + F("</h2>\n");
      sendToClient(mux_id, s.c_str());

      SendHtmlEntry(mux_id, u8Transmitter, eTemperatur);
      SendHtmlEntry(mux_id, u8Transmitter, eHumidity);
      SendHtmlEntry(mux_id, u8Transmitter, ePressure);
      SendHtmlEntry(mux_id, u8Transmitter, eRainCurrent);
      SendHtmlEntry(mux_id, u8Transmitter, eRainTotal);
      SendHtmlEntry(mux_id, u8Transmitter, eWindSpeed);
      SendHtmlEntry(mux_id, u8Transmitter, ePowerCurrent);
      SendHtmlEntry(mux_id, u8Transmitter, ePowerTotal);
      SendHtmlEntry(mux_id, u8Transmitter, eWaterCurrent);
      SendHtmlEntry(mux_id, u8Transmitter, eWaterTotal);
      SendHtmlEntry(mux_id, u8Transmitter, eGasCurrent);
      SendHtmlEntry(mux_id, u8Transmitter, eGasTotal);
    }
  }
  sendToClient(mux_id, F("</html>\n"));

  return true;
}

///////////////////////////////////////////////////////////////////////////////
void SendHtmlEntry(uint8_t mux_id, uint8_t u8Transmitter, SensorType sensorType)
{
  SensorData* pSensorData = GetSensorData(u8Transmitter, sensorType);
  // Wenn Daten vorhanden, diese zum Client zurücksenden...
  if (pSensorData && pSensorData->GetTickCount() != 0)
  {
    String s;
#if MAX_SENSOR_DATAS > 1
    // Bei Sensoren mit mehreren Messwerten, soll ein Trend gezeigt werden.
    if (pSensorData->GetSize() > 1)
    {
      float fTemperaturTrend= pSensorData->LinearRegression();
      if (fTemperaturTrend >= 0.02)
        s = HTML_ARROW_UP;
      else if (fTemperaturTrend <= -0.02)
        s =  HTML_ARROW_DOWN;
      else
        s = HTML_SPACE;
    }
#endif
    char buffer[30] = {0};
    strcpy_P(buffer, (char*)pgm_read_word(&g_sSensorName[sensorType])); // Necessary casts and dereferencing, just copy.
    lcd.print(buffer);
    s += buffer;
    // s += g_sSensorName[SensorType];

    if (GetTimeSpanSeconds(pSensorData->GetTickCount()) < RECEIVE_TIMEOUT)
    {
#if MAX_SENSOR_DATAS > 1      
        s += String(pSensorData->GetArithmeticMean(), 1);
#else
        s += String(pSensorData->GetSensorValue(), 1);
#endif
        strcpy_P(buffer, (char*)pgm_read_word(&g_sHtmlUnits[sensorType])); // Necessary casts and dereferencing, just copy.
        lcd.print(buffer);
        s += buffer;

        //s += g_sHtmlUnits[SensorType];
    }
    else
    {
      s += F("Keine Daten");
    }
    s += F("<br />\n");

    sendToClient(mux_id, s.c_str());
  }
}

///////////////////////////////////////////////////////////////////////////////
void SendResultsToCloud()
{
	// Alle empfangenen und nicht schon weitergeleiteten Sensordaten versenden.
	uint8_t u8Transmitter = 0;
	do
	{
		if (bHasWifi)
		{
			// Senden der gemessenen Sensordaten, falls noch nicht erfolgt.
			if (HasAnyNewSensorData(u8Transmitter))
			{
				if (SendToEmonCms(u8Transmitter))
				{
          ClearAllNewValueFlags(u8Transmitter);
				}
			}
		}
		else
		{
      ClearAllNewValueFlags(u8Transmitter);
    }
		u8Transmitter++;
	}while((ReadKeyStoreFromEEProm(u8Transmitter).ulMagicHeader == MAGIC_HEADER) && (u8Transmitter < MAX_TRANSMITTER));
}

///////////////////////////////////////////////////////////////////////////////
#define HTTP_HEADER F("Host: emoncms.org\r\nUser-Agent: Mozilla\r\nAccept-Language: en-US,en;q=0.5\r\nAccept-Encoding: gzip, deflate\r\nConnection: keep-alive\r\n\r\n\r\n")
bool SendToEmonCms(uint8_t u8Transmitter)
{
  bool bResult = false;

  if (u8Transmitter < MAX_TRANSMITTER)
  {
	  if (bHasWifi)
	  {
  		uint8_t mux_id = 1;

  		if (wifi.createTCP(mux_id, EMON_CMS_SERVER, EMON_CMS_PORT))
  		{
        {
    		  String sResponse = F("GET /input/post.json?node=");
    		  sResponse += String(u8Transmitter);
          sResponse += F("&json={");
          bResult = sendToClient(mux_id, sResponse.c_str());
        }

        uint8_t u8JsonDataCounter = 0;

        bResult &= SendJsonEntry(mux_id, u8Transmitter, eTemperatur, u8JsonDataCounter);
        bResult &= SendJsonEntry(mux_id, u8Transmitter, eHumidity, u8JsonDataCounter);
        bResult &= SendJsonEntry(mux_id, u8Transmitter, ePressure, u8JsonDataCounter);
        bResult &= SendJsonEntry(mux_id, u8Transmitter, eRainTotal, u8JsonDataCounter);
        bResult &= SendJsonEntry(mux_id, u8Transmitter, eRainCurrent, u8JsonDataCounter);
        bResult &= SendJsonEntry(mux_id, u8Transmitter, eWindSpeed, u8JsonDataCounter);
        bResult &= SendJsonEntry(mux_id, u8Transmitter, ePowerCurrent, u8JsonDataCounter);
        bResult &= SendJsonEntry(mux_id, u8Transmitter, ePowerTotal, u8JsonDataCounter);
        bResult &= SendJsonEntry(mux_id, u8Transmitter, eWaterCurrent, u8JsonDataCounter);
        bResult &= SendJsonEntry(mux_id, u8Transmitter, eWaterTotal, u8JsonDataCounter);
        bResult &= SendJsonEntry(mux_id, u8Transmitter, eGasCurrent, u8JsonDataCounter);
        bResult &= SendJsonEntry(mux_id, u8Transmitter, eGasTotal, u8JsonDataCounter);

        String sResponse = F("}&apikey=");
  		  sResponse += String(EMON_WRITE_API_KEY);
  		  sResponse += F(" HTTP/1.1\r\n");
  		  bResult &= sendToClient(mux_id, sResponse.c_str());
  		  bResult &= sendToClient(mux_id, HTTP_HEADER);

  		  if (!bResult)
  		  {
    			String sErr = F("Send Sensor data to ");
    			sErr += String(EMON_CMS_SERVER);
    			sErr += F(" failed\n");
    			TRACE(sErr);
  		  }
  		  wifi.releaseTCP(mux_id);
  		}
  		else
  		{
          // TRACE(F("wifi.createTCP failed\n"));
          setupWifi();
      }
  	#ifdef _DEBUG1
  		String sTimeDiff = String(GetTimeSpanMillis(lStart));
  		TRACE(String(F("SendToEmonCms time ")) + sTimeDiff \
  				+ String(F(" ms ")) \
  				+ String(F(" Sensor ")) \
  				+ String(u8Transmitter) \
  				+ String("\n"));
  	#endif
  	 }
     else
      bResult = false;
  }
  return bResult;
}

///////////////////////////////////////////////////////////////////////////////
bool SendJsonEntry(uint8_t mux_id, uint8_t u8Transmitter, SensorType sensorType, uint8_t& nJsonDataCounter)
{
  bool bResult = false;

  SensorData* pSensorData = GetSensorData(u8Transmitter, sensorType);
  if (pSensorData && pSensorData->HasNewValue())
  {
    String sResponse;
    if (nJsonDataCounter > 0)
      sResponse = ",";

    //sResponse += g_sJsonName[sensorType];
    char buffer[30];
    strcpy_P(buffer, (char*)pgm_read_word(&g_sJsonName[sensorType])); // Necessary casts and dereferencing, just copy.
    sResponse += buffer;
#if MAX_SENSOR_DATAS > 1 
    sResponse += String(pSensorData->GetArithmeticMean(), 2);
#else
    sResponse += String(pSensorData->GetSensorValue(), 2);
#endif
    nJsonDataCounter++;
    bResult = sendToClient(mux_id, sResponse.c_str());
  }
  else
    bResult = true; // Wenn keine neuen Daten vorhanden sind, ist das auch ok.

  return bResult;
}

///////////////////////////////////////////////////////////////////////////////
bool sendToClient(uint8_t mux_id, const String& sMsg)
{
  bool bResult = false;
  if (bHasWifi)
  {
    bResult = wifi.send(mux_id, (const uint8_t*)sMsg.c_str(), sMsg.length());
  }
  return bResult;
}
#endif

///////////////////////////////////////////////////////////////////////////////
uint32_t GetTimeSpanMillis(uint32_t u32StartTime)
{
    uint32_t u32CurTime = millis();
    if (u32StartTime <= u32CurTime)
      return u32CurTime - u32StartTime;
    else
      return (((uint32_t)(-1)) - u32StartTime) + u32CurTime;
}

///////////////////////////////////////////////////////////////////////////////
uint16_t GetTimeSpanSeconds(uint16_t u16StartTime)
{
    uint16_t u16CurTime = (uint16_t)(millis() / 1000L);
    if (u16StartTime <= u16CurTime)
      return u16CurTime - u16StartTime;
    else
      return (((uint16_t)(-1)) - u16StartTime) + u16CurTime;
}

///////////////////////////////////////////////////////////////////////////////
bool WriteKeyStoreToEEProm(uint8_t u8Transmitter, const KeyStore& keyStore)
{
  bool bError = false;
  unsigned char* pKeyStore = (unsigned char*)&keyStore;
  int16_t iOffset = u8Transmitter * sizeof(KeyStore);
  for (int16_t nI = 0; nI < (int16_t)sizeof(KeyStore); nI++)
  {
    EEPROM.update(iOffset + nI, pKeyStore[nI]);
    if (EEPROM.read(iOffset + nI) != pKeyStore[nI])
      bError = true;
  }
  return !bError;
}

///////////////////////////////////////////////////////////////////////////////
KeyStore ReadKeyStoreFromEEProm(uint8_t u8Transmitter)
{
  KeyStore keyStore;
  unsigned char* pKeyStore = (unsigned char*)&keyStore;
  int16_t iOffset = u8Transmitter * sizeof(KeyStore);
  for (int16_t nI = 0; nI < (int16_t)sizeof(KeyStore); nI++)
  {
    pKeyStore[nI] = EEPROM.read(iOffset + nI);
  }
  return keyStore;
}

///////////////////////////////////////////////////////////////////////////////
void ClearEEProm()
{
  for (int16_t nI = 0; nI < MAX_TRANSMITTER * (int16_t)sizeof(KeyStore); nI++)
    EEPROM.update(nI, 0);
}

///////////////////////////////////////////////////////////////////////////////
void PairDevice(uint8_t u8Transmitter, uint16_t u16DeviceID)
{
  KeyStore keyStore;
  keyStore.ulMagicHeader  = MAGIC_HEADER;
  keyStore.u16DeviceID    = u16DeviceID;
  WriteKeyStoreToEEProm(u8Transmitter, keyStore);
}

///////////////////////////////////////////////////////////////////////////////
String GetTransmitterName(uint8_t u8Transmitter, int8_t i8Page)
{
  String sResult;
    switch (u8Transmitter)
    {
      case 0: sResult = TRANSMITTER_0; break;
      case 1: sResult = TRANSMITTER_1; break;
      case 2: sResult = TRANSMITTER_2; break;
      case 3: sResult = TRANSMITTER_3; break;
      case 4: sResult = TRANSMITTER_4; break;
      case 5:
      {
          // Transmitter5 ist der Strom, Wasser und Gas messer.
          if (i8Page == -1) sResult = TRANSMITTER_5;
          if (i8Page == 0) sResult  = TRANSMITTER_5_0;
          if (i8Page == 1) sResult  = TRANSMITTER_5_1;
          if (i8Page == 2) sResult  = TRANSMITTER_5_2;

        break;
      }
      case 6: sResult = TRANSMITTER_6; break;
      case 7: sResult = TRANSMITTER_7; break;
  }

  return sResult;
}

///////////////////////////////////////////////////////////////////
int8_t GetNextPage(uint8_t u8Transmitter, int8_t i8StartPage)
{
  int8_t i8NextPage = -1;

  for (int8_t i8Page = i8StartPage+1; (i8Page < 3) && (i8NextPage == -1); i8Page++)
  {
    switch (i8Page)
    {
      case 0: // PowerMeter page
        if (HasPowerMeterData(u8Transmitter))
        {
          i8NextPage = i8Page;
          break;
        }
      case 1: // WaterMeter page
        if (HasWaterMeterData(u8Transmitter))
        {
          i8NextPage = i8Page;
          break;
        }
      case 2: // GasMeter page
        if (HasGasMeterData(u8Transmitter))
        {
          i8NextPage = i8Page;
          break;
        }
      }
  }

  return i8NextPage;
}

///////////////////////////////////////////////////////////////////
bool HasPowerMeterData(uint8_t u8Transmitter)
{
  SensorData* pData1 = GetSensorData(u8Transmitter, ePowerTotal);
  SensorData* pData2 = GetSensorData(u8Transmitter, ePowerCurrent);

  return ((pData1 && pData1->GetTickCount() != 0) || (pData2 && pData2->GetTickCount() != 0));
}

///////////////////////////////////////////////////////////////////
bool HasWaterMeterData(uint8_t u8Transmitter)
{
  SensorData* pData1 = GetSensorData(u8Transmitter, eWaterTotal);
  SensorData* pData2 = GetSensorData(u8Transmitter, eWaterCurrent);

  return ((pData1 && pData1->GetTickCount() != 0) || (pData2 && pData2->GetTickCount() != 0));
}

///////////////////////////////////////////////////////////////////
bool HasGasMeterData(uint8_t u8Transmitter)
{
  SensorData* pData1 = GetSensorData(u8Transmitter, eGasTotal);
  SensorData* pData2 = GetSensorData(u8Transmitter, eGasCurrent);

  return ((pData1 && pData1->GetTickCount() != 0) || (pData2 && pData2->GetTickCount() != 0));
}

/*
///////////////////////////////////////////////////////////////////
uint32_t FloatToUint32(float fInputValue)
{
	uint32_t u32Result = 0;

	if (sizeof(u32Result) == sizeof(fInputValue))
		memcpy(&u32Result, &fInputValue, sizeof(u32Result));

	return u32Result;
}
*/

///////////////////////////////////////////////////////////////////
float Uint32ToFloat(uint32_t u32InputValue)
{
	float fResult = 0.0;

	if (sizeof(fResult) == sizeof(u32InputValue))
		memcpy(&fResult, &u32InputValue, sizeof(fResult));

	return fResult;
}
