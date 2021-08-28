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

#include "ESP8266WiFi.h"
#include "sensorData.h"
#include "433MHz_Receiver.h"
#include "ntp.h"
#include <pgmspace.h>

extern unsigned long ntpUnixTime(WiFiUDP& udp);
extern std::vector<SensorData> m_SensorDatas;
uint8_t u8MsgIDs[MAX_TRANSMITTER] = {0};

///////////////////////////////////////////////////////////////////////////////
void setup()
{
  #ifdef _DEBUG
    Serial.begin(115200);
  #endif

  TRACE(F("\nStarting...\n"));
  EEPROM.begin(MAX_TRANSMITTER*sizeof(KeyStore));

  // Clear EEProm Switch
  pinMode(switchPin, INPUT);
  digitalWrite(switchPin, HIGH); // Internen Pullup einschalten

  // Display initialisieren
  tft.begin();
  tft.setRotation( ROTATION_270);
  #ifdef NODEMCU
    pinMode(backGndPin, OUTPUT);
    digitalWrite(backGndPin, HIGH);
  #endif
  
  #if (0)
    // Font ausgeben
    tft.setTextSize(2);
    for (int nI = 78; nI < 256; nI++)
    {
        tft.print(nI);
        tft.print("=");
        tft.write(nI);
        tft.print(" ");
    }
    while(true)
    {
      delay(50);
    }
  #endif

  // Wenn nach dem Reset 10 Sekunden der Taster betätigt wird
  // werden alle Keystore Einträge aus dem EEProm gelöscht.
  // Dies ist quasi der Werksreset!
  uint32_t u32TC = millis();
  int nCounter = 10;

  if (digitalRead(switchPin) == LOW)
    tft_ShowFrames(true);
  while (digitalRead(switchPin) == LOW)
  {
    tft_Print(25, 90, 2, true, F("Clear EEprom in:"), TFT_GREEN);
    tft_Print(-1, -1, 2, true, String(nCounter).c_str(), TFT_GREEN);
    tft_Print(-1, -1, 2, true, F("s "), TFT_GREEN);

    if (GetTimeSpanMillis(u32TC) > 1000)
    {
      u32TC = millis();
      nCounter--;
      if (nCounter == 0)
      {
        tft_Print(25, 90, 2, true, F("Clearing EEProm...     "), TFT_GREEN);
        ClearEEProm();
        delay(2000);
        break;
      }
    }
    delay(50);
  }

  ASK_driver.init();

  tft_ShowFrames();
  tft_Print(12, 90,  2, true, F("To pair the device, "), TFT_GREEN);
  tft_Print(12, 110, 2, true, F("reset the transmitter now"), TFT_GREEN);

  // Warte 7 Sekunden auf die Pairing Message des Transmitters...
  KeyStore keyStore;
  DataPacket dataPacket;
  u32TC = millis();
  do
  {
    delay(50);
    if (ReceiveDataPacket(dataPacket))
    {
        // Ist es eine Pairing Anforderung?
        if (dataPacket.eMsg == eMessage::MsgPairing)
        {
          TRACE(F("Receiving pairing msg\n"));

          // Suche im Keystore anhand der Seriennummer, ob dieser Transmitter
          // schon bekannt ist. Wenn ja, dann setzen den Rollingcode neu
          for (uint8_t u8Transmitter = 0; u8Transmitter < MAX_TRANSMITTER; u8Transmitter++)
          {
            keyStore = ReadKeyStoreFromEEProm(u8Transmitter);
            if (keyStore.ulMagicHeader == MAGIC_HEADER)
            {
                if (keyStore.u16DeviceID == dataPacket.u16DeviceID)
                {
                  tft_ShowFrames();
                  tft_Print(20, 90,  2, true, F("Repairing success"), TFT_GREEN);
                  tft_Print(20, 110, 2, true, F("Transmitter: "), TFT_GREEN);
                  tft_Print(-1, -1, 2, true, String(u8Transmitter + 1).c_str(), TFT_GREEN);

                  delay(2000);
                  break;
                }
            }
            else // Empty Entry.
            {
              // Die erhaltenen Seriennummer sichern.
              keyStore.ulMagicHeader  = MAGIC_HEADER;
              keyStore.u16DeviceID    = dataPacket.u16DeviceID;
              keyStore.fTotalRainMM   = 0;
              keyStore.u8Month        = 0;

              // Neuen KeyStore Eintrag im EEProm sichern.
              if (WriteKeyStoreToEEProm(u8Transmitter, keyStore))
              {
                tft_ShowFrames();
                tft_Print(20, 90,  2, true, F("New pairing success"), TFT_GREEN);
                tft_Print(20, 110, 2, true, F("Transmitter: "), TFT_GREEN);
                tft_Print(-1, -1, 2, true, String(u8Transmitter + 1).c_str(), TFT_GREEN);

                delay(2000);
              }
              break;
            }
          }
          break;
        }
     }
  }while ((keyStore.ulMagicHeader != MAGIC_HEADER) && (GetTimeSpanMillis(u32TC) < 7000));

  if (setupWifi())
  {
    server.on("/", handleRoot);
    server.on("/sensor/temperature", handleTemperature);
    server.begin();

  //  SendSMTP(1);
  }
  delay(2000);

  tft_ShowFrames();
  tft_Print(30, 110, 2, true, F("Receiver ready"), TFT_GREEN);
  tft.setTextWrap(false);

  delay(1000);
  tft_ShowFrames();

  tft_Disable7SegmentDisplay();
}

///////////////////////////////////////////////////////////////////////////////
void loop()
{
  delay(1);
  static uint32_t u32LastSendToCloudTC = 0;
  static uint32_t u32LastShowSensorDataGFXTC = 0;
  static uint32_t u32LastTimeSyncTC = 0;
  static uint32_t u32LastTransmitterCheckTC = 0;

  KeyStore keyStore;
  DataPacket dataPacket;

  uint8_t u8Transmitter = 0;
  bool bFound = false;

  if (ReceiveDataPacket(dataPacket))
  {
    digitalWrite(LED_BUILTIN, HIGH);

    // Suche Anhand der DeviceID die passende Transmitternummer aus dem KeyStore
    for (uint8_t u8I = 0; u8I < MAX_TRANSMITTER; u8I++)
    {
      keyStore = ReadKeyStoreFromEEProm(u8I);
      if ((keyStore.ulMagicHeader == MAGIC_HEADER) && (dataPacket.u16DeviceID == keyStore.u16DeviceID))
      {
        u8Transmitter = u8I;
        bFound = true;
        break;
      }
    }
    if (bFound)
    {
      // Nach dem erfolgreichen Empfang einer Nachricht, erstmal
      // SendResultsToCloud() nicht aufrufen, da evtl.
      // noch weitere Daten eintreffen.
      u32LastSendToCloudTC = millis();
#ifdef _DEBUG1
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
      tft_ReceiveFlag(u8Transmitter);

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
              int nTemperature = (int)(dataPacket.u32Payload >> 20) & 0x03ff;
              int nHumidity    = (int)(dataPacket.u32Payload >> 10) & 0x03ff;
              int nPressure    = (int)(dataPacket.u32Payload >> 00) & 0x03ff;

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

  if (GetTimeSpanMillis(u32LastShowSensorDataGFXTC) > 2000)
  {
    if (ShowResultsGFX())
      u32LastShowSensorDataGFXTC = millis();
    else
      u32LastShowSensorDataGFXTC = 0;
  }

  if (GetTimeSpanMillis(u32LastSendToCloudTC) > 10000)
  {
    u32LastSendToCloudTC = millis();
    SendResultsToCloud();
  }

  if (bWifiConected)
  {
    // Jede Minute die Uhrzeit mit dem NTP Server synchronisieren.
    if (GetTimeSpanMillis(u32LastTimeSyncTC) > 60L*1000L)
    {
      u32LastTimeSyncTC = millis();
      ntpUnixTime(udp);
    }

    if (GetTimeSpanMillis(u32LastTransmitterCheckTC) > 12L*3600L*1000L)
    {
      if (CheckAllTransmittersForSendingData())
        u32LastTransmitterCheckTC = millis();
    }

  }

  if (bWifiConected)
  {
      server.handleClient();
      tft_ShowTimeDate();
  }
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
      bResult = true;
      #ifdef _DEBUG1
            TRACE(F("(Msg="));
            TRACE(dataPacket.eMsg);
            TRACE(F(" MsgID="));
            TRACE(dataPacket.u8MsgID);
            TRACE(F(" Payload="));
            TRACE(dataPacket.u32Payload);
            TRACE(F(" DeviceID="));
            TRACE(dataPacket.u16DeviceID);
            TRACE(")\n");

          if (dataPacket.eMsg == MsgWeatherCompact)
          {
            int nTemperature = (int)(dataPacket.u32Payload >> 20) & 0x03ff;
            int nHumidity    = (int)(dataPacket.u32Payload >> 10) & 0x03ff;
            int nPressure    = (int)(dataPacket.u32Payload >> 00) & 0x03ff;

            TRACE(F("(Temperature="));
            TRACE(String((float)(nTemperature - 300)/10.0, 1));
            TRACE(F(" Humidity="));
            TRACE(String((float)(nHumidity)/10.0, 1));
            TRACE(F(" Pressure="));
            TRACE(String((float)(nPressure + 9630)/10.0, 1));
            TRACE(F(" \n"));
          }
      #endif
    }
    else
    {
        TRACE(F("u8BufferLen != sizeof(DataPacket)\n"));
        TRACE(u8BufferLen);
        TRACE(" != ");
        TRACE(sizeof(DataPacket));
        TRACE("\n");
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
		uint32_t u32TimeDiff = GetTimeSpanSeconds(pSensorData->GetTickCount());
    if (u32TimeDiff > 0)
    {
      float fRainMMperHour = 3600.0 * fRainDiffMM / (float)u32TimeDiff;
      StoreSensorData(u8Transmitter, eRainCurrent, fRainMMperHour);
    }
  }
  StoreSensorData(u8Transmitter, eRainTotal, fTotalRainMM);

  if (bWifiConected)
  {
    // Wenn ein neuer Monat begonnen hat, wird die aktuelle Gesamtregenmenge
    // imm EEProm gespeichert, um die aktuelle Monatsregenmenge anzeigen zu können.
    if (year() > 1970)
    {
      KeyStore  keyStore = ReadKeyStoreFromEEProm(u8Transmitter);
      if (keyStore.u8Month != month())
      {
        SensorData* pSensorData = GetSensorData(u8Transmitter, eRainTotal);
        if (pSensorData && pSensorData->GetSize() > 0)
        {
          keyStore.fTotalRainMM = fTotalRainMM;
          keyStore.u8Month = month();
          WriteKeyStoreToEEProm(u8Transmitter, keyStore);
        }
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
void StoreWindSpeedCounterData(uint8_t u8Transmitter, float fWindSpeedCounter)
{
  static float fPrevWindSpeedCounter = 0;

  float fWindSpeedkmh = 0.0;

  SensorData* pSensorData = GetSensorData(u8Transmitter, eWindSpeed);

  // Es müssen mindestens zwei WindSpeed conter Messungen vorhanden sein,
  // um eine Abschätzung über die Zeit zu berechnen.
	if (pSensorData && pSensorData->GetTickCount() != 0)
	{
    float fCounterDiff = fWindSpeedCounter - fPrevWindSpeedCounter;
    if (fCounterDiff >= 0)
    {
      // Ticks pro Minute berechnen...
  		uint32_t u32TimeDiff = GetTimeSpanSeconds(pSensorData->GetTickCount());
      if (u32TimeDiff > 0)
      {
        fWindSpeedkmh = 2.4 * fCounterDiff / (float)u32TimeDiff;
      }
    }
  }
  StoreSensorData(u8Transmitter, eWindSpeed, fWindSpeedkmh);
  fPrevWindSpeedCounter = fWindSpeedCounter;
}

///////////////////////////////////////////////////////////////////////////////
bool ShowResultsGFX()
{
  static int u8Transmitter = 0;
  static int nPage = 0;

  bool bShowPage = false;
  switch (nPage)
  {
      case 0:
        // Anzeige der gemessenen Temperatur
        if (ShowSensorDataOnGFX(u8Transmitter, eTemperatur))
          bShowPage = true;
        break;
      case 1:
        // Anzeige der gemessenen Luftfeuchtigkeit
        if (ShowSensorDataOnGFX(u8Transmitter, eHumidity))
          bShowPage = true;
        break;
      case 2:
        // Anzeige des gemessenen Luftdrucks
        if (ShowSensorDataOnGFX(u8Transmitter, ePressure))
          bShowPage = true;
        break;
      case 3:
        // Anzeige der gemessenen aktuellen Regenmenge
        if (ShowSensorDataOnGFX(u8Transmitter, eRainCurrent))
          bShowPage = true;
        break;
      case 4:
        // Anzeige der gemessenen Gesamtregenmenge
        if (ShowSensorDataOnGFX(u8Transmitter, eRainTotal))
          bShowPage = true;
        break;
      case 5:
        // Anzeige des gemessenen aktellen Stromverbrauchs
        if (ShowSensorDataOnGFX(u8Transmitter, ePowerCurrent))
          bShowPage = true;
        break;
      case 6:
        // Anzeige des gemessenen Gesamtstromverbrauchs
        if (ShowSensorDataOnGFX(u8Transmitter, ePowerTotal))
          bShowPage = true;
        break;
      case 7:
        // Anzeige des gemessenen aktuellen Wasserverbrauchs
        if (ShowSensorDataOnGFX(u8Transmitter, eWaterCurrent))
          bShowPage = true;
        break;
      case 8:
        // Anzeige des gemessenen Gesamtwasserverbrauchs
        if (ShowSensorDataOnGFX(u8Transmitter, eWaterTotal))
          bShowPage = true;
        break;
      case 9:
        // Anzeige der gemessenen aktuellen Gasverbrauchs
        if (ShowSensorDataOnGFX(u8Transmitter, eGasCurrent))
          bShowPage = true;
        break;
      case 10:
        // Anzeige der gemessenen Gesamtgasverbrauchs
        if (ShowSensorDataOnGFX(u8Transmitter, eGasTotal))
          bShowPage = true;
        break;
      case 11:
          // Anzeige der gemessenen Windgeschwindigkeit
          if (ShowSensorDataOnGFX(u8Transmitter, eWindSpeed))
            bShowPage = true;
          break;
      default:
        // Schaut im KeyStore nach, wieviele Transmitter eingelehrnt sind.
        u8Transmitter++;
        nPage = -1;

        KeyStore keyStore = ReadKeyStoreFromEEProm(u8Transmitter);
        if (keyStore.ulMagicHeader != MAGIC_HEADER)
          u8Transmitter = 0;
  }
  nPage++;

  // Rahmen erneut zeichnen
  if (bShowPage)
     tft_ShowFrames(false);

  return bShowPage;
}

///////////////////////////////////////////////////////////////////////////////
bool ShowSensorDataOnGFX(uint8_t u8Transmitter, SensorType sensorType)
{
  bool bResult = false;

  // Anzeige der gemessenen Sensordaten
  SensorData* pSensorData = GetSensorData(u8Transmitter, sensorType);
  if (pSensorData && pSensorData->GetSize() > 0)
  {
    bResult = true;

    tft_ShowTitle(u8Transmitter, sensorType);

    if (GetTimeSpanSeconds(pSensorData->GetTickCount()) < RECEIVE_TIMEOUT)
    {
      float fValue = pSensorData->GetArithmeticMean();

      tft_ShowSensorValue(u8Transmitter, sensorType, fValue);

      // Bei Sensoren mit mehreren Messwerten, eine Trendanzeige darstellen.
      if (pSensorData->GetSize() > 1)
      {
        float fTrend= pSensorData->LinearRegression();
        if (fTrend >= 0.02)
          tft_Print(320, 90, 3, false, F("\x18"), LED_COLOR);   // Pfeil hoch
        else if (fTrend <= -0.02)
           tft_Print(320, 90, 3, false, F("\x19"), LED_COLOR);  // Pfeil runter
        else
          tft_Print(320, 90, 3, false, F(" "), LED_COLOR);      // Pfeil löschen
      }
    }
    else
    {
        tft_Disable7SegmentDisplay();
    }
  }

  return bResult;
}

///////////////////////////////////////////////////////////////////////////////
bool setupWifi()
{
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  //WiFi.config(ip, gateway, subnet);
  WiFi.begin(MY_SSID, MY_PASSWORD);
  String sMsg = F("Connecting to ");
  sMsg += MY_SSID;

  tft_ShowFrames();
  tft_Print(10, 90,  2, true, sMsg.c_str(), TFT_GREEN);

  uint32_t u32TC = millis();
  int nI = 0;
  while ((WiFi.status() != WL_CONNECTED) && (GetTimeSpanMillis(u32TC) < 15000))
  {
    delay(1000);
    tft.print(".");
    if (nI++ == 3)
    {
      nI = 0;
      tft_Print(10, 90,  2, true, sMsg.c_str(), TFT_GREEN);
      tft.print("    ");
      tft_Print(10, 90,  2, true, sMsg.c_str(), TFT_GREEN);
    }
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    bWifiConected = true;

    char sBuffer[16] = {0};
    IPAddress ip = WiFi.localIP();
    sprintf(sBuffer, "%d.%d.%d.%d", ip[0],ip[1],ip[2],ip[3]);

    tft_Print(12, 110, 2, true, F("Connected to: "), TFT_GREEN);
    tft.print(MY_SSID);
    tft_Print(12, 130, 2, true, sBuffer, TFT_GREEN);
   }
   else
   {
     bWifiConected = false;

     tft_Print(12, 110, 2, true, F("Connection failed"), TFT_GREEN);
   }
   return bWifiConected;
}

///////////////////////////////////////////////////////////////////////////////
void handleRoot()
{
  String sWebPage = createWebPage();
  if (sWebPage.length() > 0)
    server.send(200, "text/html", sWebPage);
}

///////////////////////////////////////////////////////////////////////////////
void handleTemperature()
{

}

///////////////////////////////////////////////////////////////////////////////
String createWebPage()
{
  String sWebPage;

  // Und nun der Body....
  sWebPage += F("<!DOCTYPE HTML>\n");
  sWebPage += F("<html>\n");
  sWebPage += F("<h1>Meine Wetterstation</h1>");
  sWebPage += F("<hr />\n");

  // Anhand der Wetterdaten dynamisch die Html Seite aufbauen.
  for (uint8_t u8Transmitter = 0; u8Transmitter < MAX_TRANSMITTER; u8Transmitter++)
  {
    // Wenn mindestens ein Messwert vorhaden ist "Sensor:" ausgeben
    if (HasAnySensorData(u8Transmitter))
    {
      String s = String(F("<h2>Sensor: ")) + String(u8Transmitter+1) + GetTransmitterName(u8Transmitter, -1) + F("</h2>\n");
      sWebPage += s;
      sWebPage += CreateHtmlEntry(u8Transmitter, eTemperatur);
      sWebPage += CreateHtmlEntry(u8Transmitter, eHumidity);
      sWebPage += CreateHtmlEntry(u8Transmitter, ePressure);
      sWebPage += CreateHtmlEntry(u8Transmitter, eRainCurrent);
      sWebPage += CreateHtmlEntry(u8Transmitter, eRainTotal);
      sWebPage += CreateHtmlEntry(u8Transmitter, ePowerCurrent);
      sWebPage += CreateHtmlEntry(u8Transmitter, ePowerTotal);
      sWebPage += CreateHtmlEntry(u8Transmitter, eWaterCurrent);
      sWebPage += CreateHtmlEntry(u8Transmitter, eWaterTotal);
      sWebPage += CreateHtmlEntry(u8Transmitter, eGasCurrent);
      sWebPage += CreateHtmlEntry(u8Transmitter, eGasTotal);
      sWebPage += CreateHtmlEntry(u8Transmitter, eWindSpeed);
    }
  }
  sWebPage += F("</html>\n");

  return sWebPage;
}

///////////////////////////////////////////////////////////////////////////////
String CreateHtmlEntry(uint8_t u8Transmitter, SensorType sensorType)
{
  String sResponse;
  SensorData* pSensorData = GetSensorData(u8Transmitter, sensorType);

  // Wenn Daten vorhanden, diese zum Client zurücksenden...
  if (pSensorData && pSensorData->GetTickCount() != 0)
  {
    // Bei Sensoren mit mehreren Messwerten, soll ein Trend gezeigt werden.
    if (pSensorData->GetSize() > 1)
    {
      float fTrend= pSensorData->LinearRegression();
      if (fTrend >= 0.02)
        sResponse = HTML_ARROW_UP;
      else if (fTrend <= -0.02)
        sResponse =  HTML_ARROW_DOWN;
      else
        sResponse = HTML_SPACE;
    }

    String sSensorName = FPSTR(g_sSensorName[sensorType]);
    float  fSensorValue = pSensorData->GetArithmeticMean();

    // Die Anzeige der Gesamtregenmenge ist pro Monat.
    if (sensorType == eRainTotal)
    {
        // Von der aktuellen Gesamtregenmenge die Gesamtregenmenge
        // des Vormonats abziehen.
        KeyStore keyStore = ReadKeyStoreFromEEProm(u8Transmitter);
        fSensorValue -= keyStore.fTotalRainMM;
        String sMonth = FPSTR(g_sMonth[keyStore.u8Month]);
        sSensorName += sMonth;
        sSensorName += ":";
    }

    sResponse += sSensorName;

    if (GetTimeSpanSeconds(pSensorData->GetTickCount()) < RECEIVE_TIMEOUT)
    {
        sResponse += String(fSensorValue, 1);
        sResponse += FPSTR(g_sHtmlUnits[sensorType]);
    }
    else
    {
      sResponse += F("Keine Daten");
    }
    sResponse += F("<br />\n");
  }
  return sResponse;
}

///////////////////////////////////////////////////////////////////////////////
void SendResultsToCloud()
{
	// Alle empfangenen und nicht schon weitergeleiteten Sensordaten versenden.
	int u8Transmitter = 0;
	do
	{
		if (bWifiConected)
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
bool SendToEmonCms(uint8_t u8Transmitter)
{
  bool bResult = false;

  if (u8Transmitter < MAX_TRANSMITTER)
  {
	  if (bWifiConected)
	  {
      WiFiClient client;
      // Verbindung zu emoncms.org bzw. emonpi.fritz.box aufbauen
  		if (client.connect(EMON_CMS_SERVER, EMON_CMS_PORT) == 1)
  		{
  		  String sResponse = EMON_REQUEST;
  		  sResponse += String(u8Transmitter);
        sResponse += F("&json={");
        uint8_t u8JsonDataCounter = 0;

        sResponse += CreateJsonEntry(u8Transmitter, eTemperatur, u8JsonDataCounter);
        sResponse += CreateJsonEntry(u8Transmitter, eHumidity, u8JsonDataCounter);
        sResponse += CreateJsonEntry(u8Transmitter, ePressure, u8JsonDataCounter);
        sResponse += CreateJsonEntry(u8Transmitter, eRainTotal, u8JsonDataCounter);
        sResponse += CreateJsonEntry(u8Transmitter, eRainCurrent, u8JsonDataCounter);
        sResponse += CreateJsonEntry(u8Transmitter, ePowerCurrent, u8JsonDataCounter);
        sResponse += CreateJsonEntry(u8Transmitter, ePowerTotal, u8JsonDataCounter);
        sResponse += CreateJsonEntry(u8Transmitter, eWaterCurrent, u8JsonDataCounter);
        sResponse += CreateJsonEntry(u8Transmitter, eWaterTotal, u8JsonDataCounter);
        sResponse += CreateJsonEntry(u8Transmitter, eGasCurrent, u8JsonDataCounter);
        sResponse += CreateJsonEntry(u8Transmitter, eGasTotal, u8JsonDataCounter);
        sResponse += CreateJsonEntry(u8Transmitter, eWindSpeed, u8JsonDataCounter);

        sResponse +=  F("}&apikey=");
  		  sResponse += String(EMON_WRITE_API_KEY);
  		  sResponse += F(" HTTP/1.1\r\n");
        sResponse += HTTP_HEADER;

        uint32_t u32StartTC = millis();
        bResult = sendHttpRequest(client, sResponse.c_str());
        #ifdef _DEBUG
      		String sTimeDiff = String(GetTimeSpanMillis(u32StartTC));
      		TRACE(String(F("sendHttpRequest time ")) + sTimeDiff \
      				+ String(F(" ms ")) \
      				+ String(F(" Sensor ")) \
      				+ String(u8Transmitter) \
      				+ String("\n"));
      	#endif
        if (bResult)
        {
          TRACE("Server Antwort:\n")
          String sResult = ReadAnswer(client);
          TRACE(sResult);
          TRACE("\n");
        }
        else
        {
          String sErr = F("Send Sensor data to ");
          sErr += String(EMON_CMS_SERVER);
          sErr += F(" failed\n");
          TRACE(sErr);
        }

        client.stop();
  		}
      else
  		{
        client.stop();
        Serial.println("Verbindungsaufbau nicht moeglich!!!");
      }
  	 }
     else
        bResult = false;
  }

  return bResult;

}
///////////////////////////////////////////////////////////////////////////////
String CreateJsonEntry(uint8_t u8Transmitter, SensorType sensorType, uint8_t& nJsonDataCounter)
{
  String sResponse;

  SensorData* pSensorData = GetSensorData(u8Transmitter, sensorType);
  if (pSensorData && pSensorData->HasNewValue())
  {
    if (nJsonDataCounter > 0)
      sResponse = ",";

    sResponse += FPSTR(g_sJsonName[sensorType]);

    sResponse += String(pSensorData->GetArithmeticMean(), 2);
    nJsonDataCounter++;
  }
  return sResponse;
}

///////////////////////////////////////////////////////////////////////////////
bool sendHttpRequest(WiFiClient& client, const String& sMsg)
{
  bool bError = false;

  // Hat sich der ESP8266 zu Beginn erfolgreich mit dem Router verbunden?
  if (bWifiConected)
  {
    client.print(sMsg);
    unsigned long timeout = millis();
    while (client.available() == 0)
    {
     if (millis() - timeout > 5000)
     {
        TRACE("Timeout!\n");
        TRACE("Http request failed!\n");
        bError = true;
        break;
     }
     delay(100);
   }
 }
 return !bError;
}

///////////////////////////////////////////////////////////////////////////////
String ReadAnswer(WiFiClient& client)
{
  String sResult;
  while(client.available())
  {
    sResult += client.readStringUntil('\r');
  }

  return sResult;
}

///////////////////////////////////////////////////////////////////////////////
bool SendSMTP(uint8_t u8Transmitter)
{
  bool bResult = false;

  if (u8Transmitter < MAX_TRANSMITTER)
  {
	  if (bWifiConected)
	  {
      WiFiClient client;

      // Verbindung zu emoncms aufbauen
  		if (client.connect(SMTP_SERVER, SMTP_PORT) == 1)
  		{
        String sResult;
        String sMsg;

        bResult = sendHttpRequest(client, "HELO weatherstation.fritz.box\r\n");
        sResult = ReadAnswer(client);
        TRACE(sResult);

        /* Authentifikation funktioniert irgendwie nicht. Auch nicht über telnet
        // Nur zu lokalen adressen. da braucht es keine Authentifikation
        bResult &= sendHttpRequest(client, "AUTH LOGIN\r\n");
        sResult += ReadAnswer(client);

        // admin YWRtaW4=
        bResult &= sendHttpRequest(client, "YWRtaW4=\r\n");  //SECRET logn Base64 Encoded
        sResult += ReadAnswer(client);

        // suapygomumrk c3VhcHlnb211bXJr
        bResult &= sendHttpRequest(client, "c3VhcHlnb211bXJr\r\n"); //SECRET password Base64 Encoded
        sResult += ReadAnswer(client);
*/

        bResult &= sendHttpRequest(client, "MAIL FROM: wetterstation@fritz.box\r\n");
        sResult += ReadAnswer(client);
        TRACE(sResult);

        bResult &= sendHttpRequest(client, "RCPT TO: martin\r\n");
        sResult = ReadAnswer(client);
        TRACE(sResult);

        bResult &= sendHttpRequest(client, "DATA\r\n");
        sResult = ReadAnswer(client);
        TRACE(sResult);

        sMsg = "From: Wetterstation\r\n";
        sMsg += "To: martin@fritz.box\r\n";
        sMsg += "Subject: Batterie leer\r\n";
        sMsg += "\r\n\r\n";
        sMsg += "Der Transmitter ";
        sMsg += String(u8Transmitter+1);
        sMsg += " sendet seit mehr als 12 Stunden keine Daten mehr\r\n";
        sMsg += ".\r\n";
        bResult &= sendHttpRequest(client, sMsg.c_str());
        sResult = ReadAnswer(client);
        TRACE(sResult);

        if (!bResult)
        {
          String sErr = F("Send SMTP to ");
          sErr += String(SMTP_SERVER);
          sErr += F(" failed\n");
          TRACE(sErr);
        }

        client.stop();
  		}
      else
  		{
        client.stop();
        Serial.println("Connection to SMTP failed!!");
      }
  	 }
     else
        bResult = false;
  }

  return bResult;
}

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
uint32_t GetTimeSpanSeconds(uint32_t u32StartTime)
{
    uint32_t u32CurTime = millis() / 1000L;
    if (u32StartTime <= u32CurTime)
      return u32CurTime - u32StartTime;
    else
      return (((uint32_t)(-1)) - u32StartTime) + u32CurTime;
}

///////////////////////////////////////////////////////////////////////////////
bool WriteKeyStoreToEEProm(uint8_t u8Transmitter, const KeyStore& keyStore)
{
  bool bError = false;
  unsigned char* pKeyStore = (unsigned char*)&keyStore;
  int nOffset = u8Transmitter * sizeof(KeyStore);
  for (int nI = 0; nI < (int)sizeof(KeyStore); nI++)
  {
    if (EEPROM.read(nOffset + nI) != pKeyStore[nI])
      EEPROM.write(nOffset + nI, pKeyStore[nI]);
    if (EEPROM.read(nOffset + nI) != pKeyStore[nI])
      bError = true;
  }

  EEPROM.commit();

  return !bError;
}

///////////////////////////////////////////////////////////////////////////////
KeyStore ReadKeyStoreFromEEProm(uint8_t u8Transmitter)
{
  KeyStore keyStore;
  unsigned char* pKeyStore = (unsigned char*)&keyStore;
  int nOffset = u8Transmitter * sizeof(KeyStore);
  for (int nI = 0; nI < (int)sizeof(KeyStore); nI++)
  {
    pKeyStore[nI] = EEPROM.read(nOffset + nI);
  }
  return keyStore;
}

///////////////////////////////////////////////////////////////////////////////
void ClearEEProm()
{
  for (int nI = 0; nI < MAX_TRANSMITTER * (int)sizeof(KeyStore); nI++)
    EEPROM.write(nI, 0);
  EEPROM.commit();
}

///////////////////////////////////////////////////////////////////////////////
void PairDevice(uint8_t u8Transmitter, uint16_t u16DeviceID)
{
  KeyStore keyStore = ReadKeyStoreFromEEProm(u8Transmitter);
  keyStore.ulMagicHeader  = MAGIC_HEADER;
  keyStore.u16DeviceID    = u16DeviceID;
  WriteKeyStoreToEEProm(u8Transmitter, keyStore);
}

///////////////////////////////////////////////////////////////////////////////
String GetTransmitterName(uint8_t u8Transmitter, int nPage)
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
          if (nPage == -1) sResult = TRANSMITTER_5;
          if (nPage == 0) sResult  = TRANSMITTER_5_0;
          if (nPage == 1) sResult  = TRANSMITTER_5_1;
          if (nPage == 2) sResult  = TRANSMITTER_5_2;

        break;
      }
      case 6: sResult = TRANSMITTER_6; break;
      case 7: sResult = TRANSMITTER_7; break;
  }

  return sResult;
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

///////////////////////////////////////////////////////////////////////////////
bool CheckAllTransmittersForSendingData()
{
  bool bResult = false;

  SensorType types[] = {eTemperatur, eHumidity, ePressure, eRainCurrent, eRainTotal, ePowerCurrent, ePowerTotal, eWaterCurrent, eWaterTotal, eGasCurrent, eGasTotal};

  for (uint8_t u8Transmitter = 0; u8Transmitter < MAX_TRANSMITTER; u8Transmitter++)
  {
    int nFoundNewSensorData = -1;
    for (unsigned int i = 0; i < sizeof(types) / sizeof(SensorType); i++)
    {
      delay(50);
      SensorData* pSensorData = GetSensorData(u8Transmitter, types[i]);
      if (pSensorData && pSensorData->GetSize() > 0)
      {
        nFoundNewSensorData = 0;
        // Wenn 12h keine Daten eingetroffen sind eine eMail senden.
        if (GetTimeSpanSeconds(pSensorData->GetTickCount()) < 12*3600)
        {
            nFoundNewSensorData = 1;
            break;
        }
      }
    }

    if (nFoundNewSensorData == 0)
    {
      bResult = SendSMTP(u8Transmitter);
    }
    else
    {
      bResult = true;
    }
  }

return bResult;
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

///////////////////////////////////////////////////////////////////
void tft_ReceiveFlag(uint8_t u8Transmitter)
{
  // Den Indikator, das ein Transmitter empfangen wurde einblenden
  tft.setTextColor(TFT_BLUE, TFT_BLACK );
  tft.setTextSize(2);
  tft.setCursor(270, 10);

  tft.print("(");
  tft.print(u8Transmitter+1);
  tft.print(")");
}

///////////////////////////////////////////////////////////////////
void tft_ShowTitle(uint8_t u8Transmitter, SensorType sensorType)
{
  // Sendername ausgeben.
  String sTitle = F("Sender:");
  sTitle += String(u8Transmitter+1) + " ";
  sTitle += GetTransmitterName(u8Transmitter, (sensorType - ePowerCurrent)/2).c_str();
  sTitle += ("           ");
  tft_Print(10, 10, 2, true, sTitle.c_str(), TFT_YELLOW);
}

///////////////////////////////////////////////////////////////////
void tft_ShowFrames(bool bClearScreen)
{
   if (bClearScreen)
    tft.fillScreen(TFT_BLACK);

   tft.drawRoundRect(2, 0, tft.width()-2, 40, 10, TFT_BLUE);
   tft.drawRoundRect(2, 50, tft.width()-2,135, 10, TFT_GREEN);
   tft.drawRoundRect(2, 200, tft.width()-2, 40, 10, TFT_RED);

  if (bWifiConected)
    tft_ShowTimeDate();
  else
    tft_Print(30, 212, 2, true, F("Meine Wetterstation     "), TFT_YELLOW);
 }

 ///////////////////////////////////////////////////////////////////
 void tft_ShowTimeDate()
 {
   static char sCurTime[20] = {0};
   char sTime[20] = {0};
   sprintf(sTime, "%02d:%02d:%02d %02d.%02d.%02d", hour(), minute(), second(), day(), month(), year());
   if (strcmp(sTime, sCurTime) != 0)
   {
      tft_Print(50, 212, 2, true, sTime, TFT_YELLOW);
      strcpy(sCurTime, sTime);
   }
 }

///////////////////////////////////////////////////////////////////
void tft_ShowSensorValue(uint8_t u8Transmitter, SensorType sensorType, float fSensorValue)
{
    String sSensorName = FPSTR(g_sTFTSensorName[sensorType]);

    // Die Anzeige der Gesamtregenmenge ist pro Monat.
    if (sensorType == eRainTotal)
    {
        // Von der aktuellen Gesamtregenmenge die Gesamtregenmenge
        // des Vormonats abziehen.
        KeyStore keyStore = ReadKeyStoreFromEEProm(u8Transmitter);
        fSensorValue -= keyStore.fTotalRainMM;
        String sMonth = FPSTR(g_sMonth[keyStore.u8Month]);
        sSensorName += sMonth;
    }

    // Sensorname ausgeben.
    tft_Print(10, 55, 2, true, sSensorName.c_str(), TFT_WHITE);
    tft.print("              ");

    // LCD Look
    tft.setTextFont(FONT7);
    // tft_Print(220, 90, 1 , false, " 8888.8", 10<<11);
    tft_Print(220, 90, 1 , false, " 8888.8", RGB(0,0,5));
    tft_Print(220, 90, 1 , false, String(fSensorValue,1).c_str(), LED_COLOR, TRANSPARENT);
    tft.setTextFont(GFXFF);

    // Einheit rechtsbündig ausgeben...
    String s = FPSTR(g_sTFTUnits[sensorType]);
    tft_Print(226, 90, 3, true, s.c_str(), LED_COLOR);
 }

 ///////////////////////////////////////////////////////////////////////////////
 void tft_Disable7SegmentDisplay()
 {
   // Die simulierte 7segmentanzeige aus.
   tft.setTextFont(FONT7);
   // tft_Print(220, 90, 1 , false, " 8888.8", 10<<11); // In dunklem Rot
   tft_Print(220, 90, 1 , false, " 8888.8", RGB(0,0,5)); // In dunklem Rot
   tft.setTextFont(GFXFF);
 }

 ///////////////////////////////////////////////////////////////////
 void tft_Print(int16_t x, int16_t y, uint8_t s, bool bLeft, const char* pText, uint16_t colorText, uint16_t colorBack)
 {
   tft.setTextSize(s);
   if (colorBack == TRANSPARENT)
      tft.setTextColor(colorText);
   else
      tft.setTextColor(colorText, colorBack);

   if ((x >= 0) && (y >= 0))
   {
     if (bLeft == false)
     {
       // Rechtsbündig ausgeben...
       x = x - tft.textWidth(pText);
     }

     tft.setCursor(x, y);
   }

   tft.print(pText);
 }
 ///////////////////////////////////////////////////////////////////
 void tft_Print(int16_t x, int16_t y, uint8_t s, bool bLeft, const __FlashStringHelper* pText, uint16_t colorText, uint16_t colorBack)
 {
   tft.setTextSize(s);
   tft.setTextColor(colorText, colorBack );
   if ((x >= 0) && (y >= 0))
   {
     if (bLeft == false)
     {
       // Rechtsbündig ausgeben...
       x = x - tft.textWidth(FPSTR(pText));
     }
     tft.setCursor(x, y);
   }

   tft.print(pText);
 }
