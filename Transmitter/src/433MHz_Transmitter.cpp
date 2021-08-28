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

/*
Stromverbrauch:
Aktuelle Messwerte: CPU 16MHz: 10,75mA
                    CPU 8MHz:   6,32mA
                    CPU 1MHz:   2,28mA (Software läuft aber nicht)
Stromaufnahme mit PCF8583: ca. 7,5µA bzw. 39,2µA wenn Eingang geschlossen ist.

Sleep:  5.5µA (Ohne Spannungsregler, mit 3xAAA Batterien)

Zeiten:
DHT22 Read:        5,3ms
BMP085 Read:      34,0ms
DS1820 Read:      28,0ms
433MHz transmitt; 120.0ms
---> Aktivezeit: 5,3+34+3*120.0=400ms

Messzyklus: 300 Sekunden
Sleepzeit: 8 Sekunden
Passivzeit (Ohne Messung) 16µS alle 8 Sekuden (Vernachlässigbar)

Mittlerer Stromverbrauch:
[400ms * 10,75mA +(300000 - 400ms) * 0,0055mA] / 300000 = 0,0198mA = 19,8µA
Mittlerer Stromverbrach = ~20µA
P = 4,5V * 20µA = 89µW
======================

Mittlerer Stromverbrauch (Ohne Spannungsregler, mit 3xAAA Batterien, 8MHz):
[400ms * 6,32mA +(300000 - 400ms) * 0,0055mA] / 300000 = 0,0139mA = 13,9µA
Mittlerer Stromverbrach = ~14µA
P = 4.5V * 14µA = 62,6µW (~30% Ersparnis gegenüber der 16MHz Version)
========================

// AAA Alkaline Batterie ca. 900mAh (Selbstendladung ca. 35µA)
Geschätzte Laufzeit 16MHz: 900mAh / (20+35)µA = 16363 Stunden = 682 Tage = 1,8 Jahre 
Geschätzte Laufzeit 8MHz:  900mAh / (14+35)µA = 18367 Stunden = 765 Tage = 2,1 Jahre

Mit SDRUno ermittelte Sendefrequenzen:
	1 Wohnzimmer: 433,885 MHz
	2 Garten:     433,900 MHz
	3 Büro:	      433,850 MHz
	4 Terasse:    433,525 MHz
	5 Keller:     433,810 MHz
	6 Strom/Gas/W:433,865 MHz
	7 Windmesser: 433,800 MHz
  8 Freezer:
*/

#include "433MHz_Transmitter.h"

volatile float fCurrentPowerW = 0.0;
volatile float fTotalPowerWh  = 0.0;
volatile float fWaterFlowLM = 0.0;
volatile float fTotalWaterL = 0.0;
volatile float fGasFlowLM = 0.0;
volatile float fTotalGasL = 0.0;
volatile bool bPowerTrigger = false;
volatile bool bWaterTrigger = false;
volatile bool bGasTrigger = false;

///////////////////////////////////////////////////////////////////////////////
void Timer2ISR()
{
    int nPowerMeterSensorDiff = 0, nWaterMeterSensorDiff = 0, nGasMeterSensorDiff = 0;

    // Die drei optischen Sensoren durch eine Differenzmessung erfassen.
    ReadPowerWaterGasSensors(nPowerMeterSensorDiff, nWaterMeterSensorDiff, nGasMeterSensorDiff);

    // Triggerpunkte erkannt?
    if (bDetectPowerMeter)
    {
        if (DetectPowerMeterTrigger(nPowerMeterSensorDiff))
          bPowerTrigger = true;
    }
    if (bDetectWaterMeter)
    {
       if (DetectWaterMeterTrigger(nWaterMeterSensorDiff))
        bWaterTrigger = true;
    }
    if (bDetectGasMeter)
    {
        if (DetectGasMeterTrigger(nGasMeterSensorDiff))
          bGasTrigger = true;
    }
}

///////////////////////////////////////////////////////////////////////////////
void pin2_isr()
{
  // Interrupt deaktivieren
  detachInterrupt(digitalPinToInterrupt(switchPin));

  // Warte 25ms, bis das Signal stabil ist. (Entprellen)
  delayMicroseconds(25000);

  // Ist der Pin noch auf LOW?
  if (digitalRead(switchPin) == LOW)
  {
    // Dann zähle diesen Impuls.
    deviceSettings.bDetectRainSensor = true;
    bRainCounterChanged = true;
  }

  // Und Interrupt wieder einschalten
  attachInterrupt(digitalPinToInterrupt(switchPin), pin2_isr, FALLING);
}

///////////////////////////////////////////////////////////////////////////////
void setup()
{
  #ifdef _DEBUG
    //Serial.begin(38400);
    Serial.begin(115200);
  #endif

#if (0) // Testcode
  pinMode(5, INPUT);
  digitalWrite(5, HIGH); // Interner Pullup einschalten
  timer1_setup();
#endif

  pinMode(ledPin, OUTPUT);

 
  TRACE(F("Starting...\n"));

  // 433MHZ Lib initialisieren.
  ASK_driver.init();

  // Wenn nötig das EEProm löschen
  // ClearEEProm();

  // Seriennummer und Zählstände aus dem EEProm lesen
  deviceSettings = ReadDeviceSettingsFromEEProm();

#ifdef _DEBUG
  #if (0) 
    // Fake the device ID
    // deviceSettings.u16DeviceID = 28107; // Transmitter 1 (Wohnzimmer)
    // deviceSettings.u16DeviceID = 58790; // Transmitter 2 (Garten)
    // deviceSettings.u16DeviceID = 63589; // Transmitter 3 (Büro)
    // deviceSettings.u16DeviceID = 8140; // Transmitter 4 (Terasse)
    // deviceSettings.u16DeviceID = 34584; // Transmitter 5 (Keller)
    // deviceSettings.u16DeviceID = 16807; // Transmitter 7 (Wind)
    // deviceSettings.u16DeviceID = 42464; // Transmitter 8 (Freezer)
    WriteDeviceSettingsToEEProm(deviceSettings);
  #endif
 #endif
  // Wenn keine Seriennummer gepeichert wurde.
  // wird eine neue Seriennummer generiert und an den Receiver gesendet. Dieser muss ebenfalls
  // neu gestartet worden sein, damit er den Pairing Request akzeptiert.
  if (deviceSettings.ulMagicHeader != MAGIC_HEADER)
  {
    // Magic Header setzen
    deviceSettings.ulMagicHeader = MAGIC_HEADER;

    // Eine Zufällige Seriennummer generieren, im Keystore sichern und an den Receiver senden.
    unsigned long ulRandomSeed = analogRead(A2) * analogRead(A3);
    randomSeed(ulRandomSeed);
    deviceSettings.u16DeviceID = (uint16_t)random();
    deviceSettings.fTotalPowerWh = 0.0;
    deviceSettings.fTotalWaterL = 0.0;
    deviceSettings.fTotalGasL = 0.0;
    deviceSettings.fTotalRainMM = 0.0;
    deviceSettings.bDetectRainSensor = false;
    WriteDeviceSettingsToEEProm(deviceSettings);
    #ifdef _DEBUG   
       TRACE(F("RandomSeed:"));
       TRACE(ulRandomSeed);
       TRACE("\n");

       TRACE(F("Generating new DeviceID:"));
       TRACE(deviceSettings.u16DeviceID);
       TRACE("\n");
    #endif
  }

#ifdef SET_INITIAL_START_VALUES
    // Einmalig die aktuellen Zählerstände ins EEProm sichern
    deviceSettings.fTotalRainMM = INITIAL_TOTAL_RAIN_MM;
    deviceSettings.fTotalPowerWh = INITIAL_TOTAL_POWER_WH;
    deviceSettings.fTotalWaterL = INITIAL_TOTAL_WATER_L;
    deviceSettings.fTotalGasL = INITIAL_TOTAL_GAS_L;

    if (deviceSettings.fTotalRainMM > 0.0)
      deviceSettings.bDetectRainSensor = true;
    else
      deviceSettings.bDetectRainSensor = false;

    WriteDeviceSettingsToEEProm(deviceSettings);
    TRACE(F("Initial writing counters to EEProm!!\n"));
#endif

  // Die aus dem EEProm gelesenen Zählerstartwerte übernehmen.
  fTotalPowerWh = deviceSettings.fTotalPowerWh;
  fTotalWaterL = deviceSettings.fTotalWaterL;
  fTotalGasL = deviceSettings.fTotalGasL;

  TRACE(F("Sending pairing msg\n"));

  // Pairing nachricht senden
  SendMessage(eMessage::MsgPairing, 0, 0);

 
  // Stromversorgung des DHTxx Sensors über GPIO um Strom zu sparen
  pinMode(powerDHTxxPin, OUTPUT);
  digitalWrite(powerDHTxxPin, HIGH);

  // Stromversorgung des BMP085 Sensors über GPIO um Strom zu sparen
  pinMode(powerDS1820Pin, OUTPUT);
  digitalWrite(powerDS1820Pin, HIGH);

  // Stromversorgung des DS1820 Sensors über GPIO um Strom zu sparen
  pinMode(powerBMP085Pin, OUTPUT);
  digitalWrite(powerBMP085Pin, HIGH);

  // Eingang für die Regensensorwippe
  pinMode(switchPin, INPUT);
  digitalWrite(switchPin, HIGH); // Interner Pullup einschalten

  // GPIOs der IR-Dioden auf Ausgang.
  pinMode(ledIRPinPowerMeter, OUTPUT);
  pinMode(ledIRPinWaterMeter, OUTPUT);
  pinMode(ledIRPinGasMeter, OUTPUT);

  // GPIOs der Fototransistoren auf Eingang
  pinMode(analogInPinPowerMeter, INPUT);
  pinMode(analogInPinWaterMeter, INPUT);
  pinMode(analogInPinGasMeter, INPUT);

 
  // Interne Pull Ups der Fototransistoren aktivieren
  digitalWrite(analogInPinPowerMeter, INPUT_PULLUP);
  digitalWrite(analogInPinWaterMeter, INPUT_PULLUP);
  digitalWrite(analogInPinGasMeter, INPUT_PULLUP);

 
  // BMP085 initialisieren
  bDetectBMP085 = BMPSensor.begin();

  if (bDetectBMP085){
    TRACE(F("BMP085 detected\n"));}
  else
    TRACE(F("No BMP085 detected\n"));


  // Funktioniert der Zugriff auf den Luftfeuchtigkeitssensor?
  // DHT initialisieren
  dht.begin();

  int nC = 3;
  do
  {
    if (dht.read())
      bDetectDHTxx = true;
    else
      delay(2000);
  }
  while((nC-- > 0) && !bDetectDHTxx);

  if (bDetectDHTxx){
    TRACE(F("DHTxx detected\n"));}
  else
    TRACE(F("No DHTxx detected\n"));

  // Test, ob ein DS1820 angeschlossen ist.
  pinMode(ds1820Pin, INPUT);
  digitalWrite(ds1820Pin, HIGH); // Interner Pullup einschalten
  ds1820.begin();
  if (ds1820.getDeviceCount() > 0)
  {
    if (ds1820.getAddress(DS1820DeviceAdr, 0))
      bDetectDS1820 = true;
    else
      TRACE(F("Unable to find address for Device 0\n"));
  }

  if (bDetectDS1820){
    TRACE(F("DS1820 detected\n"));}
  else
    TRACE(F("No DS1820 detected\n"));

  
  // Test ob PCF8583 vorhanden
  pcf8583.setMode(MODE_CLOCK_32KHZ);
  pcf8583.setDate(11, 7, 2018);
  if ((pcf8583.getDay() == 11) && (pcf8583.getMonth() == 7) && (pcf8583.getYear() == 2018))
  {
      bDetectPCF8583 = true;

      // configure PCF8586 to event counter mode and reset counts
      pcf8583.setMode(MODE_EVENT_COUNTER);
      pcf8583.setCount(0);
  }

  if (bDetectPCF8583){
    TRACE(F("PCF8583 detected\n"));}
  else
    TRACE(F("No PCF8583 detected\n"));

  // Test, ob an den analogen Eingängen für den Strom-, Wasser-, Gaszähler ein Sensor hängt
  // Bei nicht beschalteten Analogeingängen, sollte der intern aktivierte Pullup Widerstand
  // den Eingang auf High ziehen.
  int nPowerMeterSensorDiff = 0, nWaterMeterSensorDiff = 0, nGasMeterSensorDiff = 0;
  ReadPowerWaterGasSensors(nPowerMeterSensorDiff, nWaterMeterSensorDiff, nGasMeterSensorDiff);

/*
  TRACE(F("PowerMeterValue="));
  TRACE(nPowerMeterSensorDiff);
  TRACE(F("\nWaterMeterValue="));
  TRACE(nWaterMeterSensorDiff);
  TRACE(F("\nGasMeterValue="));
  TRACE(nGasMeterSensorDiff);
  TRACE("\n");
  delay(100);
*/

  // Wenn die Eingänge auf High stehen, ist wahrscheinlich nichts angeschlossen
  bDetectPowerMeter = (nPowerMeterSensorDiff > 50);
  bDetectWaterMeter = (nWaterMeterSensorDiff > 50);
  bDetectGasMeter   = (nGasMeterSensorDiff > 50);
/*
  // Die gemessenen Werte ignorieren und selber bestimmen, welcher Sensor aktiv ist.
  bDetectPowerMeter = true;
  bDetectWaterMeter = true;
  bDetectGasMeter   = true;
*/
  
  if (bDetectPowerMeter){
    TRACE(F("PowerMeter detected\n"));}
  else
    TRACE(F("No PowerMeter detected\n"));

  if (bDetectWaterMeter){
    TRACE(F("WaterMeter detected\n"));}
  else
    TRACE(F("No WaterMeter detected\n"));

  if (bDetectGasMeter){
    TRACE(F("GasMeter detected\n"));}
  else
    TRACE(F("No GasMeter detected\n"));

  delay(100);
  
  #ifdef _CALIBRATE
    while(true)
    {
      int nPowerMeterSensorDiff = 0, nWaterMeterSensorDiff = 0, nGasMeterSensorDiff = 0;
      ReadPowerWaterGasSensors(nPowerMeterSensorDiff, nWaterMeterSensorDiff, nGasMeterSensorDiff);
      delay(200);
    };
  #endif

#ifdef USE_ISR_READING
  // Wenn mindestens ein optischer Sensor erkannt wurde, wird dieser mit
  // einem Timerinterrupt periodisch abgefragt.
  if (bDetectPowerMeter || bDetectWaterMeter || bDetectGasMeter)
  {
    MsTimer2::set(250, Timer2ISR);
    MsTimer2::start();
  }
#endif

  // Raindetector weckt den Arduino aus dem Powerdown modus.
  attachInterrupt(digitalPinToInterrupt(switchPin), pin2_isr, FALLING);
}

///////////////////////////////////////////////////////////////////////////////
void loop()
{
  // Unterscheidung zwischen Wetterstation und Energiemessung.
  if (bDetectPowerMeter || bDetectWaterMeter || bDetectGasMeter)
    loop_power_water_gas_meter();
  else
    loop_weather_station();
}

///////////////////////////////////////////////////////////////////////////////
void loop_weather_station()
{
  static float fAvgTemperatur = -1000.0; // INVALID
  static float fAvgHumidiy = -1000.0; // INVALID

  static uint32_t u32LastMessureTemperatureTC = 0;
  static uint32_t u32LastMessureHumidityTC = 0;
  static uint32_t u32LastMessurePressureTC = 0;
  static uint32_t u32LastMessureRainTC = 0;
  static uint32_t u32LastMessureWindSpeedTC = 0;
  static long lRandomTime = 0;

  // Und ab in den Tiefschlaf...
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  u32Tickcount += 8L;

#if (0) // Testcode
#ifdef _DEBUG
  int iCounter = TCNT1;
  TRACE("Counter=");
  TRACE(iCounter);
  TRACE("\n");
  delay(1000);
#endif
#endif
  // In den ersten 3 Minuten wird alle 8 Sekunden eine Messung durchgefürt und gesendet.
  // danach, um Strom zu sparen, nur all 5 Minuten +/- 45 Sekunden.
  uint32_t u32TimePeriod = 0;
  if (u32Tickcount < 3L*60L)
    u32TimePeriod = 8;
  else
  {
    // Die Variation der Sendezeiten verhindert, das mehrere Sender zufällig über einen
    // längeren Zeitraum synchron sind und immer gleichzeitig senden, was zu Störungen
    // führt.
    if (lRandomTime == 0)
       lRandomTime = random(-45, 45);
    u32TimePeriod = 5L*60L + lRandomTime;
  }

  int nTemperature = INVALID;
  int nHumidity    = INVALID;
  int nPressure    = INVALID;

  /////////////////////////////////////////////////////////////////
  // Jeder Messwert wird mit 10Bit übertragen.
  // t = -30.0 ... +70.0 °C -> T = (t + 30) * 10  -> 0 <= T <= 1000
  // h = 0.0 ... 100.0 %    -> H = h * 10         -> 0 <= H <= 1000
  // p = 963 ... 1063 hPa   -> P = (p - 963) * 10 -> 0 <= P <= 1000
  // Ein Wert von 0x3FF (1023) für T, H, oder P bedeutet 'ungültig'
  //
  // 3322 2222 2222 1111 1111 1100 0000 0000
  // 1098 7654 3210 9876 5432 1098 7654 3210
  // xxTT TTTT TTTT HHHH HHHH HHPP PPPP PPPP
  /////////////////////////////////////////////////////////////////

  // Luftfeuchtigkeitssensor vorhanden?
  if (bDetectDHTxx)
  {
      // Ist es wieder Zeit die Temperatur, oder Luftfeuchte zu messen?
      if ((u32Tickcount >= u32LastMessureTemperatureTC + u32TimePeriod) ||
          (u32Tickcount >= u32LastMessureHumidityTC + u32TimePeriod))
      {
        // Ja, dann die Stromversorgung des DHTxx Sensors an, eine Sekunde
        // warten, bis der Sensor klar ist und Messung dann durchführen.
#ifdef TOGGLE_POWER_DHTxxPIN       
        pinMode(DHTxxPin, INPUT_PULLUP);   // Datenpin wieder auf Eingang
        digitalWrite(powerDHTxxPin, HIGH); // und Pullup einschalten.
        LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
        u32Tickcount += 1;
        dht.read(true);

        // Stromversorgung wieder ausschalten
        digitalWrite(powerDHTxxPin, LOW);

        // Die Datenleitung muss ebenfalls auf 0, damit kein Strom über
        // den Pullupwiederstand fließt.
        pinMode(DHTxxPin, OUTPUT);
        digitalWrite(DHTxxPin, LOW);
#endif    
        // Temperatur der oben durchgeführten Messung auslesen.
        float t = dht.readTemperature(false, false);
        if (isnan(t) == 0)
        {
          // Beim ersten Messwert kein Mittelwert bilden
          // Der DHT11/DHT22 liefert nicht immer reproduzierbare Werte, daher
          // wird hier ein Mittelwert gebildet.
          fAvgTemperatur = (fAvgTemperatur == -1000.0) ? t : (fAvgTemperatur + t) / 2.0;

          // fAvgTemperatur = -30.0 ... +70.0 °C
          fAvgTemperatur = max(fAvgTemperatur, -30.0);
          fAvgTemperatur = min(fAvgTemperatur, 70.0);
#ifdef _DEBUG
          TRACE(F("Temperatur="));
          TRACE(String(fAvgTemperatur, 2));
          TRACE("\n");
#endif
          if (u32Tickcount >= u32LastMessureTemperatureTC + u32TimePeriod)
          {
            nTemperature = (int)((fAvgTemperatur + 30.0) * 10.0);
            u32LastMessureTemperatureTC = u32Tickcount;
          }
        }
        else
          TRACE(F("Error reading temperature\n"));

        // Luftfeuchtigkeit der oben durchgeführten Messung auslesen.
        float h = dht.readHumidity(false);
        if (isnan(h) == 0)
        {
          // Beim ersten Messwert kein Mittelwert bilden
          // Der DHT11/DHT22 liefert nicht immer reproduzierbare Werte, daher
          // wird hier ein Mittelwert gebildet.
          fAvgHumidiy = (fAvgHumidiy == -1000.0) ? h : (fAvgHumidiy + h) / 2.0;
      
          // fAvgHumidiy = 0.0 ... 100.0 %
          fAvgHumidiy = max(fAvgHumidiy, 0.0);
          fAvgHumidiy = min(h, 100.0);
#ifdef _DEBUG
          TRACE(F("Humidity="));
          TRACE(String(fAvgHumidiy, 2));
          TRACE("\n");
#endif
          if (u32Tickcount >= u32LastMessureHumidityTC + u32TimePeriod)
          {
            nHumidity = (int)(fAvgHumidiy * 10.0);
            u32LastMessureHumidityTC = u32Tickcount;
          }
        }
        else
          TRACE(F("Error reading humidity\n"));
      }
  }

  // Luftdrucksensor vorhanden?
  if (bDetectBMP085)
  {
    // Ist es wieder Zeit den Luftdruck zu messen?
    // Das Oversampling erfolgt im Sensor, daher ist hier keine
    // Glättung der Messwerte durch Mittelwertbildung notwendig.
    if (u32Tickcount >= u32LastMessurePressureTC + u32TimePeriod)
    {
      sensors_event_t event;

      // Stromversorgung des BMP085 einschalten
      digitalWrite(powerBMP085Pin, HIGH);

      // Startup time nach power up 10ms (Seite15 Datenblatt)
      LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
      
      BMPSensor.getEvent(&event);
#ifdef TOGGLE_POWER_BMP085PIN      
      // Stromversorgung des BPM085 wieder ausschalten.
      // Die I2C Datenleitungen müssen ebenfalls auf 0, damit kein Strom über
      // den Pullupwiederstand fließt.
      digitalWrite(powerBMP085Pin, LOW);
      digitalWrite(A4, LOW); 
      digitalWrite(A5, LOW);
#endif
      // Messung wird gestartet, falls Sensor bereit
      if (event.pressure)
      {
        // pressure = 963 ... 1063 hPa
        float pressure = max(event.pressure, 963.0);
        pressure = min(pressure, 1063.0);
        nPressure = (int)((pressure - 963.0) * 10.0);

#ifdef _DEBUG
      TRACE(F("Pressure="));
      TRACE(String(pressure, 2));
      TRACE("\n");
#endif
        u32LastMessurePressureTC = u32Tickcount;
      }
      else
        TRACE(F("Error reading pressure\n"));
    }
  }

  // Onewire Temperatursensor vorhanden?
  if (bDetectDS1820)
  {
    // Ist es wieder Zeit die Temperatur zu messen?
    if (u32Tickcount >= u32LastMessureTemperatureTC + u32TimePeriod)
    {
      // Temperatur messen.
      // Auch hier wird kein Mittelwert durch Oversampling durchgeführt,
      // da der Sensor bis zu 500ms braucht, um die Tmeperatur zu liefern.
      // Würde dies alle 8 Sekunden gemacht, so stiege der Stromverbrauch zu
      // stark an. Was einem Batteriebetrieb entgegen steht.
      
      // Datenleitung wieder auf Eingang und Powerpin des DS1820 an.
      pinMode(ds1820Pin, INPUT_PULLUP);
      digitalWrite(powerDS1820Pin, HIGH);

      // Temperatur einlesen
      ds1820.requestTemperatures();
      float t = ds1820.getTempC(DS1820DeviceAdr);

#ifdef TOGGLE_POWER_DS1820PIN     
      // Stromversorgung des DS1820 ausschalten
      digitalWrite(powerDS1820Pin, LOW);

      // Den Datenpin ebenfalls auf 0 legen, damit kein Strom über
      // den Pulupwiederstand fließt.
      pinMode(ds1820Pin, OUTPUT);
      digitalWrite(ds1820Pin, LOW);
#endif

#ifdef _DEBUG
      TRACE(F("Temperatur="));
      TRACE(String(t, 2));
      TRACE("\n");
#endif
      // t = -30.0 ... +70.0 °C
      t = max(t, -30.0);
      t = min(t, 70.0);
      nTemperature = (int)((t + 30.0) * 10.0);

      u32LastMessureTemperatureTC = u32Tickcount;
    }
  }

  // Mindestens ein gültiger Messwert vorhanden?
  if ((nTemperature != INVALID) || (nHumidity != INVALID) || (nPressure != INVALID))
  {
    // 32 Bit Payload mit den Wetterdaten zusammenbauen.
    uint32_t u32WeatherData = ((uint32_t)nTemperature)<<20 |
                              ((uint32_t)nHumidity)<<10 |
                               (uint32_t)nPressure;
    SendRepeatedMessage(eMessage::MsgWeatherCompact, u32WeatherData, u32Tickcount);
  }

  // Wurden schon Zählimpulse vom Regensensor registriert?
  if (deviceSettings.bDetectRainSensor)
  {
    // Regen detektiert, oder mal wieder Zeit den Regenzählstand zu senden?
    if ((bRainCounterChanged) || (u32Tickcount >= u32LastMessureRainTC + u32TimePeriod))
    {
      // Wenn sich der Zählstand geändert hat, den neuen Wert im EEProm sichern
      if (bRainCounterChanged)
      {
        // In den ersten 15 Minuten nach einem Reset, soll der Zählstand des Regensensors
        // nicht erhöht werden, da dieser evtl. noch montiert wird.
        if (u32Tickcount > 15L*60L)
        {
          deviceSettings.fTotalRainMM += MM_PER_TICK;
          WriteDeviceSettingsToEEProm(deviceSettings);
        }
        bRainCounterChanged = false;
      }

#ifdef _DEBUG
/*
      TRACE(String(deviceSettings.fTotalRainMM, 3));
      TRACE("\t-> ");
      uint32_t u32Rain = FloatToUint32(deviceSettings.fTotalRainMM);
      TRACE(u32Rain)
      TRACE("\t-> ");
      float fRain = Uint32ToFloat(u32Rain);
      TRACE(String(fRain, 3));
      TRACE("\n");
*/
#endif

      SendRepeatedMessage(eMessage::MsgTotalRainMM, FloatToUint32(deviceSettings.fTotalRainMM), u32Tickcount);
      u32LastMessureRainTC = u32Tickcount;
    }
  }

  // Wurden schon Zählimpulse vom Windmesser registriert?
  if (bDetectPCF8583)
  {
    // Ist es Zeit den Windgeschwindigkeitszählerstand zu senden?
    if (u32Tickcount >= u32LastMessureWindSpeedTC + u32TimePeriod)
    {
      uint32_t u32WindSpeedCounter = pcf8583.getCount();

      TRACE("Wind counter=");
      TRACE(u32WindSpeedCounter);
      TRACE("\n");

      SendRepeatedMessage(eMessage::MsgWindSpeedCounter, FloatToUint32(u32WindSpeedCounter), u32Tickcount);
      u32LastMessureWindSpeedTC = u32Tickcount;
    }
  }
}

//////////////////////////////////////////////////////////////////////
void loop_power_water_gas_meter()
{
  static uint32_t u32LastSendPowerMeterDataTC = millis();
  static uint32_t u32LastSendWaterMeterDataTC = millis();
  static uint32_t u32LastSendGasMeterDataTC = millis();
  static long lRandomTime = 0;
  if (lRandomTime == 0)
    lRandomTime = 1000L * random(0, 10);

  // Wiederholungsintervall bei Inaktivität.
  // Der Zufallsfaktor soll verhindern, das alle Transmitter synchron laufen können.
  const uint32_t u32SendPowerPassiveIntervall = 5L*60L*1000L + lRandomTime;
  const uint32_t u32SendWaterPassiveIntervall = 5L*60L*1000L + lRandomTime;
  const uint32_t u32SendGasPassiveIntervall   = 5L*60L*1000L + lRandomTime;

  // Wiederholungsintervall bei Aktivität.
  const uint32_t u32SendPowerActiveIntervall = 5L*60L*1000L; // Strom fließt immer
  const uint32_t u32SendWaterActiveIntervall = 1L*60L*1000L;
  const uint32_t u32SendGasActiveIntervall   = 1L*60L*1000L;

  // Die Zeitintervalle nach denen eine Message erneut gesendet wird.
  static uint32_t u32SendPowerIntervall = u32SendPowerPassiveIntervall;
  static uint32_t u32SendWaterIntervall = u32SendWaterPassiveIntervall;
  static uint32_t u32SendGasIntervall   = u32SendGasPassiveIntervall;

#ifndef USE_ISR_READING
  Timer2ISR();
#endif

  // Ist überhaupt ein IR-LED Sensor zur Abfrage des Stromzählers vorhanden?
  if (bDetectPowerMeter)
  {
      // Wurde ein Trigger registreiert, oder sind 5 Minuten um, dann daten senden.
      if (bPowerTrigger || ((uint32_t)millis() > u32LastSendPowerMeterDataTC + u32SendPowerIntervall))
      {
        float fCurrentValue = 0.0;
        u32LastSendPowerMeterDataTC = (uint32_t)millis();

        if (bPowerTrigger)
        {
          bPowerTrigger = false;
          u32SendPowerIntervall = u32SendPowerActiveIntervall;

          // Wenn ein Trigger erkannt wurde, den gemessenen Wert lokal sichern.
          fCurrentValue = fCurrentPowerW;

          // Alle 1kwh den aktuellen Wert im EEProm sichern.
          if ((fTotalPowerWh - deviceSettings.fTotalPowerWh) >= 1000.0)
          {
              deviceSettings.fTotalPowerWh = fTotalPowerWh;
              WriteDeviceSettingsToEEProm(deviceSettings);
          }
        }
        else
          u32SendPowerIntervall = u32SendPowerPassiveIntervall;

        uint32_t u32Dummy = 0;

        #ifdef _DEBUG
               TRACE("total Power=");
               TRACE(String(fTotalPowerWh / 1000.0, 3));
               TRACE(" kWh");
               TRACE("\t Current Power=");
               TRACE(String(fCurrentValue, 1));
               TRACE(" W\n");
        #endif

        // ... und senden
        SendRepeatedMessage(eMessage::MsgPowerMeterCurrent, FloatToUint32(fCurrentValue), u32Dummy);

        // Und zusätzlich den akkumulierten Stromverbrauch übertragen.
        SendRepeatedMessage(eMessage::MsgPowerMeterTotal, FloatToUint32(fTotalPowerWh), u32Dummy);
      }
  }

  // Ist überhaupt ein IR-LED Sensor zur Abfrage des Wasserzählers vorhanden?
  if (bDetectWaterMeter)
  {
      // Wurde ein Trigger registriert, oder sind 5 Minuten um, dann daten senden.
      if (bWaterTrigger || ((uint32_t)millis() > u32LastSendWaterMeterDataTC + u32SendWaterIntervall))
      {
        float fCurrentValue = 0.0;
        u32LastSendWaterMeterDataTC = millis();

        if (bWaterTrigger)
        {
          bWaterTrigger = false;
          u32SendWaterIntervall = u32SendWaterActiveIntervall;

          // Wenn ein Trigger erkannt wurde, den gemessenen Wert lokal sichern.
          fCurrentValue = fWaterFlowLM;

          // Alle 50 Liter den aktuellen Wert im EEProm sichern.
          if ((fTotalWaterL - deviceSettings.fTotalWaterL) >= 50.0)
          {
            deviceSettings.fTotalWaterL = fTotalWaterL;
            WriteDeviceSettingsToEEProm(deviceSettings);
          }
        }
        else
          u32SendWaterIntervall = u32SendWaterPassiveIntervall;

        uint32_t u32Dummy = 0;

        #ifdef _DEBUG
               TRACE("total Water=");
               TRACE(String(fTotalWaterL / 1000.0, 3));
               TRACE(" m³");
               TRACE("\t Water flow=");
               TRACE(String(fCurrentValue, 1));
               TRACE(" l/m\n");
        #endif

        // ... und senden
        SendRepeatedMessage(eMessage::MsgCurrentWaterFlow, FloatToUint32(fCurrentValue), u32Dummy);

        // Und zusätzlich die akkumulierte Wassermenge übertragen.
        SendRepeatedMessage(eMessage::MsgTotalWater, FloatToUint32(fTotalWaterL), u32Dummy);
      }
  }

  // Ist überhaupt ein IR-LED Sensor zur Abfrage des Gaszählers vorhanden?
  if (bDetectGasMeter)
  {
      // Wurde ein Trigger registreiert, oder sind 5 Minuten um, dann daten senden.
      if (bGasTrigger || ((uint32_t)millis() > u32LastSendGasMeterDataTC + u32SendGasIntervall))
      {
        float fCurrentValue = 0.0;
        u32LastSendGasMeterDataTC = millis();

        if (bGasTrigger)
        {
          bGasTrigger = false;
          u32SendGasIntervall   = u32SendGasActiveIntervall;

          // Wenn ein Trigger erkannt wurde, den gemessenen Wert lokal sichern.
          fCurrentValue = fGasFlowLM;

          // Alle 50 Liter den aktuellen Wert im EEProm sichern.
          if ((fTotalGasL - deviceSettings.fTotalGasL) >= 50.0)
          {
              deviceSettings.fTotalGasL = fTotalGasL;
              WriteDeviceSettingsToEEProm(deviceSettings);
          }
        }
        else
          u32SendGasIntervall   = u32SendGasPassiveIntervall;

        uint32_t u32Dummy = 0;

        #ifdef _DEBUG
                 TRACE("total Gas=");
                 TRACE(String(fTotalGasL / 1000.0, 3));
                 TRACE(" m³");
                 TRACE("\t Gas flow=");
                 TRACE(String(fCurrentValue, 1));
                 TRACE(" l/m\n");
        #endif

        // ... und senden
        SendRepeatedMessage(eMessage::MsgCurrentGasFlow, FloatToUint32(fCurrentValue), u32Dummy);

        // Und zusätzlich die akkumulierte Gasmenge übertragen.
        SendRepeatedMessage(eMessage::MsgTotalGas, FloatToUint32(fTotalGasL), u32Dummy);
      }
   }
}

//////////////////////////////////////////////////////////////////////
void ReadPowerWaterGasSensors(int& nPowerMeterSensorDiff, int& nWaterMeterSensorDiff, int& nGasMeterSensorDiff)
{
  // Die optisch erfassten Sensoren (Strom-, Wasser- und Gaszähler) benötigen
  // jeweils zwei Messungen. Bei aktivierter IR-LED und bei deaktivierter IR-LED.

  // Messung bei aktivierter IR-LED:
  digitalWrite(ledIRPinPowerMeter, HIGH);
  digitalWrite(ledIRPinWaterMeter, HIGH);
  digitalWrite(ledIRPinGasMeter, HIGH);
  delayMicroseconds(1000); // Die LEDs brauchen etwas Zeit, bis sie auf voller helligkeit sind

  // AVR empfielt nach dem Wechsel des Analogeinganges etwas zu warten.
  // Dies kann einfach durch zweifaches Lesen des Einganges erfolgen.
  analogRead(analogInPinPowerMeter);
  uint16_t u16PowerMeterSensorValueHigh = analogRead(analogInPinPowerMeter);

  analogRead(analogInPinWaterMeter);
  uint16_t u16WaterMeterSensorValueHigh = analogRead(analogInPinWaterMeter);

  analogRead(analogInPinGasMeter);
  uint16_t u16GasMeterSensorValueHigh = analogRead(analogInPinGasMeter);

  // Messung bei deaktivierter IR-LED:
  digitalWrite(ledIRPinPowerMeter, LOW);
  digitalWrite(ledIRPinWaterMeter, LOW);
  digitalWrite(ledIRPinGasMeter, LOW);
  delayMicroseconds(1000); // Die LEDs leuchten anscheinend etwas nach

  uint16_t u16PowerMeterSensorValueLow = POWER_METER_SENSOR_VALUE_LOW;
  uint16_t u16WaterMeterSensorValueLow = WATER_METER_SENSOR_VALUE_LOW;
  uint16_t u16GasMeterSensorValueLow = GAS_METER_SENSOR_VALUE_LOW;

// Ein Hallsensor benötigt keine Differenzmessung...

#ifdef USE_IR_SENSOR_FOR_POWER_MEASUREMENT
   analogRead(analogInPinPowerMeter);
   u16PowerMeterSensorValueLow = analogRead(analogInPinPowerMeter);
#endif

#ifdef USE_IR_SENSOR_FOR_WATER_MEASUREMENT
  analogRead(analogInPinWaterMeter);
  u16WaterMeterSensorValueLow = analogRead(analogInPinWaterMeter);
#endif

#ifdef USE_IR_SENSOR_FOR_GAS_MEASUREMENT
  analogRead(analogInPinGasMeter);
  u16GasMeterSensorValueLow = analogRead(analogInPinGasMeter);
#endif

 // Die Differenz der beiden Messungen bilden.
  nPowerMeterSensorDiff = abs(u16PowerMeterSensorValueHigh - u16PowerMeterSensorValueLow);
  nWaterMeterSensorDiff = abs(u16WaterMeterSensorValueHigh - u16WaterMeterSensorValueLow);
  nGasMeterSensorDiff = abs(u16GasMeterSensorValueHigh - u16GasMeterSensorValueLow);

#ifdef _CALIBRATE
  if (bDetectPowerMeter)
  {
    TRACE("PH=")
    TRACE(u16PowerMeterSensorValueHigh);
    TRACE("\tPL=");
    TRACE(u16PowerMeterSensorValueLow);
    TRACE("\tPD=");
    TRACE(abs(nPowerMeterSensorDiff));
    TRACE("\t");
  }

  if (bDetectWaterMeter)
  {
    TRACE("WH=");
    TRACE(u16WaterMeterSensorValueHigh);
    TRACE("\tWL=");
    TRACE(u16WaterMeterSensorValueLow);
    TRACE("\tWD=");
    TRACE(abs(nWaterMeterSensorDiff));
    TRACE("\t");
  }
  if (bDetectGasMeter)
  {
    TRACE("GH=");
    TRACE(u16GasMeterSensorValueHigh);
    TRACE("\tGL=");
    TRACE(u16GasMeterSensorValueLow);
    TRACE("\tGD=");
    TRACE(abs(nGasMeterSensorDiff));
  }
  TRACE("\n");
#endif
}

///////////////////////////////////////////////////////////////////////////////
bool DetectPowerMeterTrigger(int nSensorDiff)
{
	static bool bTriggerState = false;
	static uint32_t u32TriggerTC = 0;

	bool bNextState = bTriggerState;
	bool bDetectTrigger = false;

	if (nSensorDiff > nPowerTriggerLevelHigh)
		bNextState = false;
	else if (nSensorDiff < nPowerTriggerLevelLow)
		bNextState = true;

	if (bNextState != bTriggerState)
	{
		bTriggerState = bNextState;
	  digitalWrite(ledPin, bTriggerState);

    if (bNextState)
		{
			uint32_t u32TimeDiff = millis() - u32TriggerTC;

			// Zu kurze Impulse müssen Störungen sein. 2000ms entsprechen etwa 24KW Leistung
			if (u32TimeDiff > 2000)
			{
				// Gesamtstromverbrauch aufsummieren.
				fTotalPowerWh += (1000.0 / REV_PER_KWH);

				if (u32TriggerTC != 0)
				{
					// Aktuellen Stromverbrauch in W errechnen....
					fCurrentPowerW = (1000.0 * 3600.0 * (1000.0 / REV_PER_KWH)) / (float)u32TimeDiff;
				}
				bDetectTrigger = true;
			}
			u32TriggerTC = millis();
		}
	}

	return bDetectTrigger;
}

///////////////////////////////////////////////////////////////////////////////
bool DetectWaterMeterTrigger(int nSensorDiff)
{
	static bool bTriggerState = false;
	static uint32_t u32TriggerTC = 0;

	bool bNextState = bTriggerState;
	bool bDetectTrigger = false;

	if (nSensorDiff > nWaterTriggerLevelHigh)
		bNextState = true;
	else if (nSensorDiff < nWaterTriggerLevelLow)
		bNextState = false;

	if (bNextState != bTriggerState)
	{
		bTriggerState = bNextState;
	  digitalWrite(ledPin, bTriggerState);

    if (bNextState)
		{
			uint32_t u32TimeDiff = millis() - u32TriggerTC;

			// Zu kurze Impulse müssen Störungen sein.
			if (u32TimeDiff > 500)
			{
				// Gesamtwasserverbrauch aufsummieren.
				fTotalWaterL += WATER_LITER_PER_REV;
				if (u32TriggerTC != 0)
				{
					// Aktuellen Wasserdurchfluss pro Minute errechnen....
					fWaterFlowLM = (1000.0 * 60.0 * WATER_LITER_PER_REV) / (float)u32TimeDiff;
				}
			}
			bDetectTrigger = true;
			u32TriggerTC = millis();
		}
	}

	return bDetectTrigger;
}

///////////////////////////////////////////////////////////////////////////////
bool DetectGasMeterTrigger(int nSensorDiff)
{
	static bool bTriggerState = false;
	static uint32_t u32TriggerTC = 0;

	bool bNextState = bTriggerState;
	bool bDetectTrigger = false;

	if (nSensorDiff > nGasTriggerLevelHigh)
		bNextState = true;
	else if (nSensorDiff < nGasTriggerLevelLow)
		bNextState = false;

	if (bNextState != bTriggerState)
	{
		bTriggerState = bNextState;
		digitalWrite(ledPin, bTriggerState);

    if (bNextState)
		{
			uint32_t u32TimeDiff = millis() - u32TriggerTC;

			// Zu kurze Impulse müssen Störungen sein. Sicher ist sicher ;-)
			if (u32TimeDiff > 2000)
			{
				// Gesamtgasverbrauch aufsummieren.
				fTotalGasL += GAS_LITER_PER_REV;

				if (u32TriggerTC != 0)
				{
					// Aktuellen Gasdurchfluss in Liter pro Minute errechnen....
					fGasFlowLM = (1000.0 * 60.0 * GAS_LITER_PER_REV) / (float)u32TimeDiff;
				}
				bDetectTrigger = true;
			}
			u32TriggerTC = millis();
		}
	}

	return bDetectTrigger;
}

///////////////////////////////////////////////////////////////////////////////
void SendRepeatedMessage(eMessage eMsg, uint32_t u32Payload, uint32_t& u32Tickcount)
{
  static uint8_t u8MessageID = 0;

   // Die mehrfach gesendeten Nachrichten erhalten alle die gleiche ID.
   // Damit sie auf der Empfängerseite auch als Kopie erkannt werden.
  u8MessageID++;

  // Die LED nur beim Senden blinken, wenn Transmitter nicht als PowerMeter/WaterMeter agiert
  if (!bDetectPowerMeter && !bDetectWaterMeter && !bDetectGasMeter)
    BlinkLED(500);

  int nTimeTC = 0;
  for (int nI = 0; nI < SEND_COUNTS; nI++)
  {
    SendMessage(eMsg, u8MessageID, u32Payload);

    // Nach der letzen Wiederholung, braucht nicht gewartet werden.
    if (nI < SEND_COUNTS-1)
    {
      // Zwischen den Sendungen warten, oder auch schlafen um Strom zu sparen,
      if (bDetectPowerMeter || bDetectWaterMeter || bDetectGasMeter)
        delay(250); // Im Energiemessungsmodus soll nicht geschlafen werden.
      else
        LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
      nTimeTC += 250; // in ms
    }
  }
  u32Tickcount+= ((500 + nTimeTC) / 1000);
}

///////////////////////////////////////////////////////////////////////////////
void SendMessage(eMessage eMsg, uint8_t u8MsgID, uint32_t u32Payload)
{
  if (deviceSettings.ulMagicHeader == MAGIC_HEADER)
  {
    DataPacket dataPacket;
    dataPacket.u8MsgID = u8MsgID;                   // MessageID
    dataPacket.eMsg = eMsg;                         // Message
    dataPacket.u32Payload = u32Payload;             // Payload
    dataPacket.u16DeviceID = deviceSettings.u16DeviceID;  // Seriennummer

#ifdef _DEBUG   
    TRACE(F("DeviceID="));
    TRACE(deviceSettings.u16DeviceID);
    TRACE("\n");
#endif

    ASK_driver.send((uint8_t *)&dataPacket, sizeof(DataPacket));
    ASK_driver.waitPacketSent();
  }
  else
    TRACE(F("SendMessage failed\n"));
}

///////////////////////////////////////////////////////////////////////////////
void BlinkLED(int nDelay)
{
  digitalWrite(ledPin, HIGH);
  delayMicroseconds(nDelay);
  digitalWrite(ledPin, LOW);
}

///////////////////////////////////////////////////////////////////////////////
bool WriteDeviceSettingsToEEProm(const DeviceSettings& deviceSettings)
{
  bool bError = false;
  unsigned char* pDeviceSettings = (unsigned char*)&deviceSettings;
  for (int nI = 0; nI < (int)sizeof(DeviceSettings); nI++)
  {
    EEPROM.update(nI, pDeviceSettings[nI]);
    if( EEPROM.read(nI) != pDeviceSettings[nI] )
      bError = true;
  }

  return !bError;
}

///////////////////////////////////////////////////////////////////////////////
DeviceSettings ReadDeviceSettingsFromEEProm()
{
  DeviceSettings deviceSettings;
  unsigned char* pDeviceSettings = (unsigned char*)&deviceSettings;
  for (int nI = 0; nI < (int)sizeof(DeviceSettings); nI++)
  {
    pDeviceSettings[nI] = EEPROM.read(nI);
  }
  return deviceSettings;
}

///////////////////////////////////////////////////////////////////////////////
void ClearEEProm()
{
  for (int nI = 0; nI < (int)sizeof(DeviceSettings); nI++)
    EEPROM.update(nI, 0);
}

///////////////////////////////////////////////////////////////////
uint32_t FloatToUint32(float fInputValue)
{
	uint32_t u32Result = 0;

	if (sizeof(u32Result) == sizeof(fInputValue))
		memcpy(&u32Result, &fInputValue, sizeof(u32Result));

	return u32Result;
}

///////////////////////////////////////////////////////////////////
float Uint32ToFloat(uint32_t u32InputValue)
{
	float fResult = 0.0;

	if (sizeof(fResult) == sizeof(u32InputValue))
		memcpy(&fResult, &u32InputValue, sizeof(fResult));

	return fResult;
}
