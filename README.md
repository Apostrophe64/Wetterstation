# Wetterstation
Wetter und Energiemessung

Messung von
- Temperatur
- Luftfeuchte
- Luftdruck
- Windgeschwindigkeit
- Niederschlag
- Messung des Stromverbrauchs
- Messung des Gasverbrauchs
- Messung des Wasserverbrauchs

Das Projekt läuft bei mir seit etwa Mitte 2017 mit 8 Transmittern. Teilweise im Außenbereich.
Bis auf die Energieverbrauchsmessung laufen alle Sender im Batteriebetrieb. 

Der grobe Aufbau ist wie folgt:
Es werden derzeit 8 433MHz Sender an einer Empfangsstation betrieben.
Die Sender bestehen aus einem Arduino ProMini mit jeweils einem 433MHz Transmitter und entsprechenden Sensoren.
Die Firmware ist für alle Transmitter identisch. Es wird automatisch erkannt, welche Sensoren angeschlossen sind.
Schaltpläne habe ich derzeit nicht erstellt. Ich hoffe der Quellcode ist ausreichend kommentiert.
Als Sensoren verwende ich die folgenden Komponenten:
- DHT22 zur Messung der Temperatur und Luftfeuchte 
- DS1820 zur Messung der Temperatur
- BMP085 zur Messung des Luftdrucks
- PCF8583 als Zähler zur Messung der Windgeschwindigkeit
- SN74HC14 Schmitt Trigger zum Entprellen des Readrelais bei der Messung der Windgeschwindigkeit.
- TCRT5000 Infrarot Relexlichtschranke zur Ablesung der Strom/Gas/Wasser-zähler
- Hallsensor zur Ablesung des Wasserzählers. (War leider nach einem Wechsel des Wasserzählers notwendig.)

Der erste Empfänger besteht ebenfalls aus einem Arduino ProMini, der zu Anzeige der Messdaten
ein 4 zeiliges LCD Display verwendete. Zur Übertragung ins WLAN verwendet dieser ein ESP8266 ESP-01S.

Der zweite Empfänger besteht aus einem ESP8260NodeMCU mit 433MHz Receiver und einem 4.3" SPI Display
zur Darstellung der Messdaten. Dieser reicht die Daten zusätzlich an einem Raspberry PI weiter, auf dem
Emoncms installiert ist. Außerdem enthält er einen kleinen Webserver, der eine Html Seite mit allen Messwerten ausliefert.

Die Entwicklung erfolgte mit Microsoft Visual Studio Code und Platformio.
Alle notwendigen Einstellungen wie Netzwerkadressen, SSID, API Keys Passwörter sind direkt im Quellcode
als define bzw. const Variablen implementiert.

