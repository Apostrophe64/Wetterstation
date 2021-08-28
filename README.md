# Wetterstation
Wetter und Energiemessung von
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

Der erste Empfänger besteht ebenfalls aus einem Arduino ProMini, der zur Anzeige der Messdaten
ein 4 zeiliges LCD verwendete. Zur Übertragung ins WLAN verwendet dieser ein ESP8266 ESP-01S.

Der zweite Empfänger besteht aus einem ESP8260NodeMCU mit 433MHz Receiver und einem 4.3" SPI Display
zur Darstellung der Messdaten. Dieser reicht die Daten zusätzlich an einem Raspberry PI weiter, auf dem
Emoncms installiert ist. Außerdem enthält er einen kleinen Webserver, der eine Html Seite mit allen Messwerten ausliefert.

Die Entwicklung erfolgte mit Microsoft Visual Studio Code und Platformio.
Alle notwendigen Einstellungen wie Netzwerkadressen, SSID, API Keys Passwörter sind direkt im Quellcode
als define bzw. const Variablen implementiert.

Ein paar Worte noch zum Batteriebetrieb und der Stromaufnahme.

Um den Stromverbrauch der Sender zu minimieren, habe ich den Spannungsregler und die PowerLED der Arduinos entfernt
und anstatt der 9V Blockbatterie 3 AAA Batterien verwendet.

Stromverbrauch der ProMinis:

Taktfrequenz | Stromaufnahme
-------------|--------------
CPU 16MHz | 10,75mA
CPU 8MHz  | 6,32mA
CPU 1MHz  |  2,28mA (Software läuft aber nicht)

Stromaufnahme mit PCF8583: ca. 7,5µA bzw. 39,2µA wenn Eingang geschlossen ist.

Sleep:  5.5µA (Ohne Spannungsregler, mit 3xAAA Batterien)

Sensor | Messzeiten der Sensoren
------------|--------------------
DHT22 Read  |        5,3ms
BMP085 Read |     34,0ms
DS1820 Read |     28,0ms
433MHz transmitt | 120.0ms

---> Aktivezeit: 5,3+34+3*120.0=400ms

Messzyklus: 300 Sekunden  
Sleepzeit: 8 Sekunden  
Passivzeit (Ohne Messung) 16µS alle 8 Sekuden (Vernachlässigbar)  

Mittlerer Stromverbrauch (Ohne Spannungsregler, mit 3xAAA Batterie, 16MHz):  
(400ms * 10,75mA +(300000 - 400ms) * 0,0055mA) / 300000 = 0,0198mA = 19,8µA  
Mittlerer Stromverbrach = ~20µA  
P = 4,5V * 20µA = **89µW**


Mittlerer Stromverbrauch (Ohne Spannungsregler, mit 3xAAA Batterien, 8MHz):  
(400ms * 6,32mA +(300000 - 400ms) * 0,0055mA) / 300000 = 0,0139mA = 13,9µA  
Mittlerer Stromverbrach = ~14µA  
P = 4.5V * 14µA = **62,6µW** (~30% Ersparnis gegenüber der 16MHz Version)  


// AAA Alkaline Batterie ca. 900mAh (Selbstendladung ca. 35µA)  
Geschätzte Laufzeit 16MHz: 900mAh / (20+35)µA = 16363 Stunden = 682 Tage = **1,8 Jahre**   
Geschätzte Laufzeit 8MHz:  900mAh / (14+35)µA = 18367 Stunden = 765 Tage = **2,1 Jahre**  


In der Tat läuft einer der Sender derzeit bereits 15 Monate ohne Batteriewechsel!
Allerdings gibt es noch ein  Problem mit der Stromaufnahme des Senders mit dem Luftdruckmesser.
Bei diesem ist die Batterie bereit nach einer Woche leer. Wenn ich die Stromaufnahme hier messe, ist diese allerdings
nicht erhöht. Mit der 9V Blockbatterie lief dieser Sender allerdings durchaus 6 Monate.

