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
Bis auf die Energieverbrauchmessung laufen alle Sender im Batteriebetrieb. 

Der grobe Aufbau ist wie folgt:
Es werden derzeit 8 433MHz Sender an einer Empfangsstation betrieben.
Die Sender bestehen aus einem Arduino PRO Mini mit jeweils einem 433MHz Transmitter und entsprechenden Sensoren.
Die Firmware ist für alle Transmitter identisch. Es wird automatisch erkannt, welche Sensoren angeschlossen sind.
Schaltpläne habe ich nicht erstellt. Wen es interessiert, schaut sich die Fotos und den Quellcode an.

Der Empfänger besteht aus einem ESP8260NodeMCU mit 433MHz Receiver und einem 4.3" SPI Display zur Darstellung der Messdaten.
Dieser reicht die Daten zusätzlich an einem Raspberry PI weiter, auf dem Emoncms installiert ist.
Außerdem enthält er einen kleinen Webserver, der eine Html Seite mit allen Messwerten ausliefert.

Die Entwicklung erfolgte mit Microsoft Visual Studio Code und Platformio.
Alle notwendigen Einstellungen wie Netzwerkadressen, SSID, API Keys Passwörter sind direkt im Quellcode
als define bzw. const variablen implementiert.

