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

// Freundlicher Namen für die einzelnen Transmitter
#define TRANSMITTER_0 F(" (Wohnzimmer)")
#define TRANSMITTER_1 F(" (Garten)")
#define TRANSMITTER_2 F(" (Buero)")
#define TRANSMITTER_3 F(" (Terrasse)")
#define TRANSMITTER_4 F(" (Keller)")
#define TRANSMITTER_5 F(" (Strom/Wasser/Gas)")
#define TRANSMITTER_5_0 F(" (Strom)")
#define TRANSMITTER_5_1 F(" (Wasser)")
#define TRANSMITTER_5_2 F(" (Gas)")
#define TRANSMITTER_6 F(" (Wind)")
#define TRANSMITTER_7 F(" (Gefrierschrank)")

const char month_0[]  PROGMEM = "----";  // Monat 0 ist nur ein Dummyeintrag
const char month_1[]  PROGMEM = "Januar";
const char month_2[]  PROGMEM = "Februar";
const char month_3[]  PROGMEM = "Maerz";
const char month_4[]  PROGMEM = "April";
const char month_5[]  PROGMEM = "Mai";
const char month_6[]  PROGMEM = "Juni";
const char month_7[]  PROGMEM = "Juli";
const char month_8[]  PROGMEM = "August";
const char month_9[]  PROGMEM = "September";
const char month_10[] PROGMEM = "Oktober";
const char month_11[] PROGMEM = "November";
const char month_12[] PROGMEM = "Dezember";
const char* const g_sMonth[] PROGMEM = {month_0, month_1, month_2, month_3, month_4, month_5, month_6, month_7, month_8, month_9, month_10, month_11, month_12};

// Json Bezeichner
const char json_0[] PROGMEM = "temperature:";  // Temperatur
const char json_1[] PROGMEM = "humidity:";     // Luftfeuchtigkeit
const char json_2[] PROGMEM = "pressure:";     // Luftdruck
const char json_3[] PROGMEM = "currentrain:";  // Aktueller Niederschlag
const char json_4[] PROGMEM = "totalrain:";    // Gesamt Niederschlag
const char json_5[] PROGMEM = "curentpower:";  // Aktueller Stromverbrauch
const char json_6[] PROGMEM = "totalpower:";   // Gesamt Stromverbrauch
const char json_7[] PROGMEM = "waterflow:";    // Aktueller Wasserverbrauch
const char json_8[] PROGMEM = "totalwater:";   // Gesamt Wasserverbrauch
const char json_9[] PROGMEM = "gasflow:";      // Aktueller Gasverbrauch
const char json_10[] PROGMEM = "totalgas:";    // Gesamt Gasverbrauch
const char json_11[] PROGMEM = "windspeed:";   // Windgeschwindigkeit

const char* const g_sJsonName[] PROGMEM = {json_0, json_1, json_2, json_3, json_4, json_5, json_6, json_7, json_8, json_9, json_10, json_11};

// Sensorbezeichner der Webpage
const char string_0[] PROGMEM  = "Temperatur....: ";
const char string_1[] PROGMEM  = "Luftfeuchte...: ";
const char string_2[] PROGMEM  = "Luftdruck.....: ";
const char string_3[] PROGMEM  = "Regen aktuell.: ";
const char string_4[] PROGMEM  = "Regen im ";
const char string_5[] PROGMEM  = "Strom aktuell: ";
const char string_6[] PROGMEM  = "Strom total..: ";
const char string_7[] PROGMEM  = "Wasser aktuell: ";
const char string_8[] PROGMEM  = "Wasser total..: ";
const char string_9[] PROGMEM  = "Gas aktuell.: ";
const char string_10[] PROGMEM = "Gas total...: ";
const char string_11[] PROGMEM = "Windgeschwindigkeit: ";
const char* const g_sSensorName[] PROGMEM = {string_0, string_1, string_2, string_3, string_4, string_5, string_6, string_7, string_8, string_9, string_10, string_11};

// Sensorbezeichner des TFTs
const char tft_sensor_0[]  PROGMEM  = "Temperatur ";
const char tft_sensor_1[]  PROGMEM  = "Luftfeuchtigkeit ";
const char tft_sensor_2[]  PROGMEM  = "Luftdruck ";
const char tft_sensor_3[]  PROGMEM  = "Niederschlag aktuell ";
const char tft_sensor_4[]  PROGMEM  = "Niederschlag im ";
const char tft_sensor_5[]  PROGMEM  = "Stromverbrauch aktuell ";
const char tft_sensor_6[]  PROGMEM  = "Stromverbrauch gesamt ";
const char tft_sensor_7[]  PROGMEM  = "Wasserverbrauch aktuell ";
const char tft_sensor_8[]  PROGMEM  = "Wasserverbrauch gesamt ";
const char tft_sensor_9[]  PROGMEM  = "Gasverbrauch aktuell ";
const char tft_sensor_10[] PROGMEM  = "Gasverbrauch gesamt ";
const char tft_sensor_11[] PROGMEM  = "Windgeschwindigkeit ";
const char* const g_sTFTSensorName[] PROGMEM = {tft_sensor_0, tft_sensor_1, tft_sensor_2, tft_sensor_3, tft_sensor_4, tft_sensor_5, tft_sensor_6, tft_sensor_7, tft_sensor_8, tft_sensor_9, tft_sensor_10, tft_sensor_11};

// Einheiten der Sensoren für das TFT Display
const char tft_unit_0[]  PROGMEM = "\367C   ";    // Temperatur in °C
const char tft_unit_1[]  PROGMEM = "%    ";       // Luftfeuchtigkeit
const char tft_unit_2[]  PROGMEM = "hPa  ";       // Luftdruck
const char tft_unit_3[]  PROGMEM = "mm/h ";       // Aktueller Niederschlag
const char tft_unit_4[]  PROGMEM = "mm   ";       // Gesamt Niederschlag
const char tft_unit_5[]  PROGMEM = "Watt ";       // Aktueller Stromverbrauch
const char tft_unit_6[]  PROGMEM = "kWh  ";       // Gesamt Stromverbrauch
const char tft_unit_7[]  PROGMEM = "l/min";       // Aktueller Wasserverbrauch
const char tft_unit_8[]  PROGMEM = "m\xfe   ";    // Gesamt Wasserverbrauch in m³
const char tft_unit_9[]  PROGMEM = "l/min";       // Aktueller Gasverbrauch
const char tft_unit_10[] PROGMEM = "m\xfe   ";    // Gesamt Gasverbrauch in m³
const char tft_unit_11[] PROGMEM = "km/h   ";     // Windgeschwindigkeit in km/h
const char* const g_sTFTUnits[] PROGMEM = {tft_unit_0, tft_unit_1, tft_unit_2, tft_unit_3, tft_unit_4, tft_unit_5, tft_unit_6, tft_unit_7,tft_unit_8, tft_unit_9, tft_unit_10, tft_unit_11};

// Einheiten der Sensoren für die Webpage
const char html_unit_0[] PROGMEM = " &degC";     // Temperatur in °C
const char html_unit_1[] PROGMEM = " %";         // Luftfeuchtigkeit
const char html_unit_2[] PROGMEM = " hPa";       // Luftdruck
const char html_unit_3[] PROGMEM = " mm/h";      // Aktueller Niederschlag
const char html_unit_4[] PROGMEM = " mm";        // Gesamt Niederschlag
const char html_unit_5[] PROGMEM = " W";         // Aktueller Stromverbrauch
const char html_unit_6[] PROGMEM = " kWh";       // Gesamt Stromverbrauch
const char html_unit_7[] PROGMEM = " l/min";     // Aktueller Wasserverbrauch
const char html_unit_8[] PROGMEM = " m³";        // Gesamt Wasserverbrauch in m³
const char html_unit_9[] PROGMEM = " l/min";     // Aktueller Gasverbrauch
const char html_unit_10[] PROGMEM = " m³";       // Gesamt Gasverbrauch in m³
const char html_unit_11[] PROGMEM = " km/h";     // Windgeschwindigkeit in km/h
const char* const g_sHtmlUnits[] PROGMEM = {html_unit_0, html_unit_1, html_unit_2, html_unit_3, html_unit_4, html_unit_5, html_unit_6, html_unit_7,html_unit_8, html_unit_9, html_unit_10, html_unit_11};
