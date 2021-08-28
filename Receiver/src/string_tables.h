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

#ifndef __STRING_TABLES_H__
#define __STRING_TABLES_H__

// Freundlicher Namen für die einzelnen Transmitter
#ifdef USE_WINTEK
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
#endif

#ifdef USE_WINTEK_I2C
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
#endif

#ifdef USE_2004_LCD
  #define TRANSMITTER_0 F(" (Wohnraum)")
  #define TRANSMITTER_1 F(" (Garten)")
  #define TRANSMITTER_2 F(" (Buero)")
  #define TRANSMITTER_3 F(" (Terrasse)")
  #define TRANSMITTER_4 F(" (Keller)")
  #define TRANSMITTER_5 F(" (S/W/G)")
  #define TRANSMITTER_5_0 F(" (Strom)")
  #define TRANSMITTER_5_1 F(" (Wasser)")
  #define TRANSMITTER_5_2 F(" (Gas)")
  #define TRANSMITTER_6 F(" (Wind)")
  #define TRANSMITTER_7 F(" (Gefrierschrank)")
#endif

#ifdef ENABLE_NETWORK
  #ifdef AVR_MEGA2560
    // Json Bezeichner
    const char json_0[] PROGMEM = "temperature3:";  // Temperatur
    const char json_1[] PROGMEM = "humidity3:";     // Luftfeuchtigkeit
    const char json_2[] PROGMEM = "pressure3:";     // Luftdruck
    const char json_3[] PROGMEM = "currentrain3:";  // Aktueller Niederschlag
    const char json_4[] PROGMEM = "totalrain3:";    // Gesamt Niederschlag
    const char json_5[] PROGMEM = "curentpower3:";  // Aktueller Stromverbrauch
    const char json_6[] PROGMEM = "totalpower3:";   // Gesamt Stromverbrauch
    const char json_7[] PROGMEM = "waterflow3:";    // Aktueller Wasserverbrauch
    const char json_8[] PROGMEM = "totalwater3:";   // Gesamt Wasserverbrauch
    const char json_9[] PROGMEM = "gasflow3:";      // Aktueller Gasverbrauch
    const char json_10[] PROGMEM = "totalgas3:";    // Gesamt Gasverbrauch
    const char json_11[] PROGMEM = "windspeed3:";    // Windgeschwindigkeit

    const char* const g_sJsonName[] PROGMEM = {json_0, json_1, json_2, json_3, json_4, json_5, json_6, json_7, json_8, json_9, json_10, json_11};
  #else
    // Json Bezeichner
    const char json_0[] PROGMEM = "temperature2:";  // Temperatur
    const char json_1[] PROGMEM = "humidity2:";     // Luftfeuchtigkeit
    const char json_2[] PROGMEM = "pressure2:";     // Luftdruck
    const char json_3[] PROGMEM = "currentrain2:";  // Aktueller Niederschlag
    const char json_4[] PROGMEM = "totalrain2:";    // Gesamt Niederschlag
    const char json_5[] PROGMEM = "curentpower2:";  // Aktueller Stromverbrauch
    const char json_6[] PROGMEM = "totalpower2:";   // Gesamt Stromverbrauch
    const char json_7[] PROGMEM = "waterflow2:";    // Aktueller Wasserverbrauch
    const char json_8[] PROGMEM = "totalwater2:";   // Gesamt Wasserverbrauch
    const char json_9[] PROGMEM = "gasflow2:";      // Aktueller Gasverbrauch
    const char json_10[] PROGMEM = "totalgas2:";    // Gesamt Gasverbrauch
    const char json_11[] PROGMEM = "windspeed2:";   // Windgeschwindigkeit
    const char* const g_sJsonName[] PROGMEM = {json_0, json_1, json_2, json_3, json_4, json_5, json_6, json_7, json_8, json_9, json_10, json_11};
  #endif
#endif
// Sensorbezeichner des LCDs und der Webpage
#ifdef USE_WINTEK
  const char string_0[] PROGMEM  = "Temperatur....: ";
  const char string_1[] PROGMEM  = "Luftfeuchte...: ";
  const char string_2[] PROGMEM  = "Luftdruck.....: ";
  const char string_3[] PROGMEM  = "Regen aktuell.: ";
  const char string_4[] PROGMEM  = "Regen total...: ";
  const char string_5[] PROGMEM  = "Strom aktuell: ";
  const char string_6[] PROGMEM  = "Strom total..: ";
  const char string_7[] PROGMEM  = "Wasser aktuell: ";
  const char string_8[] PROGMEM  = "Wasser total..: ";
  const char string_9[] PROGMEM  = "Gas aktuell.: ";
  const char string_10[] PROGMEM = "Gas total...: ";
  const char string_11[] PROGMEM = "Wind....: ";
  const char* const g_sSensorName[] PROGMEM = {string_0, string_1, string_2, string_3, string_4, string_5, string_6, string_7, string_8, string_9, string_10, string_11};
#endif

#ifdef USE_WINTEK_I2C
  const char string_0[] PROGMEM  = "Temperatur....: ";
  const char string_1[] PROGMEM  = "Luftfeuchte...: ";
  const char string_2[] PROGMEM  = "Luftdruck.....: ";
  const char string_3[] PROGMEM  = "Regen aktuell.: ";
  const char string_4[] PROGMEM  = "Regen total...: ";
  const char string_5[] PROGMEM  = "Strom aktuell: ";
  const char string_6[] PROGMEM  = "Strom total..: ";
  const char string_7[] PROGMEM  = "Wasser aktuell: ";
  const char string_8[] PROGMEM  = "Wasser total..: ";
  const char string_9[] PROGMEM  = "Gas aktuell.: ";
  const char string_10[] PROGMEM = "Gas total...: ";
  const char string_11[] PROGMEM = "Wind....: ";
  const char* const g_sSensorName[] PROGMEM = {string_0, string_1, string_2, string_3, string_4, string_5, string_6, string_7, string_8, string_9, string_10, string_11};
#endif

#ifdef USE_2004_LCD
  const char string_0[] PROGMEM  = "Temp...: ";
  const char string_1[] PROGMEM  = "Feuchte: ";
  const char string_2[] PROGMEM  = "Druck..: ";
  const char string_3[] PROGMEM  = "Regen..: ";
  const char string_4[] PROGMEM  = "Regen \x03: ";
  const char string_5[] PROGMEM  = "Strom..: ";
  const char string_6[] PROGMEM  = "Strom \x03: ";
  const char string_7[] PROGMEM  = "Wasser..: ";
  const char string_8[] PROGMEM  = "Wasser \x03: ";
  const char string_9[] PROGMEM  = "Gas..: ";
  const char string_10[] PROGMEM = "Gas \x03: ";
  const char string_11[] PROGMEM = "Wind..: \x03: ";
  const char* const g_sSensorName[] PROGMEM = {string_0, string_1, string_2, string_3, string_4, string_5, string_6, string_7, string_8, string_9, string_10, string_11};
#endif

// Einheiten der Sensoren für das LCD
const char unit_0[] PROGMEM = " \337C";     // Temperatur in °C
const char unit_1[] PROGMEM = " %";         // Luftfeuchtigkeit
const char unit_2[] PROGMEM = " hPa";       // Luftdruck
const char unit_3[] PROGMEM = " mm/h";      // Aktueller Niederschlag
const char unit_4[] PROGMEM = " mm";        // Gesamt Niederschlag
const char unit_5[] PROGMEM = " W";         // Aktueller Stromverbrauch
const char unit_6[] PROGMEM = " kWh";       // Gesamt Stromverbrauch
const char unit_7[] PROGMEM = " l/min";     // Aktueller Wasserverbrauch
const char unit_8[] PROGMEM = " m\x02";     // Gesamt Wasserverbrauch in m³
const char unit_9[] PROGMEM = " l/min";     // Aktueller Gasverbrauch
const char unit_10[] PROGMEM = " m\x02";    // Gesamt Gasverbrauch in m³
const char unit_11[] PROGMEM = " km/h";     // Windgeschwindigkeit in km/h
const char* const g_sLCDUnits[] PROGMEM = {unit_0, unit_1, unit_2, unit_3, unit_4, unit_5, unit_6, unit_7,unit_8, unit_9, unit_10, unit_11};

#ifdef ENABLE_NETWORK
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
  const char html_unit_11[] PROGMEM = " km/h";      // Windgeschwindigkeit in km/h

  const char* const g_sHtmlUnits[] PROGMEM = {html_unit_0, html_unit_1, html_unit_2, html_unit_3, html_unit_4, html_unit_5, html_unit_6, html_unit_7,html_unit_8, html_unit_9, html_unit_10, html_unit_11};
#endif

#endif // __STRING_TABLES_H__