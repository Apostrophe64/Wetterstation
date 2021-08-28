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

#ifndef __SENSORDATA_H__
#define __SENSORDATA_H__

#include <vector>

//const int MAX_SENSOR_DATAS = 5;   // Mehr geht nicht, da nicht genügend Ram
//const int MAX_TRANSMITTER  = 6;  //   ""

const int MAX_SENSOR_DATAS = 1; 
const int MAX_TRANSMITTER  = 8; 
 // Vorraussichtliche max. Gesamtanzahl der Sensordaten
 // Reserviert den m_SensorDatas Vektor vor.
 // Wohnzimmer: Temp, Hum -> 2
 // Garten:		Temp, Hum, Press -> 3
 // Büro:		Temp, Hum -> 2
 // Terrasse	Temp, Rain total, Rain current -> 3
 // Keller:		Temp, Hum: -> 2
 // Verbrauch:  Strom, Wasser, Gas, jeweils total und aktuell -> 6
 // Windspeed:  Windgeschwindigkeit: -> 1
 // ====> Summe: 19
const int PRE_RESERVE_SENSOR_DATAS = 20;

/////////////////////////////////////////////////////////////////////////////
enum SensorType
{
	eInvalid = -1,
	eTemperatur,
	eHumidity,
	ePressure,

	eRainCurrent,
	eRainTotal,

	ePowerCurrent,
	ePowerTotal,

	eWaterCurrent,
	eWaterTotal,

	eGasCurrent,
	eGasTotal,

	eWindSpeed
}__attribute__ ((packed));

class SensorData
{
public:
	SensorData();
	SensorData(uint8_t u8Transmitter, SensorType sensorType, float fValue);
	~SensorData();

	void 		Add(uint8_t u8Transmitter, SensorType sensorType, float fValue);
	float 		GetSensorValue();
	float 		GetSensorValue(uint8_t nI);
	uint16_t 	GetTickCount();
	uint16_t 	GetSize();

	bool 		HasNewValue();
	void 		ClearNewValueFlag();
#if MAX_SENSOR_DATAS > 1
	float 		GetArithmeticMean();
	float 		LinearRegression();
#endif
	uint8_t 	GetTransmitter();
	SensorType	GetSensorType();

private:
	uint8_t		GetSensorID();

	bool					m_bNewValue = false;
	uint8_t					m_u8SensorID = 0;
	uint16_t				m_u16TC = 0;
	std::vector<float> 		m_fValues;

	friend bool 		StoreSensorData(uint8_t u8Transmitter, SensorType sensorType, float fValue);
	friend SensorData* 	GetSensorData(uint8_t u8Transmitter, SensorType sensorType);
	friend uint8_t		GetSensorID(uint8_t u8Transmitter, SensorType sensorType);
	friend bool 		ClearNewValueFlag(uint8_t u8Transmitter, SensorType sensorType);
	friend bool 		ClearAllNewValueFlags(uint8_t u8Transmitter);
	friend bool 		HasAnySensorData(uint8_t u8Transmitter);
	friend bool 		HasAnyNewSensorData(uint8_t u8Transmitter);
}; //__attribute__ ((packed, aligned(1)));

bool StoreSensorData(uint8_t u8Transmitter, SensorType sensorType, float fValue);
SensorData* GetSensorData(uint8_t u8Transmitter, SensorType sensorType);
uint8_t	GetSensorID(uint8_t u8Transmitter, SensorType sensorType);
bool ClearNewValueFlag(uint8_t u8Transmitter, SensorType sensorType);
bool ClearAllNewValueFlags(uint8_t u8Transmitter);
bool HasAnySensorData(uint8_t u8Transmitter);
bool HasAnyNewSensorData(uint8_t u8Transmitter);
#endif // __SENSORDATA_H__
