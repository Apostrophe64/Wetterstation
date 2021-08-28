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

#include <sensorData.h>
#include <Arduino.h>

std::vector<SensorData> g_SensorDatas;

/////////////////////////////////////////////////////////////////////////////
SensorData::SensorData()
{
}

/////////////////////////////////////////////////////////////////////////////
SensorData::SensorData(uint8_t u8Transmitter, SensorType sensorType, float fValue)
{
  Add(u8Transmitter, sensorType, fValue);
}

/////////////////////////////////////////////////////////////////////////////
SensorData::~SensorData()
{
    m_fValues.clear();
}

/////////////////////////////////////////////////////////////////////////////
void SensorData::Add(uint8_t u8Transmitter, SensorType sensorType, float fValue)
{
  // Die Sensorwerte werden aus Speicherplatzgründen als 16Bit Festkomma abgelegt
  //uint16_t u16Val = (uint16_t)(0.5 + fValue * 10.0);
  if (m_fValues.size() == 0)
  {
    int nSize = 1;
    switch (sensorType)
    {
      case eTemperatur:
      case eHumidity:
      case ePressure:
        nSize = MAX_SENSOR_DATAS;
        break;
      default:
        nSize = 1;
    }
    m_fValues.reserve(nSize);
    m_fValues.assign(nSize, fValue);
  }
  else if (m_fValues.size() == 1)
  {
    m_fValues[0] = fValue;
  }
  else
  {
    int nSize = m_fValues.size();
    for (int nI = 1; nI < nSize; nI++)
      m_fValues[nI - 1] = m_fValues[nI];
    m_fValues[nSize - 1] = fValue;

  }
  m_u16TC = millis() / 1000L;   // Auf eine Sekunde genau reicht hier.

  // Anhand einer eindeutigen SenorID werden die Sensordaten wieder gefunden
  m_u8SensorID = ::GetSensorID(u8Transmitter, sensorType);
  m_bNewValue = true;
}

/////////////////////////////////////////////////////////////////////////////
uint8_t SensorData::GetSensorID()
{
    return m_u8SensorID;
}

/////////////////////////////////////////////////////////////////////////////
uint8_t SensorData::GetTransmitter()
{
    return (m_u8SensorID >> 4);
}

/////////////////////////////////////////////////////////////////////////////
SensorType SensorData::GetSensorType()
{
  return (SensorType)(m_u8SensorID & 0x0f);
}

/////////////////////////////////////////////////////////////////////////////
float SensorData::GetSensorValue()
{
    float fResult = 0.0;
    if (m_fValues.size() == 1)
      fResult = GetSensorValue(0);

    return fResult;
}

/////////////////////////////////////////////////////////////////////////////
float SensorData::GetSensorValue(uint8_t nI)
{
    float fResult = 0.0;
    if (nI < m_fValues.size())
      fResult = (float)m_fValues[nI];

    return fResult;
}
/////////////////////////////////////////////////////////////////////////////
uint16_t SensorData::GetTickCount()
{
    return m_u16TC;
}

/////////////////////////////////////////////////////////////////////////////
bool SensorData::HasNewValue()
{
    return m_bNewValue;
}

/////////////////////////////////////////////////////////////////////////////
void SensorData::ClearNewValueFlag()
{
    m_bNewValue = false;
}

/////////////////////////////////////////////////////////////////////////////
uint16_t SensorData::GetSize()
{
    return (uint16_t)m_fValues.size();
}

// Wenn nur ein Messwert erfasst werden soll, werden diese Methoden nicht benötigt.
#if MAX_SENSOR_DATAS > 1

///////////////////////////////////////////////////////////////////////////////
float SensorData::LinearRegression()
{
	float sumX = 0.0;
	float sumY = 0.0;
	float sumXY = 0.0;
	float sumXX = 0.0;

  int nSize = m_fValues.size();
	for (int nX = 0; nX < nSize; nX++)
	{
    float fY = GetSensorValue(nX); // Die Sensordaten sind mit 10 multipliziert-
		sumXY += ((float)nX*fY);
		sumX += (float)nX;
		sumY += (float)fY;
		sumXX += (float)nX * (float)nX;
	}
	float m = ((float)nSize * sumXY - (sumX * sumY)) / ((float)nSize * sumXX - (sumX*sumX));

  return m;
}

///////////////////////////////////////////////////////////////////////////////
float SensorData::GetArithmeticMean()
{
	float fAMean = 0.0;
  int nSize = m_fValues.size();
	for (int nX = 0; nX < nSize; nX++)
	{
		float fY = GetSensorValue(nX);
		fAMean += (float)(nX+1)*fY;
  }
	fAMean /= (nSize*(nSize+1)/2);

	return fAMean;
}
#endif

/////////////////////////////////////////////////////////////////////////////
// u8Transmitter <= 15, sensorType <= 15
uint8_t	GetSensorID(uint8_t u8Transmitter, SensorType sensorType)
{
  uint8_t u8SensorID = 0;
  if ((u8Transmitter <= 15) && (sensorType <= 15))
    u8SensorID = (u8Transmitter << 4 | (uint8_t)sensorType);

  return u8SensorID;
}

/////////////////////////////////////////////////////////////////////////////
bool StoreSensorData(uint8_t u8Transmitter, SensorType sensorType, float fValue)
{
	bool bFoundEntry = false;
  uint8_t u8SensorID = GetSensorID(u8Transmitter, sensorType);
	for (auto it = g_SensorDatas.begin(); it != g_SensorDatas.end() && (bFoundEntry == false); it++)
	{
    if (it->m_u8SensorID == u8SensorID)
		{
			it->Add(u8Transmitter, sensorType, fValue);
			bFoundEntry = true;
		}
	}
	if (!bFoundEntry)
  {
      g_SensorDatas.reserve(PRE_RESERVE_SENSOR_DATAS);
      g_SensorDatas.push_back(SensorData(u8Transmitter, sensorType, fValue));
  }

	return bFoundEntry;
}

/////////////////////////////////////////////////////////////////////////////
SensorData* GetSensorData(uint8_t u8Transmitter, SensorType sensorType)
{
	SensorData* pSensorData = NULL;
	bool bFoundEntry = false;
  uint8_t u8SensorID = GetSensorID(u8Transmitter, sensorType);
	for (auto it = g_SensorDatas.begin(); it != g_SensorDatas.end() && (bFoundEntry == false); it++)
	{
    if (it->m_u8SensorID == u8SensorID)
    {
			pSensorData = it;
			bFoundEntry = true;
		}
	}
	return pSensorData;
}

/////////////////////////////////////////////////////////////////////////////
bool HasAnySensorData(uint8_t u8Transmitter)
{
  bool bResult = false;
	for (auto it = g_SensorDatas.begin(); it != g_SensorDatas.end() && bResult == false; it++)
	{
		if (it->GetTransmitter() == u8Transmitter)
		{
			if (it->m_u16TC != 0)
			   bResult = true;
		}
	}
	return bResult;
}

/////////////////////////////////////////////////////////////////////////////
bool HasAnyNewSensorData(uint8_t u8Transmitter)
{
  bool bResult = false;
	for (auto it = g_SensorDatas.begin(); it != g_SensorDatas.end() && bResult == false; it++)
	{
		if (it->GetTransmitter() == u8Transmitter)
		{
			if (it->m_bNewValue == true)
			   bResult = true;
		}
	}
	return bResult;
}

/////////////////////////////////////////////////////////////////////////////
bool ClearAllNewValueFlags(uint8_t u8Transmitter)
{
  bool bFoundEntry = false;
	for (auto it = g_SensorDatas.begin(); it != g_SensorDatas.end(); it++)
	{
		if (it->GetTransmitter() == u8Transmitter)
		{
			it->m_bNewValue = false;
      bFoundEntry = true;
		}
	}
	return bFoundEntry;
}

/////////////////////////////////////////////////////////////////////////////
bool ClearNewValueFlag(uint8_t u8Transmitter, SensorType sensorType)
{
	bool bFoundEntry = false;
  uint8_t u8SensorID = GetSensorID(u8Transmitter, sensorType);
	for (auto it = g_SensorDatas.begin(); it != g_SensorDatas.end() && (bFoundEntry == false); it++)
	{
    if (it->GetSensorID() == u8SensorID)
		{
			it->m_bNewValue = false;
			bFoundEntry = true;
		}
	}
	return bFoundEntry;
}
