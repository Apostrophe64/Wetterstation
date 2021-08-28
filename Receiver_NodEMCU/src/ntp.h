#ifndef __NTP_H
#define __NTP_H__

#include <WiFiUdp.h>
#include <Arduino.h>

static WiFiUDP udp;
unsigned long ntpUnixTime(WiFiUDP& udp);
bool summertime_EU(int year, uint8_t month, uint8_t day, uint8_t hour, uint8_t tzHours);

#endif // __NTP_H__
