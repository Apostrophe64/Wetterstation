#include  <ntp.h>
#include <TimeLib.h>
typedef  uint8_t byte;

/*
 * © Francesco Potortì 2013 - GPLv3 - Revision: 1.13
 *
 * Send an NTP packet and wait for the response, return the Unix time
 *
 * To lower the memory footprint, no buffers are allocated for sending
 * and receiving the NTP packets.  Four bytes of memory are allocated
 * for transmision, the rest is random garbage collected from the data
 * memory segment, and the received packet is read one byte at a time.
 * The Unix time is returned, that is, seconds from 1970-01-01T00:00.
 */
unsigned long ntpUnixTime (WiFiUDP &udp)
{
  static int udpInited = udp.begin(123); // open socket on arbitrary port

  /* Set this to the offset (in seconds) to your local time
     This example is GMT + 1 */
  long timeZoneOffset = (1L*3600L);

//  const char timeServer[] = "pool.ntp.org";  // NTP server
  const char timeServer[] = "fritz.box";  // NTP server

  // Only the first four bytes of an outgoing NTP packet need to be set
  // appropriately, the rest can be whatever.
  const long ntpFirstFourBytes = 0xEC0600E3; // NTP request header

  // Fail if WiFiUdp.begin() could not init a socket
  if (! udpInited)
    return 0;

  // Clear received data from possible stray received packets
  udp.flush();

  // Send an NTP request
  if (! (udp.beginPacket(timeServer, 123) // 123 is the NTP port
	 && udp.write((byte *)&ntpFirstFourBytes, 48) == 48
	 && udp.endPacket()))
    return 0;				// sending request failed

  // Wait for response; check every pollIntv ms up to maxPoll times
  const int pollIntv = 150;		// poll every this many ms
  const byte maxPoll = 15;		// poll up to this many times
  int pktLen;				// received packet length
  for (byte i=0; i<maxPoll; i++) {
    if ((pktLen = udp.parsePacket()) == 48)
      break;
    delay(pollIntv);
  }
  if (pktLen != 48)
    return 0;				// no correct packet received

  // Read and discard the first useless bytes
  // Set useless to 32 for speed; set to 40 for accuracy.
  const byte useless = 40;
  for (byte i = 0; i < useless; ++i)
    udp.read();

  // Read the integer part of sending time
  unsigned long time = udp.read();	// NTP time
  for (byte i = 1; i < 4; i++)
    time = time << 8 | udp.read();

  // Round to the nearest second if we want accuracy
  // The fractionary part is the next byte divided by 256: if it is
  // greater than 500ms we round to the next second; we also account
  // for an assumed network delay of 50ms, and (0.5-0.05)*256=115;
  // additionally, we account for how much we delayed reading the packet
  // since its arrival, which we assume on average to be pollIntv/2.
  time += (udp.read() > 115 - pollIntv/8);

  // Discard the rest of the packet
  udp.flush();

  unsigned long u32UnixTime = (time - 2208988800ul  + timeZoneOffset);
  //unsigned long u32DayLightoffset = 0;
  setTime(u32UnixTime);

  if (summertime_EU(year(), month(), day(), hour(), 0))
	{
     u32UnixTime += 3600L;
     setTime(u32UnixTime);
  }

  return u32UnixTime;
}


// European Daylight Savings Time calculation by "jurs" for German Arduino Forum
// input parameters: "normal time" for year, month, day, hour and tzHours (0=UTC, 1=MEZ)
// return value: returns true during Daylight Saving Time, false otherwise
bool summertime_EU(int year, uint8_t month, uint8_t day, uint8_t hour, uint8_t tzHours)
{
  // keine Sommerzeit in Jan, Feb, Nov, Dez
if ((month < 3) || (month > 10))
    return false;

 // Sommerzeit in Apr, Mai, Jun, Jul, Aug, Sep
 if ((month > 3) && (month < 10))
  return true;

if (month==3 && (hour + 24 * day)>=(1 + tzHours + 24*(31 - (5 * year /4 + 4) % 7)) || month==10 && (hour + 24 * day)<(1 + tzHours + 24*(31 - (5 * year /4 + 1) % 7)))
	return true;
else
	return false;
}
