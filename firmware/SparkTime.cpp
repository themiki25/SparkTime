/* Spark Time by Brian Ogilvie
   Inspired by Arduino Time by Michael Margolis
   Copyright (c) 2014 Brian Ogilvie.  All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   - Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
     AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
     IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
     ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
     LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
     CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
     SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
     INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
     CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
     ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
     POSSIBILITY OF SUCH DAMAGE.
   TODO:
         Translation for other languages

*/
/****************************** New Librares************************
 ******* DS1307 timer
 * ***** 5883L Mageto chip : 8/22/15
 * ****************************************************************/

#include "SparkTime.h"

//#include <Wire.h>
// Energia support
//#ifndef ENERGIA
//#include <avr/pgmspace.h>
//#else
#define pgm_read_word(data) *data
#define pgm_read_byte(data) *data
#define PROGMEM
//#endif
//#include "RTClib.h"
//#include <Arduino.h>


SparkTime::SparkTime()
{
    _UDPClient = NULL;
    _timezone = -5;
    _useDST = true;
    _useEuroDSTRule = false;
    _syncedOnce = false;
    _interval = 60UL * 60UL;
    _localPort = 2390;
}

void SparkTime::begin(UDP * UDPClient, const char * NTPServer) {
  _UDPClient = UDPClient;
  memcpy(_serverName, NTPServer, strlen(NTPServer)+1);
}

void SparkTime::begin(UDP * UDPClient) {
  _UDPClient = UDPClient;
  const char NTPServer[] = "pool.ntp.org";
  memcpy(_serverName, NTPServer, strlen(NTPServer)+1);
}

bool SparkTime::hasSynced() {
  return _syncedOnce;
}

void SparkTime::setUseDST(bool value) {
  _useDST = value;
}

void SparkTime::setUseEuroDSTRule(bool value) {
  _useEuroDSTRule = value;
}

uint8_t SparkTime::hour(uint32_t tnow) {
  uint8_t hTemp = ((tnow+timeZoneDSTOffset(tnow)) % 86400UL)/3600UL;
  return hTemp;
}


uint8_t SparkTime::minute(uint32_t tnow) {
  return (((tnow+timeZoneDSTOffset(tnow)) % 3600) / 60);
}


uint8_t SparkTime::second(uint32_t tnow) {
  return ((tnow+timeZoneDSTOffset(tnow)) % 60);
}


uint8_t SparkTime::dayOfWeek(uint32_t tnow) {
  uint32_t dayNum = (tnow + timeZoneDSTOffset(tnow)-SPARKTIMEEPOCHSTART)/SPARKTIMESECPERDAY;   
  //Unix epoch day 0 was a thursday
  return ((dayNum+4)%7);
}


uint8_t SparkTime::day(uint32_t tnow) {
  uint32_t dayNum = (tnow+timeZoneDSTOffset(tnow)-SPARKTIMEBASESTART)/SPARKTIMESECPERDAY;
  uint32_t tempYear = SPARKTIMEBASEYEAR;
  uint8_t  tempMonth = 0;

  while(dayNum >= YEARSIZE(tempYear)) {
    dayNum -= YEARSIZE(tempYear);
    tempYear++;
  }
  
  while(dayNum >= _monthLength[LEAPYEAR(tempYear)][tempMonth]) {
    dayNum -= _monthLength[LEAPYEAR(tempYear)][tempMonth];
    tempMonth++;
  }
  dayNum++;			// correct for zero-base
  return (uint8_t)dayNum;
}

uint8_t SparkTime::month(uint32_t tnow) {
  uint32_t dayNum = (tnow+timeZoneDSTOffset(tnow)-SPARKTIMEBASESTART)/SPARKTIMESECPERDAY;
  uint32_t tempYear = SPARKTIMEBASEYEAR;
  uint8_t  tempMonth = 0;

  while(dayNum >= YEARSIZE(tempYear)) {
    dayNum -= YEARSIZE(tempYear);
    tempYear++;
  }
  
  while(dayNum >= _monthLength[LEAPYEAR(tempYear)][tempMonth]) {
    dayNum -= _monthLength[LEAPYEAR(tempYear)][tempMonth];
    tempMonth++;
  }
  tempMonth++;
  return tempMonth;
}


uint32_t SparkTime::year(uint32_t tnow) {
  uint32_t dayNum = (tnow+timeZoneDSTOffset(tnow)-SPARKTIMEBASESTART)/SPARKTIMESECPERDAY;
  uint32_t tempYear = SPARKTIMEBASEYEAR;

  while(dayNum >= YEARSIZE(tempYear)) {
    dayNum -= YEARSIZE(tempYear);
    tempYear++;
  }
  return tempYear;
}

String SparkTime::hourString(uint32_t tnow) {
  return String(_digits[hour(tnow)]);
}

String SparkTime::hour12String(uint32_t tnow) {
  uint8_t tempHour = hour(tnow);
  if (tempHour>12) {
    tempHour -= 12;
  }
  if (tempHour == 0) {
    tempHour = 12;
  }
  return String(_digits[tempHour]);
}


String SparkTime::minuteString(uint32_t tnow) {
  return String(_digits[minute(tnow)]);
}

String SparkTime::secondString(uint32_t tnow) {
  return String(_digits[second(tnow)]);
}

String SparkTime::AMPMString(uint32_t tnow) {
  uint8_t tempHour = hour(tnow);
  if (tempHour<12) {
    return String("AM");
  } else {
    return String("PM");
  }
}

String SparkTime::dayOfWeekShortString(uint32_t tnow) {
  return String(_days_short[dayOfWeek(tnow)]);
}

String SparkTime::dayOfWeekString(uint32_t tnow) {
  return String(_days[dayOfWeek(tnow)]);
}

String SparkTime::dayString(uint32_t tnow) {
  return String(_digits[day(tnow)]);
}

String SparkTime::monthString(uint32_t tnow) {
  return String(_digits[month(tnow)]);
}

String SparkTime::monthNameShortString(uint32_t tnow) {
  return String(_months_short[month(tnow)-1]);
}

String SparkTime::monthNameString(uint32_t tnow) {
  return String(_months[month(tnow)-1]);
}

String SparkTime::yearShortString(uint32_t tnow) {
  uint32_t tempYear = year(tnow)%100;
  if (tempYear<10) {
    return String(_digits[tempYear]);
  } else {
    return String(tempYear);
  }
}

String SparkTime::yearString(uint32_t tnow) {
  return String(year(tnow));
}

String SparkTime::ISODateString(uint32_t tnow) {
  String ISOString;
  ISOString += yearString(tnow);
  ISOString += "-";
  ISOString += monthString(tnow);
  ISOString += "-";
  ISOString += dayString(tnow);
  ISOString += "T";
  ISOString += hourString(tnow);
  ISOString += ":";
  ISOString += minuteString(tnow);
  ISOString += ":";
  ISOString += secondString(tnow);

  int32_t offset = timeZoneDSTOffset(tnow)/3600L;
  // Guard against timezone problems
  if (offset>-24 && offset<24) { 
    if (offset < 0) {
      ISOString = ISOString + "-" + _digits[-offset] + "00";
    } else {
      ISOString = ISOString + "+" + _digits[offset] + "00";
    }
  }
  return ISOString;
}

String SparkTime::ISODateUTCString(uint32_t tnow) {
  uint32_t savedTimeZone = _timezone;
  bool savedUseDST = _useDST;
  _timezone = 0;
  _useDST = false;

  String ISOString;
  ISOString += yearString(tnow);
  ISOString += "-";
  ISOString += monthString(tnow);
  ISOString += "-";
  ISOString += dayString(tnow);
  ISOString += "T";
  ISOString += hourString(tnow);
  ISOString += ":";
  ISOString += minuteString(tnow);
  ISOString += ":";
  ISOString += secondString(tnow);
  ISOString += "Z";

  _timezone = savedTimeZone;
  _useDST = savedUseDST;

  return ISOString;
}

uint32_t SparkTime::now() {
  if (!_syncedOnce && !_isSyncing) {
    updateNTPTime();
  }
  if (!_syncedOnce) {  // fail!
      return SPARKTIMEBASEYEAR;	// Jan 1, 2014
  }
  return nowNoUpdate();
}

uint32_t SparkTime::nowNoUpdate() {
  uint32_t mTime = millis();
  //unsigned long mFracSec = (mTime%1000); // 0 to 999 miliseconds
  //unsigned long nowFrac  = _lastSyncNTPFrac + mFracSec*2^22/1000;
  uint32_t nowSec   = _lastSyncNTPTime + ((mTime - _lastSyncMillisTime)/1000);
  if (_lastSyncMillisTime>mTime) { // has wrapped
    nowSec = nowSec + SPARKTIMEWRAPSECS;
  }
  if (nowSec >= (_lastSyncNTPTime + _interval)) {
    updateNTPTime();
  }
  return nowSec;  
}

uint32_t SparkTime::nowEpoch() {
    return (now()-SPARKTIMEEPOCHSTART);
}

uint32_t SparkTime::lastNTPTime() {
    return _lastSyncNTPTime;
}

void SparkTime::setNTPInvterval(uint32_t intervalMinutes) {
  const uint32_t fiveMinutes = 5UL * 60UL;
  uint32_t interval = intervalMinutes * 60UL;
  _interval = max(fiveMinutes, interval);
}

void SparkTime::setTimeZone(int32_t hoursOffset) {
  _timezone = hoursOffset;
}

bool SparkTime::isUSDST(uint32_t tnow) {
  // 2am 2nd Sunday in March to 2am 1st Sunday in November
  // can't use offset here 
  bool result = false;
  uint32_t dayNum = (tnow+_timezone*3600UL-SPARKTIMEBASESTART)/SPARKTIMESECPERDAY;
  uint32_t tempYear = SPARKTIMEBASEYEAR;
  uint8_t  tempMonth = 0;
  uint8_t tempHour = ((tnow+_timezone*3600UL) % 86400UL)/3600UL;

  while(dayNum >= YEARSIZE(tempYear)) {
    dayNum -= YEARSIZE(tempYear);
    tempYear++;
  }
  
  while(dayNum >= _monthLength[LEAPYEAR(tempYear)][tempMonth]) {
    dayNum -= _monthLength[LEAPYEAR(tempYear)][tempMonth];
    tempMonth++;
  }
  tempMonth++;
  dayNum++;			// correct for zero-base
  
  if (tempMonth>3 && tempMonth<11) {
    result = true;
  } else if (tempMonth == 3) {
    if ((dayNum == _usDSTStart[tempYear-SPARKTIMEBASEYEAR] && tempHour >=2) ||
	(dayNum >  _usDSTStart[tempYear-SPARKTIMEBASEYEAR])) {
      result = true;
    }      
  } else if (tempMonth == 11) {
    if (!((dayNum == _usDSTEnd[tempYear-SPARKTIMEBASEYEAR] && tempHour >=2) ||
	  (dayNum >  _usDSTEnd[tempYear-SPARKTIMEBASEYEAR]))) {
      result = true;
    }      
  }
  return result;
}

bool SparkTime::isEuroDST(uint32_t tnow) {
  // 1am last Sunday in March to 1am on last Sunday October
  // can't use offset here 
  bool result = false;
  uint32_t dayNum = (tnow+_timezone*3600UL-SPARKTIMEBASESTART)/SPARKTIMESECPERDAY;
  uint32_t tempYear = SPARKTIMEBASEYEAR;
  uint8_t  tempMonth = 0;
  uint8_t tempHour = ((tnow+_timezone*3600UL) % 86400UL)/3600UL;

  while(dayNum >= YEARSIZE(tempYear)) {
    dayNum -= YEARSIZE(tempYear);
    tempYear++;
  }
  
  while(dayNum >= _monthLength[LEAPYEAR(tempYear)][tempMonth]) {
    dayNum -= _monthLength[LEAPYEAR(tempYear)][tempMonth];
    tempMonth++;
  }
  tempMonth++;
  dayNum++;			// correct for zero-base
  
  if (tempMonth>3 && tempMonth<10) {
    result = true;
  } else if (tempMonth == 3) {
    if ((dayNum == _EuDSTStart[tempYear-SPARKTIMEBASEYEAR] && tempHour >=1) ||
	(dayNum >  _EuDSTStart[tempYear-SPARKTIMEBASEYEAR])) {
      result = true;
    }      
  } else if (tempMonth == 10) {
    if (!((dayNum == _EuDSTEnd[tempYear-SPARKTIMEBASEYEAR] && tempHour >=1) ||
	  (dayNum >  _EuDSTEnd[tempYear-SPARKTIMEBASEYEAR]))) {
      result = true;
    }      
  }
  return result;
}

void SparkTime::updateNTPTime() {
  //digitalWrite(D7,HIGH);
  _isSyncing = true;
  _UDPClient->begin(_localPort);
  memset(_packetBuffer, 0, SPARKTIMENTPSIZE); 
  _packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  _packetBuffer[1] = 0;     // Stratum, or type of clock
  _packetBuffer[2] = 6;     // Polling Interval
  _packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 4-11 are zero
  _packetBuffer[12]  = 49; 
  _packetBuffer[13]  = 0x4E;
  _packetBuffer[14]  = 49;
  _packetBuffer[15]  = 52;

  _UDPClient->beginPacket(_serverName, 123); //NTP requests are to port 123
  _UDPClient->write(_packetBuffer,SPARKTIMENTPSIZE);
  _UDPClient->endPacket(); 
  //gather the local offset close to the send time
  uint32_t localmsec = millis();
  int32_t retries = 0;
  int32_t bytesrecv = _UDPClient->parsePacket();
  while(bytesrecv == 0 && retries < 1000) {
    bytesrecv = _UDPClient->parsePacket();
    retries++;
  }

  if (bytesrecv>0) {
    _UDPClient->read(_packetBuffer,SPARKTIMENTPSIZE);
    // Handle Kiss-of-death
    if (_packetBuffer[1]==0) {
      _interval = max(_interval * 2, 24UL*60UL*60UL); 
    }
    _lastSyncNTPTime = _packetBuffer[40] << 24 | _packetBuffer[41] << 16 | _packetBuffer[42] << 8 | _packetBuffer[43];
    _lastSyncNTPFrac = _packetBuffer[44] << 24 | _packetBuffer[45] << 16 | _packetBuffer[46] << 8 | _packetBuffer[47];
    _lastSyncMillisTime = localmsec;
    _lastSyncMillisTime -= ((_lastSyncNTPFrac >> 10) * 1000) >> 22;	// (_lastSyncNTPFrac * 1000) >> 32
    _syncedOnce = true;
  } 
  _UDPClient->stop();  
  _isSyncing = false;
}

int32_t SparkTime::timeZoneDSTOffset(uint32_t tnow) {
  int32_t result = _timezone*3600UL;
  if ((_useDST && (!_useEuroDSTRule  && isUSDST(tnow))) ||
      (_useDST && (_useEuroDSTRule && isEuroDST(tnow)))) {
    result += 3600UL;
  } 
  return result;
}

////////////*************************new DS1807 stuff *************************///////////////
//////////////////////////////////////////////////////////////////////////////////////////////
// A library for handling real-time clocks, dates, etc.
// 2010-02-04 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
// 2012-11-08 RAM methods - idreammicro.com
// 2012-11-14 SQW/OUT methods - idreammicro.com
// 2012-01-12 DS1388 support
// 2013-08-29 ENERGIA MSP430 support

//#include <Wire.h>
//// Energia support
//#ifndef ENERGIA
//#include <avr/pgmspace.h>
//#else
//#define pgm_read_word(data) *data
//#define pgm_read_byte(data) *data
//#define PROGMEM
//#endif
//#include "RTClib.h"
//#include <Arduino.h>

#define DS1307_ADDRESS          0x68
#define DS1307_CONTROL_REGISTER 0x07
#define DS1307_RAM_REGISTER     0x08

// DS1307 Control register bits.
#define RTC_DS1307__RS0         0x00
#define RTC_DS1307__RS1         0x01
#define RTC_DS1307__SQWE        0x04
#define RTC_DS1307__OUT         0x07

// DS1388 Control register bits
#define DS1388_EEPROM_0         0x01
#define DS1388_EEPROM_1         0x02


#define PCF8563_ADDRESS         0x51
#define PCF8563_SEC_ADDR        0x02

#define BQ32000_ADDRESS         0x68
// BQ32000 register addresses:
#define BQ32000_CAL_CFG1        0x07
#define BQ32000_TCH2            0x08
#define BQ32000_CFG2            0x09
#define BQ32000_SFKEY1          0x20
#define BQ32000_SFKEY2          0x21
#define BQ32000_SFR             0x22
// BQ32000 config bits:
#define BQ32000__OUT            0x07 // CAL_CFG1 - IRQ active state
#define BQ32000__FT             0x06 // CAL_CFG1 - IRQ square wave enable
#define BQ32000__CAL_S          0x05 // CAL_CFG1 - Calibration sign
#define BQ32000__TCH2_BIT       0x05 // TCH2 - Trickle charger switch 2
#define BQ32000__TCFE           0x06 // CFG2 - Trickle FET control
// BQ32000 config values:
#define BQ32000_CHARGE_ENABLE   0x05 // CFG2 - Trickle charger switch 1 enable
#define BQ32000_SFKEY1_VAL      0x5E
#define BQ32000_SFKEY2_VAL      0xC7
#define BQ32000_FTF_1HZ         0x01
#define BQ32000_FTF_512HZ       0x00

#define SECONDS_PER_DAY         86400L

////////////////////////////////////////////////////////////////////////////////
// utility code, some of this could be exposed in the DateTime API if needed

static const uint8_t daysInMonth [] PROGMEM = {
  31,28,31,30,31,30,31,31,30,31,30,31
};

// number of days since 2000/01/01, valid for 2001..2099
static uint16_t date2days(uint16_t y, uint8_t m, uint8_t d) {
    if (y >= 2000)
        y -= 2000;
    uint16_t days = d;
    for (uint8_t i = 1; i < m; ++i)
        days += pgm_read_byte(daysInMonth + i - 1);
    if (m > 2 && y % 4 == 0)
        ++days;
    return days + 365 * y + (y + 3) / 4 - 1;
}

static long time2long(uint16_t days, uint8_t h, uint8_t m, uint8_t s) {
    return ((days * 24L + h) * 60 + m) * 60 + s;
}

////////////////////////////////////////////////////////////////////////////////
// DateTime implementation - ignores time zones and DST changes
// NOTE: also ignores leap seconds, see http://en.wikipedia.org/wiki/Leap_second

DateTime::DateTime (long t) {
    ss = t % 60;
    t /= 60;
    mm = t % 60;
    t /= 60;
    hh = t % 24;
    uint16_t days = t / 24;
    uint8_t leap;
    for (yOff = 0; ; ++yOff) {
        leap = yOff % 4 == 0;
        if (days < 365 + leap)
            break;
        days -= 365 + leap;
    }
    for (m = 1; ; ++m) {
        uint8_t daysPerMonth = pgm_read_byte(daysInMonth + m - 1);
        if (leap && m == 2)
            ++daysPerMonth;
        if (days < daysPerMonth)
            break;
        days -= daysPerMonth;
    }
    d = days + 1;
}

DateTime::DateTime (uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec) {
    if (year >= 2000)
        year -= 2000;
    yOff = year;
    m = month;
    d = day;
    hh = hour;
    mm = min;
    ss = sec;
}

static uint8_t conv2d(const char* p) {
    uint8_t v = 0;
    if ('0' <= *p && *p <= '9')
        v = *p - '0';
    return 10 * v + *++p - '0';
}

// A convenient constructor for using "the compiler's time":
//   DateTime now (__DATE__, __TIME__);
// NOTE: using PSTR would further reduce the RAM footprint
DateTime::DateTime (const char* date, const char* time) {
    // sample input: date = "Dec 26 2009", time = "12:34:56"
    yOff = conv2d(date + 9);
    // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
    switch (date[0]) {
        case 'J': m = date[1] == 'a' ? 1 : m = date[2] == 'n' ? 6 : 7; break;
        case 'F': m = 2; break;
        case 'A': m = date[2] == 'r' ? 4 : 8; break;
        case 'M': m = date[2] == 'r' ? 3 : 5; break;
        case 'S': m = 9; break;
        case 'O': m = 10; break;
        case 'N': m = 11; break;
        case 'D': m = 12; break;
    }
    d = conv2d(date + 4);
    hh = conv2d(time);
    mm = conv2d(time + 3);
    ss = conv2d(time + 6);
}

uint8_t DateTime::dayOfWeek() const {
    uint16_t day = get() / SECONDS_PER_DAY;
    return (day + 6) % 7; // Jan 1, 2000 is a Saturday, i.e. returns 6
}

long DateTime::get() const {
    uint16_t days = date2days(yOff, m, d);
    return time2long(days, hh, mm, ss);
}

////////////////////////////////////////////////////////////////////////////////
// RTC_DS1307 implementation

void RTC_DS1307::adjust(const DateTime& dt) {
    Wire.beginTransmission(DS1307_ADDRESS);
      Wire.write((byte) 0);
      Wire.write(bin2bcd(dt.second()));
      Wire.write(bin2bcd(dt.minute()));
      Wire.write(bin2bcd(dt.hour()));
      Wire.write(bin2bcd(0));
      Wire.write(bin2bcd(dt.day()));
      Wire.write(bin2bcd(dt.month()));
      Wire.write(bin2bcd(dt.year() - 2000));
      Wire.write((byte) 0);
    Wire.endTransmission();
}

DateTime RTC_DS1307::now() {
    Wire.beginTransmission(DS1307_ADDRESS);
      Wire.write((byte) 0);
    Wire.endTransmission();

    Wire.requestFrom(DS1307_ADDRESS, 7);
    uint8_t ss = bcd2bin(Wire.read());
    uint8_t mm = bcd2bin(Wire.read());
    uint8_t hh = bcd2bin(Wire.read());
    Wire.read();
    uint8_t d = bcd2bin(Wire.read());
    uint8_t m = bcd2bin(Wire.read());
    uint16_t y = bcd2bin(Wire.read()) + 2000;

    return DateTime (y, m, d, hh, mm, ss);
}

void RTC_DS1307::setSqwOutLevel(uint8_t level) {
    uint8_t value = (level == LOW) ? 0x00 : (1 << RTC_DS1307__OUT);
    Wire.beginTransmission(DS1307_ADDRESS);
      Wire.write(DS1307_CONTROL_REGISTER);
      Wire.write(value);
    Wire.endTransmission();
}

void RTC_DS1307::setSqwOutSignal(Frequencies frequency) {
    uint8_t value = (1 << RTC_DS1307__SQWE);
    switch (frequency)
    {
        case Frequency_1Hz:
            // Nothing to do.
        break;
        case Frequency_4096Hz:
            value |= (1 << RTC_DS1307__RS0);
        break;
        case Frequency_8192Hz:
            value |= (1 << RTC_DS1307__RS1);
        break;
        case Frequency_32768Hz:
        default:
            value |= (1 << RTC_DS1307__RS1) | (1 << RTC_DS1307__RS0);
        break;
    }
    Wire.beginTransmission(DS1307_ADDRESS);
      Wire.write(DS1307_CONTROL_REGISTER);
      Wire.write(value);
    Wire.endTransmission();
}

uint8_t RTC_DS1307::readByteInRam(uint8_t address) {
    Wire.beginTransmission(DS1307_ADDRESS);
      Wire.write(address);
    Wire.endTransmission();

    Wire.requestFrom(DS1307_ADDRESS, 1);
    uint8_t data = Wire.read();
    Wire.endTransmission();

    return data;
}

void RTC_DS1307::readBytesInRam(uint8_t address, uint8_t length, uint8_t* p_data) {
    Wire.beginTransmission(DS1307_ADDRESS);
      Wire.write(address);
    Wire.endTransmission();

    Wire.requestFrom(DS1307_ADDRESS, (int)length);
    for (uint8_t i = 0; i < length; i++) {
        p_data[i] = Wire.read();
    }
    Wire.endTransmission();
}

void RTC_DS1307::writeByteInRam(uint8_t address, uint8_t data) {
    Wire.beginTransmission(DS1307_ADDRESS);
      Wire.write(address);
      Wire.write(data);
    Wire.endTransmission();
}

void RTC_DS1307::writeBytesInRam(uint8_t address, uint8_t length, uint8_t* p_data) {
    Wire.beginTransmission(DS1307_ADDRESS);
      Wire.write(address);
      for (uint8_t i = 0; i < length; i++) {
             Wire.write(p_data[i]);
      }
    Wire.endTransmission();
}

uint8_t RTC_DS1307::isrunning(void) {
  Wire.beginTransmission(DS1307_ADDRESS);
    Wire.write((byte) 0);
  Wire.endTransmission();

  Wire.requestFrom(DS1307_ADDRESS, 1);
  uint8_t ss = Wire.read();
  return !(ss>>7);
}

////////////////////////////////////////////////////////////////////////////////
// DS 1388 implementation
uint8_t RTC_DS1388::WDSeconds = bin2bcd(60); //default to 60 seconds;
uint8_t RTC_DS1388::WDTSeconds = bin2bcd(0); //default to 60.00 seconds;


void RTC_DS1388::adjust(const DateTime& dt) {
  Wire.beginTransmission(DS1307_ADDRESS);
    Wire.write((byte) 0);
    Wire.write(bin2bcd(0)); // hundreds of seconds 0x00
    Wire.write(bin2bcd(dt.second())); // 0x01
    Wire.write(bin2bcd(dt.minute())); // 0x02
    Wire.write(bin2bcd(dt.hour())); // 0x03
    Wire.write(bin2bcd(0)); // 0x04
    Wire.write(bin2bcd(dt.day())); // 0x05
    Wire.write(bin2bcd(dt.month())); // 0x06
    Wire.write(bin2bcd(dt.year() - 2000)); // 0x07
  Wire.endTransmission();

  Wire.beginTransmission(DS1307_ADDRESS);
    Wire.write((byte) 0x0b);
    Wire.write((byte) 0x00);      //clear the 'time is invalid ' flag bit (OSF)
  Wire.endTransmission();
}

DateTime RTC_DS1388::now() {
  Wire.beginTransmission(DS1307_ADDRESS);
    Wire.write((byte) 0);
  Wire.endTransmission();

  Wire.requestFrom(DS1307_ADDRESS, 8);
  uint8_t hs = bcd2bin(Wire.read() & 0x7F);  // hundreds of seconds
  uint8_t ss = bcd2bin(Wire.read() & 0x7F);
  uint8_t mm = bcd2bin(Wire.read());
  uint8_t hh = bcd2bin(Wire.read());
  Wire.read();
  uint8_t d = bcd2bin(Wire.read());
  uint8_t m = bcd2bin(Wire.read());
  uint16_t y = bcd2bin(Wire.read()) + 2000;

  return DateTime (y, m, d, hh, mm, ss);
}

uint8_t RTC_DS1388::isrunning() {
  Wire.beginTransmission(DS1307_ADDRESS);
    Wire.write((byte)0x0b);
  Wire.endTransmission();

  Wire.requestFrom(DS1307_ADDRESS, 1);
  uint8_t ss = Wire.read();
  return !(ss>>7); //OSF flag bit
}

uint8_t RTC_DS1388::getEEPROMBank(uint16_t pos) {
  if(pos > 255){
    return DS1307_ADDRESS | DS1388_EEPROM_1;
  } else {
    return DS1307_ADDRESS | DS1388_EEPROM_0;
  }
}

/*
 * DS1388 has 512 bytes EEPROM in 2 banks of 256 bytes each
 */
void RTC_DS1388::EEPROMWrite(uint16_t pos, uint8_t c) {
  if(pos >= 512){
    return;
  }
  uint8_t rel_pos = pos % 256;
  // Set address
  Wire.beginTransmission(getEEPROMBank(pos));
    Wire.write((byte)rel_pos);
    // Wite data
    Wire.write((byte)c);
  Wire.endTransmission();
#ifdef ENERGIA
  delay(10); // Needed on MSP430 !!
#endif
}


uint8_t RTC_DS1388::EEPROMRead(uint16_t pos) {
  if(pos >= 512){
    return 0;
  }
  uint8_t rel_pos = pos % 256;
  Wire.beginTransmission(getEEPROMBank(pos));
  // Set address
  Wire.write((byte)rel_pos);
  Wire.endTransmission(true); // Stay open
  // Request one byte
  Wire.requestFrom(getEEPROMBank(pos), (uint8_t)1);
  uint8_t c = Wire.read();
#ifdef ENERGIA
  delay(10); // Needed on MSP430 !!
#endif
  return c;
}

/*
 * DS1388 has 512 bytes EEPROM in 2 banks of 256 bytes each.
 * EEPROM is arranged in 64 pages of 8 bytes each.
 * Page operations take a page number (0-63) and write/read 8 bytes
 */
void RTC_DS1388::EEPROMWritePage(uint8_t page, uint8_t *data) {
  if(page >= 64){
    return;
  }
  Wire.beginTransmission(getEEPROMBank((uint16_t)page * 8));
  uint8_t rel_pos =((uint16_t)page * 8) % 256;
  Wire.write((byte)rel_pos);
  for(uint8_t i=0; i<8; i++){
    Wire.write((byte)data[i]);
  }
  Wire.endTransmission();
#ifdef ENERGIA
  delay(10); // Needed on MSP430 !!
#endif
}

void RTC_DS1388::EEPROMReadPage(uint8_t page, uint8_t *data) {
  if(page >= 64){
    return;
  }
  Wire.beginTransmission(getEEPROMBank((uint16_t)page * 8));
  uint8_t rel_pos =((uint16_t)page * 8) % 256;
  // Set address
  Wire.write((byte)rel_pos);
  Wire.endTransmission(true); // Stay open
  // Request 8 byte
  Wire.requestFrom(getEEPROMBank((uint16_t)page * 8), (uint8_t)8);
  for(uint8_t i=0; i<8; i++){
    data[i] = Wire.read();
  }
  Wire.endTransmission();
#ifdef ENERGIA
  delay(10); // Needed on MSP430 !!
#endif
}

void RTC_DS1388::startWatchdogTimer(uint8_t Seconds, uint8_t TSeconds) {
  WDSeconds = bin2bcd(Seconds);
  WDTSeconds = bin2bcd(TSeconds);
  resetWatchdogTimer();
} 

void RTC_DS1388::resetWatchdogTimer() {
  //Disable the RTC watchdog first.
  Wire.beginTransmission(DS1307_ADDRESS);  
    Wire.write(0x0b);
    Wire.write(0x00); //clear WF bit
    Wire.write(0x00); //turn off WD
  Wire.endTransmission();

  //Set the watchdog timer to the desired time
  Wire.beginTransmission(DS1307_ADDRESS);
    Wire.write(0x08);
    Wire.write(WDTSeconds);  //08h    time 00-99 first nibble is Tenths of Seconds, second nibble is Hundredths of Seconds
    Wire.write(WDSeconds);  //09h - time 00-99 first nibble is Ten Seconds, second nibble is seconds
  Wire.endTransmission();

  //Enable the watchdog timer in the RTC.  0x0c -> 0x03 (WDE and WDE/RST)
  Wire.beginTransmission(DS1307_ADDRESS);
    Wire.write(0x0c);
    Wire.write(0x03);
  Wire.endTransmission();
}

///////////////////////////////////////////////////////////////////////////////
// RTC_PCF8563 implementation
// contributed by @mariusster, see http://forum.jeelabs.net/comment/1902

void RTC_PCF8563::adjust(const DateTime& dt) {
    Wire.beginTransmission(PCF8563_ADDRESS);
    Wire.write((byte) 0);
    Wire.write((byte) 0x0);                 // control/status1
    Wire.write((byte) 0x0);                 // control/status2
    Wire.write(bin2bcd(dt.second()));       // set seconds
    Wire.write(bin2bcd(dt.minute()));       // set minutes
    Wire.write(bin2bcd(dt.hour()));         // set hour
    Wire.write(bin2bcd(dt.day()));          // set day
    Wire.write((byte) 0x01);                // set weekday
    Wire.write(bin2bcd(dt.month()));        // set month, century to 1
    Wire.write(bin2bcd(dt.year() - 2000));  // set year to 00-99
    Wire.write((byte) 0x80);                // minute alarm value reset to 00
    Wire.write((byte) 0x80);                // hour alarm value reset to 00
    Wire.write((byte) 0x80);                // day alarm value reset to 00
    Wire.write((byte) 0x80);                // weekday alarm value reset to 00
    Wire.write((byte) 0x0);                 // set freqout 0= 32768khz, 1= 1hz
    Wire.write((byte) 0x0);                 // timer off
    Wire.endTransmission();
}

DateTime RTC_PCF8563::now() {
    Wire.beginTransmission(PCF8563_ADDRESS);
    Wire.write(PCF8563_SEC_ADDR);
    Wire.endTransmission();

    Wire.requestFrom(PCF8563_ADDRESS, 7);
    uint8_t ss = bcd2bin(Wire.read() & 0x7F);
    uint8_t mm = bcd2bin(Wire.read() & 0x7F);
    uint8_t hh = bcd2bin(Wire.read() & 0x3F);
    uint8_t d = bcd2bin(Wire.read() & 0x3F);
    Wire.read();
    uint8_t m = bcd2bin(Wire.read()& 0x1F);
    uint16_t y = bcd2bin(Wire.read()) + 2000;

    return DateTime (y, m, d, hh, mm, ss);
}


///////////////////////////////////////////////////////////////////////////////
// RTC_BQ32000 implementation

void RTC_BQ32000::adjust(const DateTime& dt) {
    Wire.beginTransmission(BQ32000_ADDRESS);
    Wire.write((byte) 0);
    Wire.write(bin2bcd(dt.second()));
    Wire.write(bin2bcd(dt.minute()));
    Wire.write(bin2bcd(dt.hour()));
    Wire.write(bin2bcd(0));
    Wire.write(bin2bcd(dt.day()));
    Wire.write(bin2bcd(dt.month()));
    Wire.write(bin2bcd(dt.year() - 2000));
    Wire.endTransmission();
}

DateTime RTC_BQ32000::now() {
    Wire.beginTransmission(BQ32000_ADDRESS);
    Wire.write((byte) 0);
    Wire.endTransmission();

    Wire.requestFrom(DS1307_ADDRESS, 7);
    uint8_t ss = bcd2bin(Wire.read());
    uint8_t mm = bcd2bin(Wire.read());
    uint8_t hh = bcd2bin(Wire.read());
    Wire.read();
    uint8_t d = bcd2bin(Wire.read());
    uint8_t m = bcd2bin(Wire.read());
    uint16_t y = bcd2bin(Wire.read()) + 2000;

    return DateTime (y, m, d, hh, mm, ss);
}

void RTC_BQ32000::setIRQ(uint8_t state) {
    /* Set IRQ square wave output state: 0=disabled, 1=1Hz, 2=512Hz.
     */
  uint8_t reg, value;
    if (state) {
      // Setting the frequency is a bit complicated on the BQ32000:
      Wire.beginTransmission(BQ32000_ADDRESS);
      Wire.write(BQ32000_SFKEY1);
      Wire.write(BQ32000_SFKEY1_VAL);
      Wire.write(BQ32000_SFKEY2_VAL);
      Wire.write((state == 1) ? BQ32000_FTF_1HZ : BQ32000_FTF_512HZ);
      Wire.endTransmission();
    }
    value = readRegister(BQ32000_CAL_CFG1);
    value = (!state) ? value & ~(1<<BQ32000__FT) : value | (1<<BQ32000__FT);
    writeRegister(BQ32000_CAL_CFG1, value);
}

void RTC_BQ32000::setIRQLevel(uint8_t level) {
    /* Set IRQ output level when IRQ square wave output is disabled to
     * LOW or HIGH.
     */
    uint8_t value;
    // The IRQ active level bit is in the same register as the calibration
    // settings, so we preserve its current state:
    value = readRegister(BQ32000_CAL_CFG1);
    value = (!level) ? value & ~(1<<BQ32000__OUT) : value | (1<<BQ32000__OUT);
    writeRegister(BQ32000_CAL_CFG1, value);
}

void RTC_BQ32000::setCalibration(int8_t value) {
    /* Sets the calibration value to given value in the range -31 - 31, which
     * corresponds to -126ppm - +63ppm; see table 13 in th BQ32000 datasheet.
     */
    uint8_t val;
    if (value > 31) value = 31;
    if (value < -31) value = -31;
    val = (uint8_t) (value < 0) ? -value | (1<<BQ32000__CAL_S) : value;
    val |= readRegister(BQ32000_CAL_CFG1) & ~0x3f;
    writeRegister(BQ32000_CAL_CFG1, val);
}

void RTC_BQ32000::setCharger(int state) {
    /* If using a super capacitor instead of a battery for backup power, use this
     * method to set the state of the trickle charger: 0=disabled, 1=low-voltage
     * charge, 2=high-voltage charge. In low-voltage charge mode, the super cap is
     * charged through a diode with a voltage drop of about 0.5V, so it will charge
     * up to VCC-0.5V. In high-voltage charge mode the diode is bypassed and the super
     * cap will be charged up to VCC (make sure the charge voltage does not exceed your
     * super cap's voltage rating!!).
     */
    // First disable charger regardless of state (prevents it from
    // possible starting up in the high voltage mode when the low
    // voltage mode is requested):
    uint8_t value;
    writeRegister(BQ32000_TCH2, 0);
    if (state <= 0 || state > 2) return;
    value = BQ32000_CHARGE_ENABLE;
    if (state == 2) {
        // High voltage charge enable:
        value |= (1 << BQ32000__TCFE);
    }
    writeRegister(BQ32000_CFG2, value);
    // Now enable charger:
    writeRegister(BQ32000_TCH2, 1 << BQ32000__TCH2_BIT);
}


uint8_t RTC_BQ32000::readRegister(uint8_t address) {
    /* Read and return the value in the register at the given address.
     */
    Wire.beginTransmission(BQ32000_ADDRESS);
    Wire.write((byte) address);
    Wire.endTransmission();
    Wire.requestFrom(DS1307_ADDRESS, 1);
    // Get register state:
    return Wire.read();
}

uint8_t RTC_BQ32000::writeRegister(uint8_t address, uint8_t value) {
    /* Write the given value to the register at the given address.
     */
    Wire.beginTransmission(BQ32000_ADDRESS);
    Wire.write(address);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t RTC_BQ32000::isrunning() {
    return !(readRegister(0x0)>>7);
}


////////////////////////////////////////////////////////////////////////////////
// RTC_Millis implementation

long RTC_Millis::offset = 0;

void RTC_Millis::adjust(const DateTime& dt) {
    offset = dt.get() - millis() / 1000;
}

DateTime RTC_Millis::now() {
    return offset + millis() / 1000;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


// 8/22/15 Add HMC5883L
/*
HMC5883L.cpp - Class file for the HMC5883L Triple Axis Digital Compass Arduino Library.

Version: 1.1.0
(c) 2014 Korneliusz Jarzebski
www.jarzebski.pl

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

//#if ARDUINO >= 100
//#include "Arduino.h"
//#else
//#include "WProgram.h"
//#endif
//
//#include <Wire.h>
//
//#include "HMC5883L.h"

#define HMC5883L_ADDRESS              (0x1E)
#define HMC5883L_REG_CONFIG_A         (0x00)
#define HMC5883L_REG_CONFIG_B         (0x01)
#define HMC5883L_REG_MODE             (0x02)
#define HMC5883L_REG_OUT_X_M          (0x03)
#define HMC5883L_REG_OUT_X_L          (0x04)
#define HMC5883L_REG_OUT_Z_M          (0x05)
#define HMC5883L_REG_OUT_Z_L          (0x06)
#define HMC5883L_REG_OUT_Y_M          (0x07)
#define HMC5883L_REG_OUT_Y_L          (0x08)
#define HMC5883L_REG_STATUS           (0x09)
#define HMC5883L_REG_IDENT_A          (0x0A)
#define HMC5883L_REG_IDENT_B          (0x0B)
#define HMC5883L_REG_IDENT_C          (0x0C)

bool HMC5883L::begin()
{
    Wire.begin();

    if ((fastRegister8(HMC5883L_REG_IDENT_A) != 0x48)
    || (fastRegister8(HMC5883L_REG_IDENT_B) != 0x34)
    || (fastRegister8(HMC5883L_REG_IDENT_C) != 0x33))
    {
	return false;
    }

    setRange(HMC5883L_RANGE_1_3GA);
    setMeasurementMode(HMC5883L_CONTINOUS);
    setDataRate(HMC5883L_DATARATE_15HZ);
    setSamples(HMC5883L_SAMPLES_1);

    mgPerDigit = 0.92f;

    return true;
}

Vector HMC5883L::readRaw(void)
{
    v.XAxis = readRegister16(HMC5883L_REG_OUT_X_M) - xOffset;
    v.YAxis = readRegister16(HMC5883L_REG_OUT_Y_M) - yOffset;
    v.ZAxis = readRegister16(HMC5883L_REG_OUT_Z_M);

    return v;
}

Vector HMC5883L::readNormalize(void)
{
    v.XAxis = ((float)readRegister16(HMC5883L_REG_OUT_X_M) - xOffset) * mgPerDigit;
    v.YAxis = ((float)readRegister16(HMC5883L_REG_OUT_Y_M) - yOffset) * mgPerDigit;
    v.ZAxis = (float)readRegister16(HMC5883L_REG_OUT_Z_M) * mgPerDigit;

    return v;
}

void HMC5883L::setOffset(int xo, int yo)
{
    xOffset = xo;
    yOffset = yo;
}

void HMC5883L::setRange(hmc5883l_range_t range)
{
    switch(range)
    {
	case HMC5883L_RANGE_0_88GA:
	    mgPerDigit = 0.073f;
	    break;

	case HMC5883L_RANGE_1_3GA:
	    mgPerDigit = 0.92f;
	    break;

	case HMC5883L_RANGE_1_9GA:
	    mgPerDigit = 1.22f;
	    break;

	case HMC5883L_RANGE_2_5GA:
	    mgPerDigit = 1.52f;
	    break;

	case HMC5883L_RANGE_4GA:
	    mgPerDigit = 2.27f;
	    break;

	case HMC5883L_RANGE_4_7GA:
	    mgPerDigit = 2.56f;
	    break;

	case HMC5883L_RANGE_5_6GA:
	    mgPerDigit = 3.03f;
	    break;

	case HMC5883L_RANGE_8_1GA:
	    mgPerDigit = 4.35f;
	    break;

	default:
	    break;
    }

    writeRegister8(HMC5883L_REG_CONFIG_B, range << 5);
}

hmc5883l_range_t HMC5883L::getRange(void)
{
    return (hmc5883l_range_t)((readRegister8(HMC5883L_REG_CONFIG_B) >> 5));
}

void HMC5883L::setMeasurementMode(hmc5883l_mode_t mode)
{
    uint8_t value;

    value = readRegister8(HMC5883L_REG_MODE);
    value &= 0b11111100;
    value |= mode;

    writeRegister8(HMC5883L_REG_MODE, value);
}

hmc5883l_mode_t HMC5883L::getMeasurementMode(void)
{
    uint8_t value;

    value = readRegister8(HMC5883L_REG_MODE);
    value &= 0b00000011;

    return (hmc5883l_mode_t)value;
}

void HMC5883L::setDataRate(hmc5883l_dataRate_t dataRate)
{
    uint8_t value;

    value = readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b11100011;
    value |= (dataRate << 2);

    writeRegister8(HMC5883L_REG_CONFIG_A, value);
}

hmc5883l_dataRate_t HMC5883L::getDataRate(void)
{
    uint8_t value;

    value = readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b00011100;
    value >>= 2;

    return (hmc5883l_dataRate_t)value;
}

void HMC5883L::setSamples(hmc5883l_samples_t samples)
{
    uint8_t value;

    value = readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b10011111;
    value |= (samples << 5);

    writeRegister8(HMC5883L_REG_CONFIG_A, value);
}

hmc5883l_samples_t HMC5883L::getSamples(void)
{
    uint8_t value;

    value = readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b01100000;
    value >>= 5;

    return (hmc5883l_samples_t)value;
}

// Write byte to register
void HMC5883L::writeRegister8(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

// Read byte to register
uint8_t HMC5883L::fastRegister8(uint8_t reg)
{
    uint8_t value;
    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom(HMC5883L_ADDRESS, 1);
    value = Wire.read();
    Wire.endTransmission();

    return value;
}

// Read byte from register
uint8_t HMC5883L::readRegister8(uint8_t reg)
{
    uint8_t value;
    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.requestFrom(HMC5883L_ADDRESS, 1);
    while(!Wire.available()) {};
    value = Wire.read();
    Wire.endTransmission();

    return value;
}

// Read word from register
int16_t HMC5883L::readRegister16(uint8_t reg)
{
    int16_t value;
    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.requestFrom(HMC5883L_ADDRESS, 2);
    while(!Wire.available()) {};
    uint8_t vha = Wire.read();
    uint8_t vla = Wire.read();
    Wire.endTransmission();

    value = vha << 8 | vla;

    return value;
}

