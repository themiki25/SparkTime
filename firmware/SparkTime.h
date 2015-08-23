#ifndef _SPARKTIME
#define _SPARKTIME

#include "application.h"

#define SPARKTIMENTPSIZE 48
#define SPARKTIMEHOSTNAMESIZE 64
#define SPARKTIMEYEARZERO 1900
#define SPARKTIMEEPOCHSTART 2208988800UL
#define SPARKTIMESECPERDAY 86400UL
#define SPARKTIMEBASESTART 3597523200UL
#define SPARKTIMEBASEYEAR  2014UL
#define LEAPYEAR(year)  (!((year) % 4) && (((year) % 100) || !((year) % 400)))
#define YEARSIZE(year)  (LEAPYEAR(year) ? 366 : 365)
#define SPARKTIMEWRAPSECS 4294967UL

static const char _digits[60][3] = {"00","01","02","03","04","05","06","07","08","09",
			       "10","11","12","13","14","15","16","17","18","19",
			       "20","21","22","23","24","25","26","27","28","29",
			       "30","31","32","33","34","35","36","37","38","39",
			       "40","41","42","43","44","45","46","47","48","49",
			       "50","51","52","53","54","55","56","57","58","59"}; /* for speed! */
static const char _days[7][10] = {"Sunday","Monday","Tuesday","Wednesday","Thursday","Friday","Saturday"};
static const char _days_short[7][4] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
static const char _months[12][10] = {"January","February","March","April","May","June","July","August","September","October","November","December"};
static const char _months_short[12][4] = {"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
static const uint8_t _monthLength[2][12] = {
    {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},
    {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}}; // Leap year
  // for years 2014-2036 Day in March or November
  //                             2014 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36
static const uint8_t _usDSTStart[23] = { 9, 8,13,12,11,10, 8,14,13,12,10, 9, 8,14,12,11,10, 9,14,13,12,11, 9};
static const uint8_t _usDSTEnd[23]   = { 2, 1, 6, 5, 4, 3, 1, 7, 6, 5, 3, 2, 1, 7, 5, 4, 3, 2, 7, 6, 5, 4, 2};
static const uint8_t _EuDSTStart[23] = {30,29,27,26,25,31,29,28,27,26,31,30,29,28,26,25,31,30,28,27,26,25,30};
static const uint8_t _EuDSTEnd[23]   = {26,25,30,29,28,27,25,31,30,29,27,26,25,31,29,28,27,26,31,30,29,28,26};

class SparkTime {
public:
  SparkTime();
  void begin(UDP * UPDClient);
  void begin(UDP * UPDClient, const char * NTPServer);
  bool hasSynced();
  void setNTPInvterval(uint32_t intervalMinutes);
  void setTimeZone(int32_t hoursOffset);
  void setUseDST(bool value);
  void setUseEuroDSTRule(bool value);

  uint32_t now();
  uint32_t nowNoUpdate();
  uint32_t nowEpoch();
  uint32_t lastNTPTime();

  uint8_t hour(uint32_t tnow);
  uint8_t minute(uint32_t tnow);
  uint8_t second(uint32_t tnow);
  uint8_t dayOfWeek(uint32_t tnow);
  uint8_t day(uint32_t tnow);
  uint8_t month(uint32_t tnow);
  uint32_t year(uint32_t tnow);
  
  bool isUSDST(uint32_t tnow); //2nd Sun in Mar to 1st Sun in Nov
  bool isEuroDST(uint32_t tnow); //Last Sun in Mar to last Sun in Oct

  String hourString(uint32_t tnow);
  String hour12String(uint32_t tnow);
  String minuteString(uint32_t tnow);
  String secondString(uint32_t tnow);
  String AMPMString(uint32_t tnow);
  String dayOfWeekShortString(uint32_t tnow);
  String dayOfWeekString(uint32_t tnow);
  String dayString(uint32_t tnow);
  String monthString(uint32_t tnow);
  String monthNameShortString(uint32_t tnow);
  String monthNameString(uint32_t tnow);
  String yearShortString(uint32_t tnow);
  String yearString(uint32_t tnow);
  String ISODateString(uint32_t tnow);
  String ISODateUTCString(uint32_t tnow);
 
private:
  UDP * _UDPClient;
  char _serverName[SPARKTIMEHOSTNAMESIZE];
  bool _syncedOnce = false;
  bool _isSyncing = false;
  uint32_t _lastSyncMillisTime;
  uint32_t _lastSyncNTPTime;
  uint32_t _lastSyncNTPFrac;
  uint32_t _interval;
  int32_t _timezone;
  bool _useDST;
  bool _useEuroDSTRule;
  uint32_t _localPort;
  uint8_t _packetBuffer[SPARKTIMENTPSIZE];
  void updateNTPTime();
  int32_t timeZoneDSTOffset(uint32_t tnow);
};


/********************************************************************************/
// A library for handling real-time clocks, dates, etc.
// 2010-02-04 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
// 2012-11-08 RAM methods - idreammicro.com
// 2012-11-14 SQW/OUT methods - idreammicro.com
// Simple general-purpose date/time class (no TZ / DST / leap second handling!)
class DateTime {
public:
   DateTime (long t =0);
   DateTime (uint16_t year, uint8_t month, uint8_t day, uint8_t hour =0, uint8_t min =0, uint8_t sec =0);
   DateTime (const char* date, const char* time);
   uint16_t year() const { return 2000 + yOff; }
   uint8_t month() const { return m; }
   uint8_t day() const { return d; }
   uint8_t hour() const { return hh; }
   uint8_t minute() const { return mm; }
   uint8_t second() const { return ss; }
   uint8_t dayOfWeek() const;
   
   // 32-bit times as seconds since 1/1/2000
   long get() const;
protected:
   uint8_t yOff, m, d, hh, mm, ss;
};
// RTC based on the DS1307 chip connected via I2C and the Wire library
class RTC_DS1307 {
public:

   // SQW/OUT frequencies.
   enum Frequencies
   {
       Frequency_1Hz,
       Frequency_4096Hz,
       Frequency_8192Hz,
       Frequency_32768Hz
   };
   
   static void begin() {}
   static void adjust(const DateTime& dt);
   static DateTime now();
   static uint8_t isrunning();
   
   // SQW/OUT functions.
   void setSqwOutLevel(uint8_t level);
   void setSqwOutSignal(Frequencies frequency);
   
   // RAM registers read/write functions. Address locations 08h to 3Fh.
   // Max length = 56 bytes.
   static uint8_t readByteInRam(uint8_t address);
   static void readBytesInRam(uint8_t address, uint8_t length, uint8_t* p_data);
   static void writeByteInRam(uint8_t address, uint8_t data);
   static void writeBytesInRam(uint8_t address, uint8_t length, uint8_t* p_data);
   
   // utility functions
   static uint8_t bcd2bin (uint8_t val) { return val - 6 * (val >> 4); }
   static uint8_t bin2bcd (uint8_t val) { return val + 6 * (val / 10); }
};

// DS1388 version
class RTC_DS1388 {
protected:
   static uint8_t WDSeconds;
   static uint8_t WDTSeconds;
public:
   static void begin() {};
   static void adjust(const DateTime& dt);
   static DateTime now();
   static uint8_t isrunning();
   
   // EEPROM
   static uint8_t getEEPROMBank(uint16_t pos);
   static void EEPROMWrite(uint16_t pos, uint8_t c);
   static uint8_t EEPROMRead(uint16_t pos);
   static void EEPROMWritePage(uint8_t page, uint8_t* data);
   static void EEPROMReadPage(uint8_t page, uint8_t* buffer);
   
   //Watchdog
   static void startWatchdogTimer(uint8_t Seconds, uint8_t TSeconds);
   static void resetWatchdogTimer();

   // utility functions
   static uint8_t bcd2bin (uint8_t val) { return val - 6 * (val >> 4); }
   static uint8_t bin2bcd (uint8_t val) { return val + 6 * (val / 10); }

};

// RTC based on the PCF8563 chip connected via I2C and the Wire library
// contributed by @mariusster, see http://forum.jeelabs.net/comment/1902
class RTC_PCF8563 {
public:
    static void begin() {}
    static void adjust(const DateTime& dt);
    static DateTime now();
    // utility functions
    static uint8_t bcd2bin (uint8_t val) { return val - 6 * (val >> 4); }
    static uint8_t bin2bcd (uint8_t val) { return val + 6 * (val / 10); }
};

// TI BQ32000 I2C RTC
class RTC_BQ32000 {
public:
    static void begin() {}
    static void adjust(const DateTime& dt);
    static DateTime now();
    static uint8_t isrunning();
    static void setIRQ(uint8_t state);
    /* Set IRQ output state: 0=disabled, 1=1Hz, 2=512Hz.
    */
    static void setIRQLevel(uint8_t level);
    /* Set IRQ output active state to LOW or HIGH.
    */
    static void setCalibration(int8_t value);
    /* Sets the calibration value to given value in the range -31 - 31, which
    * corresponds to -126ppm - +63ppm; see table 13 in th BQ32000 datasheet.
    */
    static void setCharger(int state);
    /* If using a super capacitor instead of a battery for backup power, use this
        method to set the state of the trickle charger: 0=disabled, 1=low-voltage
        charge, 2=high-voltage charge. In low-voltage charge mode, the super cap is
        charged through a diode with a voltage drop of about 0.5V, so it will charge
        up to VCC-0.5V. In high-voltage charge mode the diode is bypassed and the super
        cap will be charged up to VCC (make sure the charge voltage does not exceed your
        super cap's voltage rating!!). */
        
    // utility functions:
    static uint8_t readRegister(uint8_t address);
    static uint8_t writeRegister(uint8_t address, uint8_t value);
    static uint8_t bcd2bin (uint8_t val) { return val - 6 * (val >> 4); }
    static uint8_t bin2bcd (uint8_t val) { return val + 6 * (val / 10); }
};
// RTC using the internal millis() clock, has to be initialized before use
// NOTE: this clock won't be correct once the millis() timer rolls over (>49d?)
class RTC_Millis {
public:
    static void begin(const DateTime& dt) { adjust(dt); }
    static void adjust(const DateTime& dt);
    static DateTime now();
protected:
    static long offset;
};


// Added 8/22/15 themik25

/*
HMC5883L.h - Header file for the HMC5883L Triple Axis Digital Compass Arduino Library.

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

//#ifndef HMC5883L_h
//#define HMC5883L_h
//
//#if ARDUINO >= 100
//#include "Arduino.h"
//#else
//#include "WProgram.h"
//#endif
//
//#define HMC5883L_ADDRESS              (0x1E)
//#define HMC5883L_REG_CONFIG_A         (0x00)
//#define HMC5883L_REG_CONFIG_B         (0x01)
//#define HMC5883L_REG_MODE             (0x02)
//#define HMC5883L_REG_OUT_X_M          (0x03)
//#define HMC5883L_REG_OUT_X_L          (0x04)
//#define HMC5883L_REG_OUT_Z_M          (0x05)
//#define HMC5883L_REG_OUT_Z_L          (0x06)
//#define HMC5883L_REG_OUT_Y_M          (0x07)
//#define HMC5883L_REG_OUT_Y_L          (0x08)
//#define HMC5883L_REG_STATUS           (0x09)
//#define HMC5883L_REG_IDENT_A          (0x0A)
//#define HMC5883L_REG_IDENT_B          (0x0B)
//#define HMC5883L_REG_IDENT_C          (0x0C)

typedef enum
{
    HMC5883L_SAMPLES_8     = 0b11,
    HMC5883L_SAMPLES_4     = 0b10,
    HMC5883L_SAMPLES_2     = 0b01,
    HMC5883L_SAMPLES_1     = 0b00
} hmc5883l_samples_t;

typedef enum
{
    HMC5883L_DATARATE_75HZ       = 0b110,
    HMC5883L_DATARATE_30HZ       = 0b101,
    HMC5883L_DATARATE_15HZ       = 0b100,
    HMC5883L_DATARATE_7_5HZ      = 0b011,
    HMC5883L_DATARATE_3HZ        = 0b010,
    HMC5883L_DATARATE_1_5HZ      = 0b001,
    HMC5883L_DATARATE_0_75_HZ    = 0b000
} hmc5883l_dataRate_t;

typedef enum
{
    HMC5883L_RANGE_8_1GA     = 0b111,
    HMC5883L_RANGE_5_6GA     = 0b110,
    HMC5883L_RANGE_4_7GA     = 0b101,
    HMC5883L_RANGE_4GA       = 0b100,
    HMC5883L_RANGE_2_5GA     = 0b011,
    HMC5883L_RANGE_1_9GA     = 0b010,
    HMC5883L_RANGE_1_3GA     = 0b001,
    HMC5883L_RANGE_0_88GA    = 0b000
} hmc5883l_range_t;

typedef enum
{
    HMC5883L_IDLE          = 0b10,
    HMC5883L_SINGLE        = 0b01,
    HMC5883L_CONTINOUS     = 0b00
} hmc5883l_mode_t;

#ifndef VECTOR_STRUCT_H
#define VECTOR_STRUCT_H
struct Vector
{
    float XAxis;
    float YAxis;
    float ZAxis;
};
#endif

class HMC5883L
{
    public:

	bool begin(void);

	Vector readRaw(void);
	Vector readNormalize(void);

	void  setOffset(int xo, int yo);

	void  setRange(hmc5883l_range_t range);
	hmc5883l_range_t getRange(void);

	void  setMeasurementMode(hmc5883l_mode_t mode);
	hmc5883l_mode_t getMeasurementMode(void);

	void  setDataRate(hmc5883l_dataRate_t dataRate);
	hmc5883l_dataRate_t getDataRate(void);

	void  setSamples(hmc5883l_samples_t samples);
	hmc5883l_samples_t getSamples(void);

    private:

	float mgPerDigit;
	Vector v;
	int xOffset, yOffset;

	void writeRegister8(uint8_t reg, uint8_t value);
	uint8_t readRegister8(uint8_t reg);
	uint8_t fastRegister8(uint8_t reg);
	int16_t readRegister16(uint8_t reg);
};

//#endif


#endif
