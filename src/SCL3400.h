/******************************************************************************
SCL3400.h
SCL3400 Arduino Driver Header File
Modified by Vasutorn Siriyakorn, 2024
This file has been modified to support the SCL3400 inclinometer.
The original functionality for the SCL3300 has been adapted to work with the SCL3400 sensor.
Version 0.1.0 - September 12, 2024
https://github.com/vasutornjays/SCL3400

Original SCL3300.cpp by David Armstrong
David Armstrong
Version 3.3.0 - September 13, 2021
https://github.com/DavidArmstrong/SCL3400

Resources:
Uses SPI.h for SPI operation

Development environment specifics:
Arduino IDE 2.3.2

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example.
Distributed as-is; no warranty is given.

******************************************************************************/

// ensure this library description is only included once
#ifndef __SCL3400_h
#define __SCL3400_h

// Uncomment the following line for debugging output
//#define debug_scl3400

// Need the following define for SAMD processors
#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  #define Serial_SCL SERIAL_PORT_USBVIRTUAL
#else
  #define Serial_SCL Serial
#endif

#include <stdint.h>

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <SPI.h>  // SPI library is used for...SPI.
#include <cmath>

#ifndef SCL3400_SPI_CLOCK
#ifdef ARDUINO_ARCH_ESP32
#define SCL3400_SPI_CLOCK 100000
#else
#define SCL3400_SPI_CLOCK 100000
#endif
#endif

#ifndef SCL3400_SPI_MODE
#define SCL3400_SPI_MODE SPI_MODE0
#endif

//Define allowed commands to SCL3400 inclinometer
#define RdAccX		0x040000f7
#define RdAccY		0x080000fd
#define RdTemp		0x140000ef
#define RdStatSum	0x180000e5
#define RdErrFlg1	0x1c0000e3
#define RdErrFlg2	0x200000c1
#define RdCMD		  0x340000df
#define ChgModeA	0xb400001f
#define ChgModeB	0xb4000338
#define SetPwrDwn	0xb400046b
#define WakeUp		0xb400001f
#define SWreset		0xb4002098
#define RdWHOAMI	0x40000091
#define RdSer1		0x640000a7
#define RdSer2		0x680000AD
#define RdCurBank	0x7c0000b3
#define SwtchBnk0	0xfc000073
#define SwtchBnk1	0xfc00016e

#define RAD_TO_DEG  (180.0 / M_PI)  // Conversion factor from radians to degrees

// Structure to hold raw sensor data
// We need to populate all this every time we read a set of data
struct SCL3400data {
  public:
    int16_t AccX;
    int16_t AccY;
    int16_t TEMP;
    uint16_t StatusSum;
    uint16_t WHOAMI;
};

// SCL3400 library interface description
class SCL3400 {
  // user-accessible "public" interface
  public:
    SPISettings spiSettings{SCL3400_SPI_CLOCK, MSBFIRST, SCL3400_SPI_MODE};
	
    SCL3400data sclData;
    boolean setMode(int mode);
    boolean begin(void);
    boolean begin(uint8_t csPin);
    boolean begin(SPIClass &spiPort, uint8_t csPin);
    //Functions to retrieve sensor data
    boolean isConnected();
    boolean available(void);
    void setFastReadMode();
    void stopFastReadMode();
    double getcalculateTiltX(void);
    double getcalculateTiltY(void);
    double getCalculatedAccelerometerX(void);
    double getCalculatedAccelerometerY(void);
    uint16_t getErrFlag1(void);
    uint16_t getErrFlag2(void);
    unsigned long getSerialNumber(void);
    double getCalculatedTemperatureCelsius(void);
    double getCalculatedTemperatureFarenheit(void);
    double acceleration(int16_t SCL3400_ACC);
    bool crcerr, statuserr;
    uint16_t powerDownMode(void);
    uint16_t WakeMeUp(void);
    uint16_t reset(void);
	
  // library-accessible "private" interface
  private:
    SPIClass *_spiPort = NULL;  //The generic connection to user's chosen spi hardware

    uint8_t scl3400_csPin = 5; // Default SPI chip select pin
    uint8_t scl3400_mode = 4; // Default inclinometer mode
    uint8_t SCL3400_CMD, SCL3400_CRC;
    uint16_t SCL3400_DATA;

    double Temperature, X_angle, Y_angle;
    bool setFastRead = false;
	
    void initSPI();
    void beginTransmission();
    void endTransmission();
    uint8_t CalculateCRC(uint32_t Data);
    uint8_t CRC8(uint8_t BitValue, uint8_t SCL3400_CRC);
    unsigned long transfer(unsigned long value);

    union FourByte {
      unsigned long bit32;
      unsigned int bit16[2];
      unsigned char bit8[4];
    };
    unsigned long modeCMD[4]  = { 0, ChgModeA, 0, ChgModeB};
};
#endif
