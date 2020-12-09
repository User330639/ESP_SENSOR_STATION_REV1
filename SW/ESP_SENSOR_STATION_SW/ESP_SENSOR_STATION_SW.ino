#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include "webPage.h"
#include "AD7792.h"
#include "SHT1x.h"

/* ESP creates WiFi network with this SSID password combination */
#define _SSID "TEST"
#define _PASS "12345678"
/* ------------------------------------------------------------ */

/* STH1x driver uses GPIO for communication; these definitions  */
/* denote GPIO pins used for that purpose                       */
#define SHT1x_dataPin  5
#define SHT1x_clockPin 4

/* Macros for lesser verbosity; corresponidng methods return    */
/* float values                                                 */
#define GET_TMP        (SHT1x.readTemperature(TempUnit::C))
#define GET_HUM        (SHT1x.readHumidity())
/* ------------------------------------------------------------ */

/* Values to be obtained during calibration                     */
/* Calibration values are arbitary                              */
#define PRS_CAL_CODE_V1        (float)0
#define PRS_CAL_CODE_V2        (float)0
#define PRS_CAL_V1             (float)0
#define PRS_CAL_V_DELTA        (float)0
#define PRS_UNITS              "kPa"

#define DUST_CAL_CODE_V1       (float)0
#define DUST_CAL_CODE_V2       (float)0
#define DUST_CAL_V1            (float)0
#define DUST_CAL_V_DELTA       (float)0
#define DUST_UNITS             "ug/m3"

#define C3H8_CAL_CODE_V1       (float)0
#define C3H8_CAL_CODE_V2       (float)0
#define C3H8_CAL_V1            (float)0
#define C3H8_CAL_V_DELTA       (float)0
#define C3H8_UNITS             "ppm"
/* ------------------------------------------------------------ */

/* Global objects and variables                                 */
// The server hosts a webpage port #80; see webPage.h and webPage.cpp
ESP8266WebServer server(80);
// Global SHT1x object; the constructor requires GPIO numbers used for communication
SHT1x SHT1x(SHT1x_dataPin, SHT1x_clockPin);
// Global variables that indicate presence or absence of AD7792/SHT1x
boolean _ad7792_present = false;
boolean _sht10_present = false;
/* ------------------------------------------------------------ */

/* Returns the value of pressure in PRS_UNITS units             */
/* The function requires valid definitions to work properly     */
float GET_PRS()
{
  AD7792_SetChannel(AD7792_CH_AIN1P_AIN1M);
  unsigned long code = AD7792_SingleConversion();
  return
    ((float)code - PRS_CAL_CODE_V1) / (PRS_CAL_CODE_V2 - PRS_CAL_CODE_V1) * PRS_CAL_V_DELTA + PRS_CAL_V1;
}
/* ------------------------------------------------------------ */

/* Same as GET_PRS() but returns dust concentration             */
float GET_DUST()
{
  AD7792_SetChannel(AD7792_CH_AIN2P_AIN2M);
  unsigned long code = AD7792_SingleConversion();
  return
    ((float)code - DUST_CAL_CODE_V1) / (DUST_CAL_CODE_V2 - DUST_CAL_CODE_V1) * DUST_CAL_V_DELTA + DUST_CAL_V1;
}
/* ------------------------------------------------------------ */

/* Same as GET_PRS() but returns C3H8 concentration             */
float GET_C3H8()
{
  AD7792_SetChannel(AD7792_CH_AIN3P_AIN3M);
  unsigned long code = AD7792_SingleConversion();
  return
    ((float)code - C3H8_CAL_CODE_V1) / (C3H8_CAL_CODE_V2 - C3H8_CAL_CODE_V1) * C3H8_CAL_V_DELTA + C3H8_CAL_V1;
}
/* ------------------------------------------------------------ */

/* Creates webpage and sends it as responce to the client       */
void handleRoot()
{
  String str = Make_page();
  server.send(200, "text/html", str); //Send web page
}
/* ------------------------------------------------------------ */

/* Collects data from sensors if present;                       */
/* if no devices were detected during setup, assigns NaN;       */
/* sends data to the client once it is available                */
void data()
{
  float tmp  = std::numeric_limits<float>::quiet_NaN();
  float hum  = std::numeric_limits<float>::quiet_NaN();
  float prs  = std::numeric_limits<float>::quiet_NaN();
  float c3h8 = std::numeric_limits<float>::quiet_NaN();
  float dust = std::numeric_limits<float>::quiet_NaN();

  /* Gets data */
  if (_ad7792_present)
  {
    prs  = GET_PRS();
    c3h8 = GET_C3H8();
    dust = GET_DUST();
  }
  else
  {
    prs  = std::numeric_limits<float>::quiet_NaN();
    c3h8 = std::numeric_limits<float>::quiet_NaN();
    dust = std::numeric_limits<float>::quiet_NaN();
  }

  if (_sht10_present)
  {
    tmp  = GET_TMP;
    hum  = GET_HUM;
  }
  else
  {
    tmp  = std::numeric_limits<float>::quiet_NaN();
    hum  = std::numeric_limits<float>::quiet_NaN();
  }

  /* Generates and sends responce to the client */
  char Str[512];
  sprintf(Str, "<br>Temperature: %.2f oC<br/>       \
                <br>Pressure: %.2f %s<br/>          \
                <br>Humidity: %.2f %c</br>          \
                <br>C3H8: %.2f %s<br/>              \
                <br>Dust density: %.2f %s<br/>",    \ 
                tmp, prs, PRS_UNITS, hum, '%', c3h8, C3H8_UNITS, dust, DUST_UNITS);
  server.send(200, "text/plane", Str); 
}
/* ------------------------------------------------------------ */

void setup()
{
  Serial.begin(115200);
  Serial.println();
  
  /* AD7792 reset, setup and internal calibration */
  Serial.println("Initializing AD7792");
  if (AD7792_Init() == 1)
  {
    Serial.println("Resetting AD7792");
    AD7792_Reset();
    Serial.println("Setting gain... ");
    AD7792_SetGain(AD7792_GAIN_1);
    Serial.println("Setting internal reference");
    AD7792_SetIntReference(AD7792_REFSEL_INT);
  
    AD7792_SetMode(AD7792_MODE_CAL_INT_ZERO);
    AD7792_Calibrate(AD7792_MODE_CAL_INT_ZERO, AD7792_CH_AIN1P_AIN1M);
    AD7792_Calibrate(AD7792_MODE_CAL_INT_ZERO, AD7792_CH_AIN2P_AIN2M);
    AD7792_Calibrate(AD7792_MODE_CAL_INT_ZERO, AD7792_CH_AIN3P_AIN3M);
  
    AD7792_SetMode(AD7792_MODE_CAL_INT_FULL);
    AD7792_Calibrate(AD7792_MODE_CAL_INT_FULL, AD7792_CH_AIN1P_AIN1M);
    AD7792_Calibrate(AD7792_MODE_CAL_INT_FULL, AD7792_CH_AIN2P_AIN2M);
    AD7792_Calibrate(AD7792_MODE_CAL_INT_FULL, AD7792_CH_AIN3P_AIN3M);

    AD7792_SetMode(AD7792_MODE_SINGLE);

    AD7792_SetRegisterValue(AD7792_REG_IO, 
                            AD7792_IEXCEN(AD7792_EN_IXCEN_10uA) | AD7792_DIR_IEXC1_IEXC2_IOUT2,
                            1, 1);
    _ad7792_present = true;
    Serial.println("AD7792 init: SUCESS");
  }
  else
  {
    _ad7792_present = false;
    Serial.println("AD7792 init: FAIL");
  }
  Serial.println();
  /* When SHT1x is created the driver only initializes GPIO */
  /* Read/write to the status register is performed to detect its actual presence */
  /* The 6th bit in the status register is read only and can have any value */
  /* Hence, feedback can either be equal to 1 or to (1 | (1 << 6)) */
  Serial.println("Initializing SHT10");
  SHT1x.writeStatusReg(1);
  unsigned char feedback = SHT1x.readStatusReg();
  if ((feedback == 1) || (feedback == (1 | 1 << 6)))
  {
    _sht10_present = true;
    Serial.println("SUCESS");
  }
  else
  {
    _sht10_present = false;
    Serial.println("FAIL");
  }
  Serial.println();
  /* Creates the access point and assigns callbacks */
  /* Sending a command to the server looks like 192.168.4.1/takeData */
  /* If there is no command, it is assumed to be empty (/) */
  /* See handleRoot() and data() functions; data() is invoked everi second (see webPage.cpp) */
  boolean result = WiFi.softAP(_SSID, _PASS);
  IPAddress IP = WiFi.softAPIP();
  server.on("/", handleRoot);
  server.on("/takeData", data);
  server.begin();
  Serial.print("Setting soft-AP... ");
  if(result == true)
  {
    Serial.println("SUCESS");
  }
  else
  {
    Serial.println("FAIL");
  }
  Serial.print("Local IP: ");
  Serial.println(IP);
}

/* Actual hosting of a server */
void loop()
{
  server.handleClient();
}
