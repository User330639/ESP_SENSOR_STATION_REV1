#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Task.h>
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

#define BUZZER_GPIO    16
#define BUZZER_OFF     analogWrite(BUZZER_GPIO, 0)
#define BUZZER_ON      analogWrite(BUZZER_GPIO, 127);

#define LED0_GPIO      9  // Normal opeartion LED
#define LED1_GPIO      10 // Alarm LED

#define DUST_SNS_GPIO  2
/* ------------------------------------------------------------ */

/* Raw-to-values macros and definitions                         */
#define AD7792_INT_VREF           1.17f
#define AD7792_MAX_CODE           65535UL
#define AD7792_INPUT_VOLTAGE(raw) ((float)raw * AD7792_INT_VREF / (float)AD7792_MAX_CODE)

#define V_TO_PRESSURE(v)          ((float)v * 5.7f / (5.0f * 0.002421f) + 0.00848f)
#define PRS_UNITS                 "kPa"

#define V_TO_C_DUST(v)            (0.07f + 0.33f * ((float)v * 5.7f - 1.0f) / 2.0f)
#define DUST_UNITS                "mg/m3"

#define C3H8_V_TO_R(v)            ((float)v / 10e-6)
#define C3H8_R0                   40000.0f
#define C3H8_R_TO_PPM(r)          exp(4.9012f + 0.0036f * ((float)r / C3H8_R0)) // exponential regression
#define C3H8_UNITS                "ppm"
/* ------------------------------------------------------------ */

#define TEMPERATURE_ALARM_TRESHLD_LO 0.0f
#define TEMPERATURE_ALARM_TRESHLD_HI 40.0f
#define HUMIDITY_ALARM_TRESHLD_LO    5.0f
#define HUMIDITY_ALARM_TRESHLD_HI    80.0f
#define PRESSURE_ALARM_TRESHLD_LO    80.0f
#define PRESSURE_ALARM_TRESHLD_HI    120.0f
#define C3H8_ALARM_TRESHLD_LO        0.0f
#define C3H8_ALARM_TRESHLD_HI        500.0f
#define CDUST_ALARM_TRESHLD_LO       0.0f
#define CDUST_ALARM_TRESHLD_HI       1.0f
#define ALARM_CONDITION                                                             \
(                                                                                   \
  (tmp  > TEMPERATURE_ALARM_TRESHLD_HI) || (tmp  < TEMPERATURE_ALARM_TRESHLD_LO) || \
  (hum  > HUMIDITY_ALARM_TRESHLD_HI)    || (hum  < HUMIDITY_ALARM_TRESHLD_LO)    || \
  (prs  > PRESSURE_ALARM_TRESHLD_HI)    || (prs  < PRESSURE_ALARM_TRESHLD_LO)    || \
  (c3h8 > C3H8_ALARM_TRESHLD_HI)        || (c3h8 < C3H8_ALARM_TRESHLD_LO)        || \
  (dust > CDUST_ALARM_TRESHLD_HI)       || (dust < CDUST_ALARM_TRESHLD_LO)          \
)

/* Global objects and variables                                 */
ESP8266WebServer server(80); // The server hosts a webpage port #80; see webPage.h and webPage.cpp
SHT1x SHT1x(SHT1x_dataPin, SHT1x_clockPin); // Global SHT1x object; the constructor requires GPIO numbers used for communication
boolean _ad7792_present = false; // Global variable that indicates presence or absence of AD7792
boolean _sht10_present = false; // Global variable that indicates presence or absence of SHT1x
TaskManager taskManager; // Task manager used to put measurements/alarms and client handling into separate threads

float tmp  = std::numeric_limits<float>::quiet_NaN();
float hum  = std::numeric_limits<float>::quiet_NaN();
float prs  = std::numeric_limits<float>::quiet_NaN();
float c3h8 = std::numeric_limits<float>::quiet_NaN();
float dust = std::numeric_limits<float>::quiet_NaN();

static const FlashMode_t ideMode = ESP.getFlashChipMode();
/* ------------------------------------------------------------ */

void dataUpdater(uint32_t deltaTime);
void clientHandler(uint32_t deltaTime);
void alarmOn(uint32_t deltaTime);

FunctionTask taskDataUpdater(dataUpdater, MsToTaskTime(100));
FunctionTask taskClientHandler(clientHandler, MsToTaskTime(1500));
FunctionTask taskAlarmOn(alarmOn, MsToTaskTime(500));


/* Returns the value of pressure in PRS_UNITS units             */
float GET_PRS()
{
  AD7792_SetChannel(AD7792_CH_AIN1P_AIN1M);
  unsigned long code = AD7792_ContinuousReadAvg(8);
  float sensor_voltage = AD7792_INPUT_VOLTAGE(code);
  return V_TO_PRESSURE(sensor_voltage);
}
/* ------------------------------------------------------------ */

/* Same as GET_PRS() but returns dust concentration             */
float GET_DUST()
{
  digitalWrite(DUST_SNS_GPIO, HIGH);
  delay(1);
  AD7792_SetChannel(AD7792_CH_AIN2P_AIN2M);
  unsigned long code = AD7792_ContinuousReadAvg(8);
  float sensor_voltage = AD7792_INPUT_VOLTAGE(code);
  delay(1);
  digitalWrite(DUST_SNS_GPIO, LOW);
  return V_TO_C_DUST(sensor_voltage);
}
/* ------------------------------------------------------------ */

/* Same as GET_PRS() but returns C3H8 concentration             */
float GET_C3H8()
{
  AD7792_SetChannel(AD7792_CH_AIN3P_AIN3M);
  unsigned long code = AD7792_ContinuousReadAvg(8);
  float sensor_voltage = AD7792_INPUT_VOLTAGE(code);
  float sensor_resistance = C3H8_V_TO_R(sensor_voltage);
  return C3H8_R_TO_PPM(sensor_resistance);
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

void dataUpdater(uint32_t deltaTime)
{
  tmp  = std::numeric_limits<float>::quiet_NaN();
  hum  = std::numeric_limits<float>::quiet_NaN();
  prs  = std::numeric_limits<float>::quiet_NaN();
  c3h8 = std::numeric_limits<float>::quiet_NaN();
  dust = std::numeric_limits<float>::quiet_NaN();

  /* Gets data */
  if (_ad7792_present)
  {
    prs  = GET_PRS();
    c3h8 = GET_C3H8();
    dust = GET_DUST();
  }
  if (_sht10_present)
  {
    tmp  = GET_TMP;
    hum  = GET_HUM;
  }
  if (ALARM_CONDITION)
  {
    taskManager.StartTask(&taskAlarmOn);
  }
}

void clientHandler(uint32_t deltaTime)
{
  server.handleClient();
}

void alarmOn(uint32_t deltaTime)
{
#ifdef ESP_QSPI_DIO
  digitalWrite(LED1_GPIO, HIGH);
#endif
  BUZZER_ON;
  delay(75);
  BUZZER_OFF;
  delay(75);
  BUZZER_ON;
  delay(75);
  BUZZER_OFF;
  delay(75);
  BUZZER_ON;
  delay(75);
  BUZZER_OFF;
  if (!ALARM_CONDITION)
  {
    if (ideMode == FM_DIO || ideMode == FM_DOUT)
    {
      digitalWrite(LED1_GPIO, LOW);
    }
    taskManager.StopTask(&taskAlarmOn);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println();

  Serial.printf("Flash ide mode:  %s\n", (ideMode == FM_QIO ? "QIO" : ideMode == FM_QOUT ? "QOUT" : ideMode == FM_DIO ? "DIO" : ideMode == FM_DOUT ? "DOUT" : "UNKNOWN"));
  
  Serial.print("Initializing GPIO... ");
  pinMode(BUZZER_GPIO, OUTPUT);
  analogWrite(BUZZER_GPIO, 0);
  if (ideMode == FM_DIO || ideMode == FM_DOUT)
  {
    pinMode(LED0_GPIO, OUTPUT);
    pinMode(LED1_GPIO, OUTPUT);
    digitalWrite(LED0_GPIO, HIGH);
    digitalWrite(LED1_GPIO, LOW);
  }
  pinMode(DUST_SNS_GPIO, OUTPUT);
  digitalWrite(DUST_SNS_GPIO, LOW);
  Serial.println("SUCESS");
  
  /* AD7792 reset, setup and internal calibration */
  Serial.print("Initializing AD7792... ");
  if (AD7792_Init() == 1)
  {
    Serial.println();
    Serial.print("Resetting AD7792... ");
    AD7792_Reset();
    Serial.println("SUCESS");

    AD7792_SetMode(AD7792_MODE_IDLE);
    
    uint16_t cfg_reg = AD7792_GetRegisterValue(AD7792_REG_CONF, 2, 0);
    Serial.print("CFG register contents after reset: ");
    Serial.println(cfg_reg);
    
    Serial.print("Disabling internal buffer and enabling unipolar mode... ");
    cfg_reg &= ~AD7792_CONF_BUF;     // Disable the internal buffer
    cfg_reg |= AD7792_CONF_UNIPOLAR; // Unipolar mode
    AD7792_SetRegisterValue(AD7792_REG_CONF, cfg_reg, 2, 0);
    Serial.println("SUCESS");
    
    Serial.print("Setting gain... ");
    AD7792_SetGain(AD7792_GAIN_1);
    Serial.println("SUCESS");

    Serial.print("Setting internal reference... ");
    AD7792_SetIntReference(AD7792_REFSEL_INT);
    Serial.println("SUCESS");
    
    Serial.print("Performing zero scale calibration... ");
    AD7792_Calibrate(AD7792_MODE_CAL_INT_ZERO, AD7792_CH_AIN1P_AIN1M);
    AD7792_Calibrate(AD7792_MODE_CAL_INT_ZERO, AD7792_CH_AIN2P_AIN2M);
    AD7792_Calibrate(AD7792_MODE_CAL_INT_ZERO, AD7792_CH_AIN3P_AIN3M);
    Serial.println("SUCESS");
    
    Serial.print("Performing full scale calibration... ");
    AD7792_Calibrate(AD7792_MODE_CAL_INT_FULL, AD7792_CH_AIN1P_AIN1M);
    AD7792_Calibrate(AD7792_MODE_CAL_INT_FULL, AD7792_CH_AIN2P_AIN2M);
    AD7792_Calibrate(AD7792_MODE_CAL_INT_FULL, AD7792_CH_AIN3P_AIN3M);
    Serial.println("SUCESS");

    Serial.print("Enabling gas sensor excitation current... ");
    AD7792_SetRegisterValue(AD7792_REG_IO, 
                            AD7792_IEXCEN(AD7792_EN_IXCEN_10uA) | AD7792_DIR_IEXC1_IOUT1_IEXC2_IOUT2,
                            1, 1);
    Serial.println("SUCESS");

    AD7792_SetMode(AD7792_MODE_CONT);
    
    _ad7792_present = true;
  }
  else
  {
    _ad7792_present = false;
    Serial.println("FAIL");
  }

  /* When SHT1x is created the driver only initializes GPIO                       */
  /* Read/write to the status register is performed to detect its actual presence */
  /* The 6th bit in the status register is read only and can have any value       */
  /* Hence, feedback can either be equal to 1 or to (1 | (1 << 6))                */
  Serial.print("Initializing SHT10... ");
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

  /* Creates the access point and assigns callbacks                                          */
  /* Sending a command to the server looks like 192.168.4.1/takeData                         */
  /* If there is no command, it is assumed to be empty (/)                                   */
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

  taskManager.StartTask(&taskDataUpdater);
  taskManager.StartTask(&taskClientHandler);
}

/* Start the task scheduler kernel */
void loop()
{
  taskManager.Loop();
}
