#include "Arduino.h"
#include "bme68xLibrary.h"
#include "Wire.h"
#include "SensirionI2CSfm3000.h"
#include <HX711_ADC.h>
#include <RTClib.h>
#include <SD.h>
#include <SPI.h>
#if defined(ESP8266) || defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

#include "time.h"
#include "sntp.h"
#include "RunningAverage.h"

#if defined(ESP32)
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
#define DEVICE "ESP32"
#elif defined(ESP8266)
#include <ESP8266WiFiMulti.h>
ESP8266WiFiMulti wifiMulti;
#define DEVICE "ESP8266"
#endif

/* WiFi AP SSID */
#define WIFI_SSID "UHN-Guest-WiFi"
// #define WIFI_SSID "WC_Guest"
/* WiFi password */
#define WIFI_PASSWORD ""
// #define WIFI_PASSWORD "WomensCollege"

#define TZ_INFO "EST5EDT"

#define TCAADDR 0x70
#define MEAS_DUR 140
#define N_SENS 2

/* Set up time with wifi */
const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;
const char* time_zone = "EST5EDT,M3.2.0,M11.1.0";

String LOG_FILE_NAME = "/";

/* set up sensors */
Bme68x bme[N_SENS];
bme68xData data[N_SENS] = { 0 };
SensirionI2CSfm3000 sfm[N_SENS];
File file;
RTC_DS3231 rtc;

uint8_t lastMeasindex = { 0 };
bme68xData sensorData = { 0 };
String logHeader;
uint32_t lastLogged = 0;

const int chipSelect = 10;

struct flowMass {
  float input;
  float output;
  float mass;
};

struct flowMass flowData;

float scalingFactor = 140.0;
float offset = 32000;

/* HX711 EEPROM */
const int calVal_eepromAdress = 0;
unsigned long t = 0;
float calVal;

/* HX711 pins */
const int HX711_dout = 4;  //mcu > HX711 dout pin
const int HX711_sck = 5;   //mcu > HX711 sck pin

/* HX711 constructor */
HX711_ADC LoadCell(HX711_dout, HX711_sck);

/* Declare running average variables */
RunningAverage inputFlowRA(10);
RunningAverage outputFlowRA(10);
RunningAverage inputResRA(10);
RunningAverage outputResRA(10);
RunningAverage inputTempRA(10);
RunningAverage outputTempRA(10);
RunningAverage inputPresRA(10);
RunningAverage outputPresRA(10);
RunningAverage inputHumRA(10);
RunningAverage outputHumRA(10);
RunningAverage massRA(10);

void setup(void) {
  Serial.begin(115200);
  Wire.begin();

  /* Set up RTC */
  if (!rtc.begin()) {
      Serial.println("RTC module is NOT found");
      while (1);
    }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting time...");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  while (!Serial)
    delay(100);

  wifiSetup();
  sdSetup();

  startLoadCell();

  bme[0].begin(0x76, Wire);
  bme[1].begin(0x77, Wire);
  bmeSetup();

  sfm3000Setup(0,0);
  sfm3000Setup(1,7);
}


void loop(void) {
  delay(200);

  flowData.input = get_sfm3000_data(0,0);
  inputFlowRA.add(flowData.input);
  flowData.output = get_sfm3000_data(1,7);
  outputFlowRA.add(flowData.output);

  getLoadCellData();
  get_bme688_data(flowData);
}


void startLoadCell(void) {
  LoadCell.begin();
  //LoadCell.setReverseOutput(); //uncomment to turn a negative output value to positive

  /* calibration value (see example file "Calibration.ino")
     store this in EEPROM */

  float calibrationValue;

#if defined(ESP8266) || defined(ESP32)
  EEPROM.begin(512);  // uncomment this if you use ESP8266/ESP32 and want to fetch the calibration value from eeprom
#endif

  if (EEPROM.read(calVal_eepromAdress) == 255) {
    calibrationValue = -128.0;  // if the eeprom is empty (255 or 0xFF), load a dummy value for calibration.  -128 is typically close to the final cal val.
  } else {
    EEPROM.get(calVal_eepromAdress, calibrationValue);  // uncomment this if you want to fetch the calibration value from eeprom
  }

  unsigned long stabilizingtime = 2000;  // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;                  //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    //    setGenericDisplay("Timeout, check MCU>HX711 wiring and pin designations", 1);
    while (1)
      ;
  } else {
    LoadCell.setCalFactor(calibrationValue);  // set calibration value (float)
  }

  LoadCell.tareNoDelay();
  delay(1000);
}


void getLoadCellData(void) {
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0;  //increase value to slow down serial print activity, default 0

  /* check for new data/start next conversion: */
  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      flowData.mass = LoadCell.getData();
      massRA.add(flowData.mass);
      Serial.println("Mass:" + String(flowData.mass));

      newDataReady = 0;
      t = millis();
    }
  }
}


void wifiSetup() {
  configTzTime(time_zone, ntpServer1, ntpServer2);

  Serial.printf("Connecting to %s ", WIFI_SSID);
  WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" CONNECTED");
}


void sdSetup(void) {
  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("1. is a card inserted?");
    Serial.println("2. is your wiring correct?");
    Serial.println("3. did you change the chipSelect pin to match your shield or module?");
    Serial.println("Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!");
    while (true);
  } else {
    LOG_FILE_NAME += timeToString();
    LOG_FILE_NAME += ".csv";

    file = SD.open(LOG_FILE_NAME, FILE_WRITE);
    if (!file) {
      Serial.println("Failed to open file for writing");
    }
    /* Parameters for logging in the file */
    logHeader = "TimeStamp,Input Temperature(deg "
                "C),Input Pressure(Pa),Input Humidity(%),Input Gas Resistance(ohm),Input Gas "
                "Index,Input Meas Index,Input idac,Input Status,Input Gas Valid,Input Heater Stable,"
                "Output Temperature(deg C),Output Pressure(Pa),Output Humidity(%),Output Gas Resistance(ohm),"
                "Output Gas Index,Output Meas Index,Output idac,Output Status,Output Gas Valid,Output Heater "
                "Stable,Input Flow,Output Flow,Mass (g),RAI Temp,RAO Temp,RAI Hum,RAO Hum,RAI Pres,"
                "RAO Pres,RAI Res,RAO Res,RAI Flow,RAO Flow, RA Mass";

    if (file.println(logHeader)) {
      file.close();
    }
    logHeader = "";
  }
  Serial.println("initialization done.");
}


void bmeSetup(void) {
  /* check if bme688 is online */
  for (int i = 0; i < N_SENS; i++) {
    if (bme[i].checkStatus()) {
      if (bme[i].checkStatus() == BME68X_ERROR) {
        Serial.println("Sensor error:" + bme[i].statusString());
        return;
      } else if (bme[i].checkStatus() == BME68X_WARNING) {
        Serial.println("Sensor Warning:" + bme[i].statusString());
      }
    }

    bme[i].setTPH();

    /* Heater temperature in degree Celsius as per the suggested heater profile
      */
    // uint16_t tempProf[10] = { 320, 100, 100, 100, 200, 200, 200, 320, 320, 320 };
    // uint16_t tempProf[10] = { 320, 320, 320, 320, 320, 320, 320, 320, 320, 320 };
    /* Multiplier to the shared heater duration */
    // uint16_t mulProf[10] = { 5, 2, 10, 30, 5, 5, 5, 5, 5, 5 };
    // /* Shared heating duration in milliseconds */
    // uint16_t sharedHeatrDur =
    //   MEAS_DUR - (bme[i].getMeasDur(BME68X_PARALLEL_MODE) / INT64_C(1000));

    // bme[i].setHeaterProf(tempProf, mulProf, sharedHeatrDur, 10);
    bme[i].setHeaterProf(320,100);

    /* Parallel mode of sensor operation */
    // bme[i].setOpMode(BME68X_PARALLEL_MODE);
  }
}


/* Set up multiplexer */
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}


void sfm3000Setup(int i, int bus) {
  uint16_t error;
  char errorMessage[64];
  
  tcaselect(bus);
  sfm[i].begin(Wire, SFM300_I2C_ADDRESS_0);

  uint32_t serialNumber;

  error = sfm[i].readSerialNumber(serialNumber);

  if (error) {
      Serial.print("Error for sensor on port " + String(bus) + "serialNumber(): ");
      errorToString(error, errorMessage, 64);
      Serial.println(errorMessage);
  } else {
      Serial.print("SerialNumber:");
      Serial.print(serialNumber);
      Serial.println();
  }

  error = sfm[i].startContinuousMeasurement();

  if (error) {
      Serial.print("Error trying to execute startContinuousMeasurement(): ");
      errorToString(error, errorMessage, 64);
      Serial.println(errorMessage);
  }   
}


float get_sfm3000_data(int i, int bus) {
  uint16_t error;
  float flow;
  char errorMessage[64];

  /* Read Flow Measurement */
  tcaselect(bus);
  error = sfm[i].readMeasurement(flow, scalingFactor, offset);

  if (error) {
    Serial.print("Error trying to execute readMeasurement(): ");
    errorToString(error, errorMessage, 64);
    Serial.println(errorMessage);
  } else {
    Serial.print("Flow" + String(i) + ":" + String(flow) + ",");
  }
  return flow;
}


void get_bme688_data(struct flowMass flowData) {
  int16_t indexDiff;
  bool newLogdata = false;
  float temperature;
  float pressure;
  float humidity;
  float resistance;
  
  if ((millis() - lastLogged) >= MEAS_DUR) {
    lastLogged = millis();
    logHeader += timeToString();
    logHeader += ",";

    for (int i = 0; i < N_SENS; i++) {
      bme[i].setOpMode(BME68X_FORCED_MODE);

      if (bme[i].fetchData()) {
        bme[i].getData(data[i]);
        temperature = data[i].temperature;
        pressure = data[i].pressure;
        humidity = data[i].humidity;
        resistance = data[i].gas_resistance;

        /* Add data to running average, only works for 2 sensors */
        if (i == 0) {
          inputTempRA.addValue(temperature);
          inputPresRA.addValue(pressure);
          inputHumRA.addValue(humidity);
          inputResRA.addValue(resistance);
        } else {
          outputTempRA.addValue(temperature);
          outputPresRA.addValue(pressure);
          outputHumRA.addValue(humidity);
          outputResRA.addValue(resistance);
        }

        // comment out next two lines so they're not on the serial plotter (limited to 8 values drawn)
        // Serial.print("Temp" + String(i) + ":" + String(temperature) + ",");
        // Serial.print("Pressure" + String(i) + String(data[i].pressure) + ",");
        Serial.print("Humidity" + String(i) + ":" + String(humidity) + ",");
        Serial.print("Resistance" + String(i) + ":" + String(resistance/1000000) + ",");

        lastMeasindex = data[i].meas_index;
      }

      logHeader += temperature;
      logHeader += ",";
      logHeader += pressure;
      logHeader += ",";
      logHeader += humidity;
      logHeader += ",";
      logHeader += resistance;
      logHeader += ",";
      logHeader += data[i].gas_index;
      logHeader += ",";
      logHeader += data[i].meas_index;
      logHeader += ",";
      logHeader += data[i].idac;
      logHeader += ",";
      logHeader += String(data[i].status, HEX);
      logHeader += ",";
      logHeader += data[i].status & BME68X_GASM_VALID_MSK;
      logHeader += ",";
      logHeader += data[i].status & BME68X_HEAT_STAB_MSK;
      logHeader += ",";
    }

    // serial plotter can't draw more than 8 values, so if we want to see mass we need to reduce the # of things to monitor
    // here we sacrifice the temperature @ input for the serial plotter
    // but temp @ input is still recorded on the datasheet
    Serial.print("temp_output:" + String(data[1].temperature) + ",");
  }

  logHeader += flowData.input;
  logHeader += ",";
  logHeader += flowData.output;
  logHeader += ",";
  logHeader += flowData.mass;

  if (inputResRA.bufferIsFull()) {
    logHeader += ",";
    logHeader += inputTempRA.getAverage();
    logHeader += ",";
    logHeader += outputTempRA.getAverage();
    logHeader += ",";
    logHeader += inputHumRA.getAverage();
    logHeader += ",";
    logHeader += outputHumRA.getAverage();
    logHeader += ",";
    logHeader += inputPresRA.getAverage();
    logHeader += ",";
    logHeader += outputPresRA.getAverage();
    logHeader += ",";
    logHeader += inputResRA.getAverage();
    logHeader += ",";
    logHeader += outputResRA.getAverage();
    logHeader += ",";
    logHeader += inputFlowRA.getAverage();
    logHeader += ",";
    logHeader += outputFlowRA.getAverage();
    logHeader += ",";
    logHeader += massRA.getAverage();
    logHeader += "\r\n";
    newLogdata = true;
  } else {
    logHeader += "\r\n";
    newLogdata = true;
  }
  
  if (newLogdata) {
    newLogdata = false;

    appendFile(logHeader);
    logHeader = "";
  }
}


/* Time using Wifi, comment out RTC time function below and uncomment this one */
String timeToString(void) {
  struct tm timeInfo;
  char buf[50];
  char timeStr[100] = "";
  String time;

  if (!getLocalTime(&timeInfo)) {
    Serial.println("Failed to obtain time");
  }

  strftime(buf, sizeof(buf), "%F", &timeInfo);
  strcat(timeStr, buf);
  strcat(timeStr, " ");

  strftime(buf, sizeof(buf), "%H%M%S", &timeInfo);
  strcat(timeStr, buf);

  time = String(timeStr);
  
  return time;
}


/* Time using RTC, comment out function above and uncomment function below */
// String timeToString(void) {
//   String time = "";
//   DateTime now = rtc.now();

//   time += now.year();
//   time += "-";

//   if (now.month() < 10) {
//     time += "0";
//     time += now.month();
//   } else {
//     time += now.month();
//   }

//   time += "-";

//   if (now.day() < 10) {
//     time += "0";
//     time += now.day();
//   } else {
//     time += now.day();
//   }

//   time += " ";

//   if (now.hour() < 10) {
//     time += "0";
//     time += now.hour();
//   } else {
//     time += now.hour();
//   }

//   if (now.minute() < 10) {
//     time += "0";
//     time += now.minute();
//   } else {
//     time += now.minute();
//   }

//   if (now.second() < 10) {
//     time += "0";
//     time += now.second();
//   } else {
//     time += now.second();
//   }
//   return time;
// }


/*!
 * @brief Writing the sensor data to the log file(csv)
 * @param sensorData
 */
static void writeFile(String sensorData) {
  if (!SD.open(LOG_FILE_NAME, FILE_WRITE)) {
    Serial.println("Failed to open file for writing");
  } else {
    SD.open(LOG_FILE_NAME);
    file.print(sensorData);
    Serial.println("Wrote to file. ");
  }
  file.close();
}


/*!
 * @brief Appending the sensor data into the log file(csv)
 * @param sensorData
 */
static void appendFile(String sensorData) {
  file = SD.open(LOG_FILE_NAME, FILE_APPEND);

  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }

  if (file.println(sensorData)) {
  } else {
    Serial.println("Failed to append");
  }
  file.close();
}