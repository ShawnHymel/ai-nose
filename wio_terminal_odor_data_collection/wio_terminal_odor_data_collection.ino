/**
 * Odor data collection
 * 
 * Press the 5-way switch hat in to start collection process for 2 seconds.
 * Sensor data is sent over serial port as CSV. Use a collection script to
 * save the data into .csv files.
 * 
 * WARNING: You really should let the gas sensors preheat for >24 hours before
 * they are accurate. However, we can get something reasonable after about 7 min
 * of preheating. After giving power to the gas sensors, wait at least 7 min.
 * 
 * Also: the gas sensors aren't really accurate (even after preheating). They
 * should be calibrated and compensated using temperature and humidity data (see
 * their respective datasheets). That being said, we just care about relative
 * data when attempting to make classification predictions, so this should be
 * good enough.
 * 
 * Collection script:
 *   https://github.com/edgeimpulse/example-data-collection-csv/blob/main/serial-data-collect-csv.py
 * 
 * Based on the work by Benjamin Cabé:
 *   https://github.com/kartben/artificial-nose
 * 
 * Sensors:
 *   https://wiki.seeedstudio.com/Grove-Multichannel-Gas-Sensor-V2/
 *   https://wiki.seeedstudio.com/Grove-Temperature_Humidity_Pressure_Gas_Sensor_BME680/
 *   https://wiki.seeedstudio.com/Grove-VOC_and_eCO2_Gas_Sensor-SGP30/
 *  
 * Install the following libraries:
 *   https://github.com/Seeed-Studio/Seeed_Multichannel_Gas_Sensor/archive/master.zip
 *   https://github.com/Seeed-Studio/Seeed_BME680/archive/refs/heads/master.zip
 *   https://github.com/Seeed-Studio/SGP30_Gas_Sensor/archive/refs/heads/master.zip
 *   
 * Author: Shawn Hymel
 * Date: July 11, 2022
 * License: 0BSD (https://opensource.org/licenses/0BSD)
 */

#include <Wire.h>
//aditi
#include <TFT_eSPI.h>

#include "Multichannel_Gas_GMXXX.h"
#include "seeed_bme680.h"
#include "sensirion_common.h"
#include "sgp30.h"
//aditi
TFT_eSPI tft;

// Settings
#define BTN_START           0                         // 1: press button to start, 0: loop
#define BTN_PIN             WIO_5S_PRESS              // Pin that button is connected to
#define SAMPLING_FREQ_HZ    4                         // Sampling frequency (Hz)
#define SAMPLING_PERIOD_MS  1000 / SAMPLING_FREQ_HZ   // Sampling period (ms)
#define NUM_SAMPLES         8                         // 8 samples at 4 Hz is 2 seconds

// Constants
#define BME680_I2C_ADDR     uint8_t(0x76)             // I2C address of BME680
#define PA_IN_KPA           1000.0                    // Convert Pa to KPa

// Global objects
GAS_GMXXX<TwoWire> gas;               // Multichannel gas sensor v2
Seeed_BME680 bme680(BME680_I2C_ADDR); // Environmental sensor

void setup() {
  
  int16_t sgp_err;
  uint16_t sgp_eth;
  uint16_t sgp_h2;

  // Initialize button
  pinMode(BTN_PIN, INPUT_PULLUP);
  
  // Start serial
  Serial.begin(115200);

  // Initialize display
  tft.begin();
  tft.setRotation(3);  // Adjust rotation if needed

  // Initialize gas sensors
  gas.begin(Wire, 0x08);

  // Initialize environmental sensor
  while (!bme680.init()) {
    //aditi
        tft.fillScreen(TFT_RED);
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(10, 50);
    tft.println("Error: Could not initialize BME680");
    Serial.println("Trying to initialize BME680...");
    delay(1000);
    tft.fillScreen(TFT_BLACK);
  }

  // Initialize VOC and eCO2 sensor
  while (sgp_probe() != STATUS_OK) {
    //aditi
    tft.fillScreen(TFT_RED);
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(10, 50);
    tft.println("Error: Could not initialize SGP30");

    Serial.println("Trying to initialize SGP30...");
    delay(1000);
    //aditi
    tft.fillScreen(TFT_BLACK);
  }

  // Perform initial read
  sgp_err = sgp_measure_signals_blocking_read(&sgp_eth, &sgp_h2);
  if (sgp_err != STATUS_OK) {
    //aditi
    tft.fillScreen(TFT_RED);
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(10, 50);
    tft.println("Error: Could not read signal from SGP30");

    Serial.println("Error: Could not read signal from SGP30");
    while (1);
  }
}

//aditi
void displayMessage(const char *message, uint16_t color) {
  tft.fillScreen(color);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(10, 50);
  tft.println(message);
  delay(1000);
  tft.fillScreen(TFT_BLACK);
}

void loop() {

  float gm_no2_v;
  float gm_eth_v;
  float gm_voc_v;
  float gm_co_v;
  int16_t sgp_err;
  uint16_t sgp_tvoc;
  uint16_t sgp_co2;
  unsigned long timestamp;

  // Wait for button press
#if BTN_START
  while (digitalRead(BTN_PIN) == 1);
#endif

  // Display status message on the Wio Terminal display
  displayMessage("Data collection in progress...", TFT_BLUE);

  // Continue with the existing serial print statements
  // Print header
  Serial.println("timestamp,temp,humd,pres,co2,voc1,voc2,no2,eth,co");

  // Transmit samples over serial port
  for (int i = 0; i < NUM_SAMPLES; i++) {

    // Take timestamp so we can hit our target frequency
    timestamp = millis();

    // Read from GM-X02b sensors (multichannel gas)
    gm_no2_v = gas.calcVol(gas.getGM102B());
    gm_eth_v = gas.calcVol(gas.getGM302B());
    gm_voc_v = gas.calcVol(gas.getGM502B());
    gm_co_v = gas.calcVol(gas.getGM702B());
  
    // Read BME680 environmental sensor
    if (bme680.read_sensor_data()) {
      displayMessage("Error: Could not read from BME680", TFT_RED);
      Serial.println("Error: Could not read from BME680");
      return;
    }
  
    // Read SGP30 sensor
    sgp_err = sgp_measure_iaq_blocking_read(&sgp_tvoc, &sgp_co2);
    if (sgp_err != STATUS_OK) {
      displayMessage("Error: Could not read from SGP30", TFT_RED);
      Serial.println("Error: Could not read from SGP30");
      return;
    }

    // Print CSV data with timestamp
    Serial.print(timestamp);
    Serial.print(",");
    Serial.print(bme680.sensor_result_value.temperature);
    Serial.print(",");
    Serial.print(bme680.sensor_result_value.humidity);
    Serial.print(",");
    Serial.print(bme680.sensor_result_value.pressure / PA_IN_KPA);
    Serial.print(",");
    Serial.print(sgp_co2);
    Serial.print(",");
    Serial.print(sgp_tvoc);
    Serial.print(",");
    Serial.print(gm_voc_v);
    Serial.print(",");
    Serial.print(gm_no2_v);
    Serial.print(",");
    Serial.print(gm_eth_v);
    Serial.print(",");
    Serial.print(gm_co_v);
    Serial.println();

    // Wait just long enough for our sampling period
    while (millis() < timestamp + SAMPLING_PERIOD_MS);
  }

  // Print empty line to transmit termination of recording
  Serial.println();
displayMessage("Data collection complete", TFT_GREEN);
  // Make sure the button has been released for a few milliseconds
#if BTN_START
  while (digitalRead(BTN_PIN) == 0);
  delay(100);
#endif
}
