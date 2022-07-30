/**
 * Artificial nose - odor classification
 * 
 * Connect the listed sensors to the Wio Terminal. Upload this program to the 
 * Wio Terminal and look for the predicted odor on the LCD screen. You can also
 * open a serial terminal to view 
 * 
 * WARNING: You really should let the gas sensors preheat for >24 hours before
 * they are accurate. However, we can get something reasonable after about 7 min
 * of preheating. After giving power to the gas sensors, wait at least 7 min.
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
 * Date: July 16, 2022
 * License: 0BSD (https://opensource.org/licenses/0BSD)
 */

#include <Wire.h>

#include "Multichannel_Gas_GMXXX.h"
#include "seeed_bme680.h"
#include "sensirion_common.h"
#include "sgp30.h"
#include "TFT_eSPI.h"                                 // Comes with Wio Terminal package

#include "ai-nose_inferencing.h"                      // Name of Edge Impulse library

// Settings
#define DEBUG               1                         // 1 to print out debugging info
#define DEBUG_NN            false                     // Print out EI debugging info
#define ANOMALY_THRESHOLD   0.3                       // Scores above this are an "anomaly"
#define SAMPLING_FREQ_HZ    4                         // Sampling frequency (Hz)
#define SAMPLING_PERIOD_MS  1000 / SAMPLING_FREQ_HZ   // Sampling period (ms)
#define NUM_SAMPLES         EI_CLASSIFIER_RAW_SAMPLE_COUNT  // 4 samples at 4 Hz
#define READINGS_PER_SAMPLE EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME // 8

// Constants
#define BME680_I2C_ADDR     uint8_t(0x76)             // I2C address of BME680
#define PA_IN_KPA           1000.0                    // Convert Pa to KPa

// Preprocessing constants (drop the timestamp column)
float mins[] = {
  28.35, 36.17, 400.0, 0.0, 1.56, 0.81, 1.51, 0.61
};
float ranges[] = {
  12.12, 22.62, 56930.0, 60000.0, 1.48, 2.05, 1.6, 2.68
};

// Global objects
GAS_GMXXX<TwoWire> gas;               // Multichannel gas sensor v2
Seeed_BME680 bme680(BME680_I2C_ADDR); // Environmental sensor
TFT_eSPI tft;                         // Wio Terminal LCD

void setup() {
  
  int16_t sgp_err;
  uint16_t sgp_eth;
  uint16_t sgp_h2;
  
  // Start serial
  Serial.begin(115200);

  // Configure LCD
  tft.begin();
  tft.setRotation(3);
  tft.setFreeFont(&FreeSansBoldOblique24pt7b);
  tft.fillScreen(TFT_BLACK);

  // Initialize gas sensors
  gas.begin(Wire, 0x08);

  // Initialize environmental sensor
  while (!bme680.init()) {
    Serial.println("Trying to initialize BME680...");
    delay(1000);
  }

  // Initialize VOC and eCO2 sensor
  while (sgp_probe() != STATUS_OK) {
    Serial.println("Trying to initialize SGP30...");
    delay(1000);
  }

  // Perform initial read
  sgp_err = sgp_measure_signals_blocking_read(&sgp_eth, &sgp_h2);
  if (sgp_err != STATUS_OK) {
    Serial.println("Error: Could not read signal from SGP30");
    while (1);
  }
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
  static float raw_buf[NUM_SAMPLES * READINGS_PER_SAMPLE];
  static signal_t signal;
  float temp;
  int max_idx = 0;
  float max_val = 0.0;
  char str_buf[40];
  
  // Collect samples and perform inference
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
      Serial.println("Error: Could not read from BME680");
      return;
    }
  
    // Read SGP30 sensor
    sgp_err = sgp_measure_iaq_blocking_read(&sgp_tvoc, &sgp_co2);
    if (sgp_err != STATUS_OK) {
      Serial.println("Error: Could not read from SGP30");
      return;
    }

    // Store raw data into the buffer
    raw_buf[(i * READINGS_PER_SAMPLE) + 0] = bme680.sensor_result_value.temperature;
    raw_buf[(i * READINGS_PER_SAMPLE) + 1] = bme680.sensor_result_value.humidity;
    raw_buf[(i * READINGS_PER_SAMPLE) + 2] = sgp_co2;
    raw_buf[(i * READINGS_PER_SAMPLE) + 3] = sgp_tvoc;
    raw_buf[(i * READINGS_PER_SAMPLE) + 4] = gm_voc_v;
    raw_buf[(i * READINGS_PER_SAMPLE) + 5] = gm_no2_v;
    raw_buf[(i * READINGS_PER_SAMPLE) + 6] = gm_eth_v;
    raw_buf[(i * READINGS_PER_SAMPLE) + 7] = gm_co_v;

    // Perform preprocessing step (normalization) on all readings in the sample
    for (int j = 0; j < READINGS_PER_SAMPLE; j++) {
      temp = raw_buf[(i * READINGS_PER_SAMPLE) + j] - mins[j];
      raw_buf[(i * READINGS_PER_SAMPLE) + j] = temp / ranges[j];
    }

    // Wait just long enough for our sampling period
    while (millis() < timestamp + SAMPLING_PERIOD_MS);
  }

  // Print out our preprocessed, raw buffer
#if DEBUG
  for (int i = 0; i < NUM_SAMPLES * READINGS_PER_SAMPLE; i++) {
    Serial.print(raw_buf[i]);
    if (i < (NUM_SAMPLES * READINGS_PER_SAMPLE) - 1) {
      Serial.print(", ");
    }
  }
  Serial.println();
#endif

  // Turn the raw buffer in a signal which we can the classify
  int err = numpy::signal_from_buffer(raw_buf, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
  if (err != 0) {
      ei_printf("ERROR: Failed to create signal from buffer (%d)\r\n", err);
      return;
  }

  // Run inference
  ei_impulse_result_t result = {0};
  err = run_classifier(&signal, &result, DEBUG_NN);
  if (err != EI_IMPULSE_OK) {
      ei_printf("ERROR: Failed to run classifier (%d)\r\n", err);
      return;
  }

  // Print the predictions
  ei_printf("Predictions ");
  ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)\r\n",
        result.timing.dsp, result.timing.classification, result.timing.anomaly);
  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    ei_printf("\t%s: %.3f\r\n", 
              result.classification[i].label, 
              result.classification[i].value);
  }

  // Print anomaly detection score
#if EI_CLASSIFIER_HAS_ANOMALY == 1
  ei_printf("\tanomaly acore: %.3f\r\n", result.anomaly);
#endif

  // Find maximum prediction
  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    if (result.classification[i].value > max_val) {
      max_val = result.classification[i].value;
      max_idx = i;
    }
  }

  // Print predicted label and value to LCD if not anomalous
  tft.fillScreen(TFT_BLACK);
  if (result.anomaly < ANOMALY_THRESHOLD) {
    tft.drawString(result.classification[max_idx].label, 20, 60);
    sprintf(str_buf, "%.3f", max_val);
    tft.drawString(str_buf, 60, 120);
  } else {
    tft.drawString("Unknown", 20, 60);
    sprintf(str_buf, "%.3f", result.anomaly);
    tft.drawString(str_buf, 60, 120);
  }
}
