#include "artificial-nose_inferencing.h"

// Settings
const int debug_nn = false;

// Copy "Raw features" from sample in Edge Impulse
float raw_buf[] = {
  0.7706, 0.5977, 0.0771, 1.0000, 0.7297, 0.2049, 0.6497, 0.4179, 0.7697, 0.5950, 0.0693, 0.9616, 0.7230, 0.2049, 0.6497, 0.4664, 0.7697, 0.5939, 0.0624, 0.9041, 0.7230, 0.2049, 0.6497, 0.4963, 0.7679, 0.5891, 0.0648, 0.8681, 0.7162, 0.2049, 0.6433, 0.4627
};

void setup() {

  // Start serial
  Serial.begin(115200);
  Serial.println("Artificial nose inferencing");
}

void loop() {
  
  // Turn the raw buffer in a signal which we can the classify
  signal_t signal;
  int err = numpy::signal_from_buffer(raw_buf, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
  if (err != 0) {
      ei_printf("ERROR: Failed to create signal from buffer (%d)\r\n", err);
      return;
  }

  // Run inference
  ei_impulse_result_t result = {0};
  err = run_classifier(&signal, &result, debug_nn);
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

  delay(2000);
}
