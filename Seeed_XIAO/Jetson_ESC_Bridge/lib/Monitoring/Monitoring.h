/*!
  \file Monitoring.h
  \brief Voltage/current monitoring for battery, ESC, and Jetson

  This library uses the ADC to read the battery voltage (from voltage divider),
  ESC current (from CURRENT pin of ESC), and Jetson current (from current-sense
  op-amp with shunt resistor)

  The readings are taken every 1ms. This is done by using a timer (TC3) configured
  to overflow every 1ms which generates an event that triggers the ADC to start sampling

  The 3 readings needed are converted to milli-values for their respective sensor type and
  stored in the monitoring struct
*/
#ifndef MONITORING_H
#define MONITORING_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint16_t battery_voltage_mv;
  uint16_t jetson_current_ma;
  uint16_t esc_current_ma;

  uint16_t adc_readings[16];
} Monitoring_t;

// Global instance of Monitoring
extern Monitoring_t monitoring;

// Functions
void monitoring_init(void);
void process_adc_readings(void);

#ifdef __cplusplus
}
#endif

#endif