#include "Monitoring.h"

#define TC3_CYCLES_PER_MS 750 // Number of timer cycles for a 1ms period with a prescaler of 64 (1ms / (1 / (48MHz / 64)) = 750)

#define NUM_ADC_INPUTS 16 // Number of inputs the ADC will sample. Need 16 inputs to sample pins 2, 3, and 17
#define ADC_VREF (3.3 / 1.48) // ADC V_REF is VDDANA / 1.48
#define ADC_LSB_MV (1000 * ADC_VREF / (2 << ADC_RESOLUTION)) // Compute ADC LSB in milli-volts

#define BATTERY_VOLTAGE_DIVISION (4.1 / (33 + 4.1)) // Battery voltage is read through a voltage divider with R1=33k and R2=4.1k (8.2k || 8.2k)
#define ESC_CURRENT_V_TO_A_RATIO 120
#define CURRENT_SENSE_OP_AMP_GAIN 50
#define SHUNT_RESISTANCE 0.01 // Resistance of the current-sense shunt resistor in ohms
/*
  Array indices for the needed readings
  Array starts at AIN[2] and ends at AIN[17]
*/
#define JETSON_CURRENT_ARR_I 0
#define BATTERY_VOLTAGE_ARR_I 1
#define ESC_CURRENT_ARR_I 15

// Initialize the Monitoring instance
Monitoring_t monitoring = {
  .battery_voltage_mv = 0,
  .jetson_current_ma = 0,
  .esc_current_ma = 0,
  .adc_readings = {}
};

// Constants
static const uint16_t VOLTAGE_LSB_MV = ADC_LSB_MV / BATTERY_VOLTAGE_DIVISION;
static const uint16_t JETSON_CURRENT_LSB_MA = ADC_LSB_MV / (CURRENT_SENSE_OP_AMP_GAIN * SHUNT_RESISTANCE);
static const uint16_t ESC_CURRENT_LSB_MA = ESC_CURRENT_V_TO_A_RATIO * ADC_LSB_MV;

// Variables
static uint16_t adc_results[NUM_ADC_INPUTS];
static uint8_t adc_result_index = 0;
static bool sampling_complete = false;

// ----------------------------------------------------------------------
// PRIVATE FUNCTIONS
// ----------------------------------------------------------------------
/*!
  \brief Configure the pins to be used as ADC inputs

  Jetson Current:   PB08 (AIN[2])
  Battery Voltage:  PB09 (AIN[3])
  ESC Current:      PA09 (AIN[17])
*/
static void configure_pins(void) {
  // Enable pin mux
  PORT->Group[PORTB].PINCFG[8].bit.PMUXEN = 1;
  PORT->Group[PORTB].PINCFG[9].bit.PMUXEN = 1;
  PORT->Group[PORTA].PINCFG[9].bit.PMUXEN = 1;

  // Select pin mux function
  PORT->Group[PORTB].PMUX[4].bit.PMUXE = MUX_PB08B_ADC_AIN2;
  PORT->Group[PORTB].PMUX[4].bit.PMUXO = MUX_PB09B_ADC_AIN3;
  PORT->Group[PORTA].PMUX[4].bit.PMUXO = MUX_PA09B_ADC_AIN17;
}

/*!
  \brief Connect the TC3 overflow event to the ADC start conversion
*/
static void configure_event_system(void) {
  // Enable EVSYS clocks
  PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;                      // Enable APBC clock for ADC
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EVSYS_CHANNEL_0) |  // Select the clock for channel 0 of event system
                      GCLK_CLKCTRL_GEN_GCLK0 |                // Use generic clock generator 0
                      GCLK_CLKCTRL_CLKEN;                     // Enable the clock
  while (GCLK->STATUS.bit.SYNCBUSY);

  // Reset event system
  EVSYS->CTRL.bit.SWRST = 1;
  
  // Configure ADC start conversion event to be triggered by event channel 0
  EVSYS->USER.reg |=  EVSYS_USER_CHANNEL(0) |   // Select user channel 0
                      EVSYS_ID_USER_ADC_START;  // Connect to ADC start conversion
  
  // Configure event channel 0 to be triggered by TC3 overflow
  EVSYS->CHANNEL.reg |= EVSYS_CHANNEL_PATH_ASYNCHRONOUS |           // Asynchronous path
                        EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_TC3_OVF) | // Trigger on TC3 overflow
                        EVSYS_CHANNEL_CHANNEL(0);                   // Use channel 0
}

/*!
  \brief Configure the ADC to sample 16 inputs on a TC3 overflow event

  Total clock cycles needed for sampling = 7 cycles/sample * 4 samples/input * 16 inputs = 448 cycles

  f_CLK_ADC = 48MHz / 32 = 1.5 MHz

  Total sampling time = 448 cycles / 1.5 MHz ~= 300us
*/
static void configure_adc(void) {
  // Enable ADC clocks
  PM->APBCMASK.reg |= PM_APBCMASK_ADC;            // Enable APBC clock for ADC
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_ADC) |  // Select the ADC clock
                      GCLK_CLKCTRL_GEN_GCLK0 |    // Use generic clock generator 0
                      GCLK_CLKCTRL_CLKEN;         // Enable the clock
  while (GCLK->STATUS.bit.SYNCBUSY);

  // Disable and reset the ADC
  ADC->CTRLA.bit.ENABLE = 0;
  ADC->CTRLA.bit.SWRST = 1;
  while (ADC->CTRLA.bit.SWRST || ADC->STATUS.bit.SYNCBUSY);

  /*
    Prescale GCLK_ADC clock by 32 to give a 1.5 MHz clock for CLK_ADC
    f_CLK_ADC maximum frequency is 2100 kHz according to the datasheet
  */
  ADC->CTRLB.reg |= ADC_CTRLB_PRESCALER_DIV32 | // Divide GCLK_ADC by 32 for CLK_ADC
                    ADC_CTRLB_RESSEL_16BIT;     // Use 16-bits for result because of averaging
  while (ADC->STATUS.bit.SYNCBUSY);
  
  // Configure ADC operation
  ADC->REFCTRL.reg |= ADC_REFCTRL_REFSEL_INTVCC0; // Use 2.23V as ADC reference (1/1.48 VDDANA)
  ADC->AVGCTRL.reg |= ADC_AVGCTRL_ADJRES(2) |     // Set divison coefficient to 4
                      ADC_AVGCTRL_SAMPLENUM_4;    // Take 4 samples per reading for averaging
  ADC->INPUTCTRL.reg |= ADC_INPUTCTRL_INPUTSCAN(NUM_ADC_INPUTS-1) | // Unfortunately due to pin selection we need to convert 16 inputs to get the 3 we need
                        ADC_INPUTCTRL_MUXPOS_PIN2;                  // Start the conversion at analog input 2
  while (ADC->STATUS.bit.SYNCBUSY);
  
  // Configure ADC trigger/interrupts
  ADC->EVCTRL.bit.STARTEI = 1; // Allow event to kick off conversion
  ADC->INTENSET.bit.RESRDY = 1; // Enable result ready interrupt
  
  // Enable the ADC
  ADC->CTRLA.bit.ENABLE = 1;
  while (ADC->STATUS.bit.SYNCBUSY);
}

/*!
  \brief Configure TC3 to generate an overflow event at 1kHz to trigger ADC conversions

  Timer frequency = 48MHz / 64 / 750 = 1kHz (1ms period)
*/
static void configure_timer(void) {
  // Enable TC3 clocks
  PM->APBCMASK.reg |= PM_APBCMASK_TC3;                // Enable APBC clock for TC3
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_TCC2_TC3) | // Select the TCC2 and TC3 clock
                      GCLK_CLKCTRL_GEN_GCLK0 |        // Use generic clock generator 0
                      GCLK_CLKCTRL_CLKEN;             // Enable the clock
  while (GCLK->STATUS.bit.SYNCBUSY);
  
  // Reset TC3 and wait for synchronization
  TC3->COUNT16.CTRLA.bit.ENABLE = 0;
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
  TC3->COUNT16.CTRLA.bit.SWRST = 1;
  while (TC3->COUNT16.CTRLA.bit.SWRST || TC3->COUNT16.STATUS.bit.SYNCBUSY);
  
  // Configure TC3 for 1kHz operation
  TC3->COUNT16.CTRLA.reg =  TC_CTRLA_PRESCALER_DIV64 |  // Set prescaler to 64
                            TC_CTRLA_WAVEGEN_MFRQ |     // Match Frequency Generation to control period with CC
                            TC_CTRLA_MODE_COUNT16;      // Use 16-bit counter mode
  TC3->COUNT16.CC->reg = TC3_CYCLES_PER_MS;             // Set CC to give 1kHz frequency
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
  
  // Enable TC3 overflow event output
  TC3->COUNT16.EVCTRL.bit.OVFEO = 1;
  
  // Enable TC3
  TC3->COUNT16.CTRLA.bit.ENABLE = 1;
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
}

// ----------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ----------------------------------------------------------------------
/*!
  \brief Initialize the monitoring system
*/
void monitoring_init(void) {
  NVIC_DisableIRQ(ADC_IRQn);
  NVIC_ClearPendingIRQ(ADC_IRQn);
  NVIC_SetPriority(ADC_IRQn, 1);

  configure_pins();
  configure_event_system();
  configure_adc();
  configure_timer();

  NVIC_EnableIRQ(ADC_IRQn);
}

/*!
  \brief Convert the ADC readings to the voltage/current units
*/
void process_adc_readings(void) {
  if (sampling_complete) {
    sampling_complete = false;
    monitoring.battery_voltage_mv = VOLTAGE_LSB_MV * adc_results[BATTERY_VOLTAGE_ARR_I];
    monitoring.jetson_current_ma = JETSON_CURRENT_LSB_MA * adc_results[JETSON_CURRENT_ARR_I];
    monitoring.esc_current_ma = ESC_CURRENT_LSB_MA * adc_results[ESC_CURRENT_ARR_I];
    for (int i = 0; i < NUM_ADC_INPUTS; i++) {
      // monitoring.adc_readings[i] = adc_results[i];
      monitoring.adc_readings[i] = 100;
    }
  }
}
// ----------------------------------------------------------------------
// INTERRUPT HANDLER
// ----------------------------------------------------------------------
void ADC_Handler(void) {
  // Add ADC result to the results array on Result Ready interrupt
  if (ADC->INTFLAG.bit.RESRDY) {
    adc_results[adc_result_index++] = ADC->RESULT.reg;
    if (adc_result_index == NUM_ADC_INPUTS) {
      adc_result_index = 0;
      sampling_complete = true;
    }
  }
}