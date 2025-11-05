#include <Arduino.h>
#include <Jetson_SPI.h>
#include <Monitoring.h>
#include <Motors.h>
#include <Status_LED.h>

// Debug
#define ENABLE_SERIAL_DEBUG // Uncomment to enable serial debugging

// Read-only register addresses
#define FIRMWARE_VERSION_REG  0x00
#define BATTERY_VOLTAGE_REG   0x01
#define ESC_CURRENT_REG       0x02
#define JETSON_CURRENT_REG    0x03
#define TELEMETRY_DATA_REG    0x04
#define ARM_STATUS_REG        0x05
#define MOTOR_SPEEDS_REG      0x06
#define SPI_DATA_CRC_REG      0x07
#define DEBUG_STATUS_REG      0x08

// Constants
const uint8_t FIRMWARE_VERSION[2] = {1, 2}; // Firmware version 1.2

// Variables
bool serial_initialized = false;
bool debug_mode = false;
bool debug_led_state = false;

uint16_t motor_speeds[4];
bool motor_speeds_updated = false;

unsigned long prev_millis = 0;

/*!
  \brief Set motor speeds to 0 and indicate they were changed
*/
void zero_motor_speeds(void) {
  for (uint8_t i = 0; i < 4; i++) {
    motor_speeds[i] = 0;
  }
  motor_speeds_updated = true;
}

/*!
  \brief Read/write data based on the received SPI command
*/
void process_spi_cmd() {
  switch (jetson_spi.received_cmd) {
    case DEBUG_MODE_CMD: {
      // Only allow debug mode if the motors aren't currently armed
      debug_mode = (jetson_spi.rd_data[0] == 0x01 && !motors.armed);
      motors.debug_mode = debug_mode;

      // Reset motors when exiting debug mode to prevent potential crazyness
      if (!debug_mode) {
        motors.armed = false;
        zero_motor_speeds();
      }
      break;
    }
    case ESC_ARM_DISARM_CMD: {
      motors.armed = (jetson_spi.rd_data[0] == 0x01); // 1 to arm, 0 to disarm
      zero_motor_speeds(); // Turn motors off when arm state changes
      break;
    }
    case MOTOR_SPEEDS_CMD: {
      if (motors.armed) {
        uint16_t tmp_motor_speeds[4];
        bool valid_speeds = true;
        // Parse motor speeds from received data
        // Each motor speed is 2 bytes (high byte first)
        for (uint8_t i = 0; i < 4; i++) {
          tmp_motor_speeds[i] = (jetson_spi.rd_data[2*i] << 8) | jetson_spi.rd_data[2*i + 1];
          if (tmp_motor_speeds[i] > 1999) {
            valid_speeds = false;
            break;
          }
        }
        // Update motor speeds if all are valid
        if (valid_speeds) {
          for (uint8_t i = 0; i < 4; i++) {
            motor_speeds[i] = tmp_motor_speeds[i];
          }
          motor_speeds_updated = true;
        }
      }
      break;
    }
    case MOTOR_STOP_CMD: {
      zero_motor_speeds();
      break;
    }
    case SET_LED_STATE_CMD: {
      debug_led_state = (jetson_spi.rd_data[0] == 0x01); // 1 to turn status LED on (only in debug mode)
      break;
    }
    case READ_REGISTER_CMD: {
      uint8_t reg_addr = jetson_spi.rd_data[0];
      switch (reg_addr) {
        case FIRMWARE_VERSION_REG: {
          write_spi_data(FIRMWARE_VERSION, 2);
          break;
        }
        case BATTERY_VOLTAGE_REG: {
          const uint8_t reg_value[2] = {
            (uint8_t) ((monitoring.battery_voltage_mv >> 8) & 0xFF),
            (uint8_t) (monitoring.battery_voltage_mv & 0xFF)
          };
          write_spi_data(reg_value, 2);
          break;
        }
        case ESC_CURRENT_REG: {
          const uint8_t reg_value[2] = {
            (uint8_t) ((monitoring.esc_current_ma >> 8) & 0xFF),
            (uint8_t) (monitoring.esc_current_ma & 0xFF)
          };
          write_spi_data(reg_value, 2);
          break;
        }
        case JETSON_CURRENT_REG: {
          const uint8_t reg_value[2] = {
            (uint8_t) ((monitoring.jetson_current_ma >> 8) & 0xFF),
            (uint8_t) (monitoring.jetson_current_ma & 0xFF)
          };
          write_spi_data(reg_value, 2);
          break;
        }
        case TELEMETRY_DATA_REG: {
          const uint8_t reg_value[6] = {
            (uint8_t) ((monitoring.battery_voltage_mv >> 8) & 0xFF),
            (uint8_t) (monitoring.battery_voltage_mv & 0xFF),
            (uint8_t) ((monitoring.esc_current_ma >> 8) & 0xFF),
            (uint8_t) (monitoring.esc_current_ma & 0xFF),
            (uint8_t) ((monitoring.jetson_current_ma >> 8) & 0xFF),
            (uint8_t) (monitoring.jetson_current_ma & 0xFF)
          };
          write_spi_data(reg_value, 6);
          break;
        }
        case ARM_STATUS_REG: {
          const uint8_t reg_value[1] = {
            (uint8_t) (motors.armed ? 1 : 0)
          };
          write_spi_data(reg_value, 1);
          break;
        }
        case MOTOR_SPEEDS_REG: {
          uint8_t reg_value[8];
          for (uint8_t i = 0; i < 4; i++) {
            reg_value[2*i] = (motor_speeds[i] >> 8) & 0xFF;
            reg_value[2*i + 1] = motor_speeds[i] & 0xFF;
          }
          write_spi_data(reg_value, 8);
          break;
        }
        case SPI_DATA_CRC_REG: {
          const uint8_t reg_value[4] = {
            (uint8_t) ((jetson_spi.received_cmd >> 8) & 0xFF),
            (uint8_t) (jetson_spi.received_cmd & 0xFF),
            (uint8_t) ((jetson_spi.computed_crc >> 8) & 0xFF),
            (uint8_t) (jetson_spi.computed_crc & 0xFF)
          };
          write_spi_data(reg_value, 4);
          break;
        }
        case DEBUG_STATUS_REG: {
          const uint8_t reg_value[1] = {
            (uint8_t) (debug_mode ? 1 : 0)
          };
          write_spi_data(reg_value, 1);
          break;
        }
        default: {
          const uint8_t reg_value[2] = {0xB, 0xAD}; // Indicate invalid register with 0xBAD
          write_spi_data(reg_value, 2);
          break;
        }
      }
      break;
    }
    default: {
      break;
    }
  }
}

void setup() {
  #ifdef ENABLE_SERIAL_DEBUG
    Serial.begin(115200);
    unsigned long serial_timeout = millis() + 5000; // 5 second timeout
    while (!Serial && (millis() < serial_timeout));
    if (!Serial) {
      // Serial not initialized within timeout
      // Handle error (e.g., blink an LED)
    } else {
      serial_initialized = true;
      Serial.println("Serial initialized");
    }
  #endif

  // Initialize peripherals
  jetson_spi_init();
  monitoring_init();
  motors_init();
  status_led_init();
  #ifdef ENABLE_SERIAL_DEBUG
    if (serial_initialized) {
      Serial.println("Peripherals initialized successfully");
    }
  #endif

  prev_millis = millis();
}

void loop() {
  // Continously process data from peripherals
  process_spi_rx_data();
  process_adc_readings();

  // If a new command has been received, process it
  if (jetson_spi.rd_data_ready) {
    process_spi_cmd();
    jetson_spi.rd_data_ready = false;
  }

  #ifdef ENABLE_SERIAL_DEBUG
    if (serial_initialized && motor_speeds_updated) {
      Serial.print("Motor speeds: ");
      for (int i = 0; i < 4; i++) {
        Serial.print(motor_speeds[i]);
        Serial.print(" ");
      }
      Serial.println();
    }
  #endif

  // Update motor speeds if they have been changed
  if (motor_speeds_updated) {
    set_motor_speed(motor_speeds);
    motor_speeds_updated = false;
  }

  // Set LED state
  if (debug_mode) {
    status_led.active = debug_led_state;
  }
  else {
    status_led.active = motors.armed;
  }
  set_led_state();

  #ifdef ENABLE_SERIAL_DEBUG
    if (serial_initialized) {
      if (millis() - prev_millis > 1000) {
        prev_millis = millis();
        // Serial.print("Batt mV: ");
        // Serial.print(monitoring.battery_voltage_mv);
        // Serial.print("\tESC mA: ");
        // Serial.print(monitoring.esc_current_ma);
        // Serial.print("\tJetson mA: ");
        // Serial.println(monitoring.jetson_current_ma);
        for (int i = 0; i < 16; i++)
        {
          Serial.print(monitoring.adc_readings[i]);
          Serial.print(" ");
        }
        Serial.println();
      }
    }
  #endif
}