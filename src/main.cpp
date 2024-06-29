/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */
#include "globals.h"
#include "USBSerial.h"
#include "mbed.h"
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "vl53l8cx.h"
// #include "vl53l8cx_api.h"
// #include "vl53l8cx_plugin_detection_thresholds.h"
// #include "vl53l8cx_plugin_motion_indicator.h"

#define LOOP_WAIT_TIME 500ms

SPI spi(D4, D5, D3); // mosi, miso, sclk
DigitalOut cs(D10);



DigitalInOut pwren(D11);

#define HIGH 0x1
#define LOW 0x0

class VL53L8CX {
public:
  VL53L8CX(SPI *spi, DigitalOut cs) {
    memset((void *)&_dev, 0x0, sizeof(VL53L8CX_Configuration));
    _dev.platform.address = VL53L8CX_DEFAULT_I2C_ADDRESS;
    _dev.device.comms_type = 0; // SPI
    // _dev.platform.spi = spi;
    // _dev.platform.cs = cs;
    // _dev.platform.dev_i2c = NULL;
    // _dev.platform.dev_spi = spi;
    // _dev.platform.cs_pin = cs_pin;
    // _dev.platform.spi_speed = spi_speed ;
    // _dev.platform.lpn_pin = lpn_pin;
    // _dev.platform.i2c_rst_pin = i2c_rst_pin;
    p_dev = &_dev;
    _spi = spi;
    _cs = cs;
  }

  virtual ~VL53L8CX() {}

  virtual void reset() {
    HAL_GPIO_WritePin(NCS_C_GPIO_Port, NCS_Pin_C, GPIO_PIN_RESET);

	/* Toggle EVK PWR EN board and Lpn pins */
	// HAL_GPIO_WritePin(LPn_C_GPIO_Port, LPn_C_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PWR_EN_C_GPIO_Port, PWR_EN_C_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);

	// HAL_GPIO_WritePin(LPn_C_GPIO_Port, LPn_C_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PWR_EN_C_GPIO_Port, PWR_EN_C_Pin, GPIO_PIN_SET);
	HAL_Delay(100);


    // HAL_GPIO_WritePin(CUSTOM_VL53L8CX_PWR_EN_PORT,
    // CUSTOM_VL53L8CX_PWR_EN_PIN, GPIO_PIN_RESET); HAL_Delay(2);
    // HAL_GPIO_WritePin(CUSTOM_VL53L8CX_PWR_EN_PORT,
    // CUSTOM_VL53L8CX_PWR_EN_PIN, GPIO_PIN_SET); HAL_Delay(2);

    // HAL_GPIO_WritePin(CUSTOM_VL53L8CX_LPn_PORT, CUSTOM_VL53L8CX_LPn_PIN,
    // GPIO_PIN_RESET); HAL_Delay(2);
  }

  virtual int begin() {
    if (_spi) {
      _cs = HIGH;
    }
    return 0;
  }

  virtual int end() {
    // set cs to output ?
    return 0;
  }

  virtual void vl53l8cx_on(void) {
    // lpn_pin = HIGHT
    HAL_Delay(10);
  }

  virtual void vl53l8cx_off(void) {
    // lpn_pin = LOW
    HAL_Delay(10);
  }

  virtual int init_sensor() {
    uint8_t status = VL53L8CX_STATUS_OK;
    uint8_t isAlive = 0;

    // vl53l8cx_on();
    // vl53l8cx_off();

    // if i2c, reset i2c
    
    
    printf("Checking alive ...\r\n");

    status = vl53l8cx_is_alive(&_dev, &isAlive);
    if (!isAlive || status != VL53L8CX_STATUS_OK) {
      printf("VL53L8CX with SPI communication not detected, error : %d\n",
             status);
      return VL53L8CX_STATUS_ERROR;
    }

    printf("Sensor initializing, please wait few seconds\n");

    status = vl53l8cx_init(&_dev);
    if (status != VL53L8CX_STATUS_OK) {
      printf("Init failed with status %d\n", status);
    }
    return (int)status;
  }

  virtual uint8_t start_ranging() {

    uint8_t status = vl53l8cx_set_ranging_frequency_hz(
        &_dev, 5); // Set 5Hz ranging frequency
    status = vl53l8cx_set_ranging_mode(
        &_dev, VL53L8CX_RANGING_MODE_AUTONOMOUS); // Set mode autonomous

    printf("Ranging starts\n");
    status = vl53l8cx_start_ranging(&_dev);

    return status;
  }

  virtual uint8_t check_data_ready(uint8_t *p_isReady) {
    return vl53l8cx_check_data_ready(&_dev, p_isReady);
  }

  virtual uint8_t get_ranging_data(VL53L8CX_ResultsData *p_results) {
    return vl53l8cx_get_ranging_data(&_dev, p_results);
  }

protected:
  SPI *_spi;
  DigitalOut _cs = DigitalOut(D10);
  VL53L8CX_Configuration _dev;
  VL53L8CX_Configuration *p_dev;
};

// ---------------------------------------------------------------------------


bool EnableAmbient = true;
bool EnableSignal = true;
uint8_t res = VL53L8CX_RESOLUTION_4X4;


// #define GPIO_PIN_RESET  0
// #define GPIO_PIN_SET 1

// ---------------------------------------------------------------------------

// void display_commands_banner(void) {
//   /* 27 is ESC command */
//   // printf("%c[2H", 27);

//   printf("53L8A1 Simple Ranging demo application\n");
//   printf("--------------------------------------\n\n");

//   printf("Use the following keys to control application\n");
//   printf(" 'r' : change resolution\n");
//   printf(" 's' : enable signal and ambient\n");
//   printf(" 'c' : clear screen\n");
//   printf("\n");
// }

// void print_result(VL53L8CX_ResultsData *Result) {
//   int8_t i, j, k, l;
//   uint8_t zones_per_line;
//   uint8_t number_of_zones = res;

//   zones_per_line = (number_of_zones == 16) ? 4 : 8;

//   display_commands_banner();

//   printf("Cell Format :\n\n");

//   for (l = 0; l < VL53L8CX_NB_TARGET_PER_ZONE; l++) {
//     printf(" \033[38;5;10m%20s\033[0m : %20s\n", "Distance [mm]", "Status");

//     if (EnableAmbient || EnableSignal) {
//       printf(" %20s : %20s\n", "Signal [kcps/spad]", "Ambient [kcps/spad]");
//     }
//   }

//   printf("\n\n");

//   for (j = 0; j < number_of_zones; j += zones_per_line) {
//     for (i = 0; i < zones_per_line; i++)
//       printf(" -----------------");

//     printf("\n");

//     for (i = 0; i < zones_per_line; i++)
//       printf("|                 ");

//     printf("|\n");

//     for (l = 0; l < VL53L8CX_NB_TARGET_PER_ZONE; l++) {
//       // Print distance and status
//       for (k = (zones_per_line - 1); k >= 0; k--) {
//         if (Result->nb_target_detected[j + k] > 0) {
//           printf(
//               "| \033[38;5;10m%5ld\033[0m  :  %5ld ",
//               (long)Result
//                   ->distance_mm[(VL53L8CX_NB_TARGET_PER_ZONE * (j + k)) + l],
//               (long)Result
//                   ->target_status[(VL53L8CX_NB_TARGET_PER_ZONE * (j + k)) + l]);

//         } else {
//           printf("| %5s  :  %5s ", "X", "X");
//         }
//       }
//       printf("|\n");

//       if (EnableAmbient || EnableSignal) {
//         // Print Signal and Ambient
//         for (k = (zones_per_line - 1); k >= 0; k--) {
//           if (Result->nb_target_detected[j + k] > 0) {
//             if (EnableSignal) {
//               printf("| %5ld  :  ",
//                      (long)Result->signal_per_spad
//                          [(VL53L8CX_NB_TARGET_PER_ZONE * (j + k)) + l]);
//             } else {
//               printf("| %5s  :  ", "X");
//             }
//             if (EnableAmbient) {
//               printf("%5ld ", (long)Result->ambient_per_spad[j + k]);
//             } else {
//               printf("%5s ", "X");
//             }
//           } else {
//             printf("| %5s  :  %5s ", "X", "X");
//           }
//         }
//         printf("|\n");
//       }
//     }
//   }
//   for (i = 0; i < zones_per_line; i++)
//     printf(" -----------------");
//   printf("\n");
// }


int main() {

  // setup:

  if (pwren.is_connected()) {
    printf("pwren is initialized and connected!\r\n");
  }

  pwren.output();
  pwren = HIGH;
  HAL_Delay(10);

  printf("PWREN: %d\r\n", pwren.read());

  MX_SPI1_Init();
  printf("[OK] - MX_SPI1_Init\r\n");

  // -- Test whoami

  // cs.output();
  // cs = HIGH;

  // spi.format(8, 3);
  // spi.frequency(5000000);

  // cs = LOW;

  // spi.write(0x8F);

  // int whoami = spi.write(0x00);
  // printf("WHOAMI register = 0x%X\n", whoami);

  // // Deselect the device
  // cs = HIGH;

  // -- VL53L8CX Begin
  // lpn_pin = LOW;

  
  VL53L8CX sensor_vl53l8cx_top(&spi, cs);

  sensor_vl53l8cx_top.reset(); // Reset_Sensor(&(Dev.platform));
  printf("[OK] - reset\r\n");
  sensor_vl53l8cx_top.init_sensor();
  printf("[OK] - init_sensor\r\n");
  // sensor_vl53l8cx_top.begin();

  // sensor_vl53l8cx_top.begin();
//   sensor_vl53l8cx_top.start_ranging();

  // // loop
  // VL53L8CX_ResultsData Results;
  // uint8_t NewDataReady = 0;
  // uint8_t status;

  // do {
  //   status = sensor_vl53l8cx_top.check_data_ready(&NewDataReady);
  // } while (!NewDataReady);

  // if ((!status) && (NewDataReady != 0)) {
  //   status = sensor_vl53l8cx_top.get_ranging_data(&Results);
  //   print_result(&Results);
  // }

  HAL_Delay(50);
  printf("maybe got here ? \r\n");

  // while (true) {
  //     printf("loop\r\n");
  //     ThisThread::sleep_for(LOOP_WAIT_TIME);
  // }
}