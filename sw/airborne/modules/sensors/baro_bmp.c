/*
 * Copyright (C) 2010 Martin Mueller
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file modules/sensors/baro_bmp.c
 * Bosch BMP085 I2C sensor interface.
 *
 * This reads the values for pressure and temperature from the Bosch BMP085 sensor through I2C.
 */


#include "baro_bmp.h"
#include "peripherals/bmp085_regs.h"

#include "mcu_periph/sys_time.h"
#include "mcu_periph/i2c.h"
#include "led.h"
#include "mcu_periph/uart.h"
#include "subsystems/abi.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
//Added lines by HW
#include "generated/flight_plan.h"
//End of added lines by HW


#ifndef BMP_I2C_DEV
#define BMP_I2C_DEV i2c0
#endif

#define BARO_BMP_R 0.5
#define BARO_BMP_SIGMA2 0.1


struct Bmp085 baro_bmp;

bool_t baro_bmp_enabled;
float baro_bmp_r;
float baro_bmp_sigma2;
int32_t baro_bmp_alt;

//Added lines by HW
bool_t bmp_calib;
int16_t cal_cnt, bmp_cnt;
float baro_bmp_tcal;     //Measured averaged temperature for calculation to MSL (K)
float baro_bmp_pcal;     //Measured averaged pressure for calculation to MSL (Pa)
float baro_bmp_tmsl;     //Init calculated temperature at MSL (K)
float baro_bmp_pmsl;     //Init calculated pressure at MSL (Pa)
float bmp_p_average;     //Averaged pressure (4 readings)
float baro_cal_altitude; //Calculated true altitude
float bmp_ground_alt;    //Ground altitude from Flight plan
double const1, tmp1, tmp2;
float vario_time_start, vario_altitude_start; //Variometer values
float vario_value, vario_time_interval;       //Variometer values
//End of added lines by HW

void baro_bmp_init(void)
{

  bmp085_init(&baro_bmp, &BMP_I2C_DEV, BMP085_SLAVE_ADDR);

  baro_bmp_r = BARO_BMP_R;
  baro_bmp_sigma2 = BARO_BMP_SIGMA2;
  baro_bmp_enabled = TRUE;

//Added lines by HW
bmp_ground_alt = GROUND_ALT;
baro_cal_altitude = GROUND_ALT;
vario_time_interval = .5;
baro_bmp_pcal = 0;
baro_bmp_tcal = 0;
baro_bmp_pmsl = BARO_BMP_PMSL;
baro_bmp_tmsl = BARO_BMP_TMSL;
bmp_calib = FALSE;
const1 = -G0/(L * RGAS);
cal_cnt = 0;
bmp_cnt = 0;
bmp_p_average = 0;
//End of added lines by HW

}

void baro_bmp_periodic(void)
{

  if (baro_bmp.initialized) {
    bmp085_periodic(&baro_bmp);
  } else {
    bmp085_read_eeprom_calib(&baro_bmp);
  }

}

void baro_bmp_event(void)
{

  bmp085_event(&baro_bmp);

  if (baro_bmp.data_available) {

    float tmp = baro_bmp.pressure / 101325.0; // pressure at sea level
    tmp = pow(tmp, 0.190295);
    baro_bmp_alt = 44330 * (1.0 - tmp);

    float pressure = (float)baro_bmp.pressure;
    AbiSendMsgBARO_ABS(BARO_BMP_SENDER_ID, pressure);
    float temp = baro_bmp.temperature / 10.0f;
    AbiSendMsgTEMPERATURE(BARO_BOARD_SENDER_ID, temp);
    baro_bmp.data_available = FALSE;

   //Added lines by HW
    //Calculate QNH and temperature at MSL once using barometric formula
    //http://en.wikipedia.org/wiki/Barometric_pressure
    if (bmp_calib == FALSE) {
      cal_cnt++;
      if ((cal_cnt > 0 ) && (cal_cnt <= 16)) {
        baro_bmp_pcal += air_data.pressure;
        baro_bmp_tcal += air_data.temperature;
      }
      if (cal_cnt == 16) {
        baro_bmp_pcal /= 16.;
        baro_bmp_tcal /= 16.;
        baro_bmp_tcal += 273.15;
        baro_bmp_tmsl = baro_bmp_tcal - bmp_ground_alt * L;
        tmp1 = baro_bmp_tmsl / baro_bmp_tcal;
        baro_bmp_pmsl = (pow(tmp1, const1)) * baro_bmp_pcal;
        bmp_calib = TRUE;
        cal_cnt = 0;
      }
    }

    //Get start conditions for variometer calculation
    if (bmp_cnt == 0) {
      vario_altitude_start = baro_cal_altitude;
      vario_time_start = (float)sys_time.nb_sec + ((float)sys_time.nb_sec_rem / (float)sys_time.cpu_ticks_per_sec);
    }

    bmp_cnt++;
    bmp_p_average += air_data.pressure;

    //Calculate QNH based altitude and variometer value (climb rate) based on 8 averaged pressure readings
    if (bmp_cnt == 8) {
      bmp_p_average /= 8.;
      tmp2 = bmp_p_average / baro_bmp_pmsl;
      tmp2 = pow(tmp2, (1.0 / const1));
      baro_cal_altitude = (baro_bmp_tmsl / L) * (tmp2 - 1);
      vario_time_interval = (float)sys_time.nb_sec + ((float)sys_time.nb_sec_rem / (float)sys_time.cpu_ticks_per_sec) - vario_time_start;
      vario_value = (baro_cal_altitude - vario_altitude_start) / vario_time_interval;
      bmp_p_average = 0;
      bmp_cnt = 0;
    }
    //End of added lines by HW

#ifdef SENSOR_SYNC_SEND
    DOWNLINK_SEND_BMP_STATUS(DefaultChannel, DefaultDevice, &baro_bmp.up,
                             &baro_bmp.ut, &baro_bmp.pressure,
                             &baro_bmp.temperature);
#else
    RunOnceEvery(10, DOWNLINK_SEND_BMP_STATUS(DefaultChannel, DefaultDevice,
                 &baro_bmp.up, &baro_bmp.ut,
                 &baro_bmp.pressure,
                 &baro_bmp.temperature));
    //Added lines by HW
    RunOnceEvery(5, DOWNLINK_SEND_BMP_ALTITUDE(DefaultChannel, DefaultDevice,
                                              &baro_bmp_tcal,
                                              &baro_bmp_pcal,
                                              &baro_bmp_tmsl,
                                              &baro_bmp_pmsl,
                                              &baro_cal_altitude,
                                              &baro_bmp_alt,
                                              &bmp_ground_alt,
                                              &vario_value,
                                              &vario_time_interval));
    //End of added lines by HW
#endif
  }
}
