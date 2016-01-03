/*
 * Copyright (C) 2011-2013 The Paparazzi Team
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
 * @file modules/sensors/baro_ms5611_i2c_twin.c
 * Measurement Specialties (Intersema) MS5611-01BA pressure/temperature sensor interface for I2C.
 *
 */


#include "modules/sensors/baro_ms5611_i2c_twin.h"

#include "math/pprz_isa.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/abi.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "math.h"


#ifndef MS5611_I2C_DEV_1
#define MS5611_I2C_DEV_1 i2c0
#endif
#ifndef MS5611_I2C_DEV_2
#define MS5611_I2C_DEV_2 i2c0
#endif

/* address can be 0xEC or 0xEE (CSB\ low = 0xEE) */
#ifndef MS5611_SLAVE_ADDR_1
#define MS5611_SLAVE_ADDR_1 0xEE
#endif
#ifndef MS5611_SLAVE_ADDR_2
#define MS5611_SLAVE_ADDR_2 0xEC
#endif
#ifndef MS5611_KALMAN_Q
#define MS5611_KALMAN_Q .5
#endif
#ifndef MS5611_KALMAN_R
#define MS5611_KALMAN_R 6.
#endif
#ifndef MS5611_AIRSPEED_SCALE_FACTOR
#define MS5611_AIRSPEED_SCALE_FACTOR 1.
#endif

struct Ms5611_I2c baro_ms5611_1;
struct Ms5611_I2c baro_ms5611_2;

float fbaroms_1, ftempms_1; // ftemps_1 not used?
float fbaroms_2, ftempms_2; // ftemps_2 not used?
float baro_ms5611_alt;
bool_t baro_ms5611_alt_valid;
bool_t baro_ms5611_enabled;

float baro_ms5611_r;
float baro_ms5611_sigma2;

// variables for differential pressure
int32_t fixed_press_offset = 0;
bool_t offset_ok = FALSE;
int offset_count = 0;
float diff_press = 0.;
float q = MS5611_KALMAN_Q;
float r = MS5611_KALMAN_R;
double p = 100.;
double k = 0.;
float x1 = 100000.;
float x2 = 100000.;

void baro_ms5611_twin_init(void)
{
  ms5611_i2c_init(&baro_ms5611_1, &MS5611_I2C_DEV_1, MS5611_SLAVE_ADDR_1, FALSE);
  ms5611_i2c_init(&baro_ms5611_2, &MS5611_I2C_DEV_2, MS5611_SLAVE_ADDR_2, FALSE);

  baro_ms5611_enabled = TRUE;
  baro_ms5611_alt_valid = FALSE;

  baro_ms5611_r = BARO_MS5611_R;
  baro_ms5611_sigma2 = BARO_MS5611_SIGMA2;
}

void baro_ms5611_twin_periodic_check(void)
{
  if (sys_time.nb_sec > 1) {
    ms5611_i2c_periodic_check(&baro_ms5611_1);
    ms5611_i2c_periodic_check(&baro_ms5611_2);

    #if SENSOR_SYNC_SEND
    // send coeff every 30s
    RunOnceEvery((30 * BARO_MS5611_TWIN_PERIODIC_CHECK_FREQ), baro_ms5611_twin_send_coeff());
    #endif
  }
}

/// trigger new measurement or initialize if needed
void baro_ms5611_twin_read(void)
{
  if (sys_time.nb_sec > 1.5) {
    ms5611_i2c_read(&baro_ms5611_1);
    ms5611_i2c_read(&baro_ms5611_2);
  }
}

void baro_ms5611_twin_event(void)
{
  ms5611_i2c_event(&baro_ms5611_1);
  ms5611_i2c_event(&baro_ms5611_2);

  // only if data is available from both baros
  if ((baro_ms5611_1.data_available) && (baro_ms5611_1.data_available)) {

    // accumulating offset values
    if (!offset_ok) {
      if (offset_count < 50) {
        if ((baro_ms5611_1.data.pressure > 90000) && (baro_ms5611_1.data.pressure < 110000)) {
          if ((baro_ms5611_2.data.pressure > 90000) && (baro_ms5611_2.data.pressure < 110000)) {
            fixed_press_offset += baro_ms5611_1.data.pressure - baro_ms5611_2.data.pressure;
            offset_count ++;
          }
        }
      }
      // averaging offset
      if (offset_count == 50) {
        fixed_press_offset /= 50;
        offset_ok = TRUE;
      }
    }

    // send unfiltered pressure from ms5611 #1 to air_data
    float pressure = (float)baro_ms5611_1.data.pressure;
    AbiSendMsgBARO_ABS(BARO_MS5611_SENDER_ID, pressure);

    // simple Kalman filtering for airspeed measurement
    p = p + q;
    k = p / (p + r);
    x1 = x1 + k * ((float)baro_ms5611_1.data.pressure - x1);
    x2 = x2 + k * ((float)baro_ms5611_2.data.pressure - x2);
    p = (1. - k) * p;

    // calculate filtered diff pressure
    diff_press = x1 - x2;
    diff_press -= (float)fixed_press_offset;

    // scaling diff pressure
    diff_press *= (float)MS5611_AIRSPEED_SCALE_FACTOR;

    // send filtered diff pressure and unfiltered temp to air_data
    AbiSendMsgBARO_DIFF(BARO_MS5611_SENDER_ID, diff_press);
    float temp_1 = baro_ms5611_1.data.temperature / 100.0f;
    AbiSendMsgTEMPERATURE(BARO_MS5611_SENDER_ID, temp_1);
    baro_ms5611_1.data_available = FALSE;
    baro_ms5611_2.data_available = FALSE;

    // send pressure from ms5611 #1 to altitude calculation
    baro_ms5611_alt = pprz_isa_altitude_of_pressure(pressure);
    baro_ms5611_alt_valid = TRUE;

#ifdef SENSOR_SYNC_SEND
    // send filtered pressure and unfiltered temp from both sensors
    fbaroms_1 = x1 / 100.;
    fbaroms_2 = x2 / 100.;
    float temp_2 = baro_ms5611_2.data.temperature / 100.0f;
    DOWNLINK_SEND_BARO_MS5611(DefaultChannel, DefaultDevice,
                              &baro_ms5611_1.data.d1, &baro_ms5611_1.data.d2,
                              &fbaroms_1, &temp_1,
                              &baro_ms5611_2.data.d1, &baro_ms5611_2.data.d2,
                              &fbaroms_2, &temp_2,
                              &fixed_press_offset);
#endif
  }
}

void baro_ms5611_twin_send_coeff(void)
{
  if ((baro_ms5611_1.initialized) || (baro_ms5611_2.initialized)) {
    DOWNLINK_SEND_MS5611_COEFF(DefaultChannel, DefaultDevice,
                               &baro_ms5611_1.data.c[0],
                               &baro_ms5611_1.data.c[1],
                               &baro_ms5611_1.data.c[2],
                               &baro_ms5611_1.data.c[3],
                               &baro_ms5611_1.data.c[4],
                               &baro_ms5611_1.data.c[5],
                               &baro_ms5611_1.data.c[6],
                               &baro_ms5611_1.data.c[7],
                               &baro_ms5611_2.data.c[0],
                               &baro_ms5611_2.data.c[1],
                               &baro_ms5611_2.data.c[2],
                               &baro_ms5611_2.data.c[3],
                               &baro_ms5611_2.data.c[4],
                               &baro_ms5611_2.data.c[5],
                               &baro_ms5611_2.data.c[6],
                               &baro_ms5611_2.data.c[7]);
  }
}
