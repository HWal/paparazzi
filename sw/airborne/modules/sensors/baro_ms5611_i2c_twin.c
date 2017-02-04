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
 * Measurement Specialties (Intersema) MS5611-01BA pressure/temperature sensors interface for I2C.
 * Read two sensors for calculating true airspeed and QNH based altitude.
 * ms5611 #1 reads the stagnation pressure, #2 reads the static pressure.
 * The static pressure reading is also used for true altitude calculation.
 */


#include "modules/sensors/baro_ms5611_i2c_twin.h"

#include "math/pprz_isa.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/abi.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
//Added lines for altitude calculation
#include "math.h"
#include "generated/flight_plan.h"

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
bool baro_ms5611_alt_valid;
bool baro_ms5611_enabled;

float baro_ms5611_r;
float baro_ms5611_sigma2;

// variables for true altitude calculation
float baro_ms5611_tcal = 0;                //Measured avg temperature on airstrip (K)
float baro_ms5611_pcal = 0;                //Measured avg pressure on airstrip (Pa)
float baro_ms5611_tmsl = BARO_MS5611_TMSL; //At init calculated temperature at MSL (K)
float baro_ms5611_pmsl = BARO_MS5611_PMSL; //At init calculated pressure at MSL (Pa)
float baro_cal_altitude = GROUND_ALT;      //Calculated true altitude
float ms5611_ground_alt = GROUND_ALT;      //Ground altitude from Flight plan
double const1, tmp1, tmp2;                 //Helper variables

// variables for differential pressure used in airspeed calculation
int32_t fixed_press_offset = 0;
bool offset_ok = false;
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
  ms5611_i2c_init(&baro_ms5611_1, &MS5611_I2C_DEV_1, MS5611_SLAVE_ADDR_1, false);
  ms5611_i2c_init(&baro_ms5611_2, &MS5611_I2C_DEV_2, MS5611_SLAVE_ADDR_2, false);

  baro_ms5611_enabled = true;
  baro_ms5611_alt_valid = false;

  // const1 used in true altitude calculation
  const1 = -G0 / (L * RGAS);

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
  if ((baro_ms5611_1.data_available) && (baro_ms5611_2.data_available)) {
    if (!offset_ok) {
      if (offset_count < 50) {
        if ((baro_ms5611_1.data.pressure > 90000) && (baro_ms5611_1.data.pressure < 110000)) {
          if ((baro_ms5611_2.data.pressure > 90000) && (baro_ms5611_2.data.pressure < 110000)) {
            // accumulate pressure diff values to calculate offset between the two barometers
            fixed_press_offset += baro_ms5611_1.data.pressure - baro_ms5611_2.data.pressure;
            // accumulate values to determine ref pressure and ref temperature on airstrip
            baro_ms5611_tcal += baro_ms5611_2.data.temperature;
            baro_ms5611_pcal += baro_ms5611_2.data.pressure;
            offset_count ++;
          }
        }
      }
      // averaging offset between the two barometers
      // averaging ref pressure and temperature on airstrip
      if (offset_count == 50) {
        fixed_press_offset /= 50;
        baro_ms5611_pcal /= 50;
        baro_ms5611_tcal /= 50;
        // convert temperature to Kelvin
        baro_ms5611_tcal = baro_ms5611_tcal / 100.0f + 273.15f;
        // calculate pressure and temperature at Mean Sea Level
        baro_ms5611_tmsl = baro_ms5611_tcal - ms5611_ground_alt * L;
        tmp1 = baro_ms5611_tmsl / baro_ms5611_tcal;
        baro_ms5611_pmsl = (pow (tmp1, const1)) * baro_ms5611_pcal;
        offset_ok = true;
      }
    }

    // send unfiltered pressure from ms5611 #2 to air_data
    float pressure = (float)baro_ms5611_2.data.pressure;
    AbiSendMsgBARO_ABS(BARO_MS5611_SENDER_ID, pressure);

    // simple Kalman filtering for airspeed measurement
    p = p + q;
    k = p / (p + r);
    x1 = x1 + k * ((float)baro_ms5611_1.data.pressure - x1);
    x2 = x2 + k * ((float)baro_ms5611_2.data.pressure - x2);
    p = (1. - k) * p;

    // calculate Kalman filtered diff pressure
    diff_press = x1 - x2;
    diff_press -= (float)fixed_press_offset;

    // scaling diff pressure
    diff_press *= (float)MS5611_AIRSPEED_SCALE_FACTOR;

    // send filtered diff pressure and unfiltered temp to air_data
    AbiSendMsgBARO_DIFF(BARO_MS5611_SENDER_ID, diff_press);
    float temp_2 = baro_ms5611_2.data.temperature / 100.0f;
    AbiSendMsgTEMPERATURE(BARO_MS5611_SENDER_ID, temp_2);

    // use filtered pressure from ms5611 #2 to calculate QNH based altitude
    tmp2 = x2 / baro_ms5611_pmsl;
    tmp2 = pow(tmp2, (1.0 / const1));
    baro_cal_altitude = (baro_ms5611_tmsl / L) * (tmp2 - 1);

    // send unfiltered pressure from ms5611 #2 to altitude calculation
    // here standard atmosphere is used, see sw/airborne/math/pprz_isa.h
    baro_ms5611_alt = pprz_isa_altitude_of_pressure(pressure);
    baro_ms5611_alt_valid = true;

    // get ready to receive new data from the baros
    baro_ms5611_1.data_available = false;
    baro_ms5611_2.data_available = false;

#ifdef SENSOR_SYNC_SEND
    // send filtered pressure and unfiltered temp from both sensors
    fbaroms_1 = x1 / 100.;
    fbaroms_2 = x2 / 100.;
    float temp_1 = baro_ms5611_1.data.temperature / 100.0f;
    DOWNLINK_SEND_BARO_MS5611(DefaultChannel, DefaultDevice,
                              &baro_ms5611_1.data.d1, &baro_ms5611_1.data.d2,
                              &fbaroms_1, &temp_1,
                              &baro_ms5611_2.data.d1, &baro_ms5611_2.data.d2,
                              &fbaroms_2, &temp_2,
                              &fixed_press_offset,
                              &baro_ms5611_tmsl,
                              &baro_ms5611_pmsl,
                              &baro_cal_altitude,
                              &baro_ms5611_alt);
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
