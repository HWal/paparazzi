/*
 * Copyright (C) 2010 Antoine Drouin <poinix@gmail.com>
 *               2013 Felix Ruess <felix.ruess@gmail.com>
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
 */

/**
 * @file subsystems/imu/imu_sen10121.c
 *
 * Driver for the Sparkfun SEN-10121 IMU.
 *
 * 6DOF IMU with ITG-3200 and ADXL345, via I2C.
 */

#include "subsystems/imu.h"
#include "subsystems/abi.h"
#include "mcu_periph/i2c.h"

/* i2c default suitable for Tiny/Twog */
#ifndef IMU_SEN10121_I2C_DEV
#define IMU_SEN10121_I2C_DEV i2c0
#endif

/** adxl345 accelerometer output rate, lowpass is set to half of rate */
#ifndef IMU_SEN10121_ACCEL_RATE
#  if PERIODIC_FREQUENCY <= 60
#    define IMU_SEN10121_ACCEL_RATE ADXL345_RATE_50HZ
#  elif PERIODIC_FREQUENCY <= 120
#    define IMU_SEN10121_ACCEL_RATE ADXL345_RATE_100HZ
#  else
#    define IMU_SEN10121_ACCEL_RATE ADXL345_RATE_200HZ
#  endif
#endif
PRINT_CONFIG_VAR(IMU_SEN10121_ACCEL_RATE)


/** gyro internal lowpass frequency */
#ifndef IMU_SEN10121_GYRO_LOWPASS
#  if PERIODIC_FREQUENCY <= 60
#    define IMU_SEN10121_GYRO_LOWPASS ITG3200_DLPF_20HZ
#  elif PERIODIC_FREQUENCY <= 120
#    define IMU_SEN10121_GYRO_LOWPASS ITG3200_DLPF_42HZ
#  else
#    define IMU_SEN10121_GYRO_LOWPASS ITG3200_DLPF_98HZ
#  endif
#endif
PRINT_CONFIG_VAR(IMU_SEN10121_GYRO_LOWPASS)


/** gyro sample rate divider */
#ifndef IMU_SEN10121_GYRO_SMPLRT_DIV
#  if PERIODIC_FREQUENCY <= 60
#    define IMU_SEN10121_GYRO_SMPLRT_DIV 19
PRINT_CONFIG_MSG("Gyro output rate is 50Hz")
#  else
#    define IMU_SEN10121_GYRO_SMPLRT_DIV 9
PRINT_CONFIG_MSG("Gyro output rate is 100Hz")
#  endif
#endif
PRINT_CONFIG_VAR(IMU_SEN10121_GYRO_SMPLRT_DIV)


struct ImuSen10121 imu_sen10121;

void imu_sen10121_init(void)
{
  /* Set accel configuration */
  adxl345_i2c_init(&imu_sen10121.acc_adxl, &(IMU_SEN10121_I2C_DEV), ADXL345_ADDR);
  /* set the data rate */
  imu_sen10121.acc_adxl.config.rate = IMU_SEN10121_ACCEL_RATE;

  /* Gyro configuration and initalization */
  itg3200_init(&imu_sen10121.gyro_itg, &(IMU_SEN10121_I2C_DEV), ITG3200_ADDR);
  /* change the default config */
  imu_sen10121.gyro_itg.config.smplrt_div = IMU_SEN10121_GYRO_SMPLRT_DIV;
  imu_sen10121.gyro_itg.config.dlpf_cfg = IMU_SEN10121_GYRO_LOWPASS;
}


void imu_sen10121_periodic(void)
{
  adxl345_i2c_periodic(&imu_sen10121.acc_adxl);

  // Start reading the latest gyroscope data
  itg3200_periodic(&imu_sen10121.gyro_itg);
}

void imu_sen10121_event(void)
{
  uint32_t now_ts = get_sys_time_usec();

  adxl345_i2c_event(&imu_sen10121.acc_adxl);
  if (imu_sen10121.acc_adxl.data_available) {
    imu.accel_unscaled.x = -imu_sen10121.acc_adxl.data.vect.x;
    imu.accel_unscaled.y =  imu_sen10121.acc_adxl.data.vect.y;
    imu.accel_unscaled.z = -imu_sen10121.acc_adxl.data.vect.z;
    imu_sen10121.acc_adxl.data_available = false;
    imu_scale_accel(&imu);
    AbiSendMsgIMU_ACCEL_INT32(IMU_SEN10121_ID, now_ts, &imu.accel);
  }

  /* If the itg3200 I2C transaction has succeeded: convert the data */
  itg3200_event(&imu_sen10121.gyro_itg);
  if (imu_sen10121.gyro_itg.data_available) {
    imu.gyro_unscaled.p = -imu_sen10121.gyro_itg.data.rates.p;
    imu.gyro_unscaled.q =  imu_sen10121.gyro_itg.data.rates.q;
    imu.gyro_unscaled.r = -imu_sen10121.gyro_itg.data.rates.r;
    imu_sen10121.gyro_itg.data_available = false;
    imu_scale_gyro(&imu);
    AbiSendMsgIMU_GYRO_INT32(IMU_SEN10121_ID, now_ts, &imu.gyro);
  }
}
