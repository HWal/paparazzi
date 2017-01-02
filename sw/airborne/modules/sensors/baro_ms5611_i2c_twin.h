#ifndef BARO_MS56111_I2C_H
#define BARO_MS56111_I2C_H

#include "std.h"
#include "peripherals/ms5611_i2c.h"

// lines added for true altitude calculation
# define BARO_MS5611_PMSL 101325.0 //ISA standard pressure at MSL (Pa)
# define BARO_MS5611_TMSL 288.15   //ISA standard temperature at MSL (K * 10)
# define G0 9.80665                //g (m/(s*s))
# define L -.0065                  //ISA standard temperature lapse rate (K/m)
# define RGAS 287.053              //gas constant for air (J/kg/K)
# define RH 0                      //relative humidity (no unit)
extern float baro_ms5611_tcal;     //Measured averaged temperature for calculation to MSL (K * 10)
extern float baro_ms5611_pcal;     //Measured averaged pressure for calculation to MSL (Pa)
extern float baro_ms5611_tmsl;     //Init calculated temperature at MSL (K * 10)
extern float baro_ms5611_pmsl;     //Init calculated pressure at MSL (Pa)
extern float baro_cal_altitude;    //Calculated true altitude
extern float ms5611_ground_alt;    //ground_alt from Flight plan

/// new measurement with every baro_ms5611_read() call
#define BARO_MS5611_DT BARO_MS5611_READ_PERIOD
#define BARO_MS5611_R 20
#define BARO_MS5611_SIGMA2 1
extern float baro_ms5611_r;
extern float baro_ms5611_sigma2;

extern int32_t fixed_press_offset;

extern float baro_ms5611_alt;
extern bool baro_ms5611_alt_valid;
extern bool baro_ms5611_enabled;

extern struct Ms5611_I2c baro_ms5611_1;
extern struct Ms5611_I2c baro_ms5611_2;

extern void baro_ms5611_twin_init(void);
extern void baro_ms5611_twin_read(void);
extern void baro_ms5611_twin_periodic_check(void);
extern void baro_ms5611_twin_event(void);
extern void baro_ms5611_twin_send_coeff(void);

#endif
