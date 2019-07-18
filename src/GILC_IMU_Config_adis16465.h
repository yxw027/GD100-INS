/*
 * TestFileConfig.h
 *
 *  Created on: May 15, 2018
 *      Author: dsf
 */

#ifndef _GILC_IMU_CONFIG_H_
#define _GILC_IMU_CONFIG_H_

#include <math.h>

 /*-------------------------------IMU BW(Hz)-----------------------------*/
#define IMU_BW_HZ                  40
//#define IMU_BW_HZ                  20

/*--------------------------------IMU WALK-----------------------------*/
/*TUpdate(,1)*/
#define GYRO_WALK (0.15*0.2)
#define VEL_WALK (1200)

/*--------------------------------IMU STD-----------------------------*/
#define GYRO_STD_X (0.003*sqrt(IMU_BW_HZ))
#define ACC_STD_X (0.000023*sqrt(IMU_BW_HZ))

#define GYRO_STD_Y GYRO_STD_X
#define GYRO_STD_Z GYRO_STD_X

#define ACC_STD_Y ACC_STD_X
#define ACC_STD_Z ACC_STD_X

/*--------------------------------IMU AXIS-----------------------------*/
#define CFG_GYRO_X_SCALE  (1)
#define CFG_GYRO_Y_SCALE  (1)
#define CFG_GYRO_Z_SCALE (-1)
#define CFG_ACC_X_SCALE   (1)
#define CFG_ACC_Y_SCALE   (1)
#define CFG_ACC_Z_SCALE  (-1)

#define CFG_GYRO_X_ROW 2
#define CFG_GYRO_Y_ROW 1
#define CFG_GYRO_Z_ROW 3
#define CFG_ACC_X_ROW 5
#define CFG_ACC_Y_ROW 4
#define CFG_ACC_Z_ROW 6
#endif
