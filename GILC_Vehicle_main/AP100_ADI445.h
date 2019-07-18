#pragma once
#pragma once
/*
* TestFileConfig.h
*
*  Created on: May 15, 2018
*      Author: dsf
*/

#ifndef _GILC_IMU_CONFIG_H_
#define _GILC_IMU_CONFIG_H_
#define IMU_T  0.01
#include <math.h>

/*-------------------------------IMU BW(Hz)-----------------------------*/
#define IMU_BW_HZ                  32.768

/*--------------------------------IMU WALK-----------------------------*/
/*TUpdate(,1)*/
#define GYRO_WALK (0.56*3.52)  //*3.5
#define VEL_WALK (7300*2.5)   //*2.5

/*--------------------------------IMU STD-----------------------------*/
#define GYRO_STD_X (0.011*sqrt(IMU_BW_HZ))*3
#define ACC_STD_X (0.000105*sqrt(IMU_BW_HZ))*3

#define ACC_STD_Y ACC_STD_X
#define ACC_STD_Z ACC_STD_X


#define GYRO_STD_Y GYRO_STD_X
#define GYRO_STD_Z GYRO_STD_X
/*--------------------------------IMU AXIS-----------------------------*/
#define CFG_GYRO_X_SCALE  (1)
#define CFG_GYRO_Y_SCALE (1)
#define CFG_GYRO_Z_SCALE  (-1)
#define CFG_ACC_X_SCALE  (1)
#define CFG_ACC_Y_SCALE   (1)
#define CFG_ACC_Z_SCALE  (-1)

#define CFG_GYRO_X_ROW 2
#define CFG_GYRO_Y_ROW 1
#define CFG_GYRO_Z_ROW 3
#define CFG_ACC_X_ROW 5
#define CFG_ACC_Y_ROW 4
#define CFG_ACC_Z_ROW 6

#endif
/*--------------------------------LEVER(m)-----------------------------*/
#define CFG_ARM_ANT_X    1.0
#define CFG_ARM_ANT_Y    0.0
#define CFG_ARM_ANT_Z    0.0
//#define WORK_PATH  "G:/test/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	   "100401_raw.txt"

#define WORK_PATH  "G:/无人船数据/"
#define TEST_RAW_FILE_PATH	 WORK_PATH
#define TEST_RAW_FILE_NAME	   "ship_627.txt"
/*--------------------------------TEST_OUT/TMP_FILE_PATH-----------------------------*/
#define TEST_OUT_FILE_PATH    TEST_RAW_FILE_PATH"OUT/"
#define TEST_TMP_FILE_PATH    "D:/TMP/"

/*--------------------------------TEST_RST_FILE_NAME----------------------------*/
#define TEST_RST_FILE_NAME     "gilc_vehicle_rst.nmea"

/*--------------------------------RAW CFG-----------------------------*/
#define CFG_POS_STD_USE 1
#define CFG_VEL_STD_USE 1
#define CFG_GNSSPOS_MODE GILC_GNSSPOS_MODE__LLA_RAD
//#define CFG_GNSSVEL_MODE GILC_GNSSVEL_MODE__ECEF
#define CFG_GNSSVEL_MODE  GILC_GNSSVEL_MODE__SPEED_HEADING
#define GILC_NMEA_OUT_PERIOD_MS   100     /*dsf90: 输出10Hz*/