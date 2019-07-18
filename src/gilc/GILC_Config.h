/*
 * GILC_Config.h
 *
 *  Created on: May 16, 2018
 *      Author: dsf
 */

#pragma once

#ifndef STM32
#include "stdio.h"
#endif
/*------------
日志：
v0.4.0_20190515
  开放里程计车速接入功能
v0.4.0_20190519
  标定/初始化结果添加阈值约束，解决IMU加速度计异常，仍能初始化成功问题；
  20分钟不能完成初始化，认为初始化失败，添加初始化失败返回值
-------------*/
/*--------------------------------GILC LIB VERSION-----------------------------*/
#define GILC_SOFT_VER  "v0.4.0"
#define GILC_SOFT_DATE "20190515"
/*-----------------------------------------------------------------------------*/

#define GILC_SIMPLIFY_USED            1
#define GILC_INSTALL_FIX              0

#define GNSS_PERIODE_MS 200

/*--------------------------------TEST_OUT_FILE_PATH-----------------------------*/
#define TEST_OUT_FILE_PATH "./GILC-Vehicle/out/"
#define TEST_TMP_FILE_PATH "./GILC-Vehicle/temp/"

/*--------------------------------IMU STD-----------------------------*/

#define GILC_GYRO_STD_X (0.003*sqrt(40))
#define GILC_GYRO_STD_Y GILC_GYRO_STD_X	
#define GILC_GYRO_STD_Z GILC_GYRO_STD_X

#define GILC_ACC_STD_X (0.000023*sqrt(40))
#define GILC_ACC_STD_Y GILC_ACC_STD_X
#define GILC_ACC_STD_Z GILC_ACC_STD_X 
/*--------------------------------IMU WALK-----------------------------*/
#define GILC_GYRO_WALK (0.15*0.2)
#define GILC_VEL_WALK (1200)
/*--------------------------------IMU RAW CFG-----------------------------*/
#define CFG_GYRO_X_SCALE (-D2R)
#define CFG_GYRO_Y_SCALE  (D2R)
#define CFG_GYRO_Z_SCALE  (D2R)
#define CFG_ACC_X_SCALE   (1)
#define CFG_ACC_Y_SCALE  (-1)
#define CFG_ACC_Z_SCALE  (-1)

#define CFG_GYRO_X_ROW 2
#define CFG_GYRO_Y_ROW 1
#define CFG_GYRO_Z_ROW 3
#define CFG_ACC_X_ROW 5
#define CFG_ACC_Y_ROW 4
#define CFG_ACC_Z_ROW 6

