#pragma once
#include "GILC_IMU_Dev.h"
#include "GILC_Vehicle_lib.h"

/*-------------------------------IMU TYPE-----------------------------*/
#define IMU_DEV_TYPE               IMU_DEV_IMU381
#if (IMU_DEV_TYPE == IMU_DEV_ADIS16445)
#include "GILC_IMU_Config_adis16445.h"
#elif (IMU_DEV_TYPE == IMU_DEV_ADIS16465)
#include "GILC_IMU_Config_adis16465.h"
#elif (IMU_DEV_TYPE == IMU_DEV_IMU380)
#include "GILC_IMU_Config_imu380.h"
#elif (IMU_DEV_TYPE == IMU_DEV_IMU381)
#include "GILC_IMU_Config_imu381_demo2.h"
#endif

/*--------------------------------LEVER(m)-----------------------------*/
#define CFG_ARM_ANT_X    0
#define CFG_ARM_ANT_Y    -0.3
#define CFG_ARM_ANT_Z    1

/*----------------------------TEST RAW FILE-------------------------*/
#ifdef WIN32
#define WORK_PATH  "E:/惯性导航/数据处理/"
#else
#define WORK_PATH  "/mnt/gilc-data/"
#endif


//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2DF/imu381_3666669/20190318/"
//#define TEST_RAW_FILE_NAME	   "021558_raw.txt"
//#define TEST_RAW_FILE_NAME	   "025311_raw.txt"

#define TEST_RAW_FILE_PATH	 WORK_PATH"P2DF/imu381_3666669/20190323/"
//#define TEST_RAW_FILE_NAME	   "084157_raw_cut.txt"
#define TEST_RAW_FILE_NAME	   "085427_raw.txt"

#define TEST_RAW_FILE_PATH	 WORK_PATH"CGI-410/"
//#define TEST_RAW_FILE_NAME	   "084157_raw_cut.txt"
//#define TEST_RAW_FILE_NAME	   "原始数据2_raw - 副本.txt"
#define TEST_RAW_FILE_NAME	   "原始数据2_raw - 2.txt"
/*--------------------------------TEST_OUT/TMP_FILE_PATH-----------------------------*/
#define TEST_OUT_FILE_PATH    TEST_RAW_FILE_PATH"OUT/"
#define TEST_TMP_FILE_PATH    "D:/TMP/"

/*--------------------------------TEST_RST_FILE_NAME----------------------------*/
#define TEST_RST_FILE_NAME     "gilc_vehicle_rst.nmea"

/*--------------------------------RAW CFG-----------------------------*/
#define CFG_POS_STD_USE 1
#define CFG_VEL_STD_USE 1
#define CFG_GNSSPOS_MODE GILC_GNSSPOS_MODE__LLA_DEG
#define CFG_GNSSVEL_MODE GILC_GNSSVEL_MODE__ECEF

#define GILC_NMEA_OUT_PERIOD_MS    100     /*dsf90: 输出10Hz*/
