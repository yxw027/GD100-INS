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
#include "GILC_IMU_Config_imu381.h"
#endif

/*--------------------------------LEVER(m)-----------------------------*/
#define CFG_ARM_ANT_X    0.0
#define CFG_ARM_ANT_Y    0.0
#define CFG_ARM_ANT_Z    0.0

/*----------------------------TEST RAW FILE-------------------------*/
//#define TEST_RAW_FILE_PATH	  "E:/惯性导航/数据处理/20180830_P2_ADIS16465/"
//#define TEST_RAW_FILE_NAME	   "070320_raw.txt"
//#define TEST_RAW_FILE_PATH	  "E:/惯性导航/数据处理/P2C/imu381/20181017/"
//#define TEST_RAW_FILE_NAME	   "011610_raw.txt"
//#define TEST_RAW_FILE_PATH	  "E:/惯性导航/数据处理/P2C/imu381/20181019/"
//#define TEST_RAW_FILE_NAME	   "060137_raw.txt"/*园区车库*/
//#define TEST_RAW_FILE_NAME	   "063038_raw.txt"/*迎宾三路-松泽-华徐-园区车库*/
//#define TEST_RAW_FILE_PATH	  "E:/惯性导航/数据处理/P2C/imu381/20181020/"
//#define TEST_RAW_FILE_NAME	   "004012_raw.txt"/*徐泾-青浦-嘉定，高速*/
//#define TEST_RAW_FILE_PATH	  "E:/惯性导航/数据处理/P2C/imu381/20181022/"
//#define TEST_RAW_FILE_NAME	   "115236_raw.txt"/*园区车库*/
//#define TEST_RAW_FILE_PATH	  "E:/惯性导航/数据处理/P2C/imu381/20181023/"
//#define TEST_RAW_FILE_NAME	   "033824_raw.txt"/*迎宾三路—内环高架*/
//#define TEST_RAW_FILE_NAME	   "060110_raw.txt"/*迎宾三路—内环高架*/
//#define TEST_RAW_FILE_NAME	   "231617_rst.txt"/*迎宾三路—内环高架,单点*/
//#define TEST_RAW_FILE_PATH	  "E:/惯性导航/数据处理/P2C/imu381/20181024/"
//#define TEST_RAW_FILE_NAME	   "004900_raw.txt"/*迎宾三路—内环高架*/
//#define TEST_RAW_FILE_PATH	  "E:/惯性导航/数据处理/P2C/imu381/20181024/"
//#define TEST_RAW_FILE_NAME	   "004900_raw.txt"/*迎宾三路—内环高架*/
#define TEST_RAW_FILE_PATH	  "E:/惯性导航/数据处理/P2C/imu381/20181025/"
#define TEST_RAW_FILE_NAME	   "134214_raw.txt"
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
