#pragma once
#include "GILC_IMU_Dev.h"
#include "GILC_Vehicle_lib.h"

/*-------------------------------IMU TYPE-----------------------------*/
#define IMU_DEV_TYPE               IMU_DEV_ADIS16445
#if (IMU_DEV_TYPE == IMU_DEV_ADIS16445)
#include "GILC_IMU_Config_adis16445_nx200.h"
#elif (IMU_DEV_TYPE == IMU_DEV_IMU380)
#include "GILC_IMU_Config_imu380_nx200.h"
#endif

/*--------------------------------LEVER(m)-----------------------------*/
#define CFG_ARM_ANT_X    1.0
#define CFG_ARM_ANT_Y    0.0
#define CFG_ARM_ANT_Z    0.0

/*----------------------------TEST RAW FILE-------------------------*/
//#define TEST_RAW_FILE_PATH	  "E:/惯性导航/数据处理/NX200/adis16445/20181020/"
//#define TEST_RAW_FILE_NAME	   "20181020_090153.txt"
//#define TEST_RAW_FILE_PATH	  "E:/惯性导航/数据处理/NX200/adis16445/20181024/"
//#define TEST_RAW_FILE_NAME	   "20181024_064111.txt"



//#define WORK_PATH  "C:/Users/huace/Desktop/NX200-gilc/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH"无人船/"
//#define TEST_RAW_FILE_NAME	   "ship2.txt"


//#define WORK_PATH  "E:/无人船项目/张晓飞测试数据/6.20测试数据/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	   "ship1.txt"


#define WORK_PATH  "G:/test/"
#define TEST_RAW_FILE_PATH	 WORK_PATH
#define TEST_RAW_FILE_NAME	   "100401_raw.txt"
/*--------------------------------TEST_OUT/TMP_FILE_PATH-----------------------------*/
#define TEST_OUT_FILE_PATH    TEST_RAW_FILE_PATH"OUT/"
#define TEST_TMP_FILE_PATH    "D:/TMP/"

/*--------------------------------TEST_RST_FILE_NAME----------------------------*/
#define TEST_RST_FILE_NAME     "gilc_vehicle_rst.nmea"

/*--------------------------------RAW CFG-----------------------------*/
#define CFG_POS_STD_USE 1
#define CFG_VEL_STD_USE 1
#define CFG_GNSSPOS_MODE GILC_GNSSPOS_MODE__LLA_RAD
#define CFG_GNSSVEL_MODE GILC_GNSSVEL_MODE__ECEF

#define GILC_NMEA_OUT_PERIOD_MS    100     /*dsf90: 输出10Hz*/
