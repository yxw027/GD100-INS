/*
 * TestFileConfig.h
 *
 *  Created on: May 15, 2018
 *      Author: dsf
 */

#pragma once

#ifndef STM32
#include "stdio.h"
#endif
#include "stdlib.h"
#include "math.h"

#include "GILC_Vehicle_lib.h"

#define _STRN(a) #a
#define STRN(a) _STRN(a)

#ifndef PI
#define PI		    3.14159265358979
#define D2R         (PI/180.0)          /* deg to rad */
#define R2D         (180.0/PI)          /* rad to deg */
#endif

#define GILC_NMEA_OUT_PERIOD_MS    100     /*dsf90: 输出10Hz*/
#define GILC_DEBUG_PROCESS_TIME    0         /*处理时间测试*/

#define ADIS16445  1
#define ADIS16465  2
#define BMI055     3

#define P2C         1
#define NX200      2
#define GI251      3

#define NX200_ADIS16445 1
#define P2_ADIS16445    2
#define P2_ADIS16465    3

//#define DATA_FORM 20180201
//#define DATA_FORM 20180327
//#define DATA_FORM 20180405
//#define DATA_FORM 20180410
//#define DATA_FORM 20180503
//#define DATA_FORM 20180507
//#define DATA_FORM 20180606
//#define DATA_FORM 20180610
//#define DATA_FORM 20180621
//#define DATA_FORM 20180703
//#define DATA_FORM 20180711
//#define DATA_FORM 20180716
//#define DATA_FORM 20180717
//#define DATA_FORM 20180818

//#define DATA_FORM 20180727 /*GI251*/

//#define DATA_FORM 20180816 /*P2C*/
//#define DATA_FORM 20180819 /*P2C*/
//#define DATA_FORM 20180820 /*P2C*/
//#define DATA_FORM 20180821 /*P2C*/
//#define DATA_FORM 20180822 /*P2C*/
#define DATA_FORM 20180830 /*P2*/

#define RAW_FILE_NAME_COMMON   "org_form_dhf.txt"
#define RST_FILE_NAME_COMMON   "gilc_vehicle_rst.nmea"

#ifdef WIN32
#define WORK_PATH  "E:/惯性导航/数据处理/"
#else
#define WORK_PATH  "/mnt/gilc-data/"
#endif

/*--------------------------------TEST_RAW_FILE_PATH-----------------------------*/
#if (DATA_FORM == 20180201)
	#define SYNC_PPS_UNUSED
	#define CFG_VEL_STD_USE 0
	#define TEST_RAW_FILE_NAME "gpsimu1.txt"           /*华徐、崧泽、徐南路*/
	//#define TEST_RAW_FILE_NAME "gpsimu2.txt"         /*沪青平、迎宾三路*/
#elif (DATA_FORM == 20180327) 
	#define TEST_RAW_FILE_NAME "20180327_093827.txt"    /*华徐、崧泽、外环、迎宾三路、徐南路、新虹桥天地*/
#elif (DATA_FORM == 20180405) 
	//#define TEST_RAW_FILE_NAME "20180405_055310.txt"    /*园区车库*/
	//#define TEST_RAW_FILE_NAME "20180405_071542.txt"    /*园区车库*/
	#define TEST_RAW_FILE_NAME "20180405_074238.txt"    /*食尚天地、华徐、崧泽、华翔、迎宾三路*/
#elif (DATA_FORM == 20180410) 
	#define TEST_RAW_FILE_NAME "20180410_071449.txt"   /*园区车库          ---------------组合有飞点！！！-----------*/
	//#define TEST_RAW_FILE_NAME "20180410_074813.txt"    /*园区车库、迎宾三路---------------GNSS数据质量差-----------*/  
#elif (DATA_FORM == 20180503)  /***NX200测试机，此后改用32Hz滤波***/
	#define TEST_RAW_FILE_NAME "20180503_065020.txt"   /*园区车库*/
	//#define TEST_RAW_FILE_NAME "20180503_073713.txt"   /*园区车库、迎宾三路*/
#elif (DATA_FORM == 20180507) 
	#define TEST_RAW_FILE_NAME "20180507_093509.txt"   /*华徐、崧泽、华翔、迎宾三路*/
#elif (DATA_FORM == 20180606) /***NX200测试机，此后换装OEM718D***/
	#define TEST_RAW_FILE_NAME "20180606_071016.txt"  /*园区车库*/
#elif (DATA_FORM == 20180610) 
	//#define TEST_RAW_FILE_NAME "org_form_dhf1.txt"   /*园区车库*/
	#define TEST_RAW_FILE_NAME "org_form_dhf2.txt"   /*食尚天地、华徐、崧泽、华翔、迎宾三路、徐南路、新虹桥天地*/
	//#define TEST_RAW_FILE_NAME "org_form_dhf3.txt"   /*园区绕圈*/
#elif (DATA_FORM == 20180621) 
	#define TEST_RAW_FILE_NAME "org_form_dhf1.txt"   /*园区车库*/   
#elif (DATA_FORM == 20180703) 
	//#define TEST_RAW_FILE_NAME "032808_raw.txt"   /*迎宾三路、延安高架*/
	#define TEST_RAW_FILE_NAME "224447_raw.txt"   /*迎宾三路、延安高架*/
#elif (DATA_FORM == 20180711) 
	//#define TEST_RAW_FILE_NAME   "030942_raw.txt"   /*华徐、崧泽、华翔、迎宾三路*/
	//#define TEST_RAW_FILE_NAME   "055949_raw.txt"   /*沪青平、迎宾三路*/
	#define TEST_RAW_FILE_NAME   "074540_raw.txt"    /*华徐、崧泽、华翔、迎宾三路，---------组合有飞点！！！------*/
#elif (DATA_FORM == 20180716) 
	//#define TEST_RAW_FILE_NAME   "045443_raw.txt"     /*园区车库、沪青平*/
	#define TEST_RAW_FILE_NAME   "072308_raw.txt"   /*华徐、华翔、迎宾三路*/
#elif (DATA_FORM == 20180717) 
	#define TEST_RAW_FILE_NAME   "030348_raw.txt"   /*食尚天地、华徐、崧泽、华翔、迎宾三路*/
#elif (DATA_FORM == 20180816) 
	#define IMU_DEV   P2C
	#if 0
		#define IMU_TYPE  ADIS16445
		#define TEST_RAW_FILE_NAME   "101955_raw.txt"	/*20180816,园区车库*/  
	#else
		#define IMU_TYPE  ADIS16465
		#define TEST_RAW_FILE_NAME   "101955_raw.txt"	/*20180816,园区车库*/  
	#endif
#elif (DATA_FORM == 20180818) 
	#define IMU_TYPE  ADIS16445
	#define TEST_RAW_FILE_NAME   "20180818_111314.txt"	/*20180816,园区车库*/  
#elif (DATA_FORM == 20180819) 
	#define IMU_DEV   P2C
	#if 0
		#define IMU_TYPE  ADIS16445
		//#define TEST_RAW_FILE_NAME   "054926_raw.txt"	   /*20180818,徐泾-千灯*/
		//#define TEST_RAW_FILE_NAME   "005820_raw.txt"    /*20180819,千灯-昆山*/
		//#define TEST_RAW_FILE_NAME   "014058_raw.txt"    /*20180819,昆山*/
		#define TEST_RAW_FILE_NAME   "031603_raw.txt"    /*20180819,昆山-千灯*/
	#else
		#define IMU_TYPE  ADIS16465
		//#define TEST_RAW_FILE_NAME   "054929_raw.txt"	   /*20180818,徐泾-千灯*/
		//#define TEST_RAW_FILE_NAME   "005829_raw.txt"    /*20180819,千灯-昆山*/
		//#define TEST_RAW_FILE_NAME   "014102raw.txt"    /*20180819,昆山*/
		#define TEST_RAW_FILE_NAME   "031607_raw.txt"    /*20180819,昆山-千灯*/ 
	#endif
#elif (DATA_FORM == 20180820) 
	#define IMU_DEV   P2C
	#if 1
		#define IMU_TYPE  ADIS16445
		#define TEST_RAW_FILE_NAME   "082023_raw.txt"	/*食尚天地、华徐、崧泽、华翔、迎宾三路*/
	#else
		#define IMU_TYPE  ADIS16465
		#define TEST_RAW_FILE_NAME   "082030_raw.txt"	/*食尚天地、华徐、崧泽、华翔、迎宾三路*/
	#endif
#elif (DATA_FORM == 20180821) 
	#define IMU_DEV   P2C
	#if 0
		#define IMU_TYPE  ADIS16445
		#define TEST_RAW_FILE_NAME   "065511_raw.txt"	/*华徐、崧泽、华翔、迎宾三路*/
	#else
		#define IMU_TYPE  ADIS16465
		#define TEST_RAW_FILE_NAME   "065509_raw.txt"	/*华徐、崧泽、华翔、迎宾三路*/
		//#define TEST_RAW_FILE_NAME   "091129_raw.txt"	/*20180829，园区拔天线比测，BD992*/
	#endif
#elif (DATA_FORM == 20180822) 
	#define IMU_DEV   P2C
	#if 0
		#define IMU_TYPE  ADIS16445
		#define TEST_RAW_FILE_PATH	   WORK_PATH "20180822_P2自研组合指标标定测试/adis16445/"
		//#define TEST_RAW_FILE_NAME   "031649_raw.txt"     /*徐泾北城，绕圈*/
		//#define TEST_RAW_FILE_NAME   "044714_raw.txt"     /*绿中海，绕圈*/
		#define TEST_RAW_FILE_NAME   "054351_raw.txt"     /*园区，绕圈*/
	#else
		#define IMU_TYPE  ADIS16465
		#define TEST_RAW_FILE_PATH	   WORK_PATH "20180822_P2自研组合指标标定测试/adis16465/"
		//#define TEST_RAW_FILE_NAME   "031647_raw.txt"     /*徐泾北城，绕圈*/
		#define TEST_RAW_FILE_NAME   "044659_raw.txt"     /*绿中海，绕圈*/
		//#define TEST_RAW_FILE_NAME   "054353_raw.txt"     /*园区，绕圈*/
	#endif
#elif (DATA_FORM == 20180830)  /*修改IMU数据更新频率、滤波器抽头数*/
	#define IMU_DEV   P2C
	#if 0
	#define IMU_TYPE  ADIS16445
	//#define TEST_RAW_FILE_NAME   "065511_raw.txt"	
	//#define TEST_RAW_FILE_NAME   "003506_raw.txt"
	//#define TEST_RAW_FILE_NAME	 "073730_raw.txt"	/*20180903，园区，绕圈*/	
	//#define TEST_RAW_FILE_NAME   "075840_raw.txt"	/*20180903，园区，绕圈*/
	//#define TEST_RAW_FILE_NAME     "060649_raw.txt"	/*20180904，蟠龙、沪青平、盈港，环测，100Hz*/
	//#define TEST_RAW_FILE_NAME   "140140_raw.txt"   /*20180904，下班路测，200Hz(未启动实时运算)*/
	//#define TEST_RAW_FILE_NAME   "141407_raw.txt"	/*20180905，下班路测，200Hz*/
	//#define TEST_RAW_FILE_NAME   "011426_raw.txt"	/*20180910，上班路测，100Hz*/
	#define TEST_RAW_FILE_NAME	 "085934_raw.txt"	/*20180910，园区-沪青平-诸光-盈港-蟠龙，200Hz*/
	//#define TEST_RAW_FILE_NAME	 "123729_raw.txt"	/*20180913，，100Hz，bw 8hz*/
	//#define TEST_RAW_FILE_NAME	 "095842_raw.txt"	/*20180913，，100Hz，bw 8hz*/
	//#define TEST_RAW_FILE_NAME	 "070320_raw.txt"	/*20180914，，100Hz，bw 32hz*/
	//#define TEST_RAW_FILE_NAME	 "101524_raw.txt"	/*20180914，高速，200Hz，bw 64hz*/
	//#define TEST_RAW_FILE_NAME	 "011127_raw.txt"	/*20180920，，100Hz，bw 32hz*/
	//#define IMU_BW	  8
	#define IMU_BW	  32
	//#define IMU_BW	  64
	#else
	#define IMU_TYPE  ADIS16465
	//#define TEST_RAW_FILE_NAME   "035129_raw.txt"
	//#define TEST_RAW_FILE_NAME   "065215_raw.txt"	
	//#define TEST_RAW_FILE_NAME	 "073730_raw.txt"	/*20180903，园区，绕圈*/	
	//#define TEST_RAW_FILE_NAME   "075841_raw.txt"	/*20180903，园区，绕圈*/
	//#define TEST_RAW_FILE_NAME   "060650_raw.txt"	/*20180904，蟠龙、沪青平、盈港，环测，100Hz*/
	//#define TEST_RAW_FILE_NAME   "140140_raw.txt"	/*20180904，下班路测，200Hz(未启动实时运算)*/
	//#define TEST_RAW_FILE_NAME   "141408_raw.txt"	/*20180905，下班路测，200Hz*/
	//#define TEST_RAW_FILE_NAME   "011427_raw.txt"	/*20180910，上班路测，100Hz*/
	//#define TEST_RAW_FILE_NAME   "085935_raw.txt"	/*20180910，园区-沪青平-诸光-盈港-蟠龙，200Hz*/
	//#define TEST_RAW_FILE_NAME	 "123730_raw.txt"	/*20180913，，100Hz，bw 10hz*/
	//#define TEST_RAW_FILE_NAME	 "095843_raw.txt"	/*20180913，，100Hz，bw 10hz*/
	//#define TEST_RAW_FILE_NAME	 "061829_raw.txt"	/*20180914，，100Hz，bw 10hz*/
	#define TEST_RAW_FILE_NAME	 "070320_raw.txt"	/*20180914，，100Hz，bw 40hz*/
	//#define TEST_RAW_FILE_NAME	 "101525_raw.txt"	/*20180914，高速，单点，200Hz，bw 80hz*/
	//#define TEST_RAW_FILE_NAME	 "011128_raw.txt"	/*20180920，，100Hz，bw 40hz*/
	//#define IMU_BW	  10
	#define IMU_BW	  40
	//#define IMU_BW	  80
#endif
#else
	#define IMU_DEV   P2C
	#if 1
	#define IMU_TYPE ADIS16445
	#define IMU_TYPE_STRN  "adis16445"
	#define IMU_BW	  32
	#define DATA_FORM 20180924 
	#define TEST_RAW_FILE_NAME	 "220217_raw.txt"	/*bw 32hz*/
	//#define DATA_FORM 20180925 
	//#define TEST_RAW_FILE_NAME	 "010911_raw.txt"	/*bw 32hz*/
	#else
	#define IMU_TYPE ADIS16465
	#define IMU_TYPE_STRN  "adis16465"
	#define IMU_BW	  40
	//#define DATA_FORM 20180927 
	//#define TEST_RAW_FILE_NAME	 "022808_raw.txt"	/*bw 40hz*/
	#define DATA_FORM 20180929 
	#define TEST_RAW_FILE_NAME	 "p2_raw.txt"	/*bw 40hz*/
	#endif

	#define TEST_RAW_FILE_PATH	 WORK_PATH "P2C/" IMU_TYPE_STRN "/" STRN(DATA_FORM) "/"
#endif

#if GILC_DEBUG_PROCESS_TIME
#define DATA_FORM 20180610
#undef  TEST_RAW_FILE_PATH
#define TEST_RAW_FILE_PATH      WORK_PATH"debug_process_time/"
#define TEST_RAW_FILE_NAME      "debug_org_sync_form_dhf.txt"
#endif

/*--------------------------------IMU TYPE / IMU DEV----------------------------*/
#ifndef IMU_DEV
#define IMU_DEV   NX200
#endif
#ifndef IMU_TYPE
#define IMU_TYPE  ADIS16445
#endif
#if (IMU_TYPE==ADIS16445)
#define IMU_TYPE_STRN "ADIS16445"
#elif (IMU_TYPE==ADIS16465)
#define IMU_TYPE_STRN "ADIS16465"
#elif (IMU_TYPE==BMI055)
#define IMU_TYPE_STRN "BMI055"
#endif

/*--------------------------------TEST_RAW_FILE_PATH----------------------------*/
#ifndef TEST_RAW_FILE_PATH
#if (IMU_DEV==P2C)
#define TEST_RAW_FILE_PATH	   WORK_PATH STRN(DATA_FORM) "_P2_" IMU_TYPE_STRN "/"
#elif (IMU_DEV==GI251)
#define TEST_RAW_FILE_PATH	   WORK_PATH STRN(DATA_FORM) "_GI251_" IMU_TYPE_STRN "/"
#elif (IMU_DEV==NX200)
#define TEST_RAW_FILE_PATH	   WORK_PATH STRN(DATA_FORM) "_NX200"  "/"
#endif
#endif

#ifndef CFG_GNSSPOS_MODE
#if (IMU_DEV==P2C)
#define CFG_GNSSPOS_MODE       GILC_GNSSPOS_MODE__LLA_DEG
#elif (IMU_DEV==GI251)
#elif (IMU_DEV==NX200)
#endif
#endif

/*--------------------------------TEST_RAW/RST_FILE_NAME----------------------------*/
#ifndef TEST_RAW_FILE_NAME
	#define TEST_RAW_FILE_NAME RAW_FILE_NAME_COMMON
#endif
#ifndef TEST_RST_FILE_NAME
	#define TEST_RST_FILE_NAME RST_FILE_NAME_COMMON
#endif

/*--------------------------------TEST_OUT/TMP_FILE_PATH-----------------------------*/
#define TEST_OUT_FILE_PATH TEST_RAW_FILE_PATH"OUT/"
//#define TEST_TMP_FILE_PATH WORK_PATH
#define TEST_TMP_FILE_PATH "D:/TMP/"

/*--------------------------------IMU STD-----------------------------*/
#if (IMU_TYPE==ADIS16465)
#ifdef IMU_BW
	#define GYRO_STD_X (0.003*sqrt(IMU_BW))
	#define ACC_STD_X (0.000023*sqrt(IMU_BW))
#else
	#define GYRO_STD_X (0.003*sqrt(40))
	#define ACC_STD_X (0.000023*sqrt(40))
#endif
#elif (IMU_TYPE==ADIS16445)
	#if (DATA_FORM == 20180201)
	#define GYRO_STD_X (0.011*sqrt(0.128)*10)
	#define ACC_STD_X (0.000105*sqrt(0.128))
	#elif (DATA_FORM == 20180323)
	#define GYRO_STD_X (0.011*sqrt(0.5)*10)
	#define ACC_STD_X (0.000105*sqrt(0.5))
	#elif (DATA_FORM == 20180327)
	#define GYRO_STD_X (0.011*sqrt(2)*10)
	#define ACC_STD_X (0.000105*sqrt(2))
	#elif (DATA_FORM == 20180405)
	#define GYRO_STD_X (0.011*sqrt(0.5)*10)
	#define ACC_STD_X (0.000105*sqrt(0.5))
	#elif (DATA_FORM == 20180410)
	#define GYRO_STD_X (0.011*sqrt(0.5)*10)
	#define ACC_STD_X (0.000105*sqrt(0.5))
	#else
		#if (IMU_DEV==NX200)
		#define GYRO_STD_X (0.011*sqrt(32))
		#define ACC_STD_X (0.000105*sqrt(32))
		#elif (IMU_DEV==P2C)
			#if(DATA_FORM >= 20180830)
				#ifdef IMU_BW
				#define GYRO_STD_X (0.011*sqrt(IMU_BW))
				#define ACC_STD_X (0.000105*sqrt(IMU_BW))
				#else
				#define GYRO_STD_X (0.011*sqrt(32))
				#define ACC_STD_X (0.000105*sqrt(32))
				#endif
			#else
			#define GYRO_STD_X (0.011*sqrt(40.96))
			#define ACC_STD_X (0.000105*sqrt(40.96))
			#endif
		#endif
	#endif
#elif (IMU_TYPE==BMI055)
	#define GYRO_STD_X (0.014*sqrt(47))
	#define ACC_STD_X (0.000150*sqrt(63))
#endif

#ifndef GYRO_STD_Y
    #define GYRO_STD_Y GYRO_STD_X
#endif
#ifndef GYRO_STD_Z
    #define GYRO_STD_Z GYRO_STD_X
#endif
#ifndef ACC_STD_Y
    #define ACC_STD_Y ACC_STD_X
#endif
#ifndef ACC_STD_Z
    #define ACC_STD_Z ACC_STD_X
#endif

/*--------------------------------IMU WALK-----------------------------*/
#if (IMU_TYPE == ADIS16445)
	#if (DATA_FORM == 20180201)
	/*TUpdate(,1)*/
	#define GYRO_WALK (0.56/20)
	#define VEL_WALK (7300)
	/*TUpdate(,0)*/
	//#define GYRO_WALK (0.56/100)
	//#define VEL_WALK (7300)
	#elif (DATA_FORM == 20180323)
	/*TUpdate(,1)*/
	#define GYRO_WALK (0.56/20)
	#define VEL_WALK (7300)
	/*TUpdate(,0)*/
	//#define GYRO_WALK (0.56/100)
	//#define VEL_WALK (7300)
	#elif (DATA_FORM == 20180327)
	/*TUpdate(,1)*/
	#define GYRO_WALK (0.56/20)
	#define VEL_WALK (7300)
	/*TUpdate(,0)*/
	//#define GYRO_WALK (0.56/100)
	//#define VEL_WALK (7300)
	#elif (DATA_FORM == 20180405)
	/*TUpdate(,1)*/
	#define GYRO_WALK (0.56/20)
	#define VEL_WALK (7300)
	#elif (DATA_FORM == 20180410)
	/*TUpdate(,1)*/
	#define GYRO_WALK (0.56/20)
	#define VEL_WALK (7300)
	#else
	/*TUpdate(,1)*/
	#define GYRO_WALK (0.56*0.05)
	#define VEL_WALK (7300)
	//#define VEL_WALK (0)
	#endif
#elif (IMU_TYPE == ADIS16465)
	/*TUpdate(,1)*/
	#define GYRO_WALK (0.15*0.05)
	#define VEL_WALK (1200)
#elif (IMU_TYPE == BMI055)
	/*TUpdate(,1)*/
	#define GYRO_WALK (0.05)
	#define VEL_WALK (10000)
#endif

/*--------------------------------RAW CFG-----------------------------*/
#ifndef CFG_POS_STD_USE
#define CFG_POS_STD_USE 1
#endif
#ifndef CFG_VEL_STD_USE
#define CFG_VEL_STD_USE 1
#endif

#ifndef CFG_GNSSPOS_MODE
#define CFG_GNSSPOS_MODE GILC_GNSSPOS_MODE__LLA_RAD
#endif

#ifndef CFG_GNSSVEL_MODE
#define CFG_GNSSVEL_MODE GILC_GNSSVEL_MODE__ECEF
#endif

//#define CFG_POS_STD_USE 0
//#define CFG_VEL_STD_USE 0

#if (IMU_DEV == NX200)
	#if (DATA_FORM == 20180201)
	#define CFG_GYRO_X_SCALE  (-1)
	#define CFG_GYRO_Y_SCALE   (1)
	#define CFG_GYRO_Z_SCALE   (1)
	#define CFG_ACC_X_SCALE    (1/9.81)
	#define CFG_ACC_Y_SCALE   (-1/9.81)
	#define CFG_ACC_Z_SCALE   (-1/9.81)

	#define CFG_GYRO_X_ROW 5
	#define CFG_GYRO_Y_ROW 4
	#define CFG_GYRO_Z_ROW 6
	#define CFG_ACC_X_ROW 2
	#define CFG_ACC_Y_ROW 1
	#define CFG_ACC_Z_ROW 3
	#elif (DATA_FORM <= 20180507)
	#define CFG_GYRO_X_SCALE  (D2R)
	#define CFG_GYRO_Y_SCALE (-D2R)
	#define CFG_GYRO_Z_SCALE  (D2R)
	#define CFG_ACC_X_SCALE  (-1)
	#define CFG_ACC_Y_SCALE   (1)
	#define CFG_ACC_Z_SCALE  (-1)

	#define CFG_GYRO_X_ROW 3
	#define CFG_GYRO_Y_ROW 2
	#define CFG_GYRO_Z_ROW 1
	#define CFG_ACC_X_ROW 6
	#define CFG_ACC_Y_ROW 5
	#define CFG_ACC_Z_ROW 4
	#elif (DATA_FORM < 20180703)
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
	#else
	#define CFG_GYRO_X_SCALE  (D2R)
	#define CFG_GYRO_Y_SCALE (-D2R)
	#define CFG_GYRO_Z_SCALE  (D2R)
	#define CFG_ACC_X_SCALE  (-1)
	#define CFG_ACC_Y_SCALE   (1)
	#define CFG_ACC_Z_SCALE  (-1)

	#define CFG_GYRO_X_ROW 2
	#define CFG_GYRO_Y_ROW 1
	#define CFG_GYRO_Z_ROW 3
	#define CFG_ACC_X_ROW 5
	#define CFG_ACC_Y_ROW 4
	#define CFG_ACC_Z_ROW 6
	#endif
#elif (IMU_DEV == P2C)
	#if (IMU_TYPE == ADIS16465)/*P2C*/
	#define CFG_GYRO_X_SCALE  (D2R)
	#define CFG_GYRO_Y_SCALE  (D2R)
	#define CFG_GYRO_Z_SCALE (-D2R)
	#define CFG_ACC_X_SCALE   (1)
	#define CFG_ACC_Y_SCALE   (1)
	#define CFG_ACC_Z_SCALE  (-1)

	#define CFG_GYRO_X_ROW 2
	#define CFG_GYRO_Y_ROW 1
	#define CFG_GYRO_Z_ROW 3
	#define CFG_ACC_X_ROW 5
	#define CFG_ACC_Y_ROW 4
	#define CFG_ACC_Z_ROW 6
	#elif (IMU_TYPE == ADIS16445)
	#define CFG_GYRO_X_SCALE  (D2R)
	#define CFG_GYRO_Y_SCALE (-D2R)
	#define CFG_GYRO_Z_SCALE  (D2R)
	#define CFG_ACC_X_SCALE   (1)
	#define CFG_ACC_Y_SCALE  (-1)
	#define CFG_ACC_Z_SCALE   (1)

	#define CFG_GYRO_X_ROW 2
	#define CFG_GYRO_Y_ROW 1
	#define CFG_GYRO_Z_ROW 3
	#define CFG_ACC_X_ROW 5
	#define CFG_ACC_Y_ROW 4
	#define CFG_ACC_Z_ROW 6
	#endif
#elif (IMU_DEV == GI251)
	#if (DATA_FORM == 20180727)/*GI251,BMI055,电源开关朝向前*/
	#define CFG_GYRO_X_SCALE  (D2R)
	#define CFG_GYRO_Y_SCALE (-D2R)
	#define CFG_GYRO_Z_SCALE  (D2R)
	#define CFG_ACC_X_SCALE   (1)
	#define CFG_ACC_Y_SCALE  (-1)
	#define CFG_ACC_Z_SCALE   (1)

	#define CFG_GYRO_X_ROW 2
	#define CFG_GYRO_Y_ROW 1
	#define CFG_GYRO_Z_ROW 3
	#define CFG_ACC_X_ROW 5
	#define CFG_ACC_Y_ROW 4
	#define CFG_ACC_Z_ROW 6
	#endif
#endif
/*--------------------------------LEVER-----------------------------*/
#if 1
#if (DATA_FORM == 20180610)
#define CFG_ARM_ANT_X    0.0
#define CFG_ARM_ANT_Y   -0.3
#define CFG_ARM_ANT_Z    0.8
#elif(DATA_FORM == 20180621)
#define CFG_ARM_ANT_X    0.5
#define CFG_ARM_ANT_Y    0.8
#define CFG_ARM_ANT_Z    0.8
#elif(DATA_FORM == 20180717)
#define CFG_ARM_ANT_X    0.2
#define CFG_ARM_ANT_Y    -1.2
#define CFG_ARM_ANT_Z    1.2
#else
#define CFG_ARM_ANT_X    0.0
#define CFG_ARM_ANT_Y    0.0
#define CFG_ARM_ANT_Z    0.0
#endif
#else
#define CFG_ARM_ANT_X    0.0
#define CFG_ARM_ANT_Y    0.0
#define CFG_ARM_ANT_Z    0.0
#endif
#ifdef STM32
int gilc_test_main(void);
#endif
