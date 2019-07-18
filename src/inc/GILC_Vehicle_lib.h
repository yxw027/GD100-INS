/*
* GILC_Vehicle_lib.h
*
*  Created on: May 7, 2018
*      Author: dsf
*        Form: ANSI
*/

#ifndef GILC_VEHICLEMEASURE_LIB_H
#define GILC_VEHICLEMEASURE_LIB_H

#ifndef STM32
#include "stdio.h"
#endif
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif
	/*gilc : gnss ins loose combination*/

#define MAXLEN		1024

#ifndef PI
#define PI		    3.14159265358979
#define D2R         (PI/180.0)          /* deg to rad */
#define R2D         (180.0/PI)          /* rad to deg */
#define RE_WGS84    6378137.0           // earth semimajor axis (WGS84) (m)
#endif
/*
运动模型：
	1 通用车载
	2 低速车载（<15km/h）
	3 轨道交通
	4 农机
	5 无人船
	6 固定翼
*/
typedef enum gilc_work_mode {
	GILC_WORK_MODE__DEFAULT = 0,
	GILC_WORK_MODE__CAR_NORMAL,
	GILC_WORK_MODE__CAR_SLOW,
	GILC_WORK_MODE__TRAIN,
	GILC_WORK_MODE__TRACTOR,
	GILC_WORK_MODE__SHIP,
	GILC_WORK_MODE__PLANE,
}gilc_work_mode_e;

typedef enum gilc_status_chc {
	GILC_STATUS_L__INIT_MODE = 0x00,
	GILC_STATUS_L__GNSS_MODE = 0x01,
	GILC_STATUS_L__GILC_MODE = 0x02,
	GILC_STATUS_L__INS_MODE  = 0x03,
	GILC_STATUS_H__RST_NOTHING    = 0x00,
	GILC_STATUS_H__SINGLE_HEADING = 0x10,
	GILC_STATUS_H__RTD_HEADING    = 0x20,
	GILC_STATUS_H__INS            = 0x30,	
	GILC_STATUS_H__RTK_HEADING    = 0x40,	
	GILC_STATUS_H__FLOAT_HEADING  = 0x50,	
	GILC_STATUS_H__SINGLE         = 0x60,
	GILC_STATUS_H__RTD	          = 0x70,
	GILC_STATUS_H__RTK	          = 0x80,
	GILC_STATUS_H__FLOAT          = 0x90,
}gilc_status_chc_e;

typedef enum gilc_ret {
	GILC_RET__ERR_CALIBRATE_FAIL = -24,
	GILC_RET__ERR_RAW_ODO_RUN_MODE_PARAM = -23,
	GILC_RET__ERR_RAW_ODO_DISTENCE_PARAM = -22,
	GILC_RET__ERR_RAW_ODO_YAW_PARAM = -21,
	GILC_RET__ERR_RAW_ODO_VEL_PARAM = -20,
	GILC_RET__ERR_CFG_POS_MODE = -15,
	GILC_RET__ERR_CFG_VEL_MODE = -14,
	GILC_RET__ERR_RAW_IMU_ACCEL_PARAM = -13,
	GILC_RET__ERR_RAW_IMU_GYRO_PARAM = -12,
	GILC_RET__ERR_RAW_GNSS_TIME_PARAM = -11,
	GILC_RET__ERR_RAW_GNSS_STATE_PARAM = -10,
	GILC_RET__ERR_RAW_POS_PARAM = -9,
	GILC_RET__ERR_RAW_VEL_PARAM = -8,
	GILC_RET__ERR_RAW_POS_STD_PARAM = -7,
	GILC_RET__ERR_RAW_VEL_STD_PARAM = -6,
	GILC_RET__ERR_RAW_PARAM = -5,
	GILC_RET__ERR_RAW_NO_UPDATE = -4,
	GILC_RET__ERR_GNSS_POS_REPEAT = -3,
	GILC_RET__ERR_STRN_PARAM = -2,
	GILC_RET__ERR_STRN_UNKNOW = -1,
	GILC_RET__RST_NOTHING = 0,
	GILC_RET__RST_INITING = 1,
	GILC_RET__RST_RESOLVING = 2,
	GILC_RET__RST_STABLE = 3,
	GILC_RET__RST_INS_MODE = 4,
	GILC_RET__LOAD_STRN_GNSS,
	GILC_RET__LOAD_STRN_IMU,
	GILC_RET__LOAD_STRN_ODO,
}gilc_ret_e;

typedef enum gilc_gnssVel_mode {
	GILC_GNSSVEL_MODE__ECEF = 0,
	GILC_GNSSVEL_MODE__SPEED_HEADING,
}gilc_gnssVel_mode_e;

typedef enum gilc_gnssPos_mode {
	GILC_GNSSPOS_MODE__LLA_RAD = 0,
	GILC_GNSSPOS_MODE__LLA_DEG,
}gilc_gnssPos_mode_e;

typedef enum gilc_outRefer_point {
	GILC_OUTREFER_POINT__IMU = 0,
	GILC_OUTREFER_POINT__GNSS,
	GILC_OUTREFER_POINT__REAR_CENTRE,
	GILC_OUTREFER_POINT__INPUT
}gilc_outRefer_point_e;

#pragma pack (4)
typedef struct odometry_data_double {
	double second;
	double wheel_heading;             //deg
	double wheel_heading_std;         //deg
	double wheel_vel_left;            //km/h 
	double wheel_vel_right;           //km/h 
	double wheel_vel;                 //km/h  
	double wheel_vel_std;             //km/s
	double wheel_distance_left;       //km 
	double wheel_distance_right;      //km 
	double wheel_distance;            //km
	double wheel_distance_std;        //km
	int    wheel_mode;                // 0:N; 1:D; 2:R; 3:P
	int    use_flag;                  // 0bit,heading;1~3bit,vel; 4~6bit, distance; 7bit mode
}odo_data_t;

typedef struct mems_data_double {
	double alldata[10];
	double accel[3];                     //accelerometer  X-Y-Z output (m/s2)
	double gyro[3];                      //gyroscope X-Y-Z output(rad/s)
	double mag[3];                       //magnetometer
	double temper;
}mems_data_t;

typedef struct gnss_data_double {
	double second;
	int week;
	double lat;  //rad
	double lon;  //rad
	double alt;  //m  
	double vx_ecef;   //m/s
	double vy_ecef;   //m/s
	double vz_ecef;   //m/s
	double std_lat;  //m
	double std_lon;  //m
	double std_alt;  //m
	double std_vx_ecef;  //m/s
	double std_vy_ecef;  //m/s
	double std_vz_ecef;  //m/s
	int stat;
	int nsused;//使用卫星数
	int ns;    //可见卫星数
	int snsum; //信噪比和
	double age;
	double hdop; 
	double pdop;
	double vdop;
	double tdop;
	double gdop;
	double speed;   //m/s
	double heading;   //

	//双天线信息
	int pos_type2;
	double baseline; //基线长,m
	double heading2; //双天线航向,deg
	double pitch2; //deg
	double std_heading2;  //deg
	double std_pitch2;  //deg
	int nsused2;//辅天线使用卫星数
	int ns2;    //辅天线搜星数

	int pos_type;
	int sol_status;
	int ext_sol_status;
	int time_status;
	int gnss_used_flag;/*bit0:GPS; bit1:glonass; bit2:galileo; bit3:beidou*/
}gnss_data_t;

typedef struct ekf_X_double {
	double Att[3];
	double Vel[3];                 
	double Pos[3];                   
	double GyoBias[3];                
	double AccBias[3];                
	double InstallErr[3];             
	double Lever[3]; 
	double WheelHeadingScale;
	double Kwh; /*方向盘转角——轮转角，比例系数*/
	double Bwh; /*方向盘转角——轮转角，截距*/
	double Kd;  /*车速/轮速传感器——车速，比例系数*/
	double DualAntErr;             
}ekf_X_t;

typedef struct gilc_cfg_double {
	/*DEBUG PARAM*/
	int    debug_level;
	char   debug_outfile_path[128];
	char   debug_tmpfile_path[128];
	bool bFilePathCfgUse;      /*启用文件路径      自定义位置*/
	bool bOutFileSaveClose;    /*关闭结算结果保存*/
	bool bTmpFileSaveClose;    /*关闭结果文件保存*/
	
	/*IMU PARAM*/
	int    imu_period_ms;
	double gyro_std[3];
	double accle_std[3];
	double gyro_walk[3];
	double vel_walk[3];
	int gyro_row[3];
	int acc_row[3];
	double gyro_scale[3];
	double acc_scale[3];
	bool bStdCfgUse;           /*启用IMU噪声       自定义配置*/
	bool bWalkCfgUse;          /*启用IMU随机游走   自定义配置*/
	bool bRowScaleCfgUse;      /*启用IMU顺序、比例 自定义配置*/
	
	/*GNSS PARAM*/
	bool bGnssPosStdUse;       /*启用GNSS位置噪声观测量*/
	bool bGnssVelStdUse;       /*启用GNSS速度噪声观测量*/
	gilc_gnssVel_mode_e eGnssVelMode;          /*0:ecef 1:enu 2:speed+heading*/
	gilc_gnssPos_mode_e eGnssPosMode;
	
	/*CALIBRATE PARAM*/
	ekf_X_t stEkfX_Init;
	bool bEkfXUse;             /*启用EKF X 初始化配置*/
	
	/*INSTALL/CAR PARAM*/
	float fIns2BodyAngle[3];     /*惯导到车辆后轮中心夹角*/
	float fIns2BodyAngleErr[3];  /*惯导到车辆后轮中心夹角-配置误差*/
	float fIns2BodyVector[3];    /*惯导到车辆后轮中心矢量*/
	float fIns2BodyVectorErr[3]; /*惯导到车辆后轮中心矢量-配置误差*/
	float fIns2GnssAngle[3];     /*惯导到GNSS定位主天线夹角*/
	float fIns2GnssAngleErr[3];  /*惯导到GNSS定位主天线夹角-配置误差*/
	float fIns2GnssVector[3];    /*惯导到GNSS定位主天线矢量*/
	float fIns2GnssVectorErr[3]; /*惯导到GNSS定位主天线矢量-配置误差*/
	float fOdoWheelVelScale[2];  /*里程计轮速系数 [0]:left;[1]:right*/
	float fOdoWheelYawScale;     /*里程计轮转角系数*/
	float fWheelDistance[2];     /*轮距 [0]:left-right;[2]front-rear*/
	int   iOutReferPoint;        /*输出参考点位置 0:IMU; 1:GNSS ANT; 2:车辆后轴中心*/
	int   iWorkMode;             /*1:common car 2:Low Speed Car 3:Track 4:farm machinery*/
	int   iReserved;
}gilc_cfg_t;

//gilc raw input
typedef struct gilc_raw_double {
	double imutimetarget;    //second in gnss week, sync time by pps
	mems_data_t memsdate;
	gnss_data_t gnssdata;
	odo_data_t ododata;
	int  gnss_delay_ms;
	int  odo_delay_ms;
	bool bMEMSavail;
	bool bGPSavail;
	bool bPPSavail;
	bool bODOavail;
}gilc_raw_t;

//measure result output
typedef struct gilc_result_double {
	double second;    //sec of gnss week
	int    week;
	double acc_car[3];
	double acc[3];
	double gyro[3];
	double lla[3];     //lat/lon/alt -- deg/deg/m
	double std_lla[3]; //lat/lon/alt -- m/m/m
	double vel_enu[3]; //m/s
	double std_vel[3]; //m/s
	double pitch;      //deg
	double roll;       //deg
	double yaw;        //deg
	double std_pry[3]; //deg/deg/deg
	double speed;      //m/s
	double std_speed;  //m/s
	double heading;    //deg
	int    bPPSSync;
	int    bHeadingOk; //航向可用标志
	int    iSensorFlag; //bit0 ~7 , IMU: acc\gyro\mag\temp; 
	                    //bit8 ~15,GNSS: pos\std_pos\vel\std_vel\heading2\std_heading2\pitch\std_pitch;
	                    //bit16~23,  DR: dual wheelvel\; 
	int    car_status;  //0.uninit 1.static 2.direct driver 3.turn right 4.turn left 5. back
	int    gilc_status;      //gilc_ret_e定义  0.uninit 1.resolving 2.convergence 3.ins  
	int    gilc_status_chc;  //gilc_status_chc_e定义，详见GPCHC status 字段含义
}gilc_result_t;
#pragma pack ()

/**
* call this function in solution cycle to perform the combine navigation solution
* @param avg: public data structure for IMU/GNSS raw date and computed results
*/
int GILC_Init(gilc_cfg_t* cfgdata);
gilc_ret_e GILC_LoadRaw_byStrn(char *buff, gilc_raw_t *pRaw);
gilc_ret_e GILC_PROCESS_Vehicle(gilc_raw_t* pstRaw, gilc_result_t* pstOut);
gilc_ret_e GILC_PROCESS_Vehicle_byStrn(char* buff, gilc_result_t* pstOut);
int GILC_Get_EKF_X(ekf_X_t *pstEkfX);

#ifdef __cplusplus
}
#endif
#endif
