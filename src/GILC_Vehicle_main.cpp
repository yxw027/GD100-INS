// GILC_Vehile_test.cpp : 定义控制台应用程序的入口点。
//
#ifdef WIN32 
#include "stdafx.h"
#endif
#ifdef __linux
#include <memory.h>
#endif

//#include <iostream>
//#include <Eigen/Dense>
//using namespace Eigen;     // 改成这样亦可 using Eigen::MatrixXd;
//using namespace std;

#include <time.h>
#ifdef WIN32
#include <windows.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include "configure.h"
#include "configure_calibrate.h"
#else
#  include <sys/time.h>
#endif
#include "hc_type.h"
#include "gilc_out_pack.h"
#include "GILC.h"
#include "Serial_Com.h"
//#include "GILC_TEST_FileLoad.h"
//#include "GILC_Test_Config_FileLoad_nx200_adis16445.h"
#include "GILC_Test_Config_FileLoad_P2C_demo_imu381.h"
//#include "GILC_Test_Config_FileLoad_P2C_demo2_imu381.h"
//#include "GILC_Test_Config_FileLoad_P2C_demo_adis16465.h"
//#include "GILC_Test_Config_FileLoad_P2C_demo2_adis16465.h"
//#include "GILC_Test_Config_FileLoad_NX200_demo_imu381.h"
#if NX200_IMU381
#include "NX200_IMU381.h"
#endif
#if PCA_IMU381
#include "pca_IMU381.h"
#endif
#if CGI310_murata
#include "CGI310-murata.h"
#endif
#if AP100_ADI16445
#include "AP100_ADI445.h"
#endif


#if I90_IMU381
#include "I90_IMU381.h"
#endif
typedef struct imu_data {
	int    iAccel[3]; /*x,y,z*/
	int    iGyro[3];  /*x,y,z*/
	int    iMag[3];   /*x,y,z*/
	int    iTemp;
	double dAccel[3];                     //accelerometer  X-Y-Z output (m/s2)
	double dGyro[3];                      //gyroscope X-Y-Z output(rad/s)
	double dMag[3];                       //magnetometer
	double dTemp;
}imu_data_t;

#if 1/*A级轿车*/
#define CFG_WHEEL_TRACK  1.7f
#define CFG_WHEEL_BASE   2.7f
#elif 0/*EVCAR电动小车*/
#define CFG_WHEEL_TRACK  1.2f
#define CFG_WHEEL_BASE   2.0f
#else/*陕汽电动小巴*/
#define CFG_WHEEL_TRACK  2.1f
#define CFG_WHEEL_BASE   3.9f
#endif

HC_INT32 s_iGilcNmeaOutPeriod_ms  = 200;     
HC_INT32 s_iGilcCfgUpdatePeriod_ms  = 60000;
static HC_UINT64 s_iOutCasco_time = 0;
gnss_data_t stGnssData = { 0 };
static HC_INT32 s_iCalibrateCfgUsed = 0;
void To_hex(unsigned int value, char buffer[], int length);
#ifdef WIN32
int gettimeofday(struct timeval *tp, void *tzp)
{
	time_t clock;
	struct tm tm;
	SYSTEMTIME wtm;
	GetLocalTime(&wtm);
	tm.tm_year = wtm.wYear - 1900;
	tm.tm_mon = wtm.wMonth - 1;
	tm.tm_mday = wtm.wDay;
	tm.tm_hour = wtm.wHour;
	tm.tm_min = wtm.wMinute;
	tm.tm_sec = wtm.wSecond;
	tm.tm_isdst = -1;
	clock = mktime(&tm);
	tp->tv_sec = (long)clock;
	tp->tv_usec = wtm.wMilliseconds * 1000;
	return (0);
}
#endif

void imu_test(imu_data_t stImuData, double dt, double	imu_gyro_deg[3])
{
	int i;
	static	int bias_cnt = 0;
	static	int init_flag = 0;
	static  double	imu_acc_sum[3] = { 0.0 };
	static  double	imu_gyro_sum[3] = { 0.0 };
	//static  double	imu_acc_bias[3] = {0.0};
	static  double	imu_gyro_bias[3] = { 0.0 };

	if (!init_flag)
	{
		if (bias_cnt < 100 * 10)
		{
			bias_cnt++;
			for (i = 0; i<3; i++)
			{
				//imu_acc_sum[i] +=  stImuData.dAccel[i];
				imu_gyro_sum[i] += stImuData.dGyro[i];
			}
		}
		else
		{
			init_flag = 1;
		}
		for (i = 0; i<3; i++)
		{
			//imu_acc_bias[i] =  imu_acc_sum[i]/bias_cnt;
			imu_gyro_bias[i] = imu_gyro_sum[i] / bias_cnt;
		}
		printf("Get GYRO BIAS: X,%3.3f	Y,%3.3f  Z,%3.3f\r\n", imu_gyro_bias[0], imu_gyro_bias[1], imu_gyro_bias[2]);
	}
	else
	{
		for (i = 0; i<3; i++)
		{
			if (dt < 0.2)
				imu_gyro_deg[i] += (stImuData.dGyro[i] - imu_gyro_bias[i])*dt;

			if (imu_gyro_deg[i]>360)
				imu_gyro_deg[i] -= 360;
			if (imu_gyro_deg[i]<0)
				imu_gyro_deg[i] += 360;
		}
		printf("GYRO DEG: X,%4.3f	Y,%4.3f   Z,%4.3f  ", imu_gyro_deg[0], imu_gyro_deg[1], imu_gyro_deg[2]);
	}
}

FILE *fd_rst = NULL;
HC_VOID gilc_save__rst_data_save( HC_UINT8 *data, HC_UINT32 len )
{
	if (fd_rst)
	{
		//gilc_log_hex("data       : ", 50, data, len);
		//printf("fwrite len %d\r\n", len);
		len = fwrite(data,len,1,fd_rst);
		//fflush(fd_rst);
	}
}

HC_VOID gilc_process__output_casco(gilc_raw_t* pstRaw, gilc_result_t *pstOut)
{
	HC_UINT8 cBuffer[1024] = { 0 };
	HC_UINT16  sSaveLen = 0;
	HC_UINT64 weektime_new = (HC_UINT64)(pstRaw->gnssdata.week * 7 * 24 * 3600 * 1000.0 + pstRaw->gnssdata.second*1000.0);
	if (weektime_new > s_iOutCasco_time)
	{
		sSaveLen = GILC_IOOut_CreatCascoPack(cBuffer,pstRaw,pstOut);
		gilc_save__rst_data_save(cBuffer, sSaveLen);
		//Write2FIFO(cBuffer, sSaveLen, COM2);
		s_iOutCasco_time = weektime_new;
	}
	else
	{
		printf("ERR: Gnss remove by code, new time %.3f, last output time %.3f------------\r\n", pstRaw->gnssdata.second, (HC_DOUBLE)(s_iOutCasco_time % (7 * 24 * 3600 * 1000)) / 1000);
	}
}

HC_VOID gilc_process__rst_prc(gilc_ret_e eGilcRet, gilc_raw_t *pstRaw, gilc_result_t *pstOut)
{
	static HC_UINT64 gilc_out_com_ctl_times = 0;
	static HC_UINT64 gilc_out_ctl_times = 0;
	static HC_UINT64 gilc_cfg_ctl_times = 0;

	HC_UINT8 cBuffer[1024] = { 0 };
	HC_UINT16  sSaveLen = 0;
#if 0
	//printf("gilc ret %d\r\n",eGilcRet);
	if (eGilcRet == GILC_RET__RST_INITING || eGilcRet == GILC_RET__RST_RESOLVING)
	{
		imu_data_t stImuData;
		stImuData.dAccel[0] = pstRaw->memsdate.accel[0];
		stImuData.dAccel[1] = pstRaw->memsdate.accel[1];
		stImuData.dAccel[2] = pstRaw->memsdate.accel[2];
		stImuData.dGyro[0] = pstRaw->memsdate.gyro[0];
		stImuData.dGyro[1] = pstRaw->memsdate.gyro[1];
		stImuData.dGyro[2] = pstRaw->memsdate.gyro[2];

		struct timeval tv_last = { 0 }, tv_new = {0};
		gettimeofday(&tv_new, NULL);
		if (tv_last.tv_sec == 0)
			tv_last = tv_new;
		double dt = (tv_new.tv_sec - tv_last.tv_sec) + (tv_new.tv_usec - tv_last.tv_usec)*1e-6;
		tv_last = tv_new;

		double	imu_gyro_deg[3] = { 0 };
		imu_test(stImuData, dt, imu_gyro_deg);
		pstOut->pitch = imu_gyro_deg[0];
		pstOut->roll = imu_gyro_deg[1];
		pstOut->yaw = imu_gyro_deg[2];
		pstOut->heading = imu_gyro_deg[2];
		pstOut->bHeadingOk = 1;
	}
#endif

	if (eGilcRet > 0 && pstOut->week != 0)
	{
		HC_UINT64 weektime_new = (HC_UINT64)(pstOut->week * 7 * 24 * 3600 * 1000.0 + pstOut->second*1000.0);
		HC_INT32 iTimeSec = round(pstOut->second * 1000);

		if (gilc_out_ctl_times == 0 && (iTimeSec%100) == 0)
		{
			gilc_out_ctl_times = weektime_new - (weektime_new % 200);
			printf("gilc_out_ctl_times init: %d,%.6f;%llu,%llu\r\n", pstOut->week, pstOut->second, weektime_new, gilc_out_ctl_times);
		}
		if (gilc_out_ctl_times != 0 && weektime_new >= gilc_out_ctl_times)
		{
			gilc_out_ctl_times += GILC_NMEA_OUT_PERIOD_MS;
			sSaveLen = GILC_IOOut_CreatNmeaPack(cBuffer, pstRaw, pstOut,0x0007);
			if (sSaveLen)
				gilc_save__rst_data_save(cBuffer, sSaveLen);
			//NMEA_CHC_recv((char*)cBuffer,sSaveLen);
			//gilc_log_hex("nmea: ",30,cBuffer, sSaveLen);
			
			//printf("POS: %8.3f   %d ;",pstOut->second,pstOut->week);
			//printf("POS: %4.3f	 %4.3f  %4.3f ;",pstOut->lla[0],pstOut->lla[1],pstOut->lla[2]);
			//printf("STD: %4.3f	 %4.3f  %4.3f ;",pstOut->std_lla[0],pstOut->std_lla[1],pstOut->std_lla[2]);
			//printf("VEL: %4.3f	 %4.3f  %4.3f ;",pstOut->vel_enu[0],pstOut->vel_enu[1],pstOut->vel_enu[2]);
			//printf("STD: %4.3f	 %4.3f  %4.3f ;",pstOut->std_vel[0],pstOut->std_vel[1],pstOut->std_vel[2]);
			//printf("PRY: %4.3f	 %4.3f  %4.3f ;",pstOut->pitch,pstOut->roll,pstOut->yaw);
			//printf("STD: %4.3f	 %4.3f  %4.3f ;",pstOut->std_pry[0],pstOut->std_pry[1],pstOut->std_pry[2]);
			//printf("SPEED: %4.3f; HEADING  %4.3f;",pstOut->speed,pstOut->heading);
			//printf("FLAG: %d %d %d %d %d\r\n",pstOut->bPPSSync,pstOut->bHeadingOk,pstOut->iSensorFlag,pstOut->car_status,pstOut->gilc_status);
			//printf("\r\n");
		}
		/*
		if (gilc_out_com_ctl_times == 0)
		{
			gilc_out_com_ctl_times = weektime_new - (weektime_new%s_iGilcNmeaOutPeriod_ms);
			printf("gilc_out_com_ctl_times init: %d,%.6f;%llu,%llu\r\n", pstOut->week, pstOut->second, weektime_new, gilc_out_ctl_times);
		}
		if (weektime_new >= gilc_out_com_ctl_times)
		{
			gilc_out_com_ctl_times += s_iGilcNmeaOutPeriod_ms;
			sSaveLen = GILC_IOOut_CreatNmeaPack(cBuffer, pstRaw, pstOut,0x0007);
			if (sSaveLen && s_iOutMsg_flag)
				Write2FIFO(cBuffer, sSaveLen, COM2);
		}
		*/
#ifdef WIN32
//#if GILC_CALIBRATE_SAVE_USED
		if (gilc_cfg_ctl_times == 0)
		{
			gilc_cfg_ctl_times = weektime_new - (weektime_new%s_iGilcCfgUpdatePeriod_ms);
			printf("gilc_cfg_ctl_times init: %d,%.6f;%llu,%llu\r\n", pstOut->week, pstOut->second, weektime_new, gilc_cfg_ctl_times);
		}

		if (weektime_new >= gilc_cfg_ctl_times)
		{
			HC_INT8 cFilePath[128] = { 0 };
			sprintf((char *)cFilePath, "%sOut/gilc_calibrate_cfg", TEST_RAW_FILE_PATH);

			config_calibrate_t stCfgData;
			memset(&stCfgData, 0, sizeof(stCfgData));

			stCfgData.time[0] = pstOut->week;
			stCfgData.time[1] = pstOut->second;
			GILC_Get_EKF_X(&stCfgData.stEkfX);
			stCfgData.iCrc = hc_crc32(&stCfgData,(HC_INT32)&stCfgData.iCrc-(HC_INT32)&stCfgData);

			gilc_cfg_ctl_times += s_iGilcCfgUpdatePeriod_ms;
			if (!s_iCalibrateCfgUsed && eGilcRet >= GILC_RET__RST_STABLE)
			{
				//static int update_flag = 0;
				//if(!update_flag) /*GILC_RET__RST_STABLE后1分钟内，更新配置*/
				{
					//update_flag++;
					//config_calibrate__update(&stCfgData);/*dsf90:imx6ul耗时约50ms，影响imu数据采集*/ 
					gilc__calibrate_cfg_save2(cFilePath,&stCfgData);
				}
			}
		}
#endif
	}
	else if (eGilcRet < 0 && eGilcRet != -3)
	{
		printf("gilc ret %d, time %d ,%.3f\r\n", eGilcRet, pstOut->week, pstOut->second);
	}
#if 0		
	if (eGilcRet <= 0)
	{
		printf("Gilc ret %d, string: %s\r\n", eGilcRet, (HC_INT8 *)dat);
	}
	else
	{
		static int cnt = 0;
		if (++cnt == 100)
		{
			cnt = 0;
			logi("Gilc ret %d, GNSS+IMU:%f,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d\n",
				eGilcRet,
				pstOut->second, pstOut->week,
				pstOut->lla_imu[0], pstOut->lla_imu[1], pstOut->lla_imu[2],
				pstOut->lla_ant[0], pstOut->lla_ant[1], pstOut->lla_ant[2],
				pstOut->ver_enu[0], pstOut->ver_enu[1], pstOut->ver_enu[2],
				pstOut->pitch*R2D, pstOut->roll*R2D, pstOut->yaw*R2D,
				pstOut->speed, pstOut->heading,
				pstOut->car_status, pstOut->gilc_status);
		}
		}
#endif
}


int gilc_process_main(char *cRawFilePath,char *cRawFileName,char *cOutFilePath,char *cTmpFilePath)
{
	int iRet = 0;
	gilc_ret_e eGilcRet = GILC_RET__RST_NOTHING;
	gilc_raw_t stRaw = { 0 }, stRawTmp = { 0 };
	gilc_result_t stOut = { 0 };
	gilc_cfg_t stCfg = { 0 };
	char buff[MAXLEN] = { 0 };
	FILE *fd = NULL;
	FILE* gilc_fp = NULL;
	char lcfile[1024] = {0};
	char lcfile_outpath[1024] = {0};
	char lcfile_tmppath[1024] = {0};
	unsigned long long gilc_out_ctl_times = 0;
	char resultname[1024] = "G:/无人船数据/I90测试数据/GILC_result.txt";
	gilc_fp = fopen(resultname, "wt");
	sprintf(lcfile,"%s%s",cRawFilePath,cRawFileName);
	sprintf(lcfile_outpath,"%s",cOutFilePath);
	sprintf(lcfile_tmppath,"%s",cTmpFilePath);

#if 0
	HC_BOOL s_bGilcProcessRt_flag = 0;
	HC_BOOL s_bGilcProcessRtStrn_flag = 0;
	HC_INT32 s_iGilcNmeaOutPeriod_ms = 0;
	HC_INT32 s_iGilcImuReadPeriod_ms = 0;
	HC_INT32 s_iGilcImuBw_hz = 10;

	HC_INT32  install_mode[3] = { -2,1,3 };/*demo2*/
	HC_DOUBLE lever_init[3] = { 0.0 };
	HC_DOUBLE wheelbase = 0.0, wheeltrack = 0.0;
	int ret;

	if (init_config()<0)
	{
		creat_config();
		config_set_imu_read_freq(100);
		config_set_imu_bw_freq(40);
		config_set_imu_installtype(install_mode);
		config_set_gilc_out_period(20);
		config_set_gilc_process_rt(1);
		config_set_gilc_process_strn(0);
		config_set_gilc_lever(lever_init);
		config_set_gilc_wheeltrack(1.6);
		config_set_gilc_wheelbase(2.7);
		save_config();
	}

	s_iGilcImuBw_hz = config_get_imu_bw_freq();
	s_iGilcImuReadPeriod_ms = 1000 / config_get_imu_read_freq();
	s_iGilcNmeaOutPeriod_ms = config_get_gilc_out_period();
	s_bGilcProcessRt_flag = config_get_gilc_process_rt();
	s_bGilcProcessRtStrn_flag = config_get_gilc_process_strn();

	config_get_imu_installtype(install_mode);
	config_get_gilc_lever(lever_init);

	wheeltrack = config_get_gilc_wheeltrack();
	wheelbase = config_get_gilc_wheelbase();

	printf("config_get_imu_install_xyz = %d %d %d\r\n", install_mode[0], install_mode[1], install_mode[2]);
	printf("config_get_imu_bw_freq = %d\r\n", s_iGilcImuBw_hz);
	printf("config_get_imu_period = %d\r\n", s_iGilcImuReadPeriod_ms);
	printf("config_get_gilc_out_period = %d\r\n", s_iGilcNmeaOutPeriod_ms);
	printf("config_get_gilc_process_rt = %d\r\n", s_bGilcProcessRt_flag);
	printf("config_get_gilc_process_strn = %d\r\n", s_bGilcProcessRtStrn_flag);
	printf("config_get_gilc_lever = %.3f %.3f %.3f\r\n", lever_init[0], lever_init[1], lever_init[2]);
	printf("config_get_gilc_wheeltrack/wheelbase = %.3f %.3f\r\n", wheeltrack, wheelbase);
	return 0;
#endif

#ifdef WIN32
	config_calibrate_t stCfgData;
	HC_INT8 cFilePath[128] = { 0 };
	memset(&stCfgData, 0, sizeof(stCfgData));
	sprintf((char *)cFilePath, "%sOut/gilc_calibrate_cfg", TEST_RAW_FILE_PATH);
	//iRet = config_calibrate__read(&stCfgData);


	iRet = gilc__calibrate_cfg_read2(cFilePath,&stCfgData);
	if(iRet == HC_OK)
	{
		if(stCfgData.iCrc == hc_crc32(&stCfgData,(HC_INT32)&stCfgData.iCrc-(HC_INT32)&stCfgData))
		{
			stCfg.stEkfX_Init = stCfgData.stEkfX;
			stCfg.bEkfXUse = true;
			s_iCalibrateCfgUsed = 1;
		}
		else
		{
			printf("config_calibrate__read crc32 error \r\n");
		}
	}
#endif

#if (!GILC_DEBUG_PROCESS_TIME)
	strncpy(stCfg.debug_outfile_path, lcfile_outpath, sizeof(stCfg.debug_outfile_path));
	strncpy(stCfg.debug_tmpfile_path, lcfile_tmppath, sizeof(stCfg.debug_tmpfile_path));
	stCfg.debug_level = 1;
	stCfg.bFilePathCfgUse = true;
	//stCfg.bOutFileSaveClose = true;
	//stCfg.bTmpFileSaveClose = true;
#endif

#if 1
#if 1
	stCfg.gyro_std[0] = GYRO_STD_X;
	stCfg.gyro_std[1] = GYRO_STD_Y;
	stCfg.gyro_std[2] = GYRO_STD_Z;
	stCfg.accle_std[0] = ACC_STD_X;
	stCfg.accle_std[1] = ACC_STD_Y;
	stCfg.accle_std[2] = ACC_STD_Z;
	stCfg.bStdCfgUse = true;

	stCfg.gyro_walk[0] = stCfg.gyro_walk[1] = stCfg.gyro_walk[2] = GYRO_WALK;
	stCfg.vel_walk[0] = stCfg.vel_walk[1] = stCfg.vel_walk[2] = VEL_WALK;
	stCfg.bWalkCfgUse = true;
#endif

	stCfg.gyro_row[0] = CFG_GYRO_X_ROW;
	stCfg.gyro_row[1] = CFG_GYRO_Y_ROW;
	stCfg.gyro_row[2] = CFG_GYRO_Z_ROW;
	stCfg.acc_row[0] = CFG_ACC_X_ROW;
	stCfg.acc_row[1] = CFG_ACC_Y_ROW;
	stCfg.acc_row[2] = CFG_ACC_Z_ROW;
	stCfg.gyro_scale[0] = CFG_GYRO_X_SCALE;
	stCfg.gyro_scale[1] = CFG_GYRO_Y_SCALE;
	stCfg.gyro_scale[2] = CFG_GYRO_Z_SCALE;
	stCfg.acc_scale[0] = CFG_ACC_X_SCALE;
	stCfg.acc_scale[1] = CFG_ACC_Y_SCALE;
	stCfg.acc_scale[2] = CFG_ACC_Z_SCALE;
	stCfg.bRowScaleCfgUse = true;

	stCfg.fIns2GnssVector[0] = CFG_ARM_ANT_X;
	stCfg.fIns2GnssVector[1] = CFG_ARM_ANT_Y;
	stCfg.fIns2GnssVector[2] = CFG_ARM_ANT_Z;

	stCfg.bGnssPosStdUse = CFG_POS_STD_USE;
	stCfg.bGnssVelStdUse = CFG_VEL_STD_USE;
	stCfg.eGnssVelMode = CFG_GNSSVEL_MODE;
	stCfg.eGnssPosMode = CFG_GNSSPOS_MODE;
	//stCfg.iOutReferPoint = GILC_OUTREFER_POINT__REAR_CENTRE;
	stCfg.iOutReferPoint = GILC_OUTREFER_POINT__GNSS;
	//stCfg.iOutReferPoint = GILC_OUTREFER_POINT__IMU;
	stCfg.fWheelDistance[0] = CFG_WHEEL_TRACK;
	stCfg.fWheelDistance[1] = CFG_WHEEL_BASE;
	//stCfg.iWorkMode = GILC_WORK_MODE__CAR_NORMAL;
	stCfg.iWorkMode = GILC_WORK_MODE__DEFAULT;

	stCfg.fIns2BodyVector[0] = 0;
	stCfg.fIns2BodyVector[1] = 2.43;
	stCfg.fIns2BodyVector[2] = -1;

	stCfg.fIns2GnssVector[0] = 0.1;
	stCfg.fIns2GnssVector[1] = 0.1;
	stCfg.fIns2GnssVector[2] = 0.3;
	
	/*
	stCfg.fIns2BodyAngle[0] = 0;
	stCfg.fIns2BodyAngle[1] = 0;
	stCfg.fIns2BodyAngle[2] = -90;
	*/
#endif
	GILC_Init(&stCfg);

	fd = fopen(lcfile, "rt");
	if (!fd)
	{
		printf("open file err! %s\r\n", lcfile);
#ifdef WIN32
		system("pause");
#endif
		return 0;
	}

	static unsigned long long llu_once = 0, llu_ave = 0, llu_total = 0, llu_cnt = 0;
	//struct timeval tv, tv_last;
	while (1)
	{
#if 0
		/*dsf90:debug 重新初始化、文件存储测试*/
		static long lDebug_cnt = 0;
		if (lDebug_cnt++ == 80000)
		{
			GILC_Init(&stCfg);
		}
#endif		
		if (feof(fd))
		{
			printf("file read over!\n");
			if (fd)
				fclose(fd);
			if (fd_rst)
				fclose(fd_rst);
			break;
		}

		fgets(buff, MAXLEN, fd);
		if (strlen(buff)<18)
		{
			printf("file read error: %s\n", buff);
			continue;
		}

#if 1		
		/*dsf90:debug 接口测试*/
		//memset(&stRaw, 0, sizeof(stRaw));
		eGilcRet = (gilc_ret_e)GILC_LoadRaw_byStrn(buff, &stRawTmp);
		if (eGilcRet == -1)
		{
			printf("GILC_Load param err!\r\n");
			continue;
		}
		else if (eGilcRet == -2)
		{
			printf("GILC_Load fail, unknow strning: %s\n", buff);
			continue;
		}

		if (stRawTmp.bODOavail)
		{
			stRaw.bODOavail = stRawTmp.bODOavail;
			stRaw.ododata = stRawTmp.ododata;
		}

		if (stRawTmp.bGPSavail)
		{
			stRaw.bGPSavail = stRawTmp.bGPSavail;
			stRaw.gnssdata = stRawTmp.gnssdata;
			stRaw.gnss_delay_ms = stRawTmp.gnss_delay_ms;
		}

		if (stRawTmp.bMEMSavail)
		{
			stRaw.bMEMSavail = stRawTmp.bMEMSavail;
			stRaw.bPPSavail = stRawTmp.bPPSavail;
			stRaw.imutimetarget = stRawTmp.imutimetarget;
			stRaw.memsdate = stRawTmp.memsdate;
		}

#if 0		
		if(stRaw.bODOavail)
		{
			static double wheel_vel_last = 0;
			if (!wheel_vel_last)
				wheel_vel_last = stRaw.ododata.wheel_vel;
			else if (stRaw.ododata.wheel_vel == wheel_vel_last)
			{
				stRaw.bODOavail = false;
				continue;
			}
			wheel_vel_last = stRaw.ododata.wheel_vel;

			static int dr_cnt = 0;
			if(++dr_cnt == 10) 
			{
				dr_cnt = 0;
			}
			else
			{
				//stRaw.bODOavail = false;
				//continue;
			}
		}
#endif

		static double start_time = 0.0;
		if ((stRaw.gnssdata.second > start_time + 3 * 60 && stRaw.gnssdata.second <  start_time + 4 * 60))
		{
			stRaw.bGPSavail = false;
			//stRaw.bODOavail = false;
		}

		if (stRaw.bODOavail)
		{
			//stRaw.bODOavail = false;
			//continue;
		}
		
		if (stRaw.gnssdata.second <= 111627)
		{
			//stRaw.bGPSavail = false;
			//continue;
			//stRaw.bODOavail = false;
		} 
#if 0
		static long lRawStart = 0;
		long lRawNum = 0;

		if (!lRawStart)
		{
			if (start_time)
				lRawStart = (long)start_time;
		}
		else
		{
			if (stRaw.bGPSavail)
				lRawNum = (long)stRaw.gnssdata.second - lRawStart;
			else
				lRawNum = (long)stRaw.imutimetarget - lRawStart;

			if (lRawNum > 5 * 60)
			{
				long lRaw_tmp = lRawNum % (10 * 60);
				if (lRaw_tmp <= 3 * 60)
					stRaw.bGPSavail = false;
			}
		}
#else

#endif		

#ifdef OUT_CASCO
		if (stRawTmp.bGPSavail)
		{
			stGnssData = stRawTmp.gnssdata;
			gilc_process__output_casco(&stRaw, &stOut);
		}
#endif

		if (stRawTmp.bODOavail)
		{
			continue;
		}
		memset(&stOut, 0, sizeof(stOut));
		eGilcRet = GILC_PROCESS_Vehicle(&stRaw, &stOut);
		//fprintf(gilc_fp, "%.3f,%.3f\n", stOut.yaw, stOut.heading);

		struct GINS_DATA gins_data;
		//gins_data.roll = 200;
		//gins_data.pitch = 500;
		//gins_data.yaw = 4500;
		//gins_data.none = 0xffff;
		//gins_data.gyro[0] = 0.01;
		//gins_data.gyro[1] = 0.02;
		//gins_data.gyro[2] = 0.03;
		//gins_data.accl[0] = -0.2;
		//gins_data.accl[1] = -0.5;
		//gins_data.accl[2] = -9.8;
		//gins_data.lng = 315000000;
		//gins_data.lat = 1215000000;
		//gins_data.vel_x = 3000;
		//gins_data.vel_y = 4000;
		int32_t  gilc_alt;
		int32_t  gilc_lon;
		int32_t  gilc_lat;
		int16_t  gilc_roll;
		int16_t  gilc_pitch;
		int16_t  gilc_yaw;
		float gilc_gyro[3];
		float gilc_acc[3];
		int16_t gilc_vx;
		int16_t gilc_vy;
		gilc_lon = stOut.lla[0] * 10000000 ;
		gilc_lat = stOut.lla[1] * 10000000;
		//printf("%f,%f\r\n", stOut.lla[0] , stOut.lla[1] );
		gilc_roll = stOut.roll * 100;
		gilc_pitch = stOut.pitch * 100;
		gilc_yaw = stOut.yaw * 100;
		gilc_vx = stOut.vel_enu[0]*100;
		gilc_vy = stOut.vel_enu[1] * 100;
		gilc_gyro[0]=stOut.gyro[0]*D2R;
		gilc_gyro[1] = stOut.gyro[1] * D2R;
		gilc_gyro[2] = stOut.gyro[2] * D2R;
		gilc_acc[0] = stOut.acc[0] * 9.80665;
		gilc_acc[1] = stOut.acc[1] * 9.80665;
		gilc_acc[2] = stOut.acc[2] * 9.80665;

		gins_data.roll = gilc_roll;
		gins_data.pitch = gilc_pitch;
		gins_data.yaw = gilc_yaw;
		gins_data.none = 0xffff;
		gins_data.gyro[0] = gilc_gyro[0];
		gins_data.gyro[1] = gilc_gyro[1];
		gins_data.gyro[2] = gilc_gyro[2];
		gins_data.accl[0] = gilc_acc[0];
		gins_data.accl[1] = gilc_acc[1];
		gins_data.accl[2] = gilc_acc[2];
		gins_data.lng = gilc_lon;
		gins_data.lat = gilc_lat;
		gins_data.vel_x = gilc_vx;
		gins_data.vel_y = gilc_vy;


		char msg[47];
		char crc = 0;
		char *sp = (char *)&gins_data;
		msg[0] = 'S';
		for (int i = 0; i < 44; i++) {
			msg[i + 1] = *sp;
			crc ^= *sp;
			sp++;
		}
		msg[45] = crc;


		if (eGilcRet > 0)
		{
			fprintf(gilc_fp, "%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d\r\n",
				gilc_roll, gilc_pitch, gilc_yaw, gilc_gyro[0], gilc_gyro[1], gilc_gyro[2],
				gilc_acc[0], gilc_acc[1], gilc_acc[2], gilc_lon, gilc_lat,
				gilc_vx, gilc_vy);
			int serial_flag = 0;
			if (WIN32)
			{
				//serial_flag = win_reloy(msg);//串口数据发送
			}
			else
			{
				//serial_flag = linux_reloy(msg);//串口数据发送
			}
		}


		//printf("%d\n", sizeof(gilc_buf));

		//printf("eGilcRet=%d\n", eGilcRet);
		if (!start_time && stRaw.imutimetarget && eGilcRet >= GILC_RET__RST_STABLE)
			start_time = stRaw.imutimetarget;

#else
		gettimeofday(&tv_last, NULL);

		/*dsf90:debug 接口测试*/
		memset(&stOut, 0, sizeof(stOut));
		eGilcRet = GILC_PROCESS_Vehicle_byStrn(buff, &stOut);

		gettimeofday(&tv, NULL);		//get system time here to get ms info, tv.tv_usec means ms
		llu_once = (tv.tv_sec - tv_last.tv_sec) * 1000000u + tv.tv_usec - tv_last.tv_usec;
		llu_cnt++; llu_total += llu_once; llu_ave = llu_total / llu_cnt;

#if GILC_DEBUG_PROCESS_TIME
		printf("---debug process time---:count,%12llu; onec,%12llu; total,%12llu; ave,%12llu\r\n",
			llu_cnt, llu_once, llu_total, llu_ave);
#endif
#endif
		gilc_process__rst_prc(eGilcRet, &stRaw,&stOut);
		stRaw.bGPSavail = 0;
		stRaw.bMEMSavail = 0;
		stRaw.bODOavail = 0;
		stRaw.bPPSavail = 0;

#ifdef OUT_CASCO
		HC_UINT64 weektime_new = (HC_UINT64)(stOut.week * 7 * 24 * 3600 * 1000.0 + stOut.second*1000.0);
		if (weektime_new > (s_iOutCasco_time + 200 + 120)) /*GNSS延迟120ms,默认数据丢失*/
		{
			HC_UINT64 gnsstime_new = s_iOutCasco_time + 200;
			stRaw.gnssdata = stGnssData;
			stRaw.bGPSavail = 1;
			stRaw.gnss_delay_ms = 120;
			stRaw.gnssdata.second = ((HC_DOUBLE)(gnsstime_new % (7 * 24 * 3600 * 1000))) / 1000.0;
			stRaw.gnssdata.week = (HC_INT32)(gnsstime_new / (7 * 24 * 3600 * 1000));
			//printf("--- %llu  %llu  %llu----", weektime_new, s_iOutCasco_time, gnsstime_new);
			//printf("imu:%d,%lf  gnss:%d,%lf\r\n", stOut.week, stOut.second, stRaw.gnssdata.week, stRaw.gnssdata.second);
			//printf("GNSS delay/lost, output gilc, new time %.3f, last time %.3f------------\r\n", stRaw.gnssdata.second, (HC_DOUBLE)(s_iOutCasco_time % (7 * 24 * 3600 * 1000)) / 1000);
			gilc_process__output_casco(&stRaw, &stOut);
		}
#endif

		if (!fd_rst && stOut.week)
		{
			char lcfile_rst[128] = {0};
			sprintf(lcfile_rst, "%s%d_%.2f_ReferPoint%d_rst.nmea", TEST_OUT_FILE_PATH, stOut.week,stOut.second, stCfg.iOutReferPoint);
			fd_rst = fopen(lcfile_rst, "wb");
			if (!fd_rst)
				printf("open file err! %s\r\n", lcfile_rst);
		}
	}
	printf("---debug process time---:count,%12llu; onec,%12llu us; total,%12llu s; ave,%12llu us\r\n",
		llu_cnt, llu_once, (llu_total / 1000000u), llu_ave);

	return 0;
}

		
int process_while()
{
	/*
	char cRawFileName[][128] = {
			"012105_raw.txt",
			"061630_raw.txt",
			"100327_raw.txt",
			"115249_raw.txt",
			"121157_raw.txt",
			"122313_raw.txt",
			"125310_raw.txt",
			"130359_raw.txt",
			"134859_raw.txt"
		};*/
	char cRawFileName[][128] = {
		"084808_raw.txt",
		"094844_raw.txt",
		"121359_raw.txt",
		"133237_raw.txt",
		"134649_raw.txt"
	};

	char cOutFilePath[1024] = {0};
	char cTmpFilePath[1024] = {0};
	int i = 0,j=0;
	int num = ARRAY_SIZE(cRawFileName);

	num = 1;
	
#ifdef WIN32
	sprintf(cTmpFilePath,"%s","D:/TMP/");
#else
	sprintf(cTmpFilePath,"%s","/mnt/gilc-data/TMP/");
#endif	
	for(j=0;j<1;j++)
	{
		sprintf(cOutFilePath,"%sOUT_%d/",TEST_RAW_FILE_PATH,j);
		for(i= 0;i<num;i++)
			gilc_process_main(TEST_RAW_FILE_PATH,cRawFileName[i], cOutFilePath, cTmpFilePath);
	}
	return 0;
}

//int eigen_main()
//{
//	MatrixXd m = MatrixXd::Random(3, 3);              //随机生成3*3的double型矩阵
//	m = (m + MatrixXd::Constant(3, 3, 1.2)) * 50;      //MatrixXd::Constant(3,3,1.2)表示生成3*3的double型矩阵，该矩阵所有元素均为1.2
//	cout << "m =" << endl << m << endl;
//	VectorXd v(3);        // 定义v为3*1的double型向量
//	v << 1, 2, 3;         // 向量赋值
//	cout << "m * v =" << endl << m * v << endl;
//	system("pause");
//	return 0;
//}

void To_hex(unsigned int value, char buffer[], int length)
{

	unsigned int i = (sizeof(unsigned int) * 2);
	unsigned int temp;
	int j = 0;
	while (i--)
	{
		temp = (value&(0xf << (4 * i))) >> (4 * i);
		if (temp > 9)
		{
			buffer[j] = 'A' + temp - 10;
		}
		else
		{
			buffer[j] = '0' + temp;
		}
		j++;
	}
	buffer[length] = '\0';
}





int main()
{
	//return eigen_main();

	char cOutFilePath[1024] = { 0 };
	char cTmpFilePath[1024] = { 0 };
	//process_while();
	sprintf(cOutFilePath, "%sOUT/", TEST_RAW_FILE_PATH);
	
#ifdef WIN32
	sprintf(cTmpFilePath,"%s","D:/TMP/");
#else
	sprintf(cTmpFilePath,"%s","/mnt/gilc-data/TMP/");
#endif
	gilc_process_main(TEST_RAW_FILE_PATH, TEST_RAW_FILE_NAME, cOutFilePath, "D:/TMP/");
#ifdef WIN32
	system("pause");
#endif
}

