// GILC_Vehile_test.cpp : 定义控制台应用程序的入口点。
//
#ifdef WIN32 
#include "stdafx.h"
#include <Windows.h>
#include <io.h>
#include <direct.h>
#endif
#ifdef __linux
#include <memory.h>
#include <unistd.h>
#include <sys/stat.h>
#endif

#include "GILC.h"

#include "GILC_Vehicle_inner.h"
//#include "GILC_TEST_FileLoad.h"
//#include "GILC_Test_Config_FileLoad_nx200_adis16445.h"
#include "GILC_Test_Config_FileLoad_P2C_demo_imu381.h"
//#include "GILC_Test_Config_FileLoad_P2C_demo_adis16465.h"

#include <time.h>
#ifdef WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif
#include "gilc_out_pack.h"

//#include "configure.h"
/*起始时间(s)、标定间隔(s)、GNSS屏蔽时间(s)*/
int gilc_calibrated_cfg[][3] = {
	{ 120,1,10 },
	{ 120,1,60 },
	{ 120,1,180 },
};

#define GILC_CALIBRATED_CFG__START 0
#define GILC_CALIBRATED_CFG__NUM   3
//#define GILC_CALIBRATED_CFG__NUM (sizeof(gilc_calibrated_cfg)/sizeof(gilc_calibrated_cfg[0]))

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

int gilc_calibrated_fun(int gilc_calibrated_cfg[3])
{
	gilc_ret_e ret = GILC_RET__RST_NOTHING;
	gilc_raw_t stRaw = { 0 };
	gilc_raw_t stRaw_tmp = { 0 };
	gilc_result_t stOut = { 0 };
	gilc_result_t stOut_tmp = { 0 };
	gilc_result_t stOut_pre = {0};
	gilc_result_t stOut_start = { 0 };
	gilc_cfg_t stCfg = { 0 };
	
	FILE *fd = NULL;
	FILE *fd_rst = NULL;
	FILE *fd_rst_cal = NULL;
	FILE *fd_rst_plot = NULL;
	char cRawPathName[1024] = {0};
	char cRstPathName[1024] = {0};
	char cGilcLibOutPath[1024] = { 0 };
	char cCalRstPathName[1024] = { 0 };
	char cPlotRstPathName[1024] = { 0 };
	char buff[MAXLEN] = { 0 };
	
	int gilc_cal_init = 0;
	int gilc_cal_cnt = 0;
	long int gilc_run_stable_time = 0;
	long int gilc_cal_start_time = 0;
	double gilc_run_distance_total = 0.0;
	double gilc_run_distance_cal_start = 0.0;
	
	long gilc_load_strn_cnt = 0;
	long gilc_load_strn_backup = 0;
	long gilc_load_strn_recovery = 0;

	sprintf(cRawPathName, "%s%s", TEST_RAW_FILE_PATH, TEST_RAW_FILE_NAME);
	sprintf(cGilcLibOutPath, "%sgilc_calibrated_cfg_%d_%d_%d/", TEST_OUT_FILE_PATH,
		gilc_calibrated_cfg[0],
		gilc_calibrated_cfg[1],
		gilc_calibrated_cfg[2]);
	sprintf(cRstPathName, "%s%s", cGilcLibOutPath, TEST_RST_FILE_NAME);
	sprintf(cCalRstPathName, "%s%s.calibrated_%d_%d_%d.txt", TEST_OUT_FILE_PATH,TEST_RAW_FILE_NAME,
		gilc_calibrated_cfg[0],
		gilc_calibrated_cfg[1],
		gilc_calibrated_cfg[2]);
	sprintf(cPlotRstPathName, "%s%s.calibrated_%d_%d_%d.plot", TEST_OUT_FILE_PATH, TEST_RAW_FILE_NAME,
		gilc_calibrated_cfg[0],
		gilc_calibrated_cfg[1],
		gilc_calibrated_cfg[2]);

#ifdef WIN32
#ifdef _DEBUG
	if (_access(TEST_OUT_FILE_PATH, 0) == -1)
		CreateDirectory(TEST_OUT_FILE_PATH, NULL);
	if (_access(cGilcLibOutPath, 0) == -1)
		CreateDirectory(cGilcLibOutPath, NULL);
#endif
#elif defined(__linux)
	char cmd_strn[512] = { 0 };
	if (access(TEST_OUT_FILE_PATH, 0) != 0)
	{
		sprintf(cmd_strn, "mkdir -p %s", TEST_OUT_FILE_PATH);
		system(cmd_strn);
	}
	if (access(cGilcLibOutPath, 0) != 0)
	{
		sprintf(cmd_strn, "mkdir -p %s", cGilcLibOutPath);
		system(cmd_strn);
	}
#endif

#if (!GILC_DEBUG_PROCESS_TIME)
	strncpy(stCfg.debug_outfile_path, cGilcLibOutPath, sizeof(stCfg.debug_outfile_path));
	strncpy(stCfg.debug_tmpfile_path, TEST_TMP_FILE_PATH, sizeof(stCfg.debug_tmpfile_path));
	stCfg.bFilePathCfgUse = true;
	stCfg.bTmpFileSaveClose = true;
	stCfg.bOutFileSaveClose = true;
#endif

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
	stCfg.iOutReferPoint = GILC_OUTREFER_POINT__GNSS;

	GILC_Init(&stCfg);

	fd_rst = fopen(cRstPathName, "wt");
	if (!fd_rst)
	{
		printf("open file err! %s\r\n", cRstPathName);
		system("pause");
		return 0;
	}

	fd_rst_cal = fopen(cCalRstPathName, "wt");
	if (!fd_rst_cal)
	{
		printf("open file err! %s\r\n", cCalRstPathName);
		system("pause");
		return 0;
	}

	fd_rst_plot = fopen(cPlotRstPathName, "wt");
	if (!fd_rst_plot)
	{
		printf("open file err! %s\r\n", cPlotRstPathName);
		system("pause");
		return 0;
	}
	gilc_cal_start_time = 0;
	gilc_load_strn_recovery = 0;
	gilc_cal_cnt = 0;

	while (1)
	{
		int car_status = 0;
		int calibrated_init_flag = 0;
		int bInsMode_detect = 1;
		gilc_cal_init = 0;
		gilc_run_distance_total = 0.0;
		gilc_run_distance_cal_start = 0.0;
		gilc_load_strn_cnt = 0;
		gilc_load_strn_backup = 0;

		fd = fopen(cRawPathName, "rt");
		if (!fd)
		{
			printf("open file err! %s\r\n", cRawPathName);
			if (fd)
				fclose(fd);
			if (fd_rst)
				fclose(fd_rst);
			fd = NULL;
			fd_rst = NULL;
			return 0;
		}

		while (1)
		{
			if (feof(fd))
			{
				printf("file read over!\n");
				goto once_over;
			}

			fgets(buff, MAXLEN, fd);
			if (strlen(buff) < 18)
			{
				printf("file read error: %s\n", buff);
				continue;
			}
			
			/*dsf90:debug 接口测试*/
			//memset(&stRaw, 0, sizeof(stRaw));
			ret = (gilc_ret_e)GILC_LoadRaw_byStrn(buff, &stRaw);
			if (ret == -1)
			{
				printf("GILC_Load param err!\r\n");
				continue;
			}
			else if (ret == -2)
			{
				printf("GILC_Load fail, unknow strning: %s\n", buff);
				continue;
			}
			stRaw_tmp = stRaw;

			gilc_load_strn_cnt++;
			if(gilc_load_strn_recovery)
			{
				if(gilc_load_strn_cnt < gilc_load_strn_recovery)
					continue;
				GILC_process_recovery();
				gilc_load_strn_recovery = 0;
				//printf("%6.6f GILC_process_recovery\r\n",stRaw.imutimetarget);
			}
									
#if 1
			double gilc_pos_total_diff = 0.0;
			double diff_time = 0.0;
			if (gilc_cal_init)
			{
				if (!gilc_cal_start_time && gilc_cal_cnt == 0)
					gilc_cal_start_time = gilc_run_stable_time + gilc_calibrated_cfg[0];

				if (stRaw.imutimetarget > gilc_cal_start_time)
				{
					/*纯惯导模式*/
					if (!calibrated_init_flag)
					{
						calibrated_init_flag = 1;

						gilc_load_strn_backup = gilc_load_strn_cnt;
						GILC_process_backup();
						//printf("%6.6f GILC_process_backup\r\n",stRaw.imutimetarget);

						gilc_run_distance_cal_start = gilc_run_distance_total;
						stOut_start = stOut;
						bInsMode_detect = 1;
					}

					diff_time = stRaw.imutimetarget - gilc_cal_start_time;
					if (diff_time < gilc_calibrated_cfg[2])
					{
						stRaw.bGPSavail = false;
					}
				}
			}
#endif
			
			//memset(&stOut, 0, sizeof(stOut));
			ret = GILC_PROCESS_Vehicle(&stRaw, &stOut);
			if (ret < 0)
			{
				//printf("Gilc ret %d, string: %s\r\n", ret, buff);
			}

			if (!gilc_cal_init && (ret > GILC_RET__RST_RESOLVING) && stRaw.bPPSavail)
			{
				gilc_cal_init = 1;
				gilc_run_stable_time = (long)round(stRaw.imutimetarget);
			}

			/*算法稳定，解算行驶里程*/
			if (ret >= GILC_RET__RST_RESOLVING)
			{
				double difpos[3] = { 0 };
				double dInsPos[3] = { 0 };
				double dInsPos_pre[3] = { 0 };

				if (!stOut_pre.week)
				{
					stOut_pre = stOut;
				}

				dInsPos[0] = stOut.lla[0] * D2R;
				dInsPos[1] = stOut.lla[1] * D2R;
				dInsPos[2] = stOut.lla[2];
				dInsPos_pre[0] = stOut_pre.lla[0] * D2R;
				dInsPos_pre[1] = stOut_pre.lla[1] * D2R;
				dInsPos_pre[2] = stOut_pre.lla[2];

				gilc_run_distance_total += difpos_enu(dInsPos_pre, dInsPos, difpos);
				//printf("---%8.6f---%6.6f\r\n", stRaw.imutimetarget,gilc_run_distance_total);
				stOut_pre = stOut;
			}

			/*非标定时间段————正常组合，保存组合结果*/
			if (fabs(gilc_run_distance_cal_start) < 1e-6)
			{
				//fprintf(fd_rst, stOut.cNmeaStrn);
				fflush(NULL);
			}

			if (calibrated_init_flag)
			{
				if(ret != GILC_RET__RST_INS_MODE)
				{
					bInsMode_detect = 0;
				}
				bInsMode_detect &= bInsMode_detect;
			
				gilc_pos_total_diff = gilc_run_distance_total - gilc_run_distance_cal_start;

				if (diff_time >= gilc_calibrated_cfg[2])
				{
					if (stRaw.bGPSavail && stOut.bPPSSync && (stRaw.gnssdata.stat == 4 || stRaw.gnssdata.stat == 50))
					{
						double difpos[3] = { 0 };
						double difpos_tmp = 0;
						double difpos_en = 0;
						double difpos_u = 0;
						double difpos_percent = 0;

						gilc_result_inner_t stRstInner;
						GILC_get_result_inner(&stRstInner);
						
						difpos[0] = stRstInner.dpos_sync[0];
						difpos[1] = stRstInner.dpos_sync[1];
						difpos[2] = stRstInner.dpos_sync[2];

						difpos_tmp = sqrt(difpos[0] * difpos[0] + difpos[1] * difpos[1] + difpos[2] * difpos[2]);
						difpos_en = sqrt(difpos[0] * difpos[0] + difpos[1] * difpos[1]);
						difpos_u = fabs(difpos[2]);

						difpos_percent = difpos_tmp / gilc_pos_total_diff*100.0;

						/*
						1、标定时段平均车速需大于于0.05
						2、标定时段比设定时间误差不超过1s(剔除由于GNSS不固定导致的,标定超时情况)
						*/
						if (gilc_pos_total_diff > gilc_calibrated_cfg[2] * 0.05 && diff_time - gilc_calibrated_cfg[2] < 1.0)
						{
							gilc_cal_cnt++;
							printf("Run count %4d, ", gilc_cal_cnt);
							if (fd_rst_cal && gilc_pos_total_diff > gilc_calibrated_cfg[2] * 0.01)
							{
								fprintf(fd_rst_cal, "%4d,%3d,%3.3f,%8.6f,%8.6f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%.3f,%d\r\n",
									gilc_cal_cnt, gilc_calibrated_cfg[2], diff_time, stOut_tmp.second, stRaw.gnssdata.second,
									difpos[0], difpos[1], difpos[2], gilc_pos_total_diff, difpos_en, difpos_u, difpos_percent, stOut_start.car_status);

								double pos[3] = { stOut_start.lla[0],stOut_start.lla[1],difpos_en};
								int stat = 0;
								switch (gilc_calibrated_cfg[2])
								{
								case 10:
									stat = (int)(difpos_en / 0.5) + 1;
									break;
								case 60:
									stat = (int)(difpos_en / 8) + 1;
									break;
								case 180:
									stat = (int)(difpos_en / 20) + 1;
									break;
								}
								if (stat > 6) stat = 6;
								printf_rtkplot(fd_rst_plot, stOut_start.week, stOut_start.second, pos, stat);

								fflush(NULL);
							}
						}
						else
						{
							printf("Low speed or gnss not good, ");
						}

						printf("gnss lost times %3d,difftime %3.3f, att %3.3f %3.3f %3.3f, gnss time %8.6f, difpos[3] %6.3f %6.3f %6.3f, difpos_total %6.3f, en %6.3f, u %6.3f, err %.3f%%\r\n",
							gilc_calibrated_cfg[2], diff_time, stOut.pitch*R2D, stOut.roll*R2D, stOut.yaw*R2D, stRaw.gnssdata.second,
							difpos[0], difpos[1], difpos[2], gilc_pos_total_diff, difpos_en, difpos_u, difpos_percent);

						gilc_cal_start_time += gilc_calibrated_cfg[1];

						if (bInsMode_detect)
						{
							//if (gilc_cal_start_time < round(stRaw.imutimetarget) + 60)
							//	gilc_cal_start_time = round(stRaw.imutimetarget) + 60;
							//printf("%.6f gnss no good >20s ......, next %d\r\n", stRaw.imutimetarget, gilc_cal_start_time);
						}

						break;
					}
				}
			}

		}
		if (fd)
			fclose(fd);
		fd = NULL;
		gilc_load_strn_recovery = gilc_load_strn_backup;
	}
once_over:
	if (fd)
		fclose(fd);
	if (fd_rst)
		fclose(fd_rst);
	if (fd_rst_cal)
		fclose(fd_rst_cal);
	if (fd_rst_plot)
		fclose(fd_rst_plot);
	fd = NULL;
	fd_rst = NULL;
	fd_rst_cal = NULL;
	fd_rst_plot = NULL;
	return 0;
}



int main()
{
	//init_config();
	//generate_default_config();
	//return 0;	
	
	for(int i = GILC_CALIBRATED_CFG__START;i<GILC_CALIBRATED_CFG__START+GILC_CALIBRATED_CFG__NUM;i++)
		gilc_calibrated_fun(gilc_calibrated_cfg[i]);

#ifdef WIN32
	system("pause");
#endif
	return 0;
}
