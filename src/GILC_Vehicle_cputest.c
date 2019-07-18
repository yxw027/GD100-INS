// GILC_Vehile_test.cpp : 定义控制台应用程序的入口点。
//
#ifdef WIN32 
#include "stdafx.h"
#endif
#ifdef __linux
#include <memory.h>
#endif

#include "GILC_Test_Config_FileLoad.h"

#include <time.h>
#ifdef WIN32
#include <windows.h>
#else
#  include <sys/time.h>
#endif
#include "gilc_out_pack.h"

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

int main() 
{
	gilc_ret_e ret = GILC_RET__RST_NOTHING;
	gilc_raw_t stRaw = {0};
	gilc_result_t stOut = {0};
	gilc_cfg_t stCfg = {0};
	char buff[MAXLEN] = {0};
	FILE *fd = NULL;
	FILE *fd_rst = NULL;
	char lcfile[1024] = TEST_RAW_FILE_PATH;
	char lcfile_rst[1024] = TEST_OUT_FILE_PATH;
	unsigned long long gilc_out_ctl_times = 0;

#if (!GILC_DEBUG_PROCESS_TIME)
	strncpy(stCfg.debug_outfile_path, TEST_OUT_FILE_PATH, sizeof(stCfg.debug_outfile_path));
	strncpy(stCfg.debug_tmpfile_path, TEST_TMP_FILE_PATH, sizeof(stCfg.debug_tmpfile_path));
	stCfg.debug_level = 1;
	stCfg.bFilePathCfgUse = true;
	stCfg.bOutFileSaveClose = true;
	stCfg.bTmpFileSaveClose = true;
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
#endif

	GILC_Init(&stCfg);
	
	strcat(lcfile,TEST_RAW_FILE_NAME);
	fd = fopen(lcfile, "rt");
	if(!fd)
	{
		printf("open file err! %s\r\n",lcfile);
		system("pause");
		return 0;
	}
	
	static unsigned long long llu_once = 0, llu_ave = 0, llu_total = 0, llu_cnt = 0;
	struct timeval tv, tv_last;
	while(1)
	{
#if 0
		/*dsf90:debug 重新初始化、文件存储测试*/
		static long lDebug_cnt = 0;
		if(lDebug_cnt++ == 80000)
		{
			GILC_Init(&stCfg);
		}
#endif		
		if (feof(fd))
		{
			printf("file read over!\n");
			if (fd)
				fclose(fd);
			if(fd_rst)
				fclose(fd_rst);
			break;
		}
		
		fgets(buff, MAXLEN, fd);
		if (strlen(buff)<18) 
		{
			printf("file read error: %s\n",buff);
			continue;
		}

#if 1		
		/*dsf90:debug 接口测试*/
		gettimeofday(&tv_last, NULL);
		ret = (gilc_ret_e)GILC_LoadRaw_byStrn(buff,&stRaw);
		if(ret == -1) 
		{
			//printf("GILC_Load param err!\r\n");
			continue;
		}
		else if(ret == -2) 
		{		
			//printf("GILC_Load fail, unknow strning: %s\n",buff);
			continue;
		}
		
		ret = GILC_PROCESS_Vehicle(&stRaw,&stOut);
		gettimeofday(&tv, NULL);
#else
		gettimeofday(&tv_last, NULL);												
		/*dsf90:debug 接口测试*/
		ret = GILC_PROCESS_Vehicle_byStrn(buff,&stOut);
		gettimeofday(&tv, NULL);		//get system time here to get ms info, tv.tv_usec means ms
#endif
		if (ret<0)
		{
			//printf("Gilc ret %d, string: %s\r\n", ret, buff);
		}
		else if (ret == 0)
		{
			//printf("Gilc ret %d, string: %s\r\n", ret, buff);
		}
		else if (ret > 0 && stOut.week != 0)
		{
			unsigned long long weektime_new = (unsigned long long)(stOut.week * 7 * 24 * 3600 * 1000.0 + stOut.second*1000.0);
			if (gilc_out_ctl_times == 0)
			{
				gilc_out_ctl_times = weektime_new - (weektime_new%GILC_NMEA_OUT_PERIOD_MS);
				printf("gilc_out_ctl_times init: %d,%.6f;%llu,%llu\r\n", stOut.week, stOut.second, weektime_new, gilc_out_ctl_times);
			}

			if (weektime_new >= gilc_out_ctl_times)
			{
				char buffer[1024] = {0};		
				GILC_IOOut_CreatCascoPack(buffer,&stRaw,&stOut);
				gilc_out_ctl_times += GILC_NMEA_OUT_PERIOD_MS;
				if (!fd_rst)
				{
					strcat(lcfile_rst, TEST_RST_FILE_NAME);
					fd_rst = fopen(lcfile_rst, "wt");
					if (!fd_rst)
						printf("open file err! %s\r\n", lcfile_rst);
				}
				else
				{
					fprintf(fd_rst, buffer);
				}
			}
		}
		llu_once = (tv.tv_sec - tv_last.tv_sec) * 1000000u + tv.tv_usec - tv_last.tv_usec;
		llu_cnt++; llu_total += llu_once; llu_ave = llu_total / llu_cnt;

		printf("---debug process time---:count,%12llu; onec,%12llu; total,%12llu; ave,%12llu\r\n",
			llu_cnt, llu_once, llu_total, llu_ave);
	}
	printf("---debug process time---:count,%12llu; onec,%12llu; total,%12llu; ave,%12llu\r\n",
		llu_cnt, llu_once, llu_total, llu_ave);
#ifdef WIN32
	system("pause");
#endif
	return 0;
}
