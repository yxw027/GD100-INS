// GILC_Vehile_test.cpp : 定义控制台应用程序的入口点。
//
#include "GILC_TEST_FileLoad.h"

#include <string.h>
#include <time.h>

#ifdef STM32
#include "ff.h"
#include "malloc.h"
#include "myprintf.h"
#endif

int gilc_test_main()
{
	FRESULT fs_rst;
	gilc_ret_e ret = GILC_RET__RST_NOTHING;
	gilc_raw_t stRaw = { 0 };
	gilc_result_t stOut = { 0 };
	gilc_cfg_t stCfg = { 0 };
	char buff[MAXLEN] = { 0 };
	static FIL *fd = NULL;
	static FIL *fd_rst = NULL;	
	char lcfile[1024] = TEST_RAW_FILE_PATH;
	char lcfile_rst[1024] = TEST_RAW_FILE_PATH;
	unsigned long long gilc_out_ctl_times = 0;

	if(!fd)
		fd = malloc(sizeof(FIL));
	if(!fd_rst)
		fd_rst = malloc(sizeof(FIL));	

	
#if (!GILC_DEBUG_PROCESS_TIME)
	strncpy(stCfg.debug_outfile_path, TEST_OUT_FILE_PATH, sizeof(stCfg.debug_outfile_path));
	strncpy(stCfg.debug_tmpfile_path, TEST_TMP_FILE_PATH, sizeof(stCfg.debug_tmpfile_path));
	stCfg.bfilePathCfgUse = true;
	stCfg.bfileSave = true;
#endif

#if 0
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

	stCfg.bGnssPosStdUse = CFG_POS_STD_USE;
	stCfg.bGnssVelStdUse = CFG_VEL_STD_USE;

	stCfg.arm_ant[0] = CFG_ARM_ANT_X;
	stCfg.arm_ant[1] = CFG_ARM_ANT_Y;
	stCfg.arm_ant[2] = CFG_ARM_ANT_Z;
#endif
	GILC_Init(&stCfg);

	fs_rst = f_open(fd,"0:debug.txt",FA_READ);
	if(fs_rst!=FR_OK)
	{
		printf("open fd err! %d\r\n",fs_rst);
		return 0;
	}
	
	fs_rst = f_open(fd_rst,"0:rst.txt",FA_READ | FA_WRITE | FA_CREATE_ALWAYS);
	if(fs_rst!=FR_OK)
	{
		printf("open fd_rst err! %d\r\n",fs_rst);
		return 0;
	}
		
	printf("mem used 0:%6d 1:%8d 2:%6d --------  \r\n",my_mem_used(0),my_mem_used(1),my_mem_used(2));
	while(1)
	{
		static unsigned long long llu_tick=0, llu_once = 0, llu_ave = 0, llu_total = 0, llu_cnt = 0;
		printf("mem used 0:%8d --------  ",my_mem_used(0));
		if (f_eof(fd))
		{
			printf("file read over!\n");
			f_close(fd);
			f_close(fd_rst);
			break;
		}
		
		f_gets(buff, MAXLEN, fd);
		if (strlen(buff)<18) 
		{
			printf("file read error: %s\n",buff);
			continue;
		}
		
		//printf(buff);
		//continue;
		llu_tick = HAL_GetTick()*100; /*stm32 systick  100us*/
		/*dsf90:debug 接口测试*/
		ret = GILC_PROCESS_Vehicle_byStrn(buff, &stOut);
		llu_once = HAL_GetTick()*100 - llu_tick;

		llu_cnt++; llu_total += llu_once; llu_ave = llu_total / llu_cnt;

#if GILC_DEBUG_PROCESS_TIME
		printf("%9llu---debug process time---:count,%12llu; onec,%12llu; total,%12llu; ave,%12llu\r\n",
			llu_tick, llu_cnt, llu_once, llu_total, llu_ave);
#endif
/*
		static int cnt = 0;
		if (++cnt == 100)
		{
			cnt = 0;
			printf("Gilc ret %d, GNSS+IMU:%f,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d\r\n",
				ret,
				stOut.second, stOut.week,
				stOut.lla_imu[0], stOut.lla_imu[1], stOut.lla_imu[2],
				stOut.lla_ant[0], stOut.lla_ant[1], stOut.lla_ant[2],
				stOut.ver_ant[0], stOut.ver_ant[1], stOut.ver_ant[2],
				stOut.pitch*R2D, stOut.roll*R2D, stOut.yaw*R2D,
				stOut.speed_ant, stOut.heading_ant,
				stOut.car_status, stOut.gilc_status);
		}
*/
	}
	return 0;
}
