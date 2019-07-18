// GILC_Vehile_test.cpp : 定义控制台应用程序的入口点。
//
#ifdef WIN32 
#include "stdafx.h"
#endif
#ifdef __linux
#include <memory.h>
#endif

#include <time.h>
#ifdef WIN32
#include <windows.h>
#else
#  include <sys/time.h>
#endif
#include <math.h>
#include "filter.h"

//#include "GILC_Test_Config_FileLoad_nx200_adis16445.h"
//#include "GILC_Test_Config_FileLoad_P2C_demo_imu381.h"
#include "GILC_Test_Config_FileLoad_P2C_demo_adis16465.h"

static void Str2Array(const char *str, const char *sep, double *val)
{
	char *p, _str[1024];
	double d[MAXVAL] = { 0.0 };
	int i, j;

	strcpy(_str, str);
	for (i = 0, p = strtok(_str, sep); p&&i<MAXVAL; p = strtok(NULL, sep), i++) {
		d[i] = atof(p);
	}

	for (j = 0; j<i; j++) val[j] = d[j];
}

/*dsf90:添加','格式识别*/
int  Str2StrVector(char *sSrc, char sVect[50][20], int nMaxItems)
{
	int   i, j;

	for (i = 0; i < nMaxItems; i++)
	{
		//去掉前导空格
		while (*sSrc)
		{
			if (!isspace(*sSrc) && (*sSrc != ',')) break;
			sSrc++;
		}
		if (*sSrc == 0) break;

		j = 0;
		while (*sSrc)
		{
			if (isspace(*sSrc) || (*sSrc == ',')) break;
			sVect[i][j++] = *sSrc;
			sSrc++;
			if (j >= 19) break; // 数据文件中的任何单独的一个数的长度不会超过20位
		}

		sVect[i][j] = 0;
		if (*sSrc == 0) break;
	}
	return i;
}

int main()
{
	DetectStatic detectStatic;
	double dSecOfWeek = 0.0;
	double ep[6] = { 0 };
	double dImuData[6] = { 0 };
	double dGpsVel[3] = { 0 };
	double dInsVel[3] = { 0 };
	int bStatic = 0;
	char buff[MAXLEN] = { 0 };
	FILE *fd = NULL;
	FILE *fd_rst = NULL;
	char lcfile[1024] = TEST_TMP_FILE_PATH;
	char lcfile_rst[1024] = TEST_TMP_FILE_PATH;
	int i = 0;
	double dGnssVel = 0.0;

	strcat(lcfile, "process.txt");
	fd = fopen(lcfile, "rt");
	if (!fd)
	{
		printf("open file err! %s\r\n", lcfile);
		system("pause");
		return 0;
	}

	strcat(lcfile_rst, "imudata.txt");
	fd_rst = fopen(lcfile_rst, "wt");
	if (!fd_rst)
	{
		printf("open file err! %s\r\n", lcfile_rst);
		system("pause");
		return 0;
	}

	while (1)
	{	
		if (feof(fd))
		{
			printf("file read over!\n");
			break;
		}

		fgets(buff, MAXLEN, fd);
		if (strlen(buff) < 18)
		{
			printf("file read error: %s\n", buff);
			continue;
		}

		double val[50] = { 0.0 };
		Str2Array(buff, " ", val);
		dSecOfWeek = val[1];

		for (i = 0; i < 3; i++)
			dInsVel[i] = val[8 + i];
		for (i = 0; i < 3; i++)
			dGpsVel[i] = val[17 + i];
		for (i = 0; i < 6;i++)
			dImuData[i] = val[20+i];
		dGnssVel = sqrt(dGpsVel[0]* dGpsVel[0]+ dGpsVel[1] * dGpsVel[1]+ dGpsVel[2] * dGpsVel[2]);
		bStatic = detectStatic.DetectStatic_car_dsf(ep,dImuData,dImuData+3,dGpsVel,dInsVel,0,1,1);
		fprintf(fd_rst, "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %d\r\n",
			dSecOfWeek,
			dImuData[0], dImuData[1], dImuData[2],
			dImuData[3], dImuData[4], dImuData[5],
			detectStatic.dImuData_iir[0], detectStatic.dImuData_iir[1], detectStatic.dImuData_iir[2], 
			detectStatic.dImuData_iir[3], detectStatic.dImuData_iir[4], detectStatic.dImuData_iir[5],
			detectStatic.dImuData_iir_dif_max[0], detectStatic.dImuData_iir_dif_max[1], detectStatic.dImuData_iir_dif_max[2],
			detectStatic.dImuData_iir_dif_max[3], detectStatic.dImuData_iir_dif_max[4], detectStatic.dImuData_iir_dif_max[5],
			dGnssVel,bStatic);
	}
#ifdef WIN32
	//system("pause");
#endif
	if (fd)
		fclose(fd);
	if (fd_rst)
		fclose(fd_rst);
	return 0;
}
