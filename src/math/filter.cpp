#ifdef WIN32
#include "stdafx.h"
#endif
#include "math.h"
#include "filter.h"
#include "stdlib.h"
#include "string.h"
#ifdef __linux
#include "memory.h"
#include "stdio.h"
#endif

#include "log.h"
#include "GILC_Config.h"

//#define  GNSS_SPEED__MAX_CALIBRATE_STATIC      0.06f
//#define  GNSS_SPEED__MIN_CALIBRATE_SPEED       0.2f
//#define  GNSS_SPEED__MIN_CALIBRATE_HEAD        0.2f
//#define  GNSS_SPEED__MAX_CALIBRATE_STATIC      0.15f /*BMI055*/
#define  GNSS_SPEED__MAX_CALIBRATE_STATIC      0.06f
#define  GNSS_SPEED__MIN_CALIBRATE_SPEED       2.0f
#define  GNSS_SPEED__MIN_CALIBRATE_HEAD        5.0f

#define SWAP3(x, y, mid)  mid = x; x = y; y = mid;
/*fs=100Hz,bw=0.5*/
static double dIIR_Order3_bw_fs_0_5[] = {
	0.037568380197861e-4,
	0.112705140593583e-4,
	0.112705140593583e-4,
	0.037568380197861e-4,
	1.000000000000000,
	-2.937170728449891,
	2.876299723479332,
	-0.939098940325283
};

static double dIIR_Order3_bw_fs_40[] = {
0.52762438250194321,
1.5828731475058295,
1.5828731475058295,
0.52762438250194321	,
1,
1.7600418803431686,
1.1828932620378305,
0.27805991763454646
};

static double dIIR_Order5_bw_fs_3[] = { 
	0.0000055603090957273324,
	0.000027801545478636663,
	0.000055603090957273319,
	0.000055603090957273319,
	0.000027801545478636663,
	0.0000055603090957273324,
	1,
	- 4.3902761942608475,
	7.7428695408010348,
	- 6.8543493508959159,
	3.0446853091802573,
	- 0.54275137493346526
};

static double dIIR_Order5_bw_fs_1_5[] = {
	0.00000020024206735968283,
	0.0000010012103367984142,
	0.0000020024206735968283,
	0.0000020024206735968283,
	0.0000010012103367984142,
	0.00000020024206735968283,
	1,
	- 4.6950406261001882,
	8.8261459225638248,
	- 8.3039666930845613,
	3.909893994115305,
	- 0.73702618974822409
};
	
static double dIIR_Order10_bw_fs_1_5[] = {
	0.000000000000040247464728293706,
	0.00000000000040247464728293707,
	0.0000000000018111359127732167,
	0.0000000000048296957673952447,
	0.0000000000084519675929416781,
	0.000000000010142361111530013,
	0.0000000000084519675929416781,
	0.0000000000048296957673952447,
	0.0000000000018111359127732165,
	0.00000000000040247464728293707,
	0.000000000000040247464728293706,
	1,
	-9.3975404901497122,
	39.758421710841752,
	-99.720637424605002,
	164.20701479066179,
	-185.48969388488243,
	145.56618126711487,
	-78.363984299422128,
	27.695618050691515,
	-5.8026748912870856,
	0.54729517107762304
};
static double GetAveStd(vector<double> a,int opt)
{
	int n=a.size();
	double avg=0.0,std=0.0,rms=0.0,sum=0.0;

	if(n==0) 
		return 99999.9;

	if (opt==2) 
	{
	   for (int i=0;i<n;i++){
		   rms+= a[i]*a[i];
	   }
	   rms=sqrt(rms/double(n));
	   return rms;
	}

	for (int i=0;i<n;i++){
		sum+= a[i];
	}
	avg=sum/n;

	if (opt==0) 
		return avg;
	
	if (opt==1) 
	{
		sum=0.0;
		for (int i=0;i<n;i++)
		{
			sum+=(a[i]-avg)*(a[i]-avg);
		}
		std=sqrt(sum/double(n-1));
		return std;
	}
	
	else
	return 0.0;
}

double GetSum(double *p, int nFrom, int nTo)
{
	double d;
	int i;

	if (nFrom < 0) nFrom = 0;
	if (nTo < 0) nTo = 0;
	if (nFrom > nTo) { SWAP3(nFrom, nTo, i); };
	d = 0;
	for (i = nFrom; i <= nTo; i++)  d += p[i];
	return d;
}

double GetAve(double *p, int nFrom, int nTo)
{
	return GetSum(p, nFrom, nTo) / (abs(nFrom - nTo) + 1);
}

void GetMaxMinAve(double *p, int nFrom, int nTo, double *dMax, double *dMin, double *dAve)
{
	int i;

	*dMax = p[nFrom];
	*dMin = p[nFrom];
	*dAve = p[nFrom];

	for (i = nFrom + 1; i <= nTo; i++)
	{
		if (*dMax < p[i]) *dMax = p[i];
		if (*dMin > p[i]) *dMin = p[i];
		*dAve += p[i];
	}

	*dAve /= (abs(nFrom - nTo) + 1);
}

Filter::Filter(void)
{
	bFirstRun=1;
	memset(iir_x,0,sizeof(iir_x));
	memset(iir_y,0,sizeof(iir_y));
}

void Filter::Init(void)
{
	bFirstRun=1;
	memset(iir_x,0,sizeof(iir_x));
	memset(iir_y,0,sizeof(iir_y));
}

///*===================================================================================
//	IIR 比 FIR 滤波器响应速度快, 计算量小, 用在机动判断应该更合适? 
//	3-order lowerpass filter model:
//	   y[n] = b[0]*x[n] + b[1]*x[n-1] + b[2]*x[n-2] + b[3]*x[n-3]
//	                    - a[1]*y[n-1] - a[2]*y[n-2] - a[3]*y[n-3]
//=====================================================================================*/
void  Filter::IIR_Lowpass_Oder3(double *dOut, double *dIn, double *dIIR, short nChan)
{	
	int    i, j;
	double dTmp;
	
	if(bFirstRun)
	{
		bFirstRun = 0;
		
		for( i = 0; i < nChan; i++ )
		{
			for( j = 0; j < IIR_NUM; j++ ) iir_x[i][j] = iir_y[i][j] = 0;
		}
	}
	
	for( i = 0; i < nChan; i++ )
	{
		iir_x[i][3] = iir_x[i][2];
		iir_x[i][2] = iir_x[i][1];
		iir_x[i][1] = iir_x[i][0];
		iir_x[i][0] = dIn[i];
		
		dTmp  = dIIR[0]*iir_x[i][0] + dIIR[1]*iir_x[i][1] + dIIR[2]*iir_x[i][2] + dIIR[3]*iir_x[i][3];
		dTmp -=                       dIIR[5]*iir_y[i][1] + dIIR[6]*iir_y[i][2] + dIIR[7]*iir_y[i][3];
		
		iir_y[i][0] = dTmp;
		iir_y[i][3] = iir_y[i][2];
		iir_y[i][2] = iir_y[i][1];
		iir_y[i][1] = iir_y[i][0];
		
    	dOut[i] = (double)dTmp;
	}
}

///*===================================================================================
//	IIR 比 FIR 滤波器响应速度快, 计算量小, 用在机动判断应该更合适? 
//	5-order lowerpass filter model:
//	   y[n] = b[0]*x[n] + b[1]*x[n-1] + b[2]*x[n-2] + b[3]*x[n-3] + b[4]*x[n-4] + b[5]*x[n-5]
//	                    - a[1]*y[n-1] - a[2]*y[n-2] - a[3]*y[n-3] - a[4]*y[n-4] - a[5]*y[n-5]
//=====================================================================================*/
void  Filter::IIR_Lowpass_Oder5(double *dOut, double *dIn, double *dIIR, short nChan)
{
	int    i, j;
	double dTmp;
	double dTmp2;

	if (bFirstRun)
	{
		bFirstRun = 0;

		for (i = 0; i < nChan; i++)
		{
			for (j = 0; j < IIR_NUM; j++) iir_x[i][j] = iir_y[i][j] = 0.0;
		}
	}

	for (i = 0; i < nChan; i++)
	{
		iir_x[i][5] = iir_x[i][4];
		iir_x[i][4] = iir_x[i][3];
		iir_x[i][3] = iir_x[i][2];
		iir_x[i][2] = iir_x[i][1];
		iir_x[i][1] = iir_x[i][0];
		iir_x[i][0] = dIn[i];

		dTmp  = dIIR[0] * iir_x[i][0] + dIIR[1] * iir_x[i][1] + dIIR[2] * iir_x[i][2] + dIIR[3] * iir_x[i][3] + dIIR[4] * iir_x[i][4] + dIIR[5] * iir_x[i][5];
		dTmp2 =                         dIIR[7] * iir_y[i][1] + dIIR[8] * iir_y[i][2] + dIIR[9] * iir_y[i][3] + dIIR[10]* iir_y[i][4] + dIIR[11]* iir_y[i][5];
		dTmp -= dTmp2;

		iir_y[i][0] = dTmp;
		iir_y[i][5] = iir_y[i][4];
		iir_y[i][4] = iir_y[i][3];
		iir_y[i][3] = iir_y[i][2];
		iir_y[i][2] = iir_y[i][1];
		iir_y[i][1] = iir_y[i][0];

		dOut[i] = (double)dTmp;
	}
}

void  Filter::IIR_Lowpass_Oder10(double *dOut, double *dIn, double *dIIR, short nChan)
{
	int    i, j;
	double dTmp;
	double dTmp2;

	if (bFirstRun)
	{
		bFirstRun = 0;

		for (i = 0; i < nChan; i++)
		{
			for (j = 0; j < IIR_NUM; j++) iir_x[i][j] = iir_y[i][j] = 0.0;
		}
	}

	for (i = 0; i < nChan; i++)
	{
		iir_x[i][10] = iir_x[i][9];
		iir_x[i][9] = iir_x[i][8];
		iir_x[i][8] = iir_x[i][7];
		iir_x[i][7] = iir_x[i][6];
		iir_x[i][6] = iir_x[i][5];
		iir_x[i][5] = iir_x[i][4];
		iir_x[i][4] = iir_x[i][3];
		iir_x[i][3] = iir_x[i][2];
		iir_x[i][2] = iir_x[i][1];
		iir_x[i][1] = iir_x[i][0];
		iir_x[i][0] = dIn[i];

		dTmp  = dIIR[0] * iir_x[i][0] + dIIR[1] * iir_x[i][1] + dIIR[2] * iir_x[i][2] + dIIR[3] * iir_x[i][3] + dIIR[4] * iir_x[i][4] + dIIR[5] * iir_x[i][5]
			                          + dIIR[6] * iir_x[i][6] + dIIR[7] * iir_x[i][7] + dIIR[8] * iir_x[i][8] + dIIR[9] * iir_x[i][9] + dIIR[10]* iir_x[i][10];
		dTmp2 =                         dIIR[12]* iir_y[i][1] + dIIR[13]* iir_y[i][2] + dIIR[14]* iir_y[i][3] + dIIR[15]* iir_y[i][4] + dIIR[16]* iir_y[i][5]
			                          + dIIR[17]* iir_y[i][6] + dIIR[18]* iir_y[i][7] + dIIR[19]* iir_y[i][8] + dIIR[20]* iir_y[i][9] + dIIR[21]* iir_y[i][10];
		dTmp -= dTmp2;

		iir_y[i][0] = dTmp;
		iir_y[i][10] = iir_y[i][9];
		iir_y[i][9] = iir_y[i][8];
		iir_y[i][8] = iir_y[i][7];
		iir_y[i][7] = iir_y[i][6];
		iir_y[i][6] = iir_y[i][5];
		iir_y[i][5] = iir_y[i][4];
		iir_y[i][4] = iir_y[i][3];
		iir_y[i][3] = iir_y[i][2];
		iir_y[i][2] = iir_y[i][1];
		iir_y[i][1] = iir_y[i][0];

		dOut[i] = (double)dTmp;
	}
}

void  Filter::IIR_Lowpass(double *dOut, double *dIn)
{	
	IIR_Lowpass_Oder3(dOut, dIn, dIIR_Order3_bw_fs_40, 6);
	//IIR_Lowpass_Oder5(dOut, dIn, dIIR_Order5_bw_fs_3, 6);
	//IIR_Lowpass_Oder10(dOut, dIn, dIIR_Order10_bw_fs_1_5, 6);
}

DetectStatic::DetectStatic(void)
{
	iStaticAcc=0;
	iStaticGyro=0;
	memset(dDetectIMU_fifo,0,sizeof(dDetectIMU_fifo));
	nIndex_Data=0;
	memset(s_cStateLast,0,sizeof(s_cStateLast));
	s_pStateSp=0;
	s_iState_pre=0;
	for(int i=0;i<3;i++)
	{
		stdaxyz[i]=stdgxyz[i]=difaxyz[i]=difgxyz[i]=0;
	}
	init_flag = 0;
	bStatic_ret = 0;
	bDetect_pre = 0;	
	num_static = 0;
	bStatic_gnss = 0;
	iNum_std = 50;
}
/*
DetectStatic& DetectStatic::operator=(const DetectStatic& detect)
{
	iStaticAcc=detect.iStaticAcc;
	iStaticGyro=detect.iStaticGyro;
	memcpy(dDetectIMU_fifo,detect.dDetectIMU_fifo,sizeof(dDetectIMU_fifo));
	nIndex_Data=detect.nIndex_Data;
	memcpy(s_cStateLast,detect.s_cStateLast,sizeof(s_cStateLast));
	s_pStateSp=detect.s_pStateSp;
	s_iState_pre=detect.s_iState_pre;
	for(int i=0;i<3;i++)
	{
		stdaxyz[i]=detect.stdaxyz[i];
		stdgxyz[i]=detect.stdgxyz[i];
		difaxyz[i]=detect.difaxyz[i];
		difgxyz[i]=detect.difgxyz[i];
	}
	init_flag = detect.init_flag;
	bStatic_ret = detect.bStatic_ret;
	bDetect_pre = detect.bDetect_pre;
	num_static = detect.num_static;
	bStatic_gnss = detect.bStatic_gnss;
	for(int i=0;i<3;i++)
	{
		axyz[i].clear();
		gxyz[i].clear();
		axyz_iir[i].clear();
		gxyz_iir[i].clear();
	}
}
*/
void DetectStatic::Init(void)
{
	iStaticAcc=0;
	iStaticGyro=0;
	memset(dDetectIMU_fifo,0,sizeof(dDetectIMU_fifo));
	nIndex_Data=0;
	memset(s_cStateLast,0,sizeof(s_cStateLast));
	s_pStateSp=0;
	s_iState_pre=0;
	for(int i=0;i<3;i++)
	{
		stdaxyz[i]=stdgxyz[i]=difaxyz[i]=difgxyz[i]=0;
	}
	init_flag = 0;
	bStatic_ret = 0;
	bDetect_pre = 0;
	num_static = 0;
	bStatic_gnss = 0;
	for(int i=0;i<3;i++)
	{
		axyz[i].clear();
		gxyz[i].clear();
		axyz_iir[i].clear();
		gxyz_iir[i].clear();
	}
}

int DetectStatic::DetectIMU(double stdAcc[3], double stdGyro[3],int opt)
{
	int      i, bTmp;
	double dTmp;

	for (i = 0; i < 3; i++)
	{
		dDetectIMU_fifo[i][nIndex_Data] = dAcc_iir[i];
		dDetectIMU_fifo[i+3][nIndex_Data] = dGyro_iir[i];
	}

	nIndex_Data++;
	nIndex_Data = nIndex_Data%DATA_LEN;

	for (i = 0; i < 6; i++)
	{
		//dAve1[i] = GetAve(dDetectIMU_fifo[i], 0, nHalf-1);           
		//dAve2[i] = GetAve(dDetectIMU_fifo[i], nHalf, DATA_LEN-1);    
		GetMaxMinAve(dDetectIMU_fifo[i], 0, DATA_LEN - 1, &dMax[i], &dMin[i], &dAve[i]); // 整段数据中的最大、最小和均值
	}

	if (!opt)
	{
		for (i = 0, bTmp = 1; i < 3; i++)
		{
			dTmp = fabs(dMax[i] - dAve[i]);
			bTmp &= (dTmp < A_AVE_THRESHOLD);
			dTmp = fabs(dMax[i] - dMin[i]);
			bTmp &= (dTmp < A_AVE_THRESHOLD * 2);
		}
	}
	else
	{
		for (i = 0, bTmp = 1; i < 3; i++)
		{
			dTmp = fabs(dMax[i] - dAve[i]);
			bTmp &= (dTmp < stdAcc[i] );
			dTmp = fabs(dMax[i] - dMin[i]);
			bTmp &= (dTmp < stdAcc[i] * 2);
		}
	}
	//iStaticAcc = bTmp;

	/*
	bTmp = 1;
	for (i = 3; i < 6; i++)
	{
		dTmp = fabs(dMax[i] - dAve[i]);
		bTmp &= (dTmp < W_AVE_THRESHOLD);
		dTmp = fabs(dMax[i] - dMin[i]);
		bTmp &= (dTmp < W_AVE_THRESHOLD * 2);
	}
	iStaticGyro = bTmp;

	dFx2 = dAve[0] * dAve[0];
	dFy2 = dAve[1] * dAve[1];
	dFz2 = dAve[2] * dAve[2];
	stInsGpsTmp.accAll = sqrt(dFx2 + dFy2 + dFz2)-g;
	dTmp = fabs(stInsGpsTmp.accAll);

	nState[nIndex_State++] = (dTmp < F_MOV_THRESHOLD);    // 机动加速度小于阈值
	*/

	return bTmp;
}

int DetectStatic::IsStatic(double stdAcc[3], double stdGyro[3], double ins_speed,int opt)
{
	int      i, bTmp;
	int iStateSpGet = 0;
	int detect_len =  BUF_LEN;

	if(opt == 0 || opt == 1)/*init*/
	{
		memset(s_cStateLast,opt,sizeof(s_cStateLast));
		s_pStateSp = 0;
		s_iState_pre = 0;
		return 0;
	}

	bTmp = DetectIMU(stdAcc,stdGyro,(opt-2));
	s_cStateLast[s_pStateSp++] = bTmp;
	s_pStateSp = s_pStateSp%(BUF_LEN);

	if(bTmp == 0 && s_iState_pre == 0)
	{
		return 0;
	}

	if(ins_speed > 2)
	{
		detect_len = BUF_LEN;
	}
	else if(ins_speed > 1)
	{
		detect_len = (int)(BUF_LEN*0.75);
	}
	else if(ins_speed > 0.5)
	{
		detect_len = (int)(BUF_LEN*0.5);
	}
	else
	{
		detect_len = (int)(BUF_LEN*0.25);
	}
	
	for (i = 0,bTmp = 0; i < detect_len; i++)
	{
		iStateSpGet = s_pStateSp - i -1;
		if(iStateSpGet<=-1)
			iStateSpGet += BUF_LEN;
		
		bTmp += s_cStateLast[iStateSpGet];
	}

	if(s_iState_pre == 0)
	{
		if(bTmp >= detect_len)
			bTmp = 1;
		else
			bTmp = 0;
	}
	
	if(s_iState_pre == 1)
	{
		if(bTmp >= detect_len*0.8)
			bTmp = 1;
		else
			bTmp = 0;
	}
		
	s_iState_pre = bTmp;

	return bTmp;
}

void DetectStatic::SvmFormTrainDataCreat(int bStatic)
{
	int i = 0;
	if(axyz[0].size()>=iNum_std)
	{
		for(i=0;i<3;i++)
		{
			axyz[i].erase(axyz[i].begin()); 
			gxyz[i].erase(gxyz[i].begin()); 
			axyz_iir[i].erase(axyz_iir[i].begin()); 
			gxyz_iir[i].erase(gxyz_iir[i].begin()); 
		}
	}
	
	for (i = 0; i<3; i++)
	{
		axyz[i].push_back(dAcc[i]);
		gxyz[i].push_back(dGyro[i]);
		axyz_iir[i].push_back(dAcc_iir[i]);
		gxyz_iir[i].push_back(dGyro_iir[i]);
	}

	if(axyz[0].size()<iNum_std)
		return;
	
	for(i=0;i<3;i++)
	{
		dAcc_std[i]=GetAveStd(axyz[i],1);
		dGyro_std[i]=GetAveStd(gxyz[i],1);
		dAcc_iir_std[i]=GetAveStd(axyz_iir[i],1);
		dGyro_iir_std[i]=GetAveStd(gxyz_iir[i],1);
	}

	static FILE* fd_svm = NULL;
	if (!fd_svm)
	{
		char cFileNamePath[128] = {0};
		sprintf(cFileNamePath, "SvmTrain.data");
		fd_svm = fopen(cFileNamePath, "wt");
		if (!fd_svm)
			gilc_log("open file err! %s\r\n", cFileNamePath);
	}
	else
	{
		fprintf(fd_svm,"%d  1:%.8f  2:%.8f  3:%.8f  4:%.8f  5:%.8f  6:%.8f  7:%.8f  8:%.8f   9:%.8f  10:%.8f  11:%.8f  12:%.8f \r\n",
					bStatic+1,dAcc_std[0],dAcc_std[1],dAcc_std[2],dGyro_std[0],dGyro_std[1],dGyro_std[2],
					dAcc_iir_std[0],dAcc_iir_std[1],dAcc_iir_std[2],dGyro_iir_std[0],dGyro_iir_std[1],dGyro_iir_std[2]);
	}
	
}

void get_imudata_stdmaxmin(int bStatic)
{
	
}

#if 0
/*
add by dsf90,2018.5.30
探测车辆行驶模式：
0：前进
1：停车
2：倒车
3：倒车假象（前进组合速度小于0，而实际未倒车）
*/
extern int DetectCar_DriverMode(double acc[3],double gyo[3],double insvel[3])
{
#define       IMU_FS        100
#define       BUF_LEN       IMU_FS

	int      i, j, bTmp = 0;
	double ins_speed = insvel[1];

	static   char     s_cStateLast[BUF_LEN] = { 0 };
	static   short    s_pStateSp = 0;

	//bTmp = IsStatic(acc,gyo,ins_speed,0);
	s_cStateLast[s_pStateSp++] = bTmp;
	s_pStateSp = s_pStateSp%(BUF_LEN);

	if(bTmp == 1  || ins_speed >= -0.01f)
		return bTmp;

	int iStateSpGet = 0;
	int detect_len =  BUF_LEN;
	for (i = 0,bTmp = 0; i < detect_len; i++)
	{
		iStateSpGet = s_pStateSp - i -1;
		if(iStateSpGet<=-1)
			iStateSpGet += BUF_LEN;
		
		bTmp &= s_cStateLast[iStateSpGet];
	}

	/*dsf90:如果上一秒数据为停车状态，则本次为真实倒车*/
	/*dsf90:假定条件，汽车倒车前必为停车*/
	//if(bTmp >= BUF_LEN*0.66)
		//return 2;
	return (bTmp?2:3);
}

int DetectStatic::DetectStatic_car(double acc[3],double gyo[3],double gpsvel[3],double insvel[3],int bupgnss,int insnum)
{
	static vector<double, malloc_allocator<double> > ax,ay,az,gx,gy,gz;
	static vector<double, malloc_allocator<double> > ax_gstaic,ay_gstaic,az_gstaic,gx_gstaic,gy_gstaic,gz_gstaic;
	static double threadax=0.1,threaday=0.1,threadaz=0.05,threadgx=0.01,threadgy=0.01,threadgz=0.05;
	static int count=0,num_static=0;
	double stdax=0,stday=0,stdaz=0,stdgx=0,stdgy=0,stdgz=0;
	double gpsspeed=0,insspeed=0;
	if(ax.size()<25)
	{
		ax.push_back(acc[0]);
		ay.push_back(acc[1]);
		az.push_back(acc[2]);
		gx.push_back(gyo[0]);
		gy.push_back(gyo[1]);
		gz.push_back(gyo[2]);
		return 0;
	}
	else
	{
		ax.erase(ax.begin()); ax.push_back(acc[0]);
		ay.erase(ay.begin()); ay.push_back(acc[1]);
		az.erase(az.begin()); az.push_back(acc[2]);
		gx.erase(gx.begin()); gx.push_back(gyo[0]);
		gy.erase(gy.begin()); gy.push_back(gyo[1]);
		gz.erase(gz.begin()); gz.push_back(gyo[2]);
		stdax=GetAveStd(ax,1);/*dsf90:标准差*/
		stday=GetAveStd(ay,1);
		stdaz=GetAveStd(az,1);
		stdgx=GetAveStd(gx,1);
		stdgy=GetAveStd(gy,1);
		stdgz=GetAveStd(gz,1);
    }
	if(bupgnss)
	{
		gpsspeed = sqrt(pow(gpsvel[0],2)+pow(gpsvel[1],2)+pow(gpsvel[2],2));
		if(gpsspeed<0.2)
		{
			count++;
			ax_gstaic.push_back(stdax);
			ay_gstaic.push_back(stday);
			az_gstaic.push_back(stdaz);
			gx_gstaic.push_back(stdgx);
			gy_gstaic.push_back(stdgy);
			gz_gstaic.push_back(stdgz);
            if(count>50)  //GNSS连续10秒
			{
				ax_gstaic.erase(ax_gstaic.begin()); 
				ay_gstaic.erase(ay_gstaic.begin()); 
				az_gstaic.erase(az_gstaic.begin()); 
				gx_gstaic.erase(gx_gstaic.begin()); 
				gy_gstaic.erase(gy_gstaic.begin()); 
				gz_gstaic.erase(gz_gstaic.begin()); 

				threadax=3*GetAveStd(ax_gstaic,0);
				threaday=3*GetAveStd(ay_gstaic,0);
				threadaz=3*GetAveStd(az_gstaic,0);
				threadgx=3*GetAveStd(gx_gstaic,0);
				threadgy=3*GetAveStd(gy_gstaic,0);
				threadgz=3*GetAveStd(gz_gstaic,0);
				//gilc_log("重置stdacc:%f, %f, %f, stdgyo:%f, %f, %f\n",threadax,threaday,threadaz,threadgx,threadgy,threadgz);
			}
		}
		else
		{
			count=0;
			gx_gstaic.clear();
			gy_gstaic.clear();
			gx_gstaic.clear();
			ax_gstaic.clear();
			ay_gstaic.clear();
			az_gstaic.clear();
		}
	}

	int rt=0;
	if(insnum<100) /*dsf90:GNSS更新正常*/
	{
		double speed = sqrt(pow(gpsvel[0],2)+pow(gpsvel[1],2)+pow(gpsvel[2],2));
		if(stdax<threadax && stday<threaday && stdaz<threadaz && stdgx<threadgx && stdgy<threadgy && stdgz<threadgz && speed<0.2)
		{
			num_static++;
			if(num_static>25)
			{
				//gilc_log("stdacc:%f, %f, %f, stdgyo:%f, %f, %f\nthracc:%f, %f, %f, thrgyo:%f, %f, %f\n",
				//stdax,stday,stdaz,stdgx,stdgy,stdgz,threadax,threaday,threadaz,threadgx,threadgy,threadgz);
				rt=1;
			}
		}
		else
		{num_static=0;}
	}
	else /*dsf90:GNSS更新异常*/
	{
		//double speed = sqrt(pow(insvel[0],2)+pow(insvel[1],2)+pow(insvel[2],2));
		if(stdax<threadax && stday<threaday && stdaz<threadaz && stdgx<threadgx && stdgy<threadgy && stdgz<threadgz)
		{
			num_static++;
			if(num_static>25)
			{
				//gilc_log("stdacc:%f, %f, %f, stdgyo:%f, %f, %f\nthracc:%f, %f, %f, thrgyo:%f, %f, %f\n",
				//stdax,stday,stdaz,stdgx,stdgy,stdgz,threadax,threaday,threadaz,threadgx,threadgy,threadgz);
				rt=1;
			}
		}
		else
		{num_static=0;}
	}

	return rt;
}
#endif
bool DetectStatic::DetectStatic_byOdo(double dWheelVel,int num)
{
	int i = 0, iWheelVel_ave = 0;
	int iStaticFlag = (dWheelVel<=0.02?0:1);
	if(vWheelVel.size()<num)
	{
		vWheelVel.push_back(iStaticFlag);
		bStatic_ret = bStatic_ret;
	}
	else
	{
		vWheelVel.erase(vWheelVel.begin()); 
		vWheelVel.push_back(iStaticFlag);
		
		for (i = 0; i < num; i++)
		{
			iWheelVel_ave += vWheelVel[i];
		}

		if(bStatic_ret)
		{
			if(iWheelVel_ave)
				bStatic_ret = false;
		}
		else
		{
			if(!iWheelVel_ave)
				bStatic_ret = true;
		}
	}
	return bStatic_ret;
}
bool DetectStatic::DetectStatic_car_dsf(double ep[6],double acc[3],double gyo[3],double gpsvel[3],double insvel[3],int bGPSavail,int bGnssLost,int bGnssState)
{
#define NUM           500
#define INIT_NUM     (60*100) /*60s*/
	double dImuData_raw[6] = {0.0};
	double dImuData_iir[6] = {0.0};
	double ins_speed = insvel[1];
	int bgnss = !bGnssLost;
	int i = 0;

	memcpy(dAcc,acc,sizeof(dAcc));
	memcpy(dGyro,gyo,sizeof(dGyro));
	memcpy(&dImuData_raw[0],dAcc,sizeof(dAcc));
	memcpy(&dImuData_raw[3],dGyro,sizeof(dGyro));
	iir_filter.IIR_Lowpass_Oder3(dImuData_iir, dImuData_raw, dIIR_Order3_bw_fs_0_5, 6);
	memcpy(dAcc_iir,&dImuData_iir[0],sizeof(dAcc_iir));
	memcpy(dGyro_iir,&dImuData_iir[3],sizeof(dGyro_iir));	
	
#if 0
	if (bgnss && bGnssState == 4)
	{		
		if(bDetect_pre!=1)
		{
			bDetect_pre = 1;
			num_static = 0;
			bStatic_gnss = bStatic_ret;
			for(i=0;i<3;i++)
			{
				axyz_iir[i].clear();
				gxyz_iir[i].clear();
				axyz[i].clear();
				gxyz[i].clear();
			}
		}
		
		if(bGPSavail)
		{
			double gnss_speed = sqrt(pow(gpsvel[0],2)+pow(gpsvel[1],2));
			if(bStatic_ret !=1)
			{
				/*dsf90:500ms连续小于0.05，非静止状态-->进入静止状态*/
				if (gnss_speed < 0.05)
				{
					num_static++;
					if(num_static>=2)
					{
						bStatic_gnss = 1;
						num_static = 0;
						gilc_log("%02d:%02d:%06.3f ----in static by gnss------\r\n", int(ep[3]), int(ep[4]), ep[5]);	
					}
				}
				else
				{
					num_static = 0;
				}
			}
			
			if(bStatic_ret!=0)
			{
				/*dsf90:500ms连续大于0.1，静止状态-->推出静止状态*/
				if (gnss_speed > 0.1)
				{
					num_static++;
					if(num_static>=2)
					{
						bStatic_gnss = 0;
						num_static = 0;
						gilc_log("%02d:%02d:%06.3f ---out static by gnss------\r\n", int(ep[3]), int(ep[4]), ep[5]);	
					}
				}
				else
				{
					num_static = 0;
				}
			}
		}
#if 0
		if (bStatic_gnss && bInstallOk) /*dsf90:bInstallOk==1 算法收敛,加速度、陀螺仪误差稳定*/
		{
			if (axyz[0].size()<NUM)
			{
				for(i=0;i<3;i++)
				{
					axyz_iir[i].push_back(ilcd.acc_iir[i]);
					gxyz_iir[i].push_back(ilcd.gyo_iir[i]);
					axyz[i].push_back(ilcd.acc[i]);
					gxyz[i].push_back(ilcd.gyo[i]);
				}
			}
			else
			{
				int j = 0;
				
				for(j = 0;j<100;j++)
				{
					for(i=0;i<3;i++)
					{
						axyz_iir[i].erase(axyz_iir[i].begin()); 
						gxyz_iir[i].erase(gxyz_iir[i].begin());
						axyz[i].erase(axyz[i].begin()); 
						gxyz[i].erase(gxyz[i].begin());
					}
				}
				
				for(j = 0;j<100;j++)
				{
					for(i=0;i<3;i++)
					{
						axyz_iir[i].erase(axyz_iir[i].end()-1); 
						gxyz_iir[i].erase(gxyz_iir[i].end()-1);
						axyz[i].erase(axyz[i].end()-1); 
						gxyz[i].erase(gxyz[i].end()-1);
					}
				}
		
				if (init_flag == 0)
				{
					init_flag += NUM;

					for(i=0;i<3;i++)
					{
						stdaxyz[i] = GetAveStd(axyz[i], 1);/*dsf90:标准差*/
						stdgxyz[i] = GetAveStd(gxyz[i], 1);
						difaxyz[i] = GetMaxMinDiff(axyz_iir[i], 1);/*dsf90:极差*/
						difgxyz[i] = GetMaxMinDiff(gxyz_iir[i], 1);
					}
				}
				else
				{
					if(init_flag < INIT_NUM)  
						init_flag += NUM;
					
					for(i=0;i<3;i++)
					{
						stdaxyz[i] = 0.9*stdaxyz[i] + 0.1*GetAveStd(axyz[i], 1);/*dsf90:标准差*/
						stdgxyz[i] = 0.9*stdgxyz[i] + 0.1*GetAveStd(gxyz[i], 1);
						difaxyz[i] = 0.9*difaxyz[i] + 0.1*GetMaxMinDiff(axyz_iir[i], 1);/*dsf90:极差*/
						difgxyz[i] = 0.9*difgxyz[i] + 0.1*GetMaxMinDiff(gxyz_iir[i], 1);
					}					
				}
				
				for(i=0;i<3;i++)
				{
					axyz_iir[i].clear();
					gxyz_iir[i].clear();
					axyz[i].clear();
					gxyz[i].clear();
				}

				gilc_log("%02d:%02d:%06.3f Get stdacc,%3.5f,%3.5f,%3.5f;stdgyro,%3.5f,%3.5f,%3.5f;difacc,%3.5f,%3.5f,%3.5f;difgyof,%3.5f,%3.5f,%3.5f\r\n",
					int(ilcd.ep[3]), int(ilcd.ep[4]), ilcd.ep[5],
					stdaxyz[0], stdaxyz[1], stdaxyz[2], 
					stdgxyz[0], stdgxyz[1], stdgxyz[2],
					difaxyz[0], difaxyz[1], difaxyz[2], 
					difgxyz[0], difgxyz[1], difgxyz[2]);
/*
				if(stdax/para.AW[0] > 0.1 && stdax/para.AW[0] < 10 &&
					stday/para.AW[1] > 0.1 && stday/para.AW[1] < 10 &&
					stdaz/para.AW[2] > 0.1 && stdaz/para.AW[2] < 10 &&
					stdgx/para.GW[0] > 0.1 && stdgx/para.GW[0] < 10 &&
					stdgy/para.GW[1] > 0.1 && stdgy/para.GW[1] < 10 &&
					stdgz/para.GW[2] > 0.1 && stdgz/para.GW[2] < 10)
				//if(stdgx < 3*para.GW[0] && stdgy < 3*para.GW[1] && stdgz < 3*para.GW[2] )
				
				if(init_flag >= INIT_NUM)  
				{
					for(i=0;i<3;i++)
					{
						para.AW[i] = stdaxyz[i];
						para.GW[i] = stdgxyz[i];
					}				
					kf.setQk(para.AW,para.GW);
					gilc_log("%02d:%02d:%06.3f   Set stdparam ok! \r\n",int(ilcd.ep[3]), int(ilcd.ep[4]), ilcd.ep[5]);
				}
				*/
			}
		}
		else
		{
			for(i=0;i<3;i++)
			{
				axyz_iir[i].clear();
				gxyz_iir[i].clear();
				axyz[i].clear();
				gxyz[i].clear();
			}
		}
#endif
		bStatic_ret = bStatic_gnss;
	}
	else
#endif
	{
		double stdax_imu = 0, stday_imu = 0, stdaz_imu = 0, stdgx_imu = 0, stdgy_imu = 0, stdgz_imu = 0;
		bool bStatic_imu = 0;
		
		if(bDetect_pre!=2)
		{
			bDetect_pre = 2;
			for(i=0;i<3;i++)
			{
				axyz_iir[i].clear();
				gxyz_iir[i].clear();
				axyz[i].clear();
				gxyz[i].clear();
			}
			IsStatic(stdaxyz,stdgxyz,ins_speed,bStatic_ret);
		}
#if 0
		if(init_flag >= INIT_NUM)  
			bStatic_imu = IsStatic(difaxyz,difgxyz,ins_speed,3);
		else
			bStatic_imu = IsStatic(difaxyz,difgxyz,ins_speed,2);
#else
		bStatic_imu = IsStatic(difaxyz,difgxyz,ins_speed,2);
#endif
		if(bStatic_ret != bStatic_imu)
		{
			if(bStatic_imu)
				gilc_log("%02d:%02d:%06.3f ----in static by imu-------\r\n", int(ep[3]), int(ep[4]), ep[5]);	
			else
				gilc_log("%02d:%02d:%06.3f ---out static by imu-------\r\n", int(ep[3]), int(ep[4]), ep[5]);	
		}
		bStatic_ret = bStatic_imu;
	}

	return bStatic_ret;

}

