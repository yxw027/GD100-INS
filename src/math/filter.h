#ifndef _FILTER_H_
#define _FILTER_H_

#include <stdlib.h>
#include <vector>
//#include "ins_type.h"

using namespace std;

#define  MAX_CHAN	6
#define  IIR_NUM	12

#define   IMU_FS                    100
#define   DATA_LEN                  (int)(IMU_FS*1)
#define   BUF_LEN                   (int)(IMU_FS*2)

#define   F_MOV_THRESHOLD            (0.05f)
#define   S_AVE_THRESHOLD			 (0.2f)

#define   A_AVE_THRESHOLD			 (0.003)  /*单位 m/s,ACC_STD_X*/
//#define   A_AVE_THRESHOLD			 (0.01)  /*单位 m/s*/

#define   W_AVE_THRESHOLD			 (1.0f)
#define   W_AVE_THRESHOLD2			 (1.0f)   /*单位 °/s*/

double GetSum(double *p, int nFrom, int nTo);
double GetAve(double *p, int nFrom, int nTo);
void GetMaxMinAve(double *p, int nFrom, int nTo, double *dMax, double *dMin, double *dAve);

class Filter
{
public:
	double iir_y[MAX_CHAN][IIR_NUM], iir_x[MAX_CHAN][IIR_NUM];
	int bFirstRun;
	Filter(void);
	void Init(void);
	void IIR_Lowpass_Oder3(double *dOut, double *dIn, double *dIIR, short nChan);
	void IIR_Lowpass_Oder5(double *dOut, double *dIn, double *dIIR, short nChan);
	void IIR_Lowpass_Oder10(double *dOut, double *dIn, double *dIIR, short nChan);
	void IIR_Lowpass(double *dOut, double *dIn);
};
class DetectStatic
{
public:
	Filter iir_filter;
	double dAcc[3];
	double dGyro[3];
	double dAcc_iir[3];
	double dGyro_iir[3];
	double dAcc_std[3];
	double dGyro_std[3];
	double dAcc_iir_std[3];
	double dGyro_iir_std[3];
	
	double dMax[6];
	double dMin[6];
	double dAve[6];
	
	int iNum_std;
		
	int iStaticAcc;
	int iStaticGyro;

	double   dDetectIMU_fifo[6][DATA_LEN];
	int      nIndex_Data;

	char   s_cStateLast[BUF_LEN];
	int    s_pStateSp;
	int    s_iState_pre;

	vector<double> axyz[3],gxyz[3],axyz_iir[3],gxyz_iir[3];
	vector<int> vWheelVel;
	double stdaxyz[3], stdgxyz[3];
	double difaxyz[3], difgxyz[3];
	int init_flag;
	bool bStatic_ret;
	int bDetect_pre;
	
	int num_static;
	int bStatic_gnss;
	
	DetectStatic(void);
	//DetectStatic& operator=(const DetectStatic& detect);
	
	void Init(void);	
	int DetectIMU(double stdAcc[3], double stdGyro[3],int opt);
	int IsStatic(double stdAcc[3], double stdGyro[3], double ins_speed,int opt);
	//int DetectStatic_car(double acc[3],double gyo[3],double gpsvel[3],double insvel[3],int bupgnss,int insnum);
	
	bool DetectStatic_byOdo(double dWheelVel, int num);
	bool DetectStatic_car_dsf(double ep[6],double acc[3],double gyo[3],double gpsvel[3],double insvel[3],int bGPSavail,int bGnssLost,int bGnssState);
	void SvmFormTrainDataCreat(int bStatic);
};

#endif /* EKF_H_ */
