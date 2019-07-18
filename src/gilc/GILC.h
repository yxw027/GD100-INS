#ifndef _GILC_H
#define _GILC_H

#include <stdlib.h>
#ifndef STM32
#include <stdio.h>
#include <iostream>
#endif
#include <math.h>
#include <string>
#include <vector>
#include <stdarg.h>
#include <algorithm> 
#include <time.h>
#ifdef __linux
#include <memory.h>
#endif

#include "log.h"
#include "filter.h"
#include "GILC_Vehicle_lib.h"
//#include "PCA.h"
#ifdef STM32
#include "malloc.h"
#include "myprintf.h"
#include "malloc_allocator.h"
#elif defined(WIN32)
template<typename T>
using malloc_allocator = class std::allocator<T>;
#elif defined(__linux)
#define malloc_allocator allocator
#endif

#define AP100_ADI16445 0
#define CGI310_murata  0
#define NX200_IMU381   0
#define PCA_IMU381     0
#define I90_IMU381     0


#define NUMX 21
#define NUMV 13

#define CAL_UPDATE_Z_WHEEL_HEADING    0x0001
#define CAL_UPDATE_Z_WHEEL_VEL        0x0002
#define CAL_UPDATE_Z_DUAL_YAW         0x0004
/***********************************UPDATE FLAG************************************/
#define UPDATE_TYPE_STATIC        0x0001
#define UPDATE_TYPE_CONSTRAINT    0x0002
#define UPDATE_TYPE_GNSS          0x0004
#define UPDATE_TYPE_ODO           0x0008

#define UPDATE_Z_ATT_X    0x0001
#define UPDATE_Z_ATT_Y    0x0002
#define UPDATE_Z_ATT_Z    0x0004
#define UPDATE_Z_ATT_XYZ  0x0007

#define UPDATE_Z_VER_X    0x0008
#define UPDATE_Z_VER_Y    0x0010
#define UPDATE_Z_VER_Z    0x0020
#define UPDATE_Z_VER_XYZ  0x0038

#define UPDATE_Z_POS_X    0x0040
#define UPDATE_Z_POS_Y    0x0080
#define UPDATE_Z_POS_Z    0x0100
#define UPDATE_Z_POS_XYZ  0x01C0

#define UPDATE_Z_CONS_VER_X   0x0200
#define UPDATE_Z_CONS_VER_Y   0x0400
#define UPDATE_Z_CONS_VER_Z   0x0800
#define UPDATE_Z_CONS_VER_XZ  0x0A00
#define UPDATE_Z_CONS_VER_XYZ 0x0D00

#define UPDATE_Z_ODO_HEADING  0x1000
#define UPDATE_Z_ODO_VER      0x2000

/**********************************************************************************/
#define REAL_PROCESS

#define P2CAR
#define GNSSFIX 4
#define GNSSFLOAT 5
#define GNSSDGPS 2
#define GNSSSINGLE 1
#define RE_WGS84    6378137.0           // earth semimajor axis (WGS84) (m)
#define FE_WGS84    (1.0/298.257223563) // earth flattening (WGS84)
#define PI		    3.14159265358979
#define D2R         (PI/180.0)          /* deg to rad */
#define R2D         (180.0/PI)          /* rad to deg */
#define glv_g0     9.7803267714
#define glv_g      9.80665              /*add by dsf90,2018.6.26*/
#define glv_deg    0.017453292519943
#define MAXVAL		50 // max value for spilting
#define MAXLEN		1024
#define EPS		2.22044604925031e-16
#define INF		1.0e100
#define COMMENTH "%"
#define DLENGTH 0.5

#define SQR(x)      ((x)*(x))
#define SQ(X)       ((X)*(X))
#define DEG_0_360(x)       {if (x > 360) x -= 360;    else if (x < 0)	   x += 360; }
#define DEG_NEG180_180(x)  {if (x > 180) x -= 360;    else if (x < -180)   x += 360; }

using namespace std;

const static double gpst0[]={1980,1, 6,0,0,0}; /* gps time reference */
const static double gst0 []={1999,8,22,0,0,0}; /* galileo system time reference */
const static double bdt0 []={2006,1, 1,0,0,0}; /* beidou time reference */

//判定动态对准完成的状态方差矩阵P对应的标准差阀值 先以航向(0.035对应2°)和高程为准
static double PXK[15]={0.02,0.02,0.0174533,0.1,0.1,0.1,2*10E-8,2*10E-8,0.3,0.001,0.001,0.001,0.14,0.035,0.015};
// class define
class CGLV;
class InitPara;
class PriPara;
class CEarth;
class CSINS;
class GIKF; //不同的组合方式通过扩充这个类来实现
class Algin; //动态对准流程
class GIProcess; //组合导航解算流程

extern CGLV	glv; //要在cpp文件中定义 这里只是声明
extern InitPara InitParam;

/*****************Common Function*********************/

extern bool is_nan(double dVel);
//临时申请内存 要free
extern void *__ml_zero(int size);
//字符串转数组，格式化转换
extern int Str2Array(const char *str,const char *sep,double *val);
//查找第一个子字符串
extern int checkstr(const char* str,const char* substr);
extern double GetAveStdRMS(const double *a, int n, int opt);
extern double GetAveStd(vector<double, malloc_allocator<double> > a,int opt);
extern double GetAveStd(vector< vector<double, malloc_allocator<double> > > a,int col,int opt); //col定义列数 从0开始
extern double GetMaxMinDiff(vector<double, malloc_allocator<double> > a,int opt);
extern char* time2str(double *ep, int n);
extern double str2num(const char *s, int i, int n); 
extern void ecef2pos(const double *r, double *pos); //xyz2blh
extern void pos2ecef(const double *pos, double *r); //blh2xyz
extern void xyz2enu(const double *pos, double *E);  //blh2 Cxyz2enu
extern void ecef2enu(const double *pos, const double *r, double *e); //xyz坐标系向量r转化到enu
extern void enu2ecef(const double *pos, const double *e, double *r); //enu坐标系向量r转化到xyz
extern void covenu(const double *pos, const double *P, double *Q);   //covariance: ecef2enu
extern void covecef(const double *pos, const double *Q, double *P);  //covariance: enu2ecef
void Var_XYZ2BLH(double xyz[3],double Pecef[3],double Penu[3]);      //var: ecef2enu
extern void matmul(const char *tr, int n, int k, int m, double alpha,
	              const double *A, const double *B, double beta, double *C);
extern double dot (const double *a, const double *b, int n);
extern double norm(const double *a, int n);
extern void cross3(const double *a, const double *b, double *c);
extern void getHMS(double ggat,double ep[6]);
extern void getPOS_rad( double lat,double lon,double hgt,double blh[3]);
extern void diffpos(double blhpre[3],double blhcur[3],double denu[3]);
//time function
const static double leaps[][7]={ /* leap seconds {y,m,d,h,m,s,utc-gpst,...} */
	{2017,1,1,0,0,0,-18},
	{2015,7,1,0,0,0,-17},
	{2012,7,1,0,0,0,-16},
    {2009,1,1,0,0,0,-15},
    {2006,1,1,0,0,0,-14},
    {1999,1,1,0,0,0,-13},
    {1997,7,1,0,0,0,-12},
    {1996,1,1,0,0,0,-11},
    {1994,7,1,0,0,0,-10},
    {1993,7,1,0,0,0, -9},
    {1992,7,1,0,0,0, -8},
    {1991,1,1,0,0,0, -7},
    {1990,1,1,0,0,0, -6},
    {1988,1,1,0,0,0, -5},
    {1985,7,1,0,0,0, -4},
    {1983,7,1,0,0,0, -3},
    {1982,7,1,0,0,0, -2},
    {1981,7,1,0,0,0, -1}
};
typedef struct {        /* time struct */
    time_t time;        /* time (s) expressed by standard time_t 从1970.01.01 0秒到现在的秒数 long int*/
    double sec;         /* fraction of second under 1 s */
} gtime_t;
extern double timediff(gtime_t t1, gtime_t t2);
extern gtime_t timeadd(gtime_t t, double sec);
extern int str2time(const char *s, int i, int n, gtime_t *t);
extern gtime_t gpst2utc (gtime_t t);
extern gtime_t epoch2time(const double *ep);
extern void time2epoch(gtime_t t, double *ep);
extern gtime_t gpst2time(int week, double sec);
extern double time2gpst(gtime_t t, int *week);

/* Matrixs calculation function, by DHF.20160510---------- */

//c[m][n]=a[m][n]+b[m][n]
void Maddn(double *a,double *b,double *c,int m,int n);
//a[m][n]=a[m][n]+b[m][n]
void Madd(double *a,double *b,int m,int n);
//c[m][n]=a[m][n]-b[m][n]
void Mminn(double *a,double *b,double *c,int m,int n);
//a[m][n]=a[m][n]-b[m][n]
void Mmin(double *a,double *b,int m,int n);
//c[m][k]=a[m][n]*b[n][k]
void Mmulnm(double *a,double *b,int m,int n,int k,double *c);
//a[m][n]=a[m][k]*b
void Mmul(double *a,int m,int n,double b);
//c[m][n]=a[m][k]*b
void Mmuln(double *a,int m,int n,double b,double *c);
//b=aT
void Mtn(double *a,int m,int n,double *b); 
//a=aT
void Mt(double *a,int m,int n); 
//inv(a)
double Minv(double a[],int n);
//b=inv(a)
double Minvn(double a[],int n,double *b);
 //A* adjoint matrix  
double Mrem(double *a,int i,int j,int n); 
 //det 
double Mdet(double *a,int n);
//N[m][n]=M[m][n]
void Mequalm(double *M,int m,int n,double *N);
//M[m][n]=a
void Mequal(double *M,int m,int n,double a);
//mean of col
double Mmean(double *a,int m);
//求实对称矩阵的特征值及特征向量的雅格比法 
//利用雅格比(Jacobi)方法求实对称矩阵的全部特征值及特征向量 
//返回值小于0表示超过迭代jt次仍未达到精度要求 
//返回值大于0表示正常返回 
//a-长度为n*n的数组，存放实对称矩阵，返回时对角线存放n个特征值 
//n-矩阵的阶数 
//u-长度为n*n的数组，返回特征向量(按列存储) 
//eps-控制精度要求 
//jt-整型变量，控制最大迭代次数 
int Meejcb(double a[],int n,double v[],double eps,int jt);
//eye
void Munit(double* a,int n);

/* attitude calculation and update function, by DHF.20160510---------- */

//val(3*1)2Skew-symmetric(3*3)
void askew(double v[], double m[]);
//Skew-symmetric(3*3)2val(3*1)
void iaskew(double v[], double m[]);
//四元数转等效旋转矩阵
void q2rv(double q[],double rv[]);
//等效旋转矩阵转四元数
void rv2q(double rv[],double q[]);
//等效旋转矩阵转方向余弦矩阵
void rv2m(double rv[],double m[]);
//旋转向量旋转矢量
void rotv(double rv[],double vi[],double vo[]);
//q-conj
void qconj(double q[],double qc[]);
//四元数相乘 赋值给新矩阵 q=q1*q2
void qmuln(double q1[],double q2[],double q[]);
//四元数相乘 改变第一个原矩阵 q1=q1*q2
void qmul(double q1[],double q2[]);
//方向余弦矩阵转四元数ned
void m2qua_ned(double m[],double q[]);
//四元数转方向余弦矩阵ned
void q2mat_ned(double qua[],double m[]);
//方向余弦矩阵转欧拉角ned
void m2att_ned(double m[],double a[]);
//欧拉角转方向余弦矩阵ned
void a2mat_ned(double att[],double m[]);
//欧拉角2方向余弦矩阵enu
void a2mat(double att[],double m[]); 
//欧拉角2四元数enu
void a2qua(double att[],double qua[]);
//方向余弦矩阵2欧拉角enu
void m2att(double mat[],double att[]); 
//方向余弦矩阵2四元数enu
void m2qua(double mat[],double qua[]);
//四元数2欧拉角
void q2att(double qua[],double att[]);
//四元数2方向余弦矩阵
void q2mat(double qua[],double mat[]);
//方向余弦矩阵2旋转矢量
void m2rv(double mat[],double rv[]);
//旋转矢量2方向余弦矩阵
void rv2m(double rv[],double mat[]);
//向量与四元数相乘 向量坐标系转换
void qmulv(double q1[],double vi[],double vv[]);
//计算四元数减去失准角
void qdelphi(double qpb[],double phi[]);
//姿态更新
void qupdt(double qnb0[],double rv_nb[],double qnb1[]);
//姿态更新，连乘两次
void qupdt2(double qnb0[],double rv_ib[],double rv_in[],double qnb1[]);

void rot_mult(double R[3][3], const double vec[3], double vec_out[3]);

void CnbDotPRY_mul_u(double PRY[3], double U[3], double out[3][3]);

void CbnDotPRY_mul_u(double PRY[3], double U[3], double out[3][3]);

void CnbDotQ_mul_u(double Qbn[4],double U[3],double out[3][4]);

void CbnDotQ_mul_u(double Qbn[4],double U[3],double out[3][4]);
/*-----------------log define by DHF ----------------------*/
extern void fopendhf(const char *file);
extern void fclosedhf(void);
extern void outdhf(const char *format, ...);

/*-----------Dynamic Identify by DHF,2016.10.12-------------*/
extern bool sort_by_value(const double& val1,const double& val2);
class DynamicIdentify
{
public:
	bool bStaticStd; /*add by dsf90,2018.6.24*/
	bool bKin;                               //是否开始运动的标志
	bool bStatic;                            //运动过程中动静判断
	bool bTurn;                              //是否转弯
	int nkin,Nkin;                           //判定为动态的连续历元数
	int ndetect,Ndetect,nwind,Nwind;         //滑动窗口的大小（默认1秒），探测窗口的大小（默认10个历元）
	double axstd,aystd,azstd,gxstd,gystd,gzstd;
	vector<double, malloc_allocator<double> > ax,ay,az,gx,gy,gz,mx,my,mz,Dax,Day,Daz,Dgx,Dgy,Dgz; //大小窗
	double acc_ave[3],gyo_ave[3];
	double daccstd[3],dgyostd[3];
	unsigned int Ndetect1,Ndetect2,Ndetect3;           //不同探测方法的窗口大小
	vector<double, malloc_allocator<double> > Dax2,Day2,Dgz2;            //不同探测方法的窗
public:
	//窗口长度，探测窗口长度，判定动态的连续历元数
	DynamicIdentify(int NW=500,int ND=10,int NKIN=3); 
	//DynamicIdentify& operator=(const DynamicIdentify& dyni);
	//option:0-判断启动dhf 1-动静态判断sxd
	bool Detect(double gyo[3],double acc[3],int option=0);
};

/*********************Loose Couple Date Struct***********************/
class CLCData
{
public:
	odo_data_t stOdoData;
	// gnss data
	double gpstimetarge; 
	int week;
	double ep[6];  // ymdhms
	double pos[3]; // gnss pos(blh)
	double undulation;
	double vn[3];  // gnss vn（ENU）
	int stat,ns;   // sol stat/ sat num
	int snsum,nsused;   // sat signal sum/used sat num
	double hdop;   // HDOP
	int m0;
	double age;    // 0/diffage
	double GPV_RK[36];
	double GA_RK[3];
	double yaw_U62R;
	double yaw_var_U62R;
	double lever;
	double heading2;/*双天线航向*/

	// ins data
	int num;
	double time;   // imu pre time
	double imutimetarge;
	double acc[3]; // xyz加速度(mg)
	double gyo[3]; // xyz陀螺(deg/s)
	double mag[3]; // Gauss
	double temper;
	double acc_iir[3]; // xyz加速度(mg),滤波
	double gyo_iir[3]; // xyz陀螺(deg/s),滤波
	double mag_iir[3]; // Gauss,滤波

	double syst;
	double calt;   // position time
	bool bUpdate, bValid;
	bool bGPSavail,bPPSavail;  //GPS available（pos/vel）, PPSavailable
	bool bOnlyVelUp;     //only veloity update,position don't update
	bool bMEMSavail;
	bool bODOavail;      //odometry updata
	double gnss_speed;
	double heading;

	void Init();
	void Rest();
	//CLCData& operator=(const CLCData& lcdata);
	void getHMS(double gpstimetarge);
	void getPOS_deg(double lat,double lon,double hgt);
	void getPOS_rad(double lat,double lon,double hgt);
	void imuConvert();
};

/****************GNSS INS PROCESS********************/

/*---------------Constant class by DHF,20160713--------------*/
class CGLV
{
public:
	double Re, f, g0, wie, g;
	double mg, ug, deg, min, sec, hur, ppm, ppmpsh;
	double dph, dpsh, dphpsh, ugpsh, ugpsHz, mpsh, mpspsh, secpsh;
	double ep[6];  // ymdhms

	CGLV(double Re=6378137.0, double f=(1.0/298.257), double wie0=7.2921151467e-5, double g0=9.7803267714);
};

/*--------------Prior Parameters of ins class by DHF,20160726------*/
class InitPara
{
public:
	double dGnssVerMin_ForInit;
	int dPpsTimeErrMax_ms;
	double dInstallErrStdMax_ForInit;
	
	double dKfInitP_AccBais;
	double dKfInitP_GyroBais;
	double dKfInitP_InstallErr;
	double dKfInitP_Lever;
	double dKfInitP_TimeDelay;
	
	double dKfInitP_OdoKd;
	double dKfInitP_OdoKwh;
	double dKfInitP_OdoBwh;
	double dKfInitP_OdoWh;
	double dKfInitP_DualYaw;
	double dKfInitP_OdoVel;
	double dKfInitP_OdoHeading;

	double dWheelTrack;
	double dWheelBase;
		
	int iImuPeriod_ms;
};

/*--------------Prior Parameters of ins class by DHF,20160726------*/
class PriPara
{
public:
	double att0[3],vn0[3],pos0[3],
		   davp[9],prnavp[9],prnObs[3],
		   GB[3],GW[3],GS[3],AB[3],AW[3],AS[3],
		   GScater[9],AScater[9],
		   lever[3],tDelay;
	
	PriPara(void) {};
	void setAtt(double pitch,double roll,double yaw,int opt=0); //opt 0:deg（默认）1:rad
	void setVn(double vn[3]);
	// 纬 经 高 rad
	void setPos(double pos[3],int opt=0); //opt 0:lat,lon,hig(默认)  1: x y z
	// 初始状态误差     RTK进程输出的GPS位置和速度标准差信息 航向° 水平角°
	void setdavp(double var_gpv[36],double var_yaw=3,double var_pr=0.5);

	// 量测噪声  根据PTK进程输出的GPS位置和速度标准差信息
	void setpobs(double var_gpv[36]);
	//根据静态数据设置IMU的噪声项 动态噪声需要放大
	void setimu(double avegyo[3],double accbias[3],double gw[3],double aw[3],double gs[3],double as[3],int scater=3);
	void setimus(double gs[9],double as[9]);
	void setlever(double lev[3],double dt=0.0);	         
};

/*--------------Earth model class by DHF,20160713-----------*/
class CEarth
{
public:
	double a,b;
	double f,e,e2;
	double wie;

	double sl,sl2,sl4,cl,tl,RMh,RNh,clRNh,f_RMh,f_RNh,f_clRNh;
	double wnie[3],wnen[3],wnin[3],gn[3],gcc[3];

	CEarth(double a0=glv.Re, double f0=glv.f, double g0=glv.g0);
	//CEarth& operator=(const CEarth& eth);
	void UpdateP(double pos[]);
	void UpdatePV(double pos[],double vn[]);
};

/*---------Inertia mechanization class by DHF,20160713----------*/
class CSINS
{
public:
	CEarth eth;
	double Kd;
	double Kwh;
	double Bwh;
	double Byaw; /*双天线安装误差*/
	double tDelay;
	double qnb[4];
	double Cnb[3*3];
	/*dsf90 add,2018.4.25,安装误差相关变量, m系: mobile 坐标系*/
	double PRY_Install[3]; 
	double qmb[4]; 
	double Cmb[3*3];
	double Cbm[3*3];
	double vm_car[3];
	double att_car[3];
	double Cnm[3*3];
	/*end add*/
	double att[3],vn[3],pos[3],vnL[3],posL[3],vnL2[3],posL2[3];
	double eb[3],db[3],lever[3],lever2[3],Kg[3*3],Ka[3*3];
	double fn[3],an[3],web[3],wnb[3],wib[3],fb[3],ab[3],am[3],wim[3], fm[3];

	double Mpv[3*3],Mpvvn[3],MpvCnb[9],CW[9];
	double wm_1[3],vm_1[3];
	
	double imutimetarge;
	double WheelHeadingScale;
	double wheel_heading;/*ins推算前轮转角*/
	double wheel_heading_h;/*前轮转角观测量*/
	double wheel_heading_raw;/*EKF修正前的前轮转角*/
	double wheel_heading_e;/*EKF修正后的前轮转角*/
	double yaw_rate_byodo;
	
	double wheel_vel;  /*轮速*/
	double wheel_vel_h;/*轮速观测量*/
	
	double dual_yaw;  /*双天线航向*/
	double dual_yaw_h;/*双天线航向观测量*/

	CSINS(void) { imutimetarge = 0; };
	CSINS(double Att[],double Vn[],double Pos[],double EB[],double DB[],double Lever[]);
	//CSINS& operator=(const CSINS& ins);
	void Init(double Att[],double Vn[],double Pos[],double EB[],double DB[],double Lever[],double Lever2[],double PRY_install[]);
	void Update(double wm[],double vm[],double dtime);
	void Lever();
};

/*----------Kalman filter class by DHF,20160713------*/
class GIKF
{
public:
	int ROW,COL,OPT;/*dsf90:ROW——NUMX;COW——NUMW*/
	double* xk;
	double* Pxk;
	double* Phi;
	double* Qk;
	double* Gk;
	double* Hk;
	double* Rk;
	double* xkpre;
	double* dpos;
	double* denu;
	int  xflag;
	int  zflag;
	bool bGnssUpdata;
	double kf_Q_init[NUMX];
	double kf_P_init[NUMX];

	double davp[9], GB[3], AB[3], GW[3], AW[3], GS[3], AS[3];

	GIKF(int row=NUMX,int col=NUMV,int opt=156);
	//GIKF(const GIKF& kftemp);                                     //深拷贝
	GIKF& operator=(const GIKF& kftemp);
	void kffree();                                                //释放内存，销毁kf对象前，必须调用此函数
	void kfinit();
	void setxkPhiHk();                                            //同时设定xkpre,dpos,denu
	void setPxk(double davp[],double GB[],double AB[]);
	void upPhi(CSINS& ins,double dt);
	void setQk(double GW[],double AW[]);
	void setGk();
	void rsetGk(CSINS& ins,int option=0);
	void upHk(CSINS& ins,double *hk);  
	void upHk_NUMX22(CSINS& ins,double *hk);
	void upHk_NUMX24(CSINS& ins, double *hk);
	void setRk_constraint(void);
	void resetRk(CLCData ilcd);
	void downgrade_car(CLCData ilcd,double denu[3],double posrk[3]);  
    void downgrade(CLCData ilcd,int scater,double posrk[3]);     
	void TUpdate(double dt,int option=1);                        
	void MUpdate(double ZK[]);
	void MUpadte(CSINS& ins,double ZK[],int zflag = 0xffff,int opt=0);
	void Feedback(CSINS& ins,double scater=1.0,int option=0);
};
void kfequal(GIKF* kfold,GIKF* kfnew);

/*----------Kinmate Align class by DHF,using GNSS postion,20170512-----*/
struct gpos
{
	double time;
	double lat,lon,hig;
	double ve,vn,vu;
	double yaw;
	int state;
};
typedef struct gpos gpos_t;
extern void equalgpos(gpos_t* GP,CLCData* lcdata);
class KinAlign
{
public:
	bool bFinshAlign;
	bool bStatic;
	double PRY_Install[3]; /*安装误差初始值*/
	double Att[3],Vn[3],Pos[3];
	int gnssstate;
	double Pgposvn[36];
	unsigned int ngnss,Ngnss;
	int nspeed;
	vector<gpos_t, malloc_allocator<gpos_t> > v_GNSS;
	vector<double, malloc_allocator<double> > yaw_gnss;
	//判断为静态 返回false 判断动态则计算航向 
	DynamicIdentify dyni;                    //动静态判断
	KinAlign(int NGNSS=10);
	//KinAlign& operator=(const KinAlign& kinalign);
	void Init(void);
	bool CalAtt(CLCData& ilcd,int opt = 0);              //根据位置计算航向
	bool CalAtt2(CLCData& ilcd);             //根据速度计算航向
	bool KinmateAlign(CLCData& ilcd,GIProcess& gipro);
};

/*----------GI Proceass class by DHF,20160727------*/
#include "GILC_KF.h"
class GIProcess
{
public:
	double dt,tpre,dt_total;                       
	int Row,Col,Opt;                     
	CSINS ins;                            
	CSINS inspre;
	CSINS inspre_forPPS;
	CSINS inspre_forStatic;
	double dGnssHeading2_forStatic;
	GIKF kf;                              
	GIKF_Calibrate kf_cal;                              
	GIKF_Calibrate_DualAnt kf_cal_dualAnt;
	PriPara para;                        
	DynamicIdentify dyni;
	DetectStatic detect;
	bool bFileSave;
	double dInitTimesMin;

	int iGilcRunMode; /*0:Normal; 1:Calibrate;*/
	int iInitTimes_ms;

	int c;                          
	bool bAlign;                         
	bool bStatic;                        
	int iDriverMode; /*add by dsf90, 2018.5.31*/
	bool bInstallOk; /*add by dsf90, 2018.6.6*/
	double bGnssTimeOut;
	bool bStaticpre;
	double kfpre[NUMX*NUMX];
	
	bool bOdoLost;                        
	double bOdoTimeOut; 
	
	bool bGnssLost;                        
	int  bGnssLostNum;                        
	bool busegnssvn;                        
	int  bGnssNum;                        
	bool bgnssskip;                        
	int bTurn;                            
	int bFinshMoveAlign;                 
	int num_FinshMoveAlign;                
	int num_GNSSrestore;                 
	int num_GNSSmupdate;                  
	double preheading;

	int num_GPSskip;                    
	int num_GPSloose;                    
	int num_ContinueFloat;               
	int num_ContinueFix;                
	int upmodel;                          
	double wmpre[3],vmpre[3];            

	double pospre[3];
	
	double dpos_sync[3];
	double dvn_sync[3];
	double dposb_sync[3];
	bool bPPSSync;

	/*里程计速度航向偏差*/
	bool bWheelHeadingCalibrateOk;
	bool bWheelVelCalibrateOk;
	bool bDualAntCalibrateOk;
	bool bWheelHeadingInitOk;
	bool bWheelVelInitOk;
	bool bDualAntInitOk;
	
	/*双天线姿态航向偏差*/
	double dDualAntYawBias;
	bool bDualAntAvail;           /*双天线启用标志*/
	double dDiffYaw_Ins2DualAnt;
	vector <double, malloc_allocator<double> > dDualAntYawBias_buf;

	
	/*里程计速度航向偏差*/
	double dOdoHeadingBias_ave;
	double dOdoHeading_true;
	bool bOdoHeadingAvail;
	double dDiffHeading_Ins2Odo;
	vector <double, malloc_allocator<double> > dOdoHeadingBias_buf;

	int iInstallOk_cnt;
	vector<vector<double, malloc_allocator<double> > > Install;
	
	vector<GIKF, malloc_allocator<GIKF> >ppsKf;
	vector<gpos, malloc_allocator<gpos> >pregpos;
	vector<int, malloc_allocator<int> >gpsstate;
	vector<vector<double, malloc_allocator<double> > >prePxk;
	double var_ins[9];                   
	//GIProcess(void){};
	GIProcess(int row=NUMX,int col=NUMV,double sam_int=0.01);
	//GIProcess& operator=(const GIProcess& gipro);
	void Init(void);
	void IMUcone(CLCData ilcd,double wmm[3],double vmm[3]); 
	void correctSideslip(void);
	void loadPPSSyncInsData(CLCData &ilcd,CSINS &ppsins);
	void getDualAntBias(CLCData &ilcd,double dDifYaw);
	void getOdoHeadingBias(CLCData &ilcd,double dDifHeading);
	void KfInitOver();
	void updateKfInitStatus(CLCData &ilcd);
	void GnssIntegrity(CLCData &ilcd,double dheading,double dvn[3]);
	void OdoIntegrity(CLCData &ilcd);
	int GIPro_P2(CLCData ilcd);          
	void setlever(double leverdt[]);
	int ZUpdate(CLCData ilcd);
};

/*--------静态判断(adis16460),by dhf,20180201------*/
extern int DetectStatic_car(double acc[3],double gyo[3],double gpsvel[3],double insvel[3],int bupgnss,int insnum);
/*--------速度约束更新位置增量在b系的投影，by dhf,20180206---------*/
extern void difpos_b(double pospre[3],double poscur[3],double att[3],double dpos_b[3]);
extern void difpos(double pospre[3],double poscur[3],double att[3],double dpos_b[3]);
/*---------确定GNSS速度是否可用，by dhf,20180201--------*/
/* arge I: gnssvel:  the velocity of GNSS
           dvel:     the differ of GNSS velocity and LC velocity
		O: 1:        use
		   0:        unuse */
extern bool busegnssvel_car(double gnssvel[3],double dvel[3]);
/*---------确定是否是GNSS跳点，by dhf,20180211--------*/
extern bool bgnssskip_car(vector<gpos_t, malloc_allocator<gpos_t> > v_gps,double iposcur[3],int numins);
/*---------通过判断加速度计零偏变化，设置自适应观测噪声，by dsf90,20180528--------*/
extern int AccBias_monitor(CSINS& ins, double xk[3], int opt);
/*---------驾驶模式判定，by dsf90,20180528--------*/
extern int DetectCar_DriverMode(double acc[3],double gyo[3],double insvel[3]);
#endif
