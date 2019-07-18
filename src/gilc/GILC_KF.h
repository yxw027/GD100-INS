#ifndef _GILC_KF_H
#define _GILC_KF_H

#include "hc_type.h"
#include "GILC.h"

#ifdef __cplusplus
extern "C" {
#endif
/*----------Kalman filter class by DSF,20190212------*/

void TUpdate_True(int ROW,int COL,double Qk[],double Gk[],double Phi[],double xk[], double Pxk[], double dt,int opt);
void MUpdate_True(int ROW,int COL,double Hk[],double Rk[],double ZK[], double xk[], double Pxk[]);
void MUpdate_Variable(int ROW,int COL,double Hk[],double Rk[],double ZK[], double xk[], double Pxk[], int zflag);

class GIKF_Calibrate
{
public:
	int ROW,COL;/*dsf90:ROW——NUMX;COL——NUMW*/
	double* xk;
	double* xk_std;
	double* Pxk;
	double* Phi;
	double* Qk;
	double* Gk;
	double* Hk;
	double* Rk;
	int  xflag;
	int  zflag;

	GIKF_Calibrate(int row=7,int col=3);
	GIKF_Calibrate& operator=(const GIKF_Calibrate& kftemp);
	
	void kffree();                                                //释放内存，销毁kf对象前，必须调用此函数
	void kfinit();
	void upPhi(CSINS& ins,double dt);
	void upHk(CSINS& ins, double Hk[]);
	void TUpdate(double dt, int opt = 1);
	void MUpadte(CSINS& ins,double ZK[],int zflag = 0xffff);
	void Feedback(CSINS& ins,double scater=1.0);
};

class GIKF_Calibrate_DualAnt
{
public:
	int ROW,COL;/*dsf90:ROW——NUMX;COL——NUMW*/
	double* xk;
	double* xk_std;
	double* Pxk;
	double* Phi;
	double* Qk;
	double* Gk;
	double* Hk;
	double* Rk;
	int  xflag;
	int  zflag;

	GIKF_Calibrate_DualAnt(int row=1,int col=1);
	GIKF_Calibrate_DualAnt& operator=(const GIKF_Calibrate_DualAnt& kftemp);
	
	void kffree();                                                //释放内存，销毁kf对象前，必须调用此函数
	void kfinit();
	void upPhi(CSINS& ins,double dt);
	void upHk(CSINS& ins, double Hk[]);
	void TUpdate(double dt, int opt = 1);
	void MUpadte(CSINS& ins,double ZK[],int zflag = 0xffff);
	void Feedback(CSINS& ins,double scater=1.0);
};

#ifdef __cplusplus
}
#endif

#endif
