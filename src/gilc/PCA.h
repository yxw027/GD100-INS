#ifndef _PCA_H
#define _PCA_H


#include "stdafx.h"
#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <vector>
#include "GILC.h"
#define pca_N  5                       //pca滤波数据窗口数
#define car_yaw_win  30                //加速过程计算航行角个数
#define WinLen      5                // 加速度平滑滤波开窗平滑的队列大小
#define acc_WinLen      50            // 加速度原始数据开窗平滑的队列大小
#define pca_GN 9.80665
#define pca_wie 0.0000729211514 //rad/s
class PCA_Filter
{
public:
		bool mat_init;
		bool pca_falg;
		vector<double> va_p;
		vector<double> va_r;
		vector<double> D_angle;
		vector<double> acc_yaw;
		vector<double> acc_x;
		vector<double> acc_y;
		vector<double> acc_z;
		vector<double> a_car;
		vector<double> car_yaw;
		vector<double> pca_pitch;
		vector<double> pca_roll;
		double Mean_data[3];
		double MAX_index;
		double Min_index;
		double Max_data;
		double Min_data[3];
		double Min_s_data[3];
		double Min_t_data[3];
		double Min_f_data[3];
		double Sum_data[2];
		double New_data[2];
		double New_angle[3];
		double Car_Angle[3];

public:
	void pca_init(void);
	int Pca_Process_Angle(double Acc[3], double result[3],int car_status);//输出pitch、roll、yaw
};

//int Pca_Process_yaw(double angle[2], double result[2]);
int AccFilter(double acc[3], double result[2]);
//int Pca_Process_struct(pca_struct_t *pca, double angle, double *result);
int acc_yaw_correct(double angle, double last_angle, double result);
double dataFilter(double acc_win[], int count);
void Comp_InstallErr_Acc(double acc[3], double installroll, double installpitch);
int InitAngle_Err(double acc[3], double gyro[3], double pos[3], double Init_angle[3]);
int InitAngle_Err1(double acc[3], double gyro[3], double pos[3], double Init_angle[3]);
extern int acc_mean(double acc[3], double result[3]);
#endif