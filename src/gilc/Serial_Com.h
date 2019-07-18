#pragma once
#if WIN32
#include<iostream>
#include<windows.h>
#include<time.h>
#include<stdlib.h>
using namespace std;
int win_reloy(char buf[]);
#endif
#if __linux
#include<stdio.h>   
#include<fcntl.h>  
#include<unistd.h>  
#include<errno.h>  
#include<termios.h>  
#include<sys/types.h>  
#include<sys/stat.h>
int linux_reloy(char buf[]);
#endif
struct GINS_DATA {//42
	int16_t roll;  //2  degree*100
	int16_t pitch; //2  degree*100
	int16_t yaw;   //2  degree*100
	int16_t none;
	float gyro[3]; //12 rad
	float accl[3]; //12 m/s/s
	int32_t lng;   //4  degree*10**7
	int32_t lat;   //4  degree*10**7
	int16_t vel_x; //2  m/s*1000
	int16_t vel_y; //2  m/s*1000
};