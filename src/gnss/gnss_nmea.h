#ifndef __GNSS_NMEA_H__
#define __GNSS_NMEA_H__

#include <hc_type.h>

#ifdef __cplusplus
extern "C" {
#endif				/*__cplusplus*/

////////////////////////////////////////////////////////////////////////////////// 	   


//GPS NMEA-0183协议重要参数结构体定义 
//卫星信息
typedef struct  
{										    
 	HC_UINT8 num;		//卫星编号
	HC_UINT8 eledeg;	//卫星仰角
	HC_UINT16 azideg;	//卫星方位角
	HC_UINT8 sn;		//信噪比		   
}nmea_slmsg;  
//UTC时间信息
typedef struct  
{										    
	HC_DOUBLE time; //utctime
	HC_UINT16 year;	//年份
	HC_UINT8 month;	//月份
	HC_UINT8 date;	//日期
	HC_UINT8 hour; 	//小时
	HC_UINT8 min; 	//分钟
	HC_UINT32 sec; 	//秒钟100倍,实际除以100.	
	//31-25: Year(0-127 org.1980), 24-21: Month(1-12), 20-16: Day(1-31) */                                                                                                                                                                                                                                      
	//15-11: Hour(0-23), 10-5: Minute(0-59), 4-0: Second(0-29 *2) */                                                                                                                                                                                                                                                
	HC_UINT32 fattime; /*文件系统时间格式*/
}nmea_utc_time; 

typedef struct
{
	HC_DOUBLE sec; //周内秒
	HC_UINT16 week;	//GPS-周
}nmea_gps_time;

typedef struct  
{										    
	HC_UINT8 insused_flag;
	HC_UINT8 ins_out_type;
	HC_UINT8 ins_status;
	HC_DOUBLE rpy[3];
	HC_DOUBLE gyo[3];/*deg/s*/
	HC_DOUBLE acc[3];/*g*/
	HC_UINT8 gilcstatus;
	HC_UINT8 warming;
}nmea_ins_msg;   	   

typedef struct  
{										    
	HC_UINT8 chip_id[16];
	HC_UINT16 chip_type; /*407*/
}nmea_sys_msg;   	   

//NMEA 0183 
typedef struct  
{		
	/*GGA msg*/
	nmea_utc_time utc;			//UTC时间
	nmea_gps_time gps_time;
	HC_DOUBLE latitude;				//纬度 
	HC_UINT8 nshemi;					//北纬/南纬,N:北纬;S:南纬				  
	HC_DOUBLE longitude;			    //经度 
	HC_UINT8 ewhemi;					//东经/西经,E:东经;W:西经
	HC_UINT8 gpssta;					//GPS状态:0,未定位;1,单点;2,RTD;4,RTK;5,浮点;6,正在估算.				  
 	HC_UINT8 posslnum;				//用于定位的卫星数,0~16.
	HC_DOUBLE hdop;					//水平精度因子 
	HC_DOUBLE altitude;			 	      //海拔高度,	 
	HC_DOUBLE height_wgs84;		      //大地水准面高度 
	HC_DOUBLE dgnss_timeout;		      //差分延时	 
	HC_UINT16 station_id;		              //基站编号.	 
	
 	HC_UINT8 possl[32];				//用于定位的卫星编号
	HC_UINT8 fixmode;					//定位类型:1,没有定位;2,2D定位;3,3D定位
	HC_DOUBLE pdop;					//位置精度因子
	HC_DOUBLE vdop;					//垂直精度因子

	HC_DOUBLE height;                //高度altitude +  height_wgs84
	HC_DOUBLE biseline;
	HC_UINT8 nsused;				//使用星数
	HC_UINT8 nsused2;				//使用星数
	HC_UINT8 svnum;					//可见卫星数
 	HC_UINT16 snsum;                                 //所有卫星信噪比之和
	nmea_slmsg slmsg[32];		                //最多32颗卫星

	HC_DOUBLE speed;					//地面速率 单位:1公里/小时
	HC_DOUBLE ve;                        //东向速度
	HC_DOUBLE vn;                        //北向速度
	HC_DOUBLE vd;                        //地向速度
	
	HC_DOUBLE heading;					
	HC_DOUBLE heading2;					
	HC_UINT32 count;					

	HC_DOUBLE sigma_lat;                /*方差*/
	HC_DOUBLE sigma_lon;
	HC_DOUBLE sigma_alt;
	HC_DOUBLE sigma_ve;
	HC_DOUBLE sigma_vn;
	HC_DOUBLE sigma_vd;

	nmea_ins_msg ins_msg;
	nmea_sys_msg sys_msg;
}nmea_msg; 

void GPS_Analysis(HC_UINT8 *buf,HC_UINT16 len,nmea_msg *gpsx);
void NMEA_GSV_Analysis(nmea_msg *gpsx,HC_UINT8 *buf);
void NMEA_GGA_Analysis(nmea_msg *gpsx,HC_UINT8 *buf);
void NMEA_GSA_Analysis(nmea_msg *gpsx,HC_UINT8 *buf);
void NMEA_RMC_Analysis(nmea_msg *gpsx,HC_UINT8 *buf);
void NMEA_VTG_Analysis(nmea_msg *gpsx,HC_UINT8 *buf);


#ifdef __cplusplus
}
#endif			/*__cplusplus*/

#endif			/*__GNSS_NMEA_H__*/

