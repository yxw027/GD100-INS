/*
 * Author   : dsf90
 * Date     : 2018.12.15
 * Breif    : gilc out pack creat
 * Version  : 0.1
 * Hisotry  : first created
 * Form     : ANSI
 */
 
 #ifndef _GILC_IOOUT_H
#define _GILC_IOOUT_H

#include "hc_type.h"
#include "GILC_Vehicle_lib.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum casco_usw_bit {
	CASCO_USW_BIT__INIT = 0,
	CASCO_USW_BIT__INS_PARAM,
	CASCO_USW_BIT__GYO_DETECT,
	CASCO_USW_BIT__ACC_DETECT,
	CASCO_USW_BIT__MAG_DETECT,
	CASCO_USW_BIT__ELECTRON,
	CASCO_USW_BIT__GNSS,
	CASCO_USW_BIT__VG3D,
	CASCO_USW_BIT__POWER_LOW,
	CASCO_USW_BIT__POWER_HIGH,
	CASCO_USW_BIT__GYO_DATA_X,
	CASCO_USW_BIT__GYO_DATA_Y,
	CASCO_USW_BIT__GYO_DATA_Z,
	CASCO_USW_BIT__MAG_DATA,
	CASCO_USW_BIT__TEMPER_DATA,
	CASCO_USW_BIT__VG3D_CAL,
}casco_usw_bit_e;

#pragma pack (1)
typedef struct gilc_casco_package {
	HC_UINT8  msg_head[2];    //0xAA 0X55
	HC_UINT8  msg_type; 
	HC_UINT8  msg_flag;
	HC_UINT16 msg_len;
	
	HC_UINT32 data_second;//sec*10^3
	HC_UINT32 data_heading;//deg*10^6
	HC_INT32  data_pitch;
	HC_INT32  data_roll;
	HC_INT64  data_lat;//deg*10^9
	HC_INT64  data_lon;
	HC_INT32  data_alt;//m*10^3
	HC_INT32  data_ve;//m/s*10^6
	HC_INT32  data_vn;
	HC_INT32  data_vu;
	HC_INT32  data_gyo[3];//deg/s*10^6
	HC_INT32  data_acc[3];//m/s^2*10^6
	HC_INT16  data_mag[3];//nT/10
	HC_UINT16 data_pbar;//Pa/2
	HC_INT16  data_temper;//deg*10^2
	HC_UINT16 data_USW;//
	HC_UINT8  data_gnss_pos_type;
	HC_INT32  data_gnss_pos_ecef_xyz[3];//m*10^2
	HC_UINT16 data_gnss_pos_std;//m*10^2
	HC_INT32  data_gnss_vel_ecef_xyz[3];//m/s*10^6	
	HC_UINT16 data_gnss_vel_std;//m/s*10^2
	HC_INT64  data_gnss_lat;//deg*10^9
	HC_INT64  data_gnss_lon;
	HC_INT32  data_gnss_alt;//m*10^3	
	HC_INT32  data_gnss_speed_h;//m/s*10^6	
	HC_INT32  data_gnss_heading2;//deg*10^6	
	HC_INT32  data_gnss_speed_v;//m/s*10^6	
	HC_UINT8  data_gnss_pos_type2;
	HC_UINT16 data_gnss_heading;//deg*10^2	
	HC_UINT8  data_gnss_nsused;	
	HC_UINT8  data_gnss_info1;	
	HC_UINT8  data_gnss_info2;	
	HC_UINT16 data_gnss_gdop;//m*10^3	
	HC_UINT16 data_gnss_pdop;	
	HC_UINT16 data_gnss_hdop;	
	HC_UINT16 data_gnss_vdop;	
	HC_UINT16 data_gnss_tdop;	
	HC_UINT16 data_gnss_age;//s*10	
	HC_UINT8  data_gnss_gps_hours;	
	HC_UINT8  data_gnss_gps_min;	
	HC_UINT8  data_gnss_gps_sec;	
	HC_UINT16 data_gnss_gps_sec_decimal;//s*10^3
	HC_UINT8  data_gnss_gps_month;	
	HC_UINT8  data_gnss_gps_day;	
	HC_UINT16 data_gnss_gps_year;	
	HC_INT64  data_gnss_utc_sec_total;//s
	HC_UINT8  data_gnss_age_bestxyz;//ms	
	HC_INT16  data_gnss_age_vel;//ms	
	HC_UINT8  data_gnss_age_bestpos;//ms	
	HC_UINT8  data_gnss_age_bestvel;//ms	
	HC_UINT8  data_gnss_update_flag;	
	
	HC_UINT16 check_sum;
}gilc_casco_t;
#pragma pack ()

int GILC_IOOut_CreatNmeaPack(unsigned char* buffer,gilc_raw_t *pstRaw,gilc_result_t *pstOut,int iMsgFlag);
int GILC_IOOut_CreatCascoPack(unsigned char* buffer,gilc_raw_t *pstRaw,gilc_result_t* pstOut);
int NMEA_CHC_recv(char *buf,int len);

#ifdef __cplusplus
}
#endif

#endif
