#include <string.h>
#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <math.h>
#include <gnss_nmea_creat.h>

#if defined(SYS_STM32)
#include "user_sys_cfg.h"
#include "sys.h"
#else
#define SOFTWARE_DATE "20171212"
#define HARDWARE_TYPE "407"
#define sys_get_chip_ID(a) {}
#endif
	
extern HC_INT8 NMEA_Check(HC_VOID *dataBuf,HC_UINT32 len);
	
static HC_INT8 s_nshemi = 'N';
static HC_INT8 s_ewhemi = 'E';

HC_UINT32 NMEA_GGA_Creat(nmea_msg *gpsx,HC_UINT8 *buf)
{
	HC_INT8 cSum = 0;
	HC_INT32 temp;
	HC_DOUBLE fLatitude,fLongitude;
		
	temp  = (HC_INT32)gpsx->latitude;
	fLatitude = (gpsx->latitude-temp)*60.0 + temp*100;
	
	temp  = (HC_INT32)gpsx->longitude;
	fLongitude = (gpsx->longitude-temp)*60.0 + temp*100;

	if(gpsx->nshemi == 'N'||gpsx->nshemi == 'S')
	{
		s_nshemi = gpsx->nshemi;
	}
	
	if(gpsx->ewhemi == 'E'||gpsx->ewhemi == 'W')
	{
		s_ewhemi = gpsx->ewhemi;
	}

	gpsx->altitude   = gpsx->height - gpsx->height_wgs84 ;
	
	if(gpsx->latitude == 0 || gpsx->longitude == 0)
	{
		sprintf((char*)buf,"$GPGGA,%02d%02d%02d.%02d,,,,,0,00,99.99,,,,,,",
			         gpsx->utc.hour,gpsx->utc.min,
			         gpsx->utc.sec/100,gpsx->utc.sec%100
			         );
		
	}
	else if(gpsx->gpssta==2||gpsx->gpssta==4||gpsx->gpssta==5)
	{
		sprintf((char*)buf,"$GPGGA,%02d%02d%02d.%02d,%.8f,%c,%.8f,%c,%d,%d,%.2f,%.3f,M,%.3f,M,%.1f,%04d",
			         gpsx->utc.hour,gpsx->utc.min,
			         gpsx->utc.sec/100,gpsx->utc.sec%100,
			         fLatitude,s_nshemi,
			         fLongitude,s_ewhemi,
			         gpsx->gpssta,gpsx->posslnum,
			         gpsx->hdop,
			         gpsx->altitude,gpsx->height_wgs84,
			         gpsx->dgnss_timeout,gpsx->station_id
			         );
	}
	else
	{
		sprintf((char*)buf,"$GPGGA,%02d%02d%02d.%02d,%.8f,%c,%.8f,%c,%d,%d,%.2f,%.3f,M,%.3f,M,,0000",
			         gpsx->utc.hour,gpsx->utc.min,
			         gpsx->utc.sec/100,gpsx->utc.sec%100,
			         fLatitude,s_nshemi,
			         fLongitude,s_ewhemi,
			         gpsx->gpssta,gpsx->posslnum,
			         gpsx->hdop,
			         gpsx->altitude,gpsx->height_wgs84
			         );
	}
	cSum = NMEA_Check(buf+1,strlen((char*)buf)-1);
	sprintf((char*)buf+strlen((char*)buf),"*%02X\r\n",cSum);
	logt("creat GGA:\r\n%s",buf);
	return strlen((char*)buf);
}

//分析GPRMC信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
HC_UINT32 NMEA_RMC_Creat(nmea_msg *gpsx,HC_UINT8 *buf)
{
	HC_INT8 cSum = 0;
	HC_DOUBLE fSpeed;
	HC_INT32 temp;
	HC_DOUBLE fLatitude,fLongitude;
		
	temp  = (HC_INT32)gpsx->latitude;
	fLatitude = (gpsx->latitude-temp)*60.0 + temp*100;
	
	temp  = (HC_INT32)gpsx->longitude;
	fLongitude = (gpsx->longitude-temp)*60.0 + temp*100;
	
	if(gpsx->nshemi == 'N'||gpsx->nshemi == 'S')
	{
		s_nshemi = gpsx->nshemi;
	}
	
	if(gpsx->ewhemi == 'E'||gpsx->ewhemi == 'W')
	{
		s_ewhemi = gpsx->ewhemi;
	}
	
	fSpeed = gpsx->speed*3.6f/1.852f;

	if(gpsx->latitude == 0 || gpsx->longitude == 0)
	{
		sprintf((char*)buf,"$GPRMC,%02d%02d%02d.%02d,V,,,,,,,%02d%02d%02d,,,N",
			         gpsx->utc.hour,gpsx->utc.min,
			         gpsx->utc.sec/100,gpsx->utc.sec%100,
			         gpsx->utc.date, gpsx->utc.month, gpsx->utc.year%100	    
			         );
	}
	else
	{
		sprintf((char*)buf,"$GPRMC,%02d%02d%02d.%02d,A,%.8f,%c,%.8f,%c,%.3f,%.2f,%02d%02d%02d,,,N",
			         gpsx->utc.hour,gpsx->utc.min,
			         gpsx->utc.sec/100,gpsx->utc.sec%100,
			         fLatitude,s_nshemi,
			         fLongitude,s_ewhemi,
			         fSpeed,gpsx->heading,
			         gpsx->utc.date, gpsx->utc.month, gpsx->utc.year%100    
			         );
	}
	cSum = NMEA_Check(buf+1,strlen((char*)buf)-1);
	sprintf((char*)buf+strlen((char*)buf),"*%02X\r\n",cSum);
	logt("creat RMC:\r\n%s",buf);
	return strlen((char*)buf);
}

/*$GPATT,-0.01,p,-0.00,r,0.000,y,20161216,s,002A00415106333539343233,ID,1,INS,411,02,1*4D*/
HC_UINT32 NMEA_ATT_Creat(nmea_msg *gpsx,HC_UINT8 *buf)
{
	static HC_UINT32 chip_id[4] = {0};
	HC_INT8 cSum = 0;
	//HC_UINT32 temp,temp2;
	//HC_DOUBLE fLatitude,fLongitude;
	//HC_DOUBLE fSpeed;

	if(chip_id[3]==0)
	{
		chip_id[3] = 1;
		sys_get_chip_ID(chip_id);
	}
	
	sprintf((char*)buf,"$GPATT,%.3f,p,%.3f,r,%.3f,y,%s,s,%08X%08X,ID,%d,INS,%s,%d,%d",
		         gpsx->ins_msg.rpy[1], 
		         gpsx->ins_msg.rpy[0], 
		         gpsx->ins_msg.rpy[2],
		         SOFTWARE_DATE, 
		         chip_id[0],
		         chip_id[2],
		         gpsx->ins_msg.insused_flag,
		         HARDWARE_TYPE,
		         gpsx->ins_msg.ins_status,
		         gpsx->ins_msg.ins_out_type
		         );
	
	cSum = NMEA_Check(buf+1,strlen((char*)buf)-1);
	sprintf((char*)buf+strlen((char*)buf),"*%02X\r\n",cSum);
	logt("creat ATT:\r\n%s",buf);
	return strlen((char*)buf);
}

HC_UINT32 NMEA_CHC_Creat(nmea_msg *gpsx,HC_UINT8 *buf)
{
	HC_INT8 cSum = 0,age;

	gpsx->altitude   = gpsx->height - gpsx->height_wgs84 ;
	
	age = (HC_INT8)round(gpsx->dgnss_timeout);
	sprintf((char*)buf,"$GPCHC,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.7f,%.7f,%.2f,%.3f,%.3f,%.3f,%.3f,%d,%d,%02X,%d,%d",
					gpsx->gps_time.week, 
					gpsx->gps_time.sec, 
					gpsx->heading, 
					gpsx->ins_msg.rpy[1], 
					gpsx->ins_msg.rpy[0],
					gpsx->ins_msg.gyo[0], 
					gpsx->ins_msg.gyo[1],
					gpsx->ins_msg.gyo[2], 
					gpsx->ins_msg.acc[0], 
					gpsx->ins_msg.acc[1],
					gpsx->ins_msg.acc[2], 
					gpsx->latitude,
					gpsx->longitude,
					gpsx->altitude,
					gpsx->ve,
					gpsx->vn,
				   -gpsx->vd,
					gpsx->speed,
					gpsx->nsused,
					gpsx->nsused2,
					gpsx->ins_msg.gilcstatus,
					age,
					gpsx->ins_msg.warming
		         );
	cSum = NMEA_Check(buf+1,strlen((char*)buf)-1);
	sprintf((char*)buf+strlen((char*)buf),"*%02X\r\n",cSum);
	logt("creat CHC:\r\n%s",buf);
	return strlen((char*)buf);
}

//提取NMEA-0183信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
HC_UINT32 NMEA_Creat(nmea_msg *gpsx,HC_UINT8 *buf)
{
	buf[0] = 0;
	NMEA_GGA_Creat(gpsx,buf+strlen((char*)buf));	
	NMEA_RMC_Creat(gpsx,buf+strlen((char*)buf));
	NMEA_ATT_Creat(gpsx,buf+strlen((char*)buf));
	return strlen((char*)buf);
}

