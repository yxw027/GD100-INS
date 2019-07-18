/*
 * Author   : dsf90
 * Date     : 2018.12.15
 * Breif    : gilc out pack creat
 * Version  : 0.1
 * Hisotry  : first created
 * Form     : ANSI
 */
 
#ifdef WIN32
#include "stdafx.h"
#endif

#include <math.h>
#include <string.h>

#include "GILC.h"
//#include "rtklib.h"
#include "gnss_nmea_creat.h"
#include "gilc_out_pack.h"

/*add by dsf90,2018.4.16*/
int GILC_IOOut_CreatNmeaPack(unsigned char* buffer,gilc_raw_t *pstRaw,gilc_result_t *pstOut,int iMsgFlag)
{
	int len = 0;
	HC_DOUBLE ep[6] = {0.0};
	nmea_msg gnss_msg = { {0},{0}, };
	gtime_t gt;
	
	gt = gpst2time(pstOut->week, pstOut->second);
	time2epoch(gpst2utc(gt), ep);

	gnss_msg.utc.year  = (int)ep[0];
	gnss_msg.utc.month = (int)ep[1];
	gnss_msg.utc.date  = (int)ep[2];
	gnss_msg.utc.hour  = (int)ep[3];
	gnss_msg.utc.min   = (int)ep[4];
	gnss_msg.utc.sec   = (int)(ep[5] * 100.0);

	gnss_msg.gps_time.week= pstOut->week;
	gnss_msg.gps_time.sec = pstOut->second;
	
	gnss_msg.utc.time = (double)gnss_msg.utc.sec / 100 + gnss_msg.utc.min * 100 + gnss_msg.utc.hour * 10000;
	gnss_msg.nsused = gnss_msg.posslnum = (pstRaw->gnssdata.nsused > 0 ? pstRaw->gnssdata.nsused : 10);
	gnss_msg.snsum  = pstRaw->gnssdata.ns;
	gnss_msg.hdop   = pstRaw->gnssdata.hdop;
	gnss_msg.dgnss_timeout = pstRaw->gnssdata.age;

	gnss_msg.ins_msg.gyo[0] = pstOut->gyro[0];
	gnss_msg.ins_msg.gyo[1] = pstOut->gyro[1];
	gnss_msg.ins_msg.gyo[2] = pstOut->gyro[2];
	gnss_msg.ins_msg.acc[0] = pstOut->acc[0];
	gnss_msg.ins_msg.acc[1] = pstOut->acc[1];
	gnss_msg.ins_msg.acc[2] = pstOut->acc[2];

	gnss_msg.ins_msg.rpy[0] = pstOut->roll;
	gnss_msg.ins_msg.rpy[1] = pstOut->pitch;
	gnss_msg.ins_msg.rpy[2] = pstOut->yaw;

	gnss_msg.biseline = pstRaw->gnssdata.baseline;
	gnss_msg.nsused2  = pstRaw->gnssdata.nsused2;

	if (pstRaw->gnssdata.stat)
		gnss_msg.gpssta = pstRaw->gnssdata.stat;
	else if (pstOut->gilc_status >= GILC_RET__RST_INITING)
		gnss_msg.gpssta = 3;

	gnss_msg.ins_msg.gilcstatus = pstOut->gilc_status_chc;

	HC_UINT8 chc_warming = 0;
	if (!pstRaw->bODOavail)
	{
		chc_warming |= (1 << 1);
	}

	gnss_msg.ins_msg.warming    = chc_warming;
	
	gnss_msg.latitude = pstOut->lla[0];
	gnss_msg.longitude = pstOut->lla[1];
	gnss_msg.height = pstOut->lla[2];
	gnss_msg.speed  = pstOut->speed;
	gnss_msg.heading= pstOut->heading;
	gnss_msg.ve = pstOut->vel_enu[0];
	gnss_msg.vn = pstOut->vel_enu[1];
	gnss_msg.vd =-pstOut->vel_enu[2];

	len = 0;
	if(iMsgFlag & 0x0001)
		len += NMEA_GGA_Creat(&gnss_msg, buffer + len);
	if(iMsgFlag & 0x0002)
		len += NMEA_RMC_Creat(&gnss_msg, buffer + len);
	if(iMsgFlag & 0x0004)
		len += NMEA_CHC_Creat(&gnss_msg, buffer + len);
	
	return len;
}

int GILC_IOOut_CreatCascoPack(unsigned char* buffer,gilc_raw_t *pstRaw,gilc_result_t* pstOut)
{
	static int s_iGnssTimeMs_pre = 0;
	gilc_casco_t stGilcCasco;
	int len = sizeof(gilc_casco_t);

	memset(&stGilcCasco,0,sizeof(stGilcCasco));

	HC_DOUBLE ep_gps[6],pos[3],r[3],vel_ecef[3],vel_enu[3],vel_h,vel_v,pos_std,vel_std;
	gtime_t gt,utc;
	
	gt = gpst2time(pstRaw->gnssdata.week, pstRaw->gnssdata.second);
	//gt = gpst2time(pstOut->week, pstOut->second);
	utc= gpst2utc(gt);
	time2epoch(gt, ep_gps);
	
	pos[0] = pstRaw->gnssdata.lat*D2R;
	pos[1] = pstRaw->gnssdata.lon*D2R;
	pos[2] = pstRaw->gnssdata.alt;
	vel_ecef[0]=pstRaw->gnssdata.vx_ecef;
	vel_ecef[1]=pstRaw->gnssdata.vy_ecef;
	vel_ecef[2]=pstRaw->gnssdata.vz_ecef;
	pos2ecef(pos,r);
	ecef2enu(pos,vel_ecef,vel_enu);
	pos_std = sqrt(SQ(pstRaw->gnssdata.std_lat)+SQ(pstRaw->gnssdata.std_lon)+SQ(pstRaw->gnssdata.std_alt));
	vel_std = sqrt(SQ(pstRaw->gnssdata.std_vx_ecef)+SQ(pstRaw->gnssdata.std_vy_ecef)+SQ(pstRaw->gnssdata.std_vz_ecef));
	vel_h   = sqrt(SQ(vel_enu[0])+SQ(vel_enu[1]));
	vel_v   = vel_enu[2];
	HC_UINT8 ion_correct_type = 0,pos_type_casco = 0,sol_status_casco = 0,time_status_casco = 0,USW_casco = 0;
	ion_correct_type = (HC_UINT8)((pstRaw->gnssdata.ext_sol_status&0x0E)>>1);
	switch(pstRaw->gnssdata.pos_type)
	{
		case 16:
	//	case 20:
			pos_type_casco = 0;
			break;
		case 17:
			pos_type_casco = 1;
			break;
		case 18:/*SBAS*/
			pos_type_casco = 2;
			break;
		case 32:
		case 33:
		case 34:
			pos_type_casco = 5;
			break;
		case 48:
		case 49:
		case 50:
			pos_type_casco = 4;
			break;
		case 69:/*PPP*/
		case 73:
			pos_type_casco = 3;
			break;
		default:
			pos_type_casco = 6;
			break;
	}
	
	switch(pstRaw->gnssdata.sol_status)
	{
		case 0:
		//case 1:
			sol_status_casco = (HC_UINT8)pstRaw->gnssdata.sol_status;
			break;
		case 1:
			sol_status_casco = 1;
			break;
		case 6:
			sol_status_casco = 2;
			break;		
		default:
			sol_status_casco = 3;
			break;
	}
	
	switch(pstRaw->gnssdata.time_status)
	{
		case 20:
			time_status_casco = 0;
			break;	
		//case 60:
		case 80:
		case 100:
		case 120:
			time_status_casco = 1;
			break;	
		case 130:
			time_status_casco = 2;
			break;
		case 160:
		case 170:
		case 180:
			time_status_casco = 3;
			break;		
		default:
			sol_status_casco = 0;
			break;
	}
	if(pstOut->gilc_status<GILC_RET__RST_STABLE)
	{
		USW_casco |= (1<<CASCO_USW_BIT__INIT);
	}

	if(pstRaw->memsdate.temper < -40 || pstRaw->memsdate.temper > 85)
	{
		USW_casco |= (1<<CASCO_USW_BIT__TEMPER_DATA);
	}

	HC_UINT8 gnss_updata_falg = 0;
	if(((int)(pstRaw->gnssdata.second*1000))!= s_iGnssTimeMs_pre)
	{
		gnss_updata_falg = 0x3F;
	}
	else
	{
		gnss_updata_falg = 0x00;
	}
	if(pstRaw->bPPSavail)
	{
		gnss_updata_falg |= (1<<3);
	}
	s_iGnssTimeMs_pre = (int)(pstRaw->gnssdata.second * 1000);

	stGilcCasco.msg_head[0] = 0xAA;
	stGilcCasco.msg_head[1] = 0x55;
	stGilcCasco.msg_type    = 0x01;
	stGilcCasco.msg_flag    = 0x67;
	stGilcCasco.msg_len     = 192;

	stGilcCasco.data_second = (HC_UINT32)(pstOut->second*1e3);
	stGilcCasco.data_heading= (HC_UINT32)(pstOut->heading*1e6);
	stGilcCasco.data_pitch  = (HC_INT32)(pstOut->pitch*1e6);
	stGilcCasco.data_roll   = (HC_INT32)(pstOut->roll*1e6);
	stGilcCasco.data_lat    = (HC_INT64)(pstOut->lla[0]*1e9);
	stGilcCasco.data_lon    = (HC_INT64)(pstOut->lla[1]*1e9);
	stGilcCasco.data_alt    = (HC_INT32)(pstOut->lla[2]*1e3);
	stGilcCasco.data_ve     = (HC_INT32)(pstOut->vel_enu[0]*1e6);
	stGilcCasco.data_vn     = (HC_INT32)(pstOut->vel_enu[1]*1e6);
	stGilcCasco.data_vu     = (HC_INT32)(pstOut->vel_enu[2]*1e6);
	
	stGilcCasco.data_gyo[0] = (HC_INT32)(pstRaw->memsdate.gyro[0]*1e6);
	stGilcCasco.data_gyo[1] = (HC_INT32)(pstRaw->memsdate.gyro[1]*1e6);
	stGilcCasco.data_gyo[2] = (HC_INT32)(pstRaw->memsdate.gyro[2]*1e6);
	stGilcCasco.data_acc[0] = (HC_INT32)(pstRaw->memsdate.accel[0]*1e6);
	stGilcCasco.data_acc[1] = (HC_INT32)(pstRaw->memsdate.accel[1]*1e6);
	stGilcCasco.data_acc[2] = (HC_INT32)(pstRaw->memsdate.accel[2]*1e6);
	stGilcCasco.data_temper               = (HC_INT16)(pstRaw->memsdate.temper*1e2);
	stGilcCasco.data_USW                  = 0;  
	
	stGilcCasco.data_gnss_pos_type        = (HC_UINT8)pstRaw->gnssdata.pos_type;
	stGilcCasco.data_gnss_pos_ecef_xyz[0] = (HC_INT32)(r[0]*1e2);
	stGilcCasco.data_gnss_pos_ecef_xyz[1] = (HC_INT32)(r[1]*1e2);
	stGilcCasco.data_gnss_pos_ecef_xyz[2] = (HC_INT32)(r[2]*1e2);
	stGilcCasco.data_gnss_pos_std         = (HC_UINT16)(pos_std*1e2);
	stGilcCasco.data_gnss_vel_ecef_xyz[0] = (HC_INT32)(pstRaw->gnssdata.vx_ecef*1e6);
	stGilcCasco.data_gnss_vel_ecef_xyz[1] = (HC_INT32)(pstRaw->gnssdata.vy_ecef*1e6);
	stGilcCasco.data_gnss_vel_ecef_xyz[2] = (HC_INT32)(pstRaw->gnssdata.vz_ecef*1e6);
	stGilcCasco.data_gnss_vel_std         = (HC_UINT16)(vel_std*1e2);
	stGilcCasco.data_gnss_lat             = (HC_INT64)(pstRaw->gnssdata.lat*1e9);
	stGilcCasco.data_gnss_lon             = (HC_INT64)(pstRaw->gnssdata.lon*1e9);
	stGilcCasco.data_gnss_alt             = (HC_INT32)(pstRaw->gnssdata.alt*1e3);
	stGilcCasco.data_gnss_speed_h         = (HC_INT32)(vel_h*1e6);
	stGilcCasco.data_gnss_heading2        = (HC_INT32)(pstRaw->gnssdata.heading2*1e6);
	stGilcCasco.data_gnss_speed_v         = (HC_INT32)(vel_v*1e6);
	stGilcCasco.data_gnss_pos_type2       = (HC_UINT8)pstRaw->gnssdata.pos_type2;
	stGilcCasco.data_gnss_heading         = (HC_UINT16)(pstRaw->gnssdata.heading*1e2);
	stGilcCasco.data_gnss_nsused          = (HC_UINT8)pstRaw->gnssdata.nsused;
	stGilcCasco.data_gnss_info1           = (HC_UINT8)((ion_correct_type<<4)|pos_type_casco);
	stGilcCasco.data_gnss_info2           = (HC_UINT8)((pstRaw->gnssdata.gnss_used_flag<<4)|(time_status_casco<<2)|sol_status_casco);
	stGilcCasco.data_gnss_gdop            = (HC_UINT16)(pstRaw->gnssdata.gdop*1e3);
	stGilcCasco.data_gnss_pdop            = (HC_UINT16)(pstRaw->gnssdata.pdop*1e3);
	stGilcCasco.data_gnss_hdop            = (HC_UINT16)(pstRaw->gnssdata.hdop*1e3);
	stGilcCasco.data_gnss_vdop            = (HC_UINT16)(pstRaw->gnssdata.vdop*1e3);
	stGilcCasco.data_gnss_tdop            = (HC_UINT16)(pstRaw->gnssdata.tdop*1e3);
	stGilcCasco.data_gnss_age             = (HC_UINT16)(pstRaw->gnssdata.age*10);
	stGilcCasco.data_gnss_gps_hours       = (HC_UINT8)ep_gps[3];
	stGilcCasco.data_gnss_gps_min         = (HC_UINT8)ep_gps[4];
	stGilcCasco.data_gnss_gps_sec         = (HC_UINT8)ep_gps[5];
	stGilcCasco.data_gnss_gps_sec_decimal = (HC_UINT16)round(ep_gps[5]*1e3)%1000;
	stGilcCasco.data_gnss_gps_month       = (HC_UINT8)ep_gps[1];
	stGilcCasco.data_gnss_gps_day         = (HC_UINT8)ep_gps[2];
	stGilcCasco.data_gnss_gps_year        = (HC_UINT16)ep_gps[0];
	stGilcCasco.data_gnss_utc_sec_total   = (HC_INT64)utc.time;
	stGilcCasco.data_gnss_age_bestxyz     = (HC_UINT8)(pstRaw->gnss_delay_ms);
	stGilcCasco.data_gnss_age_vel         = (HC_INT16)(pstRaw->gnss_delay_ms);
	stGilcCasco.data_gnss_age_bestpos     = (HC_UINT8)(pstRaw->gnss_delay_ms);
	stGilcCasco.data_gnss_age_bestvel     = (HC_UINT8)(pstRaw->gnss_delay_ms);
	stGilcCasco.data_gnss_update_flag     = gnss_updata_falg;
	
	//HC_UINT8 data_casco[] = {0xAA,0x55,0x01,0x67,0xC0,0x00,0x44,0xEC,0x47,0x1C,0xEE,0x36,0x74,0x15,0x73,0xBB,0xFC,0xFF,0x52,0x36,0x01,0x00,0xE6,0x77,0xF6,0x45,0x09,0x00,0x00,0x00,0x57,0x7A,0x17,0x14,0x1B,0x00,0x00,0x00,0xAB,0xEF,0x00,0x00,0xF6,0x26,0x00,0x00,0xDC,0xC9,0xFF,0xFF,0xCC,0x02,0x00,0x00,0x0B,0x2E,0x00,0x00,0xDC,0xE5,0xFF,0xFF,0x2C,0xDB,0xFF,0xFF,0x02,0xFA,0xFF,0xFF,0x45,0xEF,0xFF,0xFF,0x1E,0x3F,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCE,0xC6,0xDE,0x08,0x00,0x00,0x10,0x04,0xBD,0x0B,0xF3,0xBF,0x97,0x35,0x1A,0x74,0x48,0x38,0x18,0x00,0x00,0xC2,0xD5,0xFF,0xFF,0x73,0x0C,0x00,0x00,0x55,0xDA,0xFF,0xFF,0x00,0x00,0x48,0x3C,0xF6,0x45,0x09,0x00,0x00,0x00,0x9F,0x6D,0x17,0x14,0x1B,0x00,0x00,0x00,0xEF,0xF2,0x00,0x00,0xF1,0x39,0x00,0x00,0x77,0xA0,0xB4,0x08,0xD2,0xFE,0xFF,0xFF,0x00,0x00,0x00,0x10,0x00,0x0C,0xD6,0x07,0xA6,0x06,0xD7,0x04,0x8F,0x04,0x25,0x04,0x00,0x00,0x0B,0x2F,0x25,0x4B,0x02,0x0B,0x1E,0xE2,0x07,0x59,0x23,0x01,0x5C,0x00,0x00,0x00,0x00,0x12,0x00,0x00,0x12,0x12,0x33,0xCB,0x40};
	//gilc_casco_t *pstGilcCasco = (gilc_casco_t *)data_casco;

	HC_UINT16 i = 0,sum = 0;
	HC_UINT8 *data_sp = (HC_UINT8 *)&stGilcCasco;
	for(i=2;i<len-2;i++)
		sum += *(data_sp+i);	
	stGilcCasco.check_sum  = sum; 
	//gilc_log_hex("stGilcCasco: ",50,data_sp,len);
	
	memcpy(buffer,&stGilcCasco,len);
	//gilc_log_hex("data       : ",50,pstOutPack->data,len);
	return len;
}

int NMEA_CHC_recv(char *buf,int len)
{
	static char msg_buf[1024] = {0};
	static char msg_head[10] = {0};
	static char msg_end[10] = {0};
	static int msg_data_len = 0;
	static int msg_head_len = 0;
	static int msg_end_len = 0;
	static int decode_stat = 0;
	int i = 0;
	for(i = 0; i<len; i++)
	{
		if(decode_stat == 0)
		{
			if(msg_head_len>=6)
			{
				memcpy(msg_head,msg_head+1,5);
				msg_head[5] = buf[i];
				if(!memcmp(msg_head,"$GPCHC",6))
				{
					memcpy(msg_buf,msg_head,6);
					msg_data_len = 6;
					decode_stat = 1;
				}
			}
			else
			{
				msg_head[msg_head_len] = buf[i];
				msg_head_len++;
			}
		}
		else if(decode_stat == 1)
		{
			if(msg_end_len>=2)
			{
				msg_buf[msg_data_len] = msg_end[0];
				msg_data_len++;
				if(msg_data_len > sizeof(msg_buf))
				{
					decode_stat = 0;
					printf("decode chc fail\r\n");
					continue;
				}
				memcpy(msg_end,msg_end+1,1);
				msg_end[1] = buf[i];
				if(!memcmp(msg_end,"\r\n",2))
				{
					memcpy(msg_buf + msg_data_len,msg_end,2);
					msg_data_len += 2;
					msg_buf[msg_data_len] = '\0';
					decode_stat = 2;
				}
			}
			else
			{
				msg_end[msg_end_len] = buf[i];
				msg_end_len++;
			}

		}

		if(decode_stat == 2)
		{
			printf("recv: %s",msg_buf);
			msg_data_len = 0;
			msg_head_len = 0;
			msg_end_len = 0;
			decode_stat = 0;
			return 0;
		}
	}
	return -1;
}
