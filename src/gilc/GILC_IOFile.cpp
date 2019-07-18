#ifdef WIN32
#include "stdafx.h"
#include <Windows.h>
#include <io.h>
#include <direct.h>
#endif
#ifdef __linux
#include <unistd.h>
#include <sys/stat.h>
#endif

#include "GILC_IOFile.h"

//#ifndef DEBUG_LEVEL
FILE *s_flog = NULL;
//#endif
/***************************  class CLCData  *********************************/
void CLCData::Init()
{
	int i;
	for (i=0;i<3;i++) 
	{
		vn[i]=pos[i]=acc[i]=gyo[i]=mag[i]=0.0;
	}
	undulation=0;
	lever=0;
	yaw_U62R=0;
	yaw_var_U62R=0;
	for(int i=0;i<36;i++)
	{
		GPV_RK[i]=0.0;
	}
    time=gpstimetarge=imutimetarge=0.0;
    num=stat=week=0;
	syst=calt=0.0;
	ns=0;
	hdop=0;
	age=0;
	heading2=0;
	bUpdate=false;
	bValid=false;
	bGPSavail=false;
	bPPSavail=false;
	bMEMSavail=false;
	bODOavail=false;
	bOnlyVelUp=false;
	gnss_speed=0.0;
}

void CLCData::Rest()
{
	for (int i=0;i<3;i++) 
	{
		//vn[i]=acc[i]=gyo[i]=mag[i]=0.0;
		acc[i]=gyo[i]=mag[i]=0.0;
	}
	undulation=0;
	//for(int i=0;i<36;i++)
	//{GPV_RK[i]=0.0;}
	yaw_var_U62R=0;
	//stat=0;
	bUpdate=false;
	bValid=false;
	bGPSavail=false;
	bPPSavail=false;
	bMEMSavail=false;
	bODOavail=false;
	bOnlyVelUp=false;
	gnss_speed=0.0;
}
/*
CLCData& CLCData::operator=(const CLCData& lcdata)
{
	time=lcdata.time;   // gga time
	gpstimetarge=lcdata.gpstimetarge;
	week=lcdata.week;
	stat=lcdata.stat;
	ns=lcdata.ns;
	heading2=lcdata.heading2;
	nsused=lcdata.nsused;
	snsum=lcdata.snsum;
	hdop=lcdata.hdop;
	m0=lcdata.m0;
	age=lcdata.age;
	num=lcdata.num;
	imutimetarge=lcdata.imutimetarge;
	syst=lcdata.syst;
	calt=lcdata.calt;
	lever=lcdata.lever;
	bUpdate=lcdata.bUpdate;
	bValid=lcdata.bValid;
	bGPSavail=lcdata.bGPSavail;
	bPPSavail=lcdata.bPPSavail;
	bOnlyVelUp=lcdata.bOnlyVelUp;
	for (int i=0;i<3;i++)
	{
		acc[i]=lcdata.acc[i];
		gyo[i]=lcdata.gyo[i];
		mag[i]=lcdata.mag[i];
		pos[i]=lcdata.pos[i];
		vn[i]=lcdata.vn[i];
	}
	undulation=lcdata.undulation;
	yaw_U62R=lcdata.yaw_U62R;
	yaw_var_U62R=lcdata.yaw_var_U62R;
	for(int i=0;i<6;i++)
	{
		ep[i]=lcdata.ep[i];
	}
	for(int i=0;i<36;i++)
	{
		GPV_RK[i]=lcdata.GPV_RK[i];
	}
	return *this;
}
*/
void CLCData::getHMS(double ggat)
{
	ep[0]=0;ep[1]=0;ep[2]=0;
	ep[3]=int(ggat/10000.0);
	ep[4]=int(fmod(ggat,10000.0)/100.0);
	ep[5]=fmod(ggat,100.0);
}

void CLCData::getPOS_rad( double lat,double lon,double hgt )
{
	double deg,min;

	deg=(int)(lat/100.0);
	min=fmod(lat,100.0);
	pos[0]=(deg+min/60.0)*glv_deg;

	deg=(int)(lon/100.0);
	min=fmod(lon,100.0);
	pos[1]=(deg+min/60.0)*glv_deg;

	pos[2]=hgt;
}
// 解算坐标系的定义是：航向为y,向上为z,向右为x
// 数据本身的顺序是：航向为x,向下为z，向右为y
void CLCData::imuConvert()
{
	int i;
	double a[3];

#ifdef P2CAR //BASE_ON_NX200 ADIS16445 ENU-右前上 100Hz
	for (i=0;i<3;i++) a[i]=acc[i];
	acc[0]=a[1]; acc[1]=-a[0]; acc[2]=-a[2];

	for (i=0;i<3;i++) a[i]=gyo[i];
	gyo[0]=-a[1]; gyo[1]=a[0]; gyo[2]=a[2];
#endif
}

/***************************  class IMUFile_hw  *********************************/
int DebugFile::Init(char *outFilePath,bool bOutFileSave,char *tmpFilePath, bool bTmpFileSave)
{
	char logfile[256],kfile[256],posLCfile[256],posGfile[256],posNmeafile[256];
	char rawTempFile[256],proTempFile[256];
	char timeLog[256];

	numins = 0;
	bInit = true;

	if(s_flog)
		fclose(s_flog);
	s_flog = NULL;
	foutkf = NULL;
	foutposLC = NULL;
	foutposG = NULL;
	foutins = NULL;
	foutposNmea = NULL;
	fraw = NULL;
	fpro = NULL;

#ifdef WIN32
#ifdef _DEBUG
	if (_access(outFilePath, 0) == -1)
		CreateDirectory(outFilePath, NULL);
	sprintf(timeLog, "%04d%02d%02d_",
		int(glv.ep[0]), int(glv.ep[1]), (int)glv.ep[2]);
	strcat(outFilePath, timeLog);
#endif
#elif defined(__linux)
	sprintf(timeLog, "%04d%02d%02d/",
		int(glv.ep[0]), int(glv.ep[1]), (int)glv.ep[2]);
	strcat(outFilePath, timeLog);

	gilc_log("Init outPath %s\r\n", outFilePath);
	char cmd_strn[512] = { 0 };
	if (access(outFilePath, 0) != 0)
	{
		sprintf(cmd_strn, "mkdir -p %s", outFilePath);
		system(cmd_strn);
	}
#endif
	sprintf(timeLog, "%02d%02d%02d_",
		int(glv.ep[3]), int(glv.ep[4]), (int)glv.ep[5]);
	strcat(outFilePath, timeLog);

	strcpy(logfile, outFilePath);   strcat(logfile, "log.txt");
	s_flog = fopen(logfile, "wt");
	if (!s_flog)
		printf("open file err! %s\r\n", logfile);
	
	strcpy(posGfile,outFilePath);  strcat(posGfile,"gps_pos.txt");
	foutposG = fopen(posGfile, "wt");
	if(!foutposG)
		gilc_log("open file err! %s\r\n",posGfile);

	if(bOutFileSave)
	{
		strcpy(posLCfile,outFilePath); strcat(posLCfile,"ins_pos.txt");
		strcpy(posNmeafile, outFilePath);strcat(posNmeafile, "ant_pos.nmea");
		foutposLC = fopen(posLCfile, "wt");
		if(!foutposLC)
			gilc_log("open file err! %s\r\n",posLCfile);
		foutposNmea = fopen(posNmeafile, "wt");	
		if(!foutposNmea)
			gilc_log("open file err! %s\r\n",posNmeafile);
	}
	
	if(bTmpFileSave)
	{
#ifdef WIN32
#ifdef _DEBUG
		if (_access(tmpFilePath,0)==-1) 
			CreateDirectory(tmpFilePath,NULL);
#endif
#elif defined(__linux)
		char cmd_strn[512] = { 0 };
		sprintf(timeLog,"%04d%02d%02d/",
				int(glv.ep[0]),int(glv.ep[1]),(int)glv.ep[2]);
		strcat(tmpFilePath, timeLog);
		if (access(tmpFilePath,0)!=0) 
		{
			sprintf(cmd_strn,"mkdir -p %s", tmpFilePath);
			system(cmd_strn);
		}
		sprintf(timeLog, "%02d%02d%02d_",
			int(glv.ep[3]), int(glv.ep[4]), (int)glv.ep[5]);
		strcat(tmpFilePath, timeLog);
#endif

		strcpy(rawTempFile,tmpFilePath);   strcat(rawTempFile,"rawdata.txt");
		strcpy(proTempFile, tmpFilePath);  strcat(proTempFile,"process.txt");
		strcpy(kfile, tmpFilePath);	   strcat(kfile, "ins_kf.txt");

		foutkf = fopen(kfile, "wt");
		if (!foutkf)
			gilc_log("open file err! %s\r\n", kfile);
		fraw = fopen(rawTempFile, "wt");
		if(!fraw)
			gilc_log("open file err! %s\r\n",rawTempFile);
		fpro = fopen(proTempFile, "wt");
		if(!fpro)
			gilc_log("open file err! %s\r\n",proTempFile);
	}	
	return 0;
}

int DebugFile::SaveRaw(CLCData *ilcd)
{
	printf_raw(fraw,ilcd);

	if (ilcd->bGPSavail)
	{
		printf_posG(foutposG, ilcd);
	}

	return 0;
}

int DebugFile::SaveRst(int stat,GIProcess *gipro,CLCData *ilcd)
{
	if(stat>=1 && ilcd->bMEMSavail)
	{
		numins++;
		if(numins == 10) /*10Hz*/
		{	
			printf_posLC(foutposLC,&gipro->ins,ilcd);
			printf_kf(foutkf,gipro->kf,ilcd->imutimetarge);
			numins=0;
		}
	}

	if (ilcd->bMEMSavail)
	{
		printf_prosess(fpro, gipro, ilcd);
	}
	return 0;
}

int DebugFile::Close(void)
{
	if(s_flog)
		fclose(s_flog);
	s_flog = NULL;
	if(foutkf)
		fclose(foutkf);
	foutkf = NULL;
	if(foutposLC)
		fclose(foutposLC);
	foutposLC = NULL;
	if(foutposG)
		fclose(foutposG);
	foutposG = NULL;
	if(foutins)
		fclose(foutins);
	foutins = NULL;
	if(foutposNmea)
		fclose(foutposNmea);
	foutposNmea = NULL;
	
	if(fraw)
		fclose(fraw);
	fraw = NULL;
	if(fpro)
		fclose(fpro);
	fpro = NULL;
	
	bInit = false;

	return 0;
}

void fprintf_log(char* msg)
{
	if (!s_flog) 
		return;
	fprintf(s_flog, msg);
}

void printf_imu(FILE* fp,CLCData *ilcd)
{
	if(!fp) return;
	fprintf(fp, "%8d%22.15f%22.15f%22.15f%22.15f%22.15f%22.15f%22.15f%22.15f%22.15f\n",
		ilcd->num, ilcd->gyo[0],ilcd->gyo[1],ilcd->gyo[2],ilcd->acc[0],ilcd->acc[1],ilcd->acc[2],
		ilcd->mag[0],ilcd->mag[1],ilcd->mag[2]);
}
void printf_nav(FILE *fp, CSINS *ins, int num)
{
	if(!fp) return;
	fprintf(fp, "%8d %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %15.8lf %15.8lf %10.3lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf\n",
		num, ins->att_car[0], ins->att_car[1], ins->att_car[2], ins->vm_car[0], ins->vm_car[1], ins->vm_car[2],
		ins->pos[0], ins->pos[1], ins->pos[2], ins->eb[0], ins->eb[1], ins->eb[2],
		ins->db[0], ins->db[1], ins->db[2]);
}
void printf_posLC(FILE *fp, CSINS *ins, CLCData *ilcd)
{
	if(!fp) return;
	double ep[6] = { 0 };
	gtime_t gt = gpst2time(ilcd->week, ilcd->imutimetarge-1); /*RTKlib plot 默认-17秒*/
	time2epoch(gt, ep);

	fprintf(fp, "%4d/%02d/%02d %02d:%02d:%06.3f %15.9f %15.9f %11.4f %4d %4d\n",
		int(ep[0]),int(ep[1]),int(ep[2]),int(ep[3]),int(ep[4]),ep[5],ins->pos[0]*R2D,ins->pos[1]*R2D,ins->pos[2],ilcd->stat,10);
}

void printf_posAnt(FILE *fp, CSINS *ins, CLCData *ilcd)
{
	if(!fp) return;
	double ep[6] = { 0 };
	gtime_t gt = gpst2time(ilcd->week, ilcd->imutimetarge-1);
	time2epoch(gt, ep);
	fprintf(fp, "%4d/%02d/%02d %02d:%02d:%06.3f %15.9f %15.9f %11.4f %4d %4d\n",
		int(ep[0]),int(ep[1]),int(ep[2]),int(ep[3]),int(ep[4]),ep[5],ins->posL[0]*R2D,ins->posL[1]*R2D,ins->posL[2],ilcd->stat,10);
}

void printf_posG(FILE *fp, CLCData *ilcd)
{
	if(!fp) return;
	double ep[6] = {0};
	gtime_t gt = gpst2time(ilcd->week, ilcd->gpstimetarge-1);
	time2epoch(gt, ep);

	fprintf(fp,"%4d/%02d/%02d %02d:%02d:%06.3f %15.9f %15.9f %11.4f %4d %4d\n",
		int(ep[0]),int(ep[1]),int(ep[2]),int(ep[3]),int(ep[4]),ep[5],ilcd->pos[0]*R2D,ilcd->pos[1]*R2D,ilcd->pos[2],ilcd->stat, (ilcd->ns>0?ilcd->ns:4));
}

void printf_kf(FILE *fp, GIKF& kf, int num)
{
	if(!fp) return;
	fprintf(fp, "%8d ", num);
	for(int k=0; k<kf.ROW; k++)	fprintf(fp, "%15.8lf ", sqrt(kf.Pxk[k*kf.ROW+k])); //状态权对角线元素
	fprintf(fp,"%s", "\n");
}
void printf_kf(FILE *fp, GIKF& kf, double gpstimetarget)
{
	if(!fp) return;
	fprintf(fp, "%f ", gpstimetarget);
	for(int k=0; k<kf.ROW; k++)	fprintf(fp, "%19.12lf ", sqrt(kf.Pxk[k*kf.ROW+k])); //状态权对角线元素
	fprintf(fp,"%s", "\n");
}
void printf_gga(FILE *fp, CSINS *ins, CLCData *ilcd)
{
	if(!fp) return;
	
	double B,L,H,deg,min;
	char time[30],lat[30],lon[30],hgt[10];

	sprintf(time,"%02.0f%02.0f%05.2f",ilcd->ep[3],ilcd->ep[4],ilcd->ep[5]);

	B=ins->posL[0]*R2D; 
	deg = (int)(B); min = (B - deg) * 60.0;
	sprintf(lat,"%02.0f%9.6f",deg, min);

	L=ins->posL[1]*R2D;
	deg = (int)(L); min = (L - deg) * 60.0;
	sprintf(lon,"%03.0f%9.6f",deg,min);

	H=ins->posL[2];
	sprintf(hgt,"%.3f",H);

	fprintf(fp,"$GPGGA,%s,%s,N,%s,E,%d,%d,%.1f,%s,M,%d,M,%.2f,*7B\n",
		time,lat,lon,ilcd->stat,ilcd->ns,ilcd->hdop,hgt,ilcd->m0,ilcd->age);
}

void printf_prosess(FILE *fp, GIProcess *gipro, CLCData *ilcd)
{
	if (!fp) return;
	double heading_ant = atan2(gipro->ins.vnL[0], gipro->ins.vnL[1])*R2D;
	if (heading_ant < 0) heading_ant += 360;
	double heading_gnss = atan2(ilcd->vn[0], ilcd->vn[1])*R2D;
	if (heading_gnss < 0) heading_gnss += 360;

	double gnss_speed = sqrt(pow(ilcd->vn[0], 2) + pow(ilcd->vn[1], 2));
	double ins_speed = gipro->ins.vm_car[1];
	double dUtcTime = (ilcd->ep[3] * 10000 + ilcd->ep[4] * 100 + ilcd->ep[5]);

	fprintf(fp, "%9.3f,%9.3f,%d,%d,%d,%.9f,%.9f,%.9f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.9f,%.9f,%.9f,%.3f,%.3f,%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f ",
		dUtcTime, ilcd->imutimetarge, 
		gipro->c, ilcd->nsused, ilcd->stat,
		gipro->ins.pos[0] * R2D, gipro->ins.pos[1] * R2D, gipro->ins.pos[2],
		gipro->ins.vn[0], gipro->ins.vn[1], gipro->ins.vn[2],
		gipro->ins.att_car[0] * R2D, gipro->ins.att_car[1] * R2D, gipro->ins.att_car[2] * R2D,
		ilcd->pos[0] * R2D, ilcd->pos[1] * R2D, ilcd->pos[2],
		ilcd->vn[0], ilcd->vn[1], ilcd->vn[2],
		ilcd->acc[0], ilcd->acc[1], ilcd->acc[2],
		ilcd->gyo[0], ilcd->gyo[1], ilcd->gyo[2]
	);
	fprintf(fp, "%.6f %.6f %.6f %.6f %.6f %.6f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.3f %.3f %d %d %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.6f\n",
		ilcd->acc_iir[0], ilcd->acc_iir[1], ilcd->acc_iir[2],
		ilcd->gyo_iir[0], ilcd->gyo_iir[1], ilcd->gyo_iir[2],
		gipro->ins.eb[0], gipro->ins.eb[1], gipro->ins.eb[2],
		gipro->ins.db[0], gipro->ins.db[1], gipro->ins.db[2],
		gipro->ins.PRY_Install[0]*R2D,
		gipro->ins.PRY_Install[1]*R2D,
		gipro->ins.PRY_Install[2]*R2D,
		gipro->ins.lever[0],
		gipro->ins.lever[1],
		gipro->ins.lever[2],
		gnss_speed,ins_speed,
		gipro->bStatic,gipro->iDriverMode,
		heading_gnss,heading_ant,ilcd->heading2,
		gipro->ins.wheel_heading_h*gipro->ins.Kwh + gipro->ins.Bwh, ilcd->stOdoData.wheel_heading,
		gipro->ins.wheel_heading_raw, gipro->ins.Kd*ilcd->stOdoData.wheel_vel,gipro->ins.yaw_rate_byodo
	);
}

void printf_raw(FILE *fp,CLCData *ilcd)
{
	if(!fp) return;
	
	fprintf(fp,"%.3f,%.3f,%12.8f,%12.8f,%12.8f,%12.8f,%12.8f,%12.8f,%.6f,%d,%.9f,%.9f,%.3f,%.3f,%.3f,%.3f,%.20f,%.20f,%.9f,%.9f,%.9f,%.9f,%.9f,%d,%d,%f,%.2f,%f,%d,%d\n",
		(ilcd->ep[3]*10000+ilcd->ep[4]*100+ilcd->ep[5]),ilcd->imutimetarge, 
		ilcd->gyo[0], ilcd->gyo[1], ilcd->gyo[2], 
		ilcd->acc[0], ilcd->acc[1], ilcd->acc[2],
		ilcd->gpstimetarge, ilcd->week, 
		ilcd->pos[0], ilcd->pos[1], ilcd->pos[2], 
		ilcd->vn[0], ilcd->vn[1], ilcd->vn[2],
		ilcd->GPV_RK[3 * 6 + 3]*glv.Re*glv.Re, ilcd->GPV_RK[4 * 6 + 4]*glv.Re*glv.Re, ilcd->GPV_RK[5 * 6 + 5], 
		ilcd->GPV_RK[0 * 6 + 0], ilcd->GPV_RK[1 * 6 + 1], ilcd->GPV_RK[2 * 6 + 2], ilcd->GA_RK[2],
		ilcd->stat, ilcd->ns, ilcd->hdop, 
		ilcd->age, ilcd->lever,
		ilcd->bPPSavail,ilcd->bGPSavail);
}

/*endif*/

