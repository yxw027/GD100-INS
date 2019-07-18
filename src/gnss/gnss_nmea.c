#include <string.h>
#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <gnss_nmea.h>
/*----------------------------------------------
输入：
dataBuf  数据地址
len      数据长度
返回：
1字节异或结果
----------------------------------------------*/
HC_INT8 NMEA_Check(HC_VOID *dataBuf,HC_UINT32 len)
{
	HC_INT8 *data = (HC_INT8 *)dataBuf;
	HC_INT8 sum;
	HC_UINT32 i;
	if(!data||!len)
		return 0;
	sum = *data;
	for(i=1;i<len;i++)
		sum ^= *(data+i);
	return sum;
}

//从buf里面得到第cx个逗号所在的位置
//返回值:0~0XFE,代表逗号所在位置的偏移.
//       0XFF,代表不存在第cx个逗号							  
HC_UINT8 NMEA_Comma_Pos(HC_UINT8 *buf,HC_UINT8 cx)
{	 		    
	HC_UINT8 *p=buf;
	while(cx)
	{		 
		if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//遇到'*'或者非法字符,则不存在第cx个逗号
		if(*buf==',')cx--;
		buf++;
	}
	if(*buf==',') /*dsf90:两个','相邻，内容为空*/
		return 0XFF; 
	return buf-p;	 
}
//m^n函数
//返回值:m^n次方.
HC_UINT32 NMEA_Pow(HC_UINT8 m,HC_UINT8 n)
{
	HC_UINT32 result=1;	 
	while(n--)result*=m;    
	return result;
}
//str转换为数字,以','或者'*'结束
//buf:数字存储区
//dx:小数点位数,返回给调用函数
//返回值:转换后的数值
int NMEA_Str2num(HC_UINT8 *buf,HC_UINT8*dx)
{
	HC_UINT8 *p=buf;
	HC_UINT32 ires=0,fres=0;
	HC_UINT8 ilen=0,flen=0,i;
	HC_UINT8 mask=0;
	int res;
	while(1) //得到整数和小数的长度
	{
		if(*p=='-'){mask|=0X02;p++;}//是负数
		if(*p==','||(*p=='*'))break;//遇到结束了
		if(*p=='.'){mask|=0X01;p++;}//遇到小数点了
		else if(*p>'9'||(*p<'0'))	//有非法字符
		{	
			ilen=0;
			flen=0;
			break;
		}	
		if(mask&0X01)flen++;
		else ilen++;
		p++;
	}
	if(mask&0X02)buf++;	//去掉负号
	for(i=0;i<ilen;i++)	//得到整数部分数据
	{  
		ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>5)flen=5;	//最多取5位小数
	*dx=flen;	 		//小数点位数
	for(i=0;i<flen;i++)	//得到小数部分数据
	{  
		fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
	} 
	res=ires*NMEA_Pow(10,flen)+fres;
	if(mask&0X02)res=-res;		   
	return res;
}	  							 
//分析GPGSV信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GSV_Analysis(nmea_msg *gpsx,HC_UINT8 *buf)
{
	HC_UINT8 *p,*p1,dx;
	HC_UINT8 len,i,j,slx=0;
	HC_UINT8 posx; 
	
	gpsx->svnum = 0;
	gpsx->snsum = 0;
	p=buf;
	
	while(1)
	{	
		//p1=(u8*)strstr((const char *)p,"$GPGSV");
		//len=p1[7]-'0';								//得到GPGSV的条数
		p1=(HC_UINT8 *)strstr((const char *)p,"GSV");
		if(!p1) 	return;
		
		len=p1[4]-'0';								//得到GPGSV的条数
		//posx=NMEA_Comma_Pos(p1,3); 					//得到可见卫星总数
		//if(posx!=0XFF) gpsx->svnum +=NMEA_Str2num(p1+posx,&dx);
		for(i=0;i<len;i++)
		{	 
			//p1=(HC_UINT8*)strstr((const char *)p,"$GPGSV");  
			p1=(HC_UINT8*)strstr((const char *)p,"GSV");  
			if (!p1)
				return;
			for(j=0;j<4;j++)
			{	  
				posx=NMEA_Comma_Pos(p1,4+j*4);
				if(posx!=0XFF)gpsx->slmsg[slx].num=NMEA_Str2num(p1+posx,&dx);	//得到卫星编号
				else break; 
				posx=NMEA_Comma_Pos(p1,5+j*4);
				if(posx!=0XFF)gpsx->slmsg[slx].eledeg=NMEA_Str2num(p1+posx,&dx);//得到卫星仰角 
				else break;
				posx=NMEA_Comma_Pos(p1,6+j*4);
				if(posx!=0XFF)gpsx->slmsg[slx].azideg=NMEA_Str2num(p1+posx,&dx);//得到卫星方位角
				else break; 
				posx=NMEA_Comma_Pos(p1,7+j*4);
				if(posx!=0XFF)gpsx->slmsg[slx].sn=NMEA_Str2num(p1+posx,&dx);	//得到卫星信噪比
				else break;
				if(gpsx->slmsg[slx].sn>0) 										//信噪比有效
				{
					gpsx->svnum ++;
					gpsx->snsum +=gpsx->slmsg[slx].sn;
				}
				slx++;	 
			}   
	 		p=p1+1;//切换到下一个GPGSV信息
		}  		
	}
}
//分析GPGGA信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GGA_Analysis(nmea_msg *gpsx,HC_UINT8 *buf)
{
	HC_UINT8 *p1,dx;			 
	HC_UINT8 posx;    
	HC_UINT32 temp,temp2;    
	HC_INT32 iTemp;
	//p1=(HC_UINT8*)strstr((const char *)buf,"$GPGGA");
	p1=(HC_UINT8*)strstr((const char *)buf,"GGA");
	if(!p1)		return;
	posx=NMEA_Comma_Pos(p1,1);								//得到UTC时间
	if(posx!=0XFF)
	{
		temp2=NMEA_Str2num(p1+posx,&dx);	 	//得到UTC时间
		temp = temp2/NMEA_Pow(10,dx);
		gpsx->utc.time= (double)temp2/NMEA_Pow(10,dx);
		gpsx->utc.hour=temp/10000;
		gpsx->utc.min=(temp/100)%100;
		gpsx->utc.sec=temp2%10000;	
	}		
	posx=NMEA_Comma_Pos(p1,6);								//得到GPS状态
	if(posx!=0XFF)gpsx->gpssta=NMEA_Str2num(p1+posx,&dx);	
	posx=NMEA_Comma_Pos(p1,7);								//得到用于定位的卫星数
	if(posx!=0XFF)gpsx->posslnum=NMEA_Str2num(p1+posx,&dx); 
	posx=NMEA_Comma_Pos(p1,9);								//得到海拔高度
	if(posx!=0XFF)
	{
		iTemp =NMEA_Str2num(p1+posx,&dx);
		gpsx->altitude=(HC_FLOAT)iTemp/NMEA_Pow(10,dx);	
	}
	posx=NMEA_Comma_Pos(p1,11);								//得到大地水准面高度
	if(posx!=0XFF)
	{
		iTemp =NMEA_Str2num(p1+posx,&dx);
		gpsx->height_wgs84 = (HC_FLOAT)iTemp/NMEA_Pow(10,dx);
	}
	gpsx->height = gpsx->altitude + gpsx->height_wgs84;
	
	posx=NMEA_Comma_Pos(p1,13);								//差分延时
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);  
		gpsx->dgnss_timeout = (HC_FLOAT)temp/NMEA_Pow(10,dx);
	}
	
	posx=NMEA_Comma_Pos(p1,14);								//基站ID
	if(posx!=0XFF) gpsx->station_id=NMEA_Str2num(p1+posx,&dx);  

}
//分析GPGSA信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GSA_Analysis(nmea_msg *gpsx,HC_UINT8 *buf)
{
	HC_UINT8 *p1,dx;			 
	HC_UINT8 posx; 
	HC_UINT8 i;  
	HC_UINT32 temp;
	//p1=(HC_UINT8*)strstr((const char *)buf,"$GPGSA");
	p1=(HC_UINT8*)strstr((const char *)buf,"GSA");
	if(!p1)		return;
	posx=NMEA_Comma_Pos(p1,2);								//得到定位类型
	if(posx!=0XFF)gpsx->fixmode=NMEA_Str2num(p1+posx,&dx);	
	for(i=0;i<12;i++)										//得到定位卫星编号
	{
		posx=NMEA_Comma_Pos(p1,3+i);					 
		if(posx!=0XFF)gpsx->possl[i]=NMEA_Str2num(p1+posx,&dx);
		else break; 
	}				  
	posx=NMEA_Comma_Pos(p1,15);								//得到PDOP位置精度因子
	if (posx != 0XFF)
	{
		temp = NMEA_Str2num(p1 + posx, &dx);
		gpsx->pdop = (HC_FLOAT)temp / NMEA_Pow(10, dx);
	}
	posx=NMEA_Comma_Pos(p1,16);								//得到HDOP位置精度因子
	if (posx!=0XFF)
	{
		temp = NMEA_Str2num(p1 + posx, &dx);
		gpsx->hdop = (HC_FLOAT)temp / NMEA_Pow(10, dx);
	}
	posx=NMEA_Comma_Pos(p1,17);								//得到VDOP位置精度因子
	if (posx != 0XFF)
	{
		temp = NMEA_Str2num(p1 + posx, &dx);
		gpsx->vdop = (HC_FLOAT)temp / NMEA_Pow(10, dx);
	}
}
//分析GPRMC信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_RMC_Analysis(nmea_msg *gpsx,HC_UINT8 *buf)
{
	HC_UINT8 *p1,dx;			 
	HC_UINT8 posx;     
	HC_UINT32 temp,temp2;	   
	HC_DOUBLE rs;  
	//p1=(HC_UINT8*)strstr((const char *)buf,"GPRMC");//"$GPRMC",经常有&和GPRMC分开的情况,故只判断GPRMC.
	p1=(HC_UINT8*)strstr((const char *)buf,"RMC");//"$GPRMC",经常有&和GPRMC分开的情况,故只判断GPRMC.
	if(!p1)		return;
	posx=NMEA_Comma_Pos(p1,1);								//得到UTC时间
	if(posx!=0XFF)
	{
		temp2=NMEA_Str2num(p1+posx,&dx);	 	//得到UTC时间
		temp = temp2/NMEA_Pow(10,dx);
		gpsx->utc.time=(double)temp2/NMEA_Pow(10,dx);
		gpsx->utc.hour=temp/10000;
		gpsx->utc.min=(temp/100)%100;
		gpsx->utc.sec=temp2%10000;	 	 
	}	
	posx=NMEA_Comma_Pos(p1,3);								//得到纬度
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		gpsx->latitude=temp/NMEA_Pow(10,dx+2);	//得到°
		rs=temp%NMEA_Pow(10,dx+2);				//得到'		 
		gpsx->latitude=gpsx->latitude+rs/NMEA_Pow(10,dx)/60.0;//转换为° 
		if(gpsx->latitude > 0)
			gpsx->count++;
		else if (gpsx->latitude == 0)
			gpsx->count = 0;
	}
	posx=NMEA_Comma_Pos(p1,4);								//南纬还是北纬 
	if(posx!=0XFF && (*(p1+posx)=='N' || *(p1+posx)=='S' ))
		gpsx->nshemi=*(p1+posx);					 
 	posx=NMEA_Comma_Pos(p1,5);								//得到经度
	if(posx!=0XFF)
	{												  
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		gpsx->longitude=temp/NMEA_Pow(10,dx+2);	//得到°
		rs=temp%NMEA_Pow(10,dx+2);				//得到'		 
		gpsx->longitude=gpsx->longitude+(HC_DOUBLE)rs/NMEA_Pow(10,dx)/60.0;//转换为° 
	}
	posx=NMEA_Comma_Pos(p1,6);								//东经还是西经
	if(posx!=0XFF && (*(p1+posx)=='E' || *(p1+posx)=='W' ))
		gpsx->ewhemi=*(p1+posx);		 
	posx=NMEA_Comma_Pos(p1,7);								//速度
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);
		gpsx->speed = (HC_FLOAT)temp / NMEA_Pow(10, dx);
		gpsx->speed = (HC_UINT32)(gpsx->speed*1.852f/3.6f);	//海里/小时 -> m/s
	}
	posx=NMEA_Comma_Pos(p1,8);								//速度
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);
		gpsx->heading=(HC_FLOAT)temp/NMEA_Pow(10,dx);	 	 
	}
	posx=NMEA_Comma_Pos(p1,9);								//得到UTC日期
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 				//得到UTC日期
		if(temp>0)
		{
			gpsx->utc.date=temp/10000;
			gpsx->utc.month=(temp/100)%100;
			gpsx->utc.year=2000+temp%100;	 	 
			//31-25: Year(0-127 org.1980), 24-21: Month(1-12), 20-16: Day(1-31) */                                                                                                                                                                                                                                          
			//15-11: Hour(0-23), 10-5: Minute(0-59), 4-0: Second(0-29 *2) */      
			gpsx->utc.fattime = (((HC_UINT32)gpsx->utc.year-1980)&0x0000003F)<<25;			
			gpsx->utc.fattime |= ((HC_UINT32)gpsx->utc.month&0x0000000F)<<21;			
			gpsx->utc.fattime |= ((HC_UINT32)gpsx->utc.date&0x0000001F)<<16;			
			gpsx->utc.fattime |= ((HC_UINT32)gpsx->utc.hour&0x0000001F)<<11;			
			gpsx->utc.fattime |= ((HC_UINT32)gpsx->utc.min&0x0000003F)<<5;			
			gpsx->utc.fattime |= ((HC_UINT32)(gpsx->utc.sec/100/2)&0x0000001F);	
		}
	} 
}
//分析GPVTG信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_VTG_Analysis(nmea_msg *gpsx,HC_UINT8 *buf)
{
	HC_UINT8 *p1,dx;			 
	HC_UINT8 posx;  
	HC_UINT32 temp;
	//p1=(HC_UINT8*)strstr((const char *)buf,"$GPVTG");							 
	p1=(HC_UINT8*)strstr((const char *)buf,"VTG");							 
	if(!p1)		return;
	posx=NMEA_Comma_Pos(p1,7);								//得到地面速率
	if(posx!=0XFF)
	{
		temp = NMEA_Str2num(p1 + posx, &dx);
		gpsx->speed = (HC_FLOAT)temp / NMEA_Pow(10, dx);
		gpsx->speed = (HC_UINT32)(gpsx->speed*1.852f / 3.6f);	//海里/小时 -> m/s
	}
}  
//提取NMEA-0183信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void GPS_Analysis(HC_UINT8 *buf,HC_UINT16 len,nmea_msg *gpsx)
{
	NMEA_RMC_Analysis(gpsx,buf);	//GPRMC解析
	NMEA_GGA_Analysis(gpsx,buf);	//GPGGA解析 	
	NMEA_GSA_Analysis(gpsx,buf);	//GPGSA解析
	NMEA_GSV_Analysis(gpsx,buf);	//GPGSV解析
	//NMEA_VTG_Analysis(gpsx,buf);	//GPVTG解析
	/*
	HC_UINT8 i;
	printf("svnum\t\t:%u\r\n", gpsx->svnum);
	printf("latitude\t:%.7f\r\n", gpsx->latitude);
	printf("nshemi\t\t:%c\r\n", gpsx->nshemi);
	printf("longitude\t:%u\r\n", gpsx->longitude);
	printf("ewhemi\t\t:%c\r\n", gpsx->ewhemi);
	printf("gpssta\t\t:%u\r\n", gpsx->gpssta);
	printf("posslnum\t:%u\r\n", gpsx->posslnum);
	printf("possl\t\t:");
	for(i=0;i<gpsx->posslnum;i++)
		printf("%u  ", gpsx->possl[i]);
	printf("\r\n");
	printf("fixmode\t\t:%u\r\n", gpsx->fixmode);
	printf("pdop\t\t:%u\r\n", gpsx->pdop);
	printf("hdop\t\t:%u\r\n", gpsx->hdop);
	printf("vdop\t\t:%u\r\n", gpsx->vdop);
	printf("height\t\t:%d\r\n", gpsx->height);
	printf("speed\t\t:%u\r\n", gpsx->speed);
	printf("heading\t\t:%u\r\n", gpsx->heading);
	*/
}

