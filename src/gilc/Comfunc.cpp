#ifdef WIN32
#include "stdafx.h"
//#include <Eigen/Dense>
//using namespace Eigen;     // 改成这样亦可 using Eigen::MatrixXd;
#endif

#include "GILC.h"

extern bool is_nan(double dVel)
{
	if(dVel==dVel)
		return false;
	return true;
}

extern void *__ml_zero(int size)
{
	void *p = malloc(size);
	if (p == NULL) {
		gilc_log("not malloc for val\n");
	}
	memset(p, 0, size);
	return p;
}

extern int Str2Array(const char *str,const char *sep,double *val)
{
	char *p,_str[1024];
	double d[MAXVAL]={0.0};
	int i,j;

	strcpy(_str,str);
	for (i=0,p=strtok(_str,sep);p&&i<MAXVAL;p=strtok(NULL,sep),i++) { 
		d[i]=atof(p);
	}

	for(j=0;j<i;j++) val[j]=d[j];
	return i;
}
extern int checkstr(const char* str,const char* substr)
{
	int i, j, check ;
    int len = strlen(str);        /*取得字符串长度，不包括'\0'*/
    int sublen = strlen(substr);
	if(len<sublen)
	{return 0;}
    for(i=0;i<len;i++)
    {
        check = 1;                /*检测标记*/
        for(j=0;j+i<len&&j<sublen;j++)        /*逐个字符进行检测，在sublen长度内，一旦出现不同字符便将check置为0*/
        {
            if( str[i+j] != substr[j] )
            {
                check = 0;
                break;
            }
        }
        if(check == 1)           /*在sublen长度内的字符都相等*/
        {
			break;
			//count++;
			//i=i+sublen;           /*调整检测起始位置*/
        }
    }
    return i;
}
extern double GetAveStdRMS(const double *a, int n, int opt)
{
	int i;
	double avg=0.0,std=0.0,rms=0.0,sum=0.0;

	if(n==0) return 99999.9;

	for (i=0;i<n;i++){
		sum+= a[i];
		rms+= a[i]*a[i];
	}
	avg=sum/n;

	if (opt==0) return avg;

	sum=0.0;
	for (i=0;i<n;i++){
		sum+=(a[i]-avg)*(a[i]-avg);
	}

	std=sqrt(sum/double(n-1));
	rms=sqrt(rms/double(n));

	if (opt==1) return std;
	else if (opt==2) return rms;

	return 0.0;
}
extern double GetAveStd(vector<double, malloc_allocator<double> > a,int opt)
{
	int n=a.size();
	double avg=0.0,std=0.0,rms=0.0,sum=0.0;

	if(n==0) 
		return 99999.9;

	if (opt==2) 
	{
	   for (int i=0;i<n;i++){
		   rms+= a[i]*a[i];
	   }
	   rms=sqrt(rms/double(n));
	   return rms;
	}

	for (int i=0;i<n;i++){
		sum+= a[i];
	}
	avg=sum/n;

	if (opt==0) 
		return avg;
	
	if (opt==1) 
	{
		sum=0.0;
		for (int i=0;i<n;i++)
		{
			sum+=(a[i]-avg)*(a[i]-avg);
		}
		std=sqrt(sum/double(n-1));
		return std;
	}
	
	else
	return 0.0;
}
extern double GetAveStd(vector< vector<double, malloc_allocator<double> > > a,int col,int opt)
{
	int n=a.size();
	double avg=0.0,std=0.0,rms=0.0,sum=0.0;

	if(n==0) return 99999.9;

	for (int i=0;i<n;i++){
		sum+= a[i][col];
		rms+= a[i][col]*a[i][col];
	}
	avg=sum/n;

	if (opt==0) return avg;

	sum=0.0;
	for (int i=0;i<n;i++){
		sum+=(a[i][col]-avg)*(a[i][col]-avg);
	}

	std=sqrt(sum/double(n-1));
	rms=sqrt(rms/double(n));

	if (opt==1) return std;
	else if (opt==2) return rms;

	return 0.0;
}

//GetMaxMinAve(dMat_FW[i], 0, DATA_LEN - 1, &dMax[i], &dMin[i], &dAve[i]);
extern double GetMaxMinDiff(vector<double, malloc_allocator<double> > a,int opt)
{
	double dMax, dMin, dAve;
	int n=a.size();
	int i;

	dMax = a[0];
	dMin = a[0];
	dAve = a[0];

	for (i = 1; i < n; i++)
	{
		if (dMax < a[i]) dMax = a[i];
		if (dMin > a[i]) dMin = a[i];
		dAve += a[i];
	}

	dAve /= n;
	return (dMax - dMin);
}

extern char* time2str(double *ep, int n)
{
	static char buff[64];

	if (n<0) n=0;

	sprintf(buff,"%04.0f/%02.0f/%02.0f %02.0f:%02.0f:%0*.*f",ep[0],ep[1],ep[2],
		ep[3],ep[4],n<=0?2:n+3,n<=0?0:n,ep[5]);
	return buff;
}

extern double str2num(const char *s, int i, int n)
{
	double value;
	char str[256],*p=str;

	if (i<0||(int)strlen(s)<i||(int)sizeof(str)-1<n) return 0.0;
	for (s+=i;*s&&--n>=0;s++) *p++=*s=='d'||*s=='D'?'E':*s; *p='\0';
	return sscanf(str,"%lf",&value)==1?value:0.0;
}

extern void matmul(const char *tr, int n, int k, int m, double alpha,
	const double *A, const double *B, double beta, double *C)
{
	double d;
	int i,j,x,f=tr[0]=='N'?(tr[1]=='N'?1:2):(tr[1]=='N'?3:4);

	for (i=0;i<n;i++) for (j=0;j<k;j++) {
		d=0.0;
		switch (f) {
		case 1: for (x=0;x<m;x++) d+=A[i+x*n]*B[x+j*m]; break;
		case 2: for (x=0;x<m;x++) d+=A[i+x*n]*B[j+x*k]; break;
		case 3: for (x=0;x<m;x++) d+=A[x+i*m]*B[x+j*m]; break;
		case 4: for (x=0;x<m;x++) d+=A[x+i*m]*B[j+x*k]; break;
		}
		if (beta==0.0) C[i+j*n]=alpha*d; else C[i+j*n]=alpha*d+beta*C[i+j*n];
	}
}
/* transform ecef to geodetic postion ------------------------------------------
* transform ecef position to geodetic position
* args   : double *r        I   ecef position {x,y,z} (m)
*          double *pos      O   geodetic position {lat,lon,h} (rad,m)
* return : none
* notes  : WGS84, ellipsoidal height
*-----------------------------------------------------------------------------*/
extern void ecef2pos(const double *r, double *pos)
{
    double e2=FE_WGS84*(2.0-FE_WGS84),r2=dot(r,r,2),z,zk,v=RE_WGS84,sinp;
    
    for (z=r[2],zk=0.0;fabs(z-zk)>=1E-4;) {
        zk=z;
        sinp=z/sqrt(r2+z*z);
        v=RE_WGS84/sqrt(1.0-e2*sinp*sinp);
        z=r[2]+v*e2*sinp;
    }
    pos[0]=r2>1E-12?atan(z/sqrt(r2)):(r[2]>0.0?PI/2.0:-PI/2.0);
    pos[1]=r2>1E-12?atan2(r[1],r[0]):0.0;
    pos[2]=sqrt(r2+z*z)-v;
}
/* transform geodetic to ecef position -----------------------------------------
* transform geodetic position to ecef position
* args   : double *pos      I   geodetic position {lat,lon,h} (rad,m)
*          double *r        O   ecef position {x,y,z} (m)
* return : none
* notes  : WGS84, ellipsoidal height
*-----------------------------------------------------------------------------*/
extern void pos2ecef(const double *pos, double *r)
{
	double sinp=sin(pos[0]),cosp=cos(pos[0]),sinl=sin(pos[1]),cosl=cos(pos[1]);
	double e2=FE_WGS84*(2.0-FE_WGS84),v=RE_WGS84/sqrt(1.0-e2*sinp*sinp);

	r[0]=(v+pos[2])*cosp*cosl;
	r[1]=(v+pos[2])*cosp*sinl;
	r[2]=(v*(1.0-e2)+pos[2])*sinp;
}
/* ecef to local coordinate transfromation matrix ------------------------------
* compute ecef to local coordinate transfromation matrix
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *E        O   ecef to local coord transformation matrix (3x3)
* return : none
* notes  : matirix stored by column-major order (fortran convention)
*-----------------------------------------------------------------------------*/
extern void xyz2enu(const double *pos, double *E)
{
	double sinp=sin(pos[0]),cosp=cos(pos[0]),sinl=sin(pos[1]),cosl=cos(pos[1]);

	E[0]=-sinl;      E[3]=cosl;       E[6]=0.0;
	E[1]=-sinp*cosl; E[4]=-sinp*sinl; E[7]=cosp;
	E[2]=cosp*cosl;  E[5]=cosp*sinl;  E[8]=sinp;
}
/* transform ecef vector to local tangental coordinate -------------------------
* transform ecef vector to local tangental coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *r        I   vector in ecef coordinate {x,y,z}
*          double *e        O   vector in local tangental coordinate {e,n,u}
* return : none
*-----------------------------------------------------------------------------*/
extern void ecef2enu(const double *pos, const double *r, double *e)
{
	double E[9];

	xyz2enu(pos,E);
	matmul("NN",3,1,3,1.0,E,r,0.0,e);
}
/* transform local vector to ecef coordinate -----------------------------------
* transform local tangental coordinate vector to ecef
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *e        I   vector in local tangental coordinate {e,n,u}
*          double *r        O   vector in ecef coordinate {x,y,z}
* return : none
*-----------------------------------------------------------------------------*/
extern void enu2ecef(const double *pos, const double *e, double *r)
{
	double E[9];

	xyz2enu(pos,E);
	matmul("TN",3,1,3,1.0,E,e,0.0,r);
}
/* inner product ---------------------------------------------------------------
* inner product of vectors
* args   : double *a,*b     I   vector a,b (n x 1)
*          int    n         I   size of vector a,b
* return : a'*b
*-----------------------------------------------------------------------------*/
extern double dot(const double *a, const double *b, int n)
{
	double c=0.0;

	while (--n>=0) c+=a[n]*b[n];
	return c;
}
/* euclid norm -----------------------------------------------------------------
* euclid norm of vector
* args   : double *a        I   vector a (n x 1)
*          int    n         I   size of vector a
* return : || a ||
*-----------------------------------------------------------------------------*/
extern double norm(const double *a, int n)
{
	return sqrt(dot(a,a,n));
}
/* outer product of 3d vectors -------------------------------------------------
* outer product of 3d vectors 
* args   : double *a,*b     I   vector a,b (3 x 1)
*          double *c        O   outer product (a x b) (3 x 1)
* return : none
*-----------------------------------------------------------------------------*/
extern void cross3(const double *a, const double *b, double *c)
{
	c[0]=a[1]*b[2]-a[2]*b[1];
	c[1]=a[2]*b[0]-a[0]*b[2];
	c[2]=a[0]*b[1]-a[1]*b[0];
}
/* transform covariance to local tangental coordinate --------------------------
* transform ecef covariance to local tangental coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *P        I   covariance in ecef coordinate
*          double *Q        O   covariance in local tangental coordinate
* return : none
*-----------------------------------------------------------------------------*/
extern void covenu(const double *pos, const double *P, double *Q)
{
    double E[9],EP[9];
    
    xyz2enu(pos,E);
    matmul("NN",3,3,3,1.0,E,P,0.0,EP);
    matmul("NT",3,3,3,1.0,EP,E,0.0,Q);
}
/* transform local enu coordinate covariance to xyz-ecef -----------------------
* transform local enu covariance to xyz-ecef coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *Q        I   covariance in local enu coordinate
*          double *P        O   covariance in xyz-ecef coordinate
* return : none
*-----------------------------------------------------------------------------*/
extern void covecef(const double *pos, const double *Q, double *P)
{
    double E[9],EQ[9];
    
    xyz2enu(pos,E);
    matmul("TN",3,3,3,1.0,E,Q,0.0,EQ);
    matmul("NN",3,3,3,1.0,EQ,E,0.0,P);
}
//XYZ坐标系下的速度/位置标准差（m/s）转成BLH下的速度/位置方差（rad2/s2 rad2/2 m2/s2） 
// D(enu)=E(xyz2enu)D(xyz)ET(xyz2enu) 注意lib函数matmul的使用 转置与不转置的调换
/* args   : double *xyz      I   ecef position {x,y,z} (m)
*          double *Pposecef  I   vn/pos std variance in ecef
*          double *Pposenu  0    vn/pos variance in geodetic
*          int option     I    1-position  0-velocity
* return : none
*-----------------------------------------------------------------------------*/
void Var_XYZ2BLH(double xyz[3],double Pecef[3],double Penu[3])
{
	double ENU[9],blh[3];
#if 0 // POS_XYZ
	ecef2pos(xyz,blh);
#endif 
#if 1 // POS_BLH_deg
	blh[0]=xyz[0]*PI/180.0;
	blh[1]=xyz[1]*PI/180.0;
	blh[2]=xyz[2];
#endif 
	xyz2enu(blh,ENU);

	//Penu[0]=sqrt(pow(ENU[0],2)*pow(Pecef[0],2)+pow(ENU[1],2)*pow(Pecef[1],2));
	//Penu[1]=sqrt(pow(ENU[3],2)*pow(Pecef[0],2)+pow(ENU[4],2)*pow(Pecef[1],2)+pow(ENU[5],2)*pow(Pecef[2],2));
	//Penu[2]=sqrt(pow(ENU[6],2)*pow(Pecef[0],2)+pow(ENU[7],2)*pow(Pecef[1],2)+pow(ENU[8],2)*pow(Pecef[2],2));

	double Decef[3*3]={0.0},temp1[9]={0.0},temp2[9]={0.0};
	for(int i=0;i<3;i++)
	{Decef[i*3+i]=pow(Pecef[i],2);}

	matmul("TN",3,3,3,1.0,ENU,Decef,0.0,temp1);
	matmul("NN",3,3,3,1.0,temp1,ENU,0.0,temp2);

	for(int i=0;i<3;i++)
	{Penu[i]=sqrt(temp2[i*3+i]);}
}

extern void getHMS(double ggat,double ep[6])
{
	ep[0]=0;ep[1]=0;ep[2]=0;
	ep[3]=int(ggat/10000.0);
	ep[4]=int(fmod(ggat,10000.0)/100.0);
	ep[5]=fmod(ggat,100.0);
}

extern void getPOS_rad( double lat,double lon,double hgt,double blh[3])
{
	double deg,min;

	deg=(int)(lat/100.0);
	min=fmod(lat,100.0);
	blh[0]=(deg+min/60.0)*PI/180.0;

	deg=(int)(lon/100.0);
	min=fmod(lon,100.0);
	blh[1]=(deg+min/60.0)*PI/180.0;

	blh[2]=hgt;
}

/* add time --------------------------------------------------------------------
* add time to gtime_t struct
* args   : gtime_t t        I   gtime_t struct
*          double sec       I   time to add (s)
* return : gtime_t struct (t+sec)
*-----------------------------------------------------------------------------*/
extern gtime_t timeadd(gtime_t t, double sec)
{
	double tt;

	t.sec+=sec; tt=floor(t.sec); t.time+=(int)tt; t.sec-=tt;
	return t;
}
/* time difference -------------------------------------------------------------
* difference between gtime_t structs
* args   : gtime_t t1,t2    I   gtime_t structs
* return : time difference (t1-t2) (s)
*-----------------------------------------------------------------------------*/
extern double timediff(gtime_t t1, gtime_t t2)
{
	return difftime(t1.time,t2.time)+t1.sec-t2.sec;
}

/* string to time --------------------------------------------------------------
* convert substring in string to gtime_t struct
* args   : char   *s        I   string ("... yyyy mm dd hh mm ss ...")
*          int    i,n       I   substring position and width
*          gtime_t *t       O   gtime_t struct
* return : status (0:ok,0>:error)
*-----------------------------------------------------------------------------*/
extern int str2time(const char *s, int i, int n, gtime_t *t)
{
	double ep[6];
	char str[256],*p=str;

	if (i<0||(int)strlen(s)<i||(int)sizeof(str)-1<i) return -1;
	for (s+=i;*s&&--n>=0;) *p++=*s++; *p='\0';
	if (sscanf(str,"%lf/%lf/%lf %lf:%lf:%lf",ep,ep+1,ep+2,ep+3,ep+4,ep+5)<6)
		return -1;
	if (ep[0]<100.0) ep[0]+=ep[0]<80.0?2000.0:1900.0;
	ep[5]=ceil(ep[5]);
	*t=epoch2time(ep);
	return 0;
}
/* convert calendar day/time to time -------------------------------------------
* convert calendar day/time to gtime_t struct
* args   : double *ep       I   day/time {year,month,day,hour,min,sec}
* return : gtime_t struct
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
extern gtime_t epoch2time(const double *ep)
{
	const int doy[]={1,32,60,91,121,152,182,213,244,274,305,335};
	gtime_t time={0};
	int days,sec,year=(int)ep[0],mon=(int)ep[1],day=(int)ep[2];

	if (year<1970||2099<year||mon<1||12<mon) return time;

	/* leap year if year%4==0 in 1901-2099 */
	days=(year-1970)*365+(year-1969)/4+doy[mon-1]+day-2+(year%4==0&&mon>=3?1:0);
	sec=(int)floor(ep[5]);
	time.time=(time_t)days*86400+(int)ep[3]*3600+(int)ep[4]*60+sec;
	time.sec=ep[5]-sec;
	return time;
}
/* time to calendar day/time ---------------------------------------------------
* convert gtime_t struct to calendar day/time
* args   : gtime_t t        I   gtime_t struct
*          double *ep       O   day/time {year,month,day,hour,min,sec}
* return : none
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
extern void time2epoch(gtime_t t, double *ep)
{
	const int mday[]={ /* # of days in a month */
		31,28,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31,
		31,29,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31
	};
	int days,sec,mon,day;

	/* leap year if year%4==0 in 1901-2099 */
	days=(int)(t.time/86400);
	sec=(int)(t.time-(time_t)days*86400);
	for (day=days%1461,mon=0;mon<48;mon++) {
		if (day>=mday[mon]) day-=mday[mon]; else break;
	}
	ep[0]=1970+days/1461*4+mon/12; ep[1]=mon%12+1; ep[2]=day+1;
	ep[3]=sec/3600; ep[4]=sec%3600/60; ep[5]=sec%60+t.sec;
}
/* gpstime to utc --------------------------------------------------------------
* convert gpstime to utc considering leap seconds
* args   : gtime_t t        I   time expressed in gpstime
* return : time expressed in utc
* notes  : ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
extern gtime_t gpst2utc(gtime_t t)
{
	gtime_t tu;
	int i;

	for (i=0;i<(int)sizeof(leaps)/(int)sizeof(*leaps);i++) {
		tu=timeadd(t,leaps[i][6]);
		if (timediff(tu,epoch2time(leaps[i]))>=0.0) return tu;
	}
	return t;
}
/* gps time to time ------------------------------------------------------------
* convert week and tow in gps time to gtime_t struct
* args   : int    week      I   week number in gps time
*          double sec       I   time of week in gps time (s)
* return : gtime_t struct
*-----------------------------------------------------------------------------*/
extern gtime_t gpst2time(int week, double sec)
{
    gtime_t t=epoch2time(gpst0);
    
    if (sec<-1E9||1E9<sec) sec=0.0;
    t.time+=86400*7*week+(int)sec;
    t.sec=sec-(int)sec;
    return t;
}
/* time to gps time ------------------------------------------------------------
* convert gtime_t struct to week and tow in gps time
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in gps time (NULL: no output)
* return : time of week in gps time (s)
*-----------------------------------------------------------------------------*/
extern double time2gpst(gtime_t t, int *week)
{
    gtime_t t0=epoch2time(gpst0);
    time_t sec=t.time-t0.time;
    int w=(int)(sec/(86400*7));
    
    if (week) *week=w;
    return (double)(sec-w*86400*7)+t.sec;
}

/* result = b*a */
void matrix_mult_33x33f(double b[3][3], double a[3][3], double result[3][3])
{
	result[0][0] = a[0][0] * b[0][0] + a[1][0] * b[0][1] + a[2][0] * b[0][2];
	result[0][1] = a[0][1] * b[0][0] + a[1][1] * b[0][1] + a[2][1] * b[0][2];
	result[0][2] = a[0][2] * b[0][0] + a[1][2] * b[0][1] + a[2][2] * b[0][2];

	result[1][0] = a[0][0] * b[1][0] + a[1][0] * b[1][1] + a[2][0] * b[1][2];
	result[1][1] = a[0][1] * b[1][0] + a[1][1] * b[1][1] + a[2][1] * b[1][2];
	result[1][2] = a[0][2] * b[1][0] + a[1][2] * b[1][1] + a[2][2] * b[1][2];

	result[2][0] = a[0][0] * b[2][0] + a[1][0] * b[2][1] + a[2][0] * b[2][2];
	result[2][1] = a[0][1] * b[2][0] + a[1][1] * b[2][1] + a[2][1] * b[2][2];
	result[2][2] = a[0][2] * b[2][0] + a[1][2] * b[2][1] + a[2][2] * b[2][2];
}
/* result = b*a */
void matrix_mult_43x33f(double b[4][3], double a[3][3], double result[4][3])
{
	result[0][0] = a[0][0] * b[0][0] + a[1][0] * b[0][1] + a[2][0] * b[0][2];
	result[0][1] = a[0][1] * b[0][0] + a[1][1] * b[0][1] + a[2][1] * b[0][2];
	result[0][2] = a[0][2] * b[0][0] + a[1][2] * b[0][1] + a[2][2] * b[0][2];

	result[1][0] = a[0][0] * b[1][0] + a[1][0] * b[1][1] + a[2][0] * b[1][2];
	result[1][1] = a[0][1] * b[1][0] + a[1][1] * b[1][1] + a[2][1] * b[1][2];
	result[1][2] = a[0][2] * b[1][0] + a[1][2] * b[1][1] + a[2][2] * b[1][2];

	result[2][0] = a[0][0] * b[2][0] + a[1][0] * b[2][1] + a[2][0] * b[2][2];
	result[2][1] = a[0][1] * b[2][0] + a[1][1] * b[2][1] + a[2][1] * b[2][2];
	result[2][2] = a[0][2] * b[2][0] + a[1][2] * b[2][1] + a[2][2] * b[2][2];

	result[3][0] = a[0][0] * b[3][0] + a[1][0] * b[3][1] + a[2][0] * b[3][2];
	result[3][1] = a[0][1] * b[3][0] + a[1][1] * b[3][1] + a[2][1] * b[3][2];
	result[3][2] = a[0][2] * b[3][0] + a[1][2] * b[3][1] + a[2][2] * b[3][2];
}
/* result = b*a */
void matrix_mult_33x34f(double b[3][3], double a[3][4], double result[3][4])
{
	result[0][0] = a[0][0] * b[0][0] + a[1][0] * b[0][1] + a[2][0] * b[0][2];
	result[0][1] = a[0][1] * b[0][0] + a[1][1] * b[0][1] + a[2][1] * b[0][2];
	result[0][2] = a[0][2] * b[0][0] + a[1][2] * b[0][1] + a[2][2] * b[0][2];
	result[0][3] = a[0][3] * b[0][0] + a[1][3] * b[0][1] + a[2][3] * b[0][2];

	result[1][0] = a[0][0] * b[1][0] + a[1][0] * b[1][1] + a[2][0] * b[1][2];
	result[1][1] = a[0][1] * b[1][0] + a[1][1] * b[1][1] + a[2][1] * b[1][2];
	result[1][2] = a[0][2] * b[1][0] + a[1][2] * b[1][1] + a[2][2] * b[1][2];
	result[1][3] = a[0][3] * b[1][0] + a[1][3] * b[1][1] + a[2][3] * b[1][2];

	result[2][0] = a[0][0] * b[2][0] + a[1][0] * b[2][1] + a[2][0] * b[2][2];
	result[2][1] = a[0][1] * b[2][0] + a[1][1] * b[2][1] + a[2][1] * b[2][2];
	result[2][2] = a[0][2] * b[2][0] + a[1][2] * b[2][1] + a[2][2] * b[2][2];
	result[2][3] = a[0][3] * b[2][0] + a[1][3] * b[2][1] + a[2][3] * b[2][2];
}
/* result = b*a
4x4  =  4x3 * 3x4
*/
void matrix_mult_43x34f(double b[4][3], double a[3][4], double result[4][4])
{
	result[0][0] = a[0][0] * b[0][0] + a[1][0] * b[0][1] + a[2][0] * b[0][2];
	result[0][1] = a[0][1] * b[0][0] + a[1][1] * b[0][1] + a[2][1] * b[0][2];
	result[0][2] = a[0][2] * b[0][0] + a[1][2] * b[0][1] + a[2][2] * b[0][2];
	result[0][3] = a[0][3] * b[0][0] + a[1][3] * b[0][1] + a[2][3] * b[0][2];

	result[1][0] = a[0][0] * b[1][0] + a[1][0] * b[1][1] + a[2][0] * b[1][2];
	result[1][1] = a[0][1] * b[1][0] + a[1][1] * b[1][1] + a[2][1] * b[1][2];
	result[1][2] = a[0][2] * b[1][0] + a[1][2] * b[1][1] + a[2][2] * b[1][2];
	result[1][3] = a[0][3] * b[1][0] + a[1][3] * b[1][1] + a[2][3] * b[1][2];

	result[2][0] = a[0][0] * b[2][0] + a[1][0] * b[2][1] + a[2][0] * b[2][2];
	result[2][1] = a[0][1] * b[2][0] + a[1][1] * b[2][1] + a[2][1] * b[2][2];
	result[2][2] = a[0][2] * b[2][0] + a[1][2] * b[2][1] + a[2][2] * b[2][2];
	result[2][3] = a[0][3] * b[2][0] + a[1][3] * b[2][1] + a[2][3] * b[2][2];

	result[3][0] = a[0][0] * b[3][0] + a[1][0] * b[3][1] + a[2][0] * b[3][2];
	result[3][1] = a[0][1] * b[3][0] + a[1][1] * b[3][1] + a[2][1] * b[3][2];
	result[3][2] = a[0][2] * b[3][0] + a[1][2] * b[3][1] + a[2][2] * b[3][2];
	result[3][3] = a[0][3] * b[3][0] + a[1][3] * b[3][1] + a[2][3] * b[3][2];
}

void Maddn(double *a,double *b,double *c,int m,int n)
{   
	for(int i=0;i<m;i++)
		for(int j=0;j<n;j++)
		{
			*(c+i*n+j)=*(a+i*n+j)+*(b+i*n+j);
		}
}

void Madd(double *a,double *b,int m,int n)
{   
	for(int i=0;i<m;i++)
		for(int j=0;j<n;j++)
		{
			*(a+i*n+j)=*(a+i*n+j)+*(b+i*n+j);
		}
}

void Mminn(double *a,double *b,double *c,int m,int n)
{
	int i,j;

	for(i=0;i<m;i++)
		for(j=0;j<n;j++)
		{
			*(c+i*n+j)=*(a+i*n+j)-*(b+i*n+j);		
		}
}

void Mmin(double *a,double *b,int m,int n)
{
	int i,j;

	for(i=0;i<m;i++)
		for(j=0;j<n;j++)
		{
			*(a+i*n+j)=*(a+i*n+j)-*(b+i*n+j);		
		}
}
#if 1
void Mmulnm(double *ain,double *bin,int m,int n,int k,double *c) 
{
	int i,j,l,u;
	double *a = (double *)malloc(m*n*sizeof(double));
	double *b = (double *)malloc(k*n*sizeof(double));
	for(i=0;i<m*n;i++)
		a[i] = ain[i];
	for(i=0;i<k*n;i++)
		b[i] = bin[i];

	for(i=0;i<=m-1;i++)
	for(j=0;j<=k-1;j++)
	{   u=i*k+j;
	    c[u]=0.0;
	    for(l=0;l<=n-1;l++)
		    c[u]=c[u]+a[i*n+l]*b[l*k+j]; 
	}
	free(a);
	free(b);
}
#else
void Mmulnm(double *ain, double *bin, int m, int n, int k, double *cout)
{
	int i, j, l, u;
	MatrixXd a(m,n);
	MatrixXd b(n,k);
	MatrixXd c(m,k);

	for (i = 0; i < m; i++)
		for (j = 0; j < n; j++)
			a(i, j) = ain[i*n + j];

	for (i = 0; i < n; i++)
		for (j = 0; j < k; j++)
			b(i, j) = bin[i*k + j];

	c = a*b;

	for (i = 0; i < m; i++)
		for (j = 0; j < k; j++)
			cout[i*k + j] = c(i, j);
}
#endif
void Mmul(double *a,int m,int n,double b)
{
   for (int i=0;i<m;i++)
   {
	   for (int j=0;j<n;j++)
	   {
		   *(a+i*n+j) = *(a+i*n+j)*b;
	   }
   }
}

void Mmuln(double *a,int m,int n,double b,double *c)
{
	for (int i=0;i<m;i++)
   {
	   for (int j=0;j<n;j++)
	   {
		   *(c+i*n+j) = *(a+i*n+j)*b;
	   }
   }
}

void Mtn(double *a,int m,int n,double *b) 
{
	for(int l=0;l<n;l++)
		for(int k=0;k<m;k++)
		{b[l*m+k]=a[k*n+l];}

}

 void Mt(double *a,int m,int n)
{
	double *at;
	at=(double *)malloc(n*m*sizeof(double));
	for(int i=0;i<m;i++)
		for(int j=0;j<n;j++)
		{
			*(at+i*n+j)=*(a+i*n+j);
		}
	for(int l=0;l<n;l++)
		for(int k=0;k<m;k++)
		{a[l*m+k]=at[k*n+l];}
    free(at);
}

double Minv(double a[],int n)
{
   int *is,*js,i,j,k,l,u,v;
   double d,p;
   is=(int *)malloc(n*sizeof(int));
   js=(int *)malloc(n*sizeof(int));
   for(k=0;k<=n-1;k++)
    { 
       d=0.0;
       for(i=k;i<=n-1;i++)
         for(j=k;j<=n-1;j++)
           {
             l=i*n+j; 
             p=fabs(a[l]);
             if(p>d)
              {  
                d=p;
			    is[k]=i;
			    js[k]=j;
              }
           }
      if(d==0)//d+1.0==1.0 lcc
        {
          free(is);
          free(js);
		  gilc_log("\nerror:inverse matrix is not exist\n");
          return (0);
        }
      if(is[k]!=k)
        for(j=0;j<=n-1;j++)
          {
             u=k*n+j;v=is[k]*n+j;
             p=a[u];a[u]=a[v];a[v]=p;
          }
      if(js[k]!=k)
        for(i=0;i<=n-1;i++)
          {
            u=i*n+k;
            v=i*n+js[k];
            p=a[u];
            a[u]=a[v];a[v]=p;
          }
      l=k*n+k;
      a[l]=1.0/a[l];
      for(j=0;j<=n-1;j++)
        if(j!=k)
         {
            u=k*n+j;
            a[u]=a[u]*a[l];
         }
      for(i=0;i<=n-1;i++)
        if(i!=k)
          for(j=0;j<=n-1;j++)
            if(j!=k)
             {
               u=i*n+j;
               a[u]=a[u]-a[i*n+k]*a[k*n+j];
             }
      for(i=0;i<=n-1;i++)
         if(i!=k)
           {
             u=i*n+k;a[u]=-a[u]*a[l];
           }
    }     
    for(k=n-1;k>=0;k--)
      {
        if(js[k]!=k)
          for(j=0;j<=n-1;j++)
            {
               u=k*n+j;v=js[k]*n+j;
               p=a[u];
               a[u]=a[v];
               a[v]=p;
            }  
        if(is[k]!=k)
          for(i=0;i<=n-1;i++)
            {
               u=i*n+k;
			   v=i*n+is[k];	
               p=a[u];
			   a[u]=a[v];a[v]=p;
            }           
      }

     free(is);
     free(js);
     return(1);
}

double Minvn(double a[],int n,double *b)
{
	for(int i=0;i<n;i++)
		for(int j=0;j<n;j++)
		{
			*(b+i*n+j)=*(a+i*n+j);
		}

   int *is,*js,i,j,k,l,u,v;
   double d,p;
   is=(int *)malloc(n*sizeof(int));
   js=(int *)malloc(n*sizeof(int));
   for(k=0;k<=n-1;k++)
    { 
       d=0.0;
       for(i=k;i<=n-1;i++)
         for(j=k;j<=n-1;j++)
           {
             l=i*n+j; 
             p=fabs(b[l]);
             if(p>d)
              {  
                d=p;
			    is[k]=i;
			    js[k]=j;
              }
           }
      if(d==0)//d+1.0==1.0 lcc
        {
          free(is);
          free(js);
		  gilc_log("\nerror:inverse matrix is not exist\n");
          return (0);
        }
      if(is[k]!=k)
        for(j=0;j<=n-1;j++)
          {
             u=k*n+j;v=is[k]*n+j;
             p=b[u];b[u]=b[v];b[v]=p;
          }
      if(js[k]!=k)
        for(i=0;i<=n-1;i++)
          {
            u=i*n+k;
            v=i*n+js[k];
            p=b[u];
            b[u]=b[v];b[v]=p;
          }
      l=k*n+k;
      b[l]=1.0/b[l];
      for(j=0;j<=n-1;j++)
        if(j!=k)
         {
            u=k*n+j;
            b[u]=b[u]*b[l];
         }
      for(i=0;i<=n-1;i++)
        if(i!=k)
          for(j=0;j<=n-1;j++)
            if(j!=k)
             {
               u=i*n+j;
               b[u]=b[u]-b[i*n+k]*b[k*n+j];
             }
      for(i=0;i<=n-1;i++)
         if(i!=k)
           {
             u=i*n+k;b[u]=-b[u]*b[l];
           }
    }     
    for(k=n-1;k>=0;k--)
      {
        if(js[k]!=k)
          for(j=0;j<=n-1;j++)
            {
               u=k*n+j;v=js[k]*n+j;
               p=b[u];
               b[u]=b[v];
               b[v]=p;
            }  
        if(is[k]!=k)
          for(i=0;i<=n-1;i++)
            {
               u=i*n+k;
			   v=i*n+is[k];	
               p=b[u];
			   b[u]=b[v];b[v]=p;
            }           
      }

     free(is);
     free(js);
     return(1);
}

double Mrem(double *a,int i,int j,int n)        
{      
	int k,m;      
	double  *pTemp = new double[(n-1)*(n-1)];        

	for(k=0;k<i;k++)      
	{      
		for(m=0;m<j;m++) 
		{
			pTemp[k*(n-1)+m] = a[k*n+m]; 
		}
		for(m=j;m<n-1;m++)  
		{
			pTemp[k*(n-1)+m] = a[k*n+m+1]; 
		}
	}      
	for(k=i;k<n-1;k++)      
	{      
		for(m=0;m<j;m++)
		{
			pTemp[k*(n-1)+m]=a[(k+1)*n+m];
		}
		for(m=j;m<n-1;m++)  
		{
			pTemp[k*(n-1)+m]=a[(k+1)*n+m+1];  
		}
	}      
	double  dResult = (((i+j)%2==1)?-1:1)*Mdet(pTemp,n-1);      
	delete[] pTemp;      
	return  dResult;      
} 

double Mdet(double *a,int n)   //求行列式的值    
{      
	if(n==1) 
	{
		return a[0];     
	}
	double sum=0;      
	for(int j=0;j<n;j++)
	{
		sum+=a[0*n+j]*Mrem(a,0,j,n); 
	}
	return sum;      
}  

void Mequalm(double *M,int m,int n,double *N)
{
   for (int i=0;i<m;i++)
   {
	   for (int j=0;j<n;j++)
	   {
		   *(N+i*n+j) = *(M+i*n+j);
	   }
   }
}

void Mequal(double *M,int m,int n,double a)
{
	   for (int i=0;i<m;i++)
   {
	   for (int j=0;j<n;j++)
	   {
		   *(M+i*n+j) = a;
	   }
   }
}

double Mmean(double *a,int m)
{
	double sum=0;
	for (int i=0;i<m;i++)
	{
		sum=sum+a[i];
	}
	return sum/m;
} 

int eejcb(double a[],int n,double v[],double eps,int jt) 
{ 
	int i,j,p=0,q=0,u,w,t,s,l; 
	double fm,cn,sn,omega,x,y,d; 
	l=1; 
	for (i=0; i<=n-1; i++) 
	{ 
		v[i*n+i]=1.0; 
		for (j=0; j<=n-1; j++) 
		{ 
			if (i!=j) 
			{ v[i*n+j]=0.0; } 
		} 
	} 
	while (1==1) 
	{ 
		fm=0.0; 
		for (i=0; i<=n-1; i++) 
		{ 
			for (j=0; j<=n-1; j++) 
			{ 
				d=fabs(a[i*n+j]); 
				if ((i!=j)&&(d>fm)) 
				{ 
					fm=d; 
					p=i; 
					q=j; 
				} 
			} 
		} 
		if (fm<eps) 
		{ return(1); } 
		if (l>jt) 
		{ return(-1); } 
		l=l+1; 
		u=p*n+q; 
		w=p*n+p; 
		t=q*n+p; 
		s=q*n+q; 
		x=-a[u]; 
		y=(a[s]-a[w])/2.0; 
		omega=x/sqrt(x*x+y*y); 
		if (y<0.0) 
		{ omega=-omega; } 
		sn=1.0+sqrt(1.0-omega*omega); 
		sn=omega/sqrt(2.0*sn); 
		cn=sqrt(1.0-sn*sn); 
		fm=a[w]; 
		a[w]=fm*cn*cn+a[s]*sn*sn+a[u]*omega; 
		a[s]=fm*sn*sn+a[s]*cn*cn-a[u]*omega; 
		a[u]=0.0; 
		a[t]=0.0; 
		for (j=0; j<=n-1; j++) 
		{ 
			if ((j!=p)&&(j!=q)) 
			{ 
				u=p*n+j; 
				w=q*n+j; 
				fm=a[u]; 
				a[u]=fm*cn+a[w]*sn; 
				a[w]=-fm*sn+a[w]*cn; 
			} 
		} 
		for (i=0; i<=n-1; i++) 
		{ 
			if ((i!=p)&&(i!=q)) 
			{ 
				u=i*n+p; 
				w=i*n+q; 
				fm=a[u]; 
				a[u]=fm*cn+a[w]*sn; 
				a[w]=-fm*sn+a[w]*cn; 
			} 
		} 
		for (i=0; i<=n-1; i++) 
		{ 
			u=i*n+p; 
			w=i*n+q; 
			fm=v[u]; 
			v[u]=fm*cn+v[w]*sn; 
			v[w]=-fm*sn+v[w]*cn; 
		} 
	} 
	return(1); 
}

void Munit(double* a,int n)
{
	for(int i=0;i<n;i++)
	{
		for(int j=0;j<n;j++)
		{a[i*n+j]=0.0;}
	}
	for(int i=0;i<n;i++)
	{a[i*n+i]=1.0;}
}

void askew(double v[], double m[])
{
	m[0*3+0]=0.0;   m[0*3+1]=-v[2]; m[0*3+2]=v[1];
	m[1*3+0]=v[2];  m[1*3+1]=0.0;   m[1*3+2]=-v[0];
	m[2*3+0]=-v[1]; m[2*3+1]=v[0];  m[2*3+2]=0.0;
}

void iaskew(double m[], double v[])
{
	v[0]=(m[2*3+1]-m[1*3+2])*0.5;
	v[1]=(m[0*3+2]-m[2*3+0])*0.5;
	v[2]=(m[1*3+0]-m[0*3+1])*0.5;
}

void q2rv(double q[],double rv[])
{
	double Q[4]={0.0};
	Q[0]=q[0];
	Q[1]=q[1];
	Q[2]=q[2];
	Q[3]=q[3];
	if(Q[0]<0)
	{
		Q[0]=-Q[0];
	    Q[1]=-Q[1];
		Q[2]=-Q[2];
	    Q[3]=-Q[3];
	}
	double n2,k;
	n2=acos(Q[0]);
	if(n2>1e-40)
		k=2*n2/sin(n2);
	else
		k=2;
	rv[0]=k*Q[1];
	rv[1]=k*Q[2];
	rv[2]=k*Q[3];
}

void rv2q(double rv[],double q[])
{
	double temp,f;
	temp=sqrt(rv[0]*rv[0]+rv[1]*rv[1]+rv[2]*rv[2]);
	if(temp>1.0e-40)
		f=sin(temp/2)/temp;
	else
		f=0.5;
	q[0]=cos(temp/2);
	q[1]=f*rv[0];
	q[2]=f*rv[1];
	q[3]=f*rv[2];
}

void rv2m(double rv[],double m[])
{
	double xx,yy,zz,n2,n;
	xx=rv[0]*rv[0]; yy=rv[1]*rv[1]; zz=rv[2]*rv[2];
	n2=xx+yy+zz;
	double a,b;
	if(n2<1.0e-8)
	{
		a=1-n2*(1.0/6.0-n2/120.0);
		b=0.5-n2*(1.0/24.0-n2/72.0);
	}
	else
	{
		n=sqrt(n2);
		a=sin(n)/n;
		b=(1-cos(n))/n2;
	}
	double arvx,arvy,arvz,bxx,bxy,bxz,byy,byz,bzz;
	arvx=a*rv[0]; arvy=a*rv[1]; arvz=a*rv[2];
	bxx=b*xx; bxy=b*rv[0]*rv[1]; bxz=b*rv[0]*rv[2];
	byy=b*yy; byz=b*rv[1]*rv[2]; bzz=b*zz;

	m[0*3+0]=1-byy-bzz; m[0*3+1]=-arvz+bxy; m[0*3+2]=arvy+bxz;
	m[1*3+0]=arvz+bxy;  m[1*3+1]=1-bxx-bzz; m[1*3+2]=-arvx+byz;
	m[2*3+0]=-arvy+bxz; m[2*3+1]=arvx+byz;  m[2*3+2]=1-bxx-byy;
}

void rotv(double rv[],double vi[],double vo[])
{
	double n2,n,n_2,q1,s;
	n2=rv[0]*rv[0]+rv[1]*rv[1]+rv[2]*rv[2];
	if(n2<1.0e-8)
	{
		q1=1-n2*(1.0/8.0-n2/384.0);
		s=0.5-n2*(1.0/48.0-n2/3840.0);
	}
	else
	{
		n=sqrt(n2);
		n_2=n/2.0;
		q1=cos(n_2);
		s=sin(n_2)/n;
	}
	double q2,q3,q4,qo1,qo2,qo3,qo4;
	q2 = s*rv[0]; q3 = s*rv[1]; q4 = s*rv[2];
    qo1 =            - q2 * vi[0] - q3 * vi[1] - q4 * vi[2];
    qo2 = q1 * vi[0]              + q3 * vi[2] - q4 * vi[1];
    qo3 = q1 * vi[1]              + q4 * vi[0] - q2 * vi[2];
    qo4 = q1 * vi[2]              + q2 * vi[1] - q3 * vi[0];

	vo[0] = -qo1 * q2 + qo2 * q1 - qo3 * q4 + qo4 * q3;
    vo[1] = -qo1 * q3 + qo3 * q1 - qo4 * q2 + qo2 * q4;
    vo[2] = -qo1 * q4 + qo4 * q1 - qo2 * q3 + qo3 * q2;
}

void qconj(double q[],double qc[])
{
	qc[0]=q[0];
	qc[1]=-q[1];
	qc[2]=-q[2];
	qc[3]=-q[3];
}

void qmuln(double q1[],double q2[],double q[])
{
	q[0]=q1[0]*q2[0]-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3];
	q[1]=q1[0]*q2[1]+q1[1]*q2[0]+q1[2]*q2[3]-q1[3]*q2[2];
	q[2]=q1[0]*q2[2]+q1[2]*q2[0]+q1[3]*q2[1]-q1[1]*q2[3];
	q[3]=q1[0]*q2[3]+q1[3]*q2[0]+q1[1]*q2[2]-q1[2]*q2[1];
}

void qmul( double q1[], double q2[])
{
	double q[4]={0.0};
	for(int i=0;i<4;i++)
	{q[i]=q1[i];}

	q1[0]=q[0]*q2[0]-q[1]*q2[1]-q[2]*q2[2]-q[3]*q2[3];
	q1[1]=q[0]*q2[1]+q[1]*q2[0]+q[2]*q2[3]-q[3]*q2[2];
	q1[2]=q[0]*q2[2]+q[2]*q2[0]+q[3]*q2[1]-q[1]*q2[3];
	q1[3]=q[0]*q2[3]+q[3]*q2[0]+q[1]*q2[2]-q[2]*q2[1];
	
}

void m2qua_ned(double m[],double q[])
{
	double s[5]={0.0};
	s[4]=m[0*3+0]+m[1*3+1]+m[2*3+2];
	s[0]=1.0+s[4];
	s[1]=1.0+2.0*m[0*3+0]-s[4];
	s[2]=1.0+2.0*m[1*3+1]-s[4];
	s[3]=1.0+2.0*m[2*3+2]-s[4];
	int index=0;
	double max=s[0];
	for(int k=1;k<4;k++)
	{
		if(s[k]>max)
		{
			index=k;
			max=s[k];
		}
	}
switch(index)
	{
	case 0:
		q[0]=0.5*sqrt(s[0]);
		q[1]=0.25*(m[2*3+1]-m[1*3+2])/q[0];
		q[2]=0.25*(m[0*3+2]-m[2*3+0])/q[0];
		q[3]=0.25*(m[1*3+0]-m[0*3+1])/q[0];
		break;
	case 1:
		q[1]=0.5*sqrt(s[1]);
		q[2]=0.25*(m[1*3+0]+m[0*3+1])/q[1];
		q[3]=0.25*(m[0*3+2]+m[2*3+0])/q[1];
		q[0]=0.25*(m[2*3+1]-m[1*3+2])/q[1];
		break;
	case 2:
		q[2]=0.5*sqrt(s[2]);
		q[3]=0.25*(m[2*3+1]+m[1*3+2])/q[2];
		q[0]=0.25*(m[0*3+2]-m[2*3+0])/q[2];
		q[1]=0.25*(m[1*3+0]+m[0*3+1])/q[2];
		break;
	case 3:
		q[3]=0.5*sqrt(s[3]);
		q[0]=0.25*(m[1*3+0]-m[0*3+1])/q[3];
		q[1]=0.25*(m[0*3+2]+m[2*3+0])/q[3];
		q[2]=0.25*(m[2*3+1]+m[1*3+2])/q[3];
		break;
	}
}

void q2mat_ned(double qua[],double m[])
{
	double q11,q12,q13,q14,q22,q23,q24,q33,q34,q44;
	q11=qua[0]*qua[0]; q12=qua[0]*qua[1]; q13=qua[0]*qua[2]; q14=qua[0]*qua[3];
	q22=qua[1]*qua[1]; q23=qua[1]*qua[2]; q24=qua[1]*qua[3];
	q33=qua[2]*qua[2]; q34=qua[2]*qua[3];
	q44=qua[3]*qua[3];
	m[0*3+0]=q11+q22-q33-q44; m[0*3+1]=2*(q23-q14);     m[0*3+2]=2*(q24+q13);
	m[1*3+0]=2*(q23+q14);     m[1*3+1]=q11-q22+q33-q44; m[1*3+2]=2*(q34-q12);
	m[2*3+0]=2*(q24-q13);     m[2*3+1]=2*(q34+q12);     m[2*3+2]=q11-q22-q33+q44;
}

void m2att_ned(double m[],double a[])
{
	a[0]=atan2(m[2*3+1],m[2*3+2]);
	a[1]=asin(-m[2*3+0]);
	a[2]=atan2(m[1*3+0],m[0*3+0]);
}

void a2mat_ned(double att[],double m[])
{
	double phi=att[0];
	double theta=att[1];
	double psi=att[2];
	double cpsi=cos(psi); double spsi=sin(psi);
	double cthe=cos(theta); double sthe=sin(theta);
	double cphi=cos(phi); double sphi=sin(phi);
	double C1[9]={0.0};
	double C2[9]={0.0};
	double C3[9]={0.0};
	double m1[9]={0.0};
	double m2[9]={0.0};
	C1[0*3+0]=cpsi;  C1[0*3+1]=spsi;
	C1[1*3+0]=-spsi; C1[1*3+1]=cpsi;
	                                 C1[2*3+2]=1.0;
	C2[0*3+0]=cthe;                  C2[0*3+2]=-sthe;
	                 C2[1*3+1]=1.0;
	C2[2*3+0]=sthe;                  C2[2*3+2]=cthe;
	C3[0*3+0]=1.0;
	                 C3[1*3+1]=cphi; C3[1*3+2]=sphi;
	                 C3[2*3+1]=-sphi;C3[2*3+2]=cphi;
	Mmulnm(C3,C2,3,3,3,m1);
	Mmulnm(m1,C1,3,3,3,m2);
	Mtn(m2,3,3,m);
}

/*dsf90:旋转顺序Z,X,Y*/
void a2mat(double att[],double m[])
{
	double si,sj,sk,ci,cj,ck;
	si=sin(att[0]); sj=sin(att[1]); sk=sin(att[2]);
	ci=cos(att[0]); cj=cos(att[1]); ck=cos(att[2]);
    m[0*3+0]=cj*ck-si*sj*sk;
	m[0*3+1]=-ci*sk;
	m[0*3+2]=sj*ck+si*cj*sk;
	m[1*3+0]=cj*sk+si*sj*ck;
	m[1*3+1]=ci*ck;
	m[1*3+2]=sj*sk-si*cj*ck;
	m[2*3+0]=-ci*sj;
	m[2*3+1]=si;
	m[2*3+2]=ci*cj;
}
/*******************************
功能：欧拉角转四元数
输入：att[3] 三轴欧拉角度（姿态角），单位rad(弧度)
输出：qua[4] 四元数
返回：无
********************************/
void a2qua(double att[],double qua[])
{
	double att2[3]={0.0};
	att2[0]=att[0]*0.5;
	att2[1]=att[1]*0.5;
	att2[2]=att[2]*0.5;

	double sp,sr,sy,cp,cr,cy;
	sp=sin(att2[0]); sr=sin(att2[1]); sy=sin(att2[2]);
	cp=cos(att2[0]); cr=cos(att2[1]); cy=cos(att2[2]);

	qua[0]=cp*cr*cy - sp*sr*sy;
	qua[1]=sp*cr*cy - cp*sr*sy;
	qua[2]=cp*sr*cy + sp*cr*sy;
	qua[3]=cp*cr*sy + sp*sr*cy;
}

void m2att(double mat[],double att[])
{
	att[0]=asin(mat[2*3+1]);
	att[1]=atan2(-mat[2*3+0],mat[2*3+2]);
	att[2]=atan2(-mat[0*3+1],mat[1*3+1]);
}

void m2qua(double m[],double qua[])
{
	qua[0]=1+m[0*3+0]+m[1*3+1]+m[2*3+2];
	qua[1]=1+m[0*3+0]-m[1*3+1]-m[2*3+2];
	qua[2]=1-m[0*3+0]+m[1*3+1]-m[2*3+2];
	qua[3]=1-m[0*3+0]-m[1*3+1]+m[2*3+2];
	
	int sign[4];
	sign[0]=1;
	if ((m[2*3+1]-m[1*3+2])>=0)
	{sign[1]=1;}
	else 
		{sign[1]=-1;}
	if ((m[0*3+2]-m[2*3+0])>=0)
	{sign[2]=1;}
	else 
		{sign[2]=-1;}
	if ((m[1*3+0]-m[0*3+1])>=0)
	{sign[3]=1;}
	else 
		{sign[3]=-1;}

	qua[0]=sign[0]*sqrt(fabs(qua[0]))/2;
	qua[1]=sign[1]*sqrt(fabs(qua[1]))/2;
	qua[2]=sign[2]*sqrt(fabs(qua[2]))/2;
	qua[3]=sign[3]*sqrt(fabs(qua[3]))/2;
}

void q2att(double qua[],double att[])
{
	double q11,q12,q13,q14,q22,q23,q24,q33,q34,q44;
	double c12,c22,c31,c32,c33;
	q11=qua[0]*qua[0]; q12=qua[0]*qua[1]; q13=qua[0]*qua[2]; q14=qua[0]*qua[3];
	q22=qua[1]*qua[1]; q23=qua[1]*qua[2]; q24=qua[1]*qua[3];
	q33=qua[2]*qua[2]; q34=qua[2]*qua[3];
	q44=qua[3]*qua[3];
	c12=2*(q23-q14);
	c22=q11-q22+q33-q44;
	c31=2*(q24-q13); c32=2*(q34+q12); c33=q11-q22-q33+q44;
	att[0]=asin(c32);
	att[1]=atan2(-c31,c33);
	att[2]=atan2(-c12,c22);
}

void q2mat(double qua[],double mat[])
{
	double q11,q12,q13,q14,q22,q23,q24,q33,q34,q44;
	q11=qua[0]*qua[0]; q12=qua[0]*qua[1]; q13=qua[0]*qua[2]; q14=qua[0]*qua[3];
	q22=qua[1]*qua[1]; q23=qua[1]*qua[2]; q24=qua[1]*qua[3];
	q33=qua[2]*qua[2]; q34=qua[2]*qua[3];
	q44=qua[3]*qua[3];
	mat[0*3+0]=q11+q22-q33-q44; mat[0*3+1]=2*(q23-q14); mat[0*3+2]=2*(q24+q13);
	mat[1*3+0]=2*(q23+q14); mat[1*3+1]=q11-q22+q33-q44; mat[1*3+2]=2*(q34-q12);
	mat[2*3+0]=2*(q24-q13); mat[2*3+1]=2*(q34+q12); mat[2*3+2]=q11-q22-q33+q44;
}

void m2rv(double mat[],double rv[])
{
	double rvx[3*3]={0.0};
	double eye[3*3]={0.0};
	double rvxrvx[3*3]={0.0};
	double temp=0.0;
	Munit(eye,3);
	double n,n2;
	double qua[4]={0.0};
	m2qua(mat,qua);
	q2rv(qua,rv);
    askew(rv,rvx);
	for(int k=0;k<2;k++)
	{
		temp=rv[0]*rv[0]+rv[1]*rv[1]+rv[2]*rv[2];
		n2=temp;
		if(n2>1.0e-40)
		{
			n=sqrt(n2);
			Mmulnm(rvx,rvx,3,3,3,rvxrvx);
			temp=(1-cos(n))/n2;
			Mmul(rvxrvx,3,3,temp);
			Mmin(mat,eye,3,3);
			Mmin(mat,rvxrvx,3,3);
			temp=n/sin(n);
			Mmuln(mat,3,3,temp,rvx);
			iaskew(rvx,rv);
		}
		else
		{
			for(int i=0;i<3;i++)
			{ rv[i]=0.0;}
			break;
		}
	}
}

void qmulv(double q[],double vi[],double vo[])
{
	double qo1,qo2,qo3,qo4; 
    qo1 =              - q[1] * vi[0] - q[2] * vi[1] - q[3] * vi[2];
    qo2 = q[0] * vi[0]                + q[2] * vi[2] - q[3] * vi[1];
    qo3 = q[0] * vi[1]                + q[3] * vi[0] - q[1] * vi[2];
    qo4 = q[0] * vi[2]                + q[1] * vi[1] - q[2] * vi[0];
    
    vo[0] = -qo1 * q[1] + qo2 * q[0] - qo3 * q[3] + qo4 * q[2];
    vo[1] = -qo1 * q[2] + qo3 * q[0] - qo4 * q[1] + qo2 * q[3];
    vo[2] = -qo1 * q[3] + qo4 * q[0] - qo2 * q[2] + qo3 * q[1];
}

void qdelphi(double qpb[],double phi[])
{
	double q0[4]={0.0};
	double qnb[4]={0.0};
	rv2q(phi,q0);
	qmuln(q0,qpb,qnb);
	Mequalm(qnb,4,1,qpb);
}

void qupdt(double qnb0[],double rv_nb[],double qnb1[])
{
	double n2,c,s,n,n_2,nq;
	   n2 = rv_nb[0]*rv_nb[0]+rv_nb[1]*rv_nb[1]+rv_nb[2]*rv_nb[2];
	if (n2<1.0e-8)
	{
		c = 1.0-n2*(1.0/8.0-n2/384.0); 
		s = 1.0/2-n2*(1.0/48.0-n2/3840.0);
	}
	else
	{
		n = sqrt(n2); n_2 = n/2;
        c = cos(n_2); s = sin(n_2)/n;
	}
	double q2[4]={0.0};
	q2[0]=c; q2[1]=s*rv_nb[0]; q2[2]=s*rv_nb[1]; q2[3]=s*rv_nb[2]; 
    // q = qmul(q1, q2);
    qnb1[0] = qnb0[0] * q2[0] - qnb0[1] * q2[1] - qnb0[2] * q2[2] - qnb0[3] * q2[3];
    qnb1[1] = qnb0[0] * q2[1] + qnb0[1] * q2[0] + qnb0[2] * q2[3] - qnb0[3] * q2[2];
    qnb1[2] = qnb0[0] * q2[2] + qnb0[2] * q2[0] + qnb0[3] * q2[1] - qnb0[1] * q2[3];
    qnb1[3] = qnb0[0] * q2[3] + qnb0[3] * q2[0] + qnb0[1] * q2[2] - qnb0[2] * q2[1];
    // normalization
    n2 = qnb1[0]*qnb1[0]+qnb1[1]*qnb1[1]+qnb1[2]*qnb1[2]+qnb1[3]*qnb1[3];
    if (n2>1.000001 || n2<0.999999)
	{
		nq = 1.0/sqrt(n2); 
        qnb1[0] = qnb1[0]*nq; qnb1[1] = qnb1[1]*nq; qnb1[2] = qnb1[2]*nq; qnb1[3] = qnb1[3]*nq;
	}
}

void qupdt2(double qnb0[],double rv_ib[],double rv_in[],double qnb1[])
{
	double n2,n,n_2,rv_ib0,s,qb1,qb2,qb3,qb4,rv_in0,nq;
	n2 = rv_ib[0]*rv_ib[0]+rv_ib[1]*rv_ib[1]+rv_ib[2]*rv_ib[2];
    if (n2<1.0e-8)
	{
		rv_ib0 = 1-n2*(1.0/8.0-n2/384.0);
	    s = 0.5-n2*(1.0/48.0-n2/3840.0);
	}
    else
	{
		n = sqrt(n2); n_2 = n/2.0;
        rv_ib0 = cos(n_2); s = sin(n_2)/n;
	}
    rv_ib[0] = s*rv_ib[0]; rv_ib[1] = s*rv_ib[1]; rv_ib[2] = s*rv_ib[2];
    // qnb1 = qmul(qnb0, q);
    qb1 = qnb0[0] * rv_ib0   - qnb0[1] * rv_ib[0] - qnb0[2] * rv_ib[1] - qnb0[3] * rv_ib[2];
    qb2 = qnb0[0] * rv_ib[0] + qnb0[1] * rv_ib0   + qnb0[2] * rv_ib[2] - qnb0[3] * rv_ib[1];
    qb3 = qnb0[0] * rv_ib[1] + qnb0[2] * rv_ib0   + qnb0[3] * rv_ib[0] - qnb0[1] * rv_ib[2];
    qb4 = qnb0[0] * rv_ib[2] + qnb0[3] * rv_ib0   + qnb0[1] * rv_ib[1] - qnb0[2] * rv_ib[0];
    // rv2q(-rv_in)
    n2 = rv_in[0]*rv_in[0]+rv_in[1]*rv_in[1]+rv_in[2]*rv_in[2];
    if (n2<1.0e-8)
	{
	    rv_in0 = 1-n2*(1.0/8.0-n2/384.0); s = -0.5+n2*(1.0/48.0-n2/3840.0);
	}
    else
	{
		n = sqrt(n2); n_2 = n/2.0;
        rv_in0 = cos(n_2); s = -sin(n_2)/n;
	}  
    rv_in[0] = s*rv_in[0]; rv_in[1] = s*rv_in[1]; rv_in[2] = s*rv_in[2]; 
    // qnb1 = qmul(q, qnb1);  
    qnb1[0] = rv_in0 * qb1 - rv_in[0] * qb2 - rv_in[1] * qb3 - rv_in[2] * qb4;
    qnb1[1] = rv_in0 * qb2 + rv_in[0] * qb1 + rv_in[1] * qb4 - rv_in[2] * qb3;
    qnb1[2] = rv_in0 * qb3 + rv_in[1] * qb1 + rv_in[2] * qb2 - rv_in[0] * qb4;
    qnb1[3] = rv_in0 * qb4 + rv_in[2] * qb1 + rv_in[0] * qb3 - rv_in[1] * qb2;
    //normalization
    n2 = qnb1[0]*qnb1[0]+qnb1[1]*qnb1[1]+qnb1[2]*qnb1[2]+qnb1[3]*qnb1[3];
    if (n2>1.000001 || n2<0.999999)
	{
		nq = 1.0/sqrt(n2); 
        qnb1[0] = qnb1[0]*nq; qnb1[1] = qnb1[1]*nq; qnb1[2] = qnb1[2]*nq; qnb1[3] = qnb1[3]*nq;
	}
}
/**
* @brief Rotate a vector by a rotation matrix
* @param[in] R a three by three rotation matrix (first index is row)
* @param[in] vec the source vector
* @param[out] vec_out the output vector
*/
void rot_mult(double R[3][3], const double vec[3], double vec_out[3])
{
	vec_out[0] = R[0][0] * vec[0] + R[0][1] * vec[1] + R[0][2] * vec[2];
	vec_out[1] = R[1][0] * vec[0] + R[1][1] * vec[1] + R[1][2] * vec[2];
	vec_out[2] = R[2][0] * vec[0] + R[2][1] * vec[1] + R[2][2] * vec[2];
}

/*CnbDotAtt or CbsDotAtt*/
void a2mat_dotAtt(double att[3], double C[3][3][3])
{
	double si,sj,sk,ci,cj,ck;
	/*pitch,roll,yaw*/
	si=sin(att[0]); sj=sin(att[1]); sk=sin(att[2]);
	ci=cos(att[0]); cj=cos(att[1]); ck=cos(att[2]);
	/*Cnb
    m[0*3+0]= cj*ck-si*sj*sk;	m[0*3+1]=-ci*sk;	m[0*3+2]=sj*ck+si*cj*sk;
	m[1*3+0]= cj*sk+si*sj*ck;	m[1*3+1]= ci*ck;	m[1*3+2]=sj*sk-si*cj*ck;
	m[2*3+0]=-ci*sj;	        m[2*3+1]= si;	    m[2*3+2]=ci*cj;
	*/

	/*dC/dpitch*/
	C[0][0][0] = -ci*sj*sk;   C[0][0][1] =  si*sk;	        C[0][0][2] = ci*cj*sk;
	C[0][1][0] =  ci*sj*ck;   C[0][1][1] = -si*ck;       	C[0][1][2] =-ci*cj*ck;
	C[0][2][0] =  si*sj;      C[0][2][1] =  ci;             C[0][2][2] =-si*cj;

	/*dC/droll*/
	C[1][0][0] = -sj*ck-si*cj*sk;   C[1][0][1] = 0;	        C[1][0][2] = cj*ck-si*sj*sk;
	C[1][1][0] = -sj*sk+si*cj*ck;   C[1][1][1] = 0;       	C[1][1][2] = cj*sk+si*sj*ck;
	C[1][2][0] = -ci*cj;            C[1][2][1] = 0;         C[1][2][2] =-ci*sj;
	
	/*dC/dyaw*/
	C[2][0][0] =-cj*sk-si*sj*ck;    C[2][0][1] = -ci*ck;	C[2][0][2] =-sj*sk+si*cj*ck;
	C[2][1][0] = cj*ck-si*sj*sk;    C[2][1][1] = -ci*sk;    C[2][1][2] = sj*ck+si*cj*sk;
	C[2][2][0] = 0;                 C[2][2][1] = 0;         C[2][2][2] = 0;
}

/*dCnb/dotAtt X U*/
void CnbDotPRY_mul_u(double PRY[3], double U[3], double out[3][3])
{
	double dotAtt[3][3][3];
	double fTmp[3];
	int i;

	a2mat_dotAtt(PRY, dotAtt);

	for(i=0;i<3;i++)
	{
		rot_mult(dotAtt[i], U, fTmp);
		out[0][i] = fTmp[0];
		out[1][i] = fTmp[1];
		out[2][i] = fTmp[2];
	}	
}

/*dCbn/dotAtt X U*/
void CbnDotPRY_mul_u(double PRY[3], double U[3], double out[3][3])
{
	double dotAtt[3][3][3];
	double fTmp[3],C[3][3];
	int i;

	a2mat_dotAtt(PRY, dotAtt);
	
	for(i=0;i<3;i++)
	{
		Mtn((double *)dotAtt[i],3,3,(double *)C);
		rot_mult(C, U, fTmp);
		out[0][i] = fTmp[0];
		out[1][i] = fTmp[1];
		out[2][i] = fTmp[2];
	}
}

/*dCnbdot/dq*/
void q2mat_dotQ(double q[4],double out[4][3][3])
{
	/*dCnbdot/dq0*/ 
	out[0][0][0] =	2*q[0]; 	out[0][0][1] = -2*q[3]; 	out[0][0][2] =	2*q[2];
	out[0][1][0] =	2*q[3]; 	out[0][1][1] =	2*q[0]; 	out[0][1][2] = -2*q[1];
	out[0][2][0] = -2*q[2]; 	out[0][2][1] =	2*q[1]; 	out[0][2][2] =	2*q[0];
	
	/*dCnbdot/dq1*/ 
	out[1][0][0] =	2*q[1]; 	out[1][0][1] =	2*q[2]; 	out[1][0][2] =	2*q[3];
	out[1][1][0] =	2*q[2]; 	out[1][1][1] = -2*q[1]; 	out[1][1][2] = -2*q[0];
	out[1][2][0] =	2*q[3]; 	out[1][2][1] =	2*q[0]; 	out[1][2][2] = -2*q[1];
	
	/*dCnbdot/dq2*/ 
	out[2][0][0] = -2*q[2]; 	out[2][0][1] =	2*q[1]; 	out[2][0][2] =	2*q[0];
	out[2][1][0] =	2*q[1]; 	out[2][1][1] =	2*q[2]; 	out[2][1][2] =	2*q[3];
	out[2][2][0] = -2*q[0]; 	out[2][2][1] =	2*q[3]; 	out[2][2][2] = -2*q[2];
	
	/*dCnbdot/dq3*/ 
	out[3][0][0] = -2*q[3]; 	out[3][0][1] = -2*q[0]; 	out[3][0][2] =	2*q[1];
	out[3][1][0] =	2*q[0]; 	out[3][1][1] = -2*q[3]; 	out[3][1][2] =	2*q[2];
	out[3][2][0] =	2*q[1]; 	out[3][2][1] =	2*q[2]; 	out[3][2][2] =	2*q[3];
}

/*dCnb/dq(Qnb)*/
void CnbDotQ_mul_u(double Qnb[4],double U[3],double out[3][4])
{
	double dotQ[4][3][3];
	double fTmp[3];
	int i;

	q2mat_dotQ(Qnb,dotQ);

	for(i=0;i<4;i++)
	{
		rot_mult(dotQ[i], U, fTmp);
		out[0][i] = fTmp[0];
		out[1][i] = fTmp[1];
		out[2][i] = fTmp[2];
	}

}

/*dCbn/dq(Qnb)*/
void CbnDotQ_mul_u(double Qnb[4],double U[3],double out[3][4])
{
	double dotQ[4][3][3];
	double fTmp[3],C[3][3];
	int i;

	q2mat_dotQ(Qnb,dotQ);

	for(i=0;i<4;i++)
	{
		Mtn((double *)dotQ[i],3,3,(double *)C);
		rot_mult(C, U, fTmp);
		out[0][i] = fTmp[0];
		out[1][i] = fTmp[1];
		out[2][i] = fTmp[2];
	}
}

/*-----------------log define by DHF ----------------------*/
static FILE *fpoutdhf=NULL; 
extern void fopendhf(const char *file)
{
    if (!*file||!(fpoutdhf=fopen(file,"w"))) 
	{
		gilc_log("can not open outfile crated by dhf!");
		fpoutdhf=NULL;
	}
}
extern void fclosedhf(void)
{
    if (fpoutdhf) fclose(fpoutdhf);
    fpoutdhf=NULL;
}
extern void outdhf(const char *format, ...)
{
    va_list ap;
    
    if (!fpoutdhf) return;
    va_start(ap,format); vfprintf(fpoutdhf,format,ap); va_end(ap);
    fflush(fpoutdhf);
}

/*-----------Dynamic Identify by DHF,2016.10.12-------------*/
double ka[][10]={
	{0.00016, 0.02 , 0.115, 0.297, 0.554, 0.872, 1.239, 1.646, 2.088, 2.558},
	{0.001  , 0.051, 0.216, 0.484, 0.831, 1.237, 1.69 ,  2.18, 2.7  , 3.247},
	{0.004  , 0.103, 0.352, 0.711, 1.145, 1.635, 2.167, 2.733, 3.325, 3.94},
	{0.016  , 0.211, 0.584, 1.064, 1.61,  2.204, 2.833,  3.49, 4.168, 4.865}
};
extern bool sort_by_value(const double& val1,const double& val2)
{return val1<val2;}
DynamicIdentify::DynamicIdentify(int NW,int ND,int NKIN)
{
	bStaticStd=false;
	bKin=false;
	bStatic=true;
	bTurn=false;
	nkin=0;
	Nkin=NKIN;
	ndetect=0;
	Ndetect=ND;
	nwind=0;
	Nwind=NW;
	for (int i=0;i<3;i++)
	{
		acc_ave[i]=0.0;
		gyo_ave[i]=0.0;
		daccstd[i]=0.0;
		dgyostd[i]=0.0;
	}
	axstd=aystd=azstd=gxstd=gystd=gzstd=0.0;
	Ndetect1=7;Ndetect2=100;Ndetect3=100;
}
/*
DynamicIdentify& DynamicIdentify::operator=(const DynamicIdentify& dyni)
{
	bStaticStd=dyni.bStaticStd;
	bKin=dyni.bKin;
	bStatic=dyni.bStatic;
	bTurn=dyni.bTurn;
	nkin=dyni.nkin;
	Nkin=dyni.Nkin;
	ndetect=dyni.ndetect;
	Ndetect=dyni.Ndetect;
	nwind=dyni.nwind;
	Nwind=dyni.Nwind;
	for (int i=0;i<3;i++)
	{
		acc_ave[i]=dyni.acc_ave[i];
		gyo_ave[i]=dyni.gyo_ave[i];
		daccstd[i]=dyni.daccstd[i];
		dgyostd[i]=dyni.dgyostd[i];
	}
	axstd=dyni.axstd;
	aystd=dyni.aystd;
	azstd=dyni.azstd;
	gxstd=dyni.gxstd;
	gystd=dyni.gystd;
	gzstd=dyni.gzstd;
	Ndetect1=dyni.Ndetect1;
	Ndetect2=dyni.Ndetect2;
	Ndetect3=dyni.Ndetect3;
	ax.assign(dyni.ax.begin(), dyni.ax.end());
	ay.assign(dyni.ay.begin(), dyni.ay.end());
	az.assign(dyni.az.begin(), dyni.az.end());
	gx.assign(dyni.gx.begin(), dyni.gx.end());
	gy.assign(dyni.gy.begin(), dyni.gy.end());
	gz.assign(dyni.gz.begin(), dyni.gz.end());
	mx.assign(dyni.mx.begin(), dyni.mx.end());
	my.assign(dyni.my.begin(), dyni.my.end());
	mz.assign(dyni.mz.begin(), dyni.mz.end());
	Dax.assign(dyni.Dax.begin(), dyni.Dax.end());
	Day.assign(dyni.Day.begin(), dyni.Day.end());
	Daz.assign(dyni.Daz.begin(), dyni.Daz.end());
	Dgx.assign(dyni.Dgx.begin(), dyni.Dgx.end());
	Dgy.assign(dyni.Dgy.begin(), dyni.Dgy.end());
	Dgz.assign(dyni.Dgz.begin(), dyni.Dgz.end());
	Dax2.assign(dyni.Dax2.begin(), dyni.Dax2.end());
	Day2.assign(dyni.Day2.begin(), dyni.Day2.end());
	Dgz2.assign(dyni.Dgz2.begin(), dyni.Dgz2.end());
	return(*this);
}
*/
bool DynamicIdentify::Detect(double gyo[3],double acc[3],int option)
{
	if(ndetect<Ndetect)
	{
		Dax.push_back(acc[0]);
		Day.push_back(acc[1]);
		Daz.push_back(acc[2]);
		Dgx.push_back(gyo[0]);
		Dgy.push_back(gyo[1]);
		Dgz.push_back(gyo[2]);
		ndetect++;
	}
	else
	{
		Dax.erase(Dax.begin()); Dax.push_back(acc[0]);
		Day.erase(Day.begin()); Day.push_back(acc[1]);
		Daz.erase(Daz.begin()); Daz.push_back(acc[2]);
		Dgx.erase(Dgx.begin()); Dgx.push_back(gyo[0]);
		Dgy.erase(Dgy.begin()); Dgy.push_back(gyo[1]);
		Dgz.erase(Dgz.begin()); Dgz.push_back(gyo[2]);
	}
	if(nwind<Nwind)/*dsf90,风险:默认开机时静止*/
	{
		ax.push_back(acc[0]);
		ay.push_back(acc[1]);
		az.push_back(acc[2]);
		gx.push_back(gyo[0]);
		gy.push_back(gyo[1]);
		gz.push_back(gyo[2]);
		nwind++;
		return bKin;
	}
	if(nwind==Nwind)
	{
		// 标准差
		axstd=GetAveStd(ax,1); aystd=GetAveStd(ay,1); azstd=GetAveStd(az,1);
		gxstd=GetAveStd(gx,1); gystd=GetAveStd(gy,1); gzstd=GetAveStd(gz,1);
		nwind++;
		bStaticStd = true;
	}
	//判断是否开始运动 DHF 
    if(option==0) 
	{// 标准差
		daccstd[0]=GetAveStd(Dax,1); daccstd[1]=GetAveStd(Day,1); daccstd[2]=GetAveStd(Daz,1);
		dgyostd[0]=GetAveStd(Dgx,1); dgyostd[1]=GetAveStd(Dgy,1); dgyostd[2]=GetAveStd(Dgz,1);

		if(daccstd[0]>3*axstd&&daccstd[1]>3*aystd&&daccstd[2]>3*aystd&&
		   dgyostd[0]>3*gxstd&&dgyostd[1]>3*gystd&&dgyostd[2]>3*gystd)
		{
			nkin++;
			if(nkin<Nkin)
			{
				return bKin;
			}
			else
			{// 均值
				acc_ave[0]=GetAveStd(ax,0); acc_ave[1]=GetAveStd(ay,0); acc_ave[2]=GetAveStd(az,0);
				gyo_ave[0]=GetAveStd(gx,0); gyo_ave[1]=GetAveStd(gy,0); gyo_ave[2]=GetAveStd(gz,0);
				bKin=true;
				return bKin;
			}
		}
		else
		{
			nkin=0;
			return bKin;
		}
	}
	//运动过程中判断动静态 DHF 邵晓东20190330 调整Ndetect 相应的调整阈值dect
	//需重力垂直-PDR，不适用于上下坡
	if(option==1)
	{
		Ndetect1=7;
		unsigned int i=0;

		double avggyo=0.0,avac[3]={0.0},avgacc=0.0,noravacc=0.0,simggyo=0.0,simacc=0.0,dect=0.0;
		simggyo=(pow(gxstd,2)+pow(gystd,2)+pow(gzstd,2))/3.0;
		simacc =(pow(axstd,2)+pow(aystd,2)+pow(azstd,2))/3.0;
		for (i=0;i<Ndetect1;i++)
		{
			avggyo+=(pow(*(Dgx.end()-i-1),2)+pow(*(Dgy.end()-i-1),2)+pow(*(Dgz.end()-i-1),2))/simggyo;
			avac[0]+=*(Dax.end()-i-1); avac[1]+=*(Day.end()-i-1); avac[2]+=*(Daz.end()-i-1);
		}
		avggyo/=Ndetect1;
		avac[0]/=Ndetect1; avac[1]/=Ndetect1; avac[2]/=Ndetect1;
		noravacc=norm(avac,3);
		double gn=9.78;
		for (i=0;i<Ndetect1;i++)
		{ 
			avgacc+=(pow(*(Dax.end()-i-1)-gn*avac[0]/noravacc,2)
						+pow(*(Day.end()-i-1)-gn*avac[1]/noravacc,2)
						+pow(*(Daz.end()-i-1)-gn*avac[2]/noravacc,2))/simacc;
		}
		avgacc/=Ndetect1;
		dect=log(ka[2][Ndetect1-1]);
		dect=-(2.0/Ndetect1)*log(ka[2][Ndetect1-1]);
		dect=1500.0; //经验值
		if ( (avggyo+avgacc)<dect )
		{
			bStatic=true;   //检测到载体处于静止
		}
		else
		{
			bStatic=false;
		}
		return bStatic;
	}  
	//以std作为判断动静态的依据 DHF 窗口太长会耗时严重
	if(option==2)
	{
		unsigned int i=0;
		Ndetect2=50; // 100hz 1秒的窗
		if(Dax2.size()<Ndetect2)
		{
			Dax2.push_back(acc[0]);
			Day2.push_back(acc[1]);
			return bStatic;
		}
		else
		{
			Dax2.erase(Dax2.begin()); Dax2.push_back(acc[0]);
			Day2.erase(Day2.begin()); Day2.push_back(acc[1]);
		}
		vector<double, malloc_allocator<double> > stdax,stday;
		double aveax,aveay;
		aveax=GetAveStd(Dax2,0); aveay=GetAveStd(Day2,0);
		for(i=0;i<Dax2.size();i++)
		{
			stdax.push_back(pow(Dax2[i]-aveax,2));
			stday.push_back(pow(Day2[i]-aveay,2));
		}
		sort(stdax.begin(),stdax.end(),sort_by_value);
		sort(stday.begin(),stday.end(),sort_by_value);
		if(fabs(stdax[0]-stdax.back())<0.02 && fabs(stday[0]-stday.back())<0.02)
		{bStatic=true;}
		else
		{bStatic=false;}
		return bStatic;
	}
	//以Z轴陀螺的平均值判断是否转弯
	if(option==3)
	{
		Ndetect3=10;
		if(Dgz2.size()<Ndetect3)
		{
			Dgz2.push_back(gyo[2]);
			return bTurn;
		}
		else
		{
			Dgz2.erase(Dgz2.begin()); 
			Dgz2.push_back(gyo[2]);
			double avegz=GetAveStd(Dgz2,0);
			if(fabs(avegz)<0.15)
			{bTurn=false;}
			else
			{bTurn=true;}
			return bTurn;
		}
	}
	return bTurn;
}
