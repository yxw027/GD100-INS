#ifdef WIN32
#include "stdafx.h"
#endif
#include "GILC.h"
#include "filter.h"
#include "GILC_Config.h"
#include "GILC_KF.h"

CGLV glv;
InitPara InitParam;

#define DR_MODE_TEST   0
#define MUP_TEST_DSF90 1

CGLV::CGLV(double Re, double f, double wie0, double g0)
{
	this->Re = Re; this->f = f; this->wie = wie0; this->g0 = g0;
	g = glv_g;
	mg = 1.0e-3*g;
	ug = 1.0e-6*g;
	deg = PI/180.0;
	min = deg/60.0;
	sec = min/60.0;
	ppm = 1.0e-6;
	hur = 3600.0;
	dph = deg/hur;
	dpsh = deg/sqrt(hur);
	dphpsh = dph/sqrt(hur);
	ugpsHz = ug/sqrt(1.0);
	ugpsh = ug/sqrt(hur);
	mpsh = 1/sqrt(hur); 
	mpspsh = 1/1/sqrt(hur); 
	ppmpsh = ppm/sqrt(hur);
	secpsh = sec/sqrt(hur);
}

void PriPara::setAtt(double pitch,double roll,double yaw,int opt) //opt默认0--度
{
	switch (opt)
	{
	case 0:
		att0[0]=pitch*glv.deg;
		att0[1]=roll*glv.deg;
		att0[2]=yaw*glv.deg;
		break;
	case 1:
		att0[0]=pitch;
		att0[1]=roll;
		att0[2]=yaw;
		break;
	default :
		gilc_log("error:setatt option error\r\n");
		break;
	}
}

void PriPara::setVn(double vn[])
{
	for(int i=0;i<3;i++)
	{vn0[i]=vn[i];}
}

void PriPara::setPos(double pos[],int opt) //opt 0:lat,lon,hig(默认deg)  1: x y z
{
	for(int i=0;i<3;i++)
	{
		pos0[i]=pos[i];
	}

	switch(opt)
	{
	case 0:
		pos0[0]*=glv.deg;
		pos0[1]*=glv.deg;
		break;
	case 1:
		break;
	default :
		gilc_log("error:setpos option error\r\n");
		break;
	}
}

//var_gpv是方差 20170407 放大3倍
void PriPara::setdavp(double var_gpv[36],double var_yaw,double var_pr)
{
	/*dsf90:姿态att方差*/
	davp[0]=var_pr*glv.deg; 
	davp[1]=var_pr*glv.deg; 
	davp[2]=var_yaw*glv.deg;
	/*dsf90:速度方差*3*/
	if(var_gpv[0*6+0]!=0.0 && var_gpv[1*6+1]!=0.0 && var_gpv[2*6+2]!=0.0)
	{
		davp[3]=sqrt(var_gpv[0*6+0])*3; 
		davp[4]=sqrt(var_gpv[1*6+1])*3; 
		davp[5]=sqrt(var_gpv[2*6+2])*3;
	}
	else
	{
		davp[3]=davp[4]=davp[5]=0.1;
	}
	/*dsf90:位置方差*3*/
	if (var_gpv[3 * 6 + 3] != 0.0 && var_gpv[4 * 6 + 4] != 0.0 && var_gpv[5 * 6 + 5] != 0.0)
	{
		davp[6] = sqrt(var_gpv[3 * 6 + 3]) * 3;
		davp[7] = sqrt(var_gpv[4 * 6 + 4]) * 3;
		davp[8] = sqrt(var_gpv[5 * 6 + 5]) * 3;
	}
	else
	{
		davp[6] = davp[7] = davp[8] = 1;
	}
}

void PriPara::setimu(double avegyo[3],double accbias[3],double gw[3],double aw[3],double gs[3],double as[3],int scater)
{
	/*dsf90:偏差初始值设置*/
	for(int i=0;i<3;i++)
	{
		GB[i]=avegyo[i];              // rad/s
		AB[i]=accbias[i];      // m/s2
	}
	/*dsf90:偏差噪声设置*/
#if 1	//车载
	GW[0]=gw[0]*1;           // web - angular random walk (deg/sqrt(h))
	GW[1]=gw[1]*1;
	GW[2]=gw[2]*1;
	AW[0]=aw[0]*1;           // velocity random walk (ug/sqrt(Hz))
	AW[1]=aw[1]*1; 
	AW[2]=aw[2]*1; 
#endif
	/*dsf90:随机游走噪声设置*/
	for(int i=0;i<3;i++)
	{
		GS[i]=gs[i];		
		AS[i]=as[i];	  
	}
}

void PriPara::setpobs(double var_gpv[36])
{
	if (var_gpv[3 * 6 + 3] != 0.0 && var_gpv[4 * 6 + 4] != 0.0 && var_gpv[5 * 6 + 5] != 0.0)
	{
		prnObs[0] = sqrt(var_gpv[3 * 6 + 3]);
		prnObs[1] = sqrt(var_gpv[4 * 6 + 4]);
		prnObs[2] = sqrt(var_gpv[5 * 6 + 5]);
	}
	else
	{
		prnObs[0] = prnObs[1] = prnObs[2] = 1;
	}
}

void PriPara::setimus(double gs[9],double as[9])
{
	for(int i=0;i<9;i++)
	{
		GScater[i]=gs[i];
		AScater[i]=as[i];
	}
}

void PriPara::setlever(double lev[],double dt)
{
	for(int i=0;i<3;i++)
	{lever[i]=lev[i];}
	tDelay=dt;
}

CEarth::CEarth(double a0,double f0,double g0)
{
	a = a0;	f = f0; wie = glv.wie; 
	b = (1-f)*a;
	e = sqrt(a*a-b*b)/a;	e2 = e*e;
	Mequal(gn,3,1,0);
	gn[2]=-g0;
}
/*
CEarth& CEarth::operator=(const CEarth& eth)
{
	a=eth.a; 
	b=eth.b;
	f=eth.f;
	e=eth.e;
	e2=eth.e2; 
	wie=eth.wie; 
	sl=eth.sl;
	sl2=eth.sl2;
	sl4=eth.sl4;
	cl=eth.cl;
	tl=eth.tl;
	RMh=eth.RMh;
	RNh=eth.RNh; 
	clRNh=eth.clRNh;
	f_RMh=eth.f_RMh;
	f_RNh=eth.f_RNh;
	f_clRNh=eth.f_clRNh;
	for(int i=0;i<3;i++)
	{
		wnie[i]=eth.wnie[i];
		wnen[i]=eth.wnen[i];
		wnin[i]=eth.wnin[i];
		gn[i]=eth.gn[i];
		gcc[i]=eth.gcc[i];
	}
	return(*this);
}
*/
void CEarth::UpdateP(double pos[])
{
	sl=sin(pos[0]); cl=cos(pos[0]); tl=sl/cl;
	sl2=sl*sl; sl4=sl2*sl2;
	double sq=1-e2*sl*sl,sq2=sqrt(sq);
	double v1,v2,v3;
	v1=0;v2=0;v3=0;

	RMh=a*(1-e2)/sq/sq2+pos[2]; f_RMh=1.0/RMh;
	RNh=a/sq2+pos[2]; f_RNh=1.0/RNh; 
	clRNh=cl*RNh; f_clRNh=1.0/clRNh;

	gn[2]=-(glv.g0*(1+5.27094e-3*sl2+2.32718e-5*sl4)-3.086e-6*pos[2]);
	//glv.g = -gn[2];

	wnie[0]=0.0; 
	wnie[1]=glv.wie*cl;
	wnie[2]=glv.wie*sl;
	wnen[0]=-v2/RMh;
	wnen[1]=v1/RNh;
	wnen[2]=v1/(RNh*tl);
	Maddn(wnie,wnen,wnin,3,1);

	double w[3]={0.0};
	Maddn(wnie,wnin,w,3,1);
	gcc[0]=w[1]*v3-w[2]*v2;
	gcc[1]=w[2]*v1-w[0]*v3;
	gcc[2]=w[0]*v2-w[1]*v1;
	Mminn(gn,gcc,gcc,3,1);
}

void CEarth::UpdatePV(double pos[],double vn[])
{
	sl=sin(pos[0]); cl=cos(pos[0]); tl=sl/cl;
	sl2=sl*sl; sl4=sl2*sl2;
	double sq=1-e2*sl*sl,sq2=sqrt(sq);

	/*dsf90: RMh = Rm + h = Re*(1-e*e)/(1-(e*sin(lat)*(e*sin(lat))^(3/2) + h*/
	/*dsf90: RNh = Rn + h = Re/(1-(e*sin(lat)*(e*sin(lat))^(1/2) + h*/
	RMh=a*(1-e2)/sq/sq2+pos[2]; f_RMh=1.0/RMh;
	RNh=a/sq2+pos[2]; f_RNh=1.0/RNh; 
	clRNh=cl*RNh; f_clRNh=1.0/clRNh;

	gn[2]=-(glv.g0*(1+5.27094e-3*sl2+2.32718e-5*sl4)-3.086e-6*pos[2]);
	//glv.g=-gn[2];/*add by dsf90,2018.6.26*/

	/*dsf90: wnie,地球自转速率；wnen,位移速率；*/
	wnie[0]=0.0; 
	wnie[1]=glv.wie*cl;
	wnie[2]=glv.wie*sl;
	wnen[0]=-vn[1]/RMh;
	wnen[1]=vn[0]/RNh;
	wnen[2]=vn[0]/RNh*tl;
	/*dsf90: wnin = wnie + wnen*/
	Maddn(wnie,wnen,wnin,3,1);

	double w[3]={0.0};
	Maddn(wnie,wnin,w,3,1);
	/*dsf90:gcc = gn - (wnen+2wnie) X vn; w,vn 叉积 ;n系*/
	gcc[0]=w[1]*vn[2]-w[2]*vn[1];
	gcc[1]=w[2]*vn[0]-w[0]*vn[2];
	gcc[2]=w[0]*vn[1]-w[1]*vn[0];
	Mminn(gn,gcc,gcc,3,1);
}

CSINS::CSINS(double Att[],double Vn[],double Pos[],double EB[],double DB[],double Lever[])
{
	Mequalm(Att,3,1,att);
	Mequalm(Vn,3,1,vn);
	Mequalm(Pos,3,1,pos);
	Mequalm(EB,3,1,eb);
	Mequalm(DB,3,1,db);
	Mequalm(Lever,3,1,lever);

	a2qua(Att,qnb);
	a2mat(Att,Cnb);
	eth.UpdatePV(pos,vn);
	Mequalm(eth.gn,3,1,fn);
	tDelay=0.0;
	for(int i=0;i<9;i++)
	{
		Mpv[i]=0.0;
		MpvCnb[i]=0.0;
		CW[i]=0.0;
		Ka[i]=0.0;
		Kg[i]=0.0;
	}
	for(int i=0;i<3;i++)
	{
		PRY_Install[i] = 0;
	
		fn[i]=-fn[i];
		an[i]=0.0;
		web[i]=0.0;
		wnb[i]=0.0;
		Mpvvn[i]=0.0;
		vnL[i]=0.0;
		posL[i]=0.0;
		wm_1[i]=0.0;
		vm_1[i]=0.0;
		Ka[i*3+i]=1.0;
		Kg[i*3+i]=1.0;
	}
	Mpv[0*3+1]=1.0/eth.RMh;
	Mpv[1*3+0]=1.0/eth.clRNh;
	Mpv[2*3+2]=1.0;
	Mmulnm(Mpv,Vn,3,3,1,Mpvvn);
	Mmulnm(Mpv,Cnb,3,3,1,MpvCnb);
	double Cnbt[9];
	Mtn(Cnb,3,3,Cnbt);
	Mmulnm(Cnbt,eth.wnin,3,3,1,wib);
	Mmulnm(Cnbt,fn,3,3,1,fb);

	Mequalm(Cnb,3,3,Cnm);
	Mequalm(att,3,1,att_car);

	Kd = 1;
	Kwh = 1;
}
/*
CSINS& CSINS::operator=(const CSINS& ins)
{
	eth=ins.eth;
	tDelay=ins.tDelay;
	for(int i=0;i<9;i++)
	{
		Cmb[i]=ins.Cmb[i];
		Cbm[i]=ins.Cbm[i];
		Cnm[i]=ins.Cnm[i];
		
		Cnb[i]=ins.Cnb[i];
		Mpv[i]=ins.Mpv[i];
		MpvCnb[i]=ins.MpvCnb[i];
		CW[i]=ins.CW[i];
		Kg[i]=ins.Kg[i];
		Ka[i]=ins.Ka[i];
	}
	for(int i=0;i<3;i++)
	{
		PRY_Install[i]=ins.PRY_Install[i];
		att_car[i]=ins.att_car[i];
		vm_car[i]=ins.vm_car[i];
		
		att[i]=ins.att[i];
		vn[i]=ins.vn[i];
		pos[i]=ins.pos[i];
		Mpvvn[i]=ins.Mpvvn[i];
		eb[i]=ins.eb[i];
		db[i]=ins.db[i];
		fn[i]=ins.fn[i];
		an[i]=ins.an[i];
		web[i]=ins.web[i];
		wnb[i]=ins.wnb[i];
		wib[i]=ins.wib[i];
		fb[i]=ins.fb[i];
		wm_1[i]=ins.wm_1[i];
		vm_1[i]=ins.vm_1[i];
		lever[i]=ins.lever[i];
		vnL[i]=ins.vnL[i];
		posL[i]=ins.posL[i];
	}
	for(int i=0;i<4;i++)
	{
		qmb[i]=ins.qmb[i];
		
		qnb[i]=ins.qnb[i];
	}
	return(*this);
}
*/
void CSINS::Init(double Att[],double Vn[],double Pos[],double EB[],double DB[],double Lever[],double Lever2[],double PRY_install[])
{
	Mequalm(Att,3,1,att);
	Mequalm(Vn,3,1,vn);
	Mequalm(Pos,3,1,pos);
	Mequalm(EB,3,1,eb);
	Mequalm(DB,3,1,db);
	Mequalm(Lever,3,1,lever);
	Mequalm(Lever2,3,1,lever2);
	Mequalm(PRY_install,3,1,PRY_Install);
	
	a2qua(PRY_install,qmb);
	a2mat(PRY_install,Cmb);
	
	a2qua(Att,qnb);
	a2mat(Att,Cnb);
	eth.UpdatePV(pos,vn);
	Mequalm(eth.gn,3,1,fn);
	tDelay=0.0;
	for(int i=0;i<9;i++)
	{
		Mpv[i]=0.0;
		MpvCnb[i]=0.0;
		CW[i]=0.0;
		Ka[i]=0.0;
		Kg[i]=0.0;
	}
	for(int i=0;i<3;i++)
	{
		fn[i]=-fn[i];
		an[i]=0.0;
		web[i]=0.0;
		wnb[i]=0.0;
		Mpvvn[i]=0.0;
		vnL[i]=0.0;
		posL[i]=0.0;
		wm_1[i]=0.0;
		vm_1[i]=0.0;
		Ka[i*3+i]=1.0;
		Kg[i*3+i]=1.0;
	}
	Mpv[0*3+1]=1.0/eth.RMh;
	Mpv[1*3+0]=1.0/eth.clRNh;
	Mpv[2*3+2]=1.0;
	Mmulnm(Mpv,Vn,3,3,1,Mpvvn);
	Mmulnm(Mpv,Cnb,3,3,1,MpvCnb);
	double Cnbt[9];
	Mtn(Cnb,3,3,Cnbt);
	Mmulnm(Cnbt,eth.wnin,3,3,1,wib);
	Mmulnm(Cnbt,fn,3,3,1,fb);
}

void CSINS::Update(double wm[],double vm[],double dtime)
{
	double t2=dtime*0.5;
	//double ebdt[3],dbdt[3],kwm[3],kvm[3];
	//Mmuln(eb,3,1,dtime,ebdt); 
	//Mmuln(db,3,1,dtime,dbdt); 
	//Mmulnm(Kg,wm,3,3,1,kwm);
	//Mmulnm(Ka,vm,3,3,1,kvm);
	//Mminn(kwm,ebdt,wm,3,1); 
	//Mminn(kvm,dbdt,vm,3,1); 

	/*dsf90:wib = wib - eb; wib, b系角速率*/
	/*dsf90:fb	= fb - db; fb, b系加速度*/
	Mmin(wib,eb,3,1); 
	Mmin(fb,db,3,1); 

	Mmuln(wib,3,1,dtime,wm);
	Mmuln(fb,3,1,dtime,vm);

#if (!GILC_SIMPLIFY_USED)	
	/*dsf90:速度、位置预测更新*/
	/*dsf90:vn01 = vn + 0.5*dt*an ; vn01 本时间段内的中点速度*/
	double ant2[3],vn01[3],vn01t2[3],Mvt[3],pos01[3];
	Mmuln(an,3,1,t2,ant2);
	Maddn(vn,ant2,vn01,3,1);
	/*dsf90:pos01 = pos + 0.5*dt*Mpv X vn01; pos01 本时间段内的中点位置*/
	Mmuln(vn01,3,1,t2,vn01t2);
	Mmulnm(Mpv,vn01t2,3,3,1,Mvt);
	Maddn(pos,Mvt,pos01,3,1); 
	
	/*dsf90:使用位置、速度更新值，更新地球模型*/
	eth.UpdatePV(pos01,vn01);
#else	/*dsf90:简化版,差异小于1mm*/
	eth.UpdatePV(pos,vn);
#endif

	double Cnbt[3*3],Ctwie[3*1];
	Mequalm(Cnb,3,3,Cnbt);
	Mt(Cnbt,3,3);

#if (!GILC_SIMPLIFY_USED)	
	/*dsf90:web = wib - Cbn X wnie；*/
	Mmulnm(Cnbt,eth.wnie,3,3,1,Ctwie);
	Mminn(wib,Ctwie,web,3,1);
#else
	/*dsf90:wnb = wib - Cbn X wnin；*/
	Mmulnm(Cnbt,eth.wnin,3,3,1,Ctwie);
	Mminn(wib,Ctwie,wnb,3,1);
#endif

#if (!GILC_SIMPLIFY_USED)	
	/*dsf90:Cnb_new = Cwm2m = Cnb X rv2m(0.5*wib*dt);时间段中点角度增量*/
	/*dsf90:win_new = Ctwnin = Cbn_new X wnin;*/
	/*dsf90:wnb = wib - Cbn_new X wnin;疑问？wnb未使用*/
	//double wnb[3];
	double wm2m[3*3],wm2[3*1],Cwm2m[3*3],Cwm2mt[3*3],Ctwnin[3*1];
	Mmuln(wm,3,1,0.5,wm2);
	rv2m(wm2,wm2m);
	Mmulnm(Cnb,wm2m,3,3,3,Cwm2m);
	Mtn(Cwm2m,3,3,Cwm2mt);
	Mmulnm(Cwm2mt,eth.wnin,3,3,1,Ctwnin);
	Mminn(wib,Ctwnin,wnb,3,1);
#else /*dsf90:简化*/

#endif

	/*dsf90:速度更新*/
	/*dsf90:fn  = Cnb X fb; fn, n系实际加速度*/
	/*dsf90:an  = rv2m(-0.5*dt*wnin) X fn + gcc; fn, n系实际加速度*/
	/*dsf90:vn1 = vn + (rv2m(-0.5*dt*wnin) X (Cnb X fb) + gcc)*dt;*/
	/*dsf90:教材版 vn1 = vn + (Cnb X fb + gcc)*dt*/
	Mmulnm(Cnb,fb,3,3,1,fn);                /*dsf90: 等效 qmulv(qnb,fb,fn);*/
	double ant[3],vn1[3];
#if (!GILC_SIMPLIFY_USED)	
	double rtvfn[3];
	double wnint[3];
	Mmuln(eth.wnin,3,1,-t2,wnint);
	rotv(wnint,fn,rtvfn);
	Maddn(rtvfn,eth.gcc,an,3,1);
#else /*dsf90:简化版，教材版，差异小于1mm*/
	Maddn(fn,eth.gcc,an,3,1);
#endif	
	Mmuln(an,3,1,dtime,ant);
	Maddn(vn,ant,vn1,3,1);

	/*dsf90:位置更新*/
	/*dsf90:Mpvvn =  0.5 * Mpv X (vn+vn1)*/
	/*dsf90:pos = pos + 0.5 * dt * Mpv X (vn+vn1)*/
	/*dsf90:vn = vn1*/
	Mpv[0*3+1]=1.0/eth.RMh;
	Mpv[1*3+0]=1.0/eth.clRNh;
	Mpv[2*3+2]=1.0;
	double vnvn1[3],Mvnt[3];
	Maddn(vn,vn1,vnvn1,3,1);
	Mmul(vnvn1,3,1,0.5);
	Mmulnm(Mpv,vnvn1,3,3,1,Mpvvn);
	Mmuln(Mpvvn,3,1,dtime,Mvnt);
	Madd(pos,Mvnt,3,1);
	Mequalm(vn1,3,1,vn);

	/*dsf90:姿态更新*/
	/*dsf90:qnb = qupdt2(qnb,wib*dt,wnin*dt);*/
	/*dsf90:教材版 qnb = qupdt(qnb,wnb*dt) = qupdt(qnb,((wib-Cbn X wnin)*dt))*/
	double wnindt[3], qnb1[4];
	//double q[4];
#if 1
	Mmuln(eth.wnin,3,1,dtime,wnindt);
	qupdt2(qnb,wm,wnindt,qnb1);
#elif 0 /*dsf90:等效展开，误差小于1mm*/
	Mmuln(eth.wnin,3,1,-dtime,wnindt);
	qupdt(qnb,wm,qnb1);
	qdelphi(qnb1,wnindt,qnb1);
#elif 0 /*dsf90:等效展开，误差小于1mm*/
	Mmuln(eth.wnin,3,1,-dtime,wnindt);
	memcpy(qnb1,qnb,sizeof(qnb1));
	rv2q(wm,q);
	qmul(qnb1,q);
	rv2q(wnindt,q);
	qmul(q,qnb1);
	memcpy(qnb1,q,sizeof(qnb1));
#else	/*dsf90:简化版，教材版，角增量加减后更新，差异约1mm*/
	/*dsf90:wnb = wib - Cbn X wnin;*/
	double wnbdt[3];
	double Cbn[3*3],Wbin[3];
	double wnb[3];
	Mtn(Cnb,3,3,Cbn);
	Mmulnm(Cbn,eth.wnin,3,3,1,Wbin);
	Mminn(wib,Wbin,wnb,3,1);
	Mmuln(wnb,3,1,dtime,wnbdt);
	qupdt(qnb,wnbdt,qnb1);
#endif
	Mequalm(qnb1,4,1,qnb);

	q2att(qnb,att);
	q2mat(qnb,Cnb);
	/*dsf90:更新车辆速度*/
	double cbn[9],vb[3];
	Mtn(Cnb,3,3,cbn);
	Mmulnm(cbn,vn1,3,3,1,vb);
	Mmulnm(Cmb,vb,3,3,1,vm_car);
	
	Mtn(Cmb,3,3,Cbm);
	Mmulnm(Cnb,Cbm,3,3,3,Cnm);
	m2att(Cnm,att_car);
	/*dsf90:更新IMU、车辆加速度*/
	Mmulnm(cbn,an,3,3,1,ab);
	Mmulnm(Cmb,ab,3,3,1,am);

	Mmulnm(Cmb, fb, 3, 3, 1, fm);
	Mmulnm(Cmb, wib, 3, 3, 1, wim);
}
#if 0
void CSINS::Lever()
{
	double askeww[3*3]={0.0};
	double temp31[3]={0.0};


	/*dsf90:wbnb    = wbib - Cbn X wnin;*/
	/*dsf90:CW	   = Cnb X (wbnb X);*/
	askew(wnb,askeww); 
	Mmulnm(Cnb,askeww,3,3,3,CW); 

	/*dsf90:MpvCnb = Mpv X Cnb;*/
	/*dsf90:Mpvvn  = Mpv X vn;*/
	Mmulnm(Mpv,Cnb,3,3,3,MpvCnb); 
	Mmulnm(Mpv,vn,3,3,1,Mpvvn);

	/*dsf90:vnL    = vn + CW X lb;*/
	Mmulnm(CW,lever,3,3,1,temp31);
	Maddn(vn,temp31,vnL,3,1);
	Mmulnm(CW,lever2,3,3,1,temp31);
	Maddn(vn,temp31,vnL2,3,1);
	//Mminn(vn,temp31,vnL,3,1);

	/*dsf90:vnL    = vnL + an X tDelay;*/
	Mmuln(an,3,1,tDelay,temp31);
	Madd(vnL,temp31,3,1);

	/*dsf90:posL    = pos + Mpv X Cnb X lb;*/
	Mmulnm(MpvCnb,lever,3,3,1,temp31);
	Maddn(pos,temp31,posL,3,1);
	Mmulnm(MpvCnb,lever2,3,3,1,temp31);
	Maddn(pos,temp31,posL2,3,1);
	//Mminn(pos,temp31,posL,3,1);

	/*dsf90:posL    = posL + Mpv X vn X tDelay;*/
	Mmuln(Mpvvn,3,1,tDelay,temp31);
	Madd(posL,temp31,3,1);	
}
#else
void CSINS::Lever()
{
	double askeww[3 * 3] = { 0.0 };
	double temp31[3] = { 0.0 };

	/*dsf90:wbnb    = wbib - Cbn X wnin;*/
	/*dsf90:CW	   = Cnb X (wbnb X);*/
	askew(wnb, askeww);
	Mmulnm(Cnb, askeww, 3, 3, 3, CW);

	/*dsf90:MpvCnb = Mpv X Cnb;*/
	/*dsf90:Mpvvn  = Mpv X vn;*/
	Mmulnm(Mpv, Cnb, 3, 3, 3, MpvCnb);
	Mmulnm(Mpv, vn, 3, 3, 1, Mpvvn);

	/*dsf90:vnL    = vn + CW X lb;*/
	Mmulnm(CW, lever, 3, 3, 1, temp31);
	Maddn(vn, temp31, vnL, 3, 1);
	Mmulnm(CW, lever2, 3, 3, 1, temp31);
	Maddn(vnL, temp31, vnL2, 3, 1);
	//Mminn(vn,temp31,vnL,3,1);

	/*dsf90:vnL    = vnL + an X tDelay;*/
	//Mmuln(an, 3, 1, tDelay, temp31);
	//Madd(vnL, temp31, 3, 1);

	/*dsf90:posL    = pos + Mpv X Cnb X lb;*/
	Mmulnm(MpvCnb, lever, 3, 3, 1, temp31);
	Maddn(pos, temp31, posL, 3, 1);
	Mmulnm(MpvCnb, lever2, 3, 3, 1, temp31);
	Maddn(posL, temp31, posL2, 3, 1);
	//Mminn(pos,temp31,posL,3,1);

	/*dsf90:posL    = posL + Mpv X vn X tDelay;*/
	//Mmuln(Mpvvn, 3, 1, tDelay, temp31);
	//Madd(posL, temp31, 3, 1);
}

#endif
GIKF::GIKF(int row,int col,int opt)
{
	ROW=row;
	COL=col;
	OPT=opt;
	xflag=0xffffffff;
	zflag=0xffffffff;
	xk=(double*)__ml_zero(sizeof(double)*ROW*1);
	xkpre=(double*)__ml_zero(sizeof(double)*ROW*1);
	Pxk=(double*)__ml_zero(sizeof(double)*ROW*ROW);
	Phi=(double*)__ml_zero(sizeof(double)*ROW*ROW);
	Qk=(double*)__ml_zero(sizeof(double)*ROW*ROW);
	Gk=(double*)__ml_zero(sizeof(double)*ROW*ROW);
	Hk=(double*)__ml_zero(sizeof(double)*COL*ROW);
	Rk=(double*)__ml_zero(sizeof(double)*COL*COL);
	dpos=(double*)__ml_zero(sizeof(double)*3);
	denu=(double*)__ml_zero(sizeof(double)*3);
	bGnssUpdata=false;
}

//深拷贝，重新申请内存 赋值
GIKF& GIKF::operator=(const GIKF& kftemp)
//GIKF::GIKF(const GIKF& kftemp)
{
	ROW=kftemp.ROW;
	COL=kftemp.COL;
	OPT=kftemp.OPT;
	xflag=kftemp.xflag;
	zflag=kftemp.zflag;
	for(int i=0;i<ROW;i++)
	{
		*(xk+i)=*(kftemp.xk+i);
		*(xkpre+i)=*(kftemp.xkpre+i);
	}
	for(int i=0;i<ROW*ROW;i++)
	{
		*(Pxk+i)=*(kftemp.Pxk+i);
		*(Phi+i)=*(kftemp.Phi+i);
		*(Qk+i)=*(kftemp.Qk+i);
		*(Gk+i)=*(kftemp.Gk+i);
	}
	for(int i=0;i<COL*ROW;i++)
	{*(Hk+i)=*(kftemp.Hk+i);}
	for(int i=0;i<COL*COL;i++)
	{*(Rk+i)=*(kftemp.Rk+i);}
	for(int i=0;i<3;i++)
	{
		*(dpos+i)=*(kftemp.dpos+i);
		*(denu+i)=*(kftemp.denu+i);
	}
	bGnssUpdata=kftemp.bGnssUpdata;	
	return(*this);
}

void GIKF::kffree()
{
	if(xk!=NULL)
	{free(xk);xk=NULL;}
	if(xkpre!=NULL)
	{free(xkpre);xkpre=NULL;}
	if(dpos!=NULL)
	{free(dpos);dpos=NULL;}
	if(denu!=NULL)
	{free(denu);denu=NULL;}
	if(Phi!=NULL)
	{free(Phi);Phi=NULL;}
	if(Hk!=NULL)
	{free(Hk);Hk=NULL;}
	if(Pxk!=NULL)
	{free(Pxk);Pxk=NULL;}
	if(Qk!=NULL)
	{free(Qk);Qk=NULL;}
	if(Gk!=NULL)
	{free(Gk);Gk=NULL;}
	if(Rk!=NULL)
	{free(Rk);Rk=NULL;}
}

void GIKF::kfinit()
{
	xflag = 0xffffffff;
	zflag = 0xffffffff;

	memset(xk,0,sizeof(double)*ROW*1);
	memset(xkpre,0,sizeof(double)*ROW*1);
	memset(dpos,0,sizeof(double)*3*1);
	memset(denu,0,sizeof(double)*3*1);
	memset(Phi,0,sizeof(double)*ROW*ROW);
	memset(Hk,0,sizeof(double)*COL*ROW);
	memset(Rk,0,sizeof(double)*COL*1);
	memset(Pxk,0,sizeof(double)*ROW*ROW);
	memset(Qk,0,sizeof(double)*ROW*ROW);
	memset(Gk,0,sizeof(double)*ROW*ROW);
	
	/*dsf90:Pxk 初始化为噪声*/
	for (int i=0;i<ROW;i++)
	{
		Pxk[i*ROW+i]=SQR(kf_P_init[i]);    
	}

	/*dsf90:过程噪声*/
	for (int i=0;i<ROW;i++)
	{
		Qk[i*ROW+i]=SQR(kf_Q_init[i]);
	}

	Gk[0*ROW+0]=-1; Gk[0*ROW+1]=0; Gk[0*ROW+2]=0;
	Gk[1*ROW+0]=0; Gk[1*ROW+1]=-1; Gk[1*ROW+2]=0;
	Gk[2*ROW+0]=0; Gk[2*ROW+1]=0; Gk[2*ROW+2]=-1;
	Gk[3*ROW+3]=1; Gk[3*ROW+4]=0; Gk[3*ROW+5]=0;
	Gk[4*ROW+3]=0; Gk[4*ROW+4]=1; Gk[4*ROW+5]=0;
	Gk[5*ROW+3]=0; Gk[5*ROW+4]=0; Gk[5*ROW+5]=1;
	Gk[9*ROW+9]=1.0;
	Gk[10*ROW+10]=1.0; 
	Gk[11*ROW+11]=1.0;
	Gk[12*ROW+12]=1.0; 
	Gk[13*ROW+13]=1.0; 
	Gk[14*ROW+14]=1.0;	
}

void GIKF::upPhi(CSINS& ins,double dt)
{
	Mequal(Phi,ROW,ROW,0);

	double sl,cl,tl,secl,secl2,f_RMh,f_RNh,f_RMh2,f_RNh2,f_clRNh;
	sl=sin(ins.pos[0]); cl=cos(ins.pos[0]);
	tl=sl/cl; secl=1/cl;
	secl2=sl*sl;
	f_RMh=1.0/ins.eth.RMh; f_RNh=1.0/ins.eth.RNh;
	f_RMh2=f_RMh*f_RMh; f_RNh2=f_RNh*f_RNh;
	f_clRNh = 1.0/ins.eth.clRNh;

	double vn[3]={0.0};
	Mequalm(ins.vn,3,1,vn);
	double vE_clRNh,vE_RNh2,vN_RMh2;
	vE_clRNh=vn[0]*f_clRNh; vE_RNh2=vn[0]*f_RNh2; vN_RMh2=vn[1]*f_RMh2;

	double Mp1[3*3]={0.0};
	Mp1[1*3+0]=-ins.eth.wnie[2]; 
	Mp1[2*3+0]=ins.eth.wnie[1];

	double Mp2[3*3]={0.0};
	Mp2[0*3+2]=vN_RMh2; 
	Mp2[1*3+2]=-vE_RNh2; 
	Mp2[2*3+0]=vE_clRNh*secl;  Mp2[2*3+2]=-vE_RNh2*tl;

	double Avn[3*3]={0.0};
	Avn[0*3+1]=-vn[2]; Avn[0*3+2]=vn[1]; 
	Avn[1*3+0]=vn[2];                     Avn[1*3+2]=-vn[0]; 
	Avn[2*3+0]=-vn[1]; Avn[2*3+1]=vn[0];

	double Awn[3*3]={0.0};
	Awn[0*3+1]=-(ins.eth.wnie[2]+ins.eth.wnin[2]); Awn[0*3+2]=ins.eth.wnie[1]+ins.eth.wnin[1];
	Awn[1*3+0]=ins.eth.wnie[2]+ins.eth.wnin[2];                                                   Awn[1*3+2]=-(ins.eth.wnie[0]+ins.eth.wnin[0]);
	Awn[2*3+0]=-(ins.eth.wnie[1]+ins.eth.wnin[1]); Awn[2*3+1]=ins.eth.wnie[0]+ins.eth.wnin[0];

	double Maa[3*3]={0.0};
	                             Maa[0*3+1]=ins.eth.wnin[2];  Maa[0*3+2]=-ins.eth.wnin[1];
	Maa[1*3+0]=-ins.eth.wnin[2];                              Maa[1*3+2]=ins.eth.wnin[0];
	Maa[2*3+0]=ins.eth.wnin[1];  Maa[2*3+1]=-ins.eth.wnin[0];

	double Mav[3*3]={0.0};
	Mav[0*3+1]=-f_RMh;
	Mav[1*3+0]=f_RNh;
	Mav[2*3+0]=f_RNh*tl;

	double Map[3*3]={0.0};
	Maddn(Mp1,Mp2,Map,3,3);

	double Mva[3*3]={0.0};
	Mva[0*3+1]=-ins.fn[2]; Mva[0*3+2]=ins.fn[1];
	Mva[1*3+0]=ins.fn[2];                         Mva[1*3+2]=-ins.fn[0];
	Mva[2*3+0]=-ins.fn[1]; Mva[2*3+1]=ins.fn[0];

	double temp[3*3]={0.0};
	double Mvv[3*3]={0.0};
	Mmulnm(Avn,Mav,3,3,3,temp);
	Mminn(temp,Awn,Mvv,3,3);

	double Mvp[3*3]={0.0};
	Maddn(Mp1,Map,temp,3,3);
	Mmulnm(Avn,temp,3,3,3,Mvp);

	double g0=9.7803267714;
	double scl = sl*cl;

	Mvp[2*3+0]=Mvp[2*3+0]-g0*(5.27094e-3*2*scl+2.32718e-5*4*secl2*scl);
	Mvp[2*3+2]=Mvp[2*3+2]+3.086e-6;

	double Mpv[3*3]={0.0};

	Mequalm(ins.Mpv,3,3,Mpv);

	double Mpp[3*3]={0.0};
	Mpp[0*3+2]=-vN_RMh2;
	Mpp[1*3+0]=vE_clRNh*tl;  Mpp[1*3+2]=-vE_RNh2*secl;

	Phi[0*ROW+1]=Maa[0*3+1]; Phi[0*ROW+2]=Maa[0*3+2];
	Phi[1*ROW+0]=Maa[1*3+0];                         Phi[1*ROW+2]=Maa[1*3+2];
	Phi[2*ROW+0]=Maa[2*3+0]; Phi[2*ROW+1]=Maa[2*3+1]; 

	Phi[0*ROW+4]=Mav[0*3+1]; 
	Phi[1*ROW+3]=Mav[1*3+0];  
	Phi[2*ROW+3]=Mav[2*3+0];

	Phi[0*ROW+8]=Map[0*3+2];
	Phi[1*ROW+6]=Map[1*3+0];                           Phi[1*ROW+8]=Map[1*3+2];
	Phi[2*ROW+6]=Map[2*3+0];                           Phi[2*ROW+8]=Map[2*3+2];

	Phi[0*ROW+9]=-ins.Cnb[0*3+0]; Phi[0*ROW+10]=-ins.Cnb[0*3+1]; Phi[0*ROW+11]=-ins.Cnb[0*3+2];
	Phi[1*ROW+9]=-ins.Cnb[1*3+0]; Phi[1*ROW+10]=-ins.Cnb[1*3+1]; Phi[1*ROW+11]=-ins.Cnb[1*3+2];
	Phi[2*ROW+9]=-ins.Cnb[2*3+0]; Phi[2*ROW+10]=-ins.Cnb[2*3+1]; Phi[2*ROW+11]=-ins.Cnb[2*3+2];

	Phi[3*ROW+1]=Mva[0*3+1]; Phi[3*ROW+2]=Mva[0*3+2];
	Phi[4*ROW+0]=Mva[1*3+0];                         Phi[4*ROW+2]=Mva[1*3+2];
	Phi[5*ROW+0]=Mva[2*3+0]; Phi[5*ROW+1]=Mva[2*3+1];

	Phi[3*ROW+3]=Mvv[0*3+0]; Phi[3*ROW+4]=Mvv[0*3+1]; Phi[3*ROW+5]=Mvv[0*3+2];
	Phi[4*ROW+3]=Mvv[1*3+0]; Phi[4*ROW+4]=Mvv[1*3+1]; Phi[4*ROW+5]=Mvv[1*3+2];
	Phi[5*ROW+3]=Mvv[2*3+0]; Phi[5*ROW+4]=Mvv[2*3+1]; Phi[5*ROW+5]=Mvv[2*3+2];

	Phi[3*ROW+6]=Mvp[0*3+0]; Phi[3*ROW+7]=Mvp[0*3+1]; Phi[3*ROW+8]=Mvp[0*3+2];
	Phi[4*ROW+6]=Mvp[1*3+0]; Phi[4*ROW+7]=Mvp[1*3+1]; Phi[4*ROW+8]=Mvp[1*3+2];
	Phi[5*ROW+6]=Mvp[2*3+0]; Phi[5*ROW+7]=Mvp[2*3+1]; Phi[5*ROW+8]=Mvp[2*3+2];

	Phi[3*ROW+12]=ins.Cnb[0*3+0]; Phi[3*ROW+13]=ins.Cnb[0*3+1]; Phi[3*ROW+14]=ins.Cnb[0*3+2];
	Phi[4*ROW+12]=ins.Cnb[1*3+0]; Phi[4*ROW+13]=ins.Cnb[1*3+1]; Phi[4*ROW+14]=ins.Cnb[1*3+2];
	Phi[5*ROW+12]=ins.Cnb[2*3+0]; Phi[5*ROW+13]=ins.Cnb[2*3+1]; Phi[5*ROW+14]=ins.Cnb[2*3+2];

	Phi[6*ROW+4]=Mpv[0*3+1]; 
	Phi[7*ROW+3]=Mpv[1*3+0]; 
	Phi[8*ROW+5]=Mpv[2*3+2];

	Phi[6*ROW+8]=Mpp[0*3+2];
	Phi[7*ROW+6]=Mpp[1*3+0];                             Phi[7*ROW+8]=Mpp[1*3+2];

#if 0		//加计和陀螺的一阶马尔科夫模型 相关时间200s 反相关系数 与遗忘滤波相互作用了
	double t = 100.0;
	Phi[9*ROW+9]=-1.0/t;
	Phi[10*ROW+10]=-1.0/t;
	Phi[11*ROW+11]=-1.0/t;
	Phi[12*ROW+12]=-1.0/t;
	Phi[13*ROW+13]=-1.0/t;
	Phi[14*ROW+14]=-1.0/t;
#endif
	/*dsf90:Phi = I + F*T = I + Phi*dt */
	double *eye=(double*)__ml_zero(sizeof(double)*ROW*ROW);
	Munit(eye,ROW);
	Mmul(Phi,ROW,ROW,dt);
	Madd(Phi,eye,ROW,ROW);
	free(eye);
}

void GIKF::upHk(CSINS& ins,double *hk)
{
	/*
	hk[0*ROW+0]=-cos(ins.att[2]);
	hk[0*ROW+1]= sin(ins.att[2]);
	hk[0*ROW+2]= 0;
	hk[1*ROW+0]= sin(ins.att[2])/cos(ins.att[0]);
	hk[1*ROW+1]=-cos(ins.att[2])/cos(ins.att[0])+2*sin(ins.att[0])*sin(ins.att[1])*cos(ins.att[1])*sin(ins.att[2])/cos(ins.att[0]);
	hk[1*ROW+2]= 0;
	hk[2*ROW+0]=-sin(ins.att[0])*sin(ins.att[2])/cos(ins.att[0]);
	hk[2*ROW+1]= sin(ins.att[0])*cos(ins.att[2])/cos(ins.att[0]);
	hk[2*ROW+2]=-1;*/

	hk[0 * ROW + 0] = 1; hk[1 * ROW + 1] = 1; hk[2 * ROW + 2] = 1;
	hk[3 * ROW + 3] = 1; hk[4 * ROW + 4] = 1; hk[5 * ROW + 5] = 1;
	hk[6 * ROW + 6] = 1; hk[7 * ROW + 7] = 1; hk[8 * ROW + 8] = 1;
	
#if 0
	double askwnb[3*3]={0.0};
	double askwnblb[3]={0.0};
	double askaskwnblb[3*3]={0.0};
	double CnbAskAskWnbLb[3*3]={0.0};

	askew(ins.wnb,askwnb); 
	Mmulnm(askwnb,ins.lever,3,3,1,askwnblb); 
	askew(askwnblb,askaskwnblb); 
	Mmulnm(ins.Cnb,askaskwnblb,3,3,3,CnbAskAskWnbLb);

	/*dsf90:速度观测——姿态*/
	for (int i = 0; i<3; i++)
	{
		{	hk[3*ROW+0+i]=-CnbAskAskWnbLb[0*3+i];}
		{	hk[4*ROW+0+i]=-CnbAskAskWnbLb[1*3+i];}
		{	hk[5*ROW+0+i]=-CnbAskAskWnbLb[2*3+i];}
	}

	double asklb[3*3]={0.0};
	double cnbasklb[3*3]={0.0};
	double MpvCnbAskLb[3*3]={0.0};

	askew(ins.lever,asklb); 
	Mmulnm(ins.Cnb,asklb,3,3,3,cnbasklb); 
	Mmulnm(ins.Mpv,cnbasklb,3,3,3,MpvCnbAskLb); 

	/*dsf90:位置观测——姿态*/
	for (int i = 0; i<3; i++)
	{
		{	hk[6*ROW+0+i]=-MpvCnbAskLb[0*3+i];}
		{	hk[7*ROW+0+i]=-MpvCnbAskLb[1*3+i];}
		{	hk[8*ROW+0+i]=-MpvCnbAskLb[2*3+i];}
	}
#endif	

	/*dsf90:速度观测*/
	for (int i = 0; i<3; i++)
	{
		{	hk[3*ROW+18+i]=-ins.CW[0*3+i];}
		{	hk[4*ROW+18+i]=-ins.CW[1*3+i];}
		{	hk[5*ROW+18+i]=-ins.CW[2*3+i];}
	}

	double Mpvcnb[3*3]={0.0};
	Mmulnm(ins.Mpv,ins.Cnb,3,3,3,Mpvcnb);
	/*dsf90:位置观测*/
	for (int i = 0; i<3; i++)
	{
		{	hk[6*ROW+18+i]=-Mpvcnb[0*3+i];}
		{	hk[7*ROW+18+i]=-Mpvcnb[1*3+i];}
		{	hk[8*ROW+18+i]=-Mpvcnb[2*3+i];}
	}

	double cbn[9], cmn[9], vb[3], askvn[9], cmnAskvn[9],askvb[9], cmbAskvb[9];
	Mtn(ins.Cnb, 3, 3, cbn);
	Mmulnm(ins.Cmb, cbn, 3, 3, 3, cmn);
	askew(ins.vn, askvn);
	Mmulnm(cmn, askvn, 3, 3, 3, cmnAskvn);

	Mmulnm(cbn, ins.vn, 3, 3, 1, vb);
	askew(vb, askvb);
	Mmulnm(ins.Cmb, askvb, 3, 3, 3, cmbAskvb);

	/*dsf90:载体约束/里程计轮速*/
	for (int i = 0; i<3; i++)
	{
		{	hk[9 * ROW + i]  = -cmnAskvn[0 * 3 + i]; hk[9 * ROW + i + 3]  = cmn[0 * 3 + i]; hk[9 * ROW + 15 + i]  = cmbAskvb[0 * 3 + i];}
		{	hk[10 * ROW + i] = -cmnAskvn[1 * 3 + i]; hk[10 * ROW + i + 3] = cmn[1 * 3 + i]; hk[10 * ROW + 15 + i] = cmbAskvb[1 * 3 + i];}
		{	hk[11 * ROW + i] = -cmnAskvn[2 * 3 + i]; hk[11 * ROW + i + 3] = cmn[2 * 3 + i]; hk[11 * ROW + 15 + i] = cmbAskvb[2 * 3 + i];}
	}

	/*dsf90:载体速度观测（里程计航向）*/
	hk[12*ROW+11]=-1;
}

void GIKF::setRk_constraint(void)
{
	/*att*/
	Rk[0]=SQR(0.1*glv.deg); 
	Rk[1]=SQR(0.1*glv.deg); 
	Rk[2]=SQR(0.1*glv.deg);  
	/*vel*/
	Rk[3]=SQR(0.1);
	Rk[4]=SQR(0.1);
	Rk[5]=SQR(0.1);
	/*pos*/
	Rk[6]=SQR(0.0000001*glv.deg);
	Rk[7]=SQR(0.0000001*glv.deg);
	Rk[8]=SQR(0.001);
	/*vel_body*/
	Rk[9]=SQR(0.1); 
	Rk[10]=SQR(0.1);	
	Rk[11]=SQR(0.1);	
	Rk[12]=SQR(InitParam.dKfInitP_OdoHeading);
}

void GIKF::resetRk(CLCData ilcd)
{
	/*att*/
	Rk[0]=ilcd.GA_RK[0];
	Rk[1]=ilcd.GA_RK[1]; 
	Rk[2]=ilcd.GA_RK[2];  
	//Rk[2]=SQR(1*glv.deg);   
	/*vel*/
	Rk[3]=ilcd.GPV_RK[0*6+0];
	Rk[4]=ilcd.GPV_RK[1*6+1];
	Rk[5]=ilcd.GPV_RK[2*6+2];
	/*pos*/
	Rk[6]=ilcd.GPV_RK[3*6+3];
	Rk[7]=ilcd.GPV_RK[4*6+4];
	Rk[8]=ilcd.GPV_RK[5*6+5]; 
	/*vel_body*/
	Rk[9]=SQR(0.1); 
	Rk[10]=SQR(0.1);	
	Rk[11]=SQR(0.1);	
	Rk[12]=SQR(InitParam.dKfInitP_OdoHeading);
}

void GIKF::downgrade_car(CLCData ilcd,double denu[3],double posrk[3])
{
	double ver_lat=ilcd.GPV_RK[3*6+3]*glv.Re*glv.Re;  //m2
	double ver_lon=ilcd.GPV_RK[4*6+4]*glv.Re*glv.Re;  //m2
	double ver_hig=ilcd.GPV_RK[5*6+5];                //m2
	double de=pow(denu[0],2);
	double dn=pow(denu[1],2);
	double du=pow(denu[2],2);
	if(dn>ver_lat)
	{posrk[0]=dn/(glv.Re*glv.Re);}
	else
	{posrk[0]=ilcd.GPV_RK[3*6+3];}

	if(de>ver_lon)
	{posrk[1]=de/(glv.Re*glv.Re);}
	else
	{posrk[1]=ilcd.GPV_RK[4*6+4];}

	if(du>ver_hig)
	{posrk[2]=du;}
	else
	{posrk[2]=ilcd.GPV_RK[5*6+5];}
}

void GIKF::downgrade(CLCData ilcd,int scater,double posrk[3])
{
	posrk[0]=ilcd.GPV_RK[3*6+3]*scater*scater;
	posrk[1]=ilcd.GPV_RK[4*6+4]*scater*scater;
	posrk[2]=ilcd.GPV_RK[5*6+5]*scater*scater;
}

void GIKF::TUpdate(double dt,int option)
{
	TUpdate_True(ROW,COL,Qk,Gk,Phi,xk,Pxk,dt, option);
}

void GIKF::MUpdate(double ZK[])
{
	MUpdate_True(ROW,COL,Hk,Rk,ZK,xk,Pxk);
}

/*add by dsf90, 2018.5.11*/
void GIKF::MUpadte(CSINS& ins, double ZK[], int zflag, int opt)
{
	double* hk = (double*)__ml_zero(sizeof(double)*COL*ROW);

	upHk(ins,hk);
	MUpdate_Variable(ROW, COL, hk, Rk, ZK, xk, Pxk, zflag);

	free(hk);
}

void GIKF::Feedback(CSINS& ins,double scater,int option)
{
	double datt[3],dvn[3],dpos[3],deb[3],ddb[3],dPRY[3],dlever[3],dtelay;
	double* xk_f = (double*)__ml_zero(sizeof(double)*ROW);
	Mmuln(xk,ROW,1,scater,xk_f);

	datt[0]=xk_f[0]; datt[1]=xk_f[1]; datt[2]=xk_f[2];
	dvn[0]=xk_f[3];  dvn[1]=xk_f[4];  dvn[2]=xk_f[5];
	dpos[0]=xk_f[6]; dpos[1]=xk_f[7]; dpos[2]=xk_f[8];
	deb[0]=xk_f[9];  deb[1]=xk_f[10]; deb[2]=xk_f[11];
	ddb[0]=xk_f[12]; ddb[1]=xk_f[13]; ddb[2]=xk_f[14];
	if(ROW>=18)
	{	dPRY[0] = xk_f[15]; dPRY[1] = xk_f[16]; dPRY[2] = xk_f[17];}
	if(ROW>=21)
	{	dlever[0]=xk_f[18]; dlever[1]=xk_f[19]; dlever[2]=xk_f[20];}
	//if(ROW>=23)
		//dtelay = xk_f[22];

	qdelphi(ins.qnb,datt);
	Mmin(ins.vn, dvn, 3, 1);//
	Mmin(ins.pos, dpos, 3, 1);//
	Madd(ins.eb,deb,3,1); // 陀螺零偏
	Madd(ins.db,ddb,3,1);
	if(ROW>=18 && (xflag&(7<<15)))
	{
#if 1
		qdelphi(ins.qmb,dPRY);
		q2att(ins.qmb,ins.PRY_Install);
		q2mat(ins.qmb,ins.Cmb);	
#else /*dsf90:效果一致*/
		Madd(ins.PRY_Install,dPRY,3,1);
		a2qua(ins.PRY_Install,ins.qmb);
		a2mat(ins.PRY_Install,ins.Cmb);
#endif
		double cbn[9],vb[3];
		Mtn(ins.Cnb,3,3,cbn);
		Mmulnm(cbn,ins.vn,3,3,1,vb);
		Mmulnm(ins.Cmb,vb,3,3,1,ins.vm_car);
		
		Mtn(ins.Cmb,3,3,ins.Cbm);
		Mmulnm(ins.Cnb,ins.Cbm,3,3,3,ins.Cnm);
		m2att(ins.Cnm,ins.att_car);
	}
	
	if(ROW>=21 && (xflag&(7<<18)))
		Madd(ins.lever,dlever,3,1);
	/*
	if(ROW>=23 && (xflag&(1<<22)))
		ins.tDelay+=dtelay;
	*/
	ins.Lever();
	
	for(int i=0;i<ROW;i++)
	{xk[i]-=xk_f[i];}

	free(xk_f);
}

void kfequal(GIKF* kfold,GIKF* kfnew)
{
	for(int i=0;i<kfold->ROW;i++)
	{
		*(kfnew->xk+i)=*(kfold->xk+i);
		*(kfnew->xkpre+i)=*(kfold->xkpre+i);
	}
	for(int i=0;i<kfold->ROW*kfold->ROW;i++)
	{
		*(kfnew->Pxk+i)=*(kfold->Pxk+i);
		*(kfnew->Phi+i)=*(kfold->Phi+i);
		*(kfnew->Qk+i)=*(kfold->Qk+i);
		*(kfnew->Gk+i)=*(kfold->Gk+i);
	}
	for(int i=0;i<kfold->COL*kfold->ROW;i++)
	{*(kfnew->Hk+i)=*(kfold->Hk+i);}
	for(int i=0;i<kfold->COL*kfold->COL;i++)
	{*(kfnew->Rk+i)=*(kfold->Rk+i);}
	for(int i=0;i<3;i++)
	{
		*(kfnew->dpos+i)=*(kfold->dpos+i);
		*(kfnew->denu+i)=*(kfold->denu+i);
	}
}

extern bool busegnssvel_car(double gnssvel[3],double dvel[3],double dheading)
{
	int bgnss=0,bdif=0;
	if(fabs(gnssvel[0])<100 && fabs(gnssvel[1])<100&& fabs(gnssvel[2])<10) //360km/h
	{bgnss=1;}
	if(fabs(dvel[0])<5.0 && fabs(dvel[1])<5.0 && fabs(dvel[2])<5)
	{bdif=1;}
	return bgnss&&bdif;
}

extern bool bgnssskip_car(vector<gpos_t, malloc_allocator<gpos_t> > v_gps,double iposcur[3],int numins)
{
	bool bskipins=false;
	if(v_gps.size()<10 || numins>1000)
		return false;
	if(v_gps.size()>10)
	{
		int ith=v_gps.size()-1;
		double pcur[3]={v_gps[ith].lat,v_gps[ith].lon,v_gps[ith].hig};
		double ppre[3]={v_gps[ith-1].lat,v_gps[ith-1].lon,v_gps[ith-1].hig};
		double dpos_gg[3]={0},dpos_lc[3]={0};
		diffpos(ppre,pcur,dpos_gg);
		diffpos(iposcur,pcur,dpos_lc);
        if(fabs(dpos_lc[0])>10.0||fabs(dpos_lc[1])>10.0||fabs(dpos_lc[2])>10.0)
		{bskipins=true;}
	}
	//相邻历元GPS的差值<阈值
    return bskipins;
}

/***************************  class Align  *********************************/

KinAlign::KinAlign(int NGNSS)
{
	bFinshAlign=false;
	bStatic=true;
	gnssstate=0;
	ngnss=0;
	Ngnss=NGNSS;
	nspeed=0;
	for(int i=0;i<3;i++)
	{
		Att[i]=0.0;
		Vn[i]=0.0;
		Pos[i]=0.0;
		PRY_Install[i]=0.0;
	}
	for(int i=0;i<36;i++)
	{Pgposvn[i]=0.0;}
	v_GNSS.reserve(Ngnss);
}
/*
KinAlign& KinAlign::operator=(const KinAlign& kinalign)
{
	bFinshAlign=kinalign.bFinshAlign;
	bStatic=kinalign.bStatic;
	gnssstate=kinalign.gnssstate;
	ngnss=kinalign.ngnss;
	Ngnss=kinalign.Ngnss;
	nspeed=kinalign.nspeed;
	for(int i=0;i<3;i++)
	{
		Att[i]=kinalign.Att[i];
		Vn[i]=kinalign.Vn[i];
		Pos[i]=kinalign.Pos[i];
		PRY_Install[i]=kinalign.PRY_Install[i];
	}
	for(int i=0;i<36;i++)
	{
		Pgposvn[i]=kinalign.Pgposvn[i];
	}
	v_GNSS.assign(kinalign.v_GNSS.begin(),kinalign.v_GNSS.end());
	yaw_gnss.assign(kinalign.yaw_gnss.begin(),kinalign.yaw_gnss.end());
	dyni = kinalign.dyni;
	return(*this);
}
*/
void KinAlign::Init(void)
{
	bFinshAlign=false;
	bStatic=true;
	gnssstate=0;
	ngnss=0;
	Ngnss=Ngnss;
	nspeed=0;
	for(int i=0;i<3;i++)
	{
		Att[i]=0.0;
		Vn[i]=0.0;
		Pos[i]=0.0;
		PRY_Install[i]=0.0;
	}
	for(int i=0;i<36;i++)
	{Pgposvn[i]=0.0;}
	v_GNSS.reserve(Ngnss);
	v_GNSS.clear();
	yaw_gnss.clear();
}

bool KinAlign::CalAtt(CLCData& ilcd,int opt)
{
	unsigned int i,j,n=0,m=0;
	vector<double, malloc_allocator<double> > yaw;
	double dlon,dlat;
	double rr0[3],rr1[3],drr[3],blh[3],enu[3],enu_ave[3]={0.0};
	
	for (i=1;i<Ngnss;i++)
	{
		//yaw.push_back(v_GNSS[i].yaw);
		if(v_GNSS[i].yaw && ilcd.GA_RK[2] > SQ(0.001*D2R) && ilcd.GA_RK[2] < SQ(1*D2R))
		{	
			yaw.push_back(v_GNSS[i].yaw);
		}
		else
		{
			dlat=v_GNSS[i].lat-v_GNSS[0].lat; //dsf90:根据位置计算航向
			dlon=v_GNSS[i].lon-v_GNSS[0].lon;
			double iyaw=-atan2(dlon,dlat);
			yaw.push_back(iyaw);
		}
		n++;
	}

	for (i=0;i<Ngnss-1;i++)
	{
		blh[0]=v_GNSS[i].lat; blh[1]=v_GNSS[i].lon; blh[2]=v_GNSS[i].hig;
		pos2ecef(blh,rr0);

		blh[0]=v_GNSS[i+1].lat; blh[1]=v_GNSS[i+1].lon; blh[2]=v_GNSS[i+1].hig;
		pos2ecef(blh,rr1);

		for (j=0;j<3;j++) 
		{drr[j]=rr1[j]-rr0[j];}

		ecef2enu(blh,drr,enu); //ENU方向位移分量
		if (enu[0] < (50 * GNSS_PERIODE_MS / 1000.0) &&
			enu[1] < (50 * GNSS_PERIODE_MS / 1000.0) &&
			enu[2] < (50 * GNSS_PERIODE_MS / 1000.0)) //max 50m/s
		{
			for (j=0;j<3;j++) 
			{
				enu_ave[j]+=enu[j];
			}
			m++;
		}
	}
	for (j=0;j<3;j++)  /*dsf90:ENU方向平均位移*/
	{enu_ave[j]/=m;}
	/*add by dsf90,2018.10.22*/
	double std_yaw = GetAveStd(yaw, 1)/glv.deg;
	double ver_enu = norm(enu_ave, 3)*(1000/ GNSS_PERIODE_MS);
	/*end add*/
	if (std_yaw<3.0 && (opt || ver_enu > InitParam.dGnssVerMin_ForInit))/*dsf90: 航向std< 3deg, 速度>0.5m/s*/
	{
		//dlat=v_GNSS.back().lat-(v_GNSS.end()-2)->lat;
		//dlon=v_GNSS.back().lon-(v_GNSS.end()-2)->lon;
		//double iyaw=-atan2(dlon,dlat);
		//Att[2]=iyaw;
		Att[2] = (v_GNSS.end()-1)->yaw;
		double db[3];
		Mequalm(dyni.acc_ave,3,1,db);
		//double pitch=atan2(ilcd.acc[1],sqrt(pow(ilcd.acc[0],2)+pow(ilcd.acc[2],2)));
		//double roll =atan2(-ilcd.acc[0],ilcd.acc[2]);
		double pitch=atan2(db[1],sqrt(pow(db[0],2)+pow(db[2],2)));
		double roll =atan2(-db[0],db[2]);
		Att[0]=pitch;         /*dsf90,风险:初始姿态设置为起步前的姿态*/
		Att[1]=roll;
		//位置 速度 方差 解状态初始化
		Vn[0]=v_GNSS.back().ve;
		Vn[1]=v_GNSS.back().vn;
		Vn[2]=v_GNSS.back().vu;
		Pos[0]=v_GNSS.back().lat;
		Pos[1]=v_GNSS.back().lon;
		Pos[2]=v_GNSS.back().hig;
		Mequalm(ilcd.GPV_RK,6,6,Pgposvn);
		gnssstate=v_GNSS.back().state;
		return true;
	}
	else
	{
		//gilc_log("CalAtt %f %f\r\n",std_yaw,ver_enu);
		return false;
	}
}

bool KinAlign::CalAtt2(CLCData& ilcd)
{
	//double iyaw=-atan2(ilcd.vn[0],ilcd.vn[1]);
	double iyaw=-GetAveStd(yaw_gnss,0);
	Att[2]=iyaw;
	double db[3];
	Mequalm(dyni.acc_ave,3,1,db);
	double pitch=atan2(db[1],sqrt(pow(db[0],2)+pow(db[2],2)));
	double roll =atan2(-db[0],db[2]);
	Att[0]=pitch;         /*dsf90,风险:初始姿态设置为起步前的姿态*/
	Att[1]=roll;
	//位置 速度 方差 解状态初始化
	Vn[0]=ilcd.vn[0];
	Vn[1]=ilcd.vn[1];
	Vn[2]=ilcd.vn[2];
	Pos[0]=ilcd.pos[0];
	Pos[1]=ilcd.pos[1];
	Pos[2]=ilcd.pos[2];
	Mequalm(ilcd.GPV_RK,6,6,Pgposvn);
	gnssstate=ilcd.stat;
	return true;
}

bool KinAlign::KinmateAlign(CLCData& ilcd,GIProcess& gipro)
{
	//if(!bStatic || gipro.bDualAntCalibrateOk)
	if (!bStatic)
	{
		//计算航向- acc和gyo的ave和std已算
		if(ilcd.bGPSavail && ilcd.stat==GNSSFIX)
		{
			gpos_t gnsstemp;
			equalgpos(&gnsstemp,&ilcd);
			if(gipro.bDualAntCalibrateOk)
			{
				gnsstemp.yaw = ilcd.heading2  + gipro.dDualAntYawBias;
			}
			else
			{
				gnsstemp.yaw = ilcd.heading;
			}
			DEG_NEG180_180(gnsstemp.yaw);
			gnsstemp.yaw *= -D2R;
			if(v_GNSS.size()<Ngnss)
			{//GNSS点数不够
				v_GNSS.push_back(gnsstemp);
				return false;
			}
			else
			{
				v_GNSS.erase(v_GNSS.begin());
				v_GNSS.push_back(gnsstemp);
				//bFinshAlign=CalAtt(ilcd, gipro.bDualAntCalibrateOk);/*dsf90:固定解初始化*/
				bFinshAlign = CalAtt(ilcd,0);/*dsf90:固定解初始化*/
				return bFinshAlign;
			}
		}
#if 1
		else
		{
			if(ilcd.bGPSavail)
			{
				double speed=norm(ilcd.vn,3);
				if(fabs(speed)>1.0 && ilcd.nsused > 12)
				{
					double gnssyawvtg=atan2(ilcd.vn[0],ilcd.vn[1]);
					yaw_gnss.push_back(gnssyawvtg);
					nspeed++;
					if(nspeed==25)
					{
						double std_yaw = GetAveStd(yaw_gnss,1);
					    if(std_yaw<3*glv.deg)
					    {
							bFinshAlign=CalAtt2(ilcd);/*dsf90:连续20个点航向，std < 3deg, 姿态初始化*/
							return bFinshAlign;
						}
					    else
					    {
							nspeed--;
							yaw_gnss.erase(yaw_gnss.begin());
							return false;
					    }
					}
					return false;
				}
				else
				{
					nspeed=0;
					yaw_gnss.clear();
					return false;
				}
			}
			else
			{return false;}			
		}
#endif
	}
	return false;	
}

GIProcess::GIProcess(int row,int col,double sam_int)
{
	Row=row;
	Col=col;
	Opt=row*10+col;
	
	dt=sam_int;
	tpre=0.0;
	dt_total = 0.0;

	bFileSave=false;
	
	memset(kfpre,0,sizeof(kfpre));

	c=0;
	bAlign=false;
	bStatic=true;
	bStaticpre=false;
	iDriverMode=0;
	bInstallOk=false;
	bGnssTimeOut=0;
	bGnssLost=false;
	bGnssLostNum=0;
	busegnssvn=true;
	bGnssNum=0;
	bgnssskip=false;
	bTurn=false;
	bFinshMoveAlign=0;
	num_FinshMoveAlign=0;
	num_GPSskip=0;
	num_GPSloose=0;
	num_ContinueFloat=0;
	num_ContinueFix=6;
	upmodel=0;
	num_GNSSrestore=0;
	num_GNSSmupdate=0;
	preheading=0;
	iInstallOk_cnt=0;
	for(int i=0;i<3;i++)
	{
		pospre[i]=wmpre[i]=vmpre[i]=0.0;
	}
	for(int i=0;i<9;i++)
	{
		var_ins[i]=0.0;
	}

	iInitTimes_ms = 0;

	//kf=GIKF(Row,Col,Opt);
	//rava=CRAvar(5,dt);
}
/*
GIProcess& GIProcess::operator=(const GIProcess& gipro)
{
	Row=gipro.Row;
	Col=gipro.Col;
	Opt=gipro.Opt;
	
	dt=gipro.dt;
	tpre=gipro.tpre;

	ins = gipro.ins;
	inspre=gipro.ins;
	kf = gipro.kf;
	para=gipro.para;
	dyni=gipro.dyni;
	detect=gipro.detect;

	bFileSave=gipro.bFileSave;
	
	memcpy(kfpre,gipro.kfpre,sizeof(kfpre));
	
	c=gipro.c;
	nPPS=gipro.nPPS;
	bAlign=gipro.bAlign;
	bStatic=gipro.bStatic;
	bStaticpre=gipro.bStaticpre;
	iDriverMode=gipro.iDriverMode;
	bInstallOk=gipro.bInstallOk;
	bGnssTimeOut=gipro.bGnssTimeOut;
	bGnssLost=gipro.bGnssLost;
	bGnssLostNum=gipro.bGnssLostNum;
	busegnssvn=gipro.busegnssvn;
	bGnssNum=gipro.bGnssNum;
	bTurn=gipro.bTurn;
	bFinshMoveAlign=gipro.bFinshMoveAlign;
	num_FinshMoveAlign=gipro.num_FinshMoveAlign;
	num_GNSSrestore=gipro.num_GNSSrestore;
	num_GNSSmupdate=gipro.num_GNSSmupdate;
	preheading=gipro.preheading;
	num_GPSskip=gipro.num_GPSskip;
	num_GPSloose=gipro.num_GPSloose;
	num_ContinueFloat=gipro.num_ContinueFloat;
	num_ContinueFix=gipro.num_ContinueFix;
	upmodel=gipro.upmodel;
	iInstallOk_cnt=gipro.iInstallOk_cnt;
	for(int i=0;i<3;i++)
	{
		wmpre[i]=gipro.wmpre[i];
		vmpre[i]=gipro.vmpre[i];
		pospre[i]=gipro.pospre[i];
	}
	for(int i=0;i<9;i++)
	{
		var_ins[i]=gipro.var_ins[i];
	}
	Install.assign(gipro.Install.begin(),gipro.Install.end());
	ppsKf.assign(gipro.ppsKf.begin(),gipro.ppsKf.end());
	pregpos.assign(gipro.pregpos.begin(),gipro.pregpos.end());
	gpsstate.assign(gipro.gpsstate.begin(),gipro.gpsstate.end());
	prePxk.assign(gipro.prePxk.begin(),gipro.prePxk.end());
	return(*this);
}
*/
void GIProcess::Init(void)
{
	dt=dt;
	tpre=0.0;
	
	Row=Row;
	Col=Col;
	Opt=Row*10+Col;

	detect.Init();
	
	bFileSave=false;
	
	memset(kfpre,0,sizeof(kfpre));

	c=0;
	bAlign=false;
	bStatic=true;
	bStaticpre=false;
	iDriverMode=0;
	bInstallOk=false;
	bGnssTimeOut=0;
	bGnssLost=false;
	bGnssLostNum=0;
	busegnssvn=true;
	bGnssNum=0;
	bgnssskip=false;
	bTurn=false;
	bFinshMoveAlign=0;
	num_FinshMoveAlign=0;
	num_GPSskip=0;
	num_GPSloose=0;
	num_ContinueFloat=0;
	num_ContinueFix=6;
	upmodel=0;
	num_GNSSrestore=0;
	num_GNSSmupdate=0;
	preheading=0;
	iInstallOk_cnt=0;
	for(int i=0;i<3;i++)
	{
		pospre[i]=wmpre[i]=vmpre[i]=0.0;
	}
	for(int i=0;i<9;i++)
	{
		var_ins[i]=0.0;
	}

	iInitTimes_ms = 0;
	
	//kf.kffree();
	//kf=GIKF(Row,Col,Opt);
	//rava=CRAvar(5,dt);
	Install.clear();
	ppsKf.clear();
	pregpos.clear();
	gpsstate.clear();
	prePxk.clear();
}

void GIProcess::setlever(double leverdt[])
{
	para.setlever(leverdt);
}

void GIProcess::IMUcone(CLCData ilcd,double wmm[3],double vmm[3])
{
	double wcurm[3]={0.0},vcurm[3]={0.0},wprem[3]={0.0},vprem[3]={0.0};
	for(int i=0;i<3;i++)
	{
		wcurm[i]=ilcd.gyo[i]*dt;
		vcurm[i]=ilcd.acc[i]*dt;
		wprem[i]=wmpre[i]*dt;
		vprem[i]=vmpre[i]*dt;
	}
	double dphim[3]={0.0},dvbm={0.0},wm_1[3]={0.0},vm_1={0.0};
	cross3(wprem,wcurm,dphim);
	Mmul(dphim,3,1,1.0/12.0);
	for(int i=0;i<3;i++)
	{
		wm_1[i]=wcurm[i];
		wmpre[i]=ilcd.gyo[i];
	}
	Maddn(wcurm,dphim,wmm,3,1);

	double temp1[3]={0.0},temp2[3]={0.0},scullm[3]={0.0};
	cross3(wm_1,vcurm,temp1);
	cross3(wprem,wprem,temp2);
	Maddn(temp1,temp2,scullm,3,1);
	Mmul(scullm,3,1,1.0/12.0);
	for(int i=0;i<3;i++)
	{
		vmpre[i]=ilcd.acc[i];
	}
	double rotm[3]={0.0};
	cross3(wcurm,vcurm,rotm);
	Mmul(rotm,3,1,0.5);
	Maddn(vcurm,rotm,vmm,3,1);
	Madd(vmm,scullm,3,1);
}

void GIProcess::correctSideslip(void)
{
	/*dsf90:位置结果约束（限位）*/
	double dpos_b[3] = { 0 };
	int pro_flag = 0;

	if(fabs(pospre[0])<1e-6)
	{
		Mequalm(ins.pos, 3, 1, pospre);
	}
	else
	{
		difpos_b(pospre, ins.pos, ins.att_car, dpos_b);

		/*侧向位移修正*/	
		double dpos_x = sin(ins.wib[2] * dt / 2) * (ins.vm_car[1] * dt) * 2;//侧向位移增量 20为经验值
		if (fabs(dpos_b[0]) > fabs(dpos_x) && bGnssLost)
		{
			dpos_b[0] = dpos_x;
			pro_flag = 1;
		}
#if 0	
		/*高程位移修正*/	
		//double dpos_z = sin(ins.wib[0] * dt / 2) * (ins.vm_car[1] * dt) * 500;
		double dpos_z = tan(ins.wib[0] * dt / 2) * dpos_b[1] * 500;
		if (fabs(dpos_b[2]) > fabs(dpos_z) && bGnssLost)
		{
			dpos_b[2] = dpos_z;
			pro_flag = 1;
		}
#endif
		if (pro_flag)
			difpos(pospre, ins.pos, ins.att_car, dpos_b);
		Mequalm(ins.pos, 3, 1, pospre);
	}
}

void GIProcess::loadPPSSyncInsData(CLCData &ilcd,CSINS &ppsins)
{
	bPPSSync = false;
	/*PPS SYNC*/
	if(ilcd.bGPSavail)  
	{
		int imu_period_ms = InitParam.iImuPeriod_ms>(int)InitParam.dPpsTimeErrMax_ms ? InitParam.iImuPeriod_ms : (int)InitParam.dPpsTimeErrMax_ms;
		//int imu_period_ms = 20;
		ilcd.bGPSavail = false;
		if (fabs(inspre_forPPS.imutimetarge - ilcd.gpstimetarge) * 1000 < imu_period_ms)
		{	/*dsf90:匹配上次PPS位置*/
			ppsins = inspre_forPPS;
			bPPSSync = true;
			ilcd.bGPSavail = true;
		}
		else if (fabs(inspre.imutimetarge - ilcd.gpstimetarge) * 1000 < imu_period_ms)
		{	/*dsf90:匹配邻近INS位置*/
			ppsins = inspre;
			bPPSSync = true;
			ilcd.bGPSavail = true;
		}

		if (!ilcd.bGPSavail)  //GNSS 
		{
			gilc_log("%8.6f: Update GNSS flag, get pps time %.6f, gps time %.6f, new flag 0\r\n", ilcd.imutimetarge, ppsins.imutimetarge, ilcd.gpstimetarge);
		}
	}

}

void GIProcess::getDualAntBias(CLCData &ilcd,double dDifYaw)
{
	if (bInstallOk)
	{
		/*dsf90:初始化双天线安装误差*/
		if (!bTurn  && ilcd.stat == GNSSFIX && ilcd.GA_RK[2] != 0 && ilcd.gnss_speed > InitParam.dGnssVerMin_ForInit)
		{
			if (dDualAntYawBias_buf.size() < 100)
			{
				if (ilcd.GA_RK[2] && ilcd.GA_RK[2] < SQ(0.5*D2R))
					dDualAntYawBias_buf.push_back(dDifYaw);
			}
		}
		if (dDualAntYawBias_buf.size() >= 100)
		{
			dDualAntYawBias = GetAveStd(dDualAntYawBias_buf, 0);
			dDualAntYawBias_buf.clear();
			gilc_log("%02d:%02d:%06.3f  Heading2 init ok,Dual Ant install err %f deg\r\n",
				int(ilcd.ep[3]),int(ilcd.ep[4]),ilcd.ep[5],
				dDualAntYawBias);
			bDualAntInitOk = 1;
		}
	}
}

void GIProcess::KfInitOver()
{
#if 1
	//memcpy(kfpre,kf.Pxk,sizeof(kfpre));
	if (kf.ROW >= 18)
	{
		int i = 0, j = 0;
		for (i = 0; i < kf.ROW; i++)
		{
			for (j = 0; j < kf.ROW; j++)
			{
				if ((i >= 15 && i<18) || (j >= 15 && j<18))
					kf.Pxk[i*kf.ROW + j] = 0;
			}
		}
#if 1
		kf.xflag &= (~(7 << 15));
#else
		/*dsf90:重置安装误差估计*/
		for (int i = 0; i<3; i++)
			kf.Pxk[(i + 15)*kf.ROW + (i + 15)] = SQR(InitParam.dKfInitP_InstallErr / 10);
#endif				
	}
#endif

#if 0
	/*dsf90:重置加计零偏*/
	if (kf.ROW >= 15)
	{
		int i = 0, j = 0;
		for (i = 0; i < kf.ROW; i++)
		{
			for (j = 0; j < kf.ROW; j++)
			{
				if ((i >= 12 && i<15) || (j >= 12 && j<15))
					kf.Pxk[i*kf.ROW + j] = 0;
			}
		}

		for (int i = 0; i < 3; i++)
		{
			kf.Pxk[(i + 12)*kf.ROW + (i + 12)] = SQR(kf.AB[i] * glv.g0);
			kf.Qk[(i + 12)*kf.ROW + (i + 12)] = SQR(kf.AS[i]);	//vel walk
		}
	}
#endif
#if 1	
	/*dsf90:重置杆臂补偿值*/
	if (kf.ROW >= 21)
	{
		int i = 0, j = 0;
		for (i = 0; i < kf.ROW; i++)
		{
			for (j = 0; j < kf.ROW; j++)
			{
				if ((i >= 18 && i<21) || (j >= 18 && j<21))
					kf.Pxk[i*kf.ROW + j] = 0;
			}
		}
#if 1
		kf.xflag &= (~(7 << 18));
#else
		for (int i = 0; i<2; i++)
			kf.Pxk[(i + 18)*kf.ROW + (i + 18)] = SQR(InitParam.dKfInitP_Lever / 10);
#endif				
	}
#endif
}
void GIProcess::updateKfInitStatus(CLCData &ilcd)
{
	iInitTimes_ms += InitParam.iImuPeriod_ms;
	//	 状态标准差窗 需不需要一直开窗
	if(!bFinshMoveAlign && iInitTimes_ms >= dInitTimesMin*1000 && (iInitTimes_ms%200 == 0))
	{
		int i = 0;
#if 0
		vector<double, malloc_allocator<double> >temppxk;
		for(i=0;i<kf.ROW;i++)
		{
			temppxk.push_back(sqrt(kf.Pxk[i*kf.ROW+i]));
		}
		prePxk.push_back(temppxk);
		if(prePxk.size()>50)
		{
			prePxk.erase(prePxk.begin());
		}

		double rmsAccBias[2] = {0.0}, rmsYaw = 0.0, rmsInstall_x = 0.0, rmsInstall_z =  0.0;
		rmsYaw =GetAveStd(prePxk,2,2);
		rmsAccBias[0] = GetAveStd(prePxk,12,2);
		rmsAccBias[1] = GetAveStd(prePxk,13,2);
		rmsInstall_x = GetAveStd(prePxk, 15, 2);
		rmsInstall_z = GetAveStd(prePxk, 17, 2);
		if ((iGilcRunMode && rmsInstall_x<0.01*D2R && rmsInstall_z<0.01*D2R) || 
			(!iGilcRunMode && rmsAccBias[1]<kf.kf_P_init[13]*0.7))
		//if(rmsInstall_x<0.01*D2R)
		//if(rmsYaw<0.01*D2R )
		{
			num_FinshMoveAlign++;
			if(num_FinshMoveAlign>=50)
			{
				bFinshMoveAlign=1;
				bInstallOk = 1;
				KfInitOver();
				gilc_log("%.f finsh movealign!\r\n",ilcd.gpstimetarge);
			}
		}
		else
		{
			num_FinshMoveAlign=0;
		}
#else
		vector<double, malloc_allocator<double> >PRY_Install;
		for (i = 0; i<3; i++)
			PRY_Install.push_back(ins.PRY_Install[i] * R2D);
		Install.push_back(PRY_Install);
		if (Install.size()>50)
		{
			Install.erase(Install.begin());
		}

		double stdPRY_Install[3], avePRY_Install[3];
		for (i = 0; i<3; i++)
			stdPRY_Install[i] = GetAveStd(Install, i, 1);
		//gilc_log("stdPRY_Install:%f,%f,%f\r\n", stdPRY_Install[0], stdPRY_Install[1], stdPRY_Install[2]);
		if (stdPRY_Install[0] < InitParam.dInstallErrStdMax_ForInit
			&& stdPRY_Install[1] < InitParam.dInstallErrStdMax_ForInit
			&& stdPRY_Install[2] < InitParam.dInstallErrStdMax_ForInit
			&& (fabs(ins.db[0]) < 1 && fabs(ins.db[1]) < 1 && fabs(ins.db[1]) < 1)
			&& (fabs(ins.PRY_Install[0]) < 25 * D2R && fabs(ins.PRY_Install[1]) < 25 * D2R && fabs(ins.PRY_Install[2]) < 25 * D2R)
			)
		{
			iInstallOk_cnt++;
			if (iInstallOk_cnt >= 50)
			{
				bFinshMoveAlign = 1;
				bInstallOk = 1;
				KfInitOver();
				for (i = 0; i < 3; i++)
				{
					avePRY_Install[i] = GetAveStd(Install, i, 0);
					ins.PRY_Install[i] = avePRY_Install[i] * D2R;
				}
				gilc_log("%.f: Get PRY_Install ok! %f %f %f \r\n", ilcd.gpstimetarge, avePRY_Install[0], avePRY_Install[1], avePRY_Install[2]);
			}
		}
		else
		{
			iInstallOk_cnt = 0;
		}
#endif
	}
}

void GIProcess::GnssIntegrity(CLCData &ilcd,double dheading,double dvn[3])
{
	if(ilcd.bGPSavail)	//GNSS
	{	
		//判断GNSS速度是否可用
		busegnssvn=busegnssvel_car(ilcd.vn,dvn,dheading);
		/*
		if (ilcd.stat != 4 && ilcd.gnss_speed > 0.5 && bInstallOk)
		{
			if (fabs(dheading)>30)
			{
				gilc_log("%02d:%02d:%06.3f ----ins Yaw %5.3f----gnss Yaw %5.3f----ins Heading %5.3f----gnss Heading %5.3f, diff %3.3f  %3.3f\r\n",int(ilcd.ep[3]),int(ilcd.ep[4]),ilcd.ep[5],insYaw, gnssYaw, insHeading, gnssHeading,dDualAntYawBias*R2D, dheading);
				ilcd.bGPSavail = false;
			}
			if(bGnssTimeOut<2 && ilcd.gnss_speed-ins.vm_car[2]>2)
			{
				gilc_log("%02d:%02d:%06.3f ----ins Yaw %5.3f----gnss Yaw %5.3f----ins Heading %5.3f----gnss Heading %5.3f, diff %3.3f  %3.3f\r\n", int(ilcd.ep[3]), int(ilcd.ep[4]), ilcd.ep[5], insYaw, gnssYaw, insHeading, gnssHeading, dDualAntYawBias*R2D, dheading);
				ilcd.bGPSavail = false;
			}
		}*/
	
		//判断GNSS跳点
		gpos_t gpcur;
		equalgpos(&gpcur,&ilcd);
		if(pregpos.size()<15)
		{
			pregpos.push_back(gpcur);
		}
		else
		{
			pregpos.erase(pregpos.begin());
		}
		bgnssskip=bgnssskip_car(pregpos,ins.vn,c);
#if 0	/*dsf90:存在误判,2018.9.20*/
		if (bInstallOk)
		{	/*dsf90:滤波器稳定后,通过组合解判断GNSS数据质量*/
			if (bGnssTimeOut <= 60)
			{	/*dsf90:剔除一分钟内连续跳点*/
				double dPosEnu_max_err = bGnssTimeOut*0.3;	/*dsf90:INS置信度设定,0.3m/s*/
				if (fabs(enu[0]) > 3.0 || fabs(enu[1]) > 3.0 || fabs(enu[2]) > 3.0)/*dsf90:INS-GNSS，3m以上误差认为是跳点*/
				{
					if (enu[0] > dPosEnu_max_err || enu[1] > dPosEnu_max_err || enu[2] > dPosEnu_max_err)
					{
						ilcd.bGPSavail = false;
					}
				}
			}
		}
#endif
	}
	
	/*dsf90:GNSS失效时间判定*/
	if(ilcd.bGPSavail)	//GNSS
	{
		bGnssTimeOut = 0;
		if (bGnssLost)
		{
			gilc_log("%02d:%02d:%06.3f ----gnss get-------\r\n", int(ilcd.ep[3]), int(ilcd.ep[4]), ilcd.ep[5]);
		}
		bGnssLost = 0;
	}
	else
	{
		bGnssTimeOut += dt;
		if(bGnssTimeOut >= 0.5)
		{
			if (!bGnssLost)
			{
				gilc_log("%02d:%02d:%06.3f ----gnss lost-------\r\n", int(ilcd.ep[3]), int(ilcd.ep[4]), ilcd.ep[5]);
			}
			bGnssLost = 1;
		}
	}
	if(bGnssLost)
	{
		bGnssLostNum++; 
		bGnssNum = 0;
	}
	else
	{
		bGnssLostNum = 0;
		bGnssNum++; 
	}
}

void GIProcess::OdoIntegrity(CLCData &ilcd)
{
	/*dsf90:ODO失效时间判定*/
	if(ilcd.bODOavail)	//GNSS
	{
		bOdoTimeOut = 0;
		if (bOdoLost)
		{
			gilc_log("%02d:%02d:%06.3f ----Odo get-------\r\n", int(ilcd.ep[3]), int(ilcd.ep[4]), ilcd.ep[5]);
		}
		bOdoLost = 0;
	}
	else
	{
		bOdoTimeOut += dt;
		if(bOdoTimeOut >= 1)
		{
			if (!bOdoLost)
			{
				gilc_log("%02d:%02d:%06.3f ----Odo lost-------\r\n", int(ilcd.ep[3]), int(ilcd.ep[4]), ilcd.ep[5]);
			}
			bOdoLost = 1;
		}
	}
}

int GIProcess::GIPro_P2(CLCData ilcd)
{
	double *wmcur,*vmcur,wm[3],vm[3];
	int ret = 0;
//滤波器初始化
	if(!bAlign)
	{
		for (int i = 0; i<9; i++)
		{
			kf.davp[i] = para.davp[i];
		}

		for (int i = 0; i<3; i++)
		{
			kf.GB[i] = para.GB[i];//陀螺偏移
			kf.AB[i] = para.AB[i];//加计偏移
			kf.GW[i] = para.GW[i];//陀螺std
			kf.AW[i] = para.AW[i];//加计std
			kf.GS[i] = para.GS[i];//陀螺随机游走
			kf.AS[i] = para.AS[i];//加计随机游走
		}
		kf.kfinit();
		vector<double, malloc_allocator<double> >temppxk;
		for(int i=0;i<kf.ROW;i++)
		{temppxk.push_back(sqrt(kf.Pxk[i*kf.ROW+i]));}
		prePxk.push_back(temppxk);
		bAlign=true;
	}
	
	if(ilcd.bMEMSavail)
	{
		wmcur=ilcd.gyo; vmcur=ilcd.acc;
		
		/*dsf90:wm = 0.5*(wmpre + wmcur)*dt; wm, 角度变化增量 rad；疑问：dt使用前需先更新！*/
		/*dsf90:vm = 0.5*(vmpre + vmcur)*dt; vm, 速度变化增量 m/s*/
	//均值
		Maddn(wmpre,wmcur,wm,3,1);
		Maddn(vmpre,vmcur,vm,3,1);
		Mequalm(wmcur,3,1,wmpre);
		Mequalm(vmcur,3,1,vmpre);
		Mmul(wm,3,1,0.5);
		Mmul(vm,3,1,0.5);
		Mequalm(wm, 3, 1, ins.wib);
		Mequalm(vm, 3, 1, ins.fb);
		Mmul(wm,3,1,dt);
		Mmul(vm,3,1,dt);
	//需要准确的时间戳
		dt=ilcd.imutimetarge-tpre;
		dt_total += dt;
		tpre=ilcd.imutimetarge;
		/*
		if(ilcd.stat == 4) 
			kf.xflag |= (7<<18);
		else
			kf.xflag &= (~(7<<18));
		*/
		/*dsf90:预测*/
		ins.Update(wm,vm,dt);
		ins.Lever(); //臂杆和时间延迟补偿
		kf.upPhi(ins,dt);
		kf.TUpdate(dt);  //考虑GK
	}
	//判断是否有PPS
	if(ilcd.bPPSavail)
	{
		/*保存PPS时刻的ins信息*/
		inspre_forPPS = ins;
	}
	inspre = ins;
	ret = ZUpdate(ilcd);
	if(ilcd.bMEMSavail && ins.vm_car[1]>InitParam.dGnssVerMin_ForInit)
		updateKfInitStatus(ilcd);
	return ret;
}

int GIProcess::ZUpdate(CLCData ilcd)
{
	CSINS ppsins;
	int iUpdateZ_flag = 0;
	int iCalUpdateZ_flag = 0;
	int iUpdateType_flag = 0;
	int i = 0, j = 0,iRet = 0;
	double dpos[3]={0.0},dvn[3]={0.0};
	double enu[3];
	//double  rr0[3],rr1[3],drr[3];
	double dAtt[3] = {0},dvm[3]={0.0};
	double dVel = 0.0, dYawRate = 0.0,dWh = 0.0;
	double dVd = 0.0, dDualYaw = 0.0;
	c++; //imu更新计数

	double ppsinspos[3]={0.0},ppsinsvn[3]={0.0};
	double gnssHeading,insHeading,insYaw,gnssYaw,dheading;
	
	insHeading = atan2(ins.vnL[0],ins.vnL[1])*R2D;
	DEG_0_360(insHeading);
	insYaw = -ins.att_car[2]*R2D;
	DEG_0_360(insYaw);

	/*----速度姿态航向偏差----*/
	double dDiffHeadingYaw_ins = insHeading - insYaw;
	DEG_NEG180_180(dDiffHeadingYaw_ins);

	double wib_car[3]  = {0.0};
	double speed_car = 0.0;
	Mmulnm(ins.Cmb,ins.wib,3,3,1,wib_car);
	speed_car = sqrt(SQ(ins.vm_car[0])+SQ(ins.vm_car[1]));
	if(ins.vm_car[1]>0.28)
		ins.wheel_heading_raw = ins.wheel_heading = atan2(InitParam.dWheelBase*wib_car[2],speed_car)*R2D;

	/*----姿态航向变化率-----*/
	double dGyroZ = ins.wib[2]*R2D;
	//printf("dGyroZ %.3f........\r\n", dGyroZ);
	if(dGyroZ > 2)
		bTurn = 1;
	else if (dGyroZ < -2)
		bTurn = -1;
	else
		bTurn = 0;

	loadPPSSyncInsData(ilcd,ppsins);
	if(ilcd.bGPSavail)  //GNSS
	{	
		//GPS解状态窗
		gpsstate.push_back(ilcd.stat);
		if(gpsstate.size()>=10)
		{
			gpsstate.erase(gpsstate.begin());
		}
		//PPS时刻的INS位置
		Mequalm(ppsins.posL,3,1,ppsinspos);
		Mequalm(ppsins.vnL,3,1,ppsinsvn);

		Mminn(ppsinspos,ilcd.pos,dpos,3,1);
		Mminn(ppsinsvn,ilcd.vn,dvn,3,1);

		//位置+速度
		diffpos(ilcd.pos,ppsinspos,enu);
		difpos_b(ilcd.pos,ppsinspos,ppsins.att_car,dposb_sync);
		
		Mequalm(enu,3,1,dpos_sync);//pps时刻位置速度差值
		Mequalm(dvn,3,1,dvn_sync);
		
		/*车头姿态角*/
		//gnssYaw = ilcd.heading2*R2D;
		gnssYaw = ilcd.heading2;

		if (gnssYaw<0) gnssYaw += 360;	

		/*速度航向角*/
		gnssHeading = atan2(ilcd.vn[0],ilcd.vn[1])*R2D;
		if (gnssHeading<0) gnssHeading += 360;
	}
	
	dheading = insHeading - gnssHeading;
	DEG_NEG180_180(dheading);
		
	GnssIntegrity(ilcd,dheading,dvn);

	OdoIntegrity(ilcd);
	
	/*----静态判断----*/
	if(bStatic!=bStaticpre)
	{
		bStaticpre = bStatic;
		if (bStatic && ins.imutimetarge)
		{
			inspre_forStatic = ins;
		}
	}
	
#if 0
	/*dsf90,2018.9.20双天线航向修正*/
	if(ilcd.bGPSavail && (ilcd.stat == 4) && ilcd.GA_RK[2]>SQ(0.001) && ilcd.GA_RK[2] <= SQ(0.2))	//GNSS
	{
		/*车头姿态角*/
		double dYawBias = insYaw - gnssYaw;
		DEG_NEG180_180(dYawBias);

		if(!bDualAntAvail)
		{
			if(ilcd.heading2 && ilcd.GA_RK[2])
				bDualAntAvail = 1;
		}
		
		if (!bDualAntCalibrateOk)
		{
			if(!bDualAntInitOk)
			{
				if(!bTurn)
					getDualAntBias(ilcd, dYawBias); /*初始安装误差*/
			}
			else
			{
				if (!bGnssLost && ilcd.stat == 4 && ilcd.heading2  && ilcd.GA_RK[2] && !bTurn)
				{
					ins.dual_yaw   = insYaw;
					ins.dual_yaw_h = gnssYaw + dDualAntYawBias;
					
					dDualYaw = (ins.dual_yaw - ins.Byaw) - ins.dual_yaw_h;
					DEG_NEG180_180(dDualYaw);

					kf_cal.Rk[2] = ilcd.GA_RK[2] * SQ(R2D);
					iCalUpdateZ_flag |= CAL_UPDATE_Z_DUAL_YAW;

					if (kf_cal.xk_std[5] < 0.005)
						bDualAntCalibrateOk = 1; //完成安装误差标定
				}
				bDualAntCalibrateOk = 1; //完成安装误差标定
			}
		}
		
		if (bDualAntCalibrateOk)
		{
			dDiffYaw_Ins2DualAnt = insYaw - (gnssYaw + dDualAntYawBias);
			DEG_NEG180_180(dDiffYaw_Ins2DualAnt);
			if (fabs(dDiffYaw_Ins2DualAnt)<2)	//dsf90:数据跳变，认为GNSS异常
			{
				dAtt[2] = dDiffYaw_Ins2DualAnt * D2R;
				kf.Rk[2] = ilcd.GA_RK[2];
				iUpdateZ_flag |= UPDATE_Z_ATT_Z;
			}
		}
	}
#endif

#if 1 /*dsf90,2018.11.12轮速*/
	kf.setRk_constraint();
	if(ilcd.bODOavail)
	{
		static int iOdoVelCnt = 0;
		static int iOdoHeadingCnt = 0;
		static int iOdoHeadingCnt2 = 0;
		static double dOdoWheelVelScale = 0.0;
		static double dOdoWheelHeadingKwh = 0.0;
		static double dOdoWheelHeadingBwh = 0.0;

		ilcd.stOdoData.wheel_vel_std   /= 3.6;
		ilcd.stOdoData.wheel_vel_left  /= 3.6;
		ilcd.stOdoData.wheel_vel_right /= 3.6;
		if(!ilcd.stOdoData.wheel_vel)
			ilcd.stOdoData.wheel_vel = (ilcd.stOdoData.wheel_vel_left + ilcd.stOdoData.wheel_vel_right)/2.0;

		if(ilcd.stOdoData.wheel_mode == 2)/*倒车*/
			ilcd.stOdoData.wheel_vel *= -1;

		//gilc_log("ODO Kd, %d %.3f %.3f %.3f %.3f\r\n", ilcd.stOdoData.wheel_mode, ilcd.stOdoData.wheel_vel, ilcd.stOdoData.wheel_vel_left, ilcd.stOdoData.wheel_vel_right, ilcd.stOdoData.wheel_heading);
		
		if (iOdoVelCnt<=50)/*计算Kd初始值*/
		{
			if (!bGnssLost && ilcd.stat == 4 && ilcd.gnss_speed > InitParam.dGnssVerMin_ForInit * 2 && ilcd.stOdoData.wheel_vel)
			{
				if (iOdoVelCnt < 50)
				{
					dOdoWheelVelScale += ilcd.gnss_speed/ilcd.stOdoData.wheel_vel;
				}
				iOdoVelCnt++;
				if (iOdoVelCnt == 50)
				{
					dOdoWheelVelScale /= iOdoVelCnt;
					ins.Kd = dOdoWheelVelScale;
					bWheelVelInitOk = 1;
					kf_cal.kfinit();
					gilc_log("ODO Kd init, %.3f\r\n",ins.Kd);
				}
			}
		}
		else
		{
			static int cnt = 0;
			ins.wheel_vel	= ins.vm_car[1];
			ins.wheel_vel_h = ilcd.stOdoData.wheel_vel;
			dVel = ins.wheel_vel - ins.Kd*ins.wheel_vel_h;
			if (ins.Kd)
				dVd = ins.wheel_vel / ins.Kd - ins.wheel_vel_h;
			/*轮转角不可用，只用轮速*/
			//kf.Rk[12] = ilcd.GPV_RK[0*6+0]+ilcd.GPV_RK[1*6+1];
			if (!bGnssLost && ilcd.stat == 4 && ilcd.gnss_speed > InitParam.dGnssVerMin_ForInit * 3 && cnt < 5 * 60 * 50)
			{
				kf_cal.Rk[1] = SQR(ilcd.stOdoData.wheel_vel_std);
				iCalUpdateZ_flag |= CAL_UPDATE_Z_WHEEL_VEL;
				cnt++;
			}
			if(bInstallOk && cnt > 60 * 50)
			{
				if (fabs(dVel) < 2 && ilcd.stOdoData.wheel_vel_std != 0 )
				//if (bGnssTimeOut >= 2 && fabs(dVel) < 2 && ilcd.stOdoData.wheel_vel_std !=0 )
				{
					if (ins.wheel_vel > 0.028 || (ins.wheel_vel <= -0.28 && ilcd.stOdoData.wheel_mode == 2))
					{
						dvm[1] = dVel;
						//kf.Rk[10] = SQR(1/3.6);
						kf.Rk[10] = SQR(ilcd.stOdoData.wheel_vel_std);
						iUpdateZ_flag |= UPDATE_Z_CONS_VER_Y;
					}
				}
			}
		}
#if 0
		if(!bWheelHeadingCalibrateOk)
		{
			if (!bWheelHeadingInitOk)//计算Kwh初始值
			{
				if (!bGnssLost && ilcd.stat == 4 && ilcd.gnss_speed > InitParam.dGnssVerMin_ForInit && ilcd.stOdoData.wheel_heading)
				{
					if(iOdoHeadingCnt2 < 20)
					{
						dOdoWheelHeadingKwh += ilcd.stOdoData.wheel_heading/ins.wheel_heading;
						iOdoHeadingCnt2++;
						if(iOdoHeadingCnt2==20)
							dOdoWheelHeadingKwh /= 20;
					}
					
					if (iOdoHeadingCnt2==20)
					{
						ins.WheelHeadingScale = dOdoWheelHeadingKwh;
						ins.Kwh = 1;
						ins.Bwh = 0;
						bWheelHeadingInitOk = 1;
						gilc_log("ODO Kwh init, %.3f\r\n", ins.Kwh);
					}
				}
			}

			if (bWheelHeadingInitOk)
			{
				ins.wheel_heading_h = ilcd.stOdoData.wheel_heading/ dOdoWheelHeadingKwh;
				dWh = (ins.wheel_heading - ins.Bwh) / ins.Kwh - ins.wheel_heading_h;
				if (!bGnssLost  && ilcd.stat == 4 && ins.vm_car[1] > 2  && ins.wheel_heading > 2)
				{
					kf_cal.Rk[0] = SQR(InitParam.dKfInitP_OdoHeading);
					iCalUpdateZ_flag |= CAL_UPDATE_Z_WHEEL_HEADING;
					//printf("%6.6f  %6.6f  %6.6f  %6.6f	%6.6f\r\n", dWh, ins.wheel_heading_h, ins.Kwh, ins.Bwh, ins.Kd);
				}
				//if(bInstallOk)
					//bWheelHeadingCalibrateOk = 1; 
			}
		}
		
		if(bInstallOk && bWheelHeadingCalibrateOk)
		{
			ins.wheel_heading_h = ilcd.stOdoData.wheel_heading/ ins.WheelHeadingScale;
			ins.yaw_rate_byodo = tan((ins.wheel_heading_h*ins.Kwh + ins.Bwh)*D2R)*ins.wheel_vel / para.dWheelBase;
			dYawRate = ins.wib[2] - ins.yaw_rate_byodo;
			if (bGnssTimeOut >= 2 && ins.vm_car[1] > 0.28 && fabs(dYawRate)< 5*D2R)
			{
				kf.Rk[12] = SQR(0.1);
				iUpdateZ_flag |= UPDATE_Z_ODO_HEADING;
			}
		}
#endif
	}
	else
	{
		/*使用约束*/
		dvm[0] = ins.vm_car[0];
		dvm[1] = 0;
		dvm[2] = ins.vm_car[2];
		iUpdateZ_flag |= UPDATE_Z_CONS_VER_XZ;	
	}
#endif	
#if 0
	//if(NUMX==21 && iCalUpdateZ_flag && ins.Kwh && ins.Kd)
	if (NUMX == 21 && iCalUpdateZ_flag)
	{
		double zk[3] = { dWh,dVd,dDualYaw};
		kf_cal.upPhi(ins,dt);
		kf_cal.TUpdate(dt,0);  //考虑GK 			
		kf_cal.MUpadte(ins, zk, iCalUpdateZ_flag);
		kf_cal.Feedback(ins);
		//printf("%6.6f  %6.6f  %6.6f  %6.6f  %6.6f  %6.6f  %6.6f  %6.6f  %6.6f\r\n", dWh, dVd, dDualYaw, ins.Kwh, ins.Bwh, ins.Kd, ins.Byaw, sqrt(kf_cal.Pxk[kf_cal.ROW*3+3]), sqrt(kf_cal.Pxk[kf_cal.ROW*5+5]));
	}
#endif
	//零速更新
	if(bStatic)
	{	
		//if (!bGnssLost || ilcd.bGPSavail || ilcd.stat == 4)
		{
			double dPos[3] = { 0 };
			Mminn(ins.pos, inspre_forStatic.pos, dPos, 3, 1);
			
			//if(!bDualAntAvail || bGnssLost)/*单天线或不搜星，姿态航向约束*/
			{
				Mminn(ins.att_car, inspre_forStatic.att_car, dAtt, 3, 1);
				//iUpdateZ_flag |=  UPDATE_Z_ATT_Z;//无人船上航向约束不可用
			}

			double zk[NUMV] = { -dAtt[0],-dAtt[1],-dAtt[2],ins.vn[0],ins.vn[1],ins.vn[2],dPos[0],dPos[1],dPos[2],dvm[0],dvm[1],dvm[2],dYawRate};

			//iUpdateZ_flag = UPDATE_Z_ATT_Z| UPDATE_Z_VER_XYZ|UPDATE_Z_POS_XYZ;
			iUpdateZ_flag |=  UPDATE_Z_VER_XYZ;
			iUpdateType_flag = UPDATE_TYPE_STATIC | UPDATE_TYPE_CONSTRAINT;
			if (iUpdateZ_flag)
			{
				//if(bInstallOk)
				//	iUpdateZ_flag &= (~UPDATE_Z_VER_XYZ);
				kf.MUpadte(ins, zk, iUpdateZ_flag, iUpdateType_flag);
				kf.Feedback(ins, dt);
			}
			if (!ilcd.bGPSavail)
				return 1;
			else
				return 5;
		}
	}

	if(!ilcd.bGPSavail)  //无GNSS，INS机械编排阶段
	{	
		/*dsf90:侧滑约束*/
		double zk[NUMV]={0,0,0,0,0,0,0,0,0,dvm[0],dvm[1],dvm[2],dYawRate};

		if(bTurn)//无人船转弯的时候加入侧向速度约束
		{
			double dVx = fabs(ins.wib[2])*InitParam.dWheelBase;
			//dVx /= 10;
			if (dVx < 0.1)	dVx = 0.1;
			//if (dVx < 0.1)	dVx = 0.1;
			kf.Rk[9]  = SQR(dVx);
			/*
			double dVz = fabs(ins.wib[0])*para.dWheelBase;
			dVz /= 10;
			if (dVz < 0.01)	
				dVz = 0.01;
			else if (dVz > 0.1)
				dVz = 0.1;
			kf.Rk[11] = SQR(dVz);
			*/
			//iUpdateZ_flag &= (~UPDATE_Z_CONS_VER_X);/*不使用约束效果较差*/
		}

		iUpdateType_flag = UPDATE_TYPE_CONSTRAINT;
		if (iUpdateZ_flag)
		{
			//if(bInstallOk)
			//	iUpdateZ_flag &= (~UPDATE_Z_VER_XYZ);
			kf.MUpadte(ins, zk, iUpdateZ_flag, iUpdateType_flag);
			kf.Feedback(ins,dt);
		}
#if 1
		/*dsf90:位置结果约束（限位）*/
		correctSideslip();
#endif
		return 1;
	}

	double zk[NUMV]={dAtt[0],dAtt[1],dAtt[2],dvn[0],dvn[1],dvn[2],dpos[0],dpos[1],dpos[2],dvm[0],dvm[1],dvm[2],dYawRate};
#if 1  //GNSS跳点-不更新或速度更新
	if((fabs(enu[0])>2.0||fabs(enu[1])>2.0||fabs(enu[2])>2.0) && c<1000)
	{
		//gilc_log("   跳点：enu>0.5\n");
		kf.resetRk(ilcd);
		num_GPSskip++;
		if((ilcd.stat==GNSSFIX || ilcd.stat == GNSSFLOAT) && busegnssvn)
		{	
			//GNSS长时间中断，恢复时间长
			double PosRk[3]={0.0};
			kf.downgrade_car(ilcd,enu,PosRk);
			/*pos*/			
			kf.Rk[6]=PosRk[0];
			kf.Rk[7]=PosRk[1];
			kf.Rk[8]=PosRk[2];
			iUpdateZ_flag |= UPDATE_Z_VER_XYZ|UPDATE_Z_POS_XYZ;
			iUpdateType_flag |= UPDATE_TYPE_GNSS;
			c=0;
		}
		else if(busegnssvn)  //位置跳变，但速度可用
		{
			iUpdateZ_flag |= UPDATE_Z_VER_XYZ;
			iUpdateType_flag |= UPDATE_TYPE_GNSS;
			c=0;
		}
		if(iUpdateZ_flag)
		{
			//if(bInstallOk)
			//	iUpdateZ_flag &= (~UPDATE_Z_VER_XYZ);
			kf.MUpadte(ins,zk,iUpdateZ_flag,iUpdateType_flag);			
			kf.Feedback(ins,0.2);		
		}
        //位置速度均跳变，可能是GNSS异常，也可能是INS累积误差大
		//防止是INS发散，若连续10秒跳变，则置信GNSS
		if(num_GPSskip<25) 
		{
			return 2;
		}  //不做融合
	}
#endif 

#if 1   //长时间中断20s或定位差连续大于阈值--,GPS恢复阶段 - 速度和位置设为GNSS的结果
	//需要的话就开个窗
	if(c>999 || num_GPSskip>24)
	{
		kf.resetRk(ilcd);
		num_GNSSrestore++;
		//if((ilcd.nsused>=10 && ilcd.hdop<=1.0) || num_GNSSrestore>25)
		//if((ilcd.nsused>=7 && ilcd.hdop<=2.0) || num_GNSSrestore>20)
		if((ilcd.nsused>=8 || ilcd.hdop<=2.0) || num_GNSSrestore>20)
		{
			//GNSS搜星数（用于解算）和PDOP满足条件，GNSS定位质量较好
			iUpdateZ_flag |= UPDATE_Z_VER_XYZ|UPDATE_Z_POS_XYZ;
			iUpdateType_flag |= UPDATE_TYPE_GNSS;
			num_GNSSmupdate++;
		}
		//else
		//{//GNSS定位质量差，位置降权，用速度更新
		//	//double PosRk[3]={0.0};
		//	//kf.downgrade_car(ilcd,enu,PosRk);
		//	//kf.resetRK2(ilcd,PosRk);

		//	kf.MUpdate_veloicty(ilcd,dvn);
		//	kf.Feedback(ins);

		//	//num_GNSSmupdate=0;
		//}
		//退出恢复窗口 —— GNSS搜星数（用于解算）和PDOP满足条件，GNSS定位质量较好 4次
		if(num_GNSSmupdate>4)
		{
			num_GPSskip=0;
			num_GNSSmupdate=0;
			num_GNSSrestore=0;
			c=0;
			num_ContinueFix=5;
	        num_ContinueFloat=50;
		}
		if(iUpdateZ_flag)
		{
			//if(bInstallOk)
			//	iUpdateZ_flag &= (~UPDATE_Z_VER_XYZ);
			kf.MUpadte(ins,zk,iUpdateZ_flag,iUpdateType_flag);			
			kf.Feedback(ins,0.1);
		}        
	    return 2;
	}
#endif

	/*dsf90:GNSS数据可用，继续*/
	kf.resetRk(ilcd);
    c=0; //INS更新计数清零
	//对异常速度进行探测
	if(!busegnssvn) //gnss速度不可用
	{
		if(ilcd.stat==GNSSFIX)          //固定
		{
			num_ContinueFix++;	
			if(num_ContinueFix>5)       //连续固定数目>6,直接使用GNSS给出的方差信息，初始化为6
			{
			
			}
			else                        //连续固定数目<6（浮点跳固定），逐渐使用GNSS的权，平滑过渡
			{
				double PosRk[3]={0.0};
				kf.downgrade(ilcd,(6-num_ContinueFix),PosRk);
				/*pos*/
				kf.Rk[6]=PosRk[0];
				kf.Rk[7]=PosRk[1];
				kf.Rk[8]=PosRk[2];
			}
			num_ContinueFloat=0;        //连续浮点数目置零
		}
		else                            //非固定
		{
			num_ContinueFloat++;  
			if(num_ContinueFloat<10)   //固定跳浮点，2秒稳定期，使用denu和GNSS的方差信息确定观测值权，防止GNSS跳点
			{
				double PosRk[3]={0.0};				
				kf.downgrade(ilcd,(11-num_ContinueFloat),PosRk);
				//kf.downgrade_car(ilcd,enu,PosRk);
				/*pos*/
				kf.Rk[6]=PosRk[0];
				kf.Rk[7]=PosRk[1];
				kf.Rk[8]=PosRk[2];
			}
			else                        //GNSS精度趋于稳定，不在对GNSS降权，防止INS发散
			{
			
			}
			num_ContinueFix=0;          //连续固定数目置零
		}
		iUpdateZ_flag |= UPDATE_Z_POS_XYZ;
		iUpdateType_flag |= UPDATE_TYPE_GNSS;
		upmodel=3;
	}
	else
	{
		if(ilcd.stat==GNSSFIX)             //固定
		{
			num_ContinueFix++;   
			if(num_ContinueFix>5)          //连续固定数目>6,直接使用GNSS给出的方差信息，初始化为6
			{
			
			}          
			else
			{
				double PosRk[3]={0.0};
				kf.downgrade(ilcd,(6-num_ContinueFix),PosRk);
				/*pos*/
				kf.Rk[6]=PosRk[0];
				kf.Rk[7]=PosRk[1];
				kf.Rk[8]=PosRk[2];
			}
			num_ContinueFloat=0;
		}  
		else                               //非固定
		{
			num_ContinueFloat++;
			if(num_ContinueFloat<10) //100
			{
				double PosRk[3]={0.0};
				kf.downgrade(ilcd,(11-num_ContinueFloat),PosRk);
				//kf.downgrade_car(ilcd,enu,PosRk);
				/*pos*/
				kf.Rk[6]=PosRk[0];
				kf.Rk[7]=PosRk[1];
				kf.Rk[8]=PosRk[2];
			}
			else
			{

			}
			num_ContinueFix=0;          //连续固定数目置零
		}
		iUpdateZ_flag |= UPDATE_Z_VER_XYZ|UPDATE_Z_POS_XYZ;
		iUpdateType_flag |= UPDATE_TYPE_GNSS;
		upmodel=23;
	}
	
	if(iUpdateZ_flag)
	{
		//if(bInstallOk)
		//	iUpdateZ_flag &= (~UPDATE_Z_VER_XYZ);
		kf.MUpadte(ins,zk,iUpdateZ_flag,iUpdateType_flag);	
		kf.Feedback(ins,0.2);
	} 
	
	Mequalm(kf.xk,kf.ROW,1,kf.xkpre);
	Mequalm(dpos,3,1,kf.dpos);
	Mequalm(enu,3,1,kf.denu);

	num_GPSskip=0;
	return 2;
}

extern void equalgpos(gpos_t* GP,CLCData* lcdata)
{
	GP->lat=lcdata->pos[0];
	GP->lon=lcdata->pos[1];
	GP->hig=lcdata->pos[2];
	GP->ve=lcdata->vn[0];
	GP->vn=lcdata->vn[1];
	GP->vu=lcdata->vn[2];
	GP->state=lcdata->stat;
	//GP->yaw=lcdata->yaw_U62R;
	GP->yaw=lcdata->heading2*D2R;
}
void diffpos(double blhpre[3],double blhcur[3],double denu[3])
{
    double rr0[3]={0.0},rr1[3]={0.0},drr[3]={0.0};
	pos2ecef(blhpre,rr0);
	pos2ecef(blhcur,rr1);
	Mminn(rr1,rr0,drr,3,1);
	ecef2enu(blhpre,drr,denu);
}

/*dsf90:计算载体坐标系下，两个位置差*/
extern void difpos_b(double pospre[3],double poscur[3],double att[3],double dpos_b[3])
{
	double rr0[3]={0.0},rr1[3]={0.0},drr[3]={0.0},denu[3]={0};
	pos2ecef(pospre,rr0);
	pos2ecef(poscur,rr1);
	Mminn(rr1,rr0,drr,3,1);
	ecef2enu(pospre,drr,denu);

	double Cnb[9]={0},Cbn[9];
	a2mat(att,Cnb);
	Mtn(Cnb,3,3,Cbn);
	Mmulnm(Cbn,denu,3,3,1,dpos_b);

}
/*dsf90：根据b系下的位移反算ENU位置*/
extern void difpos(double pospre[3],double poscur[3],double att[3],double dpos_b[3])
{
	double rr0[3]={0.0},rr1[3]={0.0},drr[3]={0.0},denu[3]={0};;

	double Cnb[9]={0};
	a2mat(att,Cnb);
	Mmulnm(Cnb,dpos_b,3,3,1,denu);

	enu2ecef(pospre,denu,drr);
	pos2ecef(pospre,rr0);
	Maddn(rr0,drr,rr1,3,1);

	ecef2pos(rr1,poscur);
}
#if 0
/*add by dsf90,2018.5.28
opt: 1, 强制保留当前输入数据
*/
extern int AccBias_monitor(CSINS& ins,double xk[3],int opt)
{
	static vector<double, malloc_allocator<double> > x[3];
	static long cnt_times = 0;
	double std[3] = {0};
	double ave[3] = {0};
	double Ab[3] = { 0 };
	int i = 0;
	int iRet = 0;

	Maddn(ins.db, xk, Ab, 3, 1);

	if (opt == 0)
	{
		if (x[0].size() < 200)
		{
			for (i = 0; i < 3; i++)
				x[i].push_back(Ab[i]);
		}
		else
		{
			for (i = 0; i < 3; i++)
			{
				x[i].erase(x[i].begin());
				x[i].push_back(Ab[i]);
			}
		}
		return 0;
	}

	cnt_times++;
	if (cnt_times < 1500)
	{
		return 0;
	}

	if (x[0].size() < 200)
	{
		for (i = 0; i < 3; i++)
			x[i].push_back(Ab[i]);
		return 0;
	}
	else
	{
		for (i = 0; i < 3; i++)
		{
			ave[i] = GetAveStd(x[i], 0);/*dsf90:均值*/
			std[i] = GetAveStd(x[i], 1);/*dsf90:标准差*/
			if (fabs(Ab[i] - ave[i]) > 3 * std[i])
			{
				iRet = 1;
				break;
			}
			if (fabs(Ab[i] - ave[i]) > 6 * std[i])
			{
				iRet = 2;
				break;
			}
		}
		if (iRet && (opt == 2))
			return iRet;
		for (i = 0; i < 3; i++)
		{
			x[i].erase(x[i].begin()); 
			x[i].push_back(Ab[i]);
		}
	}
	return iRet;
}
#endif
