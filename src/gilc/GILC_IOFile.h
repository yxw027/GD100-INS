#ifndef _GILC_IOFILE_H
#define _GILC_IOFILE_H

#include "GILC.h"

// GNSS/MEMS file
class DebugFile
{
public:
	FILE *foutins, *foutkf, *foutposLC, *foutposG, *foutposNmea, *fraw, *fpro;
	bool bInit;
	int numins;
	
	int Init(char *outFilePath, bool bOutFileSave, char *tmpFilePath, bool bTmpFileSave);
	int SaveRaw(CLCData *ilcd);
	int SaveRst(int stat,GIProcess *gipro,CLCData *ilcd);
	int Close(void);
};

/*---------------------------------*/
void fprintf_log(char* msg);
void printf_imu(FILE* fp,CLCData *ilcd);
void printf_nav(FILE *fp, CSINS *ins, int num);
void printf_posLC(FILE *fp, CSINS *ins, CLCData *ilcd);
void printf_posAnt(FILE *fp, CSINS *ins, CLCData *ilcd);
void printf_posG(FILE *fp, CLCData *ilcd);
void printf_kf(FILE *fp, GIKF& kf, int num);
void printf_kf(FILE *fp, GIKF& kf, double gpstimetarget);
void printf_gga(FILE *fp, CSINS *ins, CLCData *ilcd);
void printf_posNmea(FILE *fp, char *buf);
void printf_prosess(FILE *fp, GIProcess *gipro, CLCData *ilcd);
void printf_raw(FILE *fp,CLCData *ilcd);

#endif
