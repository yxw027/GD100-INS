/*
 * GILC_VehicleMeasure_lib.h
 *
 *  Created on: May 9, 2018
 *      Author: dsf
 *        Form: ANSI
*/

#ifndef GILC_VEHICLE_INNER_H
#define GILC_VEHICLE_INNER_H

#include "GILC_Vehicle_lib.h"

#pragma pack (4)
typedef struct gilc_result_inner {
	double second;    //sec of gnss week
	int week;
	double dpos_sync[3];//enu m//m//m
}gilc_result_inner_t;
#pragma pack ()

void printf_rtkplot(FILE *fd,int week,double imutimetarge,double pos[3],int stat);
double difpos_enu(double pospre[3], double poscur[3], double dpos_enu[3]);
void GILC_process_backup(void);
void GILC_process_recovery(void);
int GILC_get_result_inner(gilc_result_inner_t *pstRstInner);

#endif
