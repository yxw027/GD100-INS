/*
 * Author   : dsf90
 * Date     : 2018.12.7
 * Breif    : data log for gilc.
 * Version  : 0.1
 * Hisotry  : first created
 * Form     : ANSI
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "log.h"
#include "GILC_IOFile.h"

/*dsf90:2018.1217,实测库log函数名不能与进程log函数同名*/
void gilc_log(char* fmt,...)  
{  
	char buf[2048] = {0};
	va_list ap;
	va_start(ap,fmt);
	vsprintf((char*)buf,fmt,ap);
	va_end(ap);
	fprintf_log(buf);
	gilc_printf(buf);
}

void gilc_log_hex(char *head,int line_size,void *data,int size)  
{ 
    int i; 
	unsigned char *addr = (unsigned char *)data;
	char buf[2048] = {0};
	sprintf(buf,head); 
	for(i=0;i<size;i++) 
	{ 
		if(i%line_size==0) 
		{ 
			sprintf(buf+strlen(buf),"\r\n"); 
		} 
		sprintf(buf+strlen(buf),"%02X ",addr[i]); 
	} 
	sprintf(buf+strlen(buf),"\r\n"); 
	gilc_log(buf);
}

