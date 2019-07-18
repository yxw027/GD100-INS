/*
 * Author   : dsf90
 * Date     : 2018.8.16
 * Breif    : data save for gilc.
 * Version  : 0.1
 * Hisotry  : first created
 * Form     : ANSI
 */
#ifndef _GILC_LOG_H__
#define _GILC_LOG_H__

#ifdef __cplusplus
extern "C" {
#endif				/*__cplusplus*/

#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL 4
#endif

#if(DEBUG_LEVEL>=5)
	#define logt gilc_log
#else
	#define logt {}
#endif

#if(DEBUG_LEVEL>=4)
	#define gilc_printf printf
#else
	#define gilc_printf {}
#endif

#if(DEBUG_LEVEL>=3)
	#define logd gilc_log
#else
	#define logd {}
#endif

#if(DEBUG_LEVEL>=2)
	#define logi gilc_log
#else
	#define logi {}
#endif

#if(DEBUG_LEVEL>=1)
	#define loge gilc_log
#else
	#define loge {}
#endif

extern FILE *s_flog;
void gilc_log(char* fmt,...);
void gilc_log_hex(char *head, int line_size, void *data, int size);

#ifdef __cplusplus
}
#endif			/*__cplusplus*/

#endif
