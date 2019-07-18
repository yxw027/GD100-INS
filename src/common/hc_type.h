#ifndef __HC_TYPEDEF_H__
#define __HC_TYPEDEF_H__

#include <stdint.h>
#ifndef STM32
#include <stdio.h>
#endif
#include <string.h>

//#define SYS_STM32
#include "log.h"

#ifdef SYS_STM32
#include <delay.h>
#include "app_log.h"

#define _inline inline

#include "malloc.h"  

#undef malloc
#undef free

#define hc_malloc(size)  mymalloc(SRAMIN,size)
#define hc_free(addr)    myfree(SRAMIN,addr)

#endif

typedef const uint32_t uc32;  /*!< Read Only */
typedef const uint16_t uc16;  /*!< Read Only */
typedef const uint8_t uc8;   /*!< Read Only */

typedef char               HC_CHAR;
typedef signed char        HC_INT8;
typedef signed short       HC_INT16;
typedef signed int         HC_INT32;
typedef signed long long   HC_INT64;

typedef unsigned char       HC_UINT8;
typedef unsigned short      HC_UINT16;
typedef unsigned int        HC_UINT32;
typedef unsigned long long  HC_UINT64;

//typedef void HC_VOID;
typedef int  HC_BOOL;
#define HC_VOID void


typedef unsigned long HC_ULONG;
typedef double HC_DOUBLE;
typedef float  HC_FLOAT;
typedef long HC_LONG;

#define HC_OK    0
#define HC_ERR  -1

#define HC_TRUE  1
#define HC_FALSE 0

#define HC_IN_PROCESS   1

#define HC_ON   1
#define HC_OFF  0

#define HC_EOF  1

/*HC_OUT, HC_IN, HC_IN_OUT used by function parmeter */
#define HC_OUT
#define HC_IN
#define HC_IN_OUT

/* lw: according to libc stddef.h  */
#ifdef __cplusplus
#define HC_NULL 0
#else
#define HC_NULL ((void *)0)
#endif

#define ARRAY_SIZE(x) ((unsigned int)(sizeof(x)/sizeof((x)[0])))

#define ALIGN_UP(num, size) (((num) + (size - 1)) & (~(size -1)))
#define ALIGN_DOWN(num,size) ((num) & (~(size -1)))


/* convert parmeter x to a string */
#define str(x)  #x

/* convert the value of macro to string  */
#define xstr(x) str(x)

#ifdef SYS_STM32
#define usleep(a)  delay_ms((a)/1000)
#endif

#define HC_POLYCRC32 0xEDB88320u
static inline unsigned int hc_crc32(void *data, int len)
{
    unsigned int crc=0;
	unsigned char *buff = (unsigned char *)data;
    int i,j;

    for (i=0;i<len;i++) {
        crc^=buff[i];
        for (j=0;j<8;j++) {
            if (crc&1) crc=(crc>>1)^HC_POLYCRC32; else crc>>=1;
        }
    }
    return crc;
}

#endif

