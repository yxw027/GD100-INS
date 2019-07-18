#ifndef __BUFFER_H__
#define __BUFFER_H__

#ifdef __cplusplus
extern "C" {
#endif           /*__cplusplus*/
#include "hc_type.h"

/*********************************************buffer*************************************/
typedef struct __buffer_struct
{
	HC_INT32 m_iSize;			/*buffer已使用的大小,单位:byte*/
	HC_INT32 m_iMaxSize;		/*buffer总大小,单位:byte*/
	HC_INT32 m_iStart;			/*空白空间起始位置*/
	HC_INT32 m_iEnd;			/*空白空间结束位置*/
	HC_UINT8 * m_pucBuf;		/*buffer起始地址*/
} BUFFER_STRUCT;

BUFFER_STRUCT *create_buffer( HC_INT32 iSize );
HC_VOID release_buffer( BUFFER_STRUCT* pstBuf );
HC_INT32 add_to_buffer( BUFFER_STRUCT* pstBuf, HC_UINT8 * pucData, HC_INT32 iLen );
HC_INT32 remove_from_buffer(BUFFER_STRUCT* pstBuf, HC_INT32 iLen);
HC_INT32 get_from_buffer(BUFFER_STRUCT* pstBuf, HC_UINT8 *pucData, HC_INT32 iLen);
HC_INT32 get_from_buffer_withremove(BUFFER_STRUCT* pstBuf, HC_UINT8 *pucData, HC_INT32 iLen);
HC_VOID clean_buffer(BUFFER_STRUCT *pstBuf);

#ifdef __cplusplus
}
#endif				/*__cplusplus*/
#endif				/*__BUFFER_H__*/

