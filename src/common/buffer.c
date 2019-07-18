#include <stdlib.h>
#include "buffer.h"

#define buf_malloc(size)  malloc(size)
#define buf_free(addr)    free(addr)

/********************************************************************************
*Function Name: create_buffer
*Description:  创建一块buffer空间,起始内容为0
*Input Params: @iSize - buffer大小
*Out Params:   none
*Return Value: 返回该buffer的管理结构体
*********************************************************************************
*Modified time        Version      Modified by       Modified Content
*  2016/9/28            V1.0          lcy                create    
*********************************************************************************/
BUFFER_STRUCT *create_buffer( HC_INT32 iSize )
{
	//BUFFER_STRUCT *pstBuf = ( BUFFER_STRUCT * ) calloc( 1, sizeof( BUFFER_STRUCT ) );
	//pstBuf->m_pucBuf = ( HC_UINT8 *) calloc( iSize, sizeof(HC_UINT8) );
	BUFFER_STRUCT *pstBuf = ( BUFFER_STRUCT * ) buf_malloc( sizeof( BUFFER_STRUCT ) );
	if(!pstBuf) 
	{
		loge("malloc err\r\n");
	}
	pstBuf->m_pucBuf = ( HC_UINT8 *) buf_malloc( iSize );
	if(!pstBuf->m_pucBuf) 
	{
		loge("malloc err\r\n");
	}	
	pstBuf->m_iMaxSize = iSize;
	pstBuf->m_iEnd = pstBuf->m_iStart =  pstBuf->m_iSize =0;
	return pstBuf;
}
/********************************************************************************
*Function Name: release_buffer
*Description:  释放由create_buffer创建的空间
*Input Params: @pstBuf - 对应的buffer管理结构体
*Out Params:   none
*Return Value: none
*********************************************************************************
*Modified time        Version      Modified by       Modified Content
*  2016/9/28            V1.0          lcy                create    
*********************************************************************************/

HC_VOID release_buffer( BUFFER_STRUCT* pstBuf )
{
	//free( pstBuf->m_pucBuf );
	//free( pstBuf );
	buf_free( pstBuf->m_pucBuf );
	buf_free( pstBuf );
}

/********************************************************************************
*Function Name: add_to_buffer
*Description:  向buffer中加入一段长度的数据
*Input Params: @pstBuf - 对应的buffer管理结构体
			   @pucData - 要加入数据的起始地址
			   @iLen - 要加入数据的长度
*Out Params:   none
*Return Value: 0 - success, -1 -  fail
*********************************************************************************
*Modified time        Version      Modified by       Modified Content
*  2016/9/28            V1.0          lcy                create    
*********************************************************************************/
HC_INT32 add_to_buffer( BUFFER_STRUCT* pstBuf, HC_UINT8 * pucData, HC_INT32 iLen )
{
	HC_INT32 i = 0;

	if (iLen + pstBuf->m_iSize > pstBuf->m_iMaxSize)
	{
		return HC_ERR;
	}
	
	for (i = 0; i < iLen; ++i)
	{
		if ( ( pstBuf->m_iStart + 1 ) % ( pstBuf->m_iMaxSize ) == pstBuf->m_iEnd )
		{
			return HC_ERR;
		}
		pstBuf->m_pucBuf[pstBuf->m_iStart] = pucData[i];
		//printf("%c",pstBuf->m_pucBuf[pstBuf->m_iStart]);
		pstBuf->m_iStart = (pstBuf->m_iStart + 1) % (pstBuf->m_iMaxSize);
		(pstBuf->m_iSize)++;
	}
	//printf("buf size: %d\r\n", pstBuf->m_iSize);
	return HC_OK;
}

/********************************************************************************
*Function Name: remove_from_buffer
*Description:  从buffer中删除一段长度的数据
*Input Params: @pstBuf - 对应的buffer管理结构体
			   @iLen - 要加入数据的长度
*Out Params:   none
*Return Value: 0 - success, -1 -  fail
*********************************************************************************
*Modified time        Version      Modified by       Modified Content
*  2016/9/28            V1.0          lcy                create    
*********************************************************************************/
HC_INT32 remove_from_buffer(BUFFER_STRUCT* pstBuf, HC_INT32 iLen)
{
	if (iLen > pstBuf->m_iSize)
	{
		iLen = pstBuf->m_iSize;
	}
	pstBuf->m_iEnd = (pstBuf->m_iEnd + iLen) % (pstBuf->m_iMaxSize);
	pstBuf->m_iSize -= iLen;
	return HC_OK;
}

/********************************************************************************
*Function Name: get_from_buffer
*Description:  从buffer中取出一段长度的数据
*Input Params: @pstBuf - 对应的buffer管理结构体
			   @iLen - 要加入数据的长度
*Out Params:   @pucData - 存放取出的数据
*Return Value: 返回读取的数据长度
*********************************************************************************
*Modified time        Version      Modified by       Modified Content
*  2016/9/28            V1.0          lcy                create    
*********************************************************************************/
HC_INT32 get_from_buffer(BUFFER_STRUCT* pstBuf, HC_UINT8 *pucData, HC_INT32 iLen)
{
	HC_INT32 iCnt = 0;
	if (iLen > pstBuf->m_iSize)
	{
		iLen = pstBuf->m_iSize;
	}
	while (iCnt < iLen)
	{
		pucData[iCnt] = pstBuf->m_pucBuf[(pstBuf->m_iEnd + iCnt) % (pstBuf->m_iMaxSize)];
		iCnt++;
	}
	return iLen;
}

/********************************************************************************
*Function Name: get_from_buffer_withremove
*Description:  从buffer中取出一段长度的数据,同时删除原数据
*Input Params: @pstBuf - 对应的buffer管理结构体
			   @iLen - 要加入数据的长度
*Out Params:   @pucData - 存放取出的数据
*Return Value: 0 - success, -1 -  fail
*********************************************************************************
*Modified time        Version      Modified by       Modified Content
*  2016/9/28            V1.0          lcy                create    
*********************************************************************************/
HC_INT32 get_from_buffer_withremove(BUFFER_STRUCT* pstBuf, HC_UINT8 *pucData, HC_INT32 iLen)
{
	HC_INT32 iCnt = 0;
	if ( iLen > pstBuf->m_iSize )
		iLen = pstBuf->m_iSize;
	while ( iCnt < iLen )
	{
		pucData[iCnt] = pstBuf->m_pucBuf[(pstBuf->m_iEnd + iCnt) % (pstBuf->m_iMaxSize)];
		iCnt++;
	}
	remove_from_buffer(pstBuf, iLen);
	return iLen;
}

/********************************************************************************
*Function Name: clean_buffer
*Description:  清空buffer
*Input Params: @pstBuf - 对应的buffer管理结构体
*Out Params:   @pstBuf - 对应的buffer管理结构体
*Return Value: 0 - success, -1 -  fail
*********************************************************************************
*Modified time        Version      Modified by       Modified Content
*  2016/9/28            V1.0          lcy                create    
*********************************************************************************/
HC_VOID clean_buffer(BUFFER_STRUCT *pstBuf)
{
	pstBuf->m_iEnd = pstBuf->m_iStart = pstBuf->m_iSize = 0;
}

