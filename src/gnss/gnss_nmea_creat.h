#ifndef __GNSS_NMEA_CREAT_H__
#define __GNSS_NMEA_CREAT_H__

#include <hc_type.h>
#include <gnss_nmea.h>

#ifdef __cplusplus
extern "C" {
#endif				/*__cplusplus*/

HC_UINT32 NMEA_Creat(nmea_msg *gpsx,HC_UINT8 *buf);
HC_UINT32 NMEA_CHC_Creat(nmea_msg *gpsx,HC_UINT8 *buf);
HC_UINT32 NMEA_GGA_Creat(nmea_msg *gpsx,HC_UINT8 *buf);
HC_UINT32 NMEA_RMC_Creat(nmea_msg *gpsx,HC_UINT8 *buf);
HC_UINT32 NMEA_ATT_Creat(nmea_msg *gpsx,HC_UINT8 *buf);

#ifdef __cplusplus
}
#endif			/*__cplusplus*/

#endif			/*__GNSS_NMEA_H__*/

