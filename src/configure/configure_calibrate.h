#ifndef __CONFIGURE_CONFIG_CALIBRATE_H__
#define __CONFIGURE_CONFIG_CALIBRATE_H__

#ifdef __cplusplus
extern "C" {
#endif
#include <string.h>
#include "hc_type.h"
#include "parson.h"
#include "GILC_Vehicle_lib.h"

#ifdef WIN32
#define  GILC_VEHICLE_CONFIG_CALIBRATE1_PATH        "./gilc_calibrate1.cfg"
#define  GILC_VEHICLE_CONFIG_CALIBRATE2_PATH        "./gilc_calibrate2.cfg"
#define  GILC_VEHICLE_CONFIG_CALIBRATE3_PATH        "./gilc_calibrate3.cfg"
#define  GILC_CALIBRATE_CFG_PATH                    "./gilc_calibrate_cfg"
#else
#define  GILC_VEHICLE_CONFIG_CALIBRATE1_PATH        "/mnt/gilc-data/gilc_calibrate1.cfg"
#define  GILC_VEHICLE_CONFIG_CALIBRATE2_PATH        "/mnt/gilc-data/gilc_calibrate2.cfg"
#define  GILC_VEHICLE_CONFIG_CALIBRATE3_PATH        "/mnt/gilc-data/gilc_calibrate3.cfg"
#define  GILC_CALIBRATE_CFG_PATH                    "/data/app/conf/gilc_calibrate_cfg"
#endif

#pragma pack (4)
typedef struct config_calibrate_data
{
	HC_DOUBLE time[2];
	HC_BOOL   bStatic;
	ekf_X_t   stEkfX;
	HC_INT32  iCrc;
}config_calibrate_t;
#pragma pack ()

HC_INT32 config_calibrate__open();
HC_INT32 config_calibrate__close();
HC_INT32 config_calibrate__save();
HC_INT32 config_calibrate__creat();
HC_INT32 config_calibrate__set_last_time(HC_DOUBLE data[2]);
HC_INT32 config_calibrate__get_last_time(HC_DOUBLE data[2]);
HC_INT32 config_calibrate__set_last_att(HC_DOUBLE data[3]);
HC_INT32 config_calibrate__get_last_att(HC_DOUBLE data[3]);
HC_INT32 config_calibrate__set_last_pos(HC_DOUBLE data[3]);
HC_INT32 config_calibrate__get_last_pos(HC_DOUBLE data[3]);
HC_INT32 config_calibrate__set_imu_bias(HC_DOUBLE data[6]);
HC_INT32 config_calibrate__get_imu_bias(HC_DOUBLE data[6]);
HC_INT32 config_calibrate__set_installerr(HC_DOUBLE data[3]);
HC_INT32 config_calibrate__get_installerr(HC_DOUBLE data[3]);
HC_INT32 config_calibrate__set_lever(HC_DOUBLE data[3]);
HC_INT32 config_calibrate__get_lever(HC_DOUBLE data[3]);
HC_INT32 config_calibrate__set_all(config_calibrate_t *pstCfgData);
HC_INT32 config_calibrate__get_all(config_calibrate_t *pstCfgData);
HC_INT32 config_calibrate__update(config_calibrate_t *pstCfgData);
HC_INT32 config_calibrate__read(config_calibrate_t *pstCfgData);
HC_VOID config_calibrate__creat_default_config();
	
HC_INT32 gilc__calibrate_cfg_read(config_calibrate_t *pstGilcCfg);
HC_INT32 gilc__calibrate_cfg_save(config_calibrate_t *pstGilcCfg);

HC_INT32 gilc__calibrate_cfg_read2(HC_INT8 *cFilePath, config_calibrate_t *pstGilcCfg);
HC_INT32 gilc__calibrate_cfg_save2(HC_INT8 *cFilePath, config_calibrate_t *pstGilcCfg);

#ifdef __cplusplus
}
#endif
#endif
