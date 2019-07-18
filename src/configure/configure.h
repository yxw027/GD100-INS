#ifndef __CONFIGURE_CONFIG_H__
#define __CONFIGURE_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif
#include <string.h>
#include "hc_type.h"
#include "parson.h"
#ifndef WIN32
#include "decode.h"
#endif

#ifdef WIN32
#define  GILC_VEHICLE_CONFIG_PATH                   "./gilc_vehicle.cfg"
#define  GILC_WEB_CFG_PATH                          "./gilc_web_cfg"
#else
#define  GILC_VEHICLE_CONFIG_PATH                   "/mnt/gilc_vehicle.cfg"
#define  GILC_WEB_CFG_PATH                          "/data/app/conf/gilc_web_cfg"
#endif
	HC_INT32 init_config();
	HC_VOID save_config();
	HC_VOID creat_config();
	HC_INT32 close_config();

	HC_INT32 config_set_imu_installtype(HC_INT32 data[3]);
	HC_INT32 config_get_imu_installtype(HC_INT32 data[3]);
	HC_INT32 config_set_imu_read_freq(HC_INT32 fs_hz);
	HC_INT32 config_get_imu_read_freq();
	HC_INT32 config_set_imu_bw_freq(HC_INT32 bw_hz);
	HC_INT32 config_get_imu_bw_freq();
	
	HC_INT32 config_set_gilc_out_cfg(HC_BOOL rt_flag,HC_INT32 period_ms,HC_INT32 msg_flag,HC_INT32 refer_point);
	HC_INT32 config_get_gilc_out_cfg(HC_BOOL *rt_flag,HC_INT32 *period_ms,HC_INT32 *msg_flag,HC_INT32 *refer_point);

	HC_INT32 config_set_gilc_process_strn(HC_BOOL strn_flag);
	HC_BOOL  config_get_gilc_process_strn();

	HC_INT32 config_get_gilc_lever_ins2gnss(HC_FLOAT lever[3],HC_FLOAT lever_err[3]);
	HC_INT32 config_set_gilc_lever_ins2gnss(HC_FLOAT lever[3],HC_FLOAT lever_err[3]);
	HC_INT32 config_get_gilc_lever_ins2car(HC_FLOAT lever[3],HC_FLOAT lever_err[3]);
	HC_INT32 config_set_gilc_lever_ins2car(HC_FLOAT lever[3],HC_FLOAT lever_err[3]);

	HC_INT32 config_get_gilc_wheel_distance(HC_FLOAT data[2]);
	HC_INT32 config_set_gilc_wheel_distance(HC_FLOAT data[2]);

	HC_VOID generate_default_config();

#ifndef WIN32
	HC_INT32 gilc__web_cfg_read(web_cfg_data_t *pstGilcCfg);
	HC_INT32 gilc__web_cfg_save(web_cfg_data_t *pstGilcCfg);
#endif

#ifdef __cplusplus
}
#endif
#endif
