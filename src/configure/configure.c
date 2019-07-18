//#include <unistd.h>
#include "configure.h"
#include <errno.h>

static JSON_Object        *s_cfg_obj = NULL;
static JSON_Value         *s_cfg_val = NULL;
HC_INT32 init_config()
{
	s_cfg_val = json_parse_file(GILC_VEHICLE_CONFIG_PATH);
	s_cfg_obj = json_object(s_cfg_val);

	if (s_cfg_obj == NULL)
	{
		printf("Init config file error\r\n");
		return HC_ERR;
	}

	return HC_OK;
}

HC_INT32 close_config()
{
	json_serialize_to_file_pretty(s_cfg_val, GILC_VEHICLE_CONFIG_PATH);
	json_value_free(s_cfg_val);
	return HC_OK;
}

HC_VOID save_config()
{
	if (s_cfg_val)
	{
		json_serialize_to_file_pretty(s_cfg_val, GILC_VEHICLE_CONFIG_PATH);
	}
}

HC_VOID creat_config()
{
	s_cfg_val = json_value_init_object();
	s_cfg_obj = json_object(s_cfg_val);
}

HC_INT32 config_set_imu_installtype(HC_INT32 data[3])
{
	HC_INT32 iRet = 0;
	iRet |= json_object_dotset_number(s_cfg_obj, "imu.installtype.x", data[0]);
	iRet |= json_object_dotset_number(s_cfg_obj, "imu.installtype.y", data[1]);
	iRet |= json_object_dotset_number(s_cfg_obj, "imu.installtype.z", data[2]);
	return iRet;
}

HC_INT32 config_get_imu_installtype(HC_INT32 data[3])
{
	data[0] = (HC_INT32)json_object_dotget_number(s_cfg_obj, "imu.installtype.x");
	data[1] = (HC_INT32)json_object_dotget_number(s_cfg_obj, "imu.installtype.y");
	data[2] = (HC_INT32)json_object_dotget_number(s_cfg_obj, "imu.installtype.z");
	return HC_OK;
}

HC_INT32 config_set_imu_read_freq(HC_INT32 fs_hz)
{
	if (fs_hz < 0 || fs_hz > 1000)
	{
		return HC_ERR;
	}

	json_object_dotset_number(s_cfg_obj, "imu.fs_hz", fs_hz);
	return HC_OK;
}

HC_INT32 config_get_imu_read_freq()
{
	return (HC_INT32)json_object_dotget_number(s_cfg_obj, "imu.fs_hz");
}

HC_INT32 config_set_imu_bw_freq(HC_INT32 bw_hz)
{
	if (bw_hz < 0 || bw_hz > 1000)
	{
		return HC_ERR;
	}

	json_object_dotset_number(s_cfg_obj, "imu.bw_hz", bw_hz);
	return HC_OK;
}

HC_INT32 config_get_imu_bw_freq()
{
	return (HC_INT32)json_object_dotget_number(s_cfg_obj, "imu.bw_hz");
}

HC_INT32 config_set_gilc_out_period(HC_INT32 out_ms)
{
	if (out_ms < 0 || out_ms > 1000)
	{
		return HC_ERR;
	}

	json_object_dotset_number(s_cfg_obj, "gilc.out_ms", out_ms);
	return HC_OK;
}

HC_INT32 config_get_gilc_out_period()
{
	return (HC_INT32)json_object_dotget_number(s_cfg_obj, "gilc.out_ms");
}

HC_INT32 config_set_gilc_process_rt(HC_BOOL rt_flag)
{
	json_object_dotset_boolean(s_cfg_obj, "gilc.rt_flag", rt_flag);
	return HC_OK;
}

HC_BOOL config_get_gilc_process_rt()
{
	return json_object_dotget_boolean(s_cfg_obj, "gilc.rt_flag");
}

HC_INT32 config_set_gilc_process_strn(HC_BOOL strn_flag)
{
	json_object_dotset_boolean(s_cfg_obj, "gilc.strn_flag", strn_flag);
	return HC_OK;
}

HC_BOOL config_get_gilc_process_strn()
{
	return json_object_dotget_boolean(s_cfg_obj, "gilc.strn_flag");
}

HC_INT32 config_set_gilc_out_cfg(HC_BOOL rt_flag,HC_INT32 rate_hz,HC_INT32 msg_flag,HC_INT32 refer_point)
{
	if (rate_hz < 0 || rate_hz> 1000)
	{
		return HC_ERR;
	}
	HC_INT32 iRet = 0;
	iRet |=  json_object_dotset_boolean(s_cfg_obj,"gilc.out.rt_flag", rt_flag);
	iRet |=  json_object_dotset_number(s_cfg_obj, "gilc.out.rate_hz", rate_hz);
	iRet |=  json_object_dotset_number(s_cfg_obj, "gilc.out.msg_flag", msg_flag);
	iRet |=  json_object_dotset_number(s_cfg_obj, "gilc.out.refer_point", refer_point);
	return iRet;
}

HC_INT32 config_get_gilc_out_cfg(HC_BOOL *rt_flag,HC_INT32 *rate_hz,HC_INT32 *msg_flag,HC_INT32 *refer_point)
{
	*rt_flag     = json_object_dotget_boolean(s_cfg_obj,"gilc.out.rt_flag");
	*rate_hz     = json_object_dotget_number(s_cfg_obj, "gilc.out.rate_hz");
	*msg_flag    = json_object_dotget_number(s_cfg_obj, "gilc.out.msg_flag");
	*refer_point = json_object_dotget_number(s_cfg_obj, "gilc.out.refer_point");
	return HC_OK;
}

HC_INT32 config_get_gilc_lever_ins2gnss(HC_FLOAT lever[3],HC_FLOAT lever_err[3])
{
	lever[0]     = (HC_FLOAT)json_object_dotget_number(s_cfg_obj, "gilc.ins2gnss.right");
	lever[1]     = (HC_FLOAT)json_object_dotget_number(s_cfg_obj, "gilc.ins2gnss.front");
	lever[2]     = (HC_FLOAT)json_object_dotget_number(s_cfg_obj, "gilc.ins2gnss.up");
	lever_err[0] = (HC_FLOAT)json_object_dotget_number(s_cfg_obj, "gilc.ins2gnss_err.right");
	lever_err[1] = (HC_FLOAT)json_object_dotget_number(s_cfg_obj, "gilc.ins2gnss_err.front");
	lever_err[2] = (HC_FLOAT)json_object_dotget_number(s_cfg_obj, "gilc.ins2gnss_err.up");
	return HC_OK;
}

HC_INT32 config_set_gilc_lever_ins2gnss(HC_FLOAT lever[3],HC_FLOAT lever_err[3])
{
	HC_INT32 iRet = 0;
	iRet |= json_object_dotset_number(s_cfg_obj, "gilc.ins2gnss.right", lever[0]);
	iRet |= json_object_dotset_number(s_cfg_obj, "gilc.ins2gnss.front", lever[1]);
	iRet |= json_object_dotset_number(s_cfg_obj, "gilc.ins2gnss.up",    lever[2]);
	iRet |= json_object_dotset_number(s_cfg_obj, "gilc.ins2gnss_err.right", lever_err[0]);
	iRet |= json_object_dotset_number(s_cfg_obj, "gilc.ins2gnss_err.front", lever_err[1]);
	iRet |= json_object_dotset_number(s_cfg_obj, "gilc.ins2gnss_err.up",    lever_err[2]);
	return iRet;
}

HC_INT32 config_get_gilc_lever_ins2car(HC_FLOAT lever[3],HC_FLOAT lever_err[3])
{
	lever[0]     = (HC_FLOAT)json_object_dotget_number(s_cfg_obj, "gilc.ins2car.right");
	lever[1]     = (HC_FLOAT)json_object_dotget_number(s_cfg_obj, "gilc.ins2car.front");
	lever[2]     = (HC_FLOAT)json_object_dotget_number(s_cfg_obj, "gilc.ins2car.up");
	lever_err[0] = (HC_FLOAT)json_object_dotget_number(s_cfg_obj, "gilc.ins2car_err.right");
	lever_err[1] = (HC_FLOAT)json_object_dotget_number(s_cfg_obj, "gilc.ins2car_err.front");
	lever_err[2] = (HC_FLOAT)json_object_dotget_number(s_cfg_obj, "gilc.ins2car_err.up");
	return HC_OK;
}

HC_INT32 config_set_gilc_lever_ins2car(HC_FLOAT lever[3],HC_FLOAT lever_err[3])
{
	HC_INT32 iRet = 0;
	iRet |= json_object_dotset_number(s_cfg_obj, "gilc.ins2car.right", lever[0]);
	iRet |= json_object_dotset_number(s_cfg_obj, "gilc.ins2car.front", lever[1]);
	iRet |= json_object_dotset_number(s_cfg_obj, "gilc.ins2car.up",    lever[2]);
	iRet |= json_object_dotset_number(s_cfg_obj, "gilc.ins2car_err.right", lever_err[0]);
	iRet |= json_object_dotset_number(s_cfg_obj, "gilc.ins2car_err.front", lever_err[1]);
	iRet |= json_object_dotset_number(s_cfg_obj, "gilc.ins2car_err.up",    lever_err[2]);
	return iRet;
}

HC_INT32 config_get_gilc_wheel_distance(HC_FLOAT data[2])
{
	data[0]  = (HC_FLOAT) json_object_dotget_number(s_cfg_obj, "gilc.wheel.track");
	data[1]  = (HC_FLOAT) json_object_dotget_number(s_cfg_obj, "gilc.wheel.base");
	return HC_OK;
}

HC_INT32 config_set_gilc_wheel_distance(HC_FLOAT data[2])
{
	HC_INT32 iRet = 0;
	iRet |= json_object_dotset_number(s_cfg_obj, "gilc.wheel.track", data[0]);
	iRet |= json_object_dotset_number(s_cfg_obj, "gilc.wheel.base", data[1]);
	return HC_OK;
}

HC_VOID generate_default_config()
{
	s_cfg_val = json_value_init_object();
	s_cfg_obj = json_object(s_cfg_val);

	config_set_imu_read_freq(100);
	config_set_imu_bw_freq(10);
	config_set_gilc_process_strn(0);

	json_serialize_to_file_pretty(s_cfg_val, GILC_VEHICLE_CONFIG_PATH);
	json_value_free(s_cfg_val);
}

#ifndef WIN32
HC_INT32 gilc__web_cfg_read(web_cfg_data_t *pstGilcCfg)
{
	HC_INT32 len = 0;
	FILE *fp = NULL;
	fp = fopen(GILC_WEB_CFG_PATH,"rb");
	if(!fp)
	{
		loge("open cfg file [%s] fail, errno [ %d ]\n",GILC_WEB_CFG_PATH,errno);
		return HC_ERR;
	}
	len = fread(pstGilcCfg,sizeof(web_cfg_data_t),1,fp);
	fclose(fp);
	if(len == 1)
	{
		return 0;
	}
	return -1;
}

HC_INT32 gilc__web_cfg_save(web_cfg_data_t *pstGilcCfg)
{
	HC_INT32 len = 0;
	FILE *fp = NULL;
	fp = fopen(GILC_WEB_CFG_PATH,"wb");
	if(!fp)
	{
		loge("open cfg file [%s] fail, errno [ %d ]\n",GILC_WEB_CFG_PATH,errno);
		return HC_ERR;
	}
	len = fwrite(pstGilcCfg,sizeof(web_cfg_data_t),1,fp);
	if(len == 1)
	{
		fflush(fp); 	 
#ifndef WIN32
		fsync(fileno(fp));
#endif
		fclose(fp);
		return 0;
	}	
	fclose(fp);
	return -1;
}
#endif


