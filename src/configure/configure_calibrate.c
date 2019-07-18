//#include <unistd.h>
#include "configure_calibrate.h"
#include <errno.h>

static JSON_Object        *s_cfg_obj = NULL;
static JSON_Value         *s_cfg_val = NULL;
static HC_INT8            *s_cfg_path = NULL;
static HC_INT8            *s_cfg_paths[3] = {GILC_VEHICLE_CONFIG_CALIBRATE1_PATH,GILC_VEHICLE_CONFIG_CALIBRATE2_PATH,GILC_VEHICLE_CONFIG_CALIBRATE3_PATH};
static HC_INT32 config_calibrate_open()
{
	s_cfg_val = json_parse_file(s_cfg_path);
	s_cfg_obj = json_object(s_cfg_val);

	if (s_cfg_obj == NULL)
	{
		printf("Open config file error: %s\r\n", s_cfg_path);
		return HC_ERR;
	}
	return HC_OK;
}

static HC_INT32 config_calibrate_close()
{
	json_value_free(s_cfg_val);
	return HC_OK;
}

HC_INT32 config_calibrate__open()
{
	s_cfg_val = json_parse_file(s_cfg_path);
	s_cfg_obj = json_object(s_cfg_val);

	if (s_cfg_obj == NULL)
	{
		printf("Open config file error\r\n");
		return HC_ERR;
	}
	return HC_OK;
}

HC_INT32 config_calibrate__close()
{
	json_serialize_to_file_pretty(s_cfg_val, s_cfg_path);
	json_value_free(s_cfg_val);

	if (s_cfg_obj != NULL)
	{
		printf("Open config file error\r\n");
		return HC_ERR;
	}
	return HC_OK;
}

HC_INT32 config_calibrate__save()
{
	HC_INT32 iRet = 0;
	if (s_cfg_val)
	{
		iRet = json_serialize_to_file_pretty(s_cfg_val, s_cfg_path);
	}
	return iRet;
}

HC_INT32 config_calibrate__creat()
{
	s_cfg_val = json_value_init_object();
	s_cfg_obj = json_object(s_cfg_val);

	if (s_cfg_obj == NULL)
	{
		printf("Creat config file error\r\n");
		return HC_ERR;
	}
	return HC_OK;
}

HC_INT32 config_calibrate__set_last_time(HC_DOUBLE data[2])
{
	HC_INT32 iRet = 0;
	iRet |= json_object_dotset_number(s_cfg_obj, "time.week", data[0]);
	iRet |= json_object_dotset_number(s_cfg_obj, "time.sec", data[1]);
	return iRet;
}

HC_INT32 config_calibrate__get_last_time(HC_DOUBLE data[2])
{
	data[0] = json_object_dotget_number(s_cfg_obj, "time.week");
	data[1] = json_object_dotget_number(s_cfg_obj, "time.sec");
	return HC_OK;
}

HC_INT32 config_calibrate__set_last_att(HC_DOUBLE data[3])
{
	HC_INT32 iRet = 0;
	iRet |= json_object_dotset_number(s_cfg_obj, "att.x", data[0]);
	iRet |= json_object_dotset_number(s_cfg_obj, "att.y", data[1]);
	iRet |= json_object_dotset_number(s_cfg_obj, "att.z", data[2]);
	return iRet;
}

HC_INT32 config_calibrate__get_last_att(HC_DOUBLE data[3])
{
	data[0] = json_object_dotget_number(s_cfg_obj, "att.x");
	data[1] = json_object_dotget_number(s_cfg_obj, "att.y");
	data[2] = json_object_dotget_number(s_cfg_obj, "att.z");
	return HC_OK;
}

HC_INT32 config_calibrate__set_last_vel(HC_DOUBLE data[3])
{
	HC_INT32 iRet = 0;
	iRet |= json_object_dotset_number(s_cfg_obj, "vel.x", data[0]);
	iRet |= json_object_dotset_number(s_cfg_obj, "vel.y", data[1]);
	iRet |= json_object_dotset_number(s_cfg_obj, "vel.z", data[2]);
	return iRet;
}

HC_INT32 config_calibrate__get_last_vel(HC_DOUBLE data[3])
{
	data[0] = json_object_dotget_number(s_cfg_obj, "vel.x");
	data[1] = json_object_dotget_number(s_cfg_obj, "vel.y");
	data[2] = json_object_dotget_number(s_cfg_obj, "vel.z");
	return HC_OK;
}

HC_INT32 config_calibrate__set_last_pos(HC_DOUBLE data[3])
{
	HC_INT32 iRet = 0;
	iRet |= json_object_dotset_number(s_cfg_obj, "pos.lat", data[0]);
	iRet |= json_object_dotset_number(s_cfg_obj, "pos.lon", data[1]);
	iRet |= json_object_dotset_number(s_cfg_obj, "pos.alt", data[2]);
	return iRet;
}

HC_INT32 config_calibrate__get_last_pos(HC_DOUBLE data[3])
{
	data[0] = json_object_dotget_number(s_cfg_obj, "pos.lat");
	data[1] = json_object_dotget_number(s_cfg_obj, "pos.lon");
	data[2] = json_object_dotget_number(s_cfg_obj, "pos.alt");
	return HC_OK;
}
HC_INT32 config_calibrate__set_gyo_bias(HC_DOUBLE data[6])
{
	HC_INT32 iRet = 0;
	iRet |= json_object_dotset_number(s_cfg_obj, "bias.gyo.x", data[0]);
	iRet |= json_object_dotset_number(s_cfg_obj, "bias.gyo.y", data[1]);
	iRet |= json_object_dotset_number(s_cfg_obj, "bias.gyo.z", data[2]);
	return iRet;
}

HC_INT32 config_calibrate__get_gyo_bias(HC_DOUBLE data[6])
{
	data[0] = json_object_dotget_number(s_cfg_obj, "bias.gyo.x");
	data[1] = json_object_dotget_number(s_cfg_obj, "bias.gyo.y");
	data[2] = json_object_dotget_number(s_cfg_obj, "bias.gyo.z");
	return HC_OK;
}

HC_INT32 config_calibrate__set_acc_bias(HC_DOUBLE data[3])
{
	HC_INT32 iRet = 0;
	iRet |= json_object_dotset_number(s_cfg_obj, "bias.acc.x", data[0]);
	iRet |= json_object_dotset_number(s_cfg_obj, "bias.acc.y", data[1]);
	iRet |= json_object_dotset_number(s_cfg_obj, "bias.acc.z", data[2]);
	return iRet;
}

HC_INT32 config_calibrate__get_acc_bias(HC_DOUBLE data[3])
{
	data[0] = json_object_dotget_number(s_cfg_obj, "bias.acc.x");
	data[1] = json_object_dotget_number(s_cfg_obj, "bias.acc.y");
	data[2] = json_object_dotget_number(s_cfg_obj, "bias.acc.z");
	return HC_OK;
}

HC_INT32 config_calibrate__set_installerr(HC_DOUBLE data[3])
{
	HC_INT32 iRet = 0;
	iRet |= json_object_dotset_number(s_cfg_obj, "installerr.x", data[0]);
	iRet |= json_object_dotset_number(s_cfg_obj, "installerr.y", data[1]);
	iRet |= json_object_dotset_number(s_cfg_obj, "installerr.z", data[2]);
	return iRet;
}

HC_INT32 config_calibrate__get_installerr(HC_DOUBLE data[3])
{
	data[0] = json_object_dotget_number(s_cfg_obj, "installerr.x");
	data[1] = json_object_dotget_number(s_cfg_obj, "installerr.y");
	data[2] = json_object_dotget_number(s_cfg_obj, "installerr.z");
	return HC_OK;
}

HC_INT32 config_calibrate__set_lever(HC_DOUBLE data[3])
{
	HC_INT32 iRet = 0;
	iRet |= json_object_dotset_number(s_cfg_obj, "lever.x", data[0]);
	iRet |= json_object_dotset_number(s_cfg_obj, "lever.y", data[1]);
	iRet |= json_object_dotset_number(s_cfg_obj, "lever.z", data[2]);
	return iRet;
}

HC_INT32 config_calibrate__get_lever(HC_DOUBLE data[3])
{
	data[0] = json_object_dotget_number(s_cfg_obj, "lever.x");
	data[1] = json_object_dotget_number(s_cfg_obj, "lever.y");
	data[2] = json_object_dotget_number(s_cfg_obj, "lever.z");
	return HC_OK;
}

HC_INT32 config_calibrate__set_all(config_calibrate_t * pstCfgData)
{ 
	HC_INT32 iRet = 0;
	iRet |= config_calibrate__set_last_time(pstCfgData->time);
	iRet |= config_calibrate__set_last_att(pstCfgData->stEkfX.Att);
	iRet |= config_calibrate__set_last_vel(pstCfgData->stEkfX.Vel);
	iRet |= config_calibrate__set_last_pos(pstCfgData->stEkfX.Pos);
	iRet |= config_calibrate__set_gyo_bias(pstCfgData->stEkfX.GyoBias);
	iRet |= config_calibrate__set_acc_bias(pstCfgData->stEkfX.AccBias);
	iRet |= config_calibrate__set_installerr(pstCfgData->stEkfX.InstallErr);
	iRet |= config_calibrate__set_lever(pstCfgData->stEkfX.Lever);
	iRet |= config_calibrate__save();
	return iRet;
}

HC_INT32 config_calibrate__get_all(config_calibrate_t *pstCfgData)
{ 
	config_calibrate__get_last_time(pstCfgData->time);
	config_calibrate__get_last_att(pstCfgData->stEkfX.Att);
	config_calibrate__get_last_vel(pstCfgData->stEkfX.Vel);
	config_calibrate__get_last_pos(pstCfgData->stEkfX.Pos);
	config_calibrate__get_gyo_bias(pstCfgData->stEkfX.GyoBias);
	config_calibrate__get_acc_bias(pstCfgData->stEkfX.AccBias);
	config_calibrate__get_installerr(pstCfgData->stEkfX.InstallErr);
	config_calibrate__get_lever(pstCfgData->stEkfX.Lever);
	return HC_OK;
}

HC_INT32 config_calibrate__update(config_calibrate_t *pstCfgData)
{ 
	static HC_INT8 updata_flag = 0;
	HC_INT32 iRet = 0;
	if(++updata_flag == 3) 
		updata_flag = 0;
	s_cfg_path = s_cfg_paths[updata_flag];
	iRet = config_calibrate_open();
	if(iRet!=HC_OK)
		config_calibrate__creat();
	iRet = config_calibrate__set_all(pstCfgData);
	if(iRet)
	{
		printf("config_calibrate__set_all err\r\n");
	}
	config_calibrate_close();
	return HC_OK;
}

HC_INT32 config_calibrate__read(config_calibrate_t *pstCfgData)
{ 
	HC_INT32 iRet = 0,i=0;
	HC_DOUBLE dDiffPos[3] = {0.0};
	HC_DOUBLE data_times[3][2] = {{0.0}};
	HC_DOUBLE data_poss[3][3] = {{0.0}};
	HC_DOUBLE data_time_sec[3] = {0.0};
	HC_INT8   data_time_min = 0;
	HC_INT8   data_time_max = 0;
	HC_INT8   data_time_mid = 0;

	for (i = 0; i < 3; i++)
	{
		s_cfg_path = s_cfg_paths[i];
		iRet = config_calibrate_open();
		if(iRet==HC_OK)
		{
			config_calibrate__get_last_time(data_times[i]);
			config_calibrate__get_last_pos(data_poss[i]);
			config_calibrate_close();
		}
	}

	/*dsf90: 三取二，抛弃最新时刻配置文件（防止突然断电数据不完整）*/
	data_time_sec[0] = (HC_INT32)data_times[0][0]*3600*24*7 + data_times[0][1];
	data_time_sec[1] = (HC_INT32)data_times[1][0]*3600*24*7 + data_times[1][1];
	data_time_sec[2] = (HC_INT32)data_times[2][0]*3600*24*7 + data_times[2][1];
	if(data_time_sec[0]<=data_time_sec[1])
	{
		data_time_min = 0;
		data_time_max = 1;
	}
	else
	{
		data_time_min = 1;
		data_time_max = 0;
	}
	if(data_time_sec[2] <= data_time_sec[data_time_min])
		data_time_min = 2;
	else if(data_time_sec[2] >= data_time_sec[data_time_max])
		data_time_max = 2;
	for(i=0; i<3; i++)
	{
		if(i!=data_time_min && i!=data_time_max)
		{
			data_time_mid = i;
			break;
		}
	}

	if((HC_INT32)data_times[data_time_min][0] < 2017)
	{
		printf("config_calibrate__read err: min %d, week %d sec %.6f\r\n",data_time_min,(HC_INT32)data_times[data_time_min][0],data_times[data_time_min][1]);
		return HC_ERR;
	}
	if((HC_INT32)data_times[data_time_mid][0] < 2017)
	{
		printf("config_calibrate__read err: mid %d, week %d sec %.6f\r\n",data_time_mid,(HC_INT32)data_times[data_time_mid][0],data_times[data_time_mid][1]);
		return HC_ERR;
	}

	s_cfg_path = s_cfg_paths[data_time_mid];
	config_calibrate_open();
	if(iRet!=HC_OK)
	{
		printf("config_calibrate__read err: filename, %s\r\n",s_cfg_path);
		return HC_ERR;
	}
	config_calibrate__get_all(pstCfgData);
	config_calibrate_close();
	
	/*dsf90: 判断位置变化，停车情况数据可用*/
	dDiffPos[0] = (data_poss[data_time_mid][0] - data_poss[data_time_min][0])*D2R*RE_WGS84;
	dDiffPos[1] = (data_poss[data_time_mid][1] - data_poss[data_time_min][1])*D2R*RE_WGS84;	
	pstCfgData->bStatic = true;
	if(dDiffPos[0] >= 0.5 || dDiffPos[1] >= 0.5)
	{
		pstCfgData->bStatic = false;
		printf("config_calibrate__read: pos err, diffpos  %.3f  %.3f\r\n",dDiffPos[0],dDiffPos[1]);
	}
	return HC_OK;
}

HC_INT32 gilc__calibrate_cfg_read(config_calibrate_t *pstGilcCfg)
{
	HC_INT32 len = 0;
	FILE *fp = NULL;
	fp = fopen(GILC_CALIBRATE_CFG_PATH,"rb");
	if(!fp)
	{
		loge("open cfg file [%s] fail, errno [ %d ]\n",GILC_CALIBRATE_CFG_PATH,errno);
		return HC_ERR;
	}
	len = fread(pstGilcCfg,sizeof(config_calibrate_t),1,fp);
	fclose(fp);
	if(len == 1)
	{
		return 0;
	}
	return -1;
}

HC_INT32 gilc__calibrate_cfg_save(config_calibrate_t *pstGilcCfg)
{
	HC_INT32 len = 0;
	FILE *fp = NULL;
	fp = fopen(GILC_CALIBRATE_CFG_PATH,"wb");
	if(!fp)
	{
		loge("open cfg file [%s] fail, errno [ %d ]\n",GILC_CALIBRATE_CFG_PATH,errno);
		return HC_ERR;
	}
	len = fwrite(pstGilcCfg,sizeof(config_calibrate_t),1,fp);
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


HC_INT32 gilc__calibrate_cfg_read2(HC_INT8 *cFilePath, config_calibrate_t *pstGilcCfg)
{
	HC_INT32 len = 0;
	FILE *fp = NULL;

	if (!cFilePath)
	{
		loge("param err\n");
		return HC_ERR;
	}

	fp = fopen(cFilePath, "rb");
	if (!fp)
	{
		loge("open cfg file [%s] fail, errno [ %d ]\n", cFilePath, errno);
		return HC_ERR;
	}
	len = fread(pstGilcCfg, sizeof(config_calibrate_t), 1, fp);
	fclose(fp);
	if (len == 1)
	{
		return 0;
	}
	return -1;
}

HC_INT32 gilc__calibrate_cfg_save2(HC_INT8 *cFilePath, config_calibrate_t *pstGilcCfg)
{
	HC_INT32 len = 0;
	FILE *fp = NULL;

	if (!cFilePath)
	{
		loge("param err\n");
		return HC_ERR;
	}

	fp = fopen(cFilePath, "wb");
	if (!fp)
	{
		loge("open cfg file [%s] fail, errno [ %d ]\n", cFilePath, errno);
		return HC_ERR;
	}
	len = fwrite(pstGilcCfg, sizeof(config_calibrate_t), 1, fp);
	if (len == 1)
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

