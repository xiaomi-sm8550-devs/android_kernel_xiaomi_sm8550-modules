// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2025, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/module.h>
#include <linux/firmware.h>

#include "cam_sensor_cmn_header.h"
#include "cam_ois_core.h"
#include "cam_ois_soc.h"
#include "cam_sensor_util.h"
#include "cam_debug_util.h"
#include "cam_res_mgr_api.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"
#include "xiaomi_flash_ois.h"

#include "sem1217s.h"

static int semco_ois_fw_update_op = 1;
module_param(semco_ois_fw_update_op, int, 0644);

int32_t cam_ois_construct_default_power_setting(
	struct cam_sensor_power_ctrl_t *power_info)
{
	int rc = 0;

	power_info->power_setting_size = 1;
	power_info->power_setting =
		kzalloc(sizeof(struct cam_sensor_power_setting),
			GFP_KERNEL);
	if (!power_info->power_setting)
		return -ENOMEM;

	power_info->power_setting[0].seq_type = SENSOR_VAF;
	power_info->power_setting[0].seq_val = CAM_VAF;
	power_info->power_setting[0].config_val = 1;
	power_info->power_setting[0].delay = 2;

	power_info->power_down_setting_size = 1;
	power_info->power_down_setting =
		kzalloc(sizeof(struct cam_sensor_power_setting),
			GFP_KERNEL);
	if (!power_info->power_down_setting) {
		rc = -ENOMEM;
		goto free_power_settings;
	}

	power_info->power_down_setting[0].seq_type = SENSOR_VAF;
	power_info->power_down_setting[0].seq_val = CAM_VAF;
	power_info->power_down_setting[0].config_val = 0;

	return rc;

free_power_settings:
	kfree(power_info->power_setting);
	power_info->power_setting = NULL;
	power_info->power_setting_size = 0;
	return rc;
}


/**
 * cam_ois_get_dev_handle - get device handle
 * @o_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
static int cam_ois_get_dev_handle(struct cam_ois_ctrl_t *o_ctrl,
	void *arg)
{
	struct cam_sensor_acquire_dev    ois_acq_dev;
	struct cam_create_dev_hdl        bridge_params;
	struct cam_control              *cmd = (struct cam_control *)arg;

	if (o_ctrl->bridge_intf.device_hdl != -1) {
		CAM_ERR(CAM_OIS, "Device is already acquired");
		return -EFAULT;
	}
	if (copy_from_user(&ois_acq_dev, u64_to_user_ptr(cmd->handle),
		sizeof(ois_acq_dev)))
		return -EFAULT;

	bridge_params.session_hdl = ois_acq_dev.session_handle;
	bridge_params.ops = &o_ctrl->bridge_intf.ops;
	bridge_params.v4l2_sub_dev_flag = 0;
	bridge_params.media_entity_flag = 0;
	bridge_params.priv = o_ctrl;
	bridge_params.dev_id = CAM_OIS;

	ois_acq_dev.device_handle =
		cam_create_device_hdl(&bridge_params);
	if (ois_acq_dev.device_handle <= 0) {
		CAM_ERR(CAM_OIS, "Can not create device handle");
		return -EFAULT;
	}
	o_ctrl->bridge_intf.device_hdl = ois_acq_dev.device_handle;
	o_ctrl->bridge_intf.session_hdl = ois_acq_dev.session_handle;

	CAM_DBG(CAM_OIS, "Device Handle: %d", ois_acq_dev.device_handle);
	if (copy_to_user(u64_to_user_ptr(cmd->handle), &ois_acq_dev,
		sizeof(struct cam_sensor_acquire_dev))) {
		CAM_ERR(CAM_OIS, "ACQUIRE_DEV: copy to user failed");
		return -EFAULT;
	}
	return 0;
}

static int cam_ois_power_up(struct cam_ois_ctrl_t *o_ctrl)
{
	int                                     rc = 0;
	struct cam_hw_soc_info                 *soc_info = &o_ctrl->soc_info;
	struct cam_ois_soc_private             *soc_private;
	struct cam_sensor_power_ctrl_t         *power_info;
	struct completion                      *i3c_probe_completion = NULL;

	soc_private = (struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	if ((power_info->power_setting == NULL) &&
		(power_info->power_down_setting == NULL)) {
		CAM_INFO(CAM_OIS,
			"Using default power settings");
		rc = cam_ois_construct_default_power_setting(power_info);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Construct default ois power setting failed.");
			return rc;
		}
	}

	/* Parse and fill vreg params for power up settings */
	rc = msm_camera_fill_vreg_params(
		soc_info,
		power_info->power_setting,
		power_info->power_setting_size);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"failed to fill vreg params for power up rc:%d", rc);
		return rc;
	}

	/* Parse and fill vreg params for power down settings*/
	rc = msm_camera_fill_vreg_params(
		soc_info,
		power_info->power_down_setting,
		power_info->power_down_setting_size);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"failed to fill vreg params for power down rc:%d", rc);
		return rc;
	}

	power_info->dev = soc_info->dev;

	if (o_ctrl->io_master_info.master_type == I3C_MASTER)
		i3c_probe_completion = cam_ois_get_i3c_completion(o_ctrl->soc_info.index);

	rc = cam_sensor_core_power_up(power_info, soc_info, i3c_probe_completion);
	if (rc) {
		CAM_ERR(CAM_OIS, "failed in ois power up rc %d", rc);
		return rc;
	}

	rc = camera_io_init(&o_ctrl->io_master_info);
	if (rc) {
		CAM_ERR(CAM_OIS, "cci_init failed: rc: %d", rc);
		goto cci_failure;
	}

	return rc;
cci_failure:
	if (cam_sensor_util_power_down(power_info, soc_info))
		CAM_ERR(CAM_OIS, "Power Down failed");

	return rc;
}

/**
 * cam_ois_power_down - power down OIS device
 * @o_ctrl:     ctrl structure
 *
 * Returns success or failure
 */
static int cam_ois_power_down(struct cam_ois_ctrl_t *o_ctrl)
{
	int32_t                         rc = 0;
	struct cam_sensor_power_ctrl_t  *power_info;
	struct cam_hw_soc_info          *soc_info =
		&o_ctrl->soc_info;
	struct cam_ois_soc_private *soc_private;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "failed: o_ctrl %pK", o_ctrl);
		return -EINVAL;
	}

	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;
	soc_info = &o_ctrl->soc_info;

	if (!power_info) {
		CAM_ERR(CAM_OIS, "failed: power_info %pK", power_info);
		return -EINVAL;
	}

	rc = cam_sensor_util_power_down(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_OIS, "power down the core is failed:%d", rc);
		return rc;
	}

	camera_io_release(&o_ctrl->io_master_info);
	o_ctrl->cam_ois_state = CAM_OIS_ACQUIRE;

	return rc;
}

static int cam_ois_update_time(struct i2c_settings_array *i2c_set)
{
	struct i2c_settings_list *i2c_list;
	int32_t rc = 0;
	uint32_t size = 0;
	uint32_t i = 0;
	uint64_t qtime_ns = 0;

	if (i2c_set == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	rc = cam_sensor_util_get_current_qtimer_ns(&qtime_ns);
	if (rc < 0) {
		CAM_ERR(CAM_OIS,
			"Failed to get current qtimer value: %d",
			rc);
		return rc;
	}

	list_for_each_entry(i2c_list,
		&(i2c_set->list_head), list) {
		if (i2c_list->op_code ==  CAM_SENSOR_I2C_WRITE_SEQ) {
			size = i2c_list->i2c_settings.size;
			/* qtimer is 8 bytes so validate here*/
			if (size < 8) {
				CAM_ERR(CAM_OIS, "Invalid write time settings");
				return -EINVAL;
			}
			for (i = 0; i < size; i++) {
				CAM_DBG(CAM_OIS, "time: reg_data[%d]: 0x%x",
					i, (qtime_ns & 0xFF));
				i2c_list->i2c_settings.reg_setting[i].reg_data =
					(qtime_ns & 0xFF);
				qtime_ns >>= 8;
			}
		}
	}

	return rc;
}

static int cam_ois_apply_settings(struct cam_ois_ctrl_t *o_ctrl,
	struct i2c_settings_array *i2c_set)
{
	struct i2c_settings_list *i2c_list;
	int32_t rc = 0;
	uint32_t i, size;

	if (o_ctrl == NULL || i2c_set == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	if (i2c_set->is_settings_valid != 1) {
		CAM_ERR(CAM_OIS, " Invalid settings");
		return -EINVAL;
	}

	list_for_each_entry(i2c_list,
		&(i2c_set->list_head), list) {
		if (i2c_list->op_code ==  CAM_SENSOR_I2C_WRITE_RANDOM) {
			rc = camera_io_dev_write(&(o_ctrl->io_master_info),
				&(i2c_list->i2c_settings));
			if (rc < 0) {
				CAM_ERR(CAM_OIS,
					"Failed in Applying i2c wrt settings");
				return rc;
			}
		} else if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_SEQ) {
			rc = camera_io_dev_write_continuous(
				&(o_ctrl->io_master_info),
				&(i2c_list->i2c_settings),
				CAM_SENSOR_I2C_WRITE_SEQ);
			if (rc < 0) {
				CAM_ERR(CAM_OIS,
					"Failed to seq write I2C settings: %d",
					rc);
				return rc;
			}
		} else if (i2c_list->op_code == CAM_SENSOR_I2C_POLL) {
			size = i2c_list->i2c_settings.size;
			for (i = 0; i < size; i++) {
				rc = camera_io_dev_poll(
				&(o_ctrl->io_master_info),
				i2c_list->i2c_settings.reg_setting[i].reg_addr,
				i2c_list->i2c_settings.reg_setting[i].reg_data,
				i2c_list->i2c_settings.reg_setting[i].data_mask,
				i2c_list->i2c_settings.addr_type,
				i2c_list->i2c_settings.data_type,
				i2c_list->i2c_settings.reg_setting[i].delay);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
						"i2c poll apply setting Fail");
					return rc;
				}
			}
		}
	}

	return rc;
}

#if defined(CONFIG_TARGET_PRODUCT_FUXI) || defined(CONFIG_TARGET_PRODUCT_NUWA)
#define OIS_TRANS_SIZE 256
#endif

static int cam_ois_slaveInfo_pkt_parser(struct cam_ois_ctrl_t *o_ctrl,
	uint32_t *cmd_buf, size_t len)
{
	int32_t rc = 0;
	struct cam_cmd_ois_info *ois_info;

	if (!o_ctrl || !cmd_buf || len < sizeof(struct cam_cmd_ois_info)) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	ois_info = (struct cam_cmd_ois_info *)cmd_buf;
	if (o_ctrl->io_master_info.master_type == CCI_MASTER) {
		o_ctrl->io_master_info.cci_client->i2c_freq_mode =
			ois_info->i2c_freq_mode;
		o_ctrl->io_master_info.cci_client->sid =
			ois_info->slave_addr >> 1;
		o_ctrl->ois_fw_flag = ois_info->ois_fw_flag;
		o_ctrl->is_ois_calib = ois_info->is_ois_calib;
		memcpy(o_ctrl->ois_name, ois_info->ois_name, OIS_NAME_LEN);
		o_ctrl->ois_name[OIS_NAME_LEN - 1] = '\0';
		o_ctrl->io_master_info.cci_client->retries = 3;
		o_ctrl->io_master_info.cci_client->id_map = 0;
		memcpy(&(o_ctrl->opcode), &(ois_info->opcode),
			sizeof(struct cam_ois_opcode));
		CAM_DBG(CAM_OIS, "Slave addr: 0x%x Freq Mode: %d",
			ois_info->slave_addr, ois_info->i2c_freq_mode);
	} else if (o_ctrl->io_master_info.master_type == I2C_MASTER) {
		o_ctrl->io_master_info.client->addr = ois_info->slave_addr;
		CAM_DBG(CAM_OIS, "Slave addr: 0x%x", ois_info->slave_addr);
	} else {
		CAM_ERR(CAM_OIS, "Invalid Master type : %d",
			o_ctrl->io_master_info.master_type);
		rc = -EINVAL;
	}

	return rc;
}

static int cam_default_ois_fw_download(struct cam_ois_ctrl_t *o_ctrl)
{
	uint16_t                           total_bytes = 0;
	uint8_t                           *ptr = NULL;
	#if defined(CONFIG_TARGET_PRODUCT_FUXI) || defined(CONFIG_TARGET_PRODUCT_NUWA)
	int32_t                            rc = 0, cnt, i, j;
	#else
	int32_t                            rc = 0, cnt;
	#endif
	uint32_t                           fw_size;
	const struct firmware             *fw = NULL;
	const char                        *fw_name_prog = NULL;
	const char                        *fw_name_coeff = NULL;
	#if defined(CONFIG_TARGET_PRODUCT_FUXI) || defined(CONFIG_TARGET_PRODUCT_NUWA)
	const char                        *fw_name_mem = NULL;
	#endif
	char                               name_prog[32] = {0};
	char                               name_coeff[32] = {0};
	#if defined(CONFIG_TARGET_PRODUCT_FUXI) || defined(CONFIG_TARGET_PRODUCT_NUWA)
	char                               name_mem[32] = {0};
	#endif
	struct device                     *dev = &(o_ctrl->pdev->dev);
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	void                              *vaddr = NULL;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	snprintf(name_coeff, 32, "%s.coeff", o_ctrl->ois_name);

	snprintf(name_prog, 32, "%s.prog", o_ctrl->ois_name);

	#if defined(CONFIG_TARGET_PRODUCT_FUXI) || defined(CONFIG_TARGET_PRODUCT_NUWA)
	snprintf(name_mem, 32, "%s.mem", o_ctrl->ois_name);
	#endif

	/* cast pointer as const pointer*/
	fw_name_prog = name_prog;
	fw_name_coeff = name_coeff;
	#if defined(CONFIG_TARGET_PRODUCT_FUXI) || defined(CONFIG_TARGET_PRODUCT_NUWA)
	fw_name_mem = name_mem;
	#endif

	/* Load FW */
	rc = request_firmware(&fw, fw_name_prog, dev);
	if (rc) {
		CAM_ERR(CAM_OIS, "Failed to locate %s", fw_name_prog);
		return rc;
	}

	total_bytes = fw->size;
	#if defined(CONFIG_TARGET_PRODUCT_FUXI) || defined(CONFIG_TARGET_PRODUCT_NUWA) || defined(CONFIG_TARGET_PRODUCT_ISHTAR)
	i2c_reg_setting.addr_type = o_ctrl->opcode.fw_addr_type;
	#else
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	#endif
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;
	fw_size = (sizeof(struct cam_sensor_i2c_reg_array) * total_bytes);
	vaddr = vmalloc(fw_size);
	if (!vaddr) {
		CAM_ERR(CAM_OIS,
			"Failed in allocating i2c_array: fw_size: %u", fw_size);
		release_firmware(fw);
		return -ENOMEM;
	}

	CAM_DBG(CAM_OIS, "FW prog size:%d.", total_bytes);

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (
		vaddr);

	#if defined(CONFIG_TARGET_PRODUCT_FUXI) || defined(CONFIG_TARGET_PRODUCT_NUWA)
	for (i = 0, ptr = (uint8_t *)fw->data, j = 0; i < total_bytes;) {
		for (cnt = 0; cnt < OIS_TRANS_SIZE && i < total_bytes;
			cnt++, ptr++, i++) {
	#else
	for (cnt = 0, ptr = (uint8_t *)fw->data; cnt < total_bytes;
		cnt++, ptr++) {
	#endif
		i2c_reg_setting.reg_setting[cnt].reg_addr =
	#if defined(CONFIG_TARGET_PRODUCT_FUXI) || defined(CONFIG_TARGET_PRODUCT_NUWA)
			o_ctrl->opcode.prog + j * OIS_TRANS_SIZE;
	#else
			o_ctrl->opcode.prog;
	#endif
		i2c_reg_setting.reg_setting[cnt].reg_data = *ptr;
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
	}
	#if defined(CONFIG_TARGET_PRODUCT_FUXI) || defined(CONFIG_TARGET_PRODUCT_NUWA)
		i2c_reg_setting.size = cnt;

		if (o_ctrl->opcode.is_addr_increase) {
			j++;
		}
	#endif

	rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
		&i2c_reg_setting, CAM_SENSOR_I2C_WRITE_BURST);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS FW(prog) size(%d) download failed. %d",
			total_bytes, rc);
		goto release_firmware;
	#if defined(CONFIG_TARGET_PRODUCT_FUXI) || defined(CONFIG_TARGET_PRODUCT_NUWA)
		}
	#endif
	}
	vfree(vaddr);
	vaddr = NULL;
	fw_size = 0;
	release_firmware(fw);

	rc = request_firmware(&fw, fw_name_coeff, dev);
	if (rc) {
		CAM_ERR(CAM_OIS, "Failed to locate %s", fw_name_coeff);
		return rc;
	}

	total_bytes = fw->size;
	#if defined(CONFIG_TARGET_PRODUCT_FUXI) || defined(CONFIG_TARGET_PRODUCT_NUWA) || defined(CONFIG_TARGET_PRODUCT_ISHTAR)
	i2c_reg_setting.addr_type = o_ctrl->opcode.fw_addr_type;
	#else
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	#endif
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;
	fw_size = (sizeof(struct cam_sensor_i2c_reg_array) * total_bytes);
	vaddr = vmalloc(fw_size);
	if (!vaddr) {
		CAM_ERR(CAM_OIS,
			"Failed in allocating i2c_array: fw_size: %u", fw_size);
		release_firmware(fw);
		return -ENOMEM;
	}

	CAM_DBG(CAM_OIS, "FW coeff size:%d", total_bytes);

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (
		vaddr);

	#if defined(CONFIG_TARGET_PRODUCT_FUXI) || defined(CONFIG_TARGET_PRODUCT_NUWA)
	for (i = 0, ptr = (uint8_t *)fw->data, j = 0; i < total_bytes;) {
		for (cnt = 0; cnt < OIS_TRANS_SIZE && i < total_bytes;
			cnt++, ptr++, i++) {
	#else
	for (cnt = 0, ptr = (uint8_t *)fw->data; cnt < total_bytes;
		cnt++, ptr++) {
	#endif
		i2c_reg_setting.reg_setting[cnt].reg_addr =
	#if defined(CONFIG_TARGET_PRODUCT_FUXI) || defined(CONFIG_TARGET_PRODUCT_NUWA)
			o_ctrl->opcode.coeff + j * OIS_TRANS_SIZE;
	#else
			o_ctrl->opcode.coeff;
	#endif
		i2c_reg_setting.reg_setting[cnt].reg_data = *ptr;
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
	}
	#if defined(CONFIG_TARGET_PRODUCT_FUXI) || defined(CONFIG_TARGET_PRODUCT_NUWA)		
	i2c_reg_setting.size = cnt;

		if (o_ctrl->opcode.is_addr_increase) {
			j++;
		}
	#endif

	rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
		&i2c_reg_setting, CAM_SENSOR_I2C_WRITE_BURST);

	#if defined(CONFIG_TARGET_PRODUCT_FUXI) || defined(CONFIG_TARGET_PRODUCT_NUWA)		
		if (rc < 0) {
	#else
	if (rc < 0)
	#endif
		CAM_ERR(CAM_OIS, "OIS FW(coeff) size(%d) download failed rc: %d",
			total_bytes, rc);
	#if defined(CONFIG_TARGET_PRODUCT_FUXI) || defined(CONFIG_TARGET_PRODUCT_NUWA)		
			goto release_firmware;
		}
	}
	vfree(vaddr);
	vaddr = NULL;
	fw_size = 0;
	release_firmware(fw);

	rc = request_firmware(&fw, fw_name_mem, dev);
	if (rc) {
		CAM_ERR(CAM_OIS, "Failed to locate %s", fw_name_mem);
		return rc;
	}

	total_bytes = fw->size;
	i2c_reg_setting.addr_type = o_ctrl->opcode.fw_addr_type;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;
	fw_size = (sizeof(struct cam_sensor_i2c_reg_array) * total_bytes);
	vaddr = vmalloc(fw_size);
	if (!vaddr) {
		CAM_ERR(CAM_OIS,
			"Failed in allocating i2c_array: fw_size: %u", fw_size);
		release_firmware(fw);
		return -ENOMEM;
	}

	CAM_DBG(CAM_OIS, "FW mem size:%d", total_bytes);

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (
		vaddr);

	for (i = 0, ptr = (uint8_t *)fw->data, j = 0; i < total_bytes;) {
		for (cnt = 0; cnt < OIS_TRANS_SIZE && i < total_bytes;
			cnt++, ptr++, i++) {
				i2c_reg_setting.reg_setting[cnt].reg_addr =
					o_ctrl->opcode.memory + j * OIS_TRANS_SIZE;
				i2c_reg_setting.reg_setting[cnt].reg_data = *ptr;
				i2c_reg_setting.reg_setting[cnt].delay = 0;
				i2c_reg_setting.reg_setting[cnt].data_mask = 0;
		}
		i2c_reg_setting.size = cnt;

		if (o_ctrl->opcode.is_addr_increase) {
			j++;
		}

		rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
			&i2c_reg_setting, CAM_SENSOR_I2C_WRITE_BURST);

		if (rc < 0)
			CAM_ERR(CAM_OIS, "OIS FW(mem) size(%d) download failed rc: %d",
				total_bytes, rc);
	}
	#endif

release_firmware:
	vfree(vaddr);
	vaddr = NULL;
	fw_size = 0;
	release_firmware(fw);
	return rc;
}

static int cam_sem1217s_ois_fw_download(struct cam_ois_ctrl_t *o_ctrl)
{
	uint8_t             txdata[TX_BUFFER_SIZE] = {0};
	uint8_t             rxdata[RX_BUFFER_SIZE] = {0};
	uint16_t            tx_buff_size = 0;
	uint16_t            i = 0;
	uint16_t            chk_index = 0;
	uint16_t            idx = 0;
	uint16_t            check_sum = 0;
	uint32_t            updated_ver = 0;
	uint32_t            new_fw_ver = 0;
	uint32_t            current_fw_ver = 0;
	char                *fw_name_prog = NULL;
	char                name_prog[64] = {0};
	uint8_t             *chk_buffer = NULL;
	uint8_t             *fw_data = NULL;
	int32_t             rc = 0;
	uint32_t            current_fw_ver_temp = 0;

	chk_buffer = vmalloc(APP_FW_SIZE);
	fw_data = vmalloc(APP_FW_SIZE);

	if (NULL == chk_buffer || NULL == fw_data) {
		rc = -ENOMEM;
		goto memory_free;
	}

	/* Get FW Ver from Binary File */
	snprintf(name_prog, 64, "%s.prog", o_ctrl->ois_name);
	fw_name_prog = name_prog;

	rc = load_fw_buff(o_ctrl, fw_name_prog, fw_data, APP_FW_SIZE);

	if (rc) {
		CAM_ERR(CAM_OIS, "[SEM1217S] Failed to load firmware: %s", fw_name_prog);
		goto memory_free;
	}

	new_fw_ver = *(uint32_t *)&fw_data[APP_FW_SIZE - 12];  /* 0x7FF4 ~ 0x7FF7 */

	rc = i2c_read_data(o_ctrl, REG_APP_VER, 4, rxdata);

	if (rc) {

		CAM_ERR(CAM_OIS, "[SEM1217S] Failed to read REG_APP_VER:0x%x", REG_APP_VER);
		rc = -EIO;
		goto memory_free;

	}

	current_fw_ver = *(uint32_t *)rxdata;
	CAM_INFO(CAM_OIS,
		"[SEM1217S] Current firmware version = %d, new firmware version = %d",
		current_fw_ver, new_fw_ver);

	if( ((current_fw_ver < new_fw_ver) && (0 == semco_ois_fw_update_op)) ||
		((current_fw_ver != new_fw_ver) && (FIRMWARE_UPDATE_FORCED == semco_ois_fw_update_op)) ||
		(FIRMWARE_UPDATE_EVERY_TIMES == semco_ois_fw_update_op)) {

		/* If there is firmware that needs to be updated, turn off OIS and AF */
		if (0 != current_fw_ver) {

			rc = i2c_read_data(o_ctrl, REG_OIS_STS, 1, rxdata); /* Read REG_OIS_STS */

			if (rc) {
				CAM_ERR(CAM_OIS,
					"[SEM1217S] Failed to read REG_OIS_STS:0x%x",
					REG_OIS_STS);
				rc = -EIO;
				goto memory_free;
			}

			if (rxdata[0] != STATE_READY) {
				txdata[0] = OIS_OFF; /* Set OIS_OFF */
				/* Write REG_OIS_CTRL information */
				rc = i2c_write_data(o_ctrl, REG_OIS_CTRL, 1, txdata, 0);
				if (rc) {
					CAM_ERR(CAM_OIS,
						"[SEM1217S] Failed to set REG_OIS_CTRL:0x%x,0x%x",
						REG_OIS_CTRL, txdata[0]);
					rc = -EIO;
					goto memory_free;
				}
			}

			rc = i2c_read_data(o_ctrl, REG_AF_STS, 1, rxdata); /* Read REG_AF_STS */

			if (rc) {
				CAM_ERR(CAM_OIS,
					"[SEM1217S] Failed to read REG_AF_STS:0x%x",
					REG_AF_STS);
				rc = -EIO;
				goto memory_free;
			}

			if (rxdata[0] != STATE_READY) {
				txdata[0] = AF_OFF; /* Set AF_OFF */
 				/* Write REG_AF_CTRL information */
				rc = i2c_write_data(o_ctrl, REG_AF_CTRL, 1, txdata, 0);
				if (rc) {
					CAM_ERR(CAM_OIS,
						"[SEM1217S] Failed to set REG_AF_CTRL:0x%x,0x%x",
						REG_AF_CTRL, txdata[0]);
					rc = -EIO;
					goto memory_free;
				}
			}
		}

		/* PAYLOAD_LEN = Packet size, FW_UPEN = TRUE */
		tx_buff_size = TX_SIZE_256_BYTE;
		switch (tx_buff_size) {
			case TX_SIZE_32_BYTE:
				txdata[0] = FWUP_CTRL_32_SET;
				break;
			case TX_SIZE_64_BYTE:
				txdata[0] = FWUP_CTRL_64_SET;
				break;
			case TX_SIZE_128_BYTE:
				txdata[0] = FWUP_CTRL_128_SET;
				break;
			case TX_SIZE_256_BYTE:
				txdata[0] = FWUP_CTRL_256_SET;
				break;
			default:
				/* Tx data size is not set, warning message */
				break;
		}

		/* Set the firmware version update control register */
		rc = i2c_write_data(o_ctrl, REG_FWUP_CTRL, 1, txdata, 0);
		if (rc) {
			CAM_ERR(CAM_OIS,
                                "[SEM1217S] Failed to set REG_FWUP_CTRL:0x%x,0x%x",
                                REG_AF_CTRL, txdata[0]);
			rc = -EIO;
			goto memory_free;
		}

		msleep(60);

		rc = i2c_read_data(o_ctrl, REG_OIS_STS, 1, rxdata);
		if (rc) {
			CAM_ERR(CAM_OIS, "[SEM1217S] Failed to read REG_OIS_STS:0x%x", REG_OIS_STS);
			rc = -EIO;
			goto memory_free;
		}

		if (STATE_FW_UPDATE != rxdata[0]) {
			CAM_INFO(CAM_OIS, "[SEM1217S] OIS firmware upgrade status check failed");
			rc = -EINVAL;
			goto memory_free;
		}

		rc = i2c_read_data(o_ctrl, REG_APP_VER, 4, rxdata);
		if (rc) {
			CAM_ERR(CAM_OIS, "[SEM1217S] Failed to read REG_APP_VER:0x%x", REG_APP_VER);
			rc = -EIO;
			goto memory_free;
		}

		current_fw_ver_temp = *(uint32_t *)rxdata;

		if (0x00 != current_fw_ver_temp) {
			CAM_ERR(CAM_OIS, "[SEM1217S] OIS firmware version check failed");
			goto memory_free;
		}

		for (i = 0; i < (APP_FW_SIZE / tx_buff_size); i++) {

			CAM_INFO(CAM_OIS, "[SEM1217S] Write REG_DATA_BUF i = %d",i);
			memcpy(&chk_buffer[tx_buff_size * i], &fw_data[idx], tx_buff_size);

			for (chk_index = 0; chk_index < tx_buff_size; chk_index += 2) {
				check_sum += ((chk_buffer[chk_index + 1 + (tx_buff_size * i)] << 8) |
				chk_buffer[chk_index + (tx_buff_size * i)]);
			}

			memcpy(txdata, &fw_data[idx], tx_buff_size);

			rc = i2c_write_data(o_ctrl, REG_DATA_BUF, tx_buff_size, txdata, 0);
			if (rc) {
				CAM_ERR(CAM_OIS,
					"[SEM1217S] Failed to write REG_DATA_BUF:0x%x, index = %d",
					REG_DATA_BUF, i);
				rc = -EIO;
				goto memory_free;
			}
			idx += tx_buff_size;
			msleep(20);

		}

		*(uint16_t *)txdata = check_sum;

		rc = i2c_write_data(o_ctrl, REG_FWUP_CHKSUM, 2, txdata, 0);
		if (rc) {
			CAM_ERR(CAM_OIS,
				"[SEM1217S] Failed to set REG_FWUP_CTRL:0x%x,0x%x",
				REG_AF_CTRL, txdata[0]);
			rc = -EIO;
			goto memory_free;
		}

		msleep(200);

		rc = i2c_read_data(o_ctrl, REG_FWUP_CHKSUM, 2, rxdata);
		if (rc) {
			CAM_ERR(CAM_OIS,
				"[SEM1217S] Failed to read REG_FWUP_CHKSUM:0x%x",
				REG_FWUP_CHKSUM);
			rc = -EIO;
			goto memory_free;
		}

		CAM_INFO(CAM_OIS, "[SEM1217S] REG_FWUP_CHKSUM = 0x%x, 0x%x", rxdata[0], rxdata[1]);

		rc = i2c_read_data(o_ctrl, REG_FWUP_ERR, 1, rxdata);
		if (rc) {
			CAM_ERR(CAM_OIS, "[SEM1217S] Failed to read REG_FWUP_ERR:0x%x", REG_FWUP_ERR);
			rc = -EIO;
			goto memory_free;
		}

		CAM_INFO(CAM_OIS, "[SEM1217S] REG_FWUP_ERR = 0x%x", rxdata[0]);

		if (rxdata[0] != NO_ERROR) {
			CAM_ERR(CAM_OIS, "[SEM1217S] Failed to update firmware");
			rc = -EINVAL;
			goto memory_free;
		}

		txdata[0] = RESET_REQ;
		rc = i2c_write_data(o_ctrl, REG_FWUP_CTRL, 1, txdata, 0);
		if (rc) {
			CAM_ERR(CAM_OIS,
				"[SEM1217S] Failed to set REG_FWUP_CTRL:0x%x,0x%x",
				REG_FWUP_CTRL, txdata[0]);
			rc = -EIO;
			goto memory_free;
		}

		msleep(200);

		rc = i2c_read_data(o_ctrl, REG_APP_VER, 4, rxdata);
		if (rc) {
			CAM_ERR(CAM_OIS, "[SEM1217S] Failed to read REG_APP_VER:0x%x", REG_APP_VER);
			rc = -EIO;
			goto memory_free;
		}

		updated_ver = *(uint32_t *)rxdata;
		CAM_INFO(CAM_OIS,
			"[SEM1217S] firmware version = %d, new firmware version = %d",
			updated_ver, new_fw_ver);

		if (updated_ver != new_fw_ver) {
			CAM_ERR(CAM_OIS, "[SEM1217S] updated_ver != new_fw_ver");
			rc = -EINVAL;
			goto memory_free;
		}

		CAM_INFO(CAM_OIS, "[SEM1217S] Firmware update success");
	}

memory_free:

	if (NULL != chk_buffer) {
		vfree(chk_buffer);
	}

	if (NULL != fw_data) {
		vfree(fw_data);
	}

	return rc;
}

static int cam_ois_fw_download(struct cam_ois_ctrl_t *o_ctrl)
{
	int32_t   rc  = 0;

	switch (o_ctrl->opcode.fw_download_type){
		case 1217:
			CAM_DBG(CAM_OIS, "Apply sem1217s OIS firmware update function");
			rc = cam_sem1217s_ois_fw_download(o_ctrl);
			break;

		default:
			CAM_DBG(CAM_OIS, "Apply default firmware update function");
			rc = cam_default_ois_fw_download(o_ctrl);
			break;
	}

	return rc;
}

/**
 * cam_ois_pkt_parse - Parse csl packet
 * @o_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
static int cam_ois_pkt_parse(struct cam_ois_ctrl_t *o_ctrl, void *arg)
{
	int32_t                         rc = 0;
	int32_t                         i = 0, j =0;
	uint32_t                        total_cmd_buf_in_bytes = 0;
	struct common_header           *cmm_hdr = NULL;
	uintptr_t                       generic_ptr;
	struct cam_control             *ioctl_ctrl = NULL;
	struct cam_config_dev_cmd       dev_config;
	struct i2c_settings_array      *i2c_reg_settings = NULL;
	struct cam_cmd_buf_desc        *cmd_desc = NULL;
	uintptr_t                       generic_pkt_addr;
	size_t                          pkt_len;
	size_t                          remain_len = 0;
	struct cam_packet              *csl_packet = NULL;
	struct cam_packet              *csl_packet_u = NULL;
	size_t                          len_of_buff = 0;
	uint32_t                       *offset = NULL, *cmd_buf;
	struct cam_ois_soc_private     *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t  *power_info = &soc_private->power_info;
	const struct flash_ois_function *ps = pflash_ois;
	uint8_t                          config_flag = 0;

	ioctl_ctrl = (struct cam_control *)arg;
	if (copy_from_user(&dev_config,
		u64_to_user_ptr(ioctl_ctrl->handle),
		sizeof(dev_config)))
		return -EFAULT;
	rc = cam_mem_get_cpu_buf(dev_config.packet_handle,
		&generic_pkt_addr, &pkt_len);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"error in converting command Handle Error: %d", rc);
		return rc;
	}

	remain_len = pkt_len;
	if ((sizeof(struct cam_packet) > pkt_len) ||
		((size_t)dev_config.offset >= pkt_len -
		sizeof(struct cam_packet))) {
		CAM_ERR(CAM_OIS,
			"Inval cam_packet strut size: %zu, len_of_buff: %zu",
			 sizeof(struct cam_packet), pkt_len);
		rc = -EINVAL;
		goto put_ref;
	}

	remain_len -= (size_t)dev_config.offset;
	csl_packet_u = (struct cam_packet *)
		(generic_pkt_addr + (uint32_t)dev_config.offset);
	rc = cam_packet_util_copy_pkt_to_kmd(csl_packet_u, &csl_packet, remain_len);
	if (rc) {
		CAM_ERR(CAM_OIS, "Copying packet to KMD failed");
		goto put_ref;
	}

	switch (csl_packet->header.op_code & 0xFFFFFF) {
	case CAM_OIS_PACKET_OPCODE_INIT:
		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);

		CAM_DBG(CAM_OIS, "num_cmd_buf %d",
			csl_packet->num_cmd_buf);

		/* Loop through multiple command buffers */
		for (i = 0; i < csl_packet->num_cmd_buf; i++) {
			rc = cam_packet_util_validate_cmd_desc(&cmd_desc[i]);
			if (rc) {
				CAM_ERR(CAM_OIS, "Invalid cmd desc");
				goto end;
			}

			total_cmd_buf_in_bytes = cmd_desc[i].length;
			if (!total_cmd_buf_in_bytes)
				continue;

			rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
				&generic_ptr, &len_of_buff);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "Failed to get cpu buf : 0x%x",
					cmd_desc[i].mem_handle);
				goto end;
			}
			cmd_buf = (uint32_t *)generic_ptr;
			if (!cmd_buf) {
				CAM_ERR(CAM_OIS, "invalid cmd buf");
				rc = -EINVAL;
				cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
				goto end;
			}

			if ((len_of_buff < sizeof(struct common_header)) ||
				(cmd_desc[i].offset > (len_of_buff -
				sizeof(struct common_header)))) {
				CAM_ERR(CAM_OIS,
					"Invalid length for sensor cmd");
				rc = -EINVAL;
				cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
				goto end;
			}
			remain_len = len_of_buff - cmd_desc[i].offset;
			cmd_buf += cmd_desc[i].offset / sizeof(uint32_t);
			cmm_hdr = (struct common_header *)cmd_buf;

			switch (cmm_hdr->cmd_type) {
			case CAMERA_SENSOR_CMD_TYPE_I2C_INFO:
				rc = cam_ois_slaveInfo_pkt_parser(
					o_ctrl, cmd_buf, remain_len);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
					"Failed in parsing slave info");
					break;
				}
				break;
			case CAMERA_SENSOR_CMD_TYPE_PWR_UP:
			case CAMERA_SENSOR_CMD_TYPE_PWR_DOWN:
				CAM_DBG(CAM_OIS,
					"Received power settings buffer");
				rc = cam_sensor_update_power_settings(
					cmd_buf,
					total_cmd_buf_in_bytes,
					power_info, remain_len);
				if (rc) {
					CAM_ERR(CAM_OIS,
					"Failed: parse power settings");
					break;
				}
				break;
			default:
			if (o_ctrl->i2c_init_data.is_settings_valid == 0) {
				CAM_DBG(CAM_OIS,
				"Received init/config settings");
				i2c_reg_settings =
					&(o_ctrl->i2c_init_data);
				i2c_reg_settings->is_settings_valid = 1;
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1, NULL);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
					"init parsing failed: %d", rc);
					break;
				}
			} else if ((o_ctrl->is_ois_calib != 0) &&
				(o_ctrl->i2c_calib_data.is_settings_valid ==
				0)) {
				CAM_DBG(CAM_OIS,
					"Received calib settings");
				i2c_reg_settings = &(o_ctrl->i2c_calib_data);
				i2c_reg_settings->is_settings_valid = 1;
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1, NULL);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
						"Calib parsing failed: %d", rc);
					break;
				}
			} else if (o_ctrl->i2c_fwinit_data.is_settings_valid == 0) {
				CAM_DBG(CAM_OIS, "received fwinit settings");
				i2c_reg_settings =
					&(o_ctrl->i2c_fwinit_data);
				i2c_reg_settings->is_settings_valid = 1;
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1, NULL);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
					"fw init parsing failed: %d", rc);
					break;
				}
			}
#if defined(CONFIG_TARGET_PRODUCT_NUWA)
			else if (o_ctrl->i2c_postinit_data.is_settings_valid == 0)
			{
				CAM_DBG(CAM_OIS, "received postinit settings");
				i2c_reg_settings =
					&(o_ctrl->i2c_postinit_data);
				i2c_reg_settings->is_settings_valid = 1;
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1, NULL);
				if (rc < 0)
				{
					CAM_ERR(CAM_OIS,
							"post init parsing failed: %d", rc);
					return rc;
				}
			}
#endif
			break;
			}
			cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);

			if (rc < 0)
				goto end;
		}

		if (o_ctrl->cam_ois_state != CAM_OIS_CONFIG) {
			rc = cam_ois_power_up(o_ctrl);
			if (rc) {
				CAM_ERR(CAM_OIS, " OIS Power up failed");
				goto end;
			}
		}
		//interface for different ois add by xiaomi
		if ( o_ctrl->opcode.customized_ois_flag ) {
			for(j = 0; j < sizeof(pflash_ois) / sizeof(struct flash_ois_function) ; j++)
			{
				if(ps[j].flag == o_ctrl->opcode.customized_ois_flag){
					config_flag++;
					rc = ps[j].mi_ois_pkt_download(o_ctrl);
					if (rc) {
						CAM_ERR(CAM_OIS, "Failed OIS Customer Pkt Download");
						goto pwr_dwn;
					}
				}
			}
			if ( config_flag != 1 ) {
				CAM_ERR(CAM_OIS, "ERROR! need  pkt function or repeat flag , flag  %d", config_flag);
			}
		} else {
			if (o_ctrl->i2c_fwinit_data.is_settings_valid == 1) {
				rc = cam_ois_apply_settings(o_ctrl,
					&o_ctrl->i2c_fwinit_data);
				if ((rc == -EAGAIN) &&
					(o_ctrl->io_master_info.master_type == CCI_MASTER)) {
					CAM_WARN(CAM_OIS,
						"CCI HW is restting: Reapplying fwinit settings");
					usleep_range(1000, 1010);
					rc = cam_ois_apply_settings(o_ctrl,
						&o_ctrl->i2c_fwinit_data);
				}
				if (rc) {
					CAM_ERR(CAM_OIS,
						"Cannot apply fwinit data %d",
						rc);
					goto pwr_dwn;
				} else {
					CAM_DBG(CAM_OIS, "OIS fwinit settings success");
				}
			}

			if (o_ctrl->ois_fw_flag) {
				rc = cam_ois_fw_download(o_ctrl);
				if (rc) {
					CAM_ERR(CAM_OIS, "Failed OIS FW Download");
					goto pwr_dwn;
				}
			}

			rc = cam_ois_apply_settings(o_ctrl, &o_ctrl->i2c_init_data);
			if ((rc == -EAGAIN) &&
				(o_ctrl->io_master_info.master_type == CCI_MASTER)) {
				CAM_WARN(CAM_OIS,
					"CCI HW is restting: Reapplying INIT settings");
				usleep_range(1000, 1010);
				rc = cam_ois_apply_settings(o_ctrl,
					&o_ctrl->i2c_init_data);
			}
			if (rc < 0) {
				CAM_ERR(CAM_OIS,
					"Cannot apply Init settings: rc = %d",
					rc);
				goto pwr_dwn;
			} else {
				CAM_DBG(CAM_OIS, "apply Init settings success");
			}

			if (o_ctrl->is_ois_calib) {
				rc = cam_ois_apply_settings(o_ctrl,
					&o_ctrl->i2c_calib_data);
				if ((rc == -EAGAIN) &&
					(o_ctrl->io_master_info.master_type == CCI_MASTER)) {
					CAM_WARN(CAM_OIS,
						"CCI HW is restting: Reapplying calib settings");
					usleep_range(1000, 1010);
					rc = cam_ois_apply_settings(o_ctrl,
						&o_ctrl->i2c_calib_data);
				}
				if (rc) {
					CAM_ERR(CAM_OIS, "Cannot apply calib data");
					goto pwr_dwn;
				} else {
					CAM_DBG(CAM_OIS, "apply calib data settings success");
				}
			}
#if defined(CONFIG_TARGET_PRODUCT_NUWA)
			if (o_ctrl->i2c_postinit_data.is_settings_valid == 1) {
				rc = cam_ois_apply_settings(o_ctrl, &o_ctrl->i2c_postinit_data);
				if ((rc == -EAGAIN) && (o_ctrl->io_master_info.master_type == CCI_MASTER)) {
					CAM_WARN(CAM_OIS, "CCI HW is restting: Reapplying postinit settings");
					usleep_range(1000, 1010);
					rc = cam_ois_apply_settings(o_ctrl, &o_ctrl->i2c_postinit_data);
				}
				if (rc) {
					CAM_ERR(CAM_OIS, "Cannot apply postinit data %d", rc);
					goto pwr_dwn;
				}
				else {
				CAM_DBG(CAM_OIS, "OIS postinit settings success");
				}
			}
#endif
		o_ctrl->cam_ois_state = CAM_OIS_CONFIG;
		}
		rc = delete_request(&o_ctrl->i2c_fwinit_data);
		if (rc < 0) {
			CAM_WARN(CAM_OIS,
				"Fail deleting fwinit data: rc: %d", rc);
			rc = 0;
		}

		rc = delete_request(&o_ctrl->i2c_init_data);
		if (rc < 0) {
			CAM_WARN(CAM_OIS,
				"Fail deleting Init data: rc: %d", rc);
			rc = 0;
		}
		rc = delete_request(&o_ctrl->i2c_calib_data);
		if (rc < 0) {
			CAM_WARN(CAM_OIS,
				"Fail deleting Calibration data: rc: %d", rc);
			rc = 0;
		}
#if defined(CONFIG_TARGET_PRODUCT_NUWA)
		rc = delete_request(&o_ctrl->i2c_postinit_data);
		if (rc < 0) {
			CAM_WARN(CAM_OIS,
					 "Fail deleting postinit data: rc: %d", rc);
			rc = 0;
		}
#endif
		break;
	case CAM_OIS_PACKET_OPCODE_OIS_CONTROL:
		if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Not in right state to control OIS: %d",
				o_ctrl->cam_ois_state);
			goto end;
		}
		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		i2c_reg_settings = &(o_ctrl->i2c_mode_data);
		i2c_reg_settings->is_settings_valid = 1;
		i2c_reg_settings->request_id = 0;
		rc = cam_sensor_i2c_command_parser(&o_ctrl->io_master_info,
			i2c_reg_settings,
			cmd_desc, 1, NULL);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS pkt parsing failed: %d", rc);
			goto end;
		}

		rc = cam_ois_apply_settings(o_ctrl, i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Cannot apply mode settings");
			goto end;
		}

		rc = delete_request(i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Fail deleting Mode data: rc: %d", rc);
			goto end;
		}
		break;
	case CAM_OIS_PACKET_OPCODE_READ: {
		uint64_t qtime_ns;
		struct cam_buf_io_cfg *io_cfg;
		struct i2c_settings_array i2c_read_settings;

		if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Not in right state to read OIS: %d",
				o_ctrl->cam_ois_state);
			goto end;
		}
		CAM_DBG(CAM_OIS, "number of I/O configs: %d:",
			csl_packet->num_io_configs);
		if (csl_packet->num_io_configs == 0) {
			CAM_ERR(CAM_OIS, "No I/O configs to process");
			rc = -EINVAL;
			goto end;
		}

		INIT_LIST_HEAD(&(i2c_read_settings.list_head));

		io_cfg = (struct cam_buf_io_cfg *) ((uint8_t *)
			&csl_packet->payload +
			csl_packet->io_configs_offset);

		/* validate read data io config */
		if (io_cfg == NULL) {
			CAM_ERR(CAM_OIS, "I/O config is invalid(NULL)");
			rc = -EINVAL;
			goto end;
		}

		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		i2c_read_settings.is_settings_valid = 1;
		i2c_read_settings.request_id = 0;
		rc = cam_sensor_i2c_command_parser(&o_ctrl->io_master_info,
			&i2c_read_settings,
			cmd_desc, 1, &io_cfg[0]);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS read pkt parsing failed: %d", rc);
			goto end;
		}

		rc = cam_sensor_util_get_current_qtimer_ns(&qtime_ns);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "failed to get qtimer rc:%d");
			goto end;
		}

		rc = cam_sensor_i2c_read_data(
			&i2c_read_settings,
			&o_ctrl->io_master_info);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "cannot read data rc: %d", rc);
			delete_request(&i2c_read_settings);
			goto end;
		}

		if (csl_packet->num_io_configs > 1) {
			rc = cam_sensor_util_write_qtimer_to_io_buffer(
				qtime_ns, &io_cfg[1]);
			if (rc < 0) {
				CAM_ERR(CAM_OIS,
					"write qtimer failed rc: %d", rc);
				delete_request(&i2c_read_settings);
				goto end;
			}
		}

		rc = delete_request(&i2c_read_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Failed in deleting the read settings");
			goto end;
		}
		break;
	}
	case CAM_OIS_PACKET_OPCODE_WRITE_TIME: {
		if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_ERR(CAM_OIS,
				"Not in right state to write time to OIS: %d",
				o_ctrl->cam_ois_state);
			goto end;
		}
		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		i2c_reg_settings = &(o_ctrl->i2c_time_data);
		i2c_reg_settings->is_settings_valid = 1;
		i2c_reg_settings->request_id = 0;
		rc = cam_sensor_i2c_command_parser(&o_ctrl->io_master_info,
			i2c_reg_settings,
			cmd_desc, 1, NULL);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS pkt parsing failed: %d", rc);
			goto end;
		}

		rc = cam_ois_update_time(i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Cannot update time");
			goto end;
		}

		rc = cam_ois_apply_settings(o_ctrl, i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Cannot apply mode settings");
			goto end;
		}

		rc = delete_request(i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Fail deleting Mode data: rc: %d", rc);
			goto end;
		}
		break;
	}
	default:
		CAM_ERR(CAM_OIS, "Invalid Opcode: %d",
			(csl_packet->header.op_code & 0xFFFFFF));
		rc = -EINVAL;
		goto end;
	}

	if (!rc)
		goto end;

pwr_dwn:
	cam_ois_power_down(o_ctrl);
end:
	cam_common_mem_free(csl_packet);
put_ref:
	cam_mem_put_cpu_buf(dev_config.packet_handle);
	return rc;
}

void cam_ois_shutdown(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc = 0;
	struct cam_ois_soc_private *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;

	if (o_ctrl->cam_ois_state == CAM_OIS_INIT)
		return;

	if (o_ctrl->cam_ois_state >= CAM_OIS_CONFIG) {
		rc = cam_ois_power_down(o_ctrl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "OIS Power down failed");
	}

	if (o_ctrl->cam_ois_state >= CAM_OIS_ACQUIRE) {
		rc = cam_destroy_device_hdl(o_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "destroying the device hdl");
		o_ctrl->bridge_intf.device_hdl = -1;
		o_ctrl->bridge_intf.link_hdl = -1;
		o_ctrl->bridge_intf.session_hdl = -1;
	}

	if (o_ctrl->i2c_fwinit_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_fwinit_data);

	if (o_ctrl->i2c_mode_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_mode_data);

	if (o_ctrl->i2c_calib_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_calib_data);

	if (o_ctrl->i2c_init_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_init_data);
#if defined(CONFIG_TARGET_PRODUCT_NUWA)
	if (o_ctrl->i2c_postinit_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_postinit_data);
#endif
	kfree(power_info->power_setting);
	kfree(power_info->power_down_setting);
	power_info->power_setting = NULL;
	power_info->power_down_setting = NULL;
	power_info->power_down_setting_size = 0;
	power_info->power_setting_size = 0;

	o_ctrl->cam_ois_state = CAM_OIS_INIT;
}

/**
 * cam_ois_driver_cmd - Handle ois cmds
 * @e_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
int cam_ois_driver_cmd(struct cam_ois_ctrl_t *o_ctrl, void *arg)
{
	int                              rc = 0;
	struct cam_ois_query_cap_t       ois_cap = {0};
	struct cam_control              *cmd = (struct cam_control *)arg;
	struct cam_ois_soc_private      *soc_private = NULL;
	struct cam_sensor_power_ctrl_t  *power_info = NULL;

	if (!o_ctrl || !cmd) {
		CAM_ERR(CAM_OIS, "Invalid arguments");
		return -EINVAL;
	}

	if (cmd->handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_OIS, "Invalid handle type: %d",
			cmd->handle_type);
		return -EINVAL;
	}

	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	mutex_lock(&(o_ctrl->ois_mutex));
	switch (cmd->op_code) {
	case CAM_QUERY_CAP:
		ois_cap.slot_info = o_ctrl->soc_info.index;

		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&ois_cap,
			sizeof(struct cam_ois_query_cap_t))) {
			CAM_ERR(CAM_OIS, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}
		CAM_DBG(CAM_OIS, "ois_cap: ID: %d", ois_cap.slot_info);
		break;
	case CAM_ACQUIRE_DEV:
		rc = cam_ois_get_dev_handle(o_ctrl, arg);
		if (rc) {
			CAM_ERR(CAM_OIS, "Failed to acquire dev");
			goto release_mutex;
		}

		o_ctrl->cam_ois_state = CAM_OIS_ACQUIRE;
		break;
	case CAM_START_DEV:
		if (o_ctrl->cam_ois_state != CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
			"Not in right state for start : %d",
			o_ctrl->cam_ois_state);
			goto release_mutex;
		}
		o_ctrl->cam_ois_state = CAM_OIS_START;
		break;
	case CAM_CONFIG_DEV:
		rc = cam_ois_pkt_parse(o_ctrl, arg);
		if (rc) {
			CAM_ERR(CAM_OIS, "Failed in ois pkt Parsing");
			goto release_mutex;
		}
		break;
	case CAM_RELEASE_DEV:
		if (o_ctrl->cam_ois_state == CAM_OIS_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Cant release ois: in start state");
			goto release_mutex;
		}

		if (o_ctrl->cam_ois_state == CAM_OIS_CONFIG) {
			rc = cam_ois_power_down(o_ctrl);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "OIS Power down failed");
				goto release_mutex;
			}
		}

		if (o_ctrl->bridge_intf.device_hdl == -1) {
			CAM_ERR(CAM_OIS, "link hdl: %d device hdl: %d",
				o_ctrl->bridge_intf.device_hdl,
				o_ctrl->bridge_intf.link_hdl);
			rc = -EINVAL;
			goto release_mutex;
		}
		rc = cam_destroy_device_hdl(o_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "destroying the device hdl");
		o_ctrl->bridge_intf.device_hdl = -1;
		o_ctrl->bridge_intf.link_hdl = -1;
		o_ctrl->bridge_intf.session_hdl = -1;
		o_ctrl->cam_ois_state = CAM_OIS_INIT;

		kfree(power_info->power_setting);
		kfree(power_info->power_down_setting);
		power_info->power_setting = NULL;
		power_info->power_down_setting = NULL;
		power_info->power_down_setting_size = 0;
		power_info->power_setting_size = 0;

		if (o_ctrl->i2c_mode_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_mode_data);

		if (o_ctrl->i2c_calib_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_calib_data);

		if (o_ctrl->i2c_init_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_init_data);

		if (o_ctrl->i2c_fwinit_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_fwinit_data);
#if defined(CONFIG_TARGET_PRODUCT_NUWA)
		if (o_ctrl->i2c_postinit_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_postinit_data);
#endif
		break;
	case CAM_STOP_DEV:
		if (o_ctrl->cam_ois_state != CAM_OIS_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
			"Not in right state for stop : %d",
			o_ctrl->cam_ois_state);
		}
		o_ctrl->cam_ois_state = CAM_OIS_CONFIG;
		break;
	case CAM_FLUSH_REQ:
		// ignore the flush cmd
		CAM_DBG(CAM_OIS, "CAM_FLUSH_REQ");
		break;
	default:
		CAM_ERR(CAM_OIS, "invalid opcode");
		goto release_mutex;
	}
release_mutex:
	mutex_unlock(&(o_ctrl->ois_mutex));
	return rc;
}
