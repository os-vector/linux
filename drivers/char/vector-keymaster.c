// SPDX-License-Identifier: GPL-2.0

// a small more-kernelized implementation of old qseecom
// only for the anki vector robot

// this could potentially serve as an ok backbone for a
// real legacy qseecom driver. i bet a lot of this might
// work for the snapdragon 801 and similar, which could
// open up the possibility of fingerprint sensor access
// and such

#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kernel_read_file.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>

#include <uapi/linux/vector-keymaster.h>

#define SCM_LEGACY_FNID(s, c) (((s) << 10) | ((c) & 0x3ff))
#define QCOM_SCM_INTERRUPTED 1

#define SVC_INFO 6
#define CMD_GET_FEATURE_VERSION 3
#define FEATURE_QSEE_VERSION 10
#define FEATURE_ID_WHITELIST 15
#define WHITELIST_MIN_VERSION 0x00400000

#define SCM_SVC_TZSCHEDULER 0xFC
#define SCM_TZ_CMD_DISPATCH 1

#define QSEOS_APP_START_COMMAND 1
#define QSEOS_APP_LOOKUP_COMMAND 3
#define QSEOS_CLIENT_SEND_DATA_COMMAND 6
#define QSEOS_LOAD_SERV_IMAGE_COMMAND 11
#define QSEOS_APP_REGION_NOTIFICATION 13
#define QSEOS_CLIENT_SEND_DATA_COMMAND_WHITELIST 0x1C

#define QSEOS_RESULT_SUCCESS 0x00000000
#define QSEOS_RESULT_INCOMPLETE 0x00000001
#define QSEOS_RESULT_BLOCKED_ON_LISTENER 0x00000002
#define QSEOS_RESULT_FAILURE 0xFFFFFFFF
#define QSEOS_APP_ID 0xEE01

#define MAX_APP_NAME_SIZE 64

#define MAX_ION_FD 4
#define SGLISTINFO_TABLE_SIZE (MAX_ION_FD * 8)
#define SGLISTINFO_FLAG_SINGLE_PTR (1u << 31)

struct __packed sglist_info {
	__le32 indexAndFlags;
	__le32 sizeOrCount;
};

#define SECAPP_REGION_ADDR 0x87a00000u
#define SECAPP_REGION_SIZE 0x00200000u

#define QSEECOM_ALIGN_SIZE 0x40
#define QSEECOM_ALIGN(x) (((x) + (QSEECOM_ALIGN_SIZE - 1)) & \
								~(QSEECOM_ALIGN_SIZE - 1))

struct fw_segment {
	const char *path;
	size_t offset;
	size_t expected_size;
};

static const struct fw_segment km_segments[] = {
	{ "/firmware/image/keymaste.mdt", 0,     6748  },
	{ "/firmware/image/keymaste.b00", 6748,  180   },
	{ "/firmware/image/keymaste.b01", 6928,  6568  },
	{ "/firmware/image/keymaste.b02", 13496, 18072 },
	{ "/firmware/image/keymaste.b03", 31568, 208   },
};
#define KM_MDT_SIZE		6748u
#define KM_TOTAL_SIZE		31776u
#define KM_ALLOC_SIZE		(64 * 1024)

static const struct fw_segment cmnlib_segments[] = {
	{ "/firmware/image/cmnlib.mdt", 0,      6748   },
	{ "/firmware/image/cmnlib.b00", 6748,   180    },
	{ "/firmware/image/cmnlib.b01", 6928,   6568   },
	{ "/firmware/image/cmnlib.b02", 13496,  114372 },
	{ "/firmware/image/cmnlib.b03", 127868, 4456   },
};
#define CMNLIB_MDT_SIZE 6748u
#define CMNLIB_TOTAL_SIZE 132324u
#define CMNLIB_ALLOC_SIZE (144 * 1024)

#define KEYMASTER_GENERATE_KEYPAIR 0x1
#define KEYMASTER_SIGN_DATA	0x3
#define KEYMASTER_FAILURE (-1)
#define TYPE_RSA 0x1
#define DIGEST_NONE 0x0
#define PADDING_NONE 0x0
#define KM_KEY_SIZE_MAX 512
#define KM_IV_LENGTH 16
#define KM_HMAC_LENGTH 32

struct __packed qcom_km_key_blob {
	u32 magic_num;
	u32 version_num;
	u8  modulus[KM_KEY_SIZE_MAX];
	u32 modulus_size;
	u8  public_exponent[KM_KEY_SIZE_MAX];
	u32 public_exponent_size;
	u8  iv[KM_IV_LENGTH];
	u8  encrypted_private_exponent[KM_KEY_SIZE_MAX];
	u32 encrypted_private_exponent_size;
	u8  hmac[KM_HMAC_LENGTH];
};

struct __packed km_rsa_sign_params {
	u32 digest_type;
	u32 padding_type;
};

struct __packed km_sign_data_cmd {
	u32 cmd_id;
	struct km_rsa_sign_params sign_param;
	struct qcom_km_key_blob key_blob;
	u32 data;
	u32 dlen;
};

struct __packed km_sign_data_resp {
	u32 cmd_id;
	u8  signed_data[KM_KEY_SIZE_MAX];
	u32 sig_len;
	s32 status;
};

struct km_rsa_keygen_params {
	u32 modulus_size;
	u64 public_exponent;
};

struct km_gen_keypair_cmd {
	u32 cmd_id;
	u32 key_type;
	struct km_rsa_keygen_params rsa_params;
};

struct __packed km_gen_keypair_resp {
	u32 cmd_id;
	struct qcom_km_key_blob key_blob;
	u32 key_blob_len;
	s32 status;
};

struct scm_legacy_command {
	__le32 len;
	__le32 buf_offset;
	__le32 resp_hdr_offset;
	__le32 id;
	__le32 buf[];
};

struct scm_legacy_response {
	__le32 len;
	__le32 buf_offset;
	__le32 is_complete;
};

#define LEGACY_CALL_POLL_TIMEOUT_MS 5000
#define LEGACY_CALL_INTERRUPT_TIMEOUT_MS 30000

struct __packed check_app_req {
	__le32 qsee_cmd_id;
	char   app_name[MAX_APP_NAME_SIZE];
};

struct __packed load_app_req {
	__le32 qsee_cmd_id;
	__le32 mdt_len;
	__le32 img_len;
	__le32 phy_addr;
	char   app_name[MAX_APP_NAME_SIZE];
};

struct __packed load_serv_image_req {
	__le32 qsee_cmd_id;
	__le32 mdt_len;
	__le32 img_len;
	__le32 phy_addr;
};

struct __packed send_data_req {
	__le32 qsee_cmd_id;
	__le32 app_id;
	__le32 req_ptr;
	__le32 req_len;
	__le32 rsp_ptr;
	__le32 rsp_len;
	__le32 sglistinfo_ptr;
	__le32 sglistinfo_len;
};

struct __packed region_notif_req {
	__le32 qsee_cmd_id;
	__le32 addr;
	__le32 size;
};

struct __packed scm_resp {
	__le32 result;
	__le32 resp_type;
	__le32 data;
};

struct vkm_ctx {
	struct device *dev; // our own platform device
	struct device *scm_dev; // qcom,scm device for DMA

	struct clk *clk_ce_core;
	struct clk *clk_ce_iface;
	struct clk *clk_ce_bus;
	bool clocks_enabled;

	struct mutex lock;

	bool whitelist_supported;
	bool ta_loaded;
	u32 app_id;

	struct miscdevice miscdev;
};

struct mapped_buf {
	void *virt;
	dma_addr_t phys;
	size_t size;
};

static int mbuf_alloc(struct device *dev, struct mapped_buf *m, size_t size)
{
	if (size < PAGE_SIZE)
		size = PAGE_SIZE;
	m->virt = dma_alloc_coherent(dev, size, &m->phys, GFP_KERNEL);
	if (!m->virt)
		return -ENOMEM;
	m->size = size;
	return 0;
}

static void mbuf_free(struct device *dev, struct mapped_buf *m)
{
	if (m->virt) {
		dma_free_coherent(dev, m->size, m->virt, m->phys);
		m->virt = NULL;
	}
}

static int legacy_call(struct vkm_ctx *v, u32 svc, u32 cmd,
		       const void *req, size_t req_len,
		       void *resp, size_t resp_len)
{
	size_t alloc_len = sizeof(struct scm_legacy_command) + req_len +
			   sizeof(struct scm_legacy_response) + resp_len;
	struct scm_legacy_command *cmdbuf;
	struct scm_legacy_response *rsp_hdr;
	void *req_area, *resp_area;
	dma_addr_t cmd_phys;
	struct arm_smccc_res smc_res;
	int context_id;
	int ret = 0;
	unsigned long deadline;
	unsigned long interrupt_deadline;

	cmdbuf = kzalloc(PAGE_ALIGN(alloc_len), GFP_KERNEL);
	if (!cmdbuf)
		return -ENOMEM;

	cmdbuf->len = cpu_to_le32(alloc_len);
	cmdbuf->buf_offset = cpu_to_le32(sizeof(*cmdbuf));
	cmdbuf->resp_hdr_offset = cpu_to_le32(sizeof(*cmdbuf) + req_len);
	cmdbuf->id = cpu_to_le32(SCM_LEGACY_FNID(svc, cmd));

	req_area = (void *)cmdbuf + sizeof(*cmdbuf);
	memcpy(req_area, req, req_len);

	rsp_hdr = (void *)cmdbuf + le32_to_cpu(cmdbuf->resp_hdr_offset);
	resp_area = (void *)rsp_hdr + sizeof(*rsp_hdr);

	cmd_phys = dma_map_single(v->scm_dev, cmdbuf, alloc_len, DMA_TO_DEVICE);
	if (dma_mapping_error(v->scm_dev, cmd_phys)) {
		ret = -ENOMEM;
		goto out_free;
	}

	interrupt_deadline = jiffies +
		msecs_to_jiffies(LEGACY_CALL_INTERRUPT_TIMEOUT_MS);
	do {
		arm_smccc_smc(1, (unsigned long)&context_id,
			      (unsigned long)cmd_phys,
			      0, 0, 0, 0, 0, &smc_res);
		if (smc_res.a0 != QCOM_SCM_INTERRUPTED)
			break;
		if (time_after(jiffies, interrupt_deadline)) {
			dev_err(v->dev, "SCM INTERRUPTED timeout after %u ms\n",
				LEGACY_CALL_INTERRUPT_TIMEOUT_MS);
			ret = -ETIMEDOUT;
			goto out_unmap;
		}
		cond_resched();
	} while (1);

	if (smc_res.a0) {
		ret = (int)(long)smc_res.a0;
		goto out_unmap;
	}

	deadline = jiffies + msecs_to_jiffies(LEGACY_CALL_POLL_TIMEOUT_MS);
	while (!rsp_hdr->is_complete) {
		dma_sync_single_for_cpu(v->scm_dev,
			cmd_phys + sizeof(*cmdbuf) + req_len,
			sizeof(*rsp_hdr), DMA_FROM_DEVICE);
		if (rsp_hdr->is_complete)
			break;
		if (time_after(jiffies, deadline)) {
			dev_err(v->dev, "SCM is_complete timeout after %u ms\n",
				LEGACY_CALL_POLL_TIMEOUT_MS);
			ret = -ETIMEDOUT;
			goto out_unmap;
		}
		usleep_range(100, 500);
	}

	dma_sync_single_for_cpu(v->scm_dev,
		cmd_phys + sizeof(*cmdbuf) + req_len + sizeof(*rsp_hdr),
		resp_len, DMA_FROM_DEVICE);

	memcpy(resp, resp_area, resp_len);

out_unmap:
	dma_unmap_single(v->scm_dev, cmd_phys, alloc_len, DMA_TO_DEVICE);
out_free:
	kfree(cmdbuf);
	return ret;
}

static u32 query_feature(struct vkm_ctx *v, u32 feature_id)
{
	u32 req = feature_id;
	u32 resp[4] = {0};

	if (legacy_call(v, SVC_INFO, CMD_GET_FEATURE_VERSION,
			&req, sizeof(req), resp, sizeof(resp)))
		return 0;
	return resp[0];
}

static int app_lookup(struct vkm_ctx *v, const char *app_name)
{
	struct check_app_req req = {0};
	struct scm_resp resp = {0};
	u32 result, resp_type, data;
	int ret;

	req.qsee_cmd_id = cpu_to_le32(QSEOS_APP_LOOKUP_COMMAND);
	strscpy(req.app_name, app_name, sizeof(req.app_name));

	ret = legacy_call(v, SCM_SVC_TZSCHEDULER, SCM_TZ_CMD_DISPATCH,
			  &req, sizeof(req), &resp, sizeof(resp));
	if (ret)
		return ret;

	result    = le32_to_cpu(resp.result);
	resp_type = le32_to_cpu(resp.resp_type);
	data      = le32_to_cpu(resp.data);

	if (result == QSEOS_RESULT_FAILURE)
		return 0;
	if (result == QSEOS_RESULT_SUCCESS && resp_type == QSEOS_APP_ID)
		return data;

	dev_warn(v->dev, "app_lookup unexpected result=0x%x type=0x%x\n",
		 result, resp_type);
	return -EINVAL;
}

static int app_region_notify(struct vkm_ctx *v, u32 addr, u32 size)
{
	struct region_notif_req req = {0};
	struct scm_resp resp = {0};
	u32 result;
	int ret;

	req.qsee_cmd_id = cpu_to_le32(QSEOS_APP_REGION_NOTIFICATION);
	req.addr = cpu_to_le32(addr);
	req.size = cpu_to_le32(size);

	ret = legacy_call(v, SCM_SVC_TZSCHEDULER, SCM_TZ_CMD_DISPATCH,
			  &req, sizeof(req), &resp, sizeof(resp));
	if (ret) {
		dev_err(v->dev, "region_notify legacy_call failed: %d\n", ret);
		return ret;
	}

	result = le32_to_cpu(resp.result);
	if (result == QSEOS_RESULT_SUCCESS)
		return 0;

	dev_info(v->dev, "region_notify result=0x%x\n",
		 result);
	return 0;
}

static int load_image_segments(void *dst, const struct fw_segment *segs,
			       unsigned int n_segs)
{
	unsigned int i;

	for (i = 0; i < n_segs; i++) {
		void *buf = NULL;
		size_t fsize = 0;
		ssize_t got;

		got = kernel_read_file_from_path(segs[i].path, 0, &buf,
						 SIZE_MAX, &fsize,
						 READING_FIRMWARE);
		if (got < 0)
			return (int)got;
		if (fsize != segs[i].expected_size)
			pr_warn("vector-keymaster: %s got %zu, expected %zu\n",
				segs[i].path, fsize, segs[i].expected_size);
		memcpy(dst + segs[i].offset, buf, fsize);
		vfree(buf);
	}
	return 0;
}

static int load_cmnlib(struct vkm_ctx *v)
{
	void *buf;
	dma_addr_t phys;
	struct load_serv_image_req req = {0};
	struct scm_resp resp = {0};
	u32 result;
	int ret;

	buf = kzalloc(CMNLIB_ALLOC_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = load_image_segments(buf, cmnlib_segments,
				  ARRAY_SIZE(cmnlib_segments));
	if (ret) {
		kfree(buf);
		return ret;
	}

	phys = dma_map_single(v->scm_dev, buf, CMNLIB_ALLOC_SIZE,
			      DMA_BIDIRECTIONAL);
	if (dma_mapping_error(v->scm_dev, phys)) {
		kfree(buf);
		return -ENOMEM;
	}

	req.qsee_cmd_id = cpu_to_le32(QSEOS_LOAD_SERV_IMAGE_COMMAND);
	req.mdt_len  = cpu_to_le32(CMNLIB_MDT_SIZE);
	req.img_len  = cpu_to_le32(CMNLIB_TOTAL_SIZE);
	req.phy_addr = cpu_to_le32((u32)phys);

	ret = legacy_call(v, SCM_SVC_TZSCHEDULER, SCM_TZ_CMD_DISPATCH,
			  &req, sizeof(req), &resp, sizeof(resp));
	dma_unmap_single(v->scm_dev, phys, CMNLIB_ALLOC_SIZE, DMA_BIDIRECTIONAL);
	kfree(buf);

	if (ret) {
		dev_err(v->dev, "cmnlib legacy_call failed: %d\n", ret);
		return ret;
	}

	result = le32_to_cpu(resp.result);
	if (result != QSEOS_RESULT_SUCCESS)
		dev_info(v->dev, "cmnlib result=0x%x (already loaded, benign)\n",
			 result);
	return 0;
}

static int app_start(struct vkm_ctx *v, dma_addr_t ta_phys, u32 mdt_len,
		     u32 img_len)
{
	struct load_app_req req = {0};
	struct scm_resp resp = {0};
	u32 result, resp_type, data;
	int ret;

	req.qsee_cmd_id = cpu_to_le32(QSEOS_APP_START_COMMAND);
	req.mdt_len  = cpu_to_le32(mdt_len);
	req.img_len  = cpu_to_le32(img_len);
	req.phy_addr = cpu_to_le32((u32)ta_phys);
	strscpy(req.app_name, "keymaste", sizeof(req.app_name));

	ret = legacy_call(v, SCM_SVC_TZSCHEDULER, SCM_TZ_CMD_DISPATCH,
			  &req, sizeof(req), &resp, sizeof(resp));
	if (ret) {
		dev_err(v->dev, "app_start legacy_call failed: %d\n", ret);
		return ret;
	}

	result    = le32_to_cpu(resp.result);
	resp_type = le32_to_cpu(resp.resp_type);
	data      = le32_to_cpu(resp.data);

	if (result == QSEOS_RESULT_SUCCESS && resp_type == QSEOS_APP_ID)
		return data;

	dev_err(v->dev, "app_start result=0x%x type=0x%x data=%u\n",
		result, resp_type, data);
	return -EIO;
}

static int load_keymaste(struct vkm_ctx *v)
{
	void *ta_buf;
	dma_addr_t ta_phys;
	int ret, app_id;

	ta_buf = kzalloc(KM_ALLOC_SIZE, GFP_KERNEL);
	if (!ta_buf)
		return -ENOMEM;

	ret = load_image_segments(ta_buf, km_segments, ARRAY_SIZE(km_segments));
	if (ret) {
		kfree(ta_buf);
		return ret;
	}

	ta_phys = dma_map_single(v->scm_dev, ta_buf, KM_ALLOC_SIZE,
				 DMA_BIDIRECTIONAL);
	if (dma_mapping_error(v->scm_dev, ta_phys)) {
		kfree(ta_buf);
		return -ENOMEM;
	}

	app_id = app_start(v, ta_phys, KM_MDT_SIZE, KM_TOTAL_SIZE);
	dma_unmap_single(v->scm_dev, ta_phys, KM_ALLOC_SIZE, DMA_BIDIRECTIONAL);
	kfree(ta_buf);

	if (app_id < 0)
		return app_id;

	v->app_id = app_id;
	dev_info(v->dev, "keymaste loaded, app_id=%u\n", v->app_id);
	return 0;
}

static int enable_ce_clocks_once(struct vkm_ctx *v)
{
	int ret;

	if (v->clocks_enabled)
		return 0;

	ret = clk_prepare_enable(v->clk_ce_core);
	if (ret) {
		dev_err(v->dev, "enable core clk failed %d\n", ret);
		return ret;
	}
	ret = clk_prepare_enable(v->clk_ce_iface);
	if (ret) {
		dev_err(v->dev, "enable iface clk failed %d\n", ret);
		goto off_core;
	}
	ret = clk_prepare_enable(v->clk_ce_bus);
	if (ret) {
		dev_err(v->dev, "enable bus clk failed %d\n", ret);
		goto off_iface;
	}

	v->clocks_enabled = true;
	return 0;

off_iface:
	clk_disable_unprepare(v->clk_ce_iface);
off_core:
	clk_disable_unprepare(v->clk_ce_core);
	return ret;
}

static void disable_ce_clocks(struct vkm_ctx *v)
{
	if (!v->clocks_enabled)
		return;
	clk_disable_unprepare(v->clk_ce_bus);
	clk_disable_unprepare(v->clk_ce_iface);
	clk_disable_unprepare(v->clk_ce_core);
	v->clocks_enabled = false;
}

static int ensure_ta_loaded(struct vkm_ctx *v)
{
	int ret, app_id;
	u32 qsee_ver, whitelist_ver;

	if (v->ta_loaded)
		return 0;

	ret = enable_ce_clocks_once(v);
	if (ret)
		return ret;

	qsee_ver = query_feature(v, FEATURE_QSEE_VERSION);
	whitelist_ver = query_feature(v, FEATURE_ID_WHITELIST);
	v->whitelist_supported = whitelist_ver >= WHITELIST_MIN_VERSION;
	dev_info(v->dev, "QSEE version=0x%x whitelist=0x%x (%s)\n",
		 qsee_ver, whitelist_ver,
		 v->whitelist_supported ? "supported" : "absent");

	// if keymaste is already there, that's fine
	app_id = app_lookup(v, "keymaste");
	if (app_id < 0)
		return app_id;
	if (app_id > 0) {
		v->app_id = app_id;
		v->ta_loaded = true;
		dev_info(v->dev, "keymaste already loaded, app_id=%u\n",
			 v->app_id);
		return 0;
	}

	ret = app_region_notify(v, SECAPP_REGION_ADDR, SECAPP_REGION_SIZE);
	if (ret)
		return ret;

	ret = load_cmnlib(v);
	if (ret)
		return ret;

	ret = load_keymaste(v);
	if (ret)
		return ret;

	v->ta_loaded = true;
	return 0;
}

static int send_data(struct vkm_ctx *v, dma_addr_t req_phys, u32 req_len,
		     dma_addr_t rsp_phys, u32 rsp_len,
		     dma_addr_t sglist_phys, u32 sglist_len)
{
	struct send_data_req req = {0};
	struct scm_resp resp = {0};
	u32 cmd_id = v->whitelist_supported
		? QSEOS_CLIENT_SEND_DATA_COMMAND_WHITELIST
		: QSEOS_CLIENT_SEND_DATA_COMMAND;
	u32 result;
	int ret;

	req.qsee_cmd_id = cpu_to_le32(cmd_id);
	req.app_id   = cpu_to_le32(v->app_id);
	req.req_ptr  = cpu_to_le32((u32)req_phys);
	req.req_len  = cpu_to_le32(req_len);
	req.rsp_ptr  = cpu_to_le32((u32)rsp_phys);
	req.rsp_len  = cpu_to_le32(rsp_len);
	req.sglistinfo_ptr = cpu_to_le32((u32)sglist_phys);
	req.sglistinfo_len = cpu_to_le32(sglist_len);

	ret = legacy_call(v, SCM_SVC_TZSCHEDULER, SCM_TZ_CMD_DISPATCH,
			  &req, sizeof(req), &resp, sizeof(resp));
	if (ret) {
		dev_err(v->dev, "send_data legacy_call failed: %d\n", ret);
		return ret;
	}

	result = le32_to_cpu(resp.result);
	if (result != QSEOS_RESULT_SUCCESS) {
		dev_err(v->dev, "send_data TZ result=0x%x\n", result);
		return -EIO;
	}
	return 0;
}

static int do_gen_keypair(struct vkm_ctx *v, u8 *blob_out, size_t blob_out_max,
			  size_t *blob_len_out)
{
	struct mapped_buf cmd_buf = {0}, rsp_buf = {0}, sgl_buf = {0};
	struct km_gen_keypair_cmd *cmd;
	struct km_gen_keypair_resp *rsp;
	int ret;

	ret = mbuf_alloc(v->scm_dev, &cmd_buf, QSEECOM_ALIGN(sizeof(*cmd)));
	if (ret)
		goto err;
	ret = mbuf_alloc(v->scm_dev, &rsp_buf, QSEECOM_ALIGN(sizeof(*rsp)));
	if (ret)
		goto err;
	ret = mbuf_alloc(v->scm_dev, &sgl_buf, SGLISTINFO_TABLE_SIZE);
	if (ret)
		goto err;

	cmd = cmd_buf.virt;
	rsp = rsp_buf.virt;

	cmd->cmd_id   = KEYMASTER_GENERATE_KEYPAIR;
	cmd->key_type = TYPE_RSA;
	cmd->rsa_params.modulus_size = 1024;
	cmd->rsa_params.public_exponent = 3;

	// it seems TA reads these as max-size inputs
	rsp->status = KEYMASTER_FAILURE;
	rsp->key_blob_len = sizeof(struct qcom_km_key_blob);

	ret = send_data(v, cmd_buf.phys, cmd_buf.size,
			rsp_buf.phys, rsp_buf.size,
			sgl_buf.phys, SGLISTINFO_TABLE_SIZE);
	if (ret)
		goto err;

	if (rsp->status != 0) {
		dev_err(v->dev, "gen_keypair TA status=%d\n", rsp->status);
		ret = -EIO;
		goto err;
	}
	if (rsp->key_blob_len == 0 || rsp->key_blob_len > blob_out_max) {
		dev_err(v->dev, "gen_keypair bad key_blob_len=%u\n",
			rsp->key_blob_len);
		ret = -EINVAL;
		goto err;
	}

	memcpy(blob_out, &rsp->key_blob, rsp->key_blob_len);
	*blob_len_out = rsp->key_blob_len;
	ret = 0;

err:
	mbuf_free(v->scm_dev, &cmd_buf);
	mbuf_free(v->scm_dev, &rsp_buf);
	mbuf_free(v->scm_dev, &sgl_buf);
	return ret;
}

static int do_sign(struct vkm_ctx *v,
		   const u8 *keyblob_bytes, size_t keyblob_len,
		   const u8 *data, size_t data_len,
		   u8 *sig_out, size_t sig_out_max, size_t *sig_len_out)
{
	struct mapped_buf cmd_buf = {0}, rsp_buf = {0}, dat_buf = {0}, sgl_buf = {0};
	struct km_sign_data_cmd *cmd;
	struct km_sign_data_resp *rsp;
	struct sglist_info *sgl;
	int ret;
	u32 sig_len;

	if (keyblob_len != sizeof(struct qcom_km_key_blob))
		return -EINVAL;
	if (data_len == 0 || data_len > VKM_MAX_DATA_LEN)
		return -EINVAL;

	ret = mbuf_alloc(v->scm_dev, &cmd_buf, QSEECOM_ALIGN(sizeof(*cmd)));
	if (ret)
		goto err;
	ret = mbuf_alloc(v->scm_dev, &rsp_buf, QSEECOM_ALIGN(sizeof(*rsp)));
	if (ret)
		goto err;
	// downstream uses 4096 ION page
	ret = mbuf_alloc(v->scm_dev, &dat_buf, 4096);
	if (ret)
		goto err;
	ret = mbuf_alloc(v->scm_dev, &sgl_buf, SGLISTINFO_TABLE_SIZE);
	if (ret)
		goto err;

	cmd = cmd_buf.virt;
	rsp = rsp_buf.virt;
	sgl = sgl_buf.virt;

	cmd->cmd_id = KEYMASTER_SIGN_DATA;
	cmd->sign_param.digest_type  = DIGEST_NONE;
	cmd->sign_param.padding_type = PADDING_NONE;
	memcpy(&cmd->key_blob, keyblob_bytes, keyblob_len);
	cmd->data = (u32)dat_buf.phys;
	cmd->dlen = data_len;

	rsp->status  = KEYMASTER_FAILURE;
	rsp->sig_len = KM_KEY_SIZE_MAX;

	sgl[0].indexAndFlags = cpu_to_le32(SGLISTINFO_FLAG_SINGLE_PTR |
		(u32)offsetof(struct km_sign_data_cmd, data));
	sgl[0].sizeOrCount = cpu_to_le32(dat_buf.size);

	memcpy(dat_buf.virt, data, data_len);

	ret = send_data(v, cmd_buf.phys, cmd_buf.size,
			rsp_buf.phys, rsp_buf.size,
			sgl_buf.phys, SGLISTINFO_TABLE_SIZE);
	if (ret)
		goto err;

	if (rsp->status != 0) {
		dev_err(v->dev, "sign TA status=%d\n", rsp->status);
		ret = -EIO;
		goto err;
	}

	sig_len = rsp->sig_len;
	if (sig_len > sig_out_max)
		sig_len = sig_out_max;
	memcpy(sig_out, rsp->signed_data, sig_len);
	*sig_len_out = sig_len;
	ret = 0;

err:
	mbuf_free(v->scm_dev, &cmd_buf);
	mbuf_free(v->scm_dev, &rsp_buf);
	mbuf_free(v->scm_dev, &dat_buf);
	mbuf_free(v->scm_dev, &sgl_buf);
	return ret;
}

static struct vkm_ctx *the_ctx;

static int vkm_open(struct inode *inode, struct file *filp)
{
	filp->private_data = the_ctx;
	return 0;
}

static int vkm_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static long vkm_ioctl_gen_keyblob(struct vkm_ctx *v, void __user *uarg)
{
	struct vkm_gen_keyblob *gk;
	size_t blob_len = 0;
	long ret;

	gk = kzalloc(sizeof(*gk), GFP_KERNEL);
	if (!gk)
		return -ENOMEM;

	ret = ensure_ta_loaded(v);
	if (ret)
		goto out;

	ret = do_gen_keypair(v, gk->keyblob, sizeof(gk->keyblob), &blob_len);
	if (ret)
		goto out;

	gk->length = (u32)blob_len;
	if (copy_to_user(uarg, gk, sizeof(*gk)))
		ret = -EFAULT;

out:
	kfree_sensitive(gk);
	return ret;
}

static long vkm_ioctl_sign(struct vkm_ctx *v, void __user *uarg)
{
	struct vkm_sign *s;
	size_t sig_len = 0;
	long ret;

	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;

	if (copy_from_user(s, uarg, sizeof(*s))) {
		ret = -EFAULT;
		goto out;
	}
	if (s->keyblob_len != VKM_KEYBLOB_LEN ||
	    s->data_len == 0 || s->data_len > VKM_MAX_DATA_LEN) {
		ret = -EINVAL;
		goto out;
	}

	ret = ensure_ta_loaded(v);
	if (ret)
		goto out;

	ret = do_sign(v, s->keyblob, s->keyblob_len,
		      s->data, s->data_len,
		      s->sig, sizeof(s->sig), &sig_len);
	if (ret)
		goto out;

	s->sig_len = (u32)sig_len;
	if (copy_to_user(uarg, s, sizeof(*s)))
		ret = -EFAULT;

out:
	kfree_sensitive(s);
	return ret;
}

static long vkm_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct vkm_ctx *v = filp->private_data;
	void __user *uarg = (void __user *)arg;
	long ret;

	mutex_lock(&v->lock);
	switch (cmd) {
	case VKM_IOC_GEN_KEYBLOB:
		ret = vkm_ioctl_gen_keyblob(v, uarg);
		break;
	case VKM_IOC_SIGN:
		ret = vkm_ioctl_sign(v, uarg);
		break;
	default:
		ret = -ENOTTY;
	}
	mutex_unlock(&v->lock);
	return ret;
}

static const struct file_operations vkm_fops = {
	.owner		= THIS_MODULE,
	.open		= vkm_open,
	.release	= vkm_release,
	.unlocked_ioctl	= vkm_ioctl,
};

static struct device *find_scm_device(void)
{
	struct device_node *np;
	struct platform_device *pdev;

	np = of_find_compatible_node(NULL, NULL, "qcom,scm");
	if (!np)
		return NULL;
	pdev = of_find_device_by_node(np);
	of_node_put(np);
	return pdev ? &pdev->dev : NULL;
}

static int vkm_probe(struct platform_device *pdev)
{
	struct vkm_ctx *v;
	struct device *scm_dev;
	int ret;

	scm_dev = find_scm_device();
	if (!scm_dev)
		return -EPROBE_DEFER;

	v = devm_kzalloc(&pdev->dev, sizeof(*v), GFP_KERNEL);
	if (!v)
		return -ENOMEM;

	v->dev = &pdev->dev;
	v->scm_dev = scm_dev;
	mutex_init(&v->lock);

	v->clk_ce_core  = of_clk_get_by_name(scm_dev->of_node, "core");
	v->clk_ce_iface = of_clk_get_by_name(scm_dev->of_node, "iface");
	v->clk_ce_bus   = of_clk_get_by_name(scm_dev->of_node, "bus");
	if (IS_ERR(v->clk_ce_core) || IS_ERR(v->clk_ce_iface) ||
	    IS_ERR(v->clk_ce_bus)) {
		dev_err(&pdev->dev, "CE clocks missing\n");
		ret = -ENODEV;
		goto err_clk;
	}

	v->miscdev.minor = MISC_DYNAMIC_MINOR;
	v->miscdev.name  = "vector-km";
	v->miscdev.fops  = &vkm_fops;
	v->miscdev.mode  = 0600;
	ret = misc_register(&v->miscdev);
	if (ret) {
		dev_err(&pdev->dev, "misc_register failed: %d\n", ret);
		goto err_clk;
	}

	platform_set_drvdata(pdev, v);
	the_ctx = v;
	dev_info(&pdev->dev, "keymaster thing loaded\n");
	return 0;

err_clk:
	if (!IS_ERR_OR_NULL(v->clk_ce_core))
		clk_put(v->clk_ce_core);
	if (!IS_ERR_OR_NULL(v->clk_ce_iface))
		clk_put(v->clk_ce_iface);
	if (!IS_ERR_OR_NULL(v->clk_ce_bus))
		clk_put(v->clk_ce_bus);
	return ret;
}

static void vkm_remove(struct platform_device *pdev)
{
	struct vkm_ctx *v = platform_get_drvdata(pdev);

	misc_deregister(&v->miscdev);
	disable_ce_clocks(v);
	clk_put(v->clk_ce_bus);
	clk_put(v->clk_ce_iface);
	clk_put(v->clk_ce_core);
	the_ctx = NULL;
	// leave the firmware itself actually loaded
}

static const struct of_device_id vkm_of_match[] = {
	{ .compatible = "anki,vector-keymaster" },
	{},
};
MODULE_DEVICE_TABLE(of, vkm_of_match);

static struct platform_driver vkm_driver = {
	.probe  = vkm_probe,
	.remove = vkm_remove,
	.driver = {
		.name = "vector-keymaster",
		.of_match_table = vkm_of_match,
	},
};
module_platform_driver(vkm_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Anki Vector keymaster");
MODULE_AUTHOR("kercre123");
