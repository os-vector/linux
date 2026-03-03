// SPDX-License-Identifier: GPL-2.0
/*
 * camss-vfe-3-6.c
 *
 * Qualcomm MSM Camera Subsystem - VFE (Video Front End) Module v3.6
 *
 * Copyright (c) 2013-2017, The Linux Foundation. All rights reserved.
 * Copyright (C) 2015-2018 Linaro Ltd.
 * Copyright (C) 2026 Kerigan Crfeighton
 */

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>

#include "camss.h"
#include "camss-vfe.h"
#include "camss-vfe-gen1.h"
#include "camss-vfe-vbif.h"

#define VFE_0_HW_VERSION		0x000

#define VFE_0_GLOBAL_RESET_CMD		0x004
#define VFE_0_GLOBAL_RESET_CMD_ALL	0x3FF
#define VFE_0_CGC_OVERRIDE		0x00C
#define VFE_0_CGC_OVERRIDE_ALL		0x07FFFFFF
#define VFE_0_MODULE_CFG		0x010

#define VFE_0_IRQ_CMD			0x018
#define VFE_0_IRQ_CMD_GLOBAL_CLEAR	BIT(0)
#define VFE_0_IRQ_MASK_0		0x01C
#define VFE_0_IRQ_MASK_0_CAMIF_SOF		BIT(0)
#define VFE_0_IRQ_MASK_0_PIX_REG_UPDATE	BIT(5)
#define VFE_0_IRQ_MASK_0_IMAGE_MASTER_n_PING_PONG(n)	BIT((n) + 6)
#define VFE_0_IRQ_MASK_0_IMAGE_COMPOSITE_DONE_n(n)	BIT((n) + 21)

#define VFE_0_IRQ_MASK_1		0x020
#define VFE_0_IRQ_MASK_1_CAMIF_ERROR		BIT(0)
#define VFE_0_IRQ_MASK_1_VIOLATION		BIT(7)
#define VFE_0_IRQ_MASK_1_RDIn_SOF(n)		BIT((n) + 26)
#define VFE_0_IRQ_MASK_1_RESET_ACK		BIT(23)
#define VFE_0_IRQ_MASK_1_HALT_ACK		BIT(24)

#define VFE_0_IRQ_CLEAR_0		0x024
#define VFE_0_IRQ_CLEAR_1		0x028

#define VFE_0_IRQ_STATUS_0		0x02C
#define VFE_0_IRQ_STATUS_0_CAMIF_SOF		BIT(0)
#define VFE_0_IRQ_STATUS_0_PIX_REG_UPDATE	BIT(5)
#define VFE_0_IRQ_STATUS_0_IMAGE_MASTER_n_PING_PONG(n)	BIT((n) + 6)
#define VFE_0_IRQ_STATUS_0_IMAGE_COMPOSITE_DONE_n(n)	BIT((n) + 21)

#define VFE_0_IRQ_STATUS_1		0x030
#define VFE_0_IRQ_STATUS_1_CAMIF_ERROR		BIT(0)
#define VFE_0_IRQ_STATUS_1_VIOLATION		BIT(7)
#define VFE_0_IRQ_STATUS_1_RESET_ACK		BIT(23)
#define VFE_0_IRQ_STATUS_1_HALT_ACK		BIT(24)
#define VFE_0_IRQ_STATUS_1_RDIn_SOF(n)		BIT((n) + 26)

#define VFE_0_IRQ_COMPOSITE_MASK	0x034

#define VFE_0_BUS_RELOAD		0x038
#define VFE_0_BUS_CFG			0x03C
#define VFE_0_BUS_CFG_EN		0x00000009
#define VFE_0_BUS_XBAR_CFG(wm)		(0x040 + 0x4 * ((wm) / 4))
#define VFE_0_BUS_XBAR_SHIFT(wm)	(((wm) % 4) * 8)
#define VFE_0_BUS_XBAR_RDI0_VAL		0xA0
#define VFE_0_BUS_XBAR_RDI1_VAL		0xC0
#define VFE_0_BUS_XBAR_RDI2_VAL		0xE0

//0x00: cfg
//0x04: ping address
//0x08: pong address
//0x0C: UB config
//0x10: image size
//0x14: buffer config
#define VFE_0_BUS_WM_BASE(n)		(0x04C + 0x18 * (n))
#define VFE_0_BUS_WM_CFG(n)		(VFE_0_BUS_WM_BASE(n) + 0x00)
#define VFE_0_BUS_WM_PING(n)		(VFE_0_BUS_WM_BASE(n) + 0x04)
#define VFE_0_BUS_WM_PONG(n)		(VFE_0_BUS_WM_BASE(n) + 0x08)
#define VFE_0_BUS_WM_UB_CFG(n)		(VFE_0_BUS_WM_BASE(n) + 0x0C)
#define VFE_0_BUS_WM_IMAGE_SIZE(n)	(VFE_0_BUS_WM_BASE(n) + 0x10)
#define VFE_0_BUS_WM_BUFFER_CFG(n)	(VFE_0_BUS_WM_BASE(n) + 0x14)

#define VFE_0_BUS_PING_PONG_STATUS	0x180

#define VFE_0_AXI_HALT_CMD		0x1D8
#define VFE_0_AXI_HALT_STATUS		0x1DC

#define VFE_0_CAMIF_CMD			0x1E0
#define VFE_0_CAMIF_CMD_ENABLE		0x1
#define VFE_0_CAMIF_CMD_DISABLE		0x0
#define VFE_0_CAMIF_CMD_DISABLE_IMM	0x6

#define VFE_0_CAMIF_CFG			0x1E4
#define VFE_0_CAMIF_STATUS		0x204
#define VFE_0_CAMIF_STATUS_HALT		BIT(31)

#define VFE_0_REG_UPDATE		0x260
#define VFE_0_REG_UPDATE_RDIn(n)	BIT(1 + (n))
#define VFE_0_REG_UPDATE_line_n(n)	\
		((n) == VFE_LINE_PIX ? 1 : VFE_0_REG_UPDATE_RDIn(n))

#define VFE_0_VIOLATION_STATUS		0x7B4

#define VFE_0_RDI_CFG(n)	((n) == 0 ? 0x6FC : (0x734 + 0x4 * ((n) - 1)))

#define VFE_0_RDI_CFG_STREAM_SEL_SHIFT 28
#define VFE_0_RDI_CFG_CID_SHIFT	4
#define VFE_0_RDI_CFG_ENABLE BIT(2)
#define VFE_0_RDI_CFG_FRAME_BASED(n) BIT(16 + (n))

#define CAMIF_TIMEOUT_SLEEP_US 1000
#define CAMIF_TIMEOUT_ALL_US 1000000

#define MSM_VFE_VFE0_UB_SIZE 255
#define MSM_VFE_VFE0_UB_SIZE_RDI (MSM_VFE_VFE0_UB_SIZE / 3)

static u16 vfe_get_ub_size(u8 vfe_id)
{
	if (vfe_id == 0)
		return MSM_VFE_VFE0_UB_SIZE_RDI;

	return 0;
}

static inline void vfe_reg_clr(struct vfe_device *vfe, u32 reg, u32 clr_bits)
{
	u32 bits = readl_relaxed(vfe->base + reg);

	writel_relaxed(bits & ~clr_bits, vfe->base + reg);
}

static inline void vfe_reg_set(struct vfe_device *vfe, u32 reg, u32 set_bits)
{
	u32 bits = readl_relaxed(vfe->base + reg);

	writel_relaxed(bits | set_bits, vfe->base + reg);
}

//global reset is at 0x004, value 0x3FF
static void vfe_global_reset(struct vfe_device *vfe)
{
	/* Enable RESET_ACK and HALT_ACK in IRQ Mask 1 */
	writel_relaxed(VFE_0_IRQ_MASK_1_RESET_ACK | VFE_0_IRQ_MASK_1_HALT_ACK,
		       vfe->base + VFE_0_IRQ_MASK_0);
	writel_relaxed(VFE_0_IRQ_MASK_1_RESET_ACK | VFE_0_IRQ_MASK_1_HALT_ACK,
		       vfe->base + VFE_0_IRQ_MASK_1);

	/* Clear any stale IRQ status */
	writel_relaxed(0xFFFFFFFF, vfe->base + VFE_0_IRQ_CLEAR_0);
	writel_relaxed(0xFFFFFFFF, vfe->base + VFE_0_IRQ_CLEAR_1);
	wmb();
	writel_relaxed(VFE_0_IRQ_CMD_GLOBAL_CLEAR, vfe->base + VFE_0_IRQ_CMD);
	wmb();

	/* Trigger global reset of all modules */
	writel_relaxed(VFE_0_GLOBAL_RESET_CMD_ALL,
		       vfe->base + VFE_0_GLOBAL_RESET_CMD);
}

static void vfe_halt_request(struct vfe_device *vfe)
{
	writel_relaxed(1, vfe->base + VFE_0_AXI_HALT_CMD);
}

static void vfe_halt_clear(struct vfe_device *vfe)
{
	writel_relaxed(0, vfe->base + VFE_0_AXI_HALT_CMD);
}

static void vfe_wm_enable(struct vfe_device *vfe, u8 wm, u8 enable)
{
	if (enable)
		vfe_reg_set(vfe, VFE_0_BUS_WM_CFG(wm), BIT(0));
	else
		vfe_reg_clr(vfe, VFE_0_BUS_WM_CFG(wm), BIT(0));
}

static void vfe_wm_frame_based(struct vfe_device *vfe, u8 wm, u8 enable)
{
	if (enable)
		vfe_reg_set(vfe, VFE_0_BUS_WM_CFG(wm), BIT(1));
	else
		vfe_reg_clr(vfe, VFE_0_BUS_WM_CFG(wm), BIT(1));
}

static void vfe_get_wm_sizes(struct v4l2_pix_format_mplane *pix, u8 plane,
			     u16 *width, u16 *height, u16 *bytesperline)
{
	*width = pix->width;
	*height = pix->height;
	*bytesperline = pix->plane_fmt[0].bytesperline;

	if (pix->pixelformat == V4L2_PIX_FMT_NV12 ||
	    pix->pixelformat == V4L2_PIX_FMT_NV21)
		if (plane == 1)
			*height /= 2;
}

static void vfe_wm_line_based(struct vfe_device *vfe, u32 wm,
			      struct v4l2_pix_format_mplane *pix,
			      u8 plane, u32 enable)
{
	u32 reg;

	if (enable) {
		u16 width = 0, height = 0, bytesperline = 0, wpl;

		vfe_get_wm_sizes(pix, plane, &width, &height, &bytesperline);

		wpl = vfe_word_per_line(pix->pixelformat, width);

		//(wpl-1)/2 << 16 | (height-1)
		reg = height - 1;
		reg |= ((wpl + 1) / 2 - 1) << 16;
		writel_relaxed(reg, vfe->base + VFE_0_BUS_WM_IMAGE_SIZE(wm));

		l = (bytesperline + 7) / 8;

		reg = 0x3;
		reg |= (height - 1) << 4;
		reg |= wpl << 16;
		writel_relaxed(reg, vfe->base + VFE_0_BUS_WM_BUFFER_CFG(wm));
	} else {
		writel_relaxed(0, vfe->base + VFE_0_BUS_WM_IMAGE_SIZE(wm));
		writel_relaxed(0, vfe->base + VFE_0_BUS_WM_BUFFER_CFG(wm));
	}
}

static void vfe_wm_set_framedrop_period(struct vfe_device *vfe, u8 wm, u8 per)
{
	//
}

static void vfe_wm_set_framedrop_pattern(struct vfe_device *vfe, u8 wm,
					 u32 pattern)
{
	// this seems to have helped with some memory problems
	vfe_wm_enable(vfe, wm, pattern != 0);
}

static void vfe_wm_set_ub_cfg(struct vfe_device *vfe, u8 wm,
			      u16 offset, u16 depth)
{
	u32 reg;
	reg = ((u32)offset << 16) | (depth - 1);
	writel_relaxed(reg, vfe->base + VFE_0_BUS_WM_UB_CFG(wm));
}

static void vfe_bus_reload_wm(struct vfe_device *vfe, u8 wm)
{
	wmb();
	writel_relaxed(BIT(wm), vfe->base + VFE_0_BUS_RELOAD);
	wmb();
}

static void vfe_wm_set_ping_addr(struct vfe_device *vfe, u8 wm, u32 addr)
{
	writel_relaxed(addr, vfe->base + VFE_0_BUS_WM_PING(wm));
}

static void vfe_wm_set_pong_addr(struct vfe_device *vfe, u8 wm, u32 addr)
{
	writel_relaxed(addr, vfe->base + VFE_0_BUS_WM_PONG(wm));
}

static int vfe_wm_get_ping_pong_status(struct vfe_device *vfe, u8 wm)
{
	u32 reg;

	reg = readl_relaxed(vfe->base + VFE_0_BUS_PING_PONG_STATUS);

	return (reg >> wm) & 0x1;
}

static void vfe_bus_enable_wr_if(struct vfe_device *vfe, u8 enable)
{
	if (enable)
		writel_relaxed(VFE_0_BUS_CFG_EN, vfe->base + VFE_0_BUS_CFG);
	else
		writel_relaxed(0, vfe->base + VFE_0_BUS_CFG);
}

static void vfe_bus_connect_wm_to_rdi(struct vfe_device *vfe, u8 wm,
				      enum vfe_line_id id)
{
	u32 xbar_val;
	u32 reg;
	u32 rdi_cfg;

	rdi_cfg = readl_relaxed(vfe->base + VFE_0_RDI_CFG(0));
	rdi_cfg |= VFE_0_RDI_CFG_FRAME_BASED(id);
	writel_relaxed(rdi_cfg, vfe->base + VFE_0_RDI_CFG(0));

	rdi_cfg = readl_relaxed(vfe->base + VFE_0_RDI_CFG(id));
	rdi_cfg &= 0x00070003;
	rdi_cfg |= ((u32)(id * 3) << VFE_0_RDI_CFG_STREAM_SEL_SHIFT) |
		   VFE_0_RDI_CFG_ENABLE;
	writel_relaxed(rdi_cfg, vfe->base + VFE_0_RDI_CFG(id));

	switch (id) {
	case VFE_LINE_RDI0:
	default:
		xbar_val = VFE_0_BUS_XBAR_RDI0_VAL;
		break;
	case VFE_LINE_RDI1:
		xbar_val = VFE_0_BUS_XBAR_RDI1_VAL;
		break;
	case VFE_LINE_RDI2:
		xbar_val = VFE_0_BUS_XBAR_RDI2_VAL;
		break;
	}

	reg = readl_relaxed(vfe->base + VFE_0_BUS_XBAR_CFG(wm));
	reg &= ~(0xFF << VFE_0_BUS_XBAR_SHIFT(wm));
	reg |= xbar_val << VFE_0_BUS_XBAR_SHIFT(wm);
	writel_relaxed(reg, vfe->base + VFE_0_BUS_XBAR_CFG(wm));
}

static void vfe_wm_set_subsample(struct vfe_device *vfe, u8 wm)
{
	// investigate
}

static void vfe_bus_disconnect_wm_from_rdi(struct vfe_device *vfe, u8 wm,
					   enum vfe_line_id id)
{
	u32 reg;

	vfe_reg_clr(vfe, VFE_0_RDI_CFG(0), VFE_0_RDI_CFG_FRAME_BASED(id));

	vfe_reg_clr(vfe, VFE_0_RDI_CFG(id), VFE_0_RDI_CFG_ENABLE);

	reg = readl_relaxed(vfe->base + VFE_0_BUS_XBAR_CFG(wm));
	reg &= ~(0xFF << VFE_0_BUS_XBAR_SHIFT(wm));
	writel_relaxed(reg, vfe->base + VFE_0_BUS_XBAR_CFG(wm));
}

static void vfe_set_xbar_cfg(struct vfe_device *vfe, struct vfe_output *output,
			     u8 enable)
{
	// not in 3.6
}

static void vfe_set_realign_cfg(struct vfe_device *vfe, struct vfe_line *line,
				u8 enable)
{
	//
}

static void vfe_set_rdi_cid(struct vfe_device *vfe, enum vfe_line_id id, u8 cid)
{
	u32 reg;

	reg = readl_relaxed(vfe->base + VFE_0_RDI_CFG(id));
	reg &= ~(0xF << VFE_0_RDI_CFG_CID_SHIFT);
	reg |= (u32)cid << VFE_0_RDI_CFG_CID_SHIFT;
	writel_relaxed(reg, vfe->base + VFE_0_RDI_CFG(id));
}

static void vfe_reg_update(struct vfe_device *vfe, enum vfe_line_id line_id)
{
	vfe->reg_update |= VFE_0_REG_UPDATE_line_n(line_id);
	wmb();
	writel_relaxed(vfe->reg_update, vfe->base + VFE_0_REG_UPDATE);
	wmb();
}

static inline void vfe_reg_update_clear(struct vfe_device *vfe,
					enum vfe_line_id line_id)
{
	vfe->reg_update &= ~VFE_0_REG_UPDATE_line_n(line_id);
}

static void vfe_enable_irq_wm_line(struct vfe_device *vfe, u8 wm,
				   enum vfe_line_id line_id, u8 enable)
{
	u32 irq_en0 = VFE_0_IRQ_MASK_0_IMAGE_MASTER_n_PING_PONG(wm);
	u32 irq_en1;

	if (line_id == VFE_LINE_PIX)
		irq_en1 = 0;
	else
		irq_en1 = VFE_0_IRQ_MASK_1_RDIn_SOF(line_id);

	if (enable) {
		vfe_reg_set(vfe, VFE_0_IRQ_MASK_0, irq_en0);
		if (irq_en1)
			vfe_reg_set(vfe, VFE_0_IRQ_MASK_1, irq_en1);
	} else {
		vfe_reg_clr(vfe, VFE_0_IRQ_MASK_0, irq_en0);
		if (irq_en1)
			vfe_reg_clr(vfe, VFE_0_IRQ_MASK_1, irq_en1);
	}
}

static void vfe_enable_irq_pix_line(struct vfe_device *vfe, u8 comp,
				    enum vfe_line_id line_id, u8 enable)
{
	struct vfe_output *output = &vfe->line[line_id].output;
	unsigned int i;
	u32 irq_en0;
	u32 irq_en1;
	u32 comp_mask = 0;

	irq_en0 = VFE_0_IRQ_MASK_0_CAMIF_SOF;
	irq_en0 |= VFE_0_IRQ_MASK_0_IMAGE_COMPOSITE_DONE_n(comp);
	irq_en0 |= VFE_0_IRQ_MASK_0_PIX_REG_UPDATE;
	irq_en1 = VFE_0_IRQ_MASK_1_CAMIF_ERROR;
	for (i = 0; i < output->wm_num; i++)
		comp_mask |= (1 << output->wm_idx[i]) << (comp * 8);

	if (enable) {
		vfe_reg_set(vfe, VFE_0_IRQ_MASK_0, irq_en0);
		vfe_reg_set(vfe, VFE_0_IRQ_MASK_1, irq_en1);
		vfe_reg_set(vfe, VFE_0_IRQ_COMPOSITE_MASK, comp_mask);
	} else {
		vfe_reg_clr(vfe, VFE_0_IRQ_MASK_0, irq_en0);
		vfe_reg_clr(vfe, VFE_0_IRQ_MASK_1, irq_en1);
		vfe_reg_clr(vfe, VFE_0_IRQ_COMPOSITE_MASK, comp_mask);
	}
}

static void vfe_enable_irq_common(struct vfe_device *vfe)
{
	/* Enable reset_ack, halt_ack, violation */
	vfe_reg_set(vfe, VFE_0_IRQ_MASK_1,
		    VFE_0_IRQ_MASK_1_RESET_ACK |
		    VFE_0_IRQ_MASK_1_HALT_ACK |
		    VFE_0_IRQ_MASK_1_VIOLATION);
}

// 3.6 is RDI only

static void vfe_set_demux_cfg(struct vfe_device *vfe, struct vfe_line *line)
{

}

static void vfe_set_scale_cfg(struct vfe_device *vfe, struct vfe_line *line)
{

}

static void vfe_set_crop_cfg(struct vfe_device *vfe, struct vfe_line *line)
{

}

static void vfe_set_clamp_cfg(struct vfe_device *vfe)
{

}

static void vfe_set_qos(struct vfe_device *vfe)
{
	if (vfe->res->has_vbif) {
		int ret;

		ret = vfe_vbif_apply_settings(vfe);
		if (ret < 0)
			dev_err_ratelimited(vfe->camss->dev,
					    "VFE: VBIF error %d\n", ret);
	}
}

static void vfe_set_ds(struct vfe_device *vfe)
{

}

static void vfe_set_cgc_override(struct vfe_device *vfe, u8 wm, u8 enable)
{
	// this is global, set at init
}

static void vfe_set_camif_cfg(struct vfe_device *vfe, struct vfe_line *line)
{

}

static void vfe_set_camif_cmd(struct vfe_device *vfe, u8 enable)
{
	if (enable)
		writel_relaxed(VFE_0_CAMIF_CMD_ENABLE,
			       vfe->base + VFE_0_CAMIF_CMD);
	else
		writel_relaxed(VFE_0_CAMIF_CMD_DISABLE,
			       vfe->base + VFE_0_CAMIF_CMD);
}

static void vfe_set_module_cfg(struct vfe_device *vfe, u8 enable)
{

}

static int vfe_camif_wait_for_stop(struct vfe_device *vfe, struct device *dev)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout(vfe->base + VFE_0_CAMIF_STATUS,
				 val,
				 (val & VFE_0_CAMIF_STATUS_HALT),
				 CAMIF_TIMEOUT_SLEEP_US,
				 CAMIF_TIMEOUT_ALL_US);
	if (ret < 0)
		dev_err(dev, "%s: camif stop timeout\n", __func__);

	return ret;
}

static void vfe_isr_read(struct vfe_device *vfe, u32 *value0, u32 *value1)
{
	*value0 = readl_relaxed(vfe->base + VFE_0_IRQ_STATUS_0);
	*value1 = readl_relaxed(vfe->base + VFE_0_IRQ_STATUS_1);

	/* Write status back to clear registers */
	writel_relaxed(*value0, vfe->base + VFE_0_IRQ_CLEAR_0);
	writel_relaxed(*value1, vfe->base + VFE_0_IRQ_CLEAR_1);

	wmb();
	writel_relaxed(VFE_0_IRQ_CMD_GLOBAL_CLEAR, vfe->base + VFE_0_IRQ_CMD);
}

static void vfe_violation_read(struct vfe_device *vfe)
{
	u32 violation = readl_relaxed(vfe->base + VFE_0_VIOLATION_STATUS);

	pr_err_ratelimited("VFE: violation = 0x%08x\n", violation);
}

static irqreturn_t vfe_isr(int irq, void *dev)
{
	struct vfe_device *vfe = dev;
	u32 value0, value1;
	int i, j;

	vfe->res->hw_ops->isr_read(vfe, &value0, &value1);

	dev_dbg(vfe->camss->dev, "VFE: status0 = 0x%08x, status1 = 0x%08x\n",
		value0, value1);

	if (value1 & VFE_0_IRQ_STATUS_1_RESET_ACK)
		vfe->isr_ops.reset_ack(vfe);

	if (value1 & VFE_0_IRQ_STATUS_1_VIOLATION)
		vfe->res->hw_ops->violation_read(vfe);

	if (value1 & VFE_0_IRQ_STATUS_1_HALT_ACK)
		vfe->isr_ops.halt_ack(vfe);

	for (i = VFE_LINE_RDI0; i <= VFE_LINE_RDI2; i++) {
		if (value1 & VFE_0_IRQ_STATUS_1_RDIn_SOF(i)) {
			vfe->isr_ops.sof(vfe, i);
			vfe->isr_ops.reg_update(vfe, i);
		}
	}

	if (value0 & VFE_0_IRQ_STATUS_0_PIX_REG_UPDATE)
		vfe->isr_ops.reg_update(vfe, VFE_LINE_PIX);

	if (value0 & VFE_0_IRQ_STATUS_0_CAMIF_SOF)
		vfe->isr_ops.sof(vfe, VFE_LINE_PIX);

	for (i = 0; i < MSM_VFE_COMPOSITE_IRQ_NUM; i++)
		if (value0 & VFE_0_IRQ_STATUS_0_IMAGE_COMPOSITE_DONE_n(i)) {
			vfe->isr_ops.comp_done(vfe, i);
			for (j = 0; j < ARRAY_SIZE(vfe->wm_output_map); j++)
				if (vfe->wm_output_map[j] == VFE_LINE_PIX)
					value0 &= ~VFE_0_IRQ_MASK_0_IMAGE_MASTER_n_PING_PONG(j);
		}

	for (i = 0; i < MSM_VFE_IMAGE_MASTERS_NUM; i++)
		if (value0 & VFE_0_IRQ_STATUS_0_IMAGE_MASTER_n_PING_PONG(i))
			vfe->isr_ops.wm_done(vfe, i);

	return IRQ_HANDLED;
}

static void vfe_3_6_pm_domain_off(struct vfe_device *vfe)
{
	if (!vfe->res->has_pd)
		return;

	vfe_pm_domain_off(vfe);
}

static int vfe_3_6_pm_domain_on(struct vfe_device *vfe)
{
	if (!vfe->res->has_pd)
		return 0;

	return vfe_pm_domain_on(vfe);
}

static const struct vfe_hw_ops_gen1 vfe_ops_gen1_3_6 = {
	.bus_connect_wm_to_rdi		= vfe_bus_connect_wm_to_rdi,
	.bus_disconnect_wm_from_rdi	= vfe_bus_disconnect_wm_from_rdi,
	.bus_enable_wr_if		= vfe_bus_enable_wr_if,
	.bus_reload_wm			= vfe_bus_reload_wm,
	.camif_wait_for_stop		= vfe_camif_wait_for_stop,
	.enable_irq_common		= vfe_enable_irq_common,
	.enable_irq_pix_line		= vfe_enable_irq_pix_line,
	.enable_irq_wm_line		= vfe_enable_irq_wm_line,
	.get_ub_size			= vfe_get_ub_size,
	.halt_clear			= vfe_halt_clear,
	.halt_request			= vfe_halt_request,
	.set_camif_cfg			= vfe_set_camif_cfg,
	.set_camif_cmd			= vfe_set_camif_cmd,
	.set_cgc_override		= vfe_set_cgc_override,
	.set_clamp_cfg			= vfe_set_clamp_cfg,
	.set_crop_cfg			= vfe_set_crop_cfg,
	.set_demux_cfg			= vfe_set_demux_cfg,
	.set_ds				= vfe_set_ds,
	.set_module_cfg			= vfe_set_module_cfg,
	.set_qos			= vfe_set_qos,
	.set_rdi_cid			= vfe_set_rdi_cid,
	.set_realign_cfg		= vfe_set_realign_cfg,
	.set_scale_cfg			= vfe_set_scale_cfg,
	.set_xbar_cfg			= vfe_set_xbar_cfg,
	.wm_enable			= vfe_wm_enable,
	.wm_frame_based			= vfe_wm_frame_based,
	.wm_get_ping_pong_status	= vfe_wm_get_ping_pong_status,
	.wm_line_based			= vfe_wm_line_based,
	.wm_set_framedrop_pattern	= vfe_wm_set_framedrop_pattern,
	.wm_set_framedrop_period	= vfe_wm_set_framedrop_period,
	.wm_set_ping_addr		= vfe_wm_set_ping_addr,
	.wm_set_pong_addr		= vfe_wm_set_pong_addr,
	.wm_set_subsample		= vfe_wm_set_subsample,
	.wm_set_ub_cfg			= vfe_wm_set_ub_cfg,
};

static void vfe_subdev_init(struct device *dev, struct vfe_device *vfe)
{
	vfe->isr_ops = vfe_isr_ops_gen1;
	vfe->ops_gen1 = &vfe_ops_gen1_3_6;
	vfe->video_ops = vfe_video_ops_gen1;
}

const struct vfe_hw_ops vfe_ops_3_6 = {
	.global_reset		= vfe_global_reset,
	.hw_version		= vfe_hw_version,
	.isr_read		= vfe_isr_read,
	.isr			= vfe_isr,
	.pm_domain_off		= vfe_3_6_pm_domain_off,
	.pm_domain_on		= vfe_3_6_pm_domain_on,
	.reg_update_clear	= vfe_reg_update_clear,
	.reg_update		= vfe_reg_update,
	.subdev_init		= vfe_subdev_init,
	.vfe_disable		= vfe_gen1_disable,
	.vfe_enable		= vfe_gen1_enable,
	.vfe_halt		= vfe_gen1_halt,
	.violation_read		= vfe_violation_read,
};
