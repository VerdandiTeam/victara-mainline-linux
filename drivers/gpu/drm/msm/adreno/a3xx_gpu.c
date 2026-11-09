// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2013 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * Copyright (c) 2014 The Linux Foundation. All rights reserved.
 */

#include "a3xx_gpu.h"

#define A3XX_INT0_MASK \
	(A3XX_INT0_RBBM_AHB_ERROR |        \
	 A3XX_INT0_RBBM_ATB_BUS_OVERFLOW | \
	 A3XX_INT0_CP_T0_PACKET_IN_IB |    \
	 A3XX_INT0_CP_OPCODE_ERROR |       \
	 A3XX_INT0_CP_RESERVED_BIT_ERROR | \
	 A3XX_INT0_CP_HW_FAULT |           \
	 A3XX_INT0_CP_IB1_INT |            \
	 A3XX_INT0_CP_IB2_INT |            \
	 A3XX_INT0_CP_RB_INT |             \
	 A3XX_INT0_CP_REG_PROTECT_FAULT |  \
	 A3XX_INT0_CP_AHB_ERROR_HALT |     \
	 A3XX_INT0_CACHE_FLUSH_TS |        \
	 A3XX_INT0_UCHE_OOB_ACCESS)

extern bool hang_debug;

#define A3XX_VBIF_XIN_HALT_CTRL0_MASK	GENMASK(5, 0)

static void a3xx_dump(struct msm_gpu *gpu);
static bool a3xx_idle(struct msm_gpu *gpu);

static void a3xx_submit(struct msm_gpu *gpu, struct msm_gem_submit *submit)
{
	struct a3xx_gpu *a3xx_gpu = to_a3xx_gpu(to_adreno_gpu(gpu));
	struct msm_ringbuffer *ring = submit->ring;
	unsigned int i;

	if (a3xx_gpu->pwron_fixup_pending) {
		OUT_PKT3(ring, CP_SET_PROTECTED_MODE, 1);
		OUT_RING(ring, 0);

		OUT_PKT3(ring, CP_INDIRECT_BUFFER_PFD, 2);
		OUT_RING(ring, lower_32_bits(a3xx_gpu->pwron_fixup_iova));
		OUT_RING(ring, a3xx_gpu->pwron_fixup_dwords);

		OUT_PKT3(ring, CP_SET_PROTECTED_MODE, 1);
		OUT_RING(ring, 1);

		a3xx_gpu->pwron_fixup_pending = false;
	}

	for (i = 0; i < submit->nr_cmds; i++) {
		switch (submit->cmd[i].type) {
		case MSM_SUBMIT_CMD_IB_TARGET_BUF:
			/* ignore IB-targets */
			break;
		case MSM_SUBMIT_CMD_CTX_RESTORE_BUF:
			/* ignore if there has not been a ctx switch: */
			if (ring->cur_ctx_seqno == submit->queue->ctx->seqno)
				break;
			fallthrough;
		case MSM_SUBMIT_CMD_BUF:
			OUT_PKT3(ring, CP_INDIRECT_BUFFER_PFD, 2);
			OUT_RING(ring, lower_32_bits(submit->cmd[i].iova));
			OUT_RING(ring, submit->cmd[i].size);
			OUT_PKT2(ring);
			break;
		}
	}

	OUT_PKT0(ring, REG_AXXX_CP_SCRATCH_REG2, 1);
	OUT_RING(ring, submit->seqno);

	/* Flush HLSQ lazy updates to make sure there is nothing
	 * pending for indirect loads after the timestamp has
	 * passed:
	 */
	OUT_PKT3(ring, CP_EVENT_WRITE, 1);
	OUT_RING(ring, HLSQ_FLUSH);

	/* wait for idle before cache flush/interrupt */
	OUT_PKT3(ring, CP_WAIT_FOR_IDLE, 1);
	OUT_RING(ring, 0x00000000);

	/* BIT(31) of CACHE_FLUSH_TS triggers CACHE_FLUSH_TS IRQ from GPU */
	OUT_PKT3(ring, CP_EVENT_WRITE, 3);
	OUT_RING(ring, CACHE_FLUSH_TS | CP_EVENT_WRITE_0_IRQ);
	OUT_RING(ring, rbmemptr(ring, fence));
	OUT_RING(ring, submit->seqno);

#if 0
	/* Dummy set-constant to trigger context rollover */
	OUT_PKT3(ring, CP_SET_CONSTANT, 2);
	OUT_RING(ring, CP_REG(REG_A3XX_HLSQ_CL_KERNEL_GROUP_X_REG));
	OUT_RING(ring, 0x00000000);
#endif

	adreno_flush(gpu, ring, REG_AXXX_CP_RB_WPTR);
}

static bool a3xx_me_init(struct msm_gpu *gpu)
{
	struct msm_ringbuffer *ring = gpu->rb[0];

	OUT_PKT3(ring, CP_ME_INIT, 17);
	OUT_RING(ring, 0x000003f7);
	OUT_RING(ring, 0x00000000);
	OUT_RING(ring, 0x00000000);
	OUT_RING(ring, 0x00000000);
	OUT_RING(ring, 0x00000080);
	OUT_RING(ring, 0x00000100);
	OUT_RING(ring, 0x00000180);
	OUT_RING(ring, 0x00006600);
	OUT_RING(ring, 0x00000150);
	OUT_RING(ring, 0x0000014e);
	OUT_RING(ring, 0x00000154);
	OUT_RING(ring, 0x00000001);
	OUT_RING(ring, 0x00000000);
	OUT_RING(ring, 0x00000000);
	OUT_RING(ring, 0x00000000);
	OUT_RING(ring, 0x00000000);
	OUT_RING(ring, 0x00000000);

	adreno_flush(gpu, ring, REG_AXXX_CP_RB_WPTR);
	return a3xx_idle(gpu);
}

static int a3xx_hw_init(struct msm_gpu *gpu)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a3xx_gpu *a3xx_gpu = to_a3xx_gpu(adreno_gpu);
	uint32_t *ptr, len;
	int i, ret;

	DBG("%s", gpu->name);

	gpu_write(gpu, REG_A3XX_RBBM_SW_RESET_CMD, 1);
	gpu_read(gpu, REG_A3XX_RBBM_SW_RESET_CMD);
	gpu_write(gpu, REG_A3XX_RBBM_SW_RESET_CMD, 0);

	if (adreno_is_a305(adreno_gpu)) {
		/* Set up 16 deep read/write request queues: */
		gpu_write(gpu, REG_A3XX_VBIF_IN_RD_LIM_CONF0, 0x10101010);
		gpu_write(gpu, REG_A3XX_VBIF_IN_RD_LIM_CONF1, 0x10101010);
		gpu_write(gpu, REG_A3XX_VBIF_OUT_RD_LIM_CONF0, 0x10101010);
		gpu_write(gpu, REG_A3XX_VBIF_OUT_WR_LIM_CONF0, 0x10101010);
		gpu_write(gpu, REG_A3XX_VBIF_DDR_OUT_MAX_BURST, 0x0000303);
		gpu_write(gpu, REG_A3XX_VBIF_IN_WR_LIM_CONF0, 0x10101010);
		gpu_write(gpu, REG_A3XX_VBIF_IN_WR_LIM_CONF1, 0x10101010);
		/* Enable WR-REQ: */
		gpu_write(gpu, REG_A3XX_VBIF_GATE_OFF_WRREQ_EN, 0x0000ff);
		/* Set up round robin arbitration between both AXI ports: */
		gpu_write(gpu, REG_A3XX_VBIF_ARB_CTL, 0x00000030);
		/* Set up AOOO: */
		gpu_write(gpu, REG_A3XX_VBIF_OUT_AXI_AOOO_EN, 0x0000003c);
		gpu_write(gpu, REG_A3XX_VBIF_OUT_AXI_AOOO, 0x003c003c);
	} else if (adreno_is_a305b(adreno_gpu)) {
		gpu_write(gpu, REG_A3XX_VBIF_IN_RD_LIM_CONF0, 0x00181818);
		gpu_write(gpu, REG_A3XX_VBIF_IN_WR_LIM_CONF0, 0x00181818);
		gpu_write(gpu, REG_A3XX_VBIF_OUT_RD_LIM_CONF0, 0x00000018);
		gpu_write(gpu, REG_A3XX_VBIF_OUT_WR_LIM_CONF0, 0x00000018);
		gpu_write(gpu, REG_A3XX_VBIF_DDR_OUT_MAX_BURST, 0x00000303);
		gpu_write(gpu, REG_A3XX_VBIF_ROUND_ROBIN_QOS_ARB, 0x0003);
	} else if (adreno_is_a306(adreno_gpu)) {
		gpu_write(gpu, REG_A3XX_VBIF_ROUND_ROBIN_QOS_ARB, 0x0003);
		gpu_write(gpu, REG_A3XX_VBIF_OUT_RD_LIM_CONF0, 0x0000000a);
		gpu_write(gpu, REG_A3XX_VBIF_OUT_WR_LIM_CONF0, 0x0000000a);
	} else if (adreno_is_a306a(adreno_gpu)) {
		gpu_write(gpu, REG_A3XX_VBIF_ROUND_ROBIN_QOS_ARB, 0x0003);
		gpu_write(gpu, REG_A3XX_VBIF_OUT_RD_LIM_CONF0, 0x00000010);
		gpu_write(gpu, REG_A3XX_VBIF_OUT_WR_LIM_CONF0, 0x00000010);
	} else if (adreno_is_a320(adreno_gpu)) {
		/* Set up 16 deep read/write request queues: */
		gpu_write(gpu, REG_A3XX_VBIF_IN_RD_LIM_CONF0, 0x10101010);
		gpu_write(gpu, REG_A3XX_VBIF_IN_RD_LIM_CONF1, 0x10101010);
		gpu_write(gpu, REG_A3XX_VBIF_OUT_RD_LIM_CONF0, 0x10101010);
		gpu_write(gpu, REG_A3XX_VBIF_OUT_WR_LIM_CONF0, 0x10101010);
		gpu_write(gpu, REG_A3XX_VBIF_DDR_OUT_MAX_BURST, 0x0000303);
		gpu_write(gpu, REG_A3XX_VBIF_IN_WR_LIM_CONF0, 0x10101010);
		gpu_write(gpu, REG_A3XX_VBIF_IN_WR_LIM_CONF1, 0x10101010);
		/* Enable WR-REQ: */
		gpu_write(gpu, REG_A3XX_VBIF_GATE_OFF_WRREQ_EN, 0x0000ff);
		/* Set up round robin arbitration between both AXI ports: */
		gpu_write(gpu, REG_A3XX_VBIF_ARB_CTL, 0x00000030);
		/* Set up AOOO: */
		gpu_write(gpu, REG_A3XX_VBIF_OUT_AXI_AOOO_EN, 0x0000003c);
		gpu_write(gpu, REG_A3XX_VBIF_OUT_AXI_AOOO, 0x003c003c);
		/* Enable 1K sort: */
		gpu_write(gpu, REG_A3XX_VBIF_ABIT_SORT, 0x000000ff);
		gpu_write(gpu, REG_A3XX_VBIF_ABIT_SORT_CONF, 0x000000a4);

	} else if (adreno_is_a330v2(adreno_gpu)) {
		/*
		 * Most of the VBIF registers on 8974v2 have the correct
		 * values at power on, so we won't modify those if we don't
		 * need to
		 */
		/* Enable 1k sort: */
		gpu_write(gpu, REG_A3XX_VBIF_ABIT_SORT, 0x0001003f);
		gpu_write(gpu, REG_A3XX_VBIF_ABIT_SORT_CONF, 0x000000a4);
		/* Enable WR-REQ: */
		gpu_write(gpu, REG_A3XX_VBIF_GATE_OFF_WRREQ_EN, 0x00003f);
		gpu_write(gpu, REG_A3XX_VBIF_DDR_OUT_MAX_BURST, 0x0000303);
		/* Set up VBIF_ROUND_ROBIN_QOS_ARB: */
		gpu_write(gpu, REG_A3XX_VBIF_ROUND_ROBIN_QOS_ARB, 0x0003);

	} else if (adreno_is_a330(adreno_gpu)) {
		/* Set up 16 deep read/write request queues: */
		gpu_write(gpu, REG_A3XX_VBIF_IN_RD_LIM_CONF0, 0x18181818);
		gpu_write(gpu, REG_A3XX_VBIF_IN_RD_LIM_CONF1, 0x18181818);
		gpu_write(gpu, REG_A3XX_VBIF_OUT_RD_LIM_CONF0, 0x18181818);
		gpu_write(gpu, REG_A3XX_VBIF_OUT_WR_LIM_CONF0, 0x18181818);
		gpu_write(gpu, REG_A3XX_VBIF_DDR_OUT_MAX_BURST, 0x0000303);
		gpu_write(gpu, REG_A3XX_VBIF_IN_WR_LIM_CONF0, 0x18181818);
		gpu_write(gpu, REG_A3XX_VBIF_IN_WR_LIM_CONF1, 0x18181818);
		/* Enable WR-REQ: */
		gpu_write(gpu, REG_A3XX_VBIF_GATE_OFF_WRREQ_EN, 0x00003f);
		/* Set up round robin arbitration between both AXI ports: */
		gpu_write(gpu, REG_A3XX_VBIF_ARB_CTL, 0x00000030);
		/* Set up VBIF_ROUND_ROBIN_QOS_ARB: */
		gpu_write(gpu, REG_A3XX_VBIF_ROUND_ROBIN_QOS_ARB, 0x0001);
		/* Set up AOOO: */
		gpu_write(gpu, REG_A3XX_VBIF_OUT_AXI_AOOO_EN, 0x0000003f);
		gpu_write(gpu, REG_A3XX_VBIF_OUT_AXI_AOOO, 0x003f003f);
		/* Enable 1K sort: */
		gpu_write(gpu, REG_A3XX_VBIF_ABIT_SORT, 0x0001003f);
		gpu_write(gpu, REG_A3XX_VBIF_ABIT_SORT_CONF, 0x000000a4);
		/* Disable VBIF clock gating. This is to enable AXI running
		 * higher frequency than GPU:
		 */
		gpu_write(gpu, REG_A3XX_VBIF_CLKON, 0x00000001);

	} else {
		BUG();
	}

	/* Make all blocks contribute to the GPU BUSY perf counter: */
	gpu_write(gpu, REG_A3XX_RBBM_GPU_BUSY_MASKED, 0xffffffff);

	/* Tune the hystersis counters for SP and CP idle detection: */
	gpu_write(gpu, REG_A3XX_RBBM_SP_HYST_CNT, 0x10);
	gpu_write(gpu, REG_A3XX_RBBM_WAIT_IDLE_CLOCKS_CTL, 0x10);

	/* Enable the RBBM error reporting bits.  This lets us get
	 * useful information on failure:
	 */
	gpu_write(gpu, REG_A3XX_RBBM_AHB_CTL0, 0x00000001);

	/* Enable AHB error reporting: */
	gpu_write(gpu, REG_A3XX_RBBM_AHB_CTL1, 0xa6ffffff);

	/* Turn on the power counters: */
	gpu_write(gpu, REG_A3XX_RBBM_RBBM_CTL, 0x00030000);

	/* Turn on hang detection - this spews a lot of useful information
	 * into the RBBM registers on a hang:
	 */
	gpu_write(gpu, REG_A3XX_RBBM_INTERFACE_HANG_INT_CTL, 0x00010fff);

	/* Enable 64-byte cacheline size. HW Default is 32-byte (0x000000E0): */
	gpu_write(gpu, REG_A3XX_UCHE_CACHE_MODE_CONTROL_REG, 0x00000001);

	/* Enable Clock gating: */
	if (adreno_is_a305b(adreno_gpu) ||
	    adreno_is_a306(adreno_gpu) ||
	    adreno_is_a306a(adreno_gpu))
		gpu_write(gpu, REG_A3XX_RBBM_CLOCK_CTL, 0xaaaaaaaa);
	else if (adreno_is_a320(adreno_gpu))
		gpu_write(gpu, REG_A3XX_RBBM_CLOCK_CTL, 0xbfffffff);
	else if (adreno_is_a330v2(adreno_gpu))
		gpu_write(gpu, REG_A3XX_RBBM_CLOCK_CTL, 0xaaaaaaaa);
	else if (adreno_is_a330(adreno_gpu))
		gpu_write(gpu, REG_A3XX_RBBM_CLOCK_CTL, 0xbffcffff);

	if (adreno_is_a330v2(adreno_gpu))
		gpu_write(gpu, REG_A3XX_RBBM_GPR0_CTL, 0x05515455);
	else if (adreno_is_a330(adreno_gpu))
		gpu_write(gpu, REG_A3XX_RBBM_GPR0_CTL, 0x00000000);

	/* Set the OCMEM base address for A330, etc */
	if (a3xx_gpu->ocmem.hdl) {
		gpu_write(gpu, REG_A3XX_RB_GMEM_BASE_ADDR,
			(unsigned int)(a3xx_gpu->ocmem.base >> 14));
	}

	/* Turn on performance counters: */
	gpu_write(gpu, REG_A3XX_RBBM_PERFCTR_CTL, 0x01);

	gpu_write(gpu, REG_A3XX_RBBM_INT_0_MASK, A3XX_INT0_MASK);

	ret = adreno_hw_init(gpu);
	if (ret)
		return ret;

	/*
	 * Use the default ringbuffer size and block size but disable the RPTR
	 * shadow
	 */
	gpu_write(gpu, REG_AXXX_CP_RB_CNTL,
		MSM_GPU_RB_CNTL_DEFAULT | AXXX_CP_RB_CNTL_NO_UPDATE);

	/* Set the ringbuffer address */
	gpu_write(gpu, REG_AXXX_CP_RB_BASE, lower_32_bits(gpu->rb[0]->iova));

	/* setup access protection: */
	gpu_write(gpu, REG_A3XX_CP_PROTECT_CTRL, 0x00000007);

	/* RBBM registers */
	gpu_write(gpu, REG_A3XX_CP_PROTECT(0), 0x63000040);
	gpu_write(gpu, REG_A3XX_CP_PROTECT(1), 0x62000080);
	gpu_write(gpu, REG_A3XX_CP_PROTECT(2), 0x600000cc);
	gpu_write(gpu, REG_A3XX_CP_PROTECT(3), 0x60000108);
	gpu_write(gpu, REG_A3XX_CP_PROTECT(4), 0x64000140);
	gpu_write(gpu, REG_A3XX_CP_PROTECT(5), 0x66000400);

	/* CP registers */
	gpu_write(gpu, REG_A3XX_CP_PROTECT(6), 0x65000700);
	gpu_write(gpu, REG_A3XX_CP_PROTECT(7), 0x610007d8);
	gpu_write(gpu, REG_A3XX_CP_PROTECT(8), 0x620007e0);
	gpu_write(gpu, REG_A3XX_CP_PROTECT(9), 0x61001178);
	gpu_write(gpu, REG_A3XX_CP_PROTECT(10), 0x64001180);

	/* RB registers */
	gpu_write(gpu, REG_A3XX_CP_PROTECT(11), 0x60003300);

	/* VBIF registers */
	gpu_write(gpu, REG_A3XX_CP_PROTECT(12), 0x6b00c000);

	/* NOTE: PM4/micro-engine firmware registers look to be the same
	 * for a2xx and a3xx.. we could possibly push that part down to
	 * adreno_gpu base class.  Or push both PM4 and PFP but
	 * parameterize the pfp ucode addr/data registers..
	 */

	/* Load PM4: */
	ptr = (uint32_t *)(adreno_gpu->fw[ADRENO_FW_PM4]->data);
	len = adreno_gpu->fw[ADRENO_FW_PM4]->size / 4;
	DBG("loading PM4 ucode version: %x", ptr[1]);

	gpu_write(gpu, REG_AXXX_CP_DEBUG,
			AXXX_CP_DEBUG_DYNAMIC_CLK_DISABLE |
			AXXX_CP_DEBUG_MIU_128BIT_WRITE_ENABLE);
	gpu_write(gpu, REG_AXXX_CP_ME_RAM_WADDR, 0);
	for (i = 1; i < len; i++)
		gpu_write(gpu, REG_AXXX_CP_ME_RAM_DATA, ptr[i]);

	/* Load PFP: */
	ptr = (uint32_t *)(adreno_gpu->fw[ADRENO_FW_PFP]->data);
	len = adreno_gpu->fw[ADRENO_FW_PFP]->size / 4;
	DBG("loading PFP ucode version: %x", ptr[5]);

	gpu_write(gpu, REG_A3XX_CP_PFP_UCODE_ADDR, 0);
	for (i = 1; i < len; i++)
		gpu_write(gpu, REG_A3XX_CP_PFP_UCODE_DATA, ptr[i]);

	/* CP ROQ queue sizes (bytes) - RB:16, ST:16, IB1:32, IB2:64 */
	if (adreno_is_a305(adreno_gpu) ||
	    adreno_is_a306(adreno_gpu) ||
	    adreno_is_a306a(adreno_gpu) ||
	    adreno_is_a320(adreno_gpu)) {
		gpu_write(gpu, REG_AXXX_CP_QUEUE_THRESHOLDS,
				AXXX_CP_QUEUE_THRESHOLDS_CSQ_IB1_START(2) |
				AXXX_CP_QUEUE_THRESHOLDS_CSQ_IB2_START(6) |
				AXXX_CP_QUEUE_THRESHOLDS_CSQ_ST_START(14));
	} else if (adreno_is_a330(adreno_gpu) || adreno_is_a305b(adreno_gpu)) {
		/* NOTE: this (value take from downstream android driver)
		 * includes some bits outside of the known bitfields.  But
		 * A330 has this "MERCIU queue" thing too, which might
		 * explain a new bitfield or reshuffling:
		 */
		gpu_write(gpu, REG_AXXX_CP_QUEUE_THRESHOLDS, 0x003e2008);
	}

	/* clear ME_HALT to start micro engine */
	gpu_write(gpu, REG_AXXX_CP_ME_CNTL, 0);

	a3xx_gpu->pwron_fixup_pending = true;

	return a3xx_me_init(gpu) ? 0 : -EINVAL;
}

static void a3xx_recover(struct msm_gpu *gpu)
{
	int i;

	adreno_dump_info(gpu);

	for (i = 0; i < 8; i++) {
		printk("CP_SCRATCH_REG%d: %u\n", i,
			gpu_read(gpu, REG_AXXX_CP_SCRATCH_REG0 + i));
	}

	/* dump registers before resetting gpu, if enabled: */
	if (hang_debug)
		a3xx_dump(gpu);

	gpu_write(gpu, REG_A3XX_RBBM_SW_RESET_CMD, 1);
	gpu_read(gpu, REG_A3XX_RBBM_SW_RESET_CMD);
	gpu_write(gpu, REG_A3XX_RBBM_SW_RESET_CMD, 0);
	adreno_recover(gpu);
}

static void a3xx_destroy(struct msm_gpu *gpu)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a3xx_gpu *a3xx_gpu = to_a3xx_gpu(adreno_gpu);

	DBG("%s", gpu->name);

	msm_gem_kernel_put(a3xx_gpu->pwron_fixup_bo, gpu->vm);

	adreno_gpu_cleanup(adreno_gpu);

	adreno_gpu_ocmem_cleanup(&a3xx_gpu->ocmem);

	kfree(a3xx_gpu);
}

static bool a3xx_idle(struct msm_gpu *gpu)
{
	/* wait for ringbuffer to drain: */
	if (!adreno_idle(gpu, gpu->rb[0]))
		return false;

	/* then wait for GPU to finish: */
	if (spin_until(!(gpu_read(gpu, REG_A3XX_RBBM_STATUS) &
			A3XX_RBBM_STATUS_GPU_BUSY))) {
		DRM_ERROR("%s: timeout waiting for GPU to idle!\n", gpu->name);

		/* TODO maybe we need to reset GPU here to recover from hang? */
		return false;
	}

	return true;
}

static irqreturn_t a3xx_irq(struct msm_gpu *gpu)
{
	uint32_t status;

	status = gpu_read(gpu, REG_A3XX_RBBM_INT_0_STATUS);
	DBG("%s: %08x", gpu->name, status);

	// TODO

	gpu_write(gpu, REG_A3XX_RBBM_INT_CLEAR_CMD, status);

	msm_gpu_retire(gpu);

	return IRQ_HANDLED;
}

static const unsigned int a3xx_registers[] = {
	0x0000, 0x0002, 0x0010, 0x0012, 0x0018, 0x0018, 0x0020, 0x0027,
	0x0029, 0x002b, 0x002e, 0x0033, 0x0040, 0x0042, 0x0050, 0x005c,
	0x0060, 0x006c, 0x0080, 0x0082, 0x0084, 0x0088, 0x0090, 0x00e5,
	0x00ea, 0x00ed, 0x0100, 0x0100, 0x0110, 0x0123, 0x01c0, 0x01c1,
	0x01c3, 0x01c5, 0x01c7, 0x01c7, 0x01d5, 0x01d9, 0x01dc, 0x01dd,
	0x01ea, 0x01ea, 0x01ee, 0x01f1, 0x01f5, 0x01f5, 0x01fc, 0x01ff,
	0x0440, 0x0440, 0x0443, 0x0443, 0x0445, 0x0445, 0x044d, 0x044f,
	0x0452, 0x0452, 0x0454, 0x046f, 0x047c, 0x047c, 0x047f, 0x047f,
	0x0578, 0x057f, 0x0600, 0x0602, 0x0605, 0x0607, 0x060a, 0x060e,
	0x0612, 0x0614, 0x0c01, 0x0c02, 0x0c06, 0x0c1d, 0x0c3d, 0x0c3f,
	0x0c48, 0x0c4b, 0x0c80, 0x0c80, 0x0c88, 0x0c8b, 0x0ca0, 0x0cb7,
	0x0cc0, 0x0cc1, 0x0cc6, 0x0cc7, 0x0ce4, 0x0ce5, 0x0e00, 0x0e05,
	0x0e0c, 0x0e0c, 0x0e22, 0x0e23, 0x0e41, 0x0e45, 0x0e64, 0x0e65,
	0x0e80, 0x0e82, 0x0e84, 0x0e89, 0x0ea0, 0x0ea1, 0x0ea4, 0x0ea7,
	0x0ec4, 0x0ecb, 0x0ee0, 0x0ee0, 0x0f00, 0x0f01, 0x0f03, 0x0f09,
	0x2040, 0x2040, 0x2044, 0x2044, 0x2048, 0x204d, 0x2068, 0x2069,
	0x206c, 0x206d, 0x2070, 0x2070, 0x2072, 0x2072, 0x2074, 0x2075,
	0x2079, 0x207a, 0x20c0, 0x20d3, 0x20e4, 0x20ef, 0x2100, 0x2109,
	0x210c, 0x210c, 0x210e, 0x210e, 0x2110, 0x2111, 0x2114, 0x2115,
	0x21e4, 0x21e4, 0x21ea, 0x21ea, 0x21ec, 0x21ed, 0x21f0, 0x21f0,
	0x2200, 0x2212, 0x2214, 0x2217, 0x221a, 0x221a, 0x2240, 0x227e,
	0x2280, 0x228b, 0x22c0, 0x22c0, 0x22c4, 0x22ce, 0x22d0, 0x22d8,
	0x22df, 0x22e6, 0x22e8, 0x22e9, 0x22ec, 0x22ec, 0x22f0, 0x22f7,
	0x22ff, 0x22ff, 0x2340, 0x2343, 0x2440, 0x2440, 0x2444, 0x2444,
	0x2448, 0x244d, 0x2468, 0x2469, 0x246c, 0x246d, 0x2470, 0x2470,
	0x2472, 0x2472, 0x2474, 0x2475, 0x2479, 0x247a, 0x24c0, 0x24d3,
	0x24e4, 0x24ef, 0x2500, 0x2509, 0x250c, 0x250c, 0x250e, 0x250e,
	0x2510, 0x2511, 0x2514, 0x2515, 0x25e4, 0x25e4, 0x25ea, 0x25ea,
	0x25ec, 0x25ed, 0x25f0, 0x25f0, 0x2600, 0x2612, 0x2614, 0x2617,
	0x261a, 0x261a, 0x2640, 0x267e, 0x2680, 0x268b, 0x26c0, 0x26c0,
	0x26c4, 0x26ce, 0x26d0, 0x26d8, 0x26df, 0x26e6, 0x26e8, 0x26e9,
	0x26ec, 0x26ec, 0x26f0, 0x26f7, 0x26ff, 0x26ff, 0x2740, 0x2743,
	0x300c, 0x300e, 0x301c, 0x301d, 0x302a, 0x302a, 0x302c, 0x302d,
	0x3030, 0x3031, 0x3034, 0x3036, 0x303c, 0x303c, 0x305e, 0x305f,
	~0   /* sentinel */
};

/* would be nice to not have to duplicate the _show() stuff with printk(): */
static void a3xx_dump(struct msm_gpu *gpu)
{
	printk("status:   %08x\n",
			gpu_read(gpu, REG_A3XX_RBBM_STATUS));
	adreno_dump(gpu);
}

static struct msm_gpu_state *a3xx_gpu_state_get(struct msm_gpu *gpu)
{
	struct msm_gpu_state *state = kzalloc_obj(*state);

	if (!state)
		return ERR_PTR(-ENOMEM);

	adreno_gpu_state_get(gpu, state);

	state->rbbm_status = gpu_read(gpu, REG_A3XX_RBBM_STATUS);

	return state;
}

static u64 a3xx_gpu_busy(struct msm_gpu *gpu, unsigned long *out_sample_rate)
{
	u64 busy_cycles;

	busy_cycles = gpu_read64(gpu, REG_A3XX_RBBM_PERFCTR_RBBM_1_LO);
	*out_sample_rate = clk_get_rate(gpu->core_clk);

	return busy_cycles;
}

static int a3xx_vbif_halt(struct msm_gpu *gpu)
{
	u32 ack;
	int ret;

	gpu_write(gpu, REG_A3XX_VBIF_XIN_HALT_CTRL0,
		  A3XX_VBIF_XIN_HALT_CTRL0_MASK);
	ret = spin_until(((ack = gpu_read(gpu, REG_A3XX_VBIF_XIN_HALT_CTRL1)) &
			  A3XX_VBIF_XIN_HALT_CTRL0_MASK) ==
			 A3XX_VBIF_XIN_HALT_CTRL0_MASK);
	gpu_write(gpu, REG_A3XX_VBIF_XIN_HALT_CTRL0, 0);

	if (ret)
		return -EBUSY;

	return 0;
}

static int a3xx_pm_suspend(struct msm_gpu *gpu)
{
	struct a3xx_gpu *a3xx_gpu = to_a3xx_gpu(to_adreno_gpu(gpu));
	int ret;

	ret = a3xx_vbif_halt(gpu);
	if (ret)
		return ret;

	ret = msm_gpu_pm_suspend(gpu);
	if (ret)
		return ret;

	adreno_gpu_ocmem_suspend(&a3xx_gpu->ocmem);

	icc_set_bw(a3xx_gpu->icc_path, 0, 0);
	icc_set_bw(a3xx_gpu->ocmem_icc_path, 0, 0);

	return 0;
}

static int a3xx_pm_resume(struct msm_gpu *gpu)
{
	struct a3xx_gpu *a3xx_gpu = to_a3xx_gpu(to_adreno_gpu(gpu));
	int ret;

	icc_set_bw(a3xx_gpu->icc_path, 0, Bps_to_icc(gpu->fast_rate) * 8);
	icc_set_bw(a3xx_gpu->ocmem_icc_path, 0, Bps_to_icc(gpu->fast_rate) * 8);

	ret = adreno_gpu_ocmem_resume(to_adreno_gpu(gpu), &a3xx_gpu->ocmem);
	if (ret)
		return ret;

	return msm_gpu_pm_resume(gpu);
}

static u32 a3xx_get_rptr(struct msm_gpu *gpu, struct msm_ringbuffer *ring)
{
	ring->memptrs->rptr = gpu_read(gpu, REG_AXXX_CP_RB_RPTR);
	return ring->memptrs->rptr;
}

static const u32 a3xx_pwron_fixup_fs_instructions[] = {
	0x00000000, 0x302cc300, 0x00000000, 0x302cc304, 0x00000000, 0x302cc308, 0x00000000, 0x302cc30c,
	0x00000000, 0x302cc310, 0x00000000, 0x302cc314, 0x00000000, 0x302cc318, 0x00000000, 0x302cc31c,
	0x00000000, 0x302cc320, 0x00000000, 0x302cc324, 0x00000000, 0x302cc328, 0x00000000, 0x302cc32c,
	0x00000000, 0x302cc330, 0x00000000, 0x302cc334, 0x00000000, 0x302cc338, 0x00000000, 0x302cc33c,
	0x00000000, 0x00000400, 0x00020000, 0x63808003, 0x00060004, 0x63828007, 0x000a0008, 0x6384800b,
	0x000e000c, 0x6386800f, 0x00120010, 0x63888013, 0x00160014, 0x638a8017, 0x001a0018, 0x638c801b,
	0x001e001c, 0x638e801f, 0x00220020, 0x63908023, 0x00260024, 0x63928027, 0x002a0028, 0x6394802b,
	0x002e002c, 0x6396802f, 0x00320030, 0x63988033, 0x00360034, 0x639a8037, 0x003a0038, 0x639c803b,
	0x003e003c, 0x639e803f, 0x00000000, 0x00000400, 0x00000003, 0x80d60003, 0x00000007, 0x80d60007,
	0x0000000b, 0x80d6000b, 0x0000000f, 0x80d6000f, 0x00000013, 0x80d60013, 0x00000017, 0x80d60017,
	0x0000001b, 0x80d6001b, 0x0000001f, 0x80d6001f, 0x00000023, 0x80d60023, 0x00000027, 0x80d60027,
	0x0000002b, 0x80d6002b, 0x0000002f, 0x80d6002f, 0x00000033, 0x80d60033, 0x00000037, 0x80d60037,
	0x0000003b, 0x80d6003b, 0x0000003f, 0x80d6003f, 0x00000000, 0x03000000, 0x00000000, 0x00000000,
};

static int a3xx_pwron_fixup_init(struct msm_gpu *gpu)
{
	struct a3xx_gpu *a3xx_gpu = to_a3xx_gpu(to_adreno_gpu(gpu));
	u32 *start, *cmds;

	start = msm_gem_kernel_new(gpu->dev, SZ_4K, MSM_BO_WC | MSM_BO_GPU_READONLY, gpu->vm,
				   &a3xx_gpu->pwron_fixup_bo, &a3xx_gpu->pwron_fixup_iova);
	if (IS_ERR(start))
		return PTR_ERR(start);

	msm_gem_object_set_name(a3xx_gpu->pwron_fixup_bo, "pwron_fixup");

	cmds = start;

	*cmds++ = CP_TYPE0_PKT | (1 << 16) | REG_A3XX_UCHE_CACHE_INVALIDATE0_REG;
	*cmds++ = 0x00000000;
	*cmds++ = 0x90000000;

	*cmds++ = CP_TYPE3_PKT | (CP_WAIT_FOR_IDLE << 8);
	*cmds++ = 0x00000000;

	*cmds++ = CP_TYPE3_PKT | (2 << 16) | (CP_REG_RMW << 8);
	*cmds++ = REG_A3XX_RBBM_CLOCK_CTL;
	*cmds++ = 0xfffcffff;
	*cmds++ = 0x00010000;

	*cmds++ = CP_TYPE3_PKT | (CP_WAIT_FOR_IDLE << 8);
	*cmds++ = 0x00000000;

	*cmds++ = CP_TYPE0_PKT | REG_A3XX_HLSQ_CONTROL_0_REG;
	*cmds++ = 0x1e000150;

	*cmds++ = CP_TYPE3_PKT | (CP_WAIT_FOR_IDLE << 8);
	*cmds++ = 0x00000000;

	*cmds++ = CP_TYPE3_PKT | (1 << 16) | (CP_SET_CONSTANT << 8);
	*cmds++ = (0x4 << 16) | (REG_A3XX_HLSQ_CONTROL_0_REG - 0x2000);
	*cmds++ = 0x1e000150;

	*cmds++ = CP_TYPE3_PKT | (CP_WAIT_FOR_IDLE << 8);
	*cmds++ = 0x00000000;

	*cmds++ = CP_TYPE0_PKT | REG_A3XX_HLSQ_CONTROL_0_REG;
	*cmds++ = 0x1e000150;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_HLSQ_CONTROL_1_REG;
	*cmds++ = 0x00000040;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_HLSQ_CONTROL_2_REG;
	*cmds++ = 0x80000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_HLSQ_CONTROL_3_REG;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_HLSQ_VS_CONTROL_REG;
	*cmds++ = 0x00000001;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_HLSQ_FS_CONTROL_REG;
	*cmds++ = 0x0d001002;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_HLSQ_CONST_VSPRESV_RANGE_REG;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_HLSQ_CONST_FSPRESV_RANGE_REG;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_HLSQ_CL_NDRANGE_0_REG;
	*cmds++ = 0x00401101;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_HLSQ_CL_GLOBAL_WORK_SIZE(0);
	*cmds++ = 0x00000400;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_HLSQ_CL_GLOBAL_WORK_OFFSET(0);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_HLSQ_CL_GLOBAL_WORK_SIZE(1);
	*cmds++ = 0x00000001;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_HLSQ_CL_GLOBAL_WORK_OFFSET(1);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_HLSQ_CL_GLOBAL_WORK_SIZE(2);
	*cmds++ = 0x00000001;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_HLSQ_CL_GLOBAL_WORK_OFFSET(2);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_HLSQ_CL_CONTROL_0_REG;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_HLSQ_CL_CONTROL_1_REG;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_HLSQ_CL_KERNEL_CONST_REG;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_HLSQ_CL_KERNEL_GROUP_RATIO(0);
	*cmds++ = 0x00000010;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_HLSQ_CL_KERNEL_GROUP_Y_REG;
	*cmds++ = 0x00000001;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_HLSQ_CL_KERNEL_GROUP_Z_REG;
	*cmds++ = 0x00000001;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_HLSQ_CL_WG_OFFSET_REG;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_SP_CTRL_REG;
	*cmds++ = 0x00040000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_VS_CTRL_REG0;
	*cmds++ = 0x0000000a;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_VS_CTRL_REG1;
	*cmds++ = 0x00000001;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_VS_PARAM_REG;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_VS_OUT_REG(0);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_VS_OUT_REG(1);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_VS_OUT_REG(2);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_VS_OUT_REG(3);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_VS_OUT_REG(4);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_VS_OUT_REG(5);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_VS_OUT_REG(6);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_VS_OUT_REG(7);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_VS_VPC_DST_REG(0);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_VS_VPC_DST_REG(1);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_VS_VPC_DST_REG(2);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_VS_VPC_DST_REG(3);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_VS_OBJ_OFFSET_REG;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_VS_OBJ_START_REG;
	*cmds++ = 0x00000004;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_VS_PVT_MEM_PARAM_REG;
	*cmds++ = 0x04008001;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_VS_PVT_MEM_ADDR_REG;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_VS_PVT_MEM_SIZE_REG;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_VS_LENGTH_REG;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_FS_CTRL_REG0;
	*cmds++ = 0x0db0400a;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_FS_CTRL_REG1;
	*cmds++ = 0x00300402;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_FS_OBJ_OFFSET_REG;
	*cmds++ = 0x00010000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_FS_OBJ_START_REG;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_FS_PVT_MEM_PARAM_REG;
	*cmds++ = 0x04008001;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_FS_PVT_MEM_ADDR_REG;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_FS_PVT_MEM_SIZE_REG;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_FS_FLAT_SHAD_MODE_REG_0;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_FS_FLAT_SHAD_MODE_REG_1;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_FS_OUTPUT_REG;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_FS_MRT_REG(0);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_FS_MRT_REG(1);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_FS_MRT_REG(2);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_FS_MRT_REG(3);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_FS_IMAGE_OUTPUT_REG(0);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_FS_IMAGE_OUTPUT_REG(1);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_FS_IMAGE_OUTPUT_REG(2);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_FS_IMAGE_OUTPUT_REG(3);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_SP_FS_LENGTH_REG;
	*cmds++ = 0x0000000d;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_CLIP_CNTL;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_GB_CLIP_ADJ;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_VPORT_XOFFSET;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_VPORT_XSCALE;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_VPORT_YOFFSET;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_VPORT_YSCALE;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_VPORT_ZOFFSET;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_VPORT_ZSCALE;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_USER_PLANE_X(0);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_USER_PLANE_Y(0);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_USER_PLANE_Z(0);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_USER_PLANE_W(0);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_USER_PLANE_X(1);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_USER_PLANE_Y(1);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_USER_PLANE_Z(1);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_USER_PLANE_W(1);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_USER_PLANE_X(2);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_USER_PLANE_Y(2);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_USER_PLANE_Z(2);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_USER_PLANE_W(2);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_USER_PLANE_X(3);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_USER_PLANE_Y(3);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_USER_PLANE_Z(3);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_USER_PLANE_W(3);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_USER_PLANE_X(4);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_USER_PLANE_Y(4);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_USER_PLANE_Z(4);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_USER_PLANE_W(4);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_USER_PLANE_X(5);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_USER_PLANE_Y(5);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_USER_PLANE_Z(5);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_CL_USER_PLANE_W(5);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_SU_POINT_MINMAX;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_SU_POINT_SIZE;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_SU_POLY_OFFSET_OFFSET;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_SU_POLY_OFFSET_SCALE;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_SU_MODE_CONTROL;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_SC_CONTROL;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_SC_SCREEN_SCISSOR_TL;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_SC_SCREEN_SCISSOR_BR;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_SC_WINDOW_SCISSOR_BR;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_SC_WINDOW_SCISSOR_TL;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_TSE_DEBUG_ECO;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_PERFCOUNTER0_SELECT;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_PERFCOUNTER1_SELECT;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_PERFCOUNTER2_SELECT;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_GRAS_PERFCOUNTER3_SELECT;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_MODE_CONTROL;
	*cmds++ = 0x00008000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_RENDER_CONTROL;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_MSAA_CONTROL;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_ALPHA_REF;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_MRT_CONTROL(0);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_MRT_CONTROL(1);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_MRT_CONTROL(2);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_MRT_CONTROL(3);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_MRT_BUF_INFO(0);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_MRT_BUF_INFO(1);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_MRT_BUF_INFO(2);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_MRT_BUF_INFO(3);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_MRT_BUF_BASE(0);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_MRT_BUF_BASE(1);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_MRT_BUF_BASE(2);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_MRT_BUF_BASE(3);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_MRT_BLEND_CONTROL(0);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_MRT_BLEND_CONTROL(1);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_MRT_BLEND_CONTROL(2);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_MRT_BLEND_CONTROL(3);
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_BLEND_RED;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_BLEND_GREEN;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_BLEND_BLUE;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_BLEND_ALPHA;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_CLEAR_COLOR_DW0;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_CLEAR_COLOR_DW1;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_CLEAR_COLOR_DW2;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_CLEAR_COLOR_DW3;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_COPY_CONTROL;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_COPY_DEST_BASE;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_COPY_DEST_PITCH;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_COPY_DEST_INFO;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_DEPTH_CONTROL;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_DEPTH_CLEAR;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_DEPTH_INFO;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_DEPTH_PITCH;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_STENCIL_CONTROL;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_STENCIL_CLEAR;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_STENCIL_INFO;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_STENCIL_PITCH;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_STENCILREFMASK;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_STENCILREFMASK_BF;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_LRZ_VSC_CONTROL;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_WINDOW_OFFSET;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_SAMPLE_COUNT_CONTROL;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_SAMPLE_COUNT_ADDR;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_Z_CLAMP_MIN;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_Z_CLAMP_MAX;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_GMEM_BASE_ADDR;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_DEBUG_ECO_CONTROLS_ADDR;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_PERFCOUNTER0_SELECT;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_PERFCOUNTER1_SELECT;
	*cmds++ = 0x00000000;
	*cmds++ = CP_TYPE0_PKT | REG_A3XX_RB_FRAME_BUFFER_DIMENSION;
	*cmds++ = 0x00000000;

	*cmds++ = CP_TYPE3_PKT | (3 << 16) | (CP_LOAD_STATE << 8);
	*cmds++ = CP_LOAD_STATE_0_DST_OFF(1) | CP_LOAD_STATE_0_STATE_BLOCK(SB_FRAG_SHADER) | CP_LOAD_STATE_0_NUM_UNIT(1);
	*cmds++ = CP_LOAD_STATE_1_STATE_TYPE(ST_CONSTANTS);
	*cmds++ = 0x00400000;
	*cmds++ = 0x00000000;

	*cmds++ = CP_TYPE3_PKT | (3 << 16) | (CP_LOAD_STATE << 8);
	*cmds++ = CP_LOAD_STATE_0_DST_OFF(2) | CP_LOAD_STATE_0_STATE_BLOCK(SB_FRAG_SHADER) | CP_LOAD_STATE_0_NUM_UNIT(1);
	*cmds++ = CP_LOAD_STATE_1_STATE_TYPE(ST_CONSTANTS);
	*cmds++ = 0x00400220;
	*cmds++ = 0x00000000;

	*cmds++ = CP_TYPE3_PKT | (3 << 16) | (CP_LOAD_STATE << 8);
	*cmds++ = CP_LOAD_STATE_0_STATE_BLOCK(SB_FRAG_SHADER) | CP_LOAD_STATE_0_NUM_UNIT(1);
	*cmds++ = CP_LOAD_STATE_1_STATE_TYPE(ST_CONSTANTS);
	*cmds++ = 0x00000000;
	*cmds++ = 0x00000000;

	*cmds++ = CP_TYPE3_PKT | ((ARRAY_SIZE(a3xx_pwron_fixup_fs_instructions) + 1) << 16) | (CP_LOAD_STATE << 8);
	*cmds++ = CP_LOAD_STATE_0_STATE_BLOCK(SB_FRAG_SHADER) | CP_LOAD_STATE_0_NUM_UNIT(13);
	*cmds++ = 0x00000000;

	memcpy(cmds, a3xx_pwron_fixup_fs_instructions, sizeof(a3xx_pwron_fixup_fs_instructions));
	cmds += ARRAY_SIZE(a3xx_pwron_fixup_fs_instructions);

	*cmds++ = CP_TYPE3_PKT | (CP_RUN_OPENCL << 8);
	*cmds++ = 0x00000000;

	*cmds++ = CP_TYPE3_PKT | (CP_WAIT_FOR_IDLE << 8);
	*cmds++ = 0x00000000;

	*cmds++ = CP_TYPE0_PKT | REG_A3XX_HLSQ_CL_CONTROL_0_REG;
	*cmds++ = 0x00000000;

	*cmds++ = CP_TYPE3_PKT | (CP_WAIT_FOR_IDLE << 8);
	*cmds++ = 0x00000000;

	*cmds++ = CP_TYPE0_PKT | REG_A3XX_HLSQ_CONTROL_0_REG;
	*cmds++ = 0x1e000150;

	*cmds++ = CP_TYPE3_PKT | (CP_WAIT_FOR_IDLE << 8);
	*cmds++ = 0x00000000;

	*cmds++ = CP_TYPE3_PKT | (1 << 16) | (CP_SET_CONSTANT << 8);
	*cmds++ = (0x4 << 16) | (REG_A3XX_HLSQ_CONTROL_0_REG - 0x2000);
	*cmds++ = 0x1e000050;

	*cmds++ = CP_TYPE3_PKT | (CP_WAIT_FOR_IDLE << 8);
	*cmds++ = 0x00000000;

	*cmds++ = CP_TYPE3_PKT | (2 << 16) | (CP_REG_RMW << 8);
	*cmds++ = REG_A3XX_RBBM_CLOCK_CTL;
	*cmds++ = 0xfffcffff;
	*cmds++ = 0x00000000;

	*cmds++ = CP_TYPE3_PKT | (CP_WAIT_FOR_IDLE << 8);
	*cmds++ = 0x00000000;

	a3xx_gpu->pwron_fixup_dwords = cmds - start;

	return 0;
}

static struct msm_gpu *a3xx_gpu_init(struct drm_device *dev)
{
	struct a3xx_gpu *a3xx_gpu = NULL;
	struct adreno_gpu *adreno_gpu;
	struct msm_gpu *gpu;
	struct msm_drm_private *priv = dev->dev_private;
	struct platform_device *pdev = priv->gpu_pdev;
	struct adreno_platform_config *config = pdev->dev.platform_data;
	struct icc_path *ocmem_icc_path;
	struct icc_path *icc_path;
	int ret;

	if (!pdev) {
		DRM_DEV_ERROR(dev->dev, "no a3xx device\n");
		ret = -ENXIO;
		goto fail;
	}

	a3xx_gpu = kzalloc_obj(*a3xx_gpu);
	if (!a3xx_gpu) {
		ret = -ENOMEM;
		goto fail;
	}

	adreno_gpu = &a3xx_gpu->base;
	gpu = &adreno_gpu->base;

	adreno_gpu->registers = a3xx_registers;

	ret = adreno_gpu_init(dev, pdev, adreno_gpu, config->info->funcs, 1);
	if (ret)
		goto fail;

	ret = a3xx_pwron_fixup_init(gpu);
	if (ret)
		goto fail;

	/* if needed, allocate gmem: */
	if (adreno_is_a330(adreno_gpu) || adreno_is_a305b(adreno_gpu)) {
		ret = adreno_gpu_ocmem_init(&adreno_gpu->base.pdev->dev,
					    adreno_gpu, &a3xx_gpu->ocmem);
		if (ret)
			goto fail;
	}

	icc_path = devm_of_icc_get(&pdev->dev, "gfx-mem");
	if (IS_ERR(icc_path)) {
		ret = PTR_ERR(icc_path);
		goto fail;
	}

	ocmem_icc_path = devm_of_icc_get(&pdev->dev, "ocmem");
	if (IS_ERR(ocmem_icc_path)) {
		ret = PTR_ERR(ocmem_icc_path);
		/* allow -ENODATA, ocmem icc is optional */
		if (ret != -ENODATA)
			goto fail;
		ocmem_icc_path = NULL;
	}


	/*
	 * Set the ICC path to maximum speed for now by multiplying the fastest
	 * frequency by the bus width (8). We'll want to scale this later on to
	 * improve battery life.
	 */
	icc_set_bw(icc_path, 0, Bps_to_icc(gpu->fast_rate) * 8);
	icc_set_bw(ocmem_icc_path, 0, Bps_to_icc(gpu->fast_rate) * 8);

	a3xx_gpu->icc_path = icc_path;
	a3xx_gpu->ocmem_icc_path = ocmem_icc_path;

	return gpu;

fail:
	if (a3xx_gpu)
		a3xx_destroy(&a3xx_gpu->base.base);

	return ERR_PTR(ret);
}

const struct adreno_gpu_funcs a3xx_gpu_funcs = {
	.base = {
		.get_param = adreno_get_param,
		.set_param = adreno_set_param,
		.hw_init = a3xx_hw_init,
		.pm_suspend = a3xx_pm_suspend,
		.pm_resume = a3xx_pm_resume,
		.recover = a3xx_recover,
		.submit = a3xx_submit,
		.active_ring = adreno_active_ring,
		.irq = a3xx_irq,
		.destroy = a3xx_destroy,
#if defined(CONFIG_DEBUG_FS) || defined(CONFIG_DEV_COREDUMP)
		.show = adreno_show,
#endif
		.gpu_busy = a3xx_gpu_busy,
		.gpu_state_get = a3xx_gpu_state_get,
		.gpu_state_put = adreno_gpu_state_put,
		.create_vm = adreno_create_vm,
		.get_rptr = a3xx_get_rptr,
	},
	.init = a3xx_gpu_init,
};
