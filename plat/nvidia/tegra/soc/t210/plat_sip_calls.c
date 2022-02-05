/*
 * Copyright (c) 2018, ARM Limited and Contributors. All rights reserved.
 * Copyright (c) 2020, NVIDIA Corporation. All rights reserved.
 * Copyright (c) 2019, Ezekiel Bethel.
 * Copyright (c) 2021, CTCaer.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <inttypes.h>
#include <stdint.h>

#include <arch.h>
#include <arch_helpers.h>
#include <assert.h>
#include <common/bl_common.h>
#include <common/debug.h>
#include <common/runtime_svc.h>
#include <errno.h>
#include <lib/mmio.h>
#include <lib/utils.h>
#include <lib/utils_def.h>

#include <memctrl.h>
#include <pmc.h>
#include <tegra_private.h>
#include <tegra_platform.h>
#include <tegra_def.h>

/*******************************************************************************
 * PMC parameters
 ******************************************************************************/
#define PMC_READ 			U(0xaa)
#define PMC_WRITE 			U(0xbb)

/*******************************************************************************
 * EMC parameters
 ******************************************************************************/
#define EMC_TABLE_ADDR			U(0xaa)
#define EMC_TABLE_SIZE			U(0xbb)

/*******************************************************************************
 * Tegra210 SiP SMCs
 ******************************************************************************/
#define TEGRA_SIP_PMC_COMMANDS		U(0xC2FFFE00)
#define TEGRA_SIP_EMC_COMMANDS		U(0xC2FFFE01)
#define TEGRA_SIP_R2P_COPY_TO_IRAM	U(0xC2FFFE02)
#define  R2P_WRITE_IRAM			(0U << 0)
#define  R2P_READ_IRAM			(1U << 0)
#define TEGRA_SIP_R2P_DO_REBOOT		U(0xC2FFFE03)

/*******************************************************************************
 * R2P Copy to IRAM SMC implementation
 ******************************************************************************/
static int r2p_iram_copy(uint64_t dram_addr, uint64_t iram_addr, uint32_t size, uint32_t flag)
{
	volatile uint32_t *src, *dst;
	uint64_t dram_aligned_addr = TEGRA_ALIGN_DOWN(dram_addr, PAGE_SIZE);
	uint32_t dram_aligned_size = TEGRA_ALIGN(size + (dram_addr - dram_aligned_addr), PAGE_SIZE);

	/* Error out if invalid or not 32-bit aligned */
	if (dram_addr < TEGRA_DRAM_BASE || iram_addr < TEGRA_IRAM_BASE ||
	    (iram_addr + size) > (TEGRA_IRAM_BASE + TEGRA_IRAM_SIZE) ||
	    dram_addr & 3 || iram_addr & 3 || size & 3) {
		ERROR("%s: Invalid parameters dst: %x, src: %x, size: %x\n",
			__func__, (uint32_t)dram_addr, (uint32_t)iram_addr, size);

		return -EINVAL;
	}

	/* Map source memory region */
	mmap_add_dynamic_region(dram_aligned_addr,
							dram_aligned_addr,
							dram_aligned_size,
							MT_MEMORY | MT_RW | MT_NS | MT_EXECUTE_NEVER);

	if (flag & R2P_READ_IRAM) {
		src = (uint32_t *)iram_addr;
		dst = (uint32_t *)dram_addr;
	} else {
		src = (uint32_t *)dram_addr;
		dst = (uint32_t *)iram_addr;
	}

	/* Copy payload */
	(void)memcpy((void *)(uintptr_t)dst, (const void *)src, size);

	/* Flush data and remove source memory region */
	flush_dcache_range((uintptr_t)dst, size);
	mmap_remove_dynamic_region(dram_aligned_addr, dram_aligned_size);

	return 0;
}

/*******************************************************************************
 * This function is responsible for copying default payload if current is empty
 ******************************************************************************/

static void r2p_ensure_payload_exists(const plat_params_from_bl2_t *plat_params)
{
	uint64_t payload_address = TEGRA_IRAM_BASE + TEGRA_IRAM_A_SIZE;
	uint32_t payload_entry_op = *(uint32_t *)(payload_address);

	if (payload_entry_op == 0) {
		if(plat_params->r2p_payload_base && plat_params->r2p_payload_size) {
			WARN("R2P payload is empty! Using default..\n");

			/* Clean up IRAM of any cruft */
			zeromem((void *)(uintptr_t)TEGRA_IRAM_BASE,
					TEGRA_IRAM_SIZE);

			/* Copy default r2p payload */
			(void)memcpy((void *)(uintptr_t)payload_address,
				(const void *)(plat_params->r2p_payload_base),
				plat_params->r2p_payload_size);
		} else {
			ERROR("R2P payload is empty! Rebooting normally..\n");
			tegra_pmc_system_reset();
		}
	}
}

/*******************************************************************************
 * R2P Reboot to Payload SMC implementation
 ******************************************************************************/
void r2p_reboot_to_payload()
{
	uint32_t val;
	const plat_params_from_bl2_t *plat_params = bl31_get_plat_params();

	/* Make sure we have a payload in place */
	r2p_ensure_payload_exists(plat_params);

	/* Set reset type to warmboot */
	val = tegra_pmc_read_32(PMC_SCRATCH0) | PMC_SCRATCH0_MODE_WARMBOOT;
	tegra_pmc_write_32(PMC_SCRATCH0, val);

	/* Patch SDRAM init to perform an SVC immediately after second write */
	tegra_pmc_write_32(PMC_SCRATCH45, 0x2E38DFFF);
	tegra_pmc_write_32(PMC_SCRATCH46, 0x6001DC28);

	/* Set SVC handler to jump to payload in IRAM */
	tegra_pmc_write_32(PMC_SCRATCH33, 0x40010000);
	tegra_pmc_write_32(PMC_SCRATCH40, 0x6000F208);

	/*
	 * Mark PMC as accessible to the non-secure world since bootloaders
	 * will need it to be accessible
	 */
	val = mmio_read_32(TEGRA_MISC_BASE + APB_SLAVE_SECURITY_ENABLE);
	val &= ~PMC_SECURITY_EN_BIT;
	mmio_write_32(TEGRA_MISC_BASE + APB_SLAVE_SECURITY_ENABLE, val);

	/* Allow non-secure writes to reset vectors for SC7Exit */
	plat_secure_cpu_vectors(false);

	tegra_pmc_system_reset();
}

/*******************************************************************************
 * This function is responsible for handling all T210 SiP calls
 ******************************************************************************/
int plat_sip_handler(uint32_t smc_fid,
		     uint64_t x1,
		     uint64_t x2,
		     uint64_t x3,
		     uint64_t x4,
		     const void *cookie,
		     void *handle,
		     uint64_t flags)
{
	uint32_t val, ns;
	const plat_params_from_bl2_t *plat_params = bl31_get_plat_params();

	/* Determine which security state this SMC originated from */
	ns = is_caller_non_secure(flags);
	if (!ns)
		SMC_RET1(handle, SMC_UNK);

	if (smc_fid == TEGRA_SIP_PMC_COMMANDS) {
		/* check the address is within PMC range and is 4byte aligned */
		if ((x2 >= TEGRA_PMC_SIZE) || (x2 & 0x3))
			return -EINVAL;

		switch (x2) {
		/* Black listed PMC registers */
		case PMC_SCRATCH1:
		case PMC_SCRATCH31 ... PMC_SCRATCH33:
		case PMC_SCRATCH40:
		case PMC_SCRATCH42:
		case PMC_SCRATCH43 ... PMC_SCRATCH48:
		case PMC_SCRATCH50 ... PMC_SCRATCH51:
		case PMC_SCRATCH56 ... PMC_SCRATCH57:
		/* PMC secure-only registers are not accessible */
		case PMC_DPD_ENABLE_0:
		case PMC_FUSE_CONTROL_0:
		case PMC_CRYPTO_OP_0:
		case PMC_TSC_MULT_0:
		case PMC_STICKY_BIT:
			ERROR("%s: error offset=0x%" PRIx64 "\n", __func__, x2);
			return -EFAULT;
		default:
			/* Valid register */
			break;
		}

		/* Perform PMC read/write */
		if (x1 == PMC_READ) {
			val = mmio_read_32((uint32_t)(TEGRA_PMC_BASE + x2));
			write_ctx_reg(get_gpregs_ctx(handle), CTX_GPREG_X1, val);
		} else if (x1 == PMC_WRITE) {
			mmio_write_32((uint32_t)(TEGRA_PMC_BASE + x2), (uint32_t)x3);
		} else {
			return -EINVAL;
		}
	} else if (smc_fid == TEGRA_SIP_EMC_COMMANDS) {
		if (x1 == EMC_TABLE_ADDR && plat_params->emc_table_base) {
			val = plat_params->emc_table_base;
			write_ctx_reg(get_gpregs_ctx(handle), CTX_GPREG_X1, val);
		} else if (x1 == EMC_TABLE_SIZE && plat_params->emc_table_size) {
			val = plat_params->emc_table_size;
			write_ctx_reg(get_gpregs_ctx(handle), CTX_GPREG_X1, val);
		} else {
			return -EINVAL;
		}
	} else if (smc_fid == TEGRA_SIP_R2P_COPY_TO_IRAM) {
		return r2p_iram_copy(x1, x2, x3, x4);
	} else if (smc_fid == TEGRA_SIP_R2P_DO_REBOOT) {
		r2p_reboot_to_payload();
	} else {
		return -ENOTSUP;
	}
	return 0;
}
