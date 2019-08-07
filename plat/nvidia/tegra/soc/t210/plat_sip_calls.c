/*
 * Copyright (c) 2018, ARM Limited and Contributors. All rights reserved.
 * Copyright (c) 2020, NVIDIA Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch.h>
#include <arch_helpers.h>
#include <assert.h>
#include <common/bl_common.h>
#include <common/debug.h>
#include <common/runtime_svc.h>
#include <errno.h>
#include <lib/mmio.h>
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
#define EMC_TABLE_ADDR		U(0xaa)
#define EMC_TABLE_SIZE		U(0xbb)

/*******************************************************************************
 * Tegra210 SiP SMCs
 ******************************************************************************/
#define TEGRA_SIP_PMC_COMMANDS		U(0xC2FFFE00)
#define TEGRA_SIP_EMC_COMMANDS		U(0xC2FFFE01)

#define ATMOSPHERE_COPY_TO_IRAM    U(0xC2FFFE02)
#define ATMOSPHERE_DO_REBOOT    U(0xC2FFFE03)

/*******************************************************************************
 copy to iram smc implementation
 ******************************************************************************/
int ams_iram_copy(uint64_t dram_addr, uint64_t iram_addr, uint32_t size, uint32_t flag)
{
	// Error out if either address or the size isn't dword aligned
	if (dram_addr & 3 || iram_addr & 3 || size & 3)
		return -EINVAL;

	uint64_t dram_page = dram_addr & ~0xfff; // align down
	uint32_t dram_aligned_size = (size + (dram_addr - dram_page) + 0xfff) & ~0xfff; // align up, make sure to cover entire area.
	mmap_add_dynamic_region(dram_page, dram_page, dram_aligned_size, MT_MEMORY | MT_RW | MT_NS | MT_EXECUTE_NEVER);

	volatile uint32_t *src, *dst;
	size_t num_dwords = size / 4;

	if (flag == 0) { // write IRAM
		src = (uint32_t*) dram_addr;
		dst = (uint32_t*) iram_addr;
	} else { // read iram
		src = (uint32_t*) iram_addr;
		dst = (uint32_t*) dram_addr;
	}

	for (size_t i = 0; i < num_dwords; i++)
		dst[i] = src[i];

	flush_dcache_range((uintptr_t)dst, size);
	mmap_remove_dynamic_region(dram_page, dram_aligned_size);

	return 0;
}

void ams_reboot_to_payload()
{
	const plat_params_from_bl2_t *plat_params = bl31_get_plat_params();
	uint32_t val;
	// This is basically ripped out of exosphere - https://github.com/Atmosphere-NX/Atmosphere/blob/master/exosphere/src/configitem.c#L58

	// Set reboot kind = warmboot. Make sure not to clobber the other bits in scratch0.
	tegra_pmc_write_32(0x50, tegra_pmc_read_32(0x50) | 1);
	// Patch SDRAM init to perform an SVC immediately after second write
	tegra_pmc_write_32(0x234, 0x2E38DFFF);
	tegra_pmc_write_32(0x238, 0x6001DC28);
	// Set SVC handler to jump to reboot stub in IRAM.
	tegra_pmc_write_32(0x120, 0x4003F000);
	tegra_pmc_write_32(0x13C, 0x6000F208);

	void *rebootstub_iram = (void *)(uintptr_t)(TEGRA_IRAM_BASE + 0x3F000);
	memcpy(rebootstub_iram, (void*)plat_params->rebootstub_base, plat_params->rebootstub_size);

	flush_dcache_range((uintptr_t)rebootstub_iram, plat_params->rebootstub_size);

	/*
	 * Mark PMC as accessible to the non-secure world since bootloaders
	 * will need it to be accessible
	 */
	val = mmio_read_32(TEGRA_MISC_BASE + APB_SLAVE_SECURITY_ENABLE);
	val &= ~PMC_SECURITY_EN_BIT;
	mmio_write_32(TEGRA_MISC_BASE + APB_SLAVE_SECURITY_ENABLE, val);

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
	uint64_t val2;
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
			ERROR("%s: error offset=0x%llx\n", __func__, x2);
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
		if (x1 == EMC_TABLE_ADDR) {
			val2 = plat_params->emc_table_base;
			write_ctx_reg(get_gpregs_ctx(handle), CTX_GPREG_X1, val2);
		} else if (x1 == EMC_TABLE_SIZE) {
			val2 = plat_params->emc_table_size;
			write_ctx_reg(get_gpregs_ctx(handle), CTX_GPREG_X1, val2);
		} else {
			return -EINVAL;
		}
	} else if (ATMOSPHERE_COPY_TO_IRAM) {
		return ams_iram_copy(x1, x2, x3, x4);
	} else if (ATMOSPHERE_DO_REBOOT) {
		ams_reboot_to_payload();
	} else {
		return -ENOTSUP;
	}
	return 0;
}
