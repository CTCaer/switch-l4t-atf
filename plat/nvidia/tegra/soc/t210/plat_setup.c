/*
 * Copyright (c) 2015-2019, ARM Limited and Contributors. All rights reserved.
 * Copyright (c) 2020, NVIDIA Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch_helpers.h>
#include <assert.h>
#include <cortex_a57.h>
#include <common/bl_common.h>
#include <common/debug.h>
#include <common/interrupt_props.h>
#include <drivers/console.h>
#include <lib/xlat_tables/xlat_tables_v2.h>
#include <drivers/arm/gic_common.h>
#include <drivers/arm/gicv2.h>
#include <bl31/interrupt_mgmt.h>

#include <bpmp.h>
#include <flowctrl.h>
#include <memctrl.h>
#include <plat/common/platform.h>
#include <pmc.h>
#include <security_engine.h>
#include <tegra_def.h>
#include <tegra_platform.h>
#include <tegra_private.h>

/* UART clock */
typedef struct uart_clock
{
	uint32_t reset;
	uint32_t enable;
	uint32_t source;
	uint32_t mask;
} uart_clock_t;

/* sets of MMIO ranges setup */
#define MMIO_RANGE_0_ADDR	0x50000000
#define MMIO_RANGE_1_ADDR	0x60000000
#define MMIO_RANGE_2_ADDR	0x70000000
#define MMIO_RANGE_SIZE		0x200000

/*
 * Table of regions to map using the MMU.
 */
static const mmap_region_t tegra_mmap[] = {
	MAP_REGION_FLAT(TEGRA_IRAM_BASE, 0x40000, /* 256KB */
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(MMIO_RANGE_0_ADDR, MMIO_RANGE_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(MMIO_RANGE_1_ADDR, MMIO_RANGE_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(MMIO_RANGE_2_ADDR, MMIO_RANGE_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	{0}
};

/*******************************************************************************
 * Set up the pagetables as per the platform memory map & initialize the MMU
 ******************************************************************************/
const mmap_region_t *plat_get_mmio_map(void)
{
	/* Add the map region for security engine SE2 */
	if (tegra_chipid_is_t210_b01()) {
		mmap_add_region((uint64_t)TEGRA_SE2_BASE,
				(uint64_t)TEGRA_SE2_BASE,
				(uint64_t)TEGRA_SE2_RANGE_SIZE,
				MT_DEVICE | MT_RW | MT_SECURE);
	}

	/* MMIO space */
	return tegra_mmap;
}

/*******************************************************************************
 * The Tegra power domain tree has a single system level power domain i.e. a
 * single root node. The first entry in the power domain descriptor specifies
 * the number of power domains at the highest power level.
 *******************************************************************************
 */
const unsigned char tegra_power_domain_tree_desc[] = {
	/* No of root nodes */
	1,
	/* No of clusters */
	PLATFORM_CLUSTER_COUNT,
	/* No of CPU cores - cluster0 */
	PLATFORM_MAX_CPUS_PER_CLUSTER,
	/* No of CPU cores - cluster1 */
	PLATFORM_MAX_CPUS_PER_CLUSTER
};

/*******************************************************************************
 * This function returns the Tegra default topology tree information.
 ******************************************************************************/
const unsigned char *plat_get_power_domain_tree_desc(void)
{
	return tegra_power_domain_tree_desc;
}

/*******************************************************************************
 * Handler to get the System Counter Frequency
 ******************************************************************************/
unsigned int plat_get_syscnt_freq2(void)
{
	return 19200000;
}

/*******************************************************************************
 * Maximum supported UART controllers
 ******************************************************************************/
#define TEGRA210_MAX_UART_PORTS		5

/*******************************************************************************
 * This variable holds the UART port base addresses
 ******************************************************************************/
static uint32_t tegra210_uart_addresses[TEGRA210_MAX_UART_PORTS + 1] = {
	0,	/* undefined - treated as an error case */
	TEGRA_UARTA_BASE,
	TEGRA_UARTB_BASE,
	TEGRA_UARTC_BASE,
	TEGRA_UARTD_BASE,
	TEGRA_UARTE_BASE,
};

static const uart_clock_t tegra210_uart_clocks[] = {
	{ TEGRA_RST_DEVICES_L, TEGRA_CLK_OUT_ENB_L, TEGRA_CLK_SOURCE_UARTA,   UARTA_CLK_BIT },
	{ TEGRA_RST_DEVICES_L, TEGRA_CLK_OUT_ENB_L, TEGRA_CLK_SOURCE_UARTB,   UARTB_CLK_BIT },
	{ TEGRA_RST_DEVICES_H, TEGRA_CLK_OUT_ENB_H, TEGRA_CLK_SOURCE_UARTC,   UARTC_CLK_BIT },
	{ TEGRA_RST_DEVICES_U, TEGRA_CLK_OUT_ENB_U, TEGRA_CLK_SOURCE_UARTD,   UARTD_CLK_BIT },
	{ TEGRA_RST_DEVICES_Y, TEGRA_CLK_OUT_ENB_Y, TEGRA_CLK_SOURCE_UARTAPE, UARTAPE_CLK_BIT }
};

static void plat_set_uart_clock(int32_t id)
{
	uint32_t val = 0;
	uint32_t uart_idx = id - 1;
	const uart_clock_t *clk = &tegra210_uart_clocks[uart_idx];

	/* Setup uart pinmuxing */
	mmio_write_32(TEGRA_MISC_BASE + PINMUX_AUX_UARTX_TX(uart_idx), 0);
	mmio_write_32(TEGRA_MISC_BASE + PINMUX_AUX_UARTX_RX(uart_idx),
		      PINMUX_INPUT_ENABLE | PINMUX_PULL_UP);
	mmio_write_32(TEGRA_MISC_BASE + PINMUX_AUX_UARTX_RTS(uart_idx), 0);
	mmio_write_32(TEGRA_MISC_BASE + PINMUX_AUX_UARTX_CTS(uart_idx),
		      PINMUX_INPUT_ENABLE | PINMUX_PULL_UP);

	/* Assert Reset */
	val = mmio_read_32(TEGRA_CAR_RESET_BASE + clk->reset);
	val |= clk->mask;
	mmio_write_32(TEGRA_CAR_RESET_BASE + clk->reset, val);

	/* Disable clock */
	val = mmio_read_32(TEGRA_CAR_RESET_BASE + clk->enable);
	val &= ~(clk->mask);
	mmio_write_32(TEGRA_CAR_RESET_BASE + clk->enable, val);

	/* Set clock source to PLLP with divider 2 */
	mmio_write_32(TEGRA_CAR_RESET_BASE + clk->source, (0U << 29) | 2);

	/* Enable clock */
	val = mmio_read_32(TEGRA_CAR_RESET_BASE + clk->enable);
	val |= clk->mask;
	mmio_write_32(TEGRA_CAR_RESET_BASE + clk->enable, val);

	/* De-Assert Reset */
	val = mmio_read_32(TEGRA_CAR_RESET_BASE + clk->reset);
	val &= ~(clk->mask);
	mmio_write_32(TEGRA_CAR_RESET_BASE + clk->reset, val);
}

/*******************************************************************************
 * Enable console corresponding to the console ID
 ******************************************************************************/
void plat_enable_console(int32_t id)
{
	static console_t uart_console;
	uint32_t console_clock;

	if ((id > 0) && (id < TEGRA210_MAX_UART_PORTS)) {
		/*
		 * Reference clock used by the FPGAs is a lot slower.
		 */
		if (tegra_platform_is_fpga()) {
			console_clock = TEGRA_BOOT_UART_CLK_13_MHZ;
		} else {
			console_clock = TEGRA_BOOT_UART_CLK_408_MHZ;
			plat_set_uart_clock(id);
		}

		(void)console_16550_register(tegra210_uart_addresses[id],
					     console_clock,
					     TEGRA_CONSOLE_BAUDRATE,
					     &uart_console);
		console_set_scope(&uart_console, CONSOLE_FLAG_BOOT |
			CONSOLE_FLAG_RUNTIME | CONSOLE_FLAG_CRASH);
	}
}

/*******************************************************************************
 * Return pointer to the BL31 params from previous bootloader
 ******************************************************************************/
struct tegra_bl31_params *plat_get_bl31_params(void)
{
	uint32_t val = mmio_read_32(TEGRA_PMC_BASE + PMC_SECURE_SCRATCH110);

	if (val != SCRATCH_BL31_ENABLE_MAGIC)
		return NULL;

	val = mmio_read_32(TEGRA_PMC_BASE + PMC_SECURE_SCRATCH108);

	return (struct tegra_bl31_params *)(uintptr_t)val;
}

/*******************************************************************************
 * Return pointer to the BL31 platform params from previous bootloader
 ******************************************************************************/
plat_params_from_bl2_t *plat_get_bl31_plat_params(void)
{
	uint32_t val = mmio_read_32(TEGRA_PMC_BASE + PMC_SECURE_SCRATCH110);

	if (val != SCRATCH_BL31_ENABLE_MAGIC)
		return NULL;

	val = mmio_read_32(TEGRA_PMC_BASE + PMC_SECURE_SCRATCH109);

	return (plat_params_from_bl2_t *)(uintptr_t)val;
}

/*******************************************************************************
 * Handler for early platform setup
 ******************************************************************************/
void plat_early_platform_setup(void)
{
	const plat_params_from_bl2_t *plat_params = bl31_get_plat_params();
	uint64_t val;
	uint32_t cntcr;

	/* Verify chip id is t210 */
	assert(tegra_chipid_is_t210());

	/*
	 * Do initial security configuration to allow DRAM/device access.
	 */
	tegra_memctrl_tzdram_setup(plat_params->tzdram_base,
			(uint32_t)plat_params->tzdram_size);

	/* platform parameter passed by the previous bootloader */
	if (plat_params->l2_ecc_parity_prot_dis != 1) {
		/* Enable ECC Parity Protection for Cortex-A57 CPUs */
		val = read_l2ctlr_el1();
		val |= (uint64_t)CORTEX_A57_L2_ECC_PARITY_PROTECTION_BIT;
		write_l2ctlr_el1(val);
	}

	/* Ensure that system counter is enabled in case of coldboot ATF. */
	cntcr = mmio_read_32(TEGRA_SYSCTR0_BASE + SYSCTR0_CNTCR);
	cntcr |= SYSCTR0_CNTCR_EN | SYSCTR0_CNTCR_HDBG;
	mmio_write_32(TEGRA_SYSCTR0_BASE + SYSCTR0_CNTCR, cntcr);

	/* Initialize security engine driver */
	tegra_se_init();
}

/* Secure IRQs for Tegra186 */
static const interrupt_prop_t tegra210_interrupt_props[] = {
	INTR_PROP_DESC(TEGRA_SDEI_SGI_PRIVATE, PLAT_SDEI_CRITICAL_PRI,
			GICV2_INTR_GROUP0, GIC_INTR_CFG_EDGE),
	INTR_PROP_DESC(TEGRA210_TIMER1_IRQ, PLAT_TEGRA_WDT_PRIO,
			GICV2_INTR_GROUP0, GIC_INTR_CFG_EDGE),
	INTR_PROP_DESC(TEGRA210_WDT_CPU_LEGACY_FIQ, PLAT_TEGRA_WDT_PRIO,
			GICV2_INTR_GROUP0, GIC_INTR_CFG_EDGE),
};

/*******************************************************************************
 * Handler for late platform setup
 ******************************************************************************/
void plat_late_platform_setup(void)
{
	const plat_params_from_bl2_t *plat_params = bl31_get_plat_params();
	uint32_t *iram_entry_op = (uint32_t *)(TEGRA_IRAM_BASE + TEGRA_IRAM_A_SIZE);
	uint64_t sc7entry_end, r2p_payload_end;
	uint64_t offset = 0;
	int ret;
	uint32_t val;

	/* memmap TZDRAM area containing the r2p payload firmware */
	if (plat_params->r2p_payload_base && plat_params->r2p_payload_size) {
		/* r2p payload must be _before_ BL31 base */
		assert(plat_params->tzdram_base > plat_params->r2p_payload_base);

		r2p_payload_end = plat_params->r2p_payload_base +
			       plat_params->r2p_payload_size;
		assert(r2p_payload_end < plat_params->tzdram_base);

		/* r2p payload start must be exactly 256KB behind BL31 base */
		offset = plat_params->tzdram_base - plat_params->r2p_payload_base;

		/* memmap r2p payload firmware code */
		ret = mmap_add_dynamic_region(plat_params->r2p_payload_base,
				plat_params->r2p_payload_base,
				plat_params->r2p_payload_size,
				MT_SECURE | MT_RO_DATA);
		assert(ret == 0);

		/* clear IRAM entry OP in IRAM */
		*iram_entry_op = 0;


		/* check if sc7entry firmware is missing */
		if (!plat_params->sc7entry_fw_base || !plat_params->sc7entry_fw_size) {
			/* setup secure TZDRAM area, increased by 1MB */
			tegra_memctrl_tzdram_setup(plat_params->tzdram_base - 0x100000,
						   plat_params->tzdram_size + 0x100000);
		}
	}

	/* memmap TZDRAM area containing the SC7 Entry Firmware */
	if (plat_params->sc7entry_fw_base && plat_params->sc7entry_fw_size) {
		assert(plat_params->sc7entry_fw_size <= TEGRA_IRAM_A_SIZE);

		/*
		 * Verify that the SC7 entry firmware resides inside the TZDRAM
		 * aperture, _before_ the BL31 code and the start address is
		 * exactly 1MB from BL31 base.
		 */

		/* sc7entry-fw must be _before_ r2p payload base */
		assert(plat_params->tzdram_base - offset > plat_params->sc7entry_fw_base);

		sc7entry_end = plat_params->sc7entry_fw_base +
			       plat_params->sc7entry_fw_size;
		assert(sc7entry_end < plat_params->tzdram_base - offset);

		/* sc7entry-fw start must be exactly 1MB behind BL31 base */
		offset = plat_params->tzdram_base - plat_params->sc7entry_fw_base;
		assert(offset == 0x100000);

		/* secure TZDRAM area */
		tegra_memctrl_tzdram_setup(plat_params->sc7entry_fw_base,
			plat_params->tzdram_size + offset);

		/* power off BPMP processor until SC7 entry */
		tegra_fc_bpmp_off();

		/* memmap SC7 entry firmware code */
		ret = mmap_add_dynamic_region(plat_params->sc7entry_fw_base,
				plat_params->sc7entry_fw_base,
				plat_params->sc7entry_fw_size,
				MT_SECURE | MT_RO_DATA);
		assert(ret == 0);

		/* restrict PMC access to secure world */
		if (!(plat_params->flags & TEGRA_PLAT_PMC_NON_SECURE)) {
			val = mmio_read_32(TEGRA_MISC_BASE + APB_SLAVE_SECURITY_ENABLE);
			val |= PMC_SECURITY_EN_BIT;
			mmio_write_32(TEGRA_MISC_BASE + APB_SLAVE_SECURITY_ENABLE, val);
		}
	}

	if (!(plat_params->flags & TEGRA_PLAT_PMC_NON_SECURE) &&
	    !tegra_chipid_is_t210_b01()) {
		/* restrict PMC access to secure world */
		val = mmio_read_32(TEGRA_MISC_BASE + APB_SLAVE_SECURITY_ENABLE);
		val |= PMC_SECURITY_EN_BIT;
		mmio_write_32(TEGRA_MISC_BASE + APB_SLAVE_SECURITY_ENABLE, val);
	}

	/* clear pmc reset type from before */
	tegra_pmc_clear_reset_type();
}

/*******************************************************************************
 * Initialize the GIC and SGIs
 ******************************************************************************/
void plat_gic_setup(void)
{
	tegra_gic_setup(tegra210_interrupt_props, ARRAY_SIZE(tegra210_interrupt_props));
	tegra_gic_init();

	/* Enable handling for FIQs */
	tegra_fiq_handler_setup();

	/*
	 * Enable routing watchdog FIQs from the flow controller to
	 * the GICD.
	 */
	tegra_fc_enable_fiq_to_ccplex_routing();
}
/*******************************************************************************
 * Handler to indicate support for System Suspend
 ******************************************************************************/
bool plat_supports_system_suspend(void)
{
	const plat_params_from_bl2_t *plat_params = bl31_get_plat_params();

	/*
	 * sc7entry-fw is only supported by Tegra210 SoCs.
	 */
	if (!tegra_chipid_is_t210_b01() && (plat_params->sc7entry_fw_base != 0U)) {
		return true;
	} else if (tegra_chipid_is_t210_b01()) {
		return true;
	} else {
		return false;
	}
}
/*******************************************************************************
 * Platform specific runtime setup.
 ******************************************************************************/
void plat_runtime_setup(void)
{
	/*
	 * During cold boot, it is observed that the arbitration
	 * bit is set in the Memory controller leading to false
	 * error interrupts in the non-secure world. To avoid
	 * this, clean the interrupt status register before
	 * booting into the non-secure world
	 */
	tegra_memctrl_clear_pending_interrupts();

	/*
	 * During boot, USB3 and flash media (SDMMC/SATA) devices need
	 * access to IRAM. Because these clients connect to the MC and
	 * do not have a direct path to the IRAM, the MC implements AHB
	 * redirection during boot to allow path to IRAM. In this mode
	 * accesses to a programmed memory address aperture are directed
	 * to the AHB bus, allowing access to the IRAM. This mode must be
	 * disabled before we jump to the non-secure world.
	 */
	tegra_memctrl_disable_ahb_redirection();
}
