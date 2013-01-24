/*
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2010 LGE. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/bootmem.h>
#include <linux/power_supply.h>


#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/setup.h>
#ifdef CONFIG_CACHE_L2X0
#include <asm/hardware/cache-l2x0.h>
#endif

#include <asm/mach/mmc.h>
#include <mach/vreg.h>
#include <mach/mpp.h>
#include <mach/board.h>
#include <mach/pmic.h>
#include <mach/msm_iomap.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>
#include <mach/rpc_pmapp.h>
#include <mach/msm_serial_hs.h>
#include <mach/memory.h>
#include <mach/msm_battery.h>
#include <mach/rpc_server_handset.h>
#include <mach/msm_tsif.h>

#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/i2c.h>
#include <linux/android_pmem.h>
#include <mach/camera.h>

#include "devices.h"
#include <mach/socinfo.h>
#include "clock.h"
#include "msm-keypad-devices.h"
#include "pm.h"
#ifdef CONFIG_ARCH_MSM7X27
#include <linux/msm_kgsl.h>
#endif
#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android_composite.h>
#endif
#include <mach/board_lge.h>
#include "board-gelato.h"

/* board-specific pm tuning data definitions */

/* currently, below declaration code is blocked.
 * if power management tuning is required in any board,
 * below "msm7x27_pm_data" array can be redefined and can be unblocked.
 * qualocomm's default setting value is configured in devices_lge.c
 * but that variable is declared in weak attribute
 * so board specific configuration can be redefined like "over riding" in OOP
 */
extern struct msm_pm_platform_data msm7x25_pm_data[MSM_PM_SLEEP_MODE_NR];
extern struct msm_pm_platform_data msm7x27_pm_data[MSM_PM_SLEEP_MODE_NR];
#if 0
struct msm_pm_platform_data msm7x27_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 16000,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].residency = 20000,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency = 12000,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].residency = 20000,

	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].suspend_enabled
		= 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 2000,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].residency = 0,
};
#endif

static struct platform_device *devices[] __initdata = {
	&msm_device_smd,
	&msm_device_dmov,
	&msm_device_nand,
	&msm_device_i2c,
	&msm_device_uart_dm1,
	&msm_device_snd,
	&msm_device_adspdec,
};

extern struct sys_timer msm_timer;

static void __init msm7x2x_init_irq(void)
{
	msm_init_irq();
}

static struct msm_acpu_clock_platform_data msm7x2x_clock_data = {
	.acpu_switch_time_us = 50,
	.max_speed_delta_khz = 400000,
	.vdd_switch_time_us = 62,
	.max_axi_khz = 160000,
};

void msm_serial_debug_init(unsigned int base, int irq,
			   struct device *clk_device, int signal_irq);

static void msm7x27_wlan_init(void)
{
	int rc = 0;
	/* TBD: if (machine_is_msm7x27_ffa_with_wcn1312()) */
	if (machine_is_msm7x27_ffa()) {
		rc = mpp_config_digital_out(3, MPP_CFG(MPP_DLOGIC_LVL_MSMP,
				MPP_DLOGIC_OUT_CTRL_LOW));
		if (rc)
			printk(KERN_ERR "%s: return val: %d \n",
				__func__, rc);
	}
}

/* decrease FB pmem size because gelato uses hvga
 * qualcomm's original value depends on wvga resolution
 * 2010-04-18, cleaneye.kim@lge.com
 */
unsigned pmem_fb_size = 	0x96000;
unsigned pmem_adsp_size =	0xAE4000; 

static void __init msm7x2x_init(void)
{
	if (socinfo_init() < 0)
		BUG();

#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	msm_serial_debug_init(MSM_UART3_PHYS, INT_UART3,
			&msm_device_uart3.dev, 1);
#endif


	msm_add_fb_device();

#if !defined(CONFIG_MSM_SERIAL_DEBUGGER)
	if (lge_get_uart_mode())
		platform_device_register(&msm_device_uart3);
#endif

	platform_add_devices(devices, ARRAY_SIZE(devices));
#ifdef CONFIG_ARCH_MSM7X27
	msm_add_kgsl_device();
#endif
	msm_add_usb_devices();

#ifdef CONFIG_MSM_CAMERA
	config_camera_off_gpios(); /* might not be necessary */
#endif
	msm_device_i2c_init();
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));

	if (cpu_is_msm7x27())
		msm_pm_set_platform_data(msm7x27_pm_data,
					ARRAY_SIZE(msm7x27_pm_data));
	else
		msm_pm_set_platform_data(msm7x25_pm_data,
					ARRAY_SIZE(msm7x25_pm_data));
	msm7x27_wlan_init();

#ifdef CONFIG_ANDROID_RAM_CONSOLE
	lge_add_ramconsole_devices();
	lge_add_ers_devices();
	lge_add_panic_handler_devices();
#endif
	lge_add_camera_devices();
	lge_add_lcd_devices();
	lge_add_btpower_devices();
	lge_add_mmc_devices();
	lge_add_input_devices();
	lge_add_misc_devices();
	lge_add_pm_devices();
	
	/* gpio i2c devices should be registered at latest point */
	lge_add_gpio_i2c_devices();
}


MACHINE_START(MSM7X27_GELATO, "GELATO Global board (LGE LGP690)")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= PHYS_OFFSET + 0x100,
	.init_irq		= msm7x2x_init_irq,
	.init_machine	= msm7x2x_init,
	.timer			= &msm_timer,
MACHINE_END
