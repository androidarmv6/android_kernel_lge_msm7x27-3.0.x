/* linux/arch/arm/mach-msm/lge/devices_lge.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (C) 2009-2010 LGE.
 * Author: SungEun Kim <cleaneye.kim@lge.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <asm/setup.h>
#include <asm/mach/mmc.h>
#include <mach/msm_memtypes.h>
#include <asm/mach-types.h>
#include <mach/vreg.h>
#include <mach/mpp.h>
#include <mach/board.h>
#include <mach/pmic.h>
#include <mach/msm_iomap.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>
#include <mach/rpc_pmapp.h>
#include <linux/android_pmem.h>
#ifdef CONFIG_USB_FUNCTION
#include <linux/usb/mass_storage_function.h>
#include <linux/usb/android_composite.h>
#endif
#ifdef CONFIG_USB_G_ANDROID
#include <linux/usb/android.h>
#include <mach/usbdiag.h>
#endif
#ifdef CONFIG_ANDROID_RAM_CONSOLE
#include <asm/setup.h>
#endif
#include <mach/board_lge.h>
#include "../devices.h"
#include "../pm.h"
#include <mach/socinfo.h>

/* setting board revision information */
int lge_bd_rev;

static int __init board_revno_setup(char *rev_info)
{
	char *rev_str[] = { "evb", "rev_a", "rev_b", "rev_c", "rev_d", "rev_e", "rev_f",
#if defined(CONFIG_MACH_MSM7X27_PECAN)
		"rev_g",
#endif
		"rev_10","rev_11","rev_12","rev_13",};
	int i;

	lge_bd_rev = LGE_REV_TOT_NUM;
	
	for (i = 0; i < LGE_REV_TOT_NUM; i++) 
		if (!strncmp(rev_info, rev_str[i], 6)) {
			lge_bd_rev = i;
			break;
		}

	printk(KERN_INFO"BOARD: LGE %s\n", rev_str[lge_bd_rev]);

	return 1;
}

__setup("lge.rev=", board_revno_setup);

/* setting whether uart console is enalbed or disabled */
static int uart_console_mode = 0;

int __init lge_get_uart_mode(void)
{
	return uart_console_mode;
}

static int __init lge_uart_mode(char *uart_mode)
{
	if (!strncmp("enable", uart_mode, 5)) {
		printk(KERN_INFO"UART CONSOLE : enable\n");
		uart_console_mode = 1;
	} 
	else 
		printk(KERN_INFO"UART CONSOLE : disable\n");

	return 1;
}

__setup("uart_console=", lge_uart_mode);

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct resource ram_console_resource[] = {
	{
		.name = "ram_console",
		.flags = IORESOURCE_MEM,
	}
};

static struct platform_device ram_console_device = {
	.name = "ram_console",
	.id = -1,
	.num_resources = ARRAY_SIZE(ram_console_resource),
	.resource = ram_console_resource,
};

void __init lge_add_ramconsole_devices(void)
{
	struct resource *res = ram_console_resource;
	struct membank *bank = &meminfo.bank[0];
	res->start = MSM7X27_EBI1_CS0_BASE + bank->size;
	res->end = res->start + LGE_RAM_CONSOLE_SIZE - 1;
	printk("RAM CONSOLE START ADDR : %d\n", res->start);
	printk("RAM CONSOLE END ADDR   : %d\n", res->end);
	
	platform_device_register(&ram_console_device);
}

__WEAK struct lge_panic_handler_platform_data panic_handler_data;

static struct platform_device panic_handler_device = {
	.name = "panic-handler",
	.dev    = {
		.platform_data = &panic_handler_data,
	}
};

void __init lge_add_panic_handler_devices(void)
{
	platform_device_register(&panic_handler_device);
}

static struct platform_device ers_kernel = {
	.name = "ers-kernel",
};

void __init lge_add_ers_devices(void)
{
	platform_device_register(&ers_kernel);
}

#endif




// this is only a temp home until I can get to the right place
static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = 97,
	.mdp_rev = MDP_REV_30,
};


//


static void *fb_copy_virt;
void *lge_get_fb_copy_virt_addr(void)
{
	return (void *)fb_copy_virt;
}
static size_t fb_copy_phys;
static size_t fb_copy_size;

void *lge_get_fb_copy_phys_addr(void)
{
	return (void *)fb_copy_phys;
}
static void __init lge_make_fb_pmem(void)
{
	struct membank *bank = &meminfo.bank[0];
	unsigned *temp;

	fb_copy_phys = MSM7X27_EBI1_CS0_BASE + bank->size + LGE_RAM_CONSOLE_SIZE;
	/* Modify HIDDEN_RSEET_FB_SIZE defined in board_lge.h */
	fb_copy_size = HIDDEN_RESET_FB_SIZE;

	fb_copy_virt = ioremap(fb_copy_phys, fb_copy_size);

	temp = fb_copy_virt;
	*temp = 0x0;

	printk("FB START PHYS ADDR : %x\n", fb_copy_phys);
	printk("FB START VIRT ADDR : %x\n", (unsigned)fb_copy_virt);
	printk("FB SIZE : %x\n", fb_copy_size);

	return;
}
void __init msm_add_fb_device(void) 
{
	lge_make_fb_pmem();
	platform_device_register(&msm_fb_device);
}

/* kgsl moved to mach-msm/devices-msm7x27.c since .38 */

static struct platform_device *pmem_devices[] __initdata = {
	&android_pmem_device,
	&android_pmem_adsp_device,
	&android_pmem_audio_device,
};

static unsigned pmem_kernel_ebi1_size = PMEM_KERNEL_EBI1_SIZE;
static void __init pmem_kernel_ebi1_size_setup(char *p)
{
	pmem_kernel_ebi1_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_kernel_ebi1_size", pmem_kernel_ebi1_size_setup);

static unsigned pmem_mdp_size = MSM_PMEM_MDP_SIZE;
static int __init pmem_mdp_size_setup(char *p)
{
	pmem_mdp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_mdp_size=", pmem_mdp_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_adsp_size=", pmem_adsp_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;
static int __init pmem_audio_size_setup(char *p)
{
	pmem_audio_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_audio_size", pmem_audio_size_setup);

static unsigned pmem_fb_size = MSM_FB_SIZE;
static int __init fb_size_setup(char *p)
{
	pmem_fb_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_fb_size=", fb_size_setup);


static struct memtype_reserve msm7x27_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

static void __init size_pmem_devices(void)
{
#ifdef CONFIG_ANDROID_PMEM
	android_pmem_adsp_pdata.size = pmem_adsp_size;
	android_pmem_pdata.size = pmem_mdp_size;
	android_pmem_audio_pdata.size = pmem_audio_size;
#endif
}

static void __init reserve_memory_for(struct android_pmem_platform_data *p)
{
	msm7x27_reserve_table[p->memory_type].size += p->size;
}

static void __init reserve_pmem_memory(void)
{
#ifdef CONFIG_ANDROID_PMEM
	reserve_memory_for(&android_pmem_adsp_pdata);
	reserve_memory_for(&android_pmem_pdata);
	reserve_memory_for(&android_pmem_audio_pdata);
	msm7x27_reserve_table[MEMTYPE_EBI1].size += pmem_kernel_ebi1_size;
#endif
}

static void __init msm7x27_calculate_reserve_sizes(void)
{
	size_pmem_devices();
	reserve_pmem_memory();
}

static int msm7x27_paddr_to_memtype(unsigned int paddr)
{
	return MEMTYPE_EBI1;
}

static struct reserve_info msm7x27_reserve_info __initdata = {
	.memtype_reserve_table = msm7x27_reserve_table,
	.calculate_reserve_sizes = msm7x27_calculate_reserve_sizes,
	.paddr_to_memtype = msm7x27_paddr_to_memtype,
};

static void __init msm7x27_reserve(void)
{
	reserve_info = &msm7x27_reserve_info;
	msm_reserve();
}

void __init msm_add_pmem_devices(void)
{
	platform_add_devices(pmem_devices, ARRAY_SIZE(pmem_devices));
}

/* setting power management configuration of msm7x25 */
__WEAK struct msm_pm_platform_data msm7x25_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 16000,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency = 12000,

	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 2000,
};

/* setting power management configuration of msm7x27 */
__WEAK struct msm_pm_platform_data msm7x27_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] = {
	.idle_supported = 1,
	.suspend_supported = 1,
	.suspend_enabled = 1,
	.idle_enabled = 1,
	.latency = 16000,
	.residency = 20000,
	},
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN] = {
	.idle_supported = 1,
	.suspend_supported = 1,
	.suspend_enabled = 1,
	.idle_enabled = 1,
	.latency = 12000,
	.residency = 20000,
	},
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT] = {
	.idle_supported = 1,
	.suspend_supported = 1,
	.suspend_enabled = 1,
	.idle_enabled = 1,
	.latency = 2000,
	.residency = 0,
	
};

static struct msm_pm_boot_platform_data msm_pm_boot_pdata __initdata = {
	.mode = MSM_PM_BOOT_CONFIG_RESET_VECTOR_PHYS,
	.p_addr = 0,
};
/* setting msm i2c device */
static void
msm_i2c_gpio_config(int iface, int config_type)
{
	int gpio_scl;
	int gpio_sda;
	if (iface) {
		gpio_scl = 95;
		gpio_sda = 96;
	} else {
		gpio_scl = 60;
		gpio_sda = 61;
	}
	if (config_type) {
		gpio_tlmm_config(GPIO_CFG(gpio_scl, 1, GPIO_CFG_INPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_sda, 1, GPIO_CFG_INPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	} else {
		gpio_tlmm_config(GPIO_CFG(gpio_scl, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_sda, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	}
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
	.clk_freq = 400000,
	.rmutex  = 0,
	.pri_clk = 60,
	.pri_dat = 61,
	.aux_clk = 95,
	.aux_dat = 96,
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};
#if 0 //TODO: once this is booted, port msm7627-regulator for msm7x27
static struct platform_device msm_proccomm_regulator_dev = {
	.name   = PROCCOMM_REGULATOR_DEV_NAME,
	.id     = -1,
	.dev    = {
	  .platform_data = &msm7627_proccomm_regulator_data
	}
};

static void __init msm7627_init_regulators(void)
{
	int rc = platform_device_register(&msm_proccomm_regulator_dev);
	if (rc)
	  pr_err("%s: could not register regulator device: %d\n",
	     __func__, rc);
}
#endif

void __init msm_device_i2c_init(void)
{
	if (gpio_request(60, "i2c_pri_clk"))
		pr_err("failed to request gpio i2c_pri_clk\n");
	if (gpio_request(61, "i2c_pri_dat"))
		pr_err("failed to request gpio i2c_pri_dat\n");
	if (gpio_request(95, "i2c_sec_clk"))
		pr_err("failed to request gpio i2c_sec_clk\n");
	if (gpio_request(96, "i2c_sec_dat"))
		pr_err("failed to request gpio i2c_sec_dat\n");

	if (cpu_is_msm7x27())
		msm_i2c_pdata.pm_lat =
		msm7x27_pm_data[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN]
		.latency;
	else
		msm_i2c_pdata.pm_lat =
		msm7x25_pm_data[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN]
		.latency;

	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

/* lge gpio i2c device */
#define MAX_GPIO_I2C_DEV_NUM	10
#define LOWEST_GPIO_I2C_BUS_NUM	2

static gpio_i2c_init_func_t *i2c_init_func[MAX_GPIO_I2C_DEV_NUM] __initdata;
static int i2c_dev_num __initdata = 0;

void __init lge_add_gpio_i2c_device(gpio_i2c_init_func_t *init_func)
{
	i2c_init_func[i2c_dev_num] = init_func;
	i2c_dev_num++;
}

void __init lge_add_gpio_i2c_devices(void)
{
	int index;
	gpio_i2c_init_func_t *init_func_ptr;

	for (index = 0;index < i2c_dev_num;index++) {
		init_func_ptr = i2c_init_func[index];
		(*init_func_ptr)(LOWEST_GPIO_I2C_BUS_NUM + index);
	}
}

int init_gpio_i2c_pin(struct i2c_gpio_platform_data *i2c_adap_pdata,
		struct gpio_i2c_pin gpio_i2c_pin,
		struct i2c_board_info *i2c_board_info_data)
{
	i2c_adap_pdata->sda_pin = gpio_i2c_pin.sda_pin;
	i2c_adap_pdata->scl_pin = gpio_i2c_pin.scl_pin;

	gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.sda_pin, 0, GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.scl_pin, 0, GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_set_value(gpio_i2c_pin.sda_pin, 1);
	gpio_set_value(gpio_i2c_pin.scl_pin, 1);

	if (gpio_i2c_pin.reset_pin) {
		gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.reset_pin, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_set_value(gpio_i2c_pin.reset_pin, 1);
	}

	if (gpio_i2c_pin.irq_pin) {
		gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.irq_pin, 0, GPIO_CFG_INPUT,
					GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		i2c_board_info_data->irq =
			MSM_GPIO_TO_INT(gpio_i2c_pin.irq_pin);
	}

	return 0;
}

#ifdef CONFIG_MACH_MSM7X27_PECAN
int init_gpio_i2c_pin_touch(struct i2c_gpio_platform_data *i2c_adap_pdata,
    struct gpio_i2c_pin gpio_i2c_pin,
    struct i2c_board_info *i2c_board_info_data)
{
  i2c_adap_pdata->sda_pin = gpio_i2c_pin.sda_pin;
  i2c_adap_pdata->scl_pin = gpio_i2c_pin.scl_pin;
  {
    gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.sda_pin, 0, GPIO_CFG_OUTPUT,
      GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.scl_pin, 0, GPIO_CFG_OUTPUT,
      GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

  }
  gpio_set_value(gpio_i2c_pin.sda_pin, 1);
  gpio_set_value(gpio_i2c_pin.scl_pin, 1);

  if (gpio_i2c_pin.reset_pin) {
    gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.reset_pin, 0, GPIO_CFG_OUTPUT,
      GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    gpio_set_value(gpio_i2c_pin.reset_pin, 1);
  }

  if (gpio_i2c_pin.irq_pin) {
    gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.irq_pin, 0, GPIO_CFG_INPUT,
      GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    i2c_board_info_data->irq =
      MSM_GPIO_TO_INT(gpio_i2c_pin.irq_pin);
  }

  return 0;
}
#endif

__WEAK void __init lge_add_camera_devices(void)
{
}

__WEAK void __init lge_add_input_devices(void)
{
}

__WEAK void __init lge_add_lcd_devices(void)
{
}

__WEAK void __init lge_add_btpower_devices(void)
{
}

__WEAK void __init lge_add_mmc_devices(void)
{
}

__WEAK void __init lge_add_misc_devices(void)
{
}

__WEAK void __init lge_add_pm_devices(void)
{
}
