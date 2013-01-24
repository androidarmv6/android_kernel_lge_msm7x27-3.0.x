/*
* Copyright (C) 2007 Google, Inc.
* Copyright (C) 2009-2010 LGE.
* Copyright (c) 2008-2012, Code Aurora Forum. All rights reserved.
* Copyright (c) 2013 Android ARMv6
* Author: Brian Swetland <swetland@google.com>
* Author: SungEun Kim <cleaneye.kim@lge.com>
* Author: Anthony King <anthonydking@gmail.com>
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* This is a rewrite of all the LGE msm7x27 board files 
*/

/*-----------------------------------------INCLUDES------------------------------------------*/

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/bootmem.h>
#include <linux/power_supply.h>

#include <mach/msm_memtypes.h>
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
#include <mach/socinfo.h>

#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/i2c.h>
#include <linux/android_pmem.h>
#include <mach/camera.h>

#ifdef CONFIG_USB_G_ANDROID
#include <linux/usb/android.h>
#include <mach/usbdiag.h>
#endif

//#include "board-msm7627-regulator.h"
#include "../devices.h"
#include "../clock.h"
#include "../acpuclock.h"
#include "../msm-keypad-devices.h"
#include "../pm.h"
#include "../pm-boot.h"

/*-----------------------------------------DEFINES-------------------------------------------*/

#ifdef CONFIG_ARCH_MSM7X25
#define MSM_PMEM_MDP_SIZE 0xb21000
#define MSM_PMEM_ADSP_SIZE 0x97b000
#define MSM_PMEM_AUDIO_SIZE 0x121000
#define MSM_FB_SIZE 0x200000
#define PMEM_KERNEL_EBI1_SIZE 0x64000
#endif

#ifdef CONFIG_ARCH_MSM7X27
#if defined(CONFIG_MACH_MSM7X27_PECAN)
/* TODO: optimization needed, pmem_mdp is reduced for qvga by bongkyu.kim */
#define MSM_PMEM_MDP_SIZE	0x1076000 /* 23->12MB + 4.46 MB */

#ifdef CONFIG_FB_MSM_TRIPLE_BUFFER
#define MSM_FB_SIZE		0x78000
#else
#define MSM_FB_SIZE		0x50000
#endif

#else
#define MSM_PMEM_MDP_SIZE	0x1B76000

#ifdef CONFIG_FB_MSM_TRIPLE_BUFFER
#define MSM_FB_SIZE 0x233000
#else
#define MSM_FB_SIZE 0x177000
#endif

#endif
#define MSM_PMEM_ADSP_SIZE     0xC00000 //12MB
#define MSM_PMEM_AUDIO_SIZE	0x121000
#define PMEM_KERNEL_EBI1_SIZE	0x1C000
#endif

/*-----------------------------------------USB-----------------------------------------------*/

#ifdef CONFIG_USB_G_ANDROID
static struct android_usb_platform_data android_usb_pdata = {
	.update_pid_and_serial_num = usb_diag_update_pid_and_serial_num,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};

#endif

#ifdef CONFIG_USB_EHCI_MSM_72K
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
	if (on)
		msm_hsusb_vbus_powerup();
	else
		msm_hsusb_vbus_shutdown();
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
	.phy_info       = (USB_PHY_INTEGRATED | USB_PHY_MODEL_65NM),
};
static void __init msm7x2x_init_host(void)
{
	if (machine_is_msm7x25_ffa() || machine_is_msm7x27_ffa())
		return;

	msm_add_host(0, &msm_usb_host_pdata);
}
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
struct vreg *vreg_3p3; // use vreg until we can use regulator-msm7627.c
static int msm_hsusb_ldo_init(int init)
{
	if (init) {
		/*
		 * PHY 3.3V analog domain(VDDA33) is powered up by
		 * an always enabled power supply (LP5900TL-3.3).
		 * USB VREG default source is VBUS line. Turning
		 * on USB VREG has a side effect on the USB suspend
		 * current. Hence USB VREG is explicitly turned
		 * off here.
		 */
		vreg_3p3 = vreg_get(NULL, "usb");
		if (IS_ERR(vreg_3p3))
			return PTR_ERR(vreg_3p3);
		vreg_enable(vreg_3p3);
		vreg_disable(vreg_3p3);
		vreg_put(vreg_3p3);
	}

	return 0;
}

static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init)
{
	int ret;

	if (init) {
		ret = msm_pm_app_rpc_init(callback);
	} else {
		msm_pm_app_rpc_deinit(callback);
		ret = 0;
	}
	return ret;
}

static int msm_otg_rpc_phy_reset(void __iomem *regs)
{
	return msm_hsusb_phy_reset();
}

static struct msm_otg_platform_data msm_otg_pdata = {
	.rpc_connect    	= hsusb_rpc_connect,
#ifdef CONFIG_ARCH_MSM7X27
	/* LGE_CHANGE
	 * To reset USB LDO, use RPC(only msm7x27).
	 * 2011-01-12, hyunhui.park@lge.com
	 */
	.phy_reset			= msm_otg_rpc_phy_reset,
#endif
	.pmic_vbus_notif_init	= msm_hsusb_pmic_notif_init,
	.chg_vbus_draw      = hsusb_chg_vbus_draw,
	.chg_connected      = hsusb_chg_connected,
	.chg_init        	= hsusb_chg_init,
#ifdef CONFIG_USB_EHCI_MSM_72K
	.vbus_power = msm_hsusb_vbus_power,
#endif
	.ldo_init       = msm_hsusb_ldo_init,
	.pclk_required_during_lpm = 1,
};

#ifdef CONFIG_USB_GADGET
static struct msm_hsusb_gadget_platform_data msm_gadget_pdata;
#endif
#endif

/*-----------------------------------------SND-----------------------------------------------*/


#define SND(desc, num) { .name = #desc, .id = num }
static struct snd_endpoint snd_endpoints_list[] = {
/* LGE_CHANGE_S, [junyoub.an] , 2010-02-12, Define sound device*/
#if 0						 // original
	SND(HANDSET, 0),
	SND(MONO_HEADSET, 2),
	SND(HEADSET, 3),
	SND(SPEAKER, 6),
	SND(TTY_HEADSET, 8),
	SND(TTY_VCO, 9),
	SND(TTY_HCO, 10),
	SND(BT, 12),
	SND(IN_S_SADC_OUT_HANDSET, 16),
	SND(IN_S_SADC_OUT_SPEAKER_PHONE, 25),
	SND(CURRENT, 27),
#else
#if defined(CONFIG_MACH_MSM7X27_ALESSI) || defined(CONFIG_MACH_MSM7X27_GELATO) \
	defined(CONFIG_MACH_MSM7X27_MUSCAT) || defined(CONFIG_MACH_MSM7X27_PECAN \
    defined(CONFIG_MACH_MSM7X27_THUNDERG)
	SND(HANDSET_LOOPBACK,5),
	SND(HANDSET, 6),
	SND(HEADSET_LOOPBACK, 1),
	SND(HEADSET, 2),
	SND(HEADSET_STEREO, 3),
	SND(SPEAKER, 0),
	SND(SPEAKER_IN_CALL, 7),
	SND(SPEAKER_RING, 8),
	SND(HEADSET_AND_SPEAKER, 8),
	SND(FM_HEADSET, 10),
	SND(FM_SPEAKER, 11),
	SND(BT, 13),
	SND(TTY_HEADSET, 15),
	SND(TTY_VCO, 16),
	SND(TTY_HCO, 17),
	SND(TTY_HCO_SPEAKER, 18),
	SND(HANDSET_VR, 20),
	SND(HEADSET_VR, 21),
	SND(BT_VR, 23),
	SND(CURRENT, 30),
#elif defined(CONFIG_MACH_MSM7X27_UNIVA)
	SND(HANDSET, 0),
	SND(HANDSET_CLOSED, 1),
	SND(HANDSET_VOICE_SEARCH, 2),
	SND(HANDSET_LOOPBACK, 3),
	SND(HEADSET, 4),
	SND(HEADSET_NO_MIC, 5),
	SND(HEADSET_VOICE_SEARCH, 6),
	SND(HEADSET_LOOPBACK, 7),
	SND(STEREO_HEADSET, 8),
	SND(STEREO_HEADSET_FMRADIO, 9),
	SND(SPEAKER_PHONE, 10),
	SND(SPEAKER_MEDIA, 11),
	SND(SPEAKER_FMRADIO, 12),
	SND(SPEAKER_HEADSET, 13),
	SND(SPEAKER_HEADSET_CLOSED, 14),
	SND(VOICE_RECORDER, 15),
	SND(VOICE_RECORDER_HEADSET, 16),
	SND(VIDEO_CAMCORDER, 17),
	SND(BT_SCO_HEADSET, 18),
	SND(BT_A2DP_HEADSET, 19),
	SND(TTY_HCO_SPEAKER, 20),
	SND(TTY_HEADSET, 21),
	SND(TTY_VCO, 22),
	SND(TTY_HCO, 23),
	SND(CURRENT, 25),
/* LGE_CHANGE_E, [junyoub.an] , 2010-02-12, Define sound device*/
#endif
};
#undef SND

static struct msm_snd_endpoints msm_device_snd_endpoints = {
	.endpoints = snd_endpoints_list,
	.num = sizeof(snd_endpoints_list) / sizeof(struct snd_endpoint)
};

struct platform_device msm_device_snd = {
	.name = "msm_snd",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_snd_endpoints
	},
};

/* yes this might be different to your device, but caf uses it *
 * if there are an issues, then we can start changing it       */

#define DEC0_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))

#define DEC1_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))

#define DEC2_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))

#define DEC3_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))

#define DEC4_FORMAT (1<<MSM_ADSP_CODEC_MIDI)

static unsigned int dec_concurrency_table[] = {
	/* Audio LP */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DMA)), 0,
	0, 0, 0,

	/* Concurrency 1 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	 /* Concurrency 2 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 3 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 4 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 5 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 6 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	0, 0, 0, 0,

	/* Concurrency 7 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),
};

#define DEC_INFO(name, queueid, decid, nr_codec) { .module_name = name, \
	.module_queueid = queueid, .module_decid = decid, \
	.nr_codec_support = nr_codec}

static struct msm_adspdec_info dec_info_list[] = {
#if defined(CONFIG_MACH_MSM7X27_PECAN) || defined(CONFIG_MACH_MSM7X27_UNIVA) \\
	defined(CONFIG_MACH_MSM7X27_GELATO) || defined(CONFIG_MACH_MSM7X27_MUSCAT)
	DEC_INFO("AUDPLAY0TASK", 13, 0, 11), /* AudPlay0BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY1TASK", 14, 1, 5),  /* AudPlay1BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY2TASK", 15, 2, 5),  /* AudPlay2BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY3TASK", 16, 3, 4),  /* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY4TASK", 17, 4, 1),  /* AudPlay4BitStreamCtrlQueue */
#elif defined(CONFIG_MACH_MSM7X27_ALESSI) || defined(CONFIG_MACH_MSM7X27_THUNDERG)
	DEC_INFO("AUDPLAY0TASK", 13, 0, 11), /* AudPlay0BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY1TASK", 14, 1, 11),  /* AudPlay1BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY2TASK", 15, 2, 11),  /* AudPlay2BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY3TASK", 16, 3, 11),  /* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY4TASK", 17, 4, 1),  /* AudPlay4BitStreamCtrlQueue */
#endif
};

static struct msm_adspdec_database msm_device_adspdec_database = {
	.num_dec = ARRAY_SIZE(dec_info_list),
	.num_concurrency_support = (ARRAY_SIZE(dec_concurrency_table) / \
					ARRAY_SIZE(dec_info_list)),
	.dec_concurrency_table = dec_concurrency_table,
	.dec_info_list = dec_info_list,
};

struct platform_device msm_device_adspdec = {
	.name = "msm_adspdec",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_adspdec_database
	},
};

/*-----------------------------------------PMEM----------------------------------------------*/

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};

static struct android_pmem_platform_data android_pmem_audio_pdata = {
	.name = "pmem_audio",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct platform_device android_pmem_audio_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_audio_pdata },
};

/*-----------------------------------------HS------------------------------------------------*/

static struct msm_handset_platform_data hs_platform_data = {
	.hs_name = "7k_handset",
	.pwr_key_delay_ms = 500, /* 0 will disable end key */
};

static struct platform_device hs_device = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = &hs_platform_data,
	},
};

/*-----------------------------------------TSIF----------------------------------------------*/
/* TSIF begin */
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)

#define TSIF_B_SYNC      GPIO_CFG(87, 5, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_DATA      GPIO_CFG(86, 3, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_EN        GPIO_CFG(85, 3, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_CLK       GPIO_CFG(84, 4, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)

static const struct msm_gpio tsif_gpios[] = {
	{ .gpio_cfg = TSIF_B_CLK,  .label =  "tsif_clk", },
	{ .gpio_cfg = TSIF_B_EN,   .label =  "tsif_en", },
	{ .gpio_cfg = TSIF_B_DATA, .label =  "tsif_data", },
	{ .gpio_cfg = TSIF_B_SYNC, .label =  "tsif_sync", },
};

static struct msm_tsif_platform_data tsif_platform_data = {
	.num_gpios = ARRAY_SIZE(tsif_gpios),
	.gpios = tsif_gpios,
	.tsif_clk = "core_clk",
	.tsif_pclk = "iface_clk",
	.tsif_ref_clk = "ref_clk",
};
//LGE change S
void __init lge_add_tsif_devices(void)
{
	msm_device_tsif.dev.platform_data = &tsif_platform_data;
	platform_device_register(&msm_device_tsif);
}
//LGE change E
#endif /* defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE) */

/* TSIF end   */

/*-----------------------------------------PANAL---------------------------------------------*/
// I had some trouble getting this one right. can someone look it over for me please?

#if defined(CONFIG_MACH_MSM7X27_GELATO)
static int gelato_panel_id = PANEL_ID_AUO;

static int __init gelato_panel_setup(char *panel_id)
{
	const char *panel_str[] = { "auo", "hitachi"};
	int i;

	for (i = 0; i < 2; i++)
		if (!strcmp(panel_id, panel_str[i])) {
			gelato_panel_id = i;
			break;
		}

	printk(KERN_INFO "%s: PANEL ID = %s\n",
		   __func__, panel_str[gelato_panel_id]);

	return 1;
}
__setup("panel.id=", gelato_panel_setup);
#endif

#define MSM_FB_LCDC_VREG_OP(name, op, level)			\
do { \
	vreg = vreg_get(0, name); \
	vreg_set_level(vreg, level); \
	if (vreg_##op(vreg)) \
		printk(KERN_ERR "%s: %s vreg operation failed \n", \
			(vreg_##op == vreg_enable) ? "vreg_enable" \
				: "vreg_disable", name); \
} while (0)

static char *msm_fb_vreg[] = {
	"gp1",
	"gp2",
};

#if defined(CONFIG_MACH_MSM7X27_MUSCAT) || defined(CONFIG_MACH_MSM7X27_PECAN)

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("ebi2", 0);
}


/* Use pmic_backlight function as power save function, munyoung.hwang@lge.com */
static int mddi_power_save_on;
static int ebi2_tovis_power_save(int on)
{
	struct vreg *vreg;
	int flag_on = !!on;

	printk(KERN_INFO"%s: on=%d\n", __func__, flag_on);

	if (mddi_power_save_on == flag_on)
		return 0;

	mddi_power_save_on = flag_on;

	if (on) {
		/* MSM_FB_LCDC_VREG_OP(msm_fb_vreg[0], enable, 1800); */
		MSM_FB_LCDC_VREG_OP(msm_fb_vreg[1], enable, 2800);
	} else{
		/* LGE_CHANGE, [hyuncheol0.kim@lge.com] , 2011-02-10, for current consumption */
		//MSM_FB_LCDC_VREG_OP(msm_fb_vreg[0], disable, 0);
		MSM_FB_LCDC_VREG_OP(msm_fb_vreg[1], disable, 0);
	}
	return 0;
}

static struct platform_device ebi2_tovis_panel_device = {
	.name   = "ebi2_tovis_qvga",
	.id     = 0,
	.dev    = {
		.platform_data = &ebi2_tovis_panel_data,
	}
};

static struct msm_panel_ilitek_pdata ebi2_tovis_panel_data = {
		.gpio = 102,				/* lcd reset_n */
		.lcd_power_save = ebi2_tovis_power_save,
		.maker_id = PANEL_ID_TOVIS,
		.initialized = 1,
};
#elif defined(CONFIG_MACH_MSM7X27_UNIVA) || defined(CONFIG_MACH_MSM7X27_ALESSI) \
	defined(CONFIG_MACH_MSM7X27_GELATO) || defined(CONFIG_MACH_MSM7X27_THUNDERG)
static int mddi_power_save_on;
static int msm_fb_mddi_power_save(int on)
{
	struct vreg *vreg;
	int flag_on = !!on;

	if (mddi_power_save_on == flag_on)
		return 0;

	mddi_power_save_on = flag_on;

	if (on) {
		MSM_FB_LCDC_VREG_OP(msm_fb_vreg[0], enable, 1800);
		MSM_FB_LCDC_VREG_OP(msm_fb_vreg[1], enable, 2800);
	} else{
		MSM_FB_LCDC_VREG_OP(msm_fb_vreg[0], disable, 0);
		MSM_FB_LCDC_VREG_OP(msm_fb_vreg[1], disable, 0);
	}

	return 0;
}

static struct mddi_platform_data mddi_pdata = {
	.mddi_power_save = msm_fb_mddi_power_save,
};

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("pmdh", &mddi_pdata);
	msm_fb_register_device("lcdc", 0);
}
#endif

#if defined(CONFIG_MACH_MSM7X27_UNIVA)
static int mddi_ldp_pmic_backlight(int level)
{
	/* TODO: Backlight control here */
	return 0;
}

static struct msm_panel_ldp_pdata mddi_ldp_panel_data = {
		.gpio = 102,				/* lcd reset_n */
		.pmic_backlight = mddi_ldp_pmic_backlight,
		.initialized = 1,
};

static struct platform_device mddi_ldp_panel_device = {
	.name   = "mddi_ldp_hvga",
	.id     = 0,
	.dev    = {
		.platform_data = &mddi_ldp_panel_data,
	}
};
#elif defined(CONFIG_MACH_MSM7X27_ALESSI)
static int mddi_sharp_pmic_backlight(int level)
{
	/* TODO: Backlight control here */
	return 0;
}
static struct msm_panel_common_pdata mddi_sharp_panel_data = {
	.gpio = 102,				/* lcd reset_n */
	.pmic_backlight = mddi_sharp_pmic_backlight,
};
static struct platform_device mddi_sharp_panel_device = {
	.name   = "mddi_sharp_hvga",
	.id     = 0,
	.dev    = {
		.platform_data = &mddi_sharp_panel_data,
	}
};
#elif defined(CONFIG_MACH_MSM7X27_GELATO) || defined(CONFIG_MACH_MSM7X27_THUNDERG)
static int mddi_hitachi_pmic_backlight(int level)
{
	/* TODO: Backlight control here */
	return 0;
}

static struct msm_panel_hitachi_pdata mddi_hitachi_panel_data = {
		.gpio = 102,				/* lcd reset_n */
		.pmic_backlight = mddi_hitachi_pmic_backlight,
		.initialized = 1,
};

static struct platform_device mddi_hitachi_panel_device = {
	.name   = "mddi_hitachi_hvga",
	.id     = 0,
	.dev    = {
		.platform_data = &mddi_hitachi_panel_data,
	}
};
#endif

/* backlight device */

#ifndef CONFIG_MACH_MSM7X27_GELATO
static struct gpio_i2c_pin bl_i2c_pin[] = {
	[0] = {
		.sda_pin	= 89,
		.scl_pin	= 88,
		.reset_pin	= 82,
		.irq_pin	= 0,
	},
};
#endif
static struct i2c_gpio_platform_data bl_i2c_pdata = {
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.udelay				= 2,
#ifdef CONFIG_MACH_MSM7X27_GELATO
	.sda_pin = 89,
	.scl_pin = 88,
#endif
};

static struct platform_device bl_i2c_device = {
	.name	= "i2c-gpio",
	.dev.platform_data = &bl_i2c_pdata,
};

#if defined(CONFIG_MACH_MSM7X27_UNIVA)

void bl_config_gpio(int config)
{
	if (config)
	{		/* for wake state */
		gpio_tlmm_config(GPIO_CFG(bl_i2c_pin[0].sda_pin, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(bl_i2c_pin[0].scl_pin, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	}
	else
	{		/* for sleep state */
		gpio_tlmm_config(GPIO_CFG(bl_i2c_pin[0].sda_pin, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(bl_i2c_pin[0].scl_pin, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	}
}
EXPORT_SYMBOL(bl_config_gpio);

 // UNIVA_LCD_BL
static struct lm3530_platform_data lm3530bl_data = {
  .gpio = 82,
  .version = 2862,
  .init_on_boot = 0,
};
static struct i2c_board_info bl_i2c_bdinfo[] = {
	[0] = {
		I2C_BOARD_INFO("lm3530bl", 0x38),		
		.type = "lm3530bl",
		.platform_data = NULL,
	},
};
struct device* univa_backlight_dev(void)
{
	return &bl_i2c_device.dev;
}

void __init univa_init_i2c_backlight(int bus_num)
{
	bl_i2c_device.id = bus_num;
	bl_i2c_bdinfo[0].platform_data = &lm3530bl_data;

	init_gpio_i2c_pin(&bl_i2c_pdata, bl_i2c_pin[0],	&bl_i2c_bdinfo[0]);
	i2c_register_board_info(bus_num, &bl_i2c_bdinfo[0], 1);
	platform_device_register(&bl_i2c_device);
}

static void univa_panel_set_maker_id(void)
{
	if(lge_bd_rev == LGE_REV_B)
		mddi_ldp_panel_data.maker_id = PANEL_ID_LDP;
	else
		mddi_ldp_panel_data.maker_id = PANEL_ID_AUO;
}
#if defined(UNIVA_EVB)
/* SUB PMIC device */
#include <../../../../drivers/video/backlight/lp8720.h>
static struct gpio_i2c_pin ldo_i2c_pin[] = {
	[0] = {
		.sda_pin	= 76,
		.scl_pin	= 77,
		.reset_pin	= 0,
		.irq_pin	= 0,
	},
};

static struct i2c_gpio_platform_data ldo_i2c_pdata = {
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.udelay				= 2,
};

static struct platform_device ldo_i2c_device = {
	.name	= "i2c-gpio",
	.dev.platform_data = &ldo_i2c_pdata,
};

static struct lp8720_platform_data lp8720ldo_data = {
	.en_gpio_num = LP8720_ENABLE,
};

static struct i2c_board_info ldo_i2c_bdinfo[] = {
	[0] = {
		I2C_BOARD_INFO(LP8720_I2C_NAME, LP8720_I2C_ADDR),
		.type = LP8720_I2C_NAME,
		.platform_data = NULL,
	},
};

struct device* univa_ldo_dev(void)
{
	return &ldo_i2c_device.dev;
}

void __init univa_init_i2c_ldo(int bus_num)
{
	ldo_i2c_device.id = bus_num;
	ldo_i2c_bdinfo[0].platform_data = &lp8720ldo_data;
	
	init_gpio_i2c_pin(&ldo_i2c_pdata, ldo_i2c_pin[0],	&ldo_i2c_bdinfo[0]);
	i2c_register_board_info(bus_num, &ldo_i2c_bdinfo[0], 1);
	platform_device_register(&ldo_i2c_device);
}
#endif //defined(UNIVA_EVB)

void __init lge_add_lcd_devices(void)
{
	univa_panel_set_maker_id();
	platform_device_register(&mddi_ldp_panel_device);

	msm_fb_add_devices();

	lge_add_gpio_i2c_device(univa_init_i2c_backlight);
#if defined(UNIVA_EVB)	
	lge_add_gpio_i2c_device(univa_init_i2c_ldo);
#endif //defined(UNIVA_EVB)
}

#elif defined (CONFIG_MACH_MSM7X27_ALESSI) || defined (CONFIG_MACH_MSM7X27_GELATO) \
	defined (CONFIG_MACH_MSM7X27_THUNDERG) || defined (CONFIG_MACH_MSM7X27_PECAN) || defined(CONFIG_MACH_MSM7X27_MUSCAT)

static struct aat28xx_platform_data aat2870bl_data = {
	.gpio = 82,
	.version = 2862,
	.initialized = 1,
};

static struct i2c_board_info bl_i2c_bdinfo[] = {
	[0] = {
		I2C_BOARD_INFO("aat2870bl", 0x60),
		.type = "aat2870bl",
		.platform_data = NULL,
	},
};

struct device* lge_backlight_dev(void)
{
	return &bl_i2c_device.dev;
}

void __init lge_init_i2c_backlight(int bus_num)
{
	bl_i2c_device.id = bus_num;
	bl_i2c_bdinfo[0].platform_data = &aat2870bl_data;

#ifndef CONFIG_MACH_MSM7X27_GELATO
	init_gpio_i2c_pin(&bl_i2c_pdata, bl_i2c_pin[0],	&bl_i2c_bdinfo[0]);
#endif // defined (CONFIG_MACH_MSM7X27_PECAN) ||  defined(CONFIG_MACH_MSM7X27_MUSCAT)
	i2c_register_board_info(bus_num, &bl_i2c_bdinfo[0], 1);
	platform_device_register(&bl_i2c_device);
}

#if defined (CONFIG_MACH_MSM7X27_PECAN) ||  defined(CONFIG_MACH_MSM7X27_MUSCAT)
static int lge_fb_event_notify(struct notifier_block *self,
			      unsigned long action, void *data)
{
	struct fb_event *event = data;
	struct fb_info *info = event->info;
	struct fb_var_screeninfo *var = &info->var;
	if(action == FB_EVENT_FB_REGISTERED) {
		var->width = 43;
		var->height = 58;
	}
	return 0;
}
static struct notifier_block lge_fb_event_notifier = {
	.notifier_call	= lge_fb_event_notify,
};

void __init lge_add_lcd_devices(void)
{	
	if(ebi2_tovis_panel_data.initialized)
	  ebi2_tovis_power_save(1);

	fb_register_client(&lge_fb_event_notifier);

	platform_device_register(&ebi2_tovis_panel_device);
	msm_fb_add_devices();
	lge_add_gpio_i2c_device(lge_init_i2c_backlight);
}
#endif // defined (CONFIG_MACH_MSM7X27_PECAN) ||  defined(CONFIG_MACH_MSM7X27_MUSCAT)
#ifdef CONFIG_MACH_MSM7X27_GELATO
static void gelato_panel_set_maker_id(void)
{
	mddi_hitachi_panel_data.maker_id = gelato_panel_id;
}

void __init lge_add_lcd_devices(void)
{
	gelato_panel_set_maker_id();
	platform_device_register(&mddi_hitachi_panel_device);

	msm_fb_add_devices();

	lge_add_gpio_i2c_device(gelato_init_i2c_backlight);
}
#endif //CONFIG_MACH_MSM7X27_GELATO

#ifdef CONFIG_MACH_MSM7X27_THUNDERG
void __init lge_add_lcd_devices(void)
{
	platform_device_register(&mddi_hitachi_panel_device);

	msm_fb_add_devices();

	lge_add_gpio_i2c_device(thunderg_init_i2c_backlight);
}
#endif //CONFIG_MACH_MSM7X27_THUNDERG
#ifdef CONFIG_MACH_MSM7X27_ALESSI
void __init lge_add_lcd_devices(void)
{
	platform_device_register(&mddi_sharp_panel_device);

	msm_fb_add_devices();

	lge_add_gpio_i2c_device(thunderg_init_i2c_backlight);
}
#endif //CONFIG_MACH_MSM7X27_ALESSI

#endif //the big one

/*-----------------------------------------FB------------------------------------------------*/

/* setting frame buffer device */
static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static int msm_fb_detect_panel(const char *name)
{
	int ret = -EPERM;

	if (machine_is_msm7x25_ffa() || machine_is_msm7x27_ffa()) {
		if (!strcmp(name, "lcdc_gordon_vga"))
			ret = 0;
		else
			ret = -ENODEV;
	}

	return ret;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
	.mddi_prescan = 1,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
	.dev    = {
		.platform_data = &msm_fb_pdata,
	}
};

/*-----------------------------------------BT------------------------------------------------*/



/*this is near the end, I sent to the wrong place ;) */

static struct platform_device *usb_devices[] __initdata = {
#ifdef CONFIG_USB_MSM_OTG_72K
	&msm_device_otg,
#ifdef CONFIG_USB_GADGET
	&msm_device_gadget_peripheral,
#endif
#endif

#ifdef CONFIG_USB_SUPPORT_LGE_ANDROID_AUTORUN
	/* LGE_CHANGE
	 * Add platform data and device for cdrom storage function.
	 * It will be used in Autorun feature.
	 * 2011-03-02, hyunhui.park@lge.com
	 */
	&usb_cdrom_storage_device,
#endif
#ifdef CONFIG_USB_G_ANDROID
	&android_usb_device,
#endif

};

static void usb_mpp_init(void)
{
	unsigned rc;
	unsigned mpp_usb = 7;

	if (machine_is_msm7x25_ffa() || machine_is_msm7x27_ffa()) {
		rc = mpp_config_digital_out(mpp_usb,
				MPP_CFG(MPP_DLOGIC_LVL_VDD,
					MPP_DLOGIC_OUT_CTRL_HIGH));
		if (rc)
			pr_err("%s: configuring mpp pin"
					"to enable 3.3V LDO failed\n", __func__);
	}
}

void __init msm_add_usb_devices(void) 
{
	usb_mpp_init();

#ifdef CONFIG_USB_MSM_OTG_72K
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
	if (machine_is_msm7x25_surf() || machine_is_msm7x25_ffa()) {
		msm_otg_pdata.pemp_level =
			PRE_EMPHASIS_WITH_20_PERCENT;
		msm_otg_pdata.drv_ampl = HS_DRV_AMPLITUDE_5_PERCENT;
		msm_otg_pdata.cdr_autoreset = CDR_AUTO_RESET_ENABLE;
		msm_otg_pdata.phy_reset = msm_otg_rpc_phy_reset;
	}
	if (machine_is_msm7x27_surf() || machine_is_msm7x27_ffa()) {
		msm_otg_pdata.pemp_level =
			PRE_EMPHASIS_WITH_10_PERCENT;
		msm_otg_pdata.drv_ampl = HS_DRV_AMPLITUDE_5_PERCENT;
		msm_otg_pdata.cdr_autoreset = CDR_AUTO_RESET_DISABLE;
		msm_otg_pdata.phy_reset_sig_inverted = 1;
	}

#ifdef CONFIG_USB_GADGET
	msm_otg_pdata.swfi_latency =
		msm7x27_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
	msm_gadget_pdata.is_phy_status_timer_on = 1;
#endif
#endif

	platform_add_devices(usb_devices, ARRAY_SIZE(usb_devices));

#ifdef CONFIG_USB_EHCI_MSM_72K
	msm7x2x_init_host();
#endif
}
