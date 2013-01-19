EXTRA_CFLAGS += -Iarch/arm/mach-msm
obj-$(CONFIG_LGE_HEADSET_2GPIO) += lge_gpio_h2w.o
#obj-$(CONFIG_LGE_SWITCH_GPIO) += lge_gpio_switch.o
obj-$(CONFIG_LGE_HANDLE_PANIC) += lge_handle_panic.o
#obj-$(CONFIG_LGE_BLUE_ERROR_HANDLER) += lge_errorhandler.o
obj-$(CONFIG_LGE_PROC_COMM) += lge_proc_comm.o
#obj-$(CONFIG_LGE_SUPPORT_ERS) += lge_ers.o
obj-$(CONFIG_LGE_SUPPORT_RAPI) += LG_rapi_client.o
obj-$(CONFIG_LGE_MTD_DIRECT_ACCESS) += lge_mtd_direct_access.o #need this rather than emmc direct access, maybe this was problem on previous builds?
obj-$(CONFIG_LGE_DIAGTEST) += lg_diag_testmode.o
obj-$(CONFIG_LGE_DIAGTEST) += lg_fw_diag_class.o
obj-$(CONFIG_LGE_DIAGTEST) += lg_fw_diag_communication.o
obj-$(CONFIG_LGE_DIAGTEST) += lg_diag_event_log.o
obj-$(CONFIG_LGE_DIAGTEST) += lg_diag_keypress.o 
obj-$(CONFIG_LGE_DIAGTEST) += lg_diag_screen_shot.o
#obj-$(CONFIG_LGE_DIAGTEST) += lg_fw_diag_mtc.o
#obj-$(CONFIG_LGE_DIAGTEST) += lg_diag_wifi.o

obj-$(CONFIG_MACH_MSM7X27_PECAN) += board-pecan.o
obj-$(CONFIG_MACH_MSM7X27_PECAN) += board-pecan-mmc.o
obj-$(CONFIG_MACH_MSM7X27_PECAN) += board-pecan-snd.o
obj-$(CONFIG_MACH_MSM7X27_PECAN) += board-pecan-panel.o
obj-$(CONFIG_MACH_MSM7X27_PECAN) += board-pecan-bt.o
obj-$(CONFIG_MACH_MSM7X27_PECAN) += board-pecan-camera.o
obj-$(CONFIG_MACH_MSM7X27_PECAN) += board-pecan-input.o
obj-$(CONFIG_MACH_MSM7X27_PECAN) += board-pecan-misc.o
obj-$(CONFIG_MACH_MSM7X27_PECAN) += board-pecan-pm.o

obj-$(CONFIG_MACH_LGE) += devices_lge.o

#obj-$(CONFIG_LGE_SUPPORT_AT_CMD) += lge_ats.o
#obj-$(CONFIG_LGE_SUPPORT_AT_CMD) += lge_ats_cmd.o
#obj-$(CONFIG_LGE_SUPPORT_AT_CMD) += lge_ats_eta.o
#obj-$(CONFIG_LGE_SUPPORT_AT_CMD) += lge_ats_mmc.o
#obj-$(CONFIG_LGE_SUPPORT_AT_CMD) += lge_ats_event_log.o
#obj-$(CONFIG_LGE_ATS_UEVENT_DEVICE) += lge_ats_uevent.o
obj-$(CONFIG_LGE_ATS_INPUT_DEVICE) += lge_ats_input.o
#obj-$(CONFIG_LGE_DIAG_WMC) += lg_diag_wmc.o
#obj-$(CONFIG_LGE_SUPPORT_AT_CMD) += lge_ats_flex.o # LGE_CHANGE [irene.park@lge.com] 2010-06-04 - to get flex value from ARM9 
#obj-$(CONFIG_LGE_PWRSINK) += lge_pwrsink.o
#obj-$(CONFIG_LGE_FACTORY_PATCH) += lge_temp.o lge_mtd_direct_access.o

#from .38
obj-$(CONFIG_LGE_DIAGTEST) += lg_diag_unifiedmsgtool.o
obj-$(CONFIG_LGE_DIAG_UDM) += lg_diag_udm.o
obj-$(CONFIG_LGE_FACTORY) += userDataBackUpDiag.o
obj-$(CONFIG_LGE_FACTORY) += lge_diag_eri.o
obj-$(CONFIG_LGE_FACTORY) += lg_diag_dload.o
obj-$(CONFIG_LGE_ERROR_HANDLER) += lge_test.o
