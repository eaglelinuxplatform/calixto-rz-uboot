/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * include/configs/rcar-gen3-common.h
 *	This file is R-Car Gen3 common configuration file.
 *
 * Copyright (C) 2015-2017 Renesas Electronics Corporation
 */

#ifndef __RCAR_GEN3_COMMON_H
#define __RCAR_GEN3_COMMON_H

#include <asm/arch/rmobile.h>

#define CONFIG_REMAKE_ELF

#ifdef CONFIG_SPL
#define CONFIG_SPL_TARGET	"spl/u-boot-spl.scif"
#endif

#define CONFIG_SYS_BOOTPARAMS_LEN	SZ_128K

/* boot option */

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG

/* Generic Interrupt Controller Definitions */
#define CONFIG_GICV2
#define GICD_BASE	0xF1010000
#define GICC_BASE	0xF1020000

/* console */
#define CONFIG_SYS_CBSIZE		2048
#define CONFIG_SYS_BARGSIZE		CONFIG_SYS_CBSIZE
#define CONFIG_SYS_MAXARGS		64
#define CONFIG_SYS_BAUDRATE_TABLE	{ 115200, 38400 }

/* PHY needs a longer autoneg timeout */
#define PHY_ANEG_TIMEOUT		20000

/* MEMORY */
#define CONFIG_SYS_INIT_SP_ADDR		CONFIG_SYS_TEXT_BASE

#define DRAM_RSV_SIZE			0x08000000
#define CONFIG_SYS_SDRAM_BASE		(0x40000000 + DRAM_RSV_SIZE)
#define CONFIG_SYS_SDRAM_SIZE		(0x80000000u - DRAM_RSV_SIZE)
#define CONFIG_SYS_LOAD_ADDR		0x58000000
#define CONFIG_LOADADDR			CONFIG_SYS_LOAD_ADDR
#define CONFIG_VERY_BIG_RAM
#define CONFIG_MAX_MEM_MAPPED		(0x80000000u - DRAM_RSV_SIZE)

#define CONFIG_SYS_MONITOR_BASE		0x00000000
#define CONFIG_SYS_MONITOR_LEN		(1 * 1024 * 1024)
#define CONFIG_SYS_MALLOC_LEN		(64 * 1024 * 1024)
#define CONFIG_SYS_BOOTM_LEN		(64 << 20)

/* The HF/QSPI layout permits up to 1 MiB large bootloader blob */
#define CONFIG_BOARD_SIZE_LIMIT		1048576

/* ENV setting */

#if defined(CONFIG_TARGET_SILINUX_EK874)
#define CONFIG_EXTRA_ENV_SETTINGS	\
	"usb_pgood_delay=2000\0" \
	"bootm_size=0x10000000\0" \
	"prodsdbootargs=setenv bootargs rw rootwait earlycon root=/dev/mmcblk0p2 \0" \
	"prodemmcbootargs=setenv bootargs rw rootwait earlycon root=/dev/mmcblk1p2 \0" \
	"bootimage=unzip 0x4A080000 0x48080000; booti 0x48080000 - 0x48000000 \0" \
	"emmcload=ext4load mmc 1:2 0x48080000 boot/Image;ext4load mmc 1:2 0x48000000 boot/${dtb_name};run prodemmcbootargs \0" \
	"sdload=ext4load mmc 0:2 0x48080000 boot/Image;ext4load mmc 0:2 0x48000000 boot/${dtb_name};run prodsdbootargs \0" \
	"bootcmd_check=if mmc dev 0; then run sdload; else run emmcload; fi \0" \
	"dtb_check=if test ${board_rev} = E;then setenv dtb_name r8a774c0-ek874.dtb;else setenv dtb_name r8a774c0-ek874-revc.dtb;fi \0"
#elif defined(CONFIG_TARGET_HIHOPE_RZG2)
#define CONFIG_EXTRA_ENV_SETTINGS	\
	"usb_pgood_delay=2000\0" \
	"bootm_size=0x10000000\0" \
	"prodsdbootargs=setenv bootargs rw rootwait earlycon root=/dev/mmcblk1p2 \0" \
	"prodemmcbootargs=setenv bootargs rw rootwait earlycon root=/dev/mmcblk0p2 \0" \
	"bootimage=unzip 0x4A080000 0x48080000; booti 0x48080000 - 0x48000000 \0" \
	"emmcload=ext4load mmc 1:2 0x48080000 boot/Image;ext4load mmc 1:2 0x48000000 boot/${dtb_name};run prodemmcbootargs \0" \
	"sdload=ext4load mmc 0:2 0x48080000 boot/Image;ext4load mmc 0:2 0x48000000 boot/${dtb_name};run prodsdbootargs \0" \
	"bootcmd_check=if mmc dev 0; then run sdload; else run emmcload; fi \0" \
	"g2m_v1_dtb_check=if test ${soc_id} = r8a774a1;then if test ${board_rev} = 2;then setenv dtb_name r8a774a1-hihope-rzg2m-rev2-ex.dtb;else setenv dtb_name r8a774a1-hihope-rzg2m-ex.dtb;fi;fi \0" \
	"g2m_v3_dtb_check=if test ${soc_id} = r8a774a3;then setenv dtb_name r8a774a3-hihope-rzg2m-ex.dtb;fi\0" \
	"g2n_dtb_check=if test ${soc_id} = r8a774b1;then if test ${board_rev} = 2;then setenv dtb_name r8a774b1-hihope-rzg2n-rev2-ex.dtb;else setenv dtb_name r8a774b1-hihope-rzg2n-ex.dtb;fi;fi \0" \
	"g2h_dtb_check=if test ${soc_id} = r8a774e1;then setenv dtb_name r8a774e1-hihope-rzg2h-ex.dtb;fi \0"
#else
#define CONFIG_EXTRA_ENV_SETTINGS	\
	"usb_pgood_delay=2000\0" \
	"bootm_size=0x10000000\0"
#endif

#if defined(CONFIG_TARGET_SILINUX_EK874)
#define CONFIG_BOOTCOMMAND	\
	"board_rev=${board_rev}; env default -a; " \
	"run dtb_check; run bootcmd_check; run bootimage"
#elif defined(CONFIG_TARGET_HIHOPE_RZG2)
#define CONFIG_BOOTCOMMAND	\
	"soc_id=${platform}; board_rev=${board_rev}; env default -a; " \
	"run g2m_v1_dtb_check; run g2m_v3_dtb_check; " \
	"run g2n_dtb_check; run g2h_dtb_check; " \
	"run bootcmd_check; run bootimage"
#else
#define CONFIG_BOOTCOMMAND     \
	"tftp 0x48080000 Image; " \
	"tftp 0x48000000 Image-"CONFIG_DEFAULT_FDT_FILE"; " \
	"booti 0x48080000 - 0x48000000"
#endif


/* SPL support */
#if defined(CONFIG_R8A7795) || defined(CONFIG_R8A7796) || defined(CONFIG_R8A77965)
#define CONFIG_SPL_BSS_START_ADDR	0xe633f000
#define CONFIG_SPL_BSS_MAX_SIZE		0x1000
#else
#define CONFIG_SPL_BSS_START_ADDR	0xe631f000
#define CONFIG_SPL_BSS_MAX_SIZE		0x1000
#endif
#define CONFIG_SPL_STACK		0xe6304000
#define CONFIG_SPL_MAX_SIZE		0x7000
#ifdef CONFIG_SPL_BUILD
#define CONFIG_CONS_SCIF2
#define CONFIG_SH_SCIF_CLK_FREQ		65000000
#endif

#endif	/* __RCAR_GEN3_COMMON_H */
