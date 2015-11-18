/*
 * Copyright (C) 2009, Texas Instruments, Incorporated
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __CONFIG_TI8148_IPNC_H
#define __CONFIG_TI8148_IPNC_H

#ifdef ENVAPI_IN_LINUX
#include "EnvAPI/sizes.h"
#include "model_env_config.h"
#else
#include <asm/sizes.h>
#include <model.h>
#endif

/*
 *#define CONFIG_TI814X_NO_RUNTIME_PG_DETECT
 */

#define XMK_STR(x)	#x
#define MK_STR(x)	XMK_STR(x)

/* Display CPU info */
#define CONFIG_DISPLAY_CPUINFO		1

/* In the 1st stage we have just 110K, so cut down wherever possible */
#ifdef CONFIG_TI814X_MIN_CONFIG

/* enable d-cache only on 2nd stage */
#define CONFIG_SYS_DCACHE_OFF

#define CONFIG_CMD_MEMORY	/* for mtest */
#undef CONFIG_GZIP
#undef CONFIG_ZLIB
#undef CONFIG_SYS_HUSH_PARSER
#define CONFIG_CMD_LOADB	/* loadb */
#define CONFIG_CMD_LOADY	/* loady */
#define CONFIG_SETUP_PLL
#define CONFIG_TI814X_CONFIG_DDR
#define CONFIG_TI814X_EVM_DDR3
#define CONFIG_ENV_SIZE				0x400
#define CONFIG_SYS_MALLOC_LEN		(CONFIG_ENV_SIZE + (8 * 1024))
#define CONFIG_SYS_PROMPT			"UBL#"
/* set to negative value for no autoboot */
#define CONFIG_BOOTDELAY			2
#if defined(CONFIG_SPI_BOOT)		/* Autoload the 2nd stage from SPI */
#define CONFIG_SPI					1
#if defined(CONFIG_TI81XX_PCIE_BOOT)
#define CONFIG_CMDLINE_TAG			1	/* enable passing of ATAGs  */
#define CONFIG_SETUP_MEMORY_TAGS	1
#define CONFIG_INITRD_TAG			1	/* Required for ramdisk support */
#define CONFIG_CMD_SOURCE
#define CONFIG_EXTRA_ENV_SETTINGS \
	"verify=yes\0" \
	"bootcmd=source 0x80400000\0" \
	""
/* user can override default size configuration here.
 * it will only come in effect if TI81xx_NO_PIN_GPMC
 * is defined in include/asm/arch/pcie.h
 */
#define CONFIG_BAR1_32  (0x1000000ULL)
#define CONFIG_BAR2_32  (0x800000ULL)
#define CONFIG_BAR3_32  (0xfffULL)
#define CONFIG_BAR4_32  (0x1001ULL)
#define CONFIG_REG2_64  (0x1000000ULL)
#define CONFIG_REG4_64  (0x2000000ULL)


#else
#define CONFIG_EXTRA_ENV_SETTINGS \
	"verify=yes\0" \
	"bootcmd=sf probe 0; sf read 0x81000000 0x20000 0x80000; go 0x81000000\0" \
	""

#endif
#elif defined(CONFIG_NAND_BOOT)		/* Autoload the 2nd stage from NAND */
#define CONFIG_NAND			1
//#define CONFIG_MMC			1
#define CONFIG_EXTRA_ENV_SETTINGS \
	"verify=yes\0" \
	"bootcmd=nand read 0x81000000 0x20000 0x80000; go 0x81000000\0" \
	""

#elif defined(CONFIG_SD_BOOT)		/* Autoload the 2nd stage from SD */
#define CONFIG_MMC			1
//#define CONFIG_NAND			1
#define CONFIG_EXTRA_ENV_SETTINGS \
	"verify=yes\0" \
	"bootcmd=mmc rescan 0; fatload mmc 0 0x80800000 u-boot.bin; go 0x80800000\0" \
	""

#elif defined(CONFIG_UART_BOOT)                /* stop in the min prompt */
#define CONFIG_EXTRA_ENV_SETTINGS \
	"verify=yes\0" \
	"bootcmd=\0" \
	""

#elif defined(CONFIG_ETH_BOOT)		/* Auto load 2nd stage from server */
#define CONFIG_EXTRA_ENV_SETTINGS \
	"verify=yes\0" \
	"bootcmd=setenv autoload no;dhcp; tftp 0x81000000 u-boot.bin; go 0x81000000\0" \
	""

#endif

#if 1	/* feel free to disable for development */
#define CONFIG_AUTOBOOT_KEYED		/* Enable password protection	*/
#define CONFIG_AUTOBOOT_PROMPT		">"
#define CONFIG_AUTOBOOT_DELAY_STR	"mmmoxa"	/* 1st "password"	*/
#endif

/* Watchdog setting */
#define CONFIG_HW_WATCHDOG
#define WDT_TIMEOUT_SEC				6			/* Watchdog time out (sec.)*/
#define WDT_TIMEOUT_BASE			0x00008000	/* ~1 sec. */

#elif defined(CONFIG_TI814X_OPTI_CONFIG)		/* Optimized code */
#include <config_cmd_default.h>

#define CONFIG_ZERO_BOOTDELAY_CHECK
#define CONFIG_ENV_SIZE				0x2000
#define CONFIG_SYS_MALLOC_LEN		(CONFIG_ENV_SIZE + (32 * 1024))
#define CONFIG_ENV_OVERWRITE
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_PROMPT			"UBOOT-OPTI#"
#define CONFIG_CMDLINE_TAG        	1	/* enable passing of ATAGs  */
#define CONFIG_SETUP_MEMORY_TAGS  	1
#define CONFIG_INITRD_TAG	  		1	/* Required for ramdisk support */
#define CONFIG_BOOTDELAY			1	/* set to negative value for no autoboot */
#define CONFIG_NAND					1
#define CONFIG_SETUP_PLL
#define CONFIG_TI814X_CONFIG_DDR
#define CONFIG_TI814X_EVM_DDR3
#define CONFIG_SYS_DCACHE_OFF
#define CONFIG_SYS_ICACHE_OFF


#define CONFIG_CMD_BDI		/* bdinfo			*/
#define CONFIG_CMD_BOOTD	/* bootd			*/
#define CONFIG_CMD_CONSOLE	/* coninfo			*/
#define CONFIG_CMD_ECHO		/* echo arguments		*/
#define CONFIG_CMD_EDITENV	/* editenv			*/
#define CONFIG_CMD_FPGA		/* FPGA configuration Support	*/
#define CONFIG_CMD_IMI		/* iminfo			*/
#define CONFIG_CMD_ITEST	/* Integer (and string) test	*/
#ifndef CONFIG_SYS_NO_FLASH
#define CONFIG_CMD_FLASH	/* flinfo, erase, protect	*/
#define CONFIG_CMD_IMLS		/* List all found images	*/
#endif
#define CONFIG_CMD_LOADB	/* loadb			*/
#define CONFIG_CMD_LOADS	/* loads			*/
#define CONFIG_CMD_MEMORY	/* md mm nm mw cp cmp crc base loop mtest */
#define CONFIG_CMD_MISC		/* Misc functions like sleep etc*/
#define CONFIG_CMD_NET		/* bootp, tftpboot, rarpboot	*/
#define CONFIG_CMD_NFS		/* NFS support			*/
#define CONFIG_CMD_RUN		/* run command in env variable	*/
#define CONFIG_CMD_SAVEENV	/* saveenv			*/
#define CONFIG_CMD_SETGETDCR	/* DCR support on 4xx		*/
#define CONFIG_CMD_SOURCE	/* "source" command support	*/
#define CONFIG_CMD_XIMG		/* Load part of Multi Image	*/

#undef CONFIG_GZIP
#undef CONFIG_ZLIB
#undef CONFIG_CMD_LOADB
#undef CONFIG_CMD_LOADS
#undef CONFIG_CMD_NFS
#undef CONFIG_CMD_SETGETDCR
#undef CONFIG_CMD_XIMG
#undef CONFIG_CMD_MISC
#undef CONFIG_CMD_ITEST
#undef CONFIG_CMD_FPGA
#undef CONFIG_CMD_EDITENV
#undef CONFIG_BOOTM_NETBSD
#undef CONFIG_BOOTM_RTEMS
#undef CONFIG_CMD_MISC
#undef CONFIG_CMD_IMI
#undef CONFIG_CMD_ITEST
#undef CONFIG_CMD_SOURCE
#undef CONFIG_CMD_IMLS
#undef CONFIG_CMD_ECHO

 #define CONFIG_EXTRA_ENV_SETTINGS \
 	"verify=no\0" \
	"bootdelay=0\0" \
	"bootfile=uImage\0" \
	"loadaddr=0x81000000\0" \
	"bootargs=console=ttyO0,115200n8 mem=256M  notifyk.vpssm3_sva=0xBF900000 vram=50M ubi.mtd=4 root=ubi0:rootfs rootfstype=ubifs rw rootwait=1 rw lpj=4997120 ip=${ipaddr}:${serverip}:${gateway}:${subnet}::eth0:off ethaddr=${ethaddr}\0 "\

 #define CONFIG_BOOTCOMMAND \
         "nboot 80007FC0 0 0x280000;bootm 80007FC0"

#else /* 2st stage u-boot */

#ifndef ENVAPI_IN_LINUX
#include <config_cmd_default.h>
#endif
#define CONFIG_SERIAL_TAG			1
#define CONFIG_REVISION_TAG			1
#define CONFIG_SKIP_LOWLEVEL_INIT	/* 1st stage would have done the basic init */
#define CONFIG_ENV_SIZE				0x2000
#define CONFIG_SYS_MALLOC_LEN		(CONFIG_ENV_SIZE + (512 * 1024))
#define CONFIG_ENV_OVERWRITE
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_PROMPT			"UBOOT#"
#define CONFIG_SYS_HUSH_PARSER		/* Use HUSH parser to allow command parsing */
#define CONFIG_SYS_PROMPT_HUSH_PS2	"> "
#define CONFIG_CMDLINE_TAG        	1	/* enable passing of ATAGs  */
#define CONFIG_SETUP_MEMORY_TAGS  	1
#define CONFIG_INITRD_TAG	  		1	/* Required for ramdisk support */
#define CONFIG_BOOTDELAY			3	/* set to negative value for no autoboot */
#define CONFIG_CMD_AUTOTEST	/* for autotest */
/* By default, 2nd stage will have MMC, NAND, SPI and I2C support */
#define CONFIG_MMC					1
#define CONFIG_NAND					1
#define CONFIG_SPI					1
#define CONFIG_I2C					1
#define CONFIG_EXTRA_ENV_SETTINGS \
	"verify=yes\0" \
	"bootfile=uImage\0" \
	"ramdisk_file=ramdisk.gz\0" \
	"ublfile="D4_UBL_FILE_NAME"\0" \
	"ubtfile="D4_UBOOT_FILE_NAME"\0" \
	"kernelfile="D4_KERNEL_FILE_NAME"\0" \
	"fsfile="D4_FS_FILE_NAME"\0" \
	"mpkernelfile="D4_MPKERNEL_FILE_NAME"\0" \
	"mpfsfile="D4_MPFS_FILE_NAME"\0" \
	""

#define CONFIG_BOOTARGS \
	"console=ttyO0,115200n8 rootwait=1 ro ubi.mtd=${mtd_idx},2048 rootfstype=ubifs root=ubi0:rootfs init=/init mem=120M vram=4M notifyk.vpssm3_sva=0xBFD00000 eth=${ethaddr} cmemk.phys_start=0x87800000 cmemk.phys_end=0x8cc00000 cmemk.allowOverlap=1 earlyprintk"
	//"console=ttyO0,115200n8 rootwait=1 ro ubi.mtd=${mtd_idx},2048 rootfstype=ubifs root=ubi0:rootfs init=/init mem=80M vram=4M notifyk.vpssm3_sva=0xBFD00000 ip=dhcp eth=${ethaddr} cmemk.phys_start=0x85000000 cmemk.phys_end=0x89000000 cmemk.allowOverlap=1 earlyprintk"

#define CONFIG_BOOTCOMMAND \
	"ipnc_ff_init 1;nboot ${loadaddr} 0 ${kernelflash}; bootm"

#define CONFIG_MPBOOTCOMMAND \
	"ipnc_ff_init 1;nboot ${loadaddr} 0 ${mpkernelflash}; bootm"

#define BACKDOOR_TIMEOUT			3 /* MMmoxaie backdoor timeout sec. */
#define CONFIG_PREBOOT				"moxamm"
#define CONFIG_CMD_MOXAMM
#define CONFIG_AUTO_COMPLETE		/* add autocompletion support	*/

/* Watchdog setting */
#define CONFIG_HW_WATCHDOG
#define WDT_TIMEOUT_SEC				60			/* Watchdog time out (sec.)*/
#define WDT_TIMEOUT_BASE			0x00008000	/* ~1 sec. */

#endif	/* CONFIG_TI814X_MIN_CONFIG */

#define CONFIG_SYS_GBL_DATA_SIZE	128	/* size in bytes reserved for initial data */

#define CONFIG_MISC_INIT_R			1
#ifndef CONFIG_TI814X_MIN_CONFIG
#define CONFIG_TI814X_ASCIIART		1	/* The centaur */
#define CONFIG_FORCE_SAVEENV			/* force saveenv after default */
#endif
#define CONFIG_SYS_AUTOLOAD			"yes"
#ifndef CONFIG_TI814X_OPTI_CONFIG
#define CONFIG_CMD_CACHE
#define CONFIG_CMD_ECHO
#define CONFIG_CMD_MISC
#endif

/*
 * For test or debug
 */
//#define TEST_SWITCH_MPFLAG

//wensen
//#define CONFIG_SYS_FUNCTION
//#define CONFIG_SYS_FUNCTION_RED_ENV		0x01;

//#define CONFIG_SHOW_BOOT_PROGRESS


/*
 * Miscellaneous configurable options
 */

/* max number of command args */
#define CONFIG_SYS_MAXARGS			32
/* Console I/O Buffer Size */
#define CONFIG_SYS_CBSIZE			512
/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE			(CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
/* Boot Argument Buffer Size */
#define CONFIG_SYS_BARGSIZE			CONFIG_SYS_CBSIZE
/* memtest works on 8 MB in DRAM after skipping 32MB from start addr of ram disk*/
#define CONFIG_SYS_MEMTEST_START	(PHYS_DRAM_1 + (64 *1024 *1024))
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + (8 * 1024 * 1024))
#if defined(CONFIG_CMD_AUTOTEST)
#define CONFIG_SYS_MEMTEST_SIZE 	0x10000000	/* autotest memory size*/
#endif
#undef  CONFIG_SYS_CLKS_IN_HZ				/* everything, incl board info, in Hz */
#define CONFIG_SYS_LOAD_ADDR		0x81000000  	/* Default load address */
#define CONFIG_SYS_HZ				1000        	/* 1ms clock */
#define CFG_HZ 						CONFIG_SYS_HZ
#define CFG_MEMTEST_START			CONFIG_SYS_MEMTEST_START
#define CONFIG_SYS_TEMP_ADDR		CONFIG_SYS_MEMTEST_START  	/* Default temp address */

#define CONFIG_LOADADDR 			CONFIG_SYS_LOAD_ADDR
#define CONFIG_CMDLINE_EDITING
#define CONFIG_ETHADDR 				00:90:e8:00:00:00
#define CONFIG_IPADDR 				192.168.127.100
#define CONFIG_SERVERIP 			192.168.127.98
#define CONFIG_GATEWAYIP 			192.168.127.98
#define CONFIG_DNSIP 				192.168.127.98
#define CONFIG_NTPSERVERIP 			192.168.127.98
#define CONFIG_NETMASK 				255.255.255.0
#define CFG_CONSOLE_IS_IN_ENV		1 /* stdin/stdout/stderr are in environment */
#define CONFIG_DUT_ID 				0.0.0.0.0.0
#define CONFIG_HW_VER 				0.0
#define CONFIG_FW_VER 				0.0
#define CONFIG_MP_VER 				0.0
#ifndef CONFIG_BIOS_VER
#define CONFIG_BIOS_VER				0.0
#endif
#define CONFIG_UBL_VER 				CONFIG_BIOS_VER
#define CONFIG_UBOOT_VER 			CONFIG_BIOS_VER
#define CFG_CBSIZE					CONFIG_SYS_CBSIZE
#define CONFIG_MTD_IDX 				4
#define CONFIG_MTD2_IDX 			6
#define CONFIG_MTD_IDX_MP 			9
#define CONFIG_KERNELX_REDUND		1	// 0:Off, 1:On
#define CONFIG_KERNEL_IDX 			1	// index of boot kernel
#define CONFIG_KERNEL2_IDX 			2	// index of boot kernel 2
#define CONFIG_FW_BOOTUP_COUNTER	0
#define MAX_FW_BOOTUP_COUNTER		2

#define CONFIG_DUT_MODEL 			"VPort"
#define CONFIG_DUT_MODEL_L 			"vport"
#define CONFIG_DUT_HOST 			"MOXA VPort Industrial IP Camera"
#define CONFIG_DUT_DESC 			"MOXA VPort Industrial IP Camera"
#define CONFIG_BURNIN_TIME 			40

#define D4_FWR_FILE_NAME 			"FWR_"CONFIG_DUT_MODEL_L".rom"
#define D4_FS_FILE_NAME 			"ubifs_ipnc.bin"
#define D4_KERNEL_FILE_NAME 		"uImage"
#define D4_UBOOT_FILE_NAME 			"uboot_vport.rom"
#define D4_UBL_FILE_NAME 			"ubl_vport.rom"
#define D4_MPFS_FILE_NAME 			"mp_ubifs_ipnc.bin"
#define D4_MPKERNEL_FILE_NAME 		"mp_uImage"

#define CONFIG_MP_FLAG_T0_BIOS		0x000
#define CONFIG_MP_FLAG_T1_BIOS		0x001
#define CONFIG_MP_FLAG_T1_MP		0x011
#define CONFIG_MP_FLAG_T2_MP		0x002
#define CONFIG_MP_FLAG_T2_MP_ERR	0x021
#define CONFIG_MP_FLAG_T3_BIOS		0x003
#define CONFIG_MP_FLAG_FIRMWARE		0x007
#define CONFIG_MP_FLAG_T2_T_MP		0x100
#define CONFIG_MP_FLAG_T2_T_MP_ERR	0x101

#define CONFIG_MP_FLAG 				CONFIG_MP_FLAG_T0_BIOS
#define CONFIG_IVA_FLAG 			0x00
#define CONFIG_FW_RECOVERY 			1
#define CONFIG_FW_RECOVERY_FILE		D4_FWR_FILE_NAME
#define CONFIG_FW_RECOVERY_FIX
#define CONFIG_T2_BOOTUP_COUNTER	0
#define CONFIG_SERIAL_NUMBER		000000000000

/* Hardware related */

/* Light Sensor Configuration */
#define LIGHT_SENSOR_ADDR			0x39

/* Thermal Sensor Configuration */
#define THERMAL_SENSOR_ADDR			0x48
#if defined(VPORT56) || defined(VPORT56_HIPOWER)
#define CONFIG_PI_ON_TEMP			0xe7	/* PI Heater ON temperature (-25C) */
#define CONFIG_PI_OFF_TEMP			0x1e	/* PI Heater OFF temperature (30C) */
#define CONFIG_SYS_ON_TEMP			0x0a	/* System ON temperature (10C) */
#define CONFIG_PI_OFF_OFFS			7		/* PI Heater OFF temperature offset (+7C) */
#define CONFIG_SYS_ON_OFFS			6		/* System ON temperature offset (+6C) */
#define CONFIG_SYS_ON_TIMEOUT		300		/* System ON time out (300 sec) */
#endif

/**
 * Physical Memory Map
 */

#if defined(CONFIG_TI814X_DDR3_4Gb)
#define CONFIG_NR_DRAM_BANKS		2		/* we have 2 banks of DRAM */
#define PHYS_DRAM_1					0x80000000	/* DRAM Bank #1 */
#define PHYS_DRAM_1_SIZE			0x40000000	/* 1 GB */
#define PHYS_DRAM_2					0xC0000000	/* DRAM Bank #2 */
#define PHYS_DRAM_2_SIZE			0x40000000	/* 1 GB */
#elif defined(CONFIG_TI814X_DDR3_2Gb)
#define CONFIG_NR_DRAM_BANKS		2		/* we have 2 banks of DRAM */
#define PHYS_DRAM_1					0x80000000	/* DRAM Bank #1 */
#define PHYS_DRAM_1_SIZE			0x20000000	/* 512 MB */
#define PHYS_DRAM_2					0xA0000000	/* DRAM Bank #2 */
#define PHYS_DRAM_2_SIZE			0x20000000	/* 512 MB */
#else
#define CONFIG_NR_DRAM_BANKS		2		/* we have 2 banks of DRAM */
#define PHYS_DRAM_1					0x80000000	/* DRAM Bank #1 */
#define PHYS_DRAM_1_SIZE			0x10000000	/* 256 MB */
#define PHYS_DRAM_2					0xB0000000	/* DRAM Bank #2 */
#define PHYS_DRAM_2_SIZE			0x10000000	/* 256 MB */
#endif

/**
 * Platform/Board specific defs
 */
#define CONFIG_SYS_CLK_FREQ			20000000
#define CONFIG_SYS_TIMERBASE		0x4802E000

/*
 * NS16550 Configuration
 */
#define CONFIG_SERIAL_MULTI			1
#define CONFIG_SYS_NS16550
#define CONFIG_SYS_NS16550_SERIAL
#define CONFIG_SYS_NS16550_REG_SIZE	(-4)
#define CONFIG_SYS_NS16550_CLK		(48000000)
#define CONFIG_SYS_NS16550_COM1		0x48020000	/* Base EVM has UART0 */
#define CONFIG_SYS_NS16550_COM2		0x48022000	//UART1
#define CONFIG_SYS_NS16550_COM3		0x481A6000	//UART3
#define CONFIG_SYS_NS16550_COM4		0x481A8000	//UART4

#define CONFIG_BAUDRATE				115200
#define CONFIG_SYS_BAUDRATE_TABLE	{ 110, 300, 600, 1200, 2400, \
4800, 9600, 14400, 19200, 28800, 38400, 56000, 57600, 115200 }

#if defined(CONFIG_SERIAL_MULTI)
#define CONFIG_BAUDRATE_0			CONFIG_BAUDRATE
#define CONFIG_BAUDRATE_1			CONFIG_BAUDRATE
#define CONFIG_BAUDRATE_2			CONFIG_BAUDRATE
#define CONFIG_BAUDRATE_3			CONFIG_BAUDRATE
#define CONFIG_BAUDRATE_0_ITEM		"baudrate0"
#define CONFIG_BAUDRATE_1_ITEM		"baudrate1"
#define CONFIG_BAUDRATE_2_ITEM		"baudrate2"
#define CONFIG_BAUDRATE_3_ITEM		"baudrate3"
#endif

/*
 * select serial console configuration
 */
#define CONFIG_SERIAL1				1
#define CONFIG_CONS_INDEX			1
#define CONFIG_SYS_CONSOLE_INFO_QUIET


#define DEFAULT_NAME				"serial"
#if defined(VPORT66)
#define CAMERA_NAME					"eserial2"
#define CAMERA_BAUDRATE_ITEM		CONFIG_BAUDRATE_2_ITEM
#define CAMERA_BAUDRATE				38400	// for Zoom camera module (MH310/MH322/MH326/MN330)
#define PTCTRL_NAME					"eserial2"
#define PTCTRL_BAUDRATE_ITEM		CONFIG_BAUDRATE_2_ITEM
#define PTCTRL_BAUDRATE				19200	// for MCU (PT ctrl)
#else
#define RS485_NAME					"eserial1"
#define RS485_BAUDRATE_ITEM			CONFIG_BAUDRATE_1_ITEM
#define RS485_BAUDRATE				CONFIG_BAUDRATE
#define CAMERA_NAME					"eserial2"
#define CAMERA_BAUDRATE_ITEM		CONFIG_BAUDRATE_2_ITEM
#define CAMERA_BAUDRATE				38400	// for Zoom camera module (MH310/MH322/MH326/MN330)
#define RS485_2WIRE					/* 2-wire RS-485 */
#endif

#define CONFIG_CMD_TERMINAL	/* built-in Serial Terminal */

#if defined(CONFIG_NO_ETH)
#undef CONFIG_CMD_NET
#else
#define CONFIG_CMD_NET
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_PING
//#define CONFIG_CMD_SNTP
#define CONFIG_CMD_MII
#endif

#if defined(CONFIG_CMD_NET)
#define CONFIG_DRIVER_TI_CPSW
#define CONFIG_MII
#define CONFIG_BOOTP_DEFAULT
#define CONFIG_BOOTP_DNS
#define CONFIG_BOOTP_DNS2
#define CONFIG_BOOTP_SEND_HOSTNAME
#define CONFIG_BOOTP_GATEWAY
#define CONFIG_BOOTP_SUBNETMASK
#define CONFIG_NET_RETRY_COUNT			10
#define CONFIG_NET_MULTI
#define CONFIG_PHY_GMII_MODE
#if defined(VPORT66)
#define CONFIG_PHY_GIGE
#undef CONFIG_PHY_GMII_MODE
#endif
#if defined(VPORT461A)
#define CONFIG_MV88E6063_SWITCH
#define CONFIG_PHY_RMII_MODE
#undef CONFIG_PHY_GMII_MODE
#endif
/* increase network receive packet buffer count for reliable TFTP */
#define CONFIG_SYS_RX_ETH_BUFFER		16
#endif

#if defined(CONFIG_SYS_NO_FLASH)
# define CONFIG_ENV_IS_NOWHERE
#endif

/* NAND support */
#ifdef CONFIG_NAND
#define CONFIG_CMD_NAND
#define CONFIG_NAND_TI81XX
//#define GPMC_NAND_ECC_LP_x8_LAYOUT 	1
#define GPMC_NAND_ECC_LP_x16_LAYOUT 	1
#define NAND_BASE						(0x08000000)
#define CONFIG_SYS_NAND_ADDR			NAND_BASE	/* physical address */
							/* to access nand */
#define CONFIG_SYS_NAND_BASE			NAND_BASE	/* physical address */
							/* to access nand at */
							/* CS0 */
#define CONFIG_SYS_MAX_NAND_DEVICE		1		/* Max number of NAND */

#define ISP_NAND
#define FLASH_TEST_SIZE 				SZ_128K

/*****************************************************************************/
/*  NAND Flash Layout                                                                                                      */
/*****************************************************************************/
#if defined(VPORT56_HIPOWER)

// locations in NAND flash
#define UBL_FLASH						0x00000000
#define UBOOT_FLASH						0x00020000
#define ENV2_FLASH						0x00240000
#define ENV1_FLASH						0x00260000
#define KERNEL_FLASH					0x00280000
#define ROOTFS_FLASH					0x006C0000
#define KERNEL2_FLASH					0x029C0000
#define ROOTFS2_FLASH					0x02E00000
#define DATA1_FLASH						0x05100000
#define DATA2_FLASH						0x054E0000
#define MPKERNEL_FLASH					0x058C0000
#define MPROOTFS_FLASH					0x05D00000
#define MPDATA_FLASH					0x09180000
#define DSP1_FLASH						0x09D80000
#define DSP2_FLASH						0x0CA80000

// max. sizes
#define UBL_SIZE						(1 * SZ_128K)
#define UBOOT_SIZE						(17 * SZ_128K)
#define ENV2_SIZE						(1 * SZ_128K)
#define ENV1_SIZE						(1 * SZ_128K)
#define KERNEL_SIZE						(34 * SZ_128K)
#define ROOTFS_SIZE						(280 * SZ_128K)
#define KERNEL2_SIZE					(34 * SZ_128K)
#define ROOTFS2_SIZE					(280 * SZ_128K)
#define DATA1_SIZE						(31 * SZ_128K)
#define DATA2_SIZE						(31 * SZ_128K)
#define MPKERNEL_SIZE					(34 * SZ_128K)
#define MPROOTFS_SIZE					(420 * SZ_128K)
#define MPDATA_SIZE						(96 * SZ_128K)
#define DSP1_SIZE						(360 * SZ_128K)
#define DSP2_SIZE						(428 * SZ_128K)

#define TEST_FLASH						DSP2_FLASH
#define TEST_FLASH_SIZE					DSP2_SIZE

#elif defined(VPORT461A) || defined(VPORT36_2MP)

// locations in NAND flash
#define UBL_FLASH						0x00000000
#define UBOOT_FLASH						0x00020000
#define ENV1_FLASH						0x00240000
#define ENV2_FLASH						0x00260000
#define KERNEL_FLASH					0x00280000
#define ROOTFS_FLASH					0x006C0000
#define KERNEL2_FLASH					0x047C0000
#define ROOTFS2_FLASH					0x04C00000
#define DATA1_FLASH						0x08D00000
#define MPKERNEL_FLASH					0x094C0000
#define MPROOTFS_FLASH					0x09900000
#define MPDATA_FLASH					0x0CD80000
#define CONFIG_FLASH					0x0D980000
#define CONFIG2_FLASH					0x0DC80000
#define LOG_FLASH						0x0DF80000
#define LOG2_FLASH						0x0E280000
#define RESERVE_FLASH					0x0E580000

// max. sizes
#define UBL_SIZE						(1 * SZ_128K)
#define UBOOT_SIZE						(17 * SZ_128K)
#define ENV1_SIZE						(1 * SZ_128K)
#define ENV2_SIZE						(1 * SZ_128K)
#define KERNEL_SIZE						(34 * SZ_128K)
#define ROOTFS_SIZE						(520 * SZ_128K)
#define KERNEL2_SIZE					(34 * SZ_128K)
#define ROOTFS2_SIZE					(520 * SZ_128K)
#define DATA1_SIZE						(62 * SZ_128K)
#define MPKERNEL_SIZE					(34 * SZ_128K)
#define MPROOTFS_SIZE					(420 * SZ_128K)
#define MPDATA_SIZE						(96 * SZ_128K)
#define CONFIG_SIZE						(24 * SZ_128K)
#define CONFIG2_SIZE					(24 * SZ_128K)
#define LOG_SIZE						(24 * SZ_128K)
#define LOG2_SIZE						(24 * SZ_128K)
#define RESERVE_SIZE					(212 * SZ_128K)

#define TEST_FLASH						RESERVE_FLASH
#define TEST_FLASH_SIZE					RESERVE_SIZE

#define CONFIG_SYS_REDUNDAND_ENVIRONMENT

#define MTDPARTS_DEFAULT "mtdparts=nand0:"\
	"0x00020000@0x00000000(ubl),"\
	"0x00220000@0x00020000(u-boot),"\
	"0x00020000@0x00240000(env),"\
	"0x00020000@0x00260000(env2),"\
	"0x00440000@0x00280000(kernel),"\
	"0x04100000@0x006C0000(rootfs),"\
	"0x00440000@0x047C0000(kernel2),"\
	"0x04100000@0x04C00000(rootfs2),"\
	"0x007C0000@0x08D00000(data),"\
	"0x00440000@0x094C0000(mpkernel),"\
	"0x03480000@0x09900000(mprootfs),"\
	"0x00C00000@0x0CD80000(mpdata),"\
	"0x00300000@0x0D980000(config),"\
	"0x00300000@0x0DC80000(config2),"\
	"0x00300000@0x0DF80000(log),"\
	"0x00300000@0x0E280000(log2),"\
	"-(reserved)"

#else

// locations in NAND flash
#define UBL_FLASH						0x00000000
#define UBOOT_FLASH						0x00020000
#define ENV1_FLASH						0x00260000
#define KERNEL_FLASH					0x00280000
#define ROOTFS_FLASH					0x006C0000
#define KERNEL2_FLASH					0x03B40000
#define ROOTFS2_FLASH					0x03F80000
#define DATA1_FLASH						0x07400000
#define MPKERNEL_FLASH					0x07BC0000
#define MPROOTFS_FLASH					0x08000000
#define MPDATA_FLASH					0x0B480000
#define CONFIG_FLASH					0x0C080000
#define ENV2_FLASH						0x0C380000
#define BACKUP_FLASH					0x0C3A0000
#define RESERVE_FLASH					0x0C9A0000

// max. sizes
#define UBL_SIZE						(1 * SZ_128K)
#define UBOOT_SIZE						(18 * SZ_128K)
#define ENV1_SIZE						(1 * SZ_128K)
#define KERNEL_SIZE						(34 * SZ_128K)
#define ROOTFS_SIZE						(420 * SZ_128K)
#define KERNEL2_SIZE					(34 * SZ_128K)
#define ROOTFS2_SIZE					(420 * SZ_128K)
#define DATA1_SIZE						(62 * SZ_128K)
#define MPKERNEL_SIZE					(34 * SZ_128K)
#define MPROOTFS_SIZE					(420 * SZ_128K)
#define MPDATA_SIZE						(96 * SZ_128K)
#define CONFIG_SIZE						(24 * SZ_128K)
#define ENV2_SIZE						(1 * SZ_128K)
#define BACKUP_SIZE						(48 * SZ_128K)
#define RESERVE_SIZE					(436 * SZ_128K)

#define TEST_FLASH						RESERVE_FLASH
#define TEST_FLASH_SIZE					RESERVE_SIZE

#define MTDPARTS_DEFAULT "mtdparts=nand0:"\
	"0x00020000@0x00000000(ubl),"\
	"0x00240000@0x00020000(u-boot),"\
	"0x00020000@0x00260000(env),"\
	"0x00440000@0x00280000(kernel),"\
	"0x03480000@0x006C0000(rootfs),"\
	"0x00440000@0x03B40000(kernel2),"\
	"0x03480000@0x03F80000(rootfs2),"\
	"0x007C0000@0x07400000(data),"\
	"0x00440000@0x07BC0000(mpkernel),"\
	"0x03480000@0x08000000(mprootfs),"\
	"0x00C00000@0x0B480000(mpdata),"\
	"0x00020000@0x0C380000(env2),"\
	"-(reserved)"

#endif
/*****************************************************************************/

#endif							/* devices */

/* ENV in NAND */
#if defined(CONFIG_NAND_ENV)
#undef CONFIG_ENV_IS_NOWHERE
#define CONFIG_ENV_IS_IN_NAND			1
#ifdef CONFIG_ENV_IS_IN_NAND
#define CONFIG_SYS_MAX_FLASH_SECT		520		/* max number of sectors in a chip */
#define CONFIG_SYS_MAX_FLASH_BANKS		2		/* max number of flash banks */
#define CONFIG_SYS_MONITOR_LEN			(256 << 10)	/* Reserve 2 sectors */
#define CONFIG_SYS_FLASH_BASE			boot_flash_base
#define CONFIG_SYS_MONITOR_BASE			CONFIG_SYS_FLASH_BASE
#define MNAND_ENV_OFFSET				ENV1_FLASH	/* environment starts here */
#define CONFIG_SYS_ENV_SECT_SIZE		boot_flash_sec
#define CONFIG_ENV_OFFSET				boot_flash_off
#define CONFIG_ENV_ADDR					MNAND_ENV_OFFSET

#ifdef CONFIG_SYS_REDUNDAND_ENVIRONMENT
#define CONFIG_ENV_OFFSET_REDUND		ENV2_FLASH
#define CONFIG_ENV_SIZE_REDUND			(CONFIG_ENV_SIZE)
#endif

#endif	/* CONFIG_ENV_IS_IN_NAND */

#define CONFIG_CMD_UBIFS
#define CONFIG_CMD_UBI
#define CONFIG_CMD_MTDPARTS
#define CONFIG_MTD_DEVICE
#define CONFIG_MTD_PARTITIONS
#define CONFIG_RBTREE
#define CONFIG_LZO
#define MTDIDS_DEFAULT "nand0=nand0"

#ifndef __ASSEMBLY__
extern unsigned int boot_flash_base;
extern volatile unsigned int boot_flash_env_addr;
extern unsigned int boot_flash_off;
extern unsigned int boot_flash_sec;
extern unsigned int boot_flash_type;
#endif
#endif /* NAND support */

/* GPIO setting */
#define GPIO_LED_ON			0
#define GPIO_LED_OFF		1

#define GPIO_LOW			0
#define GPIO_HIGH			1

#define GPIO_INPUT			0
#define GPIO_OUTPUT			1

#define GPIO_SYSBUTTON_ON	1
#define GPIO_SYSBUTTON_OFF	0

#if defined(VPORT66)
#define GPIO_AIC_RSTn		((0*32) + 8)	//GP0[8] (OUT) AIC_RSTn
#define GPIO_FAN_CON		((0*32) + 14)	//GP0[14] (OUT) Fan_con
#define GPIO_HEATERSYS_INT	((0*32) + 17)	//GP0[17] (IN) Heatersys_int
#define GPIO_HEATER_SYS		((0*32) + 22)	//GP0[22] (OUT) Heater_sys
#define GPIO_HEATER_CAM		((0*32) + 26)	//GP0[26] (OUT) Heater_cam
#define GPIO_SD_WP			((0*32) + 29)	//GP0[29) (IN) SD1_WPn
#define GPIO_SD_CD			((0*32) + 30)	//GP0[30) (IN) SD1_CDn
#define GPIO_SD_EN			((0*32) + 31)	//GP0[31) (OUT) SD1_EN
#define GPIO_FLASH_WP		((1*32) + 0)	//GP1[0] (OUT) FLASH_WP
#define GPIO_HEATERCAM_INT	((1*32) + 4)	//GP1[4] (IN) Heatercam_int
#define GPIO_SPI_CSC		((1*32) + 6)	//GP1[6] (OUT) SPI_CSC
#define GPIO_FAN_INT		((1*32) + 7)	//GP1[7] (IN) Fan_int
#define GPIO_MCU_RST		((2*32) + 0)	//GP2[0] (OUT) MCU_RST
#define GPIO_FPGA_RST		((2*32) + 1)	//GP2[1] (OUT) FPGA_RST
#define GPIO_RE_SETING		((2*32) + 21)	//GP2[21] (IN) Reset button
#define GPIO_PHY_RESET		((2*32) + 22)	//GP2[22] (OUT) ENET_RSTn
#define GPIO_PHY_LINKSTAT	((2*32) + 23)	//GP2[23] (IN) E_LINKSTS
#define GPIO_CAM_RST		((2*32) + 25)	//GP2[25] (OUT) CAM_REST
#define GPIO_RTC_INTn		((2*32) + 26)	//GP2[26] (IN) RTC_INTn
#define GPIO_THERMAL_SOUT	((2*32) + 31)	//GP2[31] (IN) THERMAL_SOUT
#define GPIO_ARN_IN			((3*32) + 7)	//GP3[7] (IN) DI
#define GPIO_ARN_OUT		((3*32) + 8)	//GP3[8] (OUT) DO
#define GPIO_LED_STATE		((3*32) + 12)	//GP3[12] (OUT) LED_G
#define GPIO_LED_SYS		((3*32) + 13)	//GP3[13] (OUT) LED_R
#define GPIO_FPGA_PROG		((3*32) + 17)	//GP3[17] (OUT) FPGA_PROG
#define GPIO_SYSBUTTON		GPIO_RE_SETING
#define GPIO_DI				GPIO_ARN_IN
#define GPIO_DO				GPIO_ARN_OUT
#define GPIO_SYSLED_GREEN	GPIO_LED_STATE
#define GPIO_SYSLED_RED		GPIO_LED_SYS
#undef GPIO_SYSBUTTON_ON
#undef GPIO_SYSBUTTON_OFF
#define GPIO_SYSBUTTON_ON	0
#define GPIO_SYSBUTTON_OFF	1
#elif defined(VPORT56)
#define GPIO_AIC_RSTn		((0*32) + 8)	//GP0[8] (OUT) AIC_RSTn
#define GPIO_SD_WP			((0*32) + 29)	//GP0[29) (IN) SD1_WPn
#define GPIO_SD_CD			((0*32) + 30)	//GP0[30) (IN) SD1_CDn
#define GPIO_SD_EN			((0*32) + 31)	//GP0[31) (OUT) SD1_EN
#define GPIO_FLASH_WP		((1*32) + 0)	//GP1[0] (OUT) FLASH_WP
#define GPIO_RS485_4W		((1*32) + 17)	//GP1[17] (OUT) RS485_4W
#define GPIO_RS485_ENT		((1*32) + 18)	//GP1[18] (OUT) RS485_ENT
#define GPIO_RS485_ENRn		((1*32) + 26)	//GP1[26] (OUT) RS485_ENRn
#define GPIO_RE_SETING		((2*32) + 21)	//GP2[21] (IN) Reset button
#define GPIO_PHY_RESET		((2*32) + 22)	//GP2[22] (OUT) ENET_RSTn
#define GPIO_PHY_LINKSTAT	((2*32) + 23)	//GP2[23] (IN) E_LINKSTS
#define GPIO_RTC_INTn		((2*32) + 26)	//GP2[26] (IN) RTC_INTn
#define GPIO_HEATER_CAM		((2*32) + 29)	//GP2[29] (OUT) Heater_cam
#define GPIO_CAM_RST		((2*32) + 30)	//GP2[30] (OUT) CAM_REST
#define GPIO_THERMAL_SOUT	((2*32) + 31)	//GP2[31] (IN) THERMAL_SOUT
#define GPIO_ARN_IN			((3*32) + 7)	//GP3[7] (IN) DI
#define GPIO_ARN_OUT		((3*32) + 8)	//GP3[8] (OUT) DO
#define GPIO_LED_STATE		((3*32) + 12)	//GP3[12] (OUT) LED_G
#define GPIO_LED_SYS		((3*32) + 13)	//GP3[13] (OUT) LED_R
#define GPIO_SYSBUTTON		GPIO_RE_SETING
#define GPIO_DI				GPIO_ARN_IN
#define GPIO_DO				GPIO_ARN_OUT
#define GPIO_SYSLED_GREEN	GPIO_LED_STATE
#define GPIO_SYSLED_RED		GPIO_LED_SYS
#elif defined(VPORT36_2MP)
#define GPIO_AIC_RSTn		((0*32) + 8)	//GP0[8] (OUT) AIC_RSTn
#define GPIO_SD_WP			((0*32) + 29)	//GP0[29) (IN) SD1_WPn
#define GPIO_SD_CD			((0*32) + 30)	//GP0[30) (IN) SD1_CDn
#define GPIO_SD_EN			((0*32) + 31)	//GP0[31) (OUT) SD1_EN
#define GPIO_FLASH_WP		((1*32) + 0)	//GP1[0] (OUT) FLASH_WP
#define GPIO_SPI_NCS1		((1*32) + 7)	//GP1[7] (OUT) SPI_nCS1 to Motor Drive
#define GPIO_SPI_NCS0		((1*32) + 16)	//GP1[16] (OUT) SPI_nCS0 to MN34041
#define GPIO_MD_ADIN7B		((2*32) + 2)	//GP2[2] (OUT) MD_ADIN7B
#define GPIO_MD_ADIN7A		((2*32) + 4)	//GP2[4] (OUT) MD_ADIN7A
#define GPIO_RE_SETING		((2*32) + 21)	//GP2[21] (IN) Reset button
#define GPIO_PHY_RESET		((2*32) + 22)	//GP2[22] (OUT) ENET_RSTn
#define GPIO_PHY_LINKSTAT	((2*32) + 23)	//GP2[23] (IN) E_LINKSTS
#define GPIO_MD_RESET		((2*32) + 24)	//GP2[24] (OUT) MD_RESET Motor Driver Reset
#define GPIO_CAM_RST		((2*32) + 25)	//GP2[25] (OUT) CAM_REST
#define GPIO_RTC_INTn		((2*32) + 26)	//GP2[26] (IN) RTC_INTn
#define GPIO_MD_BUSY		((2*32) + 27)	//GP2[27] (IN) MD_BUSY/MON Motor Driver(Transfer Busy)
#define GPIO_MD_POWER		((2*32) + 28)	//GP2[28] (OUT) MD_POWER
#define GPIO_RS485_4W		((2*32) + 29)	//GP2[29] (OUT) RS485_4W
#define GPIO_RS485_ENRn		((2*32) + 30)	//GP2[30] (OUT) RS485_ENRn
#define GPIO_RS485_ENT		((2*32) + 31)	//GP2[31] (OUT) RS485_ENT
#define GPIO_ARN_IN			((3*32) + 7)	//GP3[7] (IN) DI
#define GPIO_ARN_OUT		((3*32) + 8)	//GP3[8] (OUT) DO
#define GPIO_LED_STATE		((3*32) + 12)	//GP3[12] (OUT) LED_G
#define GPIO_LED_SYS		((3*32) + 13)	//GP3[13] (OUT) LED_R
#define GPIO_SYSBUTTON		GPIO_RE_SETING
#define GPIO_DI				GPIO_ARN_IN
#define GPIO_DO				GPIO_ARN_OUT
#define GPIO_SYSLED_GREEN	GPIO_LED_STATE
#define GPIO_SYSLED_RED		GPIO_LED_SYS
#elif defined(VPORT461A)
#define GPIO_AIC_RSTn		((0*32) + 8)	//GP0[8] (OUT) AIC_RSTn
#define GPIO_SD_WP			((0*32) + 29)	//GP0[29) (IN) SD1_WPn
#define GPIO_SD_CD			((0*32) + 30)	//GP0[30) (IN) SD1_CDn
#define GPIO_SD_EN			((0*32) + 31)	//GP0[31) (OUT) SD1_EN
#define GPIO_FLASH_WP		((1*32) + 0)	//GP1[0] (OUT) FLASH_WP
#define GPIO_RE_SETING		((2*32) + 21)	//GP2[21] (IN) RE_SETING
#define GPIO_CAM_RST		((2*32) + 25)	//GP2[25] (OUT) CAM_REST
#define GPIO_RTC_INTn		((2*32) + 26)	//GP2[26] (IN) RTC_INTn
#define GPIO_PHY_LINKSTAT	((3*32) + 7)	//GP3[7] (IN) E_LINKSTS
#define GPIO_PHY_RESET		((3*32) + 8)	//GP3[8] (OUT) ENET_RSTn
#define GPIO_LED_SD			((3*32) + 9)	//GP3[9] (OUT) SD_LEDn
#define GPIO_LED_PTZ		((3*32) + 10)	//GP3[10] (OUT) PTZ_LEDn
#define GPIO_LED_VIDEO		((3*32) + 11)	//GP3[11] (OUT) VIDEO_LEDn
#define GPIO_LED_STAT_G		((3*32) + 12)	//GP3[12] (OUT) STAT_GLEDn
#define GPIO_LED_STAT_R		((3*32) + 13)	//GP3[13] (OUT) STAT_RLEDn
#define GPIO_LED_FAIL		((3*32) + 14)	//GP3[14] (OUT) FAIL_LEDn
#define GPIO_SYSBUTTON		GPIO_RE_SETING
#define GPIO_SYSLED_GREEN	GPIO_LED_STAT_G
#define GPIO_SYSLED_RED		GPIO_LED_STAT_R
#undef GPIO_SYSBUTTON_ON
#undef GPIO_SYSBUTTON_OFF
#define GPIO_SYSBUTTON_ON	0
#define GPIO_SYSBUTTON_OFF	1
#define TEST_LED_BLINK_ON_MEM_TEST
#endif

#ifndef CONFIG_TI814X_OPTI_CONFIG
/* SPI support */
#ifdef CONFIG_SPI
#define CONFIG_OMAP3_SPI
#define CONFIG_MTD_DEVICE
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_WINBOND
#define CONFIG_CMD_SF
#define CONFIG_SF_DEFAULT_SPEED			(75000000)
//#define CONFIG_CODEC_AIC26				1
#define CONFIG_CMD_SPI
#define CONFIG_DEFAULT_SPI_BUS			1
#endif

/* ENV in SPI */
#if defined(CONFIG_SPI_ENV)
#undef CONFIG_ENV_IS_NOWHERE
#define CONFIG_ENV_IS_IN_SPI_FLASH		1
#ifdef CONFIG_ENV_IS_IN_SPI_FLASH
#define CONFIG_SYS_FLASH_BASE			(0)
#define SPI_FLASH_ERASE_SIZE			(4 * 1024) /* sector size of SPI flash */
#define CONFIG_SYS_ENV_SECT_SIZE		(2 * SPI_FLASH_ERASE_SIZE) /* env size */
#define CONFIG_ENV_SECT_SIZE			(CONFIG_SYS_ENV_SECT_SIZE)
#define CONFIG_ENV_OFFSET				(96 * SPI_FLASH_ERASE_SIZE)
#define CONFIG_ENV_ADDR					(CONFIG_ENV_OFFSET)
#define CONFIG_SYS_MAX_FLASH_SECT		(1024) /* no of sectors in SPI flash */
#define CONFIG_SYS_MAX_FLASH_BANKS		(1)
#endif
#endif /* SPI support */

/* ENV in MMC */
#if defined(CONFIG_MMC_ENV)
#undef CONFIG_ENV_IS_NOWHERE
#define CONFIG_ENV_IS_IN_MMC			1
#define CONFIG_SYS_MMC_ENV_DEV			0
#undef CONFIG_ENV_SIZE
#undef CONFIG_ENV_OFFSET
#define CONFIG_ENV_OFFSET				(6 * 64 * 1024)
#define CONFIG_ENV_SIZE					(8 * 1024)
#endif /* MMC support */

/* NOR support */
#if defined(CONFIG_NOR)
#undef CONFIG_CMD_NAND			/* Remove NAND support */
#undef CONFIG_NAND_TI81XX
#undef CONFIG_SKIP_LOWLEVEL_INIT
#define CONFIG_TI814X_CONFIG_DDR
#define CONFIG_SETUP_PLL
#define CONFIG_TI814X_EVM_DDR3
#undef CONFIG_ENV_IS_NOWHERE
#ifdef CONFIG_SYS_MALLOC_LEN
#undef CONFIG_SYS_MALLOC_LEN
#endif
#define CONFIG_SYS_FLASH_USE_BUFFER_WRITE 1
#define CONFIG_SYS_MALLOC_LEN			(0x100000)
#define CONFIG_SYS_FLASH_CFI
#define CONFIG_FLASH_CFI_DRIVER
#define CONFIG_FLASH_CFI_MTD
#define CONFIG_SYS_MAX_FLASH_SECT		512
#define CONFIG_SYS_MAX_FLASH_BANKS		1
#define CONFIG_SYS_FLASH_BASE			(0x08000000)
#define CONFIG_SYS_MONITOR_BASE			CONFIG_SYS_FLASH_BASE
#define CONFIG_ENV_IS_IN_FLASH			1
#define NOR_SECT_SIZE					(128 * 1024)
#define CONFIG_SYS_ENV_SECT_SIZE		(NOR_SECT_SIZE)
#define CONFIG_ENV_SECT_SIZE			(NOR_SECT_SIZE)
#define CONFIG_ENV_OFFSET				(2 * NOR_SECT_SIZE)
#define CONFIG_ENV_ADDR					(CONFIG_SYS_FLASH_BASE + CONFIG_ENV_OFFSET)
#define CONFIG_MTD_DEVICE
#endif	/* NOR support */


/* No I2C support in 1st stage */
#ifdef CONFIG_I2C

#define CONFIG_CMD_I2C
#define CONFIG_CMD_DATE
#define CONFIG_I2C_MULTI_BUS
#define CONFIG_SYS_RTC_BUS_NUM			0
#define CONFIG_TPS65911_I2C				1
//# define CONFIG_RTC_TPS65911			1
#define CONFIG_CODEC_AIC3104			1
#define CONFIG_SENSOR_MT9J003			1
#define CONFIG_HARD_I2C					1
#define CONFIG_SYS_I2C_SPEED			100000
#define CONFIG_SYS_I2C_SLAVE			1
#define CONFIG_SYS_I2C_BUS				0
#define CONFIG_SYS_I2C_BUS_SELECT		1
#define CONFIG_DRIVER_TI81XX_I2C		1

#define CONFIG_RTC_ISL1208				1
#define CONFIG_SYS_I2C_RTC_ADDR			0x6f

/* EEPROM definitions */
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN			3
#define CONFIG_SYS_I2C_EEPROM_ADDR				0x50
#define CONFIG_SYS_EEPROM_PAGE_WRITE_BITS		6
#define CONFIG_SYS_EEPROM_PAGE_WRITE_DELAY_MS	20

#endif

/* HSMMC support */
#ifdef CONFIG_MMC
#define CONFIG_CMD_MMC			1
#define CONFIG_GENERIC_MMC
#define CONFIG_OMAP_HSMMC
#define CONFIG_DOS_PARTITION	1
#define CONFIG_CMD_FAT			1
#endif
#endif	/* CONFIG_TI814X_OPTI_CONFIG */
/* U-boot Version */
#define CONFIG_VERSION_VARIABLE
#define CONFIG_IDENT_STRING "DM8127_IPNC_3.80.00"
/* Unsupported features */
#undef CONFIG_USE_IRQ

#endif	  /* ! __CONFIG_TI8148_IPNC_H */

