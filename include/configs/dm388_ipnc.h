/*
 * Copyright (C) 2011, Texas Instruments, Incorporated
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

#ifndef __CONFIG_DM385_IPNC_H
#define __CONFIG_DM385_IPNC_H

#ifdef ENVAPI_IN_LINUX
#include "EnvAPI/sizes.h"
#else
#include <asm/sizes.h>
#include <model.h>
#endif

/*
 *#define CONFIG_DM385_NO_RUNTIME_PG_DETECT
 */

/* select the DDR3 Freq and timing paramets */
//#define LOW_POWER_OPP100_MODE	/* 20150626-bobby: for test */
#ifdef LOW_POWER_OPP100_MODE
#define CONFIG_DM385_DDR3_400 /* Values supported 400,533 */
#else
#define CONFIG_DM385_DDR3_533 /* Values supported 400,533 */
#endif


#define XMK_STR(x)	#x
#define MK_STR(x)	XMK_STR(x)

/* Display CPU info */
#define CONFIG_DISPLAY_CPUINFO		1

/* In the 1st stage we have just 110K, so cut down wherever possible */
#ifdef CONFIG_DM385_MIN_CONFIG
/* enable d-cache only on 2nd stage */
#define CONFIG_SYS_DCACHE_OFF

#define CONFIG_CMD_MEMORY	/* for mtest */
#undef CONFIG_GZIP
#undef CONFIG_ZLIB
#undef CONFIG_SYS_HUSH_PARSER
#define CONFIG_CMD_LOADB	/* loadb */
#define CONFIG_CMD_LOADY	/* loady */
#define CONFIG_SETUP_PLL
#define CONFIG_DM385_CONFIG_DDR
#define CONFIG_DM385_IPNC_DDR3
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
#define CONFIG_EXTRA_ENV_SETTINGS \
	"verify=yes\0" \
	"bootcmd=nand read 0x81000000 0x20000 0x80000; go 0x81000000\0" \
	""

#elif defined(CONFIG_SD_BOOT)		/* Autoload the 2nd stage from SD */
#define CONFIG_MMC			1
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

#elif defined(CONFIG_TI814X_OPTI_CONFIG)		/* Optimized code */
#include <config_cmd_default.h>

#define CONFIG_ZERO_BOOTDELAY_CHECK
#define CONFIG_ENV_SIZE				0x2000
#define CONFIG_SYS_MALLOC_LEN		(CONFIG_ENV_SIZE + (32 * 1024))
#define CONFIG_ENV_OVERWRITE
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_PROMPT		"DM388_IPNC#"
#if 0
#define CONFIG_SYS_HUSH_PARSER		/* Use HUSH parser to allow command parsing */
#define CONFIG_SYS_PROMPT_HUSH_PS2	"> "
#endif
#define CONFIG_CMDLINE_TAG        	1	/* enable passing of ATAGs  */
#define CONFIG_SETUP_MEMORY_TAGS  	1
#define CONFIG_INITRD_TAG	  		1	/* Required for ramdisk support */
#define CONFIG_BOOTDELAY			1	/* set to negative value for no autoboot */
#define CONFIG_NAND					1
#define CONFIG_SETUP_PLL
#define CONFIG_DM385_CONFIG_DDR
#define CONFIG_DM385_IPNC_DDR3

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
/* size in bytes reserved for initial data */
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
	"console=ttyO0,115200n8 rootwait=1 rw ubi.mtd=${mtd_idx},2048 rootfstype=ubifs root=ubi0:rootfs init=/init mem=80M vram=4M notifyk.vpssm3_sva=0xBFD00000 ip=${ipaddr} eth=${ethaddr} cmemk.phys_start=0x85000000 cmemk.phys_end=0x89000000 cmemk.allowOverlap=1 earlyprintk"

#define CONFIG_BOOTCOMMAND \
	"ipnc_ff_init 1;nboot ${loadaddr} 0 ${kernelflash}; bootm"

#define CONFIG_MPBOOTCOMMAND \
	"ipnc_ff_init 1;nboot ${loadaddr} 0 ${mpkernelflash}; bootm"

#define BACKDOOR_TIMEOUT			3 /* MMmoxaie backdoor timeout sec. */
#define CONFIG_PREBOOT				"moxamm"
#define CONFIG_CMD_MOXAMM
#define CONFIG_AUTO_COMPLETE		/* add autocompletion support	*/
#endif

#define CONFIG_SYS_GBL_DATA_SIZE	128
#define CONFIG_MISC_INIT_R		1
#ifndef CONFIG_DM385_MIN_CONFIG
#define CONFIG_DM388_ASCIIART		1
#endif
#define CONFIG_CMD_CACHE
#define CONFIG_CMD_ECHO
#define CONFIG_CMD_MISC

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
#define CONFIG_HW_WATCHDOG
#define UBL_WDT_TIMEOUT_SEC			6			/* Watchdog time out (sec.) (in UBL) */
#define WDT_TIMEOUT_SEC				60			/* Watchdog time out (sec.)*/
#define WDT_TIMEOUT_BASE			0x00008000	/* ~1 sec. */

/* Light Sensor Configuration */
#define LIGHT_SENSOR_ADDR			0x39

/* Thermal Sensor Configuration */
#define THERMAL_SENSOR_ADDR			0x48
#if 0
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
/* we have 1 bank of DRAM */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_DRAM_1					0x80000000	/* DRAM Bank #1 */
//#define CONFIG_DM388_DDR3_4GB_SINGLE
#ifdef CONFIG_DM388_DDR3_4GB_SINGLE
#define PHYS_DRAM_1_SIZE			0x20000000	/* 512 MB */
#else
#define PHYS_DRAM_1_SIZE			0x40000000	/* 1 GB */
#endif
#define CONFIG_DM388_DDR3_4Gb
//#define CONFIG_DM388_DDR3_2Gb

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
//#define RS485_NAME					"eserial1"

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
//#define CONFIG_PHY_GIGE
#define CONFIG_PHY_GMII_MODE
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
#define UBOOT_SIZE						(17 * SZ_128K)
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
#endif

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

//wensen
#define CONFIG_SYS_FUNCTION
#define CONFIG_SYS_FUNCTION_RED_ENV		0x01;

#define CONFIG_SYS_REDUNDAND_ENVIRONMENT
#define CONFIG_ENV_OFFSET_REDUND		ENV2_FLASH
#define CONFIG_ENV_SIZE_REDUND			(CONFIG_ENV_SIZE)
#endif

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

#define GPIO_SYSBUTTON_ON	0
#define GPIO_SYSBUTTON_OFF	1

#define GPIO_LED_STATE		((0*32) + 1)	//(GP0[1], O) (Green)
#define GPIO_LED_SYS		((0*32) + 2)	//(GP0[2], O) (RED)
#define GPIO_LED_CONTROL	((0*32) + 3)	//(GP0[3], O)
#define GPIO_RE_SETING		((0*32) + 4)	//(GP0[4], I)
#define GPIO_PHY_INTB		((0*32) + 5)	//(GP0[5], I)
#define GPIO_MIC_SEL		((0*32) + 6)	//(GP0[6], I)
#define GPIO_AIC_RSTn		((0*32) + 9)	//(GP0[9], O)
#define GPIO_CAM_PWDN		((0*32) + 13)	//(GP0[13], O)
#define GPIO_GS_INTn		((1*32) + 0)	//(GP1[0], I)
#define GPIO_SD_CD			((1*32) + 6)	//(GP1[6], I)
#define GPIO_PHY_LINKSTAT	((1*32) + 10)	//(GP1[10], I)
#define GPIO_FLASH_WP		((1*32) + 19)	//(GP1[19], O)
#define GPIO_PHY_RESET		((1*32) + 20)	//(GP1[20], O)
#define GPIO_SD_EN			((1*32) + 21)	//(GP1[21], O)
#define GPIO_RTC_INTn		((1*32) + 22)	//(GP1[22], I)
#define GPIO_CAM_RST		((2*32) + 18)	//(GP2[18], O)
#define GPIO_ARN_IN			((3*32) + 7)	//(GP3[7], I)

#define GPIO_SYSBUTTON		GPIO_RE_SETING
#define GPIO_DI				GPIO_ARN_IN
#define GPIO_SYSLED_GREEN	GPIO_LED_STATE
#define GPIO_SYSLED_RED		GPIO_LED_SYS
#define GPIO_SYSLED_CONTROL	GPIO_LED_CONTROL


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
#define CONFIG_DM385_CONFIG_DDR
#define CONFIG_SETUP_PLL
#define CONFIG_DM385_IPNC_DDR3
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
/*# define CONFIG_SENSOR_MT9J003		1*/
#define CONFIG_SENSOR_AR0331			1
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
#define CONFIG_IDENT_STRING " DM388_IPNC_3.80.00"
/* Unsupported features */
#undef CONFIG_USE_IRQ

#endif	  /* ! __CONFIG_DM385_IPNC_H */

