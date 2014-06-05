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

#include <common.h>
#include <asm/cache.h>
#include <asm/arch/cpu.h>
#include <asm/arch/ddr_defs.h>
#include <asm/arch/hardware.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mmc_host_def.h>
#include <asm/arch/clock.h>
#include <asm/arch/mem.h>
#include <asm/arch/nand.h>
#include <linux/mtd/nand.h>
#include <nand.h>
#include <net.h>
#include <miiphy.h>
#include <netdev.h>
#ifdef CONFIG_TPS65911_I2C
#include <tps65911.h>
#endif
#ifdef CONFIG_GENERIC_MMC
#include <asm/arch/mmc_host_def.h>
#endif
#ifdef CONFIG_HW_WATCHDOG
#include <watchdog.h>
#endif

#if 0
#define __raw_readl(a)		(*(volatile unsigned int *)(a))
#define __raw_writel(v, a)	(*(volatile unsigned int *)(a) = (v))
#define __raw_readw(a)		(*(volatile unsigned short *)(a))
#define __raw_writew(v, a)	(*(volatile unsigned short *)(a) = (v))
#endif

typedef enum{
	RS485_IN_DIR,
	RS485_OUT_DIR
}RS485_dir_t;

#define CLKCTRL 				0x4
#define TENABLE 				0x8
#define TENABLEDIV 				0xC
#define M2NDIV  				0x10
#define MN2DIV 				    0x14
#define STATUS 				    0x24

#define PLL_BASE_ADDRESS        (0x481C5000)
#define VIDEO_0_PLL_BASE        (PLL_BASE_ADDRESS+0x1A0)
#define VIDEO_1_PLL_BASE        (PLL_BASE_ADDRESS+0x1D0)
#define HDMI_PLL_BASE           (PLL_BASE_ADDRESS+0x200)
#define EMIF_CLK_GATE 			(0x48140694)
#define SECSS_CLK_SRC 			(0x481C52EC)

DECLARE_GLOBAL_DATA_PTR;

#ifdef CONFIG_TI814X_CONFIG_DDR
static void cmd_macro_config(u32 ddr_phy, u32 inv_clk_out,
			 u32 ctrl_slave_ratio_cs0, u32 cmd_dll_lock_diff)
{
	u32 ddr_phy_base = (DDR_PHY0 == ddr_phy) ?
			 DDR0_PHY_BASE_ADDR : DDR1_PHY_BASE_ADDR;

	__raw_writel(inv_clk_out,
		 ddr_phy_base + CMD1_REG_PHY_INVERT_CLKOUT_0);
	__raw_writel(inv_clk_out,
		 ddr_phy_base + CMD0_REG_PHY_INVERT_CLKOUT_0);
	__raw_writel(inv_clk_out,
		 ddr_phy_base + CMD2_REG_PHY_INVERT_CLKOUT_0);

	__raw_writel(((ctrl_slave_ratio_cs0 << 10) | ctrl_slave_ratio_cs0),
		ddr_phy_base + CMD0_REG_PHY_CTRL_SLAVE_RATIO_0);
	__raw_writel(((ctrl_slave_ratio_cs0 << 10) | ctrl_slave_ratio_cs0),
		ddr_phy_base + CMD1_REG_PHY_CTRL_SLAVE_RATIO_0);
	__raw_writel(((ctrl_slave_ratio_cs0 << 10) | ctrl_slave_ratio_cs0),
		 ddr_phy_base + CMD2_REG_PHY_CTRL_SLAVE_RATIO_0);

	__raw_writel(cmd_dll_lock_diff,
		 ddr_phy_base + CMD0_REG_PHY_DLL_LOCK_DIFF_0);
	__raw_writel(cmd_dll_lock_diff,
		 ddr_phy_base + CMD1_REG_PHY_DLL_LOCK_DIFF_0);
	__raw_writel(cmd_dll_lock_diff,
		 ddr_phy_base + CMD2_REG_PHY_DLL_LOCK_DIFF_0);
}

static void data_macro_config(u32 macro_num, u32 emif, u32 rd_dqs_cs0,
		u32 wr_dqs_cs0, u32 fifo_we_cs0, u32 wr_data_cs0)
{
	/* 0xA4 is size of each data macro mmr region.
	 * phy1 is at offset 0x400 from phy0
	 */
	u32 base = (macro_num * 0xA4) + (emif * 0x400);

	__raw_writel(((rd_dqs_cs0 << 10) | rd_dqs_cs0),
		(DATA0_REG_PHY0_RD_DQS_SLAVE_RATIO_0 + base));
	__raw_writel(((wr_dqs_cs0 << 10) | wr_dqs_cs0),
		(DATA0_REG_PHY0_WR_DQS_SLAVE_RATIO_0 + base));
	__raw_writel(((PHY_WRLVL_INIT_CS1_DEFINE << 10) |
		PHY_WRLVL_INIT_CS0_DEFINE),
		(DATA0_REG_PHY0_WRLVL_INIT_RATIO_0 + base));
	__raw_writel(((PHY_GATELVL_INIT_CS1_DEFINE << 10) |
		PHY_GATELVL_INIT_CS0_DEFINE),
		(DATA0_REG_PHY0_GATELVL_INIT_RATIO_0 + base));
	__raw_writel(((fifo_we_cs0 << 10) | fifo_we_cs0),
		(DATA0_REG_PHY0_FIFO_WE_SLAVE_RATIO_0 + base));
	__raw_writel(((wr_data_cs0 << 10) | wr_data_cs0),
		(DATA0_REG_PHY0_WR_DATA_SLAVE_RATIO_0 + base));
	__raw_writel(PHY_DLL_LOCK_DIFF_DEFINE,
		(DATA0_REG_PHY0_DLL_LOCK_DIFF_0 + base));
}
#endif

int is_ddr3(void)
{
	/*
	 * PG1.0 by default uses DDR2 &  PG2.1 uses DDR3
	 * To use PG2.1 and DDR2 enable #define CONFIG_TI814X_EVM_DDR2
	 * in "include/configs/ti8148_evm.h"
	 */
	if (PG2_1 <= get_cpu_rev())
		#ifdef CONFIG_TI814X_EVM_DDR2
			return 0;
		#else
			return 1;
		#endif
	else
		return 0;
}

static void pll_config(u32, u32, u32, u32, u32);
void dss_pll_config(void);
void hdmi_pll_config(void);
void video0_pll_config(void);
void video1_pll_config(void);

void ipnc_ff_pll_init(int);

static void dsp_pll_config(void);
static u32 pll_dco_freq_sel(u32 clkout_dco);
static u32 pll_sigma_delta_val(u32 clkout_dco);
#ifdef CONFIG_SETUP_PLL
static void pll_config(u32, u32, u32, u32, u32);
#if 0
static void pcie_pll_config(void);
#endif
static void audio_pll_config(void);
static void sata_pll_config(void);
static void modena_pll_config(void);
static void l3_pll_config(void);
static void ddr_pll_config(void);
static void iss_pll_config(void);
static void iva_pll_config(void);
static void usb_pll_config(void);
#endif

static void unlock_pll_control_mmr(void);
#ifdef CONFIG_DRIVER_TI_CPSW
static void cpsw_pad_config(void);
#endif
//static void nor_pad_config_mux(void);
static void gpio_init(void);

#ifdef CONFIG_TPS65911_I2C
extern int do_date(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);
static void power_control(void);
static void show_time(void);
#endif

#if defined(CONFIG_CODEC_AIC26) || defined(CONFIG_CODEC_AIC3104)
int Audio_HW_Reset(void);
#endif

//// GPIO function ////
u32 GPIO_BaseAddr(int bank)
{
	switch(bank){
		case 0: return GPIO0_BASE; break;
		case 1: return GPIO1_BASE; break;
		case 2: return GPIO2_BASE; break;
		case 3: return GPIO3_BASE; break;
		default: return 0; break;
	}
	return 0;
}

u32 GPIO_SetClrAddr(int set)
{
	if(set) return GPIO_SETDATAOUT;
	else return GPIO_CLEARDATAOUT;
}

void GPIO_OutEn(int bank, int pin, int en) //GPIO Output Enable
{
	u32  add, val;
	add = (GPIO_BaseAddr(bank) + GPIO_OE);
	val = __raw_readl(add);
	if(!en) val |= (1<<pin);
	else val &=~(1<<pin);
	__raw_writel(val, add);
}

int GPIO_Input(int bank, int pin)
{
	return (__raw_readl((GPIO_BaseAddr(bank) + GPIO_DATAIN)) & (1<<pin)) ? 1 : 0;
}

void GPIO_Output(int bank, int pin, int set)
{
	__raw_writel((1<<pin), (GPIO_BaseAddr(bank) + GPIO_SetClrAddr(set)));
}

void GPIO_Output_High(int bank, int pin)
{
	__raw_writel((1<<pin), (GPIO_BaseAddr(bank) + GPIO_SETDATAOUT));
}

void GPIO_Output_Low(int bank, int pin)
{
	__raw_writel((1<<pin), (GPIO_BaseAddr(bank) + GPIO_CLEARDATAOUT));
}

/* GPIO0 */
void GPIO0_OutEn(int pin, int en) //GPIO Output Enable
{
	u32  add, val;
	add = (GPIO0_BASE + GPIO_OE);
	val = __raw_readl(add);
	if(!en) val |= (1<<pin);
	else val &=~(1<<pin);
	__raw_writel(val, add);
}

int GPIO0_Input(int pin)
{
	return (__raw_readl((GPIO0_BASE + GPIO_DATAIN)) & (1<<pin)) ? 1 : 0;
}

void GPIO0_Output_High(int pin)
{
	__raw_writel((1<<pin), (GPIO0_BASE + GPIO_SETDATAOUT));
}

void GPIO0_Output_Low(int pin)
{
	__raw_writel((1<<pin), (GPIO0_BASE + GPIO_CLEARDATAOUT));
}

/* GPIO1 */
void GPIO1_OutEn(int pin, int en) //GPIO Output Enable
{
	u32  add, val;
	add = (GPIO1_BASE + GPIO_OE);
	val = __raw_readl(add);
	if(!en) val |= (1<<pin);
	else val &=~(1<<pin);
	__raw_writel(val, add);
}

int GPIO1_Input(int pin)
{
	return (__raw_readl((GPIO1_BASE + GPIO_DATAIN)) & (1<<pin)) ? 1 : 0;
}

void GPIO1_Output_High(int pin)
{
	__raw_writel((1<<pin), (GPIO1_BASE + GPIO_SETDATAOUT));
}

void GPIO1_Output_Low(int pin)
{
	__raw_writel((1<<pin), (GPIO1_BASE + GPIO_CLEARDATAOUT));
}

/* GPIO2 */
void GPIO2_OutEn(int pin, int en) //GPIO Output Enable
{
	u32  add, val;
	add = (GPIO2_BASE + GPIO_OE);
	val = __raw_readl(add);
	if(!en) val |= (1<<pin);
	else val &=~(1<<pin);
	__raw_writel(val, add);
}

int GPIO2_Input(int pin)
{
	return (__raw_readl((GPIO2_BASE + GPIO_DATAIN)) & (1<<pin)) ? 1 : 0;
}

void GPIO2_Output_High(int pin)
{
	__raw_writel((1<<pin), (GPIO2_BASE + GPIO_SETDATAOUT));
}

void GPIO2_Output_Low(int pin)
{
	__raw_writel((1<<pin), (GPIO2_BASE + GPIO_CLEARDATAOUT));
}

/* GPIO3 */
void GPIO3_OutEn(int pin, int en) //GPIO Output Enable
{
	u32  add, val;
	add = (GPIO3_BASE + GPIO_OE);
	val = __raw_readl(add);
	if(!en) val |= (1<<pin);
	else val &=~(1<<pin);
	__raw_writel(val, add);
}

int GPIO3_Input(int pin)
{
	return (__raw_readl((GPIO3_BASE + GPIO_DATAIN)) & (1<<pin)) ? 1 : 0;
}

void GPIO3_Output_High(int pin)
{
	__raw_writel((1<<pin), (GPIO3_BASE + GPIO_SETDATAOUT));
}

void GPIO3_Output_Low(int pin)
{
	__raw_writel((1<<pin), (GPIO3_BASE + GPIO_CLEARDATAOUT));
}

void sys_led_R_ON(void)
{
	GPIO_Output(3, 13, 0); //GP3_13 output low
}

void sys_led_R_OFF(void)
{
	GPIO_Output(3, 13, 1); //GP3_13 output high
}

void sys_led_R_reverse(void)
{
	if(GPIO_Input(3, 13)) sys_led_R_ON();
	else sys_led_R_OFF();
}

void sys_led_G_ON(void)
{
	GPIO_Output(3, 12, 0); //GP3_12 output low
}

void sys_led_G_OFF(void)
{
	GPIO_Output(3, 12, 1); //GP3_12 output high
}

void sys_led_G_reverse(void)
{
	if(GPIO_Input(3, 12)) sys_led_G_ON();
	else sys_led_G_OFF();
}

void sys_led_RG_ON(void)
{
	sys_led_R_ON();
	sys_led_G_ON();
}

void sys_led_RG_OFF(void)
{
	sys_led_R_OFF();
	sys_led_G_OFF();
}

void sys_led_RG_reverse(void)
{
	sys_led_R_reverse();
	sys_led_G_reverse();
}

void RS485_SetTx(void)
{
	GPIO_Output(1, 18, 1);	//GP1_18-RS485_ENT output high
	GPIO_Output(1, 26, 1);	//GP1_26-RS485_ENRn output high
}

void RS485_SetRx(void)
{
	GPIO_Output(1, 18, 0);	//GP1_18-RS485_ENT output low
	GPIO_Output(1, 26, 0);	//GP1_26-RS485_ENRn output low
}

void DO_High(void)
{
	GPIO_Output(3, 8, 1);	//GP3_8 output high
}

void DO_Low(void)
{
	GPIO_Output(3, 8, 0);	//GP3_8 output low
}

u32 DI_state(void)
{
	return GPIO_Input(3, 7);	//GP3_7
}

u32 DO_state(void)
{
	return GPIO_Input(3, 8);	//GP3_8
}

u32 resetkey_state(void)
{
	return GPIO_Input(2, 21);	//GP2_21
}

u32 lightsen_state(void)
{
	return GPIO_Input(3, 9);	//GP3_9
}

void sdcard_enable(void)
{
	GPIO_Output(0, 31, 1);	//GP0_31 output high
}

void sdcard_disable(void)
{
	GPIO_Output(0, 31, 0);	//GP0_31 output low
}

int sdcard_WPn_state(void)
{
	return GPIO_Input(0, 29);	//GP0_29
}

int sdcard_CDn_state(void)
{
	return GPIO_Input(0, 30);	//GP0_30
}

int sdcard_En_state(void)
{
	return GPIO_Input(0, 31);	//GP0_31
}

void Heater_Cam_ON(void)
{
#ifdef VPORT66
	GPIO_Output(0, 26, 1);	//GP0_26 output high
#else
	GPIO_Output(2, 29, 1);	//GP2_29 output high
#endif
}

void Heater_Cam_OFF(void)
{
#ifdef VPORT66
	GPIO_Output(0, 26, 0);	//GP0_26 output low
#else
	GPIO_Output(2, 29, 0);	//GP2_29 output low
#endif
}

int Heatercam_int_state(void)
{
	return GPIO_Input(1, 4);	//GP1_4
}

void Heater_Sys_ON(void)
{
	GPIO_Output(0, 22, 1);	//GP0_22 output high
}

void Heater_Sys_OFF(void)
{
	GPIO_Output(0, 22, 0);	//GP0_22 output low
}

int Heatersys_int_state(void)
{
	return GPIO_Input(0, 17);	//GP0_17
}

void Camera_Power_ON(void)
{
#ifdef VPORT66
	GPIO_Output(2, 25, 1);	//GP2_25 output high
#else
	GPIO_Output(2, 30, 1);	//GP2_30 output high
#endif
}

void Camera_Power_OFF(void)
{
#ifdef VPORT66
	GPIO_Output(2, 25, 0);	//GP2_25 output low
#else
	GPIO_Output(2, 30, 0);	//GP2_30 output low
#endif
}

void Fan_con_ON(void)
{
	GPIO_Output(0, 14, 1);	//GP0_14 output high
}

void Fan_con_OFF(void)
{
	GPIO_Output(0, 14, 0);	//GP0_14 output low
}

int Fan_int_state(void)
{
	return GPIO_Input(1, 7);	//GP1_7
}

/*
 * spinning delay to use before udelay works
 */
static inline void delay(unsigned long loops)
{
	__asm__ volatile ("1:\n" "subs %0, %1, #1\n"
		"bne 1b" : "=r" (loops) : "0"(loops));
}

/*
 * Basic board specific setup
 */
int board_init(void)
{
	u32 regVal;

	/* Do the required pin-muxing before modules are setup */
	set_muxconf_regs();

    //nor_pad_config_mux();

	/* setup RMII_REFCLK to be sourced from audio_pll */
	__raw_writel(0x4, RMII_REFCLK_SRC);

	if (PG2_1 <= get_cpu_rev()) {
#ifndef VPORT66
		/*program GMII_SEL register for G/MII mode */
		__raw_writel(0x00,GMII_SEL);
#else
		/*program GMII_SEL register for RGMII mode */
		__raw_writel(0x30a,GMII_SEL);
#endif
	}
	gpio_init();
	/* Get Timer and UART out of reset */

	/* UART softreset */
	regVal = __raw_readl(UART_SYSCFG);
	regVal |= 0x2;
	__raw_writel(regVal, UART_SYSCFG);
	while( (__raw_readl(UART_SYSSTS) & 0x1) != 0x1);

	/* Disable smart idle */
	regVal = __raw_readl(UART_SYSCFG);
	regVal |= (1<<3);
	__raw_writel(regVal, UART_SYSCFG);

	/* mach type passed to kernel */
	gd->bd->bi_arch_number = MACH_TYPE_TI8148IPNC;

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_DRAM_1 + 0x100;
	gpmc_init();
#ifdef CONFIG_TPS65911_I2C
	power_control();
#endif

#ifndef CONFIG_NOR
	/* GPMC will come up with default buswidth configuration,
    * we will override it based on BW pin CONFIG_STATUS register.
    * This is currently required only for NAND/NOR to
    * support 8/16 bit NAND/NOR part. Also we always use chipselect 0
    * for NAND/NOR boot.
    *
    * NOTE: This code is DM8168 EVM specific, hence we are using CS 0.
    * Also, even for other boot modes user is expected to
    * on/off the BW pin on the EVM.
    */
	//gpmc_set_cs_buswidth(0, get_sysboot_ipnc_bw());
	gpmc_set_cs_buswidth(0, get_sysboot_bw());
#endif
	return 0;
}

/*
 * sets uboots idea of sdram size
 */
int dram_init(void)
{
	/* Fill up board info */
	gd->bd->bi_dram[0].start = PHYS_DRAM_1;
	gd->bd->bi_dram[0].size = PHYS_DRAM_1_SIZE;

	gd->bd->bi_dram[1].start = PHYS_DRAM_2;
	gd->bd->bi_dram[1].size = PHYS_DRAM_2_SIZE;

	return 0;
}

#ifdef CONFIG_SERIAL_TAG
/* *********************************************************
 * * get_board_serial() - setup to pass kernel board serial
 * * returns: board serial number
 * **********************************************************
 */
void get_board_serial(struct tag_serialnr *serialnr)
{
	/* ToDo: read eeprom and return*/
	serialnr->high = 0x0;
	serialnr->low = 0x0;
}
#endif

#ifdef CONFIG_REVISION_TAG
/**********************************************************
 * * get_board_rev() - setup to pass kernel board revision
 * * returns: revision
 * ********************************************************
 */
u32 get_board_rev(void)
{
	/* ToDo: read eeprom */
	return 0x0;
}
#endif

int misc_init_r (void)
{

	#ifdef CONFIG_TI814X_MIN_CONFIG
	printf("The 2nd stage U-Boot will now be auto-loaded\n");
	printf("Please do not interrupt the countdown till "
		"TI8148_EVM prompt if 2nd stage is already flashed\n");
	#endif

#ifndef CONFIG_TI814X_OPTI_CONFIG
	#ifdef CONFIG_TI814X_ASCIIART
	int i = 0, j = 0;
	char ti814x[28][54] = {
"                                                      ",
"                                                      ",
"													   ",
"													   ",
"													   ",
"													   ",
"													   ",
"													   ",
"													   ",
"													   ",
"                                                      ",
"  M           M     OOOOO     X      X        A       ",
"  MM         MM   O       O    X    X        A A      ",
"  M M       M M  O         O    X  X        A   A     ",
"  M  M     M  M O           O    XX        A     A    ",
"  M   M   M   M O           O    XX       A       A   ",
"  M    M M    M  O         O    X  X     AAAAAAAAAAA  ",
"  M     M     M   O       O    X    X   A           A ",
"  M           M     OOOOO     X      X A             A",
"                                                      ",
"        IIII     VV            VV    NNN      NN      ",
"         II       VV          VV     NNNN     NN      ",
"         II        VV        VV      NN NN    NN      ",
"         II         VV      VV       NN  NN   NN      ",
"         II          VV    VV        NN   NN  NN      ",
"         II           VV  VV         NN    NN NN      ",
"         II            VVVV          NN     NNNN      ",
"        IIII            VV           NN      NNN      "};
	for (i = 0; i<28; i++)
	{
		for(j = 0; j<54; j++)
			printf("%c",ti814x[i][j]);
			printf("\n");
	}
	printf("\n");
#endif
	#ifdef CONFIG_TPS65911_I2C
	show_time();
	#endif
#endif

	/* Enable CLKOUT0 */
	__raw_writel(0x20, CTRL_BASE + 0xA14); 
	__raw_writel(0x00, CLKOUT_MUX); 
	__raw_writel(0x80, PRCM_BASE + 0x100); /* Enable SYS_CLKOUT */

	return 0;
}

#ifdef CONFIG_TI814X_CONFIG_DDR
static void config_ti814x_ddr(void)
{
	int macro, emif;

	/*Enable the Power Domain Transition of L3 Fast Domain Peripheral*/
	__raw_writel(0x2, CM_DEFAULT_FW_CLKCTRL);
	/*Enable the Power Domain Transition of L3 Fast Domain Peripheral*/
	__raw_writel(0x2, CM_DEFAULT_L3_FAST_CLKSTCTRL);
	__raw_writel(0x2, CM_DEFAULT_EMIF_0_CLKCTRL); /*Enable EMIF0 Clock*/
	__raw_writel(0x2, CM_DEFAULT_EMIF_1_CLKCTRL); /*Enable EMIF1 Clock*/
	__raw_writel(0x2, CM_DEFAULT_DMM_CLKCTRL);

	/*Poll for L3_FAST_GCLK  & DDR_GCLK  are active*/
	while ((__raw_readl(CM_DEFAULT_L3_FAST_CLKSTCTRL) & 0x300) != 0x300);
	/*Poll for Module is functional*/
	while ((__raw_readl(CM_DEFAULT_EMIF_0_CLKCTRL)) != 0x2);
	while ((__raw_readl(CM_DEFAULT_EMIF_1_CLKCTRL)) != 0x2);
	while ((__raw_readl(CM_DEFAULT_DMM_CLKCTRL)) != 0x2);

	if (is_ddr3()) {
		cmd_macro_config(DDR_PHY0, DDR3_PHY_INVERT_CLKOUT_OFF,
				DDR3_PHY_CTRL_SLAVE_RATIO_CS0_DEFINE,
				PHY_CMD0_DLL_LOCK_DIFF_DEFINE);
		cmd_macro_config(DDR_PHY1, DDR3_PHY_INVERT_CLKOUT_OFF,
				DDR3_PHY_CTRL_SLAVE_RATIO_CS0_DEFINE,
				PHY_CMD0_DLL_LOCK_DIFF_DEFINE);

		for (emif = 0; emif <= DDR_PHY1; emif++) {
			data_macro_config(DATA_MACRO_0, emif,
				DDR3_PHY_RD_DQS_CS0_BYTE0,
				DDR3_PHY_WR_DQS_CS0_BYTE0,
				DDR3_PHY_RD_DQS_GATE_CS0_BYTE0,
				DDR3_PHY_WR_DATA_CS0_BYTE0);

			data_macro_config(DATA_MACRO_1, emif,
				DDR3_PHY_RD_DQS_CS0_BYTE1,
				DDR3_PHY_WR_DQS_CS0_BYTE1,
				DDR3_PHY_RD_DQS_GATE_CS0_BYTE1,
				DDR3_PHY_WR_DATA_CS0_BYTE1);

			data_macro_config(DATA_MACRO_2, emif,
				DDR3_PHY_RD_DQS_CS0_BYTE2,
				DDR3_PHY_WR_DQS_CS0_BYTE2,
				DDR3_PHY_RD_DQS_GATE_CS0_BYTE2,
				DDR3_PHY_WR_DATA_CS0_BYTE2);

			data_macro_config(DATA_MACRO_3, emif,
				DDR3_PHY_RD_DQS_CS0_BYTE3,
				DDR3_PHY_WR_DQS_CS0_BYTE3,
				DDR3_PHY_RD_DQS_GATE_CS0_BYTE3,
				DDR3_PHY_WR_DATA_CS0_BYTE3);
		}
	} else {
		cmd_macro_config(DDR_PHY0, PHY_INVERT_CLKOUT_DEFINE,
				DDR2_PHY_CTRL_SLAVE_RATIO_CS0_DEFINE,
				PHY_CMD0_DLL_LOCK_DIFF_DEFINE);
		cmd_macro_config(DDR_PHY1, PHY_INVERT_CLKOUT_DEFINE,
				DDR2_PHY_CTRL_SLAVE_RATIO_CS0_DEFINE,
				PHY_CMD0_DLL_LOCK_DIFF_DEFINE);

		for (emif = 0; emif <= DDR_PHY1; emif++) {
			for (macro = 0; macro <= DATA_MACRO_3; macro++) {
				data_macro_config(macro, emif,
					DDR2_PHY_RD_DQS_CS0_DEFINE,
					DDR2_PHY_WR_DQS_CS0_DEFINE,
					DDR2_PHY_RD_DQS_GATE_CS0_DEFINE,
					DDR2_PHY_WR_DATA_CS0_DEFINE);
			}
		}
	}

	/* DDR IO CTRL config */
	__raw_writel(DDR0_IO_CTRL_DEFINE, DDR0_IO_CTRL);
	__raw_writel(DDR1_IO_CTRL_DEFINE, DDR1_IO_CTRL);

	__raw_writel(__raw_readl(VTP0_CTRL_REG) | 0x00000040 , VTP0_CTRL_REG);
	__raw_writel(__raw_readl(VTP1_CTRL_REG) | 0x00000040 , VTP1_CTRL_REG);

	// Write 0 to CLRZ bit
	__raw_writel(__raw_readl(VTP0_CTRL_REG) & 0xfffffffe , VTP0_CTRL_REG);
	__raw_writel(__raw_readl(VTP1_CTRL_REG) & 0xfffffffe , VTP1_CTRL_REG);

	// Write 1 to CLRZ bit
	__raw_writel(__raw_readl(VTP0_CTRL_REG) | 0x00000001 , VTP0_CTRL_REG);
	__raw_writel(__raw_readl(VTP1_CTRL_REG) | 0x00000001 , VTP1_CTRL_REG);

	// Read VTP control registers & check READY bits
	while ((__raw_readl(VTP0_CTRL_REG) & 0x00000020) != 0x20);
	while ((__raw_readl(VTP1_CTRL_REG) & 0x00000020) != 0x20);

	/*
	 * Program the PG2.1 DMM to Access EMIF0 and EMIF1
	 * 512MB sections with 512-byte interleaving
	 */
	__raw_writel(PG2_1_DMM_LISA_MAP__0, DMM_LISA_MAP__0);
	__raw_writel(PG2_1_DMM_LISA_MAP__1, DMM_LISA_MAP__1);
	__raw_writel(PG2_1_DMM_LISA_MAP__2, DMM_LISA_MAP__2);
	__raw_writel(PG2_1_DMM_LISA_MAP__3, DMM_LISA_MAP__3);

	while (__raw_readl(DMM_LISA_MAP__0) != PG2_1_DMM_LISA_MAP__0);
	while (__raw_readl(DMM_LISA_MAP__1) != PG2_1_DMM_LISA_MAP__1);
	while (__raw_readl(DMM_LISA_MAP__2) != PG2_1_DMM_LISA_MAP__2);
	while (__raw_readl(DMM_LISA_MAP__3) != PG2_1_DMM_LISA_MAP__3);

	__raw_writel(0x80000000, DMM_PAT_BASE_ADDR);

	if (!is_ddr3()) {
		/*Program EMIF0 CFG Registers*/
		__raw_writel(DDR2_EMIF_READ_LATENCY, EMIF4_0_DDR_PHY_CTRL_1);
		__raw_writel(DDR2_EMIF_READ_LATENCY, EMIF4_0_DDR_PHY_CTRL_1_SHADOW);
		__raw_writel(DDR2_EMIF_TIM1, EMIF4_0_SDRAM_TIM_1);
		__raw_writel(DDR2_EMIF_TIM1, EMIF4_0_SDRAM_TIM_1_SHADOW);
		__raw_writel(DDR2_EMIF_TIM2, EMIF4_0_SDRAM_TIM_2);
		__raw_writel(DDR2_EMIF_TIM2, EMIF4_0_SDRAM_TIM_2_SHADOW);
		__raw_writel(DDR2_EMIF_TIM3, EMIF4_0_SDRAM_TIM_3);
		__raw_writel(DDR2_EMIF_TIM3, EMIF4_0_SDRAM_TIM_3_SHADOW);
		__raw_writel(DDR2_EMIF_SDRAM_CONFIG, EMIF4_0_SDRAM_CONFIG);

		__raw_writel(DDR_EMIF_REF_CTRL | DDR_EMIF_REF_TRIGGER,
						 EMIF4_0_SDRAM_REF_CTRL);
		__raw_writel(DDR_EMIF_REF_CTRL, EMIF4_0_SDRAM_REF_CTRL_SHADOW);
		__raw_writel(DDR2_EMIF_SDRAM_ZQCR, EMIF4_0_SDRAM_ZQCR);
		__raw_writel(DDR_EMIF_REF_CTRL, EMIF4_0_SDRAM_REF_CTRL);
		__raw_writel(DDR_EMIF_REF_CTRL, EMIF4_0_SDRAM_REF_CTRL_SHADOW);

		__raw_writel(DDR2_EMIF_REF_CTRL, EMIF4_0_SDRAM_REF_CTRL);
		__raw_writel(DDR2_EMIF_REF_CTRL, EMIF4_0_SDRAM_REF_CTRL_SHADOW);

		/*Program EMIF1 CFG Registers*/
		__raw_writel(DDR2_EMIF_READ_LATENCY, EMIF4_1_DDR_PHY_CTRL_1);
		__raw_writel(DDR2_EMIF_READ_LATENCY, EMIF4_1_DDR_PHY_CTRL_1_SHADOW);
		__raw_writel(DDR2_EMIF_TIM1, EMIF4_1_SDRAM_TIM_1);
		__raw_writel(DDR2_EMIF_TIM1, EMIF4_1_SDRAM_TIM_1_SHADOW);
		__raw_writel(DDR2_EMIF_TIM2, EMIF4_1_SDRAM_TIM_2);
		__raw_writel(DDR2_EMIF_TIM2, EMIF4_1_SDRAM_TIM_2_SHADOW);
		__raw_writel(DDR2_EMIF_TIM3, EMIF4_1_SDRAM_TIM_3);
		__raw_writel(DDR2_EMIF_TIM3, EMIF4_1_SDRAM_TIM_3_SHADOW);
		__raw_writel(DDR2_EMIF_SDRAM_CONFIG, EMIF4_1_SDRAM_CONFIG);

		__raw_writel(DDR_EMIF_REF_CTRL | DDR_EMIF_REF_TRIGGER,
						 EMIF4_1_SDRAM_REF_CTRL);
		__raw_writel(DDR_EMIF_REF_CTRL, EMIF4_1_SDRAM_REF_CTRL_SHADOW);
		__raw_writel(DDR2_EMIF_SDRAM_ZQCR, EMIF4_1_SDRAM_ZQCR);
		__raw_writel(DDR_EMIF_REF_CTRL, EMIF4_1_SDRAM_REF_CTRL);
		__raw_writel(DDR_EMIF_REF_CTRL, EMIF4_1_SDRAM_REF_CTRL_SHADOW);

		__raw_writel(DDR2_EMIF_REF_CTRL, EMIF4_1_SDRAM_REF_CTRL);
		__raw_writel(DDR2_EMIF_REF_CTRL, EMIF4_1_SDRAM_REF_CTRL_SHADOW);

	} else {
		/*Program EMIF0 CFG Registers*/
		__raw_writel(DDR3_EMIF_READ_LATENCY, EMIF4_0_DDR_PHY_CTRL_1);
		__raw_writel(DDR3_EMIF_READ_LATENCY, EMIF4_0_DDR_PHY_CTRL_1_SHADOW);
		__raw_writel(DDR3_EMIF_TIM1, EMIF4_0_SDRAM_TIM_1);
		__raw_writel(DDR3_EMIF_TIM1, EMIF4_0_SDRAM_TIM_1_SHADOW);
		__raw_writel(DDR3_EMIF_TIM2, EMIF4_0_SDRAM_TIM_2);
		__raw_writel(DDR3_EMIF_TIM2, EMIF4_0_SDRAM_TIM_2_SHADOW);
		__raw_writel(DDR3_EMIF_TIM3, EMIF4_0_SDRAM_TIM_3);
		__raw_writel(DDR3_EMIF_TIM3, EMIF4_0_SDRAM_TIM_3_SHADOW);
		__raw_writel(DDR3_EMIF_SDRAM_CONFIG, EMIF4_0_SDRAM_CONFIG);

		__raw_writel(DDR_EMIF_REF_CTRL | DDR_EMIF_REF_TRIGGER,
						 EMIF4_0_SDRAM_REF_CTRL);
		__raw_writel(DDR_EMIF_REF_CTRL, EMIF4_0_SDRAM_REF_CTRL_SHADOW);
		__raw_writel(DDR3_EMIF_SDRAM_ZQCR, EMIF4_0_SDRAM_ZQCR);
		__raw_writel(DDR_EMIF_REF_CTRL, EMIF4_0_SDRAM_REF_CTRL);
		__raw_writel(DDR_EMIF_REF_CTRL, EMIF4_0_SDRAM_REF_CTRL_SHADOW);

		__raw_writel(DDR3_EMIF_REF_CTRL, EMIF4_0_SDRAM_REF_CTRL);
		__raw_writel(DDR3_EMIF_REF_CTRL, EMIF4_0_SDRAM_REF_CTRL_SHADOW);

		/*Program EMIF1 CFG Registers*/
		__raw_writel(DDR3_EMIF_READ_LATENCY, EMIF4_1_DDR_PHY_CTRL_1);
		__raw_writel(DDR3_EMIF_READ_LATENCY, EMIF4_1_DDR_PHY_CTRL_1_SHADOW);
		__raw_writel(DDR3_EMIF_TIM1, EMIF4_1_SDRAM_TIM_1);
		__raw_writel(DDR3_EMIF_TIM1, EMIF4_1_SDRAM_TIM_1_SHADOW);
		__raw_writel(DDR3_EMIF_TIM2, EMIF4_1_SDRAM_TIM_2);
		__raw_writel(DDR3_EMIF_TIM2, EMIF4_1_SDRAM_TIM_2_SHADOW);
		__raw_writel(DDR3_EMIF_TIM3, EMIF4_1_SDRAM_TIM_3);
		__raw_writel(DDR3_EMIF_TIM3, EMIF4_1_SDRAM_TIM_3_SHADOW);
		__raw_writel(DDR3_EMIF_SDRAM_CONFIG, EMIF4_1_SDRAM_CONFIG);

		__raw_writel(DDR_EMIF_REF_CTRL | DDR_EMIF_REF_TRIGGER,
						 EMIF4_1_SDRAM_REF_CTRL);
		__raw_writel(DDR_EMIF_REF_CTRL, EMIF4_1_SDRAM_REF_CTRL_SHADOW);
		__raw_writel(DDR3_EMIF_SDRAM_ZQCR, EMIF4_1_SDRAM_ZQCR);
		__raw_writel(DDR_EMIF_REF_CTRL, EMIF4_1_SDRAM_REF_CTRL);
		__raw_writel(DDR_EMIF_REF_CTRL, EMIF4_1_SDRAM_REF_CTRL_SHADOW);

		__raw_writel(DDR3_EMIF_REF_CTRL, EMIF4_1_SDRAM_REF_CTRL);
		__raw_writel(DDR3_EMIF_REF_CTRL, EMIF4_1_SDRAM_REF_CTRL_SHADOW);
	}
}

#endif

void PLL_CLKOUT_ENABLE (u32 base)
{
    __raw_writel(__raw_readl((base + CLKCTRL)) | 0x00100000,(base + CLKCTRL));
	while (( (__raw_readl(base + STATUS)) & 0x00000020) != 0x00000020);
    __raw_writel(__raw_readl(base + CLKCTRL)| 0x00000001,(base + CLKCTRL));
}

void dss_pll_config(void)
{
	PLL_CLKOUT_ENABLE(DSS_PLL_BASE);
	pll_config(DSS_PLL_BASE,
			DSS_N, DSS_M,
			DSS_M2, DSS_CLKCTRL);
}

void hdmi_pll_config(void)
{
	PLL_CLKOUT_ENABLE(HDMI_PLL_BASE);
	pll_config(HDMI_PLL_BASE,
			HDMI_N, HDMI_M,
			HDMI_M2, HDMI_CLKCTRL);
}

void video0_pll_config(void)
{
	PLL_CLKOUT_ENABLE(VIDEO0_PLL_BASE);
	pll_config(VIDEO0_PLL_BASE,
			VIDEO0_N, VIDEO0_M,
			VIDEO0_M2, VIDEO0_CLKCTRL);
}

void video1_pll_config(void)
{
	PLL_CLKOUT_ENABLE(VIDEO1_PLL_BASE);
	pll_config(VIDEO1_PLL_BASE,
			VIDEO1_N, VIDEO1_M,
			VIDEO1_M2, VIDEO1_CLKCTRL);
}

static void dsp_pll_config()
{
	pll_config(DSP_PLL_BASE,
			DSP_N, DSP_M,
			DSP_M2, DSP_CLKCTRL);
}

#ifdef CONFIG_SETUP_PLL
static void audio_pll_config()
{
	pll_config(AUDIO_PLL_BASE,
			AUDIO_N, AUDIO_M,
			AUDIO_M2, AUDIO_CLKCTRL);
}

#if 0
static void pcie_pll_config()
{
	/* Powerdown both reclkp/n single ended receiver */
	__raw_writel(0x00000002, SERDES_REFCLK_CTRL);

	__raw_writel(0x00000000, PCIE_PLLCFG0);

	/* PCIe(2.5GHz) mode, 100MHz refclk, MDIVINT = 25,
	 * disable (50,100,125M) clks
	 */
	__raw_writel(0x00640000, PCIE_PLLCFG1);

	/* SSC Mantissa and exponent = 0 */
	__raw_writel(0x00000000, PCIE_PLLCFG2);

	/* TBD */
	__raw_writel(0x004008E0, PCIE_PLLCFG3);

	/* TBD */
	__raw_writel(0x0000609C, PCIE_PLLCFG4);

	/* pcie_serdes_cfg_misc */
	/* TODO: verify the address over here
	 * (CTRL_BASE + 0x6FC = 0x481406FC ???)
	 */
	//__raw_writel(0x00000E7B, 0x48141318);
	delay(3);

	/* Enable PLL LDO */
	__raw_writel(0x00000004, PCIE_PLLCFG0);
	delay(3);

	/* Enable DIG LDO, PLL LD0 */
	__raw_writel(0x00000014, PCIE_PLLCFG0);
	delay(3);

	/* Enable DIG LDO, ENBGSC_REF, PLL LDO */
	__raw_writel(0x00000016, PCIE_PLLCFG0);
	delay(3);
	__raw_writel(0x30000016, PCIE_PLLCFG0);
	delay(3);
	__raw_writel(0x70000016, PCIE_PLLCFG0);
	delay(3);

	/* Enable DIG LDO, SELSC, ENBGSC_REF, PLL LDO */
	__raw_writel(0x70000017, PCIE_PLLCFG0);
	delay(3);

	/* wait for ADPLL lock */
	while(__raw_readl(PCIE_PLLSTATUS) != 0x1);

}
#endif

static void sata_pll_config()
{
	//__raw_writel(0xC12C003C, SATA_PLLCFG1);
	__raw_writel(0xC12C001C, SATA_PLLCFG1);
	//__raw_writel(0x004008E0, SATA_PLLCFG3);
	__raw_writel(0x006008E0, SATA_PLLCFG3);
	delay(0xFFFF);

	__raw_writel(0x80000004, SATA_PLLCFG0);
	delay(0xFFFF);

	/* Enable PLL LDO */
	__raw_writel(0x80000014, SATA_PLLCFG0);
	delay(0xFFFF);

	/* Enable DIG LDO, ENBGSC_REF, PLL LDO */
	__raw_writel(0x80000016, SATA_PLLCFG0);
	delay(0xFFFF);

	__raw_writel(0xC0000017, SATA_PLLCFG0);
	delay(0xFFFF);

	/* wait for ADPLL lock */
	while(((__raw_readl(SATA_PLLSTATUS) & 0x01) == 0x0));

}

static void usb_pll_config()
{
	pll_config(USB_PLL_BASE,
			USB_N, USB_M,
			USB_M2, USB_CLKCTRL);
}

static void modena_pll_config()
{
	pll_config(MODENA_PLL_BASE,
			MODENA_N, MODENA_M,
			MODENA_M2, MODENA_CLKCTRL);
}

static void l3_pll_config()
{
	pll_config(L3_PLL_BASE,
			L3_N, L3_M,
			L3_M2, L3_CLKCTRL);
}

static void ddr_pll_config()
{
	pll_config(DDR_PLL_BASE,
			DDR_N, DDR_M,
			DDR_M2, DDR_CLKCTRL);
}

static void iss_pll_config()
{
	pll_config(ISS_PLL_BASE,
			ISS_N, ISS_M,
			ISS_M2, ISS_CLKCTRL);
}

static void iva_pll_config()
{
	pll_config(IVA_PLL_BASE,
			IVA_N, IVA_M,
			IVA_M2, IVA_CLKCTRL);
}
#endif

/*
 * select the HS1 or HS2 for DCO Freq
 * return : CLKCTRL
 */
static u32 pll_dco_freq_sel(u32 clkout_dco)
{
	if (clkout_dco >= DCO_HS2_MIN && clkout_dco < DCO_HS2_MAX)
		return SELFREQDCO_HS2;
	else if (clkout_dco >= DCO_HS1_MIN && clkout_dco < DCO_HS1_MAX)
		return SELFREQDCO_HS1;
	else
		return -1;

}
/*
 * select the sigma delta config
 * return: sigma delta val
 */
static u32 pll_sigma_delta_val(u32 clkout_dco)
{
	u32 sig_val = 0;
	float frac_div;

	frac_div = (float) clkout_dco / 250;
	frac_div = frac_div + 0.90;
	sig_val = (int)frac_div;
	sig_val = sig_val << 24;

	return sig_val;
}

/*
 * configure individual ADPLLJ
 */
static void pll_config(u32 base, u32 n, u32 m, u32 m2, u32 clkctrl_val)
{
	u32 m2nval, mn2val, read_clkctrl = 0, clkout_dco = 0;
	u32 sig_val = 0, hs_mod = 0;

	m2nval = (m2 << 16) | n;
	mn2val = m;

	/* calculate clkout_dco */
	clkout_dco = ((OSC_0_FREQ / (n+1)) * m);

	/* sigma delta & Hs mode selection skip for ADPLLS*/
	if (MODENA_PLL_BASE != base) {
		sig_val = pll_sigma_delta_val(clkout_dco);
		hs_mod = pll_dco_freq_sel(clkout_dco);
	}

	/* by-pass pll */
	read_clkctrl = __raw_readl(base + ADPLLJ_CLKCTRL);
	__raw_writel((read_clkctrl | 0x00800000), (base + ADPLLJ_CLKCTRL));
	while ((__raw_readl(base + ADPLLJ_STATUS) & 0x101) != 0x101);

	/* Clear TINITZ */
	read_clkctrl = __raw_readl(base + ADPLLJ_CLKCTRL);
	__raw_writel((read_clkctrl & 0xfffffffe), (base + ADPLLJ_CLKCTRL));

	/*
	 * ref_clk = 20/(n + 1);
	 * clkout_dco = ref_clk * m;
	 * clk_out = clkout_dco/m2;
	*/

	read_clkctrl = __raw_readl(base + ADPLLJ_CLKCTRL) & 0xffffe3ff;
	__raw_writel(m2nval, (base + ADPLLJ_M2NDIV));
	__raw_writel(mn2val, (base + ADPLLJ_MN2DIV));

	/* Skip for modena(ADPLLS) */
	if (MODENA_PLL_BASE != base) {
		__raw_writel(sig_val, (base + ADPLLJ_FRACDIV));
		__raw_writel((read_clkctrl | hs_mod), (base + ADPLLJ_CLKCTRL));
	}

	/* Load M2, N2 dividers of ADPLL */
	__raw_writel(0x1, (base + ADPLLJ_TENABLEDIV));
	__raw_writel(0x0, (base + ADPLLJ_TENABLEDIV));

	/* Loda M, N dividers of ADPLL */
	__raw_writel(0x1, (base + ADPLLJ_TENABLE));
	__raw_writel(0x0, (base + ADPLLJ_TENABLE));

	/* configure CLKDCOLDOEN,CLKOUTLDOEN,CLKOUT Enable BITS */
	read_clkctrl = __raw_readl(base + ADPLLJ_CLKCTRL) & 0xdfe5ffff;
	if (MODENA_PLL_BASE != base)
		__raw_writel((read_clkctrl | ADPLLJ_CLKCRTL_CLKDCO),
						base + ADPLLJ_CLKCTRL);

	/* Enable TINTZ and disable IDLE(PLL in Active & Locked Mode */
	read_clkctrl = __raw_readl(base + ADPLLJ_CLKCTRL) & 0xff7fffff;
	__raw_writel((read_clkctrl | 0x1), base + ADPLLJ_CLKCTRL);

	/* Wait for phase and freq lock */
	while ((__raw_readl(base + ADPLLJ_STATUS) & 0x600) != 0x600);

}

/*
 * Enable the clks & power for perifs (TIMER1, UART0,...)
 */
void per_clocks_enable(void)
{
	u32 temp;

	__raw_writel(0x2, CM_ALWON_L3_SLOW_CLKSTCTRL);

	/* TODO: No module level enable as in ti8148 ??? */
#if 0
	/* TIMER 1 */
	__raw_writel(0x2, CM_ALWON_TIMER_1_CLKCTRL);
#endif
	/* Selects OSC0 (20MHz) for DMTIMER1 */
	temp = __raw_readl(DMTIMER_CLKSRC);
	temp &= ~(0x7 << 3);
	temp |= (0x4 << 3);
	__raw_writel(temp, DMTIMER_CLKSRC);

#if 0
	while(((__raw_readl(CM_ALWON_L3_SLOW_CLKSTCTRL) & (0x80000<<1)) >> (19+1)) != 1);
	while(((__raw_readl(CM_ALWON_TIMER_1_CLKCTRL) & 0x30000)>>16) !=0);
#endif
	__raw_writel(0x2,(DM_TIMER1_BASE + 0x54));
	while(__raw_readl(DM_TIMER1_BASE + 0x10) & 1);

	__raw_writel(0x1,(DM_TIMER1_BASE + 0x38));

	/* UARTs */
	__raw_writel(0x2, CM_ALWON_UART_0_CLKCTRL);
	while(__raw_readl(CM_ALWON_UART_0_CLKCTRL) != 0x2);

	__raw_writel(0x2, CM_ALWON_UART_1_CLKCTRL);
	while(__raw_readl(CM_ALWON_UART_1_CLKCTRL) != 0x2);

	__raw_writel(0x2, CM_ALWON_UART_2_CLKCTRL);
	while(__raw_readl(CM_ALWON_UART_2_CLKCTRL) != 0x2);

	__raw_writel(0x2, CM_ALWON_UART_3_CLKCTRL);
	while(__raw_readl(CM_ALWON_UART_3_CLKCTRL) != 0x2);

	/* Selects UART3_CLK_SOURCE & UART4_CLK_SOURCE to SYSCLK10 */
	temp = __raw_readl(McBSP_UART_CLKSRC);
	temp &= ~(0x3 << 3);	//UART3_CLK_SOURCE
	temp |= (0x1 << 3);		//set to SYSCLK10
	temp &= ~(0x3 << 5);	//UART4_CLK_SOURCE
	temp |= (0x1 << 5);		//set to SYSCLK10
	__raw_writel(temp, McBSP_UART_CLKSRC);

	while((__raw_readl(CM_ALWON_L3_SLOW_CLKSTCTRL) & 0x2100) != 0x2100);

	/* GPIO0 */
	__raw_writel(0x2, CM_ALWON_GPIO_0_CLKCTRL);
	while(__raw_readl(CM_ALWON_GPIO_0_CLKCTRL) != 0x2);
	__raw_writel(0x2, CM_ALWON_GPIO_1_CLKCTRL);
	while(__raw_readl(CM_ALWON_GPIO_1_CLKCTRL) != 0x2);

	/* SPI */
	__raw_writel(0x2, CM_ALWON_SPI_CLKCTRL);
	while(__raw_readl(CM_ALWON_SPI_CLKCTRL) != 0x2);

	/* I2C0 and I2C2 */
	__raw_writel(0x2, CM_ALWON_I2C_0_CLKCTRL);
	while(__raw_readl(CM_ALWON_I2C_0_CLKCTRL) != 0x2);

	/* I2C1 and I2C3 */
	__raw_writel(0x2, CM_ALWON_I2C_1_CLKCTRL);
	while(__raw_readl(CM_ALWON_I2C_1_CLKCTRL) != 0x2);

	/* Ethernet */
	__raw_writel(0x2, CM_ETHERNET_CLKSTCTRL);
	__raw_writel(0x2, CM_ALWON_ETHERNET_0_CLKCTRL);
	while((__raw_readl(CM_ALWON_ETHERNET_0_CLKCTRL) & 0x30000) != 0);
	__raw_writel(0x2, CM_ALWON_ETHERNET_1_CLKCTRL);
	/* HSMMC */
	__raw_writel(0x2, CM_ALWON_HSMMC_CLKCTRL);
	while(__raw_readl(CM_ALWON_HSMMC_CLKCTRL) != 0x2);

	/* MMC0(SDIO) */
	__raw_writel(0x2, CM_ALWON_HSMMC0_CLKCTRL);
	while(__raw_readl(CM_ALWON_HSMMC0_CLKCTRL) != 0x2);

	/*
	 * McASP2
	 * select mcasp2 clk from sys_clk_22 (OSC 0)
	 * so that audio clk (sys_clk_20) can be used for RMII
	 * ToDo :
	 * This can be removed once kernel exports set_parent()
	 */
	__raw_writel(0x2, CM_AUDIOCLK_MCASP2_CLKSEL);
	while (__raw_readl(CM_AUDIOCLK_MCASP2_CLKSEL) != 0x2);

	/* WDT */
	/* For WDT to be functional, it needs to be first stopped by writing
	 * the pattern 0xAAAA followed by 0x5555 in the WDT start/stop register.
	 * After that a write-once register in Control module needs to be
	 * configured to unfreeze the timer.
	 * Note: It is important to stop the watchdog before unfreezing it
	*/
	__raw_writel(0xAAAA, WDT_WSPR);
	while (__raw_readl(WDT_WWPS) != 0x0);
	__raw_writel(0x5555, WDT_WSPR);
	while (__raw_readl(WDT_WWPS) != 0x0);

	/* Unfreeze WDT */
	__raw_writel(0x13, WDT_UNFREEZE);
}

void DDR1_PHY_PWRDN(void)
{
__raw_writel(0x2,EMIF_CLK_GATE);
}

/*
 * inits clocks for PRCM as defined in clocks.h
 */

void PLL_Bypass (u32 base)
{
    __raw_writel(__raw_readl((base + CLKCTRL))|0x00800000,(base + CLKCTRL));
	while (( (__raw_readl(base + STATUS)) & 0x00000101) != 0x00000101);
    __raw_writel(__raw_readl(base + CLKCTRL)& 0xfffffffe,(base + CLKCTRL));
}

void PLL_CLKOUT_DISABLE (u32 base)
{
    __raw_writel(__raw_readl((base + CLKCTRL)) & 0xffefffff,(base + CLKCTRL));
	while (( (__raw_readl(base + STATUS)) & 0x00000020) != 0x00000000);
    __raw_writel(__raw_readl(base + CLKCTRL)& 0xfffffffe,(base + CLKCTRL));
}

/*
 * inits clocks for PRCM as defined in clocks.h
 */
void prcm_init(u32 in_ddr)
{
	/* Enable the control module */
	__raw_writel(0x2, CM_ALWON_CONTROL_CLKCTRL);

#ifdef CONFIG_SETUP_PLL
	/* Setup the various plls */
	audio_pll_config();
	sata_pll_config();
#if 0
	pcie_pll_config();
#endif


	modena_pll_config();
	l3_pll_config();
	ddr_pll_config();
	iva_pll_config();
	iss_pll_config();
	usb_pll_config();

	PLL_CLKOUT_DISABLE(SGX_PLL_BASE);
	__raw_writel(0x1,SECSS_CLK_SRC);

	/*  With clk freqs setup to desired values,
	 *  enable the required peripherals
	 */
	per_clocks_enable();
#endif
}


void ipnc_ff_pll_init(int option)
{
	unlock_pll_control_mmr();

	if(option == 1) {
		PLL_CLKOUT_ENABLE(DSS_PLL_BASE);
		PLL_CLKOUT_ENABLE(VIDEO_0_PLL_BASE);
		PLL_CLKOUT_ENABLE(VIDEO_1_PLL_BASE);
		PLL_CLKOUT_ENABLE(HDMI_PLL_BASE);

		dsp_pll_config();
	} else {
		PLL_Bypass(VIDEO_0_PLL_BASE);
		PLL_CLKOUT_DISABLE(DSS_PLL_BASE);
		PLL_CLKOUT_DISABLE(VIDEO_0_PLL_BASE);
		PLL_CLKOUT_DISABLE(VIDEO_1_PLL_BASE);
		PLL_CLKOUT_DISABLE(HDMI_PLL_BASE);

		PLL_CLKOUT_DISABLE(DSP_PLL_BASE);
	}
}

#define PADCTRL_BASE 0x48140000

#define PAD204_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0B2c))
#define PAD205_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0B30))
#define PAD206_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0B34))
#define PAD207_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0B38))
#define PAD208_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0B3c))
#define PAD209_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0B40))
#define PAD210_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0B44))
#define PAD211_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0B48))
#define PAD212_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0B4c))
#define PAD213_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0B50))
#define PAD214_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0B54))
#define PAD215_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0B58))
#define PAD216_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0B5c))
#define PAD217_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0B60))
#define PAD218_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0B64))
#define PAD219_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0B68))
#define PAD220_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0B6c))
#define PAD221_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0B70))
#define PAD222_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0B74))
#define PAD223_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0B78))
#define PAD224_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0B7c))
#define PAD225_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0B80))
#define PAD226_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0B84))
#define PAD227_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0B88))

#define PAD232_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0B9C))
#define PAD233_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0BA0))
#define PAD234_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0BA4))
#define PAD235_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0BA8))
#define PAD236_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0BAC))
#define PAD237_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0BB0))
#define PAD238_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0BB4))
#define PAD239_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0BB8))
#define PAD240_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0BBC))
#define PAD241_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0BC0))
#define PAD242_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0BC4))
#define PAD243_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0BC8))
#define PAD244_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0BCC))
#define PAD245_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0BD0))
#define PAD246_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0BD4))
#define PAD247_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0BD8))
#define PAD248_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0BDC))
#define PAD249_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0BE0))
#define PAD250_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0BE4))
#define PAD251_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0BE8))
#define PAD252_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0BEC))
#define PAD253_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0BF0))
#define PAD254_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0BF4))
#define PAD255_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0BF8))
#define PAD256_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0BFC))
#define PAD257_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0C00))
#define PAD258_CNTRL  (*(volatile unsigned int *)(PADCTRL_BASE + 0x0C04))

#ifdef CONFIG_DRIVER_TI_CPSW
static void cpsw_pad_config()
{
	volatile u32 val = 0;

#ifndef VPORT66
	/*configure pin mux for rmii_refclk,mdio_clk,mdio_d */
	val = PAD232_CNTRL;
	PAD232_CNTRL = (volatile unsigned int) (BIT(18) | BIT(0));
	val = PAD233_CNTRL; /*mdio_clk*/
	PAD233_CNTRL = (volatile unsigned int) (BIT(19) | BIT(18) | BIT(17) | BIT(0));
	val = PAD234_CNTRL; /*mdio_d*/
	PAD234_CNTRL = (volatile unsigned int) (BIT(19) | BIT(18) | BIT(17) | BIT(0));

	/*For VPort56 we only support MII Mode, setup mii0 pins here*/
		val = PAD235_CNTRL; /*mii0_tclk*/
		PAD235_CNTRL = (volatile unsigned int) (BIT(19) | BIT(18) | BIT(0));
		val = PAD236_CNTRL; /*mii0_col*/
		PAD236_CNTRL = (volatile unsigned int) (BIT(19) | BIT(18) | BIT(0));
		val = PAD237_CNTRL; /*mii0_crs*/
		PAD237_CNTRL = (volatile unsigned int) (BIT(19) | BIT(18) | BIT(0));
		val = PAD238_CNTRL; /*mii0_rxer*/
		PAD238_CNTRL = (volatile unsigned int) (BIT(19) | BIT(18) | BIT(0));
		val = PAD239_CNTRL; /*mii0_rclk*/
		PAD239_CNTRL = (volatile unsigned int) (BIT(19) | BIT(18) | BIT(0));
		val = PAD240_CNTRL; /*mii0_rxd[0]*/
		PAD240_CNTRL = (volatile unsigned int) (BIT(18) | BIT(0));
		val = PAD241_CNTRL; /*mii0_rxd[1]*/
		PAD241_CNTRL = (volatile unsigned int) (BIT(18) | BIT(0));
		val = PAD242_CNTRL; /*mii0_rxd[2]*/
		PAD242_CNTRL = (volatile unsigned int) (BIT(18) | BIT(0));
		val = PAD243_CNTRL; /*mii1_rxd[3]*/
		PAD243_CNTRL = (volatile unsigned int) (BIT(18) | BIT(0));
		val = PAD248_CNTRL; /*mii1_rxdv*/
		PAD248_CNTRL = (volatile unsigned int) (BIT(18) | BIT(0));
		val = PAD249_CNTRL; /*no use*/
		PAD249_CNTRL = (volatile unsigned int) (BIT(18) | BIT(0));
		val = PAD250_CNTRL; /*mii0_txd[0]*/
		PAD250_CNTRL = (volatile unsigned int) (BIT(18) | BIT(0));
		val = PAD251_CNTRL; /*mii0_txd[1]*/
		PAD251_CNTRL = (volatile unsigned int) (BIT(18) | BIT(0));
		val = PAD252_CNTRL; /*mii0_txd[2]*/
		PAD252_CNTRL = (volatile unsigned int) (BIT(18) | BIT(0));
		val = PAD253_CNTRL; /*mii0_txd[3]*/
		PAD253_CNTRL = (volatile unsigned int) (BIT(18) | BIT(0));
		val = PAD258_CNTRL; /*mii0_txen*/
		PAD258_CNTRL = (volatile unsigned int) (BIT(18) | BIT(0));
#else
	/*configure pin mux for rmii_refclk,mdio_clk,mdio_d */
	val = PAD232_CNTRL;
	PAD232_CNTRL = (volatile unsigned int) (BIT(18) | BIT(0));
	val = PAD233_CNTRL;
	PAD233_CNTRL = (volatile unsigned int) (BIT(19) | BIT(17) | BIT(0));
	val = PAD234_CNTRL;
	PAD234_CNTRL = (volatile unsigned int) (BIT(19) | BIT(18) | BIT(17) |
			BIT(0));

		/* In this case we enable rgmii_en bit in GMII_SEL register and
		 * still program the pins in gmii mode: gmii0 pins in mode 1*/
		val = PAD235_CNTRL; /*rgmii0_rxc*/
		PAD235_CNTRL = (volatile unsigned int) (BIT(18) | BIT(0));
		val = PAD236_CNTRL; /*rgmii0_rxctl*/
		PAD236_CNTRL = (volatile unsigned int) (BIT(18) | BIT(0));
		val = PAD237_CNTRL; /*rgmii0_rxd[2]*/
		PAD237_CNTRL = (volatile unsigned int) (BIT(18) | BIT(0));
		val = PAD238_CNTRL; /*rgmii0_txctl*/
		PAD238_CNTRL = (volatile unsigned int) BIT(0);
		val = PAD239_CNTRL; /*rgmii0_txc*/
		PAD239_CNTRL = (volatile unsigned int) BIT(0);
		val = PAD240_CNTRL; /*rgmii0_txd[0]*/
		PAD240_CNTRL = (volatile unsigned int) BIT(0);
		val = PAD241_CNTRL; /*rgmii0_rxd[0]*/
		PAD241_CNTRL = (volatile unsigned int) (BIT(18) | BIT(0));
		val = PAD242_CNTRL; /*rgmii0_rxd[1]*/
		PAD242_CNTRL = (volatile unsigned int) (BIT(18) | BIT(0));
		val = PAD243_CNTRL; /*rgmii1_rxctl*/
		PAD243_CNTRL = (volatile unsigned int) (BIT(18) | BIT(0));
		val = PAD244_CNTRL; /*rgmii0_rxd[3]*/
		PAD244_CNTRL = (volatile unsigned int) (BIT(18) | BIT(0));
		val = PAD245_CNTRL; /*rgmii0_txd[3]*/
		PAD245_CNTRL = (volatile unsigned int) BIT(0);
		val = PAD246_CNTRL; /*rgmii0_txd[2]*/
		PAD246_CNTRL = (volatile unsigned int) BIT(0);
		val = PAD247_CNTRL; /*rgmii0_txd[1]*/
		PAD247_CNTRL = (volatile unsigned int) BIT(0);
#endif
#if 0 /* switch off second RGMII to save power */
		val = PAD248_CNTRL; /*rgmii1_rxd[1]*/
		PAD248_CNTRL = (volatile unsigned int) (BIT(18) | BIT(0));
		val = PAD249_CNTRL; /*rgmii1_rxc*/
		PAD249_CNTRL = (volatile unsigned int) (BIT(18) | BIT(0));
		val = PAD250_CNTRL; /*rgmii1_rxd[3]*/
		PAD250_CNTRL = (volatile unsigned int) (BIT(18) | BIT(0));
		val = PAD251_CNTRL; /*rgmii1_txd[1]*/
		PAD251_CNTRL = (volatile unsigned int) (BIT(0));
		val = PAD252_CNTRL; /*rgmii1_txctl*/
		PAD252_CNTRL = (volatile unsigned int) (BIT(0));
		val = PAD253_CNTRL; /*rgmii1_txd[0]*/
		PAD253_CNTRL = (volatile unsigned int) (BIT(0));
		val = PAD254_CNTRL; /*rgmii1_txd[2]*/
		PAD254_CNTRL = (volatile unsigned int) (BIT(0));
		val = PAD255_CNTRL; /*rgmii1_txc*/
		PAD255_CNTRL = (volatile unsigned int) (BIT(0));
		val = PAD256_CNTRL; /*rgmii1_rxd[0]*/
		PAD256_CNTRL = (volatile unsigned int) (BIT(18) | BIT(0));
		val = PAD257_CNTRL; /*rgmii1_txd[3]*/
		PAD257_CNTRL = (volatile unsigned int) (BIT(0));
		val = PAD258_CNTRL; /*rgmii1_rxd[2]*/
		PAD258_CNTRL = (volatile unsigned int) (BIT(18) | BIT(0));
#endif
}
#endif /* CONFIG_DRIVER_TI_CPSW */

#if 0 /* We don't use this on our IPNC */
struct nor_pad_config {
	unsigned int offset;
	unsigned int value;
};

static struct nor_pad_config nor_pad_cfg[] = {
		{GPMC_D0, MODE(1) | INPUT_EN | PULL_DIS},
		{GPMC_D1, MODE(1) | INPUT_EN | PULL_DIS},
		{GPMC_D2, MODE(1) | INPUT_EN | PULL_DIS},
		{GPMC_D3, MODE(1) | INPUT_EN | PULL_DIS},
		{GPMC_D4, MODE(1) | INPUT_EN | PULL_DIS},
		{GPMC_D5, MODE(1) | INPUT_EN | PULL_DIS},
		{GPMC_D6, MODE(1) | INPUT_EN | PULL_DIS},
		{GPMC_D7, MODE(1) | INPUT_EN | PULL_DIS},
		{GPMC_D8, MODE(1) | INPUT_EN | PULL_DIS},
		{GPMC_D9, MODE(1) | INPUT_EN | PULL_DIS},
		{GPMC_D10, MODE(1) | INPUT_EN | PULL_DIS},
		{GPMC_D11, MODE(1) | INPUT_EN | PULL_DIS},
		{GPMC_D12, MODE(1) | INPUT_EN | PULL_DIS},
		{GPMC_D13, MODE(1) | INPUT_EN | PULL_DIS},
		{GPMC_D14, MODE(1) | INPUT_EN | PULL_DIS},
		{GPMC_D15, MODE(1) | INPUT_EN | PULL_DIS},
		{GPMC_A1, MODE(2) | PULL_UP_EN},
		{GPMC_A2, MODE(2) | PULL_UP_EN},
		{GPMC_A3, MODE(2) | PULL_UP_EN},
		{GPMC_A4, MODE(2) | PULL_UP_EN},
		{GPMC_A5, MODE(5) | PULL_UP_EN},
		{GPMC_A6, MODE(5)},
		{GPMC_A7, MODE(5)},
		{GPMC_A8, MODE(5)},
		{GPMC_A9, MODE(5)},
		{GPMC_A10, MODE(5) | PULL_UP_EN},
		{GPMC_A11, MODE(5)},
		{GPMC_A12, MODE(5)},
		{GPMC_A13, MODE(2) | PULL_UP_EN},
		{GPMC_A14, MODE(2) | PULL_UP_EN},
		{GPMC_A15, MODE(2)},
		{GPMC_A16, MODE(1)},
		{GPMC_A17, MODE(1)},
		{GPMC_A18, MODE(1)},
		{GPMC_A19, MODE(1)},
		{GPMC_A20, MODE(1) | PULL_UP_EN},
		{GPMC_A21, MODE(1)},
		{GPMC_A22, MODE(1) | PULL_UP_EN},
		{GPMC_A23, MODE(1)},
		{GPMC_A24, MODE(2) | PULL_UP_EN},
		{GPMC_A25, MODE(2)},
		{GPMC_A27, MODE(8) | PULL_UP_EN},
		{GPMC_CS0_REG, MODE(1) | PULL_UP_EN},
		{GPMC_OEN, MODE(1) | PULL_UP_EN},
		{GPMC_WEN, MODE(1) | PULL_UP_EN},
		{0},
};
/*********************************************************************
 *
 * nor_pad_config_mux - configure the pin mux for NOR
 *
 *********************************************************************/
static void nor_pad_config_mux(void)
{
	u8 i = 0;

	while (nor_pad_cfg[i].offset != 0x0) {
		*(volatile u32 *)(nor_pad_cfg[i].offset) =
			nor_pad_cfg[i].value;
		i++;
	}
}
#endif
/*
 * board specific muxing of pins
 */
void set_muxconf_regs(void)
{
	u32 i, add, val;
	u32 pad_conf[] = {
#include "mux.h"
	};

	for (i = 0; i<N_PINS; i++)
	{
		add = PIN_CTRL_BASE + (i*4);
		/* 0 skips reserved regs */
		if (pad_conf[i] != 0) {
		val = __raw_readl(add);
		val &= 0xFFFFFF00;
		val |= pad_conf[i];
		__raw_writel(val, add);
		}
	}
	/* MMC/SD pull-down enable */
//	__raw_writel(0x000C0040, 0x48140928);
}

/*
 * gpio init
 */
void gpio_init(void)
{
	u32  add, val;
	/*
	   GPIO0 base 0x48032000
	   GPIO1 base 0x4804C000
	   GPIO2 base 0x481AC000
	   GPIO3 base 0x481AE000
	*/

	//GPIO0[] group
	add=(GPIO0_BASE + GPIO_OE);			//GPIO_OE Output Enable Register
	val = __raw_readl(add);
	val &=~(1<<14); 					//GP0_14-Fan_con output
	val |= (1<<17); 					//GP0_17-Heatersys_int input
	val &=~(1<<22); 					//GP0_22-Heater_sys output
	val &=~(1<<26); 					//GP0_26-Heater_cam output
	val |= (1<<29); 					//GP0_29-SD_WPn input
	val |= (1<<30); 					//GP0_30-SD_CDn input
	val &=~(1<<31); 					//GP0_31-SD_EN output
	__raw_writel(val, add);
	sdcard_enable();

	//GPIO1[] group
	add=(GPIO1_BASE + GPIO_OE);			//GPIO_OE Output Enable Register
	val = __raw_readl(add);
	val &=~(1<<0); 						//GP1_0-FLASH_WP output
#ifdef VPORT66
	val |= (1<<4);						//GP1_4-Heatercam_int input
#endif
	val &=~(1<<7); 						//GP1_7-Fan_int output
//	val |= (1<<9);						//GP1_9-Fan_int input
//	val &=~(1<<10);						//GP1_10-Fan_con output
	val &=~(1<<17); 					//GP1_17-RS485_4W output
	val &=~(1<<18); 					//GP1_18-RS485_ENT output
	val &=~(1<<26); 					//GP1_26-RS485_ENRn output
	__raw_writel(val, add);
	__raw_writel((1<<17), 0x4804c190);  //GP1_17-RS485_4W output low
	RS485_SetTx();

	//GPIO2[] group
	add=(GPIO2_BASE + GPIO_OE);			//GPIO_OE Output Enable Register
	val = __raw_readl(add);
	val &=~(1<<18);   					//GP2_18-CAM_RST output
	val &=~(1<<20);   					//GP2_20-ENET_RSTn output
	val |= (1<<21);   					//GP2_21-RE_SETING input
	val &=~(1<<22);						//GP2_22-ENET_RSTn output
	val |= (1<<23);						//GP2_23-E_LINKSTS input
	val &=~(1<<24);						//GP2_24-CMOS_OE output
	val &=~(1<<25);						//GP2_25-CMOS_RST output
	val |= (1<<26);						//GP2_26-RTC_INTn input
	val &=~(1<<27);						//GP2_27-HD_CLK output
	val &=~(1<<29);						//GP2_29-PI_POWER output
	val &=~(1<<30);						//GP2_30-CAMERA_POWER output
	__raw_writel(val, add);
	__raw_writel((1<<20), 0x481ac194);
	__raw_writel((1<<24), 0x481ac190);  //GP2_24-CMOS_OE (PWDN) output low
	/* reset sensor */
	__raw_writel((1<<18), 0x481ac190);  //output low
	__raw_writel((1<<25), 0x481ac190);  //GP2_25-CMOS_RST (RESETB) output low
	delay(1000);
	__raw_writel((1<<18), 0x481ac194);  //output high
	__raw_writel((1<<25), 0x481ac194);  //GP2_25-CMOS_RST (RESETB) output high
	/* WLAN power-on sequence */
	__raw_writel((1<<22), 0x481ac194);
	delay(30000);
	__raw_writel((1<<22), 0x481ac190);
	delay(5*30000);
	__raw_writel((1<<22), 0x481ac194);
	delay(30000);
	__raw_writel((1<<29), 0x481ac190);  //GP2_29-PI_POWER output low
	__raw_writel((1<<30), 0x481ac194);  //GP2_30-CAMERA_POWER output high

	// GPIO3[] group
	add=(GPIO3_BASE + GPIO_OE);	  		//GPIO_OE Output Enable Register
	val = __raw_readl(add);
    val |= (1<<7);						//GP3_7-ARM_IN input mode
    val &=~(1<<8);						//GP3_8-ARM_OUT output mode
    val |= (1<<9);						//GP3_9-ARM_RST input mode
    val &=~(1<<12);						//GP3_12-LED_G output mode
    val &=~(1<<13);						//GP3_13-LED_R output mode
	__raw_writel(val, add);
	while(__raw_readl(add) != val);
	sys_led_R_ON();
	sys_led_G_OFF();

	/* reset FPGA */
//	add=0x4804c134;						//GPIO_OE Output Enable Register
//	val = __raw_readl(add);
//	val &=~(1<<16);   					//GP1_16-output
//	val &=~(1<<1);						//RS485_RDE GP1_1-output
//	__raw_writel(val, add);
//	__raw_writel((1<<16), 0x4804c190);  //output low
//	delay(1000);
//	__raw_writel((1<<16), 0x4804c194);  //output high

#if defined(CONFIG_CODEC_AIC26) || defined(CONFIG_CODEC_AIC3104)
	Audio_HW_Reset();
#endif

//	#ifdef CONFIG_TI814X_MIN_CONFIG
//	__raw_writel(1<<12, 0x481ae13c);  	//output mode GP3_12-LED1_ON --Hi
//	#else
//	__raw_writel(1<<12, 0x481ae190);  	//output mode GP3_12-LED1_ON --low
//	#endif
}

#if defined(CONFIG_CODEC_AIC26) || defined(CONFIG_CODEC_AIC3104)
int Audio_HW_Reset(void)
{
	u32  add, val;
	/* Hardware reset */
	/* GP0[8] */
	add=0x48032134;						//GPIO_OE Output Enable Register
	val = __raw_readl(add);
	val &=~(1<<8);   					//GP0_8-AIC_RSTn output
	__raw_writel(val, add);
	__raw_writel((1<<8), 0x48032190);  //output low
	delay(1000);
	__raw_writel((1<<8), 0x48032194);  //output high
	return 0;
}
#endif

#ifdef CONFIG_TPS65911_I2C
static void power_control(void)
{
	int arm_freq, ddr_freq, dsp_freq, iva_freq, iss_freq, dss_freq;
	unsigned char vdd1_val, vdd2_val, vddctrl_val;

	/* clk_out  = ((OSC_0/ ( N+1 )) * M) / M2   */
	arm_freq = ((OSC_0_FREQ / (MODENA_N + 1) * MODENA_M) / MODENA_M2);
	ddr_freq = ((OSC_0_FREQ / (DDR_N + 1) * DDR_M) / DDR_M2);
	dsp_freq = ((DSP_M * OSC_0_FREQ)/(DSP_N+1)/DSP_M2);
	iva_freq = ((IVA_M * OSC_0_FREQ)/(IVA_N+1)/IVA_M2);
	iss_freq = ((ISS_M * OSC_0_FREQ)/(ISS_N+1)/ISS_M2);
	dss_freq = ((DSS_M * OSC_0_FREQ)/(DSS_N+1)/DSS_M2);

	tps65911_init();

	vdd1_val 	= (arm_freq>720)? VDD_1D35:((arm_freq>600)? VDD_1D2:VDD_1D1);
	vdd2_val 	= (iva_freq>306)? VDD_1D35:((iva_freq>266)?VDD_1D2:VDD_1D1);
	vddctrl_val = ((iss_freq>400)||(ddr_freq>400))?VDD_1D35:VDD_1D2;

	tps65911_config(VDD1_OP_REG   	, vdd1_val);
	tps65911_config(VDD2_OP_REG   	, vdd2_val);/*VDD_1D35*/
	tps65911_config(VDDCRTL_OP_REG	, vddctrl_val);
	tps65911_config(BBCH_REG      	, BBCHEN | BBSEL_3D15V);
}

static void show_time(void)
{
	do_date(NULL, 0, 1, NULL);
}
#endif

void unlock_pll_control_mmr()
{
	/* ??? */
	__raw_writel(0x1EDA4C3D, 0x481C5040);
	__raw_writel(0x2FF1AC2B, 0x48140060);
	__raw_writel(0xF757FDC0, 0x48140064);
	__raw_writel(0xE2BC3A6D, 0x48140068);
	__raw_writel(0x1EBF131D, 0x4814006c);
	__raw_writel(0x6F361E05, 0x48140070);

}
extern void l2_disable_wa(void);
/*
 * early system init of muxing and clocks.
 */
void s_init(u32 in_ddr)
{
	/* TODO: Revisit enabling of I/D-cache in 1st stage */
#if 0
	icache_enable();
	dcache_enable();
#endif

	/*
	 * Disable Write Allocate on miss to avoid starvation of other masters
	 * (than A8).
	 *
	 * Ref DM814x Erratum: TODO
	 */
	l2_disable_wa();

	/* Can be removed as A8 comes up with L2 enabled */
	l2_cache_enable();
	unlock_pll_control_mmr();
	/* Setup the PLLs and the clocks for the peripherals */
	prcm_init(in_ddr);
#if defined(CONFIG_TI814X_CONFIG_DDR)
	if (!in_ddr)
		config_ti814x_ddr();	/* Do DDR settings */
#endif
}

/*
 * Reset the board
 */
void reset_cpu (ulong addr)
{
	addr = __raw_readl(PRM_DEVICE_RSTCTRL);
	addr &= ~BIT(1);
	addr |= BIT(1);
	__raw_writel(addr, PRM_DEVICE_RSTCTRL);
}

#ifdef CONFIG_DRIVER_TI_CPSW

#define MARVELL_PHY_ID		0x01410e11	/* Marvell Giga-PHY 88E1118 */
#define LSI_PHY_ID			0x0282F014
#define PHY_CONF_REG		22
#define PHY_CONF_TXCLKEN	(1 << 5)

/* TODO : Check for the board specific PHY */
static void phy_init(char *name, int addr)
{
	unsigned short val;
	unsigned int   cntr = 0;
	unsigned int   phy_id = 0;

	miiphy_reset(name, addr);

	udelay(100000);

	miiphy_read(name, addr, PHY_PHYIDR1, &val);
	phy_id = (val << 16)  & 0xffff0000;
	miiphy_read(name, addr, PHY_PHYIDR2, &val);
	phy_id |= val & 0x0000ffff;

	if (phy_id == LSI_PHY_ID) {
		printf("Configuring LSI Phy\n\n");
		/* Enable PHY to clock out TX_CLK */
		miiphy_read(name, addr, PHY_CONF_REG, &val);
		val |= PHY_CONF_TXCLKEN;
		miiphy_write(name, addr, PHY_CONF_REG, val);
		miiphy_read(name, addr, PHY_CONF_REG, &val);
	}
	else if (phy_id == MARVELL_PHY_ID) {
		printf("Configuring MARVELL Phy\n\n");
	}
	/* Enable Autonegotiation */
	if (miiphy_read(name, addr, PHY_BMCR, &val) != 0) {
		printf("failed to read bmcr\n");
		return;
	}
	val |= PHY_BMCR_DPLX | PHY_BMCR_AUTON | PHY_BMCR_100_MBPS;
	if (miiphy_write(name, addr, PHY_BMCR, val) != 0) {
		printf("failed to write bmcr\n");
		return;
	}
	miiphy_read(name, addr, PHY_BMCR, &val);

	/* Setup GIG advertisement */
	miiphy_read(name, addr, PHY_1000BTCR, &val);
	val |= PHY_1000BTCR_1000FD;
	val &= ~PHY_1000BTCR_1000HD;
	miiphy_write(name, addr, PHY_1000BTCR, val);
	miiphy_read(name, addr, PHY_1000BTCR, &val);

	/* Setup general advertisement */
	if (miiphy_read(name, addr, PHY_ANAR, &val) != 0) {
		printf("failed to read anar\n");
		return;
	}
	val |= (PHY_ANLPAR_10 | PHY_ANLPAR_10FD | PHY_ANLPAR_TX |
		PHY_ANLPAR_TXFD);
	if (miiphy_write(name, addr, PHY_ANAR, val) != 0) {
		printf("failed to write anar\n");
		return;
	}
	miiphy_read(name, addr, PHY_ANAR, &val);

	/* Restart auto negotiation*/
	miiphy_read(name, addr, PHY_BMCR, &val);
	val |= PHY_BMCR_RST_NEG;
	miiphy_write(name, addr, PHY_BMCR, val);

	/*check AutoNegotiate complete - it can take upto 3 secs*/
	do {
		udelay(40000);

		cntr++;

		if (!miiphy_read(name, addr, PHY_BMSR, &val)) {
			if (val & PHY_BMSR_AUTN_COMP)
				break;
		}
	} while (cntr < 250);

	if (!miiphy_read(name, addr, PHY_BMSR, &val)) {
		if (!(val & PHY_BMSR_AUTN_COMP))
			printf("Auto negotitation failed\n");
	}
}

static void cpsw_control(int enabled)
{
	/* nothing for now */
	/* TODO : VTP was here before */
}

static struct cpsw_slave_data cpsw_slaves[] = {
	{
		.slave_reg_ofs  = 0x50,
		.sliver_reg_ofs = 0x700,
		.phy_id         = 1,
	},
	{
		.slave_reg_ofs	= 0x90,
		.sliver_reg_ofs = 0x740,
		.phy_id         = 0,
	},
};

static struct cpsw_platform_data cpsw_data = {
	.mdio_base              = TI814X_CPSW_MDIO_BASE,
	.cpsw_base              = TI814X_CPSW_BASE,
	.mdio_div               = 0xff,
	.channels               = 8,
	.cpdma_reg_ofs          = 0x100,
	.cpdma_sram_ofs         = 0x200,
	.slaves                 = 1,
	.slave_data             = cpsw_slaves,
	.ale_reg_ofs            = 0x600,
	.ale_entries            = 1024,
	.host_port_reg_ofs      = 0x28,
	.hw_stats_reg_ofs       = 0x400,
	.mac_control            = (1 << 5) /* MIIEN      */,
	.control                = cpsw_control,
	.phy_init               = phy_init,
	.host_port_num          = 0,
	.bd_ram_ofs             = 0x2000,
};

extern void cpsw_eth_set_mac_addr (const u_int8_t *addr);

int board_eth_init(bd_t *bis)
{
	u_int8_t mac_addr[6];
	u_int32_t mac_hi,mac_lo;

	cpsw_pad_config();

	if (!eth_getenv_enetaddr("ethaddr", mac_addr)) {
		char mac_addr_env[20];

		printf("<ethaddr> not set. Reading from E-fuse\n");
		/* try reading mac address from efuse */
		mac_lo = __raw_readl(MAC_ID0_LO);
		mac_hi = __raw_readl(MAC_ID0_HI);
		mac_addr[0] = mac_hi & 0xFF;
		mac_addr[1] = (mac_hi & 0xFF00) >> 8;
		mac_addr[2] = (mac_hi & 0xFF0000) >> 16;
		mac_addr[3] = (mac_hi & 0xFF000000) >> 24;
		mac_addr[4] = mac_lo & 0xFF;
		mac_addr[5] = (mac_lo & 0xFF00) >> 8;
		/* set the ethaddr variable with MACID detected */
		sprintf(mac_addr_env, "%02x:%02x:%02x:%02x:%02x:%02x",
			mac_addr[0], mac_addr[1], mac_addr[2],
			mac_addr[3], mac_addr[4], mac_addr[5]);
		setenv("ethaddr", mac_addr_env);
	}

	if(is_valid_ether_addr(mac_addr)) {
		printf("Detected MACID:%02x:%02x:%02x:%02x:%02x:%02x\n", 
			mac_addr[0], mac_addr[1], mac_addr[2], 
			mac_addr[3], mac_addr[4], mac_addr[5]);
		cpsw_eth_set_mac_addr(mac_addr);
	} else {
		printf("Caution:using static MACID!! Set <ethaddr> variable\n");
	}

#ifdef VPORT66
	if (PG1_0 != get_cpu_rev()) {
		cpsw_slaves[0].phy_id = 0;
		cpsw_slaves[1].phy_id = 1;
	}
#endif

	return cpsw_register(&cpsw_data);
}
#endif

#ifdef CONFIG_NAND_TI81XX
/******************************************************************************
 * Command to switch between NAND HW and SW ecc
 *****************************************************************************/
extern void ti81xx_nand_switch_ecc(nand_ecc_modes_t hardware, int32_t mode);
static int do_switch_ecc(cmd_tbl_t * cmdtp, int flag, int argc, char *argv[])
{
	int type = 0;
	if (argc < 2)
		goto usage;

	if (strncmp(argv[1], "hw", 2) == 0) {
		if (argc == 3)
			type = simple_strtoul(argv[2], NULL, 10);
		ti81xx_nand_switch_ecc(NAND_ECC_HW, type);
	}
	else if (strncmp(argv[1], "sw", 2) == 0)
		ti81xx_nand_switch_ecc(NAND_ECC_SOFT, 0);
	else
		goto usage;

	return 0;

usage:
	printf ("Usage: nandecc %s\n", cmdtp->usage);
	return 1;
}

U_BOOT_CMD(
	nandecc, 3, 1,	do_switch_ecc,
	"Switch NAND ECC calculation algorithm b/w hardware and software",
	"[sw|hw <hw_type>] \n"
	"   [sw|hw]- Switch b/w hardware(hw) & software(sw) ecc algorithm\n"
	"   hw_type- 0 for Hamming code\n"
	"            1 for bch4\n"
	"            2 for bch8\n"
	"            3 for bch16\n"
);

#endif /* CONFIG_NAND_TI81XX */

#ifdef CONFIG_GENERIC_MMC
int board_mmc_init(bd_t *bis)
{
	sdcard_enable();
	omap_mmc_init(0);
	omap_mmc_init(1);
	return 0;
}
#endif

#ifdef CONFIG_HW_WATCHDOG
static int trigger_skip = 0;
void hw_watchdog_reset(void)
{
	if(trigger_skip) return;
	__raw_writel(__raw_readl(WDT_WTGR) + 1, WDT_WTGR);
}

int hw_watchdog_op(hwwd_op_t op)
{
	switch(op){
		case HWWD_INIT:
			hw_watchdog_op(HWWD_OFF);
			delay(1000);
			__raw_writel(0x00000003, WDT_WIRQENSET); //Enable delay/overflow interrupt
			delay(10000);
			__raw_writel(0x00000010, WDT_WCLR); //Clock Divider = 1
			delay(10000);
			__raw_writel((0xFFFFFFFF - (WDT_TIMEOUT_BASE * WDT_TIMEOUT_SEC)), WDT_WLDR);
			delay(10000);
			__raw_writel(0xFFFFFFFF, WDT_WDLY);
			delay(10000);
			__raw_writel((0xFFFFFFFF - (WDT_TIMEOUT_BASE * WDT_TIMEOUT_SEC)), WDT_WCRR);
			break;
		case HWWD_OFF:
			__raw_writel(0xAAAA, WDT_WSPR);
			while (__raw_readl(WDT_WWPS) != 0x0);
			__raw_writel(0x5555, WDT_WSPR);
			while (__raw_readl(WDT_WWPS) != 0x0);
			break;
		case HWWD_ON:
			__raw_writel(0xBBBB, WDT_WSPR);
			while (__raw_readl(WDT_WWPS) != 0x0);
			__raw_writel(0x4444, WDT_WSPR);
			while (__raw_readl(WDT_WWPS) != 0x0);
			break;
		case HWWD_RST:
			__raw_writel(__raw_readl(WDT_WTGR) + 1, WDT_WTGR);
			break;
		case HWWD_SOFT_RST:
			__raw_writel(0x00000002, WDT_WDSC); //Execute software reset.
			while (( (__raw_readl(WDT_WDSC)) & 0x00000002) != 0x00000000); //Wait until reset release?
			break;
		case HWWD_FORCE_RST:
			hw_watchdog_op(HWWD_OFF);
			delay(1000);
			__raw_writel(0xFFFFFFFF, WDT_WLDR);
			delay(1000);
			__raw_writel(__raw_readl(WDT_WTGR) + 1, WDT_WTGR);
			delay(1000);
			__raw_writel(__raw_readl(WDT_WTGR) + 1, WDT_WTGR);
			delay(1000);
			__raw_writel(__raw_readl(WDT_WTGR) + 1, WDT_WTGR);
			delay(1000);
			__raw_writel(__raw_readl(WDT_WTGR) + 1, WDT_WTGR);
			break;
		case HWWD_TRIGGER_SKIP:
			trigger_skip = 1;
			break;
	}
	return 0;
}
#endif

#ifndef CONFIG_SYS_DCACHE_OFF
void enable_caches(void)
{
	/* Enable D-cache. I-cache is enabled in start.S */
	dcache_enable();
	puts("On\n");
}
#endif
