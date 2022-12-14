/*
 * Copyright (C) 2010 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ----------------------------------------------------------------------------
 *
 */

#ifndef _DDR_DEFS_TI814X_H
#define _DDR_DEFS_TI814X_H

#include <asm/arch/hardware.h>

/* DDR Phy MMRs OFFSETs */
#define CMD0_REG_PHY_CTRL_SLAVE_RATIO_0		0x01C
#define CMD0_REG_PHY_DLL_LOCK_DIFF_0		0x028
#define CMD0_REG_PHY_INVERT_CLKOUT_0		0x02C
#define CMD1_REG_PHY_CTRL_SLAVE_RATIO_0		0x050
#define CMD1_REG_PHY_DLL_LOCK_DIFF_0		0x05C
#define CMD1_REG_PHY_INVERT_CLKOUT_0		0x060
#define CMD2_REG_PHY_CTRL_SLAVE_RATIO_0		0x084
#define CMD2_REG_PHY_DLL_LOCK_DIFF_0		0x090
#define CMD2_REG_PHY_INVERT_CLKOUT_0		0x094

/* DDR0 Phy MMRs */
#define CMD0_REG_PHY0_CTRL_SLAVE_RATIO_0	(0x01C + DDR0_PHY_BASE_ADDR)
#define CMD0_REG_PHY0_DLL_LOCK_DIFF_0		(0x028 + DDR0_PHY_BASE_ADDR)
#define CMD0_REG_PHY0_INVERT_CLKOUT_0		(0x02C + DDR0_PHY_BASE_ADDR)
#define CMD1_REG_PHY0_CTRL_SLAVE_RATIO_0	(0x050 + DDR0_PHY_BASE_ADDR)
#define CMD1_REG_PHY0_DLL_LOCK_DIFF_0		(0x05C + DDR0_PHY_BASE_ADDR)
#define CMD1_REG_PHY0_INVERT_CLKOUT_0		(0x060 + DDR0_PHY_BASE_ADDR)
#define CMD2_REG_PHY0_CTRL_SLAVE_RATIO_0	(0x084 + DDR0_PHY_BASE_ADDR)
#define CMD2_REG_PHY0_DLL_LOCK_DIFF_0		(0x090 + DDR0_PHY_BASE_ADDR)
#define CMD2_REG_PHY0_INVERT_CLKOUT_0		(0x094 + DDR0_PHY_BASE_ADDR)

#define DATA0_REG_PHY0_RD_DQS_SLAVE_RATIO_0	(0x0C8 + DDR0_PHY_BASE_ADDR)
#define DATA0_REG_PHY0_WR_DQS_SLAVE_RATIO_0	(0x0DC + DDR0_PHY_BASE_ADDR)
#define DATA0_REG_PHY0_WRLVL_INIT_RATIO_0	(0x0F0 + DDR0_PHY_BASE_ADDR)
#define DATA0_REG_PHY0_WRLVL_INIT_MODE_0	(0x0F8 + DDR0_PHY_BASE_ADDR)
#define DATA0_REG_PHY0_GATELVL_INIT_RATIO_0	(0x0FC + DDR0_PHY_BASE_ADDR)
#define DATA0_REG_PHY0_GATELVL_INIT_MODE_0	(0x104 + DDR0_PHY_BASE_ADDR)
#define DATA0_REG_PHY0_FIFO_WE_SLAVE_RATIO_0	(0x108 + DDR0_PHY_BASE_ADDR)
#define DATA0_REG_PHY0_WR_DATA_SLAVE_RATIO_0	(0x120 + DDR0_PHY_BASE_ADDR)
#define DATA0_REG_PHY0_USE_RANK0_DELAYS		(0x134 + DDR0_PHY_BASE_ADDR)
#define DATA0_REG_PHY0_DLL_LOCK_DIFF_0		(0x138 + DDR0_PHY_BASE_ADDR)

#define DATA1_REG_PHY0_RD_DQS_SLAVE_RATIO_0	(0x16C + DDR0_PHY_BASE_ADDR)
#define DATA1_REG_PHY0_WR_DQS_SLAVE_RATIO_0	(0x180 + DDR0_PHY_BASE_ADDR)
#define DATA1_REG_PHY0_WRLVL_INIT_RATIO_0	(0x194 + DDR0_PHY_BASE_ADDR)
#define DATA1_REG_PHY0_WRLVL_INIT_MODE_0	(0x19C + DDR0_PHY_BASE_ADDR)
#define DATA1_REG_PHY0_GATELVL_INIT_RATIO_0	(0x1A0 + DDR0_PHY_BASE_ADDR)
#define DATA1_REG_PHY0_GATELVL_INIT_MODE_0	(0x1A8 + DDR0_PHY_BASE_ADDR)
#define DATA1_REG_PHY0_FIFO_WE_SLAVE_RATIO_0	(0x1AC + DDR0_PHY_BASE_ADDR)
#define DATA1_REG_PHY0_WR_DATA_SLAVE_RATIO_0	(0x1C4 + DDR0_PHY_BASE_ADDR)
#define DATA1_REG_PHY0_USE_RANK0_DELAYS		(0x1D8 + DDR0_PHY_BASE_ADDR)
#define DATA1_REG_PHY0_DLL_LOCK_DIFF_0		(0x1DC + DDR0_PHY_BASE_ADDR)

#define DATA2_REG_PHY0_RD_DQS_SLAVE_RATIO_0	(0x210 + DDR0_PHY_BASE_ADDR)
#define DATA2_REG_PHY0_WR_DQS_SLAVE_RATIO_0	(0x224 + DDR0_PHY_BASE_ADDR)
#define DATA2_REG_PHY0_WRLVL_INIT_RATIO_0	(0x238 + DDR0_PHY_BASE_ADDR)
#define DATA2_REG_PHY0_WRLVL_INIT_MODE_0	(0x240 + DDR0_PHY_BASE_ADDR)
#define DATA2_REG_PHY0_GATELVL_INIT_RATIO_0	(0x244 + DDR0_PHY_BASE_ADDR)
#define DATA2_REG_PHY0_GATELVL_INIT_MODE_0	(0x24C + DDR0_PHY_BASE_ADDR)
#define DATA2_REG_PHY0_FIFO_WE_SLAVE_RATIO_0	(0x250 + DDR0_PHY_BASE_ADDR)
#define DATA2_REG_PHY0_WR_DATA_SLAVE_RATIO_0	(0x268 + DDR0_PHY_BASE_ADDR)
#define DATA2_REG_PHY0_USE_RANK0_DELAYS		(0x27C + DDR0_PHY_BASE_ADDR)
#define DATA2_REG_PHY0_DLL_LOCK_DIFF_0		(0x280 + DDR0_PHY_BASE_ADDR)

#define DATA3_REG_PHY0_RD_DQS_SLAVE_RATIO_0	(0x2B4 + DDR0_PHY_BASE_ADDR)
#define DATA3_REG_PHY0_WR_DQS_SLAVE_RATIO_0	(0x2C8 + DDR0_PHY_BASE_ADDR)
#define DATA3_REG_PHY0_WRLVL_INIT_RATIO_0	(0x2DC + DDR0_PHY_BASE_ADDR)
#define DATA3_REG_PHY0_WRLVL_INIT_MODE_0	(0x2E4 + DDR0_PHY_BASE_ADDR)
#define DATA3_REG_PHY0_GATELVL_INIT_RATIO_0	(0x2E8 + DDR0_PHY_BASE_ADDR)
#define DATA3_REG_PHY0_GATELVL_INIT_MODE_0	(0x2F0 + DDR0_PHY_BASE_ADDR)
#define DATA3_REG_PHY0_FIFO_WE_SLAVE_RATIO_0	(0x2F4 + DDR0_PHY_BASE_ADDR)
#define DATA3_REG_PHY0_WR_DATA_SLAVE_RATIO_0	(0x30C + DDR0_PHY_BASE_ADDR)
#define DATA3_REG_PHY0_USE_RANK0_DELAYS		(0x320 + DDR0_PHY_BASE_ADDR)
#define DATA3_REG_PHY0_DLL_LOCK_DIFF_0		(0x324 + DDR0_PHY_BASE_ADDR)

/* DDR1 Phy MMRs */
#if !defined(CONFIG_TI813X) && !defined(CONFIG_TI811X) && !defined(CONFIG_DM385)
#define	CMD0_REG_PHY1_CTRL_SLAVE_RATIO_0	(0x01C + DDR1_PHY_BASE_ADDR)
#define	CMD0_REG_PHY1_DLL_LOCK_DIFF_0		(0x028 + DDR1_PHY_BASE_ADDR)
#define	CMD0_REG_PHY1_INVERT_CLKOUT_0		(0x02C + DDR1_PHY_BASE_ADDR)
#define	CMD1_REG_PHY1_CTRL_SLAVE_RATIO_0	(0x050 + DDR1_PHY_BASE_ADDR)
#define	CMD1_REG_PHY1_DLL_LOCK_DIFF_0		(0x05C + DDR1_PHY_BASE_ADDR)
#define	CMD1_REG_PHY1_INVERT_CLKOUT_0		(0x060 + DDR1_PHY_BASE_ADDR)
#define	CMD2_REG_PHY1_CTRL_SLAVE_RATIO_0	(0x084 + DDR1_PHY_BASE_ADDR)
#define	CMD2_REG_PHY1_DLL_LOCK_DIFF_0		(0x090 + DDR1_PHY_BASE_ADDR)
#define	CMD2_REG_PHY1_INVERT_CLKOUT_0		(0x094 + DDR1_PHY_BASE_ADDR)

#define	DATA0_REG_PHY1_RD_DQS_SLAVE_RATIO_0	(0x0C8 + DDR1_PHY_BASE_ADDR)
#define	DATA0_REG_PHY1_WR_DQS_SLAVE_RATIO_0	(0x0DC + DDR1_PHY_BASE_ADDR)
#define	DATA0_REG_PHY1_WRLVL_INIT_RATIO_0	(0x0F0 + DDR1_PHY_BASE_ADDR)
#define	DATA0_REG_PHY1_WRLVL_INIT_MODE_0	(0x0F8 + DDR1_PHY_BASE_ADDR)
#define	DATA0_REG_PHY1_GATELVL_INIT_RATIO_0	(0x0FC + DDR1_PHY_BASE_ADDR)
#define	DATA0_REG_PHY1_GATELVL_INIT_MODE_0	(0x104 + DDR1_PHY_BASE_ADDR)
#define	DATA0_REG_PHY1_FIFO_WE_SLAVE_RATIO_0	(0x108 + DDR1_PHY_BASE_ADDR)
#define	DATA0_REG_PHY1_WR_DATA_SLAVE_RATIO_0	(0x120 + DDR1_PHY_BASE_ADDR)
#define	DATA0_REG_PHY1_USE_RANK0_DELAYS		(0x134 + DDR1_PHY_BASE_ADDR)
#define	DATA0_REG_PHY1_DLL_LOCK_DIFF_0		(0x138 + DDR1_PHY_BASE_ADDR)

#define	DATA1_REG_PHY1_RD_DQS_SLAVE_RATIO_0	(0x16C + DDR1_PHY_BASE_ADDR)
#define	DATA1_REG_PHY1_WR_DQS_SLAVE_RATIO_0	(0x180 + DDR1_PHY_BASE_ADDR)
#define	DATA1_REG_PHY1_WRLVL_INIT_RATIO_0	(0x194 + DDR1_PHY_BASE_ADDR)
#define	DATA1_REG_PHY1_WRLVL_INIT_MODE_0	(0x19C + DDR1_PHY_BASE_ADDR)
#define	DATA1_REG_PHY1_GATELVL_INIT_RATIO_0	(0x1A0 + DDR1_PHY_BASE_ADDR)
#define	DATA1_REG_PHY1_GATELVL_INIT_MODE_0	(0x1A8 + DDR1_PHY_BASE_ADDR)
#define	DATA1_REG_PHY1_FIFO_WE_SLAVE_RATIO_0	(0x1AC + DDR1_PHY_BASE_ADDR)
#define	DATA1_REG_PHY1_WR_DATA_SLAVE_RATIO_0	(0x1C4 + DDR1_PHY_BASE_ADDR)
#define	DATA1_REG_PHY1_USE_RANK0_DELAYS		(0x1D8 + DDR1_PHY_BASE_ADDR)
#define	DATA1_REG_PHY1_DLL_LOCK_DIFF_0		(0x1DC + DDR1_PHY_BASE_ADDR)

#define	DATA2_REG_PHY1_RD_DQS_SLAVE_RATIO_0	(0x210 + DDR1_PHY_BASE_ADDR)
#define	DATA2_REG_PHY1_WR_DQS_SLAVE_RATIO_0	(0x224 + DDR1_PHY_BASE_ADDR)
#define	DATA2_REG_PHY1_WRLVL_INIT_RATIO_0	(0x238 + DDR1_PHY_BASE_ADDR)
#define	DATA2_REG_PHY1_WRLVL_INIT_MODE_0	(0x240 + DDR1_PHY_BASE_ADDR)
#define	DATA2_REG_PHY1_GATELVL_INIT_RATIO_0	(0x244 + DDR1_PHY_BASE_ADDR)
#define	DATA2_REG_PHY1_GATELVL_INIT_MODE_0	(0x24C + DDR1_PHY_BASE_ADDR)
#define	DATA2_REG_PHY1_FIFO_WE_SLAVE_RATIO_0	(0x250 + DDR1_PHY_BASE_ADDR)
#define	DATA2_REG_PHY1_WR_DATA_SLAVE_RATIO_0	(0x268 + DDR1_PHY_BASE_ADDR)
#define	DATA2_REG_PHY1_USE_RANK0_DELAYS		(0x27C + DDR1_PHY_BASE_ADDR)
#define	DATA2_REG_PHY1_DLL_LOCK_DIFF_0		(0x280 + DDR1_PHY_BASE_ADDR)

#define	DATA3_REG_PHY1_RD_DQS_SLAVE_RATIO_0	(0x2B4 + DDR1_PHY_BASE_ADDR)
#define	DATA3_REG_PHY1_WR_DQS_SLAVE_RATIO_0	(0x2C8 + DDR1_PHY_BASE_ADDR)
#define	DATA3_REG_PHY1_WRLVL_INIT_RATIO_0	(0x2DC + DDR1_PHY_BASE_ADDR)
#define	DATA3_REG_PHY1_WRLVL_INIT_MODE_0	(0x2E4 + DDR1_PHY_BASE_ADDR)
#define	DATA3_REG_PHY1_GATELVL_INIT_RATIO_0	(0x2E8 + DDR1_PHY_BASE_ADDR)
#define	DATA3_REG_PHY1_GATELVL_INIT_MODE_0	(0x2F0 + DDR1_PHY_BASE_ADDR)
#define	DATA3_REG_PHY1_FIFO_WE_SLAVE_RATIO_0	(0x2F4 + DDR1_PHY_BASE_ADDR)
#define	DATA3_REG_PHY1_WR_DATA_SLAVE_RATIO_0	(0x30C + DDR1_PHY_BASE_ADDR)
#define	DATA3_REG_PHY1_USE_RANK0_DELAYS		(0x320 + DDR1_PHY_BASE_ADDR)
#define	DATA3_REG_PHY1_DLL_LOCK_DIFF_0		(0x324 + DDR1_PHY_BASE_ADDR)
#endif

#define DATA_MACRO_0			0
#define DATA_MACRO_1			1
#define DATA_MACRO_2			2
#define DATA_MACRO_3			3
#define DDR_PHY0			0
#define DDR_PHY1			1

/* Common DDR PHY parameters */
#define	PHY_INVERT_CLKOUT_DEFINE		0
#define	DDR3_PHY_INVERT_CLKOUT_OFF		0
#define	PHY_REG_USE_RANK0_DELAY_DEFINE		0
#define	mDDR_PHY_REG_USE_RANK0_DELAY_DEFINE	1
#define	PHY_DLL_LOCK_DIFF_DEFINE		0x4
#define	PHY_CMD0_DLL_LOCK_DIFF_DEFINE		0x4
#define DDR_EMIF_REF_TRIGGER			0x10000000

#define	PHY_GATELVL_INIT_CS0_DEFINE		0x0
#define	PHY_WRLVL_INIT_CS0_DEFINE		0x0

#define	PHY_GATELVL_INIT_CS1_DEFINE		0x0
#define	PHY_WRLVL_INIT_CS1_DEFINE		0x0
#define	PHY_CTRL_SLAVE_RATIO_CS1_DEFINE		0x80

/* TI814X DDR2 PHY CFG parameters  <emif0 : emif1> */
#define	DDR2_PHY_RD_DQS_CS0_DEFINE		((emif == 0) ? 0x35 : 0x35)
#define	DDR2_PHY_WR_DQS_CS0_DEFINE		((emif == 0) ? 0x20 : 0x20)
#define	DDR2_PHY_RD_DQS_GATE_CS0_DEFINE		((emif == 0) ? 0x90 : 0x90)
#define	DDR2_PHY_WR_DATA_CS0_DEFINE		((emif == 0) ? 0x50 : 0x50)
#define	DDR2_PHY_CTRL_SLAVE_RATIO_CS0_DEFINE	0x80

#if defined(CONFIG_TI811X)
/* TI811X DDR3 PHY CFG parameters   <emif0> */
#define DDR3_PHY_RD_DQS_CS0_DEFINE		0x39
#define DDR3_PHY_WR_DQS_CS0_DEFINE		0x30
#define DDR3_PHY_RD_DQS_GATE_CS0_DEFINE		0xBC
#define DDR3_PHY_WR_DATA_CS0_DEFINE		0x62
#define DDR3_PHY_CTRL_SLAVE_RATIO_CS0_DEFINE	0x80

#elif defined(CONFIG_TI813X)  || defined(CONFIG_DM385_IPNC)
/* TI813X DDR3 PHY CFG parameters   <emif0> */

#define DDR3_PHY_RD_DQS_CS0_BYTE0			0x3c //0x3d //0x30//0x41
#define DDR3_PHY_RD_DQS_CS0_BYTE1			0x3d //0x3c //0x30//0x38
#define DDR3_PHY_RD_DQS_CS0_BYTE2			0x3d //0x3e //0x30//0x46
#define DDR3_PHY_RD_DQS_CS0_BYTE3			0x3e //0x3b //0x30//0x37

#define DDR3_PHY_WR_DQS_CS0_BYTE0			0x40 //0x44 //0x21//0x39
#define DDR3_PHY_WR_DQS_CS0_BYTE1			0x40 //0x43 //0x21//0x3F
#define DDR3_PHY_WR_DQS_CS0_BYTE2			0x4e //0x55 //0x21//0x46
#define DDR3_PHY_WR_DQS_CS0_BYTE3			0x53 //0x55 //0x21//0x45

#define DDR3_PHY_RD_DQS_GATE_CS0_BYTE0		0xa1 //0xa1 //0xc0//0xBD
#define DDR3_PHY_RD_DQS_GATE_CS0_BYTE1		0xa7 //0xad //0xc0//0xE0
#define DDR3_PHY_RD_DQS_GATE_CS0_BYTE2		0xcb //0xcd //0xc0//0x105
#define DDR3_PHY_RD_DQS_GATE_CS0_BYTE3		0xcb //0xc8 //0xc0//0x12B

#define DDR3_PHY_WR_DATA_CS0_BYTE0			0x7a //0x84 //0x44//0x7C
#define DDR3_PHY_WR_DATA_CS0_BYTE1			0x7a //0x7e //0x44//0x73
#define DDR3_PHY_WR_DATA_CS0_BYTE2			0x7f //0x82 //0x44//0x78
#define DDR3_PHY_WR_DATA_CS0_BYTE3			0x7f //0x81 //0x44//0x6D

#define DDR3_PHY_CTRL_SLAVE_RATIO_CS0_DEFINE	0x80

#else
/* TI814X DDR3 PHY CFG parameters   <emif0 : emif 1> */
#if defined(CONFIG_TI814X_DDR3_533)
#define DDR3_PHY_RD_DQS_CS0_BYTE0		((emif == 0) ? 0x3c : 0x39) 
#define DDR3_PHY_RD_DQS_CS0_BYTE1		((emif == 0) ? 0x3f : 0x3b) 
#define DDR3_PHY_RD_DQS_CS0_BYTE2		((emif == 0) ? 0x31 : 0x35) 
#define DDR3_PHY_RD_DQS_CS0_BYTE3		((emif == 0) ? 0x31 : 0x32) 

#define DDR3_PHY_WR_DQS_CS0_BYTE0		((emif == 0) ? 0x4d : 0x49) 
#define DDR3_PHY_WR_DQS_CS0_BYTE1		((emif == 0) ? 0x4f : 0x48) 
#define DDR3_PHY_WR_DQS_CS0_BYTE2		((emif == 0) ? 0x51 : 0x4b) 
#define DDR3_PHY_WR_DQS_CS0_BYTE3		((emif == 0) ? 0x53 : 0x49) 

#define DDR3_PHY_RD_DQS_GATE_CS0_BYTE0		((emif == 0) ? 0xb7 : 0xa9) 
#define DDR3_PHY_RD_DQS_GATE_CS0_BYTE1		((emif == 0) ? 0xb9 : 0xac) 
#define DDR3_PHY_RD_DQS_GATE_CS0_BYTE2		((emif == 0) ? 0xd4 : 0xc3) 
#define DDR3_PHY_RD_DQS_GATE_CS0_BYTE3		((emif == 0) ? 0xce : 0xc5) 

#define DDR3_PHY_WR_DATA_CS0_BYTE0		((emif == 0) ? 0x82 : 0x80) 
#define DDR3_PHY_WR_DATA_CS0_BYTE1		((emif == 0) ? 0x84 : 0x7d) 
#define DDR3_PHY_WR_DATA_CS0_BYTE2		((emif == 0) ? 0x7f : 0x80) 
#define DDR3_PHY_WR_DATA_CS0_BYTE3		((emif == 0) ? 0x87 : 0x82) 
#else /* CONFIG_TI814X_DDR3_533 */
#define DDR3_PHY_RD_DQS_CS0_BYTE0		((emif == 0) ? 0x3d : 0x39) //((emif == 0) ? 0x30 : 0x30)
#define DDR3_PHY_RD_DQS_CS0_BYTE1		((emif == 0) ? 0x3c : 0x3c) //((emif == 0) ? 0x30 : 0x30)
#define DDR3_PHY_RD_DQS_CS0_BYTE2		((emif == 0) ? 0x3e : 0x3c) //((emif == 0) ? 0x30 : 0x30)
#define DDR3_PHY_RD_DQS_CS0_BYTE3		((emif == 0) ? 0x3d : 0x3e) //((emif == 0) ? 0x30 : 0x30)

#define DDR3_PHY_WR_DQS_CS0_BYTE0		((emif == 0) ? 0x45 : 0x40) //((emif == 0) ? 0x21 : 0x21)
#define DDR3_PHY_WR_DQS_CS0_BYTE1		((emif == 0) ? 0x44 : 0x3f) //((emif == 0) ? 0x21 : 0x21)
#define DDR3_PHY_WR_DQS_CS0_BYTE2		((emif == 0) ? 0x47 : 0x45) //((emif == 0) ? 0x21 : 0x21)
#define DDR3_PHY_WR_DQS_CS0_BYTE3		((emif == 0) ? 0x47 : 0x42) //((emif == 0) ? 0x21 : 0x21)

#define DDR3_PHY_RD_DQS_GATE_CS0_BYTE0		((emif == 0) ? 0x9d : 0x9e) //((emif == 0) ? 0xc0 : 0xc0) 
#define DDR3_PHY_RD_DQS_GATE_CS0_BYTE1		((emif == 0) ? 0x9e : 0x9c) //((emif == 0) ? 0xc0 : 0xc0) 
#define DDR3_PHY_RD_DQS_GATE_CS0_BYTE2		((emif == 0) ? 0xa7 : 0xa3) //((emif == 0) ? 0xc0 : 0xc0)
#define DDR3_PHY_RD_DQS_GATE_CS0_BYTE3		((emif == 0) ? 0xa9 : 0xa9) //((emif == 0) ? 0xc0 : 0xc0)

#define DDR3_PHY_WR_DATA_CS0_BYTE0		((emif == 0) ? 0x7e : 0x78) //((emif == 0) ? 0x44 : 0x44)
#define DDR3_PHY_WR_DATA_CS0_BYTE1		((emif == 0) ? 0x7f : 0x7b) //((emif == 0) ? 0x44 : 0x44)
#define DDR3_PHY_WR_DATA_CS0_BYTE2		((emif == 0) ? 0x7f : 0x7b) //((emif == 0) ? 0x44 : 0x44)
#define DDR3_PHY_WR_DATA_CS0_BYTE3		((emif == 0) ? 0x80 : 0x7e) //((emif == 0) ? 0x44 : 0x44)
#endif /* CONFIG_TI814X_DDR3_533 */

#define DDR3_PHY_CTRL_SLAVE_RATIO_CS0_DEFINE	0x80
#endif

/* DDR0/1 IO CTRL parameters */
#define DDR0_IO_CTRL_DEFINE		0x00030303
#define DDR1_IO_CTRL_DEFINE		0x00030303

/* Initially set a large DDR refresh period */
#define DDR_EMIF_REF_CTRL		0x00004000

/* TI814X DDR2 EMIF CFG Registers values 333MHz*/
#define DDR2_EMIF_READ_LATENCY		0x00170207
#define DDR2_EMIF_TIM1				0x0AAAF552
#define DDR2_EMIF_TIM2				0x043631D2
#define DDR2_EMIF_TIM3				0x00000327
#define DDR2_EMIF_REF_CTRL			0x10000C30
#define DDR2_EMIF_SDRAM_CONFIG		0x40801AB2
#define DDR2_EMIF_SDRAM_ZQCR		0x50074BE1

/*===================================================================================================*/
#ifdef CONFIG_TI813X	
/*===================================================================================================*/
/* select the DDR3 Freq and timing paramets */
#define CONFIG_TI813X_DDR3_400 /* Values supported 400,533 */

/* TI813X DDR3 EMIF CFG Registers values 400MHz */
#if defined(CONFIG_TI813X_DDR3_400)
#define DDR3_EMIF_READ_LATENCY		0x00170209
#define DDR3_EMIF_TIM1				0x132BB953
#define DDR3_EMIF_TIM2				0x20437FDA
#define DDR3_EMIF_TIM3				0x501F87FF
#define DDR3_EMIF_REF_CTRL			0x00000C30
#define DDR3_EMIF_SDRAM_CONFIG		0x61C11B32
#define DDR3_EMIF_SDRAM_ZQCR		0x50074BE1
#endif

/* TI813X DDR3 EMIF CFG Registers values 533MHz */
#if defined(CONFIG_TI813X_DDR3_533)
#define DDR3_EMIF_READ_LATENCY		0x0017020A
#define DDR3_EMIF_TIM1				0x110F783B
#define DDR3_EMIF_TIM2				0x238581E6
#define DDR3_EMIF_TIM3				0x501F86AF
#define DDR3_EMIF_REF_CTRL			0x0000103D
#define DDR3_EMIF_SDRAM_CONFIG		0x61C121B2
#define DDR3_EMIF_SDRAM_ZQCR		0x50074BE1
#endif

/*===================================================================================================*/
#elif defined(CONFIG_DM385)	
/*===================================================================================================*/

/* DM385 DDR3 EMIF CFG Registers values 533MHz */
#if defined(CONFIG_DM385_DDR3_533)
/*
#define DDR3_EMIF_READ_LATENCY     	0x00170209 // 0x0017320A		//RD_ODT=0x2, IDLE_ODT=0x0, Dynamic power_down enabled
#define DDR3_EMIF_TIM1          	0x0CCF46B3 // 0x110F783B
#define DDR3_EMIF_TIM2          	0x20047FDA // 0x238581E6
#define DDR3_EMIF_TIM3          	0x507F855F // 0x501F86AF
#define DDR3_EMIF_REF_CTRL      	0x0000103D
#define DDR3_EMIF_SDRAM_CONFIG      0x61C11AB2 //0x61C21AB2 // 0x61C119B2 // 0x61C121B2
#define DDR3_EMIF_SDRAM_ZQCR        0x50074BE2 // 0x50074BE1
*/

/*******************************************************/
#if defined(CONFIG_DM388_DDR3_4Gb)
/* Gary - DM388 DDR3(4Gb) 533 MHz */
#define DDR3_EMIF_READ_LATENCY		0x00170209 // 0x0017320A
#define DDR3_EMIF_TIM1				0x0CCF36A3
#define DDR3_EMIF_TIM2				0x308F7FDA
#define DDR3_EMIF_TIM3				0x507F88AF
#define DDR3_EMIF_REF_CTRL			0x0000081E
#ifdef CONFIG_DM388_DDR3_4GB_SINGLE
#define DDR3_EMIF_SDRAM_CONFIG		0x61C15A32 //Bobby-20140707: Set Data Bus Width to 16bit
#else
#define DDR3_EMIF_SDRAM_CONFIG		0x61C11A32
#endif
#define DDR3_EMIF_SDRAM_ZQCR		0x500797CF
#elif defined(CONFIG_DM388_DDR3_2Gb)
/* Gary - DM388 DDR3(2Gb) 533 MHz */
#define DDR3_EMIF_READ_LATENCY		0x00170209 // 0x0017320A
#define DDR3_EMIF_TIM1				0x0CCF36A3
#define DDR3_EMIF_TIM2				0x305A7FDA
#define DDR3_EMIF_TIM3				0x507F855F
#define DDR3_EMIF_REF_CTRL			0x0000081E
#define DDR3_EMIF_SDRAM_CONFIG		0x61C11A32
#define DDR3_EMIF_SDRAM_ZQCR		0x500797CF
#else
/* Gary - DM388 DDR3(1Gb) 533 MHz */
#define DDR3_EMIF_READ_LATENCY		0x00170209 // 0x0017320A
#define DDR3_EMIF_TIM1				0x0CCF36A3
#define DDR3_EMIF_TIM2				0x303F7FDA
#define DDR3_EMIF_TIM3				0x507F83AF
#define DDR3_EMIF_REF_CTRL			0x0000081E
#ifdef CONFIG_DM388_DDR3_4GB_SINGLE
#define DDR3_EMIF_SDRAM_CONFIG		0x61C15A32 //Bobby-20140707: Set Data Bus Width to 16bit
#else
#define DDR3_EMIF_SDRAM_CONFIG		0x61C11A32
#endif
#define DDR3_EMIF_SDRAM_ZQCR		0x500797CF
#endif
/*******************************************************/

#endif

/* DM385 DDR3 EMIF CFG Registers values 400MHz */
#if defined(CONFIG_DM385_DDR3_400)
/*
#define DDR3_EMIF_READ_LATENCY		0x00170208		//RD_ODT=0x2, IDLE_ODT=0x0, Dynamic power_down enabled
#define DDR3_EMIF_TIM1				0x132BB953
#define DDR3_EMIF_TIM2				0x20437FDA
#define DDR3_EMIF_TIM3				0x501F87FF
#define DDR3_EMIF_REF_CTRL			0x00000C30
#define DDR3_EMIF_SDRAM_CONFIG		0x61C11B32
#define DDR3_EMIF_SDRAM_ZQCR		0x50074BE1
*/

/*******************************************************/
#if defined(CONFIG_DM388_DDR3_4Gb)
/* Gary - DM388 DDR3(4Gb) 400 MHz */
#define DDR3_EMIF_READ_LATENCY		0x00170209 // 0x0017320A
#define DDR3_EMIF_TIM1				0x0AAAE51B
#define DDR3_EMIF_TIM2				0x206B7FDA
#define DDR3_EMIF_TIM3				0x507F867F
#define DDR3_EMIF_REF_CTRL			0x00000618
#define DDR3_EMIF_SDRAM_CONFIG		0x61C11A32
#define DDR3_EMIF_SDRAM_ZQCR		0x500797C2
#elif defined(CONFIG_DM388_DDR3_2Gb)
/* Gary - DM388 DDR3(2Gb) 400 MHz */
#define DDR3_EMIF_READ_LATENCY		0x00170209 // 0x0017320A
#define DDR3_EMIF_TIM1				0x0AAAE51B
#define DDR3_EMIF_TIM2				0x20437FDA
#define DDR3_EMIF_TIM3				0x507F83FF
#define DDR3_EMIF_REF_CTRL			0x00000618
#define DDR3_EMIF_SDRAM_CONFIG		0x61C11A32
#define DDR3_EMIF_SDRAM_ZQCR		0x500797C2
#else
/* Gary - DM388 DDR3(1Gb) 400 MHz */
#define DDR3_EMIF_READ_LATENCY		0x00170209 // 0x0017320A
#define DDR3_EMIF_TIM1				0x0AAAE51B
#define DDR3_EMIF_TIM2				0x202F7FDA
#define DDR3_EMIF_TIM3				0x507F82BF
#define DDR3_EMIF_REF_CTRL			0x00000618
#define DDR3_EMIF_SDRAM_CONFIG		0x61C11A32
#define DDR3_EMIF_SDRAM_ZQCR		0x500797C2
#endif
/*******************************************************/
#endif

/*===================================================================================================*/
#elif defined(CONFIG_TI811X)
/*===================================================================================================*/

/* TI811X DDR3 EMIF CFG Registers values 400MHz */
#define DDR3_EMIF_READ_LATENCY		0x00170209
#define DDR3_EMIF_TIM1				0x0AAAD4DB
#define DDR3_EMIF_TIM2				0x682F7FDA
#define DDR3_EMIF_TIM3				0x501F82BF
#define DDR3_EMIF_REF_CTRL			0x00000C30
#define DDR3_EMIF_SDRAM_CONFIG		0x61C011B2
#define DDR3_EMIF_SDRAM_ZQCR		0x50074BE1

/*===================================================================================================*/
#else	
/*===================================================================================================*/
#if defined(CONFIG_TI814X_DDR3_533)
/* TI814X DDR3 EMIF CFG Registers values 533MHz */
#if defined(CONFIG_TI814X_DDR3_MICRON_4Gb)
#define DDR3_EMIF_READ_LATENCY		0x00170207      //->DDRPHYCR //RD_ODT=0x2, IDLE_ODT=0x0, Dynamic power_down enabled
#define DDR3_EMIF_TIM1				0x0EEF36F3
#define DDR3_EMIF_TIM2				0x308F7FDA
#define DDR3_EMIF_TIM3				0x507F88AF
#define DDR3_EMIF_REF_CTRL			0x0000103D 
#define DDR3_EMIF_SDRAM_CONFIG		0x61C12332
#define DDR3_EMIF_SDRAM_ZQCR		0x00170209
#else
#define DDR3_EMIF_READ_LATENCY		0x00170208      //->DDRPHYCR //RD_ODT=0x2, IDLE_ODT=0x0, Dynamic power_down enabled
#define DDR3_EMIF_TIM1				0x0CCF36A3
#define DDR3_EMIF_TIM2				0x308F7FDA
#define DDR3_EMIF_TIM3				0x507F88AF
#define DDR3_EMIF_REF_CTRL			0x0000103D
#define DDR3_EMIF_SDRAM_CONFIG		0x61C11B32
#define DDR3_EMIF_SDRAM_ZQCR		0x50074BE1
#endif
#else /* CONFIG_TI814X_DDR3_533 */
/* TI814X DDR3 EMIF CFG Registers values 400MHz */
#if defined(CONFIG_TI814X_DDR3_4Gb)
#if defined(CONFIG_TI814X_DDR3_MICRON_4Gb)
#define DDR3_EMIF_READ_LATENCY		0x00170207      //->DDRPHYCR //RD_ODT=0x2, IDLE_ODT=0x0, Dynamic power_down enabled
#define DDR3_EMIF_TIM1				0x0AAAE523
#define DDR3_EMIF_TIM2				0x206B7FDA
#define DDR3_EMIF_TIM3				0x507F867F
#define DDR3_EMIF_REF_CTRL			0x00000C30
#define DDR3_EMIF_SDRAM_CONFIG		0x61C01332
#define DDR3_EMIF_SDRAM_ZQCR		0x500742C6
#else
#define DDR3_EMIF_READ_LATENCY		0x00170208		//RD_ODT=0x2, IDLE_ODT=0x0, Dynamic power_down enabled
#define DDR3_EMIF_TIM1				0x0AAAD4DB
#define DDR3_EMIF_TIM2				0x682F7FDA
#define DDR3_EMIF_TIM3				0x501F867F		//Yosun, for 4Gb(tRFC=260@400MHz)
#define DDR3_EMIF_REF_CTRL			0x00000C30
#define DDR3_EMIF_SDRAM_CONFIG		0x61C011B2
#define DDR3_EMIF_SDRAM_ZQCR		0x50074BE1
#endif
#elif defined(CONFIG_TI814X_DDR3_2Gb)
#define DDR3_EMIF_READ_LATENCY		0x00170208		//RD_ODT=0x2, IDLE_ODT=0x0, Dynamic power_down enabled
#define DDR3_EMIF_TIM1				0x0AAAD4DB
#define DDR3_EMIF_TIM2				0x682F7FDA
#define DDR3_EMIF_TIM3				0x501F83FF		//Yosun, for 2Gb(tRFC=160@400MHz)
#define DDR3_EMIF_REF_CTRL			0x00000C30
#define DDR3_EMIF_SDRAM_CONFIG		0x61C011B2
#define DDR3_EMIF_SDRAM_ZQCR		0x50074BE1
#else
#define DDR3_EMIF_READ_LATENCY		0x00170208		//RD_ODT=0x2, IDLE_ODT=0x0, Dynamic power_down enabled
#define DDR3_EMIF_TIM1				0x0AAAD4DB
#define DDR3_EMIF_TIM2				0x682F7FDA
#define DDR3_EMIF_TIM3				0x501F82BF		//Yosun, for 1Gb(tRFC=110@400MHz)
#define DDR3_EMIF_REF_CTRL			0x00000C30
#define DDR3_EMIF_SDRAM_CONFIG		0x61C011B2
#define DDR3_EMIF_SDRAM_ZQCR		0x50074BE1
#endif
#endif /* CONFIG_TI814X_DDR3_533 */

/*===================================================================================================*/
#endif	
/*===================================================================================================*/

/*
 * TI814X PG1.0 DMM LISA MAPPING
 * Two 256MB sections with 128-byte interleaved(hole in b/w)
 */
#define PG1_0_DMM_LISA_MAP__0		0x0
#define PG1_0_DMM_LISA_MAP__1		0x0
#define PG1_0_DMM_LISA_MAP__2		0x805C0300
#define PG1_0_DMM_LISA_MAP__3		0xA05C0300

/*
 * TI814X PG2.1 DMM LISA MAPPING
 * 1G contiguous section with 128-byte interleaving
 */
#if defined(CONFIG_TI814X_DDR3_4Gb)
#define PG2_1_DMM_LISA_MAP__0		0x0
#define PG2_1_DMM_LISA_MAP__1		0x0
#define PG2_1_DMM_LISA_MAP__2		0x806C0300 /* Register 2 maps 0x80000000 to 0x00000000, length 1GB */
#define PG2_1_DMM_LISA_MAP__3		0xC06C0320 /* Register 3 maps 0xC0000000 to 0x20000000, length 1GB */
#elif defined(CONFIG_TI814X_DDR3_2Gb)
#define PG2_1_DMM_LISA_MAP__0		0x0
#define PG2_1_DMM_LISA_MAP__1		0x0
#define PG2_1_DMM_LISA_MAP__2		0x805C0300 /* Register 2 maps 0x80000000 to 0x00000000, length 512MB */
#define PG2_1_DMM_LISA_MAP__3		0xA05C0310 /* Register 3 maps 0xA0000000 to 0x10000000, length 512MB */
#else
#define PG2_1_DMM_LISA_MAP__0		0x0
#define PG2_1_DMM_LISA_MAP__1		0x0
#define PG2_1_DMM_LISA_MAP__2		0x805C0300 /* Register 2 maps 0x80000000 to 0x00000000, length 512MB */
#define PG2_1_DMM_LISA_MAP__3		0xA05C0300 /* Register 3 maps 0xA0000000 to 0x00000000, length 512MB */
#endif

/*
 * TI813X  DM385 DMM LISA MAPPING
 * 1G contiguous section with no interleaving
 */
#define DDR3_DMM_LISA_MAP__0		0x0
#define DDR3_DMM_LISA_MAP__1		0x0
#define DDR3_DMM_LISA_MAP__2		0x0
#ifdef CONFIG_DM388_DDR3_4GB_SINGLE
#define DDR3_DMM_LISA_MAP__3		0x80500100 /* Register 3 maps 0x80000000 to 0x00000000, SDRC 0 only (not interleaved), length 512MB */
#else
#define DDR3_DMM_LISA_MAP__3		0x80600100 /* Register 3 maps 0x80000000 to 0x00000000, SDRC 0 only (not interleaved), length 1GB */
#endif

#endif  /* _DDR_DEFS_TI814X_H */

