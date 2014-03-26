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
/* 
	1-MMC1_CMD 
*/
/* 1-4 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 5-8 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 9-12 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 
	14-usb1_drvvbus, fn8, pulldn enable  
	15-AIC_RSTn (GP0_8, O)
	16-AIC_MCLK
*/
/* 13-16 */	BIT(0), BIT(7), BIT(7), BIT(0),
/* 17-20 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 21-24 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 25-28 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 29-32 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 33-36 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 37-40 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 41-44 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 45-48 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 49-52 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 51-56 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 
	59-SD1_WPn (GP0_29, I)
	60-SD1_CDn (GP0_30, I)
*/
/* 57-60 */	BIT(0), BIT(0), BIT(7), BIT(7),
/* 
	61-SD1_EN (GP0_31, O)
*/
/* 61-64 */	BIT(7), BIT(0), BIT(0), BIT(0),
/* 
	68-FLASH_WP (GP1_0, O)
*/
/* 65-68 */	BIT(0), BIT(0), BIT(0), BIT(7),
/* 
	70-UART0_RXD (Console port)
	71-UART0_TXD (Console port)
*/
/* 69-72 */	BIT(7), BIT(0), BIT(0), BIT(0),
/* 
	74-(GP1_2) (x)
	75-(GP1_3) (x)
	76-UART1_TXD (RS-485 PTZ port)
*/
/* 73-76 */	BIT(0), BIT(7), BIT(7), BIT(2),
/* 
	77-UART1_RXD (RS-485 PTZ port)
	78-HDMI_SCL (x)
	79-HDMI_SDA (x)
*/
/* 77-80 */	BIT(2), BIT(1), BIT(1), BIT(0),
/* 81-84 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 
	86-RS485_4W (GP1_17, O)
	87-RS485_ENT (GP1_18, O)
	88-RS485_ENRn (GP1_26, O)
*/
/* 85-88 */	BIT(0), BIT(7), BIT(7), BIT(7),
/* 89-92 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 93-96 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 97-100 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 101-104 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 105-108 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 
	111-HDMI_CEC (x)
	112-HDMI_HPDET (x)
*/
/* 109-112 */	BIT(0), BIT(0), BIT(4), BIT(4),
/* 
	115-TIM6_IO (x)
	116-TIM7_IO (x)
*/
/* 113-116 */	BIT(0), BIT(0), BIT(6), BIT(6),
/* 117-120 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 121-124 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 125-128 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 129-132 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 
	134- CLKOUT0 
*/
/* 133-136 */	BIT(0), BIT(5), BIT(0), BIT(0), 
/*
	140~155-VIN[0]A_D[0]~VIN[0]A_D[15]
	
	Video Input 0 Data inputs.
	For 16-bit capture, 
	D[7:0] are Cb/Cr
	and [15:8] are Y
*/
/* 137-140 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 141-144 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 145-148 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 149-152 */	BIT(0), BIT(0), BIT(0), BIT(0), 
/* 
	156-CAM_D[8] 
*/
/* 153-156 */	BIT(0), BIT(0), BIT(0), BIT(1), 
/* 
	157-CAM_D[9] 
	158-CAM_D[10] 
	159-CAM_D[11] 
	160-CAM_D[12] 
*/
/* 157-160 */	BIT(1), BIT(1), BIT(1), BIT(1),
/* 
	161-CAM_D[13] 
	162-CAM_D[14] 
	163-CAM_D[15] 
	164-CAM_D[7] 
*/
/* 161-164 */	BIT(1), BIT(1), BIT(1), BIT(1),
/* 
	165-CAM_D[6] 
	166-CAM_D[5] 
	167-CAM_D[4] 
	168-CAM_D[3] 
*/
/* 165-168 */	BIT(1), BIT(1), BIT(1), BIT(1),
/* 
	169-CAM_D[2] 
	170-CAM_D[1] 
	171-CAM_D[0] 
	172-CAM_HS
*/
/* 169-172 */	BIT(1), BIT(1), BIT(1), BIT(1),
/* 
	173-CAM_VS
	174-gp0_28 (x)
	175-CAM_PCLK 
*/
/* 173-176 */	BIT(1), BIT(7), BIT(1), BIT(0),
/* 
	179-RE_SETING (GP2_21, I)
	180-ENET_RSTn (GP2_22, O)
*/
/* 177-180 */	BIT(0), BIT(0), BIT(7), BIT(7),
/* 
	181-E_LINKSTS (GP2_23, I)
*/
/* 181-184 */	BIT(7), BIT(0), BIT(0), BIT(0),
/* 
	188-CMOS_OE (GP2_24, O)
*/
/* 185-188 */	BIT(0), BIT(0), BIT(0), BIT(7),
/* 
	189-CMOS_RST (GP2_25, O)
*/
/* 189-192 */	BIT(7), BIT(0), BIT(0), BIT(0),
/* 
	196-RTC_INTn (GP2_26, I)
*/
/* 193-196 */	BIT(0), BIT(0), BIT(0), BIT(7),
/* 
	197-HD_CLK (GP2_27, O)
*/
/* 197-200 */	BIT(7), BIT(0), BIT(0), BIT(0),
/* 201-204 */	BIT(0), BIT(0), BIT(0), BIT(0),
/*
	205-PI_POWER (GP2_29, O)
	206-CAMERA_POWER (GP2_30, O)
*/
/* 205-208 */	BIT(7), BIT(7), BIT(0), BIT(0),
/* 
	211-CAMERA_TX (UART3_RXD)
	212-CAMERA_RX (UART3_TXD)
*/
/* 209-212 */	BIT(0), BIT(0), BIT(5), BIT(5), 
/* 
	215-ARM_IN (GP3_7, I)
	216-ARM_OUT (GP3_8, O)
*/
/* 213-216 */	BIT(0), BIT(0), BIT(7), BIT(7), 
/* 
	220-LED_G (GP3_12, O)
*/
/* 217-220 */	BIT(0), BIT(0), BIT(0), BIT(7), 
/* 
	221-LED_R (GP3_13, O)
*/
/* 221-224 */	BIT(7), BIT(0), BIT(0), BIT(0),
/* 225-228 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 
	232~258-GMII0 INTERFACE 
*/
/* 229-232 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 233-236 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 237-240 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 241-244 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 245-248 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 249-252 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 253-256 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 257-260 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 261-264 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 265-268 */	BIT(0), BIT(0), BIT(0), BIT(0),
/* 
	270-usb0_drvvbus, fn1, pulldn enable 
*/
/* 269-271 */	BIT(0), BIT(0), BIT(0),
