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
			BIT(0), /* 1-SD1_CLK */
			BIT(0), /* 2-SD1_CMD */
			BIT(0), /* 3-SD1_DAT[0] */
			BIT(0), /* 4-SD1_DAT[1]_SDIRQ */
			BIT(0), /* 5-SD1_DAT[2]_SDRW */
			BIT(0), /* 6-SD1_DAT[3] */
			BIT(7), /* 7-Fan_int (GP1_7, I) */
			BIT(0), /* 8 */
			BIT(0), /* 9 */
			BIT(0), /* 10 */
			BIT(0), /* 11 */
			BIT(0), /* 12 */
			BIT(0), /* 13 */
			BIT(7), /* 14-usb1_drvvbus, fn8, pulldn enable */
			BIT(7), /* 15-AIC_RSTn (GP0_8, O) */
			BIT(0), /* 16-AIC_MCLK */
			BIT(0), /* 17 */
			BIT(0), /* 18 */
			BIT(0), /* 19 */
			BIT(0), /* 20 */
			BIT(0), /* 21 */
			BIT(0), /* 22 */
			BIT(0), /* 23 */
			BIT(0), /* 24 */
			BIT(0), /* 25 */
			BIT(0), /* 26 */
			BIT(0), /* 27 */
			BIT(0), /* 28 */
			BIT(0), /* 29 */
			BIT(0), /* 30 */
			BIT(0), /* 31 */
			BIT(0), /* 32 */
			BIT(0), /* 33 */
			BIT(0), /* 34 */
			BIT(0), /* 35 */
			BIT(0), /* 36 */
			BIT(0), /* 37 */
			BIT(0), /* 38 */
			BIT(0), /* 39 */
			BIT(0), /* 40 */
			BIT(0), /* 41 */
			BIT(0), /* 42 */
			BIT(7), /* 43-Fan_con (GP0_14, O) */
			BIT(0), /* 44 */
			BIT(0), /* 45 */
			BIT(7), /* 46-Heatersys_int (GP0_17, I) */
			BIT(0), /* 47 */
			BIT(0), /* 48 */
			BIT(0), /* 49 */
			BIT(0), /* 50 */
			BIT(0), /* 51 */
			BIT(7), /* 52-Heater_sys (GP0_22, O) */
			BIT(0), /* 53 */
			BIT(0), /* 54 */
			BIT(0), /* 55 */
			BIT(7), /* 56-Heater_cam (GP0_26, O) */
			BIT(0), /* 57 */
			BIT(0), /* 58 */
			BIT(7), /* 59-SD1_WPn (GP0_29, I) */
			BIT(7), /* 60-SD1_CDn (GP0_30, I) */
			BIT(7), /* 61-SD1_EN (GP0_31, O)  */
			BIT(0), /* 62 */
			BIT(0), /* 63-(GP1_8) */
			BIT(7), /* 64-(GP1_9) */
			BIT(7), /* 65-(GP1_10) */
			BIT(0), /* 66-Reserved. Do Not Program this Register. */
			BIT(0), /* 67-Reserved. Do Not Program this Register. */
			BIT(7), /* 68-FLASH_WP (GP1_0, O) */
			BIT(7), /* 69 */
			BIT(0), /* 70-UART0_RXD (Console port) */
			BIT(0), /* 71-UART0_TXD (Console port) */
			BIT(0), /* 72-UART0_CTS */
			BIT(0), /* 73-UART0_RTS */
			BIT(0), /* 74-UART0_DCD */
			BIT(0), /* 75-UART0_DSR */ 
#ifdef VPORT66
			BIT(7), /* 76-Heatercam_int (GP1_4, I) */
			BIT(0), /* 77 */
#else
			BIT(2), /* 76-UART1_TXD (RS-485 PTZ port) */
			BIT(2), /* 77-UART1_RXD (RS-485 PTZ port) */
#endif
			BIT(0), /* 78 */
			BIT(0), /* 79 */
			BIT(0), /* 80 */
			BIT(0), /* 81 */
			BIT(0), /* 82 */
			BIT(0), /* 83 */
			BIT(0), /* 84 */
			BIT(0), /* 85 */
			BIT(7), /* 86-RS485_4W (GP1_17, O) */
			BIT(7), /* 87-RS485_ENT (GP1_18, O) */
			BIT(7), /* 88-RS485_ENRn (GP1_26, O) */
			BIT(0), BIT(0), BIT(0), BIT(0), /* 89-92 */
			BIT(0), BIT(0), BIT(0), BIT(0), /* 93-96 */
			BIT(0), BIT(0), BIT(0), BIT(0), /* 97-100 */
			BIT(0), BIT(0), BIT(0), BIT(0), /* 101-104 */
			BIT(0), BIT(0), BIT(0), BIT(0), /* 105-108 */
			BIT(0), /* 109 */
			BIT(0), /* 110 */
			BIT(4), /* 111-HDMI_CEC (x) */
			BIT(4), /* 112-HDMI_HPDET (x) */
			BIT(0), /* 113 */
			BIT(0), /* 114 */
			BIT(6), /* 115-TIM6_IO (x) */
			BIT(6), /* 116-TIM7_IO (x) */
			BIT(0), BIT(0), BIT(0), BIT(0), /* 117-120 */
			BIT(0), BIT(0), BIT(0), BIT(0), /* 121-124 */
			BIT(0), BIT(0), BIT(0), BIT(0), /* 125-128 */
			BIT(0), BIT(0), BIT(0), BIT(0), /* 129-132 */
			BIT(0), /* 133 */
			BIT(5), /* 134-CLKOUT0 */
			BIT(7), /* 135-MCU_RST (GP2_0, O) */
			BIT(7), /* 136 */
			BIT(0), /* 137 */
			BIT(0), /* 138 */
			BIT(0), /* 139 */
/*
	140~155-VIN[0]A_D[0]~VIN[0]A_D[15]

	Video Input 0 Data inputs.
	For 16-bit capture, 
	D[7:0] are Cb/Cr
	and [15:8] are Y
*/
			BIT(0), BIT(0), BIT(0), BIT(0), /* 140-143 */
			BIT(0), BIT(0), BIT(0), BIT(0), /* 144-147 */
			BIT(0), BIT(0), BIT(0), BIT(0), /* 148-151 */
			BIT(0), BIT(0), BIT(0), BIT(0), /* 152-155 */
/* 
	156-CAM_D[8]
	157-CAM_D[9] 
	158-CAM_D[10] 
	159-CAM_D[11] 
	160-CAM_D[12] 
	161-CAM_D[13] 
	162-CAM_D[14] 
	163-CAM_D[15] 
	164-CAM_D[7] 
	165-CAM_D[6] 
	166-CAM_D[5] 
	167-CAM_D[4] 
	168-CAM_D[3] 
	169-CAM_D[2] 
	170-CAM_D[1] 
	171-CAM_D[0] 
	172-CAM_HS
	173-CAM_VS
	174-CAM_FLD
	175-CAM_PCLK 
*/
			BIT(1), BIT(1), BIT(1), BIT(1), /* 156-159 */
			BIT(1), BIT(1), BIT(1), BIT(1), /* 160-163 */
			BIT(1), BIT(1), BIT(1), BIT(1), /* 164-167 */
			BIT(1), BIT(1), BIT(1), BIT(1), /* 168-171 */
			BIT(1), BIT(1), BIT(1), BIT(1), /* 172-175 */
			BIT(0), /* 176 */
			BIT(0), /* 177 */
			BIT(0), /* 178 */
			BIT(7), /* 179-RE_SETING (GP2_21, I) */
			BIT(7), /* 180-ENET_RSTn (GP2_22, O) (PHY reset) */
			BIT(7), /* 181-E_LINKSTS (GP2_23, I) */
			BIT(0), /* 182 */
			BIT(0), /* 183 */
			BIT(0), /* 184 */
			BIT(0), /* 185 */
			BIT(0), /* 186 */
			BIT(0), /* 187 */
			BIT(7), /* 188-CMOS_OE (GP2_24, O) */
			BIT(7), /* 189-CMOS_RST (GP2_25, O) */
			BIT(0), /* 190 */
			BIT(0), /* 191 */
			BIT(0), /* 192 */
			BIT(0), /* 193 */
			BIT(0), /* 194 */
			BIT(0), /* 195 */
			BIT(7), /* 196-RTC_INTn (GP2_26, I) */
			BIT(7), /* 197-HD_CLK (GP2_27, O) */
			BIT(0), /* 198 */
			BIT(0), /* 199 */
			BIT(0), /* 200 */
			BIT(0), /* 201 */
			BIT(0), /* 202 */
			BIT(0), /* 203 */
			BIT(0), /* 204 */
			BIT(7), /* 205-PI_POWER (GP2_29, O) */
			BIT(7), /* 206-CAMERA_POWER (GP2_30, O) */
			BIT(0), /* 207 */
			BIT(0), /* 208 */
			BIT(0), /* 209 */
			BIT(0), /* 210 */
			BIT(5), /* 211-CAMERA_TX (UART3_RXD) */
			BIT(5), /* 212-CAMERA_RX (UART3_TXD) */
			BIT(0), /* 213 */
			BIT(0), /* 214 */
			BIT(7), /* 215-ARM_IN (GP3_7, I) */
			BIT(7), /* 216-ARM_OUT (GP3_8, O) */
			BIT(0), /* 217 */
			BIT(0), /* 218 */
			BIT(0), /* 219 */
			BIT(7), /* 220-LED_G (GP3_12, O) */
			BIT(7), /* 221-LED_R (GP3_13, O) */
			BIT(0), /* 222 */
			BIT(0), /* 223 */
			BIT(0), /* 224 */
			BIT(0), /* 225 */
			BIT(0), /* 226 */
			BIT(0), /* 227 */
			BIT(0), /* 228 */
			BIT(0), /* 229 */
			BIT(0), /* 230 */
			BIT(0), /* 231 */
/* 
	232~258-GMII0 INTERFACE 
*/
			BIT(0), BIT(0), BIT(0), BIT(0), /* 232-235 */
			BIT(0), BIT(0), BIT(0), BIT(0), /* 236-239 */
			BIT(0), BIT(0), BIT(0), BIT(0), /* 240-243 */
			BIT(0), BIT(0), BIT(0), BIT(0), /* 244-247 */
			BIT(0), BIT(0), BIT(0), BIT(0), /* 248-251 */
			BIT(0), BIT(0), BIT(0), BIT(0), /* 252-255 */
			BIT(0), BIT(0), BIT(0), 		/* 256-258 */
			BIT(0), /* 259 */
			BIT(0), /* 260 */
			BIT(0), /* 261 */
			BIT(0), /* 262 */
			BIT(0), /* 263 */
			BIT(0), /* 264 */
			BIT(0), /* 265 */
			BIT(0), /* 266 */
			BIT(0), /* 267 */
			BIT(0), /* 268 */
			BIT(0), /* 269 */
			BIT(0), /* 270-usb0_drvvbus, fn1, pulldn enable  */
			BIT(0), /* 271 */