/*  Copyright (C) MOXA Inc. All rights reserved.

    This software is distributed under the terms of the
    MOXA License.  See the file COPYING-MOXA for details.

    mv88e60xx.h

    switch chip 6063 declear ................

    20030711	William chen
		new release
    20080709	Wensen Kuo
		Modify to VPort354...
*/


#ifndef MV88E60XX_H
#define MV88E60XX_H

#include <linux/types.h>

//===========================================================================================
// Register Map of Marvell 88E6063
#define	SMI_DEVICE_PHY						0x10  // 0x0 ~ 0x04, 5 Phys
#define	SMI_DEVICE_SWITCH_PORT		0x18  // 0x08 ~ 0x0E, 7 ports
#define	SMI_DEVICE_SWITCH_GLOBAL	0x1F

//===========================================================================================
//	PHY Registers (reg offset 00h-0x04h)
//------------------------------------------------------------------------------
//	Addr	    Bits    Reister Name
//------------------------------------------------------------------------------
//	00h	    	16	    PHY Control Register
//	01h		    16      PHY Status Register
//	02h     	16	    PHY Identifier Register 1
//	03h	    	16		PHY Identifier Register 2
//	04h     	16	    Auto-Neg Advertisement
//	05h		    16	    Link Partner Ability
//	06h	    	16	    Auto-Neg Expansion
//	07h		    16		Next Page Transmit
//	08h		    16	    Link Partner Next Page
//	10h		    16	    PHY Specific Control 1
//	11h		    16	    PHY Specific Status
//	12h		    16	    PHY Interrupt Status
//	13h		    16	    PHY Interrupt Enable
//	14h		    16	    Interrupt Port Summary
//	15h		    16	    Receive Error Counter
//	16h		    16	    LED Parallel Select
//	17h		    16	    LED Stream Select
//	18h		    16	    LED Control
//	19h		    16	    LED Override
//	1ah		    16	    VCT Control
//	1bh		    16	    VCT Status
//	1ch		    16	    PHY Specific Control 2

#define SMI_PHY_CONTROL_REG	    				0x00
#define SMI_PHY_STATUS_REG	    				0x01
#define SMI_PHY_IDENTIFIER1	    				0x02
#define SMI_PHY_IDENTIFIER2	    				0x03
#define SMI_AUTONEG_ADVERTISEMENT_REG		0x04
#define SMI_LINK_PARTNER_ABILITY_REG		0x05
#define SMI_AUTONEG_EXPANSION_REG		  	0x06
#define SMI_NEXT_PAGE_TRANSMIT_REG			0x07
#define SMI_LINKPARTNER_NEXT_PAGE_REG 	0x08
#define SMI_PHY_SPECIFIC_CONTROL_REG		0x10
#define SMI_PHY_SPECIFIC_STATUS_REG 		0x11
#define SMI_PHY_INTERRUPT_ENABLE		  	0x12
#define SMI_PHY_INTERRUPT_STATUS		  	0x13
#define SMI_INTERRUPT_PORT_SUMMARY_REG	0x14
#define SMI_RECEIVE_ERROR_COUNTER_REG		0x15
#define SMI_LED_PARALLEL_SELECT_REG			0x16
#define SMI_LED_STREAM_SELECT_REG		  	0x17
#define SMI_LED_CONTROL_REG				    	0x18
#define SMI_LED_OVERRIDE_REG			    	0x19
#define SMI_VCT_CONTROL_REG				    	0x1a
#define SMI_VCT_STATUS_REG				    	0x1b
#define SMI_PHY_SPECIFIC_CONTROL2_REG		0x1c
//For PHY Marvell 1112
#define SMI_PHY_1112_PAGE_ADDRESS			SMI_LED_PARALLEL_SELECT_REG
#define SMI_PHY_1112_FUNCTION_CONTROL	SMI_PHY_SPECIFIC_CONTROL_REG
#define SMI_PHY_1112_POLARITY_CONTROL	SMI_PHY_SPECIFIC_STATUS_REG

//===========================================================================================
//	Switch Port Register (reg offset 08h-0Dh)
//------------------------------------------------------------------------------
//	Addr	    Bits    Reister Name
//------------------------------------------------------------------------------
//	00h	    	16	    Port Status Register
//	03h	    	16			Switch Identifier Register
//	04h     	16	    Port Control Register
//	06h	    	16	    Port Based VLAN Register
//	07h	    	16	    Default Port VLAN ID & Priority Register
//	0ah	    	16	    Rate Control Register
//	0bh		    16	    Port Association Vector
//	10h		    16	    Rx Frame Counter
//	11h		    16	    Tx Frame Counter

#define SMI_PORT_STATUS_REG	    				0x00
#define SMI_SWITCH_IDENTIFIER_REG  			0x03
#define SMI_PORT_CONTROL_REG			 			0x04
#define SMI_PORT_BASED_VLAN_REG					0x06
#define SMI_PORT_DEFAULT_VLANID_REG			0x07
#define SMI_RATE_CONTROL_REG			 			0x0A
#define SMI_PORT_ASSOCIATION_VECTOR_REG 0x0B
#define SMI_RX_FRAME_COUNTER_REG		    0x10
#define SMI_TX_FRAME_COUNTER_REG	 			0x11

//===========================================================================================
//	Switch Global Register (reg offset 0Fh)
//------------------------------------------------------------------------------
//	Addr	    Bits    Reister Name
//------------------------------------------------------------------------------
//	00h	    	16	    Global Status Register
//	01h	    	16	    Switch MAC Register Bytes 0&1
//	02h	    	16			Switch MAC Register Bytes 2&3
//	03h     	16	    Switch MAC Register Bytes 4&5
//	04h	    	16	    Global Control Register
//
//	05h	    	16	    VTU Operation Register
//	06h	    	16	    VTU VID Register
//	07h	    	16	    VTU Data Ports 3:0 Register
//	08h	    	16	    VTU Data Ports 7:4 Register
//	09h	    	16	    VTU Data Ports 9:8 Register
//
//	0ah	    	16	    ATU Control Register
//	0bh		    16	    ATU Operation Register
//	0ch		    16	    ATU Data Register
//	0dh	    	16	    ATU MAC Register Bytes 0&1
//	0eh		    16	    ATU MAC Register Bytes 2&3
//	0fh		    16	    ATU MAC Register Bytes 4&5
//
//	10h	    	16	    IP-PRI Register  10h ~ 17h
//	18h	    	16	    IEEE-PRI  Register
//
//	1Dh	    	16	    Stat Operation Register
//	1Eh	    	16	    Stat Data Bytes 3:2 Register
//	1Fh	    	16	    Stat Data Bytes 1:0 Register

#define SMI_GLOBAL_STATUS_REG	  	0x00
#define SMI_SWITCH_MAC01_REG  		0x01
#define SMI_SWITCH_MAC23_REG  		0x02
#define SMI_SWITCH_MAC45_REG  		0x03
#define SMI_GLOBAL_CONTROL_REG		0x04

#define SMI_VTU_OPERATION_REG  		0x05
#define SMI_VTU_VID_REG		  			0x06
#define SMI_VTU_DATA_PORTS30_REG	0x07
#define SMI_VTU_DATA_PORTS74_REG	0x08
#define SMI_VTU_DATA_PORTS98_REG	0x09

#define SMI_ATU_CONTROL_REG				0x0a
#define SMI_ATU_OPERATION_REG			0x0b
#define SMI_ATU_DATA_REG 					0x0c
#define SMI_ATU_MAC01_REG  				0x0d
#define SMI_ATU_MAC23_REG 	 			0x0e
#define SMI_ATU_MAC45_REG  				0x0f

#define SMI_IP_PRI_MAP0_REG  			0x10
#define SMI_IP_PRI_MAP1_REG  			0x11
#define SMI_IP_PRI_MAP2_REG  			0x12
#define SMI_IP_PRI_MAP3_REG  			0x13
#define SMI_IP_PRI_MAP4_REG  			0x14
#define SMI_IP_PRI_MAP5_REG  			0x15
#define SMI_IP_PRI_MAP6_REG  			0x16
#define SMI_IP_PRI_MAP7_REG  			0x17
#define SMI_IEEE_PRI_MAP_REG  		0x18

#define SMI_STATS_OPERATION_REG 	0x1D
#define SMI_STATS_DATA32_REG  		0x1E
#define SMI_STATS_DATA10_REG  		0x1F

//===========================================================================================
// ATU Operation Register
//
#define	SMI_ATUBusy											0x8000
#define	SMI_ATUOp_FlushAll							(0x1000 | SMI_ATUBusy)
#define	SMI_ATUOp_Load									(0x3000 | SMI_ATUBusy)
#define	SMI_ATUOp_GetNext								(0x4000 | SMI_ATUBusy)
#define	SMI_ATUOp_FlushAllInOneDbnum		(0x5000 | SMI_ATUBusy)
#define	SMI_ATUOp_FlushUnlockInOneDbnum	(0x6000 | SMI_ATUBusy)
#define	SMI_ATUOp_FlushAllUnlock				(0x2000 | SMI_ATUBusy)

#define	SMI_ATUData_EntryState					0x000F
#define	SMI_ATUData_PortVector_SHIFT		4
#define	SMI_ATUData_PortVector					0x3FF0
#define	SMI_ATUData_EntryPri						0xC000
#define	SMI_ATUData_EntryPri_SHIFT			14

//===========================================================================================
// Statistics Operation Register
//
#define	SMI_StatsOP_Busy							0x8000
#define	SMI_StatsOP_FlushAll					(0x1000 | SMI_StatsOP_Busy)
#define	SMI_StatsOP_FlushAllPerPort		(0x2000 | SMI_StatsOP_Busy)
#define	SMI_StatsOP_ReadCaptured			(0x4000 | SMI_StatsOP_Busy)
#define	SMI_StatsOP_CaptureAllPerPort	(0x5000 | SMI_StatsOP_Busy)


#define	IN_UNICASTS					0x00
#define	IN_BROADCASTS				0x01
#define	IN_PAUSE						0x02
#define	IN_MULTICASTS				0x03
#define	IN_FCS_ERR					0x04
#define	IN_ALIGN_ERR				0x05

#define	IN_GOOD_OCTETS			0x06
#define	IN_BAD_OCTETS				0x07
#define	IN_UNDERSIZE				0x08
#define	IN_FRAGMENTS				0x09
#define	IN_64OCTETS					0x0a
#define	IN_127OCTETS				0x0b
#define	IN_255OCTETS				0x0c
#define	IN_511OCTETS				0x0d
#define	IN_1023OCTETS				0x0e
#define	IN_MAX_OCTETS				0x0f
#define	IN_JABBER						0x10
#define	IN_OVERSIZE					0x11

#define	IN_DISCARDS					0x12
#define	IN_FILTERED					0x13

#define	OUT_UNICASTS				0x14
#define	OUT_BROADCASTS			0x15
#define	OUT_PAUSE						0x16
#define	OUT_MULTICASTS			0x17
#define	OUT_FCS_ERR					0x18

#define	OUT_OCTETS					0x19

#define	OUT_64OCTETS				0x1a
#define	OUT_127OCTETS				0x1b
#define	OUT_255OCTETS				0x1c
#define	OUT_511OCTETS				0x1d
#define	OUT_1023OCTETS			0x1e
#define	OUT_MAX_OCTETS			0x1f

#define	OUT_COLLISIONS						0x20
#define	OUT_LATE_COLLISIONS				0x21
#define	OUT_EXCESSIVE_COLLISIONS	0x22
#define	OUT_MULTIPLE_COLLISIONS		0x23
#define	OUT_SINGLE_COLLISIONS			0x24
#define	OUT_DEFFERED_COLLISIONS		0x25
#define	OUT_DISCARDS							0x26


//===========================================================================================
//Global Functions of SMI control.

//int	Ssmi_rReg(int device_address, int reg, u_short *data);
//int	Ssmi_wReg(int device_address, int reg, u_short data);
//int	Ssmi_updateReg(int dev, int reg, u_short maskbits,u_short updatedbits);

#endif //MV88E60XX_H
