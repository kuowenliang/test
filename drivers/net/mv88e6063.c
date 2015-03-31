/*
 * (C) Copyright 2001
 * Gerald Van Baren, Custom IDEAS, vanbaren@cideas.com
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 * mv88e6063 driver
 */

#include <common.h>
#include <command.h>
#include <netdev.h>
#include <miiphy.h>
#include "mv88e60xx.h"

/* ---------------------------------------------------------------- */
#ifdef CONFIG_MV88E6063_SWITCH
#define MARVELL_88E6063_ID	0x01410c87	/* Marvell Switch 88E6063 */
#define MARVELL_88E6063_DEVID	0x1530	/* Device Identifier (Bit 15:4) */

//Port state===============================================================
#define	DSC_PORT_STATE_DISABLE		1
#define	DSC_PORT_STATE_BLOCKING		2
#define	DSC_PORT_STATE_LISTENING	3
#define	DSC_PORT_STATE_LEARNING		4
#define	DSC_PORT_STATE_FORWARDING	5
#define	DSC_PSTATE_DISABLE_AS_FORWARDING	0x0
#define	DSC_PSTATE_DISABLE_AS_BLOCK				0x01
#define	DSC_PSTATE_DISABLE_AS_DISABLE			0x04

#define REG_BASE		0x10
#define REG_PHY(p)		(REG_BASE + (p))
#define REG_PORT(p)		(REG_BASE + 8 + (p))
#define REG_GLOBAL		(REG_BASE + 0x0f)
#define NUM_PORTS		7
#define CPU_PORT		5

#define msleep(x)	udelay(x * 1000)
#define dsa_is_cpu_port(p) ((p == CPU_PORT) ? 1 : 0)

#define REG_READ(addr, reg)					\
	({							\
		int __ret, __val;					\
								\
		__ret = miiphy_read(devname, addr, reg, &__val);		\
		if (__ret != 0)					\
			return __ret;				\
		__val;						\
	})

#define REG_WRITE(addr, reg, val)				\
	({							\
		int __ret;					\
								\
		__ret = miiphy_write(devname, addr, reg, val);		\
		if (__ret != 0)					\
			return __ret;				\
	})

char *mv88e6063_probe(char *devname)
{
	int ret;
	unsigned char	addr, reg;
	unsigned short	data;

	addr = REG_PORT(0);
	reg = 0x03;
	data = 0xffff;
	if (miiphy_read (devname, addr, reg, &data) != 0) {
		printf("Error reading from the PHY addr=%02x reg=%02x\n", addr, reg);
	} else {
		data &= 0xfff0;
		if (data == MARVELL_88E6063_DEVID)
			return "Marvell 88E6063";
	}

	return NULL;
}

int mv88e6063_switch_reset(char *devname)
{
	int i;
	int ret;
	unsigned char	addr, reg;
	unsigned short	data;

	/*
	 * Set all ports to the disabled state.
	 */
	for (i = 0; i < NUM_PORTS; i++) {
		ret = REG_READ(REG_PORT(i), 0x04);
		REG_WRITE(REG_PORT(i), 0x04, ret & 0xfffc);
	}

	/*
	 * Wait for transmit queues to drain.
	 */
	msleep(2);

	/*
	 * Reset the switch.
	 */
	REG_WRITE(REG_GLOBAL, 0x0a, 0xa130);

	/*
	 * Wait up to one second for reset to complete.
	 */
	for (i = 0; i < 1000; i++) {
		ret = REG_READ(REG_GLOBAL, 0x00);
		if ((ret & 0x8000) == 0x0000)
			break;

		msleep(1);
	}
	if (i == 1000)
		return -1;

	return 0;
}

int mv88e6063_setup_global(char *devname)
{
	/*
	 * Disable discarding of frames with excessive collisions,
	 * set the maximum frame size to 1536 bytes, and mask all
	 * interrupt sources.
	 */
	REG_WRITE(REG_GLOBAL, 0x04, 0x0800);

	/*
	 * Enable automatic address learning, set the address
	 * database size to 1024 entries, and set the default aging
	 * time to 5 minutes.
	 */
	REG_WRITE(REG_GLOBAL, 0x0a, 0x2130);

	return 0;
}

int mv88e6063_setup_port(char *devname, int p)
{
	int addr = REG_PORT(p);
	int ret;

	/*
	 * Do not force flow control, disable Ingress and Egress
	 * Header tagging, disable VLAN tunneling, and set the port
	 * state to Forwarding.  Additionally, if this is the CPU
	 * port, enable Ingress and Egress Trailer tagging mode.
	 */
	ret = REG_READ(addr, 0x04);
	REG_WRITE(addr, 0x04, dsa_is_cpu_port(p) ?  (0x0007 | (ret & 0x00ff)) : 0x0003);

	/*
	 * Port based VLAN map: give each port its own address
	 * database, allow the CPU port to talk to each of the 'real'
	 * ports, and allow each of the 'real' ports to only talk to
	 * the CPU port.
	 */
	REG_WRITE(addr, 0x06, ((p & 0xf) << 12) | (~(1 << p) & 0x7f));

	/*
	 * Port Association Vector: when learning source addresses
	 * of packets, add the address to the address database using
	 * a port bitmap that has only the bit for this port set and
	 * the other bits clear.
	 */
	REG_WRITE(addr, 0x0b, 1 << p);

	return 0;
}

int mv88e6063_setup(char *devname)
{
	int i;
	int ret;

	ret = mv88e6063_switch_reset(devname);
	if (ret < 0)
		return ret;

	/* @@@ initialise atu */

	ret = mv88e6063_setup_global(devname);
	if (ret < 0)
		return ret;

	for (i = 0; i < NUM_PORTS; i++) {
		ret = mv88e6063_setup_port(devname, i);
		if (ret < 0)
			return ret;
	}

	return 0;
}

int mv88e6063_set_addr(char *devname)
{
	u_int8_t mac_addr[6];
	if (eth_getenv_enetaddr("ethaddr", mac_addr)) {
		if(is_valid_ether_addr(mac_addr)) {
			printf("Detected MACID:%02x:%02x:%02x:%02x:%02x:%02x\n",
				mac_addr[0], mac_addr[1], mac_addr[2],
				mac_addr[3], mac_addr[4], mac_addr[5]);
			REG_WRITE(REG_GLOBAL, 0x01, (mac_addr[0] << 8) | mac_addr[1]);
			REG_WRITE(REG_GLOBAL, 0x02, (mac_addr[2] << 8) | mac_addr[3]);
			REG_WRITE(REG_GLOBAL, 0x03, (mac_addr[4] << 8) | mac_addr[5]);
		} else {
			printf("Caution:using static MACID!! Set <ethaddr> variable\n");
		}
	}else{
		printf("Error: <ethaddr> not set.\n");
	}
	return 0;
}

void mv88e6063_readId(char *name)
{
	unsigned short val;
	miiphy_read(name, SMI_DEVICE_SWITCH_PORT, SMI_SWITCH_IDENTIFIER_REG, &val);
	printf("mv88e6063_readId 0x%x\n", val>>4);

}

void mv88e6063_init()
{
	unsigned short data;
	char		*name;

	/* use current device */
	name = miiphy_get_current_dev();

	//mv88e6063_readId(name);
	mv88e6063_setup(name);
}

#endif

