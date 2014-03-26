/*
 * (C) Copyright 2000-2002
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * (C) Copyright 2001 Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Andreas Heppel <aheppel@sysgo.de>

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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <command.h>
#include <environment.h>
#include <linux/stddef.h>
#include <malloc.h>

DECLARE_GLOBAL_DATA_PTR;

#ifdef CONFIG_AMIGAONEG3SE
	extern void enable_nvram(void);
	extern void disable_nvram(void);
#endif

#undef DEBUG_ENV
#ifdef DEBUG_ENV
#define DEBUGF(fmt,args...) printf(fmt ,##args)
#else
#define DEBUGF(fmt,args...)
#endif

extern env_t *env_ptr;

extern void env_relocate_spec (void);
extern uchar env_get_char_spec(int);

static uchar env_get_char_init (int index);

/************************************************************************
 * Default settings to be used when no valid environment is found
 */
//#define XMK_STR(x)	#x
//#define MK_STR(x)	XMK_STR(x)

unsigned char default_environment[] = {
#ifdef	CONFIG_NETRETRY
	"netretry="	MK_STR(CONFIG_NETRETRY)		"\0"
#endif
#ifdef	CFG_NAND_QUIET
	"quiet=" MK_STR(CFG_NAND_QUIET) 		"\0"
#endif
#ifdef	CONFIG_BOOTARGS
	"bootargs="	CONFIG_BOOTARGS			"\0"
#endif
#ifdef	CONFIG_BOOTCOMMAND
	"bootcmd="	CONFIG_BOOTCOMMAND		"\0"
#endif
#ifdef	CONFIG_MPBOOTCOMMAND
	"mpbootcmd="	CONFIG_MPBOOTCOMMAND		"\0"
#endif
#ifdef	CONFIG_RAMBOOTCOMMAND
	"ramboot="	CONFIG_RAMBOOTCOMMAND		"\0"
#endif
#ifdef	CONFIG_NFSBOOTCOMMAND
	"nfsboot="	CONFIG_NFSBOOTCOMMAND		"\0"
#endif
#if defined(CONFIG_BOOTDELAY) && (CONFIG_BOOTDELAY >= 0)
	"bootdelay="	MK_STR(CONFIG_BOOTDELAY)	"\0"
#endif
#if defined(CONFIG_BAUDRATE) && (CONFIG_BAUDRATE >= 0)
	"baudrate="	MK_STR(CONFIG_BAUDRATE)		"\0"
#endif
#if defined(CONFIG_SERIAL_MULTI)
#if defined(CONFIG_BAUDRATE_0_ITEM) && defined(CONFIG_BAUDRATE_0) && (CONFIG_BAUDRATE_0 >= 0)
	CONFIG_BAUDRATE_0_ITEM"="	MK_STR(CONFIG_BAUDRATE_0)		"\0"
#endif
#if defined(CONFIG_BAUDRATE_1_ITEM) && defined(CONFIG_BAUDRATE_1) && (CONFIG_BAUDRATE_1 >= 0)
	CONFIG_BAUDRATE_1_ITEM"="	MK_STR(CONFIG_BAUDRATE_1)		"\0"
#endif
#if defined(CONFIG_BAUDRATE_2_ITEM) && defined(CONFIG_BAUDRATE_2) && (CONFIG_BAUDRATE_2 >= 0)
	CONFIG_BAUDRATE_2_ITEM"="	MK_STR(CONFIG_BAUDRATE_2)		"\0"
#endif
#if defined(CONFIG_BAUDRATE_3_ITEM) && defined(CONFIG_BAUDRATE_3) && (CONFIG_BAUDRATE_3 >= 0)
	CONFIG_BAUDRATE_3_ITEM"="	MK_STR(CONFIG_BAUDRATE_3)		"\0"
#endif
#endif
#ifdef	CONFIG_LOADS_ECHO
	"loads_echo="	MK_STR(CONFIG_LOADS_ECHO)	"\0"
#endif
#ifdef	CONFIG_BOOTARGS_MEM
	"mem="	MK_STR(CONFIG_BOOTARGS_MEM)		"\0"
#endif
#ifdef	CONFIG_ETHADDR
	"ethaddr="	MK_STR(CONFIG_ETHADDR)		"\0"
#endif
#ifdef	CONFIG_ETH1ADDR
	"eth1addr="	MK_STR(CONFIG_ETH1ADDR)		"\0"
#endif
#ifdef	CONFIG_ETH2ADDR
	"eth2addr="	MK_STR(CONFIG_ETH2ADDR)		"\0"
#endif
#ifdef	CONFIG_ETH3ADDR
	"eth3addr="	MK_STR(CONFIG_ETH3ADDR)		"\0"
#endif
#ifdef	CONFIG_ETH4ADDR
	"eth4addr="	MK_STR(CONFIG_ETH4ADDR)		"\0"
#endif
#ifdef	CONFIG_ETH5ADDR
	"eth5addr="	MK_STR(CONFIG_ETH5ADDR)		"\0"
#endif
#ifdef	CONFIG_IPADDR
	"ipaddr="	MK_STR(CONFIG_IPADDR)		"\0"
#endif
#ifdef	CONFIG_SERVERIP
	"serverip="	MK_STR(CONFIG_SERVERIP)		"\0"
#endif
#ifdef	CONFIG_SYS_AUTOLOAD
	"autoload="	CONFIG_SYS_AUTOLOAD			"\0"
#endif
#ifdef	CONFIG_PREBOOT
	"preboot="	CONFIG_PREBOOT			"\0"
#endif
#ifdef	CONFIG_ROOTPATH
	"rootpath="	MK_STR(CONFIG_ROOTPATH)		"\0"
#endif
#ifdef	CONFIG_GATEWAYIP
	"gatewayip="	MK_STR(CONFIG_GATEWAYIP)	"\0"
#endif
#ifdef	CONFIG_DNSIP
	"dnsip="	MK_STR(CONFIG_DNSIP)	"\0"
#endif
#ifdef	CONFIG_NTPSERVERIP
	"ntpserverip="	MK_STR(CONFIG_NTPSERVERIP)	"\0"
#endif
#ifdef	CONFIG_NETMASK
	"netmask="	MK_STR(CONFIG_NETMASK)		"\0"
#endif
#ifdef	CONFIG_HOSTNAME
	"hostname="	MK_STR(CONFIG_HOSTNAME)		"\0"
#endif
#ifdef	CONFIG_BOOTFILE
	"bootfile="	MK_STR(CONFIG_BOOTFILE)		"\0"
#endif
#ifdef	CONFIG_LOADADDR
	"loadaddr="	MK_STR(CONFIG_LOADADDR)		"\0"
#endif
#ifdef	KERNEL_FLASH
	"kernelflash="	MK_STR(KERNEL_FLASH)		"\0"
#endif
#ifdef	MPKERNEL_FLASH
	"mpkernelflash="	MK_STR(MPKERNEL_FLASH)		"\0"
#endif
#ifdef  CONFIG_CLOCKS_IN_MHZ
	"clocks_in_mhz=1\0"
#endif
#if defined(CONFIG_PCI_BOOTDELAY) && (CONFIG_PCI_BOOTDELAY > 0)
	"pcidelay="	MK_STR(CONFIG_PCI_BOOTDELAY)	"\0"
#endif
#ifdef CONFIG_DUT_MODEL
	"dut_model=" CONFIG_DUT_MODEL 		"\0"
#endif
#ifdef CONFIG_DUT_HOST
	"dut_host=" CONFIG_DUT_HOST		"\0"
#endif
#ifdef CONFIG_DUT_DESC
	"dut_desc=" CONFIG_DUT_DESC		"\0"
#endif
#ifdef CONFIG_DUT_ID
	"dut_id=" MK_STR(CONFIG_DUT_ID)		"\0"
#endif
#ifdef CONFIG_HW_VER
	"hw_ver=" MK_STR(CONFIG_HW_VER)		"\0"
#endif
#ifdef CONFIG_FW_VER
	"fw_ver=" MK_STR(CONFIG_FW_VER)		"\0"
#endif
#ifdef CONFIG_MP_VER
	"mp_ver=" MK_STR(CONFIG_MP_VER)		"\0"
#endif
#ifdef CONFIG_UBL_VER
	"ubl_ver=" MK_STR(CONFIG_UBL_VER)		"\0"
#endif
#ifdef CONFIG_UBOOT_VER
	"uboot_ver=" MK_STR(CONFIG_UBOOT_VER)		"\0"
#endif
#ifdef CONFIG_UBOOT_IDX
	"uboot_idx=" MK_STR(CONFIG_UBOOT_IDX)		"\0"
#endif
#ifdef CONFIG_KERNEL_IDX
	"kernel_idx=" MK_STR(CONFIG_KERNEL_IDX)		"\0"
#endif
#ifdef CONFIG_MTD_IDX
	"mtd_idx=" MK_STR(CONFIG_MTD_IDX)		"\0"
#endif
#ifdef CONFIG_FW_BOOTUP_COUNTER
	"fwboot_ctr=" MK_STR(CONFIG_FW_BOOTUP_COUNTER)		"\0"
#endif
#ifdef CONFIG_MP_FLAG
	"mp_flag=" MK_STR(CONFIG_MP_FLAG)		"\0"
#endif
#ifdef CONFIG_IVA_FLAG
	"iva_flag=" MK_STR(CONFIG_IVA_FLAG)		"\0"
#endif
#ifdef CONFIG_BURNIN_TIME
	"burntime=" MK_STR(CONFIG_BURNIN_TIME)		"\0"
#endif
#ifdef CONFIG_FW_RECOVERY
	"fw_rcvr=" MK_STR(CONFIG_FW_RECOVERY)		"\0"
#endif
#if !(defined(CONFIG_FW_RECOVERY_FIX) && defined(CONFIG_FW_RECOVERY_FILE))
	"fw_rcvrfile=" CONFIG_FW_RECOVERY_FILE		"\0"
#endif
#ifdef CONFIG_T2_BOOTUP_COUNTER
	"boot_ctr=" MK_STR(CONFIG_T2_BOOTUP_COUNTER)		"\0"
#endif
#ifdef CONFIG_SERIAL_NUMBER
	"ser_num=" MK_STR(CONFIG_SERIAL_NUMBER)		"\0"
#endif
#ifdef CONFIG_PI_ON_TEMP
	"pi_on=" MK_STR(CONFIG_PI_ON_TEMP)		"\0"
#endif
#ifdef CONFIG_PI_OFF_TEMP
	"pi_off=" MK_STR(CONFIG_PI_OFF_TEMP)		"\0"
#endif
#ifdef CONFIG_SYS_ON_TEMP
	"sys_on=" MK_STR(CONFIG_SYS_ON_TEMP)		"\0"
#endif
#ifdef CONFIG_PI_OFF_OFFS
	"pi_off_offs=" MK_STR(CONFIG_PI_OFF_OFFS)		"\0"
#endif
#ifdef CONFIG_SYS_ON_OFFS
	"sys_on_offs=" MK_STR(CONFIG_SYS_ON_OFFS)		"\0"
#endif
#ifdef CONFIG_SYS_ON_TIMEOUT
	"sys_on_time=" MK_STR(CONFIG_SYS_ON_TIMEOUT)		"\0"
#endif
#ifdef  CONFIG_EXTRA_ENV_SETTINGS
	CONFIG_EXTRA_ENV_SETTINGS
#endif
	"\0"
};

void env_crc_update (void)
{
	env_ptr->crc = crc32(0, env_ptr->data, ENV_SIZE);
}

static uchar env_get_char_init (int index)
{
	uchar c;

	/* if crc was bad, use the default environment */
	if (gd->env_valid)
	{
		c = env_get_char_spec(index);
	} else {
		c = default_environment[index];
	}

	return (c);
}

#ifdef CONFIG_AMIGAONEG3SE
uchar env_get_char_memory (int index)
{
	uchar retval;
	enable_nvram();
	if (gd->env_valid) {
		retval = ( *((uchar *)(gd->env_addr + index)) );
	} else {
		retval = ( default_environment[index] );
	}
	disable_nvram();
	return retval;
}
#else
uchar env_get_char_memory (int index)
{
	if (gd->env_valid) {
		return ( *((uchar *)(gd->env_addr + index)) );
	} else {
		return ( default_environment[index] );
	}
}
#endif

uchar env_get_char (int index)
{
	uchar c;

	/* if relocated to RAM */
	if (gd->flags & GD_FLG_RELOC)
		c = env_get_char_memory(index);
	else
		c = env_get_char_init(index);

	return (c);
}

uchar *env_get_addr (int index)
{
	if (gd->env_valid) {
		return ( ((uchar *)(gd->env_addr + index)) );
	} else {
		return (&default_environment[index]);
	}
}

void set_default_env(void)
{
	if (sizeof(default_environment) > ENV_SIZE) {
		puts ("*** Error - default environment is too large\n\n");
		return;
	}

	memset(env_ptr, 0, sizeof(env_t));
	memcpy(env_ptr->data, default_environment,
	       sizeof(default_environment));
#ifdef CONFIG_SYS_REDUNDAND_ENVIRONMENT
	env_ptr->flags = 0xFF;
#endif
	env_crc_update ();
	gd->env_valid = 1;
}

void env_relocate (void)
{
#ifndef CONFIG_RELOC_FIXUP_WORKS
	DEBUGF ("%s[%d] offset = 0x%lx\n", __FUNCTION__,__LINE__,
		gd->reloc_off);
#endif

#ifdef CONFIG_AMIGAONEG3SE
	enable_nvram();
#endif

#ifdef ENV_IS_EMBEDDED
	/*
	 * The environment buffer is embedded with the text segment,
	 * just relocate the environment pointer
	 */
#ifndef CONFIG_RELOC_FIXUP_WORKS
	env_ptr = (env_t *)((ulong)env_ptr + gd->reloc_off);
#endif
	DEBUGF ("%s[%d] embedded ENV at %p\n", __FUNCTION__,__LINE__,env_ptr);
#else
	/*
	 * We must allocate a buffer for the environment
	 */
	env_ptr = (env_t *)malloc (CONFIG_ENV_SIZE);
	DEBUGF ("%s[%d] malloced ENV at %p\n", __FUNCTION__,__LINE__,env_ptr);
#endif

	if (gd->env_valid == 0) {
#if defined(CONFIG_GTH)	|| defined(CONFIG_ENV_IS_NOWHERE)	/* Environment not changable */
		puts ("Using default environment\n\n");
#else
		puts ("*** Warning - bad CRC, using default environment\n\n");
		show_boot_progress (-60);
#endif
		set_default_env();
	}
	else {
		env_relocate_spec ();
	}
	gd->env_addr = (ulong)&(env_ptr->data);

#ifdef CONFIG_AMIGAONEG3SE
	disable_nvram();
#endif
}

#ifdef CONFIG_AUTO_COMPLETE
int env_complete(char *var, int maxv, char *cmdv[], int bufsz, char *buf)
{
	int i, nxt, len, vallen, found;
	const char *lval, *rval;

	found = 0;
	cmdv[0] = NULL;

	len = strlen(var);
	/* now iterate over the variables and select those that match */
	for (i=0; env_get_char(i) != '\0'; i=nxt+1) {

		for (nxt=i; env_get_char(nxt) != '\0'; ++nxt)
			;

		lval = (char *)env_get_addr(i);
		rval = strchr(lval, '=');
		if (rval != NULL) {
			vallen = rval - lval;
			rval++;
		} else
			vallen = strlen(lval);

		if (len > 0 && (vallen < len || memcmp(lval, var, len) != 0))
			continue;

		if (found >= maxv - 2 || bufsz < vallen + 1) {
			cmdv[found++] = "...";
			break;
		}
		cmdv[found++] = buf;
		memcpy(buf, lval, vallen); buf += vallen; bufsz -= vallen;
		*buf++ = '\0'; bufsz--;
	}

	cmdv[found] = NULL;
	return found;
}
#endif

#ifdef CONFIG_ETHADDR
int env_get_ethaddr(char **mac)
{
	if((*mac = getenv("ethaddr")) == NULL){
		if(env_set_ethaddr(MK_STR(CONFIG_ETHADDR))) return -1;
		*mac = MK_STR(CONFIG_ETHADDR);
	}
	return 0;
}
int env_set_ethaddr(char *mac)
{
	if(setenv("ethaddr", mac)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_IPADDR
int env_get_ipaddr(char **ip)
{
	if((*ip = getenv("ipaddr")) == NULL){
		if(env_set_ipaddr(MK_STR(CONFIG_IPADDR))) return -1;
		*ip = MK_STR(CONFIG_IPADDR);
	}
	return 0;
}
int env_set_ipaddr(char *ip)
{
	if(setenv("ipaddr", ip)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_SERVERIP
int env_get_serverip(char **ip)
{
	if((*ip = getenv("serverip")) == NULL){
		if(env_set_serverip(MK_STR(CONFIG_SERVERIP))) return -1;
		*ip = MK_STR(CONFIG_SERVERIP);
	}
	return 0;
}
int env_set_serverip(char *ip)
{
	if(setenv("serverip", ip)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_GATEWAYIP
int env_get_gatewayip(char **ip)
{
	if((*ip = getenv("gatewayip")) == NULL){
		if(env_set_gatewayip(MK_STR(CONFIG_GATEWAYIP))) return -1;
		*ip = MK_STR(CONFIG_GATEWAYIP);
	}
	return 0;
}
int env_set_gatewayip(char *ip)
{
	if(setenv("gatewayip", ip)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_DNSIP
int env_get_dnsip(char **ip)
{
	if((*ip = getenv("dnsip")) == NULL){
		if(env_set_dnsip(MK_STR(CONFIG_DNSIP))) return -1;
		*ip = MK_STR(CONFIG_DNSIP);
	}
	return 0;
}
int env_set_dnsip(char *ip)
{
	if(setenv("dnsip", ip)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_NETMASK
int env_get_netmask(char **ip)
{
	if((*ip = getenv("netmask")) == NULL){
		if(env_set_netmask(MK_STR(CONFIG_NETMASK))) return -1;
		*ip = MK_STR(CONFIG_NETMASK);
	}
	return 0;
}
int env_set_netmask(char *ip)
{
	if(setenv("netmask", ip)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_BAUDRATE
int env_get_baudrate(unsigned int *rate)
{
	char* tmp = NULL;
	if((tmp = getenv("baudrate")) == NULL){
		if(env_set_baudrate(CONFIG_BAUDRATE)) return -1;
		*rate = CONFIG_BAUDRATE;
	}else{
		*rate = (int)simple_strtoul(tmp, NULL, 10);
	}
	return 0;
}
int env_set_baudrate(unsigned int rate)
{
	char tmp[5];
	sprintf(tmp, "%u", rate);
	if(setenv("baudrate", tmp)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_MP_FLAG
int env_get_mp_flag(int *flag)
{
	char* tmp = NULL;
	if((tmp = getenv("mp_flag")) == NULL){
		if(env_set_mp_flag(CONFIG_MP_FLAG)) return -1;
		*flag = CONFIG_MP_FLAG;
	}else{
		*flag = (int)simple_strtoul(tmp, NULL, 16);
	}
	return 0;
}
int env_set_mp_flag(int flag)
{
	char tmp[5];
	sprintf(tmp, "0x%03x", flag);
	if(setenv("mp_flag", tmp)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_IVA_FLAG
int env_get_iva_flag(int *flag)
{
	char* tmp = NULL;
	if((tmp = getenv("iva_flag")) == NULL){
		if(env_set_iva_flag(CONFIG_IVA_FLAG)) return -1;
		*flag = CONFIG_IVA_FLAG;
	}else{
		*flag = (int)simple_strtoul(tmp, NULL, 16);
	}
	return 0;
}
int env_set_iva_flag(int flag)
{
	char tmp[5];
	sprintf(tmp, "0x%02x", flag);
	if(setenv("iva_flag", tmp)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_BURNIN_TIME
int env_get_burnintime(unsigned int *time)
{
	char* tmp = NULL;
	if((tmp = getenv("burntime")) == NULL){
		if(env_set_burnintime(CONFIG_BURNIN_TIME)) return -1;
		*time = CONFIG_BURNIN_TIME;
	}else{
		*time = (unsigned int)simple_strtoul(tmp, NULL, 10);
	}
	return 0;
}
int env_set_burnintime(unsigned int time)
{
	char tmp[5];
	sprintf(tmp, "%u", time);
	if(setenv("burntime", tmp)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_T2_BOOTUP_COUNTER
int env_get_t2_bootup_counter(unsigned int *counter)
{
	char* tmp = NULL;
	if((tmp = getenv("boot_ctr")) == NULL){
		if(env_set_t2_bootup_counter(CONFIG_T2_BOOTUP_COUNTER)) return -1;
		*counter = CONFIG_T2_BOOTUP_COUNTER;
	}else{
		*counter = (unsigned int)simple_strtoul(tmp, NULL, 10);
	}
	return 0;
}
int env_set_t2_bootup_counter(unsigned int counter)
{
	char tmp[16];
	sprintf(tmp, "%u", counter);
	if(setenv("boot_ctr", tmp)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_SERIAL_NUMBER
int env_get_serial_number(char **number)
{
	if((*number = getenv("ser_num")) == NULL){
		if(env_set_serial_number(MK_STR(CONFIG_SERIAL_NUMBER))) return -1;
		*number = MK_STR(CONFIG_SERIAL_NUMBER);
	}
	return 0;
}
int env_set_serial_number(char *number)
{
	if(setenv("ser_num", number)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_HW_VER
int env_get_hw_version(char **verion)
{
	if((*verion = getenv("hw_ver")) == NULL){
		if(env_set_hw_version(MK_STR(CONFIG_HW_VER))) return -1;
		*verion = MK_STR(CONFIG_HW_VER);
	}
	return 0;
}
int env_set_hw_version(char *verion)
{
	if(setenv("hw_ver", verion)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_FW_VER
int env_get_fw_version(char **verion)
{
	if((*verion = getenv("fw_ver")) == NULL){
		if(env_set_fw_version(MK_STR(CONFIG_FW_VER))) return -1;
		*verion = MK_STR(CONFIG_FW_VER);
	}
	return 0;
}
int env_set_fw_version(char *verion)
{
	if(setenv("fw_ver", verion)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_MP_VER
int env_get_mp_version(char **verion)
{
	if((*verion = getenv("mp_ver")) == NULL){
		if(env_set_mp_version(MK_STR(CONFIG_MP_VER))) return -1;
		*verion = MK_STR(CONFIG_MP_VER);
	}
	return 0;
}
int env_set_mp_version(char *verion)
{
	if(setenv("mp_ver", verion)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_UBL_VER
int env_get_ubl_version(char **verion)
{
	if((*verion = getenv("bios_ver")) == NULL){
		if(env_set_ubl_version(MK_STR(CONFIG_UBL_VER))) return -1;
		*verion = MK_STR(CONFIG_UBL_VER);
	}
	return 0;
}
int env_set_ubl_version(char *verion)
{
	if(setenv("bios_ver", verion)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_UBOOT_VER
int env_get_uboot_version(char **verion)
{
	if((*verion = getenv("uboot_ver")) == NULL){
		if(env_set_uboot_version(MK_STR(CONFIG_UBOOT_VER))) return -1;
		*verion = MK_STR(CONFIG_UBOOT_VER);
	}
	return 0;
}
int env_set_uboot_version(char *verion)
{
	if(setenv("uboot_ver", verion)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_UBOOT_IDX
int env_get_uboot_idx(int *idx)
{
	char* tmp = NULL;
	if((tmp = getenv("uboot_idx")) == NULL){
		if(env_set_uboot_idx(CONFIG_UBOOT_IDX)) return -1;
		*idx = CONFIG_UBOOT_IDX;
	}else{
		*idx = (int)simple_strtoul(tmp, NULL, 10);
	}
	return 0;
}
int env_set_uboot_idx(int idx)
{
	char tmp[5];
	sprintf(tmp, "%d", idx);
	if(setenv("uboot_idx", tmp)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_KERNEL_IDX
int env_get_kernel_idx(int *idx)
{
	char* tmp = NULL;
	if((tmp = getenv("kernel_idx")) == NULL){
		if(env_set_kernel_idx(CONFIG_KERNEL_IDX)) return -1;
		*idx = CONFIG_KERNEL_IDX;
	}else{
		*idx = (int)simple_strtoul(tmp, NULL, 10);
	}
	return 0;
}
int env_set_kernel_idx(int idx)
{
	if(idx == CONFIG_KERNEL_IDX){
		if(setenv("kernel_idx", MK_STR(CONFIG_KERNEL_IDX))) return -1;
		if(setenv("kernelflash", MK_STR(KERNEL_FLASH))) return -1;
		if(setenv("mtd_idx", MK_STR(CONFIG_MTD_IDX))) return -1;
	}else 
	if(idx == CONFIG_KERNEL2_IDX){
		if(setenv("kernel_idx", MK_STR(CONFIG_KERNEL2_IDX))) return -1;
		if(setenv("kernelflash", MK_STR(KERNEL2_FLASH))) return -1;
		if(setenv("mtd_idx", MK_STR(CONFIG_MTD2_IDX))) return -1;
	}
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_MTD_IDX
int env_get_mtd_idx(int *idx)
{
	char* tmp = NULL;
	if((tmp = getenv("mtd_idx")) == NULL){
		if(env_set_mtd_idx(CONFIG_MTD_IDX)) return -1;
		*idx = CONFIG_MTD_IDX;
	}else{
		*idx = (int)simple_strtoul(tmp, NULL, 10);
	}
	return 0;
}
int env_set_mtd_idx(int idx)
{
	char tmp[5];
	sprintf(tmp, "%d", idx);
	if(setenv("mtd_idx", tmp)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef KERNEL_FLASH
int env_get_kernelflash(unsigned int *addr)
{
	char* tmp = NULL;
	if((tmp = getenv("kernelflash")) == NULL){
		if(env_set_kernelflash(KERNEL_FLASH)) return -1;
		*addr = KERNEL_FLASH;
	}else{
		*addr = (unsigned int)simple_strtoul(tmp, NULL, 16);
	}
	return 0;
}
int env_set_kernelflash(unsigned int addr)
{
	char tmp[16];
	sprintf(tmp, "0x%08X", addr);
	if(setenv("kernelflash", tmp)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_FW_BOOTUP_COUNTER
int env_get_fw_bootup_counter(unsigned int *counter)
{
	char* tmp = NULL;
	if((tmp = getenv("fwboot_ctr")) == NULL){
		if(env_set_fw_bootup_counter(CONFIG_FW_BOOTUP_COUNTER)) return -1;
		*counter = CONFIG_FW_BOOTUP_COUNTER;
	}else{
		*counter = (unsigned int)simple_strtoul(tmp, NULL, 10);
	}
	return 0;
}
int env_set_fw_bootup_counter(unsigned int counter)
{
	char tmp[16];
	sprintf(tmp, "%u", counter);
	if(setenv("fwboot_ctr", tmp)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_DUT_ID
int env_get_dut_id(char **id)
{
	if((*id = getenv("dut_id")) == NULL){
		if(env_set_dut_id(MK_STR(CONFIG_DUT_ID))) return -1;
		*id = MK_STR(CONFIG_DUT_ID);
	}
	return 0;
}
int env_set_dut_id(char *id)
{
	if(setenv("dut_id", id)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_FW_RECOVERY
int env_get_fw_recovery(unsigned int *rcv)
{
	char* tmp = NULL;
	if((tmp = getenv("fw_rcvr")) == NULL){
		if(env_set_fw_recovery(CONFIG_FW_RECOVERY)) return -1;
		*rcv = CONFIG_FW_RECOVERY;
	}else{
		*rcv = (unsigned int)simple_strtoul(tmp, NULL, 10);
	}
	return 0;
}
int env_set_fw_recovery(unsigned int rcv)
{
	char tmp[2];
	sprintf(tmp, "%u", rcv);
	if(setenv("fw_rcvr", tmp)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#if !(defined(CONFIG_FW_RECOVERY_FIX) && defined(CONFIG_FW_RECOVERY_FILE))
int env_get_fw_recovery_file(char **file)
{
	if((*file = getenv("fw_rcvrfile")) == NULL){
		if(env_set_fw_recovery_file(MK_STR(CONFIG_FW_RECOVERY_FILE))) return -1;
		*file = MK_STR(CONFIG_FW_RECOVERY_FILE);
	}
	return 0;
}
int env_set_fw_recovery_file(char *file)
{
	if(setenv("fw_rcvrfile", file)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_DUT_MODEL
int env_get_model(char **model)
{
	if((*model = getenv("dut_model")) == NULL){
		if(env_set_model(MK_STR(CONFIG_DUT_MODEL))) return -1;
		*model = MK_STR(CONFIG_DUT_MODEL);
	}
	return 0;
}
int env_set_model(char *model)
{
	if(setenv("dut_model", model)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_DUT_HOST
int env_get_host(char **host)
{
	if((*host = getenv("dut_host")) == NULL){
		if(env_set_host(MK_STR(CONFIG_DUT_HOST))) return -1;
		*host = MK_STR(CONFIG_DUT_HOST);
	}
	return 0;
}
int env_set_host(char *host)
{
	if(setenv("dut_host", host)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_DUT_DESC
int env_get_desc(char **desc)
{
	if((*desc = getenv("dut_desc")) == NULL){
		if(env_set_desc(MK_STR(CONFIG_DUT_DESC))) return -1;
		*desc = MK_STR(CONFIG_DUT_DESC);
	}
	return 0;
}
int env_set_desc(char *desc)
{
	if(setenv("dut_desc", desc)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_EXTRA_ENV_SETTINGS
int env_get_res1(int *flag)
{
	char* tmp = NULL;
	if((tmp = getenv("res_1")) == NULL){
		if(env_set_res1(0)) return -1;
		*flag = 0;
	}else{
		*flag = (int)simple_strtoul(tmp, NULL, 10);
	}
	return 0;
}
int env_set_res1(int flag)
{
	char tmp[5];
	sprintf(tmp, "%d", flag);
	if(setenv("res_1", tmp)) return -1;
	if(saveenv()) return -1;
	return 0;
}
int env_get_res2(int *flag)
{
	char* tmp = NULL;
	if((tmp = getenv("res_2")) == NULL){
		if(env_set_res2(0)) return -1;
		*flag = 0;
	}else{
		*flag = (int)simple_strtoul(tmp, NULL, 10);
	}
	return 0;
}
int env_set_res2(int flag)
{
	char tmp[5];
	sprintf(tmp, "%d", flag);
	if(setenv("res_2", tmp)) return -1;
	if(saveenv()) return -1;
	return 0;
}
int env_get_res3(int *flag)
{
	char* tmp = NULL;
	if((tmp = getenv("res_3")) == NULL){
		if(env_set_res3(0)) return -1;
		*flag = 0;
	}else{
		*flag = (int)simple_strtoul(tmp, NULL, 10);
	}
	return 0;
}
int env_set_res3(int flag)
{
	char tmp[5];
	sprintf(tmp, "%d", flag);
	if(setenv("res_3", tmp)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_PI_ON_TEMP
int env_get_pi_on_temp(int *temp)
{
	char* tmp = NULL;
	if((tmp = getenv("pi_on")) == NULL){
		if(env_set_pi_on_temp(CONFIG_PI_ON_TEMP)) return -1;
		*temp = CONFIG_PI_ON_TEMP;
	}else{
		*temp = (int)simple_strtoul(tmp, NULL, 16);
	}
	return 0;
}
int env_set_pi_on_temp(int temp)
{
	char tmp[5];
	sprintf(tmp, "0x%02x", temp & 0x000000FF);
	if(setenv("pi_on", tmp)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_PI_OFF_TEMP
int env_get_pi_off_temp(int *temp)
{
	char* tmp = NULL;
	if((tmp = getenv("pi_off")) == NULL){
		if(env_set_pi_off_temp(CONFIG_PI_OFF_TEMP)) return -1;
		*temp = CONFIG_PI_OFF_TEMP;
	}else{
		*temp = (int)simple_strtoul(tmp, NULL, 16);
	}
	return 0;
}
int env_set_pi_off_temp(int temp)
{
	char tmp[5];
	sprintf(tmp, "0x%02x", temp & 0x000000FF);
	if(setenv("pi_off", tmp)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_SYS_ON_TEMP
int env_get_sys_on_temp(int *temp)
{
	char* tmp = NULL;
	if((tmp = getenv("sys_on")) == NULL){
		if(env_set_sys_on_temp(CONFIG_SYS_ON_TEMP)) return -1;
		*temp = CONFIG_SYS_ON_TEMP;
	}else{
		*temp = (int)simple_strtoul(tmp, NULL, 16);
	}
	return 0;
}
int env_set_sys_on_temp(int temp)
{
	char tmp[5];
	sprintf(tmp, "0x%02x", temp & 0x000000FF);
	if(setenv("sys_on", tmp)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_PI_OFF_OFFS
int env_get_pi_off_offs(int *temp)
{
	char* tmp = NULL;
	if((tmp = getenv("pi_off_offs")) == NULL){
		if(env_set_pi_off_offs(CONFIG_PI_OFF_OFFS)) return -1;
		*temp = CONFIG_PI_OFF_OFFS;
	}else{
		*temp = (int)simple_strtol(tmp, NULL, 10);
	}
	return 0;
}
int env_set_pi_off_offs(int temp)
{
	char tmp[5];
	sprintf(tmp, "%d", temp);
	if(setenv("pi_off_offs", tmp)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_SYS_ON_OFFS
int env_get_sys_on_offs(int *temp)
{
	char* tmp = NULL;
	if((tmp = getenv("sys_on_offs")) == NULL){
		if(env_set_sys_on_offs(CONFIG_SYS_ON_OFFS)) return -1;
		*temp = CONFIG_SYS_ON_OFFS;
	}else{
		*temp = (int)simple_strtol(tmp, NULL, 10);
	}
	return 0;
}
int env_set_sys_on_offs(int temp)
{
	char tmp[5];
	sprintf(tmp, "%d", temp);
	if(setenv("sys_on_offs", tmp)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif
#ifdef CONFIG_SYS_ON_TIMEOUT
int env_get_sys_on_timeout(unsigned int *sec)
{
	char* tmp = NULL;
	if((tmp = getenv("sys_on_time")) == NULL){
		if(env_set_sys_on_timeout(CONFIG_SYS_ON_TIMEOUT)) return -1;
		*sec = CONFIG_SYS_ON_TIMEOUT;
	}else{
		*sec = (unsigned int)simple_strtoul(tmp, NULL, 10);
	}
	return 0;
}
int env_set_sys_on_timeout(unsigned int sec)
{
	char tmp[5];
	sprintf(tmp, "%u", sec);
	if(setenv("sys_on_time", tmp)) return -1;
	if(saveenv()) return -1;
	return 0;
}
#endif

