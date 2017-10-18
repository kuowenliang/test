/*
 * (C) Copyright 2000-2003
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
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
 * Moxa MM menu support
 */
#include <common.h>
#include <command.h>
#include <environment.h>
#include <image.h>
#include <i2c.h>
#include <mmc.h>
#include <miiphy.h>
#include <malloc.h>
#include <nand.h>
#include <net.h>
#include <spi.h>
#include <watchdog.h>
#include <asm/arch/cpu.h>
#include <asm/arch/nand.h>
#include <asm/arch/hardware.h>
#include <asm/io.h>
#include <linux/mtd/nand.h>
#include <rtc.h>
#include <tps65911.h>
#include <stdio_dev.h>
#include <serial.h>
#include <net.h>
#ifdef CONFIG_CODEC_AIC3104
#include <aic3104.h>
#endif

#ifdef CONFIG_CMD_MOXAMM

#ifndef	BOOL_WAS_DEFINED
#define BOOL_WAS_DEFINED
typedef enum { false = 0, true = 1 } bool;
#endif

typedef int		Bool;
#define TRUE		((Bool) 1)
#define FALSE	((Bool) 0)

#define DIAG_OK 0
#define DIAG_ERROR -1
#define DIAG_USER_ABORT -2
#define DIAG_FILE_NO_FIND -3

extern char * delete_char (char *buffer, char *p, int *colp, int *np, int plen);
extern int print_buffer (ulong addr, void* data, uint width, uint count, uint linelen);
extern int do_run (cmd_tbl_t * cmdtp, int flag, int argc, char *argv[]);
extern int do_reset(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);
extern int do_mem_md ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);
extern int do_mem_mw ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);
extern int net_loadimg (proto_t proto, ulong address, char* filename);
extern int serial_loadimg (char* mode, ulong address);
extern int do_fat_fsload(cmd_tbl_t *, int, int, char *[]);
extern int do_nand(cmd_tbl_t * cmdtp, int flag, int argc, char *argv[]);
//extern u32 GPIO_BaseAddr(int bank);
//extern u32 GPIO_SetClrAddr(int set);
extern void GPIO_OutEn(int bank, int pin, int en);
extern int GPIO_Input(int bank, int pin);
extern void GPIO_Output(int bank, int pin, int set);
extern int GPIO_In(int gpio_num);
extern void GPIO_Out(int gpio_num, int value);
extern int GPIO_OutStat(int gpio_num);

#if CONFIG_SYS_I2C_EEPROM
static char g_model_name[EEPROM_DATA_MODELNAME_LEN + 1] = {0};
static char g_mac_addr[18] = {0};
static char g_serial_str[EEPROM_DATA_SERIAL_LEN + 1] = {0};
static unsigned short g_mp_flag = 0;
#endif

int resetkey_state(void)
{
#ifdef GPIO_SYSBUTTON
	return (GPIO_In(GPIO_SYSBUTTON) == GPIO_SYSBUTTON_ON) ? 1 : 0;
#else
	return 0;
#endif
}

int lightsen_state(void)
{
#ifdef GPIO_LIGHT_SENSOR
	return GPIO_In(GPIO_LIGHT_SENSOR);
#else
	return 0;
#endif
}

void LED_Reverse(int led_gpio)
{
	if(GPIO_OutStat(led_gpio) == GPIO_LED_OFF) GPIO_Out(led_gpio, GPIO_LED_ON);
	else GPIO_Out(led_gpio, GPIO_LED_OFF);
}

void sys_led_R_ON(void)
{
#ifdef GPIO_SYSLED_RED
	GPIO_Out(GPIO_SYSLED_RED, GPIO_LED_ON);
#endif
}

void sys_led_R_OFF(void)
{
#ifdef GPIO_SYSLED_RED
	GPIO_Out(GPIO_SYSLED_RED, GPIO_LED_OFF);
#endif
}

void sys_led_R_reverse(void)
{
#ifdef GPIO_SYSLED_RED
	if(GPIO_OutStat(GPIO_SYSLED_RED)) sys_led_R_ON();
	else sys_led_R_OFF();
#endif
}

void sys_led_G_ON(void)
{
#ifdef GPIO_SYSLED_GREEN
	GPIO_Out(GPIO_SYSLED_GREEN, GPIO_LED_ON);
#endif
}

void sys_led_G_OFF(void)
{
#ifdef GPIO_SYSLED_GREEN
	GPIO_Out(GPIO_SYSLED_GREEN, GPIO_LED_OFF);
#endif
}

void sys_led_G_reverse(void)
{
#ifdef GPIO_SYSLED_GREEN
	if(GPIO_OutStat(GPIO_SYSLED_GREEN)) sys_led_G_ON();
	else sys_led_G_OFF();
#endif
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

#ifdef RS485_NAME
void RS485_SetTx(void)
{
#ifdef GPIO_RS485_ENT
	GPIO_Out(GPIO_RS485_ENT, GPIO_HIGH);
#endif
#ifdef GPIO_RS485_ENRn
	GPIO_Out(GPIO_RS485_ENRn, GPIO_HIGH);
#endif
}

void RS485_SetRx(void)
{
#ifdef GPIO_RS485_ENT
	GPIO_Out(GPIO_RS485_ENT, GPIO_LOW);
#endif
#ifdef GPIO_RS485_ENRn
	GPIO_Out(GPIO_RS485_ENRn, GPIO_LOW);
#endif
}
#endif

void sdcard_enable(void)
{
#ifdef GPIO_SD_EN
	GPIO_Out(GPIO_SD_EN, GPIO_HIGH);
#endif
}

void sdcard_disable(void)
{
#ifdef GPIO_SD_EN
	GPIO_Out(GPIO_SD_EN, GPIO_LOW);
#endif
}

#if defined(VPORT06EC_2V)
void Heater_Sys_ON(void)
{
#ifdef GPIO_HEATER_SYS
	GPIO_Out(GPIO_HEATER_SYS, GPIO_HIGH);
#endif
}

void Heater_Sys_OFF(void)
{
#ifdef GPIO_HEATER_SYS
	GPIO_Out(GPIO_HEATER_SYS, GPIO_LOW);
#endif
}

void Heater_Cam_ON(void)
{
#ifdef GPIO_HEATER_CAM
	GPIO_Out(GPIO_HEATER_CAM, GPIO_LOW);
#endif
}

void Heater_Cam_OFF(void)
{
#ifdef GPIO_HEATER_CAM
	GPIO_Out(GPIO_HEATER_CAM, GPIO_HIGH);
#endif
}
#else
void Heater_Sys_ON(void)
{
#ifdef GPIO_HEATER_SYS
	GPIO_Out(GPIO_HEATER_SYS, GPIO_HEATER_ON);
#endif
}

void Heater_Sys_OFF(void)
{
#ifdef GPIO_HEATER_SYS
	GPIO_Out(GPIO_HEATER_SYS, GPIO_HEATER_OFF);
#endif
}

void Heater_Cam_ON(void)
{
#ifdef GPIO_HEATER_CAM
	GPIO_Out(GPIO_HEATER_CAM, GPIO_HEATER_ON);
#endif
}

void Heater_Cam_OFF(void)
{
#ifdef GPIO_HEATER_CAM
	GPIO_Out(GPIO_HEATER_CAM, GPIO_HEATER_OFF);
#endif
}

#endif



#ifdef CONFIG_ETHADDR
#if !CONFIG_SYS_I2C_EEPROM
extern int env_get_ethaddr(char **mac);
extern int env_set_ethaddr(char *mac);
#endif
#endif

#ifdef CONFIG_IPADDR
extern int env_get_ipaddr(char **ip);
extern int env_set_ipaddr(char *ip);
#endif
#ifdef CONFIG_SERVERIP
extern int env_get_serverip(char **ip);
extern int env_set_serverip(char *ip);
#endif
#ifdef CONFIG_NETMASK
extern int env_get_netmask(char **ip);
extern int env_set_netmask(char *ip);
#endif
#ifdef CONFIG_BAUDRATE
extern int env_get_baudrate(unsigned int *rate);
extern int env_set_baudrate(unsigned int rate);
#endif

#ifdef CONFIG_MP_FLAG
#if !CONFIG_SYS_I2C_EEPROM
extern int env_get_mp_flag(int *flag);
extern int env_set_mp_flag(int flag);
#endif
#endif

#ifdef CONFIG_IVA_FLAG
extern int env_get_iva_flag(int *flag);
extern int env_set_iva_flag(int flag);
#endif
#ifdef CONFIG_BURNIN_TIME
extern int env_get_burnintime(unsigned int *time);
extern int env_set_burnintime(unsigned int time);
#endif
#ifdef CONFIG_T2_BOOTUP_COUNTER
extern int env_get_t2_bootup_counter(unsigned int *counter);
extern int env_set_t2_bootup_counter(unsigned int counter);
#endif

#ifdef CONFIG_SERIAL_NUMBER
#if !CONFIG_SYS_I2C_EEPROM
extern int env_get_serial_number(char **number);
extern int env_set_serial_number(char *number);
#endif
#endif

#ifdef CONFIG_DUT_ID
extern int env_get_dut_id(char **id);
extern int env_set_dut_id(char *id);
#endif
#ifdef CONFIG_FW_RECOVERY
extern int env_get_fw_recovery(unsigned int *rcv);
extern int env_set_fw_recovery(unsigned int rcv);
#endif
#if !(defined(CONFIG_FW_RECOVERY_FIX) && defined(CONFIG_FW_RECOVERY_FILE))
extern int env_get_fw_recovery_file(char **file);
extern int env_set_fw_recovery_file(char *file);
#endif

#ifdef CONFIG_DUT_MODEL
#if !CONFIG_SYS_I2C_EEPROM
extern int env_get_model(char **model);
extern int env_set_model(char *model);
#endif
#endif

#ifdef CONFIG_DUT_HOST
extern int env_get_host(char **host);
extern int env_set_host(char *host);
#endif
#ifdef CONFIG_DUT_DESC
extern int env_get_desc(char **desc);
extern int env_set_desc(char *desc);
#endif
extern Bool SD_test(int dev_num);
extern Bool Audio_test(void);
#ifdef CONFIG_CODEC_AIC3104
extern int Audio_HW_Reset(int bank, int pin);
extern Bool DRVfnAudio_AIC3104SendData(u8 addr, u8 val);
extern Bool DRVfnAudio_AIC3104RecvData(u8 addr, u8 *buf, int length);
extern Bool Aic3104_RegisterDump(void);
#endif

extern IPaddr_t	NetPingIP;		/* the ip address to ping		*/


//Global variable used define=======================================
static char *EqualLine = "===============================================================================";
int check_valid_char (char ch, char *buffer);
static char erase_seq[] = "\b \b";		/* erase sequence	*/
static char tab_seq[] = "        ";		/* used to expand TABs	*/
static char str_number_dec[] = "0123456789";
static char str_number_hex[] = "0123456789abcdefABCDEF";
static char str_mac[] = "0123456789abcdefABCDEF:";
static char str_ip[] = "0123456789.";
static char str_menu_operate[] = "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
#ifdef CONFIG_UBOOT_IDX
static int ubl_ubootIdx = 0;
#endif

static int diag_do_reset(int parameter);
int _erase_flash(cmd_tbl_t *cmdtp, int flag, ulong nand_addr, ulong size);

//Struct define=====================================================
typedef struct _Diag_Menu_Table_Struct {
	char						index;
	char						*Description;
	int							Parameter;
	int							(*Func)(int parameter);
	struct _Diag_Menu_Struct	*SubMenu;
} DiagMenuTableStruct, *pDiagMenuTableStruct;

typedef struct _Diag_Menu_Struct {
	int						TableNo;
	pDiagMenuTableStruct	MenuTable;
	char					*Title;
	int						Parameter;
	int						(*Func)(int parameter);
} DiagMenuStruct, *pDiagMenuStruct;

#define	_endtick(seconds) (get_ticks() + (uint64_t)(seconds) * get_tbclk())

/****************************************************************************/

#define _GETLINE_CTRLC   -1
#define _GETLINE_TIMEOUT -2
int get_line (char * prompt, char * buffer, int size, int timeout, char * valid_chars, char * invalid_chars, char * default_chars)
{
	char *p = buffer;
	uint64_t endtime = _endtick(timeout);
	char * p_buf = p;
	int	n = 0;				/* buffer index		*/
	int	plen = 0;			/* prompt length	*/
	int	col;				/* output column cnt	*/
	char c;

	/* print prompt */
	if (prompt) {
		plen = strlen (prompt);
		puts (prompt);
	}
	col = plen;

	for (;;) {

		while (!tstc()) {	/* while no incoming data */
			WATCHDOG_RESET();		/* Trigger watchdog, if needed */
			if (timeout >= 0 && get_ticks() > endtime){
				if(default_chars) memcpy(buffer, default_chars, size);
				return (_GETLINE_TIMEOUT);	/* timed out */
			}
		}

		c = getc();

		/*
		 * Special character handling
		 */
		switch (c) {
		case '\r':				/* Enter		*/
		case '\n':
			if(default_chars && !(p - p_buf)) memcpy(p, default_chars, size);
			else *p = '\0';
			puts ("\r\n");
			return (p - p_buf);

		case '\0':				/* nul			*/
			continue;

		case 0x03:				/* ^C - break		*/
			p_buf[0] = '\0';	/* discard input */
			return (_GETLINE_CTRLC);

		case 0x15:				/* ^U - erase line	*/
			while (col > plen) {
				puts (erase_seq);
				--col;
			}
			p = p_buf;
			n = 0;
			continue;

		case 0x17:				/* ^W - erase word	*/
			p=delete_char(p_buf, p, &col, &n, plen);
			while ((n > 0) && (*p != ' ')) {
				p=delete_char(p_buf, p, &col, &n, plen);
			}
			continue;

		case 0x08:				/* ^H  - backspace	*/
		case 0x7F:				/* DEL - backspace	*/
			p=delete_char(p_buf, p, &col, &n, plen);
			continue;

		default:
			/*
			 * Must be a normal character then
			 */
			if (n < size-2) {
				if (c == '\t') {	/* expand TABs		*/
					puts (tab_seq+(col&07));
					col += 8 - (col&07);
					*p++ = c;
					++n;
				} else
				if(valid_chars) {
					if(check_valid_char(c, valid_chars)){
						++col;		/* echo input		*/
						putc (c);
						*p++ = c;
						++n;
					}
				} else
				if(invalid_chars) {
					if(!check_valid_char(c, invalid_chars)){
						++col;		/* echo input		*/
						putc (c);
						*p++ = c;
						++n;
					}
				} else {
					++col;		/* echo input		*/
					putc (c);
					*p++ = c;
					++n;
				}
			} else {			/* Buffer full		*/
				putc ('\a');
			}
		}
	}
}

int check_valid_char (char ch, char *buffer)
{
	int rcode = 0;
	int i = 0;
	int len = strlen(buffer);
	for(i=0; i<len; i++){
		if(buffer[i] == ch){
			rcode = 1;
			break;
		}
	}
	return rcode;
}

void Sleep(unsigned long sec)
{
	ulong start = 0;
	ulong delay = sec * CFG_HZ;
	reset_timer();
	start = get_timer(0);
	while(get_timer(start) < delay) {
		udelay (100);
	}
}

int mem_test(size_t len, unsigned int pattern, unsigned int address, int verbose)
{
	volatile int * mem = (int*) address;
#if defined(TEST_LED_BLINK_ON_MEM_TEST) && defined(GPIO_LED_STAT_G) && defined(GPIO_LED_STAT_R)
	GPIO_Out(GPIO_LED_STAT_G, GPIO_LED_ON);
	GPIO_Out(GPIO_LED_STAT_R, GPIO_LED_OFF);
#endif
	printf("\nMemory test start, pattern = [0x%08X], addr = 0x%08X, size = 0x%08X\n", pattern, (int)(mem), len*sizeof(int));
	if(mem == NULL){
		printf("\nMemory allocte error!!\n");
		return -1;
	}
	int i;
	for ( i = 0; i < len; i++)  {
		WATCHDOG_RESET();

#if defined(TEST_LED_BLINK_ON_MEM_TEST) && defined(GPIO_LED_STAT_G) && defined(GPIO_LED_STAT_R)
		if(i % 0x20000 == 0){
			LED_Reverse(GPIO_LED_STAT_G);
			LED_Reverse(GPIO_LED_STAT_R);
		}
#endif

		if(ctrlc ()){
			printf("\nAbort!!\n");
			return -1;
		}

		if(verbose) printf("Memory test at 0x%08X               \r", (int)(mem+i));

		if((((int)(mem+i) >= PHYS_DRAM_1) && ((int)(mem+i) <= (PHYS_DRAM_1+PHYS_DRAM_1_SIZE-1)))
#ifdef PHYS_DRAM_2
			|| (((int)(mem+i) >= PHYS_DRAM_2) && ((int)(mem+i) <= (PHYS_DRAM_2+PHYS_DRAM_2_SIZE-1)))
#endif
		){
			*(mem+i) = pattern;
			if ( *(mem+i) != pattern ) {
				printf("\nMemory test error at 0x%08X\n", (int)(mem+i));
				return -1;
			}
		}else{
			printf("\nMemory test out of range (0x%08X)\n", (int)(mem+i));
			return -1;
		}
	}
	printf("\nMemory test passed.\n");
	return 0;
}

int do_memtest (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	if (argc < 1) {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return -1;
	}
	size_t len = 1024;
	unsigned int pattern = 0x5555aaaa;
	unsigned int address = CFG_MEMTEST_START;
	int verbose = 0;
	if(argc >= 2) len = (size_t)simple_strtoul(argv[1], NULL, 16);
	if(argc >= 3) pattern = (unsigned int)simple_strtoul(argv[2], NULL, 16);
	if(argc >= 4) address = (unsigned int)simple_strtoul(argv[3], NULL, 16);
	if(argc >= 5) verbose = (int)simple_strtol(argv[3], NULL, 10);
	return mem_test(len, pattern, address, verbose);
}

U_BOOT_CMD(
	memtest, 4, 0,	do_memtest,
	"Memory test",
	"Memory test\n\
	memtest [len][pattern][address][verbose]"
);

int do_initenv (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	_erase_flash(cmdtp, flag, ENV1_FLASH, ENV1_SIZE);
#ifdef ENV2_FLASH
	_erase_flash(cmdtp, flag, ENV2_FLASH, ENV2_SIZE);
#endif
	set_default_env();
	udelay(100000);
	saveenv();	/* Bobby Chen - 20151021 : force write to flash */
#ifdef CONFIG_SYS_REDUNDAND_ENVIRONMENT
	saveenv();	/* Bobby Chen - 20151021 : make sure write to each env partition */
#endif
	return 0;
}

U_BOOT_CMD(
	initenv, 1, 0,	do_initenv,
	"Init environment",
	NULL
);

int do_gpio (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	int index = 0;
	int value = 0;
	if (argc < 2) {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return -1;
	}
	if(argc >= 2) index = (int)simple_strtoul(argv[1], NULL, 10);
	if(argc >= 3){ // write operation
		value = (int)simple_strtoul(argv[2], NULL, 10);
		GPIO_Out(index, value);
	}else{ // read operation
		printf("gpio[%d]=%d\n", index, GPIO_In(index));
	}
	return 0;
}

U_BOOT_CMD(
	gpio, 3, 0,	do_gpio,
	"Set/Get GPIO",
	"Set/Get GPIO\n\
	gpio [index][set_value]"
);

#if CONFIG_SYS_I2C_EEPROM
//wensen for VPort06EC-2
int EEPROM_Clear(void)
{
	uchar chip;
	uint addr;
	int alen = 1;
	uchar value[EEPROM_DATA_PAGE_SIZE];
	int len;
	int i, j;

	alen = CONFIG_SYS_I2C_EEPROM_ADDR_LEN;
	chip = CONFIG_SYS_I2C_EEPROM_ADDR;
	addr = EEPROM_DATA_MODELNAME_POS;
	len = EEPROM_DATA_TOTAL_SIZE;

	memset(value, 0xff, EEPROM_DATA_PAGE_SIZE);
	for (i = 0; i < (EEPROM_DATA_TOTAL_SIZE / EEPROM_DATA_PAGE_SIZE); i++)
	{
		addr = (i * EEPROM_DATA_PAGE_SIZE);
		if (i2c_write(chip, addr, alen, &value[0], EEPROM_DATA_PAGE_SIZE))
		{
			printf("Failed to write I2C bus.\n");
			return DIAG_ERROR;
		}

		udelay(1000);
	}

	return DIAG_OK;
}

int EEPROM_Dump(void)
{
	uchar chip;
	uint addr;
	int alen = 1;
	uchar value[EEPROM_DATA_PAGE_SIZE];
	int len;
	int i, j;

	alen = CONFIG_SYS_I2C_EEPROM_ADDR_LEN;
	chip = CONFIG_SYS_I2C_EEPROM_ADDR;
	addr = EEPROM_DATA_MODELNAME_POS;
	len = EEPROM_DATA_TOTAL_SIZE;

	printf("Dumping eeprom from Page %d\n", 0);
	printf("--------------------------------\n");
	printf("0x| 00 01 02 03 04 05 06 07 08 |\n");
	printf("--------------------------------\n");
	for (i = 0; i < (EEPROM_DATA_TOTAL_SIZE / EEPROM_DATA_PAGE_SIZE); i++)
	{
		printf("%02X| ", (i * EEPROM_DATA_PAGE_SIZE));
		addr = (i * EEPROM_DATA_PAGE_SIZE);
		if (i2c_read(chip, addr, alen, value, EEPROM_DATA_PAGE_SIZE))
		{
			printf("Failed to read I2C bus.\n");
			return DIAG_ERROR;
		}

		for (j = 0; j < EEPROM_DATA_PAGE_SIZE; j++)
		{
			printf("%02X ", value[j]);
    	}
		printf("|");

		for (j = 0; j < EEPROM_DATA_PAGE_SIZE; j++)
		{
			if ((value[j] > 0x1f) &&
				(value[j] < 0x7f))
			{
				printf("%c", value[j]);
			}
			else
			{
				printf(".");
			}
		}
		printf("|\n");

		udelay(1000);
	}

	return DIAG_OK;
}

//-----------------------------------------------------------------------------
int EEPROM_SetHWFeature(void)
{
	uchar chip;
	uint addr;
	int alen = 0;
	uchar hw_feature[EEPROM_DATA_HWFEATURE_LEN + 1] = {0};
	int len;
	int i;
	char hw_feature_byte = 0;

	//chip = CONFIG_SYS_I2C_EEPROM_ADDR;
	//addr = EEPROM_DATA_HWFEATURE_POS;
	//len = EEPROM_DATA_HWFEATURE_LEN;
	//alen = CONFIG_SYS_I2C_EEPROM_ADDR_LEN;

	//if (i2c_read(chip, addr, alen, hw_feature, len))
	//{
	//	printf("Failed : %s : read I2C bus.\n", __func__);
	//	return DIAG_ERROR;
	//}

	//if (strncmp(hw_feature, EEPROM_DATA_PROFILE_ID, strlen(EEPROM_DATA_PROFILE_ID)) == 0)
	//{
	//	return DIAG_OK;
	//}

	chip = CONFIG_SYS_I2C_EEPROM_ADDR;
	addr = EEPROM_DATA_HWFEATURE_POS;
	sprintf(hw_feature, "%s", EEPROM_DATA_HWFEATURE_ID);

	hw_feature_byte |= (CONFIG_HF_AUDIO_LINEIN << 7);
	hw_feature_byte |= (CONFIG_HF_AUDIO_MICROPHONE << 6);
	hw_feature_byte |= (CONFIG_HF_AUDIO_OUT << 5);
	hw_feature[strlen(EEPROM_DATA_HWFEATURE_ID)] = hw_feature_byte;

	len = EEPROM_DATA_HWFEATURE_LEN;
	alen = CONFIG_SYS_I2C_EEPROM_ADDR_LEN;

	for (i = 0; i < len; i++)
	{
		if (i2c_write(chip, addr++, alen, &hw_feature[i], 1))
		{
			printf("Failed to write I2C bus.\n");
		return DIAG_ERROR;
	}
		udelay(1000);
	}

	return DIAG_OK;
}

//-----------------------------------------------------------------------------
int EEPROM_SetProfile(void)
{
	uchar chip;
	uint addr;
	int alen = 0;
	uchar profile[EEPROM_DATA_PROFILE_LEN + 1] = {0};
	int len;
	int i;

	//chip = CONFIG_SYS_I2C_EEPROM_ADDR;
	//addr = EEPROM_DATA_PROFILE_POS;
	//len = EEPROM_DATA_PROFILE_LEN;
	//alen = CONFIG_SYS_I2C_EEPROM_ADDR_LEN;

	//if (i2c_read(chip, addr, alen, profile, len))
	//{
	//	printf("Failed : %s : read I2C bus.\n", __func__);
	//	return DIAG_ERROR;
	//}

	//if (strncmp(profile, EEPROM_DATA_PROFILE_ID, strlen(EEPROM_DATA_PROFILE_ID)) == 0)
	//{
	//	return DIAG_OK;
	//}

	chip = CONFIG_SYS_I2C_EEPROM_ADDR;
	addr = EEPROM_DATA_PROFILE_POS;
	sprintf(profile, "%s", EEPROM_DATA_PROFILE_ID);
	profile[strlen(EEPROM_DATA_PROFILE_ID)] = 0x01;
	profile[strlen(EEPROM_DATA_PROFILE_ID) + 1] = ((EEPROM_DATA_TOTAL_SIZE >> 8) & 0xff);
	profile[strlen(EEPROM_DATA_PROFILE_ID) + 2] = (EEPROM_DATA_TOTAL_SIZE & 0xff);
	len = EEPROM_DATA_PROFILE_LEN;
	alen = CONFIG_SYS_I2C_EEPROM_ADDR_LEN;

	for (i = 0; i < len; i++)
	{
		if (i2c_write(chip, addr++, alen, &profile[i], 1))
		{
			printf("Failed to write I2C bus.\n");
			return DIAG_ERROR;
		}
		udelay(1000);
	}

	return DIAG_OK;
}

//---------------------------------------------------------------------------
int EEPROM_GetModelName(char *modelname)
{
	uchar chip;
	uint addr;
	int alen;
	int len;
	int i;
	char temp[10];
	int total_len = 0;

	chip = CONFIG_SYS_I2C_EEPROM_ADDR;
	addr = EEPROM_DATA_MODELNAME_POS;
	//len = EEPROM_DATA_MODELNAME_LEN;
	alen = CONFIG_SYS_I2C_EEPROM_ADDR_LEN;

	while (total_len < EEPROM_DATA_MODELNAME_LEN)
	{
		if ((EEPROM_DATA_MODELNAME_LEN - total_len) > 10)
		{
			len = 10;
		}
		else
		{
			len = EEPROM_DATA_MODELNAME_LEN - total_len;
		}
		if (i2c_read(chip, addr + total_len, alen, temp, len))
		{
			printf("Failed : %s : read I2C bus.\n", __func__);
			return DIAG_ERROR;
		}

		memcpy(modelname + total_len, temp, len);

		for (i = 0; i < len; i++)
		{
			if (temp[i] == 0x00)
			{
				break;
			}
		}
		if (i != len)
		{
			break;
		}


		total_len = total_len + len;
	}

	strcpy(modelname, modelname + strlen(EEPROM_DATA_MODELNAME_ID));

	return DIAG_OK;
}

int EEPROM_SetModelName(char *modelname)
{
	uchar chip;
	uint addr;
	int alen = 0;
	uchar value[EEPROM_DATA_MODELNAME_LEN + 1] = {0};
	int len;
	int i;

	if (strlen(modelname) > (EEPROM_DATA_MODELNAME_LEN - strlen(EEPROM_DATA_MODELNAME_ID)))
	{
		printf("The model name shall be shorter than 60 bytes\n");
		return DIAG_ERROR;
	}

	chip = CONFIG_SYS_I2C_EEPROM_ADDR;
	addr = EEPROM_DATA_MODELNAME_POS;

	sprintf(value, "%s%s", EEPROM_DATA_MODELNAME_ID, modelname);

	len = EEPROM_DATA_MODELNAME_LEN;
	alen = CONFIG_SYS_I2C_EEPROM_ADDR_LEN;

	for (i = 0; i < len; i++)
	{
		if (i2c_write(chip, addr++, alen, &value[i], 1))
		{
			printf("Failed to write I2C bus.\n");
			return DIAG_ERROR;
		}
		udelay(1000);
	}

	return DIAG_OK;
}

//-----------------------------------------------------------------------------
int EEPROM_GetMac(char *mac)
{
	uchar chip;
	uint addr;
	int alen;
	int len;
	int i;
	uchar mac_str[EEPROM_DATA_MAC_LEN] = {0};
	uchar mac_addr[6] = {0};

	chip = CONFIG_SYS_I2C_EEPROM_ADDR;
	addr = EEPROM_DATA_MAC_POS;
	len = EEPROM_DATA_MAC_LEN;
	alen = CONFIG_SYS_I2C_EEPROM_ADDR_LEN;

	if (i2c_read(chip, addr, alen, mac_str, len))
	{
		printf("Failed : %s : read I2C bus.\n", __func__);
		return DIAG_ERROR;
	}

	for (i = 0; i < 6; i++)
	{
		mac_addr[i] = mac_str[strlen(EEPROM_DATA_MAC_ID) + i];
	}

	sprintf(mac, "%02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

	return DIAG_OK;
}

int EEPROM_SetMac(char *mac)
{
	uchar chip;
	uint addr;
	int alen = 0;
	uchar mac_addr[6] = {0};
	uchar value[EEPROM_DATA_MAC_LEN] = {0};
	int len;
	int i;
	int mac_addr_idx = 0;
	int ret = 0;
	uchar temp, temp1;

	i = 0;
	while (1)
	{
		temp = mac[i++];
		if ((temp >= 0x30) && temp <= 0x39)
		{
			temp = temp - 0x30;
		}
		else if ((temp >= 0x41) && temp <= 0x46)
		{
			temp = temp - 0x41 + 10;
		}
		else if ((temp >= 0x61) && temp <= 0x66)
		{
			temp = temp - 0x61 + 10;
		}
		else
		{
			return -1;
		}

		temp1 = mac[i++];
		if ((temp1 >= 0x30) && temp1 <= 0x39)
		{
			temp1 = temp1 - 0x30;
		}
		else if ((temp1 >= 0x41) && temp1 <= 0x46)
		{
			temp1 = temp1 - 0x41 + 10;
		}
		else if ((temp1 >= 0x61) && temp1 <= 0x66)
		{
			temp1 = temp1 - 0x61 + 10;
		}
		else
		{
			return -1;
		}

		if (mac[i++] == ':' || i > strlen(mac))
		{
			mac_addr[mac_addr_idx] = (temp << 4) + temp1;
			mac_addr_idx++;
		}

		if (mac_addr_idx == 6)
		{
			break;
		}
	}

	chip = CONFIG_SYS_I2C_EEPROM_ADDR;
	addr = EEPROM_DATA_MAC_POS;
	len = EEPROM_DATA_MAC_LEN;
	alen = CONFIG_SYS_I2C_EEPROM_ADDR_LEN;

	sprintf(value, "%s", EEPROM_DATA_MAC_ID);
	for (i = 0; i < 6; i++)
	{
		value[i + strlen(EEPROM_DATA_MAC_ID)] = mac_addr[i];
	}

	for (i = 0; i < len; i++)
	{
		if (i2c_write(chip, addr++, alen, &value[i], 1))
		{
			printf("Failed to write I2C bus.\n");
			return DIAG_ERROR;
		}
		udelay(1000);
	}

	return DIAG_OK;
}

//-----------------------------------------------------------------------------
int EEPROM_GetSerial(char *serial_str)
{
	uchar chip;
	uint addr;
	int alen;
	int len;
	int i;

	chip = CONFIG_SYS_I2C_EEPROM_ADDR;
	addr = EEPROM_DATA_SERIAL_POS;
	len = EEPROM_DATA_SERIAL_LEN;
	alen = CONFIG_SYS_I2C_EEPROM_ADDR_LEN;

	if (i2c_read(chip, addr, alen, serial_str, len))
	{
		printf("Failed : %s : read I2C bus.\n", __func__);
		return DIAG_ERROR;
	}

	strcpy(serial_str, serial_str + strlen(EEPROM_DATA_SERIAL_ID));

	return DIAG_OK;
}

int EEPROM_SetSerial(char *serial_str)
{
	uchar chip;
	uint addr;
	int alen = 0;
	uchar value[EEPROM_DATA_SERIAL_LEN + 1] = {0};
	int len;
	int i;

	if (strlen(serial_str) > (EEPROM_DATA_SERIAL_LEN - strlen(EEPROM_DATA_SERIAL_ID)))
	{
		printf("The model name shall be shorter than 12 bytes\n");
	}


	chip = CONFIG_SYS_I2C_EEPROM_ADDR;
	addr = EEPROM_DATA_SERIAL_POS;
	sprintf(value, "%s%s", EEPROM_DATA_SERIAL_ID, serial_str);
	len = EEPROM_DATA_SERIAL_LEN;
	alen = CONFIG_SYS_I2C_EEPROM_ADDR_LEN;

	for (i = 0; i < len; i++)
	{
		if (i2c_write(chip, addr++, alen, &value[i], 1))
		{
			printf("Failed to write I2C bus.\n");
			return DIAG_ERROR;
		}
		udelay(1000);
	}

	return DIAG_OK;
}

//---------------------------------------------------------------------------
int EEPROM_GetMpFlag(unsigned short *mpflag)
{
	uchar chip;
	uint addr;
	int alen;
	int len;
	int i;
	uchar value[EEPROM_DATA_MPFLAG_LEN + 1] = {0};

	chip = CONFIG_SYS_I2C_EEPROM_ADDR;
	addr = EEPROM_DATA_MPFLAG_POS;
	len = EEPROM_DATA_MPFLAG_LEN;
	alen = CONFIG_SYS_I2C_EEPROM_ADDR_LEN;

	if (i2c_read(chip, addr, alen, value, len))
	{
		printf("Failed : %s : read I2C bus.\n", __func__);
		return DIAG_ERROR;
	}

	*mpflag = (value[strlen(EEPROM_DATA_MPFLAG_ID)] << 8) + value[strlen(EEPROM_DATA_MPFLAG_ID) + 1];

	return DIAG_OK;
}

int EEPROM_SetMpFlag(unsigned short mpflag)
{
	uchar chip;
	uint addr;
	int alen = 0;
	uchar value[EEPROM_DATA_MPFLAG_LEN + 1] = {0};
	int len;
	int i;

	chip = CONFIG_SYS_I2C_EEPROM_ADDR;
	addr = EEPROM_DATA_MPFLAG_POS;
	sprintf(value, "%s", EEPROM_DATA_MPFLAG_ID);
	len = EEPROM_DATA_MPFLAG_LEN;
	alen = CONFIG_SYS_I2C_EEPROM_ADDR_LEN;

	value[strlen(EEPROM_DATA_MPFLAG_ID)] = ((mpflag >> 8) & 0xff);
	value[strlen(EEPROM_DATA_MPFLAG_ID) + 1] = (mpflag & 0xff);

	for (i = 0; i < len; i++)
	{
		if (i2c_write(chip, addr++, alen, &value[i], 1))
		{
			printf("Failed to write I2C bus.\n");
			return DIAG_ERROR;
		}
		udelay(1000);
	}

	return DIAG_OK;
}

#endif

int SD_init(void)
{
	struct mmc *mmc;
	sdcard_enable();
	mmc = find_mmc_device(0);
	if (mmc) {
		if (mmc_init(mmc) == 0){
			return 0;
		}
		else
			puts("No MMC/SD card found\n");
	}
	else
		puts("MMC Device not found\n");

	return -1;
}

long load_image(cmd_tbl_t *cmdtp, int flag, char* mode, char* file, ulong buff_addr)
{
	char *args[5];
	char buffAddr[11];//0xffffffff
	char baudRate[7];//112500
	int size = 0;
	if(mode == NULL || buff_addr == 0) return -1;
	sprintf(buffAddr, "0x%08lx", buff_addr);
	sprintf(baudRate, "%d", CONFIG_BAUDRATE);
	if(strcmp(mode, "tftp") == 0){
		if(file == NULL) return -1;
		if((size = net_loadimg(TFTP, buff_addr, file)) <= 0){
			printf("Failed to download image from tftp.\n");
			return 0;
		}
	}else if(strcmp(mode, "kermit") == 0){
		if((size = serial_loadimg("loadb", buff_addr)) <= 0){
			printf("Failed to download image from serial(%s).\n", mode);
			return 0;
		}
	}else if(strcmp(mode, "xymodem") == 0){
		if((size = serial_loadimg("loady", buff_addr)) <= 0){
			printf("Failed to download image from serial(%s).\n", mode);
			return 0;
		}
	}else if(strncmp(mode, "mmc", 3) == 0){
		if(file == NULL) return -1;
		if(SD_init()) return -1;
		args[0] = "fatload";
		args[1] = "mmc";
		if(strlen(mode) > 3)
			args[2] = (mode + 4); // mmc(dev[:part]) , ex. mmc(0:2)
		else
			args[2] = "0";
		args[3] = buffAddr;
		args[4] = file;
		if (do_fat_fsload(cmdtp, 0, 5, args) != 0) {
			printf("Failed to download image at mmc device.\n");
			return 0;
		}
		if((size = (int)simple_strtoul(getenv("filesize"), NULL, 16)) <= 0){
			printf("Failed to download image at mmc device.\n");
			return 0;
		}
	}
	return (long)size;
}

int _clean_buffer(cmd_tbl_t *cmdtp, int flag, ulong buff_addr, ulong size)
{
	char *args[5];
	char buffAddr[11];//0xffffffff
	char imageLen[11];//0xffffffff

	sprintf(buffAddr, "0x%08lx", buff_addr);
	sprintf(imageLen, "0x%08lx", size);

	args[0] = "mw.b";
	args[1] = buffAddr;
	args[2] = "0xff";
	args[3] = imageLen;
	udelay(1000);
	if(do_mem_mw(cmdtp, flag, 4, args)){
		printf("Failed to write '0xff' to 0x%08lx, size 0x%08lx.\n", buff_addr, size);
		return -1;
	}

	return 0;
}

int _erase_flash(cmd_tbl_t *cmdtp, int flag, ulong nand_addr, ulong size)
{
	char *args[5];
	char nandAddr[11];//0xffffffff
	char imageLen[11];//0xffffffff

	sprintf(nandAddr, "0x%08lx", nand_addr);
	if(size == 0)
		sprintf(imageLen, "");
	else
		sprintf(imageLen, "0x%08lx", size);

	args[0] = "nand";
	args[1] = "erase";
	args[2] = nandAddr;
	args[3] = imageLen;
	udelay(1000);
	if(do_nand(cmdtp, flag, ((size != 0) ? 4 : 3), args)){
		printf("Failed to erase nand flash at %s.\n", nandAddr);
		return -1;
	}

	return 0;
}

int _erase_flash_whole(cmd_tbl_t *cmdtp, int flag)
{
	char *args[5];

	args[0] = "nand";
	args[1] = "erase";
	args[2] = "clean";
	udelay(1000);
	if(do_nand(cmdtp, flag, 3, args)){
		printf("Failed to erase whole nand flash.\n");
		return -1;
	}

	return 0;
}

int _scrub_flash(cmd_tbl_t *cmdtp, int flag, ulong nand_addr, ulong size, int quiet)
{
	int ret = 0;
	char *args[5];
	char nandAddr[11];//0xffffffff
	char imageLen[11];//0xffffffff

	if (quiet) setenv("quiet", "1");

	sprintf(nandAddr, "0x%08lx", nand_addr);
	sprintf(imageLen, "0x%08lx", size);

	args[0] = "nand";
	args[1] = "scrub";
	args[2] = nandAddr;
	args[3] = imageLen;
	udelay(1000);
	if(do_nand(cmdtp, flag, ((size != 0) ? 4 : 3), args)){
		printf("Failed to scrub nand flash at %s.\n", nandAddr);
		ret = -1;
	}

	if (quiet) setenv("quiet", "0");
	return ret;
}

int _scrub_flash_whole(cmd_tbl_t *cmdtp, int flag, int quiet)
{
	int ret = 0;
	char *args[5];

	if (quiet) setenv("quiet", "1");

	args[0] = "nand";
	args[1] = "scrub";
	udelay(1000);
	if(do_nand(cmdtp, flag, 2, args)){
		printf("Failed to scrub whole nand flash.\n");
		ret = -1;
	}

	if (quiet) setenv("quiet", "0");
	return ret;
}

int _write_flash(cmd_tbl_t *cmdtp, int flag, ulong buff_addr, ulong nand_addr, ulong size)
{
	char *args[5];
	char buffAddr[11];//0xffffffff
	char nandAddr[11];//0xffffffff
	char imageLen[11];//0xffffffff

	sprintf(buffAddr, "0x%08lx", buff_addr);
	sprintf(nandAddr, "0x%08lx", nand_addr);
	sprintf(imageLen, "0x%08lx", size);

	args[0] = "nand";
	args[1] = "write";
	args[2] = buffAddr;
	args[3] = nandAddr;
	args[4] = imageLen;
	udelay(1000);
	if(do_nand(cmdtp, flag, 5, args)){
		printf("Failed to write nand flash at %s.\n", nandAddr);
		return -1;
	}

#ifdef _DEBUG_FLASH_WRITE_
	printf("\n");
	args[0] = "nand";
	args[1] = "dump";
	args[2] = nandAddr;
	udelay(1000);
	if(!do_nand(cmdtp, flag, 5, args)){
		printf("Failed to dump nand flash at %s.\n", nandAddr);
		return -1;
	}
	printf("\n");
#endif

	return 0;
}

int _write_flash_block(cmd_tbl_t *cmdtp, int flag, char* buffer, ulong size, ulong max_size, ulong nand_base, char stuff)
{
	int block_num = 0, block_max = 0;
	ulong block_size = 0;
	unsigned char buf[0x20000];
	ulong file_length;
	ulong read_size = 0;
	ulong buf_size = 0;
	ulong nand_offset = 0, nand_firstblock = 0;
	ulong buffer_ptr = 0;

	nand_info_t *nand = &nand_info[nand_curr_device];

	//initial buffer
	block_size = buf_size = nand->erasesize;
	nand_firstblock = nand_base / block_size;
	sys_led_RG_OFF();
	setenv("quiet", "1");
	block_num = 0;
	block_max = (max_size / block_size) + (max_size % block_size ? 1 : 0);
	file_length = size;
	while(file_length > 0)
	{
		if (file_length > buf_size) read_size = buf_size;
		else read_size = file_length;
		memset(buf, stuff, buf_size);
		memcpy(buf, &buffer[buffer_ptr], read_size);
		buffer_ptr += read_size;

		//check & skip bad block
		while(block_num < block_max){
			nand_offset = block_num * block_size;
			if(nand_block_isbad(nand, nand_base+nand_offset)){
				printf("\nSkip bad block %ld (0x%08lx)\n", nand_firstblock+block_num, nand_base+nand_offset);
				block_num++;
				if( !(block_num < block_max) ){
					printf("\nToo many bad blocks(%d/%d), write failed!!\n", block_num, block_max);
					block_num = 0; // for return error
					goto write_flash_page_exit;
				}
			}else{
				break;
			}
		}
		if(_write_flash(cmdtp, flag, (ulong)buf, nand_base+nand_offset, buf_size)){
			block_num = 0;
			goto write_flash_page_exit;
		}
		printf("Read data, size 0x%lx, write to bolck %ld~%ld (0x%08lX)    \r",
			read_size, nand_firstblock, nand_firstblock+block_num, nand_base+nand_offset);
		file_length -= read_size;
		block_num++;
		sys_led_RG_reverse();
	}

write_flash_page_exit:

	sys_led_RG_OFF();
	printf("\r\n");
	setenv("quiet", "0");
	return block_num;
}

int erase_write_flash(cmd_tbl_t *cmdtp, int flag, ulong buff_addr, ulong nand_addr, ulong erase_size, ulong write_size, char stuff)
{
	ulong _write_size = (write_size > erase_size ? erase_size : write_size);
	if((_erase_flash(cmdtp, flag, nand_addr, erase_size)) ||
		(!_write_flash_block(cmdtp, flag, (char*)buff_addr, _write_size, erase_size, nand_addr, stuff))){
		printf("ERROR - [%s](buff_addr=0x%08lX, nand_addr=0x%08lX, write_size=0x%08lX, erase_size=0x%08lX)\r\n", __func__, buff_addr, nand_addr, write_size, erase_size);
		return -1;
	}
	return 0;
}

#define FIRMWARE_FEATURE_IVA 0x01

union ver_union {
	unsigned long   ul;     // version unsigned long format
	struct s_part {
		unsigned char   major;  // major version number
		unsigned char   minor;  // minor version number
		unsigned short  oem;    // oem version number
	} part;
} version;

union time_date {
	unsigned long   ul;     // time unsigned long format
	struct v_part {
		unsigned char   year;
		unsigned char   month;
		unsigned char   day;
		unsigned char   hour;
	} part_t;
};

struct fis_image_desc {
    char name[16];      // Null terminated name
    unsigned long flash_base;    // Address within FLASH of image
    unsigned long mem_base;      // Address in memory where it executes
    unsigned long size;          // Length of image
    unsigned long entry_point;   // Execution entry point
    unsigned long data_length;   // Length of actual data
    unsigned char _pad[256-(16+7*sizeof(unsigned long))];
    unsigned long desc_cksum;    // Checksum over image descriptor
    unsigned long file_cksum;    // Checksum over image data
};

typedef struct file_header_struct {
	struct fis_image_desc fis_info;
	unsigned long  	mtdno;          // for MTD using, to know which MTD
	union ver_union version;
} file_header_t;

typedef struct file_header_linker_struct {
	char	sourcefile[256];
	file_header_t	fileheader;
	struct 	file_header_linker_struct	*next;
} file_header_linker_t;

typedef struct firmware_header_struct {
	unsigned long	magiccode;		// file identifier
	union time_date	build_time;		// firmware build time
	unsigned char  featureflag;    // ex. iva_flag
	unsigned char  totalfileno;    // the firmware context include total file no.
	unsigned long   checksum;       // not include this header
	unsigned long   totallength;    // not include this header
	union ver_union version;
	unsigned short  headerlength;   // this header length
} firmware_header_t;

int get_ubl_info(int *majorVer, int *minorVer, int *cvVer, int *ubootIdx)
{
	int rcode = -1;
	return rcode;
}

int do_loadfw (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	char cmd[3];
	long size = 0;
	long data_size = 0;
	if( (argc < 2) || ( (argc < 3) && (strcmp(argv[1], "tftp") == 0))){
		printf ("Usage:\n%s\n", cmdtp->usage);
		return -1;
	}

	if((size = load_image(cmdtp, flag, argv[1], argc < 3 ? NULL : argv[2], CONFIG_SYS_LOAD_ADDR)) <= 0){
		return -1;
	}
	printf ("File size:%lu\n", size);

	char* FileBuffer = (char*)CONFIG_SYS_LOAD_ADDR;
	ulong FileBuffer_Len = size;
	firmware_header_t firmHead;
	ulong FileBuffer_ptr = 0;
	memset(&firmHead, 0, sizeof(firmware_header_t));
	firmHead.headerlength = sizeof(firmware_header_t);
	if((FileBuffer_Len - FileBuffer_ptr) < sizeof(firmware_header_t)){
		printf("Read firmware header fails!\r\n");
		return -1;
	}else{
		memcpy(&firmHead,&FileBuffer[FileBuffer_ptr], sizeof(firmware_header_t));
		FileBuffer_ptr += sizeof(firmware_header_t);
	}
#ifdef MAGICCODE
	if(firmHead.magiccode != MAGICCODE){
		printf("Magiccode error!!(0x%08lX), continue? <Y/N>:", firmHead.magiccode);
		if(get_line(NULL, cmd, sizeof(cmd), -1, "YyNn", NULL, "N") < 0) return -1;
		if((cmd[0] != 'Y') && (cmd[0] != 'y')) return -1;
	}
#endif
	printf("===============< Firmware Information >===============\r\n");
	printf("   MagicCode    : 0x%08lX\r\n", firmHead.magiccode);
	printf("   Feature Flag : 0x%02X\r\n", firmHead.featureflag);
	printf("   Total Files  : %u\r\n", firmHead.totalfileno);
	printf("   Total Length : %lu\r\n", firmHead.totallength);
	printf("   Version      : %d.%d.%d\r\n", firmHead.version.part.major, firmHead.version.part.minor, firmHead.version.part.oem);
	printf("   CheckSum     : 0x%08lX \r\n", firmHead.checksum);
	printf("=====================================================\r\n");

	//check Firmware Length
	data_size = size - sizeof(firmware_header_t);
	if(data_size != firmHead.totallength){
		printf("Firmware Length Error!(%lu)\r\n", data_size);
		return -1;
	}

#ifdef CONFIG_IVA_FLAG
	//check IVA Flag
	int iva_flag = 0;
	if(firmHead.featureflag & FIRMWARE_FEATURE_IVA){
		if(env_get_iva_flag(&iva_flag)){
			printf("Get IVA Flag error!!\r\n");
			return -1;
 		}else if(iva_flag == 0){
			printf("This module didn't support IVA function!!\r\n");
			return -1;
		}
	}
#endif

	//extract header and file content for each payload
	file_header_linker_t *curr;
	int i;
	ulong nand_base;
	ulong erase_size, write_size;
	ulong data_addr;
	ulong lCrcall, lCRC;

	//initial file header control block
	curr = (file_header_linker_t*)malloc(sizeof(file_header_linker_t));
	if ( curr == NULL ) {
		printf("firmware header malloc error !!\r\n");
		return -1;
	}
	memset(curr, 0, sizeof(file_header_linker_t));
	for(i=0;i<firmHead.totalfileno;i++)
	{
		if((FileBuffer_Len - FileBuffer_ptr) < sizeof(file_header_t)){
			printf("Read file header failed!\r\n");
			free(curr);
			return -1;
		}else{
			memcpy(&curr->fileheader, &FileBuffer[FileBuffer_ptr], sizeof(file_header_t));
			FileBuffer_ptr += sizeof(file_header_t);
		}

		printf("\r\n");
		printf("---------------< File info >---------------\r\n");
		printf("   Filename    : %s\r\n", curr->fileheader.fis_info.name);
		printf("   mtd_no      : %ld\r\n", curr->fileheader.mtdno);
		printf("   version     : %d.%d.%d\r\n", curr->fileheader.version.part.major, curr->fileheader.version.part.minor, curr->fileheader.version.part.oem);
		printf("   fis size    : %ld\r\n", curr->fileheader.fis_info.size);
		printf("   data size   : %ld\r\n", curr->fileheader.fis_info.data_length);
		printf("   flash_base  : 0x%08lX\r\n", curr->fileheader.fis_info.flash_base);
		printf("   mem_base    : 0x%08lX\r\n", curr->fileheader.fis_info.mem_base);
		printf("   entry_point : 0x%08lX\r\n", curr->fileheader.fis_info.entry_point);
		printf("   CheckSum    : 0x%08lX\r\n", curr->fileheader.fis_info.file_cksum);
		printf("-------------------------------------------\r\n");

		//cumulate file header checksum to firmware overall checksum
		lCrcall = crc32(lCrcall, (unsigned char *)&curr->fileheader, sizeof(file_header_t));

#ifdef UBL_FLASH
		if(strcmp(curr->fileheader.fis_info.name, "ubl") == 0){
			data_addr = (ulong)(&FileBuffer[FileBuffer_ptr]);
			nand_base = UBL_FLASH;
			erase_size = UBL_SIZE;
			write_size = curr->fileheader.fis_info.data_length;
			if(erase_write_flash(cmdtp, flag, data_addr, nand_base, erase_size, write_size, 0x00)) goto NEXT_FILE;
		}else
#endif
#ifdef UBOOT_FLASH
		if(strcmp(curr->fileheader.fis_info.name, "uboot") == 0){
			data_addr = (ulong)(&FileBuffer[FileBuffer_ptr]);
			nand_base = UBOOT_FLASH;
			erase_size = UBOOT_SIZE;
			write_size = curr->fileheader.fis_info.data_length;
			if(erase_write_flash(cmdtp, flag, data_addr, nand_base, erase_size, write_size, 0x00)) goto NEXT_FILE;
 #ifdef UBOOT2_FLASH
			data_addr = (ulong)(&FileBuffer[FileBuffer_ptr]);
			nand_base = UBOOT2_FLASH;
			erase_size = UBOOT2_SIZE;
			write_size = curr->fileheader.fis_info.data_length;
			if(erase_write_flash(cmdtp, flag, data_addr, nand_base, erase_size, write_size, 0x00)) goto NEXT_FILE;
 #endif
		}else
#endif
#ifdef KERNEL_FLASH
		if(strcmp(curr->fileheader.fis_info.name, "kernel") == 0){
			data_addr = (ulong)(&FileBuffer[FileBuffer_ptr]);
			nand_base = KERNEL_FLASH;
			erase_size = KERNEL_SIZE;
			write_size = curr->fileheader.fis_info.data_length;
			if(erase_write_flash(cmdtp, flag, data_addr, nand_base, erase_size, write_size, 0x00)) goto NEXT_FILE;
 #ifdef KERNEL2_FLASH
			data_addr = (ulong)(&FileBuffer[FileBuffer_ptr]);
			nand_base = KERNEL2_FLASH;
			erase_size = KERNEL2_SIZE;
			write_size = curr->fileheader.fis_info.data_length;
			if(erase_write_flash(cmdtp, flag, data_addr, nand_base, erase_size, write_size, 0x00)) goto NEXT_FILE;
 #endif
		}else
#endif
#ifdef ROOTFS_FLASH
		if(strcmp(curr->fileheader.fis_info.name, "rootfs") == 0){
			data_addr = (ulong)(&FileBuffer[FileBuffer_ptr]);
			nand_base = ROOTFS_FLASH;
			erase_size = ROOTFS_SIZE;
			write_size = curr->fileheader.fis_info.data_length;
			if(erase_write_flash(cmdtp, flag, data_addr, nand_base, erase_size, write_size, 0x00)) goto NEXT_FILE;
 #ifdef ROOTFS2_FLASH
			data_addr = (ulong)(&FileBuffer[FileBuffer_ptr]);
			nand_base = ROOTFS2_FLASH;
			erase_size = ROOTFS2_SIZE;
			write_size = curr->fileheader.fis_info.data_length;
			if(erase_write_flash(cmdtp, flag, data_addr, nand_base, erase_size, write_size, 0x00)) goto NEXT_FILE;
 #endif
		}else
#endif
#ifdef MPKERNEL_FLASH
		if(strcmp(curr->fileheader.fis_info.name, "mpkernel") == 0){
			data_addr = (ulong)(&FileBuffer[FileBuffer_ptr]);
			nand_base = MPKERNEL_FLASH;
			erase_size = MPKERNEL_SIZE;
			write_size = curr->fileheader.fis_info.data_length;
			if(erase_write_flash(cmdtp, flag, data_addr, nand_base, erase_size, write_size, 0x00)) goto NEXT_FILE;
		}else
#endif
#ifdef MPROOTFS_FLASH
		if(strcmp(curr->fileheader.fis_info.name, "mprootfs") == 0){
			data_addr = (ulong)(&FileBuffer[FileBuffer_ptr]);
			nand_base = MPROOTFS_FLASH;
			erase_size = MPROOTFS_SIZE;
			write_size = curr->fileheader.fis_info.data_length;
			if(erase_write_flash(cmdtp, flag, data_addr, nand_base, erase_size, write_size, 0x00)) goto NEXT_FILE;
		}else
#endif
#ifdef DATA1_FLASH
		if(strcmp(curr->fileheader.fis_info.name, "data1") == 0){
			data_addr = (ulong)(&FileBuffer[FileBuffer_ptr]);
			nand_base = DATA1_FLASH;
			erase_size = DATA1_SIZE;
			write_size = curr->fileheader.fis_info.data_length;
			if(erase_write_flash(cmdtp, flag, data_addr, nand_base, erase_size, write_size, 0x00)) goto NEXT_FILE;
		}else
#endif
#ifdef DATA2_FLASH
		if(strcmp(curr->fileheader.fis_info.name, "data2") == 0){
			data_addr = (ulong)(&FileBuffer[FileBuffer_ptr]);
			nand_base = DATA2_FLASH;
			erase_size = DATA2_SIZE;
			write_size = curr->fileheader.fis_info.data_length;
			if(erase_write_flash(cmdtp, flag, data_addr, nand_base, erase_size, write_size, 0x00)) goto NEXT_FILE;
		}else
#endif
#ifdef MPDATA_FLASH
		if(strcmp(curr->fileheader.fis_info.name, "mpdata") == 0){
			data_addr = (ulong)(&FileBuffer[FileBuffer_ptr]);
			nand_base = MPDATA_FLASH;
			erase_size = MPDATA_SIZE;
			write_size = curr->fileheader.fis_info.data_length;
			if(erase_write_flash(cmdtp, flag, data_addr, nand_base, erase_size, write_size, 0x00)) goto NEXT_FILE;
		}else
#endif
		{
			printf("Unsupported file %s, do nothing.\n", curr->fileheader.fis_info.name);
			goto NEXT_FILE;
		}

		//cumulate crc32 checksum
		lCRC = crc32(lCRC, (unsigned char *)(&FileBuffer[FileBuffer_ptr]), erase_size);
		printf("File CheckSum: 0x%08lX\r\n", lCRC);

		printf("Upgrade file %s successed.\n", curr->fileheader.fis_info.name);

NEXT_FILE:
		FileBuffer_ptr += curr->fileheader.fis_info.data_length;
	}
	free(curr);

//	if (lCrcall != firmHead.checksum){
		printf("Frimware CheckSum.[0x%08lX / 0x%08lX]\r\n", lCrcall, firmHead.checksum);
//		return -1;
//	}

	return 0;
}
U_BOOT_CMD(
	loadfw, 3, 0,	do_loadfw,
	"Load firmware",
	"Load firmware\n\
	loadfw [kermit/xymodem/tftp/mmc(dev[:part])][filename]"
);

int general_load (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[], ulong nand_base, ulong erase_size)
{
	ulong data_addr = CONFIG_SYS_LOAD_ADDR;
	long image_size = 0;
	if( (argc < 2) || ( (argc < 3) && ((strcmp(argv[1], "tftp") == 0) || (strncmp(argv[1], "mmc", 3) == 0)))){
		printf ("Usage:\n%s\n", cmdtp->usage);
		return -1;
	}
	if(argc >= 3)
		printf("===========>>>> %s %s %s\n", argv[0], argv[1], argv[2]);
	else
		printf("===========>>>> %s %s\n", argv[0], argv[1]);
	if(_clean_buffer(cmdtp, flag, data_addr, erase_size)) return -1;
	if((image_size = load_image(cmdtp, flag, argv[1], argc < 3 ? NULL : argv[2], data_addr)) <= 0) return -1;
	if(erase_write_flash(cmdtp, flag, data_addr, nand_base, erase_size, image_size, 0x00)) return -1;
	return 0;
}

#ifdef ROOTFS_FLASH
int do_loadfs (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	return general_load(cmdtp, flag, argc, argv, ROOTFS_FLASH, ROOTFS_SIZE);
}
U_BOOT_CMD(
	loadfs, 3, 0,	do_loadfs,
	"Load root filesystem",
	"Load root filesystem\n\
	loadfs [kermit/xymodem/tftp/mmc(dev[:part])][filename]"
);
#endif
#ifdef ROOTFS2_FLASH
int do_loadfs2 (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	return general_load(cmdtp, flag, argc, argv, ROOTFS2_FLASH, ROOTFS2_SIZE);
}
U_BOOT_CMD(
	loadfs2, 3, 0,	do_loadfs2,
	"Load root filesystem 2",
	"Load root filesystem 2\n\
	loadfs2 [kermit/xymodem/tftp/mmc(dev[:part])][filename]"
);
#endif
#ifdef MPROOTFS_FLASH
int do_loadmpfs (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	return general_load(cmdtp, flag, argc, argv, MPROOTFS_FLASH, MPROOTFS_SIZE);
}
U_BOOT_CMD(
	loadmpfs, 3, 0,	do_loadmpfs,
	"Load MP root filesystem",
	"Load MP root filesystem\n\
	loadmpfs [kermit/xymodem/tftp/mmc(dev[:part])][filename]"
);
#endif
#ifdef KERNEL_FLASH
int do_loadkernel (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	return general_load(cmdtp, flag, argc, argv, KERNEL_FLASH, KERNEL_SIZE);
}
U_BOOT_CMD(
	loadkl, 3, 0,	do_loadkernel,
	"Load kernel",
	"Load kernel\n\
	loadkl [kermit/xymodem/tftp/mmc(dev[:part])][filename]"
);
#endif
#ifdef KERNEL2_FLASH
int do_loadkernel2 (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	return general_load(cmdtp, flag, argc, argv, KERNEL2_FLASH, KERNEL2_SIZE);
}
U_BOOT_CMD(
	loadkl2, 3, 0,	do_loadkernel2,
	"Load kernel 2",
	"Load kernel 2\n\
	loadkl2 [kermit/xymodem/tftp/mmc(dev[:part])][filename]"
);
#endif
#ifdef MPKERNEL_FLASH
int do_loadmpkernel (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	return general_load(cmdtp, flag, argc, argv, MPKERNEL_FLASH, MPKERNEL_SIZE);
}
U_BOOT_CMD(
	loadmpkl, 3, 0,	do_loadmpkernel,
	"Load MP kernel",
	"Load MP kernel\n\
	loadmpkl [kermit/xymodem/tftp/mmc(dev[:part])][filename]"
);
#endif
#ifdef DATA1_FLASH
int do_loaddata1 (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	return general_load(cmdtp, flag, argc, argv, DATA1_FLASH, DATA1_SIZE);
}
U_BOOT_CMD(
	loadda1, 3, 0,	do_loaddata1,
	"Load data 1",
	"Load data 1\n\
	loadda1 [kermit/xymodem/tftp/mmc(dev[:part])][filename]"
);
#endif
#ifdef DATA2_FLASH
int do_loaddata2 (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	return general_load(cmdtp, flag, argc, argv, DATA2_FLASH, DATA2_SIZE);
}
U_BOOT_CMD(
	loadda2, 3, 0,	do_loaddata2,
	"Load data 2",
	"Load data 2\n\
	loadda2 [kermit/xymodem/tftp/mmc(dev[:part])][filename]"
);
#endif
#ifdef MPDATA_FLASH
int do_loadmpdata (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	return general_load(cmdtp, flag, argc, argv, MPDATA_FLASH, MPDATA_SIZE);
}
U_BOOT_CMD(
	loadmpda, 3, 0,	do_loadmpdata,
	"Load MP data",
	"Load MP data\n\
	loadmpda [kermit/xymodem/tftp/mmc(dev[:part])][filename]"
);
#endif
#ifdef RESERVE_FLASH
int do_loadreserve (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	return general_load(cmdtp, flag, argc, argv, RESERVE_FLASH, RESERVE_SIZE);
}
U_BOOT_CMD(
	loadrese, 3, 0,	do_loadreserve,
	"Load reserve",
	"Load reserve\n\
	loadrese [kermit/xymodem/tftp/mmc(dev[:part])][filename]"
);
#endif
#ifdef DSP1_FLASH
int do_loaddsp1 (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	return general_load(cmdtp, flag, argc, argv, DSP1_FLASH, DSP1_SIZE);
}
U_BOOT_CMD(
	loaddsp1, 3, 0,	do_loaddsp1,
	"Load DSP 1",
	"Load DSP 1\n\
	loaddsp1 [kermit/xymodem/tftp/mmc(dev[:part])][filename]"
);
#endif
#ifdef DSP2_FLASH
int do_loaddsp2 (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	return general_load(cmdtp, flag, argc, argv, DSP2_FLASH, DSP2_SIZE);
}
U_BOOT_CMD(
	loaddsp2, 3, 0,	do_loaddsp2,
	"Load DSP 2",
	"Load DSP 2\n\
	loaddsp2 [kermit/xymodem/tftp/mmc(dev[:part])][filename]"
);
#endif
#ifdef UBOOT_FLASH
int do_loaduboot (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	return general_load(cmdtp, flag, argc, argv, UBOOT_FLASH, UBOOT_SIZE);
}
U_BOOT_CMD(
	loadubt, 3, 0,	do_loaduboot,
	"Load u-boot",
	"Load u-boot\n\
	loadubt [kermit/xymodem/tftp/mmc(dev[:part])][filename]"
);
#endif
#ifdef UBOOT2_FLASH
int do_loaduboot_2 (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	return general_load(cmdtp, flag, argc, argv, UBOOT2_FLASH, UBOOT2_SIZE);
}
U_BOOT_CMD(
	loadubt2, 3, 0,	do_loaduboot_2,
	"Load second u-boot",
	"Load second u-boot\n\
	loadubt2 [kermit/xymodem/tftp/mmc(dev[:part])][filename]"
);
#endif
#ifdef UBL_FLASH
int do_loadubl(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	return general_load(cmdtp, flag, argc, argv, UBL_FLASH, UBL_SIZE);
}
U_BOOT_CMD(
	loadubl, 3, 0,	do_loadubl,
	"Load UBL",
	"Load UBL\n\
	loadubl [kermit/xymodem/tftp/mmc(dev[:part])][filename]"
);
#endif
#if 0
static int diag_run_ispnand(int parameter)
{
	int i, count = parameter;
	char tmp[5];
	char mmc[5];
	char *args[2];
	cmd_tbl_t cmd_tmp;

	if(get_line("Download image from mmc[0]* / tftp[1]:", tmp, 3, -1, "01", NULL, "0") < 0) return -1;
	if(tmp[0] == '0'){
		if(get_line("Enter mmc partition(>1):", tmp, sizeof(tmp), -1, str_number_dec, NULL, "1") < 0) return -1;
		sprintf(mmc, "mmc(0:%s)", tmp);
		args[1] = mmc;
	}else{
		args[1] = "tftp";
	}
	args[0] = "ispnand";
	for(i=0; i<count; i++){
		if(do_ispnand(&cmd_tmp, 0, 2, args) != 0) break;
	}
	return 0;
}

int do_ispnand (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	char *args[3];
	char *ptr= NULL;
	args[1] = argv[1];

	sys_led_RG_OFF();

#ifdef UBL_FLASH
	args[0] = "loadubl";
	ptr = getenv("ublfile");
	args[2] = ((ptr != NULL) ? ptr : D4_UBL_FILE_NAME);
	if(do_loadubl(cmdtp, flag, 3, args) != 0){
		printf("Error!! =====>>>> %s %s %s\n", args[0], args[1], args[2]);
		sys_led_R_ON();
		return -1;
	}
#endif
#ifdef UBOOT_FLASH
	args[0] = "loadubt";
	ptr = getenv("ubtfile");
	args[2] = ((ptr != NULL) ? ptr : D4_UBOOT_FILE_NAME);
	if(do_loaduboot(cmdtp, flag, 3, args) != 0){
		printf("Error!! =====>>>> %s %s %s\n", args[0], args[1], args[2]);
		sys_led_R_ON();
		return -1;
	}
#endif
#ifdef KERNEL_FLASH
	args[0] = "loadkl";
	ptr = getenv("kernelfile");
	args[2] = ((ptr != NULL) ? ptr : D4_KERNEL_FILE_NAME);
	if(do_loadkernel(cmdtp, flag, 3, args) != 0){
		printf("Error!! =====>>>> %s %s %s\n", args[0], args[1], args[2]);
		sys_led_R_ON();
		return -1;
	}
#endif
#ifdef KERNEL2_FLASH
	args[0] = "loadkl2";
	ptr = getenv("kernelfile");
	args[2] = ((ptr != NULL) ? ptr : D4_KERNEL_FILE_NAME);
	if(do_loadkernel2(cmdtp, flag, 3, args) != 0){
		printf("Error!! =====>>>> %s %s %s\n", args[0], args[1], args[2]);
		sys_led_R_ON();
		return -1;
	}
#endif
#ifdef MPKERNEL_FLASH
	args[0] = "loadmpkl";
	ptr = getenv("mpkernelfile");
	args[2] = ((ptr != NULL) ? ptr : D4_MPKERNEL_FILE_NAME);
	if(do_loadmpkernel(cmdtp, flag, 3, args) != 0){
		printf("Error!! =====>>>> %s %s %s\n", args[0], args[1], args[2]);
		sys_led_R_ON();
		return -1;
	}
#endif
#ifdef ROOTFS_FLASH
	args[0] = "loadfs";
	ptr = getenv("fsfile");
	args[2] = ((ptr != NULL) ? ptr : D4_FS_FILE_NAME);
	if(do_loadfs(cmdtp, flag, 3, args) != 0){
		printf("Error!! =====>>>> %s %s %s\n", args[0], args[1], args[2]);
		sys_led_R_ON();
		return -1;
	}
#endif
#ifdef ROOTFS2_FLASH
	args[0] = "loadfs2";
	ptr = getenv("fsfile");
	args[2] = ((ptr != NULL) ? ptr : D4_FS_FILE_NAME);
	if(do_loadfs2(cmdtp, flag, 3, args) != 0){
		printf("Error!! =====>>>> %s %s %s\n", args[0], args[1], args[2]);
		sys_led_R_ON();
		return -1;
	}
#endif
#ifdef MPROOTFS_FLASH
	args[0] = "loadmpfs";
	ptr = getenv("mpfsfile");
	args[2] = ((ptr != NULL) ? ptr : D4_MPFS_FILE_NAME);
	if(do_loadmpfs(cmdtp, flag, 3, args) != 0){
		printf("Error!! =====>>>> %s %s %s\n", args[0], args[1], args[2]);
		sys_led_R_ON();
		return -1;
	}
#endif
#ifdef ENV1_FLASH
	if(_erase_flash(cmdtp, flag, ENV1_FLASH, ENV1_SIZE)){
		printf("Error!! Erase nand flash at 0x%08X size:0x%08X\n", ENV1_FLASH, ENV1_SIZE);
		sys_led_R_ON();
		return -1;
	}
#endif
#ifdef ENV2_FLASH
	if(_erase_flash(cmdtp, flag, ENV2_FLASH, ENV2_SIZE)){
		printf("Error!! Erase nand flash at 0x%08X size:0x%08X\n", ENV2_FLASH, ENV2_SIZE);
		sys_led_R_ON();
		return -1;
	}
#endif
#ifdef DATA1_FLASH
	if(_erase_flash(cmdtp, flag, DATA1_FLASH, DATA1_SIZE)){
		printf("Error!! Erase nand flash at 0x%08X size:0x%08X\n", DATA1_FLASH, DATA1_SIZE);
		sys_led_R_ON();
		return -1;
	}
#endif
#ifdef DATA2_FLASH
	if(_erase_flash(cmdtp, flag, DATA2_FLASH, DATA2_SIZE)){
		printf("Error!! Erase nand flash at 0x%08X size:0x%08X\n", DATA2_FLASH, DATA2_SIZE);
		sys_led_R_ON();
		return -1;
	}
#endif
#ifdef MPDATA_FLASH
	if(_erase_flash(cmdtp, flag, MPDATA_FLASH, MPDATA_SIZE)){
		printf("Error!! Erase nand flash at 0x%08X size:0x%08X\n", MPDATA_FLASH, MPDATA_SIZE);
		sys_led_R_ON();
		return -1;
	}
#endif

	sys_led_G_ON();

	return 0;
}
#endif

#ifdef ISP_NAND
char* IsCommentLine(const char * s, const char * p) //comment line
{
	for(; ; --p){
		if ((p <= s) || (*p == '\n') || (*p == '\r')) break;
		if (*p == ';') return (char *) p;
	}
	return NULL;
}

char* strend(const char * s)
{
	char * p = s;
	for(; ; ++p){
		if (*p == '\0') return NULL;
		if ((*p == ' ') || (*p == ';') || (*p == '\n') || (*p == '\r')) break;
	}
	return (char *) p;
}

int parseIniValue(char* dest, char* src, const char* section, const char* item)
{
	char *ptr, *ptr1, *ptr2;
	char tmp[64];

	if(dest == NULL || src == NULL || section == NULL || item == NULL) return -1;
	sprintf(tmp, "[%s]", section);
	if((ptr = strstr(src, tmp)) != NULL){
		if(IsCommentLine(src, ptr)) return -1;
		ptr += strlen(tmp);
		sprintf(tmp, "%s", item);
		if((ptr1 = strstr(ptr, tmp)) != NULL){
			if((ptr2 = strchr(ptr+1, '[')) != NULL){
				if(ptr2 < ptr1) return -1; //not the same section.
			}
			if(IsCommentLine(ptr, ptr1)) return -1;
			ptr = ptr1 + strlen(tmp);
			if((ptr1 = strchr(ptr, '=')) == NULL) return -1;
			ptr1++;
			while((*ptr1 == ' ') || (*ptr1 == '\t')){
				ptr1++;
				if(ptr1 == '\0') return -1;
			}
			if((ptr2 = strend(ptr1)) != NULL){
				strncpy(dest, ptr1, ptr2 - ptr1);
				dest[ptr2 - ptr1] = '\0';
			}else{
				strcpy(dest, ptr1);
			}
			return strlen(dest);
		}else{
			return -1;
		}
	}else{
		return -1;
	}
}

int do_ispnand_from_mmc(char* buff, int size)
{
	char *args[5];
	char buffAddr[11];//0xffffffff
	char tmp[64];
	char *ptr, *ptr1, *ptr2;
	cmd_tbl_t cmd_tmp;
	bool do_saveenv = false;
	int count = 0;

	sys_led_RG_OFF();

	if(buff != NULL){
		memset(tmp, 0, sizeof(tmp));
		if(parseIniValue(tmp, buff, "mode", "bios") > 0){
			if(*tmp == '1'){
				memset(tmp, 0, sizeof(tmp));
				if(parseIniValue(tmp, buff, "file", "bios") > 0){
					printf("upgrade bios = %s\n", tmp);
					args[0] = "loadfw";
					args[1] = "mmc(0:2)";
					args[2] = tmp;
					if(do_loadfw(&cmd_tmp, 0, 3, args) != 0){
						printf("Error!! =====>>>> %s %s %s\n", args[0], args[1], args[2]);
						sys_led_R_ON();
						return -1;
					}
				}
			}
		}
		memset(tmp, 0, sizeof(tmp));
		if(parseIniValue(tmp, buff, "mode", "mp") > 0){
			if(*tmp == '1'){
				memset(tmp, 0, sizeof(tmp));
				if(parseIniValue(tmp, buff, "file", "mp") > 0){
					printf("upgrade mp = %s\n", tmp);
					args[0] = "loadfw";
					args[1] = "mmc(0:2)";
					args[2] = tmp;
					if(do_loadfw(&cmd_tmp, 0, 3, args) != 0){
						printf("Error!! =====>>>> %s %s %s\n", args[0], args[1], args[2]);
						sys_led_R_ON();
						return -1;
					}
				}
			}
		}
		memset(tmp, 0, sizeof(tmp));
		if(parseIniValue(tmp, buff, "mode", "fw") > 0){
			if(*tmp == '1'){
				memset(tmp, 0, sizeof(tmp));
				if(parseIniValue(tmp, buff, "file", "fw") > 0){
					printf("upgrade fw = %s\n", tmp);
					args[0] = "loadfw";
					args[1] = "mmc(0:2)";
					args[2] = tmp;
					if(do_loadfw(&cmd_tmp, 0, 3, args) != 0){
						printf("Error!! =====>>>> %s %s %s\n", args[0], args[1], args[2]);
						sys_led_R_ON();
						return -1;
					}
				}
			}
		}
#ifdef UBL_FLASH
		memset(tmp, 0, sizeof(tmp));
		if(parseIniValue(tmp, buff, "mode", "ubl") > 0){
			if(*tmp == '1'){
				memset(tmp, 0, sizeof(tmp));
				if(parseIniValue(tmp, buff, "file", "ubl") > 0){
					printf("upgrade ubl = %s\n", tmp);
					args[0] = "loadubl";
					args[1] = "mmc(0:2)";
					args[2] = tmp;
					if(do_loadubl(&cmd_tmp, 0, 3, args) != 0){
						printf("Error!! =====>>>> %s %s %s\n", args[0], args[1], args[2]);
						sys_led_R_ON();
						return -1;
					}
				}
			}else
			if(*tmp == '2'){
				memset(tmp, 0, sizeof(tmp));
				if(parseIniValue(tmp, buff, "file", "ubl") > 0){
					printf("set ublfile = %s\n", tmp);
					setenv ("ublfile", tmp);
					do_saveenv = true;
				}
			}else
			if(*tmp == '3'){
				if(_erase_flash(&cmd_tmp, 0, UBL_FLASH, UBL_SIZE)){
					printf("Error!! Erase nand flash at 0x%08X size:0x%08X\n", UBL_FLASH, UBL_SIZE);
					sys_led_R_ON();
					return -1;
				}
			}
		}
#endif
#ifdef UBOOT_FLASH
		memset(tmp, 0, sizeof(tmp));
		if(parseIniValue(tmp, buff, "mode", "uboot") > 0){
			if(*tmp == '1'){
				memset(tmp, 0, sizeof(tmp));
				if(parseIniValue(tmp, buff, "file", "uboot") > 0){
					printf("upgrade uboot = %s\n", tmp);
					args[0] = "loadubt";
					args[1] = "mmc(0:2)";
					args[2] = tmp;
					if(do_loaduboot(&cmd_tmp, 0, 3, args) != 0){
						printf("Error!! =====>>>> %s %s %s\n", args[0], args[1], args[2]);
						sys_led_R_ON();
						return -1;
					}
				}
			}else
			if(*tmp == '2'){
				memset(tmp, 0, sizeof(tmp));
				if(parseIniValue(tmp, buff, "file", "uboot") > 0){
					printf("set ubtfile = %s\n", tmp);
					setenv ("ubtfile", tmp);
					do_saveenv = true;
				}
			}else
			if(*tmp == '3'){
				if(_erase_flash(&cmd_tmp, 0, UBOOT_FLASH, UBOOT_SIZE)){
					printf("Error!! Erase nand flash at 0x%08X size:0x%08X\n", UBOOT_FLASH, UBOOT_SIZE);
					sys_led_R_ON();
					return -1;
				}
			}
		}
#endif
#ifdef KERNEL_FLASH
		memset(tmp, 0, sizeof(tmp));
		if(parseIniValue(tmp, buff, "mode", "kernel") > 0){
			if(*tmp == '1'){
				memset(tmp, 0, sizeof(tmp));
				if(parseIniValue(tmp, buff, "file", "kernel") > 0){
					printf("upgrade kernel = %s\n", tmp);
					args[0] = "loadkl";
					args[1] = "mmc(0:2)";
					args[2] = tmp;
					if(do_loadkernel(&cmd_tmp, 0, 3, args) != 0){
						printf("Error!! =====>>>> %s %s %s\n", args[0], args[1], args[2]);
						sys_led_R_ON();
						return -1;
					}
				}
			}else
			if(*tmp == '2'){
				memset(tmp, 0, sizeof(tmp));
				if(parseIniValue(tmp, buff, "file", "kernel") > 0){
					printf("set kernelfile = %s\n", tmp);
					setenv ("kernelfile", tmp);
					do_saveenv = true;
				}
			}else
			if(*tmp == '3'){
				if(_erase_flash(&cmd_tmp, 0, KERNEL_FLASH, KERNEL_SIZE)){
					printf("Error!! Erase nand flash at 0x%08X size:0x%08X\n", KERNEL_FLASH, KERNEL_SIZE);
					sys_led_R_ON();
					return -1;
				}
			}
		}
#endif
#ifdef ROOTFS_FLASH
		memset(tmp, 0, sizeof(tmp));
		if(parseIniValue(tmp, buff, "mode", "rootfs") > 0){
			if(*tmp == '1'){
				memset(tmp, 0, sizeof(tmp));
				if(parseIniValue(tmp, buff, "file", "rootfs") > 0){
					printf("upgrade rootfs = %s\n", tmp);
					args[0] = "loadfs";
					args[1] = "mmc(0:2)";
					args[2] = tmp;
					if(do_loadfs(&cmd_tmp, 0, 3, args) != 0){
						printf("Error!! =====>>>> %s %s %s\n", args[0], args[1], args[2]);
						sys_led_R_ON();
						return -1;
					}
				}
			}else
			if(*tmp == '2'){
				memset(tmp, 0, sizeof(tmp));
				if(parseIniValue(tmp, buff, "file", "rootfs") > 0){
					printf("set fsfile = %s\n", tmp);
					setenv ("fsfile", tmp);
					do_saveenv = true;
				}
			}else
			if(*tmp == '3'){
				if(_erase_flash(&cmd_tmp, 0, ROOTFS_FLASH, ROOTFS_SIZE)){
					printf("Error!! Erase nand flash at 0x%08X size:0x%08X\n", ROOTFS_FLASH, ROOTFS_SIZE);
					sys_led_R_ON();
					return -1;
				}
			}
		}
#endif
#ifdef KERNEL2_FLASH
		memset(tmp, 0, sizeof(tmp));
		if(parseIniValue(tmp, buff, "mode", "kernel2") > 0){
			if(*tmp == '1'){
				memset(tmp, 0, sizeof(tmp));
				if(parseIniValue(tmp, buff, "file", "kernel2") > 0){
					printf("upgrade kernel2 = %s\n", tmp);
					args[0] = "loadkl2";
					args[1] = "mmc(0:2)";
					args[2] = tmp;
					if(do_loadkernel2(&cmd_tmp, 0, 3, args) != 0){
						printf("Error!! =====>>>> %s %s %s\n", args[0], args[1], args[2]);
						sys_led_R_ON();
						return -1;
					}
				}
			}else
			if(*tmp == '2'){
				memset(tmp, 0, sizeof(tmp));
				if(parseIniValue(tmp, buff, "file", "kernel2") > 0){
					printf("set kernelfile = %s\n", tmp);
					setenv ("kernelfile", tmp);
					do_saveenv = true;
				}
			}else
			if(*tmp == '3'){
				if(_erase_flash(&cmd_tmp, 0, KERNEL2_FLASH, KERNEL2_SIZE)){
					printf("Error!! Erase nand flash at 0x%08X size:0x%08X\n", KERNEL2_FLASH, KERNEL2_SIZE);
					sys_led_R_ON();
					return -1;
				}
			}
		}
#endif
#ifdef ROOTFS2_FLASH
		memset(tmp, 0, sizeof(tmp));
		if(parseIniValue(tmp, buff, "mode", "rootfs2") > 0){
			if(*tmp == '1'){
				memset(tmp, 0, sizeof(tmp));
				if(parseIniValue(tmp, buff, "file", "rootfs2") > 0){
					printf("upgrade rootfs2 = %s\n", tmp);
					args[0] = "loadfs2";
					args[1] = "mmc(0:2)";
					args[2] = tmp;
					if(do_loadfs2(&cmd_tmp, 0, 3, args) != 0){
						printf("Error!! =====>>>> %s %s %s\n", args[0], args[1], args[2]);
						sys_led_R_ON();
						return -1;
					}
				}
			}else
			if(*tmp == '2'){
				memset(tmp, 0, sizeof(tmp));
				if(parseIniValue(tmp, buff, "file", "rootfs2") > 0){
					printf("set fsfile = %s\n", tmp);
					setenv ("fsfile", tmp);
					do_saveenv = true;
				}
			}else
			if(*tmp == '3'){
				if(_erase_flash(&cmd_tmp, 0, ROOTFS2_FLASH, ROOTFS2_SIZE)){
					printf("Error!! Erase nand flash at 0x%08X size:0x%08X\n", ROOTFS2_FLASH, ROOTFS2_SIZE);
					sys_led_R_ON();
					return -1;
				}
			}
		}
#endif
#ifdef MPKERNEL_FLASH
		memset(tmp, 0, sizeof(tmp));
		if(parseIniValue(tmp, buff, "mode", "mpkernel") > 0){
			if(*tmp == '1'){
				memset(tmp, 0, sizeof(tmp));
				if(parseIniValue(tmp, buff, "file", "mpkernel") > 0){
					printf("upgrade mpkernel = %s\n", tmp);
					args[0] = "loadmpkl";
					args[1] = "mmc(0:2)";
					args[2] = tmp;
					if(do_loadmpkernel(&cmd_tmp, 0, 3, args) != 0){
						printf("Error!! =====>>>> %s %s %s\n", args[0], args[1], args[2]);
						sys_led_R_ON();
						return -1;
					}
				}
			}else
			if(*tmp == '2'){
				memset(tmp, 0, sizeof(tmp));
				if(parseIniValue(tmp, buff, "file", "mpkernel") > 0){
					printf("set mpkernelfile = %s\n", tmp);
					setenv ("mpkernelfile", tmp);
					do_saveenv = true;
				}
			}else
			if(*tmp == '3'){
				if(_erase_flash(&cmd_tmp, 0, MPKERNEL_FLASH, MPKERNEL_SIZE)){
					printf("Error!! Erase nand flash at 0x%08X size:0x%08X\n", MPKERNEL_FLASH, MPKERNEL_SIZE);
					sys_led_R_ON();
					return -1;
				}
			}
		}
#endif
#ifdef MPROOTFS_FLASH
		memset(tmp, 0, sizeof(tmp));
		if(parseIniValue(tmp, buff, "mode", "mprootfs") > 0){
			if(*tmp == '1'){
				memset(tmp, 0, sizeof(tmp));
				if(parseIniValue(tmp, buff, "file", "mprootfs") > 0){
					printf("upgrade mprootfs = %s\n", tmp);
					args[0] = "loadmpfs";
					args[1] = "mmc(0:2)";
					args[2] = tmp;
					if(do_loadmpfs(&cmd_tmp, 0, 3, args) != 0){
						printf("Error!! =====>>>> %s %s %s\n", args[0], args[1], args[2]);
						sys_led_R_ON();
						return -1;
					}
				}
			}else
			if(*tmp == '2'){
				memset(tmp, 0, sizeof(tmp));
				if(parseIniValue(tmp, buff, "file", "mprootfs") > 0){
					printf("set mpfsfile = %s\n", tmp);
					setenv ("mpfsfile", tmp);
					do_saveenv = true;
				}
			}else
			if(*tmp == '3'){
				if(_erase_flash(&cmd_tmp, 0, MPROOTFS_FLASH, MPROOTFS_SIZE)){
					printf("Error!! Erase nand flash at 0x%08X size:0x%08X\n", MPROOTFS_FLASH, MPROOTFS_SIZE);
					sys_led_R_ON();
					return -1;
				}
			}
		}
#endif
#ifdef ENV1_FLASH
		memset(tmp, 0, sizeof(tmp));
		if(parseIniValue(tmp, buff, "mode", "env") > 0){
			if(*tmp == '3'){
				printf("Erase environment\n");
				if(_erase_flash(&cmd_tmp, 0, ENV1_FLASH, ENV1_SIZE)){
					printf("Error!! Erase nand flash at 0x%08X size:0x%08X\n", ENV1_FLASH, ENV1_SIZE);
					sys_led_R_ON();
					return -1;
				}
			}
		}
#endif
#ifdef ENV2_FLASH
		memset(tmp, 0, sizeof(tmp));
		if(parseIniValue(tmp, buff, "mode", "env2") > 0){
			if(*tmp == '3'){
				printf("Erase environment2\n");
				if(_erase_flash(&cmd_tmp, 0, ENV2_FLASH, ENV2_SIZE)){
					printf("Error!! Erase nand flash at 0x%08X size:0x%08X\n", ENV2_FLASH, ENV2_SIZE);
					sys_led_R_ON();
					return -1;
				}
			}
		}
#endif
#ifdef DATA1_FLASH
		memset(tmp, 0, sizeof(tmp));
		if(parseIniValue(tmp, buff, "mode", "data1") > 0){
			if(*tmp == '3'){
				printf("Erase data1\n");
				if(_erase_flash(&cmd_tmp, 0, DATA1_FLASH, DATA1_SIZE)){
					printf("Error!! Erase nand flash at 0x%08X size:0x%08X\n", DATA1_FLASH, DATA1_SIZE);
					sys_led_R_ON();
					return -1;
				}
			}
		}
#endif
#ifdef DATA2_FLASH
		memset(tmp, 0, sizeof(tmp));
		if(parseIniValue(tmp, buff, "mode", "data2") > 0){
			if(*tmp == '3'){
				printf("Erase data2\n");
				if(_erase_flash(&cmd_tmp, 0, DATA2_FLASH, DATA2_SIZE)){
					printf("Error!! Erase nand flash at 0x%08X size:0x%08X\n", DATA2_FLASH, DATA2_SIZE);
					sys_led_R_ON();
					return -1;
				}
			}
		}
#endif
#ifdef MPDATA_FLASH
		memset(tmp, 0, sizeof(tmp));
		if(parseIniValue(tmp, buff, "mode", "mpdata") > 0){
			if(*tmp == '3'){
				printf("Erase MP data\n");
				if(_erase_flash(&cmd_tmp, 0, MPDATA_FLASH, MPDATA_SIZE)){
					printf("Error!! Erase nand flash at 0x%08X size:0x%08X\n", MPDATA_FLASH, MPDATA_SIZE);
					sys_led_R_ON();
					return -1;
				}
			}
		}
#endif
		if(do_saveenv) saveenv();
	}
	sys_led_G_ON();

	return 0;
}

#define ISP_FILE_NAME "isp.ini"
int ispnand_from_mmc(int mmc_part, char* file)
{
	char *args[5];
	char buffAddr[11];//0xffffffff
	char buff[1024];
	char tmp[64];
	char mmc[5];
	cmd_tbl_t cmd_tmp;
	int size = 0;
	int i = 0;
	int count = 1;
	int enable = 1;
	int cleanflash = 0;
	int scrubflash = 0;
	int delay = 1;

	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));
	sprintf(buffAddr, "0x%08x", buff);

	sprintf(mmc, "0:%d", mmc_part);

	args[0] = "fatload";
	args[1] = "mmc";
	args[2] = mmc;
	args[3] = buffAddr;
	args[4] = file;
	if (do_fat_fsload(&cmd_tmp, 0, 5, args) == 0) {
		if((size = (int)simple_strtoul(getenv("filesize"), NULL, 16)) <= 0){
			return DIAG_ERROR;
		}

		memset(tmp, 0, sizeof(tmp));
		if(parseIniValue(tmp, buff, "ctrl", "enable") > 0){
			enable = (int)simple_strtoul(tmp, NULL, 10);
			printf("ctrl_enable = %d\n", enable);
			if(enable == 0) return DIAG_ERROR;
		}

		memset(tmp, 0, sizeof(tmp));
		if(parseIniValue(tmp, buff, "ctrl", "delay") > 0){
			delay = (int)simple_strtoul(tmp, NULL, 10);
			printf("ctrl_delay = %d\n", delay);
		}

		memset(tmp, 0, sizeof(tmp));
		get_line("\n Detected ISP setting, Run ISP? <Y/n>:", tmp, sizeof(tmp), delay, "YyNn", NULL, "Y");
		if((tmp[0] != 'Y') && (tmp[0] != 'y')) return DIAG_USER_ABORT;
		printf("\n");

		memset(tmp, 0, sizeof(tmp));
		if(parseIniValue(tmp, buff, "ctrl", "cleanflash") > 0){
			cleanflash = (int)simple_strtoul(tmp, NULL, 10);
			printf("ctrl_cleanflash = %d\n", cleanflash);
			if(cleanflash == 1){	/* Clean whole flash */
				if(_erase_flash_whole(&cmd_tmp, 0)) return DIAG_ERROR;
			}else
			if(cleanflash == 2){	/* Keep BIOS */
				if(_erase_flash(&cmd_tmp, 0, (UBOOT_FLASH + UBOOT_SIZE), 0)) return DIAG_ERROR;
			}
		}

		memset(tmp, 0, sizeof(tmp));
		if(parseIniValue(tmp, buff, "ctrl", "scrubflash") > 0){
			scrubflash = (int)simple_strtoul(tmp, NULL, 10);
			printf("ctrl_scrubflash = %d\n", scrubflash);
			if(scrubflash == 1){	/* Scrub whole flash */
				if(_scrub_flash_whole(&cmd_tmp, 0, 1)) return DIAG_ERROR;
			}else
			if(scrubflash == 2){	/* Keep BIOS */
				if(_scrub_flash(&cmd_tmp, 0, (UBOOT_FLASH + UBOOT_SIZE), 0, 1)) return DIAG_ERROR;
			}
		}

		memset(tmp, 0, sizeof(tmp));
		if(parseIniValue(tmp, buff, "test", "count") > 0){
			count = (int)simple_strtoul(tmp, NULL, 10);
			printf("test_count = %d\n", count);
		}

		hw_watchdog_op(HWWD_OFF);

		for(i=0; i<count; i++){
			if(do_ispnand_from_mmc(buff, size) != 0) break;
		}

		hw_watchdog_op(HWWD_ON);

	}else{
		return DIAG_FILE_NO_FIND;
	}
	return DIAG_OK;
}

int do_ispnandsd(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	char tmp[64];
	char file = NULL;
	int mmc_part = 1;

	if(argc == 1){
		if(get_line("Enter mmc partition:", tmp, sizeof(tmp), -1, str_number_dec, NULL, NULL) < 0) return DIAG_ERROR;
		mmc_part = (int)simple_strtoul(tmp, NULL, 10);
		if(get_line("File Name("ISP_FILE_NAME"):", tmp, sizeof(tmp), -1, NULL, NULL, ISP_FILE_NAME) < 0 ) return DIAG_ERROR;
		file = tmp;
	}else if(argc == 2){
		if(get_line("Enter mmc partition:", tmp, sizeof(tmp), -1, str_number_dec, NULL, NULL) < 0) return DIAG_ERROR;
		mmc_part = (int)simple_strtoul(tmp, NULL, 10);
	}else if(argc >= 3){
		mmc_part = (int)simple_strtoul(argv[1], NULL, 10);
		file = argv[2];
	}
	if(SD_init()) return DIAG_ERROR;
	return ispnand_from_mmc(mmc_part, file);
}
U_BOOT_CMD(
	ispnandsd, 3, 0,	do_ispnandsd,
	"In System Program Nand Flash(SD).",
	"In System Program Nand Flash(SD).\n\
	ispnandsd [mmc_part][filename]"
);

static int diag_ispnand_from_mmc(int parameter)
{
	int ret = DIAG_OK;
	char tmp[64];
	int mmc_part = 1;

	if(get_line("Enter mmc partition:", tmp, sizeof(tmp), -1, str_number_dec, NULL, NULL) < 0) return DIAG_ERROR;
	mmc_part = (int)simple_strtoul(tmp, NULL, 10);
	if(get_line("File Name("ISP_FILE_NAME"):", tmp, sizeof(tmp), -1, NULL, NULL, ISP_FILE_NAME) < 0 ) return DIAG_ERROR;
	if(SD_init()) return DIAG_ERROR;
	ret = ispnand_from_mmc(mmc_part, tmp);
	return ret;
}
#endif

int mmcboot_processor()
{
	int ret = DIAG_OK;
#ifdef ISP_NAND
	int i = 0;
	if(SD_init()) return DIAG_ERROR;

	for(i=1; i<=2; i++){
		ret = ispnand_from_mmc(i, ISP_FILE_NAME);
		if(ret == DIAG_FILE_NO_FIND) continue;
		else break;
	}
#endif
	return ret;
}

//MP Flag Menu==================================================================
static int MM_Set_MP_Flag(int parameter);
static int MM_Get_MP_Flag(int parameter);
static int MM_Set_MP_Flag_Manual(int parameter);

int Set_MP_Flag(int mpFlag)
{

#if CONFIG_SYS_I2C_EEPROM

	if (EEPROM_SetMpFlag(mpFlag) < 0)
	{
		printf("Set MP Flag to EEPROM Failed!!\n");
		return DIAG_ERROR;
	}
	else
	{
		EEPROM_GetMpFlag(&g_mp_flag);
	}

#else

	if (env_set_mp_flag(mpFlag) < 0)
	{
		printf("Set MP Flag to flash Failed!!\n");
		return DIAG_ERROR;
	}

#endif

	return 0;
}

int Get_MP_Flag(int *mpFlag)
{

#if CONFIG_SYS_I2C_EEPROM

	if (EEPROM_GetMpFlag(&g_mp_flag) < 0)
	{
		*mpFlag = CONFIG_MP_FLAG;
		printf("Get MP Flag from EEPROM Failed!!, use default vlaue.\n");
		return -1;
	}

	*mpFlag = g_mp_flag;

#else

	if (env_get_mp_flag(mpFlag) < 0)
	{
		*mpFlag = CONFIG_MP_FLAG;
		printf("Get MP Flag from FLASH Failed!!, use default vlaue.\n");
		return -1;
	}

#endif

	return 0;
}

char* Get_MP_Flag_String(int mpFlag)
{
	static char buf[32];
	switch(mpFlag){
		case CONFIG_MP_FLAG_T0_BIOS: sprintf(buf, "%s", "T0"); break;
		case CONFIG_MP_FLAG_T1_BIOS: sprintf(buf, "%s", "T1-BIOS"); break;
		case CONFIG_MP_FLAG_T1_MP: sprintf(buf, "%s", "T1"); break;
		case CONFIG_MP_FLAG_T2_MP: sprintf(buf, "%s", "T2"); break;
		case CONFIG_MP_FLAG_T2_MP_ERR: sprintf(buf, "%s", "T2-Dump"); break;
		case CONFIG_MP_FLAG_T3_BIOS: sprintf(buf, "%s", "T3"); break;
		case CONFIG_MP_FLAG_FIRMWARE: sprintf(buf, "%s", "FW"); break;
		case CONFIG_MP_FLAG_T2_T_MP: sprintf(buf, "%s", "EOT"); break;
		case CONFIG_MP_FLAG_T2_T_MP_ERR: sprintf(buf, "%s", "EOT-Dump"); break;
		default: sprintf(buf, "%s", "Unknown"); break;
	}
	return buf;
}

static int MM_Set_MP_Flag(int parameter)
{
	printf("Set MP Flag = 0x%03x (%s)\n", parameter, Get_MP_Flag_String(parameter));

	if (Set_MP_Flag(parameter) < 0)
	{
		printf("Set MP Flag Failed!!\n");
		return -1;
	}

	return 0;
}

static int MM_Get_MP_Flag(int parameter)
{
	int rs = 0;

	if (Get_MP_Flag(&rs) < 0)
	{
		printf("Get MP Flag Failed!!\n");
		return -1;
	}

	printf("Get MP Flag = 0x%03x (%s)\n", rs, Get_MP_Flag_String(rs));
	return 0;
}

static int MM_Set_MP_Flag_Manual(int parameter)
{
	int data;
	char tmp[5];

	if (get_line("MP Flag:0x", tmp, sizeof(tmp), -1, NULL, NULL, NULL) <= 0 )
	{
		return DIAG_ERROR;
	}

	data = (int)simple_strtoul(tmp, NULL, 16);
	printf("Set MP Flag = 0x%03x (%s)\n", data, Get_MP_Flag_String(data));

	if (Set_MP_Flag(data) < 0)
	{
		printf("Set MP Flag Failed!!\n");
		return -1;
	}

	return 0;
}

//MP Config Menu======================================================
#ifdef CONFIG_BURNIN_TIME
static int MM_Get_Burnin_Time(int parameter);
static int MM_Set_Burnin_Time(int parameter);
#endif
#ifdef CONFIG_T2_BOOTUP_COUNTER
static int MM_Get_T2_Bootup_Counter(int parameter);
static int MM_Set_T2_Bootup_Counter(int parameter);
#endif
#ifdef CONFIG_SERIAL_NUMBER
static int MM_Get_Serial_Number(int parameter);
static int MM_Set_Serial_Number(int parameter);
#endif
#ifdef CONFIG_DUT_ID
static int MM_Get_DUT_Id(int parameter);
static int MM_Set_DUT_Id(int parameter);
#endif
#ifdef CONFIG_FW_RECOVERY
static int MM_Get_FW_Recover(int parameter);
static int MM_Set_FW_Recover(int parameter);
#endif
#ifdef CONFIG_IVA_FLAG
static int MM_Get_IVA_Flag(int parameter);
static int MM_Set_IVA_Flag(int parameter);
#endif
#ifdef CONFIG_DUT_MODEL
static int MM_Get_Model(int parameter);
static int MM_Set_Model(int parameter);
#endif
#ifdef CONFIG_DUT_HOST
static int MM_Get_Host(int parameter);
static int MM_Set_Host(int parameter);
#endif
#ifdef CONFIG_DUT_DESC
static int MM_Get_Desc(int parameter);
static int MM_Set_Desc(int parameter);
#endif
static DiagMenuTableStruct MPConfigMenuTable[] = {
#ifdef CONFIG_BURNIN_TIME
	{ '0',	"Get Current Burnin Time ",		0, 	MM_Get_Burnin_Time, 		NULL},
	{ '1',	"Set Burnin Time ",				0, 	MM_Set_Burnin_Time, 		NULL},
#endif
#ifdef CONFIG_T2_BOOTUP_COUNTER
	{ '2',	"Get T2 Bootup Counter ",		0, 	MM_Get_T2_Bootup_Counter, 	NULL},
	{ '3',	"Set T2 Bootup Counter ",		0, 	MM_Set_T2_Bootup_Counter, 	NULL},
#endif
#ifdef CONFIG_SERIAL_NUMBER
	{ '4',	"Get Serial Number ",			0,	MM_Get_Serial_Number, 		NULL},
	{ '5',	"Set Serial Number",			0,	MM_Set_Serial_Number, 		NULL},
#endif
#ifdef CONFIG_DUT_ID
	{ '6',	"Get DUT Id",					0,	MM_Get_DUT_Id,				NULL},
	{ '7',	"Set DUT Id",					0,	MM_Set_DUT_Id,				NULL},
#endif
#ifdef CONFIG_FW_RECOVERY
	{ '8',	"Get FW Recovery",				0,	MM_Get_FW_Recover,			NULL},
	{ '9',	"Set FW Recovery",				0,	MM_Set_FW_Recover,			NULL},
#endif
#ifdef CONFIG_IVA_FLAG
	{ 'a',	"Get IVA Flag",					0,	MM_Get_IVA_Flag,			NULL},
	{ 'b',	"Set IVA Flag",					0,	MM_Set_IVA_Flag,			NULL},
#endif
#ifdef CONFIG_DUT_MODEL
	{ 'c',	"Get Model Name", 				0,	MM_Get_Model,			NULL},
	{ 'd',	"Set Model Name", 				0,	MM_Set_Model,			NULL},
#endif
#ifdef CONFIG_DUT_HOST
	{ 'e',	"Get Host Name", 				0,	MM_Get_Host,			NULL},
	{ 'f',	"Set Host Name", 				0,	MM_Set_Host,			NULL},
#endif
#ifdef CONFIG_DUT_DESC
	{ 'g',	"Get Description", 				0,	MM_Get_Desc,			NULL},
	{ 'h',	"Set Description", 				0,	MM_Set_Desc,			NULL},
#endif
};
DiagMenuStruct MPConfigMenu = {
	sizeof(MPConfigMenuTable)/sizeof(DiagMenuTableStruct),
	MPConfigMenuTable,
	"<<<MP Config Menu>>>",
	0, NULL
};

#ifdef CONFIG_BURNIN_TIME
static int MM_Get_Burnin_Time(int parameter)
{
	unsigned int val = 0;
	if(env_get_burnintime(&val)){
		printf("Get Burnin Time Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("Burnin Time: %u\n", val);
		return DIAG_OK;
	}
}

static int MM_Set_Burnin_Time(int parameter)
{
	unsigned int val = 0;
	char tmp[5];
	memset(tmp, 0, sizeof(tmp));
	if(get_line("Set Burnin Time:", tmp, sizeof(tmp), -1, str_number_dec, NULL, NULL) <= 0 ) return DIAG_ERROR;
	val = (int)simple_strtoul(tmp, NULL, 10);
	if(env_set_burnintime(val)){
		printf("Set Burnin Time Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("Set Burnin Time Successed.\n");
		return DIAG_OK;
	}
}
#endif

#ifdef CONFIG_T2_BOOTUP_COUNTER
static int MM_Get_T2_Bootup_Counter(int parameter)
{
	unsigned int val = 0;
	if(env_get_t2_bootup_counter(&val)){
		printf("Get Bootup Counter Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("Bootup Counter: %u\n", val);
		return DIAG_OK;
	}
}

static int MM_Set_T2_Bootup_Counter(int parameter)
{
	unsigned int val = 0;
	char tmp[18];
	memset(tmp, 0, sizeof(tmp));
	if(get_line("Set Bootup Counter:", tmp, sizeof(tmp), -1, str_number_dec, NULL, NULL) <= 0 ) return DIAG_ERROR;
	val = (int)simple_strtoul(tmp, NULL, 10);
	if(env_set_t2_bootup_counter(val)){
		printf("Set Bootup Counter Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("Set Bootup Counter Successed.\n");
		return DIAG_OK;
	}
}
#endif

#ifdef CONFIG_SERIAL_NUMBER
static int MM_Get_Serial_Number(int parameter)
{
#if CONFIG_SYS_I2C_EEPROM

	if (EEPROM_GetSerial(&g_serial_str))
	{
		printf("Get Serial Number from EEPROM Failed!!\n");
		return DIAG_ERROR;
	}
	else
	{
		printf("Serial Number: %s\n", g_serial_str);
	}

#else

	char *tmp;

	if (env_get_serial_number(&tmp))
	{
		printf("Get Serial Number Failed!!\n");
		return DIAG_ERROR;
	}
	else
	{
		printf("Serial Number: %s\n", tmp);
	}

#endif

	return DIAG_OK;
}

static int MM_Set_Serial_Number(int parameter)
{
	char tmp[14];
	memset(tmp, 0, sizeof(tmp));
	if (get_line("Set Serial Number:", tmp, sizeof(tmp), -1, str_menu_operate, NULL, NULL) <= 0)
	{
		return DIAG_ERROR;
	}

#if CONFIG_SYS_I2C_EEPROM

	if (EEPROM_SetSerial(tmp))
	{
		printf("Set Serial Number to EEPROM Failed!!\n");
		return DIAG_ERROR;
	}
	else
	{
		EEPROM_GetSerial(&g_serial_str);
		printf("Set Serial Number Successed.\n");
		return DIAG_OK;
	}

#else

	if (env_set_serial_number(tmp))
	{
		printf("Set Serial Number Failed!!\n");
		return DIAG_ERROR;
	}
	else
	{
		printf("Set Serial Number Successed.\n");
		return DIAG_OK;
	}

#endif

}
#endif

#ifdef CONFIG_DUT_ID
static int MM_Get_DUT_Id(int parameter)
{
	char *tmp;
	if(env_get_dut_id(&tmp)){
		printf("Get DUT ID Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("DUT ID: %s\n", tmp);
		return DIAG_OK;
	}
}

static int MM_Set_DUT_Id(int parameter)
{
	char tmp[19];//xx.xx.xx.xx.xx.xx
	memset(tmp, 0, sizeof(tmp));
	if(get_line("Set DUT ID:", tmp, sizeof(tmp), -1, str_ip, NULL, NULL) <= 0 ) return DIAG_ERROR;
	if(env_set_dut_id(tmp)){
		printf("Set DUT ID Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("Set DUT ID Successed.\n");
		return DIAG_OK;
	}
}
#endif

#ifdef CONFIG_FW_RECOVERY
static int MM_Get_FW_Recover(int parameter)
{
	unsigned int val = 0;
	if(env_get_fw_recovery(&val)){
		printf("Get FW Recover Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("FW Recover: %u\n", val);
		return DIAG_OK;
	}
}

static int MM_Set_FW_Recover(int parameter)
{
	unsigned int val = 0;
	char tmp[3];
	memset(tmp, 0, sizeof(tmp));
	if(get_line("Set FW Recover:", tmp, sizeof(tmp), -1, "01", NULL, NULL) <= 0 ) return DIAG_ERROR;
	val = (unsigned int)simple_strtoul(tmp, NULL, 10);
	if(env_set_fw_recovery(val)){
		printf("Set FW Recover Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("Set FW Recover Successed.\n");
		return DIAG_OK;
	}
}
#endif

#ifdef CONFIG_IVA_FLAG
static int MM_Get_IVA_Flag(int parameter)
{
	int val = 0;
	if(env_get_iva_flag(&val)){
		printf("Get IVA Flag Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("IVA Flag: 0x%04x\n", val);
		return DIAG_OK;
	}
}

static int MM_Set_IVA_Flag(int parameter)
{
	int val = 0;
	char tmp[6];
	memset(tmp, 0, sizeof(tmp));
	if(get_line("Set IVA Flag:0x", tmp, sizeof(tmp), -1, str_number_hex, NULL, NULL) <= 0 ) return DIAG_ERROR;
	val = (unsigned int)simple_strtoul(tmp, NULL, 16);
	if(env_set_iva_flag(val)){
		printf("Set IVA Flag Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("Set IVA Flag Successed.\n");
		return DIAG_OK;
	}
}
#endif

#ifdef CONFIG_DUT_MODEL
static int MM_Get_Model(int parameter)
{
#if CONFIG_SYS_I2C_EEPROM

	if (EEPROM_GetModelName(&g_model_name))
	{
		printf("Get Model Name from EEPROM Failed!!\n");
		return DIAG_ERROR;
	}
	else
	{
		printf("Model Name: %s\n", g_model_name);
	}

#else

	char *tmp;

	if (env_get_model(&tmp))
	{
		printf("Get Model Name Failed!!\n");
		return DIAG_ERROR;
	}
	else
	{
		printf("Model Name: %s\n", tmp);
	}

#endif

	return DIAG_OK;
}

static int MM_Set_Model(int parameter)
{
	char tmp[62];
	memset(tmp, 0, sizeof(tmp));
	if (get_line("Set Model Name:", tmp, sizeof(tmp), -1, NULL, NULL, NULL) <= 0)
	{
		return DIAG_ERROR;
	}

#if CONFIG_SYS_I2C_EEPROM

	EEPROM_SetProfile();
	EEPROM_SetHWFeature();

	if (EEPROM_SetModelName(tmp))
	{
		printf("Set Model Name to EEPROM Failed!!\n");
		return DIAG_ERROR;
	}
	else
	{
		EEPROM_GetModelName(&g_model_name);
		printf("Set Model Name Successed.\n");
	}

#else

	if (env_set_model(tmp))
	{
		printf("Set Model Name Failed!!\n");
		return DIAG_ERROR;
	}
	else
	{
		printf("Set Model Name Successed.\n");
		return DIAG_OK;
	}

#endif
}
#endif

#ifdef CONFIG_DUT_HOST
static int MM_Get_Host(int parameter)
{
	char *tmp;
	if(env_get_host(&tmp)){
		printf("Get Host Name Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("Host Name: %s\n", tmp);
		return DIAG_OK;
	}
}

static int MM_Set_Host(int parameter)
{
	char tmp[62];
	memset(tmp, 0, sizeof(tmp));
	if(get_line("Set Host Name:", tmp, sizeof(tmp), -1, NULL, NULL, NULL) <= 0 ) return DIAG_ERROR;
	if(env_set_host(tmp)){
		printf("Set Host Name Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("Set Host Name Successed.\n");
		return DIAG_OK;
	}
}
#endif

#ifdef CONFIG_DUT_DESC
static int MM_Get_Desc(int parameter)
{
	char *tmp;
	if(env_get_desc(&tmp)){
		printf("Get Description Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("Description: %s\n", tmp);
		return DIAG_OK;
	}
}

static int MM_Set_Desc(int parameter)
{
	char tmp[62];
	memset(tmp, 0, sizeof(tmp));
	if(get_line("Set Description:", tmp, sizeof(tmp), -1, NULL, NULL, NULL) <= 0 ) return DIAG_ERROR;
	if(env_set_desc(tmp)){
		printf("Set Description Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("Set Description Successed.\n");
		return DIAG_OK;
	}
}
#endif

//Env Config Menu======================================================
//static int EnvConfig_Save(int parameter);
//static int EnvConfig_Show(int parameter);
//static int EnvConfig_Default(int parameter);
#ifdef CONFIG_ETHADDR
static int EnvConfig_get_MAC(int parameter);
static int EnvConfig_set_MAC(int parameter);
#endif
#ifdef CONFIG_IPADDR
static int EnvConfig_get_IP(int parameter);
static int EnvConfig_set_IP(int parameter);
#endif
#ifdef CONFIG_SERVERIP
static int EnvConfig_get_ServerIP(int parameter);
static int EnvConfig_set_ServerIP(int parameter);
#endif
#ifdef CONFIG_NETMASK
static int EnvConfig_get_Netmask(int parameter);
static int EnvConfig_set_Netmask(int parameter);
#endif
#ifdef CONFIG_BAUDRATE
static int EnvConfig_get_Baudrate(int parameter);
static int EnvConfig_set_Baudrate(int parameter);
#endif

/*
static int EnvConfig_Save(int parameter)
{
	saveenv();
	return DIAG_OK;
}
*/

/*
static int EnvConfig_Show(int parameter)
{
#ifdef CONFIG_ETHADDR
	EnvConfig_get_MAC(0);
#endif
#ifdef CONFIG_IPADDR
	EnvConfig_get_IP(0);
#endif
#ifdef CONFIG_SERVERIP
	EnvConfig_get_ServerIP(0);
#endif
#ifdef CONFIG_NETMASK
	EnvConfig_get_Netmask(0);
#endif
#ifdef CONFIG_BAUDRATE
	EnvConfig_get_Baudrate(0);
#endif
	return DIAG_OK;
}
*/

/*
static int EnvConfig_Default(int parameter)
{
	set_default_env();
	saveenv();
	return DIAG_OK;
}
*/

#ifdef CONFIG_ETHADDR
static int EnvConfig_get_MAC(int parameter)
{
#if CONFIG_SYS_I2C_EEPROM

	if (EEPROM_GetMac(&g_mac_addr))
	{
		printf("Get MAC Address from EEPROM Failed!!\n");
		return DIAG_ERROR;
	}
	else
	{
		printf("MAC Address: %s\n", g_mac_addr);
	}
#else

	char *tmp;

	if (env_get_ethaddr(&tmp))
	{
		printf("Get MAC Address Failed!!\n");
		return DIAG_ERROR;
	}
	else
	{
		printf("MAC Address: %s\n", tmp);
	}

#endif

	return DIAG_OK;
}

static int EnvConfig_set_MAC(int parameter)
{
	int ret = 0;
	int i = 0;
	char tmp[19];//xx:xx:xx:xx:xx:xx
	char tmp1[19];//xx:xx:xx:xx:xx:xx
	memset(tmp, 0, sizeof(tmp));
	if ((ret = get_line("Set MAC Address:", tmp, sizeof(tmp), -1, str_mac, NULL, NULL)) < 12 )
	{
		return DIAG_ERROR;
	}

	for (i = 0; i < 15; i += 3)
	{
		memcpy(tmp1, tmp, sizeof(tmp));
		if (tmp1[i + 2] != ':')
		{
			tmp[i+2] = ':';
			memcpy(&tmp[i+3], &tmp1[i+2], sizeof(tmp)-(i+2));
		}
	}

#if CONFIG_SYS_I2C_EEPROM

	if (EEPROM_SetMac(tmp))
	{
		printf("Set MAC Address to EEPROM Failed!!\n");
		return DIAG_ERROR;
	}
	else
	{
		EEPROM_GetMac(&g_mac_addr);
		printf("Set MAC Address Successed.\n");
	}

#else

	if (env_set_ethaddr(tmp))
	{
		printf("Set MAC Address Failed!!\n");
		return DIAG_ERROR;
	}
	else
	{
		printf("Set MAC Address Successed.\n");
		return DIAG_OK;
	}

#endif

}
#endif

#ifdef CONFIG_IPADDR
static int EnvConfig_get_IP(int parameter)
{
	char *tmp;
	if(env_get_ipaddr(&tmp)){
		printf("Get IP Address Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("IP Address: %s\n", tmp);
		return DIAG_OK;
	}
}

static int EnvConfig_set_IP(int parameter)
{
	char tmp[17];//xxx.xxx.xxx.xxx
	memset(tmp, 0, sizeof(tmp));
	if(get_line("Set IP Address:", tmp, sizeof(tmp), -1, str_ip, NULL, NULL) <= 0 ) return DIAG_ERROR;
	if(env_set_ipaddr(tmp)){
		printf("Set IP Address Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("Set IP Address Successed.\n");
		return DIAG_OK;
	}
}
#endif
#ifdef CONFIG_SERVERIP
static int EnvConfig_get_ServerIP(int parameter)
{
	char *tmp;
	if(env_get_serverip(&tmp)){
		printf("Get Server IP Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("Server IP: %s\n", tmp);
		return DIAG_OK;
	}
}
static int EnvConfig_set_ServerIP(int parameter)
{
	char tmp[17];//xxx.xxx.xxx.xxx
	memset(tmp, 0, sizeof(tmp));
	if(get_line("Set Server IP:", tmp, sizeof(tmp), -1, str_ip, NULL, NULL) <= 0 ) return DIAG_ERROR;
	if(env_set_serverip(tmp)){
		printf("Set Server IP Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("Set Server IP Successed.\n");
		return DIAG_OK;
	}
}
#endif
#ifdef CONFIG_NETMASK
static int EnvConfig_get_Netmask(int parameter)
{
	char *tmp;
	if(env_get_netmask(&tmp)){
		printf("Get Net Mask Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("Net Mask: %s\n", tmp);
		return DIAG_OK;
	}
}

static int EnvConfig_set_Netmask(int parameter)
{
	char tmp[17];//xxx.xxx.xxx.xxx
	memset(tmp, 0, sizeof(tmp));
	if(get_line("Set Net Mask:", tmp, sizeof(tmp), -1, str_ip, NULL, NULL) <= 0 ) return DIAG_ERROR;
	if(env_set_netmask(tmp)){
		printf("Set Net Mask Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("Set Net Mask Successed.\n");
		return DIAG_OK;
	}
}
#endif
#ifdef CONFIG_BAUDRATE
static int EnvConfig_get_Baudrate(int parameter)
{
	unsigned int val = 0;
	if(env_get_baudrate(&val)){
		printf("Get Baudrate Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("Baudrate: %u\n", val);
		return DIAG_OK;
	}
}

static int EnvConfig_set_Baudrate(int parameter)
{
	unsigned int val = 0;
	char tmp[8];//115200
	memset(tmp, 0, sizeof(tmp));
	if(get_line("Set Baudrate:", tmp, sizeof(tmp), -1, str_number_dec, NULL, NULL) <= 0 ) return DIAG_ERROR;
	val = (unsigned int)simple_strtoul(tmp, NULL, 10);
	if(env_set_baudrate(val)){
		printf("Set Baudrate Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("Set Baudrate Successed.\n");
		return DIAG_OK;
	}
}
#endif

//Download Menu======================================================
extern int do_reset (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);

static int diag_setup_download(int parameter)
{
	int rcode = 0;
	char tmp[130];
	char prompt_string[200];
	char *ptr = NULL;

#ifdef CONFIG_SERVERIP
	sprintf(prompt_string, "Server IP (%s):", getenv("serverip"));
	memset(tmp, 0, sizeof(tmp));
	if(get_line(prompt_string, tmp, 17, -1, str_ip, NULL, NULL) > 0 ) setenv ("serverip", tmp);
	printf("Set Server IP = %s\n", getenv("serverip"));
#endif
#ifdef CONFIG_IPADDR
	sprintf(prompt_string, "DUT IP (%s):", getenv("ipaddr"));
	memset(tmp, 0, sizeof(tmp));
	if(get_line(prompt_string, tmp, 17, -1, str_ip, NULL, NULL) > 0 ) setenv ("ipaddr", tmp);
	printf("Set DUT IP = %s\n", getenv("ipaddr"));
#endif
#ifdef CONFIG_GATEWAYIP
	sprintf(prompt_string, "Gateway IP (%s):", getenv("gatewayip"));
	memset(tmp, 0, sizeof(tmp));
	if(get_line(prompt_string, tmp, 17, -1, str_ip, NULL, NULL) > 0 ) setenv ("gatewayip", tmp);
	printf("Set Gateway IP = %s\n", getenv("gatewayip"));
#endif
#ifdef CONFIG_NETMASK
	sprintf(prompt_string, "Net Mask (%s):", getenv("netmask"));
	memset(tmp, 0, sizeof(tmp));
	if(get_line(prompt_string, tmp, 17, -1, str_ip, NULL, NULL) > 0 ) setenv ("netmask", tmp);
	printf("Set Net Mask = %s\n", getenv("netmask"));
#endif
#if !(defined(CONFIG_FW_RECOVERY_FIX) && defined(CONFIG_FW_RECOVERY_FILE))
	sprintf(prompt_string, "Default FW file (%s):", getenv("fw_rcvrfile"));
	memset(tmp, 0, sizeof(tmp));
	if(get_line(prompt_string, tmp, sizeof(tmp), -1, NULL, NULL, NULL) > 0 ) setenv ("fw_rcvrfile", tmp);
	printf("Set Default FW file = %s\n", getenv("fw_rcvrfile"));
#endif
	if((ptr = getenv("ublfile")) != NULL){
		sprintf(prompt_string, "Default UBL file (%s):", ptr);
		memset(tmp, 0, sizeof(tmp));
		if(get_line(prompt_string, tmp, sizeof(tmp), -1, NULL, NULL, NULL) > 0 ) setenv ("ublfile", tmp);
		printf("Set Default UBL file = %s\n", getenv("ublfile"));
	}
	if((ptr = getenv("ubtfile")) != NULL){
		sprintf(prompt_string, "Default U-Boot file (%s):", ptr);
		memset(tmp, 0, sizeof(tmp));
		if(get_line(prompt_string, tmp, sizeof(tmp), -1, NULL, NULL, NULL) > 0 ) setenv ("ubtfile", tmp);
		printf("Set Default U-Boot file = %s\n", getenv("ubtfile"));
	}
	if((ptr = getenv("kernelfile")) != NULL){
		sprintf(prompt_string, "Default Kernel file (%s):", ptr);
		memset(tmp, 0, sizeof(tmp));
		if(get_line(prompt_string, tmp, sizeof(tmp), -1, NULL, NULL, NULL) > 0 ) setenv ("kernelfile", tmp);
		printf("Set Default Kernel file = %s\n", getenv("kernelfile"));
	}
	if((ptr = getenv("fsfile")) != NULL){
		sprintf(prompt_string, "Default FS file (%s):", ptr);
		memset(tmp, 0, sizeof(tmp));
		if(get_line(prompt_string, tmp, sizeof(tmp), -1, NULL, NULL, NULL) > 0 ) setenv ("fsfile", tmp);
		printf("Set Default FS file = %s\n", getenv("fsfile"));
	}
	if((ptr = getenv("mpkernelfile")) != NULL){
		sprintf(prompt_string, "Default MP Kernel file (%s):", ptr);
		memset(tmp, 0, sizeof(tmp));
		if(get_line(prompt_string, tmp, sizeof(tmp), -1, NULL, NULL, NULL) > 0 ) setenv ("mpkernelfile", tmp);
		printf("Set Default MP Kernel file = %s\n", getenv("mpkernelfile"));
	}
	if((ptr = getenv("mpfsfile")) != NULL){
		sprintf(prompt_string, "Default MP FS file (%s):", ptr);
		memset(tmp, 0, sizeof(tmp));
		if(get_line(prompt_string, tmp, sizeof(tmp), -1, NULL, NULL, NULL) > 0 ) setenv ("mpfsfile", tmp);
		printf("Set Default MP FS file = %s\n", getenv("mpfsfile"));
	}
	udelay(100000);
	saveenv();

	return rcode;
}

static int diag_default_download(int parameter)
{
	int rcode = 0;

#ifdef CONFIG_SERVERIP
	setenv("serverip", MK_STR(CONFIG_SERVERIP));
	printf("Set Server IP = %s\n", getenv("serverip"));
#endif
#ifdef CONFIG_IPADDR
	setenv("ipaddr", MK_STR(CONFIG_IPADDR));
	printf("Set DUT IP = %s\n", getenv("ipaddr"));
#endif
#ifdef CONFIG_GATEWAYIP
	setenv("gatewayip", MK_STR(CONFIG_GATEWAYIP));
	printf("Set Gateway IP = %s\n", getenv("gatewayip"));
#endif
#ifdef CONFIG_NETMASK
	setenv ("netmask", MK_STR(CONFIG_NETMASK));
	printf("Set Net Mask = %s\n", getenv("netmask"));
#endif
#if !(defined(CONFIG_FW_RECOVERY_FIX) && defined(CONFIG_FW_RECOVERY_FILE))
	setenv("fw_rcvrfile", CONFIG_FW_RECOVERY_FILE);
	printf("Set Default FW file = %s\n", getenv("fw_rcvrfile"));
#endif
	setenv("ublfile", D4_UBL_FILE_NAME);
	printf("Set Default UBL file = %s\n", getenv("ublfile"));
	setenv("ubtfile", D4_UBOOT_FILE_NAME);
	printf("Set Default U-Boot file = %s\n", getenv("ubtfile"));
	setenv("kernelfile", D4_KERNEL_FILE_NAME);
	printf("Set Default Kernel file = %s\n", getenv("kernelfile"));
	setenv("fsfile", D4_FS_FILE_NAME);
	printf("Set Default FS file = %s\n", getenv("fsfile"));
	setenv("mpkernelfile", D4_MPKERNEL_FILE_NAME);
	printf("Set Default MP Kernel file = %s\n", getenv("mpkernelfile"));
	setenv("mpfsfile", D4_MPFS_FILE_NAME);
	printf("Set Default MP FS file = %s\n", getenv("mpfsfile"));

	udelay(100000);
	saveenv();

	return rcode;
}

static int diag_showsetting_download(int parameter)
{
	int rcode = 0;
	char *ptr = NULL;

#ifdef CONFIG_SERVERIP
	printf("Server IP = %s\n", getenv("serverip"));
#endif
#ifdef CONFIG_IPADDR
	printf("DUT IP = %s\n", getenv("ipaddr"));
#endif
#ifdef CONFIG_GATEWAYIP
	printf("Gateway IP = %s\n", getenv("gatewayip"));
#endif
#ifdef CONFIG_NETMASK
	printf("Net Mask = %s\n", getenv("netmask"));
#endif
#if !(defined(CONFIG_FW_RECOVERY_FIX) && defined(CONFIG_FW_RECOVERY_FILE))
	printf("Default FW file = %s\n", getenv("fw_rcvrfile"));
#endif
	if((ptr = getenv("ublfile")) != NULL)
		printf("Default UBL file = %s\n", ptr);
	if((ptr = getenv("ubtfile")) != NULL)
		printf("Default U-Boot file = %s\n", ptr);
	if((ptr = getenv("kernelfile")) != NULL)
		printf("Default Kernel file = %s\n", ptr);
	if((ptr = getenv("fsfile")) != NULL)
		printf("Default FS file = %s\n", ptr);
	if((ptr = getenv("mpkernelfile")) != NULL)
		printf("Default MP Kernel file = %s\n", ptr);
	if((ptr = getenv("mpfsfile")) != NULL)
		printf("Default MP FS file = %s\n", ptr);

	return rcode;
}

static int diag_download_Firmware(int parameter)
{
	int rcode = 0;
	int ret = 0;
	char cmd[3];
	char *fw_file = NULL;
	cmd_tbl_t cmd_tmp;

	memset(cmd, 0, sizeof(cmd));
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));

#if !(defined(CONFIG_FW_RECOVERY_FIX) && defined(CONFIG_FW_RECOVERY_FILE))
	fw_file = getenv("fw_rcvrfile");
#else
	fw_file = CONFIG_FW_RECOVERY_FILE;
#endif

	if(get_line("Download 'Firmware' image from serial[0] / tftp[1]* / mmc[2]:", cmd, sizeof(cmd), -1, "012", NULL, "1") < 0) return -1;

	if(cmd[0] == '0'){
		char *args[2];
		args[0] = "loadfw";
		args[1] = "xymodem";
		if(do_loadfw(&cmd_tmp, 0, 2, args) == 0){
			printf("Download image successed.\n");
			if(*getenv("fw_rcvr") == '0') env_set_fw_recovery(1);
		}else{
			printf("Download image failed.\n");
			rcode = -1;
		}
	}else
	if((cmd[0] == '1') || (cmd[0] == '2')){
		char tmp[CFG_CBSIZE];
		memset(tmp, 0, sizeof(tmp));
		char *args[3];
		args[0] = "loadfw";
		if(cmd[0] == '1') args[1] = "tftp";
		else if(cmd[0] == '2'){
			char mmc[5];
			if(get_line("Enter mmc partition(>1):", tmp, 5, -1, str_number_dec, NULL, "1") < 0) return -1;
			sprintf(mmc, "mmc(0:%s)", tmp);
			args[1] = mmc;
		}else args[1] = "tftp";
		printf("[%s] Download file name(%s):", args[1], fw_file);
		if((ret = get_line(NULL, tmp, sizeof(tmp), -1, NULL, NULL, fw_file)) >= 0 ){
			args[2] = tmp;
			if(do_loadfw(&cmd_tmp, 0, 3, args) == 0){
				printf("Download file %s successed.\n", tmp);
				if(*getenv("fw_rcvr") == '0') env_set_fw_recovery(1);
			}else{
				printf("Download file %s failed.\n", tmp);
				rcode = -1;
			}
		}else{
			printf("Abort.\n");
			rcode = -1;
		}
	}
	return rcode;
}

#ifdef ROOTFS_FLASH
int do_eraseFilesystem (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	if(_erase_flash(cmdtp, flag, ROOTFS_FLASH, ROOTFS_SIZE)) return -1;
	return 0;
}

U_BOOT_CMD(
	erasefs, 1, 0,	do_eraseFilesystem,
	"Erase Filesystem",
	NULL
);
static int diag_erase_Filesystem(int parameter)
{
	cmd_tbl_t cmd_tmp;
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));
	if(do_eraseFilesystem(&cmd_tmp, 0, 0, NULL)) return DIAG_ERROR;
	else return DIAG_OK;
}
static int diag_download_Filesystem(int parameter)
{
	int rcode = 0;
	int ret = 0;
	char cmd[3];
	cmd_tbl_t cmd_tmp;
	memset(cmd, 0, sizeof(cmd));
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));

	if(get_line("Download 'Filesystem' image from serial[0] / tftp[1]* / mmc[2]:", cmd, sizeof(cmd), -1, "012", NULL, "1") < 0) return -1;

	if(cmd[0] == '0'){
		char *args[2];
		args[0] = "loadfs";
		args[1] = "xymodem";
		if(do_loadfs(&cmd_tmp, 0, 2, args) == 0){
			printf("Download image successed.\n");
		}else{
			printf("Download image failed.\n");
			rcode = -1;
		}
	}else
	if((cmd[0] == '1') || (cmd[0] == '2')){
		char* d4file = getenv("fsfile");
		if(d4file == NULL) d4file = D4_FS_FILE_NAME;
		char tmp[CFG_CBSIZE];
		memset(tmp, 0, sizeof(tmp));
		char *args[3];
		args[0] = "loadfs";
		if(cmd[0] == '1') args[1] = "tftp";
		else if(cmd[0] == '2'){
			char mmc[5];
			if(get_line("Enter mmc partition(>1):", tmp, 5, -1, str_number_dec, NULL, "1") < 0) return -1;
			sprintf(mmc, "mmc(0:%s)", tmp);
			args[1] = mmc;
		}else args[1] = "tftp";
		printf("[%s] Download file name(%s):", args[1], d4file);
		if((ret = get_line(NULL, tmp, sizeof(tmp), -1, NULL, NULL, d4file)) >= 0 ){
			args[2] = tmp;
			if(do_loadfs(&cmd_tmp, 0, 3, args) == 0){
				printf("Download file %s successed.\n", tmp);
			}else{
				printf("Download file %s failed.\n", tmp);
				rcode = -1;
			}
		}
	}
	return rcode;
}
#endif

#ifdef ROOTFS2_FLASH
int do_eraseFilesystem2 (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	if(_erase_flash(cmdtp, flag, ROOTFS2_FLASH, ROOTFS2_SIZE)) return -1;
	return 0;
}

U_BOOT_CMD(
	erasefs2, 1, 0,	do_eraseFilesystem2,
	"Erase Filesystem 2",
	NULL
);
static int diag_erase_Filesystem2(int parameter)
{
	cmd_tbl_t cmd_tmp;
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));
	if(do_eraseFilesystem2(&cmd_tmp, 0, 0, NULL)) return DIAG_ERROR;
	else return DIAG_OK;
}
static int diag_download_Filesystem2(int parameter)
{
	int rcode = 0;
	int ret = 0;
	char cmd[3];
	cmd_tbl_t cmd_tmp;
	memset(cmd, 0, sizeof(cmd));
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));

	if(get_line("Download 'Filesystem 2' image from serial[0] / tftp[1]* / mmc[2]:", cmd, sizeof(cmd), -1, "012", NULL, "1") < 0) return -1;

	if(cmd[0] == '0'){
		char *args[2];
		args[0] = "loadfs2";
		args[1] = "xymodem";
		if(do_loadfs2(&cmd_tmp, 0, 2, args) == 0){
			printf("Download image successed.\n");
		}else{
			printf("Download image failed.\n");
			rcode = -1;
		}
	}else
	if((cmd[0] == '1') || (cmd[0] == '2')){
		char* d4file = getenv("fsfile");
		if(d4file == NULL) d4file = D4_FS_FILE_NAME;
		char tmp[CFG_CBSIZE];
		memset(tmp, 0, sizeof(tmp));
		char *args[3];
		args[0] = "loadfs2";
		if(cmd[0] == '1') args[1] = "tftp";
		else if(cmd[0] == '2'){
			char mmc[5];
			if(get_line("Enter mmc partition(>1):", tmp, 5, -1, str_number_dec, NULL, "1") < 0) return -1;
			sprintf(mmc, "mmc(0:%s)", tmp);
			args[1] = mmc;
		}else args[1] = "tftp";
		printf("[%s] Download file name(%s):", args[1], d4file);
		if((ret = get_line(NULL, tmp, sizeof(tmp), -1, NULL, NULL, d4file)) >= 0 ){
			args[2] = tmp;
			if(do_loadfs2(&cmd_tmp, 0, 3, args) == 0){
				printf("Download file %s successed.\n", tmp);
			}else{
				printf("Download file %s failed.\n", tmp);
				rcode = -1;
			}
		}
	}
	return rcode;
}
#endif

#ifdef MPROOTFS_FLASH
int do_eraseMPFilesystem (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	if(_erase_flash(cmdtp, flag, MPROOTFS_FLASH, MPROOTFS_SIZE)) return -1;
	return 0;
}

U_BOOT_CMD(
	erasempfs, 1, 0,	do_eraseMPFilesystem,
	"Erase MP Filesystem",
	NULL
);
static int diag_erase_MPFilesystem(int parameter)
{
	cmd_tbl_t cmd_tmp;
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));
	if(do_eraseMPFilesystem(&cmd_tmp, 0, 0, NULL)) return DIAG_ERROR;
	else return DIAG_OK;
}
static int diag_download_MPFilesystem(int parameter)
{
	int rcode = 0;
	int ret = 0;
	char cmd[3];
	cmd_tbl_t cmd_tmp;
	memset(cmd, 0, sizeof(cmd));
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));

	if(get_line("Download 'MP Filesystem' image from serial[0] / tftp[1]* / mmc[2]:", cmd, sizeof(cmd), -1, "012", NULL, "1") < 0) return -1;

	if(cmd[0] == '0'){
		char *args[2];
		args[0] = "loadfs";
		args[1] = "xymodem";
		if(do_loadmpfs(&cmd_tmp, 0, 2, args) == 0){
			printf("Download image successed.\n");
		}else{
			printf("Download image failed.\n");
			rcode = -1;
		}
	}else
	if((cmd[0] == '1') || (cmd[0] == '2')){
		char* d4file = getenv("fsfile");
		if(d4file == NULL) d4file = D4_FS_FILE_NAME;
		char tmp[CFG_CBSIZE];
		memset(tmp, 0, sizeof(tmp));
		char *args[3];
		args[0] = "loadfs";
		if(cmd[0] == '1') args[1] = "tftp";
		else if(cmd[0] == '2'){
			char mmc[5];
			if(get_line("Enter mmc partition(>1):", tmp, 5, -1, str_number_dec, NULL, "1") < 0) return -1;
			sprintf(mmc, "mmc(0:%s)", tmp);
			args[1] = mmc;
		}else args[1] = "tftp";
		printf("[%s] Download file name(%s):", args[1], d4file);
		if((ret = get_line(NULL, tmp, sizeof(tmp), -1, NULL, NULL, d4file)) >= 0 ){
			args[2] = tmp;
			if(do_loadmpfs(&cmd_tmp, 0, 3, args) == 0){
				printf("Download file %s successed.\n", tmp);
			}else{
				printf("Download file %s failed.\n", tmp);
				rcode = -1;
			}
		}
	}
	return rcode;
}
#endif

#ifdef KERNEL_FLASH
int do_eraseKernel (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	if(_erase_flash(cmdtp, flag, KERNEL_FLASH, KERNEL_SIZE)) return -1;
	return 0;
}

U_BOOT_CMD(
	erasekl, 1, 0,	do_eraseKernel,
	"Erase Kernel",
	NULL
);
static int diag_erase_Kernel(int parameter)
{
	cmd_tbl_t cmd_tmp;
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));
	if(do_eraseKernel(&cmd_tmp, 0, 0, NULL)) return DIAG_ERROR;
	else return DIAG_OK;
}
static int diag_download_Kernel(int parameter)
{
	int rcode = 0;
	int ret = 0;
	char cmd[3];
	cmd_tbl_t cmd_tmp;
	memset(cmd, 0, sizeof(cmd));
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));

	if(get_line("Download 'Kernel' image from serial[0] / tftp[1]* / mmc[2]:", cmd, sizeof(cmd), -1, "012", NULL, "1") < 0) return -1;

	if(cmd[0] == '0'){
		char *args[2];
		args[0] = "loadkl";
		args[1] = "xymodem";
		if(do_loadkernel(&cmd_tmp, 0, 2, args) == 0){
			printf("Download image successed.\n");
		}else{
			printf("Download image failed.\n");
			rcode = -1;
		}
	}else
	if((cmd[0] == '1') || (cmd[0] == '2')){
		char* d4file = getenv("kernelfile");
		if(d4file == NULL) d4file = D4_KERNEL_FILE_NAME;
		char tmp[CFG_CBSIZE];
		memset(tmp, 0, sizeof(tmp));
		char *args[3];
		args[0] = "loadkl";
		if(cmd[0] == '1') args[1] = "tftp";
		else if(cmd[0] == '2'){
			char mmc[5];
			if(get_line("Enter mmc partition(>1):", tmp, 5, -1, str_number_dec, NULL, "1") < 0) return -1;
			sprintf(mmc, "mmc(0:%s)", tmp);
			args[1] = mmc;
		}else args[1] = "tftp";
		printf("[%s] Download file name(%s):", args[1], d4file);
		if((ret = get_line(NULL, tmp, sizeof(tmp), -1, NULL, NULL, d4file)) >= 0 ){
			args[2] = tmp;
			if(do_loadkernel(&cmd_tmp, 0, 3, args) == 0){
				printf("Download file %s successed.\n", tmp);
			}else{
				printf("Download file %s failed.\n", tmp);
				rcode = -1;
			}
		}
	}
	return rcode;
}
#endif

#ifdef KERNEL2_FLASH
int do_eraseKernel2 (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	if(_erase_flash(cmdtp, flag, KERNEL2_FLASH, KERNEL2_SIZE)) return -1;
	return 0;
}

U_BOOT_CMD(
	erasekl2, 1, 0,	do_eraseKernel2,
	"Erase Kernel 2",
	NULL
);
static int diag_erase_Kernel2(int parameter)
{
	cmd_tbl_t cmd_tmp;
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));
	if(do_eraseKernel2(&cmd_tmp, 0, 0, NULL)) return DIAG_ERROR;
	else return DIAG_OK;
}
static int diag_download_Kernel2(int parameter)
{
	int rcode = 0;
	int ret = 0;
	char cmd[3];
	cmd_tbl_t cmd_tmp;
	memset(cmd, 0, sizeof(cmd));
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));

	if(get_line("Download 'Kernel 2' image from serial[0] / tftp[1]* / mmc[2]:", cmd, sizeof(cmd), -1, "012", NULL, "1") < 0) return -1;

	if(cmd[0] == '0'){
		char *args[2];
		args[0] = "loadkl2";
		args[1] = "xymodem";
		if(do_loadkernel2(&cmd_tmp, 0, 2, args) == 0){
			printf("Download image successed.\n");
		}else{
			printf("Download image failed.\n");
			rcode = -1;
		}
	}else
	if((cmd[0] == '1') || (cmd[0] == '2')){
		char* d4file = getenv("kernelfile");
		if(d4file == NULL) d4file = D4_KERNEL_FILE_NAME;
		char tmp[CFG_CBSIZE];
		memset(tmp, 0, sizeof(tmp));
		char *args[3];
		args[0] = "loadkl2";
		if(cmd[0] == '1') args[1] = "tftp";
		else if(cmd[0] == '2'){
			char mmc[5];
			if(get_line("Enter mmc partition(>1):", tmp, 5, -1, str_number_dec, NULL, "1") < 0) return -1;
			sprintf(mmc, "mmc(0:%s)", tmp);
			args[1] = mmc;
		}else args[1] = "tftp";
		printf("[%s] Download file name(%s):", args[1], d4file);
		if((ret = get_line(NULL, tmp, sizeof(tmp), -1, NULL, NULL, d4file)) >= 0 ){
			args[2] = tmp;
			if(do_loadkernel2(&cmd_tmp, 0, 3, args) == 0){
				printf("Download file %s successed.\n", tmp);
			}else{
				printf("Download file %s failed.\n", tmp);
				rcode = -1;
			}
		}
	}
	return rcode;
}
#endif

#ifdef MPKERNEL_FLASH
int do_eraseMPKernel (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	if(_erase_flash(cmdtp, flag, MPKERNEL_FLASH, MPKERNEL_SIZE)) return -1;
	return 0;
}

U_BOOT_CMD(
	erasempkl, 1, 0,	do_eraseMPKernel,
	"Erase MP Kernel",
	NULL
);
static int diag_erase_MPKernel(int parameter)
{
	cmd_tbl_t cmd_tmp;
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));
	if(do_eraseMPKernel(&cmd_tmp, 0, 0, NULL)) return DIAG_ERROR;
	else return DIAG_OK;
}
static int diag_download_MPKernel(int parameter)
{
	int rcode = 0;
	int ret = 0;
	char cmd[3];
	cmd_tbl_t cmd_tmp;
	memset(cmd, 0, sizeof(cmd));
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));

	if(get_line("Download 'MP Kernel' image from serial[0] / tftp[1]* / mmc[2]:", cmd, sizeof(cmd), -1, "012", NULL, "1") < 0) return -1;

	if(cmd[0] == '0'){
		char *args[2];
		args[0] = "loadkl";
		args[1] = "xymodem";
		if(do_loadmpkernel(&cmd_tmp, 0, 2, args) == 0){
			printf("Download image successed.\n");
		}else{
			printf("Download image failed.\n");
			rcode = -1;
		}
	}else
	if((cmd[0] == '1') || (cmd[0] == '2')){
		char* d4file = getenv("kernelfile");
		if(d4file == NULL) d4file = D4_KERNEL_FILE_NAME;
		char tmp[CFG_CBSIZE];
		memset(tmp, 0, sizeof(tmp));
		char *args[3];
		args[0] = "loadkl";
		if(cmd[0] == '1') args[1] = "tftp";
		else if(cmd[0] == '2'){
			char mmc[5];
			if(get_line("Enter mmc partition(>1):", tmp, 5, -1, str_number_dec, NULL, "1") < 0) return -1;
			sprintf(mmc, "mmc(0:%s)", tmp);
			args[1] = mmc;
		}else args[1] = "tftp";
		printf("[%s] Download file name(%s):", args[1], d4file);
		if((ret = get_line(NULL, tmp, sizeof(tmp), -1, NULL, NULL, d4file)) >= 0 ){
			args[2] = tmp;
			if(do_loadmpkernel(&cmd_tmp, 0, 3, args) == 0){
				printf("Download file %s successed.\n", tmp);
			}else{
				printf("Download file %s failed.\n", tmp);
				rcode = -1;
			}
		}
	}
	return rcode;
}
#endif

#ifdef UBOOT_FLASH
int do_eraseUBoot (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	if(_erase_flash(cmdtp, flag, UBOOT_FLASH, UBOOT_SIZE)) return -1;
	return 0;
}

U_BOOT_CMD(
	eraseubt, 1, 0,	do_eraseUBoot,
	"Erase UBoot",
	NULL
);
static int diag_erase_UBoot(int parameter)
{
	cmd_tbl_t cmd_tmp;
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));
	if(do_eraseUBoot(&cmd_tmp, 0, 0, NULL)) return DIAG_ERROR;
	else return DIAG_OK;
}
static int diag_download_UBoot(int parameter)
{
	int rcode = 0;
	int ret = 0;
	char cmd[3];
	cmd_tbl_t cmd_tmp;
	memset(cmd, 0, sizeof(cmd));
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));

	if(get_line("Download 'U-Boot' image from serial[0] / tftp[1]* / mmc[2]:", cmd, sizeof(cmd), -1, "012", NULL, "1") < 0) return -1;

	if(cmd[0] == '0'){
		char *args[2];
		args[0] = "loadubt";
		args[1] = "xymodem";
		if(do_loaduboot(&cmd_tmp, 0, 2, args) == 0){
			printf("Download image successed.\n");
		}else{
			printf("Download image failed.\n");
			rcode = -1;
		}
	}else
	if((cmd[0] == '1') || (cmd[0] == '2')){
		char* d4file = getenv("ubtfile");
		if(d4file == NULL) d4file = D4_UBOOT_FILE_NAME;
		char tmp[CFG_CBSIZE];
		memset(tmp, 0, sizeof(tmp));
		char *args[3];
		args[0] = "loadubt";
		if(cmd[0] == '1') args[1] = "tftp";
		else if(cmd[0] == '2'){
			char mmc[5];
			if(get_line("Enter mmc partition(>1):", tmp, 5, -1, str_number_dec, NULL, "1") < 0) return -1;
			sprintf(mmc, "mmc(0:%s)", tmp);
			args[1] = mmc;
		}else args[1] = "tftp";
		printf("[%s] Download file name(%s):", args[1], d4file);
		if((ret = get_line(NULL, tmp, sizeof(tmp), -1, NULL, NULL, d4file)) >= 0 ){
			args[2] = tmp;
			if(do_loaduboot(&cmd_tmp, 0, 3, args) == 0){
				printf("Download file %s successed.\n", tmp);
			}else{
				printf("Download file %s failed.\n", tmp);
				rcode = -1;
			}
		}
	}
	return rcode;
}
#endif

#ifdef UBL_FLASH
int do_eraseUBL (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	if(_erase_flash(cmdtp, flag, UBL_FLASH, UBL_SIZE)) return -1;
	return 0;
}

U_BOOT_CMD(
	eraseubl, 1, 0,	do_eraseUBL,
	"Erase UBL",
	NULL
);
static int diag_erase_UBL(int parameter)
{
	cmd_tbl_t cmd_tmp;
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));
	if(do_eraseUBL(&cmd_tmp, 0, 0, NULL)) return DIAG_ERROR;
	else return DIAG_OK;
}
static int diag_download_UBL(int parameter)
{
	int rcode = 0;
	int ret = 0;
	char cmd[3];
	cmd_tbl_t cmd_tmp;
	memset(cmd, 0, sizeof(cmd));
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));

	if(get_line("Download 'UBL' image from serial[0] / tftp[1]* / mmc[2]:", cmd, sizeof(cmd), -1, "012", NULL, "1") < 0) return -1;

	if(cmd[0] == '0'){
		char *args[2];
		args[0] = "loadubl";
		args[1] = "xymodem";
		if(do_loadubl(&cmd_tmp, 0, 2, args) == 0){
			printf("Download image successed.\n");
		}else{
			printf("Download image failed.\n");
			rcode = -1;
		}
	}else
	if((cmd[0] == '1') || (cmd[0] == '2')){
		char* d4file = getenv("ublfile");
		if(d4file == NULL) d4file = D4_UBL_FILE_NAME;
		char tmp[CFG_CBSIZE];
		memset(tmp, 0, sizeof(tmp));
		char *args[3];
		args[0] = "loadubl";
		if(cmd[0] == '1') args[1] = "tftp";
		else if(cmd[0] == '2'){
			char mmc[5];
			if(get_line("Enter mmc partition(>1):", tmp, 5, -1, str_number_dec, NULL, "1") < 0) return -1;
			sprintf(mmc, "mmc(0:%s)", tmp);
			args[1] = mmc;
		}else args[1] = "tftp";
		printf("[%s] Download file name(%s):", args[1], d4file);
		if((ret = get_line(NULL, tmp, sizeof(tmp), -1, NULL, NULL, d4file)) >= 0 ){
			args[2] = tmp;
			if(do_loadubl(&cmd_tmp, 0, 3, args) == 0){
				printf("Download file %s successed.\n", tmp);
			}else{
				printf("Download file %s failed.\n", tmp);
				rcode = -1;
			}
		}
	}
	return rcode;
}
#endif

#ifdef ENV1_FLASH
int do_eraseenv (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	if(_erase_flash(cmdtp, flag, ENV1_FLASH, ENV1_SIZE)) return -1;
#ifdef ENV2_FLASH
	if(_erase_flash(cmdtp, flag, ENV2_FLASH, ENV2_SIZE)) return -1;
#endif
	return 0;
}

U_BOOT_CMD(
	eraseenv, 1, 0,	do_eraseenv,
	"Erase environment",
	NULL
);
static int diag_erase_env(int parameter)
{
	cmd_tbl_t cmd_tmp;
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));
	if(do_eraseenv(&cmd_tmp, 0, 0, NULL)) return DIAG_ERROR;
	else return DIAG_OK;
}
#endif

#ifdef DATA1_FLASH
int do_erasedata1 (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	if(_erase_flash(cmdtp, flag, DATA1_FLASH, DATA1_SIZE)) return -1;
	return 0;
}

U_BOOT_CMD(
	eraseda1, 1, 0,	do_erasedata1,
	"Erase data 1",
	NULL
);
static int diag_erase_Data1(int parameter)
{
	cmd_tbl_t cmd_tmp;
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));
	if(do_erasedata1(&cmd_tmp, 0, 0, NULL)) return DIAG_ERROR;
	else return DIAG_OK;
}
#endif

#ifdef DATA2_FLASH
int do_erasedata2 (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	if(_erase_flash(cmdtp, flag, DATA2_FLASH, DATA2_SIZE)) return -1;
	return 0;
}

U_BOOT_CMD(
	eraseda2, 1, 0,	do_erasedata2,
	"Erase data 2",
	NULL
);
static int diag_erase_Data2(int parameter)
{
	cmd_tbl_t cmd_tmp;
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));
	if(do_erasedata2(&cmd_tmp, 0, 0, NULL)) return DIAG_ERROR;
	else return DIAG_OK;
}
#endif

#ifdef MPDATA_FLASH
int do_erasempdata (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	if(_erase_flash(cmdtp, flag, MPDATA_FLASH, MPDATA_SIZE)) return -1;
	return 0;
}

U_BOOT_CMD(
	erasempda, 1, 0,	do_erasempdata,
	"Erase MP data",
	NULL
);
static int diag_erase_MPData(int parameter)
{
	cmd_tbl_t cmd_tmp;
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));
	if(do_erasempdata(&cmd_tmp, 0, 0, NULL)) return DIAG_ERROR;
	else return DIAG_OK;
}
#endif

#ifdef RESERVE_FLASH
int do_eraserese (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	if(_erase_flash(cmdtp, flag, RESERVE_FLASH, RESERVE_SIZE)) return -1;
	return 0;
}

U_BOOT_CMD(
	eraseres, 1, 0,	do_eraserese,
	"Erase reserve",
	NULL
);
static int diag_erase_Reserve(int parameter)
{
	cmd_tbl_t cmd_tmp;
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));
	if(do_eraserese(&cmd_tmp, 0, 0, NULL)) return DIAG_ERROR;
	else return DIAG_OK;
}
#endif

#ifdef DSP1_FLASH
int do_erasedsp1 (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	if(_erase_flash(cmdtp, flag, DSP1_FLASH, DSP1_SIZE)) return -1;
	return 0;
}

U_BOOT_CMD(
	erasedsp1, 1, 0,	do_erasedsp1,
	"Erase DSP 1",
	NULL
);
static int diag_erase_Dsp1(int parameter)
{
	cmd_tbl_t cmd_tmp;
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));
	if(do_erasedsp1(&cmd_tmp, 0, 0, NULL)) return DIAG_ERROR;
	else return DIAG_OK;
}
#endif

#ifdef DSP2_FLASH
int do_erasedsp2 (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	if(_erase_flash(cmdtp, flag, DSP2_FLASH, DSP2_SIZE)) return -1;
	return 0;
}

U_BOOT_CMD(
	erasedsp2, 1, 0,	do_erasedsp2,
	"Erase DSP 2",
	NULL
);
static int diag_erase_Dsp2(int parameter)
{
	cmd_tbl_t cmd_tmp;
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));
	if(do_erasedsp2(&cmd_tmp, 0, 0, NULL)) return DIAG_ERROR;
	else return DIAG_OK;
}
#endif

#ifdef CONFIG_FLASH
int do_eraseconfig (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	if(_erase_flash(cmdtp, flag, CONFIG_FLASH, CONFIG_SIZE)) return -1;
#ifdef CONFIG2_FLASH
	if(_erase_flash(cmdtp, flag, CONFIG2_FLASH, CONFIG2_SIZE)) return -1;
#endif
	return 0;
}

U_BOOT_CMD(
	eraseconfig, 1, 0,	do_eraseconfig,
	"Erase config",
	NULL
);
static int diag_erase_Config(int parameter)
{
	cmd_tbl_t cmd_tmp;
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));
	if(do_eraseconfig(&cmd_tmp, 0, 0, NULL)) return DIAG_ERROR;
	else return DIAG_OK;
}
#endif

#ifdef LOG_FLASH
int do_eraselog (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	if(_erase_flash(cmdtp, flag, LOG_FLASH, LOG_SIZE)) return -1;
#ifdef LOG2_FLASH
	if(_erase_flash(cmdtp, flag, LOG2_FLASH, LOG2_SIZE)) return -1;
#endif
	return 0;
}

U_BOOT_CMD(
	eraselog, 1, 0,	do_eraselog,
	"Erase log",
	NULL
);
static int diag_erase_Log(int parameter)
{
	cmd_tbl_t cmd_tmp;
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));
	if(do_eraselog(&cmd_tmp, 0, 0, NULL)) return DIAG_ERROR;
	else return DIAG_OK;
}
#endif

#ifdef BACKUP_FLASH
int do_erasebackup (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	if(_erase_flash(cmdtp, flag, BACKUP_FLASH, BACKUP_SIZE)) return -1;
	return 0;
}

U_BOOT_CMD(
	erasebackup, 1, 0,	do_erasebackup,
	"Erase backup",
	NULL
);
static int diag_erase_Backup(int parameter)
{
	cmd_tbl_t cmd_tmp;
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));
	if(do_erasebackup(&cmd_tmp, 0, 0, NULL)) return DIAG_ERROR;
	else return DIAG_OK;
}
#endif

static int diag_erase_WholeFlash(int parameter)
{
	char cmd[3];
	cmd_tbl_t cmd_tmp;
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));

	printf("\n");
	printf(" !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! WARNING !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	printf("                                                                              \n");
	printf(" This command will CLEAR ALL THE CONTENTS of the NAND Flash,                  \n");
	printf(" including the BOOT SECTOR,                                                   \n");
	printf(" which will cause the SYSTEM UNABLE TO BOOT.                                  \n");
	printf(" Are you sure you want to do?                                                 \n");
	printf("                                                                              \n");
	printf(" !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! WARNING !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	printf("\n");

	if(get_line("Really ERASE WHOLE NAND Flash? <y/N>:", cmd, sizeof(cmd), -1, "YyNn", NULL, "N") < 0) return DIAG_ERROR;
	if((cmd[0] == 'Y') || (cmd[0] == 'y')){
		if(_erase_flash_whole(&cmd_tmp, 0)) return DIAG_ERROR;
		else return DIAG_OK;
	}
	return DIAG_OK;
}

static int diag_erase_WholeFlash_wo_BIOS(int parameter)
{
	cmd_tbl_t cmd_tmp;
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));

	if(_erase_flash(&cmd_tmp, 0, (UBOOT_FLASH + UBOOT_SIZE), 0)) return DIAG_ERROR;

	return DIAG_OK;
}

static int diag_scrub_WholeFlash(int parameter)
{
	cmd_tbl_t cmd_tmp;
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));

	if(_scrub_flash_whole(&cmd_tmp, 0, 0)) return DIAG_ERROR;

	return DIAG_OK;
}

static int diag_scrub_WholeFlash_wo_BIOS(int parameter)
{
	cmd_tbl_t cmd_tmp;
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));

	if(_scrub_flash(&cmd_tmp, 0, (UBOOT_FLASH + UBOOT_SIZE), 0, 0)) return DIAG_ERROR;

	return DIAG_OK;
}

static int diag_do_default(int parameter)
{
	do_initenv(NULL, 0, 0, NULL);
	return DIAG_OK;
}

static int diag_do_reset(int parameter)
{
	char *args[1];
	cmd_tbl_t cmd_tmp;
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));
	args[0] = "reset";
	do_reset(&cmd_tmp, 0, 1, args);
	while(1) udelay(10000);
	return DIAG_OK;
}

//LED Test ---------------------------------------------------------------------------
//#define GPIO_LED_SYS_RED		0
//#define GPIO_LED_SYS_GREEN		1
static int led_test(int parameter);
static DiagMenuTableStruct LEDTestMenuTable[] = {
	{ '0',	"STAT Red LED",			GPIO_SYSLED_RED,		led_test,		NULL},
	{ '1',	"STAT Green LED",		GPIO_SYSLED_GREEN,		led_test,		NULL},
#ifdef GPIO_LED_SD
	{ '2',	"SD LED",				GPIO_LED_SD,			led_test,		NULL},
#endif
#ifdef GPIO_LED_PTZ
	{ '3',	"PTZ LED",				GPIO_LED_PTZ,			led_test,		NULL},
#endif
#ifdef GPIO_LED_VIDEO
	{ '4',	"VIDEO LED",			GPIO_LED_VIDEO,			led_test,		NULL},
#endif
#ifdef GPIO_LED_FAIL
	{ '5',	"FAIL LED",				GPIO_LED_FAIL,			led_test,		NULL},
#endif
#ifdef GPIO_SYSLED_CONTROL
	{ '6',	"CONTROL LED", 			GPIO_SYSLED_CONTROL,	led_test,		NULL},
#endif
#ifdef GPIO_LED_V1
	{ '7',	"VIDEO 1 LED",			GPIO_LED_V1,			led_test,		NULL},
#endif
#ifdef GPIO_LED_V2
	{ '8',	"VIDEO 2 LED",			GPIO_LED_V2,			led_test,		NULL},
#endif
#ifdef GPIO_LED_V3
	{ '9',	"VIDEO 3 LED",			GPIO_LED_V3,			led_test,		NULL},
#endif
#ifdef GPIO_LED_V4
	{ 'a',	"VIDEO 4 LED",			GPIO_LED_V4,			led_test,		NULL},
#endif
#ifdef GPIO_LINKSP_0
	{ 'b',	"LINKSP 0 LED",			GPIO_LINKSP_0,			led_test,		NULL},
#endif
#ifdef GPIO_LINKSP_1
	{ 'c',	"LINKSP 1 LED",			GPIO_LINKSP_1,			led_test,		NULL},
#endif
};
static DiagMenuStruct LedTestMenu = {
	sizeof(LEDTestMenuTable)/sizeof(DiagMenuTableStruct),
	LEDTestMenuTable,
	"<<<LED Test Menu>>>",
	0, NULL
};

static int led_test(int parameter)
{
	char cmd[3];
	unsigned char value;
	printf("Set to On[%d] / Off[%d]:", GPIO_LED_ON, GPIO_LED_OFF);
	if(get_line(NULL, cmd, sizeof(cmd), -1, "01", NULL, NULL) <= 0) return -1;
	value = (unsigned char)(simple_strtoul(cmd, NULL, 10) & 0x000000ff);
	GPIO_Out(parameter, value);
	return DIAG_OK;
}

//UART Test ---------------------------------------------------------------------------
#ifdef RS485_NAME
static int uart_test_485_tx(int parameter);
static int uart_test_485_rx(int parameter);
static int uart_test_485_loopback(int parameter);
static int uart_test_485_set_baudrate(int parameter);
#endif
#ifdef CAMERA_NAME
static int uart_test_cam_tx(int parameter);
static int uart_test_cam_rx(int parameter);
static int uart_test_cam_loopback(int parameter);
static int uart_test_cam_set_baudrate(int parameter);
static int uart_test_cam_send_command(int parameter);
#endif
#if defined(PTCTRL_NAME) || defined(PTCTRL_HERMES_NAME)
static int uart_test_pt_tx(int parameter);
static int uart_test_pt_rx(int parameter);
static int uart_test_pt_loopback(int parameter);
static int uart_test_pt_set_baudrate(int parameter);
static int uart_test_pt_send_command(int parameter);
#endif
static DiagMenuTableStruct UARTTestMenuTable[] = {
#ifdef RS485_NAME
	{ '0', "PTZ Port 485 Tx",				0, uart_test_485_tx,			NULL},
	{ '1', "PTZ Port 485 Rx",				0, uart_test_485_rx,			NULL},
	{ '2', "PTZ Port 485 loopback",			0, uart_test_485_loopback,		NULL},
	{ '3', "PTZ Port 485 set baudrate",		0, uart_test_485_set_baudrate,	NULL},
#endif
#ifdef CAMERA_NAME
	{ '4', "Camera Port Tx",				0, uart_test_cam_tx,			NULL},
	{ '5', "Camera Port Rx",				0, uart_test_cam_rx,			NULL},
	{ '6', "Camera Port loopback",			0, uart_test_cam_loopback,		NULL},
	{ '7', "Camera Port set baudrate",		0, uart_test_cam_set_baudrate,	NULL},
	{ '8', "Camera Port send command",		0, uart_test_cam_send_command,	NULL},
#endif
#if defined(PTCTRL_NAME) || defined(PTCTRL_HERMES_NAME)
	{ 'a', "PT Port Tx",					0, uart_test_pt_tx,				NULL},
	{ 'b', "PT Port Rx",					0, uart_test_pt_rx,				NULL},
	{ 'c', "PT Port loopback",				0, uart_test_pt_loopback,		NULL},
	{ 'd', "PT Port set baudrate",			0, uart_test_pt_set_baudrate,	NULL},
	{ 'e', "PT Port send command",			0, uart_test_pt_send_command,	NULL},
#endif
};
static DiagMenuStruct UARTTestMenu = {
	sizeof(UARTTestMenuTable)/sizeof(DiagMenuTableStruct),
	UARTTestMenuTable,
	"<<<UART Test Menu>>>",
	0, NULL
};

struct serial_device* get_serial_device_by_name (char *name)
{
	struct serial_device *s = serial_devices_list();

	for ( ; s; s = s->next) {
		if (strcmp (s->name, name) == 0) {
			return s;
		}
	}
	return NULL;
}

#define UART_MAX_TEST_LEN 256
#define UART_RX_BUFFER_LEN 64
#define UART_TIMEOUT_MSEC 3000
int uart_loopback_test(char *name)
{
	int i = 0, j = 0;
	int timeout_msec = 0;
	int len = UART_MAX_TEST_LEN;
	char tx_buff[UART_MAX_TEST_LEN];
	char rx_buff[UART_MAX_TEST_LEN];
	char cmd[5];
	unsigned char value = 0;
	struct serial_device *dev;

	dev = get_serial_device_by_name(name);
	if(dev == NULL) return DIAG_ERROR;
	dev->init();

	i = get_line("Start Byte:0x", cmd, sizeof(cmd), -1, str_number_hex, NULL, NULL);
	if(i < 0) return -1;
	else if(i > 0) value = (unsigned char)(simple_strtoul(cmd, NULL, 16) & 0x000000ff);

	printf("Count(max.%d):", UART_MAX_TEST_LEN);
	i = get_line(NULL, cmd, sizeof(cmd), -1, str_number_dec, NULL, NULL);
	if(i < 0) return -1;
	else if(i > 0) len = (int)(simple_strtoul(cmd, NULL, 10));
	if(len > UART_MAX_TEST_LEN) len = UART_MAX_TEST_LEN;

	for(i = 0; i < len; i++) tx_buff[i] = value + i;
	printf("tx_buff->\n");
	print_buffer(0, (void*)tx_buff, 1, len, 0);
	memset(rx_buff, 0, len);
	for(i = 0, j = 0; i < len; i++){
		dev->putc(tx_buff[i]);
		if((i == len - 1) || ((i > 0) && ((i % UART_RX_BUFFER_LEN) == (UART_RX_BUFFER_LEN - 1)))){
			timeout_msec = UART_TIMEOUT_MSEC;
			while(j <= i){
				if(dev->tstc()){
					timeout_msec = UART_TIMEOUT_MSEC;
					rx_buff[j++] = (char)dev->getc();
				}
				if(timeout_msec-- <= 0) { printf("Rx Time out!!\n"); break; }
				udelay(1000);
			}
		}
	}
	printf("rx_buff->\n");
	print_buffer(0, (void*)rx_buff, 1, len, 0);

	if(memcmp(tx_buff, rx_buff, len)){
		printf("Tx/Rx data miss match.\n");
	}else{
		printf("PASS.\n");
	}
	return DIAG_OK;
}

int uart_tx_test(char *name)
{
	struct serial_device *dev;
	dev = get_serial_device_by_name(name);
	if(dev == NULL) return DIAG_ERROR;
	dev->init();
	printf("Press 'Reset button' to exit.\n");
	while(!resetkey_state()){
		if(tstc()) dev->putc(getc());
		udelay(10000);
	}
	return DIAG_OK;
}

int uart_rx_test(char *name)
{
	struct serial_device *dev;
	dev = get_serial_device_by_name(name);
	if(dev == NULL) return DIAG_ERROR;
	dev->init();
	printf("Press 'Reset button' to exit.\n");
	while(!resetkey_state()){
		if(dev->tstc()) putc((char)dev->getc());
		udelay(10000);
	}
	return DIAG_OK;
}

int uart_set_baudrate(char *name)
{
	char tmp[8];
	printf("Baudrate(%s):", getenv(name));
	if(get_line(NULL, tmp, sizeof(tmp), -1, str_number_dec, NULL, NULL) <= 0) return DIAG_ERROR;
	if(setenv(name, tmp)) return DIAG_ERROR;
	if(saveenv()) return DIAG_ERROR;
	return DIAG_OK;
}

#ifdef RS485_NAME
static int uart_test_485_tx(int parameter)
{
	RS485_SetTx(); udelay(100000);
	return uart_tx_test(RS485_NAME);
}

static int uart_test_485_rx(int parameter)
{
#ifdef RS485_2WIRE
	RS485_SetRx(); udelay(100000);
#endif
	return uart_rx_test(RS485_NAME);
}

static int uart_test_485_loopback(int parameter)
{
	RS485_SetTx(); udelay(100000);
	return uart_loopback_test(RS485_NAME);
}

static int uart_test_485_set_baudrate(int parameter)
{
#if defined(RS485_BAUDRATE_ITEM) && defined(RS485_BAUDRATE) && (RS485_BAUDRATE >= 0)
	return uart_set_baudrate(RS485_BAUDRATE_ITEM);
#else
	return DIAG_OK;
#endif
}
#endif

int uart_manual_command_parse(char *cmdStr, char *cmdBuff, int size)
{
	int i = 0;
	int len = 0;
	char tmp[5];
	char *pstr, *pend;

	printf("Command string->\n");
	print_buffer(0, (void*)cmdStr, 1, strlen(cmdStr), 0);

	memset(cmdBuff, 0, size);
	pstr = pend = cmdStr;
	for(i = 0; i <= strlen(cmdStr); i++){
		if(((*pend) == ',') || ((*pend) == ' ') || ((*pend) == '\0')){
			if(pstr == pend){
				pstr++;
			}else{
				memset(tmp, 0, sizeof(tmp));
				strncpy(tmp, pstr, pend - pstr);
				pstr = pend + 1;
				if(len >= size) break;
				else cmdBuff[len++] = (char)(simple_strtoul(tmp, NULL, 16));
			}
		}else
		if(!((((*pend) >= '0') && ((*pend) <= '9')) || (((*pend) >= 'A') && ((*pend) <= 'F'))
			|| (((*pend) >= 'a') && ((*pend) <= 'f')))){
			pstr++;
		}
		pend++;
	}
	return len;
}

int uart_manual_response_size()
{
	char tmp[8];
	if(get_line("Response size(0:no response)->", tmp, sizeof(tmp), -1, str_number_dec, NULL, NULL) <= 0) return 0;
	return (int)(simple_strtoul(tmp, NULL, 10));
}

int uart_manual_command(char *command, int size)
{
	printf("Input format: 00,01,02,03,04,05,06,07,08,.... (or) 09 0a 0b 0c 0d 0e 0f ....\n");
	if(get_line("->", command, size, -1, "0123456789abcdefABCDEF, ", NULL, NULL) < 0) return -1;
	return 0;
}

char uart_command_checksum(char *cmdBuff, int size)
{
	int i = 0;
	char sum = 0;
	for(i = 0; i < size; i++) sum += (cmdBuff[i] & 0x7f);
	sum = ~sum; sum++; //take two's complement
	sum &= 0x7f;
	return sum;
}

#ifdef CAMERA_NAME
static int uart_cam_baudrate = CAMERA_BAUDRATE;
int uart_set_cam_baudrate()
{
	char tmp[32];
	sprintf(tmp, "%d", uart_cam_baudrate);
	if(setenv(CAMERA_BAUDRATE_ITEM, tmp)) return DIAG_ERROR;
	if(saveenv()) return DIAG_ERROR;
	return DIAG_OK;
}

static int uart_test_cam_tx(int parameter)
{
	uart_set_cam_baudrate();
	return uart_tx_test(CAMERA_NAME);
}

static int uart_test_cam_rx(int parameter)
{
	uart_set_cam_baudrate();
	return uart_loopback_test(CAMERA_NAME);
}

static int uart_test_cam_loopback(int parameter)
{
	uart_set_cam_baudrate();
	return uart_loopback_test(CAMERA_NAME);
}

static int uart_test_cam_set_baudrate(int parameter)
{
#if defined(CAMERA_BAUDRATE_ITEM) && defined(CAMERA_BAUDRATE) && (CAMERA_BAUDRATE >= 0)
	return uart_set_baudrate(CAMERA_BAUDRATE_ITEM);
#else
	return DIAG_OK;
#endif
}

static char gCamCmdSet_Initial[3] = {0xA0, 0x01, 0x0A}; //All Initial
static char gCamCmdSet_Menu_Init[3] = {0xA0, 0x01, 0x0B}; //All Menu Initial
static char gCamCmdSet_Lens_Init[3] = {0xA0, 0x01, 0x0C}; //Lens Initial
static char gCamCmdSet_Reset[3] = {0xA0, 0x01, 0x0D}; //Reset
static char gCamCmdSet_Preset[3] = {0xA0, 0x01, 0x14}; //Custom preset
#define CAM_CMDSET_NOTE_Preset "\
Custom preset [0p]\n\
 p: (0:C.Reset, 1:C.Memory, 2:C.Recall)\n"
static char gCamCmdSet_OutMode[3] = {0xA0, 0x01, 0x20}; //VideoOut Mode
#define CAM_CMDSET_NOTE_OutMode "\
VideoOut Mode [0p 0q]\n\
 p: (0:LVDS 1:Component 2:Composite [Initial value:0])\n\
 q: Digital(LVDS): 0:1080p(30/25Hz) 1:1200p(30/25Hz) 2:1080i(60/50Hz) 3:720p(60/50Hz)\n\
    Component: 0:1080p(60/50Hz) 1:1080i(60/50Hz) 2:720p(60/50Hz) 3:480p(60/50Hz)normal\n\
               4:480p(60/50Hz)crop 5:480p(60/50Hz)squeeze 6:1080p(30/25Hz)\n\
    Composite: 0:normal 1:crop 2:squeeze [Initial value:2]\n"
static char gCamCmdSet_NtscPal[3] = {0xA0, 0x01, 0x21}; //Ntsc/Pal
#define CAM_CMDSET_NOTE_NtscPal "\
Ntsc/Pal [0p]\n\
 p: (0:NTSC, 1:PAL)\n"
static char gCamCmdSet_Tele[3] = {0xA0, 0x02, 0x05}; //Tele
static char gCamCmdSet_Wide[3] = {0xA0, 0x02, 0x06}; //Wide
static char gCamCmdSet_Stop[3] = {0xA0, 0x02, 0x07}; //Tele/Wide Stop
static char gCamCmdSet_Speed[3] = {0xA0, 0x02, 0x08}; //Tele/Wide Speed
#define CAM_CMDSET_NOTE_Speed "\
Tele/Wide Speed [0p]\n\
 p: Speed 1-5 during Tele and Wide commands (1:Slow - 5:Fast [Initial value:3])\n"
static char gCamCmdSet_Direct[3] = {0xA0, 0x02, 0x09}; //Direct
#define CAM_CMDSET_NOTE_Direct "\
Direct [0p 0q 0r]\n\
 pqr: Direct Position 0:Wide, 1600:10x(optical), 1750:160x(digital)\n\
 Digital Zoom Tele Limiter command needs to be sent before moving to digital zoom field.\n"
static char gCamCmdSet_DirtFocus[3] = {0xA0, 0x02, 0x0A}; //Zoom Direct with Focus
#define CAM_CMDSET_NOTE_DirtFocus "\
Zoom Direct with Focus [0p 0q 0r 0s 0t 0u]\n\
 pqr:Zoom Direct Position\n\
 stu:Focus Direct Position\n\
 For carrying set the focus to the specified position.\n"
static char gCamCmdSet_DigiLimit[3] = {0xA0, 0x02, 0x18}; //Digital Zoom Tele Limiter
#define CAM_CMDSET_NOTE_DigiLimit "\
Digital Zoom Tele Limiter [0p]\n\
 p: Maximum digital zoom magnification (0:x1 [Initial value], 1:x2, 2:x4, 3:x8, 4:x16)\n"
static char gCamCmdSet_OpcLimit[3] = {0xA0, 0x02, 0x19}; //Optical Zoom Wide/Tele Limitter
#define CAM_CMDSET_NOTE_OpcLimit "\
Wide/Tele Optical ZOOM Limiter [xx yy]\n\
 xx: 0, 1-9 (0 is no limit and 1-9 is the Wide optical ZOOM limit)\n\
 yy: 0, 2-10 (0 is no limit and 2-10 is the Tele optical ZOOM limit)\n\
 Note: Ensure that xx < yy\n"

int uart_cam_send_command(char *name, char *tx_buff, char *rx_buff, int tx_len, int rx_len, int *len, int do_rec, int timeout)
{
	int i = 0, j = 0;
	int timeout_msec = timeout;
	char sum = 0;

	if(uart_set_cam_baudrate()) return DIAG_ERROR;

	struct serial_device *dev;
	dev = get_serial_device_by_name(name);
	if(dev == NULL) return DIAG_ERROR;
	dev->init();

	if(((*len) + 2) > tx_len) return DIAG_ERROR;

	sum = uart_command_checksum(tx_buff, (*len));
	tx_buff[(*len)] = sum; (*len)++; //Checksum Byte
	tx_buff[(*len)] = 0xFF; (*len)++; //Terminator Byte

	printf("Message(size=%d)->\n", (*len));
	print_buffer(0, (void*)tx_buff, 1, (*len), 0);

	for(i = 0, j = 0; i < (*len); i++){
		dev->putc(tx_buff[i]);
	}
	if(do_rec == 1){
		memset(rx_buff, 0, sizeof(rx_buff));
		timeout_msec = timeout;
		while(j < rx_len){
			if(dev->tstc()) {
				timeout_msec = timeout;
				rx_buff[j] = (char)dev->getc();
				if((rx_buff[j] == 0xfa) || (rx_buff[j] == 0xfb) || (rx_buff[j] == 0xfc)) { j++; break; }
				j++;
			}
			if(timeout_msec-- <= 0) { printf("Rx Time out!!\n"); return DIAG_ERROR; }
			udelay(1000);
		}
		printf("Response(size=%d)->\n", j);
		print_buffer(0, (void*)rx_buff, 1, j, 0);
		if(rx_buff[j-1] == 0xfa){
			printf("Response ACK.");
		}else
		if(rx_buff[j-1] == 0xfb){
			printf("Response NACK.");
			return DIAG_ERROR;
		}else
		if(rx_buff[j-1] == 0xfc){
			printf("Response ERR.");
			return DIAG_ERROR;
		}
	}

	return DIAG_OK;
}

static int uart_test_cam_send_command(int parameter)
{
	int len = 0;
	int do_rec = 1;
	char cmd = 0;
	char tx_buff[UART_MAX_TEST_LEN];
	char rx_buff[UART_MAX_TEST_LEN];
	char commandBuff[UART_MAX_TEST_LEN * 3 + 2];

	while(1){

		printf("\n");
		printf("Select command set:\n");
		printf("  [0]Manual command.(w/o Checksum & Terminator Byte)\n");
		printf("  [1]All Initial.\n");
		printf("  [2]All Menu Initial.\n");
		printf("  [3]Lens Initial.\n");
		printf("  [4]Reset.\n");
		printf("  [5]Custom preset.\n");
		printf("  [6]VideoOut Mode.\n");
		printf("  [7]Ntsc/Pal.\n");
		printf("  [8]Tele.\n");
		printf("  [9]Wide.\n");
		printf("  [a]Tele/Wide Stop.\n");
		printf("  [b]Tele/Wide Speed.\n");
		printf("  [c]Direct.\n");
		printf("  [d]Zoom Direct with Focus.\n");
		printf("  [e]Digital Zoom Tele Limiter.\n");
		printf("  [f]Optical Zoom Wide/Tele Limitter.\n");
		printf("  [ESC]Exit.\n");
		printf("->");
		cmd = getc();
		printf("\n");

		memset(commandBuff, 0, sizeof(commandBuff));
		memset(tx_buff, 0, sizeof(tx_buff));
		switch(cmd){
			case '0':
				if(uart_manual_command(commandBuff, sizeof(commandBuff))) break;
				len = uart_manual_command_parse(commandBuff, tx_buff, sizeof(tx_buff));
				break;
			case '1':
				len = sizeof(gCamCmdSet_Initial); memcpy(tx_buff, gCamCmdSet_Initial, len);
				if(uart_cam_send_command(CAMERA_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, do_rec, UART_TIMEOUT_MSEC)) break;
				break;
			case '2':
				len = sizeof(gCamCmdSet_Menu_Init); memcpy(tx_buff, gCamCmdSet_Menu_Init, len);
				if(uart_cam_send_command(CAMERA_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, do_rec, UART_TIMEOUT_MSEC)) break;
				break;
			case '3':
				len = sizeof(gCamCmdSet_Lens_Init); memcpy(tx_buff, gCamCmdSet_Lens_Init, len);
				if(uart_cam_send_command(CAMERA_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, do_rec, UART_TIMEOUT_MSEC)) break;
				break;
			case '4':
				len = sizeof(gCamCmdSet_Reset); memcpy(tx_buff, gCamCmdSet_Reset, len);
				if(uart_cam_send_command(CAMERA_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, do_rec, UART_TIMEOUT_MSEC)) break;
				break;
			case '5': //Custom preset
				len = sizeof(gCamCmdSet_Preset); memcpy(tx_buff, gCamCmdSet_Preset, len);
				printf(CAM_CMDSET_NOTE_Preset);
				if(uart_manual_command(commandBuff, sizeof(commandBuff))) break;
				len += uart_manual_command_parse(commandBuff, tx_buff+len, 1);
				if(uart_cam_send_command(CAMERA_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, do_rec, UART_TIMEOUT_MSEC)) break;
				break;
			case '6': //VideoOut Mode
				len = sizeof(gCamCmdSet_OutMode); memcpy(tx_buff, gCamCmdSet_OutMode, len);
				printf(CAM_CMDSET_NOTE_OutMode);
				if(uart_manual_command(commandBuff, sizeof(commandBuff))) break;
				len += uart_manual_command_parse(commandBuff, tx_buff+len, 2);
				if(uart_cam_send_command(CAMERA_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, do_rec, UART_TIMEOUT_MSEC)) break;
				break;
			case '7': //Ntsc/Pal
				len = sizeof(gCamCmdSet_NtscPal); memcpy(tx_buff, gCamCmdSet_NtscPal, len);
				printf(CAM_CMDSET_NOTE_NtscPal);
				if(uart_manual_command(commandBuff, sizeof(commandBuff))) break;
				len += uart_manual_command_parse(commandBuff, tx_buff+len, 1);
				if(uart_cam_send_command(CAMERA_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, do_rec, UART_TIMEOUT_MSEC)) break;
				break;
			case '8':
				len = sizeof(gCamCmdSet_Tele); memcpy(tx_buff, gCamCmdSet_Tele, len);
				if(uart_cam_send_command(CAMERA_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, do_rec, UART_TIMEOUT_MSEC)) break;
				break;
			case '9':
				len = sizeof(gCamCmdSet_Wide); memcpy(tx_buff, gCamCmdSet_Wide, len);
				if(uart_cam_send_command(CAMERA_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, do_rec, UART_TIMEOUT_MSEC)) break;
				break;
			case 'a':
				len = sizeof(gCamCmdSet_Stop); memcpy(tx_buff, gCamCmdSet_Stop, len);
				if(uart_cam_send_command(CAMERA_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, do_rec, UART_TIMEOUT_MSEC)) break;
				break;
			case 'b': //Tele/Wide Speed
				len = sizeof(gCamCmdSet_Speed); memcpy(tx_buff, gCamCmdSet_Speed, len);
				printf(CAM_CMDSET_NOTE_Speed);
				if(uart_manual_command(commandBuff, sizeof(commandBuff))) break;
				len += uart_manual_command_parse(commandBuff, tx_buff+len, 1);
				if(uart_cam_send_command(CAMERA_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, do_rec, UART_TIMEOUT_MSEC)) break;
				break;
			case 'c': //Direct
				len = sizeof(gCamCmdSet_Direct); memcpy(tx_buff, gCamCmdSet_Direct, len);
				printf(CAM_CMDSET_NOTE_Direct);
				if(uart_manual_command(commandBuff, sizeof(commandBuff))) break;
				len += uart_manual_command_parse(commandBuff, tx_buff+len, 3);
				if(uart_cam_send_command(CAMERA_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, do_rec, UART_TIMEOUT_MSEC)) break;
				break;
			case 'd': //Zoom Direct with Focus
				len = sizeof(gCamCmdSet_DirtFocus); memcpy(tx_buff, gCamCmdSet_DirtFocus, len);
				printf(CAM_CMDSET_NOTE_DirtFocus);
				if(uart_manual_command(commandBuff, sizeof(commandBuff))) break;
				len += uart_manual_command_parse(commandBuff, tx_buff+len, 6);
				if(uart_cam_send_command(CAMERA_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, do_rec, UART_TIMEOUT_MSEC)) break;
				break;
			case 'e': //Digital Zoom Tele Limiter
				len = sizeof(gCamCmdSet_DigiLimit); memcpy(tx_buff, gCamCmdSet_DigiLimit, len);
				printf(CAM_CMDSET_NOTE_DigiLimit);
				if(uart_manual_command(commandBuff, sizeof(commandBuff))) break;
				len += uart_manual_command_parse(commandBuff, tx_buff+len, 1);
				break;
			case 'f': //Optical Zoom Wide/Tele Limitter
				len = sizeof(gCamCmdSet_OpcLimit); memcpy(tx_buff, gCamCmdSet_OpcLimit, len);
				printf(CAM_CMDSET_NOTE_OpcLimit);
				if(uart_manual_command(commandBuff, sizeof(commandBuff))) break;
				len += uart_manual_command_parse(commandBuff, tx_buff+len, 2);
				if(uart_cam_send_command(CAMERA_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, do_rec, UART_TIMEOUT_MSEC)) break;
				break;
			case 0x1B:
				return DIAG_OK;
				break;
			default:
				continue;
		}
	}
	return DIAG_OK;
}
#endif

#if defined(PTCTRL_NAME)
static int uart_pt_baudrate = PTCTRL_BAUDRATE;
int uart_set_pt_baudrate()
{
	char tmp[32];
	sprintf(tmp, "%d", uart_pt_baudrate);
	if(setenv(PTCTRL_BAUDRATE_ITEM, tmp)) return DIAG_ERROR;
	if(saveenv()) return DIAG_ERROR;
	return DIAG_OK;
}

static int uart_test_pt_tx(int parameter)
{
	uart_set_pt_baudrate();
	return uart_tx_test(PTCTRL_NAME);
}

static int uart_test_pt_rx(int parameter)
{
	uart_set_pt_baudrate();
	return uart_loopback_test(PTCTRL_NAME);
}

static int uart_test_pt_loopback(int parameter)
{
	uart_set_pt_baudrate();
	return uart_loopback_test(PTCTRL_NAME);
}

static int uart_test_pt_set_baudrate(int parameter)
{
#if defined(PTCTRL_BAUDRATE_ITEM) && defined(PTCTRL_BAUDRATE) && (PTCTRL_BAUDRATE >= 0)
	return uart_set_baudrate(PTCTRL_BAUDRATE_ITEM);
#else
	return DIAG_OK;
#endif
}

static char gPtCmdSet_Right[] = {0xE1, 0x00, 0x41, 0x0F, 0x00}; //Right
static char gPtCmdSet_Left[] = {0xE1, 0x00, 0x42, 0x0F, 0x00}; //Left
static char gPtCmdSet_Up[] = {0xE1, 0x00, 0x44, 0xF0, 0x00}; //Up
static char gPtCmdSet_Down[] = {0xE1, 0x00, 0x48, 0xF0, 0x00}; //Down
static char gPtCmdSet_Stop[] = {0xE1, 0x00, 0x00, 0x00, 0x00}; //Stop
static char gPtCmdSet_PanDeg[] = {0xE1, 0x84}; //Set Pan Degree
#define PT_CMDSET_NOTE_PanDeg "Set Pan Degree (0~35999):"
static char gPtCmdSet_TiltDeg[] = {0xE1, 0x85}; //Set Tilt Degree
#define PT_CMDSET_NOTE_TiltDeg "Set Tilt Degree (-600~9600):"
static char gPtCmdSet_GetDeg[] = {0xE1, 0x5E, 0x34, 0x00, 0x00}; //Get Pan/Tilt Degree
static char gPtCmdSet_GetVer[] = {0xE1, 0x5C, 0xE2, 0x01, 0x00}; //Get Firmware Version
static char gPtCmdSet_GetPiH[] = {0xE1, 0xB0, 0x01, 0x00, 0x00}; //Get H PI Info
static char gPtCmdSet_GetPiV[] = {0xE1, 0xB0, 0x02, 0x00, 0x00}; //Get V PI Info
static char gPtCmdSet_Test[] = {0xE1, 0xB1, 0xE2}; //Production Test
#define PT_CMDSET_NOTE_Test "\
Production Test [p q]\n\
 p: 0:Entry Production Test Mode, 1:H Moto Continuous, 2:V Motor Continuous\n\
 q: p=0:0xFF\n\
    p=1:Direct, +1=R, 0=Stop, -1=L\n\
    p=2:Direct, +1=U, 0=Stop, -1=D\n"
static char gPtCmdSet_Test_Start[] = {0xE1, 0xB1, 0xE2, 0x00, 0xFF}; //Entry Production Test Mode
static char gPtCmdSet_Test_H_R[] = {0xE1, 0xB1, 0xE2, 0x01, 0x01}; //H Moto Continuous(R)
static char gPtCmdSet_Test_H_L[] = {0xE1, 0xB1, 0xE2, 0x01, 0xFF}; //H Moto Continuous(L)
static char gPtCmdSet_Test_V_U[] = {0xE1, 0xB1, 0xE2, 0x02, 0x01}; //V Moto Continuous(U)
static char gPtCmdSet_Test_V_D[] = {0xE1, 0xB1, 0xE2, 0x02, 0xFF}; //V Moto Continuous(D)

int uart_PtCmdParse_PTDeg(char *cmdStr, char *cmdBuff)
{
	int degree = (int)(simple_strtol(cmdStr, NULL, 10)) & 0xffff;
	cmdBuff[0] = (degree >> 8) & 0xff;
	cmdBuff[1] = degree & 0xff;
	return 2;
}

void uart_Display_PTDeg(char *rx_buff)
{
	ushort pan_degree = (ushort)((rx_buff[2] << 8) + (rx_buff[3]));
	short tilt_degree = (short)((rx_buff[4] << 8) + (rx_buff[5]));
	if(tilt_degree > 0) tilt_degree-=2;
	printf("Pan/Tilt Degree:%u/%d\n", pan_degree, tilt_degree);
}

void uart_Display_Version(char *rx_buff)
{
	printf("Firmware Version:%u.%u.%u.%u\n", rx_buff[2], rx_buff[3], rx_buff[4], rx_buff[5]);
}

int uart_pt_send_command(char *name, char *tx_buff, char *rx_buff, int tx_len, int rx_len, int *len, int rec_len, int timeout)
{
	int i = 0, j = 0;
	int timeout_msec = timeout;
	char sum = 0;

	if(uart_set_pt_baudrate()) return DIAG_ERROR;

	struct serial_device *dev;
	dev = get_serial_device_by_name(name);
	if(dev == NULL) return DIAG_ERROR;
	dev->init();

	if(((*len) + 1) > tx_len) return DIAG_ERROR;

	sum = 0;
	for(i = 1; i < (*len); i++) sum += (tx_buff[i]);
	tx_buff[(*len)] = sum; (*len)++; //Checksum Byte

	printf("Transmit(size=%d)->\n", (*len));
	print_buffer(0, (void*)tx_buff, 1, (*len), 0);

	for(i = 0; i < (*len); i++){
		dev->putc(tx_buff[i]);
	}
	if(rec_len > 0){
		memset(rx_buff, 0, rx_len);
		timeout_msec = timeout;
		j = 0;
		while(j < rx_len){
			if(dev->tstc()) {
				timeout_msec = timeout;
				rx_buff[j] = (char)dev->getc();
				j++;
				if(j >= rec_len) break;
			}
			if(timeout_msec-- <= 0) { printf("Rx Time out!!\n"); return DIAG_ERROR; }
			udelay(1000);
		}
		printf("Response(size=%d)->\n", j);
		print_buffer(0, (void*)rx_buff, 1, j, 0);
		sum = 0;
		for(i = 1; i < (j - 1); i++) sum += (rx_buff[i]);
		if(sum != rx_buff[j - 1]){
			printf("Checksum Error!!(%X)\n", sum);
			return DIAG_ERROR;
		}
	}

	return DIAG_OK;
}

#define TEST_PAN 0x01
#define TEST_TILT 0x02
static int uart_test_pt_send_command(int parameter)
{
	int len = 0;
	int rec_len = 0;
	int step = 0;
	char cmd = 0;
	char tx_buff[UART_MAX_TEST_LEN];
	char rx_buff[UART_MAX_TEST_LEN];
	char commandBuff[UART_MAX_TEST_LEN * 3 + 2];

	while(1){

		printf("\n");
		printf("Select command set:\n");
		printf("  [0]Manual command.(w/o Checksum Byte)\n");
		printf("  [1]Right.\n");
		printf("  [2]Left.\n");
		printf("  [3]Up.\n");
		printf("  [4]Down.\n");
		printf("  [5]Stop.\n");
		printf("  [6]Set Pan Degree.\n");
		printf("  [7]Set Tilt Degree.\n");
		printf("  [8]Get Degree.\n");
		printf("  [9]Production Test Mode.\n");
		printf("  [a]Get Firmware Version.\n");
		printf("  [ESC]Exit.\n");
		printf("->");
		cmd = getc();
		printf("\n");

		memset(commandBuff, 0, sizeof(commandBuff));
		memset(tx_buff, 0, sizeof(tx_buff));
		rec_len = 0;
		switch(cmd){
			case '0':
				rec_len = uart_manual_response_size();
				if(uart_manual_command(commandBuff, sizeof(commandBuff))) break;
				len = uart_manual_command_parse(commandBuff, tx_buff, sizeof(tx_buff));
				if(uart_pt_send_command(PTCTRL_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, rec_len, UART_TIMEOUT_MSEC)) break;
				break;
			case '1':
				len = sizeof(gPtCmdSet_Right); memcpy(tx_buff, gPtCmdSet_Right, len);
				if(uart_pt_send_command(PTCTRL_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, rec_len, UART_TIMEOUT_MSEC)) break;
				break;
			case '2':
				len = sizeof(gPtCmdSet_Left); memcpy(tx_buff, gPtCmdSet_Left, len);
				if(uart_pt_send_command(PTCTRL_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, rec_len, UART_TIMEOUT_MSEC)) break;
				break;
			case '3':
				len = sizeof(gPtCmdSet_Up); memcpy(tx_buff, gPtCmdSet_Up, len);
				if(uart_pt_send_command(PTCTRL_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, rec_len, UART_TIMEOUT_MSEC)) break;
				break;
			case '4':
				len = sizeof(gPtCmdSet_Down); memcpy(tx_buff, gPtCmdSet_Down, len);
				if(uart_pt_send_command(PTCTRL_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, rec_len, UART_TIMEOUT_MSEC)) break;
				break;
			case '5':
				len = sizeof(gPtCmdSet_Stop); memcpy(tx_buff, gPtCmdSet_Stop, len);
				if(uart_pt_send_command(PTCTRL_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, rec_len, UART_TIMEOUT_MSEC)) break;
				break;
			case '6':
				len = sizeof(gPtCmdSet_PanDeg); memcpy(tx_buff, gPtCmdSet_PanDeg, len);
				if(get_line(PT_CMDSET_NOTE_PanDeg, commandBuff, 7, -1, "0123456789", NULL, NULL) < 0) break;
				len += uart_PtCmdParse_PTDeg(commandBuff, tx_buff+len);
				*(tx_buff+len) = 0xFF; len++; //speed
				if(uart_pt_send_command(PTCTRL_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, rec_len, UART_TIMEOUT_MSEC)) break;
				break;
			case '7':
				len = sizeof(gPtCmdSet_TiltDeg); memcpy(tx_buff, gPtCmdSet_TiltDeg, len);
				if(get_line(PT_CMDSET_NOTE_TiltDeg, commandBuff, 6, -1, "0123456789-", NULL, NULL) < 0) break;
				len += uart_PtCmdParse_PTDeg(commandBuff, tx_buff+len);
				*(tx_buff+len) = 0xFF; len++; //speed
				if(uart_pt_send_command(PTCTRL_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, rec_len, UART_TIMEOUT_MSEC)) break;
				break;
			case '8':
				len = sizeof(gPtCmdSet_GetDeg); memcpy(tx_buff, gPtCmdSet_GetDeg, len); rec_len = 7;
				if(uart_pt_send_command(PTCTRL_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, rec_len, UART_TIMEOUT_MSEC)) break;
				uart_Display_PTDeg(rx_buff);
				break;
			case '9':
				if(get_line("(P)Pan/(T)Tilt/(A)All:", commandBuff, 3, -1, "PTApta", NULL, NULL) < 0) break;

				if((commandBuff[0] == 'A') || (commandBuff[0] == 'a')){
					step = (TEST_PAN | TEST_TILT);
				}else
				if((commandBuff[0] == 'P') || (commandBuff[0] == 'p')){
					step = TEST_PAN;
				}else
				if((commandBuff[0] == 'T') || (commandBuff[0] == 't')){
					step = TEST_TILT;
				}else{
					break;
				}

				if(step & TEST_PAN){
					len = sizeof(gPtCmdSet_Test_H_R); memcpy(tx_buff, gPtCmdSet_Test_H_R, len);
					if(uart_pt_send_command(PTCTRL_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, rec_len, UART_TIMEOUT_MSEC)) break;
					memset(tx_buff, 0, sizeof(tx_buff));
				}
				if(step & TEST_TILT){
					len = sizeof(gPtCmdSet_Test_V_U); memcpy(tx_buff, gPtCmdSet_Test_V_U, len);
					if(uart_pt_send_command(PTCTRL_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, rec_len, UART_TIMEOUT_MSEC)) break;
					memset(tx_buff, 0, sizeof(tx_buff));
				}
				if(step & (TEST_PAN | TEST_TILT)){
					len = sizeof(gPtCmdSet_Test_Start); memcpy(tx_buff, gPtCmdSet_Test_Start, len);
					if(uart_pt_send_command(PTCTRL_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, rec_len, UART_TIMEOUT_MSEC)) break;
				}
				break;
			case 'a':
				len = sizeof(gPtCmdSet_GetVer); memcpy(tx_buff, gPtCmdSet_GetVer, len); rec_len = 7;
				if(uart_pt_send_command(PTCTRL_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, rec_len, UART_TIMEOUT_MSEC)) break;
				uart_Display_Version(rx_buff);
				break;
			case 0x1B:
				return DIAG_OK;
				break;
			default:
				continue;
		}
	}
	return DIAG_OK;
}

int do_ptctrl (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	int len = 0;
	int rec_len = 0;
	int step = 0;
	char tx_buff[UART_MAX_TEST_LEN];
	char rx_buff[UART_MAX_TEST_LEN];

	if (argc < 2){
		printf ("Usage:\n%s\n", cmdtp->usage);
		return -1;
	}else{
		memset(tx_buff, 0, sizeof(tx_buff));
		if(strcmp(argv[1], "right") == 0){
			len = sizeof(gPtCmdSet_Right); memcpy(tx_buff, gPtCmdSet_Right, len); rec_len = 7;
			if(uart_pt_send_command(PTCTRL_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, rec_len, UART_TIMEOUT_MSEC)) return DIAG_ERROR;
		}else
		if(strcmp(argv[1], "left") == 0){
			len = sizeof(gPtCmdSet_Left); memcpy(tx_buff, gPtCmdSet_Left, len); rec_len = 7;
			if(uart_pt_send_command(PTCTRL_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, rec_len, UART_TIMEOUT_MSEC)) return DIAG_ERROR;
		}else
		if(strcmp(argv[1], "up") == 0){
			len = sizeof(gPtCmdSet_Up); memcpy(tx_buff, gPtCmdSet_Up, len); rec_len = 7;
			if(uart_pt_send_command(PTCTRL_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, rec_len, UART_TIMEOUT_MSEC)) return DIAG_ERROR;
		}else
		if(strcmp(argv[1], "down") == 0){
			len = sizeof(gPtCmdSet_Down); memcpy(tx_buff, gPtCmdSet_Down, len); rec_len = 7;
			if(uart_pt_send_command(PTCTRL_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, rec_len, UART_TIMEOUT_MSEC)) return DIAG_ERROR;
		}else
		if(strcmp(argv[1], "stop") == 0){
			len = sizeof(gPtCmdSet_Stop); memcpy(tx_buff, gPtCmdSet_Stop, len);
			if(uart_pt_send_command(PTCTRL_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, rec_len, UART_TIMEOUT_MSEC)) return DIAG_ERROR;
		}else
		if(strcmp(argv[1], "pandeg") == 0){
			if(argc < 3){
				printf ("Usage:\n%s\n", cmdtp->usage);
				return -1;
			}
			len = sizeof(gPtCmdSet_PanDeg); memcpy(tx_buff, gPtCmdSet_PanDeg, len);
			len += uart_PtCmdParse_PTDeg(argv[2], tx_buff+len);
			*(tx_buff+len) = 0xFF; len++; //speed
			if(uart_pt_send_command(PTCTRL_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, rec_len, UART_TIMEOUT_MSEC)) return DIAG_ERROR;
		}else
		if(strcmp(argv[1], "tiltdeg") == 0){
			if(argc < 3){
				printf ("Usage:\n%s\n", cmdtp->usage);
				return -1;
			}
			len = sizeof(gPtCmdSet_TiltDeg); memcpy(tx_buff, gPtCmdSet_TiltDeg, len);
			len += uart_PtCmdParse_PTDeg(argv[2], tx_buff+len);
			*(tx_buff+len) = 0xFF; len++; //speed
			if(uart_pt_send_command(PTCTRL_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, rec_len, UART_TIMEOUT_MSEC)) return DIAG_ERROR;
		}else
		if(strcmp(argv[1], "getdeg") == 0){
			len = sizeof(gPtCmdSet_GetDeg); memcpy(tx_buff, gPtCmdSet_GetDeg, len); rec_len = 7;
			if(uart_pt_send_command(PTCTRL_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, rec_len, UART_TIMEOUT_MSEC)) return DIAG_ERROR;
			uart_Display_PTDeg(rx_buff);
		}else
		if(strcmp(argv[1], "ver") == 0){
			len = sizeof(gPtCmdSet_GetVer); memcpy(tx_buff, gPtCmdSet_GetVer, len); rec_len = 7;
			if(uart_pt_send_command(PTCTRL_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, rec_len, UART_TIMEOUT_MSEC)) return DIAG_ERROR;
			uart_Display_Version(rx_buff);
		}else
		if(strcmp(argv[1], "test") == 0){
			if(argc < 3){
				printf ("Usage:\n%s\n", cmdtp->usage);
				return -1;
			}
			if(strcmp(argv[2], "all") == 0){
				step = (TEST_PAN | TEST_TILT);
			}else
			if(strcmp(argv[2], "pan") == 0){
				step = TEST_PAN;
			}else
			if(strcmp(argv[2], "tilt") == 0){
				step = TEST_TILT;
			}else{
				printf ("Usage:\n%s\n", cmdtp->usage);
				return -1;
			}
			if(step & TEST_PAN){
				len = sizeof(gPtCmdSet_Test_H_R); memcpy(tx_buff, gPtCmdSet_Test_H_R, len);
				if(uart_pt_send_command(PTCTRL_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, rec_len, UART_TIMEOUT_MSEC)) return DIAG_ERROR;
				memset(tx_buff, 0, sizeof(tx_buff));
			}
			if(step & TEST_TILT){
				len = sizeof(gPtCmdSet_Test_V_U); memcpy(tx_buff, gPtCmdSet_Test_V_U, len);
				if(uart_pt_send_command(PTCTRL_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, rec_len, UART_TIMEOUT_MSEC)) return DIAG_ERROR;
				memset(tx_buff, 0, sizeof(tx_buff));
			}
			if(step & (TEST_PAN | TEST_TILT)){
				len = sizeof(gPtCmdSet_Test_Start); memcpy(tx_buff, gPtCmdSet_Test_Start, len);
				if(uart_pt_send_command(PTCTRL_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, rec_len, UART_TIMEOUT_MSEC)) return DIAG_ERROR;
			}
		}else{
			printf ("Usage:\n%s\n", cmdtp->usage);
			return -1;
		}
	}
	return 0;
}

U_BOOT_CMD(ptctrl, 3, 0, do_ptctrl,
	"Pan/Tilt control",
	"Pan/Tilt control\n\
	ptctrl right - Pan right\n\
	ptctrl left - Pan left\n\
	ptctrl up - Tilt up\n\
	ptctrl down - Tilt down\n\
	ptctrl stop - Pan/Tilt stop\n\
	ptctrl pandeg [degree] - Set Pan degree\n\
	ptctrl tiltdeg [degree] - Set Tilt degree\n\
	ptctrl getdeg - Get Degree\n\
	ptctrl test [all/pan/tilt] - Production Test Mode\n\
	ptctrl ver - Get Firmware Version\n\
	"
);

#elif defined(PTCTRL_HERMES_NAME)

static int uart_pt_baudrate = PTCTRL_BAUDRATE;
void uart_hermes_calc_checksum(char *cmd_str_p, int *cmd_len_p)
{
	unsigned char Sum = 0;
	int i;

	for (i = 0; i < *cmd_len_p; i++)
	{
		Sum += cmd_str_p[i];
	}
	if (Sum & 0x80)
	{
		Sum = (Sum & 0x7F) ^ 0x1;
	}
	cmd_str_p[(*cmd_len_p)++] = Sum;
	cmd_str_p[(*cmd_len_p)++] = 0xFF;
}

int uart_hermes_check_checksum(char *cmd_str_p, int cmd_len)
{
	unsigned char Sum = 0;
	int i;

	if(cmd_len < 2) return DIAG_ERROR;

	for (i = 0; i < cmd_len - 2; i++)
	{
		Sum += cmd_str_p[i];
	}
	if (Sum & 0x80)
	{
		Sum = (Sum & 0x7F) ^ 0x1;
	}
	if((cmd_str_p[cmd_len - 2] == Sum) && (cmd_str_p[cmd_len - 1] == 0xFF)){
		return DIAG_OK;
	}else{
		printf("Checksum Error!!(%02X)\n", Sum);
		return DIAG_ERROR;
	}
}

int uart_hermes_check_ack(char *cmd_str_p, int cmd_len)
{
	if(cmd_len >= 2)
	{
		if((cmd_str_p[0] == 0x00) && (cmd_str_p[1] == 0xCC))
		{
			return DIAG_OK;
		}
		printf("NOT ACK:0x%02X 0x%02X\n", cmd_str_p[0], cmd_str_p[1]);
	}
	return DIAG_ERROR;
}

int uart_set_pt_baudrate()
{
	char tmp[32];
	int baudrate = 0;
	
	if((baudrate = (int)simple_strtoul(getenv(PTCTRL_BAUDRATE_ITEM), NULL, 10)) <= 0){
		printf("Error: getenv(%s)\n", PTCTRL_BAUDRATE_ITEM);
		return DIAG_ERROR;
	}else{
		if(baudrate != uart_pt_baudrate){
			sprintf(tmp, "%d", uart_pt_baudrate);
			if(setenv(PTCTRL_BAUDRATE_ITEM, tmp)) return DIAG_ERROR;
			if(saveenv()) return DIAG_ERROR;
		}
		return DIAG_OK;
	}
}

static int uart_test_pt_tx(int parameter)
{
	uart_set_pt_baudrate();
	return uart_tx_test(PTCTRL_HERMES_NAME);
}

static int uart_test_pt_rx(int parameter)
{
	uart_set_pt_baudrate();
	return uart_loopback_test(PTCTRL_HERMES_NAME);
}

static int uart_test_pt_loopback(int parameter)
{
	uart_set_pt_baudrate();
	return uart_loopback_test(PTCTRL_HERMES_NAME);
}

static int uart_test_pt_set_baudrate(int parameter)
{
#if defined(PTCTRL_BAUDRATE_ITEM) && defined(PTCTRL_BAUDRATE) && (PTCTRL_BAUDRATE >= 0)
	return uart_set_baudrate(PTCTRL_BAUDRATE_ITEM);
#else
	return DIAG_OK;
#endif
}

#define MCU_LIB_STD_TIMEOUT		UART_TIMEOUT_MSEC
#define DOWNLOAD_ADDRESS 		0x4000
#define TRANSMIT_STEP			12
#define MCU_STATUS_OK			0x40


#define TYPE_BYTE				2
#define SUBTYPE_BYTE			3
#define LOCATION_BYTE			4
#define CHECKSUM_BYTE			8

#define TYPE_DOME				0x01
#define SUBTYPE_LOCATION_PAN	0x05
#define SUBTYPE_LOCATION_TILT	0x06	

#define DEFAULT_PAN_MIN 0
#define DEFAULT_PAN_MAX 25599

#define DEFAULT_TILT_MIN 0
#define DEFAULT_TILT_MAX 4079

typedef enum
{
	PTSCANNER_MAINCMD_TYPE_NONE = 0,
	PTSCANNER_MAINCMD_TYPE_DOME,
	PTSCANNER_MAINCMD_TYPE_SYSTEM,
} PTSCANNER_MAINCMD_TYPE;

typedef enum
{
	PTSCANNER_DOMECMD_SUBTYPE_NONE = 0,
	PTSCANNER_DOMECMD_SUBTYPE_PAN,
	PTSCANNER_DOMECMD_SUBTYPE_TILT,
	PTSCANNER_DOMECMD_SUBTYPE_STOP,
	PTSCANNER_DOMECMD_SUBTYPE_PRESET,
	PTSCANNER_DOMECMD_SUBTYPE_PANLOCATION,
	PTSCANNER_DOMECMD_SUBTYPE_TILTLOCATION,
	PTSCANNER_DOMECMD_SUBTYPE_PANCONTINUOUS,
	PTSCANNER_DOMECMD_SUBTYPE_TILTCONTINUOUS,
	PTSCANNER_DOMECMD_SUBTYPE_EXCITATIONMODE,
} PTSCANNER_DOMECMD_SUBTYPE;

typedef enum
{
	PTSCANNER_SYSTEMCMD_SUBTYPE_NONE = 0,
	PTSCANNER_SYSTEMCMD_SUBTYPE_RESPONSE,
	PTSCANNER_SYSTEMCMD_SUBTYPE_GET_VERSION,
} PTSCANNER_SYSTEMCMD_SUBTYPE;

#define PTSCANNER_EXTRACMD_MODE_RELATIVE	0x00
#define PTSCANNER_EXTRACMD_MODE_ABSOLUTE	0x40

#define PTSCANNER_EXTRACMD_FORCEDIR_AUTO	0x00
#define PTSCANNER_EXTRACMD_FORCEDIR_LEFT	0x20
#define PTSCANNER_EXTRACMD_FORCEDIR_RIGHT	0x30

#define PTSCANNER_EXTRACMD_RESPONSE			0x08
#define PTSCANNER_EXTRACMD_HIGHSPEED		0x04

#define SPEEDTABLE_MAX (8)
static char highspeed_table[SPEEDTABLE_MAX] = {1,5,10,15,20,25,30,35};

static char gPtCmdSet_Init[] = {0x03, 0x22, 0x22};
static char gPtCmdSet_Right[] = {0x00, 0x00, PTSCANNER_MAINCMD_TYPE_DOME, PTSCANNER_DOMECMD_SUBTYPE_PANCONTINUOUS, PTSCANNER_EXTRACMD_FORCEDIR_RIGHT | PTSCANNER_EXTRACMD_HIGHSPEED}; //Right
static char gPtCmdSet_Left[] = {0x00, 0x00, PTSCANNER_MAINCMD_TYPE_DOME, PTSCANNER_DOMECMD_SUBTYPE_PANCONTINUOUS, PTSCANNER_EXTRACMD_FORCEDIR_LEFT | PTSCANNER_EXTRACMD_HIGHSPEED}; //Left
static char gPtCmdSet_Up[] = {0x00, 0x00, PTSCANNER_MAINCMD_TYPE_DOME, PTSCANNER_DOMECMD_SUBTYPE_TILTCONTINUOUS, PTSCANNER_EXTRACMD_FORCEDIR_LEFT | PTSCANNER_EXTRACMD_HIGHSPEED}; //Up
static char gPtCmdSet_Down[] = {0x00, 0x00, PTSCANNER_MAINCMD_TYPE_DOME, PTSCANNER_DOMECMD_SUBTYPE_TILTCONTINUOUS, PTSCANNER_EXTRACMD_FORCEDIR_RIGHT | PTSCANNER_EXTRACMD_HIGHSPEED}; //Down
static char gPtCmdSet_Stop[] = {0x00, 0x00, PTSCANNER_MAINCMD_TYPE_DOME, PTSCANNER_DOMECMD_SUBTYPE_STOP}; //Stop
static char gPtCmdSet_PanPos[] = {0x00, 0x00, PTSCANNER_MAINCMD_TYPE_DOME, PTSCANNER_DOMECMD_SUBTYPE_PAN, PTSCANNER_EXTRACMD_MODE_ABSOLUTE | PTSCANNER_EXTRACMD_HIGHSPEED}; //Set Pan position
static char gPtCmdSet_TiltPos[] = {0x00, 0x00, PTSCANNER_MAINCMD_TYPE_DOME, PTSCANNER_DOMECMD_SUBTYPE_TILT, PTSCANNER_EXTRACMD_MODE_ABSOLUTE | PTSCANNER_EXTRACMD_HIGHSPEED}; //Set Tilt position
static char gPtCmdSet_GetPanPos[] = {0x00, 0x00, PTSCANNER_MAINCMD_TYPE_DOME, PTSCANNER_DOMECMD_SUBTYPE_PANLOCATION}; //Get Pan position
static char gPtCmdSet_GetTiltPos[] = {0x00, 0x00, PTSCANNER_MAINCMD_TYPE_DOME, PTSCANNER_DOMECMD_SUBTYPE_TILTLOCATION}; //Get Tilt position
static char gPtCmdSet_GetVer[] = {0x00, 0x00, PTSCANNER_MAINCMD_TYPE_SYSTEM, PTSCANNER_SYSTEMCMD_SUBTYPE_GET_VERSION}; //Get Firmware Version

static int curr_Speed = 0;
int uart_PtCmd_SetSpeed(char *cmdStr, char *cmdBuff)
{
	int len = 0;
	curr_Speed = (int)(simple_strtol(cmdStr, NULL, 10)) & 0xffff;
	cmdBuff[len++] = ((highspeed_table[curr_Speed] >> 12) & 0x000f);
	cmdBuff[len++] = ((highspeed_table[curr_Speed] >>  8) & 0x000f);
	cmdBuff[len++] = ((highspeed_table[curr_Speed] >>  4) & 0x000f);
	cmdBuff[len++] = ((highspeed_table[curr_Speed]      ) & 0x000f);
	return len;
}

int uart_PtCmd_InputSpeed(char *cmdBuff)
{
	int ret = 0;
	int len = 0;
	char tmp[3];
	printf("Set Speed (0~%d)(%d):", SPEEDTABLE_MAX-1, curr_Speed);
	if(get_line(NULL, tmp, sizeof(tmp), -1, "01234567", NULL, NULL) > 0) curr_Speed = (int)(simple_strtoul(tmp, NULL, 10));
	cmdBuff[len++] = ((highspeed_table[curr_Speed] >> 12) & 0x000f);
	cmdBuff[len++] = ((highspeed_table[curr_Speed] >>  8) & 0x000f);
	cmdBuff[len++] = ((highspeed_table[curr_Speed] >>  4) & 0x000f);
	cmdBuff[len++] = ((highspeed_table[curr_Speed]      ) & 0x000f);
	return len;
}

int uart_PtCmd_SetPTPos(char *cmdStr, char *cmdBuff)
{
	int len = 0;
	int pos = (int)(simple_strtol(cmdStr, NULL, 10)) & 0xffff;
	cmdBuff[len++] = ((pos >> 12) & 0x000f);
	cmdBuff[len++] = ((pos >>  8) & 0x000f);
	cmdBuff[len++] = ((pos >>  4) & 0x000f);
	cmdBuff[len++] = ((pos      ) & 0x000f);
	return len;
}

int uart_PtCmd_InputPanPos(char *cmdBuff)
{
	int pos = 0;
	int len = 0;
	char tmp[7];
	printf("Set Pan position (%d~%d):", DEFAULT_PAN_MIN, DEFAULT_PAN_MAX);
	if(get_line(NULL, tmp, sizeof(tmp), -1, "0123456789", NULL, NULL) > 0){
		len = uart_PtCmd_SetPTPos(tmp, cmdBuff);
	}
	return len;
}

int uart_PtCmd_InputTiltPos(char *cmdBuff)
{
	int pos = 0;
	int len = 0;
	char tmp[6];
	printf("Set Tilt position (%d~%d):", DEFAULT_TILT_MIN, DEFAULT_TILT_MAX);
	if(get_line(NULL, tmp, sizeof(tmp), -1, "0123456789", NULL, NULL) > 0){
		len = uart_PtCmd_SetPTPos(tmp, cmdBuff);
	}
	return len;
}

void uart_Display_PanPos(char *rx_buff)
{
	unsigned short pan_pos = (unsigned short)((rx_buff[4] << 12) + (rx_buff[5] << 8) + (rx_buff[6] << 4) + (rx_buff[7]));
	printf("tilt_pos=%u\n", pan_pos);
}

void uart_Display_TiltPos(char *rx_buff)
{
	unsigned short tilt_pos = (unsigned short)((rx_buff[4] << 12) + (rx_buff[5] << 8) + (rx_buff[6] << 4) + (rx_buff[7]));
	printf("pan_pos=%u\n", tilt_pos);
}

void uart_Display_Version(char *rx_buff)
{
	printf("Firmware Version:%u.%u\n", (rx_buff[4] << 4) + (rx_buff[5]), (rx_buff[6] << 4) + (rx_buff[7]));
}

int uart_pt_send_command(char *name, char *tx_buff, char *rx_buff, int tx_len, int rx_len, int *len, int *rec_len, int timeout)
{
	int i = 0, j = 0;
	int timeout_msec = timeout;

	if(uart_set_pt_baudrate()) return DIAG_ERROR;

	struct serial_device *dev;
	dev = get_serial_device_by_name(name);
	if(dev == NULL) return DIAG_ERROR;
	dev->init();

	if(((*len) + 1) > tx_len) return DIAG_ERROR;

	printf("Transmit(size=%d)->\n", (*len));
	print_buffer(0, (void*)tx_buff, 1, (*len), 0);

	for(i = 0; i < (*len); i++){
		dev->putc(tx_buff[i]);
	}
	if((rx_buff != NULL) && (rx_len > 0) && (rec_len != NULL) && ((*rec_len) > 0)){
		memset(rx_buff, 0, rx_len);
		timeout_msec = timeout;
		j = 0;
		while(j < rx_len){
			if(dev->tstc()) {
				timeout_msec = timeout;
				rx_buff[j] = (char)dev->getc();
				j++;
				if(j >= (*rec_len)) break;
			}
			if(timeout_msec-- <= 0) { printf("Rx Time out!!\n"); return DIAG_ERROR; }
			udelay(1000);
		}
		printf("Response(size=%d)->\n", j);
		print_buffer(0, (void*)rx_buff, 1, j, 0);
		(*rec_len) = j;
	}

	return DIAG_OK;
}

void MCU_Reset()
{
#ifdef GPIO_MCU_RST
	GPIO_Out(GPIO_MCU_RST, GPIO_HIGH);
	udelay(500000);
	GPIO_Out(GPIO_MCU_RST, GPIO_LOW);
	udelay(500000);
	GPIO_Out(GPIO_MCU_RST, GPIO_HIGH);
	udelay(500000);
#endif
}

int MCU_FW_Go()
{
	int len = 0;
	int rec_len = 0;
	char tx_buff[UART_MAX_TEST_LEN];
	len = sizeof(gPtCmdSet_Init); memcpy(tx_buff, gPtCmdSet_Init, len);
	if(uart_pt_send_command(PTCTRL_HERMES_NAME, tx_buff, NULL, sizeof(tx_buff), 0, &len, NULL, UART_TIMEOUT_MSEC)) return DIAG_ERROR;
	return DIAG_OK;
}

static int uart_test_pt_send_command(int parameter)
{
	int len = 0;
	int ret = 0;
	int rec_len = 0;
	int step = 0;
	char cmd = 0;
	char tx_buff[UART_MAX_TEST_LEN];
	char rx_buff[UART_MAX_TEST_LEN];
	char commandBuff[UART_MAX_TEST_LEN * 3 + 2];

	while(1){

		printf("\n");
		printf("Select command set:\n");
		printf("  [0]Init\n");
		printf("  [1]Right.\n");
		printf("  [2]Left.\n");
		printf("  [3]Up.\n");
		printf("  [4]Down.\n");
		printf("  [5]Stop.\n");
		printf("  [6]Set Pan position.\n");
		printf("  [7]Set Tilt position.\n");
		printf("  [8]Get Pan position.\n");
		printf("  [9]Get Tilt position.\n");
		printf("  [a]Get Firmware Version.\n");
		printf("  [r]Reset MCU\n");
		printf("  [ESC]Exit.\n");
		printf("->");
		cmd = getc();
		printf("\n");

		memset(commandBuff, 0, sizeof(commandBuff));
		memset(tx_buff, 0, sizeof(tx_buff));
		rec_len = 0;
		switch(cmd){
			case '0': //Init
				MCU_FW_Go();
				break;
			case '1': //Right
				len = sizeof(gPtCmdSet_Right); memcpy(tx_buff, gPtCmdSet_Right, len);
				len += uart_PtCmd_InputSpeed(tx_buff+len);	//Speed
				uart_hermes_calc_checksum(tx_buff, &len);	//Add Checksum Byte & Tail Bye
				if(uart_pt_send_command(PTCTRL_HERMES_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, &rec_len, UART_TIMEOUT_MSEC)) break;
				break;
			case '2': //Left
				len = sizeof(gPtCmdSet_Left); memcpy(tx_buff, gPtCmdSet_Left, len);
				len += uart_PtCmd_InputSpeed(tx_buff+len);	//Speed
				uart_hermes_calc_checksum(tx_buff, &len);	//Add Checksum Byte & Tail Bye
				if(uart_pt_send_command(PTCTRL_HERMES_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, &rec_len, UART_TIMEOUT_MSEC)) break;
				break;
			case '3': //Up
				len = sizeof(gPtCmdSet_Up); memcpy(tx_buff, gPtCmdSet_Up, len);
				len += uart_PtCmd_InputSpeed(tx_buff+len);	//Speed
				uart_hermes_calc_checksum(tx_buff, &len);	//Add Checksum Byte & Tail Bye
				if(uart_pt_send_command(PTCTRL_HERMES_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, &rec_len, UART_TIMEOUT_MSEC)) break;
				break;
			case '4': //Down
				len = sizeof(gPtCmdSet_Down); memcpy(tx_buff, gPtCmdSet_Down, len);
				len += uart_PtCmd_InputSpeed(tx_buff+len);	//Speed
				uart_hermes_calc_checksum(tx_buff, &len);	//Add Checksum Byte & Tail Bye
				if(uart_pt_send_command(PTCTRL_HERMES_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, &rec_len, UART_TIMEOUT_MSEC)) break;
				break;
			case '5': //Stop
				len = sizeof(gPtCmdSet_Stop); memcpy(tx_buff, gPtCmdSet_Stop, len);
				uart_hermes_calc_checksum(tx_buff, &len);	//Add Checksum Byte & Tail Bye
				if(uart_pt_send_command(PTCTRL_HERMES_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, &rec_len, UART_TIMEOUT_MSEC)) break;
				break;
			case '6': //Set PanPos
				len = sizeof(gPtCmdSet_PanPos); memcpy(tx_buff, gPtCmdSet_PanPos, len);
				len += uart_PtCmd_InputSpeed(tx_buff+len);	//Speed
				len += uart_PtCmd_InputPanPos(tx_buff+len);	//Position
				uart_hermes_calc_checksum(tx_buff, &len);	//Add Checksum Byte & Tail Bye
				if(uart_pt_send_command(PTCTRL_HERMES_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, &rec_len, UART_TIMEOUT_MSEC)) break;
				break;
			case '7': //Set TiltPos
				len = sizeof(gPtCmdSet_TiltPos); memcpy(tx_buff, gPtCmdSet_TiltPos, len);
				len += uart_PtCmd_InputSpeed(tx_buff+len);	//Speed
				len += uart_PtCmd_InputTiltPos(tx_buff+len);	//Position
				uart_hermes_calc_checksum(tx_buff, &len);	//Add Checksum Byte & Tail Bye
				if(uart_pt_send_command(PTCTRL_HERMES_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, &rec_len, UART_TIMEOUT_MSEC)) break;
				break;
			case '8': //Get PanPos
				len = sizeof(gPtCmdSet_GetPanPos); memcpy(tx_buff, gPtCmdSet_GetPanPos, len); rec_len = 10;
				uart_hermes_calc_checksum(tx_buff, &len);	//Add Checksum Byte & Tail Bye
				if(uart_pt_send_command(PTCTRL_HERMES_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, &rec_len, UART_TIMEOUT_MSEC)) break;
				if(uart_hermes_check_checksum(rx_buff, rec_len)) break;
				else uart_Display_PanPos(rx_buff);
				break;
			case '9': //Get TiltPos
				len = sizeof(gPtCmdSet_GetTiltPos); memcpy(tx_buff, gPtCmdSet_GetTiltPos, len); rec_len = 10;
				uart_hermes_calc_checksum(tx_buff, &len);	//Add Checksum Byte & Tail Bye
				if(uart_pt_send_command(PTCTRL_HERMES_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, &rec_len, UART_TIMEOUT_MSEC)) break;
				uart_Display_TiltPos(rx_buff);
				break;
			case 'a': // Get Version
				len = sizeof(gPtCmdSet_GetVer); memcpy(tx_buff, gPtCmdSet_GetVer, len); rec_len = 10;
				uart_hermes_calc_checksum(tx_buff, &len);	//Add Checksum Byte & Tail Bye
				if(uart_pt_send_command(PTCTRL_HERMES_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, &rec_len, UART_TIMEOUT_MSEC)) break;
				if(uart_hermes_check_checksum(rx_buff, rec_len)) break;
				else uart_Display_Version(rx_buff);
				break;
			case 'r': //Reset MCU
				MCU_Reset();
				break;
			case 0x1B:
				return DIAG_OK;
				break;
			default:
				continue;
		}
	}
	return DIAG_OK;
}

int do_ptctrl (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	int len = 0;
	int rec_len = 0;
	int step = 0;
	char tx_buff[UART_MAX_TEST_LEN];
	char rx_buff[UART_MAX_TEST_LEN];
	char *pos;
	char *speed = "7";

	if (argc < 2){
		printf ("Usage:\n%s\n", cmdtp->usage);
		return -1;
	}else{
		memset(tx_buff, 0, sizeof(tx_buff));
		if(strcmp(argv[1], "reset") == 0){
			MCU_Reset();
		}else
		if(strcmp(argv[1], "init") == 0){
			return MCU_FW_Go();
		}else
		if(strcmp(argv[1], "right") == 0){
			if(argv[2] != NULL) speed = argv[2];
			len = sizeof(gPtCmdSet_Right); memcpy(tx_buff, gPtCmdSet_Right, len);
			len += uart_PtCmd_SetSpeed(speed, tx_buff+len);	//Speed
			uart_hermes_calc_checksum(tx_buff, &len);	//Add Checksum Byte & Tail Bye
			if(uart_pt_send_command(PTCTRL_HERMES_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, &rec_len, UART_TIMEOUT_MSEC)) return DIAG_ERROR;
		}else
		if(strcmp(argv[1], "left") == 0){
			if(argv[2] != NULL) speed = argv[2];
			len = sizeof(gPtCmdSet_Left); memcpy(tx_buff, gPtCmdSet_Left, len);
			len += uart_PtCmd_SetSpeed(speed, tx_buff+len);	//Speed
			uart_hermes_calc_checksum(tx_buff, &len);	//Add Checksum Byte & Tail Bye
			if(uart_pt_send_command(PTCTRL_HERMES_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, &rec_len, UART_TIMEOUT_MSEC)) return DIAG_ERROR;
		}else
		if(strcmp(argv[1], "up") == 0){
			if(argv[2] != NULL) speed = argv[2];
			len = sizeof(gPtCmdSet_Up); memcpy(tx_buff, gPtCmdSet_Up, len);
			len += uart_PtCmd_SetSpeed(speed, tx_buff+len);	//Speed
			uart_hermes_calc_checksum(tx_buff, &len);	//Add Checksum Byte & Tail Bye
			if(uart_pt_send_command(PTCTRL_HERMES_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, &rec_len, UART_TIMEOUT_MSEC)) return DIAG_ERROR;
		}else
		if(strcmp(argv[1], "down") == 0){
			if(argv[2] != NULL) speed = argv[2];
			len = sizeof(gPtCmdSet_Down); memcpy(tx_buff, gPtCmdSet_Down, len);
			len += uart_PtCmd_SetSpeed(speed, tx_buff+len);	//Speed
			uart_hermes_calc_checksum(tx_buff, &len);	//Add Checksum Byte & Tail Bye
			if(uart_pt_send_command(PTCTRL_HERMES_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, &rec_len, UART_TIMEOUT_MSEC)) return DIAG_ERROR;
		}else
		if(strcmp(argv[1], "stop") == 0){
			len = sizeof(gPtCmdSet_Stop); memcpy(tx_buff, gPtCmdSet_Stop, len);
			uart_hermes_calc_checksum(tx_buff, &len);	//Add Checksum Byte & Tail Bye
			if(uart_pt_send_command(PTCTRL_HERMES_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, &rec_len, UART_TIMEOUT_MSEC)) return DIAG_ERROR;
		}else
		if(strcmp(argv[1], "panpos") == 0){
			if(argc < 3){
				printf ("Usage:\n%s\n", cmdtp->usage);
				return -1;
			}
			if(argv[2] != NULL) pos = argv[2];
			if(argv[3] != NULL) speed = argv[3];
			len = sizeof(gPtCmdSet_PanPos); memcpy(tx_buff, gPtCmdSet_PanPos, len);
			len += uart_PtCmd_SetSpeed(speed, tx_buff+len);	//Speed
			len += uart_PtCmd_SetPTPos(pos, tx_buff+len); //Position
			uart_hermes_calc_checksum(tx_buff, &len);	//Add Checksum Byte & Tail Bye
			if(uart_pt_send_command(PTCTRL_HERMES_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, &rec_len, UART_TIMEOUT_MSEC)) return DIAG_ERROR;
		}else
		if(strcmp(argv[1], "tiltpos") == 0){
			if(argc < 3){
				printf ("Usage:\n%s\n", cmdtp->usage);
				return -1;
			}
			if(argv[2] != NULL) pos = argv[2];
			if(argv[3] != NULL) speed = argv[3];
			len = sizeof(gPtCmdSet_TiltPos); memcpy(tx_buff, gPtCmdSet_TiltPos, len);
			len += uart_PtCmd_SetSpeed(speed, tx_buff+len);	//Speed
			len += uart_PtCmd_SetPTPos(pos, tx_buff+len); //Position
			uart_hermes_calc_checksum(tx_buff, &len);	//Add Checksum Byte & Tail Bye
			if(uart_pt_send_command(PTCTRL_HERMES_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, &rec_len, UART_TIMEOUT_MSEC)) return DIAG_ERROR;
		}else
		if(strcmp(argv[1], "getpanpos") == 0){
			len = sizeof(gPtCmdSet_GetPanPos); memcpy(tx_buff, gPtCmdSet_GetPanPos, len); rec_len = 10;
			uart_hermes_calc_checksum(tx_buff, &len);	//Add Checksum Byte & Tail Bye
			if(uart_pt_send_command(PTCTRL_HERMES_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, &rec_len, UART_TIMEOUT_MSEC)) return DIAG_ERROR;
			if(uart_hermes_check_checksum(rx_buff, rec_len)) return DIAG_ERROR;
			else uart_Display_PanPos(rx_buff);
		}else
		if(strcmp(argv[1], "gettiltpos") == 0){
			len = sizeof(gPtCmdSet_GetTiltPos); memcpy(tx_buff, gPtCmdSet_GetTiltPos, len); rec_len = 10;
			uart_hermes_calc_checksum(tx_buff, &len);	//Add Checksum Byte & Tail Bye
			if(uart_pt_send_command(PTCTRL_HERMES_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, &rec_len, UART_TIMEOUT_MSEC)) return DIAG_ERROR;
			if(uart_hermes_check_checksum(rx_buff, rec_len)) return DIAG_ERROR;
			else uart_Display_TiltPos(rx_buff);
		}else
		if(strcmp(argv[1], "ver") == 0){
			len = sizeof(gPtCmdSet_GetVer); memcpy(tx_buff, gPtCmdSet_GetVer, len); rec_len = 10;
			uart_hermes_calc_checksum(tx_buff, &len);	//Add Checksum Byte & Tail Bye
			if(uart_pt_send_command(PTCTRL_HERMES_NAME, tx_buff, rx_buff, sizeof(tx_buff), sizeof(rx_buff), &len, &rec_len, UART_TIMEOUT_MSEC)) return DIAG_ERROR;
			if(uart_hermes_check_checksum(rx_buff, rec_len)) return DIAG_ERROR;
			else uart_Display_Version(rx_buff);
		}else{
			printf ("Usage:\n%s\n", cmdtp->usage);
			return -1;
		}
	}

	return DIAG_OK;
}

U_BOOT_CMD(ptctrl, 3, 0, do_ptctrl,
	"Pan/Tilt control",
	"Pan/Tilt control\n\
	ptctrl reset - MCU Reset\n\
	ptctrl init - MCU Initial\n\
	ptctrl right [speed*] - Pan right\n\
	ptctrl left [speed*] - Pan left\n\
	ptctrl up [speed*] - Tilt up\n\
	ptctrl down [speed*] - Tilt down\n\
	ptctrl stop - Pan/Tilt stop\n\
	ptctrl panpos [pos][speed*] - Set Pan position\n\
	ptctrl tiltpos [pos][speed*] - Set Tilt position\n\
	ptctrl getpanpos - Get Pan position\n\
	ptctrl gettiltpos - Get Tilt position\n\
	ptctrl ver - Get Firmware Version\n\
	'*': Optional.\n\
	"
);

#define HERMES_ACK_LEN 2
#define HERMES_STATUS_LEN 3
static char gPtCmdSet_Ping[] = {0x03, 0x20, 0x20};
static char gPtCmdSet_DownloadMode[] = {0x0B, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static char gPtCmdSet_Status[] = {0x03, 0x23, 0x23};
static char gPtCmdSet_Ack[] = {0x00, 0xCC};

int uart_write(struct serial_device *dev, char *buf, int size)
{
	int i = 0;

	//printf("uart_write(size=%d)->\n", size); print_buffer(0, (void*)buf, 1, size, 0);
	for(i = 0; i < size; i++){
		dev->putc(buf[i]);
	}

	return DIAG_OK;
}


int uart_read_buff(struct serial_device *dev, char *buf, int *size, int timeout)
{
	int i = 0;
	int rec_len = (*size);
	int timeout_msec = timeout;

	if(rec_len > 0){
		memset(buf, 0, rec_len);
		timeout_msec = timeout;
		i = 0;
		while(i < rec_len){
			if(dev->tstc()) {
				timeout_msec = timeout;
				buf[i] = (char)dev->getc();
				i++;
				if(i >= rec_len) break;
			}
			if(timeout_msec-- <= 0) { printf("Rx Time out!!\n"); return DIAG_ERROR; }
			udelay(1000);
		}
		//printf("\nuart_read_buff(size=%d)->\n", i); print_buffer(0, (void*)buf, 1, i, 0);
		(*size) = i;
	}

	return DIAG_OK;
}

int MCU_Ack(struct serial_device *dev, int timeout)
{
	int recv;
	char buf[UART_MAX_TEST_LEN];

	recv = HERMES_ACK_LEN;
	if(uart_read_buff(dev, buf, &recv, timeout) == DIAG_OK){
		if(recv >= HERMES_ACK_LEN)
		{
			if( (buf[0] == 0x00) && (buf[1] == 0xCC) )
			{
				//printf("ACK:0x%02X 0x%02X\n", buf[0], buf[1]);
				return 1;
			}
			printf("NOT ACK:0x%02X 0x%02X\n", buf[0], buf[1]);
		}
	}

	return 0;
}

void Send_Ack(struct serial_device *dev)
{
	char buf[UART_MAX_TEST_LEN];
	memset(buf, 0, sizeof(buf));
	buf[0] = 0x00;
	buf[1] = 0xCC;
	uart_write(dev, buf, 2);
}

char Get_Status(struct serial_device *dev, int timeout)
{
	int len = 0;
	int recv;
	char status = 0;
	char buf[UART_MAX_TEST_LEN];

	memset(buf, 0, sizeof(buf));
	len = sizeof(gPtCmdSet_Status); memcpy(buf, gPtCmdSet_Status, len); recv = HERMES_ACK_LEN + HERMES_STATUS_LEN;
	uart_write(dev, buf, len);

	memset(buf, 0, sizeof(buf));
	if(uart_read_buff(dev, buf, &recv, timeout) == DIAG_OK){
		if(uart_hermes_check_ack(buf, recv)) return DIAG_ERROR;
		
		status = buf[4];
		//printf("Status = 0x%02X\n", status);
		Send_Ack(dev);
	}

	return status;
}

static int Ping_MCU(struct serial_device *dev)
{
	unsigned char buf[UART_MAX_TEST_LEN];
	
	memset(buf, 0, sizeof(buf));
	buf[0] = 0x03;
	buf[1] = 0x20;
	buf[2] = 0x20;
	uart_write(dev, buf, 3);

	return MCU_Ack(dev, MCU_LIB_STD_TIMEOUT);
}

static char Gen_CheckSum(char *data_ptr, int length)
{
	unsigned int tmp;
	
	tmp = 0;
	while(length)
	{
		tmp += data_ptr[length-1];
		length--;
	}
	tmp &= 0xFF;

	return (char)(tmp);
}

static int Download_Mode(struct serial_device *dev, int fw_size)
{
	char buf[UART_MAX_TEST_LEN];
	
	memset(buf, 0, sizeof(buf));
	buf[ 0] = 0x0B;
	buf[ 2] = 0x21;
	buf[ 3] = (DOWNLOAD_ADDRESS >> 24)&0xFF;
	buf[ 4] = (DOWNLOAD_ADDRESS >> 16)&0xFF;
	buf[ 5] = (DOWNLOAD_ADDRESS >>  8)&0xFF;
	buf[ 6] = (DOWNLOAD_ADDRESS >>  0)&0xFF;
	buf[ 7] = (fw_size >> 24)&0xFF;
	buf[ 8] = (fw_size >> 16)&0xFF;
	buf[ 9] = (fw_size >> 8)&0xFF;
	buf[10] = (fw_size >> 0)&0xFF;
	buf[1] = Gen_CheckSum(buf+2, 9);
	
	uart_write(dev, buf, 11);
	if( MCU_Ack(dev, MCU_LIB_STD_TIMEOUT) != 1)
	{
		return -1;
	}
	return 1;
}

static int Transmit_Data(struct serial_device *dev, char *fw_ptr, int size)
{
	int transmit;
	char buf[UART_MAX_TEST_LEN];
	char status;
	
	transmit = 0;
	while(size != 0)
	{
		memset(buf, 0, sizeof(buf));
		buf[2] = 0x24;
		if(size >= TRANSMIT_STEP)
		{
			buf[0] = TRANSMIT_STEP + 3;
			memcpy(buf+3, fw_ptr+transmit, TRANSMIT_STEP);
			buf[1] = Gen_CheckSum(buf+2, TRANSMIT_STEP + 1);
			uart_write(dev, buf, buf[0]);
			transmit += TRANSMIT_STEP;
			size -= TRANSMIT_STEP;
		}
		else
		{
			buf[0] = size + 3;
			memcpy(buf+3, fw_ptr+transmit, size);
			buf[1] = Gen_CheckSum(buf+2, size + 1);
			uart_write(dev, buf, buf[0]);
			transmit += size;
			size -= size;
		}
		if(MCU_Ack(dev, MCU_LIB_STD_TIMEOUT) != 1)
		{
			return -1;
		}
		else
		{
			status = Get_Status(dev, MCU_LIB_STD_TIMEOUT);
			if(status != MCU_STATUS_OK)
			{
				printf("\nTransfer fail. Status 0x%02X", status);
				return -1;
			}
			printf("\r%d bytes transfered.", transmit);
			continue;
		}
	}
	printf("\ntransfer %d byte(s)\n", transmit);
	return 1;
}

int MCU_FW_Upload(char *fw_ptr, int size)
{
	struct serial_device *dev;

	dev = get_serial_device_by_name(PTCTRL_HERMES_NAME);
	if(dev == NULL) return DIAG_ERROR;
	dev->init();

	if( Ping_MCU(dev) != 1)
	{
		printf("Ping to MCU fail\n");
		return DIAG_ERROR;
	}
	if( Download_Mode(dev, size) != 1)
	{
		printf("MCU set download mode not ack\n");
		return DIAG_ERROR;
	}
	if( Get_Status(dev, UART_TIMEOUT_MSEC) != MCU_STATUS_OK)
	{
		printf("MCU set download mode fail\n");
		return DIAG_ERROR;
	}
	if(Transmit_Data(dev, fw_ptr, size) != 1)
	{
		printf("MCU fw upload fail\n");
		return DIAG_ERROR;
	}
	
	return DIAG_OK;
}

int do_loadmcu (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	long size = 0;
	char* FileBuffer = (char*)CONFIG_SYS_LOAD_ADDR;

	if( (argc < 2) || ( (argc < 3) && (strcmp(argv[1], "tftp") == 0))){
		printf ("Usage:\n%s\n", cmdtp->usage);
		return DIAG_ERROR;
	}

	if((size = load_image(cmdtp, flag, argv[1], argc < 3 ? NULL : argv[2], CONFIG_SYS_LOAD_ADDR)) <= 0){
		return DIAG_ERROR;
	}
	printf ("size:%lu\n", size);

	MCU_Reset();
	Sleep(1);

	if(MCU_FW_Upload(FileBuffer, (int)size) == DIAG_OK)
	{
		printf("Success!\n");
		MCU_FW_Go();
	}
	else
	{
		printf("ERROR!\n");
	}

	return DIAG_OK;
}

U_BOOT_CMD(
	loadmcu, 3, 0,	do_loadmcu,
	"Load P/T MCU firmware",
	"Load P/T MCU firmware\n\
	loadmcu [kermit/xymodem/tftp/mmc(dev[:part])][filename]"
);

static int diag_download_MCU(int parameter)
{
	int rcode = 0;
	int ret = 0;
	char cmd[3];
	cmd_tbl_t cmd_tmp;
	memset(cmd, 0, sizeof(cmd));
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));

	if(get_line("Download 'MCU' image from serial[0] / tftp[1]* / mmc[2]:", cmd, sizeof(cmd), -1, "012", NULL, "1") < 0) return -1;

	if(cmd[0] == '0'){
		char *args[2];
		args[0] = "loadmcu";
		args[1] = "xymodem";
		if(do_loadmcu(&cmd_tmp, 0, 2, args) == 0){
			printf("Download image successed.\n");
		}else{
			printf("Download image failed.\n");
			rcode = -1;
		}
	}else
	if((cmd[0] == '1') || (cmd[0] == '2')){
		char* d4file = D4_HERMES_FILE_NAME;
		char tmp[CFG_CBSIZE];
		memset(tmp, 0, sizeof(tmp));
		char *args[3];
		args[0] = "loadmcu";
		if(cmd[0] == '1') args[1] = "tftp";
		else if(cmd[0] == '2'){
			char mmc[5];
			if(get_line("Enter mmc partition(>1):", tmp, 5, -1, str_number_dec, NULL, "1") < 0) return -1;
			sprintf(mmc, "mmc(0:%s)", tmp);
			args[1] = mmc;
		}else args[1] = "tftp";
		printf("[%s] Download file name(%s):", args[1], d4file);
		if((ret = get_line(NULL, tmp, sizeof(tmp), -1, NULL, NULL, d4file)) >= 0 ){
			args[2] = tmp;
			if(do_loadmcu(&cmd_tmp, 0, 3, args) == 0){
				printf("Download file %s successed.\n", tmp);
			}else{
				printf("Download file %s failed.\n", tmp);
				rcode = -1;
			}
		}
	}
	return rcode;
}

#endif

//GPIO Test ---------------------------------------------------------------------------
enum{
	GPIO_READ_RESET_BUTTON,
	GPIO_READ_LIGHT_SENSOR,
	GPIO_READ_DI_0,
	GPIO_READ_DO_0,
	GPIO_READ_SD_En,
	GPIO_READ_SD_CDn,
	GPIO_READ_SD_WPn,
	GPIO_READ_Heatercam_Int,
	GPIO_READ_Heatersys_Int,
	GPIO_READ_FAN_Int,
	GPIO_READ_PT,

	GPIO_WRITE_HIGH_ICR,
	GPIO_WRITE_LOW_ICR,
	GPIO_WRITE_HIGH_DO_0,
	GPIO_WRITE_LOW_DO_0,
	GPIO_WRITE_HIGH_IR_LED,
	GPIO_WRITE_LOW_IR_LED,
	GPIO_WRITE_HIGH_SD_En,
	GPIO_WRITE_LOW_SD_En,
	GPIO_WRITE_HIGH_Heater_cam,
	GPIO_WRITE_LOW_Heater_cam,
	GPIO_WRITE_HIGH_Heater_sys,
	GPIO_WRITE_LOW_Heater_sys,
	GPIO_WRITE_HIGH_CAMERA_POWER,
	GPIO_WRITE_LOW_CAMERA_POWER,
	GPIO_WRITE_HIGH_FAN_CON,
	GPIO_WRITE_LOW_FAN_CON,
	GPIO_WRITE_HIGH_PT,
	GPIO_WRITE_LOW_PT,
};

static int gpio_read_func(int parameter);
static int gpio_write_func(int parameter);
static int gpio_test_func(int parameter);
static int pwm_test_func(int parameter);

static DiagMenuTableStruct PTMenuTable[] = {
	{ '0',	"PT Reset write high", 		GPIO_WRITE_HIGH_PT,				gpio_write_func,	NULL},
	{ '1',	"PT Reset write low",		GPIO_WRITE_LOW_PT,				gpio_write_func,	NULL},
	{ '2',	"PT Reset state", 			GPIO_READ_PT,					gpio_read_func,		NULL},
};
static DiagMenuStruct PTMenu = {
	sizeof(PTMenuTable)/sizeof(DiagMenuTableStruct),
	PTMenuTable,
	"<<<PT Menu>>>",
	0, NULL
};

static DiagMenuTableStruct FanMenuTable[] = {
	{ '0',	"Fan Power write high",		GPIO_WRITE_HIGH_FAN_CON,		gpio_write_func,	NULL},
	{ '1',	"Fan Power write low",		GPIO_WRITE_LOW_FAN_CON,			gpio_write_func,	NULL},
	{ '2',	"Fan_Int state",			GPIO_READ_FAN_Int,				gpio_read_func, 	NULL},
};
static DiagMenuStruct FanMenu = {
	sizeof(FanMenuTable)/sizeof(DiagMenuTableStruct),
	FanMenuTable,
	"<<<Fan Menu>>>",
	0, NULL
};

static DiagMenuTableStruct CameraMenuTable[] = {
	{ '0',	"Camera Power write high",	GPIO_WRITE_HIGH_CAMERA_POWER,	gpio_write_func,	NULL},
	{ '1',	"Camera Power write low",	GPIO_WRITE_LOW_CAMERA_POWER,	gpio_write_func,	NULL},
};
static DiagMenuStruct CameraMenu = {
	sizeof(CameraMenuTable)/sizeof(DiagMenuTableStruct),
	CameraMenuTable,
	"<<<Camera Menu>>>",
	0, NULL
};

static DiagMenuTableStruct HeaterMenuTable[] = {
	{ '0',	"Camera Heater write high",	GPIO_WRITE_HIGH_Heater_cam,		gpio_write_func,	NULL},
	{ '1',	"Camera Heater write low",	GPIO_WRITE_LOW_Heater_cam,		gpio_write_func,	NULL},
	{ '2',	"System Heater write high",	GPIO_WRITE_HIGH_Heater_sys, 	gpio_write_func,	NULL},
	{ '3',	"System Heater write low",	GPIO_WRITE_LOW_Heater_sys,		gpio_write_func,	NULL},
	{ '4',	"Heater_cam_Int state", 	GPIO_READ_Heatercam_Int,		gpio_read_func, 	NULL},
	{ '5',	"Heater_sys_Int state",		GPIO_READ_Heatersys_Int,		gpio_read_func, 	NULL},
};
static DiagMenuStruct HeaterMenu = {
	sizeof(HeaterMenuTable)/sizeof(DiagMenuTableStruct),
	HeaterMenuTable,
	"<<<Heater Menu>>>",
	0, NULL
};

static DiagMenuTableStruct SDMenuTable[] = {
	{ '0',	"SD_EN write high",			GPIO_WRITE_HIGH_SD_En,			gpio_write_func, 	NULL},
	{ '1',	"SD_EN write low",			GPIO_WRITE_LOW_SD_En,			gpio_write_func, 	NULL},
	{ '2',	"SD_EN state",				GPIO_READ_SD_En,				gpio_read_func, 	NULL},
	{ '3',	"SD_CDn state", 			GPIO_READ_SD_CDn,				gpio_read_func, 	NULL},
	{ '4',	"SD_WPn state", 			GPIO_READ_SD_WPn,				gpio_read_func, 	NULL},
};
static DiagMenuStruct SDMenu = {
	sizeof(SDMenuTable)/sizeof(DiagMenuTableStruct),
	SDMenuTable,
	"<<<SD Menu>>>",
	0, NULL
};

static DiagMenuTableStruct IRLEDMenuTable[] = {
	{ '0',	"IR LED write high",		GPIO_WRITE_HIGH_IR_LED,			gpio_write_func,	NULL},
	{ '1',	"IR LED write low",			GPIO_WRITE_LOW_IR_LED, 			gpio_write_func,	NULL},
};
static DiagMenuStruct IRLEDMenu = {
	sizeof(IRLEDMenuTable)/sizeof(DiagMenuTableStruct),
	IRLEDMenuTable,
	"<<<IR LED Menu>>>",
	0, NULL
};

static DiagMenuTableStruct PWMMenuTable[] = {
	{ '0',	"PWM 0 test",				0,								pwm_test_func,		NULL},
	{ '1',	"PWM 1 test",				1,								pwm_test_func,		NULL},
	{ '2',	"PWM 2 test",				2,								pwm_test_func,		NULL},
	{ '3',	"PWM 3 test",				3,								pwm_test_func,		NULL},
};
static DiagMenuStruct PWMMenu = {
	sizeof(PWMMenuTable)/sizeof(DiagMenuTableStruct),
	PWMMenuTable,
	"<<<PWM Menu>>>",
	0, NULL
};

static DiagMenuTableStruct ICRMenuTable[] = {
	{ '0',	"ICR write high", 			GPIO_WRITE_HIGH_ICR,			gpio_write_func,	NULL},
	{ '1',	"ICR write low",			GPIO_WRITE_LOW_ICR,				gpio_write_func,	NULL},
};
static DiagMenuStruct ICRMenu = {
	sizeof(ICRMenuTable)/sizeof(DiagMenuTableStruct),
	ICRMenuTable,
	"<<<ICR Menu>>>",
	0, NULL
};

static DiagMenuTableStruct GPIOTestMenuTable[] = {
	{ '0',	"GPIO test",		0,							gpio_test_func,		NULL},
	{ '1',	"Reset Button",		GPIO_READ_RESET_BUTTON,		gpio_read_func,		NULL},
	{ '2',	"Light Sensor",		GPIO_READ_LIGHT_SENSOR,		gpio_read_func,		NULL},
	{ '3',	"ICR test",			0,							NULL,				&ICRMenu},
	{ '4',	"PWM test",			0,							NULL,				&PWMMenu},
	{ '5',	"IR LED test",		0,							NULL,				&IRLEDMenu},
	{ '6',	"SD Card test",		0,							NULL,				&SDMenu},
	{ '7',	"Heater test",		0,							NULL,				&HeaterMenu},
	{ '8',	"Camera test",		0,							NULL,				&CameraMenu},
	{ '9',	"Fan test",			0,							NULL,				&FanMenu},
	{ 'a',	"PT test", 			0,							NULL,				&PTMenu},
};
static DiagMenuStruct GPIOTestMenu = {
	sizeof(GPIOTestMenuTable)/sizeof(DiagMenuTableStruct),
	GPIOTestMenuTable,
	"<<<GPIO Test Menu>>>",
	0, NULL
};

static int gpio_read_func(int parameter)
{
	int c = 0;
	printf("Press 'ESC' to exit.\n");
	while(c != 0x1B){
		if(tstc()) c = getc();
		switch(parameter){
			case GPIO_READ_RESET_BUTTON:
#ifdef GPIO_SYSBUTTON
				printf(" Reset Button ->%s\r", GPIO_In(GPIO_SYSBUTTON) ? "High" : "Low ");
#endif
				break;
			case GPIO_READ_LIGHT_SENSOR:
#ifdef GPIO_LIGHT_SENSOR
				printf(" Light Sensor ->%s\r", GPIO_In(GPIO_LIGHT_SENSOR) ? "High" : "Low ");
#endif
				break;
			case GPIO_READ_DI_0:
#ifdef GPIO_DI
				printf(" DI 0 ->%s\r", GPIO_In(GPIO_DI) ? "High" : "Low ");
#endif
				break;
			case GPIO_READ_DO_0:
#ifdef GPIO_DO
				printf(" DO 0 ->%s\r", GPIO_In(GPIO_DO) ? "High" : "Low ");
#endif
				break;
			case GPIO_READ_SD_En:
#ifdef GPIO_SD_EN
				printf(" SD_EN ->%s\r", GPIO_In(GPIO_SD_EN) ? "High" : "Low ");
#endif
				break;
			case GPIO_READ_SD_WPn:
#ifdef GPIO_SD_WP
				printf(" SD_WPn ->%s\r", GPIO_In(GPIO_SD_WP) ? "High" : "Low ");
#endif
				break;
			case GPIO_READ_SD_CDn:
#ifdef GPIO_SD_CD
				printf(" SD_CDn ->%s\r", GPIO_In(GPIO_SD_CD) ? "High" : "Low ");
#endif
				break;
			case GPIO_READ_Heatercam_Int:
#ifdef GPIO_HEATERCAM_INT
				printf(" Heatercam_Int ->%s\r", GPIO_In(GPIO_HEATERCAM_INT) ? "High" : "Low ");
#endif


				break;
			case GPIO_READ_Heatersys_Int:
#ifdef GPIO_HEATERSYS_INT
				printf(" Heatersys_Int ->%s\r", GPIO_In(GPIO_HEATERSYS_INT) ? "High" : "Low ");
#elif GPIO_HEATERSYS_STATUS
				printf(" Heatersys status ->%s\r", GPIO_In(GPIO_HEATERSYS_STATUS) ? "On " : "Off");
#endif
				break;
			case GPIO_READ_FAN_Int:
#ifdef GPIO_FAN_INT
				printf(" FAN_Int ->%s\r", GPIO_In(GPIO_FAN_INT) ? "High" : "Low ");
#endif
				break;
			case GPIO_READ_PT:
#ifdef GPIO_MCU_RST
				printf(" MCU_RST ->%s\r", GPIO_In(GPIO_MCU_RST) ? "High" : "Low ");
#endif
				break;
			default:
				break;
		}
		udelay(500000);
	}
	return DIAG_OK;
}

static int gpio_write_func(int parameter)
{
	switch(parameter){
		case GPIO_WRITE_HIGH_ICR:
#ifdef GPIO_ICR
			GPIO_Out(GPIO_ICR, GPIO_HIGH);
#endif
			break;
		case GPIO_WRITE_LOW_ICR:
#ifdef GPIO_ICR
			GPIO_Out(GPIO_ICR, GPIO_LOW);
#endif
			break;
		case GPIO_WRITE_HIGH_DO_0:
#ifdef GPIO_DO
			GPIO_Out(GPIO_DO, GPIO_HIGH);
#endif
			break;
		case GPIO_WRITE_LOW_DO_0:
#ifdef GPIO_DO
			GPIO_Out(GPIO_DO, GPIO_LOW);
#endif
			break;
		case GPIO_WRITE_HIGH_IR_LED:
#ifdef GPIO_IR_LED
			GPIO_Out(GPIO_IR_LED, GPIO_HIGH);
#endif
			break;
		case GPIO_WRITE_LOW_IR_LED:
#ifdef GPIO_IR_LED
			GPIO_Out(GPIO_IR_LED, GPIO_LOW);
#endif
			break;
		case GPIO_WRITE_HIGH_SD_En:
#ifdef GPIO_SD_EN
			GPIO_Out(GPIO_SD_EN, GPIO_HIGH);
#endif
			break;
		case GPIO_WRITE_LOW_SD_En:
#ifdef GPIO_SD_EN
			GPIO_Out(GPIO_SD_EN, GPIO_LOW);
#endif
			break;
		case GPIO_WRITE_HIGH_Heater_cam:
#ifdef GPIO_HEATER_CAM
			GPIO_Out(GPIO_HEATER_CAM, GPIO_HIGH);
#endif
			break;
		case GPIO_WRITE_LOW_Heater_cam:
#ifdef GPIO_HEATER_CAM
			GPIO_Out(GPIO_HEATER_CAM, GPIO_LOW);
#endif
			break;
		case GPIO_WRITE_HIGH_Heater_sys:
#ifdef GPIO_HEATER_SYS
			GPIO_Out(GPIO_HEATER_SYS, GPIO_HIGH);
#endif
			break;
		case GPIO_WRITE_LOW_Heater_sys:
#ifdef GPIO_HEATER_SYS
			GPIO_Out(GPIO_HEATER_SYS, GPIO_LOW);
#endif
			break;
		case GPIO_WRITE_HIGH_CAMERA_POWER:
#ifdef GPIO_CAM_RST
			GPIO_Out(GPIO_CAM_RST, GPIO_HIGH);
#endif
			break;
		case GPIO_WRITE_LOW_CAMERA_POWER:
#ifdef GPIO_CAM_RST
			GPIO_Out(GPIO_CAM_RST, GPIO_LOW);
#endif
			break;
		case GPIO_WRITE_HIGH_FAN_CON:
#ifdef GPIO_FAN_CON
			GPIO_Out(GPIO_FAN_CON, GPIO_HIGH);
#endif
			break;
		case GPIO_WRITE_LOW_FAN_CON:
#ifdef GPIO_FAN_CON
			GPIO_Out(GPIO_FAN_CON, GPIO_LOW);
#endif
			break;
		case GPIO_WRITE_HIGH_PT:
#ifdef GPIO_MCU_RST
			GPIO_Out(GPIO_MCU_RST, GPIO_HIGH);
#endif
			break;
		case GPIO_WRITE_LOW_PT:
#ifdef GPIO_MCU_RST
			GPIO_Out(GPIO_MCU_RST, GPIO_LOW);
#endif
			break;
		default:
			break;
	}
	return DIAG_OK;
}

static int gpio_input_stat(int bank, int pin)
{
	int c = 0;
	printf("Press 'ESC' to exit.\n");
	while(c != 0x1B){
		if(tstc()) c = getc();
		printf(" GP%d[%d] ->%s\r", bank, pin, GPIO_Input(bank, pin) ? "High" : "Low ");
		udelay(500000);
	}
	return DIAG_OK;
}

static int gpio_test_bank = 0;
static int gpio_test_pin = 0;
static int gpio_test_data = 0;
static int gpio_test_func(int parameter)
{
	int ret = 0;
	char tmp[10];

	printf("Bank[GP0~GP3](GP%d):GP", gpio_test_bank);
	ret = get_line(NULL, tmp, 3, -1, "0123", NULL, NULL);
	if(ret > 0) gpio_test_bank = (int)(simple_strtoul(tmp, NULL, 10));
	else if(ret < 0) return DIAG_ERROR;

	printf("Pin[0~31](%d):", gpio_test_pin);
	ret = get_line(NULL, tmp, 4, -1, str_number_dec, NULL, NULL);
	if(ret > 0) gpio_test_pin = (int)(simple_strtoul(tmp, NULL, 10));
	else if(ret < 0) return DIAG_ERROR;

	printf("Data[0:Low, 1:High, 2:Input](%d):", gpio_test_data);
	ret = get_line(NULL, tmp, 3, -1, "012", NULL, NULL);
	if(ret > 0) gpio_test_data = (int)(simple_strtoul(tmp, NULL, 10));
	else if(ret < 0) return DIAG_ERROR;

	if(gpio_test_data == 2){
		GPIO_OutEn(gpio_test_bank, gpio_test_pin, 0);
		gpio_input_stat(gpio_test_bank, gpio_test_pin);
	}else{
		GPIO_OutEn(gpio_test_bank, gpio_test_pin, 1);
		GPIO_Output(gpio_test_bank, gpio_test_pin, gpio_test_data);
	}
	return DIAG_OK;
}

int pwm_test_setting(u32* per, u32* ph1d)
{
	char tmp[10];
	if(get_line("Period(HEX):0x", tmp, sizeof(tmp), -1, str_number_hex, NULL, NULL) <= 0) return DIAG_ERROR;
	*per = (u32)(simple_strtoul(tmp, NULL, 16));
	if(get_line("First-Phase Duration(HEX):0x", tmp, sizeof(tmp), -1, str_number_hex, NULL, NULL) <= 0) return DIAG_ERROR;
	*ph1d = (u32)(simple_strtoul(tmp, NULL, 16));
	return DIAG_OK;
}

static int pwm_test_func(int parameter)
{
	switch(parameter){
		case 0:
			break;
		case 1:
			break;
		case 2:
			break;
		case 3:
			break;
		default:
			break;
	}
	return DIAG_OK;
}

#ifdef GPIO_FAN_CON
int do_fan (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	ulong start = 0;
	ulong delay = 0;
	int sec = 3;
	int speed = 0;
	int prev = 0;
	int curr = 0;
	int comparer = 0;
	if(argc < 2) {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return -1;
	}else{
		if(strcmp(argv[1], "on") == 0){
			GPIO_Out(GPIO_FAN_CON, GPIO_FAN_ON);
		}else if(strcmp(argv[1], "off") == 0){
			GPIO_Out(GPIO_FAN_CON, GPIO_FAN_OFF);
		}else if(strcmp(argv[1], "speed") == 0){
			reset_timer();
			start = get_timer(0);
			if(argc >= 3)
				sec = (int)(simple_strtoul(argv[2], NULL, 10));
			delay = sec * CFG_HZ;
			while(get_timer(start) < delay) {
				curr = GPIO_In(GPIO_FAN_INT);
				if(curr != prev){
					prev = curr;
					speed++;
				}
			}
			if(argc >= 4){
				comparer = (int)(simple_strtoul(argv[3], NULL, 10));
				if(speed > comparer) printf("gt\n");
				else if(speed < comparer) printf("lt\n");
				else printf("eq\n");
			}else{
				printf("FAN_INT(count)=%d/sec\n", speed/sec);
			}
		}
	}
	return 0;
}

U_BOOT_CMD(
	fan, 4, 0,	do_fan,
	"FAN control",
	"FAN On/Off/Get_speed\n\
	fan [on/off]\n\
	fan [speed][sample_sec][comparer]"
);
#endif

//DIDO Test ---------------------------------------------------------------------------
#if defined(VPORT06EC_2V)
static int dido_loopback_test(int parameter)
{
	printf("Write DO High\n");
	GPIO_Out(GPIO_DO, GPIO_HIGH);
	Sleep(1);
	if (GPIO_In(GPIO_DI) != 1)
	{
		printf("Failed : Read DI Low\n");
		return DIAG_ERROR;
	}
	else
	{
		printf("Read DI High\n");
	}
	Sleep(1);
	printf("Write DO Low\n");
	GPIO_Out(GPIO_DO, GPIO_LOW);
	Sleep(1);
	if (GPIO_In(GPIO_DI) != 0)
	{
		printf("Failed : Read DI High\n");
		return DIAG_ERROR;
	}
	else
	{
		printf("Read DI Low\n");
	}

	printf("PASS.\n");

	return DIAG_OK;
}
#endif

static DiagMenuTableStruct DIDOTestMenuTable[] = {
	{ '0', "DI 0 read status", 		GPIO_READ_DI_0,			gpio_read_func,		NULL},
	{ '1', "DO 0 read status", 		GPIO_READ_DO_0,			gpio_read_func,		NULL},
	{ '2', "DO 0 write high", 		GPIO_WRITE_HIGH_DO_0,	gpio_write_func,	NULL},
	{ '3', "DO 0 write low", 		GPIO_WRITE_LOW_DO_0,	gpio_write_func,	NULL},
#if defined(VPORT06EC_2V)
	{ '4', "DI/DO loopback test", 	0,						dido_loopback_test,	NULL},
#endif
};
static DiagMenuStruct DIDOTestMenu = {
	sizeof(DIDOTestMenuTable)/sizeof(DiagMenuTableStruct),
	DIDOTestMenuTable,
	"<<<DIDO Test Menu>>>",
	0, NULL
};

//RTC Test ---------------------------------------------------------------------------
#define _RTC_ISL1208	0
//wensen for VPort06EC-2
#define _RTC_MCP7941X	1
static int rtc_query_test(int parameter);

static DiagMenuTableStruct RTCTestMenuTable[] = {
#if defined(CONFIG_RTC_MCP7941X)
	{ '0',	"Query RTC",		_RTC_MCP7941X,	rtc_query_test, 		NULL},
#else
	{ '0',	"Query RTC",		_RTC_ISL1208,	rtc_query_test, 		NULL},
#endif
};
static DiagMenuStruct RTCTestMenu = {
	sizeof(RTCTestMenuTable)/sizeof(DiagMenuTableStruct),
	RTCTestMenuTable,
	"<<<RTC Test Menu>>>",
	0, NULL
};

int rtc_get_time(int device, struct rtc_time *tm)
{
	switch(device){
		case _RTC_ISL1208: return rtc_get(tm);
		case _RTC_MCP7941X: return rtc_get(tm);
		default: return -1;
	}
	return 0;
}

int rtc_set_time(int device, struct rtc_time *tm)
{
	switch(device){
		case _RTC_ISL1208: return rtc_set(tm);
		case _RTC_MCP7941X: return rtc_set(tm);
		default: return -1;
	}
	return 0;
}

void rtc_reset_device(int device)
{
	switch(device){
		case _RTC_ISL1208: rtc_reset();
		case _RTC_MCP7941X: rtc_reset();
	}
}

void rtc_set_time_func(int device)
{
	struct rtc_time tm;
	int ret = 0;
	char tmp[7];
	int value = 0;

	if(rtc_get_time(device, &tm)) {
		printf("## Get date failed\n");
		return;
	}

	memset(tmp, 0, sizeof(tmp));
	while(tmp[0] != 0x1B){
		ret = get_line("Set RTC Sec.(0~59):", tmp, 4, -1, str_number_dec, NULL, NULL);
		if(ret > 0){
			value = (int)simple_strtoul(tmp, NULL, 10);
			if(value < 0 || value > 59){
				printf("Bad value, must between 0~59\n");
			}else{
				tm.tm_sec = value;
				break;
			}
		}else if(ret == _GETLINE_CTRLC) return;
		else if(ret == 0) break;
	}

	memset(tmp, 0, sizeof(tmp));
	while(tmp[0] != 0x1B){
		ret = get_line("Set RTC Min.(0~59):", tmp, 4, -1, str_number_dec, NULL, NULL);
		if(ret > 0){
			value = (int)simple_strtoul(tmp, NULL, 10);
			if(value < 0 || value > 59){
				printf("Bad value, must between 0~59\n");
			}else{
				tm.tm_min = value;
				break;
			}
		}else if(ret == _GETLINE_CTRLC) return;
		else if(ret == 0) break;
	}

	memset(tmp, 0, sizeof(tmp));
	while(tmp[0] != 0x1B){
		ret = get_line("Set RTC Hour.(0~23):", tmp, 4, -1, str_number_dec, NULL, NULL);
		if(ret > 0){
			value = (int)simple_strtoul(tmp, NULL, 10);
			if(value < 0 || value > 23){
				printf("Bad value, must between 0~23\n");
			}else{
				tm.tm_hour = value;
				break;
			}
		}else if(ret == _GETLINE_CTRLC) return;
		else if(ret == 0) break;
	}

	memset(tmp, 0, sizeof(tmp));
	while(tmp[0] != 0x1B){
		ret = get_line("Set RTC Day.(1~31):", tmp, 4, -1, str_number_dec, NULL, NULL);
		if(ret > 0){
			value = (int)simple_strtoul(tmp, NULL, 10);
			if(value < 1 || value > 31){
				printf("Bad value, must between 1~31\n");
			}else{
				tm.tm_mday = value;
				break;
			}
		}else if(ret == _GETLINE_CTRLC) return;
		else if(ret == 0) break;
	}

	memset(tmp, 0, sizeof(tmp));
	while(tmp[0] != 0x1B){
		ret = get_line("Set RTC Mon.(1~12):", tmp, 4, -1, str_number_dec, NULL, NULL);
		if(ret > 0){
			value = (int)simple_strtoul(tmp, NULL, 10);
			if(value < 1 || value > 12){
				printf("Bad value, must between 1~12\n");
			}else{
				tm.tm_mon = value;
				break;
			}
		}else if(ret == _GETLINE_CTRLC) return;
		else if(ret == 0) break;
	}

	memset(tmp, 0, sizeof(tmp));
	while(tmp[0] != 0x1B){
		ret = get_line("Set RTC Tear.(2000~9999):", tmp, 6, -1, str_number_dec, NULL, NULL);
		if(ret > 0){
			value = (int)simple_strtoul(tmp, NULL, 10);
			if(value < 2000 || value > 9999){
				printf("Bad value, must between 2000~9999\n");
			}else{
				tm.tm_year = value;
				break;
			}
		}else if(ret == _GETLINE_CTRLC) return;
		else if(ret == 0) break;
	}

	if(rtc_set_time(device, &tm)) {
		printf("## Set date failed\n");
		return;
	}
}

static int rtc_query_test(int parameter)
{
	struct rtc_time tm;
	int secBak = -1;
	int c = 0;
	printf("\nPress 'ESC' to exit. / 's' to set time. / 'r' to reset RTC.\n\n");
	while(c != 0x1B){
		if(tstc()) c = getc();
		if(c == 's'){
			printf("\n");
			rtc_set_time_func(parameter);
			printf("\n");
			printf("\nPress 'ESC' to exit. / 's' to set time. / 'r' to reset RTC.\n\n");
			c = 0;
		}
		if(c == 'r'){
			rtc_reset_device(parameter);
		}
		if(rtc_get_time(parameter, &tm)) {
			printf("## Get date failed\n");
			return DIAG_ERROR;
		}
		if(secBak != tm.tm_sec){
			printf("Date: %4d-%02d-%02d  Time: %2d:%02d:%02d        \r",
				tm.tm_year, tm.tm_mon, tm.tm_mday,
				tm.tm_hour, tm.tm_min, tm.tm_sec);
			secBak = tm.tm_sec;
		}
		udelay(100000);
	}
	printf("\n");
	return DIAG_OK;
}

#if defined(CONFIG_RTC_MCP7941X)


static int rtc_get_vbat_bit(void)
{
	uchar chip;
	uint addr;
	int alen;
	int len;
	int i;
	char vbat_reg = 0;

	chip = CONFIG_SYS_I2C_RTC_ADDR;
	addr = RTC_REG_VBT;
	len = 1;
	alen = 1;

	if (i2c_read(chip, addr, alen, &vbat_reg, len))
	{
		printf("Failed : %s : read I2C bus.\n", __func__);
		return DIAG_ERROR;
	}

	if (vbat_reg & 0x10)
	{
		printf("Cool start\n");
		return 1;
	}
	else
	{
		printf("Warm start\n");
	}

	return DIAG_OK;
}

#endif //defined(CONFIG_RTC_MCP7941X)


//I2C Test ---------------------------------------------------------------------------
static char i2c_chip_addr[4];
static char i2c_reg_addr[6];
static char i2c_count[6];
static char i2c_value[4];
static char i2c_loop[3];
static int i2c_probe_test(int parameter);
static int i2c_read_test(int parameter);
static int i2c_write_test(int parameter);
#if defined(VPORT06EC_2V)
static int i2c_addr_list_func(int parameter);
#endif
static DiagMenuTableStruct I2CTestMenuTable[] = {
	{ '0',	"I2C Probe",					0,	i2c_probe_test,		NULL},
	{ '1',	"I2C Read",						0,	i2c_read_test,		NULL},
	{ '2',	"I2C Write",					0,	i2c_write_test,		NULL},
#if defined(VPORT06EC_2V)
	{ '3',	"I2C Address List",				0,	i2c_addr_list_func,	NULL},
#endif
};
static DiagMenuStruct I2CTestMenu = {
	sizeof(I2CTestMenuTable)/sizeof(DiagMenuTableStruct),
	I2CTestMenuTable,
	"<<<I2C Test Menu>>>",
	0, NULL
};

static int i2c_setbus()
{
#if defined(CONFIG_I2C_MULTI_BUS)
	char tmp[3];
	int ret = 0;
	int bus_idx = I2C_GET_BUS();
	printf("I2C BUS.(%d):", bus_idx);
	ret = get_line(NULL, tmp, sizeof(tmp), -1, str_number_dec, NULL, NULL);
	if(ret < 0) return -1;
	else if(ret > 0){
		bus_idx = simple_strtoul(tmp, NULL, 10);
		printf("Setting bus to %d\n", bus_idx);
		ret = I2C_SET_BUS(bus_idx);
		if (ret)
			printf("Failure changing bus number (%d)\n", ret);
	}
#endif
	return 0;
}

static int i2c_probe_test(int parameter)
{
	int j;
	if(i2c_setbus() != 0) return DIAG_ERROR;
	puts ("Valid chip addresses:");
	for (j = 0; j < 128; j++) {
		if (i2c_probe(j) == 0)
			printf(" %02X", j);
	}
	putc ('\n');
	return DIAG_OK;
}

static int i2c_read_test(int parameter)
{
	uchar chip;
	uint addr;
	int alen = 0;
	uchar buffer[256];
	int len;
	bool loop;

	if(i2c_setbus() != 0) return DIAG_ERROR;
	printf("I2C chip address.(0x%s):0x", i2c_chip_addr);
	if(get_line(NULL, i2c_chip_addr, sizeof(i2c_chip_addr), -1, str_number_hex, NULL, i2c_chip_addr) < 0) return DIAG_ERROR;
	printf("Register address.(0x%s):0x", i2c_reg_addr);
	if((alen = get_line(NULL, i2c_reg_addr, sizeof(i2c_reg_addr), -1, str_number_hex, NULL, i2c_reg_addr)) < 0) return DIAG_ERROR;
	printf("Count.(%s):", i2c_count);
	if(get_line(NULL, i2c_count, sizeof(i2c_count), -1, str_number_dec, NULL, i2c_count) < 0) return DIAG_ERROR;
	printf("Loop?(%s):", i2c_loop);
	if(get_line(NULL, i2c_loop, sizeof(i2c_loop), -1, "01", NULL, i2c_loop) < 0) return DIAG_ERROR;

	chip = (uchar)(simple_strtoul(i2c_chip_addr, NULL, 16) & 0x000000ff);
	addr = (uint)(simple_strtoul(i2c_reg_addr, NULL, 16));
	len = (int)(simple_strtoul(i2c_count, NULL, 10));
	alen = (alen > 3) ? 2 : 1;
	loop = (bool)(simple_strtoul(i2c_loop, NULL, 10));
	if(len > sizeof(buffer)){
		printf("Count over buffer size(%d).\n", sizeof(buffer));
		len = sizeof(buffer);
	}

	udelay(1000);
	if(loop){
		int c = 0;

		while(c != 0x1B){
			if(tstc()) c = getc();
			else c = 0;
			if(i2c_read(chip, addr, alen, buffer, len)){
				printf("Failed to read I2C bus.\n");
				return DIAG_ERROR;
			}
			printf("Press 'ESC' to exit.\n");
			print_buffer((ulong)addr, (void*)buffer, 1, len, 0);
			udelay(100000);
		}

	}else{
		if(i2c_read(chip, addr, alen, buffer, len)){
			printf("Failed to read I2C bus.\n");
			return DIAG_ERROR;
		}
		print_buffer((ulong)addr, (void*)buffer, 1, len, 0);
	}
	return DIAG_OK;
}

static int i2c_write_test(int parameter)
{
	uchar chip;
	uint addr;
	int alen = 0;
	uchar value;
	int len;

	if(i2c_setbus() != 0) return DIAG_ERROR;
	printf("I2C chip address.(0x%s):0x", i2c_chip_addr);
	if(get_line(NULL, i2c_chip_addr, sizeof(i2c_chip_addr), -1, str_number_hex, NULL, i2c_chip_addr) < 0) return DIAG_ERROR;
	printf("Register address.(0x%s):0x", i2c_reg_addr);
	if((alen = get_line(NULL, i2c_reg_addr, sizeof(i2c_reg_addr), -1, str_number_hex, NULL, i2c_reg_addr)) < 0) return DIAG_ERROR;
	printf("Data.(0x%s):0x", i2c_value);
	if(get_line(NULL, i2c_value, sizeof(i2c_value), -1, str_number_hex, NULL, i2c_value) < 0) return DIAG_ERROR;
	printf("Count.(%s):", i2c_count);
	if(get_line(NULL, i2c_count, sizeof(i2c_count), -1, str_number_dec, NULL, i2c_count) < 0) return DIAG_ERROR;

	chip = (uchar)(simple_strtoul(i2c_chip_addr, NULL, 16) & 0x000000ff);
	addr = (uint)(simple_strtoul(i2c_reg_addr, NULL, 16));
	value = (uchar)(simple_strtoul(i2c_value, NULL, 16) & 0x000000ff);
	len = (int)(simple_strtoul(i2c_count, NULL, 10));
	alen = (alen > 3) ? 2 : 1;

	udelay(1000);
	while (len-- > 0) {
		if(i2c_write(chip, addr++, alen, &value, 1)){
			printf("Failed to write I2C bus.\n");
			return DIAG_ERROR;
		}
	}
	return DIAG_OK;
}

#if defined(VPORT06EC_2V)
static int i2c_addr_list_func(int parameter)
{
	int c = 0;

	printf("\n==============================================\n");
	printf("I2C Address List.\n");
	printf("==============================================\n");
	printf("Bus: 0, Addr:0x6f -> RTC (RTC_MPC7941)\n");
	printf("Bus: 0, Addr:0x57 -> EEPROM (RTC_MPC7941)\n");
	printf("Bus: 0, Addr:0x53 -> G-Sensor\n");
	printf("Bus: 0, Addr:0x2D -> PMIC\n");
	printf("Bus: 0, Addr:0x18 -> Audio\n");
	printf("Bus: 1, Addr:0x39 -> Light Sensor\n");
	printf("Bus: 1, Addr:0x40 -> Humidity Sensor (Si7021)\n");
	printf("Bus: 1, Addr:0x48 -> Temp. Sensor\n");
	printf("Bus: 2, Addr:0x10 -> Image Sensor (AR0331)\n");
	printf("Bus: 2, Addr:0x2D -> LVDS (SN65LVDS324)\n");

	printf("Press 'ESC' to exit.\n");
	while (c != 0x1B)
	{
		if (tstc())
		{
			c = getc();
		}
		else
		{
			c = 0;
		}

		udelay(100000);
	}

	return DIAG_OK;
}
#endif

//Sensor Test ---------------------------------------------------------------------------
static int LightSensor_test_func(int parameter);
static int LightSensor_I2C_test_func(int parameter);
static int ThermalSensor_I2C_test_func(int parameter);
#ifdef CONFIG_PI_ON_TEMP
static int Get_PI_ON_TEMP(int parameter);
static int Set_PI_ON_TEMP(int parameter);
#endif
#ifdef CONFIG_PI_OFF_TEMP
static int Get_PI_OFF_TEMP(int parameter);
static int Set_PI_OFF_TEMP(int parameter);
#endif
#ifdef CONFIG_SYS_ON_TEMP
static int Get_SYS_ON_TEMP(int parameter);
static int Set_SYS_ON_TEMP(int parameter);
#endif
#ifdef CONFIG_PI_OFF_OFFS
static int Get_PI_OFF_OFFS(int parameter);
static int Set_PI_OFF_OFFS(int parameter);
#endif
#ifdef CONFIG_SYS_ON_OFFS
static int Get_SYS_ON_OFFS(int parameter);
static int Set_SYS_ON_OFFS(int parameter);
#endif
#ifdef CONFIG_SYS_ON_TIMEOUT
static int Get_SYS_ON_TIMEOUT(int parameter);
static int Set_SYS_ON_TIMEOUT(int parameter);
#endif
static DiagMenuTableStruct SensorTestMenuTable[] = {
	{ '0', "Light Sensor (GPIO)", 			0,		LightSensor_test_func,			NULL},
	{ '1', "Light Sensor (I2C)",			0,		LightSensor_I2C_test_func,		NULL},
	{ '2', "Thermal Sensor (I2C)",			0,		ThermalSensor_I2C_test_func,	NULL},
#ifdef CONFIG_PI_ON_TEMP
	{ '3', "Get PI_ON Temperature",			0,		Get_PI_ON_TEMP,					NULL},
	{ '4', "Set PI_ON Temperature",			0,		Set_PI_ON_TEMP,					NULL},
#endif
#ifdef CONFIG_PI_OFF_TEMP
	{ '5', "Get PI_OFF Temperature",		0,		Get_PI_OFF_TEMP,				NULL},
	{ '6', "Set PI_OFF Temperature",		0,		Set_PI_OFF_TEMP,				NULL},
#endif
#ifdef CONFIG_SYS_ON_TEMP
	{ '7', "Get SYS_ON Temperature",		0,		Get_SYS_ON_TEMP,				NULL},
	{ '8', "Set SYS_ON Temperature",		0,		Set_SYS_ON_TEMP,				NULL},
#endif
#ifdef CONFIG_PI_OFF_OFFS
	{ '9', "Get PI_OFF offset Temperature",	0,		Get_PI_OFF_OFFS,				NULL},
	{ 'a', "Set PI_OFF offset Temperature",	0,		Set_PI_OFF_OFFS,				NULL},
#endif
#ifdef CONFIG_SYS_ON_OFFS
	{ 'b', "Get SYS_ON offset Temperature",	0,		Get_SYS_ON_OFFS,				NULL},
	{ 'c', "Set SYS_ON offset Temperature",	0,		Set_SYS_ON_OFFS,				NULL},
#endif
#ifdef CONFIG_SYS_ON_TIMEOUT
	{ 'd', "Get SYS_ON Timeout",			0,		Get_SYS_ON_TIMEOUT,				NULL},
	{ 'e', "Set SYS_ON Timeout",			0,		Set_SYS_ON_TIMEOUT,				NULL},
#endif
};
static DiagMenuStruct SensorTestMenu = {
	sizeof(SensorTestMenuTable)/sizeof(DiagMenuTableStruct),
	SensorTestMenuTable,
	"<<<Sensor Test Menu>>>",
	0, NULL
};

static int LightSensor_test_func(int parameter)
{
	int currstat = 0, oldstat = -1;
	printf("\n==============================================\n");
	printf("Cover/Uncover Light Sensor and check LED state.\n");
	printf("[Uncover -> SYS LED:Green, IR LED:OFF]\n");
	printf("[Cover   -> SYS LED:  Red, IR LED: ON]\n");
	printf("Press reset button to exit.\n");
	printf("==============================================\n");
	while(!resetkey_state()){
		currstat = lightsen_state();
		if((oldstat == -1) || (oldstat != currstat)){
			oldstat = currstat;
			if(currstat){
				sys_led_R_OFF();
				sys_led_G_ON();
				gpio_write_func(GPIO_WRITE_HIGH_ICR);
				gpio_write_func(GPIO_WRITE_LOW_IR_LED);
				printf("[Uncover -> SYS LED:Green, IR LED:OFF]\n");
			}else{
				sys_led_G_OFF();
				sys_led_R_ON();
				gpio_write_func(GPIO_WRITE_LOW_ICR);
				gpio_write_func(GPIO_WRITE_HIGH_IR_LED);
				printf("[Cover	 -> SYS LED:  Red, IR LED: ON]\n");
			}
		}
		udelay(100000);
	}
	sys_led_RG_OFF();
	return DIAG_OK;
}

int LightSensor_I2C_reset(uchar chip)
{
	int ret = 0;
	uchar byte;
	byte = 0x00;//Power Down
	if ((ret = i2c_write(chip, 0x00, 1, &byte, 1)) != 0)
		printf("Error writing the i2c chip.(%d)\n", ret);
	udelay(100000);
	byte = 0x08;//Power On
	if ((ret = i2c_write(chip, 0x00, 1, &byte, 1)) != 0)
		printf("Error writing the i2c chip.(%d)\n", ret);
	udelay(100000);
	return DIAG_OK;
}

uint LightSensor_I2C_state(uchar chip)
{
	int ret = 0;
	uint addr, value = 0;
	unsigned char linebuf[2];
	addr = 0x06;
	if ((ret = i2c_read(chip, addr, 1, linebuf, 2)) == 0){
		value = linebuf[0] | (linebuf[1] << 8);
	}
	return value;
}

static int LightSensor_I2C_test_func(int parameter)
{
	int c = 0;
	if (i2c_probe(LIGHT_SENSOR_ADDR) != 0){
		printf("Can't find Light Sensor at i2c adderss 0x%02X\n", LIGHT_SENSOR_ADDR);
	}else{
		LightSensor_I2C_reset(LIGHT_SENSOR_ADDR);
		printf("Press 'ESC' to exit.\n");
		while(c != 0x1B){
			if(tstc()) c = getc();
			else c = 0;
			if((c == 'r') || (c == 'R')) LightSensor_I2C_reset(LIGHT_SENSOR_ADDR);
			printf(" Light Sensor -> 0x%04X\r", LightSensor_I2C_state(LIGHT_SENSOR_ADDR));
			udelay(100000);
		}
	}
	return DIAG_OK;
}

/*
LM75 Digital Thermal Sensor
Temperature | Digital Output | Binary Hex
         +125C | 0 111 1101  | 7Dh
           +25C | 0 001 1001  | 19h
             +1C | 0 000 0001  | 01h
               0C | 0 000 0000  | 00h
	      -1C | 1 111 1111  | FFh
           -25C | 1 110 0111  | E7h
           -55C | 1 100 1001  | C9h
*/
int t_temp(int value)
{
	if(value & 0x80){
		return -((~value & 0x7F) + 1);
	}else{
		return value;
	}
}

int ThermalSensor_I2C_state(uchar chip)
{
	int ret = 0;
	uint addr;
	int value = 0;
	unsigned char linebuf[2];
	addr = 0x00;
	if ((ret = i2c_read(chip, addr, 1, linebuf, 1)) == 0){
		value = (int)linebuf[0];
	}
	return (value);
}

static int ThermalSensor_I2C_test_func(int parameter)
{
	int c = 0;
	int value = 0;
	if (i2c_probe(THERMAL_SENSOR_ADDR) != 0){
		printf("Can't find Thermal Sensor at i2c adderss 0x%02X\n", THERMAL_SENSOR_ADDR);
	}else{
		printf("Press 'ESC' to exit.\n");
		while(c != 0x1B){
			if(tstc()) c = getc();
			else c = 0;
			value = (int)ThermalSensor_I2C_state(THERMAL_SENSOR_ADDR);
			printf(" Thermal Sensor -> %dC (0x%02X)               \r", t_temp(value), value);
			udelay(100000);
		}
	}
	return DIAG_OK;
}

#ifdef CONFIG_PI_ON_TEMP
static int Get_PI_ON_TEMP(int parameter)
{
	int temp = 0;
	if(env_get_pi_on_temp(&temp)){
		printf("Get PI_ON Temperature Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("PI_ON Temperature: %dC\n", t_temp(temp));
		return DIAG_OK;
	}
}

static int Set_PI_ON_TEMP(int parameter)
{
	int temp = 0;
	char tmp[62];
	memset(tmp, 0, sizeof(tmp));
	if(get_line("Set PI_ON Temperature(125 ~ -55):", tmp, sizeof(tmp), -1, "0123456789-", NULL, NULL) <= 0 ) return DIAG_ERROR;
	temp = (int)simple_strtol(tmp, NULL, 10) & 0xFF;
	if(env_set_pi_on_temp(temp)){
		printf("Set PI_ON Temperature Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("Set PI_ON Temperature = %dC (0x%02X)\n", t_temp(temp), temp);
		return DIAG_OK;
	}
}
#endif
#ifdef CONFIG_PI_OFF_TEMP
static int Get_PI_OFF_TEMP(int parameter)
{
	int temp = 0;
	if(env_get_pi_off_temp(&temp)){
		printf("Get PI_OFF Temperature Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("PI_OFF Temperature: %dC\n", t_temp(temp));
		return DIAG_OK;
	}
}

static int Set_PI_OFF_TEMP(int parameter)
{
	int temp = 0;
	char tmp[62];
	memset(tmp, 0, sizeof(tmp));
	if(get_line("Set PI_OFF Temperature(125 ~ -55):", tmp, sizeof(tmp), -1, "0123456789-", NULL, NULL) <= 0 ) return DIAG_ERROR;
	temp = (int)simple_strtol(tmp, NULL, 10) & 0xFF;
	if(env_set_pi_off_temp(temp)){
		printf("Set PI_OFF Temperature Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("Set PI_OFF Temperature = %dC (0x%02X)\n", t_temp(temp), temp);
		return DIAG_OK;
	}
}
#endif
#ifdef CONFIG_SYS_ON_TEMP
static int Get_SYS_ON_TEMP(int parameter)
{
	int temp = 0;
	if(env_get_sys_on_temp(&temp)){
		printf("Get SYS_ON Temperature Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("SYS_ON Temperature: %dC\n", t_temp(temp));
		return DIAG_OK;
	}
}

static int Set_SYS_ON_TEMP(int parameter)
{
	int temp = 0;
	char tmp[62];
	memset(tmp, 0, sizeof(tmp));
	if(get_line("Set SYS_ON Temperature(125 ~ -55):", tmp, sizeof(tmp), -1, "0123456789-", NULL, NULL) <= 0 ) return DIAG_ERROR;
	temp = (int)simple_strtol(tmp, NULL, 10) & 0xFF;
	if(env_set_sys_on_temp(temp)){
		printf("Set SYS_ON Temperature Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("Set SYS_ON Temperature = %dC (0x%02X)\n", t_temp(temp), temp);
		return DIAG_OK;
	}
}
#endif
#ifdef CONFIG_PI_OFF_OFFS
static int Get_PI_OFF_OFFS(int parameter)
{
	int temp = 0;
	if(env_get_pi_off_offs(&temp)){
		printf("Get PI_OFF_OFFS Temperature Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("PI_OFF_OFFS Temperature: %dC\n", t_temp(temp));
		return DIAG_OK;
	}
}

static int Set_PI_OFF_OFFS(int parameter)
{
	int temp = 0;
	char tmp[62];
	memset(tmp, 0, sizeof(tmp));
	if(get_line("Set PI_OFF_OFFS Temperature(125 ~ -55):", tmp, sizeof(tmp), -1, "0123456789-", NULL, NULL) <= 0 ) return DIAG_ERROR;
	temp = (int)simple_strtol(tmp, NULL, 10) & 0xFF;
	if(env_set_pi_off_offs(temp)){
		printf("Set PI_OFF_OFFS Temperature Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("Set PI_OFF_OFFS Temperature = %dC (0x%02X)\n", t_temp(temp), temp);
		return DIAG_OK;
	}
}

static int Set_PI_OFF_TEMP_By_OFFS(int sys_temp)
{
	int temp = 0;
	if(env_get_pi_off_offs(&temp)){
		printf("Get PI_OFF_OFFS Temperature Failed!!\n");
		temp = CONFIG_PI_OFF_OFFS;
	}
	temp = (temp + sys_temp) & 0x000000FF;
	if(env_set_pi_off_temp(temp)){
		printf("Set PI_OFF Temperature Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("Set PI_OFF Temperature = %dC (0x%02X)\n", t_temp(temp), temp);
		return DIAG_OK;
	}
}
#endif
#ifdef CONFIG_SYS_ON_OFFS
static int Get_SYS_ON_OFFS(int parameter)
{
	int temp = 0;
	if(env_get_sys_on_offs(&temp)){
		printf("Get SYS_ON_OFFS Temperature Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("SYS_ON_OFFS Temperature: %dC\n", t_temp(temp));
		return DIAG_OK;
	}
}

static int Set_SYS_ON_OFFS(int parameter)
{
	int temp = 0;
	char tmp[62];
	memset(tmp, 0, sizeof(tmp));
	if(get_line("Set SYS_ON_OFFS Temperature(125 ~ -55):", tmp, sizeof(tmp), -1, "0123456789-", NULL, NULL) <= 0 ) return DIAG_ERROR;
	temp = (int)simple_strtol(tmp, NULL, 10) & 0xFF;
	if(env_set_sys_on_offs(temp)){
		printf("Set SYS_ON_OFFS Temperature Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("Set SYS_ON_OFFS Temperature = %dC (0x%02X)\n", t_temp(temp), temp);
		return DIAG_OK;
	}
}

static int Set_SYS_ON_TEMP_By_OFFS(int sys_temp)
{
	int temp = 0;
	if(env_get_sys_on_offs(&temp)){
		printf("Get SYS_ON_OFFS Temperature Failed!!\n");
		temp = CONFIG_SYS_ON_OFFS;
	}
	temp = (temp + sys_temp) & 0x000000FF;
	if(env_set_sys_on_temp(temp)){
		printf("Set SYS_ON Temperature Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("Set SYS_ON Temperature = %dC (0x%02X)\n", t_temp(temp), temp);
		return DIAG_OK;
	}
}
#endif
#ifdef CONFIG_SYS_ON_TIMEOUT
static int Get_SYS_ON_TIMEOUT(int parameter)
{
	unsigned int sec = 0;
	if(env_get_sys_on_timeout(&sec)){
		printf("Get CONFIG_SYS_ON_TIMEOUT Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("CONFIG_SYS_ON_TIMEOUT : %u sec\n", sec);
		return DIAG_OK;
	}
}

static int Set_SYS_ON_TIMEOUT(int parameter)
{
	unsigned int sec = 0;
	char tmp[5];
	memset(tmp, 0, sizeof(tmp));
	if(get_line("Set CONFIG_SYS_ON_TIMEOUT (sec):", tmp, sizeof(tmp), -1, str_number_dec, NULL, NULL) <= 0 ) return DIAG_ERROR;
	sec = (unsigned int)simple_strtoul(tmp, NULL, 10);
	if(env_set_sys_on_timeout(sec)){
		printf("Set CONFIG_SYS_ON_TIMEOUT Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("Set CONFIG_SYS_ON_TIMEOUT = %d sec\n", sec);
		return DIAG_OK;
	}
}
#endif

//Diagnostic Menu======================================================
static int sdram_test_func(int parameter);
static int timer_test_func(int parameter);
#ifdef CONFIG_HW_WATCHDOG
static int wdt_reset_test_func(int parameter);
#endif
static int flash_test_func(int parameter);
static int SD_test_func(int parameter);
static int Audio_test_func(int parameter);
static int addr_read_func(int parameter);
static int addr_write_func(int parameter);
static int ping_test_func(int parameter);
static DiagMenuTableStruct DiagnosticMenuTable[] = {
	{ '0',	"DDR Test",					0x200000,	sdram_test_func,		NULL},
	{ '1',	"Timer Test",				0,	timer_test_func,		NULL},
//	{ '2',	"DMA Test",					0,	dma_test_func,			NULL},
#ifdef CONFIG_HW_WATCHDOG
	{ '3',	"Watchdog Reset Test",		0,	wdt_reset_test_func,	NULL},
#endif
//	{ '4',	"MAC Loopback Test",		0,	ftmac100_test_func,		NULL},
	{ '5',	"Flash Test",				0,	flash_test_func,		NULL},
//	{ '6',	"I2S Module1 Test",			0,	i2s_test_1,				NULL},
//	{ '7',	"I2S Module2 Test",			0,	i2s_test_2,				NULL},
	{ '8',	"I2C Test",					0,	NULL,					&I2CTestMenu},
	{ '9',	"RTC Test",					0,	NULL,					&RTCTestMenu},
//	{ 'a',	"RTC Alarm Test",			0,	rtc_alarm_test,			NULL},
	{ 'b',	"GPIO Test", 				0,	NULL, 					&GPIOTestMenu},
	{ 'c',	"LED Test", 				0,	NULL, 					&LedTestMenu},
//	{ 'd',	"Enable Cache",				0,	ec_test_func,			NULL},
//	{ 'e',	"Disable Cache",			0,	dc_test_func,			NULL},
	{ 'f',	"SD Card Test", 			0,	SD_test_func, 			NULL},
//	{ 'g',	"IDE Storage Test",			0,	IDE_main_func,			NULL},
//	{ 'h',	"PCI-X Host Test",			0,	pcix_test_func,			NULL},
//	{ 'i',	"OTG-200 Test", 			0,	OTG_200_func, 			NULL},
//	{ 'j',	"Security Test", 			0,	security_test_func, 	NULL},
//	{ 'k',	"MCP Clock Setting",		0,	mcp_clk_func,			NULL},
	{ 'l',	"Sensor Test",				0,	NULL,					&SensorTestMenu},
	{ 'm',	"Audio Test", 				0,	Audio_test_func, 		NULL},
	{ 'n',	"Address Read",				0,	addr_read_func,			NULL},
	{ 'o',	"Address Write",			0,	addr_write_func,		NULL},
	{ 'p',	"Ping test",				0,	ping_test_func,			NULL},
	{ 'r',	"DI/DO Test",				0,	NULL,					&DIDOTestMenu},
	{ 't',	"UART test",				0,	NULL,					&UARTTestMenu},
#if CONFIG_SYS_I2C_EEPROM
	{ 's',	"EEPROM clear",				0,	EEPROM_Clear, 			NULL},
#endif
//	{ 'u',	"PRTCIF Read/Write",		0,	NULL,					&PRTCIFMenu},
};
static DiagMenuStruct DiagnosticMenu = {
	sizeof(DiagnosticMenuTable)/sizeof(DiagMenuTableStruct),
	DiagnosticMenuTable,
	"<<<Diagnostic Menu>>>",
	0, NULL
};

static int sdram_test_func(int parameter)
{
	char tmp[10];//ffffffff
	unsigned int startAddr = CFG_MEMTEST_START;
	size_t len = (size_t)parameter;
	int verbose = 0;
	unsigned long Size = 0;

	memset(tmp, 0, sizeof(tmp));

#ifdef PHYS_DRAM_1
	printf("DRAM Bank #1 Addr:0x%08X\n", PHYS_DRAM_1);
	printf("DRAM Bank #1 Size:0x%08X\n", PHYS_DRAM_1_SIZE);
#endif
#ifdef PHYS_DRAM_2
	printf("DRAM Bank #2 Addr:0x%08X\n", PHYS_DRAM_2);
	printf("DRAM Bank #2 Size:0x%08X\n", PHYS_DRAM_2_SIZE);
#endif

	printf("Start address(0x%08X):0x", startAddr);
	if(get_line(NULL, tmp, sizeof(tmp), -1, str_number_hex, NULL, NULL) > 0)
		startAddr = (unsigned int)simple_strtoul(tmp, NULL, 16);
	if(((startAddr >= PHYS_DRAM_1) && (startAddr <= (PHYS_DRAM_1+PHYS_DRAM_1_SIZE-1)))
#ifdef PHYS_DRAM_2
		|| ((startAddr >= PHYS_DRAM_2) && (startAddr <= (PHYS_DRAM_2+PHYS_DRAM_2_SIZE-1)))
#endif
	){
		printf("Test size(0x%08X):0x", len);
		if(get_line(NULL, tmp, sizeof(tmp), -1, str_number_hex, NULL, NULL) > 0)
			len = (size_t)simple_strtoul(tmp, NULL, 16);
		printf("Show detail(%d):", verbose);
		if(get_line(NULL, tmp, sizeof(tmp), -1, "01", NULL, NULL) > 0)
			verbose = (int)simple_strtol(tmp, NULL, 10);
		while(1){
			printf("\npress 'Ctrl+C' to stop.\n\n");
			if(mem_test(len, 0x5555aaaa, startAddr, verbose)) return DIAG_ERROR;
			if(mem_test(len, 0xaaaa5555, startAddr, verbose)) return DIAG_ERROR;
			if(mem_test(len, 0x55555555, startAddr, verbose)) return DIAG_ERROR;
			if(mem_test(len, 0xaaaaaaaa, startAddr, verbose)) return DIAG_ERROR;
			if(mem_test(len, 0xffff0000, startAddr, verbose)) return DIAG_ERROR;
			if(mem_test(len, 0x0000ffff, startAddr, verbose)) return DIAG_ERROR;
			if(mem_test(len, 0xffffffff, startAddr, verbose)) return DIAG_ERROR;
			if(mem_test(len, 0x00000000, startAddr, verbose)) return DIAG_ERROR;
		}
	}
	return DIAG_OK;
}

static int timer_test_func(int parameter)
{
	char tmp[4];
	ulong start = 0;
	ulong count, countBak = -1;
	ulong delay = 0;

	if(get_line("Set timer time out sec.(0~99):", tmp, sizeof(tmp), -1, str_number_dec, NULL, NULL) <= 0) return DIAG_ERROR;
	delay = (ulong)simple_strtoul(tmp, NULL, 10) * CFG_HZ;

	printf("\npress 'Ctrl+C' to stop.\n\n");
	reset_timer();
	start = get_timer(0);
	while((count = get_timer(start)) < delay) {
		if(ctrlc ()){
			printf("\nAbort!!\n");
			return DIAG_OK;
		}
		count /= 1000;
		if(countBak != count){
			printf("time : %ld        \r", count);
			countBak = count;
		}
		udelay (100);
	}
	printf("\nTime out!!\n");
	return DIAG_OK;
}

#ifdef CONFIG_HW_WATCHDOG
static int wdt_reset_test_func(int parameter)
{
	int c = 0, i = 0;
	char tmp[10];//ffffffff
	printf("\nEnable Watchdog Timer.\n");
	hw_watchdog_op(HWWD_ON);
	hw_watchdog_op(HWWD_TRIGGER_SKIP);
	while(1){
		if(tstc()){
			c = getc();
			if(c == 'i') { hw_watchdog_op(HWWD_INIT); }
			else if(c == 'e') { hw_watchdog_op(HWWD_ON); }
			else if(c == 'd') { hw_watchdog_op(HWWD_OFF); }
			else if(c == 'r') { hw_watchdog_op(HWWD_SOFT_RST); }
			else if(c == 't') { hw_watchdog_op(HWWD_RST); }
			else if(c == 'f') { hw_watchdog_op(HWWD_FORCE_RST); }
			else if(c == 's'){
				hw_watchdog_op(HWWD_OFF);
				if(get_line("\nSet Watchdog Timer(sec.):", tmp, sizeof(tmp), -1, str_number_dec, NULL, NULL) > 0){
					i = (int)simple_strtoul(tmp, NULL, 10);
					(*(volatile unsigned int *)(WDT_WLDR)) = (0xFFFFFFFF - (WDT_TIMEOUT_BASE * i));
					udelay(1000); hw_watchdog_op(HWWD_RST);
					udelay(1000); hw_watchdog_op(HWWD_ON);
				}
			}
		}
		printf(" System will reboot after %d sec.(Press 'f' to force reboot)    \r",
			(0xFFFFFFFF - (*(volatile unsigned int *)(WDT_WCRR))) / WDT_TIMEOUT_BASE);
		udelay(500000);
	}
	return DIAG_OK;
}

int do_wdt (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	int i = 0;
	if(argc < 2){
		printf ("Usage:\n%s\n", cmdtp->usage);
		return -1;
	}else{
		if(strcmp(argv[1], "on") == 0){
			hw_watchdog_op(HWWD_ON);
		}else
		if(strcmp(argv[1], "off") == 0){
			hw_watchdog_op(HWWD_OFF);
		}else
		if(strcmp(argv[1], "set") == 0){
			if(argc < 3){
				printf ("Usage:\n%s\n", cmdtp->usage);
				return -1;
			}
			hw_watchdog_op(HWWD_OFF);
			i = (int)simple_strtoul(argv[2], NULL, 10);
			(*(volatile unsigned int *)(WDT_WLDR)) = (0xFFFFFFFF - (WDT_TIMEOUT_BASE * i));
			udelay(1000); hw_watchdog_op(HWWD_RST);
			udelay(1000); hw_watchdog_op(HWWD_ON);

		}else
		if(strcmp(argv[1], "rst") == 0){
			hw_watchdog_op(HWWD_RST);
		}else
		if(strcmp(argv[1], "frst") == 0){
			hw_watchdog_op(HWWD_FORCE_RST);
		}
	}
	return 0;
}

U_BOOT_CMD(wdt, 3, 0, do_wdt,
	"Watchdog Timer",
	"Watchdog Timer\n\
	wdt on/off - ON/OFF Watchdog Timer\n\
	wdt set [val] - Set Watchdog Timer\n\
	wdt rst - Watchdog Timer reset\n\
	wdt frst - Force system reboot"
);
#endif

int flash_dump_badblock(nand_info_t *nand)
{
	int ret = 0;
	ulong off = 0;
	for (off = 0; off < nand->size; off += nand->erasesize){
		if (nand_block_isbad(nand, off)){
			printf("  0x%08lX\n", off);
			ret++;
		}
	}
	return ret;
}

int flash_check_badblock(nand_info_t *nand)
{
	int ret = 0;
	int isBad = 0;
	int count = 0;
	ulong off = 0;
	for (off = 0; off < nand->size; off += nand->erasesize){
		isBad = nand_block_isbad(nand, off);
#ifdef UBL_FLASH
		if(off == UBL_FLASH){
			if(isBad){
				printf("Bad Block:0x%08lX (at the first block of UBL.)\n", off);
				return -1;
			}else count = 0;
		}else
#endif
#ifdef ENV1_FLASH
		if(off == ENV1_FLASH){
			if(isBad){
				printf("Bad Block:0x%08lX (at the first block of ENV.)\n", off);
				return -1;
			}else count = 0;
		}else
#endif
#ifdef ENV2_FLASH
		if(off == ENV2_FLASH){
			if(isBad){
				printf("Bad Block:0x%08lX (at the first block of ENV2.)\n", off);
				return -1;
			}else count = 0;
		}else
#endif
#ifdef UBL_FLASH
		if((off > UBL_FLASH) && (off < (UBL_FLASH + UBL_SIZE))){
			if(isBad){
				count++;
				printf("Bad Block:0x%08lX (in UBL), count=%d\n", off, count);
			}
		}else
#endif
#ifdef UBOOT_FLASH
		if((off >= UBOOT_FLASH) && (off < (UBOOT_FLASH + UBOOT_SIZE))){
			if(isBad){
				count++;
				printf("Bad Block:0x%08lX (in UBOOT), count=%d\n", off, count);
			}
		}else
#endif
#ifdef ENV1_FLASH
		if((off >= ENV1_FLASH) && (off < (ENV1_FLASH + ENV1_SIZE))){
			if(isBad){
				count++;
				printf("Bad Block:0x%08lX (in ENV), count=%d\n", off, count);
			}
		}else
#endif
#ifdef ENV2_FLASH
		if((off >= ENV2_FLASH) && (off < (ENV2_FLASH + ENV2_SIZE))){
			if(isBad){
				count++;
				printf("Bad Block:0x%08lX (in ENV2), count=%d\n", off, count);
			}
		}else
#endif
#ifdef KERNEL_FLASH
		if((off >= KERNEL_FLASH) && (off < (KERNEL_FLASH + KERNEL_SIZE))){
			if(isBad){
				count++;
				printf("Bad Block:0x%08lX (in KERNEL), count=%d\n", off, count);
			}
		}else
#endif
#ifdef ROOTFS_FLASH
		if((off >= ROOTFS_FLASH) && (off < (ROOTFS_FLASH + ROOTFS_SIZE))){
			if(isBad){
				count++;
				printf("Bad Block:0x%08lX (in ROOTFS), count=%d\n", off, count);
			}
		}else
#endif
#ifdef KERNEL2_FLASH
		if((off >= KERNEL2_FLASH) && (off < (KERNEL2_FLASH + KERNEL2_SIZE))){
			if(isBad){
				count++;
				printf("Bad Block:0x%08lX (in KERNEL2), count=%d\n", off, count);
			}
		}else
#endif
#ifdef ROOTFS2_FLASH
		if((off >= ROOTFS2_FLASH) && (off < (ROOTFS2_FLASH + ROOTFS2_SIZE))){
			if(isBad){
				count++;
				printf("Bad Block:0x%08lX (in ROOTFS2), count=%d\n", off, count);
			}
		}else
#endif
#ifdef DATA1_FLASH
		if((off >= DATA1_FLASH) && (off < (DATA1_FLASH + DATA1_SIZE))){
			if(isBad){
				count++;
				printf("Bad Block:0x%08lX (in DATA1), count=%d\n", off, count);
			}
		}else
#endif
#ifdef MPKERNEL_FLASH
		if((off >= MPKERNEL_FLASH) && (off < (MPKERNEL_FLASH + MPKERNEL_SIZE))){
			if(isBad){
				count++;
				printf("Bad Block:0x%08lX (in MPKERNEL), count=%d\n", off, count);
			}
		}else
#endif
#ifdef MPROOTFS_FLASH
		if((off >= MPROOTFS_FLASH) && (off < (MPROOTFS_FLASH + MPROOTFS_SIZE))){
			if(isBad){
				count++;
				printf("Bad Block:0x%08lX (in MPROOTFS), count=%d\n", off, count);
			}
		}else
#endif
#ifdef DATA2_FLASH
		if((off >= DATA2_FLASH) && (off < (DATA2_FLASH + DATA2_SIZE))){
			if(isBad){
				count++;
				printf("Bad Block:0x%08lX (in DATA2), count=%d\n", off, count);
			}
		}else
#endif
#ifdef MPDATA_FLASH
		if((off >= MPDATA_FLASH) && (off < (MPDATA_FLASH + MPDATA_SIZE))){
			if(isBad){
				count++;
				printf("Bad Block:0x%08lX (in MPDATA), count=%d\n", off, count);
			}
		}else
#endif
		{
		}
	}
	return ret;
}

int flash_erase(nand_info_t *nand, loff_t offset, size_t length)
{
	nand_erase_options_t opts;
	memset(&opts, 0, sizeof(opts));
	opts.offset = offset;
	opts.length = length;
	opts.jffs2	= 0;
	opts.scrub	= 0;
	opts.quiet	= 1;
	if (nand_erase_opts(nand, &opts)){
		printf(" Erase %d Byte at offset 0x%08llX failed!!\n", length, offset);
		return -1;
	}
	return 0;
}

#define ERR_FLASH_ERASE -1
#define ERR_FLASH_WRITE -2
#define ERR_FLASH_READ -3
#define ERR_MEMCMP -4
static char* error_string(int err_code)
{
	switch(err_code){
		case ERR_FLASH_ERASE:
			return "NAND Erase error";
			break;
		case ERR_FLASH_WRITE:
			return "NAND Write error";
			break;
		case ERR_FLASH_READ:
			return "NAND Read error";
			break;
		case ERR_MEMCMP:
			return "MEM Compare error";
			break;
		default:
			return "Unknown error";
			break;
	}
}

int flash_compared(nand_info_t *nand, loff_t offset, size_t length, char pattern)
{
	int ret = 0;
	ulong start = 0;
	long timepass = 0;
	u_char buf_rd[FLASH_TEST_SIZE];
	u_char buf_wt[FLASH_TEST_SIZE];
	if(flash_erase(nand, offset, length)) return ERR_FLASH_ERASE;
	memset(buf_wt, pattern, FLASH_TEST_SIZE);
	ret = nand_write_skip_bad(nand, offset, &length, (u_char *)buf_wt);
	if (ret != 0) return ERR_FLASH_WRITE;
	ret = nand_read_skip_bad(nand, offset, &length, (u_char *)buf_rd);
	if (ret != 0) return ERR_FLASH_READ;
	ret = memcmp(buf_rd, buf_wt, FLASH_TEST_SIZE);
	if (ret != 0) return ERR_MEMCMP;
	return 0;
}

static int flash_backup(nand_info_t *nand, loff_t offset, size_t length, u_char* buff)
{
	int ret = DIAG_OK;
	ulong start = 0;
	long timepass = 0;
	ret = nand_read_skip_bad(nand, offset, &length, (u_char *)buff);
	if (ret != 0) return ERR_FLASH_READ;
	return 0;
}

static int flash_restore(nand_info_t *nand, loff_t offset, size_t length, u_char* buff)
{
	int ret = DIAG_OK;
	ulong start = 0;
	long timepass = 0;
	if(flash_erase(nand, offset, length)) return ERR_FLASH_ERASE;
	ret = nand_write_skip_bad(nand, offset, &length, (u_char *)buff);
	if (ret != 0) return ERR_FLASH_WRITE;
	return 0;
}

#define CHK_USER_ABORT(abort) if(ctrlc()){ printf("\nAbort!!\n"); abort = 1; }

static int flash_test_func(int parameter)
{
	int ret = DIAG_OK;
	int i = 0;
	char tmp[10];//ffffffff
	u_char c_in = 0;
	struct rtc_time tm_start,tm_end;
	nand_info_t *nand = &nand_info[nand_curr_device];
	if(!nand_info[nand_curr_device].name) {
		printf("ERROR: NAND Device not available\n");
		printf("\tUse # nand device to set current device.\n");
		return DIAG_ERROR;
	}
	printf("\nDevice %d bad block list:\n", nand_curr_device);
	printf("  total %d bad blocks.\n", flash_dump_badblock(nand));
	if(flash_check_badblock(nand)){
		printf("ERROR: NAND Device has unacceptable Bad Block.\n");
		return DIAG_ERROR;
	}
	loff_t start_addr = TEST_FLASH;
	size_t test_length = TEST_FLASH_SIZE;

	printf("\nStart address(0x%08llX):0x", start_addr);
	ret = get_line(NULL, tmp, sizeof(tmp), -1, str_number_hex, NULL, NULL);
	if(ret > 0){
		start_addr = (loff_t)simple_strtoul(tmp, NULL, 16);
	}else if(ret < 0){
		puts("\ntest aborted.\n");
		return DIAG_ERROR;
	}

	printf("Test length(0x%08X):0x", test_length);
	ret = get_line(NULL, tmp, sizeof(tmp), -1, str_number_hex, NULL, NULL);
	if(ret > 0){
		test_length = (unsigned int)simple_strtoul(tmp, NULL, 16);
	}else if(ret < 0){
		puts("\ntest aborted.\n");
		return DIAG_ERROR;
	}

	printf("\nWARNING: All data in this range(0x%08llX~0x%08llX) will be lost.\n", start_addr, start_addr + test_length);
	printf("press 'Ctrl+C' to stop.\n");
	printf("make sure you want to continue <y/N>:");
	c_in = getc(); putc(c_in);
	if (c_in != 'y') {
		puts("\ntest aborted.\n");
		return DIAG_ERROR;
	}
	printf("\n\n");

	u_char buf_bk[FLASH_TEST_SIZE];
	loff_t offset = start_addr;
	size_t length = 0;
	size_t total_length = 0;
	int err = 0;
	int user_abort = 0;
	char pattern[] = {0x00, 0x55, 0xAA, 0xFF};
	do{
		err = 0;
		if(test_length > sizeof(buf_bk)) length = sizeof(buf_bk);
		else length = test_length;

		CHK_USER_ABORT(user_abort);
		if(user_abort) break;

		printf("NAND Backup  %d Bytes at offset 0x%08llX                         \r", length, offset);
		if((ret = flash_backup(nand, offset, length, buf_bk)) != 0){
			printf("\n %s!! (%d Bytes at offset 0x%08llX)\n", error_string(ret), length, offset);
			return DIAG_ERROR;
		}

		for(i = 0; i < sizeof(pattern); i++){
			CHK_USER_ABORT(user_abort);
			if(user_abort) break;
			printf("NAND Compare %d Bytes at offset 0x%08llX, pattern:0x%02X 		\r", length, offset, pattern[i]);
			if((ret = flash_compared(nand, offset, length, pattern[i])) != 0){
				err++;
				printf("\n %s!! (%d Byte at offset 0x%08llX)\n", error_string(ret), length, offset);
			}
		}

		printf("NAND Restore %d Bytes at offset 0x%08llX                         \r", length, offset);
		if((ret = flash_restore(nand, offset, length, buf_bk)) != 0){
			err++;
			printf("\n %s!! (%d Byte at offset 0x%08llX)\n", error_string(ret), length, offset);
		}

		if(err){
			printf("\n\n NAND Test %d Bytes at offset 0x%08llX Failed!!\n", length, offset);
			return DIAG_ERROR;
		}

		CHK_USER_ABORT(user_abort);
		if(user_abort) break;

		offset += length;
		total_length += length;
	}while(total_length < test_length);
	printf("\n\n NAND Test at offset 0x%08llX~0x%08llX Passed.\n", start_addr, start_addr + test_length);
	return DIAG_OK;
}

static int SD_test_func(int parameter)
{
	sdcard_enable();
	return (SD_test(parameter) ? DIAG_OK : DIAG_ERROR);
}

int do_mmctest (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	int dev_num = 0;
	if (argc < 2) dev_num = 0;
	else dev_num = simple_strtoul(argv[1], NULL, 0);
	return SD_test_func(dev_num);
}

U_BOOT_CMD(mmctest, 2, 0, do_mmctest,
	"Test MMC device",
	"mmctest [dev num]"
);

int AIC3104VolumeUpDown(int up)
{
	uchar value;
	if(up == 1){
		DRVfnAudio_AIC3104RecvData(PGAL_2_LLOPM_VOL, &value, 1); printf("%u/", value & ~ROUTE_ON);
		if((value & ~ROUTE_ON) > 0) DRVfnAudio_AIC3104SendData(PGAL_2_LLOPM_VOL, value - 1);
		DRVfnAudio_AIC3104RecvData(PGAR_2_RLOPM_VOL, &value, 1); printf("%u\n", value & ~ROUTE_ON);
		if((value & ~ROUTE_ON) > 0) DRVfnAudio_AIC3104SendData(PGAR_2_RLOPM_VOL, value - 1);
	}else{
		DRVfnAudio_AIC3104RecvData(PGAL_2_LLOPM_VOL, &value, 1); printf("%u/", value & ~ROUTE_ON);
		if((value & ~ROUTE_ON) < 118) DRVfnAudio_AIC3104SendData(PGAL_2_LLOPM_VOL, value + 1);
		DRVfnAudio_AIC3104RecvData(PGAR_2_RLOPM_VOL, &value, 1); printf("%u\n", value & ~ROUTE_ON);
		if((value & ~ROUTE_ON) < 118) DRVfnAudio_AIC3104SendData(PGAR_2_RLOPM_VOL, value + 1);
	}
	return 0;
}

static int Audio_test_func(int parameter)
{
	int ret = DIAG_OK;
	int c = 0;
	u8 buff[AIC3X_CACHEREGNUM];
	uchar addr;
	uchar value;
	int len;

	ret = Audio_test();
	Aic3104_RegisterDump();

	printf("Press 'q':Quit / r:Read / w:Write / d:Dump Reg. / '+,-':Volume)\n");
	while(c != 'q'){
		if(tstc()){
			c = getc();
			if(c == 'r') {
				printf("Register address.(0x%s):0x", i2c_reg_addr);
				if(get_line(NULL, i2c_reg_addr, sizeof(i2c_reg_addr), -1, str_number_hex, NULL, i2c_reg_addr) >= 0){
					printf("Count.(%s):", i2c_count);
					if(get_line(NULL, i2c_count, sizeof(i2c_count), -1, str_number_dec, NULL, i2c_count) >= 0){
						addr = (uchar)(simple_strtoul(i2c_reg_addr, NULL, 16) & 0x000000ff);
						len = (uchar)(simple_strtoul(i2c_count, NULL, 10));
						DRVfnAudio_AIC3104RecvData(addr, buff, len);
						print_buffer(addr, buff, 1, len, 0);
					}
				}
			}
			else if(c == 'w') {
				printf("Register address.(0x%s):0x", i2c_reg_addr);
				if(get_line(NULL, i2c_reg_addr, sizeof(i2c_reg_addr), -1, str_number_hex, NULL, i2c_reg_addr) >= 0){
					printf("Data.(0x%s):0x", i2c_value);
					if(get_line(NULL, i2c_value, sizeof(i2c_value), -1, str_number_hex, NULL, i2c_value) >= 0){
						addr = (uchar)(simple_strtoul(i2c_reg_addr, NULL, 16) & 0x000000ff);
						value = (uchar)(simple_strtoul(i2c_value, NULL, 16) & 0x000000ff);
						DRVfnAudio_AIC3104SendData(addr, value);
					}
				}
			}
			else if(c == 'd') {
				Aic3104_RegisterDump();
			}
			else if((c == '+') || (c == '=')) {
				AIC3104VolumeUpDown(1);
			}
			else if((c == '-') || (c == '_')) {
				AIC3104VolumeUpDown(0);
			}
			if(c != 'q') printf("Press 'q':Quit / r:Read / w:Write / d:Dump Reg. / '+,-':Volume)\n");
		}
		udelay(100000);
	}
	Audio_HW_Reset((GPIO_AIC_RSTn / 32), (GPIO_AIC_RSTn % 32));
	return ret;
}

int do_audio (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	int ret = 0;
	uchar addr = 0;
	uchar value = 0;
	int len = 1;
	u8 buff[AIC3X_CACHEREGNUM];
	if(strcmp(argv[1], "on") == 0){
		ret = Audio_test();
	}else if(strcmp(argv[1], "off") == 0){
		Audio_HW_Reset((GPIO_AIC_RSTn / 32), (GPIO_AIC_RSTn % 32));
	}else if(strcmp(argv[1], "dump") == 0){
		Aic3104_RegisterDump();
	}else if(strcmp(argv[1], "read") == 0){
		if (argc < 3) {
			printf ("Usage:\n%s\n", cmdtp->usage);
			return -1;
		}
		addr = (uchar)(simple_strtoul(argv[2], NULL, 16) & 0x000000ff);
		if (argc >= 4) {
			len = (uchar)(simple_strtoul(argv[3], NULL, 10));
		}
		DRVfnAudio_AIC3104RecvData(addr, buff, len);
		print_buffer(addr, buff, 1, len, 0);
	}else if(strcmp(argv[1], "write") == 0){
		if (argc < 4) {
			printf ("Usage:\n%s\n", cmdtp->usage);
			return -1;
		}
		addr = (uchar)(simple_strtoul(argv[2], NULL, 16) & 0x000000ff);
		value = (uchar)(simple_strtoul(argv[3], NULL, 16) & 0x000000ff);
		DRVfnAudio_AIC3104SendData(addr, value);
	}else if(strcmp(argv[1], "volume")){
		if (argc < 3) {
			printf ("Usage:\n%s\n", cmdtp->usage);
			return -1;
		}else{
			if(strcmp(argv[2], "up")){
				AIC3104VolumeUpDown(1);
			}else if(strcmp(argv[2], "down")){
				AIC3104VolumeUpDown(0);
			}else if(strcmp(argv[2], "set")){
				if (argc < 4) {
					printf ("Usage:\n%s\n", cmdtp->usage);
					return -1;
				}
				value = (uchar)(simple_strtoul(argv[3], NULL, 10));
				if((value & ~ROUTE_ON) < 118)
					DRVfnAudio_AIC3104SendData(PGAL_2_LLOPM_VOL, value);
			}else if(strcmp(argv[2], "get")){
				// Do nothing.
			}
			DRVfnAudio_AIC3104RecvData(PGAL_2_LLOPM_VOL, &value, 1);
			printf("volume=%u\n", value & ~ROUTE_ON);
		}
	}
	return ret;
}

U_BOOT_CMD(audio, 4, 0, do_audio,
	"Audio device test",
	"Audio device test\n\
	audio on/off - Bypass Test\n\
	audio dump - Dump registers\n\
	audio read [reg][len] - Read register\n\
	audio write [reg][val] - Write register\n\
	audio volume [up/down] - Set volume up/down\n\
	audio volume [get/set][val] - Get/Set volume"
);

static int addr_read_func(int parameter)
{
	char *args[5];
	char tmp[10];//ffffffff
	char startAddrStr[11];//0xffffffff
	char buffAddrStr[11];//0xffffffff
	char sizeStr[11];//0xffffffff
	cmd_tbl_t cmd_tmp;
	unsigned int startAddr, size;
	unsigned int* buff = NULL;

	memset(tmp, 0, sizeof(tmp));
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));

	if(get_line("Address(HEX):0x", tmp, sizeof(tmp), -1, str_number_hex, NULL, NULL) <= 0) return DIAG_ERROR;
	startAddr = (unsigned int)simple_strtoul(tmp, NULL, 16);

	if(startAddr < nand_total_size){
		if(get_line("Read size(HEX):0x", tmp, sizeof(tmp), -1, str_number_hex, NULL, NULL) <= 0) return DIAG_ERROR;
		printf("This is a NAND address...\n");
		size = (unsigned int)simple_strtoul(tmp, NULL, 16);
		buff = (unsigned int*)malloc(size);
		if(buff == NULL) return DIAG_ERROR;
		sprintf(buffAddrStr, "0x%08x", (uint)buff);
		sprintf(startAddrStr, "0x%08x", startAddr);
		sprintf(sizeStr, "0x%08x", size);
		args[0] = "nand";
		args[1] = "read";
		args[2] = buffAddrStr;
		args[3] = startAddrStr;
		args[4] = sizeStr;
		udelay(1000);
		if(do_nand(&cmd_tmp, 0, 5, args)){
			printf("Failed to read nand flash at %s.\n", startAddrStr);
			free(buff);
			return DIAG_ERROR;
		}
		print_buffer(startAddr, buff, 1, size, 0);
		free(buff);
	}else
	if(((startAddr >= PHYS_DRAM_1) && (startAddr < (PHYS_DRAM_1+PHYS_DRAM_1_SIZE)))
#ifdef PHYS_DRAM_2
		|| ((startAddr >= PHYS_DRAM_2) && (startAddr < (PHYS_DRAM_2+PHYS_DRAM_2_SIZE)))
#endif
	){
		if(get_line("Read size(HEX):0x", tmp, sizeof(tmp), -1, str_number_hex, NULL, NULL) <= 0) return DIAG_ERROR;
		printf("This is a SDRAM address...\n");
		size = (unsigned int)simple_strtoul(tmp, NULL, 16);
		print_buffer((ulong)startAddr, (void*)startAddr, 1, size, 0);
	}else{
		printf("Unknown address...\n");
		printf("[0x%08X]\n", __raw_readl(startAddr));
	}

	return DIAG_OK;
}

static int addr_write_func(int parameter)
{
	char *args[5];
	char tmp[10];//ffffffff
	char startAddrStr[11];//0xffffffff
	char buffAddrStr[11];//0xffffffff
	char valueStr[11];//0xffffffff
	char countStr[11];//0xffffffff
	char sizeStr[11];//0xffffffff
	cmd_tbl_t cmd_tmp;
	unsigned int startAddr = 0, buffAddr = 0, value = 0, size = 0, count = 0;

	memset(tmp, 0, sizeof(tmp));
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));

	if(get_line("Address(HEX):0x", tmp, sizeof(tmp), -1, str_number_hex, NULL, NULL) <= 0) return DIAG_ERROR;
	startAddr = (unsigned int)simple_strtoul(tmp, NULL, 16);

	if(startAddr < nand_total_size){
		printf("This is a NAND address...\n");
		if(get_line("Buffer address(HEX):0x", tmp, sizeof(tmp), -1, str_number_hex, NULL, NULL) <= 0) return DIAG_ERROR;
		buffAddr = (unsigned int)simple_strtoul(tmp, NULL, 16);
		printf("Write size(Pages, %d Bytes):", nand_info[nand_curr_device].writesize);
		if(get_line(NULL, tmp, sizeof(tmp), -1, str_number_hex, NULL, NULL) <= 0) return DIAG_ERROR;
		size = (unsigned int)simple_strtoul(tmp, NULL, 10);
		sprintf(buffAddrStr, "0x%08x", buffAddr);
		sprintf(startAddrStr, "0x%08x", startAddr);
		sprintf(sizeStr, "0x%08x", size * nand_info[nand_curr_device].writesize);
		args[0] = "nand";
		args[1] = "write";
		args[2] = buffAddrStr;
		args[3] = startAddrStr;
		args[4] = sizeStr;
		udelay(1000);
		if(do_nand(&cmd_tmp, 0, 5, args)){
			printf("Failed to write nand flash at %s.\n", startAddrStr);
			return DIAG_ERROR;
		}
		printf("Write 0x%08x to address 0x%08x done...\n", value, startAddr);
	}else
	if(((startAddr >= PHYS_DRAM_1) && (startAddr < (PHYS_DRAM_1+PHYS_DRAM_1_SIZE)))
#ifdef PHYS_DRAM_2
		|| ((startAddr >= PHYS_DRAM_2) && (startAddr < (PHYS_DRAM_2+PHYS_DRAM_2_SIZE)))
#endif
	){
		printf("This is a SDRAM address...\n");
		if(get_line("Value(HEX):0x", tmp, sizeof(tmp), -1, str_number_hex, NULL, NULL) <= 0) return DIAG_ERROR;
		value = (unsigned int)simple_strtoul(tmp, NULL, 16);
		if(get_line("Byte[1] / Word[2] / DWord[4]:", tmp, 3, -1, "124", NULL, NULL) <= 0) return DIAG_ERROR;
		size = (unsigned int)simple_strtoul(tmp, NULL, 10);
		if(get_line("Count(HEX):0x", tmp, sizeof(tmp), -1, str_number_hex, NULL, NULL) <= 0) return DIAG_ERROR;
		count = (unsigned int)simple_strtoul(tmp, NULL, 16);
		sprintf(startAddrStr, "0x%08x", startAddr);
		value = size==1 ? (value & 0x000000ff) : size==2 ? (value & 0x0000ffff) : value;
		sprintf(valueStr, "0x%x", value);
		sprintf(countStr, "0x%08x", count);
		args[0] = size==1 ? "mw.b" : size==2 ? "mw.w" : "mw.l";
		args[1] = startAddrStr;
		args[2] = valueStr;
		args[3] = countStr;
		if(do_mem_mw(&cmd_tmp, 0, 4, args)){
			printf("Failed to write SDRAM at %s.\n", startAddrStr);
			return DIAG_ERROR;
		}
		printf("Write 0x%x to address 0x%08x done...\n", value, startAddr);
	}else{
		printf("Unknown address...\n");
		if(get_line("Value(HEX):0x", tmp, sizeof(tmp), -1, str_number_hex, NULL, NULL) <= 0) return DIAG_ERROR;
		value = (unsigned int)simple_strtoul(tmp, NULL, 16);
		__raw_writel(value, startAddr);
	}

	return DIAG_OK;
}

int ping_ip(IPaddr_t ip)
{
	NetPingIP = ip;
	if(NetPingIP == 0) return -1;
	if(NetLoop(PING) < 0) return 0;
	else return 1;
	return 0;
}

static int ping_test_func(int parameter)
{
	char tmp[17];//xxx.xxx.xxx.xxx
	IPaddr_t ipAddr;

	strcpy(tmp, getenv("serverip"));
	printf("IP Address(%s):", tmp);
	if(get_line(NULL, tmp, sizeof(tmp), -1, str_ip, NULL, tmp) < 0) return DIAG_ERROR;

	ipAddr = string_to_ip(tmp);
	if(ipAddr == 0) {
		printf ("Bad IP number: %s\n", tmp);
		return DIAG_ERROR;
	}

	if(ping_ip(ipAddr) <= 0) {
		printf("ping failed; %s is not alive\n", tmp);
	}else{
		printf("%s is alive\n", tmp);
	}
	return DIAG_OK;
}

//Status Reader Menu======================================================
static DiagMenuTableStruct StatusReaderMenuTable[] = {
	{ '0',	"Reset Button",				GPIO_READ_RESET_BUTTON,		gpio_read_func,					NULL},
	{ '1',	"Light Sensor", 			GPIO_READ_LIGHT_SENSOR,		gpio_read_func,					NULL},
	{ '2',	"Light Sensor (I2C)",		0,							LightSensor_I2C_test_func,		NULL},
	{ '3',	"Thermal Sensor (I2C)",		0,							ThermalSensor_I2C_test_func,	NULL},
	{ '4',	"SD_EN state",				GPIO_READ_SD_En,			gpio_read_func, 				NULL},
	{ '5',	"SD_CDn state", 			GPIO_READ_SD_CDn,			gpio_read_func, 				NULL},
	{ '6',	"SD_WPn state", 			GPIO_READ_SD_WPn,			gpio_read_func, 				NULL},
	{ '7',	"Heater_cam_Int state", 	GPIO_READ_Heatercam_Int,	gpio_read_func, 				NULL},
	{ '8',	"Heater_sys_Int state", 	GPIO_READ_Heatersys_Int,	gpio_read_func, 				NULL},
	{ '9',	"Fan_Int state",			GPIO_READ_FAN_Int,			gpio_read_func, 				NULL},
#if CONFIG_SYS_I2C_EEPROM
	{ 'a',	"EEPROM dump",				0,							EEPROM_Dump, 				NULL},
#endif
};
static DiagMenuStruct StatusReaderMenu = {
	sizeof(StatusReaderMenuTable)/sizeof(DiagMenuTableStruct),
	StatusReaderMenuTable,
	"<<<Status Reader Menu>>>",
	0, NULL
};

//Show Info Menu======================================================
static int diag_get_Info(int parameter);

#define _PRINT_FIRST(s,i,j) \
do{ \
	printf("%s", s); \
	if(strlen(s)>40){ printf("\n"); j=1; } \
	else{ for(i=0; i<40-strlen(s); i++) printf(" "); j=0; } \
}while(0)

#define _PRINT_SECOND(s,j) \
do{ \
	if(!j && strlen(s)>40) printf("\n"); \
	printf("%s\n", s); \
}while(0)

void init_version_env(void)
{
	char tmp[80];
	int s = 0;

//#ifdef CONFIG_HW_VER
//	sprintf(tmp, "%s", MK_STR(CONFIG_HW_VER));
//	if(strcmp(tmp, getenv("hw_ver")) != 0){
//		setenv("hw_ver", tmp);
//		s = 1;
//	}
//#endif

#ifdef CONFIG_UBL_VER
	sprintf(tmp, "%s", MK_STR(CONFIG_UBL_VER));
	if(strcmp(tmp, getenv("ubl_ver")) != 0){
		setenv("ubl_ver", tmp);
		s = 1;
	}
#endif

#ifdef CONFIG_UBOOT_IDX
	sprintf(tmp, "%d", ubl_ubootIdx);
	if(strcmp(tmp, getenv("uboot_idx")) != 0){
		setenv("uboot_idx", tmp);
		s = 1;
	}
#endif

#ifdef CONFIG_UBOOT_VER
	sprintf(tmp, "%s", MK_STR(CONFIG_UBOOT_VER));
	if(strcmp(tmp, getenv("uboot_ver")) != 0){
		setenv("uboot_ver", tmp);
		s = 1;
	}
#endif

	if(s) saveenv();
}

static int diag_get_Info(int parameter)
{
	char tmp[80];
	int i,j;
	unsigned long Size = 0;

#ifdef CONFIG_UBOOT_IDX
	if (ubl_ubootIdx == 2)
	{
		printf("!!!! This is Second U-Boot. !!!!\n");
	}
#endif

#ifdef CONFIG_VERSION_VARIABLE
	extern char version_string[];
	printf("Ver 	   : %s\n", version_string);
#endif /* CONFIG_VERSION_VARIABLE */


#if CONFIG_SYS_I2C_EEPROM
	printf("Model name : %s\n", g_model_name);
#else
	printf("Model name : %s\n", getenv("dut_model"));
#endif

	printf("Host name  : %s\n", getenv("dut_host"));
	printf("Description: %s\n", getenv("dut_desc"));

#if defined(CONFIG_SPI_BOOT) || defined(CONFIG_SPI_ENV)
	sprintf(tmp, "Boot from  : %s", "SPI");
#elif defined(CONFIG_NAND_BOOT) || defined(CONFIG_NAND_ENV)
	sprintf(tmp, "Boot from  : %s", "NAND");
#elif defined(CONFIG_SD_BOOT) || defined(CONFIG_MMC_ENV)
	sprintf(tmp, "Boot from  : %s", "SD");
#elif defined(CONFIG_ETH_BOOT)
	sprintf(tmp, "Boot from  : %s", "ETH");
#endif
	_PRINT_FIRST(tmp, i, j);
	sprintf(tmp, "Kernel idx : %s", getenv("kernel_idx"));
	_PRINT_SECOND(tmp, j);

	sprintf(tmp, "DUT IP     : %s ", getenv("ipaddr"));
	_PRINT_FIRST(tmp, i, j);
	sprintf(tmp, "Server IP  : %s", getenv("serverip"));
	_PRINT_SECOND(tmp, j);

	sprintf(tmp, "Netmask    : %s ", getenv("netmask"));
	_PRINT_FIRST(tmp, i, j);
#if CONFIG_SYS_I2C_EEPROM
	sprintf(tmp, "MAC        : %s ", g_mac_addr);
#else
	sprintf(tmp, "MAC        : %s ", getenv("ethaddr"));
#endif
	_PRINT_SECOND(tmp, j);

#if CONFIG_SYS_I2C_EEPROM
	sprintf(tmp, "S/N        : %s ", g_serial_str);
#else
	sprintf(tmp, "S/N        : %s ", getenv("ser_num"));
#endif
	_PRINT_FIRST(tmp, i, j);
	sprintf(tmp, "DUT ID     : %s ", getenv("dut_id"));
	_PRINT_SECOND(tmp, j);

	sprintf(tmp, "FW Ver.    : %s ", getenv("fw_ver"));
	_PRINT_FIRST(tmp, i, j);
	sprintf(tmp, "MP Ver.    : %s ", getenv("mp_ver"));
	_PRINT_SECOND(tmp, j);
#if 1
	sprintf(tmp, "BIOS Ver.  : %s ", getenv("uboot_ver"));
	_PRINT_FIRST(tmp, i, j);
	sprintf(tmp, "HW Ver.    : %s ", getenv("hw_ver"));
	_PRINT_SECOND(tmp, j);
#else
	sprintf(tmp, "UBL Ver.   : %s ", getenv("ubl_ver"));
	_PRINT_FIRST(tmp, i, j);
	sprintf(tmp, "UBoot Ver. : %s ", getenv("uboot_ver"));
	_PRINT_SECOND(tmp, j);
#endif

#if CONFIG_SYS_I2C_EEPROM
	sprintf(tmp, "MP Flag    : 0x%03x ", g_mp_flag);
#else
	sprintf(tmp, "MP Flag    : %s ", getenv("mp_flag"));
#endif
	_PRINT_FIRST(tmp, i, j);
	sprintf(tmp, "Burn Time  : %s hr ", getenv("burntime"));
	_PRINT_SECOND(tmp, j);

#ifdef PHYS_DRAM_2_SIZE
	Size = PHYS_DRAM_1_SIZE + PHYS_DRAM_2_SIZE;
#else
	Size = PHYS_DRAM_1_SIZE;
#endif
	if(Size >= (1024 * 1024 * 1024)){
		sprintf(tmp, "DRAM size  : %lu GB ", Size / (1024 * 1024 * 1024));
	}else if(Size >= (1024 * 1024)){
		sprintf(tmp, "DRAM size  : %lu MB ", Size / (1024 * 1024));
	}else if(Size >= (1024)){
		sprintf(tmp, "DRAM size  : %lu KB ", Size / (1024));
	}else{
		sprintf(tmp, "DRAM size  : %lu Byte ", Size);
	}
	_PRINT_FIRST(tmp, i, j);
	if(nand_total_size >= (1024 * 1024 * 1024)){
		sprintf(tmp, "NAND size  : %lu GB (bus:%s)", nand_total_size / (1024 * 1024 * 1024), get_sysboot_bw()==0?"8-bit":"16-bit");
	}else if(nand_total_size >= (1024 * 1024)){
		sprintf(tmp, "NAND size  : %lu MB (bus:%s)", nand_total_size / (1024 * 1024), get_sysboot_bw()==0?"8-bit":"16-bit");
	}else if(nand_total_size >= (1024)){
		sprintf(tmp, "NAND size  : %lu KB (bus:%s)", nand_total_size / (1024), get_sysboot_bw()==0?"8-bit":"16-bit");
	}else{
		sprintf(tmp, "NAND size  : %lu Byte (bus:%s)", nand_total_size, get_sysboot_bw()==0?"8-bit":"16-bit");
	}
	_PRINT_SECOND(tmp, j);

	printf("%s\n",EqualLine);

	return DIAG_OK;
}

//Main Menu==========================================================
static int BootPreProcessor(int parameter);
#ifdef CMD_RUN_T0
static int RunT0(int parameter);
#endif
static int RunT1BIOS(int parameter);
static int LED_Set_Fault_Blink(int parameter);

//fw_recovery Menu==========================================================
static int ping_server(int parameter);
static int reset_FW_Recover(int parameter);
static int print_FW_Recover_Info(int parameter);
static DiagMenuTableStruct fw_recoveryMenuTable[] = {
	{'s',	"Setup download env",		0,	diag_setup_download, 	NULL},
	{'d',	"Download Firmware",		0,	diag_download_Firmware,	NULL},
	{'p',	"Ping server test", 		0,	ping_server,			NULL},
	{'r',	"Reset FW Recovery flag",	0,	reset_FW_Recover,		NULL},
};
static DiagMenuStruct fw_recoveryMenu = {
	sizeof(fw_recoveryMenuTable)/sizeof(DiagMenuTableStruct),
	fw_recoveryMenuTable,
	"<<<FW Recovery Menu>>>",
	0, print_FW_Recover_Info
};

static int ping_server(int parameter)
{
	IPaddr_t ipAddr;
	char *serverip;
	serverip = getenv("serverip");
	if(serverip){
		ipAddr = string_to_ip(serverip);
		if(ping_ip(ipAddr)){
			printf("\nServer %s is alive ...\n\n", serverip);
			return 1;
		}else{
			printf("\nServer %s is not alive ...\n\n", serverip);
			return 0;
		}
	}
	return 0;
}

static int reset_FW_Recover(int parameter)
{
	if(env_set_fw_recovery(1)){
		printf("Reset FW Recover flag Failed!!\n");
		return DIAG_ERROR;
	}else{
		printf("Reset FW Recover flag Successed.\n");
		return DIAG_OK;
	}
}

static int print_FW_Recover_Info(int parameter)
{
	printf("\n");
	printf(" !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	printf(" If you see this page, your system may be already corrupted.\n");
	printf(" This may be due to sudden power failure during the firmware updating.\n");
	printf(" Please prepare the correct firmware file and use the following method \n");
	printf(" to download and recover the system.\n");
	printf(" !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	printf("\n");
	printf("%s\n", EqualLine);
	return DIAG_OK;
}

#ifdef CMD_RUN_T0
static int RunT0(int parameter)
{
	char* ptr = NULL;
	int val = 0;

	////T0
	setenv("quiet", "1");

	//Step 1: Set Model name
	if (MM_Set_Model(0) <= 0)
	{
		printf("Set Model failed\n");
		return DIAG_ERROR;
	}

#if CONFIG_SYS_I2C_EEPROM

	//char model_name[EEPROM_DATA_MODELNAME_LEN + 1] = {0};

	if (EEPROM_GetModelName(&g_model_name))
	{
		printf("Get Model from EEPROM failed\n");
		return DIAG_ERROR;
	}

#else

	ptr = NULL;
	if (env_get_model(&ptr))
	{
		printf("Get Model from FLASH failed\n");
		return DIAG_ERROR;
	}

	if (!ptr)
	{
		printf("Get Model from FLASH failed\n");
		return DIAG_ERROR;
	}

#endif

	printf("\n[%s]\n\n", ptr);
	Sleep(1);

	//Step 2: Set Host name
	if(MM_Set_Host(0) <= 0) return DIAG_ERROR;
	ptr = NULL;
	if(env_get_host(&ptr)) return DIAG_ERROR;
	if(!ptr) return DIAG_ERROR;
	printf("\n[%s]\n\n", ptr);
	Sleep(1);

	//Step 3: Set Description
	if(MM_Set_Desc(0) <= 0) return DIAG_ERROR;
	ptr = NULL;
	if(env_get_desc(&ptr)) return DIAG_ERROR;
	if(!ptr) return DIAG_ERROR;
	printf("\n[%s]\n\n", ptr);
	Sleep(1);

	//Step 4: Set IVA Flag
	if(MM_Set_IVA_Flag(0) <= 0 ) return DIAG_ERROR;
	val = 0;
	if(env_get_iva_flag(&val)) return DIAG_ERROR;
	printf("\n[0x%02x]\n\n", val);
	Sleep(1);

	//Step 5: Set MP FLag as T1-BIOS
	printf("Set FLAG_T1_BIOS\n");

	if (Set_MP_Flag(CONFIG_MP_FLAG_T1_BIOS))
	{
		printf("Set MP Flag Failed!!\n");
		return DIAG_ERROR;
	}
	printf("\n[Set FLAG_T1_BIOS Passed.]\n\n");

	Sleep(1);

	setenv("quiet", "0");

	return DIAG_OK;
}

int do_RunT0 (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	printf("\n\n");
	RunT0(0);
	printf("\n\n");
	return 0;
}

U_BOOT_CMD(
	T0, 1, 0,	do_RunT0,
	"Run T0",
	NULL
);
#endif

void LED_All_ON()
{
#ifdef GPIO_LED_STATE
	GPIO_Out(GPIO_LED_STATE, GPIO_LED_ON);
#endif
#ifdef GPIO_LED_SYS
	GPIO_Out(GPIO_LED_SYS, GPIO_LED_ON);
#endif
#ifdef GPIO_LED_SD
	GPIO_Out(GPIO_LED_SD, GPIO_LED_ON);
#endif
#ifdef GPIO_LED_PTZ
	GPIO_Out(GPIO_LED_PTZ, GPIO_LED_ON);
#endif
#ifdef GPIO_LED_V1
	GPIO_Out(GPIO_LED_V1, GPIO_LED_ON);
#endif
#ifdef GPIO_LED_V2
	GPIO_Out(GPIO_LED_V2, GPIO_LED_ON);
#endif
#ifdef GPIO_LED_V3
	GPIO_Out(GPIO_LED_V3, GPIO_LED_ON);
#endif
#ifdef GPIO_LED_V4
	GPIO_Out(GPIO_LED_V4, GPIO_LED_ON);
#endif
#ifdef GPIO_LED_VIDEO
	GPIO_Out(GPIO_LED_VIDEO, GPIO_LED_ON);
#endif
#ifdef GPIO_LED_STAT_G
	GPIO_Out(GPIO_LED_STAT_G, GPIO_LED_ON);
#endif
#ifdef GPIO_LED_STAT_R
	GPIO_Out(GPIO_LED_STAT_R, GPIO_LED_OFF);
#endif
#ifdef GPIO_LED_FAIL
	GPIO_Out(GPIO_LED_FAIL, GPIO_LED_ON);
#endif
}

void LED_All_OFF()
{
#ifdef GPIO_LED_STATE
	GPIO_Out(GPIO_LED_STATE, GPIO_LED_OFF);
#endif
#ifdef GPIO_LED_SYS
	GPIO_Out(GPIO_LED_SYS, GPIO_LED_OFF);
#endif
#ifdef GPIO_LED_SD
	GPIO_Out(GPIO_LED_SD, GPIO_LED_OFF);
#endif
#ifdef GPIO_LED_V1
	GPIO_Out(GPIO_LED_V1, GPIO_LED_OFF);
#endif
#ifdef GPIO_LED_V2
	GPIO_Out(GPIO_LED_V2, GPIO_LED_OFF);
#endif
#ifdef GPIO_LED_V3
	GPIO_Out(GPIO_LED_V3, GPIO_LED_OFF);
#endif
#ifdef GPIO_LED_V4
	GPIO_Out(GPIO_LED_V4, GPIO_LED_OFF);
#endif
#ifdef GPIO_LED_PTZ
	GPIO_Out(GPIO_LED_PTZ, GPIO_LED_OFF);
#endif
#ifdef GPIO_LED_VIDEO
	GPIO_Out(GPIO_LED_VIDEO, GPIO_LED_OFF);
#endif
#ifdef GPIO_LED_STAT_G
	GPIO_Out(GPIO_LED_STAT_G, GPIO_LED_OFF);
#endif
#ifdef GPIO_LED_STAT_R
	GPIO_Out(GPIO_LED_STAT_R, GPIO_LED_OFF);
#endif
#ifdef GPIO_LED_FAIL
	GPIO_Out(GPIO_LED_FAIL, GPIO_LED_OFF);
#endif
}

/*
mii write 1 1c b40f
mii write 2 1c b40f
*/
int setLinkSpeedLed()
{
	unsigned char	addr, reg;
	unsigned short	data;
	int		rcode = 0;
	char		*devname;
#if defined(CONFIG_MII_INIT)
	mii_init ();
#endif
	/* use current device */
	devname = miiphy_get_current_dev();
	if(devname == NULL){
		printf("Error miiphy_get_current_dev!\n");
		return -1;
	}
	addr = 1;
	reg = 0x1c;
	data = 0xb40f;
	if (miiphy_write (devname, addr, reg, data) != 0) {
		printf("Error writing to the PHY addr=%02x reg=%02x\n", addr, reg);
		rcode = 1;
	}
	addr = 2;
	reg = 0x1c;
	data = 0xb40f;
	if (miiphy_write (devname, addr, reg, data) != 0) {
		printf("Error writing to the PHY addr=%02x reg=%02x\n", addr, reg);
		rcode = 1;
	}
	return rcode;
}

void wait_reset_button()
{
#if defined(TEST_LED_BLINK_ON_T1BIOS) && defined(GPIO_LED_STAT_G) && defined(GPIO_LED_STAT_R)
	GPIO_Out(GPIO_LED_STAT_G, GPIO_LED_ON);
	GPIO_Out(GPIO_LED_STAT_R, GPIO_LED_OFF);
#endif

#if defined(TEST_LINKSP_LED_BLINK_ON_T1BIOS) && defined(GPIO_LINKSP_0) && defined(GPIO_LINKSP_1)
	GPIO_Out(GPIO_LINKSP_0, 0);
	GPIO_Out(GPIO_LINKSP_1, 0);
	setLinkSpeedLed();
#endif

	while(!resetkey_state())
	{
#if defined(TEST_LED_BLINK_ON_T1BIOS) && defined(GPIO_LED_STAT_G) && defined(GPIO_LED_STAT_R)
		LED_Reverse(GPIO_LED_STAT_G);
		LED_Reverse(GPIO_LED_STAT_R);
#endif
#if defined(TEST_LINKSP_LED_BLINK_ON_T1BIOS) && defined(GPIO_LINKSP_0) && defined(GPIO_LINKSP_1)
		LED_Reverse(GPIO_LINKSP_0);
		LED_Reverse(GPIO_LINKSP_1);
#endif
		udelay(500000);
	}
}

static int RunT1BIOS(int parameter)
{
	////T1-BIOS

	//Step 1: Test Led
	LED_All_ON();
	printf("Check All LED were light.\n");
	Sleep(1);

	//Step 2: Test Reset Button
	printf("Press reset button for detection.\n");
	wait_reset_button();
	printf("\n[Button detected.]\n\n");
	LED_All_OFF();
	Sleep(1);

#if defined(TEST_LIGHTSENSOR_ON_T1BIOS)
	//Step 3: Test Light Sensor
	LightSensor_test_func(0);
#endif

	//Step 4: Test DDR
	if(mem_test(0x200000, 0x00000000, CFG_MEMTEST_START, 0)) return DIAG_ERROR;
	if(mem_test(0x200000, 0xffffffff, CFG_MEMTEST_START, 0)) return DIAG_ERROR;
	if(mem_test(0x200000, 0x0000ffff, CFG_MEMTEST_START, 0)) return DIAG_ERROR;
	if(mem_test(0x200000, 0xffff0000, CFG_MEMTEST_START, 0)) return DIAG_ERROR;
	if(mem_test(0x200000, 0x55555555, CFG_MEMTEST_START, 0)) return DIAG_ERROR;
	if(mem_test(0x200000, 0xaaaaaaaa, CFG_MEMTEST_START, 0)) return DIAG_ERROR;
	if(mem_test(0x200000, 0x5555aaaa, CFG_MEMTEST_START, 0)) return DIAG_ERROR;
	if(mem_test(0x200000, 0xaaaa5555, CFG_MEMTEST_START, 0)) return DIAG_ERROR;
#if defined(PHYS_DRAM_2)
	if(mem_test(0x200000, 0x00000000, PHYS_DRAM_2, 0)) return DIAG_ERROR;
	if(mem_test(0x200000, 0xffffffff, PHYS_DRAM_2, 0)) return DIAG_ERROR;
	if(mem_test(0x200000, 0x0000ffff, PHYS_DRAM_2, 0)) return DIAG_ERROR;
	if(mem_test(0x200000, 0xffff0000, PHYS_DRAM_2, 0)) return DIAG_ERROR;
	if(mem_test(0x200000, 0x55555555, PHYS_DRAM_2, 0)) return DIAG_ERROR;
	if(mem_test(0x200000, 0xaaaaaaaa, PHYS_DRAM_2, 0)) return DIAG_ERROR;
	if(mem_test(0x200000, 0x5555aaaa, PHYS_DRAM_2, 0)) return DIAG_ERROR;
	if(mem_test(0x200000, 0xaaaa5555, PHYS_DRAM_2, 0)) return DIAG_ERROR;
#endif
	printf("\n[DDR Test Passed.]\n\n");
	Sleep(1);

	//Step 5: Set MP FLag as T1-MP
	printf("Set FLAG_T1_MP\n");
	if(Set_MP_Flag(CONFIG_MP_FLAG_T1_MP)) return DIAG_ERROR;
	printf("\n[Set FLAG_T1_MP Passed.]\n\n");
	Sleep(1);

#ifdef CONFIG_HW_WATCHDOG
	//Step 6: Watchdog reboot system
	printf("DUT will reboot in %d secs.\n", WDT_TIMEOUT_SEC);
	hw_watchdog_op(HWWD_ON);
	hw_watchdog_op(HWWD_FORCE_RST);
	Sleep(5);
#endif

	//if run to here, watchdog test failed.
	return DIAG_ERROR;
}

int do_RunT1BIOS (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	printf("\n\n");
	RunT1BIOS(0);
	printf("\n\n");
	return 0;
}

U_BOOT_CMD(
	T1bios, 1, 0,	do_RunT1BIOS,
	"Run T1-BIOS",
	NULL
);

static int LED_Set_Fault_Blink(int parameter)
{
	sys_led_R_ON();
	sys_led_G_OFF();
	while(1){
		if(tstc()){ getc(); break; }
		sys_led_R_reverse();
		udelay(500000);
	}
	return DIAG_OK;
}

#ifdef CONFIG_FW_BOOTUP_COUNTER
static int check_fw_bootup_counter()
{
	unsigned int val = 0;
	int idx = 0, en = 0;
	if(env_get_fw_bootup_counter(&val)){
		printf("Get FW Bootup Counter Failed!!\n");
		if(env_set_fw_bootup_counter(CONFIG_FW_BOOTUP_COUNTER)){
			printf("Set Bootup Counter(%u) Failed!!\n", CONFIG_FW_BOOTUP_COUNTER);
			return DIAG_ERROR;
		}
		return DIAG_ERROR;
	}else{
#ifdef CONFIG_KERNELX_REDUND
		if(env_get_kernel_redund(&en)){
			printf("Get Kernel redund Failed\n");
			en = CONFIG_KERNELX_REDUND;
		}
		if(en){
			if(val > MAX_FW_BOOTUP_COUNTER){
				printf("\n");
				printf(" !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
				printf("\n  Boot times(%u) exceeds the default(%d), switch to the backup system.\n\n", val, MAX_FW_BOOTUP_COUNTER);
				printf(" !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
				printf("\n");
				if(env_get_kernel_idx(&idx)){
					printf("Get Kernel index Failed\n");
					idx = CONFIG_KERNEL_IDX;
				}
				if(idx == CONFIG_KERNEL_IDX) idx = CONFIG_KERNEL2_IDX;
				else idx = CONFIG_KERNEL_IDX;
				if(env_set_kernel_idx(idx) < 0){
					printf("Set Kernel index(%d) Failed\n", idx);
					return DIAG_ERROR;
				}
				val = 0;
				if(env_set_fw_bootup_counter(val)){
					printf("Set Bootup Counter(%u) Failed!!\n", val);
					return DIAG_ERROR;
				}
			}
		}
#endif
		printf("FW Bootup Counter: %u\n", val);


#if defined(CONFIG_RTC_MCP7941X)
#if defined(VPORT06EC_2V)
		if (rtc_get_vbat_bit() != 1 )
		{
		val++;
		}
#endif
#else
		val++;
#endif

		if (env_set_fw_bootup_counter(val))
		{
			printf("Set Bootup Counter(%u) Failed!!\n", val);
			return DIAG_ERROR;
		}
		return DIAG_OK;
	}
	return DIAG_OK;
}
#endif

#ifdef CONFIG_SYS_ON_TEMP
static int check_thermal()
{
	int ret = 0;
	int value = 0;
	int pi_on_temp = CONFIG_PI_ON_TEMP;
	int pi_off_temp = CONFIG_PI_OFF_TEMP;
	int sys_on_temp = CONFIG_SYS_ON_TEMP;
	bool pi_on = false;


#ifdef CONFIG_SYS_ON_TIMEOUT
	reset_timer();
	ulong start = get_timer(0);
	ulong count, countBak = -1;
	unsigned int sys_on_timeout = CONFIG_SYS_ON_TIMEOUT;
	env_get_sys_on_timeout(&sys_on_timeout);
#endif

	if (i2c_probe(THERMAL_SENSOR_ADDR) == 0){
		env_get_pi_on_temp(&pi_on_temp);
		env_get_pi_off_temp(&pi_off_temp);
		env_get_sys_on_temp(&sys_on_temp);
		while(1){
			value = (int)ThermalSensor_I2C_state(THERMAL_SENSOR_ADDR);
			if(t_temp(value) < t_temp(pi_on_temp)){
				if(!pi_on){
					pi_on = true;
					Set_PI_OFF_TEMP_By_OFFS(t_temp(value));
					Set_SYS_ON_TEMP_By_OFFS(t_temp(value));
					env_get_pi_off_temp(&pi_off_temp);
					env_get_sys_on_temp(&sys_on_temp);
					printf("\nCheck System temperature.(PI_ON:%dC, PI_OFF:%dC, SYS_ON:%dC)\n", t_temp(pi_on_temp), t_temp(pi_off_temp), t_temp(sys_on_temp));
					Heater_Cam_ON();
					sys_led_R_ON();
				}
			}else{
				if(!pi_on) break;
			}

			if(t_temp(value) >= t_temp(pi_off_temp)){
				if(pi_on){
					pi_on = false;
					Heater_Cam_OFF();
					sys_led_R_OFF();
				}
			}
#ifdef CONFIG_SYS_ON_TIMEOUT
			if((count = get_timer(start)) < (sys_on_timeout * 1000)) {
				printf(" (%u)", count / 1000);
			}else{
				/* time out */
				printf("\nTime out!!\n");
				break;
			}
#endif
			if(t_temp(value) >= t_temp(sys_on_temp)){
				break;
			}
			printf(" System temperature -> %dC %s                   \r", t_temp(value), pi_on?"PI_ON":"PI_OFF");
			udelay(100000);
		}
		sys_led_R_OFF();
		printf("\nBoot System. (%dC)\n", t_temp(value));
	}
	return ret;
}
#endif

static int BootPreProcessor(int parameter)
{
	char *s;
	int rs = DIAG_OK;
	if(Get_MP_Flag(&rs) < 0){
		printf("Get MP Flag Fail\n");
	}
#ifdef CONFIG_SYS_ON_TEMP
	check_thermal();
#endif
#ifdef CONFIG_HW_WATCHDOG
	s = getenv("bootwdon");
	if(s && (*s == '0')){
		printf("\nDisable HW Watchdog.\n");
		hw_watchdog_op(HWWD_OFF);
	}else{
		/* Bobby-20150604 : move to evm.c */
		//printf("\nEnable HW Watchdog.\n");
		//hw_watchdog_op(HWWD_ON);
		hw_watchdog_op(HWWD_TRIGGER_SKIP);
	}
#endif
#ifdef TEST_SWITCH_MPFLAG
	switch(rs){
		case CONFIG_MP_FLAG_T0_BIOS:
			Set_MP_Flag(CONFIG_MP_FLAG_T1_BIOS);
			break;
		case CONFIG_MP_FLAG_T1_BIOS:
			Set_MP_Flag(CONFIG_MP_FLAG_T1_MP);
			break;
		case CONFIG_MP_FLAG_T1_MP:
			Set_MP_Flag(CONFIG_MP_FLAG_T2_MP);
			break;
		case CONFIG_MP_FLAG_T2_MP:
			Set_MP_Flag(CONFIG_MP_FLAG_T2_MP_ERR);
			break;
		case CONFIG_MP_FLAG_T2_MP_ERR:
			Set_MP_Flag(CONFIG_MP_FLAG_T2_T_MP);
			break;
		case CONFIG_MP_FLAG_T2_T_MP:
			Set_MP_Flag(CONFIG_MP_FLAG_T2_T_MP_ERR);
			break;
		case CONFIG_MP_FLAG_T2_T_MP_ERR:
			Set_MP_Flag(CONFIG_MP_FLAG_T3_BIOS);
			break;
		case CONFIG_MP_FLAG_T3_BIOS:
			Set_MP_Flag(CONFIG_MP_FLAG_FIRMWARE);
			break;
		case CONFIG_MP_FLAG_FIRMWARE:
			Set_MP_Flag(CONFIG_MP_FLAG_T0_BIOS);
			break;
	}
#endif

	switch(rs){
		case CONFIG_MP_FLAG_T0_BIOS:
		case CONFIG_MP_FLAG_T1_BIOS:
		case CONFIG_MP_FLAG_T1_MP:
		case CONFIG_MP_FLAG_T2_MP:
		case CONFIG_MP_FLAG_T2_MP_ERR:
		case CONFIG_MP_FLAG_T2_T_MP:
		case CONFIG_MP_FLAG_T2_T_MP_ERR:
		case CONFIG_MP_FLAG_T3_BIOS:
			setenv("mtd_idx", MK_STR(CONFIG_MTD_IDX_MP));
			s = getenv("mpbootcmd");
			if(s){
				run_command (s, 0);
			}else printf("\nFailed to getenv 'mpbootcmd'!!\n");
			break;
		case CONFIG_MP_FLAG_FIRMWARE:
		default:
#ifdef CONFIG_FW_BOOTUP_COUNTER
			check_fw_bootup_counter();
#endif
			s = getenv("bootcmd");
			if(s) run_command (s, 0);
			else printf("\nFailed to getenv 'bootcmd'!!\n");
			break;
	}
	Heater_Sys_OFF();

	LED_Set_Fault_Blink(0);
	sys_led_R_ON();
	sys_led_G_OFF();

	while(1) Sleep(1); // will never return.

	return DIAG_OK;
}

static void printf_title(char *title)
{
	char str[128];
	int	i, j, k;

	i = strlen(EqualLine);
	j = strlen(title);
	printf("\n");
	if ( i <= j ) {
		printf("%s", title);
		return;
	}
	sprintf(str, "%s\n", EqualLine);
	for ( k=((i/2)-(j/2)); k<i && *title; k++ )
		str[k] = *title++;
	printf("%s",str);
}

static void printf_menu(pDiagMenuStruct menu, bool showBack2Root)
{
	int i,j,k;
	char res[32];
	res[0] = '\0';
	printf_title(menu->Title);
	if(menu->Func) menu->Func(menu->Parameter);
	for(i=0; i<menu->TableNo; i++){
		if( i&1 ){
			for ( k=0; k<j; k++ ) printf(" ");
			printf("[%c] %s\n", menu->MenuTable[i].index, menu->MenuTable[i].Description);
		}else{
			printf("[%c] %s", menu->MenuTable[i].index, menu->MenuTable[i].Description);
			j = (40 - strlen(menu->MenuTable[i].Description) - 4);
		}
	}
	if(showBack2Root == true)
		printf("\n[q] Quit this menu.                     [z] Back to Main Menu");
	else
		printf("\n[q] Quit this menu.");
	printf("\n%s", EqualLine);
	printf("\nPlease input the select : ");
}

void menu_process(pDiagMenuStruct menu, bool *back2Root, int *layer)
{
	bool prompt = true;
	int i;
	int res;
	char tmp[3];
	bool back2Root_en = ((back2Root != NULL) && (layer != NULL) && ((*layer) > 0)) ? true : false;

	while ( 1 )
	{
		if( prompt ){
			printf_menu(menu, back2Root_en);
			prompt = false;
		}
		res = get_line(NULL, tmp, sizeof(tmp), -1, str_menu_operate, NULL, NULL);
		if( res == _GETLINE_TIMEOUT )
			continue;

		if( tmp[0] == 'q' || tmp[0] == 'Q' )
			break;

		if( (back2Root_en == true) && (tmp[0] == 'z' || tmp[0] == 'Z') ){
			if(back2Root != NULL) *back2Root = true;
			break;
		}else{
			if(back2Root != NULL) *back2Root = false;
		}

		for(i=0; i<menu->TableNo; i++)
		{
			if(menu->MenuTable[i].index == tmp[0])
			{
				if ( menu->MenuTable[i].SubMenu ){
					if(layer != NULL) (*layer)++;
					menu_process(menu->MenuTable[i].SubMenu, back2Root, layer);
					if((layer != NULL) && ((*layer) > 0)) (*layer)--;
					if((back2Root != NULL) && (*back2Root == true)){
						if((*layer) > 0) return;
						else break;
					}
				}else if ( menu->MenuTable[i].Func ){
					menu->MenuTable[i].Func(menu->MenuTable[i].Parameter);
				}else{
					printf("\nNOT IMPLEMENTED!!\n");
				}
				break;
			}
		}
		prompt = true;
		continue;
	}
}

#ifdef CONFIG_FW_RECOVERY
int auto_fw_recovery_process(void)
{
	uint64_t endtime;
	int i = 0;
	char tmp[16];//xxx.xxx.xxx.xxx
	char *args[3];
	cmd_tbl_t cmd_tmp;
	memset(&cmd_tmp, 0, sizeof(cmd_tbl_t));
	args[0] = "loadfw";
	args[1] = "tftp";
	args[2] = getenv("fw_rcvrfile");
	if(args[2] == NULL) args[2] = CONFIG_FW_RECOVERY_FILE;
	sprintf(tmp, "%s", MK_STR(CONFIG_SERVERIP));
	env_set_serverip(tmp);
	printf("\n");
	while(1){
		sys_led_RG_OFF();
		i = 3 * 100;
		endtime = _endtick(3);
		while(get_ticks() < endtime){
			if(tstc()){
				getc();
				printf("Abort\n");
				goto auto_fw_recovery_process_exit;
			}
			if(i%100 == 0) printf(" Start Auto Firmware Recovery after %d sec.(Press any key to abort)\r", i/100);
			if(i%10 == 0) sys_led_RG_reverse();
			if(i) i--;
			udelay(10000);
		}
		printf("\n");
		if(ping_server(0)){
			if(do_loadfw(&cmd_tmp, 0, 3, args)){
				printf("Download file %s failed.\n", args[2]);
			}else{
				if(*getenv("fw_rcvr") == '0') env_set_fw_recovery(1);
				goto auto_fw_recovery_process_exit;
			}
		}
	}
auto_fw_recovery_process_exit:
	return 0;
}
#endif

#define BACKDOOR_NONE 0
#define BACKDOOR_MMMENU 1
#define BACKDOOR_FWRCVR 2
#define BACKDOOR_AUTOFR 3
#define BACKDOOR_MMCMDL 4
int backdoor_process(void)
{
	int i;
	uint64_t endtime;
	char c = 0;
	char seq[6] = {'\0','\0','\0','\0','\0','\0'};//MMmoxa

	for(i = 0; i < sizeof(seq); ) {
		endtime = _endtick(BACKDOOR_TIMEOUT);
		while(!tstc()){
			WATCHDOG_RESET();
			if(get_ticks() > endtime){
				i = sizeof(seq);				/* exit loop */
				break;
			}
#ifdef CONFIG_FW_RECOVERY
		/* Remove (20150121-Bobby) */
		//	if(resetkey_state()){
		//		udelay(500000);
		//		if(!resetkey_state()) return BACKDOOR_FWRCVR;
		//		//else return BACKDOOR_NONE;
		//	}
		/* Remove (20150121-Bobby) */
#endif
		}
		if(i == sizeof(seq)) break;
		c = getc();
		switch(c){
			case '\r':				/* Enter		*/
			case '\n':
			case '\0':				/* nul			*/
			case 0x03:				/* ^C - break		*/
				c = '\0';			/* discard input */
				seq[i] = c;
				i = sizeof(seq);				/* exit loop */
				break;
			case 0x08:				/* ^H  - backspace	*/
			case 0x7F:				/* DEL - backspace	*/
				if(i>0){
					i--;
					puts (erase_seq);
				}
				break;
			default:
				if(c >= 0x20 && c <= 0x7E){
					putc(c);
					seq[i] = c;
					i++;
				}
				break;
		}
    }
	printf("\n");
	if(seq[0]=='M' && seq[1]=='M' && seq[2]=='m' && seq[3]=='o' && seq[4]=='x' && seq[5]=='a'){
		return BACKDOOR_MMMENU;
	}else
	if(seq[0]=='M' && seq[1]=='M' && seq[2]=='c' && seq[3]=='m' && seq[4]=='d' && seq[5]=='l'){
		return BACKDOOR_MMCMDL;
	}else{
#ifdef CONFIG_FW_RECOVERY
		if(*getenv("fw_rcvr") == '0') return BACKDOOR_AUTOFR;
#endif
		return BACKDOOR_NONE;
	}
}

// T0 Flag =================================================
static DiagMenuTableStruct T0FlagMenuTable[] = {
	{ '0',	"T0 Flag",	CONFIG_MP_FLAG_T0_BIOS,	MM_Set_MP_Flag, NULL},
};
static DiagMenuStruct T0FlagMenu = {
	sizeof(T0FlagMenuTable)/sizeof(DiagMenuTableStruct),
	T0FlagMenuTable,
	"<<<T0 Flag Menu>>>",
	0, NULL
};

// T1 Flag =================================================
static DiagMenuTableStruct T1FlagMenuTable[] = {
	{ '0',	"T1(BIOS)Flag",	CONFIG_MP_FLAG_T1_BIOS,	MM_Set_MP_Flag, NULL},
	{ '1',	"T1-1(MP)Flag",	CONFIG_MP_FLAG_T1_MP,	MM_Set_MP_Flag, NULL},
};
static DiagMenuStruct T1FlagMenu = {
	sizeof(T1FlagMenuTable)/sizeof(DiagMenuTableStruct),
	T1FlagMenuTable,
	"<<<T1 Flag Menu>>>",
	0, NULL
};

// T2 Flag =================================================
static DiagMenuTableStruct T2FlagMenuTable[] = {
	{ '0',	"T2(Burn in)Flag",	CONFIG_MP_FLAG_T2_MP,		MM_Set_MP_Flag, NULL},
	{ '1',	"T2-1(Dump)Flag",	CONFIG_MP_FLAG_T2_MP_ERR,	MM_Set_MP_Flag, NULL},
};
static DiagMenuStruct T2FlagMenu = {
	sizeof(T2FlagMenuTable)/sizeof(DiagMenuTableStruct),
	T2FlagMenuTable,
	"<<<T2 Flag Menu>>>",
	0, NULL
};

// T3 Flag =================================================
static DiagMenuTableStruct T3FlagMenuTable[] = {
	{ '0',	"T3 Flag",	CONFIG_MP_FLAG_T3_BIOS,	MM_Set_MP_Flag,	NULL},
};
static DiagMenuStruct T3FlagMenu = {
	sizeof(T3FlagMenuTable)/sizeof(DiagMenuTableStruct),
	T3FlagMenuTable,
	"<<<T3 Flag Menu>>>",
	0, NULL
};

// EOT Flag =================================================
static DiagMenuTableStruct EOTFlagMenuTable[] = {
	{ '0',	"EOT(Burn in)Flag",	CONFIG_MP_FLAG_T2_T_MP,		MM_Set_MP_Flag,	NULL},
	{ '1',	"EOT-1(Dump)Flag",	CONFIG_MP_FLAG_T2_T_MP_ERR,	MM_Set_MP_Flag,	NULL},
};
static DiagMenuStruct EOTFlagMenu = {
	sizeof(EOTFlagMenuTable)/sizeof(DiagMenuTableStruct),
	EOTFlagMenuTable,
	"<<<EOT Flag Menu>>>",
	0, NULL
};

// Module Flag =================================================
static DiagMenuTableStruct ModuleFlagMenuTable[] = {
	{ '0',	"Module T0",	0,	NULL, 	NULL},
	{ '1',	"Module T1",	0,	NULL,	NULL},
};
static DiagMenuStruct ModuleFlagMenu = {
	sizeof(ModuleFlagMenuTable)/sizeof(DiagMenuTableStruct),
	ModuleFlagMenuTable,
	"<<<Module Flag Menu>>>",
	0, NULL
};

// Set/Clear ALL =================================================
static DiagMenuTableStruct ALLFlagMenuTable[] = {
	{ '0',	"Clear ALL",	CONFIG_MP_FLAG_T0_BIOS,		MM_Set_MP_Flag, NULL},
	{ '1',	"Set ALL",		CONFIG_MP_FLAG_FIRMWARE,	MM_Set_MP_Flag, NULL},
	{ '2',	"View ALL",		0,							MM_Get_MP_Flag, NULL},
	{ '3',	"Manual Set",	0,							MM_Set_MP_Flag_Manual, NULL}
};
static DiagMenuStruct ALLFlagMenu = {
	sizeof(ALLFlagMenuTable)/sizeof(DiagMenuTableStruct),
	ALLFlagMenuTable,
	"<<<Set/Clear ALL Menu>>>",
	0, NULL
};

// SET/CLEAR FLAG Menu =================================================
static DiagMenuTableStruct NewMPFlagMenuTable[] = {
	{ '0',	"T0 Flag",			0,	NULL,	&T0FlagMenu},
	{ '1',	"T1 Flag",			0,	NULL,	&T1FlagMenu},
	{ '2',	"T2 Flag",			0,	NULL,	&T2FlagMenu},
	{ '3',	"T3 Flag",			0,	NULL,	&T3FlagMenu},
	{ '4',	"EOT Flag",			0,	NULL,	&EOTFlagMenu},
	{ '5',	"Module Flag",		0,	NULL,	&ModuleFlagMenu},
	{ 'a',	"Set/Clear ALL",	0,	NULL,	&ALLFlagMenu},
};
static DiagMenuStruct NewMPFlagMenu = {
	sizeof(NewMPFlagMenuTable)/sizeof(DiagMenuTableStruct),
	NewMPFlagMenuTable,
	"<<<Set/Clear Menu>>>",
	0, NULL
};

#ifdef CONFIG_MTD_IDX
static int diag_set_mtd_index(int parameter)
{
	int idx;
	char tmp[4];
	env_get_mtd_idx(&idx);
	printf("MTD index(%d):", idx);
	if(get_line(NULL, tmp, sizeof(tmp), -1, str_number_dec, NULL, NULL) > 0 ){
		idx = (int)simple_strtoul(tmp, NULL, 10);
		if(env_set_mtd_idx(idx) < 0){
			printf("Set MTD index Fail\n");
			return DIAG_ERROR;
		}
	}
	return DIAG_OK;
}
#endif

#ifdef CONFIG_KERNELX_REDUND
static int diag_set_kernel_redund(int parameter)
{
	int en;
	char tmp[4];
	env_get_kernel_redund(&en);
	printf("Kernel redund(0:Off/1:On)(%d):", en);
	if(get_line(NULL, tmp, sizeof(tmp), -1, "01", NULL, NULL) > 0 ){
		en = (int)simple_strtoul(tmp, NULL, 10);
	}
	if(env_set_kernel_redund(en) < 0){
		printf("Set Kernel redund Fail\n");
		return DIAG_ERROR;
	}
	return DIAG_OK;
}
#endif

#ifdef CONFIG_KERNEL_IDX
static int diag_set_kernel_index(int parameter)
{
	int idx;
	char tmp[4];
	env_get_kernel_idx(&idx);
	printf("Kernel index(%d):", idx);
	if(get_line(NULL, tmp, sizeof(tmp), -1, str_number_dec, NULL, NULL) > 0 ){
		idx = (int)simple_strtoul(tmp, NULL, 10);
	}
	if(env_set_kernel_idx(idx) < 0){
		printf("Set Kernel index Fail\n");
		return DIAG_ERROR;
	}
	return DIAG_OK;
}
#endif

#ifdef CONFIG_FW_BOOTUP_COUNTER
static int diag_reset_fw_bootup_counter(int parameter)
{
	if(env_set_fw_bootup_counter(CONFIG_FW_BOOTUP_COUNTER)){
		printf("Set Bootup Counter(%u) Failed!!\n", CONFIG_FW_BOOTUP_COUNTER);
		return DIAG_ERROR;
	}
	return DIAG_OK;
}
#endif

#ifdef KERNEL_FLASH
static int diag_set_kernel_addr(int parameter)
{
	char tmp[11];//0xffffffff
	unsigned int kernelAddr = 0;
	env_get_kernelflash(&kernelAddr);
	printf("Kernel address(0x%08X):0x", kernelAddr);
	if(get_line(NULL, tmp, sizeof(tmp), -1, str_number_hex, NULL, NULL) > 0 ){
		kernelAddr = (unsigned int)simple_strtoul(tmp, NULL, 16);
		if(env_set_kernelflash(kernelAddr) < 0){
			printf("Set Kernel address Fail\n");
			return DIAG_ERROR;
		}
	}
	return DIAG_OK;
}
#endif


// Filesystem Menu =================================================
static DiagMenuTableStruct DownloadFilesystemMenuTable[] = {
#ifdef ROOTFS_FLASH
	{ '0',	"Filesystem",			0,	diag_download_Filesystem,	NULL},
#endif
#ifdef ROOTFS2_FLASH
	{ '1',	"Filesystem 2",			0,	diag_download_Filesystem2,	NULL},
#endif
#ifdef MPROOTFS_FLASH
	{ '2',	"Filesystem(MP)",		0,	diag_download_MPFilesystem, NULL},
#endif
#ifdef CONFIG_MTD_IDX
	{ 's',	"Set MTD index", 		0,	diag_set_mtd_index,			NULL},
#endif
};
static DiagMenuStruct DownloadFilesystemMenu = {
	sizeof(DownloadFilesystemMenuTable)/sizeof(DiagMenuTableStruct),
	DownloadFilesystemMenuTable,
	"<<<Download Filesystem Menu>>>",
	0, NULL
};

// Kernel Menu =================================================
static DiagMenuTableStruct DownloadKernelMenuTable[] = {
#ifdef KERNEL_FLASH
	{ '0',	"Kernel",					0,	diag_download_Kernel,			NULL},
#endif
#ifdef KERNEL2_FLASH
	{ '1',	"Kernel 2",					0,	diag_download_Kernel2,			NULL},
#endif
#ifdef MPKERNEL_FLASH
	{ '2',	"Kernel(MP)",				0,	diag_download_MPKernel, 		NULL},
#endif
#ifdef CONFIG_KERNELX_REDUND
	{ 'e',	"On/Off Kernel redund",		0,	diag_set_kernel_redund,			NULL},
#endif
#ifdef CONFIG_KERNEL_IDX
	{ 'i',	"Set Kernel index",			0,	diag_set_kernel_index,			NULL},
#endif
#ifdef CONFIG_FW_BOOTUP_COUNTER
	{ 'r',	"Reset FW Boot counter",	0,	diag_reset_fw_bootup_counter,	NULL},
#endif
#ifdef KERNEL_FLASH
	{ 's',	"Set Kernel address",		0,	diag_set_kernel_addr, 			NULL},
#endif
};
static DiagMenuStruct DownloadKernelMenu = {
	sizeof(DownloadKernelMenuTable)/sizeof(DiagMenuTableStruct),
	DownloadKernelMenuTable,
	"<<<Download Kernel Menu>>>",
	0, NULL
};

// Erase Flash Menu =================================================
static DiagMenuTableStruct EraseFlashMenuTable[] = {
	{ '0',	"Erase whole Flash",			0,	diag_erase_WholeFlash,			NULL},
	{ '1',	"Erase whole Flash (w/o BIOS)",	0,	diag_erase_WholeFlash_wo_BIOS,	NULL},
	{ '2',	"Scrub whole Flash",			0,	diag_scrub_WholeFlash,			NULL},
	{ '3',	"Scrub whole Flash (w/o BIOS)",	0,	diag_scrub_WholeFlash_wo_BIOS,	NULL},
#ifdef UBOOT_FLASH
	{ '4',	"Erase U-Boot",					0,	diag_erase_UBoot,				NULL},
#endif
#ifdef KERNEL_FLASH
	{ '5',	"Erase Kernel",					0,	diag_erase_Kernel,				NULL},
#endif
#ifdef KERNEL2_FLASH
	{ '6',	"Erase Kernel 2", 				0,	diag_erase_Kernel2,				NULL},
#endif
#ifdef MPKERNEL_FLASH
	{ '7',	"Erase Kernel(MP)",				0,	diag_erase_MPKernel, 			NULL},
#endif
#ifdef ROOTFS_FLASH
	{ '8',	"Erase Filesystem",				0,	diag_erase_Filesystem,			NULL},
#endif
#ifdef ROOTFS2_FLASH
	{ '9',	"Erase Filesystem 2",			0,	diag_erase_Filesystem2,			NULL},
#endif
#ifdef MPROOTFS_FLASH
	{ 'a',	"Erase Filesystem(MP)",			0,	diag_erase_MPFilesystem, 		NULL},
#endif
#ifdef DATA1_FLASH
	{ 'b',	"Erase Data 1",					0,	diag_erase_Data1,				NULL},
#endif
#ifdef DATA2_FLASH
	{ 'c',	"Erase Data 2",					0,	diag_erase_Data2,				NULL},
#endif
#ifdef MPDATA_FLASH
	{ 'd',	"Erase MP Data",				0,	diag_erase_MPData,				NULL},
#endif
#ifdef DSP1_FLASH
	{ 'e',	"Erase DSP 1",					0,	diag_erase_Dsp1,				NULL},
#endif
#ifdef DSP2_FLASH
	{ 'f',	"Erase DSP 2",					0,	diag_erase_Dsp2,				NULL},
#endif
#ifdef CONFIG_FLASH
	{ 'g',	"Erase Config",					0,	diag_erase_Config,				NULL},
#endif
#ifdef BACKUP_FLASH
	{ 'h',	"Erase Backup",					0,	diag_erase_Backup,				NULL},
#endif
#ifdef LOG_FLASH
	{ 'i',	"Erase Log",					0,	diag_erase_Log,					NULL},
#endif
#ifdef RESERVE_FLASH
	{ 'j',	"Erase Reserve Flash",			0,	diag_erase_Reserve,				NULL},
#endif
};
static DiagMenuStruct EraseFlashMenu = {
	sizeof(EraseFlashMenuTable)/sizeof(DiagMenuTableStruct),
	EraseFlashMenuTable,
	"<<<Erase Flash Menu>>>",
	0, NULL
};

// Download/Upload Menu =================================================
static DiagMenuTableStruct NewDownloadMenuTable[] = {
#ifdef UBOOT_FLASH
	{ '0',	"U-Boot",				0,	diag_download_UBoot,		NULL},
#endif
	{ '1',	"Firmware/MP/BIOS",		0,	diag_download_Firmware,	 	NULL},
	{ '2',	"Error Log",			0,	NULL,						NULL},
	{ '3',	"IMAGE(FB)",			0,	NULL,						NULL},
#ifdef UBL_FLASH
	{ '4',	"UBL",					0,	diag_download_UBL,			NULL},
#endif
	{ '5',	"Filesystem",			0,	NULL,					 	&DownloadFilesystemMenu},
	{ '6',	"Kernel",				0,	NULL,						&DownloadKernelMenu},
	{ '7',	"Erase Flash",			0,	NULL,						&EraseFlashMenu},
#ifdef PTCTRL_HERMES_NAME
	{ '8',	"MCU Firmware",			0,	diag_download_MCU,			NULL},
#endif
	{ '9',	"Download setting",		0,	diag_setup_download,		NULL},
#ifdef DATA1_FLASH
	{ 'a',	"Erase Data 1",			0,	diag_erase_Data1,			NULL},
#endif
#ifdef DATA2_FLASH
	{ 'b',	"Erase Data 2",			0,	diag_erase_Data2,			NULL},
#endif
	{ 'c',	"Erase whole Flash",	0,	diag_erase_WholeFlash,		NULL},
	{ 'd',	"Set Env to default",	0,	diag_do_default,			NULL},
	{ 'e',	"Reset system",			0,	diag_do_reset,				NULL},
#ifdef ISP_NAND
	{ 'f',	"ISP Nand Flash(SD)",	0,	diag_ispnand_from_mmc,		NULL},
#endif
};
static DiagMenuStruct NewDownloadMenu = {
	sizeof(NewDownloadMenuTable)/sizeof(DiagMenuTableStruct),
	NewDownloadMenuTable,
	"<<<Download/Upload Menu>>>",
	0, NULL
};

// Set SN/MAC Menu =================================================
static DiagMenuTableStruct SetSNMACMenuTable[] = {
	{ '0',	"SN Setting",			0,	MM_Set_Serial_Number,		NULL},
	{ '1',	"Mac Setting",			0,	EnvConfig_set_MAC,			NULL},
	{ '2',	"Get Current SN",		0,	MM_Get_Serial_Number,		NULL},
	{ '3',	"Get Mac Address",		0,	EnvConfig_get_MAC,			NULL},
#ifdef CONFIG_IPADDR
	{ '4',	"Get IP Address",		0,	EnvConfig_get_IP,			NULL},
	{ '5',	"Set IP Address",		0,	EnvConfig_set_IP,			NULL},
#endif
#ifdef CONFIG_SERVERIP
	{ '6',	"Get Server IP",		0,	EnvConfig_get_ServerIP,		NULL},
	{ '7',	"Set Server IP",		0,	EnvConfig_set_ServerIP,		NULL},
#endif
#ifdef CONFIG_NETMASK
	{ '8',	"Get Netmask",			0,	EnvConfig_get_Netmask,		NULL},
	{ '9',	"Set Netmask",			0,	EnvConfig_set_Netmask,		NULL},
#endif
#ifdef CONFIG_BAUDRATE
	{ 'a',	"Get Baudrate", 		0,	EnvConfig_get_Baudrate,		NULL},
	{ 'b',	"Set Baudrate", 		0,	EnvConfig_set_Baudrate,		NULL},
#endif
};
static DiagMenuStruct SetSNMACMenu = {
	sizeof(SetSNMACMenuTable)/sizeof(DiagMenuTableStruct),
	SetSNMACMenuTable,
	"<<<Set SN/MAC Menu>>>",
	0, NULL
};

// Set Burn in Time Menu =================================================
static DiagMenuTableStruct SetBurninTimeMenuTable[] = {
	{ '0',	"Burnin Time Setting",			0, 	MM_Set_Burnin_Time, 		NULL},
	{ '1',	"Get Current Burnin Time",		0, 	MM_Get_Burnin_Time, 		NULL},
};
DiagMenuStruct SetBurninTimeMenu = {
	sizeof(SetBurninTimeMenuTable)/sizeof(DiagMenuTableStruct),
	SetBurninTimeMenuTable,
	"<<<Set Burn in Time Menu>>>",
	0, NULL
};

// Model/ID Setting Menu =================================================
static DiagMenuTableStruct SetModelIDMenuTable[] = {
	{ '0',	"Model Name",				0,	MM_Set_Model,				NULL},
	{ '1',	"ID Setting",				0,	MM_Set_DUT_Id,				NULL},
	{ '2',	"Languagge Code",			0,	NULL,						NULL},
	{ '3',	"Country Code",				0,	NULL,						NULL},
	{ '4',	"Host Name",				0,	MM_Set_Host,				NULL},
	{ '5',	"Description",				0,	MM_Set_Desc,				NULL},
	{ '6',	"FW Recovery",				0,	MM_Set_FW_Recover,			NULL},
	{ '7',	"IVA Flag", 				0,	MM_Set_IVA_Flag,			NULL},
	{ '8',	"Bootup Counter ",			0,	MM_Set_T2_Bootup_Counter,	NULL},
	{ '9',	"Get Model Name",			0,	MM_Get_Model,				NULL},
	{ 'a',	"Get ID",					0,	MM_Get_DUT_Id,				NULL},
	{ 'b',	"Get Languagge Code",		0,	NULL,						NULL},
	{ 'c',	"Get Country Code", 		0,	NULL,						NULL},
	{ 'd',	"Get Host Name",			0,	MM_Get_Host,				NULL},
	{ 'e',	"Get Description",			0,	MM_Get_Desc,				NULL},
	{ 'f',	"Get FW Recovery",			0,	MM_Get_FW_Recover,			NULL},
	{ 'g',	"Get IVA Flag",				0,	MM_Get_IVA_Flag,			NULL},
	{ 'h',	"Get Bootup Counter ",		0,	MM_Get_T2_Bootup_Counter,	NULL},
};
DiagMenuStruct SetModelIDMenu = {
	sizeof(SetModelIDMenuTable)/sizeof(DiagMenuTableStruct),
	SetModelIDMenuTable,
	"<<<Model/ID Setting Menu>>>",
	0, NULL
};

// TFTP Setting Menu =================================================
static DiagMenuTableStruct SetTFTPMenuTable[] = {
	{ '0',	"Default Setting",	0, 	diag_default_download, 		NULL},
	{ '1',	"User define",		0,	diag_setup_download,		NULL},
	{ '2',	"View Setting",		0,	diag_showsetting_download,	NULL},
};
DiagMenuStruct SetTFTPMenu = {
	sizeof(SetTFTPMenuTable)/sizeof(DiagMenuTableStruct),
	SetTFTPMenuTable,
	"<<<TFTP Setting Menu>>>",
	0, NULL
};

// Ethernet Test Menu =================================================
static DiagMenuTableStruct EthernetTestMenuTable[] = {
	{ '0',	"Ping test",			0,	ping_test_func,				NULL},
};
static DiagMenuStruct EthernetTestMenu = {
	sizeof(EthernetTestMenuTable)/sizeof(DiagMenuTableStruct),
	EthernetTestMenuTable,
	"<<<Ethernet Test Menu>>>",
	0, NULL
};

// IO Test Menu =================================================
static DiagMenuTableStruct IOTestMenuTable[] = {
	{ '0',	"GPIO Test", 			0,	NULL, 				&GPIOTestMenu},
	{ '1',	"DI/DO Test",			0,	NULL,				&DIDOTestMenu},
	{ '2',	"LED Test", 			0,	NULL, 				&LedTestMenu},
	{ '3',	"I2C Test", 			0,	NULL,				&I2CTestMenu},
	{ '4',	"Sensor Test",			0,	NULL,				&SensorTestMenu},
};
static DiagMenuStruct IOTestMenu = {
	sizeof(IOTestMenuTable)/sizeof(DiagMenuTableStruct),
	IOTestMenuTable,
	"<<<IO Test Menu>>>",
	0, NULL
};

// Other Setting Menu =================================================
static DiagMenuTableStruct OtherSettingMenuTable[] = {
};
DiagMenuStruct OtherSettingMenu = {
	sizeof(OtherSettingMenuTable)/sizeof(DiagMenuTableStruct),
	OtherSettingMenuTable,
	"<<<Other Setting Menu>>>",
	0, NULL
};

// New Main Menu =================================================
static DiagMenuTableStruct NewMainMenuTable[] = {
	{ '0',	"Set/Clear Flag", 		0,	NULL,				&NewMPFlagMenu},
	{ '1',	"Download/Upload",		0,	NULL,				&NewDownloadMenu},
	{ '2',	"Set SN/MAC",			0,	NULL,				&SetSNMACMenu},
	{ '3',	"Set Burn in Time",		0,	NULL,				&SetBurninTimeMenu},
	{ '4',	"Model/ID Setting", 	0,	NULL,				&SetModelIDMenu},
	{ '5',	"TFTP Setting",			0,	NULL,				&SetTFTPMenu},
	{ '6',	"Console/UART Test",	0,	NULL,				&UARTTestMenu},
	{ '7',	"Ethernet Test",		0,	NULL,				&EthernetTestMenu},
	{ '8',	"IO Test",				0,	NULL,				&IOTestMenu},
	{ '9',	"Diagnostic Test",		0,	NULL, 				&DiagnosticMenu},
	{ 'a',	"Status Reader",		0,	NULL, 				&StatusReaderMenu},
	{ 'b',	"Bootup System", 		1,	BootPreProcessor,	NULL},
	{ 'c',	"Run T1-BIOS",			0,	RunT1BIOS,			NULL},
};
static DiagMenuStruct NewMainMenu = {
	sizeof(NewMainMenuTable)/sizeof(DiagMenuTableStruct),
	NewMainMenuTable,
	"<<<Main Menu>>>",
	0, diag_get_Info
};

int do_mmmenu (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	bool back2Root = false;
	int layer = 0;


#if CONFIG_SYS_I2C_EEPROM
	printf("Model name : ");
	EEPROM_GetModelName(&g_model_name);
	printf("%s\n", g_model_name);

	printf("Mac address : ");
	EEPROM_GetMac(&g_mac_addr);
	printf("%s\n", g_mac_addr);

	printf("Serial number : ");
	EEPROM_GetSerial(&g_serial_str);
	printf("%s\n", g_serial_str);

	printf("MP Flag : ");
	EEPROM_GetMpFlag(&g_mp_flag);
	printf("0x%03x\n", g_mp_flag);

#endif

	printf("\n\n");
	menu_process(&NewMainMenu, &back2Root, &layer);
	printf("\n\n");
	return 0;
}

U_BOOT_CMD(
	mmmenu, 1, 0,	do_mmmenu,
	"Enter Moxa MM menu",
	NULL
);

int do_moxamm (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	int boot_mode = BACKDOOR_NONE;

	/* Bobby-20150604 : move to evm.c */
	//hw_watchdog_op(HWWD_INIT);

	//wensen : let sys led to be red, because sys led is red in the init state.
	//sys_led_RG_OFF();
	sys_led_R_ON();

	Heater_Sys_OFF();


#if CONFIG_SYS_I2C_EEPROM
	{
		unsigned short check_mp_flag = 0;

		EEPROM_GetMpFlag(&check_mp_flag);
		if (check_mp_flag == 0xffff)
		{
			printf("EEPROM(Default) MP Flag : 0x%x\n", check_mp_flag);
			check_mp_flag = 0;
			EEPROM_SetMpFlag(check_mp_flag);
		}
	}
#endif


	//Heater_Cam_OFF();
	printf("\n\n%s Starting ...", CONFIG_DUT_MODEL);
#if defined(CONFIG_SD_BOOT) || defined(CONFIG_MMC_ENV)
	mmcboot_processor();
	boot_mode = BACKDOOR_MMMENU;
#else
	boot_mode = backdoor_process();
#endif
	if(boot_mode == BACKDOOR_MMMENU){
		do_mmmenu(cmdtp, flag, argc, argv);
		printf("\n\n");
	}else if(boot_mode == BACKDOOR_MMCMDL){
		printf("\nMM command line mode.\n\n");
#ifdef CONFIG_FW_RECOVERY
	}else if(boot_mode == BACKDOOR_FWRCVR){
		menu_process(&fw_recoveryMenu, NULL, NULL);
		printf("\n\n");
		diag_do_reset(0);
	}else if(boot_mode == BACKDOOR_AUTOFR){
		auto_fw_recovery_process();
		diag_do_reset(0);
#endif
	}else{
		BootPreProcessor(0);
	}
	return 0;
}

U_BOOT_CMD(
	moxamm, 1, 0,	do_moxamm,
	"Start Moxa MM",
	NULL
);

#endif /* CONFIG_CMD_MOXAMM */
                                                                                                                                                                                                                                                                                                                                                                                                              
