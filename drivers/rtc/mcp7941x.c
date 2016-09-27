/*
 * Wensen :
 * Refer http://lists.denx.de/pipermail/u-boot/2014-March/174950.html
 * MCP7941x Real Time Clock (RTC).
 *
 * based on ds1307.c
 */

#include <common.h>
#include <command.h>
#include <rtc.h>
#include <i2c.h>

#if defined(CONFIG_CMD_DATE)

/*---------------------------------------------------------------------*/
#undef DEBUG_RTC

#ifdef DEBUG_RTC
#define DEBUGR(fmt,args...) printf(fmt ,##args)
#else
#define DEBUGR(fmt,args...)
#endif
/*---------------------------------------------------------------------*/

#ifndef CONFIG_SYS_I2C_RTC_ADDR
# define CONFIG_SYS_I2C_RTC_ADDR	0x6f
#endif

//#if defined(CONFIG_RTC_DS1307) && (CONFIG_SYS_I2C_SPEED > 100000)
//# error The DS1307 is specified only up to 100kHz!
//#endif

#if defined(CONFIG_RTC_MCP7941X) && (CONFIG_SYS_I2C_SPEED > 400000)
# error The MCP7941x is specified only up to 400kHz!
#endif

/*
 * RTC register addresses
 */
#define RTC_SEC_REG_ADDR	0x00
#define RTC_MIN_REG_ADDR	0x01
#define RTC_HR_REG_ADDR		0x02
#define RTC_DAY_REG_ADDR	0x03
#define RTC_DATE_REG_ADDR	0x04
#define RTC_MON_REG_ADDR	0x05
#define RTC_YR_REG_ADDR		0x06
#define RTC_CTL_REG_ADDR	0x07

#define RTC_DAY_BIT_VBATEN	0x08	/* VBAT enable (in register 3)	*/
#define RTC_SEC_BIT_ST		0x80	/* Oscillator start (in reg 0)	*/

#define RTC_CTL_BIT_RS0		0x01	/* Rate select 0                */
#define RTC_CTL_BIT_RS1		0x02	/* Rate select 1                */
#define RTC_CTL_BIT_SQWE	0x40	/* Square Wave Enable           */
#define RTC_CTL_BIT_OUT		0x80	/* Output Control               */

static uchar rtc_read (uchar reg);
static void rtc_write (uchar reg, uchar val);

/*
 * Get the current time from the RTC
 */
int rtc_get (struct rtc_time *tmp)
{
	int rel = 0;
	uchar sec, min, hour, mday, wday, mon, year;

	sec = rtc_read (RTC_SEC_REG_ADDR);
	min = rtc_read (RTC_MIN_REG_ADDR);
	hour = rtc_read (RTC_HR_REG_ADDR);
	wday = rtc_read (RTC_DAY_REG_ADDR);
	mday = rtc_read (RTC_DATE_REG_ADDR);
	mon = rtc_read (RTC_MON_REG_ADDR);
	year = rtc_read (RTC_YR_REG_ADDR);

	DEBUGR ("Get RTC year: %02x mon: %02x mday: %02x wday: %02x "
		"hr: %02x min: %02x sec: %02x\n",
		year, mon, mday, wday, hour, min, sec);


	if (!(wday & RTC_DAY_BIT_VBATEN)) {
		printf ("### Warning: VBAT not enabled - enabling\n");
		rtc_write (RTC_DAY_REG_ADDR,
			rtc_read (RTC_DAY_REG_ADDR) | RTC_DAY_BIT_VBATEN);
	}
	if (!(sec & RTC_SEC_BIT_ST)) {
		printf ("### Warning: RTC oscillator has stopped\n");
		rtc_write (RTC_SEC_REG_ADDR,
			rtc_read (RTC_SEC_REG_ADDR) | RTC_SEC_BIT_ST);

		rel = -1;
	}



	tmp->tm_sec  = bcd2bin (sec & 0x7F);
	tmp->tm_min  = bcd2bin (min & 0x7F);
	tmp->tm_hour = bcd2bin (hour & 0x3F);
	tmp->tm_mday = bcd2bin (mday & 0x3F);
	tmp->tm_mon  = bcd2bin (mon & 0x1F);
	tmp->tm_year = bcd2bin (year) + ( bcd2bin (year) >= 70 ? 1900 : 2000);
	tmp->tm_wday = bcd2bin ((wday - 1) & 0x07);
	tmp->tm_yday = 0;
	tmp->tm_isdst= 0;

	DEBUGR ("Get DATE: %4d-%02d-%02d (wday=%d)  TIME: %2d:%02d:%02d\n",
		tmp->tm_year, tmp->tm_mon, tmp->tm_mday, tmp->tm_wday,
		tmp->tm_hour, tmp->tm_min, tmp->tm_sec);

	return rel;
}

/*
 * Set the RTC
 */
int rtc_set (struct rtc_time *tmp)
{
	DEBUGR ("Set DATE: %4d-%02d-%02d (wday=%d)  TIME: %2d:%02d:%02d\n",
		tmp->tm_year, tmp->tm_mon, tmp->tm_mday, tmp->tm_wday,
		tmp->tm_hour, tmp->tm_min, tmp->tm_sec);

	if (tmp->tm_year < 1970 || tmp->tm_year > 2069)
		printf("WARNING: year should be between 1970 and 2069!\n");

	rtc_write (RTC_YR_REG_ADDR, bin2bcd (tmp->tm_year % 100));
	rtc_write (RTC_MON_REG_ADDR, bin2bcd (tmp->tm_mon));

	/* Don't clobber the VBATEN bit. */
	rtc_write (RTC_DAY_REG_ADDR, bin2bcd (tmp->tm_wday + 1) | RTC_DAY_BIT_VBATEN);

	rtc_write (RTC_DATE_REG_ADDR, bin2bcd (tmp->tm_mday));
	rtc_write (RTC_HR_REG_ADDR, bin2bcd (tmp->tm_hour));
	rtc_write (RTC_MIN_REG_ADDR, bin2bcd (tmp->tm_min));

	/* Don't clobber the ST bit. */
	rtc_write (RTC_SEC_REG_ADDR, bin2bcd (tmp->tm_sec) | RTC_SEC_BIT_ST);

	return 0;
}


/*
 * Reset the RTC. We setting the date back to 1970-01-01.
 * We also enable the oscillator output on the SQW/OUT pin and program
 * it for 32,768 Hz output. Note that according to the datasheet, turning
 * on the square wave output increases the current drain on the backup
 * battery to something between 480nA and 800nA.
 */
void rtc_reset (void)
{
	struct rtc_time tmp;

	/* Set the oscillator start and VBATEN bits. */
	rtc_write (RTC_SEC_REG_ADDR, RTC_SEC_BIT_ST);
	rtc_write (RTC_DAY_REG_ADDR, RTC_DAY_BIT_VBATEN);

	rtc_write (RTC_CTL_REG_ADDR, RTC_CTL_BIT_SQWE | RTC_CTL_BIT_RS1 | RTC_CTL_BIT_RS0);

	tmp.tm_year = 1970;
	tmp.tm_mon = 1;
	tmp.tm_mday= 1;
	tmp.tm_hour = 0;
	tmp.tm_min = 0;
	tmp.tm_sec = 0;

	rtc_set(&tmp);

	printf ( "RTC:   %4d-%02d-%02d %2d:%02d:%02d UTC\n",
		tmp.tm_year, tmp.tm_mon, tmp.tm_mday,
		tmp.tm_hour, tmp.tm_min, tmp.tm_sec);

	return;
}


/*
 * Helper functions
 */

static
uchar rtc_read (uchar reg)
{
	return (i2c_reg_read (CONFIG_SYS_I2C_RTC_ADDR, reg));
}


static void rtc_write (uchar reg, uchar val)
{
	i2c_reg_write (CONFIG_SYS_I2C_RTC_ADDR, reg, val);
}
#endif
