#ifndef __TPS65911_H__
#define __TPS65911_H__

#define TPS65911_SLAVE_ADDR		0x2d

/* config for VDD1, VDD2, and VDDCRTL */
#ifdef CONFIG_DM385
#define VDD_1D1		(0x2f)
#else
#define VDD_1D1		(0x2b)
#endif
#define VDD_1D2		(0x33)
#define VDD_1D35	(0x3f)

/* config for Back-up battery charger control register */
#define BBCHEN      (0x01)
#define BBSEL_3V    (0x00 << 1)
#define BBSEL_2D52V (0x01 << 1)
#define BBSEL_3D15V (0x02 << 1)
#define BBSEL_VBAT  (0X03 << 1)

/* config for Device control register */
#define RTC_PWDN	(0x1 << 6)

/* config for GPIOx configuration register */
#define GPIO_SET	(0x1 << 0)
#define GPIO_STS	(0x1 << 1)
#define GPIO_CFG	(0x1 << 2)
#define GPIO_PDEN	(0x1 << 3)
#define GPIO_DEB	(0x1 << 4)
#define GPIO_ODEN	(0x1 << 5)
#define GPIO_SLEEP	(0x1 << 7)

/* registers */
#define SECONDS_REG           (0x00)
#define MINUTES_REG           (0x01)
#define HOURS_REG             (0x02)
#define DAYS_REG              (0x03)
#define MONTHS_REG            (0x04)
#define YEARS_REG             (0x05)
#define WEEKS_REG             (0x06)
#define ALARM_SECONDS_REG     (0x08)
#define ALARM_MINUTES_REG     (0x09)
#define ALARM_HOURS_REG       (0x0A)
#define ALARM_DAYS_REG        (0x0B)
#define ALARM_MONTHS_REG      (0x0C)
#define ALARM_YEARS_REG       (0x0D)
#define RTC_CTRL_REG          (0x10)
#define RTC_STATUS_REG        (0x11)
#define RTC_INTERRUPTS_REG    (0x12)
#define RTC_COMP_LSB_REG      (0x13)
#define RTC_COMP_MSB_REG      (0x14)
#define RTC_RES_PROG_REG      (0x15)
#define RTC_RESET_STATUS_REG  (0x16)
#define BCK1_REG              (0x17)
#define BCK2_REG              (0x18)
#define BCK3_REG              (0x19)
#define BCK4_REG              (0x1A)
#define BCK5_REG              (0x1B)
#define PUADEN_REG            (0x1C)
#define REF_REG               (0x1D)
#define VRTC_REG              (0x1E)
#define VIO_REG               (0x20)
#define VDD1_REG              (0x21)
#define VDD1_OP_REG           (0x22)
#define VDD1_SR_REG           (0x23)
#define VDD2_REG              (0x24)
#define VDD2_OP_REG           (0x25)
#define VDD2_SR_REG           (0x26)
#define VDDCRTL_REG           (0x27)
#define VDDCRTL_OP_REG        (0x28)
#define VDDCRTL_SR_REG        (0x29)
#define LDO1_REG              (0x30)
#define LDO2_REG              (0x31)
#define LDO5_REG              (0x32)
#define LDO8_REG              (0x33)
#define LDO7_REG              (0x34)
#define LDO6_REG              (0x35)
#define LDO4_REG              (0x36)
#define LD03_REG              (0x37)
#define THERM_REG             (0x38)
#define BBCH_REG              (0x39)
#define DCDCCTRL_REG          (0x3E)
#define DEVCTRL_REG           (0x3F)
#define DEVCTRL2_REG          (0x40)
#define SLEEP_KEEP_LDO_ON_REG (0x41)
#define SLEEP_KEEP_RES_ON_REG (0x42)
#define SLEEP_SET_LDO_OFF_REG (0x43)
#define SLEEP_SET_RES_OFF_REG (0x44)
#define EN1_LDO_ASS_REG       (0x45)
#define EN1_SMPS_ASS_REG      (0x46)
#define EN2_LDO_ASS_REG       (0x47)
#define EN2_SMPS_ASS_REG      (0x48)
#define INT_STS_REG           (0x50)
#define INT_MSK_REG           (0x51)
#define INT_STS2_REG          (0x52)
#define INT_MSK2_REG          (0x53)
#define INT_STS3_REG          (0x54)
#define INT_MSK3_REG          (0x55)
#define GPIO0_REG             (0x60)
#define GPIO1_REG             (0x61)
#define GPIO2_REG             (0x62)
#define GPIO3_REG             (0x63)
#define GPIO4_REG             (0x64)
#define GPIO5_REG             (0x65)
#define GPIO6_REG             (0x66)
#define GPIO7_REG             (0x67)
#define GPIO8_REG             (0x68)
#define WATCHDOG_REG          (0x69)
#define VMBCH_REG             (0x6A)
#define VMBCH2_REG            (0x6B)
#define LED_CTRL1_REG         (0x6C)
#define LED_CTRL2_REG1        (0x6D)
#define PWM_CTRL1_REG         (0x6E)
#define PWM_CTRL2_REG         (0x6F)
#define SPARE_REG             (0x70)
#define VERNUM_REG            (0x80)

/* functions */
int tps65911_init(void);
int tps65911_config(u8 addr, u8 val);
int tps65911_read(u8 addr, u8* buf, int length);

#endif /*__TPS65911_H__*/
