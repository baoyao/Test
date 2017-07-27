/*
 * An I2C driver for the Philips tpa6130 RTC
 * Copyright 2005-06 Tower Technologies
 *
 * Author: Alessandro Zummo <a.zummo@towertech.it>
 * Maintainers: http://www.nslu2-linux.org/
 *
 * based on the other drivers in this same directory.
 *
 * http://www.semiconductors.philips.com/acrobat/datasheets/tpa6130-04.pdf
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/of_gpio.h>

#include <linux/rockchip/iomap.h>
#include <linux/rockchip/pmu.h>

#include "../../arch/arm/mach-rockchip/pm.h"


#define DRV_VERSION "0.4.3"

#define tpa6130_REG_ST1		0x00 /* status */
#define tpa6130_REG_ST2		0x01

#define tpa6130_REG_SC		0x02 /* datetime */
#define tpa6130_REG_MN		0x03
#define tpa6130_REG_HR		0x04
#define tpa6130_REG_DM		0x05
#define tpa6130_REG_DW		0x06


#define RTC_SPEED 	50 * 1000

static struct i2c_driver tpa6130_driver;

static struct i2c_client *gClient = NULL;
unsigned char buf_readout2[1]={0x00};
unsigned char buf_writein2[1]={0x3D};//2B
unsigned char buf_writein2_bak[1]={0x00};//2B
unsigned char buf_writein2_tmp[1]={0x3e};//2B


struct i_val 
   {	unsigned int i;	
        unsigned char vol;	
		

};
static struct i_val tpa6130_vol_def[] = {
	{1, 0x01},
	{2, 0x02},
	{3, 0x03},
	{4, 0x04},
	{5, 0x05},
	{6, 0x06},
	{7, 0x07},
	{8, 0x08},
	{9, 0x09},
	{10, 0x09},
	
	
};


struct tpa6130 {
	//struct rtc_device *rtc;
	/*
	 * The meaning of MO_C bit varies by the chip type.
	 * From tpa6130 datasheet: this bit is toggled when the years
	 * register overflows from 99 to 00
	 *   0 indicates the century is 20xx
	 *   1 indicates the century is 19xx
	 * From RTC8564 datasheet: this bit indicates change of
	 * century. When the year digit data overflows from 99 to 00,
	 * this bit is set. By presetting it to 0 while still in the
	 * 20th century, it will be set in year 2000, ...
	 * There seems no reliable way to know how the system use this
	 * bit.  So let's do it heuristically, assuming we are live in
	 * 1970...2069.
	 */
	int c_polarity;	/* 0: MO_C=1 means 19xx, otherwise MO_C=1 means 20xx */
	int voltage_low; /* incicates if a low_voltage was detected */

	int irq;
	struct i2c_client *client;
	struct mutex mutex;
	struct rtc_device *rtc;
	struct rtc_wkalrm alarm;
	struct wake_lock wake_lock;
	struct class  *reg_class;
};

/*
 * In the routines that deal directly with the tpa6130 hardware, we use
 * rtc_time -- month 0-11, hour 0-23, yr = calendar year-epoch.
 */

static int i2c_master_reg8_send(const struct i2c_client *client, const char reg, const char *buf, int count, int scl_rate)
{
	struct i2c_adapter *adap=client->adapter;
	struct i2c_msg msg;
	int ret;
    //pr_info("write3  = %s\n", buf);
	char *tx_buf = (char *)kzalloc(count + 1, GFP_KERNEL);
	if(!tx_buf)
		return -ENOMEM;
	tx_buf[0] = reg;
	
	
	memcpy(tx_buf+1, buf, count); 
    pr_info("write 4 reg  = %x\n", tx_buf[0]);
	pr_info("write 4 val = %x\n", tx_buf[1]);
	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = count + 1;
	msg.buf = (char *)tx_buf;
	msg.scl_rate = scl_rate;

	ret = i2c_transfer(adap, &msg, 1);
	kfree(tx_buf);
	return (ret == 1) ? count : ret;

}
static int i2c_master_reg8_send_tmp(const struct i2c_client *client, const char reg, const char *buf, int count, int scl_rate)
{
	struct i2c_adapter *adap=client->adapter;
	struct i2c_msg msg;
	int ret;
    //pr_info("write3  = %s\n", buf);
	char *tx_buf = (char *)kzalloc(count + 1, GFP_KERNEL);
	if(!tx_buf)
		return -ENOMEM;
	tx_buf[0] = reg;
	
	
	memcpy(tx_buf+1, buf, count); 
	tx_buf[1]=buf_writein2_bak[1];
    pr_info("write 4 reg  = %x\n", tx_buf[0]);
	pr_info("write 4 val = %x\n", tx_buf[1]);
	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = count + 1;
	msg.buf = (char *)tx_buf;
	msg.scl_rate = scl_rate;

	ret = i2c_transfer(adap, &msg, 1);
	kfree(tx_buf);
	return (ret == 1) ? count : ret;

}


static int i2c_master_reg8_recv(const struct i2c_client *client, const char reg, char *buf, int count, int scl_rate)
{
	struct i2c_adapter *adap=client->adapter;
	struct i2c_msg msgs[2];
	int ret;
	char reg_buf = reg;
	
	msgs[0].addr = client->addr;
	msgs[0].flags = client->flags;
	msgs[0].len = 1;
	msgs[0].buf = &reg_buf;
	msgs[0].scl_rate = scl_rate;

	msgs[1].addr = client->addr;
	msgs[1].flags = client->flags | I2C_M_RD;
	msgs[1].len = count;
	msgs[1].buf = (char *)buf;
	msgs[1].scl_rate = scl_rate;

	ret = i2c_transfer(adap, msgs, 2);

	return (ret == 2)? count : ret;
}

static int tpa6130_i2c_read_regs(struct i2c_client *client, u8 reg, u8 buf[], unsigned len)
{
	int ret; 
	ret = i2c_master_reg8_recv(client, reg, buf, len, RTC_SPEED);
	return ret; 
}

static int tpa6130_i2c_set_regs(struct i2c_client *client, u8 reg, u8 const buf[], __u16 len)
{
	int ret; 
	pr_info("write2  = %s\n", buf);
	ret = i2c_master_reg8_send(client, reg, buf, (int)len, RTC_SPEED);
	return ret;
}




//static int tpa6130_rtc_set_power_on_time(struct device *dev, struct rtc_time *tm)
//{
//	return tpa6130_set_power_on_datetime(to_i2c_client(dev), tm);
//}



//static int tpa6130_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
//{
//    printk("tpa6130_rtc_set_alarm==> %s  %d\n", __FILE__,__LINE__);
//	struct rtc_time now, *tm = &alarm->time;
//	return tpa6130_set_power_on_datetime(to_i2c_client(dev), tm);
//}


static ssize_t  tpa6130_reg_show(struct class *cls,
struct class_attribute *attr, char *buf)
{
	int regval;
	regval=tpa6130_i2c_read_regs(gClient, 2, buf_readout2, 1);
	return sprintf(buf, "tpa6130 reg  = %x\n",buf_readout2[0]);

}
static u32 strtol(const char *nptr, int base)
{
	u32 ret;
	if(!nptr || (base!=16 && base!=10 && base!=8))
	{

		printk("%s(): NULL pointer input\n", __FUNCTION__);
		return -1;
	}
	for(ret=0; *nptr; nptr++)
	{


		if((base==16 && *nptr>='A' && *nptr<='F') || 
			(base==16 && *nptr>='a' && *nptr<='f') || 
			(base>=10 && *nptr>='0' && *nptr<='9') ||
			(base>=8 && *nptr>='0' && *nptr<='7') )
		{
			ret *= base;
			if(base==16 && *nptr>='A' && *nptr<='F')
				ret += *nptr-'A'+10;
			else if(base==16 && *nptr>='a' && *nptr<='f')
				ret += *nptr-'a'+10;
			else if(base>=10 && *nptr>='0' && *nptr<='9')
				ret += *nptr-'0';
			else if(base>=8 && *nptr>='0' && *nptr<='7')
				ret += *nptr-'0';
		}
		else
			return ret;
	}
	return ret;
}

static ssize_t tpa6130_reg_store(struct class *cls,
struct class_attribute *attr, unsigned char *buf, size_t count)
{
	char *string;
	int regval,i;
	unsigned char *str;
	int rc;
	unsigned long	pwn_val = -1;
	unsigned char tpa6130_rt_vol=0xF;
	u8 reg;
	
	#if 1
	rc = kstrtoul(buf, 0, &pwn_val);
	//unsigned char buf_writein2_bak[1]={0x3E};//
	buf_writein2_bak[1]=pwn_val;
	pr_info("write  = %x\n", pwn_val);
	//for (i = 0;i<=10; i++)
	//	{		//tc_write_reg(&tc358749_reg_def[i]);		
	//          if (pwn_val == tpa6130_vol_def[i].i)
			  	
	//		  	buf_writein2_bak[1]=tpa6130_vol_def[i].vol;
              //tpa6130_vol_def[i].i = i
				//mdelay(1);	

	//}
	pr_info("writeA  = %x\n", buf_writein2_bak[1]);
	//pr_info("write  tmp  = %x\n", buf_writein2_tmp[1]);
	if(buf_writein2_bak[1]==0)
	{
		gpio_direction_output(239,0);
	}
	else
		gpio_direction_output(239,1);
	i2c_master_reg8_send_tmp(gClient, 2, buf_writein2_bak, 1,50*1000);  
	//tpa6130_i2c_set_regs(gClient, 2, buf_writein2_tmp, 1); 
	#endif
	
	return strnlen(buf, PAGE_SIZE);
}

static CLASS_ATTR(tpa6130, 0666, tpa6130_reg_show, tpa6130_reg_store);

static int tpa6130_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
    int rc = 0;
	struct tpa6130 *tpa6130;
	struct rtc_device *rtc = NULL;

	

	struct device_node *np = client->dev.of_node;

	unsigned long irq_flags;
	int result,ret;
    int err=0;
	printk("%s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	tpa6130 = devm_kzalloc(&client->dev, sizeof(struct tpa6130),
				GFP_KERNEL);
	if (!tpa6130)
		return -ENOMEM;


   	
	tpa6130->client = client;
	tpa6130->alarm.enabled = 0;
	client->irq = 0;
	
	printk("chip found, driver version " DRV_VERSION "\n");
	printk("20170718%s\n",__func__);
	
	mutex_init(&tpa6130->mutex);
	wake_lock_init(&tpa6130->wake_lock, WAKE_LOCK_SUSPEND, "tpa6130");
	
    i2c_set_clientdata(client, tpa6130);

	//tpa6130->rtc = devm_rtc_device_register(&client->dev,
	//			tpa6130_driver.driver.name,
	//			&tpa6130_rtc_ops, THIS_MODULE);

	
	device_init_wakeup(&client->dev, 1);
		//rtc = devm_rtc_device_register(&client->dev,
		//		client->name,
		//					&tpa6130_rtc_ops, THIS_MODULE);
		//if (IS_ERR(rtc)) {
		//	rc = PTR_ERR(rtc);
		//	rtc = NULL;
		//	goto exit;
		//}
		//tpa6130->rtc = rtc;

  
  
	
	unsigned char buf_readout1[1]={0x00};
	//unsigned char buf_readout2[1]={0x00};
	unsigned char buf_readout3[1]={0x00};
	unsigned char buf_readout4[1]={0x00};
	unsigned char buf_readout5[1]={0x00};
	
	
	
	unsigned char buf_writein1[1]={0xC0};
	//unsigned char buf_writein2[1]={0x3F};//2B
	unsigned char buf_writein3[1]={0xFF};
	unsigned char buf_writein4[1]={0xFF};
	unsigned char buf_writein5[1]={0xFF};
	unsigned char data_value_clear[1] =  {0x00};
	
	
	tpa6130_i2c_set_regs(client, 1, buf_writein1, 1);
	tpa6130_i2c_set_regs(client, 2, buf_writein2, 1);
	tpa6130_i2c_set_regs(client, 3, buf_writein3, 1);
	tpa6130_i2c_set_regs(client, 4, buf_writein4, 1);
	tpa6130_i2c_set_regs(client, 5, buf_writein5, 1);
	
	tpa6130_i2c_read_regs(client, 1, buf_readout1, 1);
	tpa6130_i2c_read_regs(client, 2, buf_readout2, 1);
	tpa6130_i2c_read_regs(client, 3, buf_readout3, 1);
	tpa6130_i2c_read_regs(client, 4, buf_readout4, 1);
	tpa6130_i2c_read_regs(client, 5, buf_readout5, 1);

     gClient = client;
		
	printk(" ---000---read out %s: raw data is st1=%x\n, st2=%x\n,st3=%x\n,st4=%x\n,st5=%x\n",
		__func__, buf_readout1[0], buf_readout2[0], buf_readout3[0], buf_readout4[0], buf_readout5[0]);

   // if((buf_readout[0]!='y')||(buf_readout1[0]!='u')||(buf_readout2[0]!='d')||(buf_readout3[0]!='i')||(buf_readout4[0]!='t')||(buf_readout5[0]!='e'))
    //{
	//   printk("error  error   error");	
	//    while(1);
		
	//}		
	
	//data_value_clear[0] =  0x02;//0x03;
	//tpa6130_i2c_set_regs(client, tpa6130_REG_ST2, data_value_clear, 1);
	//data_value_clear[0] =  0x08;
	//tpa6130_i2c_set_regs(client, tpa6130_REG_ST1, data_value_clear, 1);

	//tpa6130_i2c_read_regs(client, 0, buf_readout, 13);

	//printk(" ---111---read out %s: raw data is st1=%02x, st2=%02x\n",
	//	__func__, buf_readout[0], buf_readout[1]);
	
	
	
	//for (i = 0; tc358749_reg_def[i].reg != 0; i++) {
	//	pr_info("0x%x, 0x%x\n", tc358749_reg_def[i].reg, tc_read_reg(&tc358749_reg_def[i]));
	//}
	
	tpa6130->reg_class = class_create(THIS_MODULE, "tpa6130_reg");
	if(IS_ERR(tpa6130->reg_class))
		pr_err("Failed to create tpa6130 class!.\n");
	ret = class_create_file(tpa6130->reg_class, &class_attr_tpa6130);
	

exit:
	if (tpa6130) {
		wake_lock_destroy(&tpa6130->wake_lock);
	}		
	//if (IS_ERR(tpa6130->rtc))
	//	return PTR_ERR(tpa6130->rtc);

	return 0;
}

static int tpa6130_remove(struct i2c_client *client)
{
	//class_remove_file(tpa6130->reg_class, &class_attr_tpa6130);
	//class_destroy(tpa6130->reg_class);
	//tpa6130->reg_class = NULL;
	return 0;
}

static const struct i2c_device_id tpa6130_id[] = {
	{ "tpa6130", 0 },
	{ "rtc8564", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tpa6130_id);

#ifdef CONFIG_OF
static const struct of_device_id tpa6130_of_match[] = {
	{ .compatible = "rockchip,tpa6130" },
	{}
};
MODULE_DEVICE_TABLE(of, tpa6130_of_match);
#endif

static struct i2c_driver tpa6130_driver = {
	.driver		= {
		.name	= "ydt-tpa6130",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(tpa6130_of_match),
	},
	.probe		= tpa6130_probe,
	.remove		= tpa6130_remove,
	.id_table	= tpa6130_id,
};

//module_i2c_driver(tpa6130_driver);
static int __init tpa6130_init(void)
{
	return i2c_add_driver(&tpa6130_driver);
}

static void __exit tpa6130_exit(void)
{
	i2c_del_driver(&tpa6130_driver);
}


MODULE_AUTHOR("Alessandro Zummo <a.zummo@towertech.it>");
MODULE_DESCRIPTION("Philips tpa6130/Epson RTC8564 RTC driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

module_init(tpa6130_init);
module_exit(tpa6130_exit);

