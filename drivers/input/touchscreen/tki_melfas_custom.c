/* file_name: tki_melfas.c
 *
 * description: Melfas Touch Key Interface drv
 *
 * Copyright (C) 2008-2010 Pantech Co. Ltd.
 *
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/earlysuspend.h>
#include <asm/io.h>
#include <linux/gpio.h>
#include <mach/vreg.h>
#include <linux/regulator/consumer.h>
#include <mach/gpio.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <BOARD_REV.h>

/* -------------------------------------------------------------------- */
/* debug option */
/* -------------------------------------------------------------------- */

//#define TKI_DBG_ENABLE
#ifdef TKI_DBG_ENABLE
#define dbg(fmt, args...)	printk("[TKI] " fmt, ##args)
#else
#define dbg(fmt, args...)
#endif
#define dbg_func_in()		dbg("[+] %s\n", __func__)
#define dbg_func_out()		dbg("[-] %s\n", __func__)
#define dbg_line()			dbg("line : %d | func : %s\n", __LINE__, __func__)


/* -------------------------------------------------------------------- */
/* tki i2c slave address */
/* -------------------------------------------------------------------- */
#define TKI_I2C_ADDR       0x20
/* -------------------------------------------------------------------- */
/* GPIO Setting */
/* -------------------------------------------------------------------- */
#define GPIO_TKI_CHG			50
#define GPIO_TKI_SDA			51
#define GPIO_TKI_SCL			52
#define IRQ_TKI_INT			gpio_to_irq(GPIO_TKI_CHG)

/* -------------------------------------------------------------------- */
/* Define Value							*/
/* -------------------------------------------------------------------- */
#define TKI_REG_READ_KEYCODE		0x00

#if defined(CONFIG_MACH_MSM8X60_PRESTO) && (BOARD_REV <= WS10)
#define TKI_KEYCODE_BACK	0x01
#define TKI_KEYCODE_MENU	0x02
#define TKI_KEYCODE_HOME	0x03
#define TKI_KEYCODE_SEARCH	0x04
#else
#define TKI_KEYCODE_MENU	0x01
#define TKI_KEYCODE_HOME	0x02
#define TKI_KEYCODE_BACK	0x03
#define TKI_KEYCODE_SEARCH	0x04
#endif

#define TKI_KEY_RELEASE		0x08
#define TKI_KEY_PRESS		0x00
#define TKI_KEY_MASK		0x07

/* -------------------------------------------------------------------- */
/* function proto type & variable for driver				*/
/* -------------------------------------------------------------------- */
struct tki_data_t
{
    struct i2c_client *client;
    struct input_dev *input_dev;
    struct work_struct work;
    struct early_suspend es;
    bool power_enabled;
};
static struct tki_data_t tki_data;
static struct workqueue_struct *tki_wq;
static struct regulator *vreg_power;

static int __devinit tki_probe(struct i2c_client *client,
				const struct i2c_device_id *id);
static int __devexit tki_remove(struct i2c_client *client);
static int tki_power_on();
static int tki_power_off();
#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_PM)
static void tki_late_resume(struct early_suspend *h);
static void tki_early_suspend(struct early_suspend *h);
#endif /* CONFIG_HAS_EARLYSUSPEND */
#ifdef CONFIG_PM_SLEEP
static int tki_suspend(struct device *dev);
static int tki_resume(struct device *dev);
static SIMPLE_DEV_PM_OPS(tki_pm_ops, tki_suspend, tki_resume);
#endif /* CONFIG_PM_SLEEP */

static const struct i2c_device_id tki_id[] = {
	{ "tki-i2c", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tki_id);

static struct i2c_driver tki_driver = {
	.driver = {
		.name	= "tki-i2c",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM_SLEEP
		.pm	= &tki_pm_ops,
#endif /* CONFIG_PM_SLEEP */
	},
	.probe		= tki_probe,
	.remove		= __devexit_p(tki_remove),
	.id_table	= tki_id,
};

static bool wake_from_early_suspend = 1;
module_param(wake_from_early_suspend, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(wake_from_early_suspend, "Allow the Touch Key Interface to "
	"wake the screen from early suspend.");

static void tki_get_message(struct work_struct * p)
{
	int rc = -1, key_press =1;
	u8 byte_data;
	dbg_func_in();

	rc = i2c_smbus_read_i2c_block_data(tki_data.client,
		TKI_REG_READ_KEYCODE, sizeof(byte_data), &byte_data);
	enable_irq(tki_data.client->irq);
	if (rc < 0) {
		dbg("i2c read failed (%d)\n", rc);
		return;
	}

	dbg("KeyCode : %02x\n", byte_data);

	if(byte_data & TKI_KEY_RELEASE)
		key_press = 0;

	switch(byte_data & TKI_KEY_MASK)
	{
		case TKI_KEYCODE_BACK :
			input_report_key(tki_data.input_dev, KEY_RECENT, key_press);
			break;
		case TKI_KEYCODE_MENU :
			input_report_key(tki_data.input_dev, KEY_BACK, key_press);
			break;
		case TKI_KEYCODE_HOME :
			input_report_key(tki_data.input_dev, KEY_HOME, key_press);
			break; 
		case TKI_KEYCODE_SEARCH :
			input_report_key(tki_data.input_dev, KEY_POWER, key_press);
			break; 
		default:
			dbg("Unknown Keycode [%02x]\n", byte_data & TKI_KEY_MASK);
			break;
	}

	input_sync(tki_data.input_dev);

	dbg_func_out();
	return;
}

static irqreturn_t tki_irq_handler(int irq, void *dev_id)
{
	dbg_func_in();
	disable_irq_nosync(tki_data.client->irq);
	queue_work(tki_wq, &tki_data.work);
	dbg_func_out();
	return IRQ_HANDLED;
}

#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_PM)
static void tki_early_suspend(struct early_suspend *h)
{
	dbg_func_in();
	if (!device_may_wakeup(&tki_data.client->dev) &&
			!wake_from_early_suspend) {
		tki_power_off();
	}
	dbg_func_out();
}

static void tki_late_resume(struct early_suspend *h)
{
	dbg_func_in();
	// This function is called when the screen turns on. The touch-key
	// interface should be powered-on no matter what when the screen is on.
	tki_power_on();
	dbg_func_out();
}
#endif /* CONFIG_HAS_EARLYSUSPEND */

#ifdef CONFIG_PM_SLEEP
static int tki_suspend(struct device *dev)
{
	dbg_func_in();
	if (device_may_wakeup(dev)) {
		enable_irq_wake(tki_data.client->irq);
        } else {
		tki_power_off();
	}
	dbg_func_out();
	return 0;
}

static int tki_resume(struct device *dev)
{
	dbg_func_in();
	if (device_may_wakeup(dev)) {
		disable_irq_wake(tki_data.client->irq);
	} else if (wake_from_early_suspend) {
		tki_power_on();
	}
	dbg_func_out();
	return 0;
}
#endif

static int __devinit tki_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int rc;

	dbg_func_in();

	tki_data.client = client;

	tki_wq = create_singlethread_workqueue("tki_wq");
	if (!tki_wq)
	{
		dbg("create_singlethread_workqueue(tki_wq) error.\n");
		return -ENOMEM;
	}
	INIT_WORK(&tki_data.work, tki_get_message);

	tki_data.input_dev = input_allocate_device();
	if (tki_data.input_dev == NULL)
	{
		rc = -ENOMEM;
		dbg("tki_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	tki_data.input_dev->name = "tki_input";

	set_bit(KEY_RECENT, tki_data.input_dev->keybit);
	set_bit(KEY_BACK, tki_data.input_dev->keybit);
	set_bit(KEY_HOME, tki_data.input_dev->keybit);
	set_bit(KEY_POWER, tki_data.input_dev->keybit);
	set_bit(EV_SYN, tki_data.input_dev->evbit);
	set_bit(EV_KEY, tki_data.input_dev->evbit);

	rc = input_register_device(tki_data.input_dev);
	if (rc) 
	{
		dbg("tki_probe: Unable to register %s input device\n",
			tki_data.input_dev->name);
		goto err_input_register_device_failed;
	}

	tki_data.client->irq = IRQ_TKI_INT;
	rc = request_irq(tki_data.client->irq, tki_irq_handler,
				IRQF_TRIGGER_LOW, "tki-irq", &tki_data);
	if (rc)
	{
		pr_err("tki_probe: request_irq failed\n");
		goto err_request_irq_failed;
	}


#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_PM)
	tki_data.es.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	tki_data.es.suspend = (void*)tki_early_suspend;
	tki_data.es.resume = (void*)tki_late_resume;
	register_early_suspend(&tki_data.es);
#endif /* CONFIG_HAS_EARLYSUSPEND */

	dbg("tki_probe: Start touch key interface %s\n",
		tki_data.input_dev->name);

	dbg_func_out();
	return 0;

err_request_irq_failed:
	input_unregister_device(tki_data.input_dev);
	goto err_input_dev_alloc_failed;
err_input_register_device_failed:
	input_free_device(tki_data.input_dev);
err_input_dev_alloc_failed:
	destroy_workqueue(tki_wq);

	pr_err("tki probe failed: rc=%d\n", rc);
	return rc;
}

static int tki_remove(struct i2c_client *client)
{
	dbg_func_in();
	free_irq(client->irq, 0);
	input_unregister_device(tki_data.input_dev);
	if (tki_wq)    	
		destroy_workqueue(tki_wq);    
	dbg_func_out();    
	return 0;
}

static int tki_power_on(void)
{
	int rc;
	dbg_func_in();

	if (tki_data.power_enabled) {
		goto out;
	}

	rc = regulator_set_voltage(vreg_power, 3300000, 3300000);
	if (rc) {
		printk(KERN_ERR "%s: vreg set level failed (%d)\n", __func__, rc);
		return rc;
	}

	rc = regulator_enable(vreg_power);
	if (rc) {
		printk(KERN_ERR "%s: regulator enable failed (%d)\n", __func__, rc);
		return rc;
	}

	tki_data.power_enabled = 1;
out:
	dbg_func_out();
	return 0;
}

static int tki_power_off(void)
{
	int rc;
	dbg_func_in();

	if (!tki_data.power_enabled) {
		goto out;
	}

	rc = regulator_disable(vreg_power);
	if (rc) {
		printk(KERN_ERR "%s: regulator disable failed (%d)\n", __func__, rc);
		return rc;
	}

	tki_data.power_enabled = 0;

out:
	dbg_func_out();
	return 0;
}

static int tki_hw_setting(void)
{
	int rc; 
	unsigned gpioConfig;
	dbg_func_in();

	vreg_power = regulator_get(NULL, "8058_l19");
	if (IS_ERR(vreg_power)) {
		rc = PTR_ERR(vreg_power);
		printk(KERN_ERR "%s: regulator get of %s failed (%d)\n",
				__func__, "vreg_touch_power", rc);
		return rc;
	}

	tki_data.power_enabled = 0;
	rc = tki_power_on();
	if (rc) {
		goto power_on_fail;
	}
	
	gpio_request(GPIO_TKI_CHG, "tki_chg_int");
	gpioConfig = GPIO_CFG(GPIO_TKI_CHG, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP, GPIO_CFG_2MA);
	rc = gpio_tlmm_config(gpioConfig, GPIO_CFG_ENABLE);
	if (rc) {
		printk(KERN_ERR "%s: gpio configuration failed (%d)\n", __func__, rc);
		goto gpio_fail;
	}
	
	dbg_func_out();
	return 0;

gpio_fail:
	tki_power_off();
power_on_fail:
	regulator_put(vreg_power);
	return rc;
}

int __init tki_init(void)
{
    int rc;

    dbg_func_in();
    rc = tki_hw_setting();
    if (rc < 0)
    {
	printk(KERN_ERR "%s: tki_hw_setting failed. (rc=%d)\n", __func__, rc);
    	return rc;
    }

    dbg("i2c_add_driver\n");
    rc = i2c_add_driver(&tki_driver);
    if (rc)
    {
	printk(KERN_ERR, "%s: i2c_add_driver failed. (rc=%d)\n", __func__, rc);
    }

    dbg_func_out();
    return rc;
}

void __exit tki_exit(void)
{
	dbg_func_in();
	i2c_del_driver(&tki_driver);
	tki_power_off();
	regulator_put(vreg_power);
	dbg_func_out();
	return;
}

//late_initcall(tki_init);
module_init(tki_init);
module_exit(tki_exit);

MODULE_DESCRIPTION("MELFAS Touch Key Interface Driver");
MODULE_LICENSE("GPL");
