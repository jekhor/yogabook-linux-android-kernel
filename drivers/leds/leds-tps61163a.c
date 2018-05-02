/*
 * linux/drivers/leds-tps61163a.c
 *
 * simple PWM based LED control
 *
 * based on leds-pwm.c by Luotao Fu @ Pengutronix (l.fu@pengutronix.de)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/fb.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/leds_pwm.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>

#define DEBUG_LED_TAG	"pwm_tps61163"
struct led_pwm_data {
	struct led_classdev	cdev;
	struct pwm_device	*pwm;
	struct work_struct	work;
	unsigned int		active_low;
	unsigned int		period;
	int			duty;
	bool			can_sleep;
	unsigned		en_gpio;
};

struct led_pwm_priv {
	int num_leds;
	struct led_pwm_data leds[0];
};

static void __led_pwm_set(struct led_pwm_data *led_dat)
{
	int new_duty = led_dat->duty;
	pwm_config(led_dat->pwm, new_duty, led_dat->period);
	if (new_duty == 0){
		pwm_disable(led_dat->pwm);
		/*Add Begin by xucm1 20160628,Yeti_M-4746 for control KB led on/off*/
		gpio_set_value(led_dat->en_gpio,0);
		/*Add End by xucm1 20160628*/
	}else{
		/*Add Begin by xucm1 20160628,Yeti_M-4746 for control KB led on/off*/
		gpio_set_value(led_dat->en_gpio,1);
		/*Add End by xucm1 20160628*/
		pwm_enable(led_dat->pwm);
	}
}

static void led_pwm_work(struct work_struct *work)
{
	struct led_pwm_data *led_dat =
		container_of(work, struct led_pwm_data, work);

	__led_pwm_set(led_dat);
}

static void led_pwm_set(struct led_classdev *led_cdev,
	enum led_brightness brightness)
{

	struct led_pwm_data *led_dat =
		container_of(led_cdev, struct led_pwm_data, cdev);
	unsigned int max = led_dat->cdev.max_brightness;
	if(brightness >= max)
		brightness = max;
	if(brightness <= 0)
		brightness = 0;
	
	unsigned long long duty =  led_dat->period;
	duty *= brightness;
	do_div(duty, max);
	led_dat->duty = duty;

	if (led_dat->can_sleep)
		schedule_work(&led_dat->work);
	else
		__led_pwm_set(led_dat);
}

static inline size_t sizeof_pwm_leds_priv(int num_leds)
{
	return sizeof(struct led_pwm_priv) +
		      (sizeof(struct led_pwm_data) * num_leds);
}

static void led_pwm_cleanup(struct led_pwm_priv *priv)
{
	while (priv->num_leds--) {
		led_classdev_unregister(&priv->leds[priv->num_leds].cdev);
		if (priv->leds[priv->num_leds].can_sleep)
			cancel_work_sync(&priv->leds[priv->num_leds].work);
	}
}

static int led_pwm_create_of(struct platform_device *pdev,
			     struct led_pwm_priv *priv)
{
	struct device_node *child;
	int ret;

	for_each_child_of_node(pdev->dev.of_node, child) {
		struct led_pwm_data *led_dat = &priv->leds[priv->num_leds];

		led_dat->cdev.name = of_get_property(child, "label",
						     NULL) ? : child->name;

		led_dat->pwm = devm_of_pwm_get(&pdev->dev, child, NULL);
		if (IS_ERR(led_dat->pwm)) {
			dev_err(&pdev->dev, "unable to request PWM for %s\n",
				led_dat->cdev.name);
			ret = PTR_ERR(led_dat->pwm);
			goto err;
		}
		/* Get the period from PWM core when n*/
		led_dat->period = pwm_get_period(led_dat->pwm);

		led_dat->cdev.default_trigger = of_get_property(child,
						"linux,default-trigger", NULL);
		of_property_read_u32(child, "max-brightness",
				     &led_dat->cdev.max_brightness);

		led_dat->cdev.brightness_set = led_pwm_set;
		led_dat->cdev.brightness = LED_OFF;
		led_dat->cdev.flags |= LED_CORE_SUSPENDRESUME;

		led_dat->can_sleep = pwm_can_sleep(led_dat->pwm);
		if (led_dat->can_sleep)
			INIT_WORK(&led_dat->work, led_pwm_work);

		ret = led_classdev_register(&pdev->dev, &led_dat->cdev);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to register for %s\n",
				led_dat->cdev.name);
			of_node_put(child);
			goto err;
		}
		priv->num_leds++;
	}

	return 0;
err:
	led_pwm_cleanup(priv);

	return ret;
}

static int led_pwm_probe(struct platform_device *pdev)
{
	struct led_pwm_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct led_pwm_priv *priv;
	int count, i;
	int ret = 0;

	pr_info("%s: begin...\n", __func__);

	if (pdata)
		count = pdata->num_leds;
	else
		count = of_get_child_count(pdev->dev.of_node);

	if (!count)
		return -EINVAL;

	priv = devm_kzalloc(&pdev->dev, sizeof_pwm_leds_priv(count),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	if (pdata) {
		for (i = 0; i < count; i++) {
			struct led_pwm *cur_led = &pdata->leds[i];
			struct led_pwm_data *led_dat = &priv->leds[i];
#if 1
			struct pwm_chip *chip = find_pwm_dev(1);
			int pwm_id;

			if (!chip) {
				pr_err("%s: Could not get pwm chip", __func__);
				return 0;
			}
			pwm_id = chip->pwms[0].pwm;
			led_dat->pwm = pwm_request(pwm_id, cur_led->name);
#else
			led_dat->pwm = pwm_get(&pdev->dev, cur_led->name);
#endif
			if (IS_ERR(led_dat->pwm)) {
				ret = PTR_ERR(led_dat->pwm);
				dev_err(&pdev->dev,
					"unable to request PWM for %s\n",
					cur_led->name);
				goto err;
			}

			led_dat->cdev.name = cur_led->name;
			led_dat->cdev.default_trigger = cur_led->default_trigger;
			led_dat->active_low = cur_led->active_low;
			led_dat->period = cur_led->pwm_period_ns;
			led_dat->cdev.brightness_set = led_pwm_set;
			led_dat->cdev.brightness = LED_OFF;
			led_dat->cdev.max_brightness = cur_led->max_brightness;
			//led_dat->cdev.flags |= LED_CORE_SUSPENDRESUME;

			led_dat->can_sleep = pwm_can_sleep(led_dat->pwm);
			if (led_dat->can_sleep)
				INIT_WORK(&led_dat->work, led_pwm_work);
			
			ret = gpio_request(cur_led->en_gpio, "enable_keyboard_led");
			if (ret) {
				pr_err("%s: Failed to get gpio %d (code: %d)",
						__func__, cur_led->en_gpio, ret);
				goto err;
			}
			ret = gpio_direction_output(cur_led->en_gpio, 1);
			if (ret) {
				pr_err("%s: Failed to set gpio %d direction",
						__func__, cur_led->en_gpio);
				goto err;
			}
			led_dat->en_gpio = cur_led->en_gpio;

			ret = led_classdev_register(&pdev->dev, &led_dat->cdev);
			if (ret < 0)
				goto err;
		}
		priv->num_leds = count;
	} else {
		ret = led_pwm_create_of(pdev, priv);
		if (ret)
			return ret;
	}

	platform_set_drvdata(pdev, priv);

	return 0;

err:
	priv->num_leds = i;
	led_pwm_cleanup(priv);

	return ret;
}

static int led_pwm_remove(struct platform_device *pdev)
{
	struct led_pwm_priv *priv = platform_get_drvdata(pdev);
	int count, i;

	pr_info("%s: begin...\n", __func__);

	count = priv->num_leds;
	if (!count)
		return 0;

	led_pwm_cleanup(priv);

	for (i = 0; i < count; i++) {
		struct led_pwm_data *led_dat = &priv->leds[i];
		if (led_dat->en_gpio) {
			gpio_direction_output(led_dat->en_gpio, 0);
			gpio_free(led_dat->en_gpio);
		}
	}

	return 0;
}

#if CONFIG_PM
static int leds_tps61163_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct led_pwm_priv *priv = platform_get_drvdata(pdev);
	int count, i;

	pr_info("%s: begin...\n", __func__);

	count = priv->num_leds;
	if (!count)
		return 0;

	for (i = 0; i < count; i++) {
		struct led_pwm_data *led_dat = &priv->leds[i];

		if (led_dat->en_gpio) {
			gpio_direction_output(led_dat->en_gpio, 0);
		}
	}

	return 0;
}

static int leds_tps61163_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct led_pwm_priv *priv = platform_get_drvdata(pdev);
	int count, i;

	pr_info("%s: begin...\n", __func__);

	count = priv->num_leds;
	if (!count)
		return 0;

	for (i = 0; i < count; i++) {
		struct led_pwm_data *led_dat = &priv->leds[i];

		if (led_dat->en_gpio) {
			gpio_direction_output(led_dat->en_gpio, 1);
		}
	}

	return 0;
}

static const struct dev_pm_ops leds_tps61163_pm = {
	.suspend	= leds_tps61163_suspend,
	.resume		= leds_tps61163_resume,
};
#endif


static const struct of_device_id of_pwm_leds_match[] = {
	{ .compatible = "pwm-tps61163", },
	{},
};
MODULE_DEVICE_TABLE(of, of_pwm_leds_match);

static struct platform_driver led_pwm_driver = {
	.probe		= led_pwm_probe,
	.remove		= led_pwm_remove,
	.driver		= {
		.name	= "leds_tps61163",
		.owner	= THIS_MODULE,
		.of_match_table = of_pwm_leds_match,
#if CONFIG_PM
		.pm = &leds_tps61163_pm,
#endif
	},
};

module_platform_driver(led_pwm_driver);

MODULE_DESCRIPTION("PWM LED driver for tps61163");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:leds-pwm");
