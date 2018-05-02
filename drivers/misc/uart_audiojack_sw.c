/*
 * uart_audiojack_switch.c - For switching the UART_1 and Audio Jack path
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: <lvxin1@lenovo.com>
 *
 * Note : This driver should be integrated with AP/CP switch driver, these similary
 *        function driver will be placed in the same /proc directory, for example:
 *        /proc/racer-switch/
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
//#include <linux/sysdev.h>
#include <linux/proc_fs.h>
#include <linux/gpio.h>

#define UART_AUDIOJACK_SWITCH_ENTRY	"uart_audiojack_switch"

/*
 * Considering the Core gpio, need add 96
 */
 
#define gpio_southwest_NUM	98
#define gpio_north_NUM	73
#define gpio_east_NUM	27
#define gpio_southeast_NUM	86

#define gpio_southwest_base	(ARCH_NR_GPIOS-gpio_southwest_NUM)
#define gpio_north_base		(gpio_southwest_base - gpio_north_NUM)
#define gpio_east_base		(gpio_north_base - gpio_east_NUM)
#define gpio_southeast_base	(gpio_east_base - gpio_southeast_NUM)

#define GPIO_SPI_CS1_B		66 
#define UART_AUDIOJACK_SWITCH_PIN	(gpio_southeast_base + GPIO_SPI_CS1_B)

#define SWITCH_DBG
#ifdef SWITCH_DBG
#define switch_dbg(format, args...)	printk(KERN_INFO "%s:"format, \
		__FUNCTION__, ##args)
#else
#define switch_dbg(fmt, args...)
#endif

#define UART_PATH_EVT	(1)
#define AUDIO_PATH_EVT	(0)

unsigned int __read_mostly switch_to_uart = AUDIO_PATH_EVT;

static ssize_t switch_write(struct file *file, const char __user *buf, ssize_t nbytes, loff_t *ppos)
{
	char string[nbytes];
	sscanf(buf, "%s", string);
	if (!strcmp((const char *)string, (const char *)"uart")) {
		switch_dbg("switch to uart\n");
		gpio_set_value(UART_AUDIOJACK_SWITCH_PIN, UART_PATH_EVT);
	} else if (!strcmp((const char *)string, (const char *)"audiojack")) {
		switch_dbg("switch to audiojack\n");
		gpio_set_value(UART_AUDIOJACK_SWITCH_PIN, AUDIO_PATH_EVT);
	} else {
		switch_dbg("Command Error : \n");
		switch_dbg("Switch to uart : echo \"uart\" > /proc/uart_audiojack_switch\n");
		switch_dbg("Switch to AUDIO JACK : echo \"audiojack\" > /proc/uart_audiojack_switch\n");
	}
	return nbytes;
}

static const struct file_operations uart_audiojack_switch_operations = {
	.owner = THIS_MODULE,
	.write = switch_write
};


static int __init racer_switch_init(void)
{
	int ret;
	struct proc_dir_entry	*pde;

	/* Setting UART & AUDIOJACK switch gpio, default switch to uart debug port */
	ret = gpio_request(UART_AUDIOJACK_SWITCH_PIN, "uart_ajck_switch");
	if (ret) {
		printk(KERN_INFO "Request UART_AUDIOJACK_SWITCH_PIN failed!\n");
		return 0;
	}

    if (switch_to_uart)
        gpio_direction_output(UART_AUDIOJACK_SWITCH_PIN, UART_PATH_EVT);
    else
        gpio_direction_output(UART_AUDIOJACK_SWITCH_PIN, AUDIO_PATH_EVT);

	/*Create /proc/racer_switch/uart_audiojack_switch */
	pde = proc_create(UART_AUDIOJACK_SWITCH_ENTRY, S_IFREG | S_IWUGO | S_IWUSR, NULL, &uart_audiojack_switch_operations);
	if (!pde)
		goto err1;

	return 0;

err1:
	remove_proc_entry(UART_AUDIOJACK_SWITCH_ENTRY, NULL);
	gpio_free(UART_AUDIOJACK_SWITCH_PIN);
	return -ENOMEM;
}


static void __exit racer_switch_exit(void)
{
	gpio_free(UART_AUDIOJACK_SWITCH_PIN);
	remove_proc_entry(UART_AUDIOJACK_SWITCH_ENTRY, NULL);
	return ;
}

static int __init switch_to_uart_setup(char *str)
{
    switch_to_uart = simple_strtoul(str, NULL, 0);
    return 1;
}

__setup("switch_to_uart=", switch_to_uart_setup);

MODULE_AUTHOR("lvxin1@lenovo.com");
MODULE_DESCRIPTION("UART_AUDIOJACK switch driver");
MODULE_LICENSE("GPL");

fs_initcall(racer_switch_init);
module_exit(racer_switch_exit);
