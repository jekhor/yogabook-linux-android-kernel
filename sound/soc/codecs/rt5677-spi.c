/*
 * rt5677-spi.c  --  RT5677 ALSA SoC audio codec driver
 *
 * Copyright 2013 Realtek Semiconductor Corp.
 * Author: Oder Chiou <oder_chiou@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_qos.h>
#include <linux/sysfs.h>
#include <linux/clk.h>
#include "rt5677.h"
#include "rt5677-spi.h"
#include <linux/acpi.h>
#include "../intel/board/cht_bl_pinctrl.h"
static struct spi_device *g_spi;

int rt5677_spi_single_write(u32 addr, unsigned int val, size_t len)
{
	struct spi_device *spi = g_spi;
	int status;
	u8 write_buf[10];


	write_buf[1] = (addr & 0xff000000) >> 24;
	write_buf[2] = (addr & 0x00ff0000) >> 16;
	write_buf[3] = (addr & 0x0000ff00) >> 8;
	write_buf[4] = (addr & 0x000000ff) >> 0;

	if (len == 4) {
		write_buf[0] = RT5677_SPI_CMD_32_WRITE;
		write_buf[5] = (val & 0xff000000) >> 24;
		write_buf[6] = (val & 0x00ff0000) >> 16;
		write_buf[7] = (val & 0x0000ff00) >> 8;
		write_buf[8] = (val & 0x000000ff) >> 0;
	} else {
		write_buf[0] = RT5677_SPI_CMD_16_WRITE;
		write_buf[5] = (val & 0x0000ff00) >> 8;
		write_buf[6] = (val & 0x000000ff) >> 0;
	}

	status = spi_write(spi, write_buf,
		(len == 4) ? sizeof(write_buf) : sizeof(write_buf) - 2);

	if (status)
		dev_err(&spi->dev, "%s error %d\n", __FUNCTION__, status);

	return status;
}
EXPORT_SYMBOL_GPL(rt5677_spi_single_write);

/**
 * rt5677_spi_write - Write data to SPI.
 * @txbuf: Data Buffer for writing.
 * @len: Data length.
 *
 *
 * Returns true for success.
 */
int rt5677_spi_write(u8 *txbuf, size_t len)
{
	int status;

	status = spi_write(g_spi, txbuf, len);

	if (status)
		dev_err(&g_spi->dev, "rt5677_spi_write error %d\n", status);

	return status;
}

/**
 * rt5677_spi_burst_read - Read data from SPI by rt5677 dsp memory address.
 * @addr: Start address.
 * @rxbuf: Data Buffer for reading.
 * @len: Data length, it must be a multiple of 8.
 *
 *
 * Returns true for success.
 */
int rt5677_spi_burst_read(unsigned int addr, u8 *rxbuf, size_t len)
{
	u8 spi_cmd = RT5677_SPI_CMD_BURST_READ;
	int status;
	u8 write_buf[8];
	unsigned int i, end, offset = 0;

	struct spi_message message;
	struct spi_transfer x[3];

	while (offset < len) {
		if (offset + RT5677_SPI_BUF_LEN <= len)
			end = RT5677_SPI_BUF_LEN;
		else
			end = len % RT5677_SPI_BUF_LEN;

		write_buf[0] = spi_cmd;
		write_buf[1] = ((addr + offset) & 0xff000000) >> 24;
		write_buf[2] = ((addr + offset) & 0x00ff0000) >> 16;
		write_buf[3] = ((addr + offset) & 0x0000ff00) >> 8;
		write_buf[4] = ((addr + offset) & 0x000000ff) >> 0;

		spi_message_init(&message);
		memset(x, 0, sizeof(x));

		x[0].len = 5;
		x[0].tx_buf = write_buf;
		spi_message_add_tail(&x[0], &message);

		x[1].len = 4;
		x[1].tx_buf = write_buf;
		spi_message_add_tail(&x[1], &message);

		x[2].len = len;
		x[2].rx_buf = rxbuf + offset;
		spi_message_add_tail(&x[2], &message);

		status = spi_sync(g_spi, &message);

		if (status)
			return false;

		offset += RT5677_SPI_BUF_LEN;
	}

	for (i = 0; i < len; i += 8) {
		write_buf[0] = rxbuf[i + 0];
		write_buf[1] = rxbuf[i + 1];
		write_buf[2] = rxbuf[i + 2];
		write_buf[3] = rxbuf[i + 3];
		write_buf[4] = rxbuf[i + 4];
		write_buf[5] = rxbuf[i + 5];
		write_buf[6] = rxbuf[i + 6];
		write_buf[7] = rxbuf[i + 7];

		rxbuf[i + 0] = write_buf[7];
		rxbuf[i + 1] = write_buf[6];
		rxbuf[i + 2] = write_buf[5];
		rxbuf[i + 3] = write_buf[4];
		rxbuf[i + 4] = write_buf[3];
		rxbuf[i + 5] = write_buf[2];
		rxbuf[i + 6] = write_buf[1];
		rxbuf[i + 7] = write_buf[0];
	}

	return true;
}
EXPORT_SYMBOL_GPL(rt5677_spi_burst_read);

/**
 * rt5677_spi_burst_write - Write data to SPI by rt5677 dsp memory address.
 * @addr: Start address.
 * @txbuf: Data Buffer for writng.
 * @len: Data length, it must be a multiple of 8.
 *
 *
 * Returns true for success.
 */
int rt5677_spi_burst_write(u32 addr, const u8 *txbuf, size_t len)
{
	u8 spi_cmd = RT5677_SPI_CMD_BURST_WRITE;
	u8 *write_buf;
	unsigned int i, end, offset = 0;

	write_buf = kmalloc(RT5677_SPI_BUF_LEN + 6, GFP_KERNEL);

	if (write_buf == NULL)
		return -ENOMEM;

	while (offset < len) {
		if (offset + RT5677_SPI_BUF_LEN <= len)
			end = RT5677_SPI_BUF_LEN;
		else
			end = len % RT5677_SPI_BUF_LEN;

		write_buf[0] = spi_cmd;
		write_buf[1] = ((addr + offset) & 0xff000000) >> 24;
		write_buf[2] = ((addr + offset) & 0x00ff0000) >> 16;
		write_buf[3] = ((addr + offset) & 0x0000ff00) >> 8;
		write_buf[4] = ((addr + offset) & 0x000000ff) >> 0;

		for (i = 0; i < end; i += 8) {
			write_buf[i + 12] = txbuf[offset + i + 0];
			write_buf[i + 11] = txbuf[offset + i + 1];
			write_buf[i + 10] = txbuf[offset + i + 2];
			write_buf[i +  9] = txbuf[offset + i + 3];
			write_buf[i +  8] = txbuf[offset + i + 4];
			write_buf[i +  7] = txbuf[offset + i + 5];
			write_buf[i +  6] = txbuf[offset + i + 6];
			write_buf[i +  5] = txbuf[offset + i + 7];
		}

		write_buf[end + 5] = spi_cmd;

		rt5677_spi_write(write_buf, end + 6);

		offset += RT5677_SPI_BUF_LEN;
	}

	kfree(write_buf);

	return 0;
}
EXPORT_SYMBOL_GPL(rt5677_spi_burst_write);

static int rt5677_spi_probe(struct spi_device *spi)
{
	g_spi = spi;
	printk("%s\n", __func__);
	return 0;
}

static const struct spi_device_id rt5677_spi_ids[] = {
	{ "rt5677", RT5677 },
	{ },
};
MODULE_DEVICE_TABLE(spi, rt5677_spi_ids);

static struct spi_driver rt5677_spi_driver = {
	.driver = {
		.name = "rt5677",
		.owner = THIS_MODULE,
		//.pm	= &rt5677_pm_ops,
	},
	.probe = rt5677_spi_probe,
	.id_table = rt5677_spi_ids,
};
module_spi_driver(rt5677_spi_driver);

static struct spi_board_info __initdata rt5677_board_info = {
	.platform_data = NULL,
};

static void rt5677_register_spi_boardinfo(void)
{
    void *pdata = NULL;
	int ret = -1;

    memset(&rt5677_board_info, 0, sizeof(rt5677_board_info));
    strncpy(rt5677_board_info.modalias, "rt5677", sizeof("rt5677"));
    rt5677_board_info.irq = 0;
    rt5677_board_info.bus_num = 1;
    rt5677_board_info.chip_select = 0;
    rt5677_board_info.max_speed_hz = 5000000;
    pdata = NULL;

    rt5677_board_info.platform_data = pdata;

    pr_info("SPI bus=%d, name=%16.16s, irq=0x%2x, max_freq=%d, cs=%d, pdata=0x%x\n",
                rt5677_board_info.bus_num,
                rt5677_board_info.modalias,
                rt5677_board_info.irq,
                rt5677_board_info.max_speed_hz,
                rt5677_board_info.chip_select,
		rt5677_board_info.platform_data);

    ret = spi_register_board_info(&rt5677_board_info, 1);
	printk("%s, ret = %d\n", __func__, ret);
}

static int __init rt5677_spi_init(void)
{
	int ret;

	rt5677_register_spi_boardinfo();

	ret = gpio_request_one(GPIO_CODEC_RST_N, GPIOF_DIR_OUT | GPIOF_INIT_HIGH, "codec ldo en");
	gpio_set_value_cansleep(GPIO_CODEC_RST_N , 1);

	ret = gpio_request_one(GPIO_CODEC_LDO_ENABLE, GPIOF_DIR_OUT | GPIOF_INIT_HIGH, "codec rst");
        gpio_set_value_cansleep(GPIO_CODEC_LDO_ENABLE , 1);

	return 0;
}
device_initcall(rt5677_spi_init);


MODULE_DESCRIPTION("ASoC RT5677 SPI driver");
MODULE_AUTHOR("Oder Chiou <oder_chiou@realtek.com>");
MODULE_LICENSE("GPL v2");
