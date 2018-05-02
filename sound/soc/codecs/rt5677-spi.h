/*
 * rt5677-spi.h  --  RT5677 ALSA SoC audio codec driver
 *
 * Copyright 2013 Realtek Semiconductor Corp.
 * Author: Oder Chiou <oder_chiou@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __RT5677_SPI_H__
#define __RT5677_SPI_H__

int rt5677_spi_write(u8 *txbuf, size_t len);
int rt5677_spi_burst_read(unsigned int addr, u8 *rxbuf, size_t len);
int rt5677_spi_burst_write(u32 addr, const u8 *txbuf, size_t len);
int rt5677_spi_single_write(u32 addr, unsigned int val, size_t len);

#endif /* __RT5677_SPI_H__ */
