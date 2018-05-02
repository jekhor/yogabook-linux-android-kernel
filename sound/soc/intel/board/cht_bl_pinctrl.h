/* 
 *  Machine driver for rt5677
 *
 *  Copyright (C) 2014 Intel Corporation
 *  Authors:<liyh23@lenovo.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  rt5677 and ts3a227e and speaker PA need the related GPIO to configure on ASOC, so define the MARCO at here.
 */


#define gpio_southwest_NUM 98
#define gpio_north_NUM 73
#define gpio_east_NUM 27
#define gpio_southeast_NUM 86

#define gpio_southwest_base (ARCH_NR_GPIOS-gpio_southwest_NUM) //414
#define gpio_north_base (gpio_southwest_base - gpio_north_NUM) //341
#define gpio_east_base (gpio_north_base - gpio_east_NUM) //314
#define gpio_southeast_base (gpio_east_base - gpio_southeast_NUM)//228

#define UART0_DATAIN 77//491, GPIO number: sw77 , package ball number BJ37. used as the mic_switch_int
#define GPIO_MIC_SWITCH_IRQ (gpio_southwest_base + UART0_DATAIN )


#define LPC_FRAME_N 48//276, GPIO number: se48 , package ball number BJ19. used as the spealer PA  enable
#define GPIO_SPK_EN (gpio_southeast_base + LPC_FRAME_N )


#define SD2_D0 25//253, GPIO number: se25 , package ball number BT10. used as the codec reset
#define GPIO_CODEC_RST_N (gpio_southeast_base + SD2_D0 )

#define SD2_D1 18//246, GPIO number: se18 , package ball number BR11. used as the codec ldo enable
#define GPIO_CODEC_LDO_ENABLE (gpio_southeast_base + SD2_D1 )
