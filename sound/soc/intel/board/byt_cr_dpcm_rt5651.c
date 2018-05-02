/*
 *  byt_bl_rt5651.c - ASoc Machine driver for Intel BYT-CR platform
 *
 *  Copyright (C) 2014 Intel Corp
 *  Author: Ola Lilja <ola.lilja@intel.com>
 *  This file is modified from byt_bl_rt5651.c written by:
 *  Author: Omair Mohammed Abdullah <omair.m.abdullah@intel.com>
 *  Author: Govind Singh <govind.singh@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/vlv2_plat_clock.h>
#include <linux/input.h>
#include <linux/mfd/intel_soc_pmic.h>
#include <asm/intel-mid.h>
#include <asm/platform_byt_audio.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include "../../codecs/rt5651.h"

#include "byt_cr_board_configs.h"

#define BYT_PLAT_CLK_3_HZ	25000000

#define BYT_JD_INTR_DEBOUNCE            0

#define VLV2_PLAT_CLK_AUDIO	3
#define PLAT_CLK_FORCE_ON	1
#define PLAT_CLK_FORCE_OFF	2

#define BYT_T_JACK_RECHECK	300 /* ms */
#define BYT_N_JACK_RECHECK	5
#define BYT_T_BUTTONS_RECHECK	100 /* ms */

/* 0 = 25MHz from crystal, 1 = 19.2MHz from PLL */
#define PLAT_CLK_FREQ_XTAL	0

enum {
	RT5651_GPIO_JD_INT,
	RT5651_GPIO_JD_INT2,
	RT5651_GPIO_JACK_SWITCH,
	RT5651_GPIO_ALC105_RESET,
	RT5651_GPIO_JD_BUTTONS,
};

#define RT5651_GPIO_NA		-1

struct rt5651_gpios {
	int jd_int_gpio;
	int jd_int2_gpio;
	int jd_buttons_gpio;
	int debug_mux_gpio;
	int alc105_reset_gpio;
};

struct byt_drvdata {
	struct snd_soc_jack jack;
	struct delayed_work hs_jack_recheck;
	struct delayed_work hs_buttons_recheck;
	int t_jack_recheck;
	int t_buttons_recheck;
	int jack_hp_count;
	struct mutex jack_mlock;
	struct rt5651_gpios gpios;
};

static inline void byt_force_enable_pin(struct snd_soc_codec *codec,
			 const char *bias_widget, bool enable)
{
	if (enable)
		snd_soc_dapm_force_enable_pin(&codec->dapm, bias_widget);
	else
		snd_soc_dapm_disable_pin(&codec->dapm, bias_widget);

	pr_debug("%s: %s widget %s.\n", __func__,
		enable ? "Enabled" : "Disabled", bias_widget);
	snd_soc_dapm_sync(&codec->dapm);
}
static inline void byt_set_mic_bias_ldo(struct snd_soc_codec *codec,
				bool enable, struct mutex *mlock)
{
	bool was_locked = (mlock->count.counter == 0);

	if (was_locked)
		mutex_unlock(mlock);

	if (enable) {
		byt_force_enable_pin(codec, "micbias1", true);
		byt_force_enable_pin(codec, "LDO2", true);
	} else {
		byt_force_enable_pin(codec, "micbias1", false);
		byt_force_enable_pin(codec, "LDO2", false);
	}

	if (was_locked)
		mutex_lock(mlock);

	snd_soc_dapm_sync(&codec->dapm);

}

/* HS-button handling */

static int byt_hs_buttons_check(struct byt_drvdata *drvdata, bool is_recheck)
{
	struct snd_soc_jack *jack = &drvdata->jack;
	struct gpio_desc *desc;
	int val;

	pr_debug("%s: Enter (jack->status = %d).\n", __func__, jack->status);

	if (!(jack->status & SND_JACK_MICROPHONE)) {
		pr_debug("%s: Button-interrupt in non-HS mode.\n", __func__);
		return jack->status;
	}

	desc = gpio_to_desc(drvdata->gpios.jd_buttons_gpio);
	val = gpiod_get_value(desc);
	if ((val == 0) && (jack->status & SND_JACK_BTN_0)) {
		if (!is_recheck) {
			pr_debug("%s: Button release.\n", __func__);
			jack->status &= ~SND_JACK_BTN_0;
		} else
			pr_warn("%s: Fishy interrupt detected.\n", __func__);
	} else if ((val == 1) && !(jack->status & SND_JACK_BTN_0)) {
		if (!is_recheck) {
			pr_debug("%s: Button press (preliminary).\n", __func__);
			schedule_delayed_work(&drvdata->hs_buttons_recheck,
				drvdata->t_buttons_recheck);
		} else {
			jack->status |= SND_JACK_BTN_0;
			pr_debug("%s: Button press.\n", __func__);
		}
	}

	return jack->status;
}

static int byt_hs_buttons_interrupt(void *data)
{
	struct byt_drvdata *drvdata = (struct byt_drvdata *)data;
	int status;

	status = cancel_delayed_work_sync(&drvdata->hs_buttons_recheck);
	if (status)
		pr_debug("%s: Delayed work cancelled!\n", __func__);

	mutex_lock(&drvdata->jack_mlock);
	pr_debug("%s: Enter.\n", __func__);

	status = byt_hs_buttons_check(drvdata, false);

	mutex_unlock(&drvdata->jack_mlock);
	return status;
}

static void byt_hs_buttons_recheck(struct work_struct *work)
{
	struct byt_drvdata *drvdata =
		container_of(work, struct byt_drvdata, hs_buttons_recheck.work);
	struct snd_soc_jack *jack = &drvdata->jack;
	int status;

	mutex_lock(&drvdata->jack_mlock);
	pr_debug("%s: Enter.\n", __func__);

	status = byt_hs_buttons_check(drvdata, true);
	snd_soc_jack_report(jack, status, SND_JACK_BTN_0);

	mutex_unlock(&drvdata->jack_mlock);
}

/* HS-jack handling */

/* Returns true if headset/headphones is inserted */
static inline bool byt_hs_inserted(struct byt_drvdata *drvdata)
{
	bool val;
	const struct gpio_desc *desc;

	desc = gpio_to_desc(drvdata->gpios.jd_int2_gpio);
	val = (bool)gpiod_get_value(desc);

	/* TEMP for MRD7 until active_low is working properly with ACPI */
	if (drvdata->gpios.jd_int2_gpio == RT5651_GPIO_NA)
		val = !val;

	pr_info("%s: val = %d (pin = %d, active_low = %d)\n", __func__, val,
		drvdata->gpios.jd_int_gpio, gpiod_is_active_low(desc));

	return val;
}

static int byt_hs_jack_check(struct byt_drvdata *drvdata, bool is_recheck)
{
	struct snd_soc_jack *jack = &drvdata->jack;
	struct snd_soc_codec *codec = jack->codec;
	int inserted, status;

	pr_debug("%s: Enter (jack->status = %d).\n", __func__, jack->status);

	inserted = byt_hs_inserted(drvdata);

	if (inserted) {
		if (!(jack->status & SND_JACK_HEADPHONE)) {
			status = rt5651_headset_detect(codec, true);
			if (status == RT5651_HEADPHO_DET) {
				if (drvdata->jack_hp_count > 0) {
					pr_debug("%s: Headphones detected (preliminary, %d).\n",
						__func__,
						drvdata->jack_hp_count);
					drvdata->jack_hp_count--;
					schedule_delayed_work(
						&drvdata->hs_jack_recheck,
						drvdata->t_jack_recheck);
				} else {
					pr_info("%s: Headphones present.\n",
						__func__);
					jack->status |= SND_JACK_HEADPHONE;
					drvdata->jack_hp_count =
						BYT_N_JACK_RECHECK;
				}
			} else if (status == RT5651_HEADSET_DET) {
				pr_info("%s: Headset present.\n", __func__);
				byt_set_mic_bias_ldo(codec, true,
					&drvdata->jack_mlock);
				jack->status |= SND_JACK_HEADSET;
				drvdata->jack_hp_count = BYT_N_JACK_RECHECK;
			} else
				pr_warn("%s: No valid accessory present!\n",
					__func__);
		}
	} else {
		if (jack->status & SND_JACK_HEADPHONE) {
			if (jack->status & SND_JACK_MICROPHONE) {
				jack->status &= ~SND_JACK_HEADSET;
				byt_set_mic_bias_ldo(codec, false,
					&drvdata->jack_mlock);
				pr_info("%s: Headset removed.\n", __func__);
			} else {
				jack->status &= ~SND_JACK_HEADPHONE;
				pr_info("%s: Headphone removed.\n", __func__);
			}
		} else
			pr_warn("%s: Remove-interrupt while no accessory present!\n",
					__func__);
	}

	return jack->status;
}

static int byt_jack_interrupt(void *data)
{
	struct byt_drvdata *drvdata = (struct byt_drvdata *)data;
	int status;

	status = cancel_delayed_work_sync(&drvdata->hs_jack_recheck);
	if (status)
		pr_debug("%s: Delayed work cancelled!\n", __func__);

	mutex_lock(&drvdata->jack_mlock);
	pr_debug("%s: Enter.\n", __func__);

	status = byt_hs_jack_check(drvdata, false);

	mutex_unlock(&drvdata->jack_mlock);
	return status;
}

static void byt_hs_jack_recheck(struct work_struct *work)
{
	struct byt_drvdata *drvdata =
		container_of(work, struct byt_drvdata, hs_jack_recheck.work);
	struct snd_soc_jack *jack = &drvdata->jack;
	int status;

	mutex_lock(&drvdata->jack_mlock);
	pr_debug("%s: Enter.\n", __func__);

	status = byt_hs_jack_check(drvdata, true);
	snd_soc_jack_report(jack, status, SND_JACK_HEADSET);

	mutex_unlock(&drvdata->jack_mlock);
}

/* Jack GPIO definitions */

static struct snd_soc_jack_gpio hs_gpio[] = {
	{
		.name                   = "byt-jd-int",
		.report                 = SND_JACK_HEADSET,
		.debounce_time          = BYT_JD_INTR_DEBOUNCE,
		.jack_status_check      = byt_jack_interrupt,
	},
	{
		.name                   = "byt-hs-but-int",
		.report                 = SND_JACK_BTN_0,
		.debounce_time          = BYT_JD_INTR_DEBOUNCE,
		.jack_status_check      = byt_hs_buttons_interrupt,
	},

};

static inline struct snd_soc_codec *byt_get_codec(struct snd_soc_card *card)
{
	bool found = false;
	struct snd_soc_codec *codec;

	list_for_each_entry(codec, &card->codec_dev_list, card_list) {
		if (!strstr(codec->name, "i2c-10EC5651:00"))
			continue;
		else {
			found = true;
			break;
		}
	}
	if (found == false) {
		pr_err("%s: Codec not found!\n", __func__);
		return NULL;
	}
	return codec;
}

static int byt_set_alc105(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *k, int  event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct byt_drvdata *drvdata = snd_soc_card_get_drvdata(dapm->card);
	struct snd_soc_codec *codec;
	struct gpio_desc *desc;

	pr_debug("%s: Enter.\n", __func__);

	codec = byt_get_codec(card);
	if (!codec) {
		pr_err("%s: Codec not found; Unable to set platform clock\n",
			__func__);
		return -EIO;
	}

	desc = gpio_to_desc(drvdata->gpios.alc105_reset_gpio);
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		pr_debug("%s: ALC105 ON.\n", __func__);
		gpiod_set_value(desc, 0);
	} else {
		pr_debug("%s: ALC105 OFF.\n", __func__);
		gpiod_set_value(desc, 1);
	}

	return 0;
}

static int byt_set_platform_clock(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *k, int  event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct snd_soc_codec *codec;

	pr_debug("%s: Enter.\n", __func__);

	codec = byt_get_codec(card);
	if (!codec) {
		pr_err("%s: Codec not found; Unable to set platform clock\n",
			__func__);
		return -EIO;
	}
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		vlv2_plat_configure_clock(VLV2_PLAT_CLK_AUDIO,
				PLAT_CLK_FORCE_ON);

		pr_debug("%s: Platform clk turned ON\n", __func__);
		snd_soc_write(codec, RT5651_ADDA_CLK1, 0x0014);
	} else {
		/* Set codec clock source to internal clock before
		   turning off the platform clock. Codec needs clock
		   for Jack detection and button press */
		snd_soc_write(codec, RT5651_ADDA_CLK1, 0x7774);
		/* snd_soc_codec_set_sysclk(codec, RT5651_SCLK_S_RCCLK,
				0, 0, SND_SOC_CLOCK_IN); */
		vlv2_plat_configure_clock(VLV2_PLAT_CLK_AUDIO,
				PLAT_CLK_FORCE_OFF);

		pr_debug("%s: Platform clk turned OFF\n", __func__);
	}

	return 0;
}

static const struct snd_soc_dapm_widget byt_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Int Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", byt_set_alc105),
	SND_SOC_DAPM_SUPPLY("Platform Clock", SND_SOC_NOPM, 0, 0,
			byt_set_platform_clock, SND_SOC_DAPM_PRE_PMU |
			SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route byt_audio_map[] = {
	/* Playback */

	{"Ext Spk", NULL, "Platform Clock"},
	{"Headphone", NULL, "Platform Clock"},

	{"AIF1 Playback", NULL, "ssp2 Tx"},
	{"ssp2 Tx", NULL, "codec_out0"},
	{"ssp2 Tx", NULL, "codec_out1"},

	{"Headphone", NULL, "HPOL"},
	{"Headphone", NULL, "HPOR"},

	{"Ext Spk", NULL, "LOUTL"},
	{"Ext Spk", NULL, "LOUTR"},

	/* Capture */

	{"Headset Mic", NULL, "Platform Clock"},
	{"IN1P", NULL, "Headset Mic"},

	{"Int Mic", NULL, "Platform Clock"},
	{"LDO2", NULL, "Int Mic"},
	{"micbias1", NULL, "Int Mic"},

	{"codec_in0", NULL, "ssp2 Rx"},
	{"codec_in1", NULL, "ssp2 Rx"},
	{"ssp2 Rx", NULL, "AIF1 Capture"},

	{"Dummy Playback", NULL, "ssp1 Tx"},
	{"ssp1 Rx", NULL, "Dummy Capture"},

	{ "ssp1 Tx", NULL, "bt_fm_out"},
	{ "bt_fm_in", NULL, "ssp1 Rx" },
};

static const struct snd_soc_dapm_route byt_audio_map_default[] = {
	{"IN3P", NULL, "micbias1"},
};

static const struct snd_kcontrol_new byt_mc_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
	SOC_DAPM_PIN_SWITCH("Ext Spk"),
};

/* Sets dai format and pll */
static int byt_set_dai_fmt_pll(struct snd_soc_dai *codec_dai,
					int source, unsigned int freq_out)
{
	int ret;
	unsigned int fmt;

	pr_debug("%s: Enter.\n", __func__);

	/* Codec slave-mode */
	fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBS_CFS;

	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		pr_err("%s: Failed to set codec-DAI cfg (%d)!\n", __func__,
			ret);
		return ret;
	}

	ret = snd_soc_dai_set_pll(codec_dai, 0, source,
			BYT_PLAT_CLK_3_HZ, freq_out * 512);
	if (ret < 0) {
		pr_err("%s: Failed to set codec-PLL (%d)!\n", __func__, ret);
		return ret;
	}

	snd_soc_codec_set_sysclk(codec_dai->codec, RT5651_SCLK_S_PLL1, 0,
				BYT_PLAT_CLK_3_HZ, SND_SOC_CLOCK_IN);

	return 0;
}

static int byt_aif1_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	pr_debug("%s: Enter.\n", __func__);

	if (strncmp(codec_dai->name, "rt5651-aif1", 11))
		return 0;

	/* Setecodec DAI confinuration */
	return byt_set_dai_fmt_pll(codec_dai, RT5651_PLL1_S_MCLK,
			params_rate(params));
}

static struct snd_soc_ops byt_be_ssp2_ops = {
	.hw_params = byt_aif1_hw_params,
};

static int byt_set_bias_level(struct snd_soc_card *card,
				struct snd_soc_dapm_context *dapm,
				enum snd_soc_bias_level level)
{
	pr_debug("%s: Enter.\n", __func__);

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
	case SND_SOC_BIAS_OFF:
		break;
	default:
		pr_err("%s: Invalid bias level=%d\n", __func__, level);
		return -EINVAL;
	}

	if (&card->dapm == dapm)
		card->dapm.bias_level = level;
	pr_debug("%s: card: %s, bias_level: %u\n", __func__, card->name,
			card->dapm.bias_level);

	return 0;
}

static void byt_export_gpio(struct gpio_desc *desc, char *name)
{
	int ret = gpiod_export(desc, true);
	if (ret)
		pr_debug("%s: Unable to export GPIO%d (%s)! Returned %d.\n",
			__func__, desc_to_gpio(desc), name, ret);
}

static int byt_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret, dir, count;
	struct snd_soc_codec *codec;
	struct snd_soc_card *card = runtime->card;
	struct byt_drvdata *drvdata = snd_soc_card_get_drvdata(runtime->card);
	struct gpio_desc *desc;

	pr_debug("%s: Enter.\n", __func__);

	codec = byt_get_codec(card);
	if (!codec) {
		pr_err("%s: Codec not found!\n", __func__);
		return -EIO;
	}

	card->dapm.idle_bias_off = true;

	/* Threshold base = 2000uA; scale factor = 0.5 =>
	   effective threshold of 1000uA for micbias resistor for 2.2K */
	rt5651_config_ovcd_thld(codec, RT5651_MIC1_OVTH_2000UA,
			RT5651_MIC_OVCD_SF_0P5);

	mutex_init(&drvdata->jack_mlock);


	/* GPIOs */

	desc = devm_gpiod_get_index(codec->dev, NULL, RT5651_GPIO_JD_INT);
	if (!IS_ERR(desc)) {
		drvdata->gpios.jd_int_gpio = desc_to_gpio(desc);
		byt_export_gpio(desc, "JD-int");

		pr_info("%s: GPIOs - JD-int: %d (pol = %d, val = %d)\n",
			__func__, drvdata->gpios.jd_int_gpio,
			gpiod_is_active_low(desc), gpiod_get_value(desc));

		devm_gpiod_put(codec->dev, desc);
	} else {
		drvdata->gpios.jd_int_gpio = RT5651_GPIO_NA;
		pr_err("%s: GPIOs - JD-int: Not present!\n", __func__);
	}

	desc = devm_gpiod_get_index(codec->dev, NULL, RT5651_GPIO_JD_INT2);
	if (!IS_ERR(desc)) {
		drvdata->gpios.jd_int2_gpio = desc_to_gpio(desc);
		byt_export_gpio(desc, "JD-int2");

		pr_info("%s: GPIOs - JD-int2: %d (pol = %d, val = %d)\n",
			__func__, drvdata->gpios.jd_int2_gpio,
			gpiod_is_active_low(desc), gpiod_get_value(desc));

		devm_gpiod_put(codec->dev, desc);
	} else {
		drvdata->gpios.jd_int2_gpio = RT5651_GPIO_NA;
		pr_warn("%s: GPIOs - JD-int2: Not present!\n", __func__);
	}

	desc = devm_gpiod_get_index(codec->dev, NULL, RT5651_GPIO_JACK_SWITCH);
	if (!IS_ERR(desc)) {
		drvdata->gpios.debug_mux_gpio = desc_to_gpio(desc);
		byt_export_gpio(desc, "debug-mux");

		dir = gpiod_get_direction(desc);
		if (dir < 0)
			pr_warn("%s: Unable to get direction for GPIO%d from GPIO-driver (err = %d)!\n",
				__func__, drvdata->gpios.debug_mux_gpio, dir);
		else if (dir == GPIOF_DIR_IN)
			pr_warn("%s: Direction for GPIO%d is set to input (dir = %d)! Headset-path will have no audio!\n",
				__func__, drvdata->gpios.debug_mux_gpio, dir);
		else
			pr_debug("%s: Direction for GPIO%d is set to output (dir = %d)!\n",
				__func__, drvdata->gpios.debug_mux_gpio, dir);

		pr_info("%s: GPIOs - Debug-mux: %d (dir = %d, val = %d)\n",
			__func__, drvdata->gpios.debug_mux_gpio, dir,
			gpiod_get_value(desc));

		devm_gpiod_put(codec->dev, desc);
	} else {
		drvdata->gpios.debug_mux_gpio = RT5651_GPIO_NA;
		pr_warn("%s: GPIOs - Debug-mux: Not present!\n", __func__);
	}

	desc = devm_gpiod_get_index(codec->dev, NULL, RT5651_GPIO_ALC105_RESET);
	if (!IS_ERR(desc)) {
		drvdata->gpios.alc105_reset_gpio = desc_to_gpio(desc);
		byt_export_gpio(desc, "ALC105");

		pr_info("%s: GPIOs - ALC105: %d (pol = %d, val = %d)\n",
			__func__, drvdata->gpios.alc105_reset_gpio,
			gpiod_is_active_low(desc), gpiod_get_value(desc));

		devm_gpiod_put(codec->dev, desc);

	} else {
		drvdata->gpios.alc105_reset_gpio = RT5651_GPIO_NA;
		pr_warn("%s: GPIOs - ALC105 reset: Not present!\n", __func__);
	}

	desc = devm_gpiod_get_index(codec->dev, NULL, RT5651_GPIO_JD_BUTTONS);
	if (!IS_ERR(desc)) {
		drvdata->gpios.jd_buttons_gpio = desc_to_gpio(desc);
		byt_export_gpio(desc, "JD-buttons");

		pr_info("%s: GPIOs - JD-buttons: %d (pol = %d, val = %d)\n",
			__func__, drvdata->gpios.jd_buttons_gpio,
			gpiod_is_active_low(desc), gpiod_get_value(desc));

		devm_gpiod_put(codec->dev, desc);

	} else {
		drvdata->gpios.jd_buttons_gpio = RT5651_GPIO_NA;
		pr_warn("%s: GPIOs - JD-buttons: Not present!\n", __func__);
	}

	/* BYT-CR Audio Jack */
	count = 0;
	if (drvdata->gpios.jd_int2_gpio != RT5651_GPIO_NA) {
		hs_gpio[count].gpio = drvdata->gpios.jd_int2_gpio;
		hs_gpio[count].data = drvdata;
		count++;
	}

	if (drvdata->gpios.jd_buttons_gpio != RT5651_GPIO_NA) {
		hs_gpio[count].gpio = drvdata->gpios.jd_buttons_gpio;
		hs_gpio[count].data = drvdata;
		count++;
	}

	drvdata->t_jack_recheck = msecs_to_jiffies(BYT_T_JACK_RECHECK);
	INIT_DELAYED_WORK(&drvdata->hs_jack_recheck, byt_hs_jack_recheck);
	drvdata->t_buttons_recheck = msecs_to_jiffies(BYT_T_BUTTONS_RECHECK);
	INIT_DELAYED_WORK(&drvdata->hs_buttons_recheck, byt_hs_buttons_recheck);
	drvdata->jack_hp_count = 5;

	if (!count) {
		/* Someting wrong with ACPI configuration */
		WARN(1, "Wrong ACPI configuration !");
	} else {
		ret = snd_soc_jack_new(codec, "BYT-CR Audio Jack",
				SND_JACK_HEADSET | SND_JACK_BTN_0,
				 &drvdata->jack);
		if (ret) {
			pr_err("%s: snd_soc_jack_new failed (ret = %d)!\n",
				__func__, ret);
			return ret;
		}

		ret = snd_soc_jack_add_gpios(&drvdata->jack, count,
				&hs_gpio[0]);
		if (ret) {
			pr_err("%s: snd_soc_jack_add_gpios failed (ret = %d)!\n",
				__func__, ret);
			return ret;
		}

		snd_jack_set_key(drvdata->jack.jack, SND_JACK_BTN_0, KEY_MEDIA);
	}

	ret = snd_soc_add_card_controls(card, byt_mc_controls,
					ARRAY_SIZE(byt_mc_controls));
	if (ret) {
		pr_err("%s: Unable to add card controls!\n", __func__);
		return ret;
	}
	ret = snd_soc_dapm_sync(&card->dapm);
	if (ret) {
		pr_err("%s: snd_soc_dapm_sync failed!\n", __func__);
		return ret;
	}
	return ret;
}

/* AIF1 */

static unsigned int rates_8000_16000[] = {
	8000,
	16000,
};

static struct snd_pcm_hw_constraint_list constraints_8000_16000 = {
	.count = ARRAY_SIZE(rates_8000_16000),
	.list = rates_8000_16000,
};
static unsigned int rates_48000[] = {
	48000,
};

static struct snd_pcm_hw_constraint_list constraints_48000 = {
	.count = ARRAY_SIZE(rates_48000),
	.list  = rates_48000,
};

static int byt_aif1_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&constraints_48000);
}

static struct snd_soc_ops byt_aif1_ops = {
	.startup = byt_aif1_startup,
};

static const struct snd_soc_pcm_stream byt_dai_params = {
	.formats = SNDRV_PCM_FMTBIT_S24_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
};

static int byt_8k_16k_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
		SNDRV_PCM_HW_PARAM_RATE,
		&constraints_8000_16000);
}

static struct snd_soc_ops byt_8k_16k_ops = {
	.startup = byt_8k_16k_startup,
	.hw_params = byt_aif1_hw_params,
};

static struct snd_soc_dai_link byt_dailink[] = {
	[BYT_DPCM_AUDIO] = {
		.name = "Baytrail Audio Port",
		.stream_name = "Baytrail Audio",
		.cpu_dai_name = "Headset-cpu-dai",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "sst-platform",
		.init = byt_init,
		.ignore_suspend = 1,
		.dynamic = 1,
		.ops = &byt_aif1_ops,
	},
	[BYT_DPCM_VOIP] = {
		.name = "Baytrail VOIP Port",
		.stream_name = "Baytrail Voip",
		.cpu_dai_name = "Voip-cpu-dai",
		.platform_name = "sst-platform",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &byt_8k_16k_ops,
		.dynamic = 1,
	},

	/* CODEC<->CODEC link */
	{
		.name = "Baytrail BTFM-Loop Port",
		.stream_name = "Baytrail BTFM-Loop",
		.cpu_dai_name = "ssp1-port",
		.platform_name = "sst-platform",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.params = &byt_dai_params,
		.dsp_loopback = true,
	},

	/* Backends */
	{
		.name = "SSP0-Codec",
		.be_id = 1,
		.cpu_dai_name = "ssp2-port",
		.platform_name = "sst-platform",
		.no_pcm = 1,
		.codec_dai_name = "rt5651-aif1",
		.codec_name = "i2c-10EC5651:00",
		.ignore_suspend = 1,
		.ops = &byt_be_ssp2_ops,
	},
	{
		.name = "SSP1-BTFM",
		.be_id = 2,
		.cpu_dai_name = "snd-soc-dummy-dai",
		.platform_name = "snd-soc-dummy",
		.no_pcm = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.ignore_suspend = 1,
	},
};

#ifdef CONFIG_PM_SLEEP
static int snd_byt_prepare(struct device *dev)
{
	pr_debug("%s: Enter.\n", __func__);

	return snd_soc_suspend(dev);
}

static void snd_byt_complete(struct device *dev)
{
	pr_debug("%s: Enter.\n", __func__);

	snd_soc_resume(dev);
}

static int snd_byt_poweroff(struct device *dev)
{
	pr_debug("%s: Enter.\n", __func__);

	return snd_soc_poweroff(dev);
}
#else
#define snd_byt_prepare NULL
#define snd_byt_complete NULL
#define snd_byt_poweroff NULL
#endif

/* SoC card */
static struct snd_soc_card snd_soc_card_byt_default = {
	.name = "bytcr-rt5651",
	.dai_link = byt_dailink,
	.num_links = ARRAY_SIZE(byt_dailink),
	.set_bias_level = byt_set_bias_level,
	.dapm_widgets = byt_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(byt_dapm_widgets),
	.dapm_routes = byt_audio_map,
	.num_dapm_routes = ARRAY_SIZE(byt_audio_map),
};

static int snd_byt_mc_probe(struct platform_device *pdev)
{
	int ret_val = 0;
	struct byt_drvdata *drvdata;
	struct snd_soc_card *card;
	const struct snd_soc_dapm_route *routes;
	const struct board_config *conf;

	pr_debug("%s: Enter.\n", __func__);

	drvdata = devm_kzalloc(&pdev->dev, sizeof(*drvdata), GFP_ATOMIC);
	if (!drvdata) {
		pr_err("Allocation failed!\n");
		return -ENOMEM;
	}

	/* Get board-specific HW-settings */
	conf = get_board_config(get_mc_link());
	switch (conf->idx) {
	case RT5651_ANCHOR8:
	case RT5651_DEFAULT:
		card = &snd_soc_card_byt_default;
		routes = &byt_audio_map_default[0];
		break;
	default:
		BUG_ON(true);
	}

	/* Register the soc-card */
	card->dev = &pdev->dev;
	snd_soc_card_set_drvdata(card, drvdata);
	ret_val = snd_soc_register_card(card);
	if (ret_val) {
		pr_err("snd_soc_register_card failed %d\n", ret_val);
		return ret_val;
	}
	platform_set_drvdata(pdev, card);

	ret_val = snd_soc_dapm_add_routes(&card->dapm, routes, 1);
	if (ret_val) {
		pr_err("%s: Failed to add board-specific routes!\n",
			__func__);
		return ret_val;
	}

	if (conf->idx == RT5651_ANCHOR8) {
		pr_err("%s: Setting special-bit for ANCHOR8-board.\n",
			__func__);
		snd_soc_update_bits(byt_get_codec(card), RT5651_JD_CTRL1,
				RT5651_JD_MASK, RT5651_JD_JD1_IN4P);
	}

	pr_debug("%s: Exit.\n", __func__);
	return ret_val;
}

static void snd_byt_unregister_jack(struct byt_drvdata *drvdata)
{
	cancel_delayed_work_sync(&drvdata->hs_jack_recheck);
	snd_soc_jack_free_gpios(&drvdata->jack, 2, hs_gpio);
}

static int snd_byt_mc_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct byt_drvdata *drvdata = snd_soc_card_get_drvdata(soc_card);

	pr_debug("%s: Enter.\n", __func__);

	snd_byt_unregister_jack(drvdata);
	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(soc_card);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static void snd_byt_mc_shutdown(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct byt_drvdata *drvdata = snd_soc_card_get_drvdata(soc_card);

	pr_debug("%s: Enter.\n", __func__);

	snd_byt_unregister_jack(drvdata);
}

static const struct dev_pm_ops snd_byt_mc_pm_ops = {
	.prepare = snd_byt_prepare,
	.complete = snd_byt_complete,
	.poweroff = snd_byt_poweroff,
};

static struct platform_driver snd_byt_mc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "byt_rt5651",
		.pm = &snd_byt_mc_pm_ops,
	},
	.probe = snd_byt_mc_probe,
	.remove = snd_byt_mc_remove,
	.shutdown = snd_byt_mc_shutdown,
};

static int __init snd_byt_driver_init(void)
{
	int ret;

	ret = platform_driver_register(&snd_byt_mc_driver);
	if (ret)
		pr_err("%s: Failed to register BYT-CR-RT5651 Machine driver!\n",
			__func__);
	else
		pr_info("%s: BYT-CR-RT5651 Machine driver registered.\n",
			__func__);
	return ret;
}
late_initcall(snd_byt_driver_init);

static void __exit snd_byt_driver_exit(void)
{
	pr_debug("%s: Enter.\n", __func__);

	platform_driver_unregister(&snd_byt_mc_driver);
}
module_exit(snd_byt_driver_exit);

MODULE_LICENSE("GPL v2");
