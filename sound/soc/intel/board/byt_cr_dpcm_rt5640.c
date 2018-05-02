/*
 *  byt_cr_dpcm_rt5640.c - ASoc Machine driver for Intel Byt CR platform
 *
 *  Copyright (C) 2014 Intel Corp
 *  Author: Ola Lilja <ola.lilja@inte.com>
 *  This file originates from byt_cr_rt5640.c for byt cr dpcm configuration
 *  written by Subhransu S. Prusty <subhransu.s.prusty@intel.com>
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
#include <linux/dmi.h>
#include <asm/intel-mid.h>
#include <asm/platform_byt_audio.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include "../../codecs/rt5640.h"

#include "byt_cr_board_configs.h"

#define BYT_PLAT_CLK_3_HZ	25000000

#define BYT_JD_INTR_DEBOUNCE	0

#define VLV2_PLAT_CLK_AUDIO	3
#define PLAT_CLK_FORCE_ON	1
#define PLAT_CLK_FORCE_OFF	2

#define BYT_T_JACK_RECHECK	300 /* ms */
#define BYT_N_JACK_RECHECK	5
#define BYT_T_BUTTONS_RECHECK	25 /* ms */

enum {
	RT5640_GPIO_JD_INT,
	RT5640_GPIO_JD_INT2,
	RT5640_GPIO_JACK_SWITCH,
	RT5640_GPIO_JD_BUTTONS,
	RT5640_GPIO_I2S_TRISTATE,
};

#define RT5640_GPIO_NA		-1

struct rt5640_gpios {
	int jd_int_gpio;
	int jd_int2_gpio;
	int jd_buttons_gpio;
	int debug_mux_gpio;
	int i2s_tristate_en_gpio;
	int int_count;
};

struct byt_drvdata {
	const struct board_config *board_cfg;
	struct snd_soc_jack jack;
	struct delayed_work jack_recheck;
	struct delayed_work bp_recheck;
	int t_jack_recheck;
	int t_buttons_recheck;
	int jack_hp_count;
	struct mutex jack_mlock;
	struct rt5640_gpios gpios;
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
}

static inline void byt_set_mic_bias_ldo(struct snd_soc_codec *codec,
				bool enable)
{
	if (enable) {
		byt_force_enable_pin(codec, "micbias1", true);
		byt_force_enable_pin(codec, "LDO2", true);
	} else {
		byt_force_enable_pin(codec, "micbias1", false);
		byt_force_enable_pin(codec, "LDO2", false);
	}

	snd_soc_dapm_sync(&codec->dapm);
}

static inline bool byt_hs_inserted(struct byt_drvdata *drvdata)
{
	bool val;
	const struct gpio_desc *desc;
	int gpio;

	if (drvdata->board_cfg->jack_int_sel == JACK_INT1)
		gpio = drvdata->gpios.jd_int_gpio;
	else /* JACK_INT2 */
		gpio = drvdata->gpios.jd_int2_gpio;

	desc = gpio_to_desc(gpio);
	val = (bool)gpiod_get_value(desc);

	if (drvdata->board_cfg->jack_active_low)
		val = !val;

	pr_info("%s: val = %d (pin = %d, active_low = %d, jack_int_sel = %d)\n",
		__func__, val, gpio, drvdata->board_cfg->jack_active_low,
		drvdata->board_cfg->jack_int_sel);

	return val;
}

static int byt_bp_check(struct byt_drvdata *drvdata, bool is_recheck)
{
	struct snd_soc_jack *jack = &drvdata->jack;
	struct snd_soc_codec *codec = jack->codec;
	int val;
	struct gpio_desc *desc;

	pr_debug("%s: Enter.\n", __func__);

	if (drvdata->board_cfg->jack_bp_sel == JACK_BP_CODEC)
		val = rt5640_check_bp_status(codec);
	else { /* JACK_BP_MICBIAS */
		desc = gpio_to_desc(drvdata->gpios.jd_buttons_gpio);
		val = gpiod_get_value(desc);
	}

	if (((val == 0) && !(jack->status & SND_JACK_BTN_0)) ||
		((val > 0) && (jack->status & SND_JACK_BTN_0))) {
		pr_debug("%s: No change (%d).\n", __func__, val);
		return false;
	}

	if (val == 0) {
		if (!is_recheck) {
			pr_info("%s: Button release.\n", __func__);
			jack->status &= ~SND_JACK_BTN_0;
			return true;
		} else {
			pr_warn("%s: Fishy interrupt detected.\n", __func__);
			return false;
		}
	} else {
		if (!is_recheck) {
			pr_debug("%s: Button press (preliminary).\n", __func__);
			schedule_delayed_work(&drvdata->bp_recheck,
				drvdata->t_buttons_recheck);
			return false;
		} else {
			jack->status |= SND_JACK_BTN_0;
			pr_info("%s: Button press.\n", __func__);
			return true;
		}
	}

	return true;
}

static void byt_bp_recheck(struct work_struct *work)
{
	struct byt_drvdata *drvdata =
		container_of(work, struct byt_drvdata, bp_recheck.work);
	struct snd_soc_jack *jack = &drvdata->jack;

	mutex_lock(&drvdata->jack_mlock);
	pr_debug("%s: Enter.\n", __func__);

	if (byt_bp_check(drvdata, true))
		snd_soc_jack_report(jack, jack->status, SND_JACK_BTN_0);

	mutex_unlock(&drvdata->jack_mlock);
}

static bool byt_jack_check(struct byt_drvdata *drvdata, bool is_recheck,
							bool is_resume_check)
{
	struct snd_soc_jack *jack = &drvdata->jack;
	struct snd_soc_codec *codec = jack->codec;
	int inserted, status;
	bool changed = false;

	pr_debug("%s: Enter (jack->status = %d).\n", __func__, jack->status);

	inserted = byt_hs_inserted(drvdata);
	if (inserted) {
		if (!(jack->status & SND_JACK_HEADPHONE)) {
			status = rt5640_detect_hs_type(codec, true);
			if (status == RT5640_HEADPHO_DET) {
				if (drvdata->jack_hp_count > 0) {
					pr_debug("%s: Headphones detected (preliminary, %d).\n",
						__func__,
						drvdata->jack_hp_count);
					drvdata->jack_hp_count--;
					schedule_delayed_work(
						&drvdata->jack_recheck,
						drvdata->t_jack_recheck);
				} else {
					pr_info("%s: Headphones present.\n",
						__func__);
					jack->status |= SND_JACK_HEADPHONE;
					changed = true;
					drvdata->jack_hp_count =
						BYT_N_JACK_RECHECK;
				}
			} else if (status == RT5640_HEADSET_DET) {
				pr_info("%s: Headset present.\n", __func__);
				byt_set_mic_bias_ldo(codec, true);
				if (drvdata->board_cfg->jack_bp_sel ==
					JACK_BP_CODEC)
					rt5640_enable_ovcd_interrupt(codec,
								true);
				jack->status |= SND_JACK_HEADSET;
				changed = true;
				drvdata->jack_hp_count = BYT_N_JACK_RECHECK;
			} else
				pr_warn("%s: No valid accessory present!\n",
					__func__);
		}
	} else {
		if (jack->status & SND_JACK_HEADPHONE) {
			if (jack->status & SND_JACK_MICROPHONE) {
				jack->status &= ~SND_JACK_HEADSET;
				changed = true;
				byt_set_mic_bias_ldo(codec, false);
				if (drvdata->board_cfg->jack_bp_sel ==
					JACK_BP_CODEC)
					rt5640_enable_ovcd_interrupt(codec,
								false);
				pr_info("%s: Headset removed.\n", __func__);
			} else {
				jack->status &= ~SND_JACK_HEADPHONE;
				pr_info("%s: Headphone removed.\n", __func__);
			}
		} else if (!is_resume_check)
			pr_warn("%s: Remove-interrupt while no accessory present!\n",
					__func__);
	}

	return changed;
}

static void byt_jack_recheck(struct work_struct *work)
{
	struct byt_drvdata *drvdata =
		container_of(work, struct byt_drvdata, jack_recheck.work);
	struct snd_soc_jack *jack = &drvdata->jack;

	mutex_lock(&drvdata->jack_mlock);
	pr_debug("%s: Enter.\n", __func__);

	if (byt_jack_check(drvdata, true, false))
		snd_soc_jack_report(jack, jack->status, SND_JACK_HEADSET);

	mutex_unlock(&drvdata->jack_mlock);
}

/* Interrupt on micbias (JACK_DET_BAK/HOOK_INT) */
static int byt_micbias_interrupt(void *data)
{
	struct byt_drvdata *drvdata = (struct byt_drvdata *)data;
	struct snd_soc_jack *jack = &drvdata->jack;

	mutex_lock(&drvdata->jack_mlock);
	pr_debug("%s: Enter.\n", __func__);

	if (cancel_delayed_work_sync(&drvdata->bp_recheck))
		pr_debug("%s: bp-recheck interrupted!\n", __func__);

	if (jack->status & SND_JACK_MICROPHONE)
		if (byt_bp_check(drvdata, false))
			snd_soc_jack_report(jack, jack->status, SND_JACK_BTN_0);

	mutex_unlock(&drvdata->jack_mlock);
	return jack->status;
}

/* Interrupt on int2 (JACK_DET) */
static int byt_jack_int2_interrupt(void *data)
{
	struct byt_drvdata *drvdata = (struct byt_drvdata *)data;
	struct snd_soc_jack *jack = &drvdata->jack;

	if ((drvdata->board_cfg->jack_bp_sel == JACK_BP_MICBIAS) &&
		(drvdata->board_cfg->jack_int_sel == JACK_INT1)) {
		pr_debug("%s: INT2 not used. Returning...\n", __func__);
		return jack->status;
	}

	mutex_lock(&drvdata->jack_mlock);
	pr_debug("%s: Enter (jack->status = %d).\n", __func__, jack->status);

	if (cancel_delayed_work_sync(&drvdata->bp_recheck))
		pr_debug("%s: bp-recheck interrupted!\n", __func__);
	if (cancel_delayed_work_sync(&drvdata->jack_recheck))
		pr_debug("%s: jack-recheck interrupted!\n", __func__);

	/* Check for jack-events */
	byt_jack_check(drvdata, false, false);

	mutex_unlock(&drvdata->jack_mlock);
	return jack->status;
}

/* Interrupt on int1 (AUDIO_INT) */
static int byt_jack_int1_interrupt(void *data)
{
	struct byt_drvdata *drvdata = (struct byt_drvdata *)data;
	struct snd_soc_jack *jack = &drvdata->jack;

	if ((drvdata->board_cfg->jack_bp_sel == JACK_BP_MICBIAS) &&
		(drvdata->board_cfg->jack_int_sel == JACK_INT2)) {
		pr_debug("%s: INT1 not used. Returning...\n", __func__);
		return jack->status;
	}

	mutex_lock(&drvdata->jack_mlock);
	pr_debug("%s: Enter (jack->status = %d).\n", __func__, jack->status);

	if (cancel_delayed_work_sync(&drvdata->bp_recheck))
		pr_debug("%s: bp-recheck interrupted!\n", __func__);
	if (drvdata->board_cfg->jack_bp_sel == JACK_BP_MICBIAS)
		if (cancel_delayed_work_sync(&drvdata->jack_recheck))
			pr_debug("%s: jack-recheck interrupted!\n", __func__);

	/* Check for button-events if a headset is present (codec-mode only) */
	if (drvdata->board_cfg->jack_bp_sel == JACK_BP_CODEC) {
		if (jack->status & SND_JACK_MICROPHONE)
			if (byt_bp_check(drvdata, false))
				snd_soc_jack_report(jack, jack->status,
						SND_JACK_BTN_0);
	} else /* In micbias-mode we use int1 for jack-check */
		byt_jack_check(drvdata, false, false);

	mutex_unlock(&drvdata->jack_mlock);
	return jack->status;
}

static inline struct snd_soc_codec *byt_get_codec(struct snd_soc_card *card)
{
	bool found = false;
	struct snd_soc_codec *codec;

	list_for_each_entry(codec, &card->codec_dev_list, card_list) {
		if (!strstr(codec->name, "i2c-10EC5640:00"))
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

/* Widget-events */

static int platform_clock_control(struct snd_soc_dapm_widget *w,
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

		pr_debug("%s: Platform-clk turned on.\n", __func__);
	snd_soc_codec_set_sysclk(codec, RT5640_SCLK_S_PLL1, 0,
		BYT_PLAT_CLK_3_HZ, SND_SOC_CLOCK_IN);
	snd_soc_write(codec, RT5640_ADDA_CLK1, 0x0014);
	} else {
		/* Set codec clock source to internal clock before
		   turning off the platform clock. Codec needs clock
		   for Jack detection and button press */
		snd_soc_write(codec, RT5640_ADDA_CLK1, 0x7774);
		snd_soc_codec_set_sysclk(codec, RT5640_SCLK_S_RCCLK,
				0, 0, SND_SOC_CLOCK_IN);
		vlv2_plat_configure_clock(VLV2_PLAT_CLK_AUDIO,
				PLAT_CLK_FORCE_OFF);
		pr_debug("%s: Platform-clk turned off.\n", __func__);
	}

	return 0;
}

/* Widget-definitions */
static const struct snd_soc_dapm_widget byt_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Int Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_SUPPLY("Platform Clock", SND_SOC_NOPM, 0, 0,
			platform_clock_control, SND_SOC_DAPM_PRE_PMU |
			SND_SOC_DAPM_POST_PMD),
};

/* DAPM-routes */
static const struct snd_soc_dapm_route byt_audio_map[] = {
	{"IN2P", NULL, "Headset Mic"},
	{"IN2N", NULL, "Headset Mic"},
	{"LDO2", NULL, "Int Mic"},
	{"micbias1", NULL, "Int Mic"},
	{"Headphone", NULL, "HPOL"},
	{"Headphone", NULL, "HPOR"},
	{"Ext Spk", NULL, "SPOLP"},
	{"Ext Spk", NULL, "SPOLN"},
	{"Ext Spk", NULL, "SPORP"},
	{"Ext Spk", NULL, "SPORN"},

	{"Headphone", NULL, "Platform Clock"},
	{"Headset Mic", NULL, "Platform Clock"},
	{"Int Mic", NULL, "Platform Clock"},
	{"Ext Spk", NULL, "Platform Clock"},
	{"AIF1 Playback", NULL, "ssp2 Tx"},
	{"ssp2 Tx", NULL, "codec_out0"},
	{"ssp2 Tx", NULL, "codec_out1"},
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

static const struct snd_soc_dapm_route byt_audio_map_t100[] = {
	{"IN1P", NULL, "micbias1"},
};

static const struct snd_kcontrol_new byt_mc_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
	SOC_DAPM_PIN_SWITCH("Ext Spk"),
};

static int byt_set_dai_fmt_pll(struct snd_soc_dai *codec_dai,
			int source, unsigned int freq_out)
{
	int ret;
	unsigned int fmt;

	pr_debug("%s: Enter.\n", __func__);

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

	return 0;
}

static int byt_aif1_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	pr_debug("%s: Enter.\n", __func__);

	if (strncmp(codec_dai->name, "rt5640-aif1", 11))
		return 0;

	/* Set codec-DAI cfg */
	return byt_set_dai_fmt_pll(codec_dai, RT5640_PLL1_S_MCLK,
			params_rate(params));
}

static int byt_codec_fixup(struct snd_soc_pcm_runtime *rtd,
			struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);

	pr_debug("%s: Enter (dai_link: %s).\n", __func__, rtd->dai_link->name);

	/* The DSP will covert the FE rate to 48k, stereo, 24bits */
	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;

	/* set SSP2 to 24-bit */
	snd_mask_set(&params->masks[SNDRV_PCM_HW_PARAM_FORMAT -
				    SNDRV_PCM_HW_PARAM_FIRST_MASK],
				    SNDRV_PCM_FORMAT_S24_LE);

	return 0;
}

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

static void byt_init_gpios(struct snd_soc_codec *codec,
			struct byt_drvdata *drvdata)
{
	int ret, dir;
	struct gpio_desc *desc;

	desc = devm_gpiod_get_index(codec->dev, NULL, RT5640_GPIO_JD_INT);
	if (!IS_ERR(desc)) {
		drvdata->gpios.jd_int_gpio = desc_to_gpio(desc);
		byt_export_gpio(desc, "JD-int");

		pr_info("%s: GPIOs - JD-int: %d (pol = %d, val = %d)\n",
			__func__, drvdata->gpios.jd_int_gpio,
			gpiod_is_active_low(desc), gpiod_get_value(desc));

		devm_gpiod_put(codec->dev, desc);
	} else {
		drvdata->gpios.jd_int_gpio = RT5640_GPIO_NA;
		pr_err("%s: GPIOs - JD-int: Not present!\n", __func__);
	}

	desc = devm_gpiod_get_index(codec->dev, NULL, RT5640_GPIO_JD_INT2);
	if (!IS_ERR(desc)) {
		drvdata->gpios.jd_int2_gpio = desc_to_gpio(desc);
		byt_export_gpio(desc, "JD-int2");

		pr_info("%s: GPIOs - JD-int2: %d (pol = %d, val = %d)\n",
			__func__, drvdata->gpios.jd_int2_gpio,
			gpiod_is_active_low(desc), gpiod_get_value(desc));

		devm_gpiod_put(codec->dev, desc);
	} else {
		drvdata->gpios.jd_int2_gpio = RT5640_GPIO_NA;
		pr_warn("%s: GPIOs - JD-int2: Not present!\n", __func__);
	}

	desc = devm_gpiod_get_index(codec->dev, NULL, RT5640_GPIO_JACK_SWITCH);
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
		drvdata->gpios.debug_mux_gpio = RT5640_GPIO_NA;
		pr_warn("%s: GPIOs - Debug-mux: Not present!\n", __func__);
	}

	desc = devm_gpiod_get_index(codec->dev, NULL, RT5640_GPIO_JD_BUTTONS);
	if (!IS_ERR(desc)) {
		drvdata->gpios.jd_buttons_gpio = desc_to_gpio(desc);
		byt_export_gpio(desc, "JD-buttons");

		pr_info("%s: GPIOs - JD-buttons: %d (pol = %d, val = %d)\n",
			__func__, drvdata->gpios.jd_buttons_gpio,
			gpiod_is_active_low(desc), gpiod_get_value(desc));

		devm_gpiod_put(codec->dev, desc);
	} else {
		drvdata->gpios.jd_buttons_gpio = RT5640_GPIO_NA;
		pr_warn("%s: GPIOs - JD-buttons: Not present!\n", __func__);
	}

	desc = devm_gpiod_get_index(codec->dev, NULL, RT5640_GPIO_I2S_TRISTATE);
	if (!IS_ERR(desc)) {
		drvdata->gpios.i2s_tristate_en_gpio = desc_to_gpio(desc);

		ret = gpiod_direction_output(desc, 0);
		if (ret)
			pr_warn("%s: Failed to set direction for GPIO%d (err = %d)!\n",
				__func__, drvdata->gpios.i2s_tristate_en_gpio,
				ret);

		byt_export_gpio(desc, "I2S-Tristate-En");

		pr_info("%s: GPIOs - I2S-Tristate-En: %d (pol = %d, val = %d)\n",
			__func__, drvdata->gpios.i2s_tristate_en_gpio,
			gpiod_is_active_low(desc), gpiod_get_value(desc));

		devm_gpiod_put(codec->dev, desc);
	} else {
		drvdata->gpios.i2s_tristate_en_gpio = RT5640_GPIO_NA;
		pr_warn("%s: GPIOs - i2s_tristate_en-mux: Not present!\n",
				__func__);
	}
}

/* ASoC Jack-definitions */

static struct snd_soc_jack_gpio jack_gpio_int1[] = {
	{
		.name                   = "byt-int1-int",
		.report                 = 0, /* Set dynamically */
		.debounce_time          = BYT_JD_INTR_DEBOUNCE,
		.jack_status_check      = byt_jack_int1_interrupt,
	},
};

static struct snd_soc_jack_gpio jack_gpio_int2[] = {
	{
		.name                   = "byt-int2-int",
		.report                 = SND_JACK_HEADSET,
		.debounce_time          = BYT_JD_INTR_DEBOUNCE,
		.jack_status_check      = byt_jack_int2_interrupt,
	},
};

static struct snd_soc_jack_gpio jack_gpio_micbias[] = {
	{
		.name                   = "byt-micbias-int",
		.report                 = SND_JACK_BTN_0,
		.debounce_time          = BYT_JD_INTR_DEBOUNCE,
		.jack_status_check      = byt_micbias_interrupt,
	},
};

static int byt_config_jack_gpios(struct byt_drvdata *drvdata)
{
	int int_sel = drvdata->board_cfg->jack_int_sel;
	int bp_sel = drvdata->board_cfg->jack_bp_sel;
	int int1_gpio = drvdata->gpios.jd_int_gpio;
	int int2_gpio = drvdata->gpios.jd_int2_gpio;
	int bp_gpio = drvdata->gpios.jd_buttons_gpio;

	pr_info("%s: Jack-GPIO config: jack_bp_sel = %d, jack_int_sel = %d\n",
		__func__, bp_sel, int_sel);

	if ((bp_sel == JACK_BP_CODEC) && (int_sel == JACK_INT1)) {
		pr_err("%s: Unsupported jack-config (jack_bp_sel = %d, jack_int_sel = %d)!\n",
			__func__, JACK_BP_CODEC, JACK_INT1);
		return -EINVAL;
	}

	if ((bp_sel == JACK_BP_MICBIAS) && (bp_gpio == RT5640_GPIO_NA)) {
		pr_err("%s: Micbias-GPIO missing (jack_bp_sel = %d, jack_int_sel = %d)!\n",
			__func__, JACK_BP_MICBIAS, int_sel);
		return -EINVAL;
	}

	if (((int_sel == JACK_INT1) && (int1_gpio == RT5640_GPIO_NA)) ||
		((int_sel == JACK_INT2) && (int2_gpio == RT5640_GPIO_NA))) {
		pr_err("%s: Missing jack-GPIO (jack_bp_sel = %d, jack_int_sel = %d, )\n",
			__func__, bp_sel, int_sel);
		return -EINVAL;
	}

	if ((int_sel == JACK_INT1) || (bp_sel == JACK_BP_CODEC)) {
		jack_gpio_int1[0].gpio = int1_gpio;
		jack_gpio_int1[0].data = drvdata;
		jack_gpio_int1[0].report = (int_sel == JACK_INT1) ?
				SND_JACK_HEADSET : SND_JACK_BTN_0;
		pr_debug("%s: Add jack-GPIO 1 (%d).\n", __func__, int1_gpio);
		if (snd_soc_jack_add_gpios(&drvdata->jack, 1,
					&jack_gpio_int1[0])) {
			pr_err("%s: Failed to add jack-GPIO 1!\n", __func__);
			return -EIO;
		}
	}

	if (int_sel == JACK_INT2) {
		jack_gpio_int2[0].gpio = int2_gpio;
		jack_gpio_int2[0].data = drvdata;
		pr_debug("%s: Add jack-GPIO 2 (%d).\n", __func__, int2_gpio);
		if (snd_soc_jack_add_gpios(&drvdata->jack, 1,
					&jack_gpio_int2[0])) {
			pr_err("%s: Failed to add jack-GPIO 2!\n", __func__);
			return -EIO;
		}
	}

	if (bp_sel == JACK_BP_MICBIAS) {
		jack_gpio_micbias[0].gpio = bp_gpio;
		jack_gpio_micbias[0].data = drvdata;
		pr_debug("%s: Add micbias-GPIO (%d).\n", __func__, bp_gpio);
		if (snd_soc_jack_add_gpios(&drvdata->jack, 1,
					&jack_gpio_micbias[0])) {
			pr_err("%s: Failed to add micbias-GPIO!\n", __func__);
			return -EIO;
		}
	}

	return 0;
}

static int byt_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret;
	struct snd_soc_codec *codec;
	struct snd_soc_card *card = runtime->card;
	struct byt_drvdata *drvdata = snd_soc_card_get_drvdata(runtime->card);

	pr_debug("%s: Enter.\n", __func__);

	codec = byt_get_codec(card);
	if (!codec) {
		pr_err("%s: Codec not found!\n", __func__);
		return -EIO;
	}

	card->dapm.idle_bias_off = true;

	/* Init GPIOs */
	byt_init_gpios(codec, drvdata);

	/* Add ASoC-jack */
	ret = snd_soc_jack_new(codec, "BYT-CR Audio Jack",
			SND_JACK_HEADSET | SND_JACK_BTN_0,
			 &drvdata->jack);
	if (ret) {
		pr_err("%s: snd_soc_jack_new failed (ret = %d)!\n", __func__,
			ret);
		return ret;
	}

	/* Init jack-mutex before GPIO-interrupts are added */
	mutex_init(&drvdata->jack_mlock);

	/* Handle jack/bp GPIOs */
	ret = byt_config_jack_gpios(drvdata);
	if (ret)
		return ret;

	/* Other jack/bp stuff */

	/* JACK_DET_N signal as JD-source */
	snd_soc_update_bits(codec, RT5640_JD_CTRL,
			RT5640_JD_MASK, RT5640_JD_JD1_IN4P);

	/* Prevent sta_jd_internal to trigger IRQ in CODEC-mode */
	if (drvdata->board_cfg->jack_bp_sel == JACK_BP_CODEC)
		snd_soc_write(codec, RT5640_IRQ_CTRL1, 0x0000);
	else
		snd_soc_write(codec, RT5640_IRQ_CTRL1, 0x8000);

	/* Set OVCD-threshold */
	rt5640_config_ovcd_thld(codec, RT5640_MIC1_OVTH_1500UA,
				RT5640_MIC_OVCD_SF_1P0);

	drvdata->t_jack_recheck = msecs_to_jiffies(BYT_T_JACK_RECHECK);
	INIT_DELAYED_WORK(&drvdata->jack_recheck, byt_jack_recheck);
	drvdata->t_buttons_recheck = msecs_to_jiffies(BYT_T_BUTTONS_RECHECK);
	INIT_DELAYED_WORK(&drvdata->bp_recheck, byt_bp_recheck);
	drvdata->jack_hp_count = 5;

	snd_jack_set_key(drvdata->jack.jack, SND_JACK_BTN_0, KEY_MEDIA);

	/* Add machine-driver ALSA-controls */
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
static struct snd_soc_ops byt_be_ssp2_ops = {
	.hw_params = byt_aif1_hw_params,
};

static const struct snd_soc_pcm_stream byt_dai_params = {
	.formats = SNDRV_PCM_FMTBIT_S24_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
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
		.name = "SSP2-Codec",
		.be_id = 1,
		.cpu_dai_name = "ssp2-port",
		.platform_name = "sst-platform",
		.no_pcm = 1,
		.codec_dai_name = "rt5640-aif1",
		.codec_name = "i2c-10EC5640:00",
		.be_hw_params_fixup = byt_codec_fixup,
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
	struct platform_device *pdev = to_platform_device(dev);
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct byt_drvdata *drvdata = snd_soc_card_get_drvdata(soc_card);
	struct snd_soc_jack *jack = &drvdata->jack;

	mutex_lock(&drvdata->jack_mlock);
	pr_debug("%s: Enter.\n", __func__);

	byt_jack_check(drvdata, false, true);
	snd_soc_jack_report(jack, jack->status, SND_JACK_HEADSET);

	mutex_unlock(&drvdata->jack_mlock);

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
	.name = NULL,
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

	pr_debug("%s: Enter.\n", __func__);

	drvdata = devm_kzalloc(&pdev->dev, sizeof(*drvdata), GFP_ATOMIC);
	if (!drvdata) {
		pr_err("Allocation failed!\n");
		return -ENOMEM;
	}

	/* Get board-specific HW-settings */
	drvdata->board_cfg = get_board_config(get_mc_link());

	/* Configure board-specific routes */
	card = &snd_soc_card_byt_default;
	card->name = drvdata->board_cfg->name;
	switch (drvdata->board_cfg->mic_input) {
	case 1:
		routes = &byt_audio_map_t100[0];
		break;
	case 3:
		routes = &byt_audio_map_default[0];
		break;
	default:
		BUG_ON(true);
	}

	/* register the soc card */
	card->dev = &pdev->dev;
	snd_soc_card_set_drvdata(card, drvdata);
	ret_val = snd_soc_register_card(card);
	if (ret_val) {
		pr_err("%s: snd_soc_register_card failed %d\n", __func__,
			ret_val);
		return ret_val;
	}
	platform_set_drvdata(pdev, card);

	ret_val = snd_soc_dapm_add_routes(&card->dapm, routes, 1);
	if (ret_val) {
		pr_err("%s: Failed to add board-specific routes!\n", __func__);
		return ret_val;
	}

	return ret_val;
}

static void snd_byt_unregister_jack(struct byt_drvdata *drvdata)
{
	cancel_delayed_work_sync(&drvdata->jack_recheck);
	cancel_delayed_work_sync(&drvdata->bp_recheck);
	snd_soc_jack_free_gpios(&drvdata->jack, drvdata->gpios.int_count,
				jack_gpio_int1);

}
static int snd_byt_mc_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct byt_drvdata *drv = snd_soc_card_get_drvdata(soc_card);

	pr_debug("%s: Enter.\n", __func__);

	snd_byt_unregister_jack(drv);
	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(soc_card);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static void snd_byt_mc_shutdown(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct byt_drvdata *drv = snd_soc_card_get_drvdata(soc_card);

	pr_debug("In %s\n", __func__);
	snd_byt_unregister_jack(drv);
}

static const struct dev_pm_ops snd_byt_mc_pm_ops = {
	.prepare = snd_byt_prepare,
	.complete = snd_byt_complete,
	.poweroff = snd_byt_poweroff,
};

static const struct acpi_device_id byt_mc_acpi_ids[] = {
	{ "AMCR0F28", 0 },
	{},
};
MODULE_DEVICE_TABLE(acpi, byt_mc_acpi_ids);

static struct platform_driver snd_byt_mc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "byt_rt5640",
		.pm = &snd_byt_mc_pm_ops,
	},
	.probe = snd_byt_mc_probe,
	.remove = snd_byt_mc_remove,
	.shutdown = snd_byt_mc_shutdown,
};

static int __init snd_byt_driver_init(void)
{
	int ret;

	pr_debug("%s: Enter.\n", __func__);

	ret = platform_driver_register(&snd_byt_mc_driver);
	if (ret)
		pr_err("%s: Failed to register BYT-CR-RT5640 Machine driver!\n",
			__func__);
	else
		pr_info("%s: BYT-CR-RT5640 Machine driver registered.\n",
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

MODULE_DESCRIPTION("ASoC Intel(R) BYT-CR-RT5640 Machine driver");
MODULE_AUTHOR("Ola Lilja <ola.lilja@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:bytrt5640-audio");
