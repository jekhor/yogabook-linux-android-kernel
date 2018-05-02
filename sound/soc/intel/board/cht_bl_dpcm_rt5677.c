/*
 *  cht_bl_dpcm_rt5677.c - ASoc DPCM Machine driver
 *  for Intel CherryTrail MID platform
 *
 *  Copyright (C) 2014 Intel Corp
 *  Author: Mythri P K <mythri.p.k@intel.com>
 *  This file is modified from byt_bl_rt5642.c for cherrytrail
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

#define DEBUG
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/vlv2_plat_clock.h>
#include <linux/mutex.h>
#include <linux/dmi.h>
#include <linux/delay.h>
#include <asm/platform_cht_audio.h>
#include <asm/intel-mid.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <asm-generic/delay.h>
#include "../../codecs/rt5677.h"
#include <cht_bl_pinctrl.h>
#define CHT_PLAT_CLK_3_HZ	19200000

#define MCLKONALWAYS


static inline void cht_force_enable_pin(struct snd_soc_codec *codec,
			 const char *bias_widget, bool enable)
{
	pr_debug("%s %s\n", enable ? "enable" : "disable", bias_widget);
	if (enable)
		snd_soc_dapm_force_enable_pin(&codec->dapm, bias_widget);
	else
		snd_soc_dapm_disable_pin(&codec->dapm, bias_widget);
}

static inline struct snd_soc_codec *cht_get_codec(struct snd_soc_card *card)
{
	bool found = false;
	struct snd_soc_codec *codec;

	list_for_each_entry(codec, &card->codec_dev_list, card_list) {
		pr_debug("codec name is %s", codec->name);
		if (!strstr(codec->name, "rt5677")) {
			pr_debug("codec was %s", codec->name);
			continue;
		} else {
			found = true;
			break;
		}
	}
	if (found == false) {
		pr_err("%s: cant find codec", __func__);
		return NULL;
	}
	return codec;
}

#define VLV2_PLAT_CLK_AUDIO	3
#define PLAT_CLK_FORCE_ON	1
#define PLAT_CLK_FORCE_OFF	2
static int platform_clock_control(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *k, int  event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct snd_soc_codec *codec;

	codec = cht_get_codec(card);
	if (!codec) {
		pr_err("Codec not found; Unable to set platform clock\n");
		return -EIO;
	}
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		vlv2_plat_configure_clock(VLV2_PLAT_CLK_AUDIO,
				PLAT_CLK_FORCE_ON);
		pr_debug("Platform clk turned ON\n");
	} else {
#ifdef MCLKONALWAYS
		/* Set codec clock source to internal clock before
		 * turning off the platform clock. Codec needs clock
		 * for Jack detection and button press
		 */

		vlv2_plat_configure_clock(VLV2_PLAT_CLK_AUDIO,
				PLAT_CLK_FORCE_OFF);
		pr_debug("Platform clk turned OFF\n");
#endif
	}

	return 0;
}

static int cht_rt5677_hp_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *k, int  event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct snd_soc_codec *codec;

	pr_err("%s, %d\n", __func__, __LINE__);
	codec = cht_get_codec(card);
	if (!codec) {
		pr_err("Codec not found; Unable to set platform clock\n");
		return -EIO;
	}
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		pr_err("%s, %d\n", __func__, __LINE__);
		msleep(20);
		snd_soc_update_bits(codec, RT5677_GPIO_CTRL2,
			RT5677_GPIO5_OUT_MASK, RT5677_GPIO5_OUT_HI);
		msleep(50);
	} else {
		pr_err("%s, %d\n", __func__, __LINE__);
		snd_soc_update_bits(codec, RT5677_GPIO_CTRL2,
			RT5677_GPIO5_OUT_MASK, RT5677_GPIO5_OUT_LO);
	}

	return 0;
}

static int cht_rt5677_spk_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *k, int  event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct snd_soc_codec *codec;

	pr_err("%s, %d\n", __func__, __LINE__);
	codec = cht_get_codec(card);
	if (!codec) {
		pr_err("Codec not found; Unable to set platform clock\n");
		return -EIO;
	}
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		pr_err("%s, %d\n", __func__, __LINE__);
		// use gpio GPIO_SPK_EN to enable/disable ext boost pa
		// use mode 3
		gpio_direction_output(GPIO_SPK_EN, 1);
		udelay(2);
		gpio_direction_output(GPIO_SPK_EN, 0);
		udelay(2);
		gpio_direction_output(GPIO_SPK_EN, 1);
		udelay(2);
		gpio_direction_output(GPIO_SPK_EN, 0);
		udelay(2);
		gpio_direction_output(GPIO_SPK_EN, 1);
		snd_soc_update_bits(codec, RT5677_GPIO_CTRL2,
			RT5677_GPIO3_OUT_MASK, RT5677_GPIO3_OUT_HI);
		msleep(50);
	} else {
		pr_err("%s, %d\n", __func__, __LINE__);
		// use gpio GPIO_SPK_EN to enable/disable ext boost pa
		gpio_direction_output(GPIO_SPK_EN, 0);
		snd_soc_update_bits(codec, RT5677_GPIO_CTRL2,
			RT5677_GPIO3_OUT_MASK, RT5677_GPIO3_OUT_LO);
	}

	return 0;
}

static const struct snd_soc_dapm_widget cht_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", cht_rt5677_hp_event),
	SND_SOC_DAPM_SPK("Speaker", cht_rt5677_spk_event),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Int Mic", NULL),
	SND_SOC_DAPM_SUPPLY("Platform Clock", SND_SOC_NOPM, 0, 0,
			platform_clock_control, SND_SOC_DAPM_PRE_PMU|
			SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route cht_audio_map[] = {
	{"IN1P", NULL, "Headset Mic"},
	{"IN1N", NULL, "Headset Mic"},
	{"DMIC L1", NULL, "Int Mic"},
	{"DMIC R1", NULL, "Int Mic"},
	{"Headphone", NULL, "LOUT1"},
	{"Headphone", NULL, "LOUT2"},
	{"Speaker", NULL, "LOUT1"},
	{"Speaker", NULL, "LOUT2"},

	{ "AIF1 Playback", NULL, "ssp2 Tx"},
	{ "ssp2 Tx", NULL, "codec_out0"},
	{ "ssp2 Tx", NULL, "codec_out1"},
	{ "codec_in0", NULL, "ssp2 Rx" },
	{ "codec_in1", NULL, "ssp2 Rx" },
	{ "ssp2 Rx", NULL, "AIF1 Capture"},

	{ "ssp0 Tx", NULL, "modem_out"},
	{ "modem_in", NULL, "ssp0 Rx" },

	{ "ssp1 Tx", NULL, "bt_fm_out"},
	{ "bt_fm_in", NULL, "ssp1 Rx" },

	{"AIF1 Playback", NULL, "Platform Clock"},
	{"AIF1 Capture", NULL, "Platform Clock"},
};

static const struct snd_kcontrol_new cht_mc_controls[] = {
	SOC_DAPM_PIN_SWITCH("Speaker"),
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
};


static int cht_aif1_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int fmt;
	int ret;

	pr_debug("Enter:%s", __func__);

	/* proceed only if dai is valid */
	if (strncmp(codec_dai->name, "rt5677-aif1", 11))
		return 0;

	/* TDM 4 slot 24 bit set the Rx and Tx bitmask to
	 * 4 active slots as 0xF
	 */
#if 0
	ret = snd_soc_dai_set_tdm_slot(codec_dai, 0xF, 0xF, 4,
			SNDRV_PCM_FORMAT_GSM);
#else
	ret = snd_soc_dai_set_tdm_slot(codec_dai, 0xF, 0xF, 4,
			25); /* Bard: Intel use 25 fs format */
#endif
	if (ret < 0) {
		pr_err("can't set codec TDM slot %d\n", ret);
		return ret;
	}

	/* TDM slave Mode */
	fmt =   SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_IB_NF
		| SND_SOC_DAIFMT_CBS_CFS;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_pll(codec_dai, 0, RT5677_PLL1_S_MCLK,
				  CHT_PLAT_CLK_3_HZ, params_rate(params) * 512);
	if (ret < 0) {
		pr_err("can't set codec pll: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, RT5677_SCLK_S_PLL1,
	//ret = snd_soc_dai_set_sysclk(codec_dai, RT5677_SCLK_S_MCLK,
				     params_rate(params) * 512,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		pr_err("can't set codec sysclk: %d\n", ret);
		return ret;
	}
	return 0;
}

static int cht_compr_set_params(struct snd_compr_stream *cstream)
{
	return 0;
}

static const struct snd_soc_pcm_stream cht_dai_params = {
	.formats = SNDRV_PCM_FMTBIT_S24_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
};

static int cht_codec_fixup(struct snd_soc_pcm_runtime *rtd,
			    struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);

	pr_debug("Invoked %s for dailink %s\n", __func__, rtd->dai_link->name);

	/* The DSP will covert the FE rate to 48k, stereo, 24bits */
	rate->min = rate->max = 48000;
	channels->min = channels->max = 4;

	/* set SSP2 to 24-bit */
	snd_mask_set(&params->masks[SNDRV_PCM_HW_PARAM_FORMAT -
				    SNDRV_PCM_HW_PARAM_FIRST_MASK],
				    SNDRV_PCM_FORMAT_S24_LE);
	return 0;
}

static int cht_set_bias_level(struct snd_soc_card *card,
				struct snd_soc_dapm_context *dapm,
				enum snd_soc_bias_level level)
{
	int ret = 0;
	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
	case SND_SOC_BIAS_OFF:
		card->dapm.bias_level = level;
		pr_debug("card(%s)->bias_level %u\n", card->name,
				card->dapm.bias_level);
		break;
	default:
		pr_err("%s: Invalid bias level=%d\n", __func__, level);
		ret =  -EINVAL;
	}

	return ret;
}

static int cht_audio_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret;
	struct snd_soc_codec *codec;
	struct snd_soc_card *card = runtime->card;

	ret = 0;
	pr_debug("Enter:%s\n", __func__);
	codec = cht_get_codec(card);
	if (!codec) {
		pr_err("Codec not found; %s: failed\n", __func__);
		return -EIO;
	}

	/* Set codec bias level */
	cht_set_bias_level(card, &card->dapm, SND_SOC_BIAS_OFF);
	card->dapm.idle_bias_off = true;

	/* Keep the voice call paths active during
	 * suspend. Mark the end points ignore_suspend
	 */
	/*TODO: CHECK this */
#if 0
	snd_soc_dapm_ignore_suspend(&card->dapm, "HPOL");
	snd_soc_dapm_ignore_suspend(&card->dapm, "HPOR");

	snd_soc_dapm_ignore_suspend(&card->dapm, "SPOL");
	snd_soc_dapm_ignore_suspend(&card->dapm, "SPOR");
#endif
	snd_soc_dapm_enable_pin(&card->dapm, "Headset Mic");
	snd_soc_dapm_enable_pin(&card->dapm, "Headphone");
	snd_soc_dapm_enable_pin(&card->dapm, "Int Mic");

	snd_soc_dapm_sync(&card->dapm);

	return ret;
}

static unsigned int rates_8000_16000[] = {
	8000,
	16000,
};

static struct snd_pcm_hw_constraint_list constraints_8000_16000 = {
	.count = ARRAY_SIZE(rates_8000_16000),
	.list  = rates_8000_16000,
};

static unsigned int rates_48000[] = {
	48000,
};

static struct snd_pcm_hw_constraint_list constraints_48000 = {
	.count = ARRAY_SIZE(rates_48000),
	.list  = rates_48000,
};

static int cht_aif1_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&constraints_48000);
}

static struct snd_soc_ops cht_aif1_ops = {
	.startup = cht_aif1_startup,
};

static int cht_8k_16k_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&constraints_8000_16000);
}

static struct snd_soc_ops cht_8k_16k_ops = {
	.startup = cht_8k_16k_startup,
	.hw_params = cht_aif1_hw_params,
};
static struct snd_soc_ops cht_be_ssp2_ops = {
	.hw_params = cht_aif1_hw_params,
};

static struct snd_soc_compr_ops cht_compr_ops = {
	.set_params = cht_compr_set_params,
};

static struct snd_soc_dai_link cht_dailink[] = {
	[CHT_DPCM_AUDIO] = {
		.name = "Cherrytrail Audio Port",
		.stream_name = "Cherrytrail Audio",
		.cpu_dai_name = "Headset-cpu-dai",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "sst-platform",
		.init = cht_audio_init,
		.ignore_suspend = 1,
		.dynamic = 1,
		.ops = &cht_aif1_ops,
		.dpcm_playback = 1,
	},
	[CHT_DPCM_DB] = {
		.name = "Cherrytrail DB Audio Port",
		.stream_name = "Deep Buffer Audio",
		.cpu_dai_name = "Deepbuffer-cpu-dai",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "sst-platform",
		.ignore_suspend = 1,
		.dynamic = 1,
		.ops = &cht_aif1_ops,
		.dpcm_playback = 1,
	},
	[CHT_DPCM_COMPR] = {
		.name = "Cherrytrail Compressed Port",
		.stream_name = "Cherrytrail Compress",
		.cpu_dai_name = "Compress-cpu-dai",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "sst-platform",
		.ignore_suspend = 1,
		.dynamic = 1,
		.compr_ops = &cht_compr_ops,
		.dpcm_playback = 1,
	},
	[CHT_DPCM_VOIP] = {
		.name = "Cherrytrail VOIP Port",
		.stream_name = "Cherrytrail Voip",
		.cpu_dai_name = "Voip-cpu-dai",
		.platform_name = "sst-platform",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &cht_8k_16k_ops,
		.dynamic = 1,
	},
	[CHT_DPCM_LL] = {
		.name = "Cherrytrail LL Audio Port",
		.stream_name = "Low Latency Audio",
		.cpu_dai_name = "Lowlatency-cpu-dai",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "sst-platform",
		.ignore_suspend = 1,
		.dynamic = 1,
		.ops = &cht_aif1_ops,
	},
	[CHT_DPCM_PROBE] = {
		.name = "Cherrytrail Probe Port",
		.stream_name = "Cherrytrail Probe",
		.cpu_dai_name = "Probe-cpu-dai",
		.platform_name = "sst-platform",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.playback_count = 8,
		.capture_count = 8,
	},
	/* CODEC<->CODEC link */
	{
		.name = "Cherrytrail Codec-Loop Port",
		.stream_name = "Cherrytrail Codec-Loop",
		.cpu_dai_name = "ssp2-port",
		.platform_name = "sst-platform",
		.codec_dai_name = "rt5677-aif1",
		.codec_name = "rt5677.0-002c",
		.dai_fmt = SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_IB_NF
			| SND_SOC_DAIFMT_CBS_CFS,
		.params = &cht_dai_params,
		.dsp_loopback = true,
	},
	{
		.name = "Cherrytrail Modem-Loop Port",
		.stream_name = "Cherrytrail Modem-Loop",
		.cpu_dai_name = "ssp0-port",
		.platform_name = "sst-platform",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.params = &cht_dai_params,
		.dsp_loopback = true,
	},
	{
		.name = "Cherrytrail BTFM-Loop Port",
		.stream_name = "Cherrytrail BTFM-Loop",
		.cpu_dai_name = "ssp1-port",
		.platform_name = "sst-platform",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.params = &cht_dai_params,
		.dsp_loopback = true,
	},
	/* Back ends */
	{
		.name = "SSP2-Codec",
		.be_id = 1,
		.cpu_dai_name = "ssp2-port",
		.platform_name = "sst-platform",
		.no_pcm = 1,
		.codec_dai_name = "rt5677-aif1",
		.codec_name = "rt5677.0-002c",
		.be_hw_params_fixup = cht_codec_fixup,
		.ignore_suspend = 1,
		.ops = &cht_be_ssp2_ops,
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
	{
		.name = "SSP0-Modem",
		.be_id = 3,
		.cpu_dai_name = "snd-soc-dummy-dai",
		.platform_name = "snd-soc-dummy",
		.no_pcm = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.ignore_suspend = 1,
	},
};

#ifdef CONFIG_PM_SLEEP
static int snd_cht_prepare(struct device *dev)
{
	pr_debug("In %s device name\n", __func__);
	return snd_soc_suspend(dev);
}

static void snd_cht_complete(struct device *dev)
{
	pr_debug("In %s\n", __func__);
#ifdef MCLKONALWAYS
	vlv2_plat_configure_clock(VLV2_PLAT_CLK_AUDIO, PLAT_CLK_FORCE_OFF);
#endif
	snd_soc_resume(dev);
}

static int snd_cht_poweroff(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	return snd_soc_poweroff(dev);
}
#else
#define snd_cht_prepare NULL
#define snd_cht_complete NULL
#define snd_cht_poweroff NULL
#endif

/* SoC card */
static struct snd_soc_card snd_soc_card_cht = {
	.name = "cherrytrailaud",
	.dai_link = cht_dailink,
	.num_links = ARRAY_SIZE(cht_dailink),
	.set_bias_level = cht_set_bias_level,
	.dapm_widgets = cht_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(cht_dapm_widgets),
	.dapm_routes = cht_audio_map,
	.num_dapm_routes = ARRAY_SIZE(cht_audio_map),
	.controls = cht_mc_controls,
	.num_controls = ARRAY_SIZE(cht_mc_controls),
};

static int snd_cht_mc_probe(struct platform_device *pdev)
{
	int ret_val = 0;
	//struct cht_mc_private *drv;
	struct snd_soc_codec *codec;

	pr_debug("Entry %s\n", __func__);

#ifdef MCLKONALWAYS
	/* Audio Platform clock is on by default. The machine driver requests
	 * this clock to be turned ON and OFF on playing any stream. But
	 * until any stream is played the clock remains ON. Hence request the
	 * clock to be turned OFF initially.
	 */
	vlv2_plat_configure_clock(VLV2_PLAT_CLK_AUDIO, PLAT_CLK_FORCE_OFF);
#endif

	/* register the soc card */
	snd_soc_card_cht.dev = &pdev->dev;
	//snd_soc_card_set_drvdata(&snd_soc_card_cht, drv);
	ret_val = snd_soc_register_card(&snd_soc_card_cht);
	if (ret_val) {
		pr_err("snd_soc_register_card failed %d\n", ret_val);
		return ret_val;
	}
	platform_set_drvdata(pdev, &snd_soc_card_cht);

	codec = cht_get_codec(&snd_soc_card_cht);
        
        // use gpio GPIO_SPK_EN to enable/diable ext boost pa
	gpio_request(GPIO_SPK_EN, "speaker boost pa ctl");
	gpio_direction_output(GPIO_SPK_EN, 0);
	
	//for test
	/*Open MCLK before Set Codec CLK*/
	//vlv2_plat_configure_clock(VLV2_PLAT_CLK_AUDIO, PLAT_CLK_FORCE_ON);
	pr_info("%s successful\n", __func__);
	return ret_val;
}

static int snd_cht_mc_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);

	pr_debug("In %s\n", __func__);
	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(soc_card);
	platform_set_drvdata(pdev, NULL);
	// use gpio GPIO_SPK_EN to enable/diable ext boost pa
	gpio_free(GPIO_SPK_EN);
	return 0;
}

static void snd_cht_mc_shutdown(struct platform_device *pdev)
{
	struct snd_soc_codec *codec = NULL;

	printk("In %s\n", __func__);
	codec = cht_get_codec(&snd_soc_card_cht);
        if(codec == NULL){
                pr_err("%s, codec has not probed yet!\n", __func__);
                return;
        }
	//STEP 1:  use gpio GPIO_SPK_EN to enable/disable ext boost pa whatever current output device is HP or SPK
	gpio_direction_output(GPIO_SPK_EN, 0);
	snd_soc_update_bits(codec, RT5677_GPIO_CTRL2,
                           RT5677_GPIO5_OUT_MASK | RT5677_GPIO3_OUT_MASK,
                           RT5677_GPIO5_OUT_LO | RT5677_GPIO3_OUT_LO);
	//STEP 2: mute all of source to the stereo1 DAC digital mixer
	snd_soc_update_bits(codec, RT5677_STO1_DAC_MIXER,
			RT5677_M_ST_DAC1_L|RT5677_M_DAC1_L_STO_L|RT5677_M_DAC2_L_STO_L|RT5677_M_DAC1_R_STO_L|RT5677_M_ST_DAC1_R|RT5677_M_DAC1_R_STO_R|RT5677_M_DAC2_R_STO_R|RT5677_M_DAC1_L_STO_R,
			RT5677_M_ST_DAC1_L|RT5677_M_DAC1_L_STO_L|RT5677_M_DAC2_L_STO_L|RT5677_M_DAC1_R_STO_L|RT5677_M_ST_DAC1_R|RT5677_M_DAC1_R_STO_R|RT5677_M_DAC2_R_STO_R|RT5677_M_DAC1_L_STO_R);
}

const struct dev_pm_ops snd_cht_mc_pm_ops = {
	.prepare = snd_cht_prepare,
	.complete = snd_cht_complete,
	.poweroff = snd_cht_poweroff,
};

static struct platform_driver snd_cht_mc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "cht_rt5677",
		.pm = &snd_cht_mc_pm_ops,
	},
	.probe = snd_cht_mc_probe,
	.remove = snd_cht_mc_remove,
	.shutdown = snd_cht_mc_shutdown,
};

void rt5677_enable_micbias(bool enable)
{
	struct snd_soc_codec *codec;

	pr_err("%s, %d\n", __func__, enable);
	codec = cht_get_codec(&snd_soc_card_cht);
	if(codec == NULL){
		pr_err("%s, codec has not probed yet!\n", __func__);
		return;
	}
	if (enable) {
		snd_soc_dapm_force_enable_pin(&codec->dapm, "MICBIAS1");
	}
	else {
		snd_soc_dapm_disable_pin(&codec->dapm, "MICBIAS1");
	}
	snd_soc_dapm_sync(&codec->dapm);
}
EXPORT_SYMBOL_GPL(rt5677_enable_micbias);

static int __init snd_cht_driver_init(void)
{
	pr_info("Cherrytrail Machine Driver cht_rt5677 driver registerd.\n");
	return platform_driver_register(&snd_cht_mc_driver);
}
late_initcall(snd_cht_driver_init);

static void __exit snd_cht_driver_exit(void)
{
	pr_debug("In %s\n", __func__);
	platform_driver_unregister(&snd_cht_mc_driver);
}
module_exit(snd_cht_driver_exit);

MODULE_DESCRIPTION("ASoC Intel(R) Cherrytrail Machine driver");
MODULE_AUTHOR("Mythri P K <mythri.p.k@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:cht_rt5677");
