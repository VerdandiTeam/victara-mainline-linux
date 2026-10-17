// SPDX-License-Identifier: GPL-2.0-only

#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "common.h"
#include "qdsp6/q6afe.h"

#define MSM8974_MI2S_BCLK_RATE	1536000

static int msm8974_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd, struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;
	snd_mask_none(fmt);
	snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S16_LE);

	return 0;
}

static int msm8974_mi2s_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	int ret;

	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_BP_FP | SND_SOC_DAIFMT_I2S);
	if (ret)
		return ret;

	ret = snd_soc_dai_set_sysclk(cpu_dai, LPAIF_BIT_CLK, MSM8974_MI2S_BCLK_RATE, 0);
	if (ret)
		dev_err(rtd->card->dev, "failed to enable MI2S bit clock: %d\n", ret);

	return ret;
}

static void msm8974_mi2s_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	int ret;

	ret = snd_soc_dai_set_sysclk(cpu_dai, LPAIF_BIT_CLK, 0, 0);
	if (ret)
		dev_err(rtd->card->dev, "failed to disable MI2S bit clock: %d\n", ret);
}

static const struct snd_soc_ops msm8974_mi2s_ops = {
	.startup = msm8974_mi2s_startup,
	.shutdown = msm8974_mi2s_shutdown,
};

static int msm8974_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct snd_soc_dai_link *link;
	struct snd_soc_card *card;
	int ret;
	int i;

	card = devm_kzalloc(dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;

	card->driver_name = "msm8974";
	card->components = "qdsp6";
	card->dev = dev;
	card->owner = THIS_MODULE;
	dev_set_drvdata(dev, card);

	ret = qcom_snd_parse_of(card);
	if (ret)
		return ret;

	for_each_card_prelinks(card, i, link) {
		if (link->no_pcm) {
			link->ops = &msm8974_mi2s_ops;
			link->be_hw_params_fixup = msm8974_be_hw_params_fixup;
		}
	}

	return devm_snd_soc_register_card(dev, card);
}

static const struct of_device_id msm8974_snd_dt_match[] = {
	{ .compatible = "qcom,msm8974-qdsp6-sndcard" },
	{}
};
MODULE_DEVICE_TABLE(of, msm8974_snd_dt_match);

static struct platform_driver msm8974_snd_driver = {
	.probe = msm8974_platform_probe,
	.driver = {
		.name = "msm-snd-msm8974",
		.of_match_table = msm8974_snd_dt_match,
	},
};
module_platform_driver(msm8974_snd_driver);

MODULE_DESCRIPTION("MSM8974 QDSP6 ASoC Machine Driver");
MODULE_LICENSE("GPL");
