/*
 * LM4857 AMP driver
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 * Author: Graeme Gregory
 *         graeme.gregory@wolfsonmicro.com or linux@wolfsonmicro.com
 * Copyright 2011 Lars-Peter Clausen <lars@metafoo.de>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "lm4857.h"

struct lm4857 {
	struct i2c_client *i2c;
	uint8_t state;
};

static const uint8_t lm4857_default_regs[] = {
	0x00, 0x00, 0x00, 0x00,
};

/* The register offsets in the cache array */
#define LM4857_MVOL 0
#define LM4857_LVOL 1
#define LM4857_RVOL 2
#define LM4857_CTRL 3

/* the shifts required to set these bits */
#define LM4857_3D 5
#define LM4857_WAKEUP 5
#define LM4857_EPGAIN 4

#if 0
static int lm4857_write_regs(struct lm4857 *lm4857)
{
	unsigned int i;
	uint8_t regs[4];

	for (i = 0; i < 4; ++i)
		snd_soc_cache_read(lm4857->codec, i, &regs[i]);

	ret = i2c_master_send(lm4857->i2c, regs, 4);
	if (ret != 4) {
		if (ret >= 0)
		    ret = -EIO;
		dev_err(&lm4857.i2c->dev, "lm4857: i2c write failed\n");
	}
	return ret;
}
#endif

static int lm4857_write(struct snd_soc_codec *codec, unsigned int reg,
		unsigned int value)
{
	uint8_t data;
	int ret;

	data = (reg << 6) | value;

	ret = i2c_master_send(codec->control_data, &data, 1);
	if (ret != 1) {
		dev_err(codec->dev, "Failed to write register: %d\n", ret);
		return ret;
	}

	return 0;
}

static unsigned int lm4857_read(struct snd_soc_codec *codec,
		unsigned int reg)
{
	unsigned int val;
	int ret;

	ret = snd_soc_cache_read(codec, reg, &val);
	if (ret)
		return -1;

	return val;
}

static int lm4857_get_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	uint8_t value = snd_soc_read(codec, LM4857_CTRL) & 0x0F;

	if (value)
		value -= 5;

	ucontrol->value.integer.value[0] = value;

	return 0;
}

static int lm4857_set_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	uint8_t value = ucontrol->value.integer.value[0];

	if (value)
		value += 5;

	snd_soc_update_bits(codec, LM4857_CTRL, 0xF0, value);

	return 1;
}

static int lm4857_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	struct lm4857 *lm4857 = snd_soc_codec_get_drvdata(codec);

	switch (level) {
	case SND_SOC_BIAS_ON:
		snd_soc_update_bits(codec, LM4857_CTRL, 0xF0, lm4857->state);
		break;
	case SND_SOC_BIAS_OFF:
		lm4857->state = snd_soc_read(codec, LM4857_CTRL) & 0x0F;
		snd_soc_update_bits(codec, LM4857_CTRL, 0xF0, 0);
		break;
	default:
		break;
	}

	codec->dapm.bias_level = level;

	return 0;
}

static const char *lm4857_mode[] = {
	"Off",
	"Call Speaker",
	"Stereo Speakers",
	"Stereo Speakers + Headphones",
	"Headphones"
};

static const struct soc_enum lm4857_mode_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(lm4857_mode), lm4857_mode),
};

static const DECLARE_TLV_DB_SCALE(stereo_tlv, -4050, 150, 0);
static const DECLARE_TLV_DB_SCALE(mono_tlv, -3450, 150, 0);

static const struct snd_kcontrol_new lm4857_controls[] = {
	SOC_SINGLE_TLV("Amp Left Playback Volume", LM4857_LVOL, 0, 31, 0,
		stereo_tlv),
	SOC_SINGLE_TLV("Amp Right Playback Volume", LM4857_RVOL, 0, 31, 0,
		stereo_tlv),
	SOC_SINGLE_TLV("Amp Mono Playback Volume", LM4857_MVOL, 0, 31, 0,
		mono_tlv),
	SOC_ENUM_EXT("Amp Mode", lm4857_mode_enum[0],
		lm4857_get_mode, lm4857_set_mode),
	SOC_SINGLE("Amp Spk 3D Playback Switch", LM4857_LVOL, 5, 1, 0),
	SOC_SINGLE("Amp HP 3D Playback Switch", LM4857_RVOL, 5, 1, 0),
	SOC_SINGLE("Amp Fast Wakeup Playback Switch", LM4857_CTRL, 5, 1, 0),
	SOC_SINGLE("Amp Earpiece 6dB Playback Switch", LM4857_CTRL, 4, 1, 0),
};

static int lm4857_probe(struct snd_soc_codec *codec)
{
	struct lm4857 *lm4857 = snd_soc_codec_get_drvdata(codec);

	codec->control_data = lm4857->i2c;

	return snd_soc_add_controls(codec, lm4857_controls,
				ARRAY_SIZE(lm4857_controls));
}

static int lm4857_remove(struct snd_soc_codec *codec)
{
    lm4857_set_bias_level(codec, SND_SOC_BIAS_OFF);

    return 0;
}

static int lm4857_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
    lm4857_set_bias_level(codec, SND_SOC_BIAS_OFF);

    return 0;
}

static int lm4857_resume(struct snd_soc_codec *codec)
{
    lm4857_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

    return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_lm4857 = {
	.write = lm4857_write,
	.read = lm4857_read,
	.probe = lm4857_probe,
	.remove = lm4857_remove,
	.suspend = lm4857_suspend,
	.resume = lm4857_resume,
	.reg_cache_size = ARRAY_SIZE(lm4857_default_regs),
	.reg_word_size = sizeof(uint8_t),
	.reg_cache_default = lm4857_default_regs,
	.set_bias_level = lm4857_set_bias_level,
};

static int __devinit lm4857_i2c_probe(struct i2c_client *i2c,
	const struct i2c_device_id *id)
{
	struct lm4857 *lm4857;
	int ret;

	lm4857 = kzalloc(sizeof(*lm4857), GFP_KERNEL);
	if (!lm4857)
		return -ENOMEM;

	i2c_set_clientdata(i2c, lm4857);

	lm4857->i2c = i2c;

	ret = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_lm4857, NULL, 0);

	if (ret) {
	    kfree(lm4857);
	    return ret;
	}

	return 0;
}

static int __devexit lm4857_i2c_remove(struct i2c_client *i2c)
{
	struct lm4857 *lm4857 = i2c_get_clientdata(i2c);

	snd_soc_unregister_codec(&i2c->dev);
	kfree(lm4857);

	return 0;
}

static const struct i2c_device_id lm4857_i2c_id[] = {
	{ "lm4857", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lm4857_i2c_id);

static struct i2c_driver lm4857_i2c_driver = {
	.driver = {
		.name = "lm4857",
		.owner = THIS_MODULE,
	},
	.probe = lm4857_i2c_probe,
	.remove = __devexit_p(lm4857_i2c_remove),
	.id_table = lm4857_i2c_id,
};

static int __init lm4857_init(void)
{
	return i2c_add_driver(&lm4857_i2c_driver);
}
module_init(lm4857_init);

static void __exit lm4857_exit(void)
{
	i2c_del_driver(&lm4857_i2c_driver);
}
module_exit(lm4857_exit);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("LM4857 amplifier driver");
MODULE_LICENSE("GPL");
