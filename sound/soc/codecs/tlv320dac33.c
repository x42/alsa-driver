/*
 * ALSA SoC Texas Instruments TLV320DAC33 codec driver
 *
 * Author:	Peter Ujfalusi <peter.ujfalusi@nokia.com>
 *
 * Copyright:   (C) 2009 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include <sound/tlv320dac33-plat.h>
#include "tlv320dac33.h"

#define DAC33_BUFFER_SIZE_BYTES		24576	/* bytes, 12288 16 bit words,
						 * 6144 stereo */
#define DAC33_BUFFER_SIZE_SAMPLES	6144

#define NSAMPLE_MAX		5700

#define LATENCY_TIME_MS		20

static struct snd_soc_codec *tlv320dac33_codec;

enum dac33_state {
	DAC33_IDLE = 0,
	DAC33_PREFILL,
	DAC33_PLAYBACK,
	DAC33_FLUSH,
};

enum dac33_fifo_modes {
	DAC33_FIFO_BYPASS = 0,
	DAC33_FIFO_MODE1,
	DAC33_FIFO_LAST_MODE,
};

#define DAC33_NUM_SUPPLIES 3
static const char *dac33_supply_names[DAC33_NUM_SUPPLIES] = {
	"AVDD",
	"DVDD",
	"IOVDD",
};

struct tlv320dac33_priv {
	struct mutex mutex;
	struct workqueue_struct *dac33_wq;
	struct work_struct work;
	struct snd_soc_codec codec;
	struct regulator_bulk_data supplies[DAC33_NUM_SUPPLIES];
	int power_gpio;
	int chip_power;
	int irq;
	unsigned int refclk;

	unsigned int alarm_threshold;	/* set to be half of LATENCY_TIME_MS */
	unsigned int nsample_min;	/* nsample should not be lower than
					 * this */
	unsigned int nsample_max;	/* nsample should not be higher than
					 * this */
	enum dac33_fifo_modes fifo_mode;/* FIFO mode selection */
	unsigned int nsample;		/* burst read amount from host */

	enum dac33_state state;
};

static const u8 dac33_reg[DAC33_CACHEREGNUM] = {
0x00, 0x00, 0x00, 0x00, /* 0x00 - 0x03 */
0x00, 0x00, 0x00, 0x00, /* 0x04 - 0x07 */
0x00, 0x00, 0x00, 0x00, /* 0x08 - 0x0b */
0x00, 0x00, 0x00, 0x00, /* 0x0c - 0x0f */
0x00, 0x00, 0x00, 0x00, /* 0x10 - 0x13 */
0x00, 0x00, 0x00, 0x00, /* 0x14 - 0x17 */
0x00, 0x00, 0x00, 0x00, /* 0x18 - 0x1b */
0x00, 0x00, 0x00, 0x00, /* 0x1c - 0x1f */
0x00, 0x00, 0x00, 0x00, /* 0x20 - 0x23 */
0x00, 0x00, 0x00, 0x00, /* 0x24 - 0x27 */
0x00, 0x00, 0x00, 0x00, /* 0x28 - 0x2b */
0x00, 0x00, 0x00, 0x80, /* 0x2c - 0x2f */
0x80, 0x00, 0x00, 0x00, /* 0x30 - 0x33 */
0x00, 0x00, 0x00, 0x00, /* 0x34 - 0x37 */
0x00, 0x00,             /* 0x38 - 0x39 */
/* Registers 0x3a - 0x3f are reserved  */
            0x00, 0x00, /* 0x3a - 0x3b */
0x00, 0x00, 0x00, 0x00, /* 0x3c - 0x3f */

0x00, 0x00, 0x00, 0x00, /* 0x40 - 0x43 */
0x00, 0x80,             /* 0x44 - 0x45 */
/* Registers 0x46 - 0x47 are reserved  */
            0x80, 0x80, /* 0x46 - 0x47 */

0x80, 0x00, 0x00,       /* 0x48 - 0x4a */
/* Registers 0x4b - 0x7c are reserved  */
                  0x00, /* 0x4b        */
0x00, 0x00, 0x00, 0x00, /* 0x4c - 0x4f */
0x00, 0x00, 0x00, 0x00, /* 0x50 - 0x53 */
0x00, 0x00, 0x00, 0x00, /* 0x54 - 0x57 */
0x00, 0x00, 0x00, 0x00, /* 0x58 - 0x5b */
0x00, 0x00, 0x00, 0x00, /* 0x5c - 0x5f */
0x00, 0x00, 0x00, 0x00, /* 0x60 - 0x63 */
0x00, 0x00, 0x00, 0x00, /* 0x64 - 0x67 */
0x00, 0x00, 0x00, 0x00, /* 0x68 - 0x6b */
0x00, 0x00, 0x00, 0x00, /* 0x6c - 0x6f */
0x00, 0x00, 0x00, 0x00, /* 0x70 - 0x73 */
0x00, 0x00, 0x00, 0x00, /* 0x74 - 0x77 */
0x00, 0x00, 0x00, 0x00, /* 0x78 - 0x7b */
0x00,                   /* 0x7c        */

      0xda, 0x33, 0x03, /* 0x7d - 0x7f */
};

/* Register read and write */
static inline unsigned int dac33_read_reg_cache(struct snd_soc_codec *codec,
						unsigned reg)
{
	u8 *cache = codec->reg_cache;
	if (reg >= DAC33_CACHEREGNUM)
		return 0;

	return cache[reg];
}

static inline void dac33_write_reg_cache(struct snd_soc_codec *codec,
					 u8 reg, u8 value)
{
	u8 *cache = codec->reg_cache;
	if (reg >= DAC33_CACHEREGNUM)
		return;

	cache[reg] = value;
}

static int dac33_read(struct snd_soc_codec *codec, unsigned int reg,
		      u8 *value)
{
	struct tlv320dac33_priv *dac33 = codec->private_data;
	int val;

	*value = reg & 0xff;

	/* If powered off, return the cached value */
	if (dac33->chip_power) {
		val = i2c_smbus_read_byte_data(codec->control_data, value[0]);
		if (val < 0) {
			dev_err(codec->dev, "Read failed (%d)\n", val);
			value[0] = dac33_read_reg_cache(codec, reg);
		} else {
			value[0] = val;
			dac33_write_reg_cache(codec, reg, val);
		}
	} else {
		value[0] = dac33_read_reg_cache(codec, reg);
	}

	return 0;
}

static int dac33_write(struct snd_soc_codec *codec, unsigned int reg,
		       unsigned int value)
{
	struct tlv320dac33_priv *dac33 = codec->private_data;
	u8 data[2];
	int ret = 0;

	/*
	 * data is
	 *   D15..D8 dac33 register offset
	 *   D7...D0 register data
	 */
	data[0] = reg & 0xff;
	data[1] = value & 0xff;

	dac33_write_reg_cache(codec, data[0], data[1]);
	if (dac33->chip_power) {
		ret = codec->hw_write(codec->control_data, data, 2);
		if (ret != 2)
			dev_err(codec->dev, "Write failed (%d)\n", ret);
		else
			ret = 0;
	}

	return ret;
}

static int dac33_write_locked(struct snd_soc_codec *codec, unsigned int reg,
		       unsigned int value)
{
	struct tlv320dac33_priv *dac33 = codec->private_data;
	int ret;

	mutex_lock(&dac33->mutex);
	ret = dac33_write(codec, reg, value);
	mutex_unlock(&dac33->mutex);

	return ret;
}

#define DAC33_I2C_ADDR_AUTOINC	0x80
static int dac33_write16(struct snd_soc_codec *codec, unsigned int reg,
		       unsigned int value)
{
	struct tlv320dac33_priv *dac33 = codec->private_data;
	u8 data[3];
	int ret = 0;

	/*
	 * data is
	 *   D23..D16 dac33 register offset
	 *   D15..D8  register data MSB
	 *   D7...D0  register data LSB
	 */
	data[0] = reg & 0xff;
	data[1] = (value >> 8) & 0xff;
	data[2] = value & 0xff;

	dac33_write_reg_cache(codec, data[0], data[1]);
	dac33_write_reg_cache(codec, data[0] + 1, data[2]);

	if (dac33->chip_power) {
		/* We need to set autoincrement mode for 16 bit writes */
		data[0] |= DAC33_I2C_ADDR_AUTOINC;
		ret = codec->hw_write(codec->control_data, data, 3);
		if (ret != 3)
			dev_err(codec->dev, "Write failed (%d)\n", ret);
		else
			ret = 0;
	}

	return ret;
}

static void dac33_restore_regs(struct snd_soc_codec *codec)
{
	struct tlv320dac33_priv *dac33 = codec->private_data;
	u8 *cache = codec->reg_cache;
	u8 data[2];
	int i, ret;

	if (!dac33->chip_power)
		return;

	for (i = DAC33_PWR_CTRL; i <= DAC33_INTP_CTRL_B; i++) {
		data[0] = i;
		data[1] = cache[i];
		/* Skip the read only registers */
		if ((i >= DAC33_INT_OSC_STATUS &&
				i <= DAC33_INT_OSC_FREQ_RAT_READ_B) ||
		    (i >= DAC33_FIFO_WPTR_MSB && i <= DAC33_FIFO_IRQ_FLAG) ||
		    i == DAC33_DAC_STATUS_FLAGS ||
		    i == DAC33_SRC_EST_REF_CLK_RATIO_A ||
		    i == DAC33_SRC_EST_REF_CLK_RATIO_B)
			continue;
		ret = codec->hw_write(codec->control_data, data, 2);
		if (ret != 2)
			dev_err(codec->dev, "Write failed (%d)\n", ret);
	}
	for (i = DAC33_LDAC_PWR_CTRL; i <= DAC33_LINEL_TO_LLO_VOL; i++) {
		data[0] = i;
		data[1] = cache[i];
		ret = codec->hw_write(codec->control_data, data, 2);
		if (ret != 2)
			dev_err(codec->dev, "Write failed (%d)\n", ret);
	}
	for (i = DAC33_LINER_TO_RLO_VOL; i <= DAC33_OSC_TRIM; i++) {
		data[0] = i;
		data[1] = cache[i];
		ret = codec->hw_write(codec->control_data, data, 2);
		if (ret != 2)
			dev_err(codec->dev, "Write failed (%d)\n", ret);
	}
}

static inline void dac33_soft_power(struct snd_soc_codec *codec, int power)
{
	u8 reg;

	reg = dac33_read_reg_cache(codec, DAC33_PWR_CTRL);
	if (power)
		reg |= DAC33_PDNALLB;
	else
		reg &= ~DAC33_PDNALLB;
	dac33_write(codec, DAC33_PWR_CTRL, reg);
}

static int dac33_hard_power(struct snd_soc_codec *codec, int power)
{
	struct tlv320dac33_priv *dac33 = codec->private_data;
	int ret;

	mutex_lock(&dac33->mutex);
	if (power) {
		ret = regulator_bulk_enable(ARRAY_SIZE(dac33->supplies),
					  dac33->supplies);
		if (ret != 0) {
			dev_err(codec->dev,
				"Failed to enable supplies: %d\n", ret);
				goto exit;
		}

		if (dac33->power_gpio >= 0)
			gpio_set_value(dac33->power_gpio, 1);

		dac33->chip_power = 1;

		/* Restore registers */
		dac33_restore_regs(codec);

		dac33_soft_power(codec, 1);
	} else {
		dac33_soft_power(codec, 0);
		if (dac33->power_gpio >= 0)
			gpio_set_value(dac33->power_gpio, 0);

		ret = regulator_bulk_disable(ARRAY_SIZE(dac33->supplies),
					     dac33->supplies);
		if (ret != 0) {
			dev_err(codec->dev,
				"Failed to disable supplies: %d\n", ret);
			goto exit;
		}

		dac33->chip_power = 0;
	}

exit:
	mutex_unlock(&dac33->mutex);
	return ret;
}

static int dac33_get_nsample(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct tlv320dac33_priv *dac33 = codec->private_data;

	ucontrol->value.integer.value[0] = dac33->nsample;

	return 0;
}

static int dac33_set_nsample(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct tlv320dac33_priv *dac33 = codec->private_data;
	int ret = 0;

	if (dac33->nsample == ucontrol->value.integer.value[0])
		return 0;

	if (ucontrol->value.integer.value[0] < dac33->nsample_min ||
	    ucontrol->value.integer.value[0] > dac33->nsample_max)
		ret = -EINVAL;
	else
		dac33->nsample = ucontrol->value.integer.value[0];

	return ret;
}

static int dac33_get_fifo_mode(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct tlv320dac33_priv *dac33 = codec->private_data;

	ucontrol->value.integer.value[0] = dac33->fifo_mode;

	return 0;
}

static int dac33_set_fifo_mode(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct tlv320dac33_priv *dac33 = codec->private_data;
	int ret = 0;

	if (dac33->fifo_mode == ucontrol->value.integer.value[0])
		return 0;
	/* Do not allow changes while stream is running*/
	if (codec->active)
		return -EPERM;

	if (ucontrol->value.integer.value[0] < 0 ||
	    ucontrol->value.integer.value[0] >= DAC33_FIFO_LAST_MODE)
		ret = -EINVAL;
	else
		dac33->fifo_mode = ucontrol->value.integer.value[0];

	return ret;
}

/* Codec operation modes */
static const char *dac33_fifo_mode_texts[] = {
	"Bypass", "Mode 1"
};

static const struct soc_enum dac33_fifo_mode_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(dac33_fifo_mode_texts),
			    dac33_fifo_mode_texts);

/*
 * DACL/R digital volume control:
 * from 0 dB to -63.5 in 0.5 dB steps
 * Need to be inverted later on:
 * 0x00 == 0 dB
 * 0x7f == -63.5 dB
 */
static DECLARE_TLV_DB_SCALE(dac_digivol_tlv, -6350, 50, 0);

static const struct snd_kcontrol_new dac33_snd_controls[] = {
	SOC_DOUBLE_R_TLV("DAC Digital Playback Volume",
		DAC33_LDAC_DIG_VOL_CTRL, DAC33_RDAC_DIG_VOL_CTRL,
		0, 0x7f, 1, dac_digivol_tlv),
	SOC_DOUBLE_R("DAC Digital Playback Switch",
		 DAC33_LDAC_DIG_VOL_CTRL, DAC33_RDAC_DIG_VOL_CTRL, 7, 1, 1),
	SOC_DOUBLE_R("Line to Line Out Volume",
		 DAC33_LINEL_TO_LLO_VOL, DAC33_LINER_TO_RLO_VOL, 0, 127, 1),
};

static const struct snd_kcontrol_new dac33_nsample_snd_controls[] = {
	SOC_SINGLE_EXT("nSample", 0, 0, 5900, 0,
		 dac33_get_nsample, dac33_set_nsample),
	SOC_ENUM_EXT("FIFO Mode", dac33_fifo_mode_enum,
		 dac33_get_fifo_mode, dac33_set_fifo_mode),
};

/* Analog bypass */
static const struct snd_kcontrol_new dac33_dapm_abypassl_control =
	SOC_DAPM_SINGLE("Switch", DAC33_LINEL_TO_LLO_VOL, 7, 1, 1);

static const struct snd_kcontrol_new dac33_dapm_abypassr_control =
	SOC_DAPM_SINGLE("Switch", DAC33_LINER_TO_RLO_VOL, 7, 1, 1);

static const struct snd_soc_dapm_widget dac33_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("LEFT_LO"),
	SND_SOC_DAPM_OUTPUT("RIGHT_LO"),

	SND_SOC_DAPM_INPUT("LINEL"),
	SND_SOC_DAPM_INPUT("LINER"),

	SND_SOC_DAPM_DAC("DACL", "Left Playback", DAC33_LDAC_PWR_CTRL, 2, 0),
	SND_SOC_DAPM_DAC("DACR", "Right Playback", DAC33_RDAC_PWR_CTRL, 2, 0),

	/* Analog bypass */
	SND_SOC_DAPM_SWITCH("Analog Left Bypass", SND_SOC_NOPM, 0, 0,
				&dac33_dapm_abypassl_control),
	SND_SOC_DAPM_SWITCH("Analog Right Bypass", SND_SOC_NOPM, 0, 0,
				&dac33_dapm_abypassr_control),

	SND_SOC_DAPM_REG(snd_soc_dapm_mixer, "Output Left Amp Power",
			 DAC33_OUT_AMP_PWR_CTRL, 6, 3, 3, 0),
	SND_SOC_DAPM_REG(snd_soc_dapm_mixer, "Output Right Amp Power",
			 DAC33_OUT_AMP_PWR_CTRL, 4, 3, 3, 0),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* Analog bypass */
	{"Analog Left Bypass", "Switch", "LINEL"},
	{"Analog Right Bypass", "Switch", "LINER"},

	{"Output Left Amp Power", NULL, "DACL"},
	{"Output Right Amp Power", NULL, "DACR"},

	{"Output Left Amp Power", NULL, "Analog Left Bypass"},
	{"Output Right Amp Power", NULL, "Analog Right Bypass"},

	/* output */
	{"LEFT_LO", NULL, "Output Left Amp Power"},
	{"RIGHT_LO", NULL, "Output Right Amp Power"},
};

static int dac33_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, dac33_dapm_widgets,
				  ARRAY_SIZE(dac33_dapm_widgets));

	/* set up audio path interconnects */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	return 0;
}

static int dac33_set_bias_level(struct snd_soc_codec *codec,
				enum snd_soc_bias_level level)
{
	int ret;

	switch (level) {
	case SND_SOC_BIAS_ON:
		dac33_soft_power(codec, 1);
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		if (codec->bias_level == SND_SOC_BIAS_OFF) {
			ret = dac33_hard_power(codec, 1);
			if (ret != 0)
				return ret;
		}

		dac33_soft_power(codec, 0);
		break;
	case SND_SOC_BIAS_OFF:
		ret = dac33_hard_power(codec, 0);
		if (ret != 0)
			return ret;

		break;
	}
	codec->bias_level = level;

	return 0;
}

static inline void dac33_prefill_handler(struct tlv320dac33_priv *dac33)
{
	struct snd_soc_codec *codec;

	codec = &dac33->codec;

	switch (dac33->fifo_mode) {
	case DAC33_FIFO_MODE1:
		dac33_write16(codec, DAC33_NSAMPLE_MSB,
				DAC33_THRREG(dac33->nsample));
		dac33_write16(codec, DAC33_PREFILL_MSB,
				DAC33_THRREG(dac33->alarm_threshold));
		break;
	default:
		dev_warn(codec->dev, "Unhandled FIFO mode: %d\n",
							dac33->fifo_mode);
		break;
	}
}

static inline void dac33_playback_handler(struct tlv320dac33_priv *dac33)
{
	struct snd_soc_codec *codec;

	codec = &dac33->codec;

	switch (dac33->fifo_mode) {
	case DAC33_FIFO_MODE1:
		dac33_write16(codec, DAC33_NSAMPLE_MSB,
				DAC33_THRREG(dac33->nsample));
		break;
	default:
		dev_warn(codec->dev, "Unhandled FIFO mode: %d\n",
							dac33->fifo_mode);
		break;
	}
}

static void dac33_work(struct work_struct *work)
{
	struct snd_soc_codec *codec;
	struct tlv320dac33_priv *dac33;
	u8 reg;

	dac33 = container_of(work, struct tlv320dac33_priv, work);
	codec = &dac33->codec;

	mutex_lock(&dac33->mutex);
	switch (dac33->state) {
	case DAC33_PREFILL:
		dac33->state = DAC33_PLAYBACK;
		dac33_prefill_handler(dac33);
		break;
	case DAC33_PLAYBACK:
		dac33_playback_handler(dac33);
		break;
	case DAC33_IDLE:
		break;
	case DAC33_FLUSH:
		dac33->state = DAC33_IDLE;
		/* Mask all interrupts from dac33 */
		dac33_write(codec, DAC33_FIFO_IRQ_MASK, 0);

		/* flush fifo */
		reg = dac33_read_reg_cache(codec, DAC33_FIFO_CTRL_A);
		reg |= DAC33_FIFOFLUSH;
		dac33_write(codec, DAC33_FIFO_CTRL_A, reg);
		break;
	}
	mutex_unlock(&dac33->mutex);
}

static irqreturn_t dac33_interrupt_handler(int irq, void *dev)
{
	struct snd_soc_codec *codec = dev;
	struct tlv320dac33_priv *dac33 = codec->private_data;

	queue_work(dac33->dac33_wq, &dac33->work);

	return IRQ_HANDLED;
}

static void dac33_shutdown(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct tlv320dac33_priv *dac33 = codec->private_data;
	unsigned int pwr_ctrl;

	/* Stop pending workqueue */
	if (dac33->fifo_mode)
		cancel_work_sync(&dac33->work);

	mutex_lock(&dac33->mutex);
	pwr_ctrl = dac33_read_reg_cache(codec, DAC33_PWR_CTRL);
	pwr_ctrl &= ~(DAC33_OSCPDNB | DAC33_DACRPDNB | DAC33_DACLPDNB);
	dac33_write(codec, DAC33_PWR_CTRL, pwr_ctrl);
	mutex_unlock(&dac33->mutex);
}

static void dac33_oscwait(struct snd_soc_codec *codec)
{
	int timeout = 20;
	u8 reg;

	do {
		msleep(1);
		dac33_read(codec, DAC33_INT_OSC_STATUS, &reg);
	} while (((reg & 0x03) != DAC33_OSCSTATUS_NORMAL) && timeout--);
	if ((reg & 0x03) != DAC33_OSCSTATUS_NORMAL)
		dev_err(codec->dev,
			"internal oscillator calibration failed\n");
}

static int dac33_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params,
			   struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;

	/* Check parameters for validity */
	switch (params_rate(params)) {
	case 44100:
	case 48000:
		break;
	default:
		dev_err(codec->dev, "unsupported rate %d\n",
			params_rate(params));
		return -EINVAL;
	}

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	default:
		dev_err(codec->dev, "unsupported format %d\n",
			params_format(params));
		return -EINVAL;
	}

	return 0;
}

#define CALC_OSCSET(rate, refclk) ( \
	((((rate * 10000) / refclk) * 4096) + 5000) / 10000)
#define CALC_RATIOSET(rate, refclk) ( \
	((((refclk  * 100000) / rate) * 16384) + 50000) / 100000)

/*
 * tlv320dac33 is strict on the sequence of the register writes, if the register
 * writes happens in different order, than dac33 might end up in unknown state.
 * Use the known, working sequence of register writes to initialize the dac33.
 */
static int dac33_prepare_chip(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct tlv320dac33_priv *dac33 = codec->private_data;
	unsigned int oscset, ratioset, pwr_ctrl, reg_tmp;
	u8 aictrl_a, fifoctrl_a;

	switch (substream->runtime->rate) {
	case 44100:
	case 48000:
		oscset = CALC_OSCSET(substream->runtime->rate, dac33->refclk);
		ratioset = CALC_RATIOSET(substream->runtime->rate,
					 dac33->refclk);
		break;
	default:
		dev_err(codec->dev, "unsupported rate %d\n",
			substream->runtime->rate);
		return -EINVAL;
	}


	aictrl_a = dac33_read_reg_cache(codec, DAC33_SER_AUDIOIF_CTRL_A);
	aictrl_a &= ~(DAC33_NCYCL_MASK | DAC33_WLEN_MASK);
	fifoctrl_a = dac33_read_reg_cache(codec, DAC33_FIFO_CTRL_A);
	fifoctrl_a &= ~DAC33_WIDTH;
	switch (substream->runtime->format) {
	case SNDRV_PCM_FORMAT_S16_LE:
		aictrl_a |= (DAC33_NCYCL_16 | DAC33_WLEN_16);
		fifoctrl_a |= DAC33_WIDTH;
		break;
	default:
		dev_err(codec->dev, "unsupported format %d\n",
			substream->runtime->format);
		return -EINVAL;
	}

	mutex_lock(&dac33->mutex);
	dac33_soft_power(codec, 1);

	reg_tmp = dac33_read_reg_cache(codec, DAC33_INT_OSC_CTRL);
	dac33_write(codec, DAC33_INT_OSC_CTRL, reg_tmp);

	/* Write registers 0x08 and 0x09 (MSB, LSB) */
	dac33_write16(codec, DAC33_INT_OSC_FREQ_RAT_A, oscset);

	/* calib time: 128 is a nice number ;) */
	dac33_write(codec, DAC33_CALIB_TIME, 128);

	/* adjustment treshold & step */
	dac33_write(codec, DAC33_INT_OSC_CTRL_B, DAC33_ADJTHRSHLD(2) |
						 DAC33_ADJSTEP(1));

	/* div=4 / gain=1 / div */
	dac33_write(codec, DAC33_INT_OSC_CTRL_C, DAC33_REFDIV(4));

	pwr_ctrl = dac33_read_reg_cache(codec, DAC33_PWR_CTRL);
	pwr_ctrl |= DAC33_OSCPDNB | DAC33_DACRPDNB | DAC33_DACLPDNB;
	dac33_write(codec, DAC33_PWR_CTRL, pwr_ctrl);

	dac33_oscwait(codec);

	if (dac33->fifo_mode) {
		/* 50-51 : ASRC Control registers */
		dac33_write(codec, DAC33_ASRC_CTRL_A, (1 << 4)); /* div=2 */
		dac33_write(codec, DAC33_ASRC_CTRL_B, 1); /* ??? */

		/* Write registers 0x34 and 0x35 (MSB, LSB) */
		dac33_write16(codec, DAC33_SRC_REF_CLK_RATIO_A, ratioset);

		/* Set interrupts to high active */
		dac33_write(codec, DAC33_INTP_CTRL_A, DAC33_INTPM_AHIGH);

		dac33_write(codec, DAC33_FIFO_IRQ_MODE_B,
			    DAC33_ATM(DAC33_FIFO_IRQ_MODE_LEVEL));
		dac33_write(codec, DAC33_FIFO_IRQ_MASK, DAC33_MAT);
	} else {
		/* 50-51 : ASRC Control registers */
		dac33_write(codec, DAC33_ASRC_CTRL_A, DAC33_SRCBYP);
		dac33_write(codec, DAC33_ASRC_CTRL_B, 0); /* ??? */
	}

	if (dac33->fifo_mode)
		fifoctrl_a &= ~DAC33_FBYPAS;
	else
		fifoctrl_a |= DAC33_FBYPAS;
	dac33_write(codec, DAC33_FIFO_CTRL_A, fifoctrl_a);

	dac33_write(codec, DAC33_SER_AUDIOIF_CTRL_A, aictrl_a);
	reg_tmp = dac33_read_reg_cache(codec, DAC33_SER_AUDIOIF_CTRL_B);
	if (dac33->fifo_mode)
		reg_tmp &= ~DAC33_BCLKON;
	else
		reg_tmp |= DAC33_BCLKON;
	dac33_write(codec, DAC33_SER_AUDIOIF_CTRL_B, reg_tmp);

	if (dac33->fifo_mode) {
		/* 20: BCLK divide ratio */
		dac33_write(codec, DAC33_SER_AUDIOIF_CTRL_C, 3);

		dac33_write16(codec, DAC33_ATHR_MSB,
			      DAC33_THRREG(dac33->alarm_threshold));
	} else {
		dac33_write(codec, DAC33_SER_AUDIOIF_CTRL_C, 32);
	}

	mutex_unlock(&dac33->mutex);

	return 0;
}

static void dac33_calculate_times(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct tlv320dac33_priv *dac33 = codec->private_data;
	unsigned int nsample_limit;

	/* Number of samples (16bit, stereo) in one period */
	dac33->nsample_min = snd_pcm_lib_period_bytes(substream) / 4;

	/* Number of samples (16bit, stereo) in ALSA buffer */
	dac33->nsample_max = snd_pcm_lib_buffer_bytes(substream) / 4;
	/* Subtract one period from the total */
	dac33->nsample_max -= dac33->nsample_min;

	/* Number of samples for LATENCY_TIME_MS / 2 */
	dac33->alarm_threshold = substream->runtime->rate /
				 (1000 / (LATENCY_TIME_MS / 2));

	/* Find and fix up the lowest nsmaple limit */
	nsample_limit = substream->runtime->rate / (1000 / LATENCY_TIME_MS);

	if (dac33->nsample_min < nsample_limit)
		dac33->nsample_min = nsample_limit;

	if (dac33->nsample < dac33->nsample_min)
		dac33->nsample = dac33->nsample_min;

	/*
	 * Find and fix up the highest nsmaple limit
	 * In order to not overflow the DAC33 buffer substract the
	 * alarm_threshold value from the size of the DAC33 buffer
	 */
	nsample_limit = DAC33_BUFFER_SIZE_SAMPLES - dac33->alarm_threshold;

	if (dac33->nsample_max > nsample_limit)
		dac33->nsample_max = nsample_limit;

	if (dac33->nsample > dac33->nsample_max)
		dac33->nsample = dac33->nsample_max;
}

static int dac33_pcm_prepare(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	dac33_calculate_times(substream);
	dac33_prepare_chip(substream);

	return 0;
}

static int dac33_pcm_trigger(struct snd_pcm_substream *substream, int cmd,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct tlv320dac33_priv *dac33 = codec->private_data;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (dac33->fifo_mode) {
			dac33->state = DAC33_PREFILL;
			queue_work(dac33->dac33_wq, &dac33->work);
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (dac33->fifo_mode) {
			dac33->state = DAC33_FLUSH;
			queue_work(dac33->dac33_wq, &dac33->work);
		}
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int dac33_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct tlv320dac33_priv *dac33 = codec->private_data;
	u8 ioc_reg, asrcb_reg;

	ioc_reg = dac33_read_reg_cache(codec, DAC33_INT_OSC_CTRL);
	asrcb_reg = dac33_read_reg_cache(codec, DAC33_ASRC_CTRL_B);
	switch (clk_id) {
	case TLV320DAC33_MCLK:
		ioc_reg |= DAC33_REFSEL;
		asrcb_reg |= DAC33_SRCREFSEL;
		break;
	case TLV320DAC33_SLEEPCLK:
		ioc_reg &= ~DAC33_REFSEL;
		asrcb_reg &= ~DAC33_SRCREFSEL;
		break;
	default:
		dev_err(codec->dev, "Invalid clock ID (%d)\n", clk_id);
		break;
	}
	dac33->refclk = freq;

	dac33_write_reg_cache(codec, DAC33_INT_OSC_CTRL, ioc_reg);
	dac33_write_reg_cache(codec, DAC33_ASRC_CTRL_B, asrcb_reg);

	return 0;
}

static int dac33_set_dai_fmt(struct snd_soc_dai *codec_dai,
			     unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u8 aictrl_a, aictrl_b;

	aictrl_a = dac33_read_reg_cache(codec, DAC33_SER_AUDIOIF_CTRL_A);
	aictrl_b = dac33_read_reg_cache(codec, DAC33_SER_AUDIOIF_CTRL_B);
	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		/* Codec Master */
		aictrl_a |= (DAC33_MSBCLK | DAC33_MSWCLK);
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		/* Codec Slave */
		aictrl_a &= ~(DAC33_MSBCLK | DAC33_MSWCLK);
		break;
	default:
		return -EINVAL;
	}

	aictrl_a &= ~DAC33_AFMT_MASK;
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		aictrl_a |= DAC33_AFMT_I2S;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		aictrl_a |= DAC33_AFMT_DSP;
		aictrl_b &= ~DAC33_DATA_DELAY_MASK;
		aictrl_b |= DAC33_DATA_DELAY(1); /* 1 bit delay */
		break;
	case SND_SOC_DAIFMT_DSP_B:
		aictrl_a |= DAC33_AFMT_DSP;
		aictrl_b &= ~DAC33_DATA_DELAY_MASK; /* No delay */
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		aictrl_a |= DAC33_AFMT_RIGHT_J;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		aictrl_a |= DAC33_AFMT_LEFT_J;
		break;
	default:
		dev_err(codec->dev, "Unsupported format (%u)\n",
			fmt & SND_SOC_DAIFMT_FORMAT_MASK);
		return -EINVAL;
	}

	dac33_write_reg_cache(codec, DAC33_SER_AUDIOIF_CTRL_A, aictrl_a);
	dac33_write_reg_cache(codec, DAC33_SER_AUDIOIF_CTRL_B, aictrl_b);

	return 0;
}

static void dac33_init_chip(struct snd_soc_codec *codec)
{
	/* 44-46: DAC Control Registers */
	/* A : DAC sample rate Fsref/1.5 */
	dac33_write(codec, DAC33_DAC_CTRL_A, DAC33_DACRATE(1));
	/* B : DAC src=normal, not muted */
	dac33_write(codec, DAC33_DAC_CTRL_B, DAC33_DACSRCR_RIGHT |
					     DAC33_DACSRCL_LEFT);
	/* C : (defaults) */
	dac33_write(codec, DAC33_DAC_CTRL_C, 0x00);

	/* 64-65 : L&R DAC power control
	 Line In -> OUT 1V/V Gain, DAC -> OUT 4V/V Gain*/
	dac33_write(codec, DAC33_LDAC_PWR_CTRL, DAC33_LROUT_GAIN(2));
	dac33_write(codec, DAC33_RDAC_PWR_CTRL, DAC33_LROUT_GAIN(2));

	/* 73 : volume soft stepping control,
	 clock source = internal osc (?) */
	dac33_write(codec, DAC33_ANA_VOL_SOFT_STEP_CTRL, DAC33_VOLCLKEN);

	/* 66 : LOP/LOM Modes */
	dac33_write(codec, DAC33_OUT_AMP_CM_CTRL, 0xff);

	/* 68 : LOM inverted from LOP */
	dac33_write(codec, DAC33_OUT_AMP_CTRL, (3<<2));

	dac33_write(codec, DAC33_PWR_CTRL, DAC33_PDNALLB);
}

static int dac33_soc_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	struct tlv320dac33_priv *dac33;
	int ret = 0;

	BUG_ON(!tlv320dac33_codec);

	codec = tlv320dac33_codec;
	socdev->card->codec = codec;
	dac33 = codec->private_data;

	/* Power up the codec */
	dac33_hard_power(codec, 1);
	/* Set default configuration */
	dac33_init_chip(codec);

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		dev_err(codec->dev, "failed to create pcms\n");
		goto pcm_err;
	}

	snd_soc_add_controls(codec, dac33_snd_controls,
			     ARRAY_SIZE(dac33_snd_controls));
	/* Only add the nSample controls, if we have valid IRQ number */
	if (dac33->irq >= 0)
		snd_soc_add_controls(codec, dac33_nsample_snd_controls,
				     ARRAY_SIZE(dac33_nsample_snd_controls));

	dac33_add_widgets(codec);

	/* power on device */
	dac33_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	/* Bias level configuration has enabled regulator an extra time */
	regulator_bulk_disable(ARRAY_SIZE(dac33->supplies), dac33->supplies);

	return 0;

pcm_err:
	dac33_hard_power(codec, 0);
	return ret;
}

static int dac33_soc_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	dac33_set_bias_level(codec, SND_SOC_BIAS_OFF);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);

	return 0;
}

static int dac33_soc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	dac33_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int dac33_soc_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	dac33_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	dac33_set_bias_level(codec, codec->suspend_bias_level);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_tlv320dac33 = {
	.probe = dac33_soc_probe,
	.remove = dac33_soc_remove,
	.suspend = dac33_soc_suspend,
	.resume = dac33_soc_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_tlv320dac33);

#define DAC33_RATES	(SNDRV_PCM_RATE_44100 | \
			 SNDRV_PCM_RATE_48000)
#define DAC33_FORMATS	SNDRV_PCM_FMTBIT_S16_LE

static struct snd_soc_dai_ops dac33_dai_ops = {
	.shutdown	= dac33_shutdown,
	.hw_params	= dac33_hw_params,
	.prepare	= dac33_pcm_prepare,
	.trigger	= dac33_pcm_trigger,
	.set_sysclk	= dac33_set_dai_sysclk,
	.set_fmt	= dac33_set_dai_fmt,
};

struct snd_soc_dai dac33_dai = {
	.name = "tlv320dac33",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = DAC33_RATES,
		.formats = DAC33_FORMATS,},
	.ops = &dac33_dai_ops,
};
EXPORT_SYMBOL_GPL(dac33_dai);

static int dac33_i2c_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct tlv320dac33_platform_data *pdata;
	struct tlv320dac33_priv *dac33;
	struct snd_soc_codec *codec;
	int ret, i;

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "Platform data not set\n");
		return -ENODEV;
	}
	pdata = client->dev.platform_data;

	dac33 = kzalloc(sizeof(struct tlv320dac33_priv), GFP_KERNEL);
	if (dac33 == NULL)
		return -ENOMEM;

	codec = &dac33->codec;
	codec->private_data = dac33;
	codec->control_data = client;

	mutex_init(&codec->mutex);
	mutex_init(&dac33->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	codec->name = "tlv320dac33";
	codec->owner = THIS_MODULE;
	codec->read = dac33_read_reg_cache;
	codec->write = dac33_write_locked;
	codec->hw_write = (hw_write_t) i2c_master_send;
	codec->bias_level = SND_SOC_BIAS_OFF;
	codec->set_bias_level = dac33_set_bias_level;
	codec->dai = &dac33_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = ARRAY_SIZE(dac33_reg);
	codec->reg_cache = kmemdup(dac33_reg, ARRAY_SIZE(dac33_reg),
				   GFP_KERNEL);
	if (codec->reg_cache == NULL) {
		ret = -ENOMEM;
		goto error_reg;
	}

	i2c_set_clientdata(client, dac33);

	dac33->power_gpio = pdata->power_gpio;
	dac33->irq = client->irq;
	dac33->nsample = NSAMPLE_MAX;
	/* Disable FIFO use by default */
	dac33->fifo_mode = DAC33_FIFO_BYPASS;

	tlv320dac33_codec = codec;

	codec->dev = &client->dev;
	dac33_dai.dev = codec->dev;

	/* Check if the reset GPIO number is valid and request it */
	if (dac33->power_gpio >= 0) {
		ret = gpio_request(dac33->power_gpio, "tlv320dac33 reset");
		if (ret < 0) {
			dev_err(codec->dev,
				"Failed to request reset GPIO (%d)\n",
				dac33->power_gpio);
			snd_soc_unregister_dai(&dac33_dai);
			snd_soc_unregister_codec(codec);
			goto error_gpio;
		}
		gpio_direction_output(dac33->power_gpio, 0);
	} else {
		dac33->chip_power = 1;
	}

	/* Check if the IRQ number is valid and request it */
	if (dac33->irq >= 0) {
		ret = request_irq(dac33->irq, dac33_interrupt_handler,
				  IRQF_TRIGGER_RISING | IRQF_DISABLED,
				  codec->name, codec);
		if (ret < 0) {
			dev_err(codec->dev, "Could not request IRQ%d (%d)\n",
						dac33->irq, ret);
			dac33->irq = -1;
		}
		if (dac33->irq != -1) {
			/* Setup work queue */
			dac33->dac33_wq =
				create_singlethread_workqueue("tlv320dac33");
			if (dac33->dac33_wq == NULL) {
				free_irq(dac33->irq, &dac33->codec);
				ret = -ENOMEM;
				goto error_wq;
			}

			INIT_WORK(&dac33->work, dac33_work);
		}
	}

	for (i = 0; i < ARRAY_SIZE(dac33->supplies); i++)
		dac33->supplies[i].supply = dac33_supply_names[i];

	ret = regulator_bulk_get(codec->dev, ARRAY_SIZE(dac33->supplies),
				 dac33->supplies);

	if (ret != 0) {
		dev_err(codec->dev, "Failed to request supplies: %d\n", ret);
		goto err_get;
	}

	ret = regulator_bulk_enable(ARRAY_SIZE(dac33->supplies),
				    dac33->supplies);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to enable supplies: %d\n", ret);
		goto err_enable;
	}

	ret = snd_soc_register_codec(codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register codec: %d\n", ret);
		goto error_codec;
	}

	ret = snd_soc_register_dai(&dac33_dai);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register DAI: %d\n", ret);
		snd_soc_unregister_codec(codec);
		goto error_codec;
	}

	/* Shut down the codec for now */
	dac33_hard_power(codec, 0);

	return ret;

error_codec:
	regulator_bulk_disable(ARRAY_SIZE(dac33->supplies), dac33->supplies);
err_enable:
	regulator_bulk_free(ARRAY_SIZE(dac33->supplies), dac33->supplies);
err_get:
	if (dac33->irq >= 0) {
		free_irq(dac33->irq, &dac33->codec);
		destroy_workqueue(dac33->dac33_wq);
	}
error_wq:
	if (dac33->power_gpio >= 0)
		gpio_free(dac33->power_gpio);
error_gpio:
	kfree(codec->reg_cache);
error_reg:
	tlv320dac33_codec = NULL;
	kfree(dac33);

	return ret;
}

static int dac33_i2c_remove(struct i2c_client *client)
{
	struct tlv320dac33_priv *dac33;

	dac33 = i2c_get_clientdata(client);
	dac33_hard_power(&dac33->codec, 0);

	if (dac33->power_gpio >= 0)
		gpio_free(dac33->power_gpio);
	if (dac33->irq >= 0)
		free_irq(dac33->irq, &dac33->codec);

	regulator_bulk_free(ARRAY_SIZE(dac33->supplies), dac33->supplies);

	destroy_workqueue(dac33->dac33_wq);
	snd_soc_unregister_dai(&dac33_dai);
	snd_soc_unregister_codec(&dac33->codec);
	kfree(dac33->codec.reg_cache);
	kfree(dac33);
	tlv320dac33_codec = NULL;

	return 0;
}

static const struct i2c_device_id tlv320dac33_i2c_id[] = {
	{
		.name = "tlv320dac33",
		.driver_data = 0,
	},
	{ },
};

static struct i2c_driver tlv320dac33_i2c_driver = {
	.driver = {
		.name = "tlv320dac33",
		.owner = THIS_MODULE,
	},
	.probe		= dac33_i2c_probe,
	.remove		= __devexit_p(dac33_i2c_remove),
	.id_table	= tlv320dac33_i2c_id,
};

static int __init dac33_module_init(void)
{
	int r;
	r = i2c_add_driver(&tlv320dac33_i2c_driver);
	if (r < 0) {
		printk(KERN_ERR "DAC33: driver registration failed\n");
		return r;
	}
	return 0;
}
module_init(dac33_module_init);

static void __exit dac33_module_exit(void)
{
	i2c_del_driver(&tlv320dac33_i2c_driver);
}
module_exit(dac33_module_exit);


MODULE_DESCRIPTION("ASoC TLV320DAC33 codec driver");
MODULE_AUTHOR("Peter Ujfalusi <peter.ujfalusi@nokia.com>");
MODULE_LICENSE("GPL");
