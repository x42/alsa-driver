/*
 * ALC262 quirk models
 * included by patch_realtek.c
 */

/* ALC262 models */
enum {
	ALC262_AUTO,
	ALC262_BASIC,
	ALC262_ULTRA,
	ALC262_MODEL_LAST /* last tag */
};

#define ALC262_DIGOUT_NID	ALC880_DIGOUT_NID
#define ALC262_DIGIN_NID	ALC880_DIGIN_NID

#define alc262_dac_nids		alc260_dac_nids
#define alc262_adc_nids		alc882_adc_nids
#define alc262_adc_nids_alt	alc882_adc_nids_alt
#define alc262_capsrc_nids	alc882_capsrc_nids
#define alc262_capsrc_nids_alt	alc882_capsrc_nids_alt

#define alc262_modes		alc260_modes
#define alc262_capture_source	alc882_capture_source

static const hda_nid_t alc262_dmic_adc_nids[1] = {
	/* ADC0 */
	0x09
};

static const hda_nid_t alc262_dmic_capsrc_nids[1] = { 0x22 };

static const struct snd_kcontrol_new alc262_base_mixer[] = {
	HDA_CODEC_VOLUME("Front Playback Volume", 0x0c, 0x0, HDA_OUTPUT),
	HDA_CODEC_MUTE("Front Playback Switch", 0x14, 0x0, HDA_OUTPUT),
	HDA_CODEC_VOLUME("CD Playback Volume", 0x0b, 0x04, HDA_INPUT),
	HDA_CODEC_MUTE("CD Playback Switch", 0x0b, 0x04, HDA_INPUT),
	HDA_CODEC_VOLUME("Line Playback Volume", 0x0b, 0x02, HDA_INPUT),
	HDA_CODEC_MUTE("Line Playback Switch", 0x0b, 0x02, HDA_INPUT),
	HDA_CODEC_VOLUME("Mic Playback Volume", 0x0b, 0x0, HDA_INPUT),
	HDA_CODEC_MUTE("Mic Playback Switch", 0x0b, 0x0, HDA_INPUT),
	HDA_CODEC_VOLUME("Mic Boost Volume", 0x18, 0, HDA_INPUT),
	HDA_CODEC_VOLUME("Front Mic Playback Volume", 0x0b, 0x01, HDA_INPUT),
	HDA_CODEC_MUTE("Front Mic Playback Switch", 0x0b, 0x01, HDA_INPUT),
	HDA_CODEC_VOLUME("Front Mic Boost Volume", 0x19, 0, HDA_INPUT),
	HDA_CODEC_VOLUME("Headphone Playback Volume", 0x0D, 0x0, HDA_OUTPUT),
	HDA_CODEC_MUTE("Headphone Playback Switch", 0x15, 0x0, HDA_OUTPUT),
	HDA_CODEC_VOLUME_MONO("Mono Playback Volume", 0x0e, 2, 0x0, HDA_OUTPUT),
	HDA_CODEC_MUTE_MONO("Mono Playback Switch", 0x16, 2, 0x0, HDA_OUTPUT),
	{ } /* end */
};

#define alc262_capture_mixer		alc882_capture_mixer
#define alc262_capture_alt_mixer	alc882_capture_alt_mixer

/*
 * generic initialization of ADC, input mixers and output mixers
 */
static const struct hda_verb alc262_init_verbs[] = {
	/*
	 * Unmute ADC0-2 and set the default input to mic-in
	 */
	{0x07, AC_VERB_SET_CONNECT_SEL, 0x00},
	{0x07, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_UNMUTE(0)},
	{0x08, AC_VERB_SET_CONNECT_SEL, 0x00},
	{0x08, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_UNMUTE(0)},
	{0x09, AC_VERB_SET_CONNECT_SEL, 0x00},
	{0x09, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_UNMUTE(0)},

	/* Mute input amps (CD, Line In, Mic 1 & Mic 2) of the analog-loopback
	 * mixer widget
	 * Note: PASD motherboards uses the Line In 2 as the input for
	 * front panel mic (mic 2)
	 */
	/* Amp Indices: Mic1 = 0, Mic2 = 1, Line1 = 2, Line2 = 3, CD = 4 */
	{0x0b, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(0)},
	{0x0b, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(1)},
	{0x0b, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(2)},
	{0x0b, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(3)},
	{0x0b, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(4)},

	/*
	 * Set up output mixers (0x0c - 0x0e)
	 */
	/* set vol=0 to output mixers */
	{0x0c, AC_VERB_SET_AMP_GAIN_MUTE, AMP_OUT_ZERO},
	{0x0d, AC_VERB_SET_AMP_GAIN_MUTE, AMP_OUT_ZERO},
	{0x0e, AC_VERB_SET_AMP_GAIN_MUTE, AMP_OUT_ZERO},
	/* set up input amps for analog loopback */
	/* Amp Indices: DAC = 0, mixer = 1 */
	{0x0c, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_UNMUTE(0)},
	{0x0c, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_UNMUTE(1)},
	{0x0d, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_UNMUTE(0)},
	{0x0d, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_UNMUTE(1)},
	{0x0e, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_UNMUTE(0)},
	{0x0e, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_UNMUTE(1)},

	{0x14, AC_VERB_SET_PIN_WIDGET_CONTROL, 0x40},
	{0x15, AC_VERB_SET_PIN_WIDGET_CONTROL, 0xc0},
	{0x16, AC_VERB_SET_PIN_WIDGET_CONTROL, 0x40},
	{0x18, AC_VERB_SET_PIN_WIDGET_CONTROL, 0x24},
	{0x1a, AC_VERB_SET_PIN_WIDGET_CONTROL, 0x20},
	{0x1c, AC_VERB_SET_PIN_WIDGET_CONTROL, 0x20},

	{0x14, AC_VERB_SET_AMP_GAIN_MUTE, 0x0000},
	{0x15, AC_VERB_SET_AMP_GAIN_MUTE, 0x0000},
	{0x16, AC_VERB_SET_AMP_GAIN_MUTE, 0x0000},
	{0x18, AC_VERB_SET_AMP_GAIN_MUTE, 0x0000},
	{0x1a, AC_VERB_SET_AMP_GAIN_MUTE, 0x0000},

	{0x14, AC_VERB_SET_CONNECT_SEL, 0x00},
	{0x15, AC_VERB_SET_CONNECT_SEL, 0x01},

	/* FIXME: use matrix-type input source selection */
	/* Mixer elements: 0x18, 19, 1a, 1b, 1c, 1d, 14, 15, 16, 17, 0b */
	/* Input mixer1: unmute Mic, F-Mic, Line, CD inputs */
	{0x24, AC_VERB_SET_AMP_GAIN_MUTE, (0x7000 | (0x00 << 8))},
	{0x24, AC_VERB_SET_AMP_GAIN_MUTE, (0x7080 | (0x03 << 8))},
	{0x24, AC_VERB_SET_AMP_GAIN_MUTE, (0x7080 | (0x02 << 8))},
	{0x24, AC_VERB_SET_AMP_GAIN_MUTE, (0x7080 | (0x04 << 8))},
	/* Input mixer2 */
	{0x23, AC_VERB_SET_AMP_GAIN_MUTE, (0x7000 | (0x00 << 8))},
	{0x23, AC_VERB_SET_AMP_GAIN_MUTE, (0x7080 | (0x03 << 8))},
	{0x23, AC_VERB_SET_AMP_GAIN_MUTE, (0x7080 | (0x02 << 8))},
	{0x23, AC_VERB_SET_AMP_GAIN_MUTE, (0x7080 | (0x04 << 8))},
	/* Input mixer3 */
	{0x22, AC_VERB_SET_AMP_GAIN_MUTE, (0x7000 | (0x00 << 8))},
	{0x22, AC_VERB_SET_AMP_GAIN_MUTE, (0x7080 | (0x03 << 8))},
	{0x22, AC_VERB_SET_AMP_GAIN_MUTE, (0x7080 | (0x02 << 8))},
	{0x22, AC_VERB_SET_AMP_GAIN_MUTE, (0x7080 | (0x04 << 8))},

	{ }
};

/* Samsung Q1 Ultra Vista model setup */
static const struct snd_kcontrol_new alc262_ultra_mixer[] = {
	HDA_CODEC_VOLUME("Master Playback Volume", 0x0c, 0x0, HDA_OUTPUT),
	HDA_BIND_MUTE("Master Playback Switch", 0x0c, 2, HDA_INPUT),
	HDA_CODEC_VOLUME("Mic Playback Volume", 0x0b, 0x01, HDA_INPUT),
	HDA_CODEC_MUTE("Mic Playback Switch", 0x0b, 0x01, HDA_INPUT),
	HDA_CODEC_VOLUME("Mic Boost Volume", 0x19, 0, HDA_INPUT),
	HDA_CODEC_VOLUME("Headphone Mic Boost Volume", 0x15, 0, HDA_INPUT),
	{ } /* end */
};

static const struct hda_verb alc262_ultra_verbs[] = {
	/* output mixer */
	{0x0c, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(0)},
	{0x0c, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(1)},
	{0x0c, AC_VERB_SET_AMP_GAIN_MUTE, AMP_OUT_ZERO},
	/* speaker */
	{0x14, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_OUT},
	{0x14, AC_VERB_SET_AMP_GAIN_MUTE, AMP_OUT_MUTE},
	{0x14, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_UNMUTE(0)},
	{0x14, AC_VERB_SET_CONNECT_SEL, 0x00},
	/* HP */
	{0x15, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_HP},
	{0x15, AC_VERB_SET_AMP_GAIN_MUTE, AMP_OUT_MUTE},
	{0x15, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_UNMUTE(0)},
	{0x15, AC_VERB_SET_CONNECT_SEL, 0x00},
	{0x15, AC_VERB_SET_UNSOLICITED_ENABLE, AC_USRSP_EN | ALC_HP_EVENT},
	/* internal mic */
	{0x19, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_VREF80},
	{0x0b, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(1)},
	/* ADC, choose mic */
	{0x07, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(0)},
	{0x24, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(0)},
	{0x24, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_UNMUTE(1)},
	{0x24, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(2)},
	{0x24, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(3)},
	{0x24, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(4)},
	{0x24, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(5)},
	{0x24, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(6)},
	{0x24, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(7)},
	{0x24, AC_VERB_SET_AMP_GAIN_MUTE, AMP_IN_MUTE(8)},
	{}
};

/* mute/unmute internal speaker according to the hp jack and mute state */
static void alc262_ultra_automute(struct hda_codec *codec)
{
	struct alc_spec *spec = codec->spec;
	unsigned int mute;

	mute = 0;
	/* auto-mute only when HP is used as HP */
	if (!spec->cur_mux[0]) {
		spec->hp_jack_present = snd_hda_jack_detect(codec, 0x15);
		if (spec->hp_jack_present)
			mute = HDA_AMP_MUTE;
	}
	/* mute/unmute internal speaker */
	snd_hda_codec_amp_stereo(codec, 0x14, HDA_OUTPUT, 0,
				 HDA_AMP_MUTE, mute);
	/* mute/unmute HP */
	snd_hda_codec_amp_stereo(codec, 0x15, HDA_OUTPUT, 0,
				 HDA_AMP_MUTE, mute ? 0 : HDA_AMP_MUTE);
}

/* unsolicited event for HP jack sensing */
static void alc262_ultra_unsol_event(struct hda_codec *codec,
				       unsigned int res)
{
	if ((res >> 26) != ALC_HP_EVENT)
		return;
	alc262_ultra_automute(codec);
}

static const struct hda_input_mux alc262_ultra_capture_source = {
	.num_items = 2,
	.items = {
		{ "Mic", 0x1 },
		{ "Headphone", 0x7 },
	},
};

static int alc262_ultra_mux_enum_put(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct hda_codec *codec = snd_kcontrol_chip(kcontrol);
	struct alc_spec *spec = codec->spec;
	int ret;

	ret = alc_mux_enum_put(kcontrol, ucontrol);
	if (!ret)
		return 0;
	/* reprogram the HP pin as mic or HP according to the input source */
	snd_hda_codec_write_cache(codec, 0x15, 0,
				  AC_VERB_SET_PIN_WIDGET_CONTROL,
				  spec->cur_mux[0] ? PIN_VREF80 : PIN_HP);
	alc262_ultra_automute(codec); /* mute/unmute HP */
	return ret;
}

static const struct snd_kcontrol_new alc262_ultra_capture_mixer[] = {
	HDA_CODEC_VOLUME("Capture Volume", 0x07, 0x0, HDA_INPUT),
	HDA_CODEC_MUTE("Capture Switch", 0x07, 0x0, HDA_INPUT),
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Capture Source",
		.info = alc_mux_enum_info,
		.get = alc_mux_enum_get,
		.put = alc262_ultra_mux_enum_put,
	},
	{
		.iface = NID_MAPPING,
		.name = "Capture Source",
		.private_value = 0x15,
	},
	{ } /* end */
};

/*
 * configuration and preset
 */
static const char * const alc262_models[ALC262_MODEL_LAST] = {
	[ALC262_BASIC]		= "basic",
	[ALC262_ULTRA]		= "ultra",
	[ALC262_AUTO]		= "auto",
};

static const struct snd_pci_quirk alc262_cfg_tbl[] = {
	SND_PCI_QUIRK_MASK(0x144d, 0xff00, 0xc032, "Samsung Q1",
			   ALC262_ULTRA),
	{}
};

static const struct alc_config_preset alc262_presets[] = {
	[ALC262_BASIC] = {
		.mixers = { alc262_base_mixer },
		.init_verbs = { alc262_init_verbs },
		.num_dacs = ARRAY_SIZE(alc262_dac_nids),
		.dac_nids = alc262_dac_nids,
		.hp_nid = 0x03,
		.num_channel_mode = ARRAY_SIZE(alc262_modes),
		.channel_mode = alc262_modes,
		.input_mux = &alc262_capture_source,
	},
	[ALC262_ULTRA] = {
		.mixers = { alc262_ultra_mixer },
		.cap_mixer = alc262_ultra_capture_mixer,
		.init_verbs = { alc262_ultra_verbs },
		.num_dacs = ARRAY_SIZE(alc262_dac_nids),
		.dac_nids = alc262_dac_nids,
		.num_channel_mode = ARRAY_SIZE(alc262_modes),
		.channel_mode = alc262_modes,
		.input_mux = &alc262_ultra_capture_source,
		.adc_nids = alc262_adc_nids, /* ADC0 */
		.capsrc_nids = alc262_capsrc_nids,
		.num_adc_nids = 1, /* single ADC */
		.unsol_event = alc262_ultra_unsol_event,
		.init_hook = alc262_ultra_automute,
	},
};

