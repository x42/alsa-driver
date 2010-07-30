/*
 *  Loopback soundcard
 *
 *  Original code:
 *  Copyright (c) by Jaroslav Kysela <perex@perex.cz>
 *
 *  More accurate positioning and full-duplex support:
 *  Copyright (c) Ahmet İnan <ainan at mathematik.uni-freiburg.de>
 *
 *  Major (almost complete) rewrite:
 *  Copyright (c) by Takashi Iwai <tiwai@suse.de>
 *
 *  A next major update in 2010 (separate timers for playback and capture):
 *  Copyright (c) Jaroslav Kysela <perex@perex.cz>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/initval.h>

MODULE_AUTHOR("Jaroslav Kysela <perex@perex.cz>");
MODULE_DESCRIPTION("A loopback soundcard");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("{{ALSA,Loopback soundcard}}");

#define MAX_PCM_SUBSTREAMS	8

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;	/* Index 0-MAX */
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;	/* ID for this card */
static int enable[SNDRV_CARDS] = {1, [1 ... (SNDRV_CARDS - 1)] = 0};
static int pcm_substreams[SNDRV_CARDS] = {[0 ... (SNDRV_CARDS - 1)] = 8};
static int pcm_notify[SNDRV_CARDS];

module_param_array(index, int, NULL, 0444);
MODULE_PARM_DESC(index, "Index value for loopback soundcard.");
module_param_array(id, charp, NULL, 0444);
MODULE_PARM_DESC(id, "ID string for loopback soundcard.");
module_param_array(enable, bool, NULL, 0444);
MODULE_PARM_DESC(enable, "Enable this loopback soundcard.");
module_param_array(pcm_substreams, int, NULL, 0444);
MODULE_PARM_DESC(pcm_substreams, "PCM substreams # (1-8) for loopback driver.");
module_param_array(pcm_notify, int, NULL, 0444);
MODULE_PARM_DESC(pcm_notify, "Break capture when PCM format/rate/channels changes.");

struct loopback_pcm;

struct loopback_cable {
	spinlock_t lock;
	struct loopback_pcm *streams[2];
	struct snd_pcm_hardware hw;
	/* flags */
	unsigned int valid;
	unsigned int running;
};

struct loopback {
	struct snd_card *card;
	struct mutex cable_lock;
	struct loopback_cable *cables[MAX_PCM_SUBSTREAMS][2];
	struct snd_pcm *pcm[2];
	unsigned int notify: 1;
};

struct loopback_pcm {
	struct loopback *loopback;
	struct snd_pcm_substream *substream;
	struct loopback_cable *cable;
	unsigned int pcm_buffer_size;
	unsigned int buf_pos;	/* position in buffer */
	unsigned int silent_size;
	/* PCM parameters */
	unsigned int pcm_period_size;
	unsigned int pcm_bps;		/* bytes per second */
	unsigned int pcm_salign;	/* bytes per sample * channels */
	/* flags */
	unsigned int period_update_pending :1;
	/* timer stuff */
	unsigned int irq_pos;		/* fractional IRQ position */
	unsigned int period_size_frac;
	unsigned long last_jiffies;
	struct timer_list timer;
};

static struct platform_device *devices[SNDRV_CARDS];

static inline unsigned int byte_pos(struct loopback_pcm *dpcm, unsigned int x)
{
	x /= HZ;
	return x - (x % dpcm->pcm_salign);
}

#define frac_pos(x)	((x) * HZ)

static void loopback_timer_start(struct loopback_pcm *dpcm)
{
	unsigned long tick;

	tick = dpcm->period_size_frac - dpcm->irq_pos;
	tick = (tick + dpcm->pcm_bps - 1) / dpcm->pcm_bps;
	dpcm->timer.expires = jiffies + tick;
	add_timer(&dpcm->timer);
}

static inline void loopback_timer_stop(struct loopback_pcm *dpcm)
{
	del_timer(&dpcm->timer);
}

#define CABLE_VALID_PLAYBACK	(1 << SNDRV_PCM_STREAM_PLAYBACK)
#define CABLE_VALID_CAPTURE	(1 << SNDRV_PCM_STREAM_CAPTURE)
#define CABLE_VALID_BOTH	(CABLE_VALID_PLAYBACK|CABLE_VALID_CAPTURE)

static int loopback_check_format(struct loopback_cable *cable, int stream)
{
	struct snd_pcm_runtime *runtime;
	int check;

	if (cable->valid != CABLE_VALID_BOTH)
		return 0;
	runtime = cable->streams[SNDRV_PCM_STREAM_PLAYBACK]->
							substream->runtime;
	check = cable->hw.formats != (1ULL << runtime->format) ||
		cable->hw.rate_min != runtime->rate ||
		cable->hw.rate_max != runtime->rate ||
		cable->hw.channels_min != runtime->channels ||
		cable->hw.channels_max != runtime->channels;
	if (!check)
		return 0;
	if (stream == SNDRV_PCM_STREAM_CAPTURE) {
		return -EIO;
	} else {
		snd_pcm_stop(cable->streams[SNDRV_PCM_STREAM_CAPTURE]->
					substream, SNDRV_PCM_STATE_DRAINING);
	}
	return 0;
}

static int loopback_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct loopback_pcm *dpcm = runtime->private_data;
	struct loopback_cable *cable = dpcm->cable;
	int err;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		err = loopback_check_format(cable, substream->stream);
		if (err < 0)
			return err;
		dpcm->last_jiffies = jiffies;
		loopback_timer_start(dpcm);
		cable->running |= (1 << substream->stream);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		cable->running &= ~(1 << substream->stream);
		loopback_timer_stop(dpcm);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int loopback_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct loopback_pcm *dpcm = runtime->private_data;
	struct loopback_cable *cable = dpcm->cable;
	unsigned int bps, salign;

	salign = (snd_pcm_format_width(runtime->format) *
						runtime->channels) / 8;
	bps = salign * runtime->rate;
	if (bps <= 0 || salign <= 0)
		return -EINVAL;

	dpcm->buf_pos = 0;
	dpcm->pcm_buffer_size = frames_to_bytes(runtime, runtime->buffer_size);
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		/* clear capture buffer */
		dpcm->silent_size = dpcm->pcm_buffer_size;
		snd_pcm_format_set_silence(runtime->format, runtime->dma_area,
					   runtime->buffer_size * runtime->channels);
	}

	dpcm->irq_pos = 0;
	dpcm->period_update_pending = 0;
	dpcm->pcm_bps = bps;
	dpcm->pcm_salign = salign;
	dpcm->pcm_period_size = frames_to_bytes(runtime, runtime->period_size);
	dpcm->period_size_frac = frac_pos(dpcm->pcm_period_size);

	mutex_lock(&dpcm->loopback->cable_lock);
	if (!(cable->valid & ~(1 << substream->stream))) {
		cable->hw.formats = (1ULL << runtime->format);
		cable->hw.rate_min = runtime->rate;
		cable->hw.rate_max = runtime->rate;
		cable->hw.channels_min = runtime->channels;
		cable->hw.channels_max = runtime->channels;
	}
	cable->valid |= 1 << substream->stream;
	mutex_unlock(&dpcm->loopback->cable_lock);

	return 0;
}

static void clear_capture_buf(struct loopback_pcm *dpcm, unsigned int bytes)
{
	struct snd_pcm_runtime *runtime = dpcm->substream->runtime;
	char *dst = runtime->dma_area;
	unsigned int dst_off = dpcm->buf_pos;

	if (dpcm->silent_size >= dpcm->pcm_buffer_size)
		return;
	if (dpcm->silent_size + bytes > dpcm->pcm_buffer_size)
		bytes = dpcm->pcm_buffer_size - dpcm->silent_size;

	for (;;) {
		unsigned int size = bytes;
		if (dst_off + size > dpcm->pcm_buffer_size)
			size = dpcm->pcm_buffer_size - dst_off;
		snd_pcm_format_set_silence(runtime->format, dst + dst_off,
					   bytes_to_frames(runtime, size) *
					   	runtime->channels);
		dpcm->silent_size += size;
		bytes -= size;
		if (!bytes)
			break;
		dst_off = 0;
	}
}

static void copy_play_buf(struct loopback_pcm *play,
			  struct loopback_pcm *capt,
			  unsigned int bytes)
{
	struct snd_pcm_runtime *runtime = play->substream->runtime;
	char *src = play->substream->runtime->dma_area;
	char *dst = capt->substream->runtime->dma_area;
	unsigned int src_off = play->buf_pos;
	unsigned int dst_off = capt->buf_pos;
	unsigned int clear_bytes = 0;

	/* check if playback is draining, trim the capture copy size
	 * when our pointer is at the end of playback ring buffer */
	if (runtime->status->state == SNDRV_PCM_STATE_DRAINING &&
	    snd_pcm_playback_hw_avail(runtime) < runtime->buffer_size) { 
	    	snd_pcm_uframes_t appl_ptr, appl_ptr1, diff;
		appl_ptr = appl_ptr1 = runtime->control->appl_ptr;
		appl_ptr1 -= appl_ptr1 % runtime->buffer_size;
		appl_ptr1 += play->buf_pos / play->pcm_salign;
		if (appl_ptr < appl_ptr1)
			appl_ptr1 -= runtime->buffer_size;
		diff = (appl_ptr - appl_ptr1) * play->pcm_salign;
		if (diff < bytes) {
			clear_bytes = bytes - diff;
			bytes = diff;
		}
	}

	for (;;) {
		unsigned int size = bytes;
		if (src_off + size > play->pcm_buffer_size)
			size = play->pcm_buffer_size - src_off;
		if (dst_off + size > capt->pcm_buffer_size)
			size = capt->pcm_buffer_size - dst_off;
		memcpy(dst + dst_off, src + src_off, size);
		capt->silent_size = 0;
		bytes -= size;
		if (!bytes)
			break;
		src_off = (src_off + size) % play->pcm_buffer_size;
		dst_off = (dst_off + size) % capt->pcm_buffer_size;
	}

	if (clear_bytes > 0)
		clear_capture_buf(capt, clear_bytes);
}

#define BYTEPOS_UPDATE_POSONLY	0
#define BYTEPOS_UPDATE_CLEAR	1
#define BYTEPOS_UPDATE_COPY	2

static void loopback_bytepos_update(struct loopback_pcm *dpcm,
				    unsigned int delta,
				    unsigned int cmd)
{
	unsigned int count;
	unsigned long last_pos;

	last_pos = byte_pos(dpcm, dpcm->irq_pos);
	dpcm->irq_pos += delta * dpcm->pcm_bps;
	count = byte_pos(dpcm, dpcm->irq_pos) - last_pos;
	if (!count)
		return;
	if (cmd == BYTEPOS_UPDATE_CLEAR)
		clear_capture_buf(dpcm, count);
	else if (cmd == BYTEPOS_UPDATE_COPY)
		copy_play_buf(dpcm->cable->streams[SNDRV_PCM_STREAM_PLAYBACK],
			      dpcm->cable->streams[SNDRV_PCM_STREAM_CAPTURE],
			      count);
	dpcm->buf_pos += count;
	dpcm->buf_pos %= dpcm->pcm_buffer_size;
	if (dpcm->irq_pos >= dpcm->period_size_frac) {
		dpcm->irq_pos %= dpcm->period_size_frac;
		dpcm->period_update_pending = 1;
	}
}

static void loopback_pos_update(struct loopback_cable *cable)
{
	struct loopback_pcm *dpcm_play =
			cable->streams[SNDRV_PCM_STREAM_PLAYBACK];
	struct loopback_pcm *dpcm_capt =
			cable->streams[SNDRV_PCM_STREAM_CAPTURE];
	unsigned long delta_play = 0, delta_capt = 0;

	spin_lock(&cable->lock);	
	if (cable->running & (1 << SNDRV_PCM_STREAM_PLAYBACK)) {
		delta_play = jiffies - dpcm_play->last_jiffies;
		dpcm_play->last_jiffies += delta_play;
	}

	if (cable->running & (1 << SNDRV_PCM_STREAM_CAPTURE)) {
		delta_capt = jiffies - dpcm_capt->last_jiffies;
		dpcm_capt->last_jiffies += delta_capt;
	}

	if (delta_play == 0 && delta_capt == 0) {
		spin_unlock(&cable->lock);
		return;
	}
		
	if (delta_play > delta_capt) {
		loopback_bytepos_update(dpcm_play, delta_play - delta_capt,
					BYTEPOS_UPDATE_POSONLY);
		delta_play = delta_capt;
	} else if (delta_play < delta_capt) {
		loopback_bytepos_update(dpcm_capt, delta_capt - delta_play,
					BYTEPOS_UPDATE_CLEAR);
		delta_capt = delta_play;
	}

	if (delta_play == 0 && delta_capt == 0) {
		spin_unlock(&cable->lock);
		return;
	}
	/* note delta_capt == delta_play at this moment */
	loopback_bytepos_update(dpcm_capt, delta_capt, BYTEPOS_UPDATE_COPY);
	loopback_bytepos_update(dpcm_play, delta_play, BYTEPOS_UPDATE_POSONLY);
	spin_unlock(&cable->lock);
}

static void loopback_timer_function(unsigned long data)
{
	struct loopback_pcm *dpcm = (struct loopback_pcm *)data;
	int stream;

	loopback_pos_update(dpcm->cable);
	stream = dpcm->substream->stream;
	if (dpcm->cable->running & (1 << stream))
		loopback_timer_start(dpcm);
	if (dpcm->period_update_pending) {
		dpcm->period_update_pending = 0;
		if (dpcm->cable->running & (1 << stream))
			snd_pcm_period_elapsed(dpcm->substream);
	}
}

static snd_pcm_uframes_t loopback_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct loopback_pcm *dpcm = runtime->private_data;

	loopback_pos_update(dpcm->cable);
	return bytes_to_frames(runtime, dpcm->buf_pos);
}

static struct snd_pcm_hardware loopback_pcm_hardware =
{
	.info =		(SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_MMAP |
			 SNDRV_PCM_INFO_MMAP_VALID),
	.formats =	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S16_BE |
			 SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S32_BE |
			 SNDRV_PCM_FMTBIT_FLOAT_LE | SNDRV_PCM_FMTBIT_FLOAT_BE),
	.rates =	SNDRV_PCM_RATE_CONTINUOUS | SNDRV_PCM_RATE_8000_192000,
	.rate_min =		8000,
	.rate_max =		192000,
	.channels_min =		1,
	.channels_max =		32,
	.buffer_bytes_max =	2 * 1024 * 1024,
	.period_bytes_min =	64,
	.period_bytes_max =	2 * 1024 * 1024,
	.periods_min =		1,
	.periods_max =		1024,
	.fifo_size =		0,
};

static void loopback_runtime_free(struct snd_pcm_runtime *runtime)
{
	struct loopback_pcm *dpcm = runtime->private_data;
	kfree(dpcm);
}

static int loopback_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	return snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
}

static int loopback_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct loopback_pcm *dpcm = runtime->private_data;
	struct loopback_cable *cable = dpcm->cable;

	mutex_lock(&dpcm->loopback->cable_lock);
	cable->valid &= ~(1 << substream->stream);
	mutex_unlock(&dpcm->loopback->cable_lock);
	return snd_pcm_lib_free_pages(substream);
}

static unsigned int get_cable_index(struct snd_pcm_substream *substream)
{
	if (!substream->pcm->device)
		return substream->stream;
	else
		return !substream->stream;
}

static int loopback_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct loopback *loopback = substream->private_data;
	struct loopback_pcm *dpcm;
	struct loopback_cable *cable;
	int err = 0;
	int dev = get_cable_index(substream);

	mutex_lock(&loopback->cable_lock);
	dpcm = kzalloc(sizeof(*dpcm), GFP_KERNEL);
	if (!dpcm) {
		err = -ENOMEM;
		goto unlock;
	}
	dpcm->loopback = loopback;
	dpcm->substream = substream;
	setup_timer(&dpcm->timer, loopback_timer_function,
		    (unsigned long)dpcm);

	cable = loopback->cables[substream->number][dev];
	if (!cable) {
		cable = kzalloc(sizeof(*cable), GFP_KERNEL);
		if (!cable) {
			kfree(dpcm);
			err = -ENOMEM;
			goto unlock;
		}
		spin_lock_init(&cable->lock);
		cable->hw = loopback_pcm_hardware;
		loopback->cables[substream->number][dev] = cable;
	}
	dpcm->cable = cable;
	cable->streams[substream->stream] = dpcm;

	snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);

	runtime->private_data = dpcm;
	runtime->private_free = loopback_runtime_free;
	if (loopback->notify &&
	    substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		runtime->hw = loopback_pcm_hardware;
	} else {
		runtime->hw = cable->hw;
	}
 unlock:
	mutex_unlock(&loopback->cable_lock);
	return err;
}

static int loopback_close(struct snd_pcm_substream *substream)
{
	struct loopback *loopback = substream->private_data;
	struct loopback_pcm *dpcm = substream->runtime->private_data;
	struct loopback_cable *cable;
	int dev = get_cable_index(substream);

	loopback_timer_stop(dpcm);
	mutex_lock(&loopback->cable_lock);
	cable = loopback->cables[substream->number][dev];
	if (cable->streams[!substream->stream]) {
		/* other stream is still alive */
		cable->streams[substream->stream] = NULL;
	} else {
		/* free the cable */
		loopback->cables[substream->number][dev] = NULL;
		kfree(cable);
	}
	mutex_unlock(&loopback->cable_lock);
	return 0;
}

static struct snd_pcm_ops loopback_playback_ops = {
	.open =		loopback_open,
	.close =	loopback_close,
	.ioctl =	snd_pcm_lib_ioctl,
	.hw_params =	loopback_hw_params,
	.hw_free =	loopback_hw_free,
	.prepare =	loopback_prepare,
	.trigger =	loopback_trigger,
	.pointer =	loopback_pointer,
};

static struct snd_pcm_ops loopback_capture_ops = {
	.open =		loopback_open,
	.close =	loopback_close,
	.ioctl =	snd_pcm_lib_ioctl,
	.hw_params =	loopback_hw_params,
	.hw_free =	loopback_hw_free,
	.prepare =	loopback_prepare,
	.trigger =	loopback_trigger,
	.pointer =	loopback_pointer,
};

static int __devinit loopback_pcm_new(struct loopback *loopback,
				      int device, int substreams)
{
	struct snd_pcm *pcm;
	int err;

	err = snd_pcm_new(loopback->card, "Loopback PCM", device,
			  substreams, substreams, &pcm);
	if (err < 0)
		return err;
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &loopback_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &loopback_capture_ops);

	pcm->private_data = loopback;
	pcm->info_flags = 0;
	strcpy(pcm->name, "Loopback PCM");

	loopback->pcm[device] = pcm;

	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_CONTINUOUS,
			snd_dma_continuous_data(GFP_KERNEL),
			0, 2 * 1024 * 1024);
	return 0;
}

static int __devinit loopback_mixer_new(struct loopback *loopback)
{
	struct snd_card *card = loopback->card;

	strcpy(card->mixername, "Loopback Mixer");
	return 0;
}

static int __devinit loopback_probe(struct platform_device *devptr)
{
	struct snd_card *card;
	struct loopback *loopback;
	int dev = devptr->id;
	int err;

	err = snd_card_create(index[dev], id[dev], THIS_MODULE,
			      sizeof(struct loopback), &card);
	if (err < 0)
		return err;
	loopback = card->private_data;

	if (pcm_substreams[dev] < 1)
		pcm_substreams[dev] = 1;
	if (pcm_substreams[dev] > MAX_PCM_SUBSTREAMS)
		pcm_substreams[dev] = MAX_PCM_SUBSTREAMS;
	
	loopback->card = card;
	loopback->notify = pcm_notify[dev] ? 1 : 0;
	mutex_init(&loopback->cable_lock);

	err = loopback_pcm_new(loopback, 0, pcm_substreams[dev]);
	if (err < 0)
		goto __nodev;
	err = loopback_pcm_new(loopback, 1, pcm_substreams[dev]);
	if (err < 0)
		goto __nodev;
	err = loopback_mixer_new(loopback);
	if (err < 0)
		goto __nodev;
	strcpy(card->driver, "Loopback");
	strcpy(card->shortname, "Loopback");
	sprintf(card->longname, "Loopback %i", dev + 1);
	err = snd_card_register(card);
	if (!err) {
		platform_set_drvdata(devptr, card);
		return 0;
	}
      __nodev:
	snd_card_free(card);
	return err;
}

static int __devexit loopback_remove(struct platform_device *devptr)
{
	snd_card_free(platform_get_drvdata(devptr));
	platform_set_drvdata(devptr, NULL);
	return 0;
}

#ifdef CONFIG_PM
static int loopback_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	struct snd_card *card = platform_get_drvdata(pdev);
	struct loopback *loopback = card->private_data;

	snd_power_change_state(card, SNDRV_CTL_POWER_D3hot);

	snd_pcm_suspend_all(loopback->pcm[0]);
	snd_pcm_suspend_all(loopback->pcm[1]);
	return 0;
}
	
static int loopback_resume(struct platform_device *pdev)
{
	struct snd_card *card = platform_get_drvdata(pdev);

	snd_power_change_state(card, SNDRV_CTL_POWER_D0);
	return 0;
}
#endif

#define SND_LOOPBACK_DRIVER	"snd_aloop"

static struct platform_driver loopback_driver = {
	.probe		= loopback_probe,
	.remove		= __devexit_p(loopback_remove),
#ifdef CONFIG_PM
	.suspend	= loopback_suspend,
	.resume		= loopback_resume,
#endif
	.driver		= {
		.name	= SND_LOOPBACK_DRIVER
	},
};

static void loopback_unregister_all(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(devices); ++i)
		platform_device_unregister(devices[i]);
	platform_driver_unregister(&loopback_driver);
}

static int __init alsa_card_loopback_init(void)
{
	int i, err, cards;

	err = platform_driver_register(&loopback_driver);
	if (err < 0)
		return err;


	cards = 0;
	for (i = 0; i < SNDRV_CARDS; i++) {
		struct platform_device *device;
		if (!enable[i])
			continue;
		device = platform_device_register_simple(SND_LOOPBACK_DRIVER,
							 i, NULL, 0);
		if (IS_ERR(device))
			continue;
		if (!platform_get_drvdata(device)) {
			platform_device_unregister(device);
			continue;
		}
		devices[i] = device;
		cards++;
	}
	if (!cards) {
#ifdef MODULE
		printk(KERN_ERR "aloop: No loopback enabled\n");
#endif
		loopback_unregister_all();
		return -ENODEV;
	}
	return 0;
}

static void __exit alsa_card_loopback_exit(void)
{
	loopback_unregister_all();
}

module_init(alsa_card_loopback_init)
module_exit(alsa_card_loopback_exit)
