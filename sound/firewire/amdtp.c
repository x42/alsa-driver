/*
 * Audio and Music Data Transmission Protocol (IEC 61883-6) streams
 * with Common Isochronous Packet (IEC 61883-1) headers
 *
 * Copyright (c) Clemens Ladisch <clemens@ladisch.de>
 * Licensed under the terms of the GNU General Public License, version 2.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/firewire.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <sound/pcm.h>
#include "amdtp.h"
#include "digimagic.h"

#define TICKS_PER_CYCLE		3072
#define CYCLES_PER_SECOND	8000
#define TICKS_PER_SECOND	(TICKS_PER_CYCLE * CYCLES_PER_SECOND)

#define TRANSFER_DELAY_TICKS	0x2e00 /* 479.17 µs */

#define TAG_CIP			1

#define CIP_EOH			(1u << 31)
#define CIP_FMT_AM		(0x10 << 24)
#define AMDTP_FDF_AM824		(0 << 19)
#define AMDTP_FDF_SFC_SHIFT	16

/* TODO: make these configurable */
#if 0 // dz
#define INTERRUPT_INTERVAL	24
#define QUEUE_LENGTH		96
#else
#define INTERRUPT_INTERVAL	16
#define QUEUE_LENGTH		48
#endif

/**
 * amdtp_stream_init - initialize an AMDTP stream structure
 * @s: the AMDTP stream to initialize
 * @unit: the target of the stream
 * @flags: the packet transfer method to use
 */
int amdtp_stream_init(struct amdtp_stream *s, struct fw_unit *unit,
			  enum cip_flags flags)
{
	s->unit = fw_unit_get(unit);
	s->flags = flags;
	s->context = ERR_PTR(-1);
	mutex_init(&s->mutex);
	s->packet_index = 0;

	s->data_block_state = 0;
	s->use_digimagic = false;

	return 0;
}
EXPORT_SYMBOL(amdtp_stream_init);

/**
 * amdtp_stream_destroy - free stream resources
 * @s: the AMDTP stream to destroy
 */
void amdtp_stream_destroy(struct amdtp_stream *s)
{
	WARN_ON(amdtp_stream_running(s));
	mutex_destroy(&s->mutex);
	fw_unit_put(s->unit);
}
EXPORT_SYMBOL(amdtp_stream_destroy);

const unsigned int amdtp_syt_intervals[CIP_SFC_COUNT] = {
	[CIP_SFC_32000]  =  8,
	[CIP_SFC_44100]  =  8,
	[CIP_SFC_48000]  =  8,
	[CIP_SFC_88200]  = 16,
	[CIP_SFC_96000]  = 16,
	[CIP_SFC_176400] = 32,
	[CIP_SFC_192000] = 32,
};
EXPORT_SYMBOL(amdtp_syt_intervals);

/**
 * amdtp_stream_set_parameters - set stream parameters
 * @s: the AMDTP stream to configure
 * @rate: the sample rate
 * @pcm_channels: the number of PCM samples in each data block, to be encoded
 *                as AM824 multi-bit linear audio
 * @midi_data_channels: the number of MIDI Conformant Data Channels, i.e.,
 *                      quadlets (_not_ the number of MPX-MIDI Data Channels)
 *
 * The parameters must be set before the stream is started, and must not be
 * changed while the stream is running.
 */
void amdtp_stream_set_parameters(struct amdtp_stream *s,
				     unsigned int rate,
				     unsigned int pcm_channels,
				     unsigned int midi_data_channels)
{
	static const unsigned int rates[] = {
		[CIP_SFC_32000]  =  32000,
		[CIP_SFC_44100]  =  44100,
		[CIP_SFC_48000]  =  48000,
		[CIP_SFC_88200]  =  88200,
		[CIP_SFC_96000]  =  96000,
		[CIP_SFC_176400] = 176400,
		[CIP_SFC_192000] = 192000,
	};
	unsigned int sfc, i;

	if (WARN_ON(amdtp_stream_running(s)) ||
	    WARN_ON(pcm_channels > AMDTP_MAX_CHANNELS_PCM) ||
	    WARN_ON(midi_data_channels > AMDTP_MAX_CHANNELS_MIDI))
		return;

	for (sfc = 0; sfc < CIP_SFC_COUNT; ++sfc)
		if (rates[sfc] == rate)
			goto sfc_found;
	WARN_ON(1);
	return;

sfc_found:
	s->dual_wire = (s->flags & CIP_HI_DUALWIRE) && sfc > CIP_SFC_96000;
	if (s->dual_wire) {
		sfc -= 2;
		rate /= 2;
		pcm_channels *= 2;
	}
	s->sfc = sfc;
	s->data_block_quadlets = pcm_channels + midi_data_channels;
	s->pcm_channels = pcm_channels;
	s->midi_data_channels = midi_data_channels;

	if (!s->dual_wire)
		for (i = 0; i < pcm_channels; ++i)
			s->pcm_quadlets[i] = i;
	else
		for (i = 0; i < pcm_channels / 2; ++i) {
			s->pcm_quadlets[i                   ] = 2 * i;
			s->pcm_quadlets[i + pcm_channels / 2] = 2 * i + 1;
		}
	for (i = 0; i < midi_data_channels; ++i)
		s->midi_quadlets[i] = i;

	s->syt_interval = amdtp_syt_intervals[sfc];

	/* default buffering in the device */
	s->transfer_delay = TRANSFER_DELAY_TICKS - TICKS_PER_CYCLE;
	if (s->flags & CIP_BLOCKING)
		/* additional buffering needed to adjust for no-data packets */
		s->transfer_delay += TICKS_PER_SECOND * s->syt_interval / rate;
}
EXPORT_SYMBOL(amdtp_stream_set_parameters);

/**
 * amdtp_stream_get_max_payload - get the stream's packet size
 * @s: the AMDTP stream
 *
 * This function must not be called before the stream has been configured
 * with amdtp_stream_set_parameters().
 */
unsigned int amdtp_stream_get_max_payload(struct amdtp_stream *s)
{
	return 8 + s->syt_interval * s->data_block_quadlets * 4;
}
EXPORT_SYMBOL(amdtp_stream_get_max_payload);

static unsigned int calculate_data_blocks(struct amdtp_stream *s)
{
	unsigned int phase, data_blocks;

	if (!cip_sfc_is_base_44100(s->sfc)) {
		/* Sample_rate / 8000 is an integer, and precomputed. */
		if (!s->use_digimagic) {
			data_blocks = s->data_block_state;
		} else {
			if (s->sfc == CIP_SFC_96000) {
#if 0 // TODO
				phase = s->data_block_state;
				data_blocks = (11 + (phase % 2) * 3) * pattern003_96[phase];
				if (++phase >= 6)
					phase = 0;
				s->data_block_state = phase;
#else
			data_blocks = s->data_block_state;
#endif
			} else {  //48000Hz 003
				phase = s->data_block_state;
				if (phase >= 16)
					phase = 0;

				data_blocks = ((phase % 16) > 7) ? 5 : 7;
				if (++phase >= 16)
					phase = 0;
				s->data_block_state = phase;
			}
		}

	} else {
		phase = s->data_block_state;

		/*
		 * This calculates the number of data blocks per packet so that
		 * 1) the overall rate is correct and exactly synchronized to
		 *    the bus clock, and
		 * 2) packets with a rounded-up number of blocks occur as early
		 *    as possible in the sequence (to prevent underruns of the
		 *    device's buffer).
		 */
		if (s->sfc == CIP_SFC_44100)
			/* 6 6 5 6 5 6 5 ... */
			data_blocks = 5 + ((phase & 1) ^
					   (phase == 0 || phase >= 40));
		else
			/* 12 11 11 11 11 ... or 23 22 22 22 22 ... */
			data_blocks = 11 * (s->sfc >> 1) + (phase == 0);
		if (++phase >= (80 >> (s->sfc >> 1)))
			phase = 0;
		s->data_block_state = phase;
	}

	return data_blocks;
}

static unsigned int calculate_syt(struct amdtp_stream *s,
				  unsigned int cycle)
{
	unsigned int syt_offset, phase, index, syt;

	if (s->last_syt_offset < TICKS_PER_CYCLE) {
		if (!cip_sfc_is_base_44100(s->sfc))
			syt_offset = s->last_syt_offset + s->syt_offset_state;
		else {
		/*
		 * The time, in ticks, of the n'th SYT_INTERVAL sample is:
		 *   n * SYT_INTERVAL * 24576000 / sample_rate
		 * Modulo TICKS_PER_CYCLE, the difference between successive
		 * elements is about 1386.23.  Rounding the results of this
		 * formula to the SYT precision results in a sequence of
		 * differences that begins with:
		 *   1386 1386 1387 1386 1386 1386 1387 1386 1386 1386 1387 ...
		 * This code generates _exactly_ the same sequence.
		 */
			phase = s->syt_offset_state;
			index = phase % 13;
			syt_offset = s->last_syt_offset;
			syt_offset += 1386 + ((index && !(index & 3)) ||
					      phase == 146);
			if (++phase >= 147)
				phase = 0;
			s->syt_offset_state = phase;
		}
	} else
		syt_offset = s->last_syt_offset - TICKS_PER_CYCLE;
	s->last_syt_offset = syt_offset;

	if (syt_offset < TICKS_PER_CYCLE) {
		syt_offset += s->transfer_delay;
		syt = (cycle + syt_offset / TICKS_PER_CYCLE) << 12;
		syt += syt_offset % TICKS_PER_CYCLE;

		return syt & 0xffff;
	} else {
		return 0xffff; /* no info */
	}
}

static void amdtp_write_samples(struct amdtp_stream *s,
				struct snd_pcm_substream *pcm,
				__be32 *buffer, unsigned int frames)
{
	struct snd_pcm_runtime *runtime = pcm->runtime;
	unsigned int channels, remaining_frames, frame_step, i, c;
	const u32 *src;
	DigiMagic digistate;

	channels = s->pcm_channels;
	src = (void *)runtime->dma_area +
			s->pcm_buffer_pointer * (runtime->frame_bits / 8);

	remaining_frames = runtime->buffer_size - s->pcm_buffer_pointer;
	frame_step = s->data_block_quadlets;

	for (i = 0; i < frames; ++i) {
		digi_state_reset(&digistate);
		for (c = 0; c < channels; ++c) {
			buffer[s->pcm_quadlets[c]] =
					cpu_to_be32((*src >> 8) | 0x40000000);
			if (s->use_digimagic) {
				digi_encode_step(&digistate, &buffer[s->pcm_quadlets[c]]);
			}
			src++;
		}
		buffer += frame_step;
		if (--remaining_frames == 0)
			src = (void *)runtime->dma_area;
	}
}

static void amdtp_fill_pcm_silence(struct amdtp_stream *s,
				   __be32 *buffer, unsigned int frames)
{
	unsigned int i, c;

	for (i = 0; i < frames; ++i) {
		for (c = 0; c < s->pcm_channels; ++c)
			buffer[s->pcm_quadlets[c]] = cpu_to_be32(0x40000000);
		buffer += s->data_block_quadlets;
	}
}

static void amdtp_fill_midi(struct amdtp_stream *s,
			    __be32 *buffer, unsigned int frames)
{
	unsigned int i, c;

	for (i = 0; i < frames; ++i) {
		for (c = 0; c < s->midi_data_channels; ++c)
			buffer[s->midi_quadlets[c]] = cpu_to_be32(0x80000000);
		buffer += s->data_block_quadlets;
	}
}

static void amdtp_fill_midi_as_pcm(struct amdtp_stream *s,
				   __be32 *buffer, unsigned int frames)
{
	unsigned int i, c;

	for (i = 0; i < frames; ++i) {
		for (c = 0; c < s->midi_data_channels; ++c)
			buffer[s->midi_quadlets[c]] = cpu_to_be32(0x40000000);
		buffer += s->data_block_quadlets;
	}
}

static void queue_out_dummy_packet(struct amdtp_stream *s, unsigned int cycle)
{
	__be32 *buffer;
	unsigned int index, data_blocks, syt, ptr;
	struct snd_pcm_substream *pcm;
	struct fw_iso_packet packet;
	int err;

	if (s->packet_index < 0)
		return;
	index = s->packet_index;

	syt = calculate_syt(s, cycle);
	if (!(s->flags & CIP_BLOCKING)) {
		data_blocks = calculate_data_blocks(s);
	} else {
		if (syt != 0xffff) {
			data_blocks = s->syt_interval;
		} else {
			data_blocks = 0;
			syt = 0xffffff;
		}
	}

	buffer = s->buffer.packets[index].buffer;
	buffer[0] = cpu_to_be32(0x00 << 24 /*ACCESS_ONCE(s->source_node_id_field)*/|
				(s->data_block_quadlets << 16) |
				s->data_block_counter); //dzhack
	buffer[1] = cpu_to_be32(CIP_EOH | CIP_FMT_AM | AMDTP_FDF_AM824 |
				(s->sfc << AMDTP_FDF_SFC_SHIFT) ); //| syt);
	buffer += 2;

	amdtp_fill_pcm_silence(s, buffer, data_blocks);
	if (s->midi_data_channels > 0)
		amdtp_fill_midi_as_pcm(s, buffer, data_blocks);

	s->data_block_counter = (s->data_block_counter + data_blocks) & 0xff;

	packet.payload_length = 8 + data_blocks * 4 * s->data_block_quadlets;
	packet.interrupt = IS_ALIGNED(index + 1, INTERRUPT_INTERVAL);
	packet.skip = 0;
	packet.tag = TAG_CIP;
	packet.sy = 0;
	packet.header_length = 0;

	err = fw_iso_context_queue(s->context, &packet, &s->buffer.iso_buffer,
				   s->buffer.packets[index].offset);
	if (err < 0) {
		dev_err(&s->unit->device, "queueing error: %d\n", err);
		s->packet_index = -1;
		amdtp_stream_pcm_abort(s);
		return;
	}

	if (++index >= QUEUE_LENGTH)
		index = 0;
	s->packet_index = index;

	pcm = ACCESS_ONCE(s->pcm);
	if (pcm) {
		if (s->dual_wire)
			data_blocks *= 2;

		ptr = s->pcm_buffer_pointer + data_blocks;
		if (ptr >= pcm->runtime->buffer_size)
			ptr -= pcm->runtime->buffer_size;
		ACCESS_ONCE(s->pcm_buffer_pointer) = ptr;

		s->pcm_period_pointer += data_blocks;
		if (s->pcm_period_pointer >= pcm->runtime->period_size) {
			s->pcm_period_pointer -= pcm->runtime->period_size;
			snd_pcm_period_elapsed(pcm);
		}
	}
}

static void queue_out_packet(struct amdtp_stream *s, unsigned int cycle)
{
	__be32 *buffer;
	unsigned int index, data_blocks, syt, ptr;
	struct snd_pcm_substream *pcm;
	struct fw_iso_packet packet;
	int err;

	if (s->packet_index < 0)
		return;
	index = s->packet_index;

	syt = calculate_syt(s, cycle);
	if (!(s->flags & CIP_BLOCKING)) {
		data_blocks = calculate_data_blocks(s);
	} else {
		if (syt != 0xffff) {
			data_blocks = s->syt_interval;
		} else {
			data_blocks = 0;
			syt = 0xffffff;
		}
	}

	buffer = s->buffer.packets[index].buffer;
	buffer[0] = cpu_to_be32( ((s->use_digimagic)? 0x00 : ACCESS_ONCE(s->source_node_id_field) ) |
				(s->data_block_quadlets << 16) |
				s->data_block_counter);
	buffer[1] = cpu_to_be32(CIP_EOH | CIP_FMT_AM | AMDTP_FDF_AM824 |
				(s->sfc << AMDTP_FDF_SFC_SHIFT) | ((s->use_digimagic)? 0x00 : syt) );
	buffer += 2;

	pcm = ACCESS_ONCE(s->pcm);
	if (pcm)
		amdtp_write_samples(s, pcm, buffer, data_blocks);
	else
		amdtp_fill_pcm_silence(s, buffer, data_blocks);
	if (s->midi_data_channels > 0)
		amdtp_fill_midi(s, buffer, data_blocks);

	s->data_block_counter = (s->data_block_counter + data_blocks) & 0xff;

	packet.payload_length = 8 + data_blocks * 4 * s->data_block_quadlets;
	packet.interrupt = IS_ALIGNED(index + 1, INTERRUPT_INTERVAL);
	packet.skip = 0;
	packet.tag = TAG_CIP;
	packet.sy = 0;
	packet.header_length = 0;

	err = fw_iso_context_queue(s->context, &packet, &s->buffer.iso_buffer,
				   s->buffer.packets[index].offset);
	if (err < 0) {
		dev_err(&s->unit->device, "queueing error: %d\n", err);
		s->packet_index = -1;
		amdtp_stream_pcm_abort(s);
		return;
	}

	if (++index >= QUEUE_LENGTH)
		index = 0;
	s->packet_index = index;

	if (pcm) {
		if (s->dual_wire)
			data_blocks *= 2;

		ptr = s->pcm_buffer_pointer + data_blocks;
		if (ptr >= pcm->runtime->buffer_size)
			ptr -= pcm->runtime->buffer_size;
		ACCESS_ONCE(s->pcm_buffer_pointer) = ptr;

		s->pcm_period_pointer += data_blocks;
		if (s->pcm_period_pointer >= pcm->runtime->period_size) {
			s->pcm_period_pointer -= pcm->runtime->period_size;
			snd_pcm_period_elapsed(pcm);
		}
	}
}

static void out_packet_callback(struct fw_iso_context *context, u32 cycle,
				size_t header_length, void *header, void *data)
{
	struct amdtp_stream *s = data;
	unsigned int i, packets = header_length / 4;

	/*
	 * Compute the cycle of the last queued packet.
	 * (We need only the four lowest bits for the SYT, so we can ignore
	 * that bits 0-11 must wrap around at 3072.)
	 */
	cycle += QUEUE_LENGTH - packets;

	for (i = 0; i < packets; ++i)
		queue_out_packet(s, ++cycle);
	fw_iso_context_queue_flush(s->context);
	if (s->use_digimagic) {
		s->cycle = cycle;
	}
}

static int queue_initial_skip_packets(struct amdtp_stream *s)
{
	struct fw_iso_packet skip_packet = {
		.skip = 1,
	};
	unsigned int i;
	int err;

	for (i = 0; i < QUEUE_LENGTH; ++i) {
		skip_packet.interrupt = IS_ALIGNED(s->packet_index + 1,
						   INTERRUPT_INTERVAL);
		err = fw_iso_context_queue(s->context, &skip_packet, NULL, 0);
		if (err < 0)
			return err;
		if (++s->packet_index >= QUEUE_LENGTH)
			s->packet_index = 0;
	}

	return 0;
}

static int queue_initial_dummy_packets(struct amdtp_stream *s)
{
	unsigned int i;

	unsigned int packets = 96;

	for (i = 0; i < packets; ++i)
	{
		queue_out_dummy_packet(s, s->cycle);
		s->cycle += 1;
	}
	fw_iso_context_queue_flush(s->context);
	return 0;
}

/**
 * amdtp_stream_start - start sending packets
 * @s: the AMDTP stream to start
 * @channel: the isochronous channel on the bus
 * @speed: firewire speed code
 *
 * The stream cannot be started until it has been configured with
 * amdtp_stream_set_parameters(),
 * and it must be started before any PCM or MIDI device can be started.
 */
int amdtp_stream_start(struct amdtp_stream *s, int channel, int speed)
{
	static const struct {
		unsigned int data_block;
		unsigned int syt_offset;
	} initial_state[] = {
		[CIP_SFC_32000]  = {  4, 3072 },
		[CIP_SFC_48000]  = {  6, 1024 },
		[CIP_SFC_96000]  = { 12, 1024 },
		[CIP_SFC_192000] = { 24, 1024 },
		[CIP_SFC_44100]  = {  0,   67 },
		[CIP_SFC_88200]  = {  0,   67 },
		[CIP_SFC_176400] = {  0,   67 },
	};
	int err;

	mutex_lock(&s->mutex);

	if (WARN_ON(amdtp_stream_running(s) ||
		    (!s->pcm_channels && !s->midi_data_channels))) {
		err = -EBADFD;
		goto err_unlock;
	}

	if (s->use_digimagic) {
		s->data_block_state = 0;
	} else {
		s->data_block_state = initial_state[s->sfc].data_block;
	}
	s->syt_offset_state = initial_state[s->sfc].syt_offset;
	s->last_syt_offset = TICKS_PER_CYCLE;

	err = iso_packets_buffer_init(&s->buffer, s->unit, QUEUE_LENGTH,
				      amdtp_stream_get_max_payload(s),
				      DMA_TO_DEVICE);
	if (err < 0)
		goto err_unlock;

	s->context = fw_iso_context_create(fw_parent_device(s->unit)->card,
					   FW_ISO_CONTEXT_TRANSMIT,
					   channel, speed, 0,
					   out_packet_callback, s);
	if (IS_ERR(s->context)) {
		err = PTR_ERR(s->context);
		if (err == -EBUSY)
			dev_err(&s->unit->device,
				"no free stream on this controller\n");
		goto err_buffer;
	}

	amdtp_stream_update(s);

	s->packet_index = 0;
	s->data_block_counter = 0;
	if (!s->use_digimagic) {
		err = queue_initial_skip_packets(s);
	} else {
		s->cycle = 0;
		err = queue_initial_dummy_packets(s);
	}
	if (err < 0)
		goto err_context;

	err = fw_iso_context_start(s->context, -1, 0, 0);
	if (err < 0)
		goto err_context;

	mutex_unlock(&s->mutex);

	return 0;

err_context:
	fw_iso_context_destroy(s->context);
	s->context = ERR_PTR(-1);
err_buffer:
	iso_packets_buffer_destroy(&s->buffer, s->unit);
err_unlock:
	mutex_unlock(&s->mutex);

	return err;
}
EXPORT_SYMBOL(amdtp_stream_start);

/**
 * amdtp_stream_update - update the stream after a bus reset
 * @s: the AMDTP stream
 */
void amdtp_stream_update(struct amdtp_stream *s)
{
	ACCESS_ONCE(s->source_node_id_field) =
		(fw_parent_device(s->unit)->card->node_id & 0x3f) << 24;
}
EXPORT_SYMBOL(amdtp_stream_update);

/**
 * amdtp_stream_stop - stop sending packets
 * @s: the AMDTP stream to stop
 *
 * All PCM and MIDI devices of the stream must be stopped before the stream
 * itself can be stopped.
 */
void amdtp_stream_stop(struct amdtp_stream *s)
{
	mutex_lock(&s->mutex);

	if (!amdtp_stream_running(s)) {
		mutex_unlock(&s->mutex);
		return;
	}

	fw_iso_context_stop(s->context);
	fw_iso_context_destroy(s->context);
	s->context = ERR_PTR(-1);
	iso_packets_buffer_destroy(&s->buffer, s->unit);

	mutex_unlock(&s->mutex);
}
EXPORT_SYMBOL(amdtp_stream_stop);

/**
 * amdtp_stream_pcm_abort - abort the running PCM device
 * @s: the AMDTP stream about to be stopped
 *
 * If the isochronous stream needs to be stopped asynchronously, call this
 * function first to stop the PCM device.
 */
void amdtp_stream_pcm_abort(struct amdtp_stream *s)
{
	struct snd_pcm_substream *pcm;

	pcm = ACCESS_ONCE(s->pcm);
	if (pcm) {
		snd_pcm_stream_lock_irq(pcm);
		if (snd_pcm_running(pcm))
			snd_pcm_stop(pcm, SNDRV_PCM_STATE_XRUN);
		snd_pcm_stream_unlock_irq(pcm);
	}
}
EXPORT_SYMBOL(amdtp_stream_pcm_abort);
