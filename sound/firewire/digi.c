/*
 * TC Applied Technologies Digital Interface Communications Engine driver
 *
 * Copyright (c) Clemens Ladisch <clemens@ladisch.de>
 * Licensed under the terms of the GNU General Public License, version 2.
 */

#include <linux/compat.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firewire.h>
#include <linux/firewire-constants.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <sound/control.h>
#include <sound/core.h>
#include <sound/firewire.h>
#include <sound/hwdep.h>
#include <sound/info.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include "amdtp.h"
#include "iso-resources.h"
#include "lib.h"
#include "dice-interface.h"


#define MODES	3
//3

#define MAX_RX	4
//4

/*
 * Digidesign 003Rack driver
 *
 * Copyright (c) Damien Zammit <damien.zammit@gmail.com>
 * Licensed under the terms of the GNU General Public License, version 2.
 */
#define R003_HARDWARE_ADDR      0xffff00000000ULL

#define VENDOR_DIGIDESIGN       0x00a07e
#define VENDOR_DIGIDESIGN_NAME  " "
#define R003_MODEL_ID           0x00ab0000
//#define R003_MODEL_ID           0x00000002
#define R003_MODEL_NAME         " 003Rack "

#define R003_STREAMS_W_REG      0xe0000004
#define R003_STREAMS_R_REG      0xe0000000
#define R003_STREAMS_OFF        0x00000000
#define R003_STREAMS_ON         0x00000001
#define R003_STREAMS_INIT       0x00000002
#define R003_STREAMS_SHUTDOWN   0x00000003

#define R003_SAMPLERATE_REG     0xe0000110
#define R003_SAMPLERATE_44100   0x00000000
#define R003_SAMPLERATE_48000   0x00000001
#define R003_SAMPLERATE_88200   0x00000002
#define R003_SAMPLERATE_96000   0x00000003

#define R003_CLOCKSOURCE_REG    0xe0000118
#define R003_CLOCK_INTERNAL     0x00000000
#define R003_CLOCK_SPDIF        0x00000001
#define R003_CLOCK_ADAT         0x00000002
#define R003_CLOCK_WORDCLOCK    0x00000003

#define R003_MIX                (0xe0000300 | R003_HARDWARE_ADDR)
#define R003_MIX_ANALOG_1L      (0x00 | R003_MIX)
#define R003_MIX_ANALOG_1R      (0x04 | R003_MIX)
#define R003_MIX_ANALOG_2L      (0x08 | R003_MIX)
#define R003_MIX_ANALOG_2R      (0x0c | R003_MIX)
#define R003_MIX_ANALOG_3L      (0x10 | R003_MIX)
#define R003_MIX_ANALOG_3R      (0x14 | R003_MIX)
#define R003_MIX_ANALOG_4L      (0x18 | R003_MIX)
#define R003_MIX_ANALOG_4R      (0x1c | R003_MIX)
#define R003_MIX_ANALOG_5L      (0x20 | R003_MIX)
#define R003_MIX_ANALOG_5R      (0x24 | R003_MIX)
#define R003_MIX_ANALOG_6L      (0x28 | R003_MIX)
#define R003_MIX_ANALOG_6R      (0x2c | R003_MIX)
#define R003_MIX_ANALOG_7L      (0x30 | R003_MIX)
#define R003_MIX_ANALOG_7R      (0x34 | R003_MIX)
#define R003_MIX_ANALOG_8L      (0x38 | R003_MIX)
#define R003_MIX_ANALOG_8R      (0x3c | R003_MIX)
#define R003_MIX_SPDIF_1L       (0x40 | R003_MIX)
#define R003_MIX_SPDIF_1R       (0x44 | R003_MIX)
#define R003_MIX_SPDIF_2L       (0x48 | R003_MIX)
#define R003_MIX_SPDIF_2R       (0x4c | R003_MIX)
#define R003_MIX_ADAT_1L        (0x50 | R003_MIX)
#define R003_MIX_ADAT_1R        (0x54 | R003_MIX)
#define R003_MIX_ADAT_2L        (0x58 | R003_MIX)
#define R003_MIX_ADAT_2R        (0x5c | R003_MIX)
#define R003_MIX_ADAT_3L        (0x60 | R003_MIX)
#define R003_MIX_ADAT_3R        (0x64 | R003_MIX)
#define R003_MIX_ADAT_4L        (0x68 | R003_MIX)
#define R003_MIX_ADAT_4R        (0x6c | R003_MIX)
#define R003_MIX_ADAT_5L        (0x70 | R003_MIX)
#define R003_MIX_ADAT_5R        (0x74 | R003_MIX)
#define R003_MIX_ADAT_6L        (0x78 | R003_MIX)
#define R003_MIX_ADAT_6R        (0x7c | R003_MIX)
#define R003_MIX_ADAT_7L        (0x80 | R003_MIX)
#define R003_MIX_ADAT_7R        (0x84 | R003_MIX)
#define R003_MIX_ADAT_8L        (0x88 | R003_MIX)
#define R003_MIX_ADAT_8R        (0x8c | R003_MIX)

#define R003_MIX_NONE           0x00000000
#define R003_MIX_1_TO_STEREO    0x18000000
#define R003_MIX_1_TO_1         0x20000000

#define BYTESWAP32_CONST(x) ((((x) & 0x000000FF) << 24) |   \
                             (((x) & 0x0000FF00) << 8) |    \
                             (((x) & 0x00FF0000) >> 8) |    \
                             (((x) & 0xFF000000) >> 24))

struct digi {
	struct snd_card *card;
	struct fw_unit *unit;
	spinlock_t lock;
	struct mutex mutex;
	unsigned int vendor;
	unsigned int global_offset;
	unsigned int rx_offset;
	unsigned int rx_size;
	unsigned int clock_caps;
	unsigned int rx_count[MODES];
	unsigned int rx_channels[MODES];
	struct {
		u8 pcm_channels[MODES];
		u8 midi_ports[MODES];
	} rx[MAX_RX];
	struct fw_address_handler notification_handler;
	int owner_generation;
	int dev_lock_count; /* > 0 driver, < 0 userspace */
	bool dev_lock_changed;
	bool global_enabled;
	unsigned int current_mode;
	struct completion clock_accepted;
	wait_queue_head_t hwdep_wait;
	u32 notification_bits;
	struct fw_iso_resources rx_resources;
	struct amdtp_stream rx_stream;
};

MODULE_DESCRIPTION("DIGIDESIGN 003RACK driver");
MODULE_AUTHOR("Damien Zammit <damien.zammit@gmail.com>");
MODULE_LICENSE("GPL v2");

static inline u64 global_address(struct digi *digi, unsigned int offset)
{
	return R003_HARDWARE_ADDR; /* + digi->global_offset */ //+ offset;
}

static inline u64 rx_address(struct digi *digi, unsigned int index, unsigned int offset)
{
	return R003_HARDWARE_ADDR; //+ offset;//+ digi->rx_offset +
				   //index * digi->rx_size + offset;
}

#if 0
static inline u64 global_address(struct digi *digi, unsigned int offset)
{
	return R003_HARDWARE_ADDR + offset;
}

// TODO: rx index
static inline u64 rx_address(struct digi *digi, unsigned int offset)
{
	return R003_HARDWARE_ADDR + offset;
}
#endif

void write_quadlet(struct digi *digi, unsigned long long int reg, unsigned int data)
{
	data = BYTESWAP32_CONST(data);
	snd_fw_transaction(digi->unit, TCODE_WRITE_QUADLET_REQUEST, reg, &data, 4, 0);
}

unsigned int read_quadlet(struct digi *digi, unsigned long long int reg)
{
	unsigned int data = 0;
	snd_fw_transaction(digi->unit, TCODE_READ_QUADLET_REQUEST, reg, &data, 4, 0);
	return BYTESWAP32_CONST(data);
}

static const unsigned int digi_rates[] = {
	/* mode 0 */
	[0] =  48000,
	[1] =  48000,
	[2] =  48000,
	/* mode 1 */
	[3] =  96000,
	[4] =  96000,
	/* mode 2 */
//	[5] = 176400,
//	[6] = 192000,
};

static inline int poll_until(struct digi *digi, unsigned long long int reg, unsigned int expect)
{
	int timeout = 1024;
	while (read_quadlet(digi, reg) != expect && --timeout);
	return ( timeout == 0 );
}

static void rack_init_write_814_block(struct digi *digi)
{
/*
 * write_block_request, offs=0xffffe0000008, data_length=0x0008, extended_tcode=0x0000, data=[ffc2ffff 00000000]
 * write_block_request, offs=0xffffe0000014, data_length=0x0008, extended_tcode=0x0000, data=[ffc2ffff 00000040]
 */
#if 1   /* use transaction */
	__be32 data[2];
	data[0] = BYTESWAP32_CONST(0xffc2ffff);
	data[1] = BYTESWAP32_CONST(0x00000000);
	snd_fw_transaction(digi->unit, TCODE_WRITE_BLOCK_REQUEST, 0xffffe0000008ULL, &data, 8, 0);

	data[0] = BYTESWAP32_CONST(0xffc2ffff);
	data[1] = BYTESWAP32_CONST(0x00000040);
	snd_fw_transaction(digi->unit, TCODE_WRITE_BLOCK_REQUEST, 0xffffe0000014ULL, &data, 8, 0);

#elif 0 /* write two quadlets instead of continuous block */

	write_quadlet(digi, 0xffffe0000008ULL, 0xffc2ffff);
	write_quadlet(digi, 0xffffe000000cULL, 0x00000000);

	write_quadlet(digi, 0xffffe0000014ULL, 0xffc2ffff);
	write_quadlet(digi, 0xffffe0000018ULL, 0x00000040);
#endif
}

static int rack_init(struct digi *digi)
{
	/* sweep read all data regs */
	int i;
	for (i=0; i < /* 0x468 */ 0x010; i++) {
		if (i == 4) continue;
		read_quadlet(digi, 0xfffff0000400ULL + i);
	}
	read_quadlet(digi, 0xfffff0000400ULL);

	/* initialization sequence */

	write_quadlet(digi, 0xffffe0000004ULL, 0x00000002);
	if (poll_until(digi, 0xffffe0000000ULL, 0x00000000)) return -1;

	write_quadlet(digi, 0xffffe0000110ULL, 0x00000000); // set samplerate?

	rack_init_write_814_block(digi);

	write_quadlet(digi, 0xffffe0000110ULL, 0x00000001); // set samplerate?
	write_quadlet(digi, 0xffffe0000100ULL, 0x00000000); // ??
	write_quadlet(digi, 0xffffe0000100ULL, 0x00000001); // ??

	write_quadlet(digi, 0xffffe000011cULL, 0x00000000); //use for x44.1?
	write_quadlet(digi, 0xffffe0000120ULL, 0x00000003); //use for x44.1?

	write_quadlet(digi, 0xffffe0000118ULL, 0x00000000); // set clocksrc

	write_quadlet(digi, 0xffffe0000004ULL, 0x00000001); // start streaming
	if (poll_until(digi, 0xffffe0000000ULL, 0x00000001)) return -1;

	read_quadlet(digi, 0xffffe0000118ULL); // reset clock

	write_quadlet(digi, 0xffffe0000004ULL, 0x00000000);
	if (poll_until(digi, 0xffffe0000000ULL, 0x00000000)) return -1;

	write_quadlet(digi, 0xffffe0000124ULL, 0x00000001); //enable midi or low latency?

	write_quadlet(digi, 0xffffe0000004ULL, 0x00000001); // start streaming
	if (poll_until(digi, 0xffffe0000000ULL, 0x00000001)) return -1;

	read_quadlet(digi, 0xffffe0000118ULL); // reset clock

	write_quadlet(digi, 0xffffe0000004ULL, 0x00000000);
	if (poll_until(digi, 0xffffe0000000ULL, 0x00000000)) return -1;

	write_quadlet(digi, 0xffffe0000004ULL, 0x00000003);
	if (poll_until(digi, 0xffffe0000000ULL, 0x00000003)) return -1;

	write_quadlet(digi, 0xffffe0000004ULL, 0x00000002);
	if (poll_until(digi, 0xffffe0000000ULL, 0x00000000)) return -1;

	write_quadlet(digi, 0xffffe0000110ULL, 0x00000000); // set samplerate?

	rack_init_write_814_block(digi);

	write_quadlet(digi, 0xffffe0000110ULL, 0x00000000); // set samplerate?

	write_quadlet(digi, 0xffffe0000100ULL, 0x00000000); // ??
	write_quadlet(digi, 0xffffe0000100ULL, 0x00000001); // ??

	write_quadlet(digi, 0xffffe0000004ULL, 0x00000001); // start streaming
	if (poll_until(digi, 0xffffe0000000ULL, 0x00000001)) return -1;

	read_quadlet(digi, 0xffffe0000118ULL); // reset clock

	write_quadlet(digi, 0xffffe0000124ULL, 0x00000001); // stop control

	write_quadlet(digi, 0xffffe0000004ULL, 0x00000000);
	if (poll_until(digi, 0xffffe0000000ULL, 0x00000000)) return -1;

	write_quadlet(digi, 0xffffe0000004ULL, 0x00000003); // shutdown streaming
	if (poll_until(digi, 0xffffe0000000ULL, 0x00000003)) return -1;

	write_quadlet(digi, 0xffffe0000004ULL, 0x00000002);
	if (poll_until(digi, 0xffffe0000000ULL, 0x00000000)) return -1;

	write_quadlet(digi, 0xffffe0000110ULL, 0x00000000); // set samplerate?

	rack_init_write_814_block(digi);

	write_quadlet(digi, 0xffffe0000110ULL, 0x00000001); // set samplerate?

	write_quadlet(digi, 0xffffe0000100ULL, 0x00000000); // ??
	write_quadlet(digi, 0xffffe0000100ULL, 0x00000001); // ??

	write_quadlet(digi, 0xffffe0000004ULL, 0x00000001); // start streaming
	if (poll_until(digi, 0xffffe0000000ULL, 0x00000001)) return -1;

	read_quadlet(digi, 0xffffe0000118ULL); // reset clock

	write_quadlet(digi, 0xffffe0000124ULL, 0x00000001); // stop control

#if 0
	//write_quadlet(digi, 0xffffe0000124ULL, 0x00000000); // start control
	/* No monitoring of inputs */

	write_quadlet(digi, R003_MIX_ANALOG_1L, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ANALOG_1R, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ANALOG_2L, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ANALOG_2R, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ANALOG_3L, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ANALOG_3R, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ANALOG_4L, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ANALOG_4R, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ANALOG_5L, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ANALOG_5R, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ANALOG_6L, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ANALOG_6R, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ANALOG_7L, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ANALOG_7R, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ANALOG_8L, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ANALOG_8R, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_SPDIF_1L, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_SPDIF_1R, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_SPDIF_1L, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_SPDIF_1R, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ADAT_1L, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ADAT_1R, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ADAT_2L, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ADAT_2R, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ADAT_3L, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ADAT_3R, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ADAT_4L, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ADAT_4R, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ADAT_5L, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ADAT_5R, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ADAT_6L, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ADAT_6R, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ADAT_7L, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ADAT_7R, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ADAT_8L, R003_MIX_NONE);
	write_quadlet(digi, R003_MIX_ADAT_8R, R003_MIX_NONE);
#endif
	return 0;
}

static void rack_shutdown(struct digi *digi)
{
#if 0
	write_quadlet(digi, 0xffffe0000124ULL, 0x00000001);   // stop control
#endif
	write_quadlet(digi, 0xffffe0000004ULL, 0x00000000);   // stop streams
	poll_until(digi, 0xffffe0000000ULL, 0x00000000);

	write_quadlet(digi, 0xffffe0000004ULL, 0x00000001);   // start streams
	poll_until(digi, 0xffffe0000000ULL, 0x00000001);
	write_quadlet(digi, 0xffffe0000118ULL, 0x00000000);

	write_quadlet(digi, 0xffffe0000004ULL, 0x00000000);   // stop streams
	poll_until(digi, 0xffffe0000000ULL, 0x00000000);

	write_quadlet(digi, 0xffffe0000004ULL, 0x00000003);   // shutdown streams
	poll_until(digi, 0xffffe0000000ULL, 0x00000003);
}

static int digi_owner_set(struct digi *digi)
{
	return 0;
}

static int digi_owner_update(struct digi *digi)
{
	return 0;
}
static void digi_owner_clear(struct digi *digi)
{
	return;
}

static int digi_enable_set(struct digi *digi)
{
	return 0;
}

static void digi_enable_clear(struct digi *digi)
{
	return;
}

static unsigned int rate_to_index(unsigned int rate)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(digi_rates); ++i)
		if (digi_rates[i] == rate)
			return i;

	return 0;
}

static unsigned int rate_index_to_mode(unsigned int rate_index)
{
	return ((int)rate_index - 1) / 2;
}

static void digi_lock_changed(struct digi *digi)
{
	digi->dev_lock_changed = true;
	wake_up(&digi->hwdep_wait);
}

static int digi_try_lock(struct digi *digi)
{
	int err;

	spin_lock_irq(&digi->lock);

	if (digi->dev_lock_count < 0) {
		err = -EBUSY;
		goto out;
	}

	if (digi->dev_lock_count++ == 0)
		digi_lock_changed(digi);
	err = 0;

out:
	spin_unlock_irq(&digi->lock);

	return err;
}

static void digi_unlock(struct digi *digi)
{
	spin_lock_irq(&digi->lock);

	if (WARN_ON(digi->dev_lock_count <= 0))
		goto out;

	if (--digi->dev_lock_count == 0)
		digi_lock_changed(digi);

out:
	spin_unlock_irq(&digi->lock);
}

static void dice_notification(struct fw_card *card, struct fw_request *request,
			      int tcode, int destination, int source,
			      int generation, unsigned long long offset,
			      void *data, size_t length, void *callback_data)
{
	struct digi *digi = callback_data;
	u32 bits;
	unsigned long flags;

	if (tcode != TCODE_WRITE_QUADLET_REQUEST) {
		fw_send_response(card, request, RCODE_TYPE_ERROR);
		return;
	}
	if ((offset & 3) != 0) {
		fw_send_response(card, request, RCODE_ADDRESS_ERROR);
		return;
	}

	bits = be32_to_cpup(data);

	spin_lock_irqsave(&digi->lock, flags);
	digi->notification_bits |= bits;
	spin_unlock_irqrestore(&digi->lock, flags);

	fw_send_response(card, request, RCODE_COMPLETE);

	if (bits & NOTIFY_CLOCK_ACCEPTED)
		complete(&digi->clock_accepted);
	wake_up(&digi->hwdep_wait);
}

static int digi_rate_constraint(struct snd_pcm_hw_params *params,
				struct snd_pcm_hw_rule *rule)
{
	struct digi *digi = rule->private;
	const struct snd_interval *channels =
		hw_param_interval_c(params, SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_interval *rate =
		hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval allowed_rates = {
		.min = UINT_MAX, .max = 0, .integer = 1
	};
	unsigned int i, mode;

	for (i = 0; i < ARRAY_SIZE(digi_rates); ++i) {
		mode = rate_index_to_mode(i);
		if ((digi->clock_caps & (1 << i)) &&
		    snd_interval_test(channels, digi->rx_channels[mode])) {
			allowed_rates.min = min(allowed_rates.min,
						digi_rates[i]);
			allowed_rates.max = max(allowed_rates.max,
						digi_rates[i]);
		}
	}

	return snd_interval_refine(rate, &allowed_rates);
}

static int digi_channels_constraint(struct snd_pcm_hw_params *params,
				    struct snd_pcm_hw_rule *rule)
{
	struct digi *digi = rule->private;
	const struct snd_interval *rate =
		hw_param_interval_c(params, SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels =
		hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_interval allowed_channels = {
		.min = UINT_MAX, .max = 0, .integer = 1
	};
	unsigned int i, mode;

	for (i = 0; i < ARRAY_SIZE(digi_rates); ++i)
		if ((digi->clock_caps & (1 << i)) &&
		    snd_interval_test(rate, digi_rates[i])) {
			mode = rate_index_to_mode(i);
			allowed_channels.min = min(allowed_channels.min,
						   digi->rx_channels[mode]);
			allowed_channels.max = max(allowed_channels.max,
						   digi->rx_channels[mode]);
		}

	return snd_interval_refine(channels, &allowed_channels);
}

static int digi_open(struct snd_pcm_substream *substream)
{
	static const struct snd_pcm_hardware hardware = {
		.info = SNDRV_PCM_INFO_MMAP |
			SNDRV_PCM_INFO_MMAP_VALID |
			SNDRV_PCM_INFO_BATCH |
			SNDRV_PCM_INFO_INTERLEAVED |
			SNDRV_PCM_INFO_BLOCK_TRANSFER,
		.formats = AMDTP_PCM_FORMAT_BIT,
		.channels_min = 19,
		.channels_max = 19,
		.buffer_bytes_max = 19 * 1024 * 1024,
		.period_bytes_min = 1,
		.period_bytes_max = UINT_MAX,
		.periods_min = 1,
		.periods_max = UINT_MAX,
	};
	struct digi *digi = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned int i;
	int err;

	err = digi_try_lock(digi);
	if (err < 0)
		goto error;

	runtime->hw = hardware;

	for (i = 0; i < ARRAY_SIZE(digi_rates); ++i)
		if (digi->clock_caps & (1 << i))
			runtime->hw.rates |=
				snd_pcm_rate_to_rate_bit(digi_rates[i]);
	snd_pcm_limit_hw_rates(runtime);

	for (i = 0; i < MODES; ++i)
		if (digi->rx_channels[i]) {
			runtime->hw.channels_min = min(runtime->hw.channels_min,
						       digi->rx_channels[i]);
			runtime->hw.channels_max = max(runtime->hw.channels_max,
						       digi->rx_channels[i]);
		}

	err = snd_pcm_hw_rule_add(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
				  digi_rate_constraint, digi,
				  SNDRV_PCM_HW_PARAM_CHANNELS, -1);
	if (err < 0)
		goto err_lock;
	err = snd_pcm_hw_rule_add(runtime, 0, SNDRV_PCM_HW_PARAM_CHANNELS,
				  digi_channels_constraint, digi,
				  SNDRV_PCM_HW_PARAM_RATE, -1);
	if (err < 0)
		goto err_lock;

	err = snd_pcm_hw_constraint_step(runtime, 0,
					 SNDRV_PCM_HW_PARAM_PERIOD_SIZE, 32);
	if (err < 0)
		goto err_lock;
	err = snd_pcm_hw_constraint_step(runtime, 0,
					 SNDRV_PCM_HW_PARAM_BUFFER_SIZE, 32);
	if (err < 0)
		goto err_lock;

	err = snd_pcm_hw_constraint_minmax(runtime,
					   SNDRV_PCM_HW_PARAM_PERIOD_TIME,
					   5000, UINT_MAX);
	if (err < 0)
		goto err_lock;

	err = snd_pcm_hw_constraint_msbits(runtime, 0, 32, 24);
	if (err < 0)
		goto err_lock;

	return 0;

err_lock:
	digi_unlock(digi);
error:
	return err;
}

static int digi_close(struct snd_pcm_substream *substream)
{
	struct digi *digi = substream->private_data;

	digi_unlock(digi);

	return 0;
}

static void digi_free_resources(struct digi *digi)
{
	unsigned int i;
	__be32 channel;

	channel = cpu_to_be32((u32)-1);
#if 0
	for (i = 0; i < digi->rx_count[digi->current_mode]; ++i)
		snd_fw_transaction(digi->unit, TCODE_WRITE_QUADLET_REQUEST,
				   rx_address(digi, i, RX_ISOCHRONOUS),
				   &channel, 4, 0);
#endif
	rack_shutdown(digi);

	fw_iso_resources_free(&digi->rx_resources);
}

static int digi_allocate_resources(struct digi *digi)
{
	unsigned int seq_start, i;
	__be32 values[2];
	int err;

	if (digi->rx_resources.allocated)
		return 0;

	err = fw_iso_resources_allocate(&digi->rx_resources,
			amdtp_stream_get_max_payload(&digi->rx_stream),
			fw_parent_device(digi->unit)->max_speed);
	if (err < 0)
		return err;

	values[0] = cpu_to_be32(digi->rx_resources.channel);
	seq_start = 0;
	for (i = 0; i < digi->rx_count[digi->current_mode]; ++i) {
		values[1] = cpu_to_be32(seq_start);
#if 0
		err = snd_fw_transaction(digi->unit,
					 TCODE_WRITE_BLOCK_REQUEST,
					 rx_address(digi, i, RX_ISOCHRONOUS),
					 values, 2 * 4, 0);
#else
		err = 0;
#endif
		if (err < 0) {
			digi_free_resources(digi);
			return err;
		}
		seq_start += digi->rx[i].pcm_channels[digi->current_mode];
		if (digi->rx_stream.dual_wire)
			seq_start += digi->rx[i].pcm_channels[digi->current_mode];
		seq_start += digi->rx[i].midi_ports[digi->current_mode] > 0;
	}

	return 0;
}

static int digi_start_packet_streaming(struct digi *digi)
{
	int err;

	if (amdtp_stream_running(&digi->rx_stream))
		return 0;

	err = amdtp_stream_start(&digi->rx_stream,
				 digi->rx_resources.channel,
				 fw_parent_device(digi->unit)->max_speed);
	if (err < 0)
		return err;

	err = digi_enable_set(digi);
	if (err < 0) {
		amdtp_stream_stop(&digi->rx_stream);
		return err;
	}

	return 0;
}

static int digi_start_streaming(struct digi *digi)
{
	int err = 0;

	err = digi_allocate_resources(digi);
	if (err < 0)
		return err;

	if (rack_init(digi)) {
		digi_free_resources(digi);
		return -1;
	}

	err = digi_start_packet_streaming(digi);
	if (err < 0) {
		digi_free_resources(digi);
		return err;
	}

	return 0;
}

static void digi_stop_packet_streaming(struct digi *digi)
{
	if (amdtp_stream_running(&digi->rx_stream)) {
		digi_enable_clear(digi);
		amdtp_stream_stop(&digi->rx_stream);
	}
}

static void digi_stop_streaming(struct digi *digi)
{
	digi_stop_packet_streaming(digi);

	if (digi->rx_resources.allocated)
		digi_free_resources(digi);
}

static int digi_change_rate(struct digi *digi, unsigned int clock_rate)
{
#if 0
	__be32 value;
	int err;

	INIT_COMPLETION(digi->clock_accepted);

	value = cpu_to_be32(clock_rate | CLOCK_SOURCE_ARX1);
	err = snd_fw_transaction(digi->unit, TCODE_WRITE_QUADLET_REQUEST,
				 global_address(digi, GLOBAL_CLOCK_SELECT),
				 &value, 4, 0);

	err = 0;
	if (err < 0)
		return err;

	if (!wait_for_completion_timeout(&digi->clock_accepted,
					 msecs_to_jiffies(100)))
		dev_warn(&digi->unit->device, "clock change timed out\n");
#endif
	return 0;
}

static int digi_hw_params(struct snd_pcm_substream *substream,
			  struct snd_pcm_hw_params *hw_params)
{
	struct digi *digi = substream->private_data;
	unsigned int rate_index, mode, midi_data_channels;
	unsigned int q, ch, m, rx;
	int err;

	mutex_lock(&digi->mutex);
	digi_stop_streaming(digi);
	mutex_unlock(&digi->mutex);

	err = snd_pcm_lib_alloc_vmalloc_buffer(substream,
					       params_buffer_bytes(hw_params));
	if (err < 0)
		return err;

	rate_index = rate_to_index(params_rate(hw_params));
	err = digi_change_rate(digi, rate_index << CLOCK_RATE_SHIFT);
	if (err < 0)
		return err;

	mode = rate_index_to_mode(rate_index);
	digi->current_mode = mode;

	midi_data_channels = 1;
#if 0
	for (rx = 0; rx < digi->rx_count[mode]; ++rx)
		midi_data_channels += digi->rx[rx].midi_ports[mode] > 0;
#endif
	amdtp_stream_set_parameters(&digi->rx_stream,
				    params_rate(hw_params),
				    params_channels(hw_params),
				    midi_data_channels);

	/*
	 * When using multiple receivers with MIDI or dual-wire, the packets in
	 * a data block are not in the default order.
	 */
	q = 0;
	ch = 0;
	m = 0;
	digi->rx_stream.midi_quadlets[0] = 0;
	digi->rx_stream.pcm_quadlets[0] = 1;
	digi->rx_stream.pcm_quadlets[1] = 2;
	digi->rx_stream.pcm_quadlets[2] = 3;
	digi->rx_stream.pcm_quadlets[3] = 4;
	digi->rx_stream.pcm_quadlets[4] = 5;
	digi->rx_stream.pcm_quadlets[5] = 6;
	digi->rx_stream.pcm_quadlets[6] = 7;
	digi->rx_stream.pcm_quadlets[7] = 8;
	digi->rx_stream.pcm_quadlets[8] = 9;
	digi->rx_stream.pcm_quadlets[9] = 10;
	digi->rx_stream.pcm_quadlets[10] = 11;
	digi->rx_stream.pcm_quadlets[11] = 12;
	digi->rx_stream.pcm_quadlets[12] = 13;
	digi->rx_stream.pcm_quadlets[13] = 14;
	digi->rx_stream.pcm_quadlets[14] = 15;
	digi->rx_stream.pcm_quadlets[15] = 16;
	digi->rx_stream.pcm_quadlets[16] = 17;
	digi->rx_stream.pcm_quadlets[17] = 18;

#if 0
	//digi->rx_stream.midi_quadlets[0] = 0;
	digi->rx_stream.pcm_quadlets[0] = 0;
	digi->rx_stream.pcm_quadlets[1] = 1;
	digi->rx_stream.pcm_quadlets[2] = 2;
	digi->rx_stream.pcm_quadlets[3] = 3;
	digi->rx_stream.pcm_quadlets[4] = 4;
	digi->rx_stream.pcm_quadlets[5] = 5;
	digi->rx_stream.pcm_quadlets[6] = 6;
	digi->rx_stream.pcm_quadlets[7] = 7;
	digi->rx_stream.pcm_quadlets[8] = 8;
	digi->rx_stream.pcm_quadlets[9] = 9;
	digi->rx_stream.pcm_quadlets[10] = 10;
	digi->rx_stream.pcm_quadlets[11] = 11;
	digi->rx_stream.pcm_quadlets[12] = 12;
	digi->rx_stream.pcm_quadlets[13] = 13;
	digi->rx_stream.pcm_quadlets[14] = 14;
	digi->rx_stream.pcm_quadlets[15] = 15;
	digi->rx_stream.pcm_quadlets[16] = 16;
	digi->rx_stream.pcm_quadlets[17] = 17;
#endif
	return 0;
}

static int digi_hw_free(struct snd_pcm_substream *substream)
{
	struct digi *digi = substream->private_data;

	mutex_lock(&digi->mutex);
	digi_stop_streaming(digi);

	mutex_unlock(&digi->mutex);

	return snd_pcm_lib_free_vmalloc_buffer(substream);
}

static int digi_prepare(struct snd_pcm_substream *substream)
{
	struct digi *digi = substream->private_data;
	int err;

	mutex_lock(&digi->mutex);

	if (amdtp_streaming_error(&digi->rx_stream))
		digi_stop_packet_streaming(digi);

	err = digi_start_streaming(digi);
	if (err < 0) {
		mutex_unlock(&digi->mutex);
		return err;
	}

	mutex_unlock(&digi->mutex);

	amdtp_stream_pcm_prepare(&digi->rx_stream);

	return 0;
}

static int digi_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct digi *digi = substream->private_data;
	struct snd_pcm_substream *pcm;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		pcm = substream;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		pcm = NULL;
		break;
	default:
		return -EINVAL;
	}
	amdtp_stream_pcm_trigger(&digi->rx_stream, pcm);

	return 0;
}

static snd_pcm_uframes_t digi_pointer(struct snd_pcm_substream *substream)
{
	struct digi *digi = substream->private_data;

	return amdtp_stream_pcm_pointer(&digi->rx_stream);
}

static int digi_create_pcm(struct digi *digi)
{
	static struct snd_pcm_ops ops = {
		.open      = digi_open,
		.close     = digi_close,
		.ioctl     = snd_pcm_lib_ioctl,
		.hw_params = digi_hw_params,
		.hw_free   = digi_hw_free,
		.prepare   = digi_prepare,
		.trigger   = digi_trigger,
		.pointer   = digi_pointer,
		.page      = snd_pcm_lib_get_vmalloc_page,
		.mmap      = snd_pcm_lib_mmap_vmalloc,
	};
	struct snd_pcm *pcm;
	int err;

	err = snd_pcm_new(digi->card, "003R", 0, 1, 0, &pcm);
	if (err < 0)
		return err;
	pcm->private_data = digi;
	strcpy(pcm->name, digi->card->shortname);
	pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream->ops = &ops;

	return 0;
}

static long digi_hwdep_read(struct snd_hwdep *hwdep, char __user *buf,
			    long count, loff_t *offset)
{
	struct digi *digi = hwdep->private_data;
	DEFINE_WAIT(wait);
	union snd_firewire_event event;

	spin_lock_irq(&digi->lock);

	while (!digi->dev_lock_changed && digi->notification_bits == 0) {
		prepare_to_wait(&digi->hwdep_wait, &wait, TASK_INTERRUPTIBLE);
		spin_unlock_irq(&digi->lock);
		schedule();
		finish_wait(&digi->hwdep_wait, &wait);
		if (signal_pending(current))
			return -ERESTARTSYS;
		spin_lock_irq(&digi->lock);
	}

	memset(&event, 0, sizeof(event));
	if (digi->dev_lock_changed) {
		event.lock_status.type = SNDRV_FIREWIRE_EVENT_LOCK_STATUS;
		event.lock_status.status = digi->dev_lock_count > 0;
		digi->dev_lock_changed = false;

		count = min(count, (long)sizeof(event.lock_status));
	} else {
		event.dice_notification.type = SNDRV_FIREWIRE_EVENT_DICE_NOTIFICATION;
		event.dice_notification.notification = digi->notification_bits;
		digi->notification_bits = 0;

		count = min(count, (long)sizeof(event.dice_notification));
	}

	spin_unlock_irq(&digi->lock);

	if (copy_to_user(buf, &event, count))
		return -EFAULT;

	return count;
}

static unsigned int digi_hwdep_poll(struct snd_hwdep *hwdep, struct file *file,
				    poll_table *wait)
{
	struct digi *digi = hwdep->private_data;
	unsigned int events;

	poll_wait(file, &digi->hwdep_wait, wait);

	spin_lock_irq(&digi->lock);
	if (digi->dev_lock_changed || digi->notification_bits != 0)
		events = POLLIN | POLLRDNORM;
	else
		events = 0;
	spin_unlock_irq(&digi->lock);

	return events;
}

static int digi_hwdep_get_info(struct digi *digi, void __user *arg)
{
	struct fw_device *dev = fw_parent_device(digi->unit);
	struct snd_firewire_get_info info;

	memset(&info, 0, sizeof(info));
	info.type = SNDRV_FIREWIRE_TYPE_DICE;
	info.card = dev->card->index;
	*(__be32 *)&info.guid[0] = cpu_to_be32(dev->config_rom[3]);
	*(__be32 *)&info.guid[4] = cpu_to_be32(dev->config_rom[4]);
	strlcpy(info.device_name, dev_name(&dev->device),
		sizeof(info.device_name));

	if (copy_to_user(arg, &info, sizeof(info)))
		return -EFAULT;

	return 0;
}

static int digi_hwdep_lock(struct digi *digi)
{
	int err;

	spin_lock_irq(&digi->lock);

	if (digi->dev_lock_count == 0) {
		digi->dev_lock_count = -1;
		err = 0;
	} else {
		err = -EBUSY;
	}

	spin_unlock_irq(&digi->lock);

	return err;
}

static int digi_hwdep_unlock(struct digi *digi)
{
	int err;

	spin_lock_irq(&digi->lock);

	if (digi->dev_lock_count == -1) {
		digi->dev_lock_count = 0;
		err = 0;
	} else {
		err = -EBADFD;
	}

	spin_unlock_irq(&digi->lock);

	return err;
}

static int digi_hwdep_release(struct snd_hwdep *hwdep, struct file *file)
{
	struct digi *digi = hwdep->private_data;

	spin_lock_irq(&digi->lock);
	if (digi->dev_lock_count == -1)
		digi->dev_lock_count = 0;
	spin_unlock_irq(&digi->lock);

	return 0;
}

static int digi_hwdep_ioctl(struct snd_hwdep *hwdep, struct file *file,
			    unsigned int cmd, unsigned long arg)
{
	struct digi *digi = hwdep->private_data;

	switch (cmd) {
	case SNDRV_FIREWIRE_IOCTL_GET_INFO:
		return digi_hwdep_get_info(digi, (void __user *)arg);
	case SNDRV_FIREWIRE_IOCTL_LOCK:
		return digi_hwdep_lock(digi);
	case SNDRV_FIREWIRE_IOCTL_UNLOCK:
		return digi_hwdep_unlock(digi);
	default:
		return -ENOIOCTLCMD;
	}
}

#ifdef CONFIG_COMPAT
static int digi_hwdep_compat_ioctl(struct snd_hwdep *hwdep, struct file *file,
				   unsigned int cmd, unsigned long arg)
{
	return digi_hwdep_ioctl(hwdep, file, cmd,
				(unsigned long)compat_ptr(arg));
}
#else
#define digi_hwdep_compat_ioctl NULL
#endif

static int digi_create_hwdep(struct digi *digi)
{
	static const struct snd_hwdep_ops ops = {
		.read         = digi_hwdep_read,
		.release      = digi_hwdep_release,
		.poll         = digi_hwdep_poll,
		.ioctl        = digi_hwdep_ioctl,
		.ioctl_compat = digi_hwdep_compat_ioctl,
	};
	struct snd_hwdep *hwdep;
	int err;

	err = snd_hwdep_new(digi->card, "003R", 0, &hwdep);
	if (err < 0)
		return err;
	strcpy(hwdep->name, "003R");
	hwdep->iface = SNDRV_HWDEP_IFACE_FW_DICE;
	hwdep->ops = ops;
	hwdep->private_data = digi;
	hwdep->exclusive = true;

	return 0;
}

static int digi_proc_read_mem(struct digi *digi, void *buffer,
			      unsigned int offset_q, unsigned int quadlets)
{
	unsigned int i;
	int err;
#if 0
	err = snd_fw_transaction(digi->unit, TCODE_READ_BLOCK_REQUEST,
				 DICE_PRIVATE_SPACE + 4 * offset_q,
				 buffer, 4 * quadlets, 0);
#endif
	err = 0;
	if (err < 0)
		return err;

	for (i = 0; i < quadlets; ++i)
		be32_to_cpus(&((u32 *)buffer)[i]);

	return 0;
}

static const char *str_from_array(const char *const strs[], unsigned int count,
				  unsigned int i)
{
	if (i < count)
		return strs[i];
	else
		return "(unknown)";
}

static void digi_proc_fixup_string(char *s, unsigned int size)
{
	unsigned int i;

	for (i = 0; i < size; i += 4)
		cpu_to_le32s((u32 *)(s + i));

	for (i = 0; i < size - 2; ++i) {
		if (s[i] == '\0')
			return;
		if (s[i] == '\\' && s[i + 1] == '\\') {
			s[i + 2] = '\0';
			return;
		}
	}
	s[size - 1] = '\0';
}

static void digi_proc_read(struct snd_info_entry *entry,
			   struct snd_info_buffer *buffer)
{
	static const char *const section_names[5] = {
		"global", "tx", "rx", "ext_sync", "unused2"
	};
	static const char *const clock_sources[] = {
		"aes1", "aes2", "aes3", "aes4", "aes", "adat", "tdif",
		"wc", "arx1", "arx2", "arx3", "arx4", "internal"
	};
	static const char *const rates[] = {
		"48000", "48000", "48000", "96000", "96000", "96000", "96000",
		"any low", "any mid", "any high", "none"
	};
	struct digi *digi = entry->private_data;
	u32 sections[ARRAY_SIZE(section_names) * 2];
	struct {
		u32 number;
		u32 size;
	} tx_rx_header;
	union {
		struct {
			u32 owner_hi, owner_lo;
			u32 notification;
			char nick_name[NICK_NAME_SIZE];
			u32 clock_select;
			u32 enable;
			u32 status;
			u32 extended_status;
			u32 sample_rate;
			u32 version;
			u32 clock_caps;
			char clock_source_names[CLOCK_SOURCE_NAMES_SIZE];
		} global;
		struct {
			u32 iso;
			u32 number_audio;
			u32 number_midi;
			u32 speed;
			char names[TX_NAMES_SIZE];
			u32 ac3_caps;
			u32 ac3_enable;
		} tx;
		struct {
			u32 iso;
			u32 seq_start;
			u32 number_audio;
			u32 number_midi;
			char names[RX_NAMES_SIZE];
			u32 ac3_caps;
			u32 ac3_enable;
		} rx;
		struct {
			u32 clock_source;
			u32 locked;
			u32 rate;
			u32 adat_user_data;
		} ext_sync;
	} buf;
	unsigned int quadlets, stream, i;

	if (digi_proc_read_mem(digi, sections, 0, ARRAY_SIZE(sections)) < 0)
		return;
	snd_iprintf(buffer, "sections:\n");
	for (i = 0; i < ARRAY_SIZE(section_names); ++i)
		snd_iprintf(buffer, "  %s: offset %u, size %u\n",
			    section_names[i],
			    sections[i * 2], sections[i * 2 + 1]);

	quadlets = min_t(u32, sections[1], sizeof(buf.global) / 4);
	if (digi_proc_read_mem(digi, &buf.global, sections[0], quadlets) < 0)
		return;
	snd_iprintf(buffer, "global:\n");
	snd_iprintf(buffer, "  owner: %04x:%04x%08x\n",
		    buf.global.owner_hi >> 16,
		    buf.global.owner_hi & 0xffff, buf.global.owner_lo);
	snd_iprintf(buffer, "  notification: %08x\n", buf.global.notification);
	digi_proc_fixup_string(buf.global.nick_name, NICK_NAME_SIZE);
	snd_iprintf(buffer, "  nick name: %s\n", buf.global.nick_name);
	snd_iprintf(buffer, "  clock select: %s %s\n",
		    str_from_array(clock_sources, ARRAY_SIZE(clock_sources),
				   buf.global.clock_select & CLOCK_SOURCE_MASK),
		    str_from_array(rates, ARRAY_SIZE(rates),
				   (buf.global.clock_select & CLOCK_RATE_MASK)
				   >> CLOCK_RATE_SHIFT));
	snd_iprintf(buffer, "  enable: %u\n", buf.global.enable);
	snd_iprintf(buffer, "  status: %slocked %s\n",
		    buf.global.status & STATUS_SOURCE_LOCKED ? "" : "un",
		    str_from_array(rates, ARRAY_SIZE(rates),
				   (buf.global.status &
				    STATUS_NOMINAL_RATE_MASK)
				   >> CLOCK_RATE_SHIFT));
	snd_iprintf(buffer, "  ext status: %08x\n", buf.global.extended_status);
	snd_iprintf(buffer, "  sample rate: %u\n", buf.global.sample_rate);
	snd_iprintf(buffer, "  version: %u.%u.%u.%u\n",
		    (buf.global.version >> 24) & 0xff,
		    (buf.global.version >> 16) & 0xff,
		    (buf.global.version >>  8) & 0xff,
		    (buf.global.version >>  0) & 0xff);
	if (quadlets >= 90) {
		snd_iprintf(buffer, "  clock caps:");
		for (i = 0; i <= 6; ++i)
			if (buf.global.clock_caps & (1 << i))
				snd_iprintf(buffer, " %s", rates[i]);
		for (i = 0; i <= 12; ++i)
			if (buf.global.clock_caps & (1 << (16 + i)))
				snd_iprintf(buffer, " %s", clock_sources[i]);
		snd_iprintf(buffer, "\n");
		digi_proc_fixup_string(buf.global.clock_source_names,
				       CLOCK_SOURCE_NAMES_SIZE);
		snd_iprintf(buffer, "  clock source names: %s\n",
			    buf.global.clock_source_names);
	}

	if (digi_proc_read_mem(digi, &tx_rx_header, sections[2], 2) < 0)
		return;
	quadlets = min_t(u32, tx_rx_header.size, sizeof(buf.tx));
	for (stream = 0; stream < tx_rx_header.number; ++stream) {
		if (digi_proc_read_mem(digi, &buf.tx, sections[2] + 2 +
				       stream * tx_rx_header.size,
				       quadlets) < 0)
			break;
		snd_iprintf(buffer, "tx %u:\n", stream);
		snd_iprintf(buffer, "  iso channel: %d\n", (int)buf.tx.iso);
		snd_iprintf(buffer, "  audio channels: %u\n",
			    buf.tx.number_audio);
		snd_iprintf(buffer, "  midi ports: %u\n", buf.tx.number_midi);
		snd_iprintf(buffer, "  speed: S%u\n", 100u << buf.tx.speed);
		if (quadlets >= 68) {
			digi_proc_fixup_string(buf.tx.names, TX_NAMES_SIZE);
			snd_iprintf(buffer, "  names: %s\n", buf.tx.names);
		}
		if (quadlets >= 70) {
			snd_iprintf(buffer, "  ac3 caps: %08x\n",
				    buf.tx.ac3_caps);
			snd_iprintf(buffer, "  ac3 enable: %08x\n",
				    buf.tx.ac3_enable);
		}
	}

	if (digi_proc_read_mem(digi, &tx_rx_header, sections[4], 2) < 0)
		return;
	quadlets = min_t(u32, tx_rx_header.size, sizeof(buf.rx));
	for (stream = 0; stream < tx_rx_header.number; ++stream) {
		if (digi_proc_read_mem(digi, &buf.rx, sections[4] + 2 +
				       stream * tx_rx_header.size,
				       quadlets) < 0)
			break;
		snd_iprintf(buffer, "rx %u:\n", stream);
		snd_iprintf(buffer, "  iso channel: %d\n", (int)buf.rx.iso);
		snd_iprintf(buffer, "  sequence start: %u\n", buf.rx.seq_start);
		snd_iprintf(buffer, "  audio channels: %u\n",
			    buf.rx.number_audio);
		snd_iprintf(buffer, "  midi ports: %u\n", buf.rx.number_midi);
		if (quadlets >= 68) {
			digi_proc_fixup_string(buf.rx.names, RX_NAMES_SIZE);
			snd_iprintf(buffer, "  names: %s\n", buf.rx.names);
		}
		if (quadlets >= 70) {
			snd_iprintf(buffer, "  ac3 caps: %08x\n",
				    buf.rx.ac3_caps);
			snd_iprintf(buffer, "  ac3 enable: %08x\n",
				    buf.rx.ac3_enable);
		}
	}

	quadlets = min_t(u32, sections[7], sizeof(buf.ext_sync) / 4);
	if (quadlets >= 4) {
		if (digi_proc_read_mem(digi, &buf.ext_sync,
				       sections[6], 4) < 0)
			return;
		snd_iprintf(buffer, "ext status:\n");
		snd_iprintf(buffer, "  clock source: %s\n",
			    str_from_array(clock_sources,
					   ARRAY_SIZE(clock_sources),
					   buf.ext_sync.clock_source));
		snd_iprintf(buffer, "  locked: %u\n", buf.ext_sync.locked);
		snd_iprintf(buffer, "  rate: %s\n",
			    str_from_array(rates, ARRAY_SIZE(rates),
					   buf.ext_sync.rate));
		snd_iprintf(buffer, "  adat user data: ");
		if (buf.ext_sync.adat_user_data & ADAT_USER_DATA_NO_DATA)
			snd_iprintf(buffer, "-\n");
		else
			snd_iprintf(buffer, "%x\n",
				    buf.ext_sync.adat_user_data);
	}
}

static void digi_create_proc(struct digi *digi)
{
	struct snd_info_entry *entry;

	if (!snd_card_proc_new(digi->card, "digi", &entry))
		snd_info_set_text_ops(entry, digi, digi_proc_read);
}

static void digi_card_free(struct snd_card *card)
{
	struct digi *digi = card->private_data;

	amdtp_stream_destroy(&digi->rx_stream);
	fw_core_remove_address_handler(&digi->notification_handler);
	fw_unit_put(digi->unit);
	mutex_destroy(&digi->mutex);
}

#define OUI_MAUDIO		0x000d6c
#define OUI_WEISS		0x001c6a

#define DICE_CATEGORY_ID	0x04
#define WEISS_CATEGORY_ID	0x00

static int __devinit digi_interface_check(struct fw_unit *unit)
{
	static const int min_values[10] __devinitconst = {
		10, 0x64 / 4,
		10, 0x18 / 4,
		10, 0x18 / 4,
		0, 0,
		0, 0,
	};
	struct fw_device *device = fw_parent_device(unit);
	struct fw_csr_iterator it;
	int key, value, vendor = -1, model = -1, err;
	unsigned int category, i;
	__be32 pointers[ARRAY_SIZE(min_values)];
	__be32 version;

	/*
	 * Check that GUID and unit directory are constructed according to DICE
	 * rules, i.e., that the specifier ID is the GUID's OUI, and that the
	 * GUID chip ID consists of the 8-bit category ID, the 10-bit product
	 * ID, and a 22-bit serial number.
	 */
	fw_csr_iterator_init(&it, unit->directory);
	while (fw_csr_iterator_next(&it, &key, &value)) {
		switch (key) {
		case CSR_SPECIFIER_ID:
			vendor = value;
			break;
		case CSR_MODEL:
			model = value;
			break;
		}
	}
	if (vendor == OUI_WEISS)
		category = WEISS_CATEGORY_ID;
	else
		category = DICE_CATEGORY_ID;
	if (device->config_rom[3] != ((vendor << 8) | category) ||
	    device->config_rom[4] >> 22 != model)
		return -ENODEV;

#if 0
	/*
	 * Check that the sub address spaces exist and are located inside the
	 * private address space.  The minimum values are chosen so that all
	 * minimally required registers are included.
	 */
	err = snd_fw_transaction(unit, TCODE_READ_BLOCK_REQUEST,
				 DICE_PRIVATE_SPACE,
				 pointers, sizeof(pointers), 0);
#endif

	err = 0;
	if (err < 0)
		return -ENODEV;
	for (i = 0; i < ARRAY_SIZE(pointers); ++i) {
		value = be32_to_cpu(pointers[i]);
		if (value < min_values[i] || value >= 0x40000)
			return -ENODEV;
	}

	/*
	 * Check that the implemented DICE driver specification major version
	 * number matches.
	 */
/*	err = snd_fw_transaction(unit, TCODE_READ_QUADLET_REQUEST,
				 DICE_PRIVATE_SPACE +
				 be32_to_cpu(pointers[0]) * 4 + GLOBAL_VERSION,
				 &version, 4, 0);
*/
	err = 0;
	if (err < 0)
		return -ENODEV;
	if ((version & cpu_to_be32(0xff000000)) != cpu_to_be32(0x01000000)) {
		dev_err(&unit->device,
			"unknown DICE version: 0x%08x\n", be32_to_cpu(version));
		return -ENODEV;
	}

	return vendor;
}

static int __devinit highest_supported_mode_rate(struct digi *digi,
						 unsigned int mode)
{
	int i;

	for (i = ARRAY_SIZE(digi_rates) - 1; i >= 0; --i)
		if ((digi->clock_caps & (1 << i)) &&
		    rate_index_to_mode(i) == mode)
			return i;

	return -1;
}

static int __devinit digi_read_mode_params(struct digi *digi, unsigned int mode)
{
	__be32 values[2];
	int rate_index, err;
	unsigned int i;

	rate_index = highest_supported_mode_rate(digi, mode);
	if (rate_index < 0) {
		digi->rx_count[mode] = 0;
		digi->rx_channels[mode] = 0;
		return 0;
	}

	err = digi_change_rate(digi, rate_index << CLOCK_RATE_SHIFT);
	if (err < 0)
		return err;
#if 0
	err = snd_fw_transaction(digi->unit, TCODE_READ_QUADLET_REQUEST,
				 rx_address(digi, 0, RX_NUMBER),
				 values, 4, 0);
#endif
	err = 0;
	if (err < 0)
		return err;
	digi->rx_count[mode] = 1; //be32_to_cpu(values[0]);
	if (digi->rx_count[mode] > MAX_RX) {
		dev_err(&digi->unit->device, "#rx(%u) = %u: too large\n",
			mode, digi->rx_count[mode]);
		return -ENXIO;
	}

	digi->rx_channels[mode] = 0;
	for (i = 0; i < digi->rx_count[mode]; ++i) {
#if 0
		err = snd_fw_transaction(digi->unit, TCODE_READ_BLOCK_REQUEST,
					 rx_address(digi, i, RX_NUMBER_AUDIO),
					 values, 2 * 4, 0);
#else
		err = 0;
#endif
		if (err < 0)
			return err;
		digi->rx[i].pcm_channels[mode] = 18; //be32_to_cpu(values[0]);
		digi->rx[i].midi_ports[mode]   = 1; //be32_to_cpu(values[1]);
#if 0
		if (digi->rx[i].pcm_channels[mode] > (mode < 2 ? 16 : 8) &&
		    (digi->vendor != OUI_MAUDIO || i > 0)) {
			dev_err(&digi->unit->device,
				"rx%u(%u): #PCM = %u: too large\n",
				i, mode, digi->rx[i].pcm_channels[mode]);
			return -ENXIO;
		}
		if (digi->rx[i].midi_ports[mode] > 8) {
			dev_err(&digi->unit->device,
				"rx%u(%u): #MIDI = %u: too large\n",
				i, mode, digi->rx[i].midi_ports[mode]);
			return -ENXIO;
		}
#endif
		digi->rx_channels[mode] += digi->rx[i].pcm_channels[mode];
	}

	if (digi->vendor == OUI_MAUDIO && digi->rx_count[mode] > 1) {
		if (digi->rx[0].pcm_channels[mode] <= (mode < 2 ? 16 : 8))
			digi->rx[0].pcm_channels[mode] =
						digi->rx_channels[mode];
		digi->rx_count[mode] = 1;
	}

	return 0;
}

static int __devinit digi_read_params(struct digi *digi)
{
	__be32 pointers[6];
	__be32 value;
	int mode, err;
#if 0
	err = snd_fw_transaction(digi->unit, TCODE_READ_BLOCK_REQUEST,
				 DICE_PRIVATE_SPACE,
				 pointers, sizeof(pointers), 0);
#endif
	err = 0;
	if (err < 0)
		return err;
	digi->global_offset = 0;//be32_to_cpu(pointers[0]) * 4;
	digi->rx_offset = 0;//be32_to_cpu(pointers[4]) * 4;

#if 0
	err = snd_fw_transaction(digi->unit, TCODE_READ_QUADLET_REQUEST,
				 rx_address(digi, 0, RX_SIZE),
				 &value, 4, 0);
#endif
	err = 0;
	if (err < 0)
		return err;
	digi->rx_size = 7; // XXX ? //be32_to_cpu(value) * 4;

	/* this should be supported by any device */
	digi->clock_caps = CLOCK_CAP_RATE_48000 |
			   CLOCK_CAP_RATE_96000 |
			   //CLOCK_CAP_SOURCE_ARX1 |
			   CLOCK_CAP_SOURCE_INTERNAL;

	for (mode = 2; mode >= 0; --mode) {
		err = digi_read_mode_params(digi, mode);
		if (err < 0)
			return err;
	}

	return 0;
}

static void __devinit digi_card_strings(struct digi *digi)
{
	struct snd_card *card = digi->card;
	struct fw_device *dev = fw_parent_device(digi->unit);
	char vendor[32], model[32];
	unsigned int i;
	int err;

	strcpy(card->driver, "003R");

	strcpy(card->shortname, "003R");
	BUILD_BUG_ON(NICK_NAME_SIZE < sizeof(card->shortname));
#if 0
	err = snd_fw_transaction(digi->unit, TCODE_READ_BLOCK_REQUEST,
				 global_address(digi, GLOBAL_NICK_NAME),
				 card->shortname, sizeof(card->shortname), 0);
#endif
	err = 0;
	if (err >= 0) {
		/* DICE strings are returned in "always-wrong" endianness */
		BUILD_BUG_ON(sizeof(card->shortname) % 4 != 0);
		for (i = 0; i < sizeof(card->shortname); i += 4)
			swab32s((u32 *)&card->shortname[i]);
		card->shortname[sizeof(card->shortname) - 1] = '\0';
	}

	strcpy(vendor, "?");
	fw_csr_string(dev->config_rom + 5, CSR_VENDOR, vendor, sizeof(vendor));
	strcpy(model, "?");
	fw_csr_string(digi->unit->directory, CSR_MODEL, model, sizeof(model));
	snprintf(card->longname, sizeof(card->longname),
		 "%s %s (serial %u) at %s, S%d",
		 vendor, model, dev->config_rom[4] & 0x3fffff,
		 dev_name(&digi->unit->device), 100 << dev->max_speed);

	strcpy(card->mixername, "003R");
}

static int __devinit digi_probe(struct device *unit_dev)
{
	struct fw_unit *unit = fw_unit(unit_dev);
	struct snd_card *card;
	struct digi *digi;
	__be32 clock_sel;
	int vendor, err;

	vendor = VENDOR_DIGIDESIGN; //digi_interface_check(unit);
	if (vendor < 0)
		return vendor;

	err = snd_card_create(-1, NULL, THIS_MODULE, sizeof(*digi), &card);
	if (err < 0)
		return err;
	snd_card_set_dev(card, unit_dev);

	digi = card->private_data;
	digi->card = card;
	spin_lock_init(&digi->lock);
	mutex_init(&digi->mutex);
	digi->unit = fw_unit_get(unit);
	init_completion(&digi->clock_accepted);
	init_waitqueue_head(&digi->hwdep_wait);
	digi->vendor = vendor;

	digi->notification_handler.length = 4;
	digi->notification_handler.address_callback = dice_notification;
	digi->notification_handler.callback_data = digi;
	err = fw_core_add_address_handler(&digi->notification_handler,
					  &fw_high_memory_region);
	if (err < 0)
		goto err_unit;

	err = digi_owner_set(digi);
	if (err < 0)
		goto err_notification_handler;

	err = digi_read_params(digi);
	if (err < 0)
		goto err_owner;

	err = fw_iso_resources_init(&digi->rx_resources, unit);
	if (err < 0)
		goto err_owner;
	digi->rx_resources.channels_mask = 0x0000000000000002uLL; // XYZ

	err = amdtp_stream_init(&digi->rx_stream, unit, CIP_NONBLOCKING);
	if (err < 0)
		goto err_resources;
#if 0
	if (vendor != OUI_MAUDIO)
		digi->rx_stream.flags |= CIP_HI_DUALWIRE;
#endif
	digi->rx_stream.use_digimagic=true;

	card->private_free = digi_card_free;

	digi_card_strings(digi);
#if 0
	err = snd_fw_transaction(unit, TCODE_READ_QUADLET_REQUEST,
				 global_address(digi, GLOBAL_CLOCK_SELECT),
				 &clock_sel, 4, 0);
	if (err < 0)
		goto error;
	clock_sel &= cpu_to_be32(~CLOCK_SOURCE_MASK);
	clock_sel |= cpu_to_be32(CLOCK_SOURCE_ARX1);
	err = snd_fw_transaction(unit, TCODE_WRITE_QUADLET_REQUEST,
				 global_address(digi, GLOBAL_CLOCK_SELECT),
				 &clock_sel, 4, 0);
#endif
	err = 0;
	if (err < 0)
		goto error;

	err = digi_create_pcm(digi);
	if (err < 0)
		goto error;

	err = digi_create_hwdep(digi);
	if (err < 0)
		goto error;

	digi_create_proc(digi);

	err = snd_card_register(card);
	if (err < 0)
		goto error;

	dev_set_drvdata(unit_dev, digi);

	return 0;

err_resources:
	fw_iso_resources_destroy(&digi->rx_resources);
err_owner:
	digi_owner_clear(digi);
err_notification_handler:
	fw_core_remove_address_handler(&digi->notification_handler);
err_unit:
	fw_unit_put(digi->unit);
	mutex_destroy(&digi->mutex);
error:
	snd_card_free(card);
	return err;
}

static int __devexit digi_remove(struct device *dev)
{
	struct digi *digi = dev_get_drvdata(dev);

	amdtp_stream_pcm_abort(&digi->rx_stream);

	snd_card_disconnect(digi->card);

	mutex_lock(&digi->mutex);

	digi_stop_streaming(digi);

	digi_owner_clear(digi);

	mutex_unlock(&digi->mutex);

	snd_card_free_when_closed(digi->card);

	return 0;
}

static void digi_bus_reset(struct fw_unit *unit)
{
	struct digi *digi = dev_get_drvdata(&unit->device);

	/*
	 * On a bus reset, the DICE firmware disables streaming and then goes
	 * off contemplating its own navel for hundreds of milliseconds before
	 * it can react to any of our attempts to reenable streaming.  This
	 * means that we lose synchronization anyway, so we force our streams
	 * to stop so that the application can restart them in an orderly
	 * manner.
	 */
	amdtp_stream_pcm_abort(&digi->rx_stream);

	mutex_lock(&digi->mutex);

	digi->global_enabled = false;
	digi_stop_packet_streaming(digi);

	digi_owner_update(digi);

	fw_iso_resources_update(&digi->rx_resources);

	mutex_unlock(&digi->mutex);
}

#define DICE_INTERFACE	0x000001

static const struct ieee1394_device_id digi_id_table[] = {
	{
		.match_flags = IEEE1394_MATCH_VERSION,
		.version     = DICE_INTERFACE,
	},
	{
		.match_flags  = IEEE1394_MATCH_SPECIFIER_ID |
				IEEE1394_MATCH_VERSION,
		.specifier_id = OUI_MAUDIO,
		.version      = 0x0100d1,
	},
	{
		.match_flags  = IEEE1394_MATCH_VENDOR_ID |
		                IEEE1394_MATCH_MODEL_ID,
		.vendor_id    = VENDOR_DIGIDESIGN,
//                .model_id     = R003_MODEL_ID,
	},
	{ }
};
MODULE_DEVICE_TABLE(ieee1394, digi_id_table);

static struct fw_driver digi_driver = {
	.driver   = {
		.owner	= THIS_MODULE,
		.name	= KBUILD_MODNAME,
		.bus	= &fw_bus_type,
		.probe	= digi_probe,
		.remove	= __devexit_p(digi_remove),
	},
	.update   = digi_bus_reset,
	.id_table = digi_id_table,
};

static int __init alsa_digi_init(void)
{
	return driver_register(&digi_driver.driver);
}

static void __exit alsa_digi_exit(void)
{
	driver_unregister(&digi_driver.driver);
}

module_init(alsa_digi_init);
module_exit(alsa_digi_exit);

/* vi:set ts=8 sts=8 sw=8: */
