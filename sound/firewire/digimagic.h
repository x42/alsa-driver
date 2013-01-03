/*
 * Copyright (C) 2012 Robin Gareus <robin@gareus.org>
 * Copyright (C) 2012 Damien Zammit <damien@zamaudio.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef DIGIMAGIC_H_INCLUDED
#define DIGIMAGIC_H_INCLUDED

#include <linux/types.h>

struct DigiMagic {
	__u8 carry;
	__u8 idx;
	unsigned int off;
};

typedef struct DigiMagic DigiMagic;

/** prepare raw audio data for sending to digi003
 *
 * This function takes a pointer to interleaved audio data. Each sample must
 * be BYTE_PER_SAMPLE bytes long and there must be nch continuous samples.
 *
 * This function needs to be called once for each frame.
 *
 * It rewrites the byte MAGIC_DIGI_BYTE of each sample according to
 * digidesign003(TM) magic(TM).
 *
 * @param data interleaved audio-data to be rewritten in place
 * @param nch number of channels per frame
 */
void digi_encode(__u8 * const data, const int nch);

/** identical to \ref digi_encode except the samples do not need to
 * be in a packed continuous structure.
 *
 * @param data audio-data buffer to be rewritten in place
 * @param pcm_quadlets offset for each channel in the buffer.
 * @param nch number of channels per frame
 */
void digi_encode_qmap(__be32 * const buffer, __u8 *pcm_quadlets, const int nch);

/** decode audio data received from a digi003
 *
 * see \ref digi_encode for details
 *
 * @param data interleaved audio-data to be rewritten in place
 * @param nch number of channels per frame
 */
void digi_decode(__u8 * const data, const int nch);

/** identical to \ref digi_decode except the samples do not need to
 * be in a packed continuous structure.
 *
 * @param data audio-data buffer to be rewritten in place
 * @param pcm_quadlets offset for each channel in the buffer.
 * @param nch number of channels per frame
 */
void digi_decode_qmap(__be32 * const buffer, __u8 *pcm_quadlets, const int nch);

/**
 */
void digi_state_reset(DigiMagic *state);

/**
 */
void digi_encode_step(DigiMagic *state, __be32 * const buffer);

/**
 */
void digi_decode_step(DigiMagic *state, __be32 * const buffer);

#endif

