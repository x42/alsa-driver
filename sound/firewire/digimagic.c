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

#include "digimagic.h"

#define BYTE_PER_SAMPLE (4)
#define MAGIC_DIGI_BYTE (2)

#define MAGIC_BYTE_OFF(x) ( ( (x) * BYTE_PER_SAMPLE ) + MAGIC_DIGI_BYTE )

/** digi look up table
 *
 * @param idx index byte (audio-sample data) 0x00..0xff
 * @param off channel offset shift
 * @return salt to XOR with given data
 * */
static const __u8 digiscrt(const __u8 idx, const unsigned int off) {
	/** the length of the added pattern only depends on the lower nibble
	 * of the last non-zero data */
	const __u8 len[16] = {0, 1, 3, 5, 7, 9, 11, 13, 14, 12, 10, 8, 6, 4, 2, 0};

	/** the lower nibble of the salt. Interleaved sequence.
	 * this is walked backwards according to len[] */
	const __u8 nib[15] = {0x8, 0x7, 0x9, 0x6, 0xa, 0x5, 0xb, 0x4, 0xc, 0x3, 0xd, 0x2, 0xe, 0x1, 0xf};

	/** circular list for the salt's hi nibble. */
	const __u8 hir[15] = {0x0, 0x6, 0xf, 0x8, 0x7, 0x5, 0x3, 0x4, 0xc, 0xd, 0xe, 0x1, 0x2, 0xb, 0xa};

	/** start offset for upper nibble mapping.
	 * note: 9 is /special/. In the case where the high nibble == 0x9, hir[] is
	 * not used and - coincidentally - the salt's hi nibble is 0x09 regardless
	 * of the offset.
	 */
	const __u8 hio[16] = {0, 11, 12, 6, 7, 5, 1, 4, 3, 0x00, 14, 13, 8, 9, 10, 2};

#if 0 /* the current algorithm never calls digiscrt(.., 0) -- optimize away */
	if (off == 0) return idx; /* the first byte is identical to itself */
#endif

	const __u8 ln = idx & 0xf;
	const __u8 hn = (idx >> 4) & 0xf;
	const __u8 hr = (hn == 0x9) ? 0x9 : hir[ (hio[hn] + off) % 15 ];

	if (len[ln] < off) return 0x00;
	return ( (nib[14 + off - len[ln]]) | (hr << 4) );
}


/*  ----- SIMPLE API ----- */

void digi_encode(__u8 * const data, const int nch) {
	int c;
	DigiMagic state = {0x00, 0x00, 0};

	for (c = 0; c < nch; ++c) {
		if (data[MAGIC_BYTE_OFF(c)] != 0x00) {
			state.off = 0;
			state.idx = data[MAGIC_BYTE_OFF(c)] ^ state.carry;
		}
		data[MAGIC_BYTE_OFF(c)] ^= state.carry;
		state.carry = digiscrt(state.idx, ++(state.off));
	}
}

void digi_decode(__u8 * const data, const int nch) {
	int c;
	DigiMagic state = {0x00, 0x00, 0};

	for (c = 0; c < nch; ++c) {
		data[MAGIC_BYTE_OFF(c)] ^= state.carry;
		if (data[MAGIC_BYTE_OFF(c)] != 0x00) {
			state.off = 0;
			state.idx = data[MAGIC_BYTE_OFF(c)] ^ state.carry;
		}
		state.carry = digiscrt(state.idx, ++(state.off));
	}
}

/*  ----- Iterative call API ----- */

void digi_state_reset(DigiMagic *state) {
	state->carry = 0x00;
	state->idx   = 0x00;
	state->off   = 0;
}

void digi_encode_step(DigiMagic *state, __be32 * const buffer) {
	__u8 * const data = (__u8*) buffer;

	if (data[MAGIC_DIGI_BYTE] != 0x00) {
		state->off = 0;
		state->idx = data[MAGIC_DIGI_BYTE] ^ state->carry;
	}
	data[MAGIC_DIGI_BYTE] ^= state->carry;
	state->carry = digiscrt(state->idx, ++(state->off));
}

void digi_encode_qmap(__be32 * const buffer, __u8 *pcm_quadlets, const int nch) {
	int c;
	DigiMagic state;
	digi_state_reset(&state);

	for (c = 0; c < nch; ++c) {
		digi_encode_step(&state, &buffer[pcm_quadlets[c]]);
	}
}

void digi_decode_step(DigiMagic *state, __be32 * const buffer) {
	__u8 * const data = (__u8*) buffer;

	data[MAGIC_DIGI_BYTE] ^= state->carry;
	if (data[MAGIC_DIGI_BYTE] != 0x00) {
		state->off = 0;
		state->idx = data[MAGIC_DIGI_BYTE] ^ state->carry;
	}
	state->carry = digiscrt(state->idx, ++(state->off));
}

void digi_decode_qmap(__be32 * const buffer, __u8 *pcm_quadlets, const int nch) {
	int c;
	DigiMagic state;
	digi_state_reset(&state);

	for (c = 0; c < nch; ++c) {
		digi_decode_step(&state, &buffer[pcm_quadlets[c]]);
	}
}
