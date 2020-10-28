/*
 * Port of Ardupilot's SBUS decoder, which in turn was based on src/modules/px4iofirmware/sbus.c
 * from PX4Firmware and modified for use in AP_HAL_* by Andrew Tridgell.
 *
 * Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define SBUS_FRAME_SIZE         25
#define SBUS_INPUT_CHANNELS     16
#define SBUS_FLAGS_BYTE         23
#define SBUS_FAILSAFE_BIT       3
#define SBUS_FRAMELOST_BIT      2

/* define range mapping here, -+100% -> 1000..2000 */
#define SBUS_RANGE_MIN          200.0f
#define SBUS_RANGE_MAX          1800.0f

#define SBUS_TARGET_MIN         1000.0f
#define SBUS_TARGET_MAX         2000.0f

/* pre-calculate the floating point stuff as far as possible at compile time */
#define SBUS_SCALE_FACTOR       ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN))
#define SBUS_SCALE_OFFSET       (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f))

#define MIN_RCIN_CHANNELS       5
#define MAX_RCIN_CHANNELS       16

#define MIN(_a, _b)             ((_a) < (_b) ? (_a) : (_b))
#define MAX(_a, _b)             ((_a) > (_b) ? (_a) : (_b))


typedef struct sbus_decoder_t
{
    struct
    {
        uint16_t  bytes[25]; // including start bit, parity and stop bits
        uint16_t  bit_ofs;
    } sbus_state;
    uint16_t     chan_values[MAX_RCIN_CHANNELS]; /* PWMs */
    uint8_t      chan_num;
    uint32_t     rc_input_count;
} sbus_decoder_t;


/* Single SBUS decoder instance */
static sbus_decoder_t g_sbus_decoder;


/*
 * S.bus decoder matrix.
 *
 * Each channel value can come from up to 3 input bytes. Each row in the
 * matrix describes up to three bytes, and each entry gives:
 *
 * - byte offset in the data portion of the frame
 * - right shift applied to the data byte
 * - mask for the data byte
 * - left shift applied to the result into the channel value
 */
struct sbus_bit_pick
{
    uint8_t byte;
    uint8_t rshift;
    uint8_t mask;
    uint8_t lshift;
};
static const struct sbus_bit_pick sbus_decoder[SBUS_INPUT_CHANNELS][3] =
{
    /*  0 */ { { 0, 0, 0xff, 0}, { 1, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
    /*  1 */ { { 1, 3, 0x1f, 0}, { 2, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
    /*  2 */ { { 2, 6, 0x03, 0}, { 3, 0, 0xff, 2}, { 4, 0, 0x01, 10} },
    /*  3 */ { { 4, 1, 0x7f, 0}, { 5, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
    /*  4 */ { { 5, 4, 0x0f, 0}, { 6, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
    /*  5 */ { { 6, 7, 0x01, 0}, { 7, 0, 0xff, 1}, { 8, 0, 0x03,  9} },
    /*  6 */ { { 8, 2, 0x3f, 0}, { 9, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
    /*  7 */ { { 9, 5, 0x07, 0}, {10, 0, 0xff, 3}, { 0, 0, 0x00,  0} },
    /*  8 */ { {11, 0, 0xff, 0}, {12, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
    /*  9 */ { {12, 3, 0x1f, 0}, {13, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
    /* 10 */ { {13, 6, 0x03, 0}, {14, 0, 0xff, 2}, {15, 0, 0x01, 10} },
    /* 11 */ { {15, 1, 0x7f, 0}, {16, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
    /* 12 */ { {16, 4, 0x0f, 0}, {17, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
    /* 13 */ { {17, 7, 0x01, 0}, {18, 0, 0xff, 1}, {19, 0, 0x03,  9} },
    /* 14 */ { {19, 2, 0x3f, 0}, {20, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
    /* 15 */ { {20, 5, 0x07, 0}, {21, 0, 0xff, 3}, { 0, 0, 0x00,  0} }
};

static void
add_input(sbus_decoder_t *dec, uint8_t num_values, uint16_t *values, bool in_failsafe)
{
    num_values = MIN(num_values, MAX_RCIN_CHANNELS);
    memcpy(dec->chan_values, values, num_values * sizeof(uint16_t));
    dec->chan_num = num_values;
    if (!in_failsafe) {
        dec->rc_input_count++;
    }
}

static bool
sbus_decode(const uint8_t frame[25], uint16_t *values, uint16_t *num_values,
            bool *sbus_failsafe, bool *sbus_frame_drop, uint16_t max_values)
{
    /* check frame boundary markers to avoid out-of-sync cases */
    if ((frame[0] != 0x0f)) {
        return false;
    }

    switch (frame[24]) {
        case 0x00:
        /* this is S.BUS 1 */
        break;
        case 0x03:
        /* S.BUS 2 SLOT0: RX battery and external voltage */
        break;
        case 0x83:
        /* S.BUS 2 SLOT1 */
        break;
        case 0x43:
        case 0xC3:
        case 0x23:
        case 0xA3:
        case 0x63:
        case 0xE3:
        break;
        default:
        /* we expect one of the bits above, but there are some we don't know yet */
        break;
    }

    unsigned channel;
    unsigned chancount = (max_values > SBUS_INPUT_CHANNELS) ?
                 SBUS_INPUT_CHANNELS : max_values;

    /* use the decoder matrix to extract channel data */
    for (channel = 0; channel < chancount; channel++) {
        unsigned value = 0;
        unsigned pick;

        for (pick = 0; pick < 3; pick++) {
            const struct sbus_bit_pick *decode = &sbus_decoder[channel][pick];

            if (decode->mask != 0) {
                unsigned piece = frame[1 + decode->byte];
                piece >>= decode->rshift;
                piece &= decode->mask;
                piece <<= decode->lshift;

                value |= piece;
            }
        }


        /* convert 0-2048 values to 1000-2000 ppm encoding in a not too sloppy fashion */
        values[channel] = (uint16_t)(value * SBUS_SCALE_FACTOR +.5f) + SBUS_SCALE_OFFSET;
    }

    /* decode switch channels if data fields are wide enough */
    if (max_values > 17 && chancount > 15) {
        chancount = 18;

        /* channel 17 (index 16) */
        values[16] = (frame[SBUS_FLAGS_BYTE] & (1 << 0)) * 1000 + 998;
        /* channel 18 (index 17) */
        values[17] = (frame[SBUS_FLAGS_BYTE] & (1 << 1)) * 1000 + 998;
    }

    /* note the number of channels decoded */
    *num_values = chancount;

    /* decode and handle failsafe and frame-lost flags */
    if (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FAILSAFE_BIT)) { /* failsafe */
        /* report that we failed to read anything valid off the receiver */
        *sbus_failsafe = true;
        *sbus_frame_drop = true;
    }
    else if (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FRAMELOST_BIT)) { /* a frame was lost */
        /* set a special warning flag
         *
         * Attention! This flag indicates a skipped frame only, not a total link loss! Handling this
         * condition as fail-safe greatly reduces the reliability and range of the radio link,
         * e.g. by prematurely issuing return-to-launch!!! */

        *sbus_failsafe = false;
        *sbus_frame_drop = true;
    } else {
        *sbus_failsafe = false;
        *sbus_frame_drop = false;
    }

    return true;
}

/*
 * Process a SBUS input pulse of the given width.
 */
static void
process_pulse(sbus_decoder_t *dec, uint32_t width_s0, uint32_t width_s1)
{
    // convert to bit widths, allowing for up to 1usec error, assuming 100000 bps
    uint16_t bits_s0 = (width_s0+1) / 10;
    uint16_t bits_s1 = (width_s1+1) / 10;
    uint16_t nlow;

    uint8_t byte_ofs = dec->sbus_state.bit_ofs/12;
    uint8_t bit_ofs = dec->sbus_state.bit_ofs%12;

    if (bits_s0 == 0 || bits_s1 == 0) {
        // invalid data
        goto reset;
    }

    if (bits_s0+bit_ofs > 10) {
        // invalid data as last two bits must be stop bits
        goto reset;
    }

    // pull in the high bits
    dec->sbus_state.bytes[byte_ofs] |= ((1U<<bits_s0)-1) << bit_ofs;
    dec->sbus_state.bit_ofs += bits_s0;
    bit_ofs += bits_s0;

    // pull in the low bits
    nlow = bits_s1;
    if (nlow + bit_ofs > 12) {
        nlow = 12 - bit_ofs;
    }
    bits_s1 -= nlow;
    dec->sbus_state.bit_ofs += nlow;

    if (dec->sbus_state.bit_ofs == 25*12 && bits_s1 > 12) {
        // we have a full frame
        uint8_t bytes[25];
        uint8_t i;
        for (i=0; i<25; i++) {
            // get inverted data
            uint16_t v = ~dec->sbus_state.bytes[i];
            // check start bit
            if ((v & 1) != 0) {
                goto reset;
            }
            // check stop bits
            if ((v & 0xC00) != 0xC00) {
                goto reset;
            }
            // check parity
            uint8_t parity = 0, j;
            for (j=1; j<=8; j++) {
                parity ^= (v & (1U<<j))?1:0;
            }
            if (parity != (v&0x200)>>9) {
                goto reset;
            }
            bytes[i] = ((v>>1) & 0xFF);
        }
        uint16_t values[MAX_RCIN_CHANNELS];
        uint16_t num_values=0;
        bool sbus_failsafe=false, sbus_frame_drop=false;
        if (sbus_decode(bytes, values, &num_values,
                        &sbus_failsafe, &sbus_frame_drop,
                        MAX_RCIN_CHANNELS) &&
            num_values >= MIN_RCIN_CHANNELS) {
            add_input(dec, num_values, values, sbus_failsafe);
        }
        goto reset;
    } else if (bits_s1 > 12) {
        // break
        goto reset;
    }
    return;
reset:
    memset(&dec->sbus_state, 0, sizeof(dec->sbus_state));
}


/*** Public API ****************************************************************/

void rc_sbus_decoder_init(void)
{
    memset(&g_sbus_decoder, 0, sizeof(g_sbus_decoder));
}

void rc_sbus_decoder_process_pulse(uint32_t width_s0, uint32_t width_s1)
{
    process_pulse(&g_sbus_decoder, width_s0, width_s1);
}

int rc_sbus_decoder_get_num_channels(void)
{
    return g_sbus_decoder.chan_num;
}

int rc_sbus_decoder_get_channel_pwm(int chan)
{
    if (chan < g_sbus_decoder.chan_num)
        return g_sbus_decoder.chan_values[chan];
    else
        return -1;
}
