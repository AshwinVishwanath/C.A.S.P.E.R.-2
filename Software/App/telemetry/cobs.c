#include "cobs.h"

int cobs_encode(const uint8_t *input, int in_len,
                uint8_t *output, int out_max)
{
    if (in_len < 0 || out_max < in_len + 1) {
        return -1;
    }

    int out_idx = 0;
    int code_idx = out_idx;   /* position of the next overhead byte */
    uint8_t code = 1;

    out_idx++;  /* reserve space for first overhead byte */
    if (out_idx > out_max) {
        return -1;
    }

    for (int i = 0; i < in_len; i++) {
        if (input[i] == 0x00) {
            /* Write the distance code at the saved position */
            if (code_idx >= out_max) {
                return -1;
            }
            output[code_idx] = code;
            code_idx = out_idx;
            code = 1;
            out_idx++;
            if (out_idx > out_max) {
                return -1;
            }
        } else {
            if (out_idx >= out_max) {
                return -1;
            }
            output[out_idx] = input[i];
            out_idx++;
            code++;

            if (code == 0xFF) {
                /* Max block length reached — emit code and start new block */
                if (code_idx >= out_max) {
                    return -1;
                }
                output[code_idx] = code;
                code_idx = out_idx;
                code = 1;
                out_idx++;
                if (out_idx > out_max) {
                    return -1;
                }
            }
        }
    }

    /* Write final overhead byte */
    if (code_idx >= out_max) {
        return -1;
    }
    output[code_idx] = code;

    return out_idx;
}

int cobs_decode(const uint8_t *input, int in_len,
                uint8_t *output, int out_max)
{
    if (in_len < 1) {
        return -1;
    }

    int in_idx = 0;
    int out_idx = 0;

    while (in_idx < in_len) {
        uint8_t code = input[in_idx];
        in_idx++;

        if (code == 0x00) {
            return -1;  /* unexpected zero in COBS stream */
        }

        for (uint8_t i = 1; i < code; i++) {
            if (in_idx >= in_len) {
                return -1;  /* truncated */
            }
            if (out_idx >= out_max) {
                return -1;  /* output overflow */
            }
            output[out_idx] = input[in_idx];
            out_idx++;
            in_idx++;
        }

        /* If code < 0xFF and not at end, insert a zero */
        if (code < 0xFF && in_idx < in_len) {
            if (out_idx >= out_max) {
                return -1;
            }
            output[out_idx] = 0x00;
            out_idx++;
        }
    }

    return out_idx;
}
