#ifndef APP_TELEMETRY_COBS_H
#define APP_TELEMETRY_COBS_H

#include <stdint.h>

/**
 * COBS-encode input into output buffer.
 * Does NOT append the 0x00 delimiter — caller must do that.
 *
 * @param input   Source data (must not contain 0x00 in valid COBS use)
 * @param in_len  Number of source bytes
 * @param output  Destination buffer (must be at least in_len + 1 bytes)
 * @param out_max Size of output buffer
 * @return        Number of bytes written to output, or -1 on error
 */
int cobs_encode(const uint8_t *input, int in_len,
                uint8_t *output, int out_max);

/**
 * COBS-decode input into output buffer.
 * Input should NOT include the trailing 0x00 delimiter.
 *
 * @param input   COBS-encoded data (without delimiter)
 * @param in_len  Number of encoded bytes
 * @param output  Destination buffer
 * @param out_max Size of output buffer
 * @return        Number of decoded bytes, or -1 on error
 */
int cobs_decode(const uint8_t *input, int in_len,
                uint8_t *output, int out_max);

#endif /* APP_TELEMETRY_COBS_H */
