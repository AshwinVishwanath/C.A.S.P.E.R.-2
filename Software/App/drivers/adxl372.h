#ifndef ADXL372_H
#define ADXL372_H

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* Return codes */
#define ADXL372_READ_OK   0
#define ADXL372_ERROR    -1

/* ------------------------------------------------------------------ */
/*  Register addresses (from Analog Devices ADXL372 datasheet Rev B)  */
/* ------------------------------------------------------------------ */
#define ADXL372_ADI_DEVID        0x00u   /* Analog Devices ID: expect 0xAD */
#define ADXL372_MST_DEVID        0x01u   /* MEMS ID: expect 0x1D */
#define ADXL372_DEVID            0x02u   /* Device ID: expect 0xFA */
#define ADXL372_REVID            0x03u   /* Silicon revision */
#define ADXL372_STATUS_1         0x04u   /* DATA_RDY, FIFO_RDY, FIFO_FULL, FIFO_OVR */
#define ADXL372_STATUS_2         0x05u
#define ADXL372_FIFO_ENTRIES_2   0x06u
#define ADXL372_FIFO_ENTRIES_1   0x07u
#define ADXL372_X_DATA_H         0x08u   /* X-axis data bits [11:4] */
#define ADXL372_X_DATA_L         0x09u   /* X-axis data bits [3:0] left-justified */
#define ADXL372_Y_DATA_H         0x0Au
#define ADXL372_Y_DATA_L         0x0Bu
#define ADXL372_Z_DATA_H         0x0Cu
#define ADXL372_Z_DATA_L         0x0Du
#define ADXL372_X_MAXPEAK_H      0x15u
#define ADXL372_X_MAXPEAK_L      0x16u
#define ADXL372_Y_MAXPEAK_H      0x17u
#define ADXL372_Y_MAXPEAK_L      0x18u
#define ADXL372_Z_MAXPEAK_H      0x19u
#define ADXL372_Z_MAXPEAK_L      0x1Au
#define ADXL372_OFFSET_X         0x20u
#define ADXL372_OFFSET_Y         0x21u
#define ADXL372_OFFSET_Z         0x22u
#define ADXL372_THRESH_ACT_H     0x23u   /* Activity threshold high byte   */
#define ADXL372_THRESH_ACT_L     0x24u   /* Activity threshold low byte    */
#define ADXL372_TIME_ACT         0x25u   /* Activity time (samples)        */
#define ADXL372_ACT_INACT_CTL    0x27u   /* Activity/inactivity control    */
#define ADXL372_HPF              0x38u
#define ADXL372_FIFO_SAMPLES     0x39u
#define ADXL372_FIFO_CTL         0x3Au
#define ADXL372_INT1_MAP         0x3Bu
#define ADXL372_INT2_MAP         0x3Cu
#define ADXL372_TIMING           0x3Du   /* ODR selection (bits [7:5]) */
#define ADXL372_MEASURE          0x3Eu   /* Bandwidth (bits [2:0]) */
#define ADXL372_POWER_CTL        0x3Fu   /* Operating mode (bits [1:0]) */
#define ADXL372_SELF_TEST        0x40u
#define ADXL372_SRESET           0x41u   /* Write 0x52 for soft reset */
#define ADXL372_FIFO_DATA        0x42u

/* Expected device ID values */
#define ADXL372_ADI_DEVID_VAL    0xADu
#define ADXL372_MST_DEVID_VAL    0x1Du
#define ADXL372_DEVID_VAL        0xFAu

/* Soft-reset magic value */
#define ADXL372_RESET_CODE       0x52u

/* POWER_CTL operating modes (bits [1:0]) */
#define ADXL372_OP_STANDBY             0x00u
#define ADXL372_OP_WAKE_UP             0x01u
#define ADXL372_OP_INSTANT_ON          0x02u
#define ADXL372_OP_FULL_BW_MEASUREMENT 0x03u

/* MEASURE register bit fields (0x3E) */
#define ADXL372_LOW_NOISE_EN     (1u << 3)   /* Bit 3: low noise mode */

/* POWER_CTL additional bit fields (0x3F) */
#define ADXL372_LPF_DISABLE      (1u << 3)   /* Bit 3: disable digital LPF */
#define ADXL372_HPF_DISABLE      (1u << 2)   /* Bit 2: disable digital HPF */
#define ADXL372_FILTER_SETTLE_16 (1u << 4)   /* Bit 4: 16ms settle (vs 370ms) */

/* HPF corner frequency (register 0x38, bits [1:0]) */
/* At 400 Hz ODR: Corner0=1.90Hz, Corner1=0.97Hz, Corner2=0.49Hz, Corner3=0.24Hz */
#define ADXL372_HPF_CORNER0      0x00u
#define ADXL372_HPF_CORNER1      0x01u
#define ADXL372_HPF_CORNER2      0x02u
#define ADXL372_HPF_CORNER3      0x03u

/* TIMING register ODR (bits [7:5]) */
#define ADXL372_ODR_400HZ   (0u << 5)
#define ADXL372_ODR_800HZ   (1u << 5)
#define ADXL372_ODR_1600HZ  (2u << 5)
#define ADXL372_ODR_3200HZ  (3u << 5)
#define ADXL372_ODR_6400HZ  (4u << 5)

/* MEASURE register bandwidth (bits [2:0]) */
#define ADXL372_BW_200HZ    0u
#define ADXL372_BW_400HZ    1u
#define ADXL372_BW_800HZ    2u
#define ADXL372_BW_1600HZ   3u
#define ADXL372_BW_3200HZ   4u

/* FIFO_CTL (0x3A) mode bits [2:1] */
#define ADXL372_FIFO_DISABLED        (0u << 1)
#define ADXL372_FIFO_OLDEST_SAVED    (1u << 1)
#define ADXL372_FIFO_STREAM          (2u << 1)
#define ADXL372_FIFO_TRIGGERED       (3u << 1)
/* FIFO_CTL format bit [3] — 0=XYZ, 1=X-only */
#define ADXL372_FIFO_FORMAT_XYZ      (0u << 3)
#define ADXL372_FIFO_FORMAT_X        (1u << 3)

/* STATUS_1 flags */
#define ADXL372_STATUS_DATA_RDY  0x01u
#define ADXL372_STATUS_FIFO_RDY  0x02u
#define ADXL372_STATUS_FIFO_FULL 0x04u
#define ADXL372_STATUS_FIFO_OVR  0x08u

/* STATUS_2 flags */
#define ADXL372_STATUS2_ACT      0x10u   /* Bit 4: activity detected */

/* INT1_MAP activity bit */
#define ADXL372_INT1_ACT         0x10u   /* Bit 4: map activity to INT1 */

/* Wake-up mode launch threshold (build-time override via -DADXL_LAUNCH_G=X) */
#ifndef ADXL_LAUNCH_G
#define ADXL_LAUNCH_G            3.0f
#endif

/* ------------------------------------------------------------------ */
/*  Driver struct                                                      */
/* ------------------------------------------------------------------ */
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;

    float              accel_g[3];    /* Acceleration in g (+-200g range, 100 mg/LSB) */
    int16_t            raw_accel[3];  /* Raw 12-bit left-justified register values */
    volatile bool      data_ready;    /* For future interrupt-driven reads */
    uint8_t            device_id;     /* PARTID readback (expect 0xFA) */
} adxl372_t;

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

/* Initialise: soft-reset, verify DEVID, configure ODR/BW/mode.
 * Returns true on success. */
bool adxl372_init(adxl372_t *dev, SPI_HandleTypeDef *hspi,
                  GPIO_TypeDef *cs_port, uint16_t cs_pin);

/* Read 3-axis acceleration from data registers.
 * Stores result in dev->accel_g[].
 * Returns ADXL372_READ_OK on success. */
int adxl372_read(adxl372_t *dev);

/* Reconfigure for FIFO stream mode at given ODR. */
void adxl372_fifo_init(adxl372_t *dev, uint8_t odr_bits);

/* Read FIFO entry count (number of individual axis samples, not triplets). */
uint16_t adxl372_fifo_entries(adxl372_t *dev);

/* Read one XYZ triplet from FIFO. Stores raw int16 in dev->raw_accel[].
 * Returns 1 on success, 0 if FIFO empty. */
int adxl372_fifo_read(adxl372_t *dev);

/* Read a single register (for diagnostics / readback). */
uint8_t adxl372_read_reg_ext(adxl372_t *dev, uint8_t reg);

/* Call from HAL_GPIO_EXTI_Callback when INT pin fires (future use). */
void adxl372_irq_handler(adxl372_t *dev);

/* Configure ADXL372 for wake-up mode with activity detection.
 * threshold_g: acceleration threshold in g (e.g. 3.0)
 * time_act: number of consecutive samples above threshold (e.g. 6 → ~115ms @ 52Hz) */
void adxl372_wakeup_init(adxl372_t *dev, float threshold_g, uint8_t time_act);

/* Check if activity interrupt is asserted (reads STATUS_2 register).
 * Returns true if sustained acceleration above threshold detected. */
bool adxl372_activity_detected(adxl372_t *dev);

/* Transition from wake-up mode back to full measurement mode.
 * Call after launch detection to enable high-rate data logging. */
void adxl372_enter_measurement(adxl372_t *dev);

#endif /* ADXL372_H */
