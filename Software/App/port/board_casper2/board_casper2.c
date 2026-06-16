/**
 * @file  board_casper2.c
 * @brief CASPER port-seam implementation for the Casper 2 STM32H750VBT6 board.
 *
 * THIS IS THE ONLY FILE IN App/ ALLOWED TO #include "main.h" OR
 * "stm32h7xx_hal.h" (except App/radio/sx1276.c, which is an approved
 * exception because it is Casper-2-only and will be replaced on Casper 3).
 *
 * Implements every function declared in:
 *   casper_time.h   — casper_millis / casper_delay_ms / casper_micros
 *   casper_gpio.h   — casper_gpio_write / read / toggle
 *   casper_spi.h    — casper_spi_transmit / receive / transceive
 *   casper_i2c.h    — casper_i2c_mem_read / mem_write / master_tx / dev_ready
 *   casper_adc.h    — casper_adc_init_all / casper_adc_read
 *   casper_qspi.h   — casper_qspi_command / transmit / receive /
 *                     transmit_it / autopoll_it / set_handler +
 *                     HAL_QSPI_*Callback definitions
 *   casper_crc.h    — casper_crc32_init / casper_crc32_compute
 *   casper_pwm.h    — casper_pwm_tone_start / set / off
 *
 * Also exports board singletons (bus handles, GPIO pin descriptors) that
 * are declared in board_casper2.h.
 *
 * All implementations are thin one-liner wrappers around the STM32H7 HAL.
 * No business logic belongs here.
 */

/* =========================================================================
 *  MUST come first — the only App/ file that includes these headers.
 * ======================================================================= */
#include "main.h"              /* HAL handles externs, GPIO_PIN_*, GPIO_TypeDef */
#include "stm32h7xx_hal.h"     /* HAL_SPI_*, HAL_I2C_*, HAL_ADC_*, DWT, … */

#include "board_casper2.h"     /* Our own exported declarations */

#include <stdint.h>
#include <stdbool.h>

/* =========================================================================
 *  Concrete struct definitions  (forward-declared in casper_types.h)
 *
 *  App/ code holds only pointers to these types and never dereferences them.
 *  The definitions are therefore invisible outside this translation unit.
 * ======================================================================= */

struct casper_spi_s {
    SPI_HandleTypeDef *h;  /**< HAL SPI peripheral handle. */
};

struct casper_i2c_s {
    I2C_HandleTypeDef *h;  /**< HAL I2C peripheral handle. */
};

struct casper_qspi_s {
    QSPI_HandleTypeDef *h;                        /**< HAL QSPI handle.           */
    void (*handler)(void *ctx, casper_qspi_evt_t); /**< IT event callback, or NULL.*/
    void *handler_ctx;                             /**< Context passed to handler. */
};

struct casper_adc_s {
    ADC_HandleTypeDef *h;       /**< HAL ADC peripheral handle (hadc1/2/3).   */
    uint32_t           channel; /**< ADC_CHANNEL_n value for this logical ch. */
};

struct casper_pwm_s {
    TIM_HandleTypeDef *h;    /**< HAL TIM handle. */
    uint32_t           chan; /**< TIM_CHANNEL_n constant. */
};

/* =========================================================================
 *  Board singleton definitions
 *
 *  The extern HAL handles (hspi2, hi2c1, hqspi, hadc1 …) are defined in
 *  Core/Src/main.c by CubeMX-generated code.  We reference them here via
 *  standard C external linkage; no declaration needed because main.h (which
 *  we include above) pulls in stm32h7xx_hal.h which includes the type
 *  definitions, and the linker resolves the symbols at link time.
 *
 *  HAL handle externs — declared implicitly via main.h → stm32h7xx_hal.h
 *  types, but not declared as extern variables.  Declare them explicitly:
 * ======================================================================= */
extern SPI_HandleTypeDef  hspi1;
extern SPI_HandleTypeDef  hspi2;
extern SPI_HandleTypeDef  hspi3;
extern SPI_HandleTypeDef  hspi4;
extern I2C_HandleTypeDef  hi2c1;
extern I2C_HandleTypeDef  hi2c3;
extern QSPI_HandleTypeDef hqspi;
extern ADC_HandleTypeDef  hadc1;
extern ADC_HandleTypeDef  hadc2;
extern ADC_HandleTypeDef  hadc3;
extern CRC_HandleTypeDef  hcrc;
extern TIM_HandleTypeDef  htim4;

/* ── SPI buses ── */
casper_spi_t BSP_SPI_IMU   = { &hspi2 };  /* LSM6DSO32 — SPI2, Mode 3, 5.25 MHz  */
casper_spi_t BSP_SPI_BARO  = { &hspi4 };  /* MS5611    — SPI4, Mode 0, 10.5 MHz  */
casper_spi_t BSP_SPI_HIGHG = { &hspi3 };  /* ADXL372   — SPI3                     */

/* ── I2C buses ── */
casper_i2c_t BSP_I2C_GPS = { &hi2c1 };  /* MAX-M10M GPS, addr8 = 0x84 */
casper_i2c_t BSP_I2C_MAG = { &hi2c3 };  /* MMC5983MA mag, addr8 = 0x60 */

/* ── QSPI flash ── */
casper_qspi_t BSP_QSPI_FLASH = { &hqspi, NULL, NULL };

/* ── PWM buzzer (TIM4 CH3, PD14) ── */
casper_pwm_t BSP_PWM_BUZZ = { &htim4, TIM_CHANNEL_3 };

/* ── ADC continuity channels ──────────────────────────────────────────────
 *
 *  Mapping mirrors the hw[] table in casper_pyro.c exactly:
 *    Index 0 — CH1: ADC1 channel 4  (PC4,   CONT1     ADC1_CH4)
 *    Index 1 — CH2: ADC1 channel 3  (PA6,   CONT_2    ADC1_CH3)
 *    Index 2 — CH3: ADC3 channel 1  (PC3_C, CONT_3    ADC3_CH1)
 *    Index 3 — CH4: ADC2 channel 10 (PC0,   CONT_4    ADC2_CH10)
 *
 *  casper_adc_t is an opaque type; the extern in board_casper2.h declares a
 *  pointer so that App/ code never needs the concrete struct size.  We define
 *  a static array here and export a pointer to its first element.
 * ----------------------------------------------------------------------- */
static casper_adc_t s_adc_cont[4] = {
    { &hadc1, ADC_CHANNEL_4  },   /* [0] CH1 — ADC1_CH4 */
    { &hadc1, ADC_CHANNEL_3  },   /* [1] CH2 — ADC1_CH3 */
    { &hadc3, ADC_CHANNEL_1  },   /* [2] CH3 — ADC3_CH1 */
    { &hadc2, ADC_CHANNEL_10 },   /* [3] CH4 — ADC2_CH10 */
};

/** Pointer to the continuity ADC channel array; indexable as BSP_ADC_CONT[0..3]. */
casper_adc_t *BSP_ADC_CONT = s_adc_cont;

/* =========================================================================
 *  GPIO pin descriptor definitions
 *
 *  port is cast from GPIO_TypeDef* to void* so that board_casper2.h can
 *  remain free of HAL types (App/ code never dereferences the void*;
 *  only casper_gpio_write/read/toggle in this file cast it back).
 * ======================================================================= */

/* ── IMU ── */
casper_pin_t BSP_PIN_IMU_CS   = { (void *)SPI2_CS_GPIO_Port,  SPI2_CS_Pin  };
casper_pin_t BSP_PIN_IMU_INT  = { (void *)SPI2_INT_GPIO_Port, SPI2_INT_Pin };

/* ── Barometer ── */
casper_pin_t BSP_PIN_BARO_CS  = { (void *)SPI4_CS_GPIO_Port, SPI4_CS_Pin };

/* ── High-G accelerometer ── */
casper_pin_t BSP_PIN_HIGHG_CS  = { (void *)SPI3_CS_GPIO_Port,  SPI3_CS_Pin  };
casper_pin_t BSP_PIN_HIGHG_INT = { (void *)SPI3_INT_GPIO_Port, SPI3_INT_Pin };

/* ── Radio ── */
casper_pin_t BSP_PIN_RADIO_CS   = { (void *)Radio_CS_GPIO_Port,    Radio_CS_Pin    };
casper_pin_t BSP_PIN_RADIO_NRST = { (void *)RADIO_NRST_GPIO_Port,  RADIO_NRST_Pin  };
casper_pin_t BSP_PIN_RADIO_DIO0 = { (void *)SPI1_INT_GPIO_Port,    SPI1_INT_Pin    };
casper_pin_t BSP_PIN_RADIO_DIO1 = { (void *)RADIO_DIO1_GPIO_Port,  RADIO_DIO1_Pin  };
casper_pin_t BSP_PIN_RADIO_DIO2 = { (void *)RADIO_DIO2_GPIO_Port,  RADIO_DIO2_Pin  };
casper_pin_t BSP_PIN_RADIO_DIO3 = { (void *)RADIO_DIO3_GPIO_Port,  RADIO_DIO3_Pin  };
casper_pin_t BSP_PIN_RADIO_DIO4 = { (void *)RADIO_DIO4_GPIO_Port,  RADIO_DIO4_Pin  };
casper_pin_t BSP_PIN_RADIO_DIO5 = { (void *)RADIO_DIO5_GPIO_Port,  RADIO_DIO5_Pin  };

/* ── GPS ── */
casper_pin_t BSP_PIN_GPS_NRST      = { (void *)NRST_GPS_GPIO_Port,       NRST_GPS_Pin       };
casper_pin_t BSP_PIN_GPS_TIMEPULSE = { (void *)GPS_TIMEPULSE_GPIO_Port,  GPS_TIMEPULSE_Pin  };
casper_pin_t BSP_PIN_GPS_INT       = { (void *)I2C1_INT_GPIO_Port,       I2C1_INT_Pin       };

/* ── Magnetometer ── */
casper_pin_t BSP_PIN_MAG_INT = { (void *)I2C_3_INT_GPIO_Port, I2C_3_INT_Pin };

/* ── Pyro fire outputs ── */
casper_pin_t BSP_PIN_PY1 = { (void *)PY1_GPIO_Port, PY1_Pin };
casper_pin_t BSP_PIN_PY2 = { (void *)PY2_GPIO_Port, PY2_Pin };
casper_pin_t BSP_PIN_PY3 = { (void *)PY3_GPIO_Port, PY3_Pin };
casper_pin_t BSP_PIN_PY4 = { (void *)PY4_GPIO_Port, PY4_Pin };

/* ── Continuity LEDs ── */
casper_pin_t BSP_PIN_CONT_YN_1 = { (void *)CONT_YN_1_GPIO_Port, CONT_YN_1_Pin };
casper_pin_t BSP_PIN_CONT_YN_2 = { (void *)CONT_YN_2_GPIO_Port, CONT_YN_2_Pin };
casper_pin_t BSP_PIN_CONT_YN_3 = { (void *)CONT_YN_3_GPIO_Port, CONT_YN_3_Pin };
casper_pin_t BSP_PIN_CONT_YN_4 = { (void *)CONT_YN_4_GPIO_Port, CONT_YN_4_Pin };

/* =========================================================================
 *  Helper: translate HAL_StatusTypeDef → casper_status_t
 * ======================================================================= */
static casper_status_t hal_to_casper(HAL_StatusTypeDef s)
{
    switch (s) {
        case HAL_OK:      return CASPER_OK;
        case HAL_TIMEOUT: return CASPER_TIMEOUT;
        default:          return CASPER_ERR;
    }
}

/* =========================================================================
 *  casper_time.h implementation
 * ======================================================================= */

uint32_t casper_millis(void)
{
    return HAL_GetTick();
}

void casper_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

uint32_t casper_micros(void)
{
    /*
     * DWT->CYCCNT increments at SYSCLK rate.  Dividing by the number of
     * cycles per microsecond gives elapsed µs.  SystemCoreClock is set by
     * HAL_RCC_GetSysClockFreq() / SystemCoreClockUpdate() during board init
     * and is 432 000 000 on Casper 2.  This function deliberately avoids
     * hard-coding that frequency — it reads it at run-time so that any future
     * clock change requires no edit here.
     *
     * Division is integer; sub-microsecond remainder is discarded (acceptable
     * for the short-interval profiling uses of this function).
     */
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

/* =========================================================================
 *  casper_gpio.h implementation
 * ======================================================================= */

void casper_gpio_write(casper_pin_t pin, casper_pin_state_t s)
{
    HAL_GPIO_WritePin((GPIO_TypeDef *)pin.port,
                      (uint16_t)pin.pin,
                      (s == CASPER_PIN_HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

casper_pin_state_t casper_gpio_read(casper_pin_t pin)
{
    GPIO_PinState state = HAL_GPIO_ReadPin((GPIO_TypeDef *)pin.port,
                                           (uint16_t)pin.pin);
    return (state == GPIO_PIN_SET) ? CASPER_PIN_HIGH : CASPER_PIN_LOW;
}

void casper_gpio_toggle(casper_pin_t pin)
{
    HAL_GPIO_TogglePin((GPIO_TypeDef *)pin.port, (uint16_t)pin.pin);
}

/* =========================================================================
 *  casper_spi.h implementation
 * ======================================================================= */

casper_status_t casper_spi_transmit(casper_spi_t *bus,
                                    const uint8_t *tx,
                                    uint16_t n,
                                    uint32_t to_ms)
{
    /* HAL_SPI_Transmit takes a non-const pointer; cast is safe — HAL does
     * not modify the buffer during a transmit-only transfer. */
    return hal_to_casper(HAL_SPI_Transmit(bus->h, (uint8_t *)tx, n, to_ms));
}

casper_status_t casper_spi_receive(casper_spi_t *bus,
                                   uint8_t *rx,
                                   uint16_t n,
                                   uint32_t to_ms)
{
    return hal_to_casper(HAL_SPI_Receive(bus->h, rx, n, to_ms));
}

casper_status_t casper_spi_transceive(casper_spi_t *bus,
                                      const uint8_t *tx,
                                      uint8_t *rx,
                                      uint16_t n,
                                      uint32_t to_ms)
{
    return hal_to_casper(
        HAL_SPI_TransmitReceive(bus->h, (uint8_t *)tx, rx, n, to_ms));
}

/* =========================================================================
 *  casper_i2c.h implementation
 * ======================================================================= */

casper_status_t casper_i2c_mem_read(casper_i2c_t *bus,
                                    uint16_t addr8,
                                    uint16_t reg,
                                    uint16_t reg_sz,
                                    uint8_t *buf,
                                    uint16_t n,
                                    uint32_t to_ms)
{
    /* reg_sz: 1 → I2C_MEMADD_SIZE_8BIT, 2 → I2C_MEMADD_SIZE_16BIT */
    uint16_t mem_add_size = (reg_sz == 2) ? I2C_MEMADD_SIZE_16BIT
                                          : I2C_MEMADD_SIZE_8BIT;
    return hal_to_casper(
        HAL_I2C_Mem_Read(bus->h, addr8, reg, mem_add_size, buf, n, to_ms));
}

casper_status_t casper_i2c_mem_write(casper_i2c_t *bus,
                                     uint16_t addr8,
                                     uint16_t reg,
                                     uint16_t reg_sz,
                                     const uint8_t *buf,
                                     uint16_t n,
                                     uint32_t to_ms)
{
    uint16_t mem_add_size = (reg_sz == 2) ? I2C_MEMADD_SIZE_16BIT
                                          : I2C_MEMADD_SIZE_8BIT;
    return hal_to_casper(
        HAL_I2C_Mem_Write(bus->h, addr8, reg, mem_add_size,
                          (uint8_t *)buf, n, to_ms));
}

casper_status_t casper_i2c_master_tx(casper_i2c_t *bus,
                                     uint16_t addr8,
                                     const uint8_t *buf,
                                     uint16_t n,
                                     uint32_t to_ms)
{
    return hal_to_casper(
        HAL_I2C_Master_Transmit(bus->h, addr8, (uint8_t *)buf, n, to_ms));
}

casper_status_t casper_i2c_dev_ready(casper_i2c_t *bus,
                                     uint16_t addr8,
                                     uint32_t trials,
                                     uint32_t to_ms)
{
    return hal_to_casper(
        HAL_I2C_IsDeviceReady(bus->h, addr8, trials, to_ms));
}

/* =========================================================================
 *  casper_adc.h implementation
 * ======================================================================= */

void casper_adc_init_all(void)
{
    /*
     * STM32H7 ADC must be calibrated after each power-on before the first
     * conversion.  Run offset calibration for single-ended mode on all three
     * ADC instances used for pyro continuity sensing.
     *
     * Channel configuration (ADC_ChannelConfTypeDef) per channel is applied
     * lazily inside casper_adc_read() so that each conversion always sets the
     * correct channel — this matches the behaviour of the original read_adc()
     * helper in casper_pyro.c.
     */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
}

casper_status_t casper_adc_read(casper_adc_t *ch, uint16_t *out_raw)
{
    /*
     * Configure, start, poll, read, stop — identical sequence to the
     * original read_adc() in casper_pyro.c.  SamplingTime and mode are
     * fixed per the original implementation.
     */
    ADC_ChannelConfTypeDef cfg = {0};
    cfg.Channel      = ch->channel;
    cfg.Rank         = ADC_REGULAR_RANK_1;
    cfg.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
    cfg.SingleDiff   = ADC_SINGLE_ENDED;
    cfg.OffsetNumber = ADC_OFFSET_NONE;
    cfg.Offset       = 0;

    if (HAL_ADC_ConfigChannel(ch->h, &cfg) != HAL_OK)
        return CASPER_ERR;

    if (HAL_ADC_Start(ch->h) != HAL_OK)
        return CASPER_ERR;

    HAL_StatusTypeDef poll = HAL_ADC_PollForConversion(ch->h, 2);
    *out_raw = (uint16_t)HAL_ADC_GetValue(ch->h);
    HAL_ADC_Stop(ch->h);

    if (poll == HAL_TIMEOUT)
        return CASPER_TIMEOUT;
    if (poll != HAL_OK)
        return CASPER_ERR;

    return CASPER_OK;
}

/* =========================================================================
 *  casper_qspi.h — blocking API
 * ======================================================================= */

/**
 * @brief Map a casper_qspi_cmd_t to a QSPI_CommandTypeDef and issue the
 *        command via HAL_QSPI_Command().
 */
casper_status_t casper_qspi_command(casper_qspi_t *dev,
                                    const casper_qspi_cmd_t *cmd,
                                    uint32_t to_ms)
{
    QSPI_CommandTypeDef hcmd = {0};

    /* Instruction phase */
    hcmd.InstructionMode = (cmd->instruction_lines == 4)
                               ? QSPI_INSTRUCTION_4_LINES
                               : QSPI_INSTRUCTION_1_LINE;
    hcmd.Instruction = cmd->instruction;

    /* Address phase */
    if (cmd->address_lines == 0) {
        hcmd.AddressMode = QSPI_ADDRESS_NONE;
    } else {
        hcmd.AddressMode = (cmd->address_lines == 4)
                               ? QSPI_ADDRESS_4_LINES
                               : QSPI_ADDRESS_1_LINE;
        hcmd.AddressSize = (cmd->address_bytes == 4)
                               ? QSPI_ADDRESS_32_BITS
                               : QSPI_ADDRESS_24_BITS;
        hcmd.Address = cmd->address;
    }

    /* Alternate byte phase — not used by w25q512jv driver */
    hcmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;

    /* Dummy cycles */
    hcmd.DummyCycles = cmd->dummy_cycles;

    /* Data phase */
    if (cmd->data_lines == 0) {
        hcmd.DataMode = QSPI_DATA_NONE;
    } else {
        hcmd.DataMode = (cmd->data_lines == 4)
                            ? QSPI_DATA_4_LINES
                            : QSPI_DATA_1_LINE;
        hcmd.NbData = cmd->data_len;
    }

    /* DDR / SIOO not used */
    hcmd.DdrMode  = QSPI_DDR_MODE_DISABLE;
    hcmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

    return hal_to_casper(HAL_QSPI_Command(dev->h, &hcmd, to_ms));
}

casper_status_t casper_qspi_transmit(casper_qspi_t *dev,
                                     const uint8_t *buf,
                                     uint32_t to_ms)
{
    return hal_to_casper(HAL_QSPI_Transmit(dev->h, (uint8_t *)buf, to_ms));
}

casper_status_t casper_qspi_receive(casper_qspi_t *dev,
                                    uint8_t *buf,
                                    uint32_t to_ms)
{
    return hal_to_casper(HAL_QSPI_Receive(dev->h, buf, to_ms));
}

/* =========================================================================
 *  casper_qspi.h — non-blocking (IT) API
 * ======================================================================= */

casper_status_t casper_qspi_transmit_it(casper_qspi_t *dev,
                                        const uint8_t *buf)
{
    return hal_to_casper(HAL_QSPI_Transmit_IT(dev->h, (uint8_t *)buf));
}

casper_status_t casper_qspi_autopoll_it(casper_qspi_t *dev,
                                        const casper_qspi_poll_t *poll)
{
    QSPI_CommandTypeDef cmd = {0};
    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction       = poll->instruction;
    cmd.AddressMode       = QSPI_ADDRESS_NONE;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode          = QSPI_DATA_1_LINE;
    cmd.DummyCycles       = 0;
    cmd.NbData            = 1;
    cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    QSPI_AutoPollingTypeDef cfg = {0};
    cfg.Match           = poll->match;
    cfg.Mask            = poll->mask;
    cfg.MatchMode       = QSPI_MATCH_MODE_AND;
    cfg.StatusBytesSize = 1;
    cfg.Interval        = poll->poll_interval;
    cfg.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

    return hal_to_casper(HAL_QSPI_AutoPolling_IT(dev->h, &cmd, &cfg));
}

void casper_qspi_set_handler(casper_qspi_t *dev,
                             void (*fn)(void *ctx, casper_qspi_evt_t evt),
                             void *ctx)
{
    dev->handler     = fn;
    dev->handler_ctx = ctx;
}

/* =========================================================================
 *  QSPI HAL IT callbacks  (weak symbols overridden here)
 *
 *  These three callbacks are the only HAL weak-symbol overrides in App/.
 *  They translate HAL events to casper_qspi_evt_t and dispatch to the
 *  handler registered via casper_qspi_set_handler().
 *
 *  NOTE: these live here, NOT in w25q512jv.c.  The driver's IT state
 *  machine will be ported to use casper_qspi_set_handler() in a later
 *  migration subsystem.  Until then w25q512jv.c still defines its own
 *  versions — so these definitions are intentionally NOT present yet to
 *  avoid duplicate symbol errors.  They are provided here as the target
 *  landing site and are guarded so they only compile once the driver has
 *  been migrated (CASPER_QSPI_CALLBACKS_IN_BOARD defined by the build).
 *
 *  IMPORTANT FOR THE INTEGRATOR:
 *    When w25q512jv.c's HAL_QSPI_*Callback definitions are removed as part
 *    of the QSPI migration subsystem, remove the #ifdef guard below so
 *    these definitions become active.
 * ======================================================================= */

#ifdef CASPER_QSPI_CALLBACKS_IN_BOARD

void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi)
{
    (void)hqspi;
    if (BSP_QSPI_FLASH.handler)
        BSP_QSPI_FLASH.handler(BSP_QSPI_FLASH.handler_ctx,
                               CASPER_QSPI_EVT_TX_DONE);
}

void HAL_QSPI_StatusMatchCallback(QSPI_HandleTypeDef *hqspi)
{
    (void)hqspi;
    if (BSP_QSPI_FLASH.handler)
        BSP_QSPI_FLASH.handler(BSP_QSPI_FLASH.handler_ctx,
                               CASPER_QSPI_EVT_MATCH);
}

void HAL_QSPI_ErrorCallback(QSPI_HandleTypeDef *hqspi)
{
    (void)hqspi;
    if (BSP_QSPI_FLASH.handler)
        BSP_QSPI_FLASH.handler(BSP_QSPI_FLASH.handler_ctx,
                               CASPER_QSPI_EVT_ERROR);
}

#endif /* CASPER_QSPI_CALLBACKS_IN_BOARD */

/* =========================================================================
 *  casper_crc.h implementation
 * ======================================================================= */

void casper_crc32_init(void)
{
    /*
     * Reconfigure the CRC peripheral for standard CRC-32 (IEEE 802.3):
     *   polynomial  : 0x04C11DB7 (default STM32 poly)
     *   reflected I : byte-level inversion (CRC_INPUTDATA_INVERSION_BYTE)
     *   reflected O : enabled
     * The final XOR of 0xFFFFFFFF is applied in casper_crc32_compute().
     *
     * Clock enable is idempotent; safe to call more than once.
     * This exactly mirrors crc32_hw_init() in crc32_hw.c.
     */
    __HAL_RCC_CRC_CLK_ENABLE();

    hcrc.Instance = CRC;
    hcrc.Init.DefaultPolynomialUse    = DEFAULT_POLYNOMIAL_ENABLE;
    hcrc.Init.DefaultInitValueUse     = DEFAULT_INIT_VALUE_ENABLE;
    hcrc.Init.InputDataInversionMode  = CRC_INPUTDATA_INVERSION_BYTE;
    hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;
    hcrc.InputDataFormat              = CRC_INPUTDATA_FORMAT_BYTES;

    HAL_CRC_Init(&hcrc);
}

uint32_t casper_crc32_compute(const uint8_t *data, uint32_t len)
{
    uint32_t raw = HAL_CRC_Calculate(&hcrc, (uint32_t *)data, len);
    /* Apply final XOR for standard CRC-32. */
    return raw ^ 0xFFFFFFFFu;
}

/* =========================================================================
 *  casper_pwm.h implementation
 * ======================================================================= */

void casper_pwm_tone_start(casper_pwm_t *pwm)
{
    HAL_TIM_PWM_Start(pwm->h, pwm->chan);
}

void casper_pwm_tone_set(casper_pwm_t *pwm, uint32_t arr, uint32_t ccr)
{
    __HAL_TIM_SET_AUTORELOAD(pwm->h, arr);
    __HAL_TIM_SET_COMPARE(pwm->h, pwm->chan, ccr);
}

void casper_pwm_tone_off(casper_pwm_t *pwm)
{
    HAL_TIM_PWM_Stop(pwm->h, pwm->chan);
}
