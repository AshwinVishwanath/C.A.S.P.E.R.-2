/* HAL stub for host-side testing — provides types only, no hardware access */
#ifndef STM32H7XX_HAL_H
#define STM32H7XX_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ---- Status types ---- */
typedef enum {
    HAL_OK      = 0x00,
    HAL_ERROR   = 0x01,
    HAL_BUSY    = 0x02,
    HAL_TIMEOUT = 0x03
} HAL_StatusTypeDef;

typedef enum {
    HAL_UNLOCKED = 0x00,
    HAL_LOCKED   = 0x01
} HAL_LockTypeDef;

/* ---- GPIO types ---- */
typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];
} GPIO_TypeDef;

typedef enum {
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET   = 1
} GPIO_PinState;

/* GPIO pin defines */
#define GPIO_PIN_0    ((uint16_t)0x0001)
#define GPIO_PIN_1    ((uint16_t)0x0002)
#define GPIO_PIN_2    ((uint16_t)0x0004)
#define GPIO_PIN_3    ((uint16_t)0x0008)
#define GPIO_PIN_4    ((uint16_t)0x0010)
#define GPIO_PIN_5    ((uint16_t)0x0020)
#define GPIO_PIN_6    ((uint16_t)0x0040)
#define GPIO_PIN_7    ((uint16_t)0x0080)
#define GPIO_PIN_8    ((uint16_t)0x0100)
#define GPIO_PIN_9    ((uint16_t)0x0200)
#define GPIO_PIN_10   ((uint16_t)0x0400)
#define GPIO_PIN_11   ((uint16_t)0x0800)
#define GPIO_PIN_12   ((uint16_t)0x1000)
#define GPIO_PIN_13   ((uint16_t)0x2000)
#define GPIO_PIN_14   ((uint16_t)0x4000)
#define GPIO_PIN_15   ((uint16_t)0x8000)

/* ---- SPI types ---- */
typedef struct {
    uint32_t Mode;
    uint32_t Direction;
    uint32_t DataSize;
    uint32_t CLKPolarity;
    uint32_t CLKPhase;
    uint32_t NSS;
    uint32_t BaudRatePrescaler;
    uint32_t FirstBit;
    uint32_t TIMode;
    uint32_t CRCCalculation;
    uint32_t CRCPolynomial;
    uint32_t NSSPMode;
    uint32_t NSSPolarity;
    uint32_t FifoThreshold;
    uint32_t TxCRCInitializationPattern;
    uint32_t RxCRCInitializationPattern;
    uint32_t MasterSSIdleness;
    uint32_t MasterInterDataIdleness;
    uint32_t MasterReceiverAutoSusp;
    uint32_t MasterKeepIOState;
    uint32_t IOSwap;
} SPI_InitTypeDef;

typedef struct {
    void *Instance;
    SPI_InitTypeDef Init;
    HAL_LockTypeDef Lock;
    volatile uint32_t State;
    volatile uint32_t ErrorCode;
} SPI_HandleTypeDef;

/* ---- I2C types ---- */
typedef struct {
    uint32_t Timing;
    uint32_t OwnAddress1;
    uint32_t AddressingMode;
    uint32_t DualAddressMode;
    uint32_t OwnAddress2;
    uint32_t OwnAddress2Masks;
    uint32_t GeneralCallMode;
    uint32_t NoStretchMode;
} I2C_InitTypeDef;

typedef struct {
    void *Instance;
    I2C_InitTypeDef Init;
    HAL_LockTypeDef Lock;
    volatile uint32_t State;
    volatile uint32_t ErrorCode;
} I2C_HandleTypeDef;

/* ---- ADC types ---- */
typedef struct {
    uint32_t ClockPrescaler;
    uint32_t Resolution;
    uint32_t ScanConvMode;
    uint32_t EOCSelection;
    uint32_t ContinuousConvMode;
    uint32_t NbrOfConversion;
    uint32_t DiscontinuousConvMode;
    uint32_t ExternalTrigConv;
    uint32_t ExternalTrigConvEdge;
    uint32_t ConversionDataManagement;
    uint32_t Overrun;
    uint32_t OversamplingMode;
} ADC_InitTypeDef;

typedef struct {
    void *Instance;
    ADC_InitTypeDef Init;
    HAL_LockTypeDef Lock;
    volatile uint32_t State;
    volatile uint32_t ErrorCode;
} ADC_HandleTypeDef;

typedef struct {
    uint32_t Channel;
    uint32_t Rank;
    uint32_t SamplingTime;
    uint32_t SingleDiff;
    uint32_t OffsetNumber;
    uint32_t Offset;
} ADC_ChannelConfTypeDef;

/* ADC channel defines */
#define ADC_CHANNEL_0   0
#define ADC_CHANNEL_1   1
#define ADC_CHANNEL_2   2
#define ADC_CHANNEL_3   3
#define ADC_CHANNEL_4   4
#define ADC_CHANNEL_5   5
#define ADC_CHANNEL_6   6
#define ADC_CHANNEL_7   7
#define ADC_CHANNEL_8   8
#define ADC_CHANNEL_9   9
#define ADC_CHANNEL_10  10
#define ADC_CHANNEL_11  11

/* ADC config constants */
#define ADC_REGULAR_RANK_1        1
#define ADC_SAMPLETIME_64CYCLES_5 6
#define ADC_SINGLE_ENDED          0
#define ADC_OFFSET_NONE           0
#define ADC_CALIB_OFFSET          0

/* ---- CRC types ---- */
#define DEFAULT_POLYNOMIAL_ENABLE    0
#define DEFAULT_INIT_VALUE_ENABLE    0
#define CRC_INPUTDATA_INVERSION_BYTE 1
#define CRC_OUTPUTDATA_INVERSION_ENABLE 1
#define CRC_INPUTDATA_FORMAT_BYTES   1

typedef struct {
    uint32_t DefaultPolynomialUse;
    uint32_t DefaultInitValueUse;
    uint32_t GeneratingPolynomial;
    uint32_t CRCLength;
    uint32_t InitValue;
    uint32_t InputDataInversionMode;
    uint32_t OutputDataInversionMode;
} CRC_InitTypeDef;

typedef struct {
    void *Instance;
    CRC_InitTypeDef Init;
    HAL_LockTypeDef Lock;
    volatile uint32_t State;
    uint32_t InputDataFormat;
} CRC_HandleTypeDef;

/* CRC instance placeholder */
#define CRC ((void *)0x40023000)

/* ---- Timer types ---- */
typedef struct {
    uint32_t Prescaler;
    uint32_t CounterMode;
    uint32_t Period;
    uint32_t ClockDivision;
    uint32_t RepetitionCounter;
    uint32_t AutoReloadPreload;
} TIM_Base_InitTypeDef;

typedef struct {
    void *Instance;
    TIM_Base_InitTypeDef Init;
    HAL_LockTypeDef Lock;
    volatile uint32_t State;
} TIM_HandleTypeDef;

/* ---- QSPI types ---- */
typedef struct {
    uint32_t ClockPrescaler;
    uint32_t FifoThreshold;
    uint32_t SampleShifting;
    uint32_t FlashSize;
    uint32_t ChipSelectHighTime;
    uint32_t ClockMode;
    uint32_t FlashID;
    uint32_t DualFlash;
} QSPI_InitTypeDef;

typedef struct {
    void *Instance;
    QSPI_InitTypeDef Init;
    HAL_LockTypeDef Lock;
    volatile uint32_t State;
    volatile uint32_t ErrorCode;
} QSPI_HandleTypeDef;

typedef struct {
    uint32_t Instruction;
    uint32_t Address;
    uint32_t AlternateBytes;
    uint32_t AddressSize;
    uint32_t AlternateBytesSize;
    uint32_t DummyCycles;
    uint32_t InstructionMode;
    uint32_t AddressMode;
    uint32_t AlternateByteMode;
    uint32_t DataMode;
    uint32_t NbData;
    uint32_t DdrMode;
    uint32_t DdrHoldHalfCycle;
    uint32_t SIOOMode;
} QSPI_CommandTypeDef;

/* ---- UART types ---- */
typedef struct {
    uint32_t BaudRate;
    uint32_t WordLength;
    uint32_t StopBits;
    uint32_t Parity;
    uint32_t Mode;
    uint32_t HwFlowCtl;
    uint32_t OverSampling;
    uint32_t OneBitSampling;
    uint32_t ClockPrescaler;
} UART_InitTypeDef;

typedef struct {
    void *Instance;
    UART_InitTypeDef Init;
    HAL_LockTypeDef Lock;
    volatile uint32_t State;
    volatile uint32_t ErrorCode;
} UART_HandleTypeDef;

/* ---- FatFs types needed by some drivers ---- */
/* Defined separately if needed */

/* ---- HAL function declarations (implemented in mocks or stubs) ---- */
uint32_t HAL_GetTick(void);
void     HAL_Delay(uint32_t ms);

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData,
                                    uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData,
                                   uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi,
                                           uint8_t *pTxData, uint8_t *pRxData,
                                           uint16_t Size, uint32_t Timeout);

void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                     uint16_t MemAddress, uint16_t MemAddSize,
                                     uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                    uint16_t MemAddress, uint16_t MemAddSize,
                                    uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                           uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                          uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                         uint32_t Trials, uint32_t Timeout);

HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *hadc, uint32_t Timeout);
uint32_t          HAL_ADC_GetValue(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *hadc,
                                         ADC_ChannelConfTypeDef *sConfig);
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *hadc,
                                               uint32_t CalibrationMode,
                                               uint32_t SingleDiff);

HAL_StatusTypeDef HAL_CRC_Init(CRC_HandleTypeDef *hcrc);
uint32_t          HAL_CRC_Calculate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength);
uint32_t          HAL_CRC_Accumulate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength);

HAL_StatusTypeDef HAL_QSPI_Command(QSPI_HandleTypeDef *hqspi, QSPI_CommandTypeDef *cmd, uint32_t Timeout);
HAL_StatusTypeDef HAL_QSPI_Transmit(QSPI_HandleTypeDef *hqspi, uint8_t *pData, uint32_t Timeout);
HAL_StatusTypeDef HAL_QSPI_Receive(QSPI_HandleTypeDef *hqspi, uint8_t *pData, uint32_t Timeout);
HAL_StatusTypeDef HAL_QSPI_AutoPolling(QSPI_HandleTypeDef *hqspi, QSPI_CommandTypeDef *cmd,
                                        void *pCfg, uint32_t Timeout);

/* ---- Clock enable macros (no-ops in test) ---- */
#define __HAL_RCC_CRC_CLK_ENABLE()   ((void)0)
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOD_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOE_CLK_ENABLE() ((void)0)

/* ---- Dummy GPIO port instances ---- */
extern GPIO_TypeDef gpio_stub_a, gpio_stub_b, gpio_stub_c, gpio_stub_d, gpio_stub_e;
#define GPIOA (&gpio_stub_a)
#define GPIOB (&gpio_stub_b)
#define GPIOC (&gpio_stub_c)
#define GPIOD (&gpio_stub_d)
#define GPIOE (&gpio_stub_e)

/* ---- FATFS minimal types (for drivers that reference FIL) ---- */
#ifndef _FATFS
#define _FATFS
typedef struct { int dummy; } FIL;
typedef int FRESULT;
#define FR_OK 0
#define FR_DISK_ERR 1
#define FA_WRITE 0x02
#define FA_CREATE_ALWAYS 0x08
#define FA_OPEN_APPEND 0x30

static inline FRESULT f_open(FIL *fp, const char *path, uint8_t mode) {
    (void)fp; (void)path; (void)mode; return FR_OK;
}
static inline FRESULT f_write(FIL *fp, const void *buff, unsigned btw, unsigned *bw) {
    (void)fp; (void)buff; *bw = btw; return FR_OK;
}
static inline FRESULT f_sync(FIL *fp) { (void)fp; return FR_OK; }
static inline FRESULT f_close(FIL *fp) { (void)fp; return FR_OK; }
static inline FRESULT f_lseek(FIL *fp, unsigned ofs) { (void)fp; (void)ofs; return FR_OK; }
static inline unsigned f_size(FIL *fp) { (void)fp; return 0; }
#endif

#endif /* STM32H7XX_HAL_H */
