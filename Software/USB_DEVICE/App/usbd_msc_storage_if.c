/**
 * USB MSC Storage Interface — bridges SCSI commands to W25Q512JV QSPI flash.
 *
 * The MSC view exposes only the FATFS partition (top 4 MB of flash), NOT
 * the whole 64 MB device. This keeps the flight log invisible to the host
 * (so a Windows "format this drive?" prompt cannot wipe flight data) and
 * matches user_diskio.c so the FATFS view and MSC view are identical.
 *
 * MSC sector 0 → physical flash FATFS_FLASH_BASE (0x03C00000)
 * Block size = 4096 bytes, block count = 1024 (4 MB).
 *
 * Flight data extraction uses the CDC dump path, not MSC.
 */

#include "usbd_msc_storage_if.h"
#include "w25q512jv.h"
#include "log_types.h"

/* The flash device struct is defined in main.c */
extern w25q512jv_t flash;

#define STORAGE_BLK_NBR   FATFS_SECTOR_COUNT       /* 1024 sectors = 4 MB */
#define STORAGE_BLK_SIZ   W25Q512JV_SECTOR_SIZE    /* 4096                */

/* SCSI INQUIRY response: 8-byte vendor + 16-byte product + 4-byte revision */
static const int8_t STORAGE_Inquirydata[] = {
  /* LUN 0 */
  0x00,  /* Direct Access Device */
  0x80,  /* Removable */
  0x02,  /* SPC-2 compliance */
  0x02,  /* Response data format */
  36 - 5,/* Additional length */
  0x00, 0x00, 0x00,
  'C', 'A', 'S', 'P', 'E', 'R', ' ', ' ',   /* Vendor (8 chars) */
  'Q', 'S', 'P', 'I', ' ', 'F', 'l', 'a',   /* Product (16 chars) */
  's', 'h', ' ', ' ', ' ', ' ', ' ', ' ',
  '0', '0', '0', '1'                          /* Revision (4 chars) */
};

static int8_t STORAGE_Init(uint8_t lun);
static int8_t STORAGE_GetCapacity(uint8_t lun, uint32_t *block_num,
                                  uint16_t *block_size);
static int8_t STORAGE_IsReady(uint8_t lun);
static int8_t STORAGE_IsWriteProtected(uint8_t lun);
static int8_t STORAGE_Read(uint8_t lun, uint8_t *buf, uint32_t blk_addr,
                           uint16_t blk_len);
static int8_t STORAGE_Write(uint8_t lun, uint8_t *buf, uint32_t blk_addr,
                            uint16_t blk_len);
static int8_t STORAGE_GetMaxLun(void);

USBD_StorageTypeDef USBD_MSC_fops = {
  STORAGE_Init,
  STORAGE_GetCapacity,
  STORAGE_IsReady,
  STORAGE_IsWriteProtected,
  STORAGE_Read,
  STORAGE_Write,
  STORAGE_GetMaxLun,
  (int8_t *)STORAGE_Inquirydata
};

static int8_t STORAGE_Init(uint8_t lun)
{
  (void)lun;
  return flash.initialized ? 0 : -1;
}

static int8_t STORAGE_GetCapacity(uint8_t lun, uint32_t *block_num,
                                  uint16_t *block_size)
{
  (void)lun;
  *block_num = STORAGE_BLK_NBR;
  *block_size = STORAGE_BLK_SIZ;
  return 0;
}

static int8_t STORAGE_IsReady(uint8_t lun)
{
  (void)lun;
  return flash.initialized ? 0 : -1;
}

static int8_t STORAGE_IsWriteProtected(uint8_t lun)
{
  (void)lun;
  return 1;  /* read-only: protect flight data from accidental format */
}

static int8_t STORAGE_Read(uint8_t lun, uint8_t *buf, uint32_t blk_addr,
                           uint16_t blk_len)
{
  (void)lun;
  if (w25q512jv_read(&flash,
                     FATFS_FLASH_BASE + (uint32_t)blk_addr * STORAGE_BLK_SIZ,
                     buf, (uint32_t)blk_len * STORAGE_BLK_SIZ) != W25Q_OK)
    return -1;
  return 0;
}

static int8_t STORAGE_Write(uint8_t lun, uint8_t *buf, uint32_t blk_addr,
                            uint16_t blk_len)
{
  /* Drive is advertised as read-only via STORAGE_IsWriteProtected().
   * Windows still issues WRITE_10 commands during mount / fix-disk anyway.
   * Refuse them here so they don't erase flight data. */
  (void)lun; (void)buf; (void)blk_addr; (void)blk_len;
  return -1;
}

static int8_t STORAGE_GetMaxLun(void)
{
  return 0;
}
