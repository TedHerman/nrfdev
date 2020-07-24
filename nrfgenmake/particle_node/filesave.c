#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "nrf.h"
#include "boards.h"
#include "bsp.h"
#include "app_scheduler.h"

#include "ff.h"
#include "diskio_blkdev.h"
#include "nrf_block_dev_sdc.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/*********************************************************
 *   See SDK15.3/external/fatfs/doc/en/ for many html    *
 *   files that document fatfs file system calls         *
 *********************************************************/

#define SDC_SCK_PIN  _PINNUM(1,15) // particle
#define SDC_MISO_PIN _PINNUM(1,14) // particle
#define SDC_MOSI_PIN _PINNUM(1,13) // particle
#define SDC_CS_PIN   _PINNUM(0,31) // particle but it's SPI0_SS

extern int16_t node_addr;

// variables used with Nordic's FATFS implementation
NRF_BLOCK_DEV_SDC_DEFINE(
  m_block_dev_sdc,
  NRF_BLOCK_DEV_SDC_CONFIG(
    SDC_SECTOR_SIZE,
    APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)
    ),
  NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
  );
static FATFS fs;
static FIL file;
static diskio_blkdev_t drives[] = \
  { DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), 
    NULL) };

void set_file_name(uint8_t* p) {
  // set a customized file name for this node
  char basename[] = "ANCHR000.DAT";
  memcpy(p,basename,13);
  uint16_t node_suffix = node_addr%10000;  
  p[5] = '0' + (node_suffix / 100);
  node_suffix = node_suffix % 100; 
  p[6] = '0' + (node_suffix / 10);
  node_suffix = node_suffix % 10; 
  p[7] = '0' + node_suffix;
  return;
  }

int data_store(uint8_t* data, uint16_t data_len) {
  // return 0 if file opened and written, else return -1
  uint32_t bytes_written;
  uint8_t file_name[13];
  FRESULT ff_result;
  DSTATUS disk_state = STA_NOINIT;
  // register just assigns variables, should never fail
  diskio_blockdev_register(drives, ARRAY_SIZE(drives));
  // disk_initialize might fail, and has timing difficulties
  for (uint32_t retries = 8; retries && disk_state; --retries) 
    disk_state = disk_initialize(0); 
  if (disk_state) { NRF_LOG_INFO("Disk initialization failed."); return -1; }
  NRF_LOG_INFO("Mounting volume...");
  ff_result = f_mount(&fs, "", 1);
  if (ff_result) { NRF_LOG_INFO("Mount failed."); return -1; }
  set_file_name(file_name);
  ff_result = f_open(&file,(const TCHAR*)file_name, FA_READ | FA_WRITE | FA_OPEN_APPEND);
  if (ff_result != FR_OK) { NRF_LOG_INFO("Unable open file"); return -1; }
  // NRF_LOG_INFO("Preparing to write %d bytes",data_len);
  // NRF_LOG_FLUSH();
  ff_result = f_write(&file, data, data_len, (UINT *)&bytes_written);
  if (ff_result != FR_OK) {
    NRF_LOG_INFO("Write failed");
    f_close(&file);
    return -1;
    }
  f_close(&file);
  f_mount(NULL,"", 0);
  NRF_LOG_INFO("%d bytes saved in file",data_len);
  return 0;
  }
