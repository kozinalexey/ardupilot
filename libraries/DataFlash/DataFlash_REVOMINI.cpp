/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#include "DataFlash_REVOMINI.h"
#include <wirish.h>

#define ENABLE_FASTSERIAL_DEBUG

#ifdef ENABLE_FASTSERIAL_DEBUG
 #define serialDebug(fmt, args...)  do {hal.console->printf_P(PSTR( __FUNCTION__ ":%d:" fmt "\n"), __LINE__, ##args); } while(0)
#else
 # define serialDebug(fmt, args...)
#endif


//Micron M25P16 Serial Flash Embedded Memory 16 Mb, 3V
#define JEDEC_WRITE_ENABLE           0x06
#define JEDEC_WRITE_DISABLE          0x04
#define JEDEC_READ_STATUS            0x05
#define JEDEC_WRITE_STATUS           0x01
#define JEDEC_READ_DATA              0x03
#define JEDEC_FAST_READ              0x0b
#define JEDEC_DEVICE_ID              0x9F
#define JEDEC_PAGE_WRITE             0x02

#define JEDEC_STATUS_BUSY            0x01
#define JEDEC_STATUS_WRITEPROTECT    0x02
#define JEDEC_STATUS_BP0             0x04
#define JEDEC_STATUS_BP1             0x08
#define JEDEC_STATUS_BP2             0x10
#define JEDEC_STATUS_TP              0x20
#define JEDEC_STATUS_SEC             0x40
#define JEDEC_STATUS_SRP0            0x80

#define expect_memorytype            0x20
#define expect_capacity              0x15
#define sector_erase                 0xD8

uint8_t BlockBuffer[256];




uint8_t DataFlash_REVOMINI::spi_read(void) { uint8_t b;  _spi->transfer(NULL,0, &b, 1); return b; }
void DataFlash_REVOMINI::spi_write(uint8_t b) {  _spi->transfer(&b,1, NULL, 0);  }


bool DataFlash_REVOMINI::cs_assert(){    
    if (!_sem_take(50))
        return false;

    hal.gpio->write(DF_RESET,0); 
    return true; 
}

void DataFlash_REVOMINI::cs_release(){   
    hal.gpio->write(DF_RESET,1); 

    _spi_sem->give();
}

/*
  try to take a semaphore safely from both in a timer and outside
 */
bool DataFlash_REVOMINI::_sem_take(uint8_t timeout)
{
    if (hal.scheduler->in_timerprocess()) {
        return _spi_sem->take_nonblocking();
    }
    return _spi_sem->take(timeout);
}

void DataFlash_REVOMINI::Init(void)
{
    // init to zero
    df_NumPages = 0;

    hal.gpio->pinMode(DF_RESET,OUTPUT);
    // Reset the chip
    hal.gpio->write(DF_RESET,0);
    hal.scheduler->delay(1);
    hal.gpio->write(DF_RESET,1);

    //_spi = std::move(hal.spi->get_device(HAL_DATAFLASH_NAME) );
    _spi = hal.spi->get_device(HAL_DATAFLASH_NAME);

    if (!_spi) {
        AP_HAL::panic("PANIC: DataFlash SPIDeviceDriver not found");
        return; /* never reached */
    }

    _spi_sem = _spi->get_semaphore();
    if (!_spi_sem) {
        AP_HAL::panic("PANIC: DataFlash SPIDeviceDriver semaphore is null");
        return; /* never reached */
    }

    df_PageSize = PageSize();

    // the last page is reserved for config information
    df_NumPages = DF_LAST_PAGE - 1;
}

// This function is mainly to test the device
void DataFlash_REVOMINI::ReadManufacturerID()
{
    // activate dataflash command decoder
    if (!cs_assert()) return;

    // Read manufacturer and ID command...
    spi_write(JEDEC_DEVICE_ID); //

    df_manufacturer = spi_read();
    df_device = spi_read(); //memorytype
    df_device = (df_device << 8) | spi_read(); //capacity
    spi_read();

    // release SPI bus for use by other sensors
    cs_release();
}

// This function return 1 if Card is inserted on SD slot
bool DataFlash_REVOMINI::CardInserted()
{
    return true;
}

// Read the status register
// Assumes _spi_sem handled by caller
uint8_t DataFlash_REVOMINI::ReadStatusReg()
{
    uint8_t tmp;

    // activate dataflash command decoder
    if (!cs_assert()) return JEDEC_STATUS_BUSY;

    // Read status command
    spi_write(JEDEC_READ_STATUS);
    tmp = spi_read(); // We only want to extract the READY/BUSY bit

    // release SPI bus for use by other sensors
    cs_release();

    return tmp;
}

// Read the status of the DataFlash
// Assumes _spi_sem handled by caller.
inline
uint8_t DataFlash_REVOMINI::ReadStatus()
{
  // We only want to extract the READY/BUSY bit
    int32_t status = ReadStatusReg();
    if (status < 0)
	    return -1;
    return status & JEDEC_STATUS_BUSY;
}

inline
uint16_t DataFlash_REVOMINI::PageSize()
{
    return 256;
}

// Wait until DataFlash is in ready state...
// Assumes _spi_sem handled by caller.
void DataFlash_REVOMINI::WaitReady()
{
    while(ReadStatus() != 0);
}

/**
 * @brief Execute the write enable instruction and returns the status
 * @returns 0 if successful, -1 if unable to claim bus
 */
void DataFlash_REVOMINI::Flash_Jedec_WriteEnable(void)
{
    // activate dataflash command decoder
    if (!cs_assert()) return;

    spi_write(JEDEC_WRITE_ENABLE);

    cs_release();
}

void DataFlash_REVOMINI::BufferToPage (uint32_t IntPageAdr)
{
    uint8_t *pData = BlockBuffer;

    uint8_t cmd[4];
    cmd[0] = JEDEC_PAGE_WRITE;
    cmd[1] = (IntPageAdr >> 16) & 0xff;
    cmd[2] = (IntPageAdr >>  8) & 0xff;
    cmd[3] = (IntPageAdr >>  0) & 0xff;

    Flash_Jedec_WriteEnable();

    if (!cs_assert()) return;
    _spi->transfer(cmd, sizeof(cmd),NULL, 0);

    _spi->transfer((uint8_t *)pData, sizeof(BlockBuffer), NULL, 0);

    // release SPI bus for use by other sensors
    cs_release();

    WaitReady();
}

// Write block of data to temporary buffer
void DataFlash_REVOMINI::BlockWrite (uint32_t BufferIdx, const void *pHeader, uint8_t hdr_size, const void *pBuffer, uint16_t size)
{
    uint8_t *pData = BlockBuffer;

    pData += BufferIdx;
    if (hdr_size != 0) {
	memcpy( pData, (const uint8_t *)pHeader, hdr_size);
	pData += hdr_size;
    }
    memcpy( pData, (const uint8_t *)pBuffer, size);
}

bool DataFlash_REVOMINI::BlockRead (uint32_t IntPageAdr, void *pBuffer, uint16_t size)
{
    // activate dataflash command decoder
    if (!cs_assert()) return false;

    uint8_t cmd[4];
    cmd[0] = JEDEC_READ_DATA;
    cmd[1] = (IntPageAdr >> 16) & 0xff;
    cmd[2] = (IntPageAdr >>  8) & 0xff;
    cmd[3] = (IntPageAdr >>  0) & 0xff;

    _spi->transfer(cmd, sizeof(cmd), NULL, 0);

    uint8_t *pData = (uint8_t *)pBuffer;
    while (size--) {
        *pData++ = spi_read();
    }
    // release SPI bus for use by other sensors
    cs_release();

    return true;
}

/**
 * @brief Erase a sector on the flash chip
 * @param[in] chip_offset Sector number of flash to erase
 */

void DataFlash_REVOMINI::Flash_Jedec_EraseSector(uint32_t chip_offset)
{
    uint8_t cmd[4];

    cmd[0] = sector_erase;
    cmd[1] = (chip_offset >> 16) & 0xff;
    cmd[2] = (chip_offset >>  8) & 0xff;
    cmd[3] = (chip_offset >>  0) & 0xff;

    Flash_Jedec_WriteEnable();

    if (!cs_assert()) return;

    _spi->transfer(cmd, sizeof(cmd), NULL, 0);

    cs_release();
}

// *** END OF INTERNAL FUNCTIONS ***

