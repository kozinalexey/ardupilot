/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* ************************************************************ */
/* DataFlash_REVOMINI Log library                                 */
/* ************************************************************ */
#ifndef __DATAFLASH_REVOMINI_H__
#define __DATAFLASH_REVOMINI_H__

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/GPIO.h>
#include "DataFlash.h"
#include <AP_HAL_REVOMINI/AP_HAL_REVOMINI.h>
#include "DataFlash_Block.h"

// flash size
#define DF_LAST_PAGE 0x1f00

#define DF_RESET BOARD_DATAFLASH_CS_PIN // RESET (PB3)

extern const AP_HAL::HAL& hal;

using namespace REVOMINI;

class DataFlash_REVOMINI : public DataFlash_Block 
{

private:
    void              WaitReady();
    uint8_t           ReadStatusReg();
    uint16_t          PageSize();
    void              Flash_Jedec_WriteEnable();
    void 	      Flash_Jedec_EraseSector(uint32_t chip_offset);
    void              BufferToPage (uint32_t IntPageAdr);
    void	      BlockWrite(uint32_t BufferIdx, const void *pHeader, uint8_t hdr_size, const void *pBuffer, uint16_t size);
    bool              BlockRead(uint32_t IntPageAdr, void *pBuffer, uint16_t size);
    
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _spi;

//    AP_HAL::SPIDevice *_spi;
    AP_HAL::Semaphore *_spi_sem;

    // take a semaphore safely
    bool	      _sem_take(uint8_t timeout);
    uint16_t df_NumPages;

 // Select device 
    bool cs_assert();

    // Deselect device
    void cs_release();
    
    uint8_t spi_read(void);
    void spi_write(uint8_t data);
    void spi_write(int data) { spi_write((uint8_t)data); }

public:
    void        Init();
    void        ReadManufacturerID();
    bool        CardInserted();
    uint8_t     ReadStatus();
    
};

#endif
