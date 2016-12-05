#ifndef __EEPROM_H
#define __EEPROM_H

#define EEPROM_USES_16BIT_WORDS

#include "wirish.h"
#include "stm32f4xx_flash.h"

#define EEPROM_PAGE_SIZE (uint16_t)0x4000 /* Page size = 16kbyte*/
#define EEPROM_START_ADDRESS 	((uint32_t)(0x8008000))

/* Pages 0 and 1 base and end addresses */
#define EEPROM_PAGE0_BASE		((uint32_t)(EEPROM_START_ADDRESS))
#define EEPROM_PAGE1_BASE		((uint32_t)(EEPROM_START_ADDRESS + EEPROM_PAGE_SIZE))

/* Page status definitions */
#define EEPROM_ERASED			((uint16_t)0xFFFF)	/* PAGE is empty */
#define EEPROM_RECEIVE_DATA		((uint16_t)0xEEEE)	/* PAGE is marked to receive data */
#define EEPROM_VALID_PAGE		((uint16_t)0xAAAA)	/* PAGE containing valid data */

#define ADDRESS_MASK 0x3fff // valid address always below it
#define FLAGS_MASK   0xC000 // if this bits are set then we have partially written slot

/* Page full define */
enum //: uint16_t
{
	EEPROM_OK            = ((uint16_t)0x0000),
	EEPROM_OUT_SIZE      = ((uint16_t)0x0081),
	EEPROM_BAD_ADDRESS   = ((uint16_t)0x0082),
	EEPROM_BAD_FLASH     = ((uint16_t)0x0083),
	EEPROM_NOT_INIT      = ((uint16_t)0x0084),
	EEPROM_NO_VALID_PAGE = ((uint16_t)0x00AB)
};

#define EEPROM_DEFAULT_DATA		0xFFFF


class EEPROMClass
{
public:
	EEPROMClass(void);

	uint16_t init(void);
	uint16_t init(uint32_t, uint32_t, uint32_t);

	uint16_t format(void);

	uint16_t erases(uint16_t *);
	uint16_t read (uint16_t address);
	uint16_t read (uint16_t address, uint16_t *data);
	uint16_t write(uint16_t address, uint16_t data);
	uint16_t count(uint16_t *);
	uint16_t maxcount(void);

	uint32_t PageBase0;
	uint32_t PageBase1;
	uint32_t PageSize;
	uint16_t Status;
private:
	FLASH_Status EE_ErasePage(uint32_t);
        FLASH_Status FLASH_ErasePage(uint32_t Page_Address);

	uint16_t EE_CheckPage(uint32_t, uint16_t);
	uint16_t EE_CheckErasePage(uint32_t, uint16_t);
	uint16_t EE_Format(void);
	uint32_t EE_FindValidPage(void);
	uint16_t EE_GetVariablesCount(uint32_t, uint16_t);
	uint16_t EE_PageTransfer(uint32_t, uint32_t, uint16_t);
	uint16_t EE_VerifyPageFullWriteVariable(uint16_t, uint16_t);
};


extern EEPROMClass EEPROM;

#endif	/* __EEPROM_H */
