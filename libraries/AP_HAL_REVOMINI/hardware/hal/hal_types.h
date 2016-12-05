#ifndef _HAL_TYPES_H_
#define _HAL_TYPES_H_

#include <stdint.h>


typedef void (*voidFuncPtr)(void);

#define __attr_flash __attribute__((section (".USER_FLASH")))
#define __packed __attribute__((__packed__))

#ifndef NULL
#define NULL 0
#endif

#endif

