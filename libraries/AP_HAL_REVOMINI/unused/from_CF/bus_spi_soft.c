/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#ifdef USE_SOFTSPI

#include "build/build_config.h"


#include "io.h"
#include "io_impl.h"
#include "bus_spi.h"
#include "bus_spi_soft.h"


void softSpiInit(const softSPIDevice_t *dev)
{
    // SCK as output
    IOInit(IOGetByTag(dev->sckTag),  OWNER_SOFTSPI, RESOURCE_SPI_SCK,  SOFT_SPIDEV_1 + 1);

    IOConfigGPIOAF(IOGetByTag(dev->sckTag), SPI_IO_AF_CFG, 0);

    // MOSI as output
    IOInit(IOGetByTag(dev->mosiTag),  OWNER_SOFTSPI, RESOURCE_SPI_MOSI,  SOFT_SPIDEV_1 + 1);

    IOConfigGPIOAF(IOGetByTag(dev->mosiTag), SPI_IO_AF_CFG, 0);

    // MISO as input
    IOInit(IOGetByTag(dev->misoTag),  OWNER_SOFTSPI, RESOURCE_SPI_MISO,  SOFT_SPIDEV_1 + 1);

    IOConfigGPIOAF(IOGetByTag(dev->misoTag), SPI_IO_AF_CFG, 0);

    // NSS as output
    if (dev->nssTag != IOTAG_NONE) {
        IOInit(IOGetByTag(dev->nssTag),  OWNER_SOFTSPI, RESOURCE_SPI_CS,  SOFT_SPIDEV_1 + 1);

        IOConfigGPIOAF(IOGetByTag(dev->nssTag), SPI_IO_AF_CFG, 0);
    }
}

uint8_t softSpiTransferByte(const softSPIDevice_t *dev, uint8_t byte)
{
    for(int ii = 0; ii < 8; ++ii) {
        if (byte & 0x80) {
            IOHi(IOGetByTag(dev->mosiTag));
        } else {
            IOLo(IOGetByTag(dev->mosiTag));
        }
        IOHi(IOGetByTag(dev->sckTag));
        byte <<= 1;
        if (IORead(IOGetByTag(dev->misoTag)) == 1) {
            byte |= 1;
        }
        IOLo(IOGetByTag(dev->sckTag));
    }
    return byte;
}
#endif
