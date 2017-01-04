/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL/AP_HAL.h>

#pragma GCC push_options
#pragma GCC optimize ("O0")



#include <AP_HAL/AP_HAL.h>

#include "SPIDevice.h"
#include "GPIO.h"


#include "Semaphores.h"
#include <spi.h>
#include <io.h>

using namespace REVOMINI;

extern const AP_HAL::HAL& hal;

REVOMINI::Semaphore SPIDevice::_semaphores[3]; // per bus

SPIDesc SPIDeviceManager::_device[] = {    // different SPI tables per board subtype
    //          name                device   bus mode       cs_pin                             speed_low      speed_high
    SPIDesc(HAL_INS_MPU60x0_NAME,   _SPI1,   1,  SPI_MODE_3, MPU6000_CS_PIN       /* pa4 */, SPI_562_500KHZ,  SPI_4_5MHZ /*SPI_1_125MHZ*/), // 400ns SCK time
    SPIDesc(HAL_DATAFLASH_NAME,     _SPI3,   3,  SPI_MODE_3, BOARD_SPI3_CS_DF_PIN /* pb3 */, SPI_1_125MHZ,    SPI_18MHZ),
};


#define REVOMINI_SPI_DEVICE_NUM_DEVICES ARRAY_SIZE(SPIDeviceManager::_device)

const uint8_t SPIDeviceManager::_n_device_desc = REVOMINI_SPI_DEVICE_NUM_DEVICES;

AP_HAL::OwnPtr<AP_HAL::SPIDevice>
SPIDeviceManager::get_device(const char *name)
{
    SPIDesc *desc = nullptr;
    
    /* Find the bus description in the table */
    for (uint8_t i = 0; i < _n_device_desc; i++) {
        if (!strcmp(_device[i].name, name)) {
            desc = &_device[i];
            break;
        }
    }
 
    if (!desc) {
        AP_HAL::panic("SPI: invalid device name");
    }


    return AP_HAL::OwnPtr<AP_HAL::SPIDevice>(new SPIDevice(*desc));
}


SPIDevice::SPIDevice(SPIDesc &device_desc)
    : _desc(device_desc)
    , _initialized(false)
{
    _cs = REVOMINIGPIO::get_channel(_desc.cs_pin);
    if (!_cs) {
        AP_HAL::panic("Unable to instantiate cs pin");
    }
        
}        

void SPIDevice::init(){
    _cs->mode(HAL_GPIO_OUTPUT);
    _cs_release();    // do not hold the SPI bus initially

    _speed = _desc.lowspeed;
        
    spi_baud_rate baud = determine_baud_rate(_speed);

//    configure_gpios(_desc.dev, true);
    const spi_pins *pins = dev_to_spi_pins(_desc.dev);

    if (!pins || pins->nss > BOARD_NR_GPIO_PINS || pins->sck > BOARD_NR_GPIO_PINS || pins->mosi > BOARD_NR_GPIO_PINS || pins->miso > BOARD_NR_GPIO_PINS) {
        return;
    }

    //init the device
    spi_init(_desc.dev);

    const stm32_pin_info *nssi  = &PIN_MAP[pins->nss];
    const stm32_pin_info *scki  = &PIN_MAP[pins->sck];
    const stm32_pin_info *misoi = &PIN_MAP[pins->miso];
    const stm32_pin_info *mosii = &PIN_MAP[pins->mosi];

    spi_gpio_cfg(_desc.dev,
		true,
                nssi->gpio_device,
                nssi->gpio_bit,
                scki->gpio_device,
                scki->gpio_bit,
                misoi->gpio_bit,
                mosii->gpio_bit);


    spi_master_enable(_desc.dev, baud, _desc.mode, MSBFIRST);
}

bool SPIDevice::set_speed(AP_HAL::Device::Speed speed)
{

    if(!_initialized) {
        init();
        _initialized=true;
    }

    switch (speed) {
    case AP_HAL::Device::SPEED_HIGH:
        _speed = _desc.highspeed;
        break;
    case AP_HAL::Device::SPEED_LOW:
        _speed = _desc.lowspeed;
        break;
    }

    spi_baud_rate baud = determine_baud_rate(_speed);
    spi_master_enable(_desc.dev, baud, _desc.mode, MSBFIRST);
    
    return true;
}


const spi_pins* SPIDevice::dev_to_spi_pins(const spi_dev *dev) {
    if (     dev->SPIx == SPI1)
        return &board_spi_pins[0];
    else if (dev->SPIx == SPI2)
        return &board_spi_pins[1];
    else if (dev->SPIx == SPI3)
        return &board_spi_pins[2];
    else {
          assert_param(0);
          return NULL;
    }
}



spi_baud_rate SPIDevice::determine_baud_rate(SPIFrequency freq)
{

	spi_baud_rate rate;

	switch(freq) {
	case SPI_18MHZ:
		rate = SPI_BAUD_PCLK_DIV_4;
		break;
	case SPI_9MHZ:
		rate = SPI_BAUD_PCLK_DIV_8;
		break;
	case SPI_4_5MHZ:
		rate = SPI_BAUD_PCLK_DIV_16;
		break;
	case SPI_2_25MHZ:
		rate = SPI_BAUD_PCLK_DIV_32;
		break;
	case SPI_1_125MHZ:
		rate = SPI_BAUD_PCLK_DIV_64;
		break;
	case SPI_562_500KHZ:
		rate = SPI_BAUD_PCLK_DIV_128;
		break;
	case SPI_281_250KHZ:
		rate = SPI_BAUD_PCLK_DIV_256;
		break;
	case SPI_140_625KHZ:
		rate = SPI_BAUD_PCLK_DIV_256;
		break;
	default:
		rate = SPI_BAUD_PCLK_DIV_32;
		break;
	}
	return rate;
}

inline uint8_t SPIDevice::_transfer(uint8_t data) {
    uint8_t buf[1];

    //write 1byte
    spi_tx(_desc.dev, &data, 1);

    //read one byte
    while (!spi_is_rx_nonempty(_desc.dev))
            ;
    buf[0] = (uint8_t)spi_rx_reg(_desc.dev);
    return buf[0];
}


bool SPIDevice::transfer(const uint8_t *send, uint32_t send_len, uint8_t *recv, uint32_t recv_len){
    _cs_assert();
/*
    if (send != NULL && send_len) {
        for (uint16_t i = 0; i < send_len; i++) {
            _transfer(send[i]);
        }    
    } 
    
    
    if(recv !=NULL && recv_len) {
        for (uint16_t i = 0; i < recv_len; i++) {
            recv[i] = _transfer(0);
        }
    }
*/
// spimaster_transfer(const spi_dev *dev, uint8_t *txbuf, uint32_t txcount, uint8_t *rxbuf, uint32_t rxcount)

    int ret = spimaster_transfer(_desc.dev, send, send_len, recv, recv_len);
    
    _cs_release();
    return ret==0;

}


bool SPIDevice::transfer_fullduplex(const uint8_t *send, uint8_t *recv, uint32_t len) {

    _cs_assert();
    if (send != NULL && recv !=NULL && len) {
        for (uint16_t i = 0; i < len; i++) {
            recv[i] = _transfer(send[i]);
        }    
    } 
    _cs_release();
    return true;
}

