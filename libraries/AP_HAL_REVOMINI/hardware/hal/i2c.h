#ifndef _I2C_H
#define _I2C_H

#include <stm32f4xx.h>
#include <hal.h>

#define I2C_100KHz_SPEED                        100000
#define I2C_250KHz_SPEED                        250000
#define I2C_400KHz_SPEED                        400000

/* I2C clock speed configuration (in Hz)
  WARNING:
   Make sure that this define is not already declared in other files (ie.
  stm324xg_eval.h file). It can be used in parallel by other modules. */
#ifndef I2C_SPEED
 #define I2C_SPEED              100000
#endif /* I2C_SPEED */

//#define I2C_SLAVE_ADDRESS7      0xA0
//#define sEE_PAGESIZE            32


/* Maximum Timeout values for flags and events waiting loops. These timeouts are
   not based on accurate values, they just guarantee that the application will
   not remain stuck if the I2C communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */
   
#undef I2C_FLAG_TIMEOUT
#undef I2C_LONG_TIMEOUT
#define I2C_FLAG_TIMEOUT         ((uint32_t)0x4000)
#define I2C_LONG_TIMEOUT         ((uint32_t)(50 * I2C_FLAG_TIMEOUT))


#define sEE_I2C2                          I2C2
#define sEE_I2C1                          I2C1

/*
#define sEE_I2C2_DMA                      DMA1
#define sEE_I2C2_DMA_CHANNEL              DMA_Channel_7
#define sEE_I2C2_DMA_STREAM_TX            DMA1_Stream7
#define sEE_I2C2_DMA_STREAM_RX            DMA1_Stream2
#define sEE_I2C2_DMA_CLK                  RCC_AHB1Periph_DMA1
#define sEE_I2C2_DR_Address               ((uint32_t)0x40005810)

#define sEE_I2C2_DMA_TX_IRQn              DMA1_Stream7_IRQn
#define sEE_I2C2_DMA_RX_IRQn              DMA1_Stream2_IRQn
#define sEE_I2C2_DMA_TX_IRQHandler        DMA1_Stream7_IRQHandler
#define sEE_I2C2_DMA_RX_IRQHandler        DMA1_Stream2_IRQHandler
#define sEE_I2C2_DMA_PREPRIO              0
#define sEE_I2C2_DMA_SUBPRIO              0

#define sEE_I2C1_DMA                      DMA1
#define sEE_I2C1_DMA_CHANNEL              DMA_Channel_1
#define sEE_I2C1_DMA_STREAM_TX            DMA1_Stream6
#define sEE_I2C1_DMA_STREAM_RX            DMA1_Stream5
#define sEE_I2C1_DMA_CLK                  RCC_AHB1Periph_DMA1
#define sEE_I2C1_DR_Address               ((uint32_t)0x40005410)

#define sEE_I2C1_DMA_TX_IRQn              DMA1_Stream6_IRQn
#define sEE_I2C1_DMA_RX_IRQn              DMA1_Stream5_IRQn
#define sEE_I2C1_DMA_TX_IRQHandler        DMA1_Stream6_IRQHandler
#define sEE_I2C1_DMA_RX_IRQHandler        DMA1_Stream5_IRQHandler
#define sEE_I2C1_DMA_PREPRIO              0
#define sEE_I2C1_DMA_SUBPRIO              0

#define sEE2_TX_DMA_FLAG_FEIF             DMA_FLAG_FEIF7
#define sEE2_TX_DMA_FLAG_DMEIF            DMA_FLAG_DMEIF7
#define sEE2_TX_DMA_FLAG_TEIF             DMA_FLAG_TEIF7
#define sEE2_TX_DMA_FLAG_HTIF             DMA_FLAG_HTIF7
#define sEE2_TX_DMA_FLAG_TCIF             DMA_FLAG_TCIF7
#define sEE2_RX_DMA_FLAG_FEIF             DMA_FLAG_FEIF2
#define sEE2_RX_DMA_FLAG_DMEIF            DMA_FLAG_DMEIF2
#define sEE2_RX_DMA_FLAG_TEIF             DMA_FLAG_TEIF2
#define sEE2_RX_DMA_FLAG_HTIF             DMA_FLAG_HTIF2
#define sEE2_RX_DMA_FLAG_TCIF             DMA_FLAG_TCIF2

#define sEE1_TX_DMA_FLAG_FEIF             DMA_FLAG_FEIF6
#define sEE1_TX_DMA_FLAG_DMEIF            DMA_FLAG_DMEIF6
#define sEE1_TX_DMA_FLAG_TEIF             DMA_FLAG_TEIF6
#define sEE1_TX_DMA_FLAG_HTIF             DMA_FLAG_HTIF6
#define sEE1_TX_DMA_FLAG_TCIF             DMA_FLAG_TCIF6
#define sEE1_RX_DMA_FLAG_FEIF             DMA_FLAG_FEIF5
#define sEE1_RX_DMA_FLAG_DMEIF            DMA_FLAG_DMEIF5
#define sEE1_RX_DMA_FLAG_TEIF             DMA_FLAG_TEIF5
#define sEE1_RX_DMA_FLAG_HTIF             DMA_FLAG_HTIF5
#define sEE1_RX_DMA_FLAG_TCIF             DMA_FLAG_TCIF5

#define sEE_DIRECTION_TX                 0
#define sEE_DIRECTION_RX                 1
*/

/* Time constant for the delay caclulation allowing to have a millisecond
   incrementing counter. This value should be equal to (System Clock / 1000).
   ie. if system clock = 168MHz then sEE_TIME_CONST should be 168. */
//#define sEE_TIME_CONST                   168
/**
 * @brief I2C device type.
 */
typedef struct i2c_dev {
    I2C_TypeDef* I2Cx;          
    const gpio_dev *gpio_port;        
    uint8_t sda_pin;             
    uint8_t scl_pin;             
    uint32_t clk;          
    uint8_t gpio_af;     
    IRQn_Type ev_nvic_line;  /* Event IRQ number */
    IRQn_Type er_nvic_line;  /* Error IRQ number */        
} i2c_dev;

#ifdef __cplusplus
  extern "C" {
#endif
 

void i2c_init(const i2c_dev *dev, uint16_t address, uint32_t speed);
void i2c_deinit(const i2c_dev *dev);

uint8_t i2c_is_busy();

uint32_t i2c_write(const i2c_dev *dev, uint8_t addr, const uint8_t *tx_buff, uint8_t *len);
uint32_t i2c_read(const i2c_dev *dev, uint8_t addr, const uint8_t *tx_buff, uint8_t txlen, uint8_t *rx_buff, uint8_t *rxlen);


void i2c_master_release_bus(const i2c_dev *dev);
void i2c_bus_reset(const i2c_dev *dev);

extern const i2c_dev* const _I2C1;
extern const i2c_dev* const _I2C2;

#ifdef __cplusplus
  }
#endif
 

#endif
