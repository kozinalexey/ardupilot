#ifndef _BOARD_STM32V1F4_H_
#define _BOARD_STM32V1F4_H_
/*
 *  add new boards steps
 *  lets name of new board is <new board>
 *  in folder  wirish/boards make directory <new board>
 *  copy to this folder all files from revomini_MP32V1F4
 *  rename files part revomini_MP32V1F4 -change to <new board>
 *  change in <new board>.h  pin definitions. see pin number in PIN_MAP array
 *  example 'D2' = 2 = cpu pin named 'PB12'
 *  in folder /mk file board_REVOMINI.mk copy to <new board> ,
 *  inside this file change constant BOARD  ?= revomini_MP32V1F4 to  BOARD  ?= <new board>
  */

//#define BOARD_PINOUT_REVO 1
#define BOARD_PINOUT_F4BY 2
//#define BOARD_PINOUT BOARD_PINOUT_F4BY

/**
 * @brief Configuration of the Cortex-M4 Processor and Core Peripherals
 */
#define __CM4_REV                 0x0001  /*!< Core revision r0p1                            */
#define __MPU_PRESENT             1       /*!< STM32F4XX provides an MPU                     */
#define __NVIC_PRIO_BITS          4       /*!< STM32F4XX uses 4 Bits for the Priority Levels */
#define __Vendor_SysTickConfig    0       /*!< Set to 1 if different SysTick Config is used  */
#define __FPU_PRESENT             1       /*!< FPU present      */

#define CYCLES_PER_MICROSECOND  168
#define SYSTICK_RELOAD_VAL      (CYCLES_PER_MICROSECOND*1000-1)

#undef  STM32_PCLK1
#undef  STM32_PCLK2
#define STM32_PCLK1   (CYCLES_PER_MICROSECOND*1000000/4)
#define STM32_PCLK2   (CYCLES_PER_MICROSECOND*1000000/2)



//#define BOARD_BUTTON_PIN     254

//#define BOARD_RFM22B_CS_PIN     103 // PA15 CS_RFM22B
//#define BOARD_RFM22B_INT_PIN    26  // PD2

#define BUZZER_PIN              71 //f4by PE5
#define BOARD_NR_USARTS         5  //f4by 5 usart


#define BOARD_USART1_TX_PIN     37 //f4by D37/PB6 serial1
#define BOARD_USART1_RX_PIN     38 //f4by D38/PB7

#define BOARD_USART3_TX_PIN     95 //f4by D95/PD8 serial3
#define BOARD_USART3_RX_PIN     97 //f4by D97/PD9

#define BOARD_USART6_TX_PIN     85 //f4by D85/PD5 serial2
#define BOARD_USART6_RX_PIN     28 //f4by D28/PD6

#define BOARD_USART4_TX_PIN     16 //f4by D16/PC10 serial4 inverted
#define BOARD_USART4_RX_PIN     17 //f4by D17/PC11

#define BOARD_USART5_TX_PIN     18 //f4by D18/PC12 serial5
#define BOARD_USART5_RX_PIN     26 //f4by D26/PD2


//#define BOARD_USART5_RX_PIN     26  // PD2  EXTI_RFM22B / UART5_RX
//#define BOARD_BUTTON_PIN        103 // PA15 CS_RFM22B

#define BOARD_SPEKTRUM_RX_PIN   255
#define BOARD_SPEKTRUM_PWR_PIN  255
#define BOARD_SPEKTRUM_PWR_ON   1
#define BOARD_SPEKTRUM_PWR_OFF  0

#define BOARD_USB_SENSE 23 			//pa9


#define BOARD_NR_SPI            3

#define BOARD_SPI1_NSS_PIN      255
#define BOARD_SPI1_SCK_PIN      52  //f4by D52/PA5
#define BOARD_SPI1_MISO_PIN     53  //f4by D53/PA6
#define BOARD_SPI1_MOSI_PIN     54  //f4by D54/PA7
//#define BOARD_SPI1_CS_BR_PIN    51

#define BOARD_SPI2_NSS_PIN      255
#define BOARD_SPI2_SCK_PIN      255
#define BOARD_SPI2_MISO_PIN     255
#define BOARD_SPI2_MOSI_PIN     255

#define BOARD_SPI3_NSS_PIN      255
#define BOARD_SPI3_MOSI_PIN     5  //f4by D5/PB15
#define BOARD_SPI3_MISO_PIN     4  //f4by D4/PB14
#define BOARD_SPI3_SCK_PIN      3  //f4by D3/PB13

#define BOARD_DATAFLASH_CS_PIN    92 //f4by dataflash cs D92/PE15

#define BOARD_HMC5883_DRDY_PIN  45 //f4byD45/PB1 - but it not used by driver

#define MPU6000_CS_PIN			51  //f4by D51/PA4
#define BOARD_MPU6000_DRDY_PIN	46  //f4by D46/PB0

#define BOARD_SBUS_INVERTER     255
#define BOARD_NR_GPIO_PINS      108





#endif
