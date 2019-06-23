/*
 generated hardware definitions from hwdef.dat - DO NOT EDIT
*/

#pragma once

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

// MCU type (ChibiOS define)
#define STM32F4xx_MCUCONF
#define STM32F407_MCUCONF

#define STM32F407xx

// crystal frequency
#define STM32_HSECLK 8000000U

// UART used for stdout (printf)
// MMC via SPI available, enable POSIX filesystem support
#define USE_POSIX

#define HAL_USE_MMC_SPI TRUE
#define HAL_USE_SDC FALSE
#define HAL_SDCARD_SPI_HOOK TRUE
#define STM32_USB_USE_OTG1                  TRUE
#define HAL_USE_USB TRUE
#define HAL_USE_SERIAL_USB TRUE
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_CHIBIOS_F4BY
#define STM32_PLLM_VALUE 8
#define STM32_VDD 330U
#define STM32_ST_USE_TIMER 5
#define HAL_STORAGE_SIZE 16384
#define STORAGE_FLASH_PAGE 22
#define HAL_WITH_RAMTRON 1
#define HAL_OS_FATFS_IO 1
#define HAL_BOARD_LOG_DIRECTORY "/APM/LOGS"
#define HAL_BOARD_TERRAIN_DIRECTORY "/APM/TERRAIN"
#define HAL_I2C_MAX_CLOCK 100000
#define HAL_PROBE_EXTERNAL_I2C_COMPASSES
#define HAL_HAVE_PIXRACER_LED
#define HAL_GPIO_A_LED_PIN 10
#define HAL_GPIO_B_LED_PIN 11
#define HAL_GPIO_C_LED_PIN 12
#define HAL_GPIO_LED_ON  0
#define HAL_GPIO_LED_OFF 1
#define AP_FEATURE_BRD_PWM_COUNT_PARAM 1
#define BOARD_PWM_COUNT_DEFAULT 12
#define BOARD_FLASH_SIZE 1024
#define CRT1_AREAS_NUMBER 1

// location of loaded firmware
#define FLASH_LOAD_ADDRESS 0x08004000

// memory regions
#define HAL_MEMORY_REGIONS {(void*)0x20000000, 0x00020000, 0x01 }, {(void*)0x10000000, 0x00010000, 0x02 }

// CPU serial number (12 bytes)
#define UDID_START 0x1fff7a10


// APJ board ID (for bootloaders)
#define APJ_BOARD_ID 20
// USB configuration
#define HAL_USB_VENDOR_ID 0x27AC
#define HAL_USB_PRODUCT_ID 0x0201
#define HAL_USB_STRING_MANUFACTURER "Swift-Flyer"
#define HAL_USB_STRING_PRODUCT "%BOARD%"
#define HAL_USB_STRING_SERIAL "%SERIAL%"


#define HAL_SPI1_CONFIG { &SPID1, 1, STM32_SPI_SPI1_DMA_STREAMS }
#define HAL_SPI2_CONFIG { &SPID2, 2, STM32_SPI_SPI2_DMA_STREAMS }
#define HAL_SPI_BUS_LIST HAL_SPI1_CONFIG,HAL_SPI2_CONFIG


// SPI device table
#define HAL_SPI_DEVICE0  SPIDesc("mpu6000"        ,  0,  1, PAL_LINE(GPIOA,4U) , SPIDEV_MODE3,   1*MHZ,   8*MHZ)
#define HAL_SPI_DEVICE1  SPIDesc("ramtron"        ,  1,  2, PAL_LINE(GPIOB,12U), SPIDEV_MODE3,   8*MHZ,   8*MHZ)
#define HAL_SPI_DEVICE2  SPIDesc("sdcard"         ,  1,  3, PAL_LINE(GPIOE,15U), SPIDEV_MODE0, 400*KHZ,  25*MHZ)
#define HAL_SPI_DEVICE_LIST HAL_SPI_DEVICE0,HAL_SPI_DEVICE1,HAL_SPI_DEVICE2

// ADC config
#define ANALOG_VCC_5V_PIN 14
#define HAL_HAVE_BOARD_VOLTAGE 1
#define FMU_SERVORAIL_ADC_CHAN 15
#define HAL_HAVE_SERVO_VOLTAGE 1
#define HAL_ANALOG_PINS { \
{ 10,    3.30/4096 }, /* PC0 PRESSURE_SENS */ \
{ 11,    3.30/4096 }, /* PC1 RSSI_IN */ \
{ 12,  2*3.30/4096 }, /* PC2 BATT_CURRENT_SENS */ \
{ 13,  2*3.30/4096 }, /* PC3 BATT_VOLTAGE_SENS */ \
{ 14,  2*3.30/4096 }, /* PC4 VDD_5V_SENS */ \
{ 15,  2*3.30/4096 }, /* PC5 FMU_SERVORAIL_VCC_SENS */ \
}

// GPIO config
#define HAL_GPIO_LINE_GPIO10 PAL_LINE(GPIOE,  3U)
#define HAL_GPIO_LINE_GPIO11 PAL_LINE(GPIOE,  2U)
#define HAL_GPIO_LINE_GPIO12 PAL_LINE(GPIOE,  1U)
#define HAL_GPIO_LINE_GPIO13 PAL_LINE(GPIOE,  0U)
#define HAL_GPIO_LINE_GPIO50 PAL_LINE(GPIOA,  0U)
#define HAL_GPIO_LINE_GPIO51 PAL_LINE(GPIOA,  1U)
#define HAL_GPIO_LINE_GPIO52 PAL_LINE(GPIOA,  2U)
#define HAL_GPIO_LINE_GPIO53 PAL_LINE(GPIOA,  3U)
#define HAL_GPIO_LINE_GPIO54 PAL_LINE(GPIOE,  9U)
#define HAL_GPIO_LINE_GPIO55 PAL_LINE(GPIOE, 11U)
#define HAL_GPIO_LINE_GPIO56 PAL_LINE(GPIOE, 13U)
#define HAL_GPIO_LINE_GPIO57 PAL_LINE(GPIOE, 14U)
#define HAL_GPIO_LINE_GPIO58 PAL_LINE(GPIOD, 13U)
#define HAL_GPIO_LINE_GPIO59 PAL_LINE(GPIOD, 12U)
#define HAL_GPIO_LINE_GPIO60 PAL_LINE(GPIOD, 15U)
#define HAL_GPIO_LINE_GPIO61 PAL_LINE(GPIOD, 14U)
#define HAL_GPIO_PINS { \
{  10, true,  0, PAL_LINE(GPIOE,  3U)}, /* PE3 LED_RED OUTPUT */ \
{  11, true,  0, PAL_LINE(GPIOE,  2U)}, /* PE2 LED_GREEN OUTPUT */ \
{  12, true,  0, PAL_LINE(GPIOE,  1U)}, /* PE1 LED_BLUE OUTPUT */ \
{  13, true,  0, PAL_LINE(GPIOE,  0U)}, /* PE0 LED_YELOW OUTPUT */ \
{  50, true,  1, PAL_LINE(GPIOA,  0U)}, /* PA0 TIM2_CH1 TIM2 AF1 PWM1 */ \
{  51, true,  2, PAL_LINE(GPIOA,  1U)}, /* PA1 TIM2_CH2 TIM2 AF1 PWM2 */ \
{  52, true,  3, PAL_LINE(GPIOA,  2U)}, /* PA2 TIM2_CH3 TIM2 AF1 PWM3 */ \
{  53, true,  4, PAL_LINE(GPIOA,  3U)}, /* PA3 TIM2_CH4 TIM2 AF1 PWM4 */ \
{  54, true,  5, PAL_LINE(GPIOE,  9U)}, /* PE9 TIM1_CH1 TIM1 AF1 PWM5 */ \
{  55, true,  6, PAL_LINE(GPIOE, 11U)}, /* PE11 TIM1_CH2 TIM1 AF1 PWM6 */ \
{  56, true,  7, PAL_LINE(GPIOE, 13U)}, /* PE13 TIM1_CH3 TIM1 AF1 PWM7 */ \
{  57, true,  8, PAL_LINE(GPIOE, 14U)}, /* PE14 TIM1_CH4 TIM1 AF1 PWM8 */ \
{  58, true,  9, PAL_LINE(GPIOD, 13U)}, /* PD13 TIM4_CH2 TIM4 AF2 PWM9 */ \
{  59, true, 10, PAL_LINE(GPIOD, 12U)}, /* PD12 TIM4_CH1 TIM4 AF2 PWM10 */ \
{  60, true, 11, PAL_LINE(GPIOD, 15U)}, /* PD15 TIM4_CH4 TIM4 AF2 PWM11 */ \
{  61, true, 12, PAL_LINE(GPIOD, 14U)}, /* PD14 TIM4_CH3 TIM4 AF2 PWM12 */ \
}

// full pin define list
#define HAL_GPIO_PIN_BATT_CURRENT_SENS    PAL_LINE(GPIOC,2U)
#define HAL_GPIO_PIN_BATT_VOLTAGE_SENS    PAL_LINE(GPIOC,3U)
#define HAL_GPIO_PIN_DRDY_HMC5883         PAL_LINE(GPIOB,1U)
#define HAL_GPIO_PIN_EXTERN_GPIO1         PAL_LINE(GPIOC,14U)
#define HAL_GPIO_PIN_EXTERN_GPIO2         PAL_LINE(GPIOC,13U)
#define HAL_GPIO_PIN_EXTERN_GPIO3         PAL_LINE(GPIOE,4U)
#define HAL_GPIO_PIN_EXTERN_GPIO4         PAL_LINE(GPIOE,6U)
#define HAL_GPIO_PIN_EXTERN_GPIO5         PAL_LINE(GPIOC,9U)
#define HAL_GPIO_PIN_EXTI_MPU6000         PAL_LINE(GPIOB,0U)
#define HAL_GPIO_PIN_FLASH_CS             PAL_LINE(GPIOE,15U)
#define HAL_GPIO_PIN_FMU_SERVORAIL_VCC_SENS PAL_LINE(GPIOC,5U)
#define HAL_GPIO_PIN_FRAM_CS              PAL_LINE(GPIOB,12U)
#define HAL_GPIO_PIN_I2C1_SCL             PAL_LINE(GPIOB,8U)
#define HAL_GPIO_PIN_I2C1_SDA             PAL_LINE(GPIOB,9U)
#define HAL_GPIO_PIN_I2C1_SCL             PAL_LINE(GPIOB,8U)
#define HAL_GPIO_PIN_I2C2_SCL             PAL_LINE(GPIOB,10U)
#define HAL_GPIO_PIN_I2C2_SDA             PAL_LINE(GPIOB,11U)
#define HAL_GPIO_PIN_I2C2_SCL             PAL_LINE(GPIOB,10U)
#define HAL_GPIO_PIN_JTCK_SWCLK           PAL_LINE(GPIOA,14U)
#define HAL_GPIO_PIN_JTMS_SWDIO           PAL_LINE(GPIOA,13U)
#define HAL_GPIO_PIN_LED_BLUE             PAL_LINE(GPIOE,1U)
#define HAL_GPIO_PIN_LED_GREEN            PAL_LINE(GPIOE,2U)
#define HAL_GPIO_PIN_LED_RED              PAL_LINE(GPIOE,3U)
#define HAL_GPIO_PIN_LED_YELOW            PAL_LINE(GPIOE,0U)
#define HAL_GPIO_PIN_MPU_CS               PAL_LINE(GPIOA,4U)
#define HAL_GPIO_PIN_OTG_FS_DM            PAL_LINE(GPIOA,11U)
#define HAL_GPIO_PIN_OTG_FS_DP            PAL_LINE(GPIOA,12U)
#define HAL_GPIO_PIN_PRESSURE_SENS        PAL_LINE(GPIOC,0U)
#define HAL_GPIO_PIN_RSSI_IN              PAL_LINE(GPIOC,1U)
#define HAL_GPIO_PIN_SPI1_MISO            PAL_LINE(GPIOA,6U)
#define HAL_GPIO_PIN_SPI1_MOSI            PAL_LINE(GPIOA,7U)
#define HAL_GPIO_PIN_SPI1_SCK             PAL_LINE(GPIOA,5U)
#define HAL_GPIO_PIN_SPI2_MISO            PAL_LINE(GPIOB,14U)
#define HAL_GPIO_PIN_SPI2_MOSI            PAL_LINE(GPIOB,15U)
#define HAL_GPIO_PIN_SPI2_SCK             PAL_LINE(GPIOB,13U)
#define HAL_GPIO_PIN_TIM1_CH1             PAL_LINE(GPIOE,9U)
#define HAL_GPIO_PIN_TIM1_CH2             PAL_LINE(GPIOE,11U)
#define HAL_GPIO_PIN_TIM1_CH3             PAL_LINE(GPIOE,13U)
#define HAL_GPIO_PIN_TIM1_CH4             PAL_LINE(GPIOE,14U)
#define HAL_GPIO_PIN_TIM2_CH1             PAL_LINE(GPIOA,0U)
#define HAL_GPIO_PIN_TIM2_CH2             PAL_LINE(GPIOA,1U)
#define HAL_GPIO_PIN_TIM2_CH3             PAL_LINE(GPIOA,2U)
#define HAL_GPIO_PIN_TIM2_CH4             PAL_LINE(GPIOA,3U)
#define HAL_GPIO_PIN_TIM4_CH1             PAL_LINE(GPIOD,12U)
#define HAL_GPIO_PIN_TIM4_CH2             PAL_LINE(GPIOD,13U)
#define HAL_GPIO_PIN_TIM4_CH3             PAL_LINE(GPIOD,14U)
#define HAL_GPIO_PIN_TIM4_CH4             PAL_LINE(GPIOD,15U)
#define HAL_GPIO_PIN_TIM8_CH2             PAL_LINE(GPIOC,7U)
#define HAL_GPIO_PIN_TIM9_CH1             PAL_LINE(GPIOE,5U)
#define HAL_GPIO_PIN_UART4_RX             PAL_LINE(GPIOC,11U)
#define HAL_GPIO_PIN_UART4_TX             PAL_LINE(GPIOC,10U)
#define HAL_GPIO_PIN_UART5_RX             PAL_LINE(GPIOD,2U)
#define HAL_GPIO_PIN_UART5_TX             PAL_LINE(GPIOC,12U)
#define HAL_GPIO_PIN_USART1_RX            PAL_LINE(GPIOB,7U)
#define HAL_GPIO_PIN_USART1_TX            PAL_LINE(GPIOB,6U)
#define HAL_GPIO_PIN_USART2_RX            PAL_LINE(GPIOD,6U)
#define HAL_GPIO_PIN_USART2_TX            PAL_LINE(GPIOD,5U)
#define HAL_GPIO_PIN_USART3_RX            PAL_LINE(GPIOD,9U)
#define HAL_GPIO_PIN_USART3_TX            PAL_LINE(GPIOD,8U)
#define HAL_GPIO_PIN_VBUS                 PAL_LINE(GPIOA,9U)
#define HAL_GPIO_PIN_VDD_5V_SENS          PAL_LINE(GPIOC,4U)

#define HAL_INS_PROBE1  ADD_BACKEND(AP_InertialSensor_Invensense::probe(*this,hal.spi->get_device("mpu6000"),ROTATION_NONE))
#define HAL_INS_PROBE_LIST HAL_INS_PROBE1

#define HAL_MAG_PROBE1  ADD_BACKEND(DRIVER_HMC5843, AP_Compass_HMC5843::probe(hal.i2c_mgr->get_device(0,0x1e),false,ROTATION_YAW_270))
#define HAL_MAG_PROBE_LIST HAL_MAG_PROBE1

#define HAL_BARO_PROBE1  ADD_BACKEND(AP_Baro_MS56XX::probe(*this,std::move(hal.i2c_mgr->get_device(0,0x76))))
#define HAL_BARO_PROBE2  ADD_BACKEND(AP_Baro_MS56XX::probe(*this,std::move(hal.i2c_mgr->get_device(0,0x77))))
#define HAL_BARO_PROBE_LIST HAL_BARO_PROBE1;HAL_BARO_PROBE2

// peripherals enabled
#define STM32_I2C_USE_I2C1                  TRUE
#define STM32_I2C_USE_I2C2                  TRUE
#define STM32_USB_USE_OTG1                  TRUE
#define STM32_SPI_USE_SPI1                  TRUE
#define STM32_SPI_USE_SPI2                  TRUE
#ifndef STM32_SERIAL_USE_UART4 
#define STM32_SERIAL_USE_UART4  TRUE
#endif
#ifndef STM32_SERIAL_USE_UART5 
#define STM32_SERIAL_USE_UART5  TRUE
#endif
#ifndef STM32_SERIAL_USE_USART1
#define STM32_SERIAL_USE_USART1 TRUE
#endif
#ifndef STM32_SERIAL_USE_USART2
#define STM32_SERIAL_USE_USART2 TRUE
#endif
#ifndef STM32_SERIAL_USE_USART3
#define STM32_SERIAL_USE_USART3 TRUE
#endif


// auto-generated DMA mapping from dma_resolver.py

// Note: The following peripherals can't be resolved for DMA: ['USART1_RX']

#define STM32_ADC_ADC1_DMA_STREAM      STM32_DMA_STREAM_ID(2, 4)
#define STM32_ADC_ADC1_DMA_CHAN        0
#define STM32_I2C_I2C1_RX_DMA_STREAM   STM32_DMA_STREAM_ID(1, 0)
#define STM32_I2C_I2C1_RX_DMA_CHAN     1
#define STM32_I2C_I2C1_TX_DMA_STREAM   STM32_DMA_STREAM_ID(1, 7) // shared TIM2_UP,I2C1_TX,I2C2_TX
#define STM32_I2C_I2C1_TX_DMA_CHAN     1
#define STM32_I2C_I2C2_RX_DMA_STREAM   STM32_DMA_STREAM_ID(1, 3) // shared SPI2_RX,I2C2_RX
#define STM32_I2C_I2C2_RX_DMA_CHAN     7
#define STM32_I2C_I2C2_TX_DMA_STREAM   STM32_DMA_STREAM_ID(1, 7) // shared TIM2_UP,I2C1_TX,I2C2_TX
#define STM32_I2C_I2C2_TX_DMA_CHAN     7
#define STM32_SPI_SPI1_RX_DMA_STREAM   STM32_DMA_STREAM_ID(2, 0)
#define STM32_SPI_SPI1_RX_DMA_CHAN     3
#define STM32_SPI_SPI1_TX_DMA_STREAM   STM32_DMA_STREAM_ID(2, 3)
#define STM32_SPI_SPI1_TX_DMA_CHAN     3
#define STM32_SPI_SPI2_RX_DMA_STREAM   STM32_DMA_STREAM_ID(1, 3) // shared SPI2_RX,I2C2_RX
#define STM32_SPI_SPI2_RX_DMA_CHAN     0
#define STM32_SPI_SPI2_TX_DMA_STREAM   STM32_DMA_STREAM_ID(1, 4)
#define STM32_SPI_SPI2_TX_DMA_CHAN     0
#define STM32_TIM_TIM1_UP_DMA_STREAM   STM32_DMA_STREAM_ID(2, 5)
#define STM32_TIM_TIM1_UP_DMA_CHAN     6
#define STM32_TIM_TIM2_UP_DMA_STREAM   STM32_DMA_STREAM_ID(1, 7) // shared TIM2_UP,I2C1_TX,I2C2_TX
#define STM32_TIM_TIM2_UP_DMA_CHAN     3
#define STM32_TIM_TIM4_UP_DMA_STREAM   STM32_DMA_STREAM_ID(1, 6)
#define STM32_TIM_TIM4_UP_DMA_CHAN     2
#define STM32_TIM_TIM8_CH2_DMA_STREAM  STM32_DMA_STREAM_ID(2, 2)
#define STM32_TIM_TIM8_CH2_DMA_CHAN    0
#define STM32_UART_UART4_RX_DMA_STREAM STM32_DMA_STREAM_ID(1, 2)
#define STM32_UART_UART4_RX_DMA_CHAN   4
#define STM32_UART_USART1_TX_DMA_STREAM STM32_DMA_STREAM_ID(2, 7)
#define STM32_UART_USART1_TX_DMA_CHAN  4
#define STM32_UART_USART2_RX_DMA_STREAM STM32_DMA_STREAM_ID(1, 5)
#define STM32_UART_USART2_RX_DMA_CHAN  4
#define STM32_UART_USART3_RX_DMA_STREAM STM32_DMA_STREAM_ID(1, 1)
#define STM32_UART_USART3_RX_DMA_CHAN  4


// generated UART DMA configuration lines
#define STM32_USART1_RX_DMA_CONFIG false, 0, 0
#define STM32_USART1_TX_DMA_CONFIG true, STM32_UART_USART1_TX_DMA_STREAM, STM32_UART_USART1_TX_DMA_CHAN
#define STM32_USART2_RX_DMA_CONFIG true, STM32_UART_USART2_RX_DMA_STREAM, STM32_UART_USART2_RX_DMA_CHAN
#define STM32_USART2_TX_DMA_CONFIG false, 0, 0
#define STM32_USART3_RX_DMA_CONFIG true, STM32_UART_USART3_RX_DMA_STREAM, STM32_UART_USART3_RX_DMA_CHAN
#define STM32_USART3_TX_DMA_CONFIG false, 0, 0
#define STM32_UART4_RX_DMA_CONFIG true, STM32_UART_UART4_RX_DMA_STREAM, STM32_UART_UART4_RX_DMA_CHAN
#define STM32_UART4_TX_DMA_CONFIG false, 0, 0
#define STM32_UART5_RX_DMA_CONFIG false, 0, 0
#define STM32_UART5_TX_DMA_CONFIG false, 0, 0


// generated SPI DMA configuration lines
#define STM32_SPI_SPI1_DMA_STREAMS STM32_SPI_SPI1_TX_DMA_STREAM, STM32_SPI_SPI1_RX_DMA_STREAM
#define STM32_SPI_SPI2_DMA_STREAMS STM32_SPI_SPI2_TX_DMA_STREAM, STM32_SPI_SPI2_RX_DMA_STREAM
// RC input config
#define HAL_USE_ICU TRUE
#define STM32_ICU_USE_TIM8 TRUE
#define RCIN_ICU_TIMER ICUD8
#define RCIN_ICU_CHANNEL ICU_CHANNEL_2
#define STM32_RCIN_DMA_STREAM STM32_TIM_TIM8_CH2_DMA_STREAM
#define STM32_RCIN_DMA_CHANNEL STM32_TIM_TIM8_CH2_DMA_CHAN


// Alarm PWM output config
#define STM32_PWM_USE_TIM9 TRUE
#define STM32_TIM9_SUPPRESS_ISR
#define HAL_PWM_ALARM \
        { /* pwmGroup */ \
          0,  /* Timer channel */ \
          { /* PWMConfig */ \
            1000000,    /* PWM clock frequency. */ \
            1000,    /* Initial PWM period 20ms. */ \
            NULL,  /* no callback */ \
            { /* Channel Config */ \
             {PWM_OUTPUT_ACTIVE_HIGH, NULL}, \
             {PWM_OUTPUT_DISABLED, NULL}, \
             {PWM_OUTPUT_DISABLED, NULL}, \
             {PWM_OUTPUT_DISABLED, NULL}  \
            }, \
            0, 0 \
          }, \
          &PWMD9 /* PWMDriver* */ \
        }

// PWM timer config
#define STM32_PWM_USE_TIM1 TRUE
#define STM32_TIM1_SUPPRESS_ISR
#define STM32_PWM_USE_TIM2 TRUE
#define STM32_TIM2_SUPPRESS_ISR
#define STM32_PWM_USE_TIM4 TRUE
#define STM32_TIM4_SUPPRESS_ISR

// PWM output config
#if defined(STM32_TIM_TIM1_UP_DMA_STREAM) && defined(STM32_TIM_TIM1_UP_DMA_CHAN)
# define HAL_PWM1_DMA_CONFIG true, STM32_TIM_TIM1_UP_DMA_STREAM, STM32_TIM_TIM1_UP_DMA_CHAN
#else
# define HAL_PWM1_DMA_CONFIG false, 0, 0
#endif
#define HAL_PWM_GROUP1 { true, \
        {4, 5, 6, 7}, \
        /* Group Initial Config */ \
        { \
          1000000,  /* PWM clock frequency. */ \
          20000,   /* Initial PWM period 20ms. */ \
          NULL,     /* no callback */ \
          { \
           /* Channel Config */ \
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}, \
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}, \
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}, \
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}  \
          }, 0, 0}, &PWMD1, \
          HAL_PWM1_DMA_CONFIG, \
          { 1, 1, 1, 1 }, \
          { PAL_LINE(GPIOE, 9U), PAL_LINE(GPIOE, 11U), PAL_LINE(GPIOE, 13U), PAL_LINE(GPIOE, 14U) }}
#if defined(STM32_TIM_TIM2_UP_DMA_STREAM) && defined(STM32_TIM_TIM2_UP_DMA_CHAN)
# define HAL_PWM2_DMA_CONFIG true, STM32_TIM_TIM2_UP_DMA_STREAM, STM32_TIM_TIM2_UP_DMA_CHAN
#else
# define HAL_PWM2_DMA_CONFIG false, 0, 0
#endif
#define HAL_PWM_GROUP2 { false, \
        {0, 1, 2, 3}, \
        /* Group Initial Config */ \
        { \
          1000000,  /* PWM clock frequency. */ \
          20000,   /* Initial PWM period 20ms. */ \
          NULL,     /* no callback */ \
          { \
           /* Channel Config */ \
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}, \
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}, \
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}, \
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}  \
          }, 0, 0}, &PWMD2, \
          HAL_PWM2_DMA_CONFIG, \
          { 1, 1, 1, 1 }, \
          { PAL_LINE(GPIOA, 0U), PAL_LINE(GPIOA, 1U), PAL_LINE(GPIOA, 2U), PAL_LINE(GPIOA, 3U) }}
#if defined(STM32_TIM_TIM4_UP_DMA_STREAM) && defined(STM32_TIM_TIM4_UP_DMA_CHAN)
# define HAL_PWM4_DMA_CONFIG true, STM32_TIM_TIM4_UP_DMA_STREAM, STM32_TIM_TIM4_UP_DMA_CHAN
#else
# define HAL_PWM4_DMA_CONFIG false, 0, 0
#endif
#define HAL_PWM_GROUP3 { false, \
        {9, 8, 11, 10}, \
        /* Group Initial Config */ \
        { \
          1000000,  /* PWM clock frequency. */ \
          20000,   /* Initial PWM period 20ms. */ \
          NULL,     /* no callback */ \
          { \
           /* Channel Config */ \
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}, \
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}, \
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}, \
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}  \
          }, 0, 0}, &PWMD4, \
          HAL_PWM4_DMA_CONFIG, \
          { 2, 2, 2, 2 }, \
          { PAL_LINE(GPIOD, 12U), PAL_LINE(GPIOD, 13U), PAL_LINE(GPIOD, 14U), PAL_LINE(GPIOD, 15U) }}
#define HAL_PWM_GROUPS HAL_PWM_GROUP1,HAL_PWM_GROUP2,HAL_PWM_GROUP3

// I2C configuration

#if defined(STM32_I2C_I2C2_RX_DMA_STREAM) && defined(STM32_I2C_I2C2_TX_DMA_STREAM)
#define HAL_I2C2_CONFIG { &I2CD2, STM32_I2C_I2C2_RX_DMA_STREAM, STM32_I2C_I2C2_TX_DMA_STREAM, HAL_GPIO_PIN_I2C2_SCL, HAL_GPIO_PIN_I2C2_SDA }
#else
#define HAL_I2C2_CONFIG { &I2CD2, SHARED_DMA_NONE, SHARED_DMA_NONE, HAL_GPIO_PIN_I2C2_SCL, HAL_GPIO_PIN_I2C2_SDA }
#endif

#if defined(STM32_I2C_I2C1_RX_DMA_STREAM) && defined(STM32_I2C_I2C1_TX_DMA_STREAM)
#define HAL_I2C1_CONFIG { &I2CD1, STM32_I2C_I2C1_RX_DMA_STREAM, STM32_I2C_I2C1_TX_DMA_STREAM, HAL_GPIO_PIN_I2C1_SCL, HAL_GPIO_PIN_I2C1_SDA }
#else
#define HAL_I2C1_CONFIG { &I2CD1, SHARED_DMA_NONE, SHARED_DMA_NONE, HAL_GPIO_PIN_I2C1_SCL, HAL_GPIO_PIN_I2C1_SDA }
#endif

#define HAL_I2C_DEVICE_LIST HAL_I2C2_CONFIG,HAL_I2C1_CONFIG


// UART configuration
#define HAL_UARTA_DRIVER ChibiOS::UARTDriver uartADriver(0)
#define HAL_UARTB_DRIVER ChibiOS::UARTDriver uartBDriver(1)
#define HAL_UARTC_DRIVER ChibiOS::UARTDriver uartCDriver(2)
#define HAL_UARTD_DRIVER ChibiOS::UARTDriver uartDDriver(3)
#define HAL_UARTE_DRIVER ChibiOS::UARTDriver uartEDriver(4)
#define HAL_UARTF_DRIVER Empty::UARTDriver uartFDriver
#define HAL_UARTG_DRIVER Empty::UARTDriver uartGDriver
#define HAL_WITH_IO_MCU 0

#define HAL_OTG1_CONFIG {(BaseSequentialStream*) &SDU1, true, false, 0, 0, false, 0, 0}
#define HAL_USART3_CONFIG { (BaseSequentialStream*) &SD3, false, STM32_USART3_RX_DMA_CONFIG, STM32_USART3_TX_DMA_CONFIG, 0, -1, 0, -1, 0}
#define HAL_USART2_CONFIG { (BaseSequentialStream*) &SD2, false, STM32_USART2_RX_DMA_CONFIG, STM32_USART2_TX_DMA_CONFIG, 0, -1, 0, -1, 0}
#define HAL_USART1_CONFIG { (BaseSequentialStream*) &SD1, false, STM32_USART1_RX_DMA_CONFIG, STM32_USART1_TX_DMA_CONFIG, 0, -1, 0, -1, 0}
#define HAL_UART5_CONFIG { (BaseSequentialStream*) &SD5, false, STM32_UART5_RX_DMA_CONFIG, STM32_UART5_TX_DMA_CONFIG, 0, -1, 0, -1, 0}
#define HAL_UART_DEVICE_LIST HAL_OTG1_CONFIG,HAL_USART3_CONFIG,HAL_USART2_CONFIG,HAL_USART1_CONFIG,HAL_UART5_CONFIG

#define HAL_HAVE_AP_ROMFS_EMBEDDED_H 1

/*
* I/O ports initial setup, this configuration is established soon after reset
* in the initialization code.
* Please refer to the STM32 Reference Manual for details.
*/
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

/* PORTA:
 PA0 TIM2_CH1 TIM2 AF1 PWM1
 PA1 TIM2_CH2 TIM2 AF1 PWM2
 PA2 TIM2_CH3 TIM2 AF1 PWM3
 PA3 TIM2_CH4 TIM2 AF1 PWM4
 PA4 MPU_CS CS
 PA5 SPI1_SCK SPI1 AF5
 PA6 SPI1_MISO SPI1 AF5
 PA7 SPI1_MOSI SPI1 AF5
 PA9 VBUS INPUT
 PA11 OTG_FS_DM OTG1 AF10
 PA12 OTG_FS_DP OTG1 AF10
 PA13 JTMS-SWDIO SWD AF0
 PA14 JTCK-SWCLK SWD AF0
*/

#define VAL_GPIOA_MODER   (PIN_MODE_ALTERNATE(0U) | \
                           PIN_MODE_ALTERNATE(1U) | \
                           PIN_MODE_ALTERNATE(2U) | \
                           PIN_MODE_ALTERNATE(3U) | \
                           PIN_MODE_OUTPUT(4U) | \
                           PIN_MODE_ALTERNATE(5U) | \
                           PIN_MODE_ALTERNATE(6U) | \
                           PIN_MODE_ALTERNATE(7U) | \
                           PIN_MODE_INPUT(8U) | \
                           PIN_MODE_INPUT(9U) | \
                           PIN_MODE_INPUT(10U) | \
                           PIN_MODE_ALTERNATE(11U) | \
                           PIN_MODE_ALTERNATE(12U) | \
                           PIN_MODE_ALTERNATE(13U) | \
                           PIN_MODE_ALTERNATE(14U) | \
                           PIN_MODE_INPUT(15U))

#define VAL_GPIOA_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_OPENDRAIN(9U) | \
                           PIN_OTYPE_PUSHPULL(10U) | \
                           PIN_OTYPE_PUSHPULL(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOA_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOA_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_FLOATING(2U) | \
                           PIN_PUPDR_FLOATING(3U) | \
                           PIN_PUPDR_PULLUP(4U) | \
                           PIN_PUPDR_FLOATING(5U) | \
                           PIN_PUPDR_FLOATING(6U) | \
                           PIN_PUPDR_FLOATING(7U) | \
                           PIN_PUPDR_FLOATING(8U) | \
                           PIN_PUPDR_FLOATING(9U) | \
                           PIN_PUPDR_FLOATING(10U) | \
                           PIN_PUPDR_FLOATING(11U) | \
                           PIN_PUPDR_FLOATING(12U) | \
                           PIN_PUPDR_FLOATING(13U) | \
                           PIN_PUPDR_FLOATING(14U) | \
                           PIN_PUPDR_FLOATING(15U))

#define VAL_GPIOA_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOA_AFRL    (PIN_AFIO_AF(0U, 1U) | \
                           PIN_AFIO_AF(1U, 1U) | \
                           PIN_AFIO_AF(2U, 1U) | \
                           PIN_AFIO_AF(3U, 1U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 5U) | \
                           PIN_AFIO_AF(6U, 5U) | \
                           PIN_AFIO_AF(7U, 5U))

#define VAL_GPIOA_AFRH    (PIN_AFIO_AF(8U, 0U) | \
                           PIN_AFIO_AF(9U, 0U) | \
                           PIN_AFIO_AF(10U, 0U) | \
                           PIN_AFIO_AF(11U, 10U) | \
                           PIN_AFIO_AF(12U, 10U) | \
                           PIN_AFIO_AF(13U, 0U) | \
                           PIN_AFIO_AF(14U, 0U) | \
                           PIN_AFIO_AF(15U, 0U))

/* PORTB:
 PB0 EXTI_MPU6000 INPUT
 PB1 DRDY_HMC5883 INPUT
 PB6 USART1_TX USART1 AF7
 PB7 USART1_RX USART1 AF7
 PB8 I2C1_SCL I2C1 AF4
 PB9 I2C1_SDA I2C1 AF4
 PB10 I2C2_SCL I2C2 AF4
 PB11 I2C2_SDA I2C2 AF4
 PB12 FRAM_CS CS
 PB13 SPI2_SCK SPI2 AF5
 PB14 SPI2_MISO SPI2 AF5
 PB15 SPI2_MOSI SPI2 AF5
*/

#define VAL_GPIOB_MODER   (PIN_MODE_INPUT(0U) | \
                           PIN_MODE_INPUT(1U) | \
                           PIN_MODE_INPUT(2U) | \
                           PIN_MODE_INPUT(3U) | \
                           PIN_MODE_INPUT(4U) | \
                           PIN_MODE_INPUT(5U) | \
                           PIN_MODE_ALTERNATE(6U) | \
                           PIN_MODE_ALTERNATE(7U) | \
                           PIN_MODE_ALTERNATE(8U) | \
                           PIN_MODE_ALTERNATE(9U) | \
                           PIN_MODE_ALTERNATE(10U) | \
                           PIN_MODE_ALTERNATE(11U) | \
                           PIN_MODE_OUTPUT(12U) | \
                           PIN_MODE_ALTERNATE(13U) | \
                           PIN_MODE_ALTERNATE(14U) | \
                           PIN_MODE_ALTERNATE(15U))

#define VAL_GPIOB_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_OPENDRAIN(8U) | \
                           PIN_OTYPE_OPENDRAIN(9U) | \
                           PIN_OTYPE_OPENDRAIN(10U) | \
                           PIN_OTYPE_OPENDRAIN(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOB_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_VERYLOW(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOB_PUPDR   (PIN_PUPDR_PULLUP(0U) | \
                           PIN_PUPDR_PULLUP(1U) | \
                           PIN_PUPDR_FLOATING(2U) | \
                           PIN_PUPDR_FLOATING(3U) | \
                           PIN_PUPDR_FLOATING(4U) | \
                           PIN_PUPDR_FLOATING(5U) | \
                           PIN_PUPDR_PULLUP(6U) | \
                           PIN_PUPDR_PULLUP(7U) | \
                           PIN_PUPDR_FLOATING(8U) | \
                           PIN_PUPDR_FLOATING(9U) | \
                           PIN_PUPDR_FLOATING(10U) | \
                           PIN_PUPDR_FLOATING(11U) | \
                           PIN_PUPDR_PULLUP(12U) | \
                           PIN_PUPDR_FLOATING(13U) | \
                           PIN_PUPDR_FLOATING(14U) | \
                           PIN_PUPDR_FLOATING(15U))

#define VAL_GPIOB_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOB_AFRL    (PIN_AFIO_AF(0U, 0U) | \
                           PIN_AFIO_AF(1U, 0U) | \
                           PIN_AFIO_AF(2U, 0U) | \
                           PIN_AFIO_AF(3U, 0U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 0U) | \
                           PIN_AFIO_AF(6U, 7U) | \
                           PIN_AFIO_AF(7U, 7U))

#define VAL_GPIOB_AFRH    (PIN_AFIO_AF(8U, 4U) | \
                           PIN_AFIO_AF(9U, 4U) | \
                           PIN_AFIO_AF(10U, 4U) | \
                           PIN_AFIO_AF(11U, 4U) | \
                           PIN_AFIO_AF(12U, 0U) | \
                           PIN_AFIO_AF(13U, 5U) | \
                           PIN_AFIO_AF(14U, 5U) | \
                           PIN_AFIO_AF(15U, 5U))

/* PORTC:
 PC0 PRESSURE_SENS ADC1 ADC1_IN10
 PC1 RSSI_IN ADC1 ADC1_IN11
 PC2 BATT_CURRENT_SENS ADC1 ADC1_IN12
 PC3 BATT_VOLTAGE_SENS ADC1 ADC1_IN13
 PC4 VDD_5V_SENS ADC1 ADC1_IN14
 PC5 FMU_SERVORAIL_VCC_SENS ADC1 ADC1_IN15
 PC7 TIM8_CH2 TIM8 AF3
 PC9 EXTERN_GPIO5 GPIO(5)
 PC10 UART4_TX UART4 AF8
 PC11 UART4_RX UART4 AF8
 PC12 UART5_TX UART5 AF8
 PC13 EXTERN_GPIO2 GPIO(2)
 PC14 EXTERN_GPIO1 GPIO(1)
*/

#define VAL_GPIOC_MODER   (PIN_MODE_ANALOG(0U) | \
                           PIN_MODE_ANALOG(1U) | \
                           PIN_MODE_ANALOG(2U) | \
                           PIN_MODE_ANALOG(3U) | \
                           PIN_MODE_ANALOG(4U) | \
                           PIN_MODE_ANALOG(5U) | \
                           PIN_MODE_INPUT(6U) | \
                           PIN_MODE_ALTERNATE(7U) | \
                           PIN_MODE_INPUT(8U) | \
                           PIN_MODE_INPUT(9U) | \
                           PIN_MODE_ALTERNATE(10U) | \
                           PIN_MODE_ALTERNATE(11U) | \
                           PIN_MODE_ALTERNATE(12U) | \
                           PIN_MODE_INPUT(13U) | \
                           PIN_MODE_INPUT(14U) | \
                           PIN_MODE_INPUT(15U))

#define VAL_GPIOC_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_PUSHPULL(9U) | \
                           PIN_OTYPE_PUSHPULL(10U) | \
                           PIN_OTYPE_PUSHPULL(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOC_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOC_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_FLOATING(2U) | \
                           PIN_PUPDR_FLOATING(3U) | \
                           PIN_PUPDR_FLOATING(4U) | \
                           PIN_PUPDR_FLOATING(5U) | \
                           PIN_PUPDR_FLOATING(6U) | \
                           PIN_PUPDR_PULLUP(7U) | \
                           PIN_PUPDR_FLOATING(8U) | \
                           PIN_PUPDR_FLOATING(9U) | \
                           PIN_PUPDR_PULLUP(10U) | \
                           PIN_PUPDR_PULLUP(11U) | \
                           PIN_PUPDR_PULLUP(12U) | \
                           PIN_PUPDR_FLOATING(13U) | \
                           PIN_PUPDR_FLOATING(14U) | \
                           PIN_PUPDR_FLOATING(15U))

#define VAL_GPIOC_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_LOW(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOC_AFRL    (PIN_AFIO_AF(0U, 0U) | \
                           PIN_AFIO_AF(1U, 0U) | \
                           PIN_AFIO_AF(2U, 0U) | \
                           PIN_AFIO_AF(3U, 0U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 0U) | \
                           PIN_AFIO_AF(6U, 0U) | \
                           PIN_AFIO_AF(7U, 3U))

#define VAL_GPIOC_AFRH    (PIN_AFIO_AF(8U, 0U) | \
                           PIN_AFIO_AF(9U, 0U) | \
                           PIN_AFIO_AF(10U, 8U) | \
                           PIN_AFIO_AF(11U, 8U) | \
                           PIN_AFIO_AF(12U, 8U) | \
                           PIN_AFIO_AF(13U, 0U) | \
                           PIN_AFIO_AF(14U, 0U) | \
                           PIN_AFIO_AF(15U, 0U))

/* PORTD:
 PD2 UART5_RX UART5 AF8
 PD5 USART2_TX USART2 AF7
 PD6 USART2_RX USART2 AF7
 PD8 USART3_TX USART3 AF7
 PD9 USART3_RX USART3 AF7
 PD12 TIM4_CH1 TIM4 AF2 PWM10
 PD13 TIM4_CH2 TIM4 AF2 PWM9
 PD14 TIM4_CH3 TIM4 AF2 PWM12
 PD15 TIM4_CH4 TIM4 AF2 PWM11
*/

#define VAL_GPIOD_MODER   (PIN_MODE_INPUT(0U) | \
                           PIN_MODE_INPUT(1U) | \
                           PIN_MODE_ALTERNATE(2U) | \
                           PIN_MODE_INPUT(3U) | \
                           PIN_MODE_INPUT(4U) | \
                           PIN_MODE_ALTERNATE(5U) | \
                           PIN_MODE_ALTERNATE(6U) | \
                           PIN_MODE_INPUT(7U) | \
                           PIN_MODE_ALTERNATE(8U) | \
                           PIN_MODE_ALTERNATE(9U) | \
                           PIN_MODE_INPUT(10U) | \
                           PIN_MODE_INPUT(11U) | \
                           PIN_MODE_ALTERNATE(12U) | \
                           PIN_MODE_ALTERNATE(13U) | \
                           PIN_MODE_ALTERNATE(14U) | \
                           PIN_MODE_ALTERNATE(15U))

#define VAL_GPIOD_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_PUSHPULL(9U) | \
                           PIN_OTYPE_PUSHPULL(10U) | \
                           PIN_OTYPE_PUSHPULL(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOD_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOD_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_PULLUP(2U) | \
                           PIN_PUPDR_FLOATING(3U) | \
                           PIN_PUPDR_FLOATING(4U) | \
                           PIN_PUPDR_PULLUP(5U) | \
                           PIN_PUPDR_PULLUP(6U) | \
                           PIN_PUPDR_FLOATING(7U) | \
                           PIN_PUPDR_PULLUP(8U) | \
                           PIN_PUPDR_PULLUP(9U) | \
                           PIN_PUPDR_FLOATING(10U) | \
                           PIN_PUPDR_FLOATING(11U) | \
                           PIN_PUPDR_FLOATING(12U) | \
                           PIN_PUPDR_FLOATING(13U) | \
                           PIN_PUPDR_FLOATING(14U) | \
                           PIN_PUPDR_FLOATING(15U))

#define VAL_GPIOD_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOD_AFRL    (PIN_AFIO_AF(0U, 0U) | \
                           PIN_AFIO_AF(1U, 0U) | \
                           PIN_AFIO_AF(2U, 8U) | \
                           PIN_AFIO_AF(3U, 0U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 7U) | \
                           PIN_AFIO_AF(6U, 7U) | \
                           PIN_AFIO_AF(7U, 0U))

#define VAL_GPIOD_AFRH    (PIN_AFIO_AF(8U, 7U) | \
                           PIN_AFIO_AF(9U, 7U) | \
                           PIN_AFIO_AF(10U, 0U) | \
                           PIN_AFIO_AF(11U, 0U) | \
                           PIN_AFIO_AF(12U, 2U) | \
                           PIN_AFIO_AF(13U, 2U) | \
                           PIN_AFIO_AF(14U, 2U) | \
                           PIN_AFIO_AF(15U, 2U))

/* PORTE:
 PE0 LED_YELOW OUTPUT
 PE1 LED_BLUE OUTPUT
 PE2 LED_GREEN OUTPUT
 PE3 LED_RED OUTPUT
 PE4 EXTERN_GPIO3 GPIO(3)
 PE5 TIM9_CH1 TIM9 AF3
 PE6 EXTERN_GPIO4 GPIO(4)
 PE9 TIM1_CH1 TIM1 AF1 PWM5
 PE11 TIM1_CH2 TIM1 AF1 PWM6
 PE13 TIM1_CH3 TIM1 AF1 PWM7
 PE14 TIM1_CH4 TIM1 AF1 PWM8
 PE15 FLASH_CS CS
*/

#define VAL_GPIOE_MODER   (PIN_MODE_OUTPUT(0U) | \
                           PIN_MODE_OUTPUT(1U) | \
                           PIN_MODE_OUTPUT(2U) | \
                           PIN_MODE_OUTPUT(3U) | \
                           PIN_MODE_INPUT(4U) | \
                           PIN_MODE_ALTERNATE(5U) | \
                           PIN_MODE_INPUT(6U) | \
                           PIN_MODE_INPUT(7U) | \
                           PIN_MODE_INPUT(8U) | \
                           PIN_MODE_ALTERNATE(9U) | \
                           PIN_MODE_INPUT(10U) | \
                           PIN_MODE_ALTERNATE(11U) | \
                           PIN_MODE_INPUT(12U) | \
                           PIN_MODE_ALTERNATE(13U) | \
                           PIN_MODE_ALTERNATE(14U) | \
                           PIN_MODE_OUTPUT(15U))

#define VAL_GPIOE_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_PUSHPULL(9U) | \
                           PIN_OTYPE_PUSHPULL(10U) | \
                           PIN_OTYPE_PUSHPULL(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOE_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOE_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_FLOATING(2U) | \
                           PIN_PUPDR_FLOATING(3U) | \
                           PIN_PUPDR_FLOATING(4U) | \
                           PIN_PUPDR_FLOATING(5U) | \
                           PIN_PUPDR_FLOATING(6U) | \
                           PIN_PUPDR_FLOATING(7U) | \
                           PIN_PUPDR_FLOATING(8U) | \
                           PIN_PUPDR_FLOATING(9U) | \
                           PIN_PUPDR_FLOATING(10U) | \
                           PIN_PUPDR_FLOATING(11U) | \
                           PIN_PUPDR_FLOATING(12U) | \
                           PIN_PUPDR_FLOATING(13U) | \
                           PIN_PUPDR_FLOATING(14U) | \
                           PIN_PUPDR_PULLUP(15U))

#define VAL_GPIOE_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOE_AFRL    (PIN_AFIO_AF(0U, 0U) | \
                           PIN_AFIO_AF(1U, 0U) | \
                           PIN_AFIO_AF(2U, 0U) | \
                           PIN_AFIO_AF(3U, 0U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 3U) | \
                           PIN_AFIO_AF(6U, 0U) | \
                           PIN_AFIO_AF(7U, 0U))

#define VAL_GPIOE_AFRH    (PIN_AFIO_AF(8U, 0U) | \
                           PIN_AFIO_AF(9U, 1U) | \
                           PIN_AFIO_AF(10U, 0U) | \
                           PIN_AFIO_AF(11U, 1U) | \
                           PIN_AFIO_AF(12U, 0U) | \
                           PIN_AFIO_AF(13U, 1U) | \
                           PIN_AFIO_AF(14U, 1U) | \
                           PIN_AFIO_AF(15U, 0U))

/* PORTF:
*/

#define VAL_GPIOF_MODER   (PIN_MODE_INPUT(0U) | \
                           PIN_MODE_INPUT(1U) | \
                           PIN_MODE_INPUT(2U) | \
                           PIN_MODE_INPUT(3U) | \
                           PIN_MODE_INPUT(4U) | \
                           PIN_MODE_INPUT(5U) | \
                           PIN_MODE_INPUT(6U) | \
                           PIN_MODE_INPUT(7U) | \
                           PIN_MODE_INPUT(8U) | \
                           PIN_MODE_INPUT(9U) | \
                           PIN_MODE_INPUT(10U) | \
                           PIN_MODE_INPUT(11U) | \
                           PIN_MODE_INPUT(12U) | \
                           PIN_MODE_INPUT(13U) | \
                           PIN_MODE_INPUT(14U) | \
                           PIN_MODE_INPUT(15U))

#define VAL_GPIOF_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_PUSHPULL(9U) | \
                           PIN_OTYPE_PUSHPULL(10U) | \
                           PIN_OTYPE_PUSHPULL(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOF_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOF_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_FLOATING(2U) | \
                           PIN_PUPDR_FLOATING(3U) | \
                           PIN_PUPDR_FLOATING(4U) | \
                           PIN_PUPDR_FLOATING(5U) | \
                           PIN_PUPDR_FLOATING(6U) | \
                           PIN_PUPDR_FLOATING(7U) | \
                           PIN_PUPDR_FLOATING(8U) | \
                           PIN_PUPDR_FLOATING(9U) | \
                           PIN_PUPDR_FLOATING(10U) | \
                           PIN_PUPDR_FLOATING(11U) | \
                           PIN_PUPDR_FLOATING(12U) | \
                           PIN_PUPDR_FLOATING(13U) | \
                           PIN_PUPDR_FLOATING(14U) | \
                           PIN_PUPDR_FLOATING(15U))

#define VAL_GPIOF_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOF_AFRL    (PIN_AFIO_AF(0U, 0U) | \
                           PIN_AFIO_AF(1U, 0U) | \
                           PIN_AFIO_AF(2U, 0U) | \
                           PIN_AFIO_AF(3U, 0U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 0U) | \
                           PIN_AFIO_AF(6U, 0U) | \
                           PIN_AFIO_AF(7U, 0U))

#define VAL_GPIOF_AFRH    (PIN_AFIO_AF(8U, 0U) | \
                           PIN_AFIO_AF(9U, 0U) | \
                           PIN_AFIO_AF(10U, 0U) | \
                           PIN_AFIO_AF(11U, 0U) | \
                           PIN_AFIO_AF(12U, 0U) | \
                           PIN_AFIO_AF(13U, 0U) | \
                           PIN_AFIO_AF(14U, 0U) | \
                           PIN_AFIO_AF(15U, 0U))

/* PORTG:
*/

#define VAL_GPIOG_MODER   (PIN_MODE_INPUT(0U) | \
                           PIN_MODE_INPUT(1U) | \
                           PIN_MODE_INPUT(2U) | \
                           PIN_MODE_INPUT(3U) | \
                           PIN_MODE_INPUT(4U) | \
                           PIN_MODE_INPUT(5U) | \
                           PIN_MODE_INPUT(6U) | \
                           PIN_MODE_INPUT(7U) | \
                           PIN_MODE_INPUT(8U) | \
                           PIN_MODE_INPUT(9U) | \
                           PIN_MODE_INPUT(10U) | \
                           PIN_MODE_INPUT(11U) | \
                           PIN_MODE_INPUT(12U) | \
                           PIN_MODE_INPUT(13U) | \
                           PIN_MODE_INPUT(14U) | \
                           PIN_MODE_INPUT(15U))

#define VAL_GPIOG_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_PUSHPULL(9U) | \
                           PIN_OTYPE_PUSHPULL(10U) | \
                           PIN_OTYPE_PUSHPULL(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOG_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOG_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_FLOATING(2U) | \
                           PIN_PUPDR_FLOATING(3U) | \
                           PIN_PUPDR_FLOATING(4U) | \
                           PIN_PUPDR_FLOATING(5U) | \
                           PIN_PUPDR_FLOATING(6U) | \
                           PIN_PUPDR_FLOATING(7U) | \
                           PIN_PUPDR_FLOATING(8U) | \
                           PIN_PUPDR_FLOATING(9U) | \
                           PIN_PUPDR_FLOATING(10U) | \
                           PIN_PUPDR_FLOATING(11U) | \
                           PIN_PUPDR_FLOATING(12U) | \
                           PIN_PUPDR_FLOATING(13U) | \
                           PIN_PUPDR_FLOATING(14U) | \
                           PIN_PUPDR_FLOATING(15U))

#define VAL_GPIOG_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOG_AFRL    (PIN_AFIO_AF(0U, 0U) | \
                           PIN_AFIO_AF(1U, 0U) | \
                           PIN_AFIO_AF(2U, 0U) | \
                           PIN_AFIO_AF(3U, 0U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 0U) | \
                           PIN_AFIO_AF(6U, 0U) | \
                           PIN_AFIO_AF(7U, 0U))

#define VAL_GPIOG_AFRH    (PIN_AFIO_AF(8U, 0U) | \
                           PIN_AFIO_AF(9U, 0U) | \
                           PIN_AFIO_AF(10U, 0U) | \
                           PIN_AFIO_AF(11U, 0U) | \
                           PIN_AFIO_AF(12U, 0U) | \
                           PIN_AFIO_AF(13U, 0U) | \
                           PIN_AFIO_AF(14U, 0U) | \
                           PIN_AFIO_AF(15U, 0U))

/* PORTH:
*/

#define VAL_GPIOH_MODER   (PIN_MODE_INPUT(0U) | \
                           PIN_MODE_INPUT(1U))

#define VAL_GPIOH_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U))

#define VAL_GPIOH_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U))

#define VAL_GPIOH_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U))

#define VAL_GPIOH_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U))

#define VAL_GPIOH_AFRL    (PIN_AFIO_AF(0U, 0U) | \
                           PIN_AFIO_AF(1U, 0U))

#define VAL_GPIOH_AFRH    (0)

/* PORTI:
*/

#define VAL_GPIOI_MODER               0x0
#define VAL_GPIOI_OTYPER              0x0
#define VAL_GPIOI_OSPEEDR             0x0
#define VAL_GPIOI_PUPDR               0x0
#define VAL_GPIOI_ODR                 0x0
#define VAL_GPIOI_AFRL                0x0
#define VAL_GPIOI_AFRH                0x0



/* PORTJ:
*/

#define VAL_GPIOJ_MODER               0x0
#define VAL_GPIOJ_OTYPER              0x0
#define VAL_GPIOJ_OSPEEDR             0x0
#define VAL_GPIOJ_PUPDR               0x0
#define VAL_GPIOJ_ODR                 0x0
#define VAL_GPIOJ_AFRL                0x0
#define VAL_GPIOJ_AFRH                0x0



/* PORTK:
*/

#define VAL_GPIOK_MODER               0x0
#define VAL_GPIOK_OTYPER              0x0
#define VAL_GPIOK_OSPEEDR             0x0
#define VAL_GPIOK_PUPDR               0x0
#define VAL_GPIOK_ODR                 0x0
#define VAL_GPIOK_AFRL                0x0
#define VAL_GPIOK_AFRH                0x0



