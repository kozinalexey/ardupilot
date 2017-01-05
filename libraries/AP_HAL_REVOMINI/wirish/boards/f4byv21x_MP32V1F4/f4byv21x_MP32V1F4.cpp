#ifndef BOARD_STM32V1F4
#define BOARD_STM32V1F4

#include "f4byv21x_MP32V1F4.h"
#include "hal_types.h"
#include "wirish_types.h"
#include "gpio_hal.h"
#include "adc.h"
#include "timer.h"
#include "wirish_types.h"


extern const stm32_pin_info PIN_MAP[BOARD_NR_GPIO_PINS] = {

    /* Top header */
/*
    const gpio_dev  * const gpio_device;      < Maple pin's GPIO device 
    const timer_dev * const timer_device;     < Pin's timer device, if any. 
    const adc_dev   * const adc_device;       < ADC device, if any. 
    uint8_t gpio_bit;             < Pin's GPIO port bit. 
    uint8_t timer_channel;        < Timer channel, or 0 if none. 
    uint8_t adc_channel;          < Pin ADC channel, or ADCx if none. 
*/
															//	PINS OF STM32F407VGTX 100-PIN CHIP
    {&gpiob,   NULL,  	NULL, 10, 0, ADCx}, 	// D0/PB10  0   47	PB10	I/O				TIM2_CH3		I2C2_SCL	I2S2_CK/SPI2_SCK		USART3_TX			USB_OTG_HS_ULPI_D3	ETH_RX_ER
    {&gpiob,   NULL,    NULL,  2, 0, ADCx}, 	// D1/PB2   1  	37	PB2	    				EXTI2
    {&gpiob,   NULL,    NULL, 12, 0, ADCx},    	// D2/PB12  2 	51	PB12	I/O				TIM1_BKIN		I2C2_SMBA	I2S2_WS/SPI2_NSS		USART3_CK		CAN2_RX	USB_OTG_HS_ULPI_D5	ETH_TXD0	USB_OTG_HS_ID
    {&gpiob,   NULL,    NULL, 13, 0, ADCx},    	// D3/PB13  3 	52	PB13	I/O				TIM1_CH1N		I2S2_CK/SPI2_SCK		USART3_CTS		CAN2_TX	USB_OTG_HS_ULPI_D6	ETH_TXD1
    {&gpiob,   &timer12,NULL, 14, 1, ADCx}, 	// D4/PB14  4 	53	PB14	I/O				TIM1_CH2N		TIM8_CH2N		SPI2_MISO	I2S2_ext_SD	USART3_RTS		TIM12_CH1			USB_OTG_HS_DM
    {&gpiob,   &timer12,NULL, 15, 2, ADCx},     // D5/pb15 	5	54	PB15	I/O			RTC_REFIN	TIM1_CH3N		TIM8_CH3N		I2S2_SD/SPI2_MOSI				TIM12_CH2			USB_OTG_HS_DP
    {&gpioc,   NULL,  &_adc1, 0,  0,   10}, 	// D6/PC0  	6 	15	PC0		I/O
    {&gpioc,   NULL,  &_adc1, 1,  0,   11},     // D7/PC1   7 	16	PC1		I/O
    {&gpioc,   NULL,  &_adc1, 2,  0,   12}, 	// D8/PC2   8 	17	PC2		I/O
    {&gpioc,   NULL,  &_adc1, 3,  0,   13},   	// D9/PC3   9   18	PC3		I/O
    {&gpioc,   NULL,  &_adc1, 4,  0,   14},     // D10/PC4  10 	33	PC4		I/O
    {&gpioc,   NULL,  &_adc1, 5,  0,   15},     // D11/PC5  1 	34	PC5		I/O
    {&gpioc,   &timer8,NULL,  6,  1, ADCx}, 	// D12/PC6  2 	63	PC6		I/O					TIM3_CH1	TIM8_CH1		I2S2_MCK			USART6_TX				SDIO_D6	DCMI_D0
    {&gpioc,   &timer8,NULL,  7,  2, ADCx}, 	// D13/PC7  3   64	PC7		I/O					TIM3_CH2	TIM8_CH2			I2S3_MCK		USART6_RX				SDIO_D7	DCMI_D1
    {&gpioc,   &timer8,NULL,  8,  3, ADCx}, 	// D14/PC8  4	65	PC8		I/O					TIM3_CH3	TIM8_CH3					USART6_CK				SDIO_D0	DCMI_D2
    {&gpioc,   &timer8,NULL,  9,  4, ADCx}, 	// D15/PC9  5 	66	PC9		I/O			RCC_MCO_2		TIM3_CH4	TIM8_CH4	I2C3_SDA	I2S_CKIN							SDIO_D1	DCMI_D3
    {&gpioc,   NULL,   NULL, 10,  0, ADCx}, 	// D16/PC10 6   78	PC10	I/O									I2S3_CK/SPI3_SCK	USART3_TX	UART4_TX				SDIO_D2	DCMI_D8
    {&gpioc,   NULL,   NULL, 11,  0, ADCx},     // D17/PC11 7 	79	PC11	I/O								I2S3_ext_SD	SPI3_MISO	USART3_RX	UART4_RX				SDIO_D3	DCMI_D4
    {&gpioc,   NULL,   NULL, 12,  0, ADCx},     // D18/PC12 8   80	PC12	I/O									I2S3_SD/SPI3_MOSI	USART3_CK	UART5_TX				SDIO_CK	DCMI_D9
    {&gpioc,   NULL,   NULL, 13,  0, ADCx},     // D19/PC13 9    7	PC13-ANTI_TAMP	I/O			RTC_AF1
    {&gpioc,   NULL,   NULL, 14,  0, ADCx},     // D20/PC14 20 	 8	PC14-OSC32_IN	I/O			RCC_OSC32_IN
    {&gpioc,   NULL,   NULL, 15,  0, ADCx},     // D21/PC15 1    9	PC15-OSC32_OUT	I/O			RCC_OSC32_OUT

    {&gpioa,   NULL,   NULL,  8,  0, ADCx},  //TIMER1CH1 USED PE9  D22/PA8  2   67	PA8		I/O			RCC_MCO_1	TIM1_CH1			I2C3_SCL			USART1_CK			USB_OTG_FS_SOF
    {&gpioa,   NULL,   NULL,  9,  0, ADCx},  //TIMER1CH2 USED pe11 D23/PA9  3   68	PA9		I/O				TIM1_CH2			I2C3_SMBA			USART1_TX						DCMI_D0
    {&gpioa,   NULL, NULL,   10,  0, ADCx},  //TIMER1CH3 D24/PA10 4             69	PA10	I/O				TIM1_CH3						USART1_RX			USB_OTG_FS_ID			DCMI_D1

    {&gpiob,   NULL, NULL,    9,  4, ADCx},     // D25/PB9  5 	96	PB9		I/O					TIM4_CH4	TIM11_CH1	I2C1_SDA	I2S2_WS/SPI2_NSS				CAN1_TX			SDIO_D5	DCMI_D7
    {&gpiod,   NULL, NULL,    2,  0, ADCx}, 	// D26/PD2  6 	83	PD2		I/O					TIM3_ETR						UART5_RX				SDIO_CMD	DCMI_D11
    {&gpiod,   NULL, NULL,    3,  0, ADCx}, 	// D27/PD3  7 	84	PD3		I/O										USART2_CTS					FSMC_CLK
    {&gpiod,   NULL, NULL,    6,  0, ADCx},    	// D28/PD6  8	87	PD6		I/O										USART2_RX					FSMC_NWAIT
    {&gpiog,   NULL, NULL,   11,  0, ADCx}, 	// D29/PG11 9   not found at 100 pins chips
    {&gpiog,   NULL, NULL,   12,  0, ADCx}, 	// D30/PG12 30  not found at 100 pins chips
    {&gpiog,   NULL, NULL,   13,  0, ADCx}, 	// D31/PG13 1	not found at 100 pins chips
    {&gpiog,   NULL, NULL,   14,  0, ADCx},		// D32/PG14 2   not found at 100 pins chips
    {&gpiog,   NULL, NULL,    8,  0, ADCx}, 	// D33/PG8  3   not found at 100 pins chips
    {&gpiog,   NULL, NULL,    7,  0, ADCx}, 	// D34/PG7  4   not found at 100 pins chips
    {&gpiog,   NULL, NULL,    6,  0, ADCx}, 	// D35/PG6  5   not found at 100 pins chips
    {&gpiob, &timer3,NULL,    5,  2, ADCx}, //TIMER! D36/PB5  6 91	PB5	I/O					TIM3_CH2		I2C1_SMBA	SPI1_MOSI	I2S3_SD/SPI3_MOSI			CAN2_RX	USB_OTG_HS_ULPI_D7	ETH_PPS_OUT		DCMI_D10
    {&gpiob, &timer4,NULL,    6,  1, ADCx}, //timer! D37/PB6  7 92	PB6	I/O					TIM4_CH1		I2C1_SCL			USART1_TX		CAN2_TX				DCMI_D5
    {&gpiob, &timer4,NULL,    7,  2, ADCx}, //timer! D38/PB7  8 93	PB7	I/O					TIM4_CH2		I2C1_SDA			USART1_RX					FSMC_NL	DCMI_VSYNC
    {&gpiof,   NULL,&_adc3,   6,  0,    4}, 	// D39/PF6    9 not found at 100 pins chips
    {&gpiof,   NULL,&_adc3,   7,  0,    5}, 	// D40/PF7  40 		not found at 100 pins chips
    {&gpiof,   NULL,&_adc3,   8,  0,    6}, 	// D41/PF8 	1 	not found at 100 pins chips
    {&gpiof,   NULL,&_adc3,   9,  0,    7}, 	// D42/PF9 	2   not found at 100 pins chips
    {&gpiof,   NULL,&_adc3,  10,  0,    8}, 	// D43/PF10	3 	not found at 100 pins chips
    {&gpiof,   NULL, NULL,   11,  0, ADCx}, 	// D44/PF11	4 	not found at 100 pins chips
    {&gpiob, &timer3,&_adc1, 1,   4, 	9},     // D45/PB1 	5  36	PB1	I/O				TIM1_CH3N	TIM3_CH4	TIM8_CH3N							USB_OTG_HS_ULPI_D2	ETH_RXD3
    {&gpiob, &timer3,&_adc1,  0,  3,  	8}, 	// D46/PB0  6  35	PB0	I/O				TIM1_CH2N	TIM3_CH3	TIM8_CH2N							USB_OTG_HS_ULPI_D1	ETH_RXD2

    {&gpioa, &timer2,&_adc1,0, 	  1,    0}, /* D47/PA0  7  SERVO6 / UART4_TX */
    {&gpioa, &timer2,&_adc1,1, 2,    1}, /* D48/PA1  8  SERVO5 / UART4_RX */
    {&gpioa, &timer2,&_adc1,2, 3,    2}, /* D49/PA2  9  SERVO4 TIMER*/
    {&gpioa, &timer2,&_adc1,3, 4,    3}, /* D50/PA3  50 SERVO3 */
    {&gpioa,   NULL, &_adc1,4, 0,    4}, /* D51/PA4  1 CS_MPU6000 */
    {&gpioa,   NULL, &_adc1,5, 0,    5}, /* D52/PA5  2 SPI1_CLK */
    {&gpioa, &timer3,&_adc1,6, 1,    6}, /* D53/PA6  3 SPI1_MISO */
    {&gpioa, &timer3,&_adc1,7, 2,    7}, /* D54/PA7  4 SPI1_MOSI */
    {&gpiof,   NULL, NULL,  0, 0, ADCx}, /* D55/PF0  5*/
    {&gpiod,   NULL, NULL, 11, 0, ADCx}, /* D56/PD11 6*/
    {&gpiod, &timer4,NULL, 14, 3, ADCx}, /* D57/PD14 7*/
    {&gpiof,   NULL, NULL,  1, 0, ADCx}, /* D58/PF1  8*/
    {&gpiod, &timer4,NULL, 12, 1, ADCx}, /* D59/PD12 9*/
    {&gpiod, &timer4,NULL, 15, 4, ADCx}, /* D60/PD15 60*/
    {&gpiof,   NULL, NULL,  2, 0, ADCx}, /* D61/PF2  1*/
    {&gpiod, &timer4,NULL, 13, 2, ADCx}, /* D62/PD13 2*/
    {&gpiod,   NULL, NULL,  0, 0, ADCx}, /* D63/PD0  3*/
    {&gpiof,   NULL, NULL,  3, 0, ADCx}, /* D64/PF3  4*/
    {&gpioe,   NULL, NULL,  3, 0, ADCx}, /* D65/PE3  5*/
    {&gpiod,   NULL, NULL,  1, 0, ADCx}, /* D66/PD1  6*/
    {&gpiof,   NULL, NULL,  4, 0, ADCx}, /* D67/PF4  7*/
    {&gpioe,   NULL, NULL,  4, 0, ADCx}, /* D68/PE4  8*/
    {&gpioe,   NULL, NULL,  7, 0, ADCx}, /* D69/PE7  9*/
    {&gpiof,   NULL, NULL,  5, 0, ADCx}, /* D70/PF5  70*/
    {&gpioe,   NULL, NULL,  5, 0, ADCx}, /* D71/PE5  1*/
    {&gpioe,   NULL, NULL,  8, 0, ADCx}, /* D72/PE8  2*/
    {&gpiof,   NULL, NULL, 12, 0, ADCx}, /* D73/PF12 3*/
    {&gpioe,   NULL, NULL,  6, 0, ADCx}, /* D74/PE6  4*/
    {&gpioe, &timer1,NULL,  9, 1, ADCx}, /* D75/PE9  */
    {&gpiof,   NULL, NULL, 13, 0, ADCx}, /* D76/PF13 6*/
    {&gpioe,   NULL, NULL, 10, 0, ADCx}, /* D77/PE10 7*/
    {&gpiof,   NULL, NULL, 14, 0, ADCx}, /* D78/PF14 8*/
    {&gpiog,   NULL, NULL,  9, 0, ADCx}, /* D79/PG9  9*/
    {&gpioe, &timer1,NULL, 11, 2, ADCx}, /* D80/PE11 */
    {&gpiof,   NULL, NULL, 15, 0, ADCx}, /* D81/PF15 1*/
    {&gpiog,   NULL, NULL, 10, 0, ADCx}, /* D82/PG10 2*/
    {&gpioe,   NULL, NULL, 12, 0, ADCx}, /* D83/PE12 3*/
    {&gpiog,   NULL, NULL,  0, 0, ADCx}, /* D84/PG0  4*/
    {&gpiod,   NULL, NULL,  5, 0, ADCx}, /* D85/PD5  5*/
    {&gpioe, &timer1,NULL, 13, 3, ADCx}, /* D86/PE13 */
    {&gpiog,   NULL, NULL,  1, 0, ADCx}, /* D87/PG1  7*/
    {&gpiod,   NULL, NULL,  4, 0, ADCx}, /* D88/PD4  8*/
    {&gpioe, &timer1,NULL, 14, 4, ADCx}, /* D89/PE14 */
    {&gpiog,   NULL, NULL,  2, 0, ADCx}, /* D90/PG2  90*/
    {&gpioe,   NULL, NULL,  1, 0, ADCx}, /* D91/PE1  1*/
    {&gpioe,   NULL, NULL, 15, 0, ADCx}, /* D92/PE15 2*/
    {&gpiog,   NULL, NULL,  3, 0, ADCx}, /* D93/PG3  3*/
    {&gpioe,   NULL, NULL,  0, 0, ADCx}, /* D94/PE0  4*/
    {&gpiod,   NULL, NULL,  8, 0, ADCx}, /* D95/PD8  5*/
    {&gpiog,   NULL, NULL,  4, 0, ADCx}, /* D96/PG4  6*/
    {&gpiod,   NULL, NULL,  9, 0, ADCx}, /* D97/PD9  7*/
    {&gpiog,   NULL, NULL,  5, 0, ADCx}, /* D98/PG5  8*/
    {&gpiod,   NULL, NULL, 10, 0, ADCx}, /* D99/PD10 9*/
    {&gpiob,   NULL, NULL, 11, 0, ADCx}, /* D100/PB11 100 USART3_RX/I2C2-SDA */
    {&gpiob,   NULL, NULL,  8, 0, ADCx}, /* D101/PB8  I2C1_SCL  */
    {&gpioe,   NULL, NULL,  2, 0, ADCx}, /* D102/PE2 */
    {&gpioa,   NULL, NULL, 15, 0, ADCx}, /* D103/PA15 CS_RFM22B */
    {&gpiob,   NULL, NULL,  3, 0, ADCx}, /* D104/PB3  CS_FLASH */
    {&gpiob,   NULL, NULL,  4, 0, ADCx}, /* D105/PB4  LED_RED */
    {&gpioa,   NULL, NULL, 13, 0, ADCx}, /* D106/PA13 LED_MOTOR */
    {&gpioa,   NULL, NULL, 14, 0, ADCx}  /* D107/PA14 */
};




void boardInit(void) {

    /* Configure PA.13 (JTMS/SWDIO), PA.14 (JTCK/SWCLK) as output push-pull */
//    afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY); // there is error in this function so instead of disabling only JTAG pins it disables SWD pins
    afio_cfg_debug_ports(AFIO_DEBUG_FULL_SWJ);  // so let's it be fill debug 


/* we don't use RFM22! this pins are used for other needs so will be initialized in respective places

    // Init RFM22B SC pin and set to HI
    gpio_set_mode( PIN_MAP[BOARD_RFM22B_CS_PIN].gpio_device, PIN_MAP[BOARD_RFM22B_CS_PIN].gpio_bit, GPIO_OUTPUT_PP);
    gpio_write_bit(PIN_MAP[BOARD_RFM22B_CS_PIN].gpio_device, PIN_MAP[BOARD_RFM22B_CS_PIN].gpio_bit, 1);
    
    // Init RFM22B EXT_INT pin
    gpio_set_mode(PIN_MAP[BOARD_RFM22B_INT_PIN].gpio_device, PIN_MAP[BOARD_RFM22B_INT_PIN].gpio_bit, GPIO_INPUT_PU);
*/

    // Init HMC5883 DRDY EXT_INT pin - but it not used by driver
    gpio_set_mode(PIN_MAP[BOARD_HMC5883_DRDY_PIN].gpio_device, PIN_MAP[BOARD_HMC5883_DRDY_PIN].gpio_bit, GPIO_INPUT_PU);

    // Init MPU6000 DRDY pin - but it not used by driver
    gpio_set_mode(PIN_MAP[BOARD_MPU6000_DRDY_PIN].gpio_device, PIN_MAP[BOARD_MPU6000_DRDY_PIN].gpio_bit, GPIO_INPUT_PU);


// it is not necessary because of 10K resistor to ground
    gpio_set_mode( PIN_MAP[BOARD_SBUS_INVERTER].gpio_device, PIN_MAP[BOARD_SBUS_INVERTER].gpio_bit, GPIO_OUTPUT_PP);
    gpio_write_bit(PIN_MAP[BOARD_SBUS_INVERTER].gpio_device, PIN_MAP[BOARD_SBUS_INVERTER].gpio_bit, 0); // not inverted



//*///    enable clock in sleep for debugging
    DBGMCU->CR |= DBGMCU_STANDBY | DBGMCU_STOP | DBGMCU_SLEEP;
//*///

}


#endif
