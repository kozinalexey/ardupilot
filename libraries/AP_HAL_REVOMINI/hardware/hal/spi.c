#include <spi.h>
#include <hal.h>

/*
 * SPI devices
 */

static const spi_dev spi1 = {
    .SPIx     = SPI1,
    .afio     = GPIO_AF_SPI1,
    .irq      = SPI1_IRQn,
    .clock    = RCC_APB2Periph_SPI1,
};
/** SPI device 1 */
const spi_dev * const _SPI1 = &spi1;

static const spi_dev spi2 = {
    .SPIx     = SPI2,
    .afio     = GPIO_AF_SPI2,
    .irq      = SPI2_IRQn,
    .clock    = RCC_APB1Periph_SPI2,
};
/** SPI device 2 */
const spi_dev * const _SPI2 = &spi2;

static const spi_dev spi3 = {
    .SPIx     = SPI3,
    .afio     = GPIO_AF_SPI3,
    .irq      = SPI3_IRQn,
    .clock    = RCC_APB1Periph_SPI3,
};
/** SPI device 3 */
const spi_dev * const _SPI3 = &spi3;


void spi_init(const spi_dev *dev) {
	SPI_I2S_DeInit(dev->SPIx);
}

/**
 * @brief Call a function on each SPI port
 * @param fn Function to call.
 */
void spi_foreach(void (*fn)(const spi_dev*)) {
    fn(_SPI1);
    fn(_SPI2);
    fn(_SPI3);
}

/**
 * @brief Enable a SPI peripheral
 * @param dev Device to enable
 */
void spi_peripheral_enable(const spi_dev *dev) {
	SPI_Cmd(dev->SPIx, ENABLE);
}

/**
 * @brief Disable a SPI peripheral
 * @param dev Device to disable
 */
void spi_peripheral_disable(const spi_dev *dev) {
	SPI_Cmd(dev->SPIx, DISABLE);
}

void spi_gpio_cfg(const spi_dev *dev,
		  uint8_t as_master,
                  const gpio_dev *nss_dev,
                  uint8_t nss_bit,
                  const gpio_dev *comm_dev,
                  uint8_t sck_bit,
                  uint8_t miso_bit,
                  uint8_t mosi_bit) {

    if (as_master) {
	/* Configure NSS pin */	
        gpio_set_mode(nss_dev, nss_bit, GPIO_OUTPUT_PP);
	gpio_write_bit(nss_dev, nss_bit, 1);
	
	/* Configure SCK pin */
        gpio_set_mode(comm_dev, sck_bit, GPIO_AF_OUTPUT_PP);
        gpio_set_af_mode(comm_dev, sck_bit, dev->afio);
        gpio_set_speed(comm_dev, sck_bit, GPIO_Speed_100MHz);
        
        /* Configure MISO pin */
        gpio_set_mode(comm_dev, miso_bit, GPIO_AF_OUTPUT_OD);
        gpio_set_af_mode(comm_dev, miso_bit, dev->afio);
        gpio_set_speed(comm_dev, miso_bit, GPIO_Speed_100MHz);
        
        /* Configure MOSI pin */
        gpio_set_mode(comm_dev, mosi_bit, GPIO_AF_OUTPUT_PP);
	gpio_set_af_mode(comm_dev, mosi_bit, dev->afio);        
	gpio_set_speed(comm_dev, mosi_bit, GPIO_Speed_100MHz);
    } else {
	/* Configure NSS pin */	
        gpio_set_mode(nss_dev, nss_bit, GPIO_INPUT_FLOATING);
        /* Configure SCK pin */
        gpio_set_mode(comm_dev, sck_bit, GPIO_INPUT_FLOATING);
        /* Configure MISO pin */
        gpio_set_mode(comm_dev, miso_bit, GPIO_AF_OUTPUT_PP);
        /* Configure MOSI pin */
        gpio_set_mode(comm_dev, mosi_bit, GPIO_INPUT_FLOATING);
    }
}

/*
 * SPI auxiliary routines
 */
static void spi_reconfigure(const spi_dev *dev, uint8_t ismaster, uint16_t baudPrescaler, uint16_t bitorder, uint8_t mode) {
    SPI_InitTypeDef  SPI_InitStructure;
	
    spi_irq_disable(dev, SPI_INTERRUPTS_ALL);
    SPI_I2S_DeInit(dev->SPIx);

	/* Enable the SPI clock */
	if (dev->SPIx == SPI1)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);  
	else if (dev->SPIx == SPI2)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	else
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
	
	/* SPI configuration */
	SPI_StructInit(&SPI_InitStructure);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	
	switch(mode) {
	case SPI_MODE_0:
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
		break;
	case SPI_MODE_1:
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
		break;
	case SPI_MODE_2:
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
		break;
	case SPI_MODE_3:
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
		break;
	default:
		break;
	}

/*
 spiInit.SPI_Mode = SPI_Mode_Master;
    spiInit.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spiInit.SPI_DataSize = SPI_DataSize_8b;
    spiInit.SPI_NSS = SPI_NSS_Soft;
    spiInit.SPI_FirstBit = SPI_FirstBit_MSB;
    spiInit.SPI_CRCPolynomial = 7;
    spiInit.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;

    spiInit.SPI_CPOL = SPI_CPOL_High;
    spiInit.SPI_CPHA = SPI_CPHA_2Edge;
*/

	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = baudPrescaler;
	if (bitorder == LSBFIRST)
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
	else
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
		
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	if (ismaster)
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	else
		SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
	
	SPI_Init(dev->SPIx, &SPI_InitStructure);

	if (!ismaster) {
		/* Enable the Rx buffer not empty interrupt */
		spi_irq_enable(dev, SPI_I2S_IT_RXNE);
	}
	
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Configure the Priority Group to 1 bit */                
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
	/* Configure the SPI interrupt priority */
/*
	NVIC_InitStructure.NVIC_IRQChannel = dev->irq;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
*/	    	
    SPI_Cmd(dev->SPIx, ENABLE);
}

/**
 * @brief Configure and enable a SPI device as bus master.
 *
 * The device's peripheral will be disabled before being reconfigured.
 *
 */
inline void spi_master_enable(const spi_dev *dev,
                       spi_baud_rate baudPrescaler,
                       spi_mode mode,
                       uint16_t bitorder) 
{
    spi_reconfigure(dev, 1, baudPrescaler, bitorder, mode);
}

/**
 * @brief Configure and enable a SPI device as a bus slave.
 *
 * The device's peripheral will be disabled before being reconfigured.
 *
 */
inline void spi_slave_enable(const spi_dev *dev,
                      spi_mode mode,
                      uint16_t bitorder)
{
    spi_reconfigure(dev, 0, 0, bitorder, mode);
}


// Transmit command and/or receive result in bidirectional master mode
int spimaster_transfer(const spi_dev *dev,
                       const uint8_t *txbuf,
                       uint32_t txcount,
                       uint8_t *rxbuf,
                       uint32_t rxcount)
{
	//errno_r = 0;

	// Validate parameters
	if ((txbuf == NULL) && (txcount != 0)){
		//errno_r = EINVAL;
		return __LINE__ - 3;
	}

	if ((txcount == 0) && (txbuf != NULL)){
		//errno_r = EINVAL;
		return __LINE__ - 3;
	}

	if ((rxbuf == NULL) && (rxcount != 0)){
		//errno_r = EINVAL;
		return __LINE__ - 3;
	}

	if ((rxcount == 0) && (rxbuf != NULL)){
		//errno_r = EINVAL;
		return __LINE__ - 3;
	}

    uint16_t tmp;

	// Transfer command data out
	while (txcount--){
	    while (SPI_I2S_GetFlagStatus(dev->SPIx, SPI_I2S_FLAG_TXE) == RESET);  //while (!(dev->SPIx->SR & SPI_I2S_FLAG_TXE));
	    SPI_I2S_SendData(dev->SPIx, *txbuf++);//		dev->SPIx->DR = *txbuf++;
	    while (SPI_I2S_GetFlagStatus(dev->SPIx, SPI_I2S_FLAG_RXNE) == RESET);//while (!(dev->SPIx->SR & SPI_I2S_FLAG_RXNE));
	    tmp=SPI_I2S_ReceiveData(dev->SPIx); // (void) dev->SPIx->DR;
	}

	// Transfer response data in
	while (rxcount--){
	    while (SPI_I2S_GetFlagStatus(dev->SPIx, SPI_I2S_FLAG_TXE) == RESET);  //while (!(dev->SPIx->SR & SPI_I2S_FLAG_TXE));
	    SPI_I2S_SendData(dev->SPIx, 0); // dev->SPIx->DR = 0;
	    while (SPI_I2S_GetFlagStatus(dev->SPIx, SPI_I2S_FLAG_RXNE) == RESET); //while (!(dev->SPIx->SR & SPI_I2S_FLAG_RXNE));
	    *rxbuf++ = SPI_I2S_ReceiveData(dev->SPIx); // dev->SPIx->DR;
	}

	// Wait until the transfer is complete
//	while (dev->SPIx->SR & SPI_I2S_FLAG_BSY);
        tmp = 0;

	return tmp;
}



uint32_t spi_tx(const spi_dev *dev, const void *buf, uint32_t len) {
    uint32_t txed = 0;
    uint8_t byte_frame = spi_dff(dev) == SPI_DataSize_8b;
    while (spi_is_tx_empty(dev) && (txed < len)) {
        if (byte_frame) {
            dev->SPIx->DR = ((const uint8_t*)buf)[txed++];
        } else {
            dev->SPIx->DR = ((const uint16_t*)buf)[txed++];
        }
    }
    return txed;
}
