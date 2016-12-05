
#include "i2c_soft.h"

// Software I2C driver
// Can be configured to use any suitable pins

static uint16_t               scl_pin,   sda_pin;
static volatile GPIO_TypeDef *scl_port, *sda_port;
static volatile uint16_t i2cErrorCount = 0;


#define SCL_H_NW      {scl_port->BSRRL = scl_pin; }
#define SCL_H         {scl_port->BSRRL = scl_pin; while(!SCL_read) /* wait for extended clock done */ ; I2C_delay();}
#define SCL_L         {scl_port->BSRRH = scl_pin; I2C_delay();}

#define SDA_H         {sda_port->BSRRL = sda_pin; I2C_delay();}
#define SDA_L         {sda_port->BSRRH = sda_pin; I2C_delay();}

#define SCL_read      ((scl_port->IDR & scl_pin)!=0)
#define SDA_read      ((sda_port->IDR & sda_pin)!=0)


static void delay_10us(){
    stopwatch_delay_us(10);
}

static void I2C_delay(void)
{

    stopwatch_delay_us(2); // 250kHz
/*
    volatile int i = 7;
    while (i) {
        i--;
    }
*/
}

static bool I2C_Start(void)
{
    SDA_H;
    SCL_H_NW;
    
    if (!SDA_read)       return false;
    SDA_L;
    if (SDA_read)        return false;
    SCL_L;
    return true;
}

static void I2C_Stop(void)
{
    SCL_L;
    SDA_L;
    SCL_H;
    SDA_H;
}

static void I2C_Ack(void)
{
    SCL_L;
    SDA_L;
    SCL_H;
    SCL_L;
}

static void I2C_NoAck(void)
{
    SCL_L;
    SDA_H;
    SCL_H;
    SCL_L;
}

static bool I2C_WaitAck(void)
{
    SCL_L;
    SDA_H;
    SCL_H;
    if (SDA_read) {
        SCL_L;
        return false;
    }
    SCL_L;
    return true;
}

static void I2C_SendByte(uint8_t bt)
{
    uint8_t i = 8;
    while (i--) {
        SCL_L;
        if (bt & 0x80) { SDA_H; }
        else           { SDA_L; }
        bt <<= 1;
        SCL_H;
    }
    SCL_L;
}

static uint8_t I2C_ReceiveByte(void)
{
    uint8_t i = 8;
    uint8_t bt = 0;

    SDA_H;
    while (i--) {
        bt <<= 1;
        SCL_L;
        SCL_H;
        if (SDA_read) bt |= 0x01;
    }
    SCL_L;
    return bt;
}

void s_i2cInit(const gpio_dev *sda_dev, uint8_t sda_bit, const gpio_dev *scl_dev, uint8_t scl_bit)
{
    gpio_set_mode(scl_dev, scl_bit, GPIO_OUTPUT_OD_PU);
    gpio_set_mode(sda_dev, sda_bit, GPIO_OUTPUT_OD_PU);

    sda_port = sda_dev->GPIOx;
    sda_pin  = 1<<sda_bit;

    scl_port = scl_dev->GPIOx;
    scl_pin  = 1<<scl_bit;
}

uint32_t  s_i2cWriteBuffer( uint8_t addr, uint8_t reg, uint8_t len, const uint8_t *data)
{

    int i;
    if (!I2C_Start()) {
        i2cErrorCount++;
        return I2C_ERROR;
    }
    I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck()) {
        I2C_Stop();
        return I2C_NO_DEVICE;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    for (i = 0; i < len; i++) {
        I2C_SendByte(data[i]);
        if (!I2C_WaitAck()) {
            I2C_Stop();
            i2cErrorCount++;
            return I2C_ERROR;
        }
    }
    I2C_Stop();
    return I2C_OK;
}

uint32_t s_i2cWrite( uint8_t addr, uint8_t reg, uint8_t data)
{
    if (!I2C_Start()) {
        return I2C_ERROR;
    }
    I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck()) {
        I2C_Stop();
        i2cErrorCount++;
        return I2C_NO_DEVICE;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    I2C_SendByte(data);
    I2C_WaitAck();
    I2C_Stop();
    return I2C_OK;
}

uint32_t s_i2cRead( uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!I2C_Start()) {
        return I2C_ERROR;
    }
    I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck()) {
        I2C_Stop();
        i2cErrorCount++;
        return I2C_NO_DEVICE;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(addr << 1 | I2C_Direction_Receiver);
    I2C_WaitAck();
    uint8_t cnt=0;
    while (len) {
        buf[cnt++] = I2C_ReceiveByte();
        if (len == 1) I2C_NoAck();
        else          I2C_Ack();

        len--;
    }
    I2C_Stop();
    return I2C_OK;
}


uint32_t s_i2cTransfer(uint8_t  addr, uint8_t  send_len, const uint8_t *send, uint8_t len, uint8_t *buf){
    if (!I2C_Start()) {
        return I2C_ERROR;
    }
    I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck()) {
        I2C_Stop();
        i2cErrorCount++;
        return I2C_NO_DEVICE;
    }
    uint8_t cnt=0;
    while(send_len++){
        I2C_SendByte(send[cnt++]);
        if (!I2C_WaitAck()) {
            I2C_Stop();
            i2cErrorCount++;
            return I2C_ERROR;
        }
    }

    I2C_Start();
    I2C_SendByte(addr << 1 | I2C_Direction_Receiver);
    I2C_WaitAck();
    cnt=0;
    while (len) {
        buf[cnt++] = I2C_ReceiveByte();
        if(len == 1) I2C_NoAck();
        else         I2C_Ack();
        len--;
    }
    I2C_Stop();
    return I2C_OK;

}

uint16_t s_i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}


void s_i2c_bus_reset(void) {

again:
    /* Wait for any clock stretching to finish */
    while (!SCL_read) // device can output 1 so check clock first
            ;
    delay_10us();       // 50kHz

    while (!SDA_read) {
        /* Wait for any clock stretching to finish */
        while (!SCL_read)
            ;
        delay_10us();   // 50kHz

        /* Pull low */
        SCL_L;
        delay_10us();

        /* Release high again */
        SCL_L;
        delay_10us();
    }

    /* Generate start then stop condition */
    SDA_L;
    delay_10us();
    SCL_L;
    delay_10us();
    SCL_H;
    delay_10us();
    SDA_H;
    
    uint32_t rtime = stopwatch_getticks();
    uint32_t dt    = us_ticks * 50; // 50uS

    while ((stopwatch_getticks() - rtime) < dt) {
        if (!SCL_read)  goto again; // any SCL activity after STOP
    }
}


