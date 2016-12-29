#ifndef USB_STM32_H
#define USB_STM32_H

#include <stm32f4xx.h>
#include <gpio_hal.h>
#include <usart.h>

#include "../STM32_USB_Driver/usb_bsp.h"
#include "../STM32_USB_Driver/usb_regs.h"
#include "../STM32_USB_Driver/usbd_conf.h"
#include "../STM32_USB_Driver/usbd_usr.h"
#include "../STM32_USB_Driver/usbd_desc.h"
#include "../STM32_USB_Driver/usbd_cdc_core.h"
#include "../STM32_USB_Driver/usbd_ioreq.h"
#include "../STM32_USB_Driver/usbd_req.h"
#include "../STM32_USB_Driver/usbd_core.h"

#include "../STM32_USB_Driver/ring_buff.h"
#include "../STM32_USB_Driver/min_max.h"




#define USBD_MANUFACTURER_STRING        "RevoMini"
#define USBD_PRODUCT_FS_STRING          "3DR Virtual COM"
#define USBD_SERIALNUMBER_FS_STRING     "00000000050C"
#define USBD_CONFIGURATION_FS_STRING    "VCP Config"
#define USBD_INTERFACE_FS_STRING        "VCP Interface"
  
#define USBD_VID                        0x26ac
#define USBD_PID                        0x0011
#define USBD_LANGID_STRING              0x409
    
#define USB_RXFIFO_SIZE 256
#define USB_TXFIFO_SIZE 256



#define USB_IRQ		OTG_FS_IRQn
#define USB_AF		GPIO_AF_OTG1_FS
#define USB_CLOCK	RCC_AHB2Periph_OTG_FS


/* USB D-/D+ (DM/DP) */
#define DM_PIN_PORT   		_GPIOA  //PA11
#define DM_PIN_PIN   		11
#define DP_PIN_PORT   		_GPIOA  //PA12
#define DP_PIN_PIN   		12  //PA12


/* Exported typef ------------------------------------------------------------*/
/* The following structures groups all needed parameters to be configured for the 
   ComPort. These parameters can modified on the fly by the host through CDC class
   command class requests. */
typedef struct
{
  uint32_t bitrate;
  uint8_t  format;
  uint8_t  paritytype;
  uint8_t  datatype;
} LINE_CODING;

/* USB Attribute Data Structure */
typedef struct {
	uint8_t	preempt_prio;				/* pre-emption priority for the IRQ channel */
	uint8_t	sub_prio;						/* subpriority level for the IRQ channel  */
	uint8_t  use_present_pin;
	const gpio_dev *present_port;
	uint16_t present_pin;
	char *description;
	char *manufacturer;
	char *serial_number;
	char *configuration;
	char *interface;
} usb_attr_t;

#define I_USB_CLEAR		1
#define I_USB_CONNECTED		2
#define I_USB_GETATTR		3
#define I_USB_SETATTR		4

#ifdef __cplusplus
  extern "C" {
#endif

/* functions */
int usb_open(void);
int usb_close(void);
int usb_ioctl(int request, void * ctl);
int usb_write(uint8_t *buf, unsigned int nbytes);
int usb_read(void  * buf, unsigned int nbytes);
void usb_default_attr(usb_attr_t *attr);
int usb_configure(usb_attr_t * attr);
uint8_t usb_getc(void);
uint32_t usb_data_available(void);
uint16_t usb_tx_pending(void);
void usb_putc(uint8_t byte);
void usb_reset_rx(void);
void usb_reset_tx(void);


#define DEFAULT_CONFIG                  0
#define OTHER_CONFIG                    1

#ifdef __cplusplus
  }
#endif


/* Get the total number of data/space bytes available */
unsigned VCP_DataAvail(void);
unsigned VCP_SpaceAvail(void);

/* Get the number of contiguous data/space bytes available */
unsigned VCP_DataAvailContig(void);
unsigned VCP_SpaceAvailContig(void);

/* Get/put data from/to contiguous area */
unsigned VCP_GetContig(void* buff, unsigned max_len);
unsigned VCP_PutContig(void const* buff, unsigned len);

/* Get/put as much as possible */
unsigned VCP_Get(void* buff, unsigned max_len);
unsigned VCP_Put(void const* buff, unsigned len);

/* Returns pointer to contiguous input data area */
static inline uint8_t const* VCP_DataPtr(void)
{
        return USB_Rx_Buffer + USB_Rx_buff_tail;
}

/* Returns pointer to contiguous output free space area */
static inline uint8_t* VCP_SpacePtr(void)
{
        return USB_Tx_Buffer + USB_Tx_buff_head;
}

/* Mark data as read */
void VCP_MarkRead(unsigned sz);

/* Mark space as written */
void VCP_MarkWritten(unsigned sz);

int usb_periphcfg(FunctionalState state);

#endif
