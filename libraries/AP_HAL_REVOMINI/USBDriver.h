
#ifndef __AP_HAL_REVOMINI_USBDRIVER_H__
#define __AP_HAL_REVOMINI_USBDRIVER_H__

#include <AP_HAL_REVOMINI/AP_HAL_REVOMINI.h>

#include <usb.h>
#include <gpio_hal.h>

#define DEFAULT_TX_TIMEOUT 10000

namespace REVOMINI {

class USBDriver : public AP_HAL::UARTDriver  {
public:
    USBDriver(bool usb);

  /* REVOMINI implementations of UARTDriver virtual methods */
  void begin(uint32_t b);
  void begin(uint32_t b, uint16_t rxS, uint16_t txS);
  void end();
  void flush();
  bool is_initialized(){ return _initialized; }

  void set_blocking_writes(bool blocking);

  bool tx_pending();

  /* REVOMINI implementations of Stream virtual methods */
  uint32_t available() override;
  uint32_t txspace() override;
  int16_t read() override;

  /* Empty implementations of Print virtual methods */
  size_t write(uint8_t c);
  size_t write(const uint8_t *buffer, size_t size);

private:
    uint8_t _usb_present;
    bool _initialized;
};

}

#endif // __AP_HAL_EMPTY_UARTDRIVER_H__
