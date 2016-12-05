
#ifndef __AP_HAL_REVOMINI_NAMESPACE_H__
#define __AP_HAL_REVOMINI_NAMESPACE_H__

/* While not strictly required, names inside the REVOMINI namespace are prefixed
 * with REVOMINI for clarity. (Some of our users aren't familiar with all of the
 * C++ namespace rules.)
 */

namespace REVOMINI {
    class REVOMINIUARTDriver;
    class USBDriver;
    class SPIDeviceManager;
    class SPIDevice;
    class REVOMINIAnalogSource;
    class REVOMINIAnalogIn;
    class REVOMINIStorage;
    class REVOMINIGPIO;
    class REVOMINIDigitalSource;
    class REVOMINIRCInput;
    class REVOMINIRCOutput;
    class Semaphore;
    class REVOMINIScheduler;
    class REVOMINIUtil;
    class REVOI2CDevice;
    class I2CDeviceManager;
    
//    class UART_softDriver;
    class SerialDriver;
}

#endif // __AP_HAL_REVOMINI_NAMESPACE_H__

