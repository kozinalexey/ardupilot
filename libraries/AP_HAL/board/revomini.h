#define AP_HAL_BOARD_DRIVER AP_HAL_REVOMINI
#define HAL_BOARD_NAME "REVOMINI"

#define HAL_CPU_CLASS HAL_CPU_CLASS_150
#define HAL_OS_POSIX_IO 0
#define HAL_STORAGE_SIZE            4096 // EEPROM size

#define HAL_BARO_DEFAULT HAL_BARO_MS5611_I2C
#define HAL_BARO_MS5611_I2C_BUS 0
#define HAL_BARO_MS5611_I2C_ADDR 0x77
#define HAL_BARO_MS5611_I2C_BUS_EXT 2  // external baro on soft I2C



#define HAL_COMPASS_DEFAULT HAL_COMPASS_HMC5843
#define HAL_COMPASS_HMC5843_I2C_BUS 0
#define HAL_COMPASS_HMC5843_I2C_EXT_BUS 2 // external compass on soft I2C
#define HAL_COMPASS_HMC5843_I2C_ADDR 0x1E

#define HAL_SERIAL0_BAUD_DEFAULT 115200
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_NONE

#define HAL_INS_DEFAULT HAL_INS_MPU60XX_SPI
#define HAL_INS_DEFAULT_ROTATION  ROTATION_YAW_180 
#define HAL_INS_MPU60x0_NAME "mpu6000"
#define INVENSENSE_DRDY_PIN BOARD_MPU6000_DRDY_PIN 

#define HAL_DATAFLASH_NAME "dataflash"
#define HMC5883_DRDY_PIN BOARD_HMC5883_DRDY_PIN


# define HAL_GPIO_A_LED_PIN        36  // BLUE
//# define HAL_GPIO_B_LED_PIN        37  // YELLOW OPTIONAL (not included)
# define HAL_GPIO_B_LED_PIN        9      //  frequency select - resistor to VCC or ground
# define HAL_GPIO_C_LED_PIN        105 // RED

# define HAL_GPIO_LED_ON           LOW
# define HAL_GPIO_LED_OFF          HIGH

 # define PUSHBUTTON_PIN   254
 # define USB_MUX_PIN      -1
 # define BATTERY_VOLT_PIN     8   // Battery voltage on A0 (PC2) D8
 # define BATTERY_CURR_PIN     7   // Battery current on A1 (PC1) D7
 # define CONFIG_SONAR_SOURCE_ANALOG_PIN 254
 


#undef TOSHIBA_LED_I2C_BUS // someone placed this not in board config
#define TOSHIBA_LED_I2C_ADDR 0x55    // default I2C bus address
#define TOSHIBA_LED_I2C_BUS  2       // external I2C

/*
#define EXTERNAL_LED_GPS          28    // GPS LED - AN10
#define EXTERNAL_LED_ARMED        29    // Armed LED - AN11
#define EXTERNAL_LED_MOTOR1       30    // Motor1 LED - AN8
#define EXTERNAL_LED_MOTOR2       31    // Motor2 LED - AN12
*/

 #define CONFIG_IMU_TYPE   CONFIG_IMU_MPU6000
 #define CONFIG_SONAR_SOURCE SONAR_SOURCE_ANALOG_PIN
 #define MAGNETOMETER ENABLED
 #define CONFIG_BARO     HAL_BARO_MS5611

 #define AC_TERRAIN             DISABLED // no SD card
 #define OPTFLOW                DISABLED
 #define ADSB_ENABLED           DISABLED
 #define PRECISION_LANDING      DISABLED
 #define CONFIG_PUSHBUTTON      DISABLED
 #define CONFIG_RELAY           DISABLED
 #define RANGEFINDER_ENABLED    DISABLED
 #define SPRAYER                DISABLED
 #define EPM_ENABLED            DISABLED
 #define MOUNT                  DISABLED // don't fit to flash
 #define CLI_ENABLED            DISABLED
 
 // for debugging
 #define FRSKY_TELEM_ENABLED   DISABLED
 #define NAV_GUIDED            DISABLED
// #define AC_RALLY              DISABLED
// #define PROXIMITY_ENABLED     DISABLED
// #define CAMERA                DISABLED
// #define AC_FENCE              DISABLED
// #define AC_FENCE               DISABLED // causes compilation error
 #define NAV_GUIDED            DISABLED
 #define POSHOLD_ENABLED       DISABLED 

 #define LOGGING_ENABLED ENABLED

 