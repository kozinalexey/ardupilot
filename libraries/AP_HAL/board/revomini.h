#define AP_HAL_BOARD_DRIVER AP_HAL_REVOMINI
#define HAL_BOARD_NAME "REVOMINI"

#define HAL_CPU_CLASS HAL_CPU_CLASS_150
#define HAL_OS_POSIX_IO 0
#define HAL_STORAGE_SIZE            4096 // EEPROM size

#define HAL_BARO_DEFAULT HAL_BARO_MS5611_I2C
#define HAL_BARO_MS5611_I2C_BUS 0
#define HAL_BARO_MS5611_I2C_ADDR 0x77

#define HAL_COMPASS_DEFAULT HAL_COMPASS_HMC5843
#define HAL_COMPASS_HMC5843_I2C_BUS 0
#define HAL_COMPASS_HMC5843_I2C_EXT_BUS 2 // external compass on soft I2C
#define HAL_COMPASS_HMC5843_I2C_ADDR 0x1E

#define HAL_SERIAL0_BAUD_DEFAULT 115200
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_NONE

#define HAL_INS_DEFAULT HAL_INS_MPU60XX_SPI
#define HAL_INS_DEFAULT_ROTATION  ROTATION_YAW_180 // ROTATION_ROLL_180 // ROTATION_NONE
#define HAL_INS_MPU60x0_NAME "mpu6000"
#define Invensense_DRDY_PIN BOARD_MPU6000_DRDY_PIN 

#define HAL_DATAFLASH_NAME "dataflash"
#define HMC5883_DRDY_PIN BOARD_HMC5883_DRDY_PIN


# define HAL_GPIO_A_LED_PIN        36  // BLUE
//# define HAL_GPIO_B_LED_PIN        37  // YELLOW OPTIONAL (not included)
//# define HAL_GPIO_B_LED_PIN        26   //  EXTI_RFM22B /
//# define HAL_GPIO_B_LED_PIN        103  //  RFM22 CS    /  already occupied by BOARD_SPEKTRUM_**_PIN
# define HAL_GPIO_B_LED_PIN        9      //  frequency select - resistor to VCC or ground
# define HAL_GPIO_C_LED_PIN        105 // RED

# define HAL_GPIO_LED_ON           LOW
# define HAL_GPIO_LED_OFF          HIGH


#undef TOSHIBA_LED_I2C_BUS // someone placed this not in board config
#define TOSHIBA_LED_I2C_ADDR 0x55    // default I2C bus address
#define TOSHIBA_LED_I2C_BUS  2       // external I2C

/*
#define EXTERNAL_LED_GPS          28    // GPS LED - AN10
#define EXTERNAL_LED_ARMED        29    // Armed LED - AN11
#define EXTERNAL_LED_MOTOR1       30    // Motor1 LED - AN8
#define EXTERNAL_LED_MOTOR2       31    // Motor2 LED - AN12
*/