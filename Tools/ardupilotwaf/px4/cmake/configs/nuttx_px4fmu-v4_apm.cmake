include(configs/nuttx_px4fmu-common_apm)

list(APPEND config_module_list
    drivers/boards/px4fmu-v4
    drivers/pwm_input
    modules/uavcan
    lib/rc
)

list(APPEND config_extra_libs
    uavcan
    uavcan_stm32_driver
)
