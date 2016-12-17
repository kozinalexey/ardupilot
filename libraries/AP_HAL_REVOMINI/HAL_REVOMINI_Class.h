
#ifndef __AP_HAL_REVOMINI_CLASS_H__
#define __AP_HAL_REVOMINI_CLASS_H__

#include <AP_HAL_REVOMINI/AP_HAL_REVOMINI.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_REVOMINI
#include <AP_Param/AP_Param.h>

#include "AP_HAL_REVOMINI_Namespace.h"
#include <wirish.h>
#include <hal.h>


class HAL_REVOMINI : public AP_HAL::HAL {
public:
    HAL_REVOMINI();
    void run(int argc, char * const argv[], Callbacks* callbacks) const override;

    void lateInit(); // 2nd stage

    static const struct AP_Param::GroupInfo var_info[];

    // parameters
    AP_Int8 _motor_layout;
    AP_Int8 _use_softserial;
};


// motor layouts
#define REVO_MOTORS_ARDUCOPTER 0
#define REVO_MOTORS_OPENPILOT 1
#define REVO_MOTORS_CLEANFLIGHT 2


#endif // __AP_HAL_REVOMINI_CLASS_H__
#endif // __HAL_BOARD_REVOMINI__
