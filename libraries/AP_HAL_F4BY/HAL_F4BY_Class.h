#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_F4BY

#include "AP_HAL_F4BY.h"
#include "AP_HAL_F4BY_Namespace.h"
#include <systemlib/visibility.h>
#include <systemlib/perf_counter.h>

class HAL_F4BY : public AP_HAL::HAL {
public:
    HAL_F4BY();
    void run(int argc, char* const argv[], Callbacks* callbacks) const override;
};

void hal_f4by_set_priority(uint8_t priority);

#endif // CONFIG_HAL_BOARD == HAL_BOARD_F4BY
