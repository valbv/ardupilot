#pragma once

#include <AP_ExternalAHRS/AP_ExternalAHRS_config.h>

#if AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED

#include <stddef.h>

namespace InertialLabs {

struct Data {
    char *data;
    size_t size;
};

} // namespace InertialLabs

#endif // AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED
