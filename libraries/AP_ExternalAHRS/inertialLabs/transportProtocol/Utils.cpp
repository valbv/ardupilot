#include <AP_ExternalAHRS/AP_ExternalAHRS_config.h>

#if AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED

#include "Utils.h"

namespace InertialLabs::Protocol::Transport {

uint16_t calculate_checksum(const uint8_t *buffer, uint16_t size) {
    uint16_t checksum{0};

    for (uint16_t i = 0; i < size; ++i)
    {
        checksum += buffer[i];
    }

    return checksum;
}

} // namespace InertialLabs::Protocol::Transport

#endif // AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED
