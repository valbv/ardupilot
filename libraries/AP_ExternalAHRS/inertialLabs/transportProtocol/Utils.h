#pragma once

#include <AP_ExternalAHRS/AP_ExternalAHRS_config.h>

#if AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED

#include <stdint.h>

namespace InertialLabs::Protocol::Transport {

uint16_t calculate_checksum(const uint8_t *buffer, uint16_t size);

} // namespace InertialLabs::Protocol::Transport

#endif // AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED
