#pragma once

#include <AP_ExternalAHRS/AP_ExternalAHRS_config.h>

#if AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED

#include <stdint.h>

namespace InertialLabs::Protocol::Transport {

constexpr uint8_t MAX_MESSAGE_TYPE_VALUE = 1;
constexpr uint8_t PAYLOAD_LENGTH_SIZE = 2;
constexpr uint16_t MAX_PAYLOAD_LENGTH = 65530;
constexpr uint8_t CHECKSUM_SIZE = 2;

/// @note messageType(1 byte) + dataIdentifier(1 byte) +
/// messageLength(2 byte) + checksum(2 byte) = 6 byte
const uint16_t PACKAGE_ADDITIONAL_PARAMS_SIZE = 6;

} // namespace InertialLabs::Protocol::Transport

#endif // AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED
