#include <AP_ExternalAHRS/AP_ExternalAHRS_config.h>

#if AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED

#include "Handler.h"

#include <AP_HAL/HAL.h>

#include <stddef.h>

extern const AP_HAL::HAL &hal;

namespace InertialLabs::Payload {

constexpr size_t PAYLOAD_SIZE = sizeof(HEADER) + sizeof(Sensor_Data);

struct Handler::Impl {
    Sensor_Data *sensor_data{nullptr};
};

Handler::Handler()
    : m_impl{new Impl()}
{}

Handler::~Handler() {
    delete m_impl;
}

Sensor_Data Handler::parse(const Data &data) {
    if (data.size != PAYLOAD_SIZE) {
        hal.console->printf("Inertial Labs payload: Invalid payload size\n");
        return {};
    }

    const bool is_header_different = memcmp(HEADER, data.data, sizeof(HEADER));
    if (is_header_different) {
        hal.console->printf("Inertial Labs payload: Invalid payload header\n");
        return {};
    }

    m_impl->sensor_data = reinterpret_cast<Sensor_Data*>(&data.data[sizeof(HEADER)]);

    return *m_impl->sensor_data;
}

} // namespace InertialLabs::Payload

#endif // AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED
