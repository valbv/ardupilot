#pragma once

#include <AP_ExternalAHRS/AP_ExternalAHRS_config.h>

#if AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED

#include <AP_ExternalAHRS/inertialLabs/Data.h>
#include <AP_ExternalAHRS/inertialLabs/payload/SensorData.h>

namespace InertialLabs::Payload {

class Handler{
public:
    Handler();
    ~Handler();

    Sensor_Data parse(const Data &data);

private:
    struct Impl;
    Impl *m_impl;
};

} // namespace InertialLabs::Payload

#endif // AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED
