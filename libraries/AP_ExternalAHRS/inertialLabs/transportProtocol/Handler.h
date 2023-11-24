#pragma once

#include <AP_ExternalAHRS/AP_ExternalAHRS_config.h>

#if AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED

#include <AP_ExternalAHRS/inertialLabs/Data.h>

#include <AP_HAL/utility/functor.h>

#include <stddef.h>
#include <stdint.h>

namespace InertialLabs::Protocol::Transport {

enum class MessageType : uint8_t {
    Command = 0,
    TransferredData = 1,
};

class Handler{
public:
    FUNCTOR_TYPEDEF(process_callback_type, void, char*, size_t);

    explicit Handler(process_callback_type process_callback);
    ~Handler();

    void parse(const Data &data_chunk);
    void reset_parse_params();

private:
    struct Impl;
    Impl *m_impl;
};

} // namespace InertialLabs::Protocol::Transport

#endif // AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED
