#include <AP_ExternalAHRS/AP_ExternalAHRS_config.h>

#if AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED

#include "Handler.h"

#include "Constants.h"
#include "Utils.h"

#include <AP_HAL/HAL.h>

extern const AP_HAL::HAL &hal;

namespace {

enum class Parse_Stage {
    HEADER_1,
    HEADER_2,
    MESSAGE_TYPE,
    DATA_IDENTIFIER,
    PAYLOAD_LENGTH,
    PAYLOAD_DATA,
    CHECKSUM,
};

} // namespace

namespace InertialLabs::Protocol::Transport {

struct Handler::Impl {
    Impl(process_callback_type callback)
        : process_callback(callback)
    {}

    void reset_parse_params() {
        message_type = 0;
        data_identifier = 0;

        payload_length_buffer_index = 0;
        payload_length = 0;

        payload_buffer_index = 0;

        checksum_buffer_index = 0;
        checksum = 0;
    }

    void parse(const Data &data_chunk) {
        for (size_t i = 0; i < data_chunk.size; ++i) {
            const uint8_t byte = data_chunk.data[i];
            switch (state) {
                case Parse_Stage::HEADER_1:
                    if (0xAA == byte) {
                        reset_parse_params();
                        state = Parse_Stage::HEADER_2;
                    }
                    break;
                case Parse_Stage::HEADER_2:
                    state = 0x55 == byte
                                ? Parse_Stage::MESSAGE_TYPE
                                : Parse_Stage::HEADER_1;
                    break;
                case Parse_Stage::MESSAGE_TYPE:
                    if (byte > MAX_MESSAGE_TYPE_VALUE) {
                        hal.console->printf("Inertial Labs transport protocol: Parsing error. Incorrect payload length\n");
                        state = Parse_Stage::HEADER_1;
                        break;
                    }
                    message_type = byte;
                    state = Parse_Stage::DATA_IDENTIFIER;
                    break;
                case Parse_Stage::DATA_IDENTIFIER:
                    data_identifier = byte;
                    state = Parse_Stage::PAYLOAD_LENGTH;
                    break;
                case Parse_Stage::PAYLOAD_LENGTH:
                {
                    payload_length_buffer[payload_length_buffer_index] = byte;
                    if (++payload_length_buffer_index < PAYLOAD_LENGTH_SIZE) {
                        break;
                    }

                    payload_length_buffer_index = 0;
                    const uint16_t payload_with_params_length = *reinterpret_cast<uint16_t*>(payload_length_buffer);
                    if (payload_with_params_length < PACKAGE_ADDITIONAL_PARAMS_SIZE) {
                        hal.console->printf("Inertial Labs transport protocol: Parsing error. Invalid payload with params length\n");
                        state = Parse_Stage::HEADER_1;
                        break;
                    }
                    payload_length = payload_with_params_length - PACKAGE_ADDITIONAL_PARAMS_SIZE;

                    state = Parse_Stage::PAYLOAD_DATA;
                    break;
                }
                case Parse_Stage::PAYLOAD_DATA:
                    payload_buffer[payload_buffer_index] = byte;
                    if (++payload_buffer_index < payload_length) {
                        break;
                    }

                    payload_buffer_index = 0;
                    state = Parse_Stage::CHECKSUM;
                    break;
                case Parse_Stage::CHECKSUM:
                {
                    checksum_buffer[checksum_buffer_index] = byte;
                    if (++checksum_buffer_index < CHECKSUM_SIZE) {
                        break;
                    }

                    checksum = *reinterpret_cast<uint16_t*>(checksum_buffer);
                    uint16_t calculated_checksum = message_type +
                                                  data_identifier +
                                                  calculate_checksum(payload_length_buffer, PAYLOAD_LENGTH_SIZE) +
                                                  calculate_checksum(payload_buffer, payload_length);

                    if (checksum != calculated_checksum) {
                        hal.console->printf("Inertial Labs transport protocol: Parsing error. Invalid checksum of package\n");
                        state = Parse_Stage::HEADER_1;
                        break;
                    }

                    process_callback(reinterpret_cast<char*>(payload_buffer), payload_length);

                    state = Parse_Stage::HEADER_1;
                    break;
                }
                default:
                    hal.console->printf("Inertial Labs transport protocol: Invalid data parse state\n");
                    state = Parse_Stage::HEADER_1;
            }
        }
    }

public:
    process_callback_type process_callback;

    Parse_Stage state{Parse_Stage::HEADER_1};
    uint8_t message_type{0};
    uint8_t data_identifier{0};

    uint16_t payload_length_buffer_index{0};
    uint8_t payload_length_buffer[PAYLOAD_LENGTH_SIZE];
    uint16_t payload_length{0};

    uint16_t payload_buffer_index{0};
    uint8_t payload_buffer[MAX_PAYLOAD_LENGTH];

    uint16_t checksum_buffer_index{0};
    uint8_t checksum_buffer[CHECKSUM_SIZE];
    uint16_t checksum{0};
};

Handler::Handler(process_callback_type process_callback)
    : m_impl{new Impl(process_callback)}
{}

Handler::~Handler() {
    delete m_impl;
}

void Handler::parse(const Data & data_chunk) {
    m_impl->parse(data_chunk);
}

void Handler::reset_parse_params() {
    m_impl->state = Parse_Stage::HEADER_1;
    m_impl->reset_parse_params();
}

} // namespace InertialLabs::Protocol::Transport

#endif // AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED
