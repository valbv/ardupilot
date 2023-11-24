/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  support for serial connected AHRS systems
 */

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include <AP_ExternalAHRS/AP_ExternalAHRS_config.h>

#if AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED

#include "AP_ExternalAHRS_InertialLabs.h"

#include <AP_ExternalAHRS/inertialLabs/payload/Handler.h>
#include <AP_ExternalAHRS/inertialLabs/transportProtocol/Handler.h>

#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_HAL/utility/functor.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include <stddef.h>

extern const AP_HAL::HAL &hal;

namespace {

constexpr uint16_t BUFFER_SIZE = 500;
constexpr int8_t INVALID_PORT = -1;

} // namrspace

struct AP_ExternalAHRS_InertialLabs::Impl {
    Impl(AP_ExternalAHRS_InertialLabs * externalAHRS_InertialLabs)
        : self{externalAHRS_InertialLabs}
        , protocol_handler{FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_InertialLabs::Impl::process_payload, void, char*, size_t)}
    {
        AP_SerialManager &serial_manager = AP::serialmanager();
        uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
        if (!uart) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS no UART");
            return;
        }

        baudrate = serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
        port_number = serial_manager.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

        const bool isOk = hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_InertialLabs::Impl::update_thread, void),
                                                                           "AHRS",
                                                                           2048,
                                                                           AP_HAL::Scheduler::PRIORITY_SPI,
                                                                           0);
        if (!isOk) {
            AP_HAL::panic("Failed to start ExternalAHRS update thread");
        }

        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS initialised");
    }

    void update_thread() {
        // Open port in the thread
        uart->begin(baudrate, 1024, 512);

        is_setup_complete = true;
        while (true) {
            if (!check_uart()) {
                hal.scheduler->delay(1);
            }
        }
    }

    /*
    check the UART for more data
    returns true if the function should be called again straight away
    */
    bool check_uart() {
        if (!is_setup_complete) {
            return false;
        }

        WITH_SEMAPHORE(self->state.sem);
        uint32_t available_bytes_count = uart->available();
        if (available_bytes_count == 0) {
            return false;
        }

        const size_t read_bytes_count = uart->read(buffer, MIN(available_bytes_count, BUFFER_SIZE));
        if (read_bytes_count <= 0) {
            return false;
        }

        protocol_handler.parse(InertialLabs::Data{reinterpret_cast<char*>(buffer), read_bytes_count});
        return true;
    }

    void process_payload(char *buf, size_t size) {
        last_package_ms = AP_HAL::millis();
        last_package = payload_handler.parse(InertialLabs::Data{buf, size});

        WITH_SEMAPHORE(self->state.sem);

        self->state.accel = Vector3f{static_cast<float>(last_package.accel_x * 1.0e-6),
                                     static_cast<float>(last_package.accel_y * 1.0e-6),
                                     static_cast<float>(last_package.accel_z * 1.0e-6)};
        self->state.gyro = Vector3f{static_cast<float>(last_package.gyro_x * 1.0e-5),
                                    static_cast<float>(last_package.gyro_y * 1.0e-5),
                                    static_cast<float>(last_package.gyro_z * 1.0e-5)};
        self->state.quat = Quaternion{static_cast<float>(last_package.quaternion_0),
                                      static_cast<float>(last_package.quaternion_1),
                                      static_cast<float>(last_package.quaternion_2),
                                      static_cast<float>(last_package.quaternion_3)};
        self->state.have_quaternion = true;
        self->state.velocity = Vector3f{static_cast<float>(last_package.east_speed * 1.0e-2),
                                        static_cast<float>(last_package.north_speed * 1.0e-2),
                                        static_cast<float>(last_package.vertical_speed * 1.0e-2)};
        self->state.have_velocity = true;
        self->state.location = Location{last_package.gnss_latitude,
                                        last_package.gnss_longitude,
                                        last_package.gnss_altitude,
                                        Location::AltFrame::ABSOLUTE};
        self->state.have_location = true;

#if AP_BARO_EXTERNALAHRS_ENABLED
        AP_ExternalAHRS::baro_data_message_t baro;
        baro.instance = 0;
        baro.pressure_pa = static_cast<float>(last_package.pressure * 2);
        baro.temperature = static_cast<float>(last_package.temperature * 0.1);
        AP::baro().handle_external(baro);
#endif

#if AP_COMPASS_EXTERNALAHRS_ENABLED
        AP_ExternalAHRS::mag_data_message_t mag;
        mag.field = Vector3f{static_cast<float>(last_package.mag_x * 1.0e10),
                             static_cast<float>(last_package.mag_y * 1.0e10),
                             static_cast<float>(last_package.mag_z * 1.0e10)};
        AP::compass().handle_external(mag);
#endif

        AP_ExternalAHRS::ins_data_message_t ins;
        ins.accel = self->state.accel;
        ins.gyro = self->state.gyro;
        ins.temperature = static_cast<float>(last_package.temperature * 0.1);
        AP::ins().handle_external(ins);

        AP::logger().WriteStreaming("InertialLabsEAHRS", // name
                                    "TimeUS,MagX,MagY,MagZ,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,Q1,Q2,Q3,Q4,VelN,VelE,VelV,Lat,Lon,Alt,Temp,Pres", // labels
                                    "sGGGoooEEE----nnnDUmdP", // units
                                    "F0000000000000000GG000", // mults
                                    "QffffffffffffffffLLfff", // format
                                    AP_HAL::micros64(),
                                    last_package.mag_x,
                                    last_package.mag_y,
                                    last_package.mag_z,
                                    last_package.accel_x,
                                    last_package.accel_y,
                                    last_package.accel_z,
                                    last_package.gyro_x,
                                    last_package.gyro_y,
                                    last_package.gyro_z,
                                    last_package.quaternion_0,
                                    last_package.quaternion_1,
                                    last_package.quaternion_2,
                                    last_package.quaternion_3,
                                    last_package.north_speed,
                                    last_package.east_speed,
                                    last_package.vertical_speed,
                                    last_package.latitude,
                                    last_package.longitude,
                                    last_package.altitude,
                                    last_package.pressure * 2,
                                    last_package.temperature * 0.1);
    }

public:
    AP_ExternalAHRS_InertialLabs *self{nullptr};

    InertialLabs::Protocol::Transport::Handler protocol_handler;
    InertialLabs::Payload::Handler payload_handler;

    AP_HAL::UARTDriver *uart{nullptr};
    int8_t port_number{INVALID_PORT};
    uint32_t baudrate{0};

    bool is_setup_complete{false};

    uint8_t buffer[BUFFER_SIZE];
    InertialLabs::Payload::Sensor_Data last_package;

    uint32_t last_package_ms{0};
};

AP_ExternalAHRS_InertialLabs::AP_ExternalAHRS_InertialLabs(AP_ExternalAHRS *_frontend,
                                                           AP_ExternalAHRS::state_t &_state)
    : AP_ExternalAHRS_backend{_frontend, _state}
    , m_impl{new Impl(this)}
{}

AP_ExternalAHRS_InertialLabs::~AP_ExternalAHRS_InertialLabs() {
    delete m_impl;
}

int8_t AP_ExternalAHRS_InertialLabs::get_port() const {
    if (!m_impl->uart) {
        return INVALID_PORT;
    }
    return m_impl->port_number;
};

const char* AP_ExternalAHRS_InertialLabs::get_name() const {
    return "InertialLabs INS";
}

bool AP_ExternalAHRS_InertialLabs::healthy() const {
    const uint32_t now = AP_HAL::millis();
    return now - m_impl->last_package_ms < 40;
}

bool AP_ExternalAHRS_InertialLabs::initialised() const {
    return m_impl->is_setup_complete && m_impl->last_package_ms != 0;
}

bool AP_ExternalAHRS_InertialLabs::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const {
    if (!m_impl->is_setup_complete) {
        hal.util->snprintf(failure_msg, failure_msg_len, "InertialLabs IMU setup failed");
        return false;
    }
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "InertialLabs IMU unhealthy");
        return false;
    }

    return true;
}

void AP_ExternalAHRS_InertialLabs::get_filter_status(nav_filter_status &status) const {
    memset(&status, 0, sizeof(status));

    /// @todo check the relevance of these flags
    status.flags.initalized = initialised();
    if (healthy()) {
        status.flags.attitude = true;
        status.flags.vert_vel = true;
        status.flags.vert_pos = true;

        status.flags.horiz_vel = true;
        status.flags.horiz_pos_rel = true;
        status.flags.horiz_pos_abs = true;
        status.flags.pred_horiz_pos_rel = true;
        status.flags.pred_horiz_pos_abs = true;
        status.flags.using_gps = true;
    }
}

// send an EKF_STATUS message to GCS
/// @todo Does it mean need to enable EKF or it's already enabled?
void AP_ExternalAHRS_InertialLabs::send_status_report(GCS_MAVLINK &link) const {
    if (!initialised()) {
        return;
    }

    uint16_t flags = 0;
    nav_filter_status filterStatus;
    get_filter_status(filterStatus);
    if (filterStatus.flags.attitude) {
        flags |= EKF_ATTITUDE;
    }
    if (filterStatus.flags.horiz_vel) {
        flags |= EKF_VELOCITY_HORIZ;
    }
    if (filterStatus.flags.vert_vel) {
        flags |= EKF_VELOCITY_VERT;
    }
    if (filterStatus.flags.horiz_pos_rel) {
        flags |= EKF_POS_HORIZ_REL;
    }
    if (filterStatus.flags.horiz_pos_abs) {
        flags |= EKF_POS_HORIZ_ABS;
    }
    if (filterStatus.flags.vert_pos) {
        flags |= EKF_POS_VERT_ABS;
    }
    if (filterStatus.flags.terrain_alt) {
        flags |= EKF_POS_VERT_AGL;
    }
    if (filterStatus.flags.const_pos_mode) {
        flags |= EKF_CONST_POS_MODE;
    }
    if (filterStatus.flags.pred_horiz_pos_rel) {
        flags |= EKF_PRED_POS_HORIZ_REL;
    }
    if (filterStatus.flags.pred_horiz_pos_abs) {
        flags |= EKF_PRED_POS_HORIZ_ABS;
    }
    if (!filterStatus.flags.initalized) {
        flags |= EKF_UNINITIALIZED;
    }
}

void AP_ExternalAHRS_InertialLabs::update() {
    m_impl->check_uart();
}

#endif // AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED
