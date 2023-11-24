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
  simulate InertialLabs serial AHRS
*/

#include "SIM_InertialLabs.h"

#include <AP_ExternalAHRS/inertialLabs/payload/SensorData.h>
#include <AP_ExternalAHRS/inertialLabs/transportProtocol/Utils.h>
#include <SITL/SITL.h>

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

namespace {
constexpr char PACKAGE_HEADER[] = {"\xAA\x55\x01\x95\xAE\x00"};

constexpr size_t FULL_PACKAGE_SIZE = 176;
char FULL_PACKAGE[FULL_PACKAGE_SIZE];

const ::InertialLabs::Payload::Sensor_Data * generate_payload(const SITL::sitl_fdm &fdm) {
    static struct ::InertialLabs::Payload::Sensor_Data pkt{};
    pkt.accel_x = fdm.xAccel;
    pkt.accel_y = fdm.yAccel;
    pkt.accel_z = fdm.zAccel;

    pkt.mag_x = fdm.bodyMagField.x*0.001;
    pkt.mag_y = fdm.bodyMagField.y*0.001;
    pkt.mag_z = fdm.bodyMagField.z*0.001;

    const float gyro_noise = 0.05;
    pkt.gyro_x = radians(fdm.rollRate + gyro_noise * rand_float());
    pkt.gyro_y = radians(fdm.pitchRate + gyro_noise * rand_float());
    pkt.gyro_z = radians(fdm.yawRate + gyro_noise * rand_float());

    float sigma, delta, theta;
    AP_Baro::SimpleAtmosphere(fdm.altitude * 0.001f, sigma, delta, theta);
    pkt.pressure = SSL_AIR_PRESSURE * delta * 0.001 + rand_float() * 0.01;

    pkt.quaternion_0 = fdm.quaternion.q1;
    pkt.quaternion_1 = fdm.quaternion.q2;
    pkt.quaternion_2 = fdm.quaternion.q3;
    pkt.quaternion_3 = fdm.quaternion.q4;

    pkt.latitude = fdm.latitude;
    pkt.longitude = fdm.longitude;
    pkt.altitude = fdm.altitude;

    pkt.east_speed = fdm.speedE;
    pkt.north_speed = fdm.speedN;
    pkt.vertical_speed = fdm.speedD;

    pkt.gnss_latitude = fdm.latitude;
    pkt.gnss_longitude = fdm.longitude;
    pkt.gnss_altitude = fdm.altitude;

    return &pkt;
}

} // namespace

namespace SITL {

/*
  get timeval using simulation time
 */
static void simulation_timeval(struct timeval *tv) {
    uint64_t now = AP_HAL::micros64();
    static uint64_t first_usec;
    static struct timeval first_tv;
    if (first_usec == 0) {
        first_usec = now;
        first_tv.tv_sec = AP::sitl()->start_time_UTC;
    }
    *tv = first_tv;
    tv->tv_sec += now / 1000000ULL;
    uint64_t new_usec = tv->tv_usec + (now % 1000000ULL);
    tv->tv_sec += new_usec / 1000000ULL;
    tv->tv_usec = new_usec % 1000000ULL;
}

void InertialLabs::send_packet() {
    using ::InertialLabs::Payload::HEADER;
    using ::InertialLabs::Payload::Sensor_Data;
    using ::InertialLabs::Protocol::Transport::calculate_checksum;

    struct timeval tv;
    simulation_timeval(&tv);

    const sitl_fdm &fdm = _sitl->state;

    memcpy(FULL_PACKAGE, PACKAGE_HEADER, sizeof(PACKAGE_HEADER));

    const uint8_t payload_header_offset = sizeof(PACKAGE_HEADER);
    memcpy(&FULL_PACKAGE[payload_header_offset], HEADER, sizeof(HEADER));

    const Sensor_Data * payload = generate_payload(fdm);
    const uint8_t payload_offset = payload_header_offset + sizeof(HEADER);
    memcpy(&FULL_PACKAGE[payload_offset], reinterpret_cast<const char*>(payload), sizeof(Sensor_Data));

    const uint16_t checksum = calculate_checksum(reinterpret_cast<const uint8_t*>(&FULL_PACKAGE[2]), FULL_PACKAGE_SIZE - 4);
    const uint8_t checksum_offset = payload_header_offset + sizeof(Sensor_Data);
    memcpy(&FULL_PACKAGE[checksum_offset], reinterpret_cast<const char*>(&checksum), sizeof(checksum));

    write_to_autopilot(FULL_PACKAGE, FULL_PACKAGE_SIZE);
}

/*
  send InertialLabs data
 */
void InertialLabs::update() {
    if (!init_sitl_pointer()) {
        return;
    }

    uint32_t now = AP_HAL::micros();

    const uint32_t sendTimePeriod = 25000; // microseconds
    if (now - last_packet_timestamp >= sendTimePeriod) {
        last_packet_timestamp = now;
        send_packet();
    }
}

} // namespace SITL
