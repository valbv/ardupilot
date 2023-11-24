#pragma once

#include <AP_ExternalAHRS/AP_ExternalAHRS_config.h>

#if AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED

#include <AP_Common/AP_Common.h>

#include <stdint.h>
namespace InertialLabs::Payload {

constexpr char HEADER[] = {
    '\x1C', //< Count of measurement types in payload = 28
    '\x21', //< Gyro data HR (X, Y, Z), deg/s*1.0e5
    '\x23', //< Accelerometer data HR (X, Y, Z), g*1.0e6
    '\x24', //< Magnetometer data (X, Y, Z), nT/10
    '\x10', //< Position (Latitude, Longitude, Altitude), deg*1.0e7
    '\x12', //< Velocities (East, North, Vertical speed), m/s*100
    '\x30', //< GNSS Position (Latitude, Longitude, Altitude), deg*1.0e7
    '\x32', //< GNSS Velocity (Horizontal speed, Track over ground, Vertical speed), m/s*100
    '\x09', //< Quaternion of orientation (q0, q1, q2, q3) q*10000
    '\x01', //< GPS INS Time (round), milliseconds from the beginning of the GPS reference week, rounded to 1000/(output data rate)
    '\x25', //< Barometer data: Pressure, Pa/2; Baro altitude, m*100
    '\x28', //< Differential pressure, mbar*1.0e4
    '\x3E', //< GNSS Position timestamp, ms
    '\x36', //< GNSS info short: GNSS_info1, GNSS_info2
    '\x3B', //< Number of satellites used in solution
    '\x41', //< New GPS, Indicator of new update of GPS data
    '\x80', //< Static pressure, Pa
    '\x81', //< Dynamic pressure, Pa*100
    '\x85', //< Calibrated airspeed (CAS), m/s*100
    '\x86', //< True airspeed (TAS), m/s*100
    '\x88', //< True angle of attack, deg*100
    '\x8A', //< Wind speed (East, North, Vertical), m/s*100
    '\x8D', //< Air data unit (ADU) status
    '\x52', //< Temperature, °C*10
    '\x50', //< Supply voltage, VDC*100
    '\x53', //< Unit status word (USW)
    '\x5A', //< Unit status word 2 (USW2)
    '\x4A', //< GNSS extended info
    '\xC0', //< u-blox jamming status
};

struct PACKED Sensor_Data {
    // deg/s*1.0e5
    int32_t gyro_x{0};
    int32_t gyro_y{0};
    int32_t gyro_z{0};

    // g*1.0e6
    int32_t accel_x{0};
    int32_t accel_y{0};
    int32_t accel_z{0};

    // nT/10
    int16_t mag_x{0};
    int16_t mag_y{0};
    int16_t mag_z{0};

    // deg*1.0e7
    int32_t latitude{0};
    int32_t longitude{0};
    // m*100
    int32_t altitude{0};

    // m/s*100
    int32_t east_speed{0};
    int32_t north_speed{0};
    int32_t vertical_speed{0};

    // deg*1.0e7
    int32_t gnss_latitude{0};
    int32_t gnss_longitude{0};
    // m*100
    int32_t gnss_altitude{0};

    // m/s*100
    int32_t gnss_horizontal_speed{0};
    uint16_t track_over_ground{0};
    int32_t gnss_vertical_speed{0};

    int16_t quaternion_0{0};
    int16_t quaternion_1{0};
    int16_t quaternion_2{0};
    int16_t quaternion_3{0};

    uint32_t gps_ins_time{0};
    // Pa/2
    uint16_t pressure{0};
    int32_t baro_altitude{0};

    int32_t differential_pressure{0};

    uint32_t gnssPosition_timestamp{0};

    char gnss_info_1{0};
    char gnss_info_2{0};

    char used_satellite_number{0};

    char new_gps_data_indicator{0};

    int32_t static_pressure{0};

    int32_t dynamic_pressure{0};

    int16_t calibrated_airspeed{0};

    int16_t true_airspeed{0};

    int16_t true_attack_angle{0};

    int16_t east_wind_speed{0};
    int16_t north_wind_speed{0};
    int16_t vertical_wind_speed{0};

    uint16_t air_data_unit_status{0};
    // °C*10
    int16_t temperature{0};

    uint16_t supply_voltage{0};

    uint16_t unit_status_word_1{0};

    uint16_t unit_status_word_2{0};

    char gps_fix_type{0};
    char spoofing_detect_type{0};

    char ublox_jamming_status{0};
};

} // namespace InertialLabs::Payload

#endif // AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED
