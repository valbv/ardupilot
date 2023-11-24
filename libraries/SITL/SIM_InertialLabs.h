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
  simulate serial InertialLabs AHRS

  Usage example:
     SERIAL5_PROTOCOL = 36
     AHRS_EKF_TYPE = 11
     EAHRS_TYPE=3

     sim_vehicle.py -D --console --map -A --uartF=sim:InertialLabs
*/

#pragma once

#include "SIM_SerialDevice.h"

namespace SITL {

class InertialLabs : public SerialDevice {
public:
    InertialLabs() = default;

    // update state
    void update();

private:
    uint32_t last_packet_timestamp{0};

    void send_packet();
};

} // namespace SITL
