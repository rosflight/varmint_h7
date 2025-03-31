/*
 * Copyright (c) 2017, James Jackson and Daniel Koch, BYU MAGICC Lab
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * This file is a replica of the virtual functions defined in board.h the GNSS structs in sensors.h found in the main
 * rosflight_firmware repository. This is used only for building and testing the varmint board implementation
 * independently of the rosflight_firmware repo.
 */

#ifndef ROSFLIGHT_FIRMWARE_BOARD_H
#define ROSFLIGHT_FIRMWARE_BOARD_H

#include <cstdbool>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace rosflight_firmware
{

enum GNSSFixType
{
  GNSS_FIX_TYPE_NO_FIX = 0,
  GNSS_FIX_TYPE_DEAD_RECKONING_ONLY = 1,
  GNSS_FIX_TYPE_2D_FIX = 2,
  GNSS_FIX_TYPE_3D_FIX = 3,
  GNSS_FIX_TYPE_GNSS_PLUS_DEAD_RECKONING = 4,
  GNSS_FIX_TYPE_TIME_FIX_ONLY = 5,
};

struct GNSSData
{
  uint64_t time_of_week;  // Used internally to firmware for checking gnss
                          // messages. Not included in ROS2 or MAVlink messages
  uint64_t seconds;  // Unix time, in seconds
  uint64_t nanos; // Fractional time
  GNSSFixType fix_type;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t num_sat;
  int32_t lon;
  int32_t lat;
  int32_t height;
  int32_t vel_n;  // mm/s
  int32_t vel_e;  // mm/s
  int32_t vel_d;  // mm/s
  uint32_t h_acc; // mm
  uint32_t v_acc; // mm
  uint32_t s_acc; // mm/s

  GNSSData() { memset(this, 0, sizeof(GNSSData)); }
};

class Board
{
public:
  typedef enum
  {
    RC_TYPE_PPM = 0,
    RC_TYPE_SBUS = 1
  } rc_type_t;

  // setup
  virtual void init_board() = 0;
  virtual void board_reset(bool bootloader) = 0;

  // clock
  virtual uint32_t clock_millis() = 0;
  virtual uint64_t clock_micros() = 0;
  virtual void clock_delay(uint32_t milliseconds) = 0;

  // serial
  virtual void serial_init(uint32_t baud_rate, uint32_t dev) = 0;
  virtual void serial_write(const uint8_t * src, size_t len, uint8_t qos) = 0;
  virtual uint16_t serial_bytes_available() = 0;
  virtual uint8_t serial_read() = 0;
  virtual void serial_flush() = 0;

  // sensors
  virtual void sensors_init() = 0;
  virtual uint16_t sensors_errors_count() = 0;
  virtual uint16_t sensors_init_message_count() = 0;
  virtual bool sensors_init_message_good(uint16_t i) = 0;
  virtual uint16_t sensors_init_message(char * message, uint16_t size, uint16_t i) = 0;

  // IMU
  virtual bool imu_present() =0;
  virtual bool imu_has_new_data() = 0;
  virtual bool imu_read(float accel[3], float * temperature, float gyro[3], uint64_t * time) = 0;
  virtual void imu_not_responding_error() = 0;

  // Mag
  virtual bool mag_present() = 0;
  virtual bool mag_has_new_data() = 0;
  virtual bool mag_read(float mag[3]) = 0;

  // Baro
  virtual bool baro_present() = 0;
  virtual bool baro_has_new_data() = 0;
  virtual bool baro_read(float * pressure, float * temperature) = 0;

  // Pitot
  virtual bool diff_pressure_present() = 0;
  virtual bool diff_pressure_has_new_data() = 0;
  virtual bool diff_pressure_read(float * diff_pressure, float * temperature) = 0;

  // Sonar
  virtual bool sonar_present() = 0;
  virtual bool sonar_has_new_data() = 0;
  virtual bool sonar_read(float * range) = 0;

  // GPS
  virtual bool gnss_present() = 0;
  virtual bool gnss_has_new_data() = 0;
  virtual bool gnss_read(GNSSData * gnss) = 0;

  // Battery
  virtual bool battery_present() = 0;
  virtual bool battery_has_new_data() = 0;
  virtual bool battery_read(float * voltage, float * current) = 0;
  virtual void battery_voltage_set_multiplier(double multiplier) = 0;
  virtual void battery_current_set_multiplier(double multiplier) = 0;

  // RC
  virtual void rc_init(rc_type_t rc_type) = 0;
  virtual bool rc_lost() = 0;
  virtual bool rc_has_new_data() = 0;
  virtual float rc_read(uint8_t chan) = 0;

  // PWM
  virtual void pwm_init(uint32_t refresh_rate, uint16_t idle_pwm) = 0;
  virtual void pwm_init_multi(const float * rate, uint32_t channels) = 0;
  virtual void pwm_disable() = 0;
  virtual void pwm_write(uint8_t channel, float value) = 0;
  virtual void pwm_write_multi(float * value, uint32_t channels) = 0;

  // non-volatile memory
  virtual void memory_init() = 0;
  virtual bool memory_read(void * dest, size_t len) = 0;
  virtual bool memory_write(const void * src, size_t len) = 0;

  // LEDs
  virtual void led0_on() = 0;
  virtual void led0_off() = 0;
  virtual void led0_toggle() = 0;

  virtual void led1_on() = 0;
  virtual void led1_off() = 0;
  virtual void led1_toggle() = 0;

  // Backup memory
  virtual void backup_memory_init() = 0;
  virtual bool backup_memory_read(void * dest, size_t len) = 0;
  virtual void backup_memory_write(const void * src, size_t len) = 0;
  virtual void backup_memory_clear(size_t len) = 0;
};

} // namespace rosflight_firmware

#endif // ROSLFIGHT_FIRMWARE_BOARD_H
