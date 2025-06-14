#pragma once

#include <libhal/units.hpp>

namespace hal::actuator {
/// Operating baudrate of all RMD-X smart servos
static constexpr hertz baudrate_hz = 1'000'000;
static constexpr float dps_per_lsb_speed = 0.01f;
static constexpr float dps_per_lsb_angle = 1.0f;

constexpr auto movement_threshold = 5.0_rpm;
// Hardware limits based on 32-bit signed values and conversion factors
constexpr auto max_hardware_velocity =
  static_cast<float>(std::numeric_limits<std::int32_t>::max()) *
  dps_per_lsb_speed;
constexpr auto max_position_degrees =
  static_cast<float>(std::numeric_limits<std::int32_t>::max()) *
  dps_per_lsb_angle;

static constexpr hal::u32 first_device_address = 0x140;
static constexpr hal::u32 last_device_address = first_device_address + 32;
/// Messages returned from these motor drivers are the same as motor ID plus
/// this offset.
static constexpr hal::u32 response_id_offset = 0x100;
/// Error state flag indicating the motor is stalling or has stalled
static constexpr hal::u16 motor_stall_mask = 0x0002;
/// Error state flag indicating the motor is suffering from low pressure...
/// I'm not sure what that means
static constexpr hal::u16 low_pressure_mask = 0x0004;
/// Error state flag indicating the motor has seen a voltage spike above its
/// maximum rating.
static constexpr hal::u16 over_voltage_mask = 0x0008;
/// Error state flag indicating the motor drew more current than it was
/// prepared to handle.
static constexpr hal::u16 over_current_mask = 0x0010;
/// Error state flag indicating the motor drew more power than it was
/// prepared to handle.
static constexpr hal::u16 power_overrun_mask = 0x0040;
/// NOTE: I don't know what this means exactly...
static constexpr hal::u16 speeding_mask = 0x0100;
/// Error state flag indicating the motor temperature has exceeded its
/// rating.
static constexpr hal::u16 over_temperature_mask = 0x1000;
/// Error state flag indicating the encoder calibration failed.
static constexpr hal::u16 encoder_calibration_error_mask = 0x2000;
}  // namespace hal::actuator
