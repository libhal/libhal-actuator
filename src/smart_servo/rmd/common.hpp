#pragma once

#include <array>

#include <libhal/can.hpp>

namespace hal {
inline constexpr can::message_t message(hal::can::id_t p_device_id,
                                        std::array<hal::byte, 8> p_payload)
{
  can::message_t message{ .id = p_device_id, .length = 8 };
  message.payload = p_payload;
  return message;
}

template<std::integral T>
T bounds_check(std::floating_point auto p_float)
{
  using float_t = decltype(p_float);
  constexpr auto min = static_cast<float_t>(std::numeric_limits<T>::min());
  constexpr auto max = static_cast<float_t>(std::numeric_limits<T>::max());

  return static_cast<T>(std::clamp(p_float, min, max));
}
}  // namespace hal
