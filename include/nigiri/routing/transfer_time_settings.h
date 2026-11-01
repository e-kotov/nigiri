#pragma once

#include "nigiri/types.h"

namespace nigiri::routing {

struct transfer_time_settings {
  bool operator==(transfer_time_settings const& o) const {
    if (default_ && o.default_) {
      return true;
    }
    return std::tie(default_, min_transfer_time_, additional_time_, factor_) ==
           std::tie(o.default_, o.min_transfer_time_, o.additional_time_, o.factor_);
  }

  bool is_default() const {
    return default_ && min_transfer_time_ == duration_t{0} &&
           additional_time_ == duration_t{0} && factor_ == 1.0F;
  }

  bool default_{true};
  duration_t min_transfer_time_{0};
  duration_t additional_time_{0};
  float factor_{1.0F};
};

template <typename T>
inline constexpr T adjusted_transfer_time(
    transfer_time_settings const& settings, T const duration) {
  if (settings.default_) {
    return duration;
  } else {
    return static_cast<T>(settings.additional_time_.count()) +
           std::max(
               static_cast<T>(settings.min_transfer_time_.count()),
               static_cast<T>(static_cast<float>(duration) * settings.factor_));
  }
}

template <typename Rep>
inline constexpr std::chrono::duration<Rep, std::ratio<60>>
adjusted_transfer_time(
    transfer_time_settings const& settings,
    std::chrono::duration<Rep, std::ratio<60>> const duration) {
  if (settings.default_) {
    return duration;
  } else {
    return std::chrono::duration<Rep, std::ratio<60>>{
        static_cast<Rep>(settings.additional_time_.count()) +
        std::max(static_cast<Rep>(settings.min_transfer_time_.count()),
                 static_cast<Rep>(static_cast<float>(duration.count()) *
                                  settings.factor_))};
  }
}

}  // namespace nigiri::routing
