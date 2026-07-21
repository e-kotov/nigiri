#pragma once

#include <vector>
#include <algorithm>

#include "nigiri/common/delta_t.h"
#include "nigiri/types.h"

namespace nigiri::routing {

// The core 16-byte NodeLabel (due to Nigiri using a 2-byte delta_t)
struct tad_node_label {
  delta_t arrival_time_{kInvalidDelta<direction::kForward>}; // 2 bytes
  std::uint16_t time_stamp_{0U};                               // 2 bytes
  location_idx_t parent_{location_idx_t::invalid()};           // 4 bytes
  transport_idx_t parent_transport_{transport_idx_t::invalid()}; // 4 bytes
  day_idx_t parent_day_{day_idx_t::invalid()};                 // 2 bytes
  bool reached_by_walking_{false};                             // 1 byte
  // 1 byte padding -> 16 bytes total
};
static_assert(sizeof(tad_node_label) <= 16, "NodeLabel should be tightly packed to 16 bytes");

// The highly optimized 4-byte Lite Vehicle Label
struct tad_vehicle_label {
  delta_t arrival_time_{kInvalidDelta<direction::kForward>}; // 2 bytes
  std::uint16_t time_stamp_{0U};                               // 2 bytes
};
static_assert(sizeof(tad_vehicle_label) == 4, "VehicleLabel must be exactly 4 bytes");

struct tad_state {
  tad_state() = default;
  tad_state(tad_state const&) = delete;
  tad_state& operator=(tad_state const&) = delete;
  tad_state(tad_state&&) = default;
  tad_state& operator=(tad_state&&) = default;
  ~tad_state() = default;

  tad_state& resize(unsigned n_locations, unsigned n_route_transports, unsigned n_days, unsigned n_rt_transports = 0) {
    n_locations_ = n_locations;
    n_route_transports_ = n_route_transports;
    n_days_ = n_days;
    n_rt_transports_ = n_rt_transports;
    node_labels_.resize(n_locations);
    vehicle_labels_.resize(n_route_transports * n_days + n_rt_transports);
    return *this;
  }

  // Uses a 16-bit monotonic counter to lazy-clear arrays without O(N) resets every query
  void next_start_time() {
    ++current_time_stamp_;
    // Handle wrap-around gracefully
    if (current_time_stamp_ == 0U) {
      std::fill(node_labels_.begin(), node_labels_.end(), tad_node_label{});
      std::fill(vehicle_labels_.begin(), vehicle_labels_.end(), tad_vehicle_label{});
      current_time_stamp_ = 1U;
    }
  }

  unsigned n_locations_{0U};
  unsigned n_route_transports_{0U};
  unsigned n_days_{0U};
  unsigned n_rt_transports_{0U};
  std::uint16_t current_time_stamp_{0U};

  std::vector<tad_node_label> node_labels_;
  std::vector<tad_vehicle_label> vehicle_labels_;
};

}  // namespace nigiri::routing
