#pragma once

#include <vector>
#include <algorithm>

#include "nigiri/common/delta_t.h"
#include "nigiri/types.h"

namespace nigiri::routing {

struct tad_node_label {
  delta_t arrival_time_{kInvalidDelta<direction::kForward>};
  std::uint16_t time_stamp_{0U};
  location_idx_t parent_{location_idx_t::invalid()};
  transport_idx_t parent_transport_{transport_idx_t::invalid()};
  day_idx_t parent_day_{day_idx_t::invalid()};
  bool reached_by_walking_{false};
};

struct tad_vehicle_label {
  delta_t arrival_time_{kInvalidDelta<direction::kForward>};
  std::uint16_t time_stamp_{0U};
};

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
    is_target_.resize(n_locations, false);
    target_offsets_.resize(n_locations, duration_t{0});
    return *this;
  }

  void next_start_time() {
    ++current_time_stamp_;
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
  std::vector<bool> is_target_;
  std::vector<duration_t> target_offsets_;
};

}  // namespace nigiri::routing
