#pragma once

#include "nigiri/common/dial.h"
#include "nigiri/routing/tad/tad_state.h"
#include "nigiri/routing/transfer_time_settings.h"
#include "nigiri/timetable.h"

namespace nigiri::routing {

// The priority queue label structure for our K-Heap
struct tad_dial_label {
  location_idx_t l_;
  std::uint16_t d_;
  friend bool operator>(tad_dial_label const& a, tad_dial_label const& b) {
    return a.d_ > b.d_;
  }
};

struct tad_get_bucket {
  std::uint16_t operator()(tad_dial_label const& l) const { return l.d_; }
};

template <direction SearchDir>
struct tad_search {
  static constexpr auto const kFwd = (SearchDir == direction::kForward);
  static constexpr auto const kBwd = (SearchDir == direction::kBackward);

  tad_search(timetable const& tt, tad_state& state, transfer_time_settings const& tts)
      : tt_{tt}, state_{state}, transfer_time_settings_{tts} {
      
      // Initialize the state dimensions
      // Note: vehicle labels are scaled by total route_transports, though in MVP
      // we could scale by something smaller depending on how we index active vehicles.
      state_.resize(tt_.n_locations(), tt_.transport_traffic_days_.size());
      pq_.n_buckets(65535);
  }

  void add_start(location_idx_t const l, delta_t const start_time) {
      auto& label = state_.node_labels_[to_idx(l)];
      label.arrival_time_ = start_time;
      label.time_stamp_ = state_.current_time_stamp_;
      
      // Seed the priority queue
      pq_.push(tad_dial_label{l, static_cast<std::uint16_t>(start_time)});
  }

  void execute() {
      // 1. Advance the global timestamp to lazy-clear old labels
      state_.next_start_time();
      
      // 2. Main external K-Heap expansion loop
      while (!pq_.empty()) {
          auto const current = pq_.top();
          pq_.pop();

          auto const l = current.l_;
          auto const current_time = static_cast<delta_t>(current.d_);
          
          // Domination check: If we've already found a strictly better way to this stop
          // during the current search timestamp, skip this label.
          auto const& node_label = state_.node_labels_[cista::to_idx(l)];
          if (node_label.time_stamp_ == state_.current_time_stamp_ && 
              node_label.arrival_time_ < current_time) {
              continue;
          }
          
          // --- Phase 1: Relax Footpaths (Walking Transfers) ---
          auto const& fps = tt_.locations_.footpaths_out_[profile_idx_t{0}][l];
          for (auto const& fp : fps) {
              auto const target = fp.target();
              auto const duration = fp.duration().count();
              auto const transfer_time = kFwd ? 
                  adjusted_transfer_time(transfer_time_settings_, duration) : 
                  adjusted_transfer_time(transfer_time_settings_, duration); 
                  
              // For Fwd search, add duration. 
              auto const fp_target_time = current_time + transfer_time;
              
              auto& target_label = state_.node_labels_[cista::to_idx(target)];
              if (target_label.time_stamp_ != state_.current_time_stamp_ || fp_target_time < target_label.arrival_time_) {
                  target_label.arrival_time_ = fp_target_time;
                  target_label.time_stamp_ = state_.current_time_stamp_;
                  pq_.push(tad_dial_label{target, static_cast<std::uint16_t>(fp_target_time)});
              }
          }
          
          // --- Phase 2: Scan Routes (Transit Transfers) ---
          for (auto const& r : tt_.location_routes_[l]) {
              auto const stop_seq = tt_.route_location_seq_[r];
              
              for (unsigned stop_idx = 0; stop_idx < stop_seq.size(); ++stop_idx) {
                  auto const stp = stop(stop_seq[stop_idx]);
                  if (cista::to_idx(stp.location_idx()) == cista::to_idx(l)) {
                      if (!stp.can_start<SearchDir>(false)) continue;
                      
                      auto const min_transfer = tt_.locations_.transfer_time_[l].count();
                      auto const transfer_time = adjusted_transfer_time(transfer_time_settings_, min_transfer);
                      auto const board_time = current_time + transfer_time;
                      
                      auto const [board_day, board_mam] = split_day_mam(
                          day_idx_t{0}, board_time);
                          
                      // Extremely simplified earliest trip scan for MVP (Assumes daily operation for now)
                      // In a full implementation, we must iterate days and check tt_.bitfield_indices_
                      auto const dep_times = tt_.event_times_at_stop(r, stop_idx, event_type::kDep);
                      auto const transports = tt_.route_transport_ranges_[r];
                      
                      for (unsigned t_offset = 0; t_offset < dep_times.size(); ++t_offset) {
                          auto const ev = dep_times[t_offset];
                          if (ev.mam() >= board_mam.count()) {
                              auto const t = transports[t_offset];
                              
                              // Scan the rest of the route
                              for (unsigned target_idx = stop_idx + 1; target_idx < stop_seq.size(); ++target_idx) {
                                  auto const target_stp = stop(stop_seq[target_idx]);
                                  if (!target_stp.can_finish<SearchDir>(false)) continue;
                                  
                                  auto const target_l = target_stp.location_idx();
                                  auto const target_arr_mam = tt_.event_mam(r, t, target_idx, event_type::kArr);
                                  auto const arr_time = tt_to_delta(
                                      day_idx_t{0}, 
                                      board_day, // Approximated
                                      minutes_after_midnight_t{target_arr_mam.count()});
                                      
                                  auto& target_label = state_.node_labels_[cista::to_idx(target_l)];
                                  if (target_label.time_stamp_ != state_.current_time_stamp_ || arr_time < target_label.arrival_time_) {
                                      target_label.arrival_time_ = arr_time;
                                      target_label.time_stamp_ = state_.current_time_stamp_;
                                      pq_.push(tad_dial_label{target_l, static_cast<std::uint16_t>(arr_time)});
                                  }
                              }
                              break; // Found earliest transport, move to next route
                          }
                      }
                  }
              }
          }
      }
  }

private:
  timetable const& tt_;
  tad_state& state_;
  transfer_time_settings transfer_time_settings_;
  dial<tad_dial_label, tad_get_bucket> pq_;
};

} // namespace nigiri::routing
