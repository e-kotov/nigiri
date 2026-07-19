#pragma once

#include <queue>
#include "nigiri/routing/tad/tad_state.h"
#include "nigiri/routing/transfer_time_settings.h"
#include "nigiri/routing/get_earliest_transport.h"
#include "nigiri/timetable.h"

namespace nigiri::routing {

struct tad_pq_label {
  location_idx_t l_;
  delta_t arr_;
  friend bool operator<(tad_pq_label const& a, tad_pq_label const& b) {
    return a.arr_ > b.arr_;
  }
};

struct tad_search {
  tad_search(timetable const& tt, tad_state& state, transfer_time_settings const& tts)
      : tt_{tt}, state_{state}, transfer_time_settings_{tts} {
      auto const n_days = static_cast<unsigned>(tt_.internal_interval_days().size().count());
      state_.resize(tt_.n_locations(), tt_.transport_traffic_days_.size(), n_days);
  }

  void execute(location_idx_t const start_l, delta_t const start_time) {
      state_.next_start_time();
      
      pq_.push(tad_pq_label{start_l, start_time});
      
      auto& start_label = state_.node_labels_[cista::to_idx(start_l)];
      start_label.arrival_time_ = start_time;
      start_label.time_stamp_ = state_.current_time_stamp_;

      while (!pq_.empty()) {
          auto const current = pq_.top();
          pq_.pop();

          auto const l = current.l_;
          auto const current_time = current.arr_;
          
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
              auto const transfer_time = adjusted_transfer_time(transfer_time_settings_, duration); 
                  
              auto const fp_target_time = current_time + transfer_time;
              
              auto& target_label = state_.node_labels_[cista::to_idx(target)];
              if (target_label.time_stamp_ != state_.current_time_stamp_ || fp_target_time < target_label.arrival_time_) {
                  target_label.arrival_time_ = static_cast<delta_t>(fp_target_time);
                  target_label.time_stamp_ = state_.current_time_stamp_;
                  target_label.parent_ = l;
                  target_label.parent_transport_ = transport_idx_t::invalid();
                  target_label.parent_day_ = day_idx_t::invalid();
                  target_label.reached_by_walking_ = true;
                  pq_.push(tad_pq_label{target, static_cast<delta_t>(fp_target_time)});
              }
          }
          
          // --- Phase 2: Scan Routes (Transit Transfers) ---
          bool const is_initial = (cista::to_idx(l) == cista::to_idx(start_l) && current_time == start_time);
          auto const min_transfer = tt_.locations_.transfer_time_[l].count();
          auto const transfer_time = is_initial ? 0 : adjusted_transfer_time(transfer_time_settings_, min_transfer);
          auto const board_time = current_time + transfer_time;
          auto const [board_day, board_mam] = split_day_mam(day_idx_t{0}, board_time);
          
          for (auto const& r : tt_.location_routes_[l]) {
              auto const stop_seq = tt_.route_location_seq_[r];
              
              for (unsigned stop_idx = 0; stop_idx < stop_seq.size(); ++stop_idx) {
                  auto const stp = stop(stop_seq[stop_idx]);
                  if (cista::to_idx(stp.location_idx()) != cista::to_idx(l) || !stp.can_start<direction::kForward>(false)) {
                      continue;
                  }
                  
                  auto const trans = get_earliest_transport<direction::kForward>(
                      tt_, tt_, 0U, r, stop_idx, board_day, 
                      minutes_after_midnight_t{board_mam.count()}, l,
                      [](day_idx_t, delta_t) { return false; });
                      
                  if (trans.is_valid()) {
                      auto const t = trans.t_idx_;
                      auto const start_day = trans.day_;
                      
                      // Vehicle Label Dominance: Prune if we already boarded this exact transport on this day at an earlier or equal time
                      auto const v_label_idx = cista::to_idx(t) * state_.n_days_ + cista::to_idx(start_day);
                      auto& v_label = state_.vehicle_labels_[v_label_idx];
                      auto const actual_board_mam = tt_.event_mam(r, t, stop_idx, event_type::kDep);
                      auto const actual_board_time = tt_to_delta(day_idx_t{0}, start_day, minutes_after_midnight_t{actual_board_mam.count()});
                      
                      if (v_label.time_stamp_ == state_.current_time_stamp_ && v_label.arrival_time_ <= actual_board_time) {
                          continue; // Pruned by VLabel
                      }
                      
                      v_label.arrival_time_ = actual_board_time;
                      v_label.time_stamp_ = state_.current_time_stamp_;
                      
                      // Scan the rest of the route
                      for (unsigned target_idx = stop_idx + 1; target_idx < stop_seq.size(); ++target_idx) {
                          auto const target_stp = stop(stop_seq[target_idx]);
                          if (!target_stp.can_finish<direction::kForward>(false)) continue;
                          
                          auto const target_l = target_stp.location_idx();
                          auto const target_arr_mam = tt_.event_mam(r, t, target_idx, event_type::kArr);
                          auto const arr_time = tt_to_delta(day_idx_t{0}, start_day, minutes_after_midnight_t{target_arr_mam.count()});
                              
                          auto& target_label = state_.node_labels_[cista::to_idx(target_l)];
                          if (target_label.time_stamp_ != state_.current_time_stamp_ || arr_time < target_label.arrival_time_) {
                              target_label.arrival_time_ = arr_time;
                              target_label.time_stamp_ = state_.current_time_stamp_;
                              target_label.parent_ = l;
                              target_label.parent_transport_ = t;
                              target_label.parent_day_ = start_day;
                              target_label.reached_by_walking_ = false;
                              
                              pq_.push(tad_pq_label{target_l, arr_time});
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
  std::priority_queue<tad_pq_label> pq_;
};

} // namespace nigiri::routing
