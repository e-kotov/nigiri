#pragma once

#include <vector>
#include <algorithm>

#include "nigiri/routing/journey.h"
#include "nigiri/routing/tad/tad_state.h"
#include "nigiri/timetable.h"

namespace nigiri::routing {

inline void tad_reconstruct_journey(timetable const& tt,
                                    transfer_time_settings const& tts,
                                    tad_state const& state,
                                    location_idx_t const start_l,
                                    location_idx_t const dest_l,
                                    date::sys_days const base,
                                    journey& j) {
  auto curr_l = dest_l;
  
  while (curr_l != start_l && curr_l != location_idx_t::invalid()) {
    auto const& lbl = state.node_labels_[cista::to_idx(curr_l)];
    if (lbl.time_stamp_ != state.current_time_stamp_) {
      throw std::runtime_error("TAD Reconstruct: broken path (timestamp mismatch)");
    }
    
    auto const arr_time = lbl.arrival_time_;
    auto const parent_l = lbl.parent_;
    
    if (parent_l == location_idx_t::invalid()) {
      throw std::runtime_error("TAD Reconstruct: broken path (invalid parent)");
    }
    
    auto const& parent_lbl = state.node_labels_[cista::to_idx(parent_l)];
    auto const dep_time = parent_lbl.arrival_time_;

    if (lbl.reached_by_walking_) {
      // Footpath leg
      j.add(journey::leg{direction::kForward,
                         parent_l,
                         curr_l,
                         delta_to_unix(base, dep_time),
                         delta_to_unix(base, arr_time),
                         footpath{curr_l, duration_t{static_cast<std::int16_t>(arr_time - dep_time)}}});
    } else {
      // Transport leg
      auto const t = lbl.parent_transport_;
      auto const start_day = lbl.parent_day_;
      auto const r = tt.transport_route_[t];
      auto const seq = tt.route_location_seq_[r];
      
      stop_idx_t board_stop_idx = 0;
      delta_t actual_board_time = kInvalidDelta<direction::kForward>;
      
      for (unsigned i = 0; i < seq.size(); ++i) {
        if (stop{seq[i]}.location_idx() == parent_l && stop{seq[i]}.can_start<direction::kForward>(false)) {
          auto const dep_mam = tt.event_mam(r, t, i, event_type::kDep);
          auto const event_time = tt_to_delta(day_idx_t{0}, start_day, minutes_after_midnight_t{dep_mam.count()});
          if (event_time >= dep_time) {
            board_stop_idx = static_cast<stop_idx_t>(i);
            actual_board_time = event_time;
            break;
          }
        }
      }
      
      stop_idx_t alight_stop_idx = 0;
      for (unsigned i = board_stop_idx + 1; i < seq.size(); ++i) {
        if (stop{seq[i]}.location_idx() == curr_l && stop{seq[i]}.can_finish<direction::kForward>(false)) {
          auto const arr_mam = tt.event_mam(r, t, i, event_type::kArr);
          auto const event_time = tt_to_delta(day_idx_t{0}, start_day, minutes_after_midnight_t{arr_mam.count()});
          if (event_time == arr_time) {
            alight_stop_idx = static_cast<stop_idx_t>(i);
            break;
          }
        }
      }
      
      rt::run r_run{.t_ = transport{t, start_day},
                    .stop_range_ = interval<stop_idx_t>{0, static_cast<stop_idx_t>(seq.size())}};
                    
      j.add(journey::leg{direction::kForward,
                         parent_l,
                         curr_l,
                         delta_to_unix(base, actual_board_time),
                         delta_to_unix(base, arr_time),
                         journey::run_enter_exit{r_run, board_stop_idx, alight_stop_idx}});
                         
      // Self-transfer check
      if (parent_l != start_l && !parent_lbl.reached_by_walking_) {
          auto const min_transfer = tt.locations_.transfer_time_[parent_l].count();
          auto const transfer_dur = adjusted_transfer_time(tts, min_transfer);
          if (transfer_dur > 0) {
              j.add(journey::leg{direction::kForward,
                                 parent_l,
                                 parent_l,
                                 delta_to_unix(base, dep_time),
                                 delta_to_unix(base, dep_time + transfer_dur),
                                 footpath{parent_l, duration_t{transfer_dur}}});
          }
      }
    }
    
    curr_l = parent_l;
  }
  
  if (curr_l != start_l) {
    throw std::runtime_error("TAD Reconstruct: failed to reach start location");
  }
  
  // The journey was reconstructed backwards, so reverse it
  std::reverse(j.legs_.begin(), j.legs_.end());
}

} // namespace nigiri::routing
