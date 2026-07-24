#pragma once

#include <chrono>
#include <memory>
#include <queue>
#include <vector>

#define NIGIRI_TAD_PROFILING 0
#include <stdexcept>

#include "nigiri/routing/get_earliest_transport.h"
#include "nigiri/routing/tad/tad_state.h"
#include "nigiri/routing/tad/tad_transfer_index.h"
#include "nigiri/routing/tad/tad_transfer_provider.h"
#include "nigiri/routing/transfer_time_settings.h"
#include "nigiri/rt/rt_timetable.h"
#include "nigiri/timetable.h"

namespace nigiri::routing {

struct tad_pq_label {
  location_idx_t l_;
  delta_t arr_;
};

struct tad_pq_cmp {
  direction search_dir_{direction::kForward};

  bool operator()(tad_pq_label const& a, tad_pq_label const& b) const {
    return search_dir_ == direction::kForward ? a.arr_ > b.arr_
                                              : a.arr_ < b.arr_;
  }
};

struct tad_stats {
  std::uint64_t settled_stop_labels_{0};
  std::uint64_t scanned_routes_{0};
  std::uint64_t scanned_stop_events_{0};
  std::uint64_t relaxed_walking_edges_{0};
  std::uint64_t relaxed_transfers_{0};
  std::uint64_t pruned_scans_{0};
  std::uint64_t queue_updates_{0};
  std::uint64_t time_pq_pop_us_{0};
  std::uint64_t time_footpaths_us_{0};
  std::uint64_t time_routes_us_{0};
  std::uint64_t time_rt_transports_us_{0};
};

struct tad_search {
  tad_search(timetable const& tt,
             rt_timetable const* rtt,
             tad_state& state,
             transfer_time_settings const& tts,
             profile_idx_t prf_idx,
             clasz_mask_t allowed_claszes,
             bool require_bike,
             bool require_car,
             day_idx_t search_base_day_idx = day_idx_t{0},
             direction search_dir = direction::kForward,
             bool is_first_leg = true,
             delta_t query_delta = kInvalidDelta<direction::kForward>,
             duration_t max_travel_time = kMaxTravelTime,
             std::uint8_t max_transfers = kMaxTransfers,
             std::vector<uint8_t>* route_active = nullptr,
             bitfield search_mask = {},
             tad_transfer_index const* transfer_index = nullptr)
      : tt_{tt},
        rtt_{rtt},
        state_{state},
        transfer_time_settings_{tts},
        prf_idx_{prf_idx},
        allowed_claszes_{allowed_claszes},
        require_bike_{require_bike},
        require_car_{require_car},
        search_base_day_idx_{search_base_day_idx},
        search_dir_{search_dir},
        is_first_leg_{is_first_leg},
        query_delta_{query_delta},
        max_travel_time_{max_travel_time},
        max_transfers_{max_transfers},
        route_active_{route_active},
        search_mask_{search_mask},
        pq_{tad_pq_cmp{search_dir}} {
    auto const n_days =
        static_cast<unsigned>(tt_.internal_interval_days().size().count());
    state_.resize(tt_.n_locations(), tt_.transport_traffic_days_.size(), n_days,
                  rtt_ ? rtt_->n_rt_transports() : 0U);
    if (transfer_index != nullptr && transfer_time_settings_.default_ &&
        transfer_index->is_valid_for(prf_idx_)) {
      transfer_provider_ = std::make_unique<shortcut_transfer_provider>(
          transfer_index->profiles_[cista::to_idx(prf_idx_)]);
    } else {
      transfer_provider_ = std::make_unique<footpath_transfer_provider>(tt_, prf_idx_);
    }
  }

  tad_stats execute(std::vector<std::pair<location_idx_t, delta_t>> const& starts,
                    std::vector<std::pair<location_idx_t, duration_t>> const& targets = {}) {
    stats_ = tad_stats{};
    state_.next_start_time();
    has_best_target_time_ = false;
    best_target_time_ = kInvalidDelta<direction::kForward>;

    for (auto const& [l, dur] : targets) {
      auto const idx = cista::to_idx(l);
      if (idx >= state_.is_target_.size()) {
        state_.is_target_.resize(idx + 1, false);
        state_.target_offsets_.resize(idx + 1, duration_t{0});
      }
      if (!state_.is_target_[idx]) {
        state_.is_target_[idx] = true;
        state_.target_offsets_[idx] = dur;
      } else {
        if (dur < state_.target_offsets_[idx]) {
          state_.target_offsets_[idx] = dur;
        }
      }
    }

    for (auto const& [start_l, start_time] : starts) {
      if (exceeds_travel_time(start_time)) continue;

      pq_.push(tad_pq_label{start_l, start_time});
      auto& start_label = state_.node_labels_[cista::to_idx(start_l)];
      if (start_label.time_stamp_ != state_.current_time_stamp_ ||
          better(start_time, start_label.arrival_time_)) {
        start_label.arrival_time_ = start_time;
        start_label.time_stamp_ = state_.current_time_stamp_;
        start_label.parent_ = start_l;
        start_label.reached_by_walking_ = false;
        start_label.parent_transport_ = transport_idx_t::invalid();

        auto const s_idx = cista::to_idx(start_l);
        if (s_idx < state_.is_target_.size() && state_.is_target_[s_idx]) {
          auto const time_at_target = add_duration(start_time, state_.target_offsets_[s_idx]);
          if (!has_best_target_time_ || better(time_at_target, best_target_time_)) {
            best_target_time_ = time_at_target;
            has_best_target_time_ = true;
          }
        }
      }
    }

    while (!pq_.empty()) {
      auto const current = pq_.top();

      if (has_best_target_time_ && better_or_equal(best_target_time_, current.arr_)) {
        break;
      }

#if NIGIRI_TAD_PROFILING
      auto const t0 = std::chrono::steady_clock::now();
#endif
      pq_.pop();
#if NIGIRI_TAD_PROFILING
      auto const t1 = std::chrono::steady_clock::now();
      stats_.time_pq_pop_us_ += std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
#endif
      ++stats_.settled_stop_labels_;

      auto const l = current.l_;
      auto const current_time = current.arr_;

      auto const& node_label = state_.node_labels_[cista::to_idx(l)];
      if (node_label.time_stamp_ == state_.current_time_stamp_ &&
          better(node_label.arrival_time_, current_time)) {
        continue;
      }

#if NIGIRI_TAD_PROFILING
      auto const t2 = std::chrono::steady_clock::now();
#endif
      relax_transfers(l, current_time);
#if NIGIRI_TAD_PROFILING
      auto const t3 = std::chrono::steady_clock::now();
      stats_.time_footpaths_us_ += std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
#endif

#if NIGIRI_TAD_PROFILING
      auto const t4 = std::chrono::steady_clock::now();
#endif
      scan_routes(l, current_time, starts);
#if NIGIRI_TAD_PROFILING
      auto const t5 = std::chrono::steady_clock::now();
      stats_.time_routes_us_ += std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count();
#endif

#if NIGIRI_TAD_PROFILING
      auto const t6 = std::chrono::steady_clock::now();
#endif
      scan_rt_transports(l, current_time, starts);
#if NIGIRI_TAD_PROFILING
      auto const t7 = std::chrono::steady_clock::now();
      stats_.time_rt_transports_us_ += std::chrono::duration_cast<std::chrono::microseconds>(t7 - t6).count();
#endif
    }
    for (auto const& [l, dur] : targets) {
      auto const idx = cista::to_idx(l);
      if (idx < state_.is_target_.size()) {
        state_.is_target_[idx] = false;
      }
    }
    return stats_;
  }

private:
  bool fwd() const { return search_dir_ == direction::kForward; }

  bool better(delta_t const a, delta_t const b) const {
    return fwd() ? a < b : a > b;
  }

  bool better_or_equal(delta_t const a, delta_t const b) const {
    return fwd() ? a <= b : a >= b;
  }

  bool exceeds_travel_time(delta_t const t) const {
    if (query_delta_ == kInvalidDelta<direction::kForward>) return false;
    auto const limit = fwd() ? static_cast<std::int64_t>(query_delta_) + max_travel_time_.count()
                             : static_cast<std::int64_t>(query_delta_) - max_travel_time_.count();
    return fwd() ? static_cast<std::int64_t>(t) > limit : static_cast<std::int64_t>(t) < limit;
  }

  delta_t add_duration(delta_t const t, duration_t const d) const {
    return clamp(static_cast<int>(t) + (fwd() ? d.count() : -d.count()));
  }

  void update_label(location_idx_t const target,
                    delta_t const time,
                    location_idx_t const parent,
                    transport_idx_t const parent_transport,
                    day_idx_t const parent_day,
                    bool const reached_by_walking) {
    if (exceeds_travel_time(time)) return;

    auto& target_label = state_.node_labels_[cista::to_idx(target)];
    if (target_label.time_stamp_ != state_.current_time_stamp_ ||
        better(time, target_label.arrival_time_)) {
      target_label.arrival_time_ = time;
      target_label.time_stamp_ = state_.current_time_stamp_;
      target_label.parent_ = parent;
      target_label.parent_transport_ = parent_transport;
      target_label.parent_day_ = parent_day;
      target_label.reached_by_walking_ = reached_by_walking;
      pq_.push(tad_pq_label{target, time});
      ++stats_.queue_updates_;

      auto const t_idx = cista::to_idx(target);
      if (t_idx < state_.is_target_.size() && state_.is_target_[t_idx]) {
        auto const time_at_target = add_duration(time, state_.target_offsets_[t_idx]);
        if (!has_best_target_time_ || better(time_at_target, best_target_time_)) {
          best_target_time_ = time_at_target;
          has_best_target_time_ = true;
        }
      }
    }
  }

  bool is_initial(location_idx_t const l,
                  delta_t const current_time,
                  std::vector<std::pair<location_idx_t, delta_t>> const& starts) const {
    for (auto const& [s_l, s_time] : starts) {
      if (cista::to_idx(l) == cista::to_idx(s_l) && current_time == s_time) {
        return true;
      }
    }
    return false;
  }

  void relax_transfers(location_idx_t const l, delta_t const current_time) {
    transfer_edges_buf_.clear();
    transfer_provider_->stop_transfers(l, fwd(), transfer_edges_buf_);
    for (auto const& edge : transfer_edges_buf_) {
      ++stats_.relaxed_walking_edges_;
      ++stats_.relaxed_transfers_;
      auto const target = edge.target_;
      auto const transfer_time = duration_t{adjusted_transfer_time(
          transfer_time_settings_, edge.duration_.count())};
      update_label(target, add_duration(current_time, transfer_time), l,
                   transport_idx_t::invalid(), day_idx_t::invalid(), true);
    }
  }

  void scan_routes(location_idx_t const l,
                   delta_t const current_time,
                   std::vector<std::pair<location_idx_t, delta_t>> const& starts) {
    auto const min_transfer = tt_.locations_.transfer_time_[l].count();
    auto const transfer_time = (is_first_leg_ && is_initial(l, current_time, starts))
                                   ? duration_t{0}
                                   : duration_t{adjusted_transfer_time(
                                         transfer_time_settings_, min_transfer)};
    auto const event_time = add_duration(current_time, transfer_time);
    auto const [event_day, event_mam] = split_day_mam(search_base_day_idx_, event_time);
    bool const is_wheelchair = prf_idx_ == kWheelchairProfile;

    for (auto const& r : tt_.location_routes_[l]) {
      auto const r_idx = cista::to_idx(r);
      if (route_active_ != nullptr) {
        auto& status = (*route_active_)[r_idx];
        if (status == 0) {
          bool active = false;
          auto const transport_range = tt_.route_transport_ranges_[r];
          for (auto const t : transport_range) {
            bitfield_idx_t bf_idx;
            if (rtt_ != nullptr) {
                bf_idx = rtt_->transport_traffic_days_[t];
                if ((rtt_->bitfields_[bf_idx] & search_mask_).any()) { active = true; break; }
            } else {
                bf_idx = tt_.transport_traffic_days_[t];
                if ((tt_.bitfields_[bf_idx] & search_mask_).any()) { active = true; break; }
            }
          }
          status = active ? 1 : 2;
        }
        if (status == 2) {
          continue;
        }
      }
      if (allowed_claszes_ != all_clasz_allowed() &&
          !is_allowed(allowed_claszes_, tt_.route_clasz_[r])) {
        continue;
      }
      if (require_bike_ && !tt_.route_bikes_allowed_.test(r_idx * 2) &&
          !tt_.route_bikes_allowed_.test(r_idx * 2 + 1)) {
        continue;
      }
      if (require_car_ && !tt_.route_cars_allowed_.test(r_idx * 2) &&
          !tt_.route_cars_allowed_.test(r_idx * 2 + 1)) {
        continue;
      }
      ++stats_.scanned_routes_;
      auto const stop_seq = tt_.route_location_seq_[r];

      for (unsigned stop_idx = 0; stop_idx < stop_seq.size(); ++stop_idx) {
        auto const stp = stop(stop_seq[stop_idx]);
        if (cista::to_idx(stp.location_idx()) != cista::to_idx(l)) {
          continue;
        }
        if (fwd()) {
          if (!stp.can_start<direction::kForward>(is_wheelchair)) {
            continue;
          }
          scan_route_forward(r, stop_seq, stop_idx, event_day, event_mam, event_time,
                             l, is_wheelchair);
        } else {
          if (!stp.can_start<direction::kBackward>(is_wheelchair)) {
            continue;
          }
          scan_route_backward(r, stop_seq, stop_idx, event_day, event_mam, event_time,
                              l, is_wheelchair);
        }
      }
    }
  }

  void scan_route_forward(route_idx_t const r,
                          decltype(std::declval<timetable const>().route_location_seq_[route_idx_t{}]) stop_seq,
                          unsigned const stop_idx,
                          day_idx_t const board_day,
                          minutes_after_midnight_t const board_mam,
                          delta_t,
                          location_idx_t const l,
                          bool const is_wheelchair) {
    auto const trans = rtt_ ? get_earliest_transport<direction::kForward>(
                                  tt_, *rtt_, 0U, r, stop_idx, board_day,
                                  minutes_after_midnight_t{board_mam.count()}, l,
                                  [](day_idx_t, delta_t) { return false; })
                            : get_earliest_transport<direction::kForward>(
                                  tt_, tt_, 0U, r, stop_idx, board_day,
                                  minutes_after_midnight_t{board_mam.count()}, l,
                                  [](day_idx_t, delta_t) { return false; });
    if (!trans.is_valid()) {
      return;
    }
    auto const t = trans.t_idx_;
    auto const start_day = trans.day_;
    auto const actual_board_mam = tt_.event_mam(r, t, stop_idx, event_type::kDep);
    auto const actual_board_time = tt_to_delta(
        search_base_day_idx_, start_day, minutes_after_midnight_t{actual_board_mam.count()});
    auto const v_label_idx = cista::to_idx(t) * state_.n_days_ + cista::to_idx(start_day);
    if (v_label_idx >= state_.vehicle_labels_.size()) {
      throw std::runtime_error("TAD Search: v_label_idx out of bounds");
    }
    auto& v_label = state_.vehicle_labels_[v_label_idx];
    if (v_label.time_stamp_ == state_.current_time_stamp_ &&
        v_label.arrival_time_ <= actual_board_time) {
      ++stats_.pruned_scans_;
      return;
    }
    v_label.arrival_time_ = actual_board_time;
    v_label.time_stamp_ = state_.current_time_stamp_;

    for (unsigned target_idx = stop_idx + 1; target_idx < stop_seq.size(); ++target_idx) {
      ++stats_.scanned_stop_events_;
      if (require_bike_ && !tt_.route_bikes_allowed_per_section_.empty() &&
          !tt_.route_bikes_allowed_per_section_[r].empty() &&
          !tt_.route_bikes_allowed_per_section_[r][target_idx - 1]) {
        break;
      }
      if (require_car_ && !tt_.route_cars_allowed_per_section_.empty() &&
          !tt_.route_cars_allowed_per_section_[r].empty() &&
          !tt_.route_cars_allowed_per_section_[r][target_idx - 1]) {
        break;
      }
      auto const target_stp = stop(stop_seq[target_idx]);
      if (!target_stp.can_finish<direction::kForward>(is_wheelchair)) {
        continue;
      }
      auto const target_l = target_stp.location_idx();
      auto const arr_mam = tt_.event_mam(r, t, target_idx, event_type::kArr);
      auto const arr_time = tt_to_delta(
          search_base_day_idx_, start_day, minutes_after_midnight_t{arr_mam.count()});
      update_label(target_l, arr_time, l, t, start_day, false);
    }
  }

  void scan_route_backward(route_idx_t const r,
                           decltype(std::declval<timetable const>().route_location_seq_[route_idx_t{}]) stop_seq,
                           unsigned const stop_idx,
                           day_idx_t const arr_day,
                           minutes_after_midnight_t const arr_mam,
                           delta_t,
                           location_idx_t const l,
                           bool const is_wheelchair) {
    if (stop_idx == 0U) {
      return;
    }
    auto const trans = rtt_ ? get_earliest_transport<direction::kBackward>(
                                  tt_, *rtt_, 0U, r, stop_idx, arr_day,
                                  minutes_after_midnight_t{arr_mam.count()}, l,
                                  [](day_idx_t, delta_t) { return false; })
                            : get_earliest_transport<direction::kBackward>(
                                  tt_, tt_, 0U, r, stop_idx, arr_day,
                                  minutes_after_midnight_t{arr_mam.count()}, l,
                                  [](day_idx_t, delta_t) { return false; });
    if (!trans.is_valid()) {
      return;
    }
    auto const t = trans.t_idx_;
    auto const start_day = trans.day_;
    auto const actual_arr_mam = tt_.event_mam(r, t, stop_idx, event_type::kArr);
    auto const actual_arr_time = tt_to_delta(
        search_base_day_idx_, start_day, minutes_after_midnight_t{actual_arr_mam.count()});
    auto const v_label_idx = cista::to_idx(t) * state_.n_days_ + cista::to_idx(start_day);
    if (v_label_idx >= state_.vehicle_labels_.size()) {
      throw std::runtime_error("TAD Search: v_label_idx out of bounds");
    }
    auto& v_label = state_.vehicle_labels_[v_label_idx];
    if (v_label.time_stamp_ == state_.current_time_stamp_ &&
        v_label.arrival_time_ >= actual_arr_time) {
      ++stats_.pruned_scans_;
      return;
    }
    v_label.arrival_time_ = actual_arr_time;
    v_label.time_stamp_ = state_.current_time_stamp_;

    for (auto target_idx = static_cast<int>(stop_idx) - 1; target_idx >= 0; --target_idx) {
      ++stats_.scanned_stop_events_;
      auto const section_idx = static_cast<unsigned>(target_idx);
      if (require_bike_ && !tt_.route_bikes_allowed_per_section_.empty() &&
          !tt_.route_bikes_allowed_per_section_[r].empty() &&
          !tt_.route_bikes_allowed_per_section_[r][section_idx]) {
        break;
      }
      if (require_car_ && !tt_.route_cars_allowed_per_section_.empty() &&
          !tt_.route_cars_allowed_per_section_[r].empty() &&
          !tt_.route_cars_allowed_per_section_[r][section_idx]) {
        break;
      }
      auto const target_stp = stop(stop_seq[static_cast<unsigned>(target_idx)]);
      if (!target_stp.can_finish<direction::kBackward>(is_wheelchair)) {
        continue;
      }
      auto const target_l = target_stp.location_idx();
      auto const dep_mam = tt_.event_mam(r, t, static_cast<unsigned>(target_idx), event_type::kDep);
      auto const dep_time = tt_to_delta(
          search_base_day_idx_, start_day, minutes_after_midnight_t{dep_mam.count()});
      update_label(target_l, dep_time, l, t, start_day, false);
    }
  }

  void scan_rt_transports(location_idx_t const l,
                          delta_t const current_time,
                          std::vector<std::pair<location_idx_t, delta_t>> const& starts) {
    if (rtt_ == nullptr) {
      return;
    }
    auto const min_transfer = tt_.locations_.transfer_time_[l].count();
    auto const transfer_time = (is_first_leg_ && is_initial(l, current_time, starts))
                                   ? duration_t{0}
                                   : duration_t{adjusted_transfer_time(
                                         transfer_time_settings_, min_transfer)};
    auto const event_time = add_duration(current_time, transfer_time);
    auto const day_offset = static_cast<int>(rtt_->base_day_idx_.v_) -
                            static_cast<int>(search_base_day_idx_.v_);
    bool const is_wheelchair = prf_idx_ == kWheelchairProfile;

    for (auto const& rt_t : rtt_->location_rt_transports_[l]) {
      auto const rt_t_idx = cista::to_idx(rt_t);
      if (allowed_claszes_ != all_clasz_allowed() &&
          !is_allowed(allowed_claszes_, rtt_->rt_transport_section_clasz_[rt_t][0])) {
        continue;
      }
      if (require_bike_) {
        auto const bikes_all = rtt_->rt_transport_bikes_allowed_.test(rt_t_idx * 2);
        auto const bikes_some = rtt_->rt_transport_bikes_allowed_.test(rt_t_idx * 2 + 1);
        if (!bikes_all && !bikes_some) {
          continue;
        }
      }
      if (require_car_) {
        auto const cars_all = rtt_->rt_transport_cars_allowed_.test(rt_t_idx * 2);
        auto const cars_some = rtt_->rt_transport_cars_allowed_.test(rt_t_idx * 2 + 1);
        if (!cars_all && !cars_some) {
          continue;
        }
      }

      ++stats_.scanned_routes_;
      auto const stop_seq = rtt_->rt_transport_location_seq_[rt_t];
      for (unsigned stop_idx = 0; stop_idx < stop_seq.size(); ++stop_idx) {
        auto const stp = stop(stop_seq[stop_idx]);
        if (cista::to_idx(stp.location_idx()) != cista::to_idx(l)) {
          continue;
        }
        if (fwd()) {
          if (!stp.can_start<direction::kForward>(is_wheelchair)) {
            continue;
          }
          scan_rt_transport_forward(rt_t, rt_t_idx, stop_seq, stop_idx,
                                    event_time, day_offset, is_wheelchair);
        } else {
          if (!stp.can_start<direction::kBackward>(is_wheelchair)) {
            continue;
          }
          scan_rt_transport_backward(rt_t, rt_t_idx, stop_seq, stop_idx,
                                     event_time, day_offset, is_wheelchair);
        }
      }
    }
  }

  void set_rt_parent(tad_node_label& target_label, rt_transport_idx_t const rt_t) const {
    auto const static_trans = rtt_->rt_transport_static_transport_[rt_t];
    if (holds_alternative<transport>(static_trans)) {
      auto const t = static_trans.as<transport>();
      target_label.parent_transport_ = t.t_idx_;
      target_label.parent_day_ = t.day_;
    } else {
      target_label.parent_transport_ = transport_idx_t::invalid();
      target_label.parent_day_ = day_idx_t::invalid();
    }
  }

  void update_rt_label(location_idx_t const target,
                       delta_t const time,
                       location_idx_t const parent,
                       rt_transport_idx_t const rt_t) {
    if (exceeds_travel_time(time)) return;

    auto& target_label = state_.node_labels_[cista::to_idx(target)];
    if (target_label.time_stamp_ != state_.current_time_stamp_ ||
        better(time, target_label.arrival_time_)) {
      target_label.arrival_time_ = time;
      target_label.time_stamp_ = state_.current_time_stamp_;
      target_label.parent_ = parent;
      set_rt_parent(target_label, rt_t);
      target_label.reached_by_walking_ = false;
      pq_.push(tad_pq_label{target, time});
      ++stats_.queue_updates_;

      auto const t_idx = cista::to_idx(target);
      if (t_idx < state_.is_target_.size() && state_.is_target_[t_idx]) {
        auto const time_at_target = add_duration(time, state_.target_offsets_[t_idx]);
        if (!has_best_target_time_ || better(time_at_target, best_target_time_)) {
          best_target_time_ = time_at_target;
          has_best_target_time_ = true;
        }
      }
    }
  }

  void scan_rt_transport_forward(rt_transport_idx_t const rt_t,
                                 std::size_t const rt_t_idx,
                                 decltype(std::declval<rt_timetable const>().rt_transport_location_seq_[rt_transport_idx_t{}]) stop_seq,
                                 unsigned const stop_idx,
                                 delta_t const event_time,
                                 int const day_offset,
                                 bool const is_wheelchair) {
    auto const board_event = static_cast<delta_t>(
        day_offset * 1440 + rtt_->rt_transport_stop_times_[rt_t][stop_idx * 2]);
    if (board_event < event_time) {
      return;
    }
    auto const v_label_idx = state_.n_route_transports_ * state_.n_days_ + rt_t_idx;
    auto& v_label = state_.vehicle_labels_[v_label_idx];
    if (v_label.time_stamp_ == state_.current_time_stamp_ &&
        v_label.arrival_time_ <= board_event) {
      ++stats_.pruned_scans_;
      return;
    }
    v_label.arrival_time_ = board_event;
    v_label.time_stamp_ = state_.current_time_stamp_;

    for (unsigned target_idx = stop_idx + 1; target_idx < stop_seq.size(); ++target_idx) {
      ++stats_.scanned_stop_events_;
      if (require_bike_ && !rtt_->rt_transport_bikes_allowed_.test(rt_t_idx * 2)) {
        break;
      }
      if (require_car_ && !rtt_->rt_transport_cars_allowed_.test(rt_t_idx * 2)) {
        break;
      }
      auto const target_stp = stop(stop_seq[target_idx]);
      if (!target_stp.can_finish<direction::kForward>(is_wheelchair)) {
        continue;
      }
      auto const target_l = target_stp.location_idx();
      auto const arr_time = static_cast<delta_t>(
          day_offset * 1440 + rtt_->rt_transport_stop_times_[rt_t][target_idx * 2 - 1]);
      update_rt_label(target_l, arr_time, stop(stop_seq[stop_idx]).location_idx(), rt_t);
    }
  }

  void scan_rt_transport_backward(rt_transport_idx_t const rt_t,
                                  std::size_t const rt_t_idx,
                                  decltype(std::declval<rt_timetable const>().rt_transport_location_seq_[rt_transport_idx_t{}]) stop_seq,
                                  unsigned const stop_idx,
                                  delta_t const event_time,
                                  int const day_offset,
                                  bool const is_wheelchair) {
    if (stop_idx == 0U) {
      return;
    }
    auto const arr_event = static_cast<delta_t>(
        day_offset * 1440 + rtt_->rt_transport_stop_times_[rt_t][stop_idx * 2 - 1]);
    if (arr_event > event_time) {
      return;
    }
    auto const v_label_idx = state_.n_route_transports_ * state_.n_days_ + rt_t_idx;
    auto& v_label = state_.vehicle_labels_[v_label_idx];
    if (v_label.time_stamp_ == state_.current_time_stamp_ &&
        v_label.arrival_time_ >= arr_event) {
      ++stats_.pruned_scans_;
      return;
    }
    v_label.arrival_time_ = arr_event;
    v_label.time_stamp_ = state_.current_time_stamp_;

    for (auto target_idx = static_cast<int>(stop_idx) - 1; target_idx >= 0; --target_idx) {
      ++stats_.scanned_stop_events_;
      if (require_bike_ && !rtt_->rt_transport_bikes_allowed_.test(rt_t_idx * 2)) {
        break;
      }
      if (require_car_ && !rtt_->rt_transport_cars_allowed_.test(rt_t_idx * 2)) {
        break;
      }
      auto const target_stp = stop(stop_seq[static_cast<unsigned>(target_idx)]);
      if (!target_stp.can_finish<direction::kBackward>(is_wheelchair)) {
        continue;
      }
      auto const dep_time = static_cast<delta_t>(
          day_offset * 1440 + rtt_->rt_transport_stop_times_[rt_t][static_cast<unsigned>(target_idx) * 2]);
      update_rt_label(target_stp.location_idx(), dep_time,
                      stop(stop_seq[stop_idx]).location_idx(), rt_t);
    }
  }

  timetable const& tt_;
  rt_timetable const* rtt_{nullptr};
  tad_state& state_;
  transfer_time_settings transfer_time_settings_;
  profile_idx_t prf_idx_{0};
  clasz_mask_t allowed_claszes_{all_clasz_allowed()};
  bool require_bike_{false};
  bool require_car_{false};
  tad_stats stats_;
  day_idx_t search_base_day_idx_{0};
  direction search_dir_{direction::kForward};
  bool is_first_leg_{true};
  delta_t query_delta_{0};
  duration_t max_travel_time_{kMaxTravelTime};
  std::uint8_t max_transfers_{kMaxTransfers};
  std::vector<uint8_t>* route_active_{nullptr};
  bitfield search_mask_{};
  std::priority_queue<tad_pq_label, std::vector<tad_pq_label>, tad_pq_cmp> pq_;
  bool has_best_target_time_{false};
  delta_t best_target_time_{0};
  std::unique_ptr<tad_transfer_provider> transfer_provider_;
  std::vector<tad_transfer_edge> transfer_edges_buf_;
};

}  // namespace nigiri::routing
