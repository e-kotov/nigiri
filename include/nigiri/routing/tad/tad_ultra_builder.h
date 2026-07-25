#pragma once

#include <algorithm>
#include <chrono>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <vector>

#include "nigiri/routing/tad/tad_station_grouping.h"
#include "nigiri/routing/tad/tad_transfer_index.h"
#include "nigiri/stop.h"
#include "nigiri/timetable.h"
#include "nigiri/types.h"

namespace nigiri::routing {

inline duration_t safe_duration_cast(std::int64_t const minutes) {
  constexpr auto kMax = static_cast<std::int64_t>(std::numeric_limits<std::int16_t>::max());
  constexpr auto kMin = static_cast<std::int64_t>(std::numeric_limits<std::int16_t>::min());
  if (minutes > kMax) {
    return duration_t{static_cast<std::int16_t>(kMax)};
  }
  if (minutes < kMin) {
    return duration_t{static_cast<std::int16_t>(kMin)};
  }
  return duration_t{static_cast<std::int16_t>(minutes)};
}

inline duration_t safe_duration_convert(duration_t const d) {
  return safe_duration_cast(d.count());
}

inline duration_t safe_duration_add(duration_t const a, duration_t const b) {
  auto const sum = static_cast<std::int64_t>(a.count()) + static_cast<std::int64_t>(b.count());
  return safe_duration_cast(sum);
}

inline duration_t safe_limit_duration(int const limit_minutes) {
  if (limit_minutes < 0) {
    return duration_t{0};
  }
  return safe_duration_cast(static_cast<std::int64_t>(limit_minutes));
}

struct tad_ultra_builder {
  timetable const& tt_;
  int witness_transfer_limit_minutes_{15};

  tad_transfer_index build() const {
    tad_transfer_index idx;
    idx.version_ = tad_transfer_index::kVersion;
    idx.n_locations_ = tt_.n_locations();

    auto const n_profiles = tt_.locations_.footpaths_out_.size();
    idx.profiles_.resize(n_profiles);

    auto const n_locs = tt_.n_locations();

    for (profile_idx_t prf = 0; prf < n_profiles; ++prf) {
      auto& prf_shortcuts = idx.profiles_[cista::to_idx(prf)];
      prf_shortcuts.transfer_time_fingerprint_ = "ultra_witness_shortcuts_v1";

      auto const grouping = build_station_grouping(tt_, prf);

      std::vector<std::map<location_idx_t, duration_t>> out_shortcuts(n_locs);

      if (prf < tt_.locations_.footpaths_out_.size()) {
        auto const& footpaths_out = tt_.locations_.footpaths_out_[prf];
        for (std::size_t i = 0; i < n_locs; ++i) {
          auto const src = location_idx_t{static_cast<std::uint32_t>(i)};
          if (i < footpaths_out.size()) {
            for (auto const& fp : footpaths_out[src]) {
              auto const target = fp.target();
              if (cista::to_idx(target) < n_locs && target != src) {
                auto const dur = fp.duration();
                auto it = out_shortcuts[i].find(target);
                if (it == out_shortcuts[i].end() || dur < it->second) {
                  out_shortcuts[i][target] = dur;
                }
              }
            }
          }
        }
      }

      run_witness_search(prf, grouping, out_shortcuts);

      prf_shortcuts.out_offsets_.resize(n_locs + 1U, 0U);
      for (std::size_t i = 0; i < n_locs; ++i) {
        for (auto const& [target, dur] : out_shortcuts[i]) {
          prf_shortcuts.out_edges_.push_back(
              tad_transfer_edge{target, safe_duration_convert(dur)});
        }
        prf_shortcuts.out_offsets_[i + 1U] =
            static_cast<std::uint32_t>(prf_shortcuts.out_edges_.size());
      }

      std::vector<std::map<location_idx_t, duration_t>> in_shortcuts(n_locs);
      for (std::size_t i = 0; i < n_locs; ++i) {
        auto const src = location_idx_t{static_cast<std::uint32_t>(i)};
        for (auto const& [target, dur] : out_shortcuts[i]) {
          auto const target_idx = cista::to_idx(target);
          auto it = in_shortcuts[target_idx].find(src);
          if (it == in_shortcuts[target_idx].end() || dur < it->second) {
            in_shortcuts[target_idx][src] = dur;
          }
        }
      }

      prf_shortcuts.in_offsets_.resize(n_locs + 1U, 0U);
      for (std::size_t i = 0; i < n_locs; ++i) {
        for (auto const& [src, dur] : in_shortcuts[i]) {
          prf_shortcuts.in_edges_.push_back(
              tad_transfer_edge{src, safe_duration_convert(dur)});
        }
        prf_shortcuts.in_offsets_[i + 1U] =
            static_cast<std::uint32_t>(prf_shortcuts.in_edges_.size());
      }
    }

    return idx;
  }

private:
  void run_witness_search(
      profile_idx_t const prf,
      station_grouping const& grouping,
      std::vector<std::map<location_idx_t, duration_t>>& out_shortcuts) const {
    auto const n_locs = tt_.n_locations();
    if (n_locs == 0) return;

    struct label {
      std::uint32_t ts_0_{0};
      std::uint32_t ts_1_{0};
      std::uint32_t ts_2_{0};
      unixtime_t arr_0_{unixtime_t::max()};
      unixtime_t arr_1_{unixtime_t::max()};
      unixtime_t arr_2_{unixtime_t::max()};
      location_idx_t transfer_parent_1_{location_idx_t::invalid()};
      location_idx_t route_parent_2_{location_idx_t::invalid()};
    };

    std::vector<label> labels(n_locs);
    std::uint32_t timestamp = 0;

    auto const kMaxTime = unixtime_t::max();
    auto const limit_dur = safe_limit_duration(witness_transfer_limit_minutes_);

    auto const get_arr_0 = [&](location_idx_t l) {
      auto const idx = cista::to_idx(l);
      return (idx < n_locs && labels[idx].ts_0_ == timestamp) ? labels[idx].arr_0_ : kMaxTime;
    };
    auto const get_arr_1 = [&](location_idx_t l) {
      auto const idx = cista::to_idx(l);
      return (idx < n_locs && labels[idx].ts_1_ == timestamp) ? labels[idx].arr_1_ : kMaxTime;
    };
    auto const get_arr_2 = [&](location_idx_t l) {
      auto const idx = cista::to_idx(l);
      return (idx < n_locs && labels[idx].ts_2_ == timestamp) ? labels[idx].arr_2_ : kMaxTime;
    };

    auto const* footpaths_out = prf < tt_.locations_.footpaths_out_.size()
                                    ? &tt_.locations_.footpaths_out_[prf]
                                    : nullptr;

    auto const n_days = static_cast<std::size_t>(tt_.internal_interval_days().size().count());

    for (auto const& st : grouping.stations_) {
      auto const rep = st.representative_;
      if (rep == location_idx_t::invalid() || cista::to_idx(rep) >= n_locs) continue;

      std::vector<duration_t> direct_dist(n_locs, duration_t::max());
      std::vector<location_idx_t> direct_stops;

      {
        struct heap_entry {
          location_idx_t loc_;
          duration_t dist_;
          bool operator>(heap_entry const& o) const { return dist_ > o.dist_; }
        };
        std::priority_queue<heap_entry, std::vector<heap_entry>, std::greater<>> pq;

        direct_dist[cista::to_idx(rep)] = duration_t{0};
        direct_stops.push_back(rep);
        pq.push({rep, duration_t{0}});

        while (!pq.empty()) {
          auto const [u, d] = pq.top();
          pq.pop();

          auto const u_idx = cista::to_idx(u);
          if (d > direct_dist[u_idx]) continue;

          if (footpaths_out != nullptr && u_idx < footpaths_out->size()) {
            for (auto const& fp : (*footpaths_out)[u]) {
              auto const v = fp.target();
              auto const v_idx = cista::to_idx(v);
              if (v_idx < n_locs) {
                auto const new_d = safe_duration_add(d, fp.duration());
                if (new_d < direct_dist[v_idx]) {
                  if (direct_dist[v_idx] == duration_t::max()) {
                    direct_stops.push_back(v);
                  }
                  direct_dist[v_idx] = new_d;
                  pq.push({v, new_d});
                }
              }
            }
          }
        }
      }

      std::set<unixtime_t> departure_times;

      for (auto const u : direct_stops) {
        auto const u_idx = cista::to_idx(u);
        auto const walk_d = direct_dist[u_idx];
        if (u_idx >= tt_.location_routes_.size()) continue;

        for (auto const r : tt_.location_routes_[u]) {
          auto const stop_seq = tt_.route_location_seq_[r];
          stop_idx_t stop_idx = 0;
          for (std::size_t s = 0; s < stop_seq.size(); ++s) {
            if (stop{stop_seq[s]}.location_idx() == u) {
              stop_idx = static_cast<stop_idx_t>(s);
              break;
            }
          }

          auto const transports = tt_.route_transport_ranges_[r];
          for (auto t = transports.from_; t != transports.to_; ++t) {
            auto const traffic_days = tt_.transport_traffic_days_[t];
            for (std::size_t d = 0; d < n_days; ++d) {
              if (tt_.bitfields_[traffic_days].test(d)) {
                auto const day = day_idx_t{static_cast<std::uint16_t>(d)};
                auto const dep = tt_.event_time(transport{t, day}, stop_idx, event_type::kDep);
                if (dep != unixtime_t::max() && dep >= unixtime_t{0_minutes} + walk_d) {
                  departure_times.insert(dep - walk_d);
                }
              }
            }
          }
        }
      }

      for (auto const dep_time : departure_times) {
        ++timestamp;

        for (auto const u : direct_stops) {
          auto const u_idx = cista::to_idx(u);
          labels[u_idx].ts_0_ = timestamp;
          labels[u_idx].arr_0_ = dep_time + direct_dist[u_idx];
        }

        std::set<route_idx_t> r1_routes;
        for (auto const u : direct_stops) {
          auto const arr_0_u = get_arr_0(u);
          if (arr_0_u == kMaxTime) continue;
          auto const u_idx = cista::to_idx(u);
          if (u_idx >= tt_.location_routes_.size()) continue;

          for (auto const r : tt_.location_routes_[u]) {
            r1_routes.insert(r);
          }
        }

        std::vector<location_idx_t> r1_updated_stops;
        for (auto const r : r1_routes) {
          auto const stop_seq = tt_.route_location_seq_[r];
          auto const transports = tt_.route_transport_ranges_[r];

          for (std::size_t i = 0; i < stop_seq.size(); ++i) {
            auto const board_stop = stop{stop_seq[i]}.location_idx();
            auto const arr_0_board = get_arr_0(board_stop);
            if (arr_0_board == kMaxTime) continue;

            auto const board_stop_idx = static_cast<stop_idx_t>(i);

            for (auto t = transports.from_; t != transports.to_; ++t) {
              auto const traffic_days = tt_.transport_traffic_days_[t];
              for (std::size_t d = 0; d < n_days; ++d) {
                if (!tt_.bitfields_[traffic_days].test(d)) continue;
                auto const day = day_idx_t{static_cast<std::uint16_t>(d)};
                auto const dep = tt_.event_time(transport{t, day}, board_stop_idx, event_type::kDep);
                if (dep >= arr_0_board) {
                  for (std::size_t j = i + 1; j < stop_seq.size(); ++j) {
                    auto const arr_stop = stop{stop_seq[j]}.location_idx();
                    auto const arr_stop_idx = static_cast<stop_idx_t>(j);
                    auto const arr_t = tt_.event_time(transport{t, day}, arr_stop_idx, event_type::kArr);
                    if (arr_t < get_arr_1(arr_stop)) {
                      auto const arr_idx = cista::to_idx(arr_stop);
                      labels[arr_idx].ts_1_ = timestamp;
                      labels[arr_idx].arr_1_ = arr_t;
                      labels[arr_idx].transfer_parent_1_ = arr_stop;
                      r1_updated_stops.push_back(arr_stop);
                    }
                  }
                  break;
                }
              }
            }
          }
        }

        if (r1_updated_stops.empty()) continue;

        std::vector<location_idx_t> t1_updated_stops;
        {
          struct heap_entry {
            location_idx_t loc_;
            unixtime_t arr_;
            bool operator>(heap_entry const& o) const { return arr_ > o.arr_; }
          };
          std::priority_queue<heap_entry, std::vector<heap_entry>, std::greater<>> pq;

          for (auto const u : r1_updated_stops) {
            pq.push({u, get_arr_1(u)});
          }

          auto const limit_dur = duration_t{witness_transfer_limit_minutes_};

          while (!pq.empty()) {
            auto const [u, arr] = pq.top();
            pq.pop();

            if (arr > get_arr_1(u)) continue;
            t1_updated_stops.push_back(u);

            auto const u_idx = cista::to_idx(u);
            if (footpaths_out != nullptr && u_idx < footpaths_out->size()) {
              auto const parent = labels[u_idx].transfer_parent_1_;
              auto const parent_arr = get_arr_1(parent);
              for (auto const& fp : (*footpaths_out)[u]) {
                auto const v = fp.target();
                auto const new_arr = arr + fp.duration();
                if (parent_arr != kMaxTime && new_arr >= parent_arr &&
                    (new_arr - parent_arr) <= limit_dur) {
                  if (new_arr < get_arr_1(v)) {
                    auto const v_idx = cista::to_idx(v);
                    labels[v_idx].ts_1_ = timestamp;
                    labels[v_idx].arr_1_ = new_arr;
                    labels[v_idx].transfer_parent_1_ = parent;
                    pq.push({v, new_arr});
                  }
                }
              }
            }
          }
        }

        if (t1_updated_stops.empty()) continue;

        std::set<route_idx_t> r2_routes;
        for (auto const u : t1_updated_stops) {
          auto const u_idx = cista::to_idx(u);
          if (u_idx >= tt_.location_routes_.size()) continue;

          for (auto const r : tt_.location_routes_[u]) {
            r2_routes.insert(r);
          }
        }

        std::vector<location_idx_t> r2_updated_stops;
        for (auto const r : r2_routes) {
          auto const stop_seq = tt_.route_location_seq_[r];
          auto const transports = tt_.route_transport_ranges_[r];

          for (std::size_t i = 0; i < stop_seq.size(); ++i) {
            auto const board_stop = stop{stop_seq[i]}.location_idx();
            auto const arr_1_board = get_arr_1(board_stop);
            if (arr_1_board == kMaxTime) continue;

            auto const board_stop_idx = static_cast<stop_idx_t>(i);

            for (auto t = transports.from_; t != transports.to_; ++t) {
              auto const traffic_days = tt_.transport_traffic_days_[t];
              for (std::size_t d = 0; d < n_days; ++d) {
                if (!tt_.bitfields_[traffic_days].test(d)) continue;
                auto const day = day_idx_t{static_cast<std::uint16_t>(d)};
                auto const dep = tt_.event_time(transport{t, day}, board_stop_idx, event_type::kDep);
                if (dep >= arr_1_board) {
                  auto const board_idx = cista::to_idx(board_stop);
                  auto const transfer_parent = labels[board_idx].transfer_parent_1_;

                  for (std::size_t j = i + 1; j < stop_seq.size(); ++j) {
                    auto const arr_stop = stop{stop_seq[j]}.location_idx();
                    auto const arr_stop_idx = static_cast<stop_idx_t>(j);
                    auto const arr_t = tt_.event_time(transport{t, day}, arr_stop_idx, event_type::kArr);
                    if (arr_t < get_arr_2(arr_stop)) {
                      auto const arr_idx = cista::to_idx(arr_stop);
                      labels[arr_idx].ts_2_ = timestamp;
                      labels[arr_idx].arr_2_ = arr_t;
                      if (transfer_parent != location_idx_t::invalid() && transfer_parent != board_stop) {
                        labels[arr_idx].route_parent_2_ = board_stop;
                      } else {
                        labels[arr_idx].route_parent_2_ = location_idx_t::invalid();
                      }
                      r2_updated_stops.push_back(arr_stop);
                    }
                  }
                  break;
                }
              }
            }
          }
        }

        if (r2_updated_stops.empty()) continue;

        {
          struct heap_entry {
            location_idx_t loc_;
            unixtime_t arr_;
            bool operator>(heap_entry const& o) const { return arr_ > o.arr_; }
          };
          std::priority_queue<heap_entry, std::vector<heap_entry>, std::greater<>> pq;

          for (auto const u : r2_updated_stops) {
            pq.push({u, get_arr_2(u)});
          }

          while (!pq.empty()) {
            auto const [w, arr] = pq.top();
            pq.pop();

            if (arr > get_arr_2(w)) continue;

            auto const w_idx = cista::to_idx(w);
            auto const route_parent = labels[w_idx].route_parent_2_;
            if (route_parent != location_idx_t::invalid()) {
              auto const rp_idx = cista::to_idx(route_parent);
              auto const transfer_parent = labels[rp_idx].transfer_parent_1_;
              if (transfer_parent != location_idx_t::invalid() && transfer_parent != route_parent) {
                auto const arr_u = get_arr_1(transfer_parent);
                auto const arr_y = get_arr_1(route_parent);
                if (arr_u != kMaxTime && arr_y != kMaxTime && arr_y >= arr_u) {
                  auto const dur_minutes =
                      std::chrono::duration_cast<std::chrono::minutes>(arr_y - arr_u).count();
                  auto const dur = safe_duration_cast(dur_minutes);
                  auto const tp_idx = cista::to_idx(transfer_parent);
                  auto it = out_shortcuts[tp_idx].find(route_parent);
                  if (it == out_shortcuts[tp_idx].end() || dur < it->second) {
                    out_shortcuts[tp_idx][route_parent] = dur;
                  }
                }
              }
              labels[w_idx].route_parent_2_ = location_idx_t::invalid();
            }

            if (footpaths_out != nullptr && w_idx < footpaths_out->size()) {
              for (auto const& fp : (*footpaths_out)[w]) {
                auto const v = fp.target();
                auto const new_arr = arr + fp.duration();
                if (new_arr < get_arr_2(v)) {
                  auto const v_idx = cista::to_idx(v);
                  labels[v_idx].ts_2_ = timestamp;
                  labels[v_idx].arr_2_ = new_arr;
                  labels[v_idx].route_parent_2_ = route_parent;
                  pq.push({v, new_arr});
                }
              }
            }
          }
        }
      }
    }
  }
};

inline tad_transfer_index build_tad_ultra_index(timetable const& tt,
                                                int const witness_transfer_limit_minutes = 15) {
  return tad_ultra_builder{tt, witness_transfer_limit_minutes}.build();
}

}  // namespace nigiri::routing
