#include "nigiri/routing/one_to_all.h"

#include <chrono>
#include <ranges>
#include <utility>
#include <variant>
#include <vector>

#include "utl/verify.h"

#include "nigiri/for_each_meta.h"
#include "nigiri/routing/raptor/raptor.h"
#include "nigiri/routing/start_times.h"

namespace nigiri::routing {

constexpr auto const kVias = via_offset_t{0U};

day_idx_t make_base(timetable const& tt, unixtime_t start_time) {
  return day_idx_t{std::chrono::duration_cast<date::days>(
                       std::chrono::round<std::chrono::days>(start_time) -
                       tt.internal_interval().from_)
                       .count()};
}

template <direction SearchDir, bool Rt>
void run_raptor(raptor<SearchDir, Rt, kVias, search_mode::kOneToAll>&& algo,
                timetable const& tt,
                rt_timetable const* rtt,
                unixtime_t const& start_time,
                unixtime_t const worst_time_at_dest,
                query const& q) {
  auto results = pareto_set<journey>{};
  algo.next_start_time();
  for (auto const& s : q.start_) {
    auto const t = SearchDir == direction::kForward ? start_time + s.duration()
                                                    : start_time - s.duration();
    trace("init: time_at_stop={} at {}\n", t, location_idx_t{s.target()});
    nigiri::routing::for_each_meta(
        tt, rt_virts(rtt, q.prf_idx_), q.start_match_mode_, s.target(),
        [&](nigiri::location_idx_t const l) { algo.add_start(l, t); });
  }

  auto worst = std::array<unixtime_t, kMaxTransfers + 2U>{};
  worst.fill(worst_time_at_dest);

  algo.execute(start_time, q.max_transfers_, worst, results);
}

template <direction SearchDir>
void finish_one_to_all(delta_t const worst_at_dest, raptor_state& state) {
  constexpr auto const kInvalid = kInvalidDelta<SearchDir>;
  auto const is_better = [](delta_t const a, delta_t const b) {
    return SearchDir == direction::kForward ? a < b : a > b;
  };

  auto best = state.get_best<kVias>();
  auto& arrivals = state.one_to_all_arrivals_;

  // Rows from one_to_all_dirty_rows_ on were not reached by this query and
  // are kInvalid already. Rows are visited in order (sequential access); per
  // location the rounds are still taken in ascending order.
  auto const n_rows = state.one_to_all_dirty_rows_;
  for (auto l = location_idx_t{0U}; l != state.n_locations_; ++l) {
    best[to_idx(l)][kVias] = kInvalid;
  }
  for (auto k = 0U; k != n_rows; ++k) {
    auto* const row = arrivals.entries_.data() +
                      static_cast<std::size_t>(k) * arrivals.n_columns_;
    for (auto l = 0U; l != state.n_locations_; ++l) {
      auto& b = best[l][kVias];
      auto& t = row[l];
      if (is_better(t, b) && !is_better(worst_at_dest, t)) {
        b = t;
      } else {
        t = kInvalid;
      }
    }
  }
}

template <direction SearchDir, bool Rt>
void one_to_all(timetable const& tt,
                rt_timetable const* rtt,
                query const& q,
                raptor_state& state) {
  utl::verify(std::holds_alternative<unixtime_t>(q.start_time_),
              "Start-time must be a time point (unixtime_t)");
  utl::verify(q.via_stops_.empty(),
              "One-to-All search not supported with vias");
  auto const& start_time = std::get<unixtime_t>(q.start_time_);

  auto is_dest = bitvec{rtt == nullptr ? tt.n_locations() : rtt->n_locations()};
  auto is_via = std::array<bitvec, kMaxVias>{};
  auto dist_to_dest = std::vector<std::uint16_t>{};
  auto lb = std::vector<std::uint16_t>(
      rtt == nullptr ? tt.n_locations() : rtt->n_locations(), 0U);
  auto const base = make_base(tt, start_time);
  auto const is_wheelchair = q.prf_idx_ == kWheelchairProfile;

  auto r = raptor<SearchDir, Rt, kVias, search_mode::kOneToAll>{
      tt,
      rtt,
      state,
      is_dest,
      is_via,
      dist_to_dest,
      q.td_dest_,
      lb,
      q.via_stops_,
      base,
      q.allowed_claszes_,
      q.require_bike_transport_,
      q.require_car_transport_,
      is_wheelchair,
      q.no_compulsory_reservation_,
      q.transfer_time_settings_,
      q.prf_idx_};

  auto const worst_time_at_dest =
      start_time +
      (SearchDir == direction::kForward ? 1 : -1) * q.max_travel_time_;
  auto const worst_at_dest = r.to_delta(worst_time_at_dest);
  run_raptor(std::move(r), tt, rtt, start_time, worst_time_at_dest, q);
  finish_one_to_all<SearchDir>(worst_at_dest, state);
}

template <direction SearchDir>
void one_to_all(timetable const& tt,
                rt_timetable const* rtt,
                query const& q,
                raptor_state& state) {
  if (rtt == nullptr) {
    one_to_all<SearchDir, false>(tt, rtt, q, state);
  } else {
    one_to_all<SearchDir, true>(tt, rtt, q, state);
  }
}

template <direction SearchDir>
raptor_state one_to_all(timetable const& tt,
                        rt_timetable const* rtt,
                        query const& q) {
  auto state = raptor_state{};
  one_to_all<SearchDir>(tt, rtt, q, state);
  return state;
}

fastest_offset get_fastest_one_to_all_offsets(timetable const& tt,
                                              raptor_state const& state,
                                              direction const search_dir,
                                              location_idx_t const l,
                                              unixtime_t const start_time,
                                              std::uint8_t const transfers) {
  auto const invalid_delta = search_dir == direction::kForward
                                 ? kInvalidDelta<direction::kForward>
                                 : kInvalidDelta<direction::kBackward>;
  auto const& arrivals = state.one_to_all_arrivals_;
  for (auto const k : std::views::iota(std::uint8_t{0U}, transfers + 2U)  //
                          | std::views::reverse) {
    if (arrivals[k][to_idx(l)] != invalid_delta) {
      auto const base =
          tt.internal_interval_days().from_ +
          static_cast<int>(to_idx(make_base(tt, start_time))) * date::days{1};
      auto end_time = delta_to_unix(base, arrivals[k][to_idx(l)]);
      return {
          .duration_ = static_cast<delta_t>((end_time - start_time).count()),
          .k_ = k,
      };
    }
  }
  return {};
};

void for_each_one_to_all_round_time(
    timetable const& tt,
    raptor_state const& state,
    direction const search_dir,
    location_idx_t const l,
    unixtime_t const start_time,
    std::uint8_t const transfers,
    std::function<void(std::uint8_t, duration_t)> const& cb) {
  auto const invalid_delta = search_dir == direction::kForward
                                 ? kInvalidDelta<direction::kForward>
                                 : kInvalidDelta<direction::kBackward>;
  auto const& arrivals = state.one_to_all_arrivals_;
  auto const base =
      tt.internal_interval_days().from_ +
      static_cast<int>(to_idx(make_base(tt, start_time))) * date::days{1};
  for (auto const k : std::views::iota(std::uint8_t{0U}, transfers + 2U)) {
    if (arrivals[k][to_idx(l)] != invalid_delta) {
      auto end_time = delta_to_unix(base, arrivals[k][to_idx(l)]);
      cb(k, search_dir == direction::kForward ? end_time - start_time
                                              : start_time - end_time);
    }
  }
}

template raptor_state one_to_all<direction::kForward>(timetable const&,
                                                      rt_timetable const*,
                                                      query const&);
template raptor_state one_to_all<direction::kBackward>(timetable const&,
                                                       rt_timetable const*,
                                                       query const&);
template void one_to_all<direction::kForward>(timetable const&,
                                              rt_timetable const*,
                                              query const&,
                                              raptor_state&);
template void one_to_all<direction::kBackward>(timetable const&,
                                               rt_timetable const*,
                                               query const&,
                                               raptor_state&);

}  // namespace nigiri::routing
