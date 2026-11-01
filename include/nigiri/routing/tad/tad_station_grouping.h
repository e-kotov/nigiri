#pragma once

#include <algorithm>
#include <cstdint>
#include <utility>
#include <vector>

#include "nigiri/timetable.h"
#include "nigiri/types.h"

namespace nigiri::routing {

struct station_grouping {
  struct station {
    location_idx_t representative_{location_idx_t::invalid()};
    std::vector<location_idx_t> member_stops_;
  };

  std::vector<location_idx_t> representative_of_stop_;
  std::vector<station> stations_;

  bool is_representative(location_idx_t const l) const {
    auto const idx = cista::to_idx(l);
    return idx < representative_of_stop_.size() &&
           representative_of_stop_[idx] == l;
  }

  location_idx_t representative(location_idx_t const l) const {
    auto const idx = cista::to_idx(l);
    return idx < representative_of_stop_.size()
               ? representative_of_stop_[idx]
               : location_idx_t::invalid();
  }
};

// A representative may replace a source only if every member can reach every
// other member at zero cost in this profile's directed transfer graph. Parent
// stations and equivalence metadata alone do not establish that property.
inline station_grouping build_station_grouping(timetable const& tt,
                                               profile_idx_t const prf) {
  auto const n_locs = tt.n_locations();
  station_grouping grouping;
  grouping.representative_of_stop_.resize(n_locs, location_idx_t::invalid());

  std::vector<std::vector<std::uint32_t>> outgoing(n_locs), incoming(n_locs);
  if (prf < tt.locations_.footpaths_out_.size()) {
    auto const& graph = tt.locations_.footpaths_out_[prf];
    for (std::size_t u = 0; u < n_locs && u < graph.size(); ++u) {
      for (auto const& edge : graph[location_idx_t{static_cast<std::uint32_t>(u)}]) {
        auto const v = cista::to_idx(edge.target());
        if (edge.duration() == 0_minutes && v < n_locs) {
          outgoing[u].push_back(v);
          incoming[v].push_back(static_cast<std::uint32_t>(u));
        }
      }
    }
  }

  // Iterative Kosaraju traversal avoids recursion on large
  // station/transfer networks. The reverse graph is derived from outgoing
  // footpaths, so no independently populated incoming table is required.
  std::vector<bool> seen(n_locs, false);
  std::vector<std::uint32_t> finish;
  std::vector<std::pair<std::uint32_t, std::size_t>> dfs;
  finish.reserve(n_locs);
  for (std::size_t source = 0; source < n_locs; ++source) {
    if (seen[source]) continue;
    seen[source] = true;
    dfs.emplace_back(static_cast<std::uint32_t>(source), 0U);
    while (!dfs.empty()) {
      auto& [u, next] = dfs.back();
      if (next == outgoing[u].size()) {
        finish.push_back(u);
        dfs.pop_back();
      } else {
        auto const v = outgoing[u][next++];
        if (!seen[v]) {
          seen[v] = true;
          dfs.emplace_back(v, 0U);
        }
      }
    }
  }

  std::fill(seen.begin(), seen.end(), false);
  std::vector<std::uint32_t> pending;
  for (auto it = finish.rbegin(); it != finish.rend(); ++it) {
    if (seen[*it]) continue;
    station_grouping::station station;
    auto representative = *it;
    pending.push_back(*it);
    seen[*it] = true;
    while (!pending.empty()) {
      auto const u = pending.back();
      pending.pop_back();
      representative = std::min(representative, u);
      station.member_stops_.push_back(location_idx_t{u});
      for (auto const v : incoming[u]) {
        if (!seen[v]) {
          seen[v] = true;
          pending.push_back(v);
        }
      }
    }
    station.representative_ = location_idx_t{representative};
    std::sort(station.member_stops_.begin(), station.member_stops_.end());
    for (auto const member : station.member_stops_) {
      grouping.representative_of_stop_[cista::to_idx(member)] = station.representative_;
    }
    grouping.stations_.push_back(std::move(station));
  }
  std::sort(grouping.stations_.begin(), grouping.stations_.end(),
            [](auto const& a, auto const& b) {
              return a.representative_ < b.representative_;
            });
  return grouping;
}

}  // namespace nigiri::routing
