#pragma once

#include <algorithm>
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

struct dsu {
  explicit dsu(std::size_t const n) : parent_(n) {
    for (std::size_t i = 0; i < n; ++i) {
      parent_[i] = static_cast<std::uint32_t>(i);
    }
  }

  std::uint32_t find(std::uint32_t const i) {
    auto root = i;
    while (root != parent_[root]) {
      root = parent_[root];
    }
    auto curr = i;
    while (curr != root) {
      auto const next = parent_[curr];
      parent_[curr] = root;
      curr = next;
    }
    return root;
  }

  void unite(std::uint32_t const i, std::uint32_t const j) {
    auto const root_i = find(i);
    auto const root_j = find(j);
    if (root_i != root_j) {
      if (root_i < root_j) {
        parent_[root_j] = root_i;
      } else {
        parent_[root_i] = root_j;
      }
    }
  }

  std::vector<std::uint32_t> parent_;
};

inline station_grouping build_station_grouping(timetable const& tt,
                                                profile_idx_t const prf) {
  auto const n_locs = tt.n_locations();
  station_grouping grouping;
  grouping.representative_of_stop_.resize(n_locs, location_idx_t::invalid());

  dsu sets(n_locs);

  for (std::size_t i = 0; i < n_locs; ++i) {
    auto const l = location_idx_t{static_cast<std::uint32_t>(i)};
    auto const p = tt.locations_.parents_[l];
    if (p != location_idx_t::invalid() && cista::to_idx(p) < n_locs) {
      sets.unite(static_cast<std::uint32_t>(i), cista::to_idx(p));
    }
    for (auto const eq : tt.locations_.equivalences_[l]) {
      if (cista::to_idx(eq) < n_locs) {
        sets.unite(static_cast<std::uint32_t>(i), cista::to_idx(eq));
      }
    }
  }

  if (prf < tt.locations_.footpaths_out_.size()) {
    auto const& footpaths_out = tt.locations_.footpaths_out_[prf];
    for (std::size_t i = 0; i < n_locs; ++i) {
      auto const l = location_idx_t{static_cast<std::uint32_t>(i)};
      if (i < footpaths_out.size()) {
        for (auto const& fp : footpaths_out[l]) {
          if (fp.duration() == 0_minutes) {
            auto const target = fp.target();
            if (cista::to_idx(target) < n_locs) {
              sets.unite(static_cast<std::uint32_t>(i), cista::to_idx(target));
            }
          }
        }
      }
    }
  }

  std::vector<std::vector<location_idx_t>> members_by_rep(n_locs);
  for (std::size_t i = 0; i < n_locs; ++i) {
    auto const rep_idx = sets.find(static_cast<std::uint32_t>(i));
    auto const rep = location_idx_t{rep_idx};
    grouping.representative_of_stop_[i] = rep;
    members_by_rep[rep_idx].push_back(location_idx_t{static_cast<std::uint32_t>(i)});
  }

  for (std::size_t i = 0; i < n_locs; ++i) {
    if (!members_by_rep[i].empty()) {
      station_grouping::station st;
      st.representative_ = location_idx_t{static_cast<std::uint32_t>(i)};
      st.member_stops_ = std::move(members_by_rep[i]);
      grouping.stations_.push_back(std::move(st));
    }
  }

  return grouping;
}

}  // namespace nigiri::routing
