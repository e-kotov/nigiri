#pragma once

#include <limits>
#include <queue>
#include <vector>

#include "nigiri/timetable.h"
#include "nigiri/types.h"

namespace nigiri::routing {

struct tad_transfer_edge {
  location_idx_t target_;
  duration_t duration_;
};

struct tad_transfer_provider {
  virtual ~tad_transfer_provider() = default;

  virtual char const* name() const = 0;

  virtual void stop_transfers(location_idx_t l, bool fwd,
                              std::vector<tad_transfer_edge>& out) const = 0;
};

struct footpath_transfer_provider : public tad_transfer_provider {
  timetable const& tt_;
  profile_idx_t prf_idx_;

  footpath_transfer_provider(timetable const& tt, profile_idx_t prf_idx)
      : tt_{tt}, prf_idx_{prf_idx} {}

  char const* name() const override { return "footpath_transfer_provider"; }

  void stop_transfers(location_idx_t const l, bool const fwd,
                      std::vector<tad_transfer_edge>& out) const override {
    auto const& fps = fwd ? tt_.locations_.footpaths_out_[prf_idx_][l]
                          : tt_.locations_.footpaths_in_[prf_idx_][l];
    for (auto const& fp : fps) {
      out.push_back({fp.target(), fp.duration()});
    }
  }
};

// Endpoint distances use the original directed graph, independently of any
// transit witness. Paths too long for one footpath remain available as base edges.
struct endpoint_transfer_provider : public tad_transfer_provider {
  timetable const& tt_;
  profile_idx_t prf_idx_;
  endpoint_transfer_provider(timetable const& tt, profile_idx_t const prf)
      : tt_{tt}, prf_idx_{prf} {}
  char const* name() const override { return "endpoint_transfer_provider"; }
  void stop_transfers(location_idx_t const source, bool const fwd,
                      std::vector<tad_transfer_edge>& out) const override {
    auto const& graph = fwd ? tt_.locations_.footpaths_out_[prf_idx_]
                            : tt_.locations_.footpaths_in_[prf_idx_];
    if (to_idx(source) >= tt_.n_locations()) return;
    auto const inf = std::numeric_limits<int>::max();
    std::vector<int> distance(tt_.n_locations(), inf);
    using entry = std::pair<int, location_idx_t>;
    std::priority_queue<entry, std::vector<entry>, std::greater<>> queue;
    distance[to_idx(source)] = 0;
    queue.emplace(0, source);
    while (!queue.empty()) {
      auto const [d, u] = queue.top(); queue.pop();
      if (d != distance[to_idx(u)]) continue;
      if (u != source) out.push_back({u, duration_t{static_cast<std::int16_t>(d)}});
      if (to_idx(u) >= graph.size()) continue;
      for (auto const& edge : graph[u]) {
        auto const v = edge.target();
        auto const next = d + edge.duration().count();
        if (to_idx(v) < distance.size() && next <= footpath::kMaxDuration.count() &&
            next < distance[to_idx(v)]) {
          distance[to_idx(v)] = next;
          queue.emplace(next, v);
        }
      }
    }
  }
};

}  // namespace nigiri::routing
