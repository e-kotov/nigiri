#pragma once

#include <functional>
#include <limits>
#include <queue>
#include <vector>

#include "nigiri/routing/tad/tad_transfer_index.h"
#include "nigiri/timetable.h"
#include "nigiri/types.h"

namespace nigiri::routing {

inline tad_transfer_index build_tad_transfer_index(timetable const& tt) {
  tad_transfer_index idx;
  idx.version_ = tad_transfer_index::kVersion;
  idx.n_locations_ = tad_transfer_index::expected_n_locations(tt);
  idx.witness_limit_minutes_ = 0U;
  idx.timetable_n_routes_ = static_cast<std::uint64_t>(tt.route_location_seq_.size());
  idx.timetable_n_transports_ = static_cast<std::uint64_t>(tt.transport_traffic_days_.size());
  idx.timetable_fingerprint_ = tad_transfer_index::compute_tt_fingerprint(tt);
  // Derived from tt itself, so it matches by construction.
  idx.timetable_validated_ = true;

  auto const n_profiles = tt.locations_.footpaths_out_.size();
  idx.profiles_.resize(n_profiles);

  auto constexpr kInfDuration = i32_minutes{std::numeric_limits<std::int32_t>::max()};
  auto constexpr kMaxShortcutDuration = i32_minutes{std::numeric_limits<std::int16_t>::max()};

  for (profile_idx_t prf = 0; prf < n_profiles; ++prf) {
    auto& prf_shortcuts = idx.profiles_[cista::to_idx(prf)];
    prf_shortcuts.transfer_time_fingerprint_ = "transitive_footpath_closure_v1";

    auto const n_locs = tt.n_locations();
    prf_shortcuts.out_offsets_.resize(n_locs + 1U, 0U);
    prf_shortcuts.in_offsets_.resize(n_locs + 1U, 0U);

    struct dist_label {
      location_idx_t loc_;
      i32_minutes dist_;
      bool operator>(dist_label const& o) const { return dist_ > o.dist_; }
    };

    {
      std::vector<i32_minutes> dist(n_locs, kInfDuration);
      std::vector<location_idx_t> visited;
      auto const& footpaths_out = tt.locations_.footpaths_out_[prf];

      for (std::size_t l = 0; l < n_locs; ++l) {
        auto const src_loc = location_idx_t{static_cast<std::uint32_t>(l)};
        dist[l] = i32_minutes{0};
        visited.push_back(src_loc);

        std::priority_queue<dist_label, std::vector<dist_label>, std::greater<>> pq;
        pq.push({src_loc, i32_minutes{0}});

        while (!pq.empty()) {
          auto const top = pq.top();
          pq.pop();

          if (top.dist_ > dist[cista::to_idx(top.loc_)]) {
            continue;
          }

          auto const top_loc_idx = cista::to_idx(top.loc_);
          if (top_loc_idx < footpaths_out.size()) {
            auto const& fps = footpaths_out[top.loc_];
            for (auto const& fp : fps) {
              auto const target = fp.target();
              auto const new_dist = top.dist_ + i32_minutes{fp.duration().count()};
              auto const target_idx = cista::to_idx(target);

              if (target_idx < n_locs && new_dist < dist[target_idx]) {
                if (dist[target_idx] == kInfDuration) {
                  visited.push_back(target);
                }
                dist[target_idx] = new_dist;
                pq.push({target, new_dist});
              }
            }
          }
        }

        for (auto const target : visited) {
          auto const target_dist = dist[cista::to_idx(target)];
          if (target != src_loc && target_dist <= kMaxShortcutDuration) {
            prf_shortcuts.out_edges_.push_back(tad_transfer_edge{
                target, duration_t{static_cast<std::int16_t>(target_dist.count())}});
          }
          dist[cista::to_idx(target)] = kInfDuration;
        }
        visited.clear();
        prf_shortcuts.out_offsets_[l + 1U] = static_cast<std::uint32_t>(
            prf_shortcuts.out_edges_.size());
      }
    }

    {
      std::vector<i32_minutes> dist(n_locs, kInfDuration);
      std::vector<location_idx_t> visited;
      auto const& footpaths_in = tt.locations_.footpaths_in_[prf];

      for (std::size_t l = 0; l < n_locs; ++l) {
        auto const src_loc = location_idx_t{static_cast<std::uint32_t>(l)};
        dist[l] = i32_minutes{0};
        visited.push_back(src_loc);

        std::priority_queue<dist_label, std::vector<dist_label>, std::greater<>> pq;
        pq.push({src_loc, i32_minutes{0}});

        while (!pq.empty()) {
          auto const top = pq.top();
          pq.pop();

          if (top.dist_ > dist[cista::to_idx(top.loc_)]) {
            continue;
          }

          auto const top_loc_idx = cista::to_idx(top.loc_);
          if (top_loc_idx < footpaths_in.size()) {
            auto const& fps = footpaths_in[top.loc_];
            for (auto const& fp : fps) {
              auto const target = fp.target();
              auto const new_dist = top.dist_ + i32_minutes{fp.duration().count()};
              auto const target_idx = cista::to_idx(target);

              if (target_idx < n_locs && new_dist < dist[target_idx]) {
                if (dist[target_idx] == kInfDuration) {
                  visited.push_back(target);
                }
                dist[target_idx] = new_dist;
                pq.push({target, new_dist});
              }
            }
          }
        }

        for (auto const target : visited) {
          auto const target_dist = dist[cista::to_idx(target)];
          if (target != src_loc && target_dist <= kMaxShortcutDuration) {
            prf_shortcuts.in_edges_.push_back(tad_transfer_edge{
                target, duration_t{static_cast<std::int16_t>(target_dist.count())}});
          }
          dist[cista::to_idx(target)] = kInfDuration;
        }
        visited.clear();
        prf_shortcuts.in_offsets_[l + 1U] = static_cast<std::uint32_t>(
            prf_shortcuts.in_edges_.size());
      }
    }
  }

  return idx;
}

}  // namespace nigiri::routing
