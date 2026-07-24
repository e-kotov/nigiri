#pragma once

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

  virtual void stop_transfers(location_idx_t l, bool fwd,
                              std::vector<tad_transfer_edge>& out) const = 0;
};

struct footpath_transfer_provider : public tad_transfer_provider {
  timetable const& tt_;
  profile_idx_t prf_idx_;

  footpath_transfer_provider(timetable const& tt, profile_idx_t prf_idx)
      : tt_{tt}, prf_idx_{prf_idx} {}

  void stop_transfers(location_idx_t const l, bool const fwd,
                      std::vector<tad_transfer_edge>& out) const override {
    auto const& fps = fwd ? tt_.locations_.footpaths_out_[prf_idx_][l]
                          : tt_.locations_.footpaths_in_[prf_idx_][l];
    for (auto const& fp : fps) {
      out.push_back({fp.target(), fp.duration()});
    }
  }
};

}  // namespace nigiri::routing
