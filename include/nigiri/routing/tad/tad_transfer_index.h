#pragma once

#include <cstdint>

#include "nigiri/routing/tad/tad_transfer_provider.h"
#include "nigiri/types.h"

namespace nigiri::routing {

struct tad_profile_shortcuts {
  vector<std::uint32_t> out_offsets_;
  vector<tad_transfer_edge> out_edges_;
  vector<std::uint32_t> in_offsets_;
  vector<tad_transfer_edge> in_edges_;
  string transfer_time_fingerprint_;

  bool has_offsets_for(std::uint32_t const n_locations) const {
    return valid_offsets(out_offsets_, out_edges_, n_locations) &&
           valid_offsets(in_offsets_, in_edges_, n_locations);
  }

  static bool valid_offsets(vector<std::uint32_t> const& offsets,
                            vector<tad_transfer_edge> const& edges,
                            std::uint32_t const n_locations) {
    if (offsets.size() != n_locations + 1U || offsets.empty() ||
        offsets.front() != 0U || offsets.back() > edges.size()) {
      return false;
    }
    for (auto i = 1U; i != offsets.size(); ++i) {
      if (offsets[i] < offsets[i - 1U]) {
        return false;
      }
    }
    return true;
  }
};

struct tad_transfer_index {
  static constexpr std::uint32_t kVersion = 1U;

  std::uint32_t version_{kVersion};
  std::uint32_t n_locations_{0U};
  vector<tad_profile_shortcuts> profiles_;

  bool is_valid_for(profile_idx_t const prf_idx) const {
    auto const p_idx = cista::to_idx(prf_idx);
    return version_ == kVersion && p_idx < profiles_.size() &&
           profiles_[p_idx].has_offsets_for(n_locations_);
  }
};

struct shortcut_transfer_provider : public tad_transfer_provider {
  tad_profile_shortcuts const& shortcuts_;

  explicit shortcut_transfer_provider(tad_profile_shortcuts const& shortcuts)
      : shortcuts_{shortcuts} {}

  void stop_transfers(location_idx_t const l,
                      bool const fwd,
                      std::vector<tad_transfer_edge>& out) const override {
    auto const l_idx = cista::to_idx(l);

    auto const& offsets = fwd ? shortcuts_.out_offsets_ : shortcuts_.in_offsets_;
    auto const& edges = fwd ? shortcuts_.out_edges_ : shortcuts_.in_edges_;

    if (l_idx + 1 >= offsets.size()) return;
    auto const start = offsets[l_idx];
    auto const end = offsets[l_idx + 1];

    for (auto i = start; i < end; ++i) {
      out.push_back(edges[i]);
    }
  }
};

}  // namespace nigiri::routing
