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

#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>

struct tad_transfer_index {
  static constexpr std::uint32_t kVersion = 2U;

  std::uint32_t version_{kVersion};
  std::uint32_t n_locations_{0U};
  std::uint32_t witness_limit_minutes_{0U};
  std::uint64_t timetable_n_routes_{0U};
  std::uint64_t timetable_n_transports_{0U};
  vector<tad_profile_shortcuts> profiles_;

  bool is_valid_for(profile_idx_t const prf_idx) const {
    auto const p_idx = cista::to_idx(prf_idx);
    return version_ == kVersion && p_idx < profiles_.size() &&
           profiles_[p_idx].has_offsets_for(n_locations_);
  }

  bool is_valid_for(timetable const& tt, profile_idx_t const prf_idx, int const witness_limit) const {
    return is_valid_for(prf_idx) &&
           n_locations_ == static_cast<std::uint32_t>(tt.locations_.parents_.size()) &&
           witness_limit_minutes_ == static_cast<std::uint32_t>(witness_limit) &&
           timetable_n_routes_ == static_cast<std::uint64_t>(tt.route_location_seq_.size()) &&
           timetable_n_transports_ == static_cast<std::uint64_t>(tt.transport_traffic_days_.size());
  }

  void write(std::filesystem::path const& path) const {
    std::ofstream out{path, std::ios::binary};
    if (!out.is_open()) return;

    out.write(reinterpret_cast<char const*>(&version_), sizeof(version_));
    out.write(reinterpret_cast<char const*>(&n_locations_), sizeof(n_locations_));
    out.write(reinterpret_cast<char const*>(&witness_limit_minutes_), sizeof(witness_limit_minutes_));
    out.write(reinterpret_cast<char const*>(&timetable_n_routes_), sizeof(timetable_n_routes_));
    out.write(reinterpret_cast<char const*>(&timetable_n_transports_), sizeof(timetable_n_transports_));

    auto const n_profiles = static_cast<std::uint32_t>(profiles_.size());
    out.write(reinterpret_cast<char const*>(&n_profiles), sizeof(n_profiles));

    for (auto const& prof : profiles_) {
      auto write_vec_u32 = [&](vector<std::uint32_t> const& vec) {
        std::uint64_t sz = vec.size();
        out.write(reinterpret_cast<char const*>(&sz), sizeof(sz));
        if (sz > 0) out.write(reinterpret_cast<char const*>(vec.data()), sz * sizeof(std::uint32_t));
      };
      auto write_vec_edges = [&](vector<tad_transfer_edge> const& vec) {
        std::uint64_t sz = vec.size();
        out.write(reinterpret_cast<char const*>(&sz), sizeof(sz));
        if (sz > 0) out.write(reinterpret_cast<char const*>(vec.data()), sz * sizeof(tad_transfer_edge));
      };

      write_vec_u32(prof.out_offsets_);
      write_vec_edges(prof.out_edges_);
      write_vec_u32(prof.in_offsets_);
      write_vec_edges(prof.in_edges_);
    }
  }

  static std::optional<tad_transfer_index> read(std::filesystem::path const& path) {
    std::ifstream in{path, std::ios::binary};
    if (!in.is_open()) return std::nullopt;

    tad_transfer_index idx;
    in.read(reinterpret_cast<char*>(&idx.version_), sizeof(idx.version_));
    if (idx.version_ != kVersion) return std::nullopt;

    in.read(reinterpret_cast<char*>(&idx.n_locations_), sizeof(idx.n_locations_));
    in.read(reinterpret_cast<char*>(&idx.witness_limit_minutes_), sizeof(idx.witness_limit_minutes_));
    in.read(reinterpret_cast<char*>(&idx.timetable_n_routes_), sizeof(idx.timetable_n_routes_));
    in.read(reinterpret_cast<char*>(&idx.timetable_n_transports_), sizeof(idx.timetable_n_transports_));

    std::uint32_t n_profiles{0};
    in.read(reinterpret_cast<char*>(&n_profiles), sizeof(n_profiles));
    if (!in || n_profiles > kNProfiles) return std::nullopt;

    idx.profiles_.resize(n_profiles);
    for (std::uint32_t p = 0; p < n_profiles; ++p) {
      auto read_vec_u32 = [&](vector<std::uint32_t>& vec) -> bool {
        std::uint64_t sz{0};
        in.read(reinterpret_cast<char*>(&sz), sizeof(sz));
        if (!in) return false;
        vec.resize(sz);
        if (sz > 0) in.read(reinterpret_cast<char*>(vec.data()), sz * sizeof(std::uint32_t));
        return static_cast<bool>(in);
      };
      auto read_vec_edges = [&](vector<tad_transfer_edge>& vec) -> bool {
        std::uint64_t sz{0};
        in.read(reinterpret_cast<char*>(&sz), sizeof(sz));
        if (!in) return false;
        vec.resize(sz);
        if (sz > 0) in.read(reinterpret_cast<char*>(vec.data()), sz * sizeof(tad_transfer_edge));
        return static_cast<bool>(in);
      };

      if (!read_vec_u32(idx.profiles_[p].out_offsets_) ||
          !read_vec_edges(idx.profiles_[p].out_edges_) ||
          !read_vec_u32(idx.profiles_[p].in_offsets_) ||
          !read_vec_edges(idx.profiles_[p].in_edges_)) {
        return std::nullopt;
      }
    }
    return idx;
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
