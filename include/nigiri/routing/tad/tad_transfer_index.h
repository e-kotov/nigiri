#pragma once

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <system_error>

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
  static constexpr std::uint32_t kVersion = 3U;

  std::uint32_t version_{kVersion};
  std::uint32_t n_locations_{0U};
  std::uint32_t witness_limit_minutes_{0U};
  std::uint64_t timetable_n_routes_{0U};
  std::uint64_t timetable_n_transports_{0U};
  std::uint64_t timetable_fingerprint_{0U};
  vector<tad_profile_shortcuts> profiles_;

  static std::uint64_t compute_tt_fingerprint(timetable const& tt) {
    std::uint64_t fp = 14695981039346656037ULL;  // FNV-1a 64-bit offset basis
    auto const fnv_combine = [&](std::uint64_t val) {
      fp ^= val;
      fp *= 1099511628211ULL;
    };
    fnv_combine(static_cast<std::uint64_t>(tt.n_locations()));
    fnv_combine(static_cast<std::uint64_t>(tt.route_location_seq_.size()));
    fnv_combine(static_cast<std::uint64_t>(tt.transport_traffic_days_.size()));
    fnv_combine(static_cast<std::uint64_t>(tt.locations_.parents_.size()));

    auto const n_parents = tt.locations_.parents_.size();
    for (std::size_t i = 0; i < std::min<std::size_t>(n_parents, 100); ++i) {
      auto const loc = location_idx_t{static_cast<std::uint32_t>(i)};
      fnv_combine(static_cast<std::uint64_t>(cista::to_idx(tt.locations_.parents_[loc])));
    }
    auto const n_routes = tt.route_location_seq_.size();
    for (std::size_t r = 0; r < std::min<std::size_t>(n_routes, 50); ++r) {
      auto const route_idx = route_idx_t{static_cast<std::uint32_t>(r)};
      auto const seq = tt.route_location_seq_[route_idx];
      fnv_combine(static_cast<std::uint64_t>(seq.size()));
      if (!seq.empty()) {
        fnv_combine(static_cast<std::uint64_t>(stop{seq[0]}.location_idx().v_));
        fnv_combine(static_cast<std::uint64_t>(stop{seq[seq.size() - 1U]}.location_idx().v_));
      }
    }
    return fp;
  }

  bool is_valid_for(profile_idx_t const prf_idx) const {
    auto const p_idx = cista::to_idx(prf_idx);
    return version_ == kVersion && p_idx < profiles_.size() &&
           profiles_[p_idx].has_offsets_for(n_locations_) &&
           !profiles_[p_idx].transfer_time_fingerprint_.empty();
  }

  bool is_valid_for(timetable const& tt,
                    profile_idx_t const prf_idx,
                    transfer_time_settings const& tts = {}) const {
    return is_valid_for(tt, prf_idx, static_cast<int>(witness_limit_minutes_), tts);
  }

  bool is_valid_for(timetable const& tt,
                    profile_idx_t const prf_idx,
                    int const witness_limit,
                    transfer_time_settings const& tts = {}) const {
    auto const normalized_witness_limit =
        witness_limit < 0 ? 0U : static_cast<std::uint32_t>(witness_limit);
    if (!is_valid_for(prf_idx)) return false;

    auto const expected_tt_fp = compute_tt_fingerprint(tt);
    auto const expected_n_locs = tt.locations_.parents_.empty()
                                     ? tt.n_locations()
                                     : static_cast<std::uint32_t>(tt.locations_.parents_.size());

    return n_locations_ == expected_n_locs &&
           witness_limit_minutes_ == normalized_witness_limit &&
           timetable_n_routes_ == static_cast<std::uint64_t>(tt.route_location_seq_.size()) &&
           timetable_n_transports_ == static_cast<std::uint64_t>(tt.transport_traffic_days_.size()) &&
           timetable_fingerprint_ == expected_tt_fp &&
           tts.default_;
  }

  bool write(std::filesystem::path const& path) const {
    std::ofstream out{path, std::ios::binary};
    if (!out.is_open()) return false;

    out.write(reinterpret_cast<char const*>(&version_), sizeof(version_));
    out.write(reinterpret_cast<char const*>(&n_locations_), sizeof(n_locations_));
    out.write(reinterpret_cast<char const*>(&witness_limit_minutes_), sizeof(witness_limit_minutes_));
    out.write(reinterpret_cast<char const*>(&timetable_n_routes_), sizeof(timetable_n_routes_));
    out.write(reinterpret_cast<char const*>(&timetable_n_transports_), sizeof(timetable_n_transports_));
    out.write(reinterpret_cast<char const*>(&timetable_fingerprint_), sizeof(timetable_fingerprint_));

    auto const n_profiles = static_cast<std::uint32_t>(profiles_.size());
    out.write(reinterpret_cast<char const*>(&n_profiles), sizeof(n_profiles));

    for (auto const& prof : profiles_) {
      auto write_str = [&](string const& str) {
        std::uint64_t sz = str.size();
        out.write(reinterpret_cast<char const*>(&sz), sizeof(sz));
        if (sz > 0) out.write(str.data(), sz);
      };
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

      write_str(prof.transfer_time_fingerprint_);
      write_vec_u32(prof.out_offsets_);
      write_vec_edges(prof.out_edges_);
      write_vec_u32(prof.in_offsets_);
      write_vec_edges(prof.in_edges_);
    }
    out.close();
    return static_cast<bool>(out);
  }

  static std::optional<tad_transfer_index> read(std::filesystem::path const& path) {
    std::ifstream in{path, std::ios::binary};
    if (!in.is_open()) return std::nullopt;

    std::error_code ec;
    auto const file_size = std::filesystem::file_size(path, ec);
    if (ec || file_size < 36U) return std::nullopt;

    auto const max_u32_entries = file_size / sizeof(std::uint32_t);
    auto const max_edge_entries = file_size / sizeof(tad_transfer_edge);

    tad_transfer_index idx;
    in.read(reinterpret_cast<char*>(&idx.version_), sizeof(idx.version_));
    if (idx.version_ != kVersion) return std::nullopt;

    in.read(reinterpret_cast<char*>(&idx.n_locations_), sizeof(idx.n_locations_));
    in.read(reinterpret_cast<char*>(&idx.witness_limit_minutes_), sizeof(idx.witness_limit_minutes_));
    in.read(reinterpret_cast<char*>(&idx.timetable_n_routes_), sizeof(idx.timetable_n_routes_));
    in.read(reinterpret_cast<char*>(&idx.timetable_n_transports_), sizeof(idx.timetable_n_transports_));
    in.read(reinterpret_cast<char*>(&idx.timetable_fingerprint_), sizeof(idx.timetable_fingerprint_));

    std::uint32_t n_profiles{0};
    in.read(reinterpret_cast<char*>(&n_profiles), sizeof(n_profiles));
    if (!in || n_profiles > kNProfiles) return std::nullopt;

    idx.profiles_.resize(n_profiles);
    for (std::uint32_t p = 0; p < n_profiles; ++p) {
      auto read_str = [&](string& str) -> bool {
        std::uint64_t sz{0};
        in.read(reinterpret_cast<char*>(&sz), sizeof(sz));
        if (!in || sz > file_size) return false;
        std::string tmp(sz, '\0');
        if (sz > 0) in.read(tmp.data(), sz);
        str = tmp.c_str();
        return static_cast<bool>(in);
      };
      auto read_vec_u32 = [&](vector<std::uint32_t>& vec) -> bool {
        std::uint64_t sz{0};
        in.read(reinterpret_cast<char*>(&sz), sizeof(sz));
        if (!in || sz > max_u32_entries) return false;
        vec.resize(sz);
        if (sz > 0) in.read(reinterpret_cast<char*>(vec.data()), sz * sizeof(std::uint32_t));
        return static_cast<bool>(in);
      };
      auto read_vec_edges = [&](vector<tad_transfer_edge>& vec) -> bool {
        std::uint64_t sz{0};
        in.read(reinterpret_cast<char*>(&sz), sizeof(sz));
        if (!in || sz > max_edge_entries) return false;
        vec.resize(sz);
        if (sz > 0) in.read(reinterpret_cast<char*>(vec.data()), sz * sizeof(tad_transfer_edge));
        return static_cast<bool>(in);
      };

      if (!read_str(idx.profiles_[p].transfer_time_fingerprint_) ||
          !read_vec_u32(idx.profiles_[p].out_offsets_) ||
          !read_vec_edges(idx.profiles_[p].out_edges_) ||
          !read_vec_u32(idx.profiles_[p].in_offsets_) ||
          !read_vec_edges(idx.profiles_[p].in_edges_)) {
        return std::nullopt;
      }
    }

    if (in.peek() != std::ifstream::traits_type::eof()) {
      return std::nullopt;
    }
    return idx;
  }
};

struct shortcut_transfer_provider : public tad_transfer_provider {
  tad_profile_shortcuts const& shortcuts_;

  explicit shortcut_transfer_provider(tad_profile_shortcuts const& shortcuts)
      : shortcuts_{shortcuts} {}

  char const* name() const override { return "shortcut_transfer_provider"; }

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
