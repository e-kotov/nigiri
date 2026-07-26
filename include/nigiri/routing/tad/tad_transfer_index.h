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
    fnv_combine(static_cast<std::uint64_t>(tt.locations_.equivalences_.size()));
    fnv_combine(static_cast<std::uint64_t>(tt.locations_.children_.size()));
    fnv_combine(static_cast<std::uint64_t>(tt.location_routes_.size()));
    fnv_combine(static_cast<std::uint64_t>(tt.bitfields_.size()));
    fnv_combine(static_cast<std::uint64_t>(tt.route_stop_times_.size()));
    fnv_combine(static_cast<std::uint64_t>(tt.transport_route_.size()));
    fnv_combine(static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::minutes>(
            tt.date_range_.from_.time_since_epoch()).count()));
    fnv_combine(static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::minutes>(
            tt.date_range_.to_.time_since_epoch()).count()));

    // Hash all parent relationships
    for (auto const p : tt.locations_.parents_) {
      fnv_combine(static_cast<std::uint64_t>(cista::to_idx(p)));
    }

    // Hash all location equivalences
    for (auto const& eq : tt.locations_.equivalences_) {
      fnv_combine(static_cast<std::uint64_t>(eq.size()));
      for (auto const loc : eq) {
        fnv_combine(static_cast<std::uint64_t>(cista::to_idx(loc)));
      }
    }

    // Hash all location children relationships
    for (auto const& children : tt.locations_.children_) {
      fnv_combine(static_cast<std::uint64_t>(children.size()));
      for (auto const child : children) {
        fnv_combine(static_cast<std::uint64_t>(cista::to_idx(child)));
      }
    }

    // Hash location_routes_
    for (auto const& routes : tt.location_routes_) {
      fnv_combine(static_cast<std::uint64_t>(routes.size()));
      for (auto const r : routes) {
        fnv_combine(static_cast<std::uint64_t>(cista::to_idx(r)));
      }
    }

    // Hash all footpaths_out and durations across profile layers
    for (auto const& prf_footpaths : tt.locations_.footpaths_out_) {
      fnv_combine(static_cast<std::uint64_t>(prf_footpaths.size()));
      for (auto const& fps : prf_footpaths) {
        fnv_combine(static_cast<std::uint64_t>(fps.size()));
        for (auto const& fp_elem : fps) {
          fnv_combine(static_cast<std::uint64_t>(cista::to_idx(fp_elem.target())));
          fnv_combine(static_cast<std::uint64_t>(fp_elem.duration().count()));
        }
      }
    }

    // Hash all footpaths_in and durations across profile layers
    for (auto const& prf_footpaths : tt.locations_.footpaths_in_) {
      fnv_combine(static_cast<std::uint64_t>(prf_footpaths.size()));
      for (auto const& fps : prf_footpaths) {
        fnv_combine(static_cast<std::uint64_t>(fps.size()));
        for (auto const& fp_elem : fps) {
          fnv_combine(static_cast<std::uint64_t>(cista::to_idx(fp_elem.target())));
          fnv_combine(static_cast<std::uint64_t>(fp_elem.duration().count()));
        }
      }
    }

    // Hash all route location sequences
    for (auto const& seq : tt.route_location_seq_) {
      fnv_combine(static_cast<std::uint64_t>(seq.size()));
      for (auto const stp : seq) {
        fnv_combine(static_cast<std::uint64_t>(stop{stp}.location_idx().v_));
      }
    }

    // Hash route transport ranges
    for (auto const& range : tt.route_transport_ranges_) {
      fnv_combine(static_cast<std::uint64_t>(cista::to_idx(range.from_)));
      fnv_combine(static_cast<std::uint64_t>(cista::to_idx(range.to_)));
    }

    // Hash route stop time ranges
    for (auto const& range : tt.route_stop_time_ranges_) {
      fnv_combine(static_cast<std::uint64_t>(range.from_));
      fnv_combine(static_cast<std::uint64_t>(range.to_));
    }

    // Hash route stop times (event times)
    for (auto const d : tt.route_stop_times_) {
      fnv_combine(static_cast<std::uint64_t>(d.count()));
    }

    // Hash transport route mapping
    for (auto const r : tt.transport_route_) {
      fnv_combine(static_cast<std::uint64_t>(cista::to_idx(r)));
    }

    // Hash transport traffic day indices
    for (auto const td : tt.transport_traffic_days_) {
      fnv_combine(static_cast<std::uint64_t>(cista::to_idx(td)));
    }

    // Hash bitfield bit blocks
    for (auto const& bf : tt.bitfields_) {
      fnv_combine(static_cast<std::uint64_t>(bf.size()));
      for (std::size_t b = 0; b < bf.size(); b += 64) {
        std::uint64_t word = 0;
        for (std::size_t k = 0; k < 64 && (b + k) < bf.size(); ++k) {
          if (bf.test(b + k)) {
            word |= (1ULL << k);
          }
        }
        fnv_combine(word);
      }
    }

    return fp;
  }

  bool is_valid_for_tt(timetable const& tt) const {
    auto const expected_tt_fp = compute_tt_fingerprint(tt);
    auto const expected_n_locs = tt.locations_.parents_.empty()
                                     ? tt.n_locations()
                                     : static_cast<std::uint32_t>(tt.locations_.parents_.size());

    return version_ == kVersion &&
           n_locations_ == expected_n_locs &&
           timetable_n_routes_ == static_cast<std::uint64_t>(tt.route_location_seq_.size()) &&
           timetable_n_transports_ == static_cast<std::uint64_t>(tt.transport_traffic_days_.size()) &&
           timetable_fingerprint_ == expected_tt_fp;
  }

  bool is_valid_for(profile_idx_t const prf_idx,
                    transfer_time_settings const& tts = {}) const {
    return is_valid_for(prf_idx, static_cast<int>(witness_limit_minutes_), tts);
  }

  bool is_valid_for(profile_idx_t const prf_idx,
                    int const witness_limit,
                    transfer_time_settings const& tts = {}) const {
    auto const normalized_witness_limit =
        witness_limit < 0 ? 0U : static_cast<std::uint32_t>(witness_limit);
    auto const p_idx = cista::to_idx(prf_idx);

    return version_ == kVersion && p_idx < profiles_.size() &&
           profiles_[p_idx].has_offsets_for(n_locations_) &&
           !profiles_[p_idx].transfer_time_fingerprint_.empty() &&
           witness_limit_minutes_ == normalized_witness_limit &&
           tts.is_default();
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
    return is_valid_for(prf_idx, witness_limit, tts) && is_valid_for_tt(tt);
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
