#pragma once

#include <array>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <optional>
#include <system_error>

#include "nigiri/logging.h"
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

  static inline std::atomic<std::uint64_t> n_offset_validations_{0U};

  static bool valid_offsets(vector<std::uint32_t> const& offsets,
                            vector<tad_transfer_edge> const& edges,
                            std::uint32_t const n_locations) {
    n_offset_validations_.fetch_add(1U, std::memory_order_relaxed);
    if (offsets.size() != n_locations + 1U || offsets.empty() ||
        offsets.front() != 0U || offsets.back() != edges.size()) {
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
  static constexpr std::uint32_t kVersion = 5U;

  std::uint32_t version_{kVersion};
  std::uint32_t n_locations_{0U};
  std::uint32_t witness_limit_minutes_{0U};
  std::uint64_t timetable_n_routes_{0U};
  std::uint64_t timetable_n_transports_{0U};
  std::uint64_t timetable_fingerprint_{0U};
  vector<tad_profile_shortcuts> profiles_;

  // Runtime-only, never serialized: set by validate_for_timetable() at index
  // load time, or by a builder that derived the index from the timetable it is
  // about to be used with. read() leaves it false, so a freshly loaded index is
  // unusable until it has been validated once.
  bool timetable_validated_{false};
  std::array<bool, kNProfiles> profile_validated_{};

  // Diagnostic: how often the O(timetable) fingerprint was computed. Setup
  // hashes the timetable once per index; a query must never increment this.
  static inline std::atomic<std::uint64_t> n_fingerprint_computations_{0U};

  static std::uint64_t compute_tt_fingerprint(timetable const& tt) {
    n_fingerprint_computations_.fetch_add(1U, std::memory_order_relaxed);
    std::uint64_t fp = 14695981039346656037ULL;
    auto const add = [&](auto const value) {
      fp ^= static_cast<std::uint64_t>(value);
      fp *= 1099511628211ULL;
    };
    auto const locations = [&](auto const& buckets) {
      add(buckets.size());
      for (auto const& bucket : buckets) {
        add(bucket.size());
        for (auto const l : bucket) add(to_idx(l));
      }
    };
    add(tt.n_locations());
    add(tt.date_range_.from_.time_since_epoch().count());
    add(tt.date_range_.to_.time_since_epoch().count());
    for (auto const l : tt.locations_.parents_) add(to_idx(l));
    for (auto const t : tt.locations_.transfer_time_) add(t.count());
    locations(tt.locations_.equivalences_);
    locations(tt.locations_.children_);
    locations(tt.location_routes_);
    for (auto const& seq : tt.route_location_seq_) {
      add(seq.size());
      for (auto const stp : seq) add(stp);  // includes boarding permissions
    }
    for (auto const& profiles : {&tt.locations_.footpaths_out_, &tt.locations_.footpaths_in_}) {
      for (auto const& buckets : *profiles) {
        add(buckets.size());
        for (auto const& bucket : buckets) {
          add(bucket.size());
          for (auto const& edge : bucket) {
            add(to_idx(edge.target()));
            add(edge.duration().count());
          }
        }
      }
    }
    for (auto const& range : tt.route_transport_ranges_) {
      add(to_idx(range.from_)); add(to_idx(range.to_));
    }
    // Hash the compressed representation without dereferencing mutable indices:
    // even a malformed cache-invalidation fixture must not cause out-of-bounds reads.
    for (auto const& rt : tt.route_times_) {
      add(to_idx(rt.transports_.from_)); add(to_idx(rt.transports_.to_));
      add(rt.pattern_start_); add(rt.n_patterns_);
    }
    for (auto const d : tt.transport_first_dep_) add(d.count());
    for (auto const pattern : tt.transport_pattern_) add(pattern);
    for (auto const d : tt.pattern_offsets_) add(d.count());
    for (auto const r : tt.transport_route_) add(to_idx(r));
    for (auto const td : tt.transport_traffic_days_) add(to_idx(td));
    for (auto const& bf : tt.bitfields_) {
      add(bf.size());
      for (std::size_t i = 0; i < bf.size(); ++i) add(bf.test(i));
    }
    return fp;
  }

  // Structural scans belong to setup only. After setup the index must remain
  // immutable while queries hold it; revalidate after any construction-time edit.
  bool validate_structure() {
    profile_validated_.fill(false);
    std::array<bool, kNProfiles> valid{};
    if (profiles_.size() > kNProfiles) return false;
    for (std::size_t p = 0; p < profiles_.size(); ++p) {
      auto const& profile = profiles_[p];
      if (!profile.has_offsets_for(n_locations_)) return false;
      for (auto const* edges : {&profile.out_edges_, &profile.in_edges_}) {
        for (auto const& edge : *edges) {
          if (to_idx(edge.target_) >= n_locations_ || edge.duration_ < duration_t{0} ||
              edge.duration_ > footpath::kMaxDuration) return false;
        }
      }
      valid[p] = !profile.transfer_time_fingerprint_.empty();
    }
    profile_validated_ = valid;
    return true;
  }

  // register_location() keeps parents_ in lockstep with names_, so n_locations()
  // is the single source of truth for the location count. Timetables assembled
  // by hand in tests may leave parents_ empty; anything else is a bug.
  static std::uint32_t expected_n_locations(timetable const& tt) {
    assert(tt.locations_.parents_.empty() ||
           tt.locations_.parents_.size() == tt.n_locations());
    return static_cast<std::uint32_t>(tt.n_locations());
  }

  // O(timetable): re-hashes every input group the builder consumed.
  bool is_valid_for_tt(timetable const& tt) const {
    auto const expected_tt_fp = compute_tt_fingerprint(tt);

    return version_ == kVersion &&
           n_locations_ == expected_n_locations(tt) &&
           timetable_n_routes_ == static_cast<std::uint64_t>(tt.route_location_seq_.size()) &&
           timetable_n_transports_ == static_cast<std::uint64_t>(tt.transport_traffic_days_.size()) &&
           timetable_fingerprint_ == expected_tt_fp;
  }

  bool matches_witness_limit(int const witness_limit) const {
    return witness_limit_minutes_ ==
           (witness_limit < 0 ? 0U : static_cast<std::uint32_t>(witness_limit));
  }

  // O(1): are there usable shortcuts for this profile? Independent of the
  // timetable contents.
  bool has_shortcuts_for(profile_idx_t const prf_idx) const {
    auto const p_idx = cista::to_idx(prf_idx);
    return version_ == kVersion && p_idx < profiles_.size() && p_idx < kNProfiles &&
           profile_validated_[p_idx];
  }

  // O(timetable). Call exactly once, at index load or build time. Caches the
  // verdict so the query path can stay O(1).
  bool validate_for_timetable(timetable const& tt, int const witness_limit) {
    timetable_validated_ = validate_structure() && matches_witness_limit(witness_limit) && is_valid_for_tt(tt);
    return timetable_validated_;
  }

  // O(1). The query hot path: no timetable hashing, no allocation.
  bool is_valid_for_query(profile_idx_t const prf_idx,
                          transfer_time_settings const& tts = {}) const {
    return timetable_validated_ && has_shortcuts_for(prf_idx) && tts.is_default();
  }

  // O(timetable). One-shot equivalent of validate_for_timetable() followed by
  // is_valid_for_query(), for callers holding a const index.
  bool is_valid_for(timetable const& tt,
                    profile_idx_t const prf_idx,
                    int const witness_limit,
                    transfer_time_settings const& tts = {}) const {
    return has_shortcuts_for(prf_idx) && matches_witness_limit(witness_limit) &&
           tts.is_default() && is_valid_for_tt(tt);
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
    if (!idx.validate_structure()) return std::nullopt;
    return idx;
  }
};

// A search handed an index it cannot use silently degrades to base footpaths,
// which looks like a correct but slow result. Say so once per process so the
// degradation is visible in a server log, not only in a benchmark harness.
inline void warn_transfer_index_unusable(tad_transfer_index const& idx,
                                         profile_idx_t const prf_idx,
                                         transfer_time_settings const& tts) {
  static std::once_flag once;
  std::call_once(once, [&]() {
    log(log_lvl::error, "routing.tad",
        "transfer index unusable, falling back to footpath_transfer_provider "
        "(timetable_validated={}, profile={}, has_shortcuts={}, "
        "default_transfer_time_settings={})",
        idx.timetable_validated_,
        static_cast<unsigned>(cista::to_idx(prf_idx)),
        idx.has_shortcuts_for(prf_idx), tts.is_default());
  });
}

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
