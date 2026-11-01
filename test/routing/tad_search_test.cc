#include "gtest/gtest.h"

#include "nigiri/loader/hrd/load_timetable.h"
#include "nigiri/loader/init_finish.h"

#include "../loader/hrd/hrd_timetable.h"

#include "nigiri/routing/tad/tad_reconstruct.h"
#include "nigiri/routing/tad/tad_route.h"
#include "nigiri/routing/tad/tad_search.h"
#include "nigiri/routing/tad/tad_state.h"
#include "nigiri/routing/tad/tad_station_grouping.h"
#include "nigiri/routing/tad/tad_ultra_builder.h"
#include "nigiri/routing/search.h"
#include "nigiri/common/parse_time.h"
#include "nigiri/routing/raptor/raptor.h"
#include "nigiri/routing/raptor/raptor_state.h"
#include "nigiri/routing/transfer_time_settings.h"
#include "../raptor_search.h"
#include "nigiri/timetable.h"
#include "results_to_string.h"

using namespace date;
using namespace nigiri;
using namespace nigiri::loader;
using namespace nigiri::test_data::hrd_timetable;

// Helper to run both TAD and RAPTOR and assert they yield the same Earliest Arrival Time
void verify_tad_raptor_equivalence(timetable const& tt, std::string const& start_id, std::string const& dest_id, unixtime_t start_time) {
  constexpr auto const src = source_idx_t{0U};
  auto const start_loc_it = tt.locations_.location_id_to_idx_.find({start_id, src});
  ASSERT_NE(start_loc_it, tt.locations_.location_id_to_idx_.end()) << "Start location " << start_id << " not found!";
  auto const start_loc = start_loc_it->second;

  auto const dest_loc_it = tt.locations_.location_id_to_idx_.find({dest_id, src});
  ASSERT_NE(dest_loc_it, tt.locations_.location_id_to_idx_.end()) << "Destination location " << dest_id << " not found!";
  auto const dest_loc = dest_loc_it->second;

  // 1. Run TAD
  routing::transfer_time_settings tts;
  routing::tad_state state;
  routing::tad_search search(tt, nullptr, state, tts, nigiri::profile_idx_t{0}, nigiri::routing::all_clasz_allowed(), false, false);
  
  auto const base = tt.internal_interval_days().from_;
  auto const start_delta = unix_to_delta(base, start_time);
  
  search.execute({{start_loc, start_delta}});
  
  auto const dest_label = state.node_labels_[to_idx(dest_loc)];
  bool tad_found = dest_label.time_stamp_ == state.current_time_stamp_;
  auto const tad_arrival = tad_found ? dest_label.arrival_time_ : kInvalidDelta<direction::kForward>;
  
  // 2. Run RAPTOR
  auto const raptor_results = nigiri::test::raptor_search(
      tt, nullptr, start_id, dest_id,
      interval{start_time, start_time + 1_hours});
      
  bool raptor_found = !raptor_results.empty();
  delta_t raptor_arrival = kInvalidDelta<direction::kForward>;
  if (raptor_found) {
      for (auto const& j : raptor_results) {
          auto arr = unix_to_delta(base, j.legs_.back().arr_time_);
          if (raptor_arrival == kInvalidDelta<direction::kForward> || arr < raptor_arrival) {
              raptor_arrival = arr;
          }
      }
  }
  
  // 3. Assert Equivalence
  ASSERT_EQ(tad_found, raptor_found) << "TAD and RAPTOR disagree on reachability";
  if (tad_found) {
      EXPECT_EQ(tad_arrival, raptor_arrival) 
          << "TAD and RAPTOR disagree on Earliest Arrival Time.\n"
          << "RAPTOR output:\n" << to_string(tt, raptor_results)
          << "TAD arrival: " << tad_arrival << ", RAPTOR arrival: " << raptor_arrival;
          
      routing::journey tad_journey;
      routing::tad_reconstruct_journey(tt, tts, state, start_loc, dest_loc, nigiri::profile_idx_t{0}, base, day_idx_t{0}, tad_journey);
      
      routing::journey const* best_raptor = nullptr;
      for (auto const& j : raptor_results) {
          auto arr = unix_to_delta(base, j.legs_.back().arr_time_);
          if (arr == raptor_arrival) {
              best_raptor = &j;
              break;
          }
      }
      ASSERT_NE(best_raptor, nullptr);
      
      EXPECT_EQ(tad_journey.legs_.size(), best_raptor->legs_.size()) << "Leg count mismatch";
      if (tad_journey.legs_.size() == best_raptor->legs_.size()) {
          for (size_t i = 0; i < tad_journey.legs_.size(); ++i) {
              auto const& tl = tad_journey.legs_[i];
              auto const& rl = best_raptor->legs_[i];
              EXPECT_EQ(tl.from_, rl.from_) << "Leg " << i << " from_ mismatch";
              EXPECT_EQ(tl.to_, rl.to_) << "Leg " << i << " to_ mismatch";
              EXPECT_EQ(tl.dep_time_, rl.dep_time_) << "Leg " << i << " dep_time mismatch";
              EXPECT_EQ(tl.arr_time_, rl.arr_time_) << "Leg " << i << " arr_time mismatch";
          }
      }
  }
}

loader::mem_dir custom_hrd(std::string const& services) {
  return base().add({loader::hrd::hrd_5_20_26.fplan_ / "services.101", services});
}

// 1. Direct Trips & 2. Valid Transfer (already covered by files_abc() in tad_forward)
TEST(routing, tad_forward_basic) {
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, files_abc(), tt);
  finalize(tt);

  auto const start_time = unixtime_t{sys_days{2020_y / March / 30}} + 5_hours;
  verify_tad_raptor_equivalence(tt, "0000001", "0000003", start_time);
}

// 3. Rejected Transfer Buffer
TEST(routing, tad_rejected_transfer) {
  constexpr auto const services = R"(
*Z 01337 80____       048 030                             %
*A VE 0000001 0000002 000005                              %
*G RE  0000001 0000002                                    %
0000001 A                            00230                %
0000002 B                     00330                       %
*Z 07331 80____       092 015                             %
*A VE 0000002 0000003 000005                              %
*G RE  0000002 0000003                                    %
0000002 B                            00331                %
0000003 C                     00430                       %
)";
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, custom_hrd(services), tt);
  finalize(tt);
  
  auto const start_0 = unixtime_t{sys_days{2020_y / March / 30}} + 0_hours;
  verify_tad_raptor_equivalence(tt, "0000001", "0000003", start_0);
}

// 4. Later First Leg Yielding Earlier Final Arrival
TEST(routing, tad_later_first_leg) {
  constexpr auto const services = R"(
*Z 01000 80____       048 030                             %
*A VE 0000001 0000002 000005                              %
*G RE  0000001 0000002                                    %
0000001 A                            00100                %
0000002 B                     00200                       %
*Z 02000 80____       048 030                             %
*A VE 0000001 0000002 000005                              %
*G ICE 0000001 0000002                                    %
0000001 A                            00130                %
0000002 B                     00145                       %
*Z 03000 80____       048 030                             %
*A VE 0000002 0000003 000005                              %
*G RE  0000002 0000003                                    %
0000002 B                            00155                %
0000003 C                     00215                       %
)";
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, custom_hrd(services), tt);
  finalize(tt);
  
  verify_tad_raptor_equivalence(tt, "0000001", "0000003", unixtime_t{sys_days{2020_y / March / 30}});
}

// 6. Unlimited-Transfer Behavior & 7. Deterministic Equal-Time
TEST(routing, tad_unlimited_transfers) {
  constexpr auto const services = R"(
*Z 00001 80____       048 030                             %
*A VE 0000001 0000002 000005                              %
*G RE  0000001 0000002                                    %
0000001 A                            00100                %
0000002 B                     00110                       %
*Z 00002 80____       048 030                             %
*A VE 0000002 0000003 000005                              %
*G RE  0000002 0000003                                    %
0000002 B                            00120                %
0000003 C                     00130                       %
*Z 00003 80____       048 030                             %
*A VE 0000003 0000004 000005                              %
*G RE  0000003 0000004                                    %
0000003 C                            00140                %
0000004 D                     00150                       %
*Z 00004 80____       048 030                             %
*A VE 0000004 0000005 000005                              %
*G RE  0000004 0000005                                    %
0000004 D                            00200                %
0000005 E                     00210                       %
)";
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, custom_hrd(services), tt);
  finalize(tt);
  
  verify_tad_raptor_equivalence(tt, "0000001", "0000005", unixtime_t{sys_days{2020_y / March / 30}});
}

// 8. No-Route Behavior
TEST(routing, tad_no_route) {
  constexpr auto const services = R"(
*Z 00001 80____       048 030                             %
*A VE 0000001 0000002 000005                              %
*G RE  0000001 0000002                                    %
0000001 A                            00100                %
0000002 B                     00110                       %
*Z 00002 80____       048 030                             %
*A VE 0000003 0000004 000005                              %
*G RE  0000003 0000004                                    %
0000003 C                            00120                %
0000004 D                     00130                       %
)";
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, custom_hrd(services), tt);
  finalize(tt);
  
  verify_tad_raptor_equivalence(tt, "0000001", "0000004", unixtime_t{sys_days{2020_y / March / 30}});
}

// 9. Walking Transfer Improvement
TEST(routing, tad_walking_transfer_improvement) {
  constexpr auto const services = R"(
*Z 00001 80____       048 030                             %
*A VE 0000001 0000006 000005                              %
*G RE  0000001 0000006                                    %
0000001 A                            00100                %
0000006 F                     00200                       %
*Z 00002 80____       048 030                             %
*A VE 0000001 0000010 000005                              %
*G RE  0000001 0000010                                    %
0000001 A                            00110                %
0000010 F_META                00150                       %
)";
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  
  constexpr auto const footpaths = R"(
0000010 0000006  02
0000006 0000010  02
)";
  
  auto dir = test_data::hrd_timetable::base();
  dir.add({loader::hrd::hrd_5_20_26.fplan_ / "services.101", services});
  dir.dir_[loader::hrd::hrd_5_20_26.core_data_ / loader::hrd::hrd_5_20_26.required_files_[loader::hrd::FOOTPATHS][0]] = footpaths;

  load_timetable(src, loader::hrd::hrd_5_20_26, dir, tt);
  finalize(tt);
  
  verify_tad_raptor_equivalence(tt, "0000001", "0000006", unixtime_t{sys_days{2020_y / March / 30}});
  
  // Explicitly verify walking improvement chain
  auto const start_it = tt.locations_.location_id_to_idx_.find({"0000001", src});
  ASSERT_NE(start_it, tt.locations_.location_id_to_idx_.end());
  auto const start_loc = start_it->second;
  auto const dest_it = tt.locations_.location_id_to_idx_.find({"0000006", src});
  ASSERT_NE(dest_it, tt.locations_.location_id_to_idx_.end());
  auto const dest_loc = dest_it->second;
  
  routing::transfer_time_settings tts;
  routing::tad_state state;
  routing::tad_search search(tt, nullptr, state, tts, nigiri::profile_idx_t{0}, nigiri::routing::all_clasz_allowed(), false, false);
  auto const start_time = unixtime_t{sys_days{2020_y / March / 30}};
  auto const base = tt.internal_interval_days().from_;
  
  auto const f_meta_it = tt.locations_.location_id_to_idx_.find({"0000010", src});
  ASSERT_NE(f_meta_it, tt.locations_.location_id_to_idx_.end());
  auto const f_meta_loc = f_meta_it->second;
  
  search.execute({{start_loc, unix_to_delta(base, start_time)}});
  
  auto const dest_label = state.node_labels_[to_idx(dest_loc)];
  
  ASSERT_EQ(dest_label.time_stamp_, state.current_time_stamp_);
  
  // F (0000006) should be reached via a walking transfer from F_META (0000010)
  EXPECT_TRUE(dest_label.reached_by_walking_) << "Walking transfer was not chosen over direct transit";
  auto const parent_loc = dest_label.parent_;
  EXPECT_EQ(parent_loc, f_meta_loc);
}

// 10. Deterministic Equal-Time Reconstruction
TEST(routing, tad_deterministic_equal_time) {
  constexpr auto const services = R"(
*Z 00001 80____       048 030                             %
*A VE 0000001 0000003 000005                              %
*G RE  0000001 0000003                                    %
0000001 A                            00100                %
0000003 C                     00200                       %
*Z 00002 80____       048 030                             %
*A VE 0000001 0000002 000005                              %
*G RE  0000001 0000002                                    %
0000001 A                            00100                %
0000002 B                     00130                       %
*Z 00003 80____       048 030                             %
*A VE 0000002 0000003 000005                              %
*G RE  0000002 0000003                                    %
0000002 B                            00140                %
0000003 C                     00200                       %
)";
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, custom_hrd(services), tt);
  finalize(tt);
  
  verify_tad_raptor_equivalence(tt, "0000001", "0000003", unixtime_t{sys_days{2020_y / March / 30}});
  
  // Explicitly verify deterministic equal-time reconstruction chain
  auto const start_it = tt.locations_.location_id_to_idx_.find({"0000001", src});
  ASSERT_NE(start_it, tt.locations_.location_id_to_idx_.end());
  auto const start_loc = start_it->second;
  auto const dest_it = tt.locations_.location_id_to_idx_.find({"0000003", src});
  ASSERT_NE(dest_it, tt.locations_.location_id_to_idx_.end());
  auto const dest_loc = dest_it->second;
  
  routing::transfer_time_settings tts;
  routing::tad_state state;
  routing::tad_search search(tt, nullptr, state, tts, nigiri::profile_idx_t{0}, nigiri::routing::all_clasz_allowed(), false, false);
  auto const start_time = unixtime_t{sys_days{2020_y / March / 30}};
  auto const base = tt.internal_interval_days().from_;
  search.execute({{start_loc, unix_to_delta(base, start_time)}});
  
  auto const dest_label = state.node_labels_[to_idx(dest_loc)];
  ASSERT_EQ(dest_label.time_stamp_, state.current_time_stamp_);
  
  // Since A->C direct and A->B->C have identical arrivals (00200) and we process in deterministic order, 
  // we check that the chosen parent is deterministic (e.g., direct route via A->C should be preferred or predictable)
  auto const parent_loc = dest_label.parent_;
  auto const a_loc = start_loc;
  // A direct trip A->C will have A as parent. A transfer A->B->C will have B as parent.
  // Because A->C arrives at 2:00, and A->B->C arrives at 2:00. A->C is discovered first in the route scan.
  EXPECT_EQ(parent_loc, a_loc) << "Parent reconstruction broke determinism on equal arrival times";
}

// Phase 1: Same-station queries
TEST(routing, tad_same_station) {
  constexpr auto const services = R"(
*Z 00001 80____       048 030                             %
*A VE 0000001 0000002 000005                              %
*G RE  0000001 0000002                                    %
0000001 A                            00100                %
0000002 B                     00110                       %
)";
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, custom_hrd(services), tt);
  finalize(tt);
  
  routing::transfer_time_settings tts;
  routing::tad_state state;
  routing::tad_search search(tt, nullptr, state, tts, nigiri::profile_idx_t{0}, nigiri::routing::all_clasz_allowed(), false, false);
  
  auto const start_loc_it = tt.locations_.location_id_to_idx_.find({"0000001", src});
  ASSERT_NE(start_loc_it, tt.locations_.location_id_to_idx_.end());
  auto const start_loc = start_loc_it->second;

  auto const base = tt.internal_interval_days().from_;
  auto const start_time = unixtime_t{sys_days{2020_y / March / 30}};
  auto const start_delta = unix_to_delta(base, start_time);
  
  search.execute({{start_loc, start_delta}});
  
  auto const dest_label = state.node_labels_[to_idx(start_loc)];
  EXPECT_EQ(dest_label.time_stamp_, state.current_time_stamp_) << "TAD did not reach the start station itself";
  EXPECT_EQ(dest_label.arrival_time_, start_delta) << "TAD arrival time at start station should be the start time";
}

// Phase 1: Zero-duration transfer
TEST(routing, tad_zero_duration_transfer) {
  constexpr auto const services = R"(
*Z 00001 80____       048 030                             %
*A VE 0000001 0000002 000005                              %
*G RE  0000001 0000002                                    %
0000001 A                            00100                %
0000002 B                     00130                       %
*Z 00002 80____       048 030                             %
*A VE 0000002 0000003 000005                              %
*G RE  0000002 0000003                                    %
0000002 B                            00130                %
0000003 C                     00200                       %
)";
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, custom_hrd(services), tt);
  finalize(tt);
  
  // Set transfer time for B to 0 to test zero-duration transfers
  auto const b_it = tt.locations_.location_id_to_idx_.find({"0000002", src});
  ASSERT_NE(b_it, tt.locations_.location_id_to_idx_.end());
  tt.locations_.transfer_time_[b_it->second] = 0_minutes;

  auto const start_time = unixtime_t{sys_days{2020_y / March / 30}};
  auto const base = tt.internal_interval_days().from_;
  auto const start_delta = unix_to_delta(base, start_time);
  
  auto const start_loc_it = tt.locations_.location_id_to_idx_.find({"0000001", src});
  auto const start_loc = start_loc_it->second;
  auto const dest_loc_it = tt.locations_.location_id_to_idx_.find({"0000003", src});
  auto const dest_loc = dest_loc_it->second;

  routing::transfer_time_settings tts;
  routing::tad_state state;
  routing::tad_search search(tt, nullptr, state, tts, nigiri::profile_idx_t{0}, nigiri::routing::all_clasz_allowed(), false, false);
  search.execute({{start_loc, start_delta}});
  
  auto const dest_label = state.node_labels_[to_idx(dest_loc)];
  EXPECT_EQ(dest_label.time_stamp_, state.current_time_stamp_) << "TAD failed to find zero-duration transfer route";
  
  // Verify with RAPTOR dynamically to avoid strict leg count which differs on footpaths
  auto const raptor_results = nigiri::test::raptor_search(
      tt, nullptr, "0000001", "0000003",
      interval{start_time, start_time + 1_hours});
  
  ASSERT_FALSE(raptor_results.empty()) << "RAPTOR failed to find zero-duration transfer route";
  
  delta_t raptor_arrival = kInvalidDelta<direction::kForward>;
  for (auto const& j : raptor_results) {
      auto arr = unix_to_delta(base, j.legs_.back().arr_time_);
      if (raptor_arrival == kInvalidDelta<direction::kForward> || arr < raptor_arrival) {
          raptor_arrival = arr;
      }
  }
  EXPECT_EQ(dest_label.arrival_time_, raptor_arrival) << "TAD Earliest Arrival Time mismatch with RAPTOR";
}

// Phase 1: Overnight/multi-day query
TEST(routing, tad_multi_day) {
  constexpr auto const services = R"(
*Z 00001 80____       048 030                             %
*A VE 0000001 0000002 000005                              %
*G RE  0000001 0000002                                    %
0000001 A                            2350                %
0000002 B                     0020                       %
*Z 00002 80____       048 030                             %
*A VE 0000002 0000003 000005                              %
*G RE  0000002 0000003                                    %
0000002 B                            0025                %
0000003 C                     0130                       %
)";
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, custom_hrd(services), tt);
  finalize(tt);
  
  verify_tad_raptor_equivalence(tt, "0000001", "0000003", unixtime_t{sys_days{2020_y / March / 30}} + 23_hours + 40_minutes);
}

// Phase 1: Multiple start-offset cases and Multiple destinations
TEST(routing, tad_multiple_starts_and_destinations) {
  constexpr auto const services = R"(
*Z 00001 80____       048 030                             %
*A VE 0000001 0000002 000005                              %
*G RE  0000001 0000002                                    %
0000001 A                            00100                %
0000002 B                     00130                       %
*Z 00002 80____       048 030                             %
*A VE 0000002 0000003 000005                              %
*G RE  0000002 0000003                                    %
0000002 B                            00140                %
0000003 C                     00200                       %
*Z 00003 80____       048 030                             %
*A VE 0000001 0000004 000005                              %
*G RE  0000001 0000004                                    %
0000001 A                            00200                %
0000004 D                     00230                       %
)";
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, custom_hrd(services), tt);
  finalize(tt);
  
  auto const start_it = tt.locations_.location_id_to_idx_.find({"0000001", src});
  ASSERT_NE(start_it, tt.locations_.location_id_to_idx_.end());
  auto const start_loc = start_it->second;

  auto const dest1_it = tt.locations_.location_id_to_idx_.find({"0000003", src});
  ASSERT_NE(dest1_it, tt.locations_.location_id_to_idx_.end());
  auto const dest1_loc = dest1_it->second;

  auto const dest2_it = tt.locations_.location_id_to_idx_.find({"0000004", src});
  ASSERT_NE(dest2_it, tt.locations_.location_id_to_idx_.end());
  auto const dest2_loc = dest2_it->second;

  routing::transfer_time_settings tts;
  routing::tad_state state;
  routing::tad_search search(tt, nullptr, state, tts, nigiri::profile_idx_t{0}, nigiri::routing::all_clasz_allowed(), false, false);
  
  auto const base = tt.internal_interval_days().from_;
  auto const base_time = unixtime_t{sys_days{2020_y / March / 30}};
  
  std::vector<std::pair<location_idx_t, delta_t>> starts = {
      {start_loc, unix_to_delta(base, base_time)},
      {start_loc, unix_to_delta(base, base_time + 90_minutes)} // Second offset at 01:30
  };
  
  search.execute(starts);
  
  auto const dest1_label = state.node_labels_[to_idx(dest1_loc)];
  EXPECT_EQ(dest1_label.time_stamp_, state.current_time_stamp_);
  
  auto const dest2_label = state.node_labels_[to_idx(dest2_loc)];
  EXPECT_EQ(dest2_label.time_stamp_, state.current_time_stamp_);
  
  // Use RAPTOR to fetch the exact expected arrivals for robust verification against timezones
  auto raptor_c = nigiri::test::raptor_search(tt, nullptr, "0000001", "0000003", interval{base_time, base_time + 2_hours});
  ASSERT_FALSE(raptor_c.empty());
  delta_t expected_c_arrival = kInvalidDelta<direction::kForward>;
  for (auto const& j : raptor_c) {
      auto const arr = unix_to_delta(base, j.legs_.back().arr_time_);
      if (expected_c_arrival == kInvalidDelta<direction::kForward> || arr < expected_c_arrival) { expected_c_arrival = arr; }
  }
  EXPECT_EQ(dest1_label.arrival_time_, expected_c_arrival);
  
  auto raptor_d = nigiri::test::raptor_search(tt, nullptr, "0000001", "0000004", interval{base_time, base_time + 3_hours});
  ASSERT_FALSE(raptor_d.empty());
  delta_t expected_d_arrival = kInvalidDelta<direction::kForward>;
  for (auto const& j : raptor_d) {
      auto const arr = unix_to_delta(base, j.legs_.back().arr_time_);
      if (expected_d_arrival == kInvalidDelta<direction::kForward> || arr < expected_d_arrival) { expected_d_arrival = arr; }
  }
  EXPECT_EQ(dest2_label.arrival_time_, expected_d_arrival);
}

// Phase 2: Coordinate offsets / td_start_ and td_dest_
TEST(routing, tad_coordinate_offsets) {
  constexpr auto const services = R"(
*Z 00001 80____       048 030                             %
*A VE 0000001 0000002 000005                              %
*G RE  0000001 0000002                                    %
0000001 A                            00100                %
0000002 B                     00130                       %
)";
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, custom_hrd(services), tt);
  finalize(tt);
  
  auto const start_loc = tt.locations_.location_id_to_idx_.at({"0000001", src});
  auto const dest_loc = tt.locations_.location_id_to_idx_.at({"0000002", src});
  
  auto const base_time = unixtime_t{sys_days{2020_y / March / 30}} + 23_hours + 40_minutes;
  
  routing::query q;
  q.start_time_ = interval{base_time, base_time + 2_hours};
  
  // Coordinate start: 10 mins to walk to A
  q.td_start_[start_loc] = {routing::td_offset{
      unixtime_t{0_minutes}, 10_minutes, 0U
  }};
  
  // Coordinate dest: 15 mins to walk from B
  q.td_dest_[dest_loc] = {routing::td_offset{
      unixtime_t{0_minutes}, 15_minutes, 0U
  }};
  
  // Need to set intermodal match mode to insert the artificial leg
  q.start_match_mode_ = routing::location_match_mode::kIntermodal;
  q.dest_match_mode_ = routing::location_match_mode::kIntermodal;
  
  // 1. Run RAPTOR
  routing::search_state raptor_s_state;
  routing::raptor_state raptor_r_state;
  // RAPTOR API usually uses search template directly or a wrapper, we'll use raptor_search
  // wait, raptor_search in test wrapper uses start_id not td_start_.
  // We can just call routing::search directly.
  using raptor_algo_t = routing::raptor<direction::kForward, false, 0, routing::search_mode::kOneToOne>;
  routing::routing_result raptor_res = routing::search<direction::kForward, raptor_algo_t>{
      tt, nullptr, raptor_s_state, raptor_r_state, q, std::nullopt}.execute();
      
  // 2. Run TAD
  routing::search_state tad_s_state;
  routing::tad_state tad_t_state;
  routing::routing_result tad_res = routing::tad_route(
      tt, nullptr, tad_s_state, tad_t_state, q, direction::kForward);
      
  // 3. Compare EAT
  ASSERT_EQ(raptor_res.journeys_->empty(), tad_res.journeys_->empty());
  if (!raptor_res.journeys_->empty()) {
      auto raptor_eat = raptor_res.journeys_->begin()->dest_time_;
      auto tad_eat = tad_res.journeys_->begin()->dest_time_;
      EXPECT_EQ(raptor_eat, tad_eat);
  }
}

TEST(routing, tad_multiple_td_offsets) {
  constexpr auto const services = R"(
*Z 00001 80____       048 030                             %
*A VE 0000001 0000003 000005                              %
*G RE  0000001 0000002                                    %
0000001 A                            00100                %
0000002 B                     00130                       %
0000003 C                     00140                       %
)";
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, custom_hrd(services), tt);
  finalize(tt);
  
  auto const start_loc_1 = tt.locations_.location_id_to_idx_.at({"0000001", src});
  auto const start_loc_2 = tt.locations_.location_id_to_idx_.at({"0000002", src});
  auto const dest_loc = tt.locations_.location_id_to_idx_.at({"0000003", src});
  
  auto const base_time = unixtime_t{sys_days{2020_y / March / 30}} + 23_hours + 40_minutes;
  
  routing::query q;
  q.start_time_ = interval{base_time, base_time + 2_hours};
  
  // Coordinate start: Multiple offsets to different stations
  q.td_start_[start_loc_1] = {routing::td_offset{unixtime_t{0_minutes}, 10_minutes, 0U}};
  q.td_start_[start_loc_2] = {routing::td_offset{unixtime_t{0_minutes}, 5_minutes, 0U}};
  q.td_dest_[dest_loc] = {routing::td_offset{unixtime_t{0_minutes}, 15_minutes, 0U}};
  
  q.start_match_mode_ = routing::location_match_mode::kIntermodal;
  q.dest_match_mode_ = routing::location_match_mode::kIntermodal;
  
  routing::search_state raptor_s_state;
  routing::raptor_state raptor_r_state;
  using raptor_algo_t = routing::raptor<direction::kForward, false, 0, routing::search_mode::kOneToOne>;
  routing::routing_result raptor_res = routing::search<direction::kForward, raptor_algo_t>{
      tt, nullptr, raptor_s_state, raptor_r_state, q, std::nullopt}.execute();
      
  routing::search_state tad_s_state;
  routing::tad_state tad_t_state;
  routing::routing_result tad_res = routing::tad_route(
      tt, nullptr, tad_s_state, tad_t_state, q, direction::kForward);
      
  ASSERT_EQ(raptor_res.journeys_->empty(), tad_res.journeys_->empty());
  if (!raptor_res.journeys_->empty()) {
      EXPECT_EQ(raptor_res.journeys_->begin()->dest_time_, tad_res.journeys_->begin()->dest_time_);
  }
}

TEST(routing, tad_time_varying_offsets) {
  constexpr auto const services = R"(
*Z 00001 80____       048 030                             %
*A VE 0000001 0000002 000005                              %
*G RE  0000001 0000002                                    %
0000001 A                            00100                %
0000002 B                     00130                       %
)";
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, custom_hrd(services), tt);
  finalize(tt);
  
  auto const start_loc = tt.locations_.location_id_to_idx_.at({"0000001", src});
  auto const dest_loc = tt.locations_.location_id_to_idx_.at({"0000002", src});
  auto const base_time = unixtime_t{sys_days{2020_y / March / 30}} + 23_hours + 40_minutes;
  
  routing::query q;
  q.start_time_ = interval{base_time, base_time + 2_hours};
  
  // Coordinate start: Time-varying offsets
  q.td_start_[start_loc] = {
      routing::td_offset{unixtime_t{0_minutes}, 20_minutes, 0U},
      routing::td_offset{base_time + 10_minutes, 5_minutes, 0U}
  };
  q.td_dest_[dest_loc] = {routing::td_offset{unixtime_t{0_minutes}, 15_minutes, 0U}};
  
  q.start_match_mode_ = routing::location_match_mode::kIntermodal;
  q.dest_match_mode_ = routing::location_match_mode::kIntermodal;
  
  routing::search_state raptor_s_state;
  routing::raptor_state raptor_r_state;
  using raptor_algo_t = routing::raptor<direction::kForward, false, 0, routing::search_mode::kOneToOne>;
  routing::routing_result raptor_res = routing::search<direction::kForward, raptor_algo_t>{
      tt, nullptr, raptor_s_state, raptor_r_state, q, std::nullopt}.execute();
      
  routing::search_state tad_s_state;
  routing::tad_state tad_t_state;
  routing::routing_result tad_res = routing::tad_route(
      tt, nullptr, tad_s_state, tad_t_state, q, direction::kForward);
      
  ASSERT_EQ(raptor_res.journeys_->empty(), tad_res.journeys_->empty());
  if (!raptor_res.journeys_->empty()) {
      EXPECT_EQ(raptor_res.journeys_->begin()->dest_time_, tad_res.journeys_->begin()->dest_time_);
  }
}

TEST(routing, tad_mixed_static_td_offsets) {
  constexpr auto const services = R"(
*Z 00001 80____       048 030                             %
*A VE 0000001 0000003 000005                              %
*G RE  0000001 0000002                                    %
0000001 A                            00100                %
0000002 B                     00130                       %
0000003 C                     00140                       %
)";
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, custom_hrd(services), tt);
  finalize(tt);
  
  auto const start_loc = tt.locations_.location_id_to_idx_.at({"0000001", src});
  auto const dest_loc = tt.locations_.location_id_to_idx_.at({"0000002", src});
  auto const dest_loc_td = tt.locations_.location_id_to_idx_.at({"0000003", src});
  auto const base_time = unixtime_t{sys_days{2020_y / March / 30}} + 23_hours + 40_minutes;
  
  routing::query q;
  q.start_time_ = interval{base_time, base_time + 2_hours};
  
  // Mixed static and td offsets for destinations
  q.td_start_[start_loc] = {routing::td_offset{unixtime_t{0_minutes}, 10_minutes, 0U}};
  q.destination_ = {{dest_loc, 0_minutes, 0U}}; // static dest
  q.td_dest_[dest_loc_td] = {routing::td_offset{unixtime_t{0_minutes}, 15_minutes, 0U}};
  
  q.start_match_mode_ = routing::location_match_mode::kIntermodal;
  q.dest_match_mode_ = routing::location_match_mode::kIntermodal;
  
  routing::search_state raptor_s_state;
  routing::raptor_state raptor_r_state;
  using raptor_algo_t = routing::raptor<direction::kForward, false, 0, routing::search_mode::kOneToOne>;
  routing::routing_result raptor_res = routing::search<direction::kForward, raptor_algo_t>{
      tt, nullptr, raptor_s_state, raptor_r_state, q, std::nullopt}.execute();
      
  routing::search_state tad_s_state;
  routing::tad_state tad_t_state;
  routing::routing_result tad_res = routing::tad_route(
      tt, nullptr, tad_s_state, tad_t_state, q, direction::kForward);
      
  ASSERT_EQ(raptor_res.journeys_->empty(), tad_res.journeys_->empty());
  if (!raptor_res.journeys_->empty()) {
      EXPECT_EQ(raptor_res.journeys_->begin()->dest_time_, tad_res.journeys_->begin()->dest_time_);
  }
}

TEST(routing, tad_reconstruct_parent_cycle_guard) {
  timetable tt;
  routing::transfer_time_settings tts;
  routing::tad_state state;
  state.resize(3U, 0U, 1U);
  state.next_start_time();

  auto const a = location_idx_t{0U};
  auto const b = location_idx_t{1U};
  auto const c = location_idx_t{2U};
  state.node_labels_[cista::to_idx(a)].time_stamp_ = state.current_time_stamp_;
  state.node_labels_[cista::to_idx(a)].arrival_time_ = delta_t{0};
  state.node_labels_[cista::to_idx(a)].parent_ = c;
  state.node_labels_[cista::to_idx(a)].reached_by_walking_ = true;
  state.node_labels_[cista::to_idx(b)].time_stamp_ = state.current_time_stamp_;
  state.node_labels_[cista::to_idx(b)].arrival_time_ = delta_t{1};
  state.node_labels_[cista::to_idx(b)].parent_ = c;
  state.node_labels_[cista::to_idx(b)].reached_by_walking_ = true;
  state.node_labels_[cista::to_idx(c)].time_stamp_ = state.current_time_stamp_;
  state.node_labels_[cista::to_idx(c)].arrival_time_ = delta_t{2};
  state.node_labels_[cista::to_idx(c)].parent_ = b;
  state.node_labels_[cista::to_idx(c)].reached_by_walking_ = true;

  routing::journey j;
  EXPECT_THROW(routing::tad_reconstruct_journey(
                   tt, tts, state, a, c, profile_idx_t{0}, sys_days{2020_y / March / 30}, day_idx_t{0}, j),
               std::runtime_error);
}

TEST(routing, tad_adversarial_class_filter) {
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, files_abc(), tt);
  finalize(tt);

  auto const start_loc = tt.locations_.location_id_to_idx_.at({"0000001", src});
  auto const dest_loc = tt.locations_.location_id_to_idx_.at({"0000003", src});
  auto const start_time = unixtime_t{sys_days{2020_y / March / 30}} + 5_hours;
  auto const start_delta = unix_to_delta(tt.internal_interval_days().from_, start_time);

  auto reaches_dest = [&](routing::clasz_mask_t const allowed_claszes) {
    routing::tad_state state;
    routing::transfer_time_settings tts;
    routing::tad_search search(tt, nullptr, state, tts, profile_idx_t{0}, allowed_claszes, false, false);
    search.execute(std::vector<std::pair<location_idx_t, delta_t>>{{start_loc, start_delta}});
    auto const& dest_label = state.node_labels_[cista::to_idx(dest_loc)];
    return dest_label.time_stamp_ == state.current_time_stamp_ &&
           dest_label.arrival_time_ != kInvalidDelta<direction::kForward>;
  };

  EXPECT_TRUE(reaches_dest(routing::all_clasz_allowed()));
  EXPECT_FALSE(reaches_dest(routing::to_mask(clasz::kShip)));
}

TEST(routing, tad_max_transfers_post_reconstruction_limitation) {
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, files_abc(), tt);
  finalize(tt);

  auto const start_loc = tt.locations_.location_id_to_idx_.at({"0000001", src});
  auto const dest_loc = tt.locations_.location_id_to_idx_.at({"0000004", src});
  auto const start_time = unixtime_t{sys_days{2020_y / March / 30}} + 2_hours;

  routing::query q;
  q.start_time_ = start_time;
  q.start_.push_back(routing::offset{start_loc, 0_minutes, 0});
  q.destination_.push_back(routing::offset{dest_loc, 0_minutes, 0});
  q.max_transfers_ = 0;

  routing::search_state s_state;
  routing::tad_state t_state;

  auto const res = routing::tad_route(tt, nullptr, s_state, t_state, q, direction::kForward);
  EXPECT_TRUE(res.journeys_->empty());
}

TEST(routing, tad_interval_early_stop) {
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, files_abc(), tt);
  finalize(tt);

  auto const start_loc = tt.locations_.location_id_to_idx_.at({"0000001", src});
  auto const dest_loc = tt.locations_.location_id_to_idx_.at({"0000003", src});
  auto const start_time = unixtime_t{sys_days{2020_y / March / 30}} + 2_hours;
  auto const end_time = start_time + 12_hours;

  routing::query q;
  q.start_time_ = interval<unixtime_t>{start_time, end_time};
  q.start_.push_back(routing::offset{start_loc, 0_minutes, 0});
  q.destination_.push_back(routing::offset{dest_loc, 0_minutes, 0});
  q.min_connection_count_ = 1;

  routing::search_state s_state;
  routing::tad_state t_state;

  auto const res = routing::tad_route(tt, nullptr, s_state, t_state, q, direction::kForward);
  EXPECT_FALSE(res.journeys_->empty());
  EXPECT_LE(res.journeys_->size(), 1U);
}

TEST(routing, tad_via_leg_destination_node_propagation) {
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, files_abc(), tt);
  finalize(tt);

  auto const start_loc = tt.locations_.location_id_to_idx_.at({"0000001", src});
  auto const via_loc = tt.locations_.location_id_to_idx_.at({"0000002", src});
  auto const dest_loc = tt.locations_.location_id_to_idx_.at({"0000003", src});
  auto const start_time = unixtime_t{sys_days{2020_y / March / 30}} + 2_hours;

  routing::query q;
  q.start_time_ = start_time;
  q.start_.push_back(routing::offset{start_loc, 0_minutes, 0});
  q.destination_.push_back(routing::offset{dest_loc, 0_minutes, 0});
  q.via_stops_.push_back(routing::via_stop{via_loc, 0_minutes});

  routing::search_state s_state;
  routing::tad_state t_state;

  auto const res = routing::tad_route(tt, nullptr, s_state, t_state, q, direction::kForward);
  EXPECT_FALSE(res.journeys_->empty());
}

#include "nigiri/routing/tad/tad_transfer_index_builder.h"

TEST(routing, tad_shortcut_transfer_provider) {
  routing::tad_transfer_index idx;
  idx.version_ = routing::tad_transfer_index::kVersion;
  idx.n_locations_ = 10;
  idx.profiles_.resize(1);

  auto& prf = idx.profiles_[0];
  prf.transfer_time_fingerprint_ = "mock_fingerprint";
  prf.out_offsets_.resize(11, 0);
  prf.in_offsets_.resize(11, 0);
  prf.out_offsets_[1] = 0;
  for (std::size_t i = 2; i <= 10; ++i) {
    prf.out_offsets_[i] = 1;
  }
  prf.out_edges_.push_back(routing::tad_transfer_edge{location_idx_t{2}, duration_t{5}});

  EXPECT_TRUE(idx.has_shortcuts_for(profile_idx_t{0}));
  EXPECT_FALSE(idx.has_shortcuts_for(profile_idx_t{1}));

  routing::shortcut_transfer_provider provider(prf);
  std::vector<routing::tad_transfer_edge> edges;
  provider.stop_transfers(location_idx_t{1}, true, edges);
  ASSERT_EQ(1U, edges.size());
  EXPECT_EQ(location_idx_t{2}, edges[0].target_);
  EXPECT_EQ(duration_t{5}, edges[0].duration_);

  edges.clear();
  provider.stop_transfers(location_idx_t{1}, false, edges);
  EXPECT_TRUE(edges.empty());

  prf.out_offsets_[3] = 0;
  EXPECT_FALSE(idx.has_shortcuts_for(profile_idx_t{0}));
}

TEST(routing, tad_shortcut_scalar_parity) {
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, files_abc(), tt);
  finalize(tt);

  auto const transfer_idx = routing::build_tad_transfer_index(tt);
  EXPECT_TRUE(transfer_idx.is_valid_for_query(profile_idx_t{0}));

  auto const start_loc = tt.locations_.location_id_to_idx_.at({"0000001", src});
  auto const dest_loc = tt.locations_.location_id_to_idx_.at({"0000003", src});
  auto const start_time = unixtime_t{sys_days{2020_y / March / 30}} + 2_hours;

  routing::query q;
  q.start_time_ = start_time;
  q.start_.push_back(routing::offset{start_loc, 0_minutes, 0});
  q.destination_.push_back(routing::offset{dest_loc, 0_minutes, 0});

  routing::search_state s_state_default;
  routing::tad_state t_state_default;
  auto const res_default = routing::tad_route(
      tt, nullptr, s_state_default, t_state_default, q, direction::kForward);

  routing::search_state s_state_shortcut;
  routing::tad_state t_state_shortcut;
  auto const res_shortcut = routing::tad_route(
      tt, nullptr, s_state_shortcut, t_state_shortcut, q, direction::kForward,
      std::nullopt, &transfer_idx);

  EXPECT_EQ(0ULL, res_default.algo_stats_.at("tad_using_shortcut_provider"));
  EXPECT_EQ(1ULL, res_shortcut.algo_stats_.at("tad_using_shortcut_provider"));

  ASSERT_EQ(res_default.journeys_->size(), res_shortcut.journeys_->size());
  if (!res_default.journeys_->empty()) {
    EXPECT_EQ(res_default.journeys_->begin()->dest_time_,
              res_shortcut.journeys_->begin()->dest_time_);
    EXPECT_EQ(res_default.journeys_->begin()->legs_,
              res_shortcut.journeys_->begin()->legs_);
  }
}

TEST(routing, tad_station_grouping) {
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, files_abc(), tt);
  finalize(tt);

  auto const grouping = routing::build_station_grouping(tt, profile_idx_t{0});
  EXPECT_EQ(tt.n_locations(), grouping.representative_of_stop_.size());

  std::size_t total_members = 0;
  for (auto const& st : grouping.stations_) {
    EXPECT_NE(location_idx_t::invalid(), st.representative_);
    EXPECT_FALSE(st.member_stops_.empty());
    EXPECT_TRUE(grouping.is_representative(st.representative_));
    total_members += st.member_stops_.size();
    for (auto const member : st.member_stops_) {
      EXPECT_EQ(st.representative_, grouping.representative(member));
      EXPECT_EQ(st.representative_, grouping.representative(grouping.representative(member)));
      EXPECT_LE(cista::to_idx(st.representative_), cista::to_idx(member));
    }
  }
  EXPECT_EQ(tt.n_locations(), total_members);
}

TEST(routing, tad_station_grouping_explicit_relationships) {
  timetable tt;
  constexpr std::size_t n_locs = 5;
  tt.locations_.names_.resize(n_locs);
  tt.locations_.parents_.resize(n_locs, location_idx_t::invalid());
  tt.locations_.types_.resize(n_locs, location_type::kStation);
  tt.locations_.footpaths_out_[0].resize(n_locs);

  auto const loc0 = location_idx_t{0};
  auto const loc1 = location_idx_t{1};
  auto const loc2 = location_idx_t{2};
  auto const loc3 = location_idx_t{3};
  auto const loc4 = location_idx_t{4};

  tt.locations_.parents_[loc1] = loc0;

  for (std::size_t i = 0; i < n_locs; ++i) {
    tt.locations_.equivalences_.emplace_back();
  }
  tt.locations_.equivalences_[loc0].emplace_back(loc2);

  tt.locations_.footpaths_out_[0][loc0].push_back(footpath{loc3, 0_minutes});
  tt.locations_.footpaths_out_[0][loc0].push_back(footpath{loc4, 5_minutes});

  auto const grouping = routing::build_station_grouping(tt, profile_idx_t{0});

  EXPECT_EQ(loc0, grouping.representative(loc0));
  EXPECT_EQ(loc0, grouping.representative(loc1));
  EXPECT_EQ(loc0, grouping.representative(loc2));
  EXPECT_EQ(loc0, grouping.representative(loc3));
  EXPECT_EQ(loc4, grouping.representative(loc4));

  for (auto const member : {loc0, loc1, loc2, loc3}) {
    EXPECT_LE(cista::to_idx(grouping.representative(member)), cista::to_idx(member));
  }
}

TEST(routing, tad_ultra_builder_valid_index) {
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, files_abc(), tt);
  finalize(tt);

  auto const idx = routing::build_tad_ultra_index(tt, 15);
  EXPECT_TRUE(idx.is_valid_for_query(profile_idx_t{0}));
  EXPECT_EQ(tt.n_locations(), idx.n_locations_);
  EXPECT_EQ("ultra_witness_shortcuts_v1", idx.profiles_[0].transfer_time_fingerprint_);
}

TEST(routing, tad_ultra_shortcut_scalar_parity) {
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, files_abc(), tt);
  finalize(tt);

  auto const ultra_idx = routing::build_tad_ultra_index(tt, 15);
  EXPECT_TRUE(ultra_idx.is_valid_for_query(profile_idx_t{0}));

  auto const start_loc = tt.locations_.location_id_to_idx_.at({"0000001", src});
  auto const dest_loc = tt.locations_.location_id_to_idx_.at({"0000003", src});
  auto const start_time = unixtime_t{sys_days{2020_y / March / 30}} + 2_hours;

  routing::query q;
  q.start_time_ = start_time;
  q.start_.push_back(routing::offset{start_loc, 0_minutes, 0});
  q.destination_.push_back(routing::offset{dest_loc, 0_minutes, 0});

  routing::search_state s_state_default;
  routing::tad_state t_state_default;
  auto const res_default = routing::tad_route(
      tt, nullptr, s_state_default, t_state_default, q, direction::kForward);

  routing::search_state s_state_shortcut;
  routing::tad_state t_state_shortcut;
  auto const res_shortcut = routing::tad_route(
      tt, nullptr, s_state_shortcut, t_state_shortcut, q, direction::kForward,
      std::nullopt, &ultra_idx);

  EXPECT_EQ(0ULL, res_default.algo_stats_.at("tad_using_shortcut_provider"));
  EXPECT_EQ(1ULL, res_shortcut.algo_stats_.at("tad_using_shortcut_provider"));

  ASSERT_EQ(res_default.journeys_->size(), res_shortcut.journeys_->size());
  if (!res_default.journeys_->empty()) {
    EXPECT_EQ(res_default.journeys_->begin()->dest_time_,
              res_shortcut.journeys_->begin()->dest_time_);
    EXPECT_EQ(res_default.journeys_->begin()->legs_,
              res_shortcut.journeys_->begin()->legs_);
  }
}

namespace {

inline loader::mem_dir files_two_routes_multihop() {
  constexpr auto const services_multihop = R"(
*Z 01337 80____       048 030                             %
*A VE 0000001 0000002 000005                              %
*G RE  0000001 0000002                                    %
0000001 A                            00200                %
0000002 B                     00230                       %
*Z 07331 80____       048 030                             %
*A VE 0000004 0000005 000005                              %
*G RE  0000004 0000005                                    %
0000004 C                            00245                %
0000005 D                     00315                       %
)";
  return test_data::hrd_timetable::base().add(
      {loader::hrd::hrd_5_20_26.fplan_ / "services.101", services_multihop});
}

}  // namespace

TEST(routing, tad_ultra_builder_emitted_walking_shortcut_fixture) {
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, files_two_routes_multihop(), tt);

  auto const loc_a = tt.locations_.location_id_to_idx_.at({"0000001", src});
  auto const loc_b = tt.locations_.location_id_to_idx_.at({"0000002", src});
  auto const loc_x = tt.locations_.location_id_to_idx_.at({"0000003", src});
  auto const loc_c = tt.locations_.location_id_to_idx_.at({"0000004", src});
  auto const loc_d = tt.locations_.location_id_to_idx_.at({"0000005", src});

  finalize(tt);

  tt.locations_.footpaths_out_[0].resize(tt.n_locations());
  tt.locations_.footpaths_in_[0].resize(tt.n_locations());

  // Add multi-hop base footpaths: B -> X (3 min) and X -> C (4 min).
  // CRITICAL: There is NO direct base footpath B -> C in tt!
  tt.locations_.footpaths_out_[0][loc_b].push_back(footpath{loc_x, 3_minutes});
  tt.locations_.footpaths_in_[0][loc_x].push_back(footpath{loc_b, 3_minutes});

  tt.locations_.footpaths_out_[0][loc_x].push_back(footpath{loc_c, 4_minutes});
  tt.locations_.footpaths_in_[0][loc_c].push_back(footpath{loc_x, 4_minutes});

  // Add a long base footpath A -> D (120 min) to prove long base footpaths are preserved without dominating transit.
  tt.locations_.footpaths_out_[0][loc_a].push_back(footpath{loc_d, 120_minutes});
  tt.locations_.footpaths_in_[0][loc_d].push_back(footpath{loc_a, 120_minutes});

  // 1. Build index with witness transfer limit = 15 minutes (>= 7 min multi-hop walk).
  auto const ultra_idx_15 = routing::build_tad_ultra_index(tt, 15);
  ASSERT_TRUE(ultra_idx_15.is_valid_for_query(profile_idx_t{0}));

  auto const b_idx = cista::to_idx(loc_b);
  auto const b_start = ultra_idx_15.profiles_[0].out_offsets_[b_idx];
  auto const b_end = ultra_idx_15.profiles_[0].out_offsets_[b_idx + 1];

  // Prove that B -> X (3 min) intermediate footpath between distinct stations is PRUNED (not seeded unconditionally).
  bool found_b_to_x = false;
  for (auto i = b_start; i < b_end; ++i) {
    if (ultra_idx_15.profiles_[0].out_edges_[i].target_ == loc_x) {
      found_b_to_x = true;
    }
  }
  EXPECT_FALSE(found_b_to_x);

  // Prove that run_witness_search() emitted the NEW SHORTCUT edge B -> C (7 min),
  // which was NOT a direct base footpath in tt!
  bool found_emitted_b_to_c = false;
  for (auto i = b_start; i < b_end; ++i) {
    auto const target = ultra_idx_15.profiles_[0].out_edges_[i].target_;
    auto const dur = ultra_idx_15.profiles_[0].out_edges_[i].duration_;
    if (target == loc_c) {
      found_emitted_b_to_c = true;
      EXPECT_EQ(7_minutes, dur);
    }
  }
  EXPECT_TRUE(found_emitted_b_to_c);

  // Verify reverse in_edge C -> B exists with duration 7_minutes.
  auto const c_idx = cista::to_idx(loc_c);
  auto const c_in_start = ultra_idx_15.profiles_[0].in_offsets_[c_idx];
  auto const c_in_end = ultra_idx_15.profiles_[0].in_offsets_[c_idx + 1];
  bool found_c_in_b = false;
  for (auto i = c_in_start; i < c_in_end; ++i) {
    if (ultra_idx_15.profiles_[0].in_edges_[i].target_ == loc_b) {
      found_c_in_b = true;
      EXPECT_EQ(7_minutes, ultra_idx_15.profiles_[0].in_edges_[i].duration_);
    }
  }
  EXPECT_TRUE(found_c_in_b);

  // Verify un-contracted base footpath A -> D (120 min) is pruned.
  auto const a_idx = cista::to_idx(loc_a);
  auto const a_start = ultra_idx_15.profiles_[0].out_offsets_[a_idx];
  auto const a_end = ultra_idx_15.profiles_[0].out_offsets_[a_idx + 1];
  bool found_long_a_to_d = false;
  for (auto i = a_start; i < a_end; ++i) {
    if (ultra_idx_15.profiles_[0].out_edges_[i].target_ == loc_d) {
      found_long_a_to_d = true;
    }
  }
  EXPECT_FALSE(found_long_a_to_d);

  // 2. Build index with witness transfer limit = 5 minutes (< 7 min multi-hop walk).
  auto const ultra_idx_5 = routing::build_tad_ultra_index(tt, 5);
  ASSERT_TRUE(ultra_idx_5.is_valid_for_query(profile_idx_t{0}));

  // Verify intermediate footpath B -> X (3 min) is pruned.
  auto const b_start_5 = ultra_idx_5.profiles_[0].out_offsets_[b_idx];
  auto const b_end_5 = ultra_idx_5.profiles_[0].out_offsets_[b_idx + 1];
  bool found_b_to_x_5 = false;
  bool found_emitted_b_to_c_5 = false;
  for (auto i = b_start_5; i < b_end_5; ++i) {
    if (ultra_idx_5.profiles_[0].out_edges_[i].target_ == loc_x) {
      found_b_to_x_5 = true;
    }
    if (ultra_idx_5.profiles_[0].out_edges_[i].target_ == loc_c) {
      found_emitted_b_to_c_5 = true;
    }
  }
  EXPECT_FALSE(found_b_to_x_5);
  EXPECT_FALSE(found_emitted_b_to_c_5);

  // 3. Scalar search parity for query A -> D requiring the intermediate transfer.
  auto const start_time = unixtime_t{sys_days{2020_y / March / 30}} + 2_hours;
  routing::query q;
  q.start_time_ = start_time;
  q.start_.push_back(routing::offset{loc_a, 0_minutes, 0});
  q.destination_.push_back(routing::offset{loc_d, 0_minutes, 0});

  routing::search_state s_state_default;
  routing::tad_state t_state_default;
  auto const res_default = routing::tad_route(
      tt, nullptr, s_state_default, t_state_default, q, direction::kForward);

  routing::search_state s_state_shortcut;
  routing::tad_state t_state_shortcut;
  auto const res_shortcut = routing::tad_route(
      tt, nullptr, s_state_shortcut, t_state_shortcut, q, direction::kForward,
      std::nullopt, &ultra_idx_15);

  ASSERT_EQ(1U, res_default.journeys_->size());
  ASSERT_EQ(res_default.journeys_->size(), res_shortcut.journeys_->size());
  EXPECT_EQ(unixtime_t{sys_days{2020_y / March / 30}} + 3_hours + 15_minutes,
            res_shortcut.journeys_->begin()->dest_time_);
  EXPECT_EQ(res_default.journeys_->begin()->dest_time_,
            res_shortcut.journeys_->begin()->dest_time_);
  // Default search uses 2 footpaths (B->X and X->C), yielding 4 legs total.
  EXPECT_EQ(4U, res_default.journeys_->begin()->legs_.size());
  // Shortcut search uses the single emitted ULTRA shortcut (B->C), yielding 3 legs total.
  EXPECT_EQ(3U, res_shortcut.journeys_->begin()->legs_.size());
}

TEST(routing, tad_ultra_builder_limit_validation) {
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, files_two_routes_multihop(), tt);

  auto const loc_b = tt.locations_.location_id_to_idx_.at({"0000002", src});
  auto const loc_x = tt.locations_.location_id_to_idx_.at({"0000003", src});
  auto const loc_c = tt.locations_.location_id_to_idx_.at({"0000004", src});

  finalize(tt);

  tt.locations_.footpaths_out_[0].resize(tt.n_locations());
  tt.locations_.footpaths_in_[0].resize(tt.n_locations());
  tt.locations_.footpaths_out_[0][loc_b].push_back(footpath{loc_x, 3_minutes});
  tt.locations_.footpaths_in_[0][loc_x].push_back(footpath{loc_b, 3_minutes});
  tt.locations_.footpaths_out_[0][loc_x].push_back(footpath{loc_c, 4_minutes});
  tt.locations_.footpaths_in_[0][loc_c].push_back(footpath{loc_x, 4_minutes});

  // 1. Negative limit (-10) is clamped to 0 minutes via safe_limit_duration, emitting no multi-hop shortcuts.
  auto const ultra_idx_neg = routing::build_tad_ultra_index(tt, -10);
  ASSERT_TRUE(ultra_idx_neg.is_valid_for_query(profile_idx_t{0}));
  auto const b_idx = cista::to_idx(loc_b);
  auto const b_start_neg = ultra_idx_neg.profiles_[0].out_offsets_[b_idx];
  auto const b_end_neg = ultra_idx_neg.profiles_[0].out_offsets_[b_idx + 1];
  bool found_b_to_c_neg = false;
  for (auto i = b_start_neg; i < b_end_neg; ++i) {
    if (ultra_idx_neg.profiles_[0].out_edges_[i].target_ == loc_c) {
      found_b_to_c_neg = true;
    }
  }
  EXPECT_FALSE(found_b_to_c_neg);

  // 2. Oversized limit (100000) is clamped to 32767 minutes without wrapping into negative duration.
  auto const ultra_idx_large = routing::build_tad_ultra_index(tt, 100000);
  ASSERT_TRUE(ultra_idx_large.is_valid_for_query(profile_idx_t{0}));
  auto const b_start_large = ultra_idx_large.profiles_[0].out_offsets_[b_idx];
  auto const b_end_large = ultra_idx_large.profiles_[0].out_offsets_[b_idx + 1];
  bool found_b_to_c_large = false;
  for (auto i = b_start_large; i < b_end_large; ++i) {
    if (ultra_idx_large.profiles_[0].out_edges_[i].target_ == loc_c) {
      found_b_to_c_large = true;
      EXPECT_EQ(7_minutes, ultra_idx_large.profiles_[0].out_edges_[i].duration_);
    }
  }
  EXPECT_TRUE(found_b_to_c_large);
}

TEST(routing, tad_access_shortcut_integration) {
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, files_two_routes_multihop(), tt);

  auto const loc_b = tt.locations_.location_id_to_idx_.at({"0000002", src});
  auto const loc_x = tt.locations_.location_id_to_idx_.at({"0000003", src});
  auto const loc_c = tt.locations_.location_id_to_idx_.at({"0000004", src});
  auto const loc_d = tt.locations_.location_id_to_idx_.at({"0000005", src});

  tt.locations_.footpaths_out_[0].resize(tt.n_locations());
  tt.locations_.footpaths_in_[0].resize(tt.n_locations());
  tt.locations_.footpaths_out_[0][loc_b].push_back(footpath{loc_x, 3_minutes});
  tt.locations_.footpaths_in_[0][loc_x].push_back(footpath{loc_b, 3_minutes});
  tt.locations_.footpaths_out_[0][loc_x].push_back(footpath{loc_c, 4_minutes});
  tt.locations_.footpaths_in_[0][loc_c].push_back(footpath{loc_x, 4_minutes});

  finalize(tt);

  auto const ultra_idx = routing::build_tad_ultra_index(tt, 15);
  ASSERT_TRUE(ultra_idx.is_valid_for_query(profile_idx_t{0}));

  auto const start_time = unixtime_t{sys_days{2020_y / March / 30}} + 2_hours;
  routing::query q;
  q.start_time_ = start_time;
  q.start_.push_back(routing::offset{loc_b, 0_minutes, 0});
  q.use_start_footpaths_ = true;
  q.max_start_offset_ = 20_minutes;
  q.destination_.push_back(routing::offset{loc_d, 0_minutes, 0});

  // 1. Shortcut search with ULTRA transfer_index
  routing::search_state s_state_shortcut;
  routing::tad_state t_state_shortcut;
  auto const res_shortcut = routing::tad_route(
      tt, nullptr, s_state_shortcut, t_state_shortcut, q, direction::kForward,
      std::nullopt, &ultra_idx);

  ASSERT_EQ(1U, res_shortcut.journeys_->size());
  auto const& j_shortcut = *res_shortcut.journeys_->begin();
  EXPECT_EQ(unixtime_t{sys_days{2020_y / March / 30}} + 2_hours + 45_minutes,
            j_shortcut.dest_time_);
  // Access shortcut replaces B->X (3m) and X->C (4m) with direct B->C (7m) access leg. Total 2 legs: B->C walk + C->D train.
  ASSERT_EQ(2U, j_shortcut.legs_.size());
  EXPECT_EQ(loc_b, j_shortcut.legs_[0].from_);
  EXPECT_EQ(loc_c, j_shortcut.legs_[0].to_);
  EXPECT_EQ(7_minutes, j_shortcut.legs_[0].arr_time_ - j_shortcut.legs_[0].dep_time_);
  ASSERT_TRUE(std::holds_alternative<footpath>(j_shortcut.legs_[0].uses_));
  auto const access_fp = std::get<footpath>(j_shortcut.legs_[0].uses_);
  EXPECT_EQ(loc_c, access_fp.target());
  EXPECT_EQ(7_minutes, access_fp.duration());

  // 2. Default search without shortcut index (uses base footpaths only: B->X (3m) and X->C (4m), yielding 3 legs)
  routing::search_state s_state_default;
  routing::tad_state t_state_default;
  auto const res_default = routing::tad_route(
      tt, nullptr, s_state_default, t_state_default, q, direction::kForward);

  ASSERT_EQ(1U, res_default.journeys_->size());
  auto const& j_default = *res_default.journeys_->begin();
  EXPECT_EQ(j_shortcut.dest_time_, j_default.dest_time_);
  ASSERT_EQ(3U, j_default.legs_.size());
  EXPECT_EQ(loc_b, j_default.legs_[0].from_);
  EXPECT_EQ(loc_x, j_default.legs_[0].to_);
  EXPECT_EQ(3_minutes, j_default.legs_[0].arr_time_ - j_default.legs_[0].dep_time_);
}

TEST(routing, tad_access_shortcut_conflict_resolution) {
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, files_two_routes_multihop(), tt);

  auto const loc_b = tt.locations_.location_id_to_idx_.at({"0000002", src});
  auto const loc_x = tt.locations_.location_id_to_idx_.at({"0000003", src});
  auto const loc_c = tt.locations_.location_id_to_idx_.at({"0000004", src});
  auto const loc_d = tt.locations_.location_id_to_idx_.at({"0000005", src});

  tt.locations_.footpaths_out_[0].resize(tt.n_locations());
  tt.locations_.footpaths_in_[0].resize(tt.n_locations());
  tt.locations_.footpaths_out_[0][loc_b].push_back(footpath{loc_x, 3_minutes});
  tt.locations_.footpaths_in_[0][loc_x].push_back(footpath{loc_b, 3_minutes});
  tt.locations_.footpaths_out_[0][loc_x].push_back(footpath{loc_c, 4_minutes});
  tt.locations_.footpaths_in_[0][loc_c].push_back(footpath{loc_x, 4_minutes});

  finalize(tt);

  auto const ultra_idx = routing::build_tad_ultra_index(tt, 15);
  ASSERT_TRUE(ultra_idx.is_valid_for_query(profile_idx_t{0}));

  auto const start_time = unixtime_t{sys_days{2020_y / March / 30}} + 2_hours;
  routing::query q;
  q.start_time_ = start_time;
  // Conflict setup: q.start_ contains BOTH direct C (20 min delay) AND root start B (0 min delay).
  // B -> C expanded access shortcut (7 min walk) yields combined duration 7 min, beating direct C (20 min).
  q.start_.push_back(routing::offset{loc_b, 0_minutes, 0});
  q.start_.push_back(routing::offset{loc_c, 20_minutes, 0});
  q.destination_.push_back(routing::offset{loc_d, 0_minutes, 0});
  q.use_start_footpaths_ = true;

  routing::search_state s_state;
  routing::tad_state t_state;
  auto const res = routing::tad_route(
      tt, nullptr, s_state, t_state, q, direction::kForward,
      std::nullopt, &ultra_idx);

  ASSERT_EQ(1U, res.journeys_->size());
  auto const& j = *res.journeys_->begin();
  // Expanded B->C access shortcut must win over direct C (20m offset).
  ASSERT_EQ(2U, j.legs_.size());
  EXPECT_EQ(loc_b, j.legs_[0].from_);
  EXPECT_EQ(loc_c, j.legs_[0].to_);
  EXPECT_EQ(7_minutes, j.legs_[0].arr_time_ - j.legs_[0].dep_time_);
  ASSERT_TRUE(std::holds_alternative<footpath>(j.legs_[0].uses_));
  auto const access_fp = std::get<footpath>(j.legs_[0].uses_);
  EXPECT_EQ(loc_c, access_fp.target());
  EXPECT_EQ(7_minutes, access_fp.duration());
}

TEST(routing, tad_egress_shortcut_integration) {
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, files_two_routes_multihop(), tt);

  auto const loc_a = tt.locations_.location_id_to_idx_.at({"0000001", src});
  auto const loc_b = tt.locations_.location_id_to_idx_.at({"0000002", src});
  auto const loc_x = tt.locations_.location_id_to_idx_.at({"0000003", src});
  auto const loc_c = tt.locations_.location_id_to_idx_.at({"0000004", src});

  tt.locations_.footpaths_out_[0].resize(tt.n_locations());
  tt.locations_.footpaths_in_[0].resize(tt.n_locations());
  tt.locations_.footpaths_out_[0][loc_b].push_back(footpath{loc_x, 3_minutes});
  tt.locations_.footpaths_in_[0][loc_x].push_back(footpath{loc_b, 3_minutes});
  tt.locations_.footpaths_out_[0][loc_x].push_back(footpath{loc_c, 4_minutes});
  tt.locations_.footpaths_in_[0][loc_c].push_back(footpath{loc_x, 4_minutes});

  finalize(tt);

  auto const ultra_idx = routing::build_tad_ultra_index(tt, 15);
  ASSERT_TRUE(ultra_idx.is_valid_for_query(profile_idx_t{0}));

  auto const start_time = unixtime_t{sys_days{2020_y / March / 30}} + 2_hours;
  routing::query q;
  q.start_time_ = start_time;
  q.start_.push_back(routing::offset{loc_a, 0_minutes, 0});
  q.destination_.push_back(routing::offset{loc_c, 0_minutes, 0});

  // 1. Shortcut search with ULTRA transfer_index (destination C reached via egress shortcut B->C)
  routing::search_state s_state_shortcut;
  routing::tad_state t_state_shortcut;
  auto const res_shortcut = routing::tad_route(
      tt, nullptr, s_state_shortcut, t_state_shortcut, q, direction::kForward,
      std::nullopt, &ultra_idx);

  ASSERT_EQ(1U, res_shortcut.journeys_->size());
  auto const& j_shortcut = *res_shortcut.journeys_->begin();
  // Arrival at B via Train 1 is 02:30. Egress shortcut B->C (7m) yields arrival at C at 02:37.
  EXPECT_EQ(unixtime_t{sys_days{2020_y / March / 30}} + 2_hours + 37_minutes,
            j_shortcut.dest_time_);
  // Reconstructed journey uses 2 legs: Leg 0 is train A->B, Leg 1 is egress shortcut B->C (7m).
  ASSERT_EQ(2U, j_shortcut.legs_.size());
  EXPECT_EQ(loc_b, j_shortcut.legs_[1].from_);
  EXPECT_EQ(loc_c, j_shortcut.legs_[1].to_);
  EXPECT_EQ(7_minutes, j_shortcut.legs_[1].arr_time_ - j_shortcut.legs_[1].dep_time_);
  ASSERT_TRUE(std::holds_alternative<footpath>(j_shortcut.legs_[1].uses_));
  auto const egress_fp = std::get<footpath>(j_shortcut.legs_[1].uses_);
  EXPECT_EQ(loc_c, egress_fp.target());
  EXPECT_EQ(7_minutes, egress_fp.duration());

  // 2. Default search without shortcut index (uses base footpaths B->X (3m) and X->C (4m), yielding 3 legs)
  routing::search_state s_state_default;
  routing::tad_state t_state_default;
  auto const res_default = routing::tad_route(
      tt, nullptr, s_state_default, t_state_default, q, direction::kForward);

  ASSERT_EQ(1U, res_default.journeys_->size());
  auto const& j_default = *res_default.journeys_->begin();
  EXPECT_EQ(j_shortcut.dest_time_, j_default.dest_time_);
  ASSERT_EQ(3U, j_default.legs_.size());
  EXPECT_EQ(loc_b, j_default.legs_[1].from_);
  EXPECT_EQ(loc_x, j_default.legs_[1].to_);
  EXPECT_EQ(3_minutes, j_default.legs_[1].arr_time_ - j_default.legs_[1].dep_time_);
  EXPECT_EQ(loc_x, j_default.legs_[2].from_);
  EXPECT_EQ(loc_c, j_default.legs_[2].to_);
  EXPECT_EQ(4_minutes, j_default.legs_[2].arr_time_ - j_default.legs_[2].dep_time_);

  // 3. Fallback search with invalid transfer_index (must safely fall back to base footpaths)
  routing::tad_transfer_index invalid_idx = ultra_idx;
  invalid_idx.version_ = 999;
  routing::search_state s_state_invalid;
  routing::tad_state t_state_invalid;
  auto const res_invalid = routing::tad_route(
      tt, nullptr, s_state_invalid, t_state_invalid, q, direction::kForward,
      std::nullopt, &invalid_idx);

  ASSERT_EQ(1U, res_invalid.journeys_->size());
  EXPECT_EQ(res_default.journeys_->begin()->dest_time_,
            res_invalid.journeys_->begin()->dest_time_);
  EXPECT_EQ(3U, res_invalid.journeys_->begin()->legs_.size());
}

TEST(routing, tad_ultra_direct_transfer_edge_routing) {
  constexpr auto const services = R"(
*Z 00001 80____       048 030                             %
*A VE 0000001 0000002 000005                              %
*G RE  0000001 0000002                                    %
0000001 A                            00200                %
0000002 B                     00220                       %
*Z 00002 80____       048 030                             %
*A VE 0000003 0000004 000005                              %
*G RE  0000003 0000004                                    %
0000003 C                            00230                %
0000004 D                     00250                       %
)";
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();

  constexpr auto const footpaths = R"(
0000002 0000003  05
0000003 0000002  05
)";

  auto dir = test_data::hrd_timetable::base();
  dir.add({loader::hrd::hrd_5_20_26.fplan_ / "services.101", services});
  dir.dir_[loader::hrd::hrd_5_20_26.core_data_ / loader::hrd::hrd_5_20_26.required_files_[loader::hrd::FOOTPATHS][0]] = footpaths;

  load_timetable(src, loader::hrd::hrd_5_20_26, dir, tt);
  finalize(tt);

  auto const loc_b = tt.locations_.location_id_to_idx_.at({"0000002", src});
  auto const loc_c = tt.locations_.location_id_to_idx_.at({"0000003", src});

  // Build ULTRA transfer index with witness limit = 15 mins
  auto const ultra_idx = routing::build_tad_ultra_index(tt, 15);
  ASSERT_TRUE(ultra_idx.is_valid_for_query(profile_idx_t{0}));

  // Verify that the ULTRA index preserves the direct transfer edge B -> C
  // using the duration produced by the HRD fixture loader.
  auto expected_direct_transfer = duration_t::max();
  for (auto const& fp : tt.locations_.footpaths_out_[0][loc_b]) {
    if (fp.target() == loc_c && fp.duration() < expected_direct_transfer) {
      expected_direct_transfer = fp.duration();
    }
  }
  ASSERT_NE(duration_t::max(), expected_direct_transfer)
      << "Fixture is missing direct transfer edge B -> C";

  auto const b_idx = cista::to_idx(loc_b);
  auto const& ps = ultra_idx.profiles_[cista::to_idx(profile_idx_t{0})];
  auto const start_off = ps.out_offsets_[b_idx];
  auto const end_off = ps.out_offsets_[b_idx + 1];

  bool found_shortcut_bc = false;
  for (auto i = start_off; i < end_off; ++i) {
    if (ps.out_edges_[i].target_ == loc_c) {
      found_shortcut_bc = true;
      EXPECT_EQ(expected_direct_transfer, ps.out_edges_[i].duration_);
    }
  }
  EXPECT_TRUE(found_shortcut_bc) << "ULTRA index is missing transfer edge B -> C";

  // Verify routing A -> D via the indexed transfer edge
  auto const loc_a = tt.locations_.location_id_to_idx_.at({"0000001", src});
  auto const loc_d = tt.locations_.location_id_to_idx_.at({"0000004", src});
  auto const start_time = unixtime_t{sys_days{2020_y / March / 30}} + 2_hours;

  routing::query q;
  q.start_time_ = start_time;
  q.start_.push_back(routing::offset{loc_a, 0_minutes, 0});
  q.destination_.push_back(routing::offset{loc_d, 0_minutes, 0});

  routing::search_state s_state;
  routing::tad_state t_state;
  auto const res = routing::tad_route(
      tt, nullptr, s_state, t_state, q, direction::kForward,
      std::nullopt, &ultra_idx);

  ASSERT_EQ(1U, res.journeys_->size());
  auto const& j = *res.journeys_->begin();
  EXPECT_EQ(unixtime_t{sys_days{2020_y / March / 30}} + 2_hours + 50_minutes, j.dest_time_);
  ASSERT_EQ(3U, j.legs_.size());
  // Leg 0: A -> B train (02:00 -> 02:20)
  EXPECT_EQ(loc_a, j.legs_[0].from_);
  EXPECT_EQ(loc_b, j.legs_[0].to_);
  // Leg 1: B -> C indexed transfer edge
  EXPECT_EQ(loc_b, j.legs_[1].from_);
  EXPECT_EQ(loc_c, j.legs_[1].to_);
  EXPECT_EQ(expected_direct_transfer,
            j.legs_[1].arr_time_ - j.legs_[1].dep_time_);
  // Leg 2: C -> D train (02:30 -> 02:50)
  EXPECT_EQ(loc_c, j.legs_[2].from_);
  EXPECT_EQ(loc_d, j.legs_[2].to_);
}

TEST(routing, tad_ultra_index_binary_serialization_and_validation) {
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, files_two_routes_multihop(), tt);
  finalize(tt);

  auto const idx = routing::build_tad_ultra_index(tt, 15);
  EXPECT_TRUE(idx.is_valid_for(tt, profile_idx_t{0}, 15));

  auto const tmp_path = std::filesystem::temp_directory_path() / "test_ultra_index.bin";
  std::filesystem::remove(tmp_path);

  // 1. Roundtrip serialization test
  ASSERT_TRUE(idx.write(tmp_path));
  auto const read_idx = routing::tad_transfer_index::read(tmp_path);
  ASSERT_TRUE(read_idx.has_value());
  EXPECT_TRUE(read_idx->is_valid_for(tt, profile_idx_t{0}, 15));
  EXPECT_EQ(idx.version_, read_idx->version_);
  EXPECT_EQ(idx.n_locations_, read_idx->n_locations_);
  EXPECT_EQ(idx.witness_limit_minutes_, read_idx->witness_limit_minutes_);
  EXPECT_EQ(idx.timetable_n_routes_, read_idx->timetable_n_routes_);
  EXPECT_EQ(idx.timetable_n_transports_, read_idx->timetable_n_transports_);
  EXPECT_EQ(idx.timetable_fingerprint_, read_idx->timetable_fingerprint_);
  EXPECT_EQ(idx.profiles_[0].transfer_time_fingerprint_, read_idx->profiles_[0].transfer_time_fingerprint_);

  // 2. Reject non-default transfer_time_settings
  routing::transfer_time_settings non_default_tts;
  non_default_tts.default_ = false;
  EXPECT_FALSE(read_idx->is_valid_for(tt, profile_idx_t{0}, 15, non_default_tts));

  // 3. Reject mismatched witness limit
  EXPECT_FALSE(read_idx->is_valid_for(tt, profile_idx_t{0}, 30));

  // 4. Reject truncated/corrupted file
  auto const file_sz = std::filesystem::file_size(tmp_path);
  std::ofstream truncate_out{tmp_path, std::ios::binary | std::ios::in | std::ios::out};
  truncate_out.seekp(file_sz - 10);
  truncate_out.write("CORRUPT!", 8);
  truncate_out.close();

  std::filesystem::resize_file(tmp_path, file_sz - 5);
  auto const corrupt_read = routing::tad_transfer_index::read(tmp_path);
  EXPECT_FALSE(corrupt_read.has_value());

  std::filesystem::remove(tmp_path);
}

TEST(routing, tad_ultra_index_fingerprint_invalidation_mutations) {
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, files_two_routes_multihop(), tt);
  finalize(tt);

  auto const idx = routing::build_tad_ultra_index(tt, 15);
  ASSERT_TRUE(idx.is_valid_for(tt, profile_idx_t{0}, 15));

  // Every in-place mutation below indexes element 0 of some container. If the
  // fixture were degenerate the mutation would be a silent no-op and the test
  // would still report green, so require a non-empty fixture up front.
  ASSERT_NE(0U, tt.n_locations());
  ASSERT_FALSE(tt.route_stop_times_.empty());
  ASSERT_FALSE(tt.transport_route_.empty());
  ASSERT_FALSE(tt.transport_traffic_days_.empty());
  ASSERT_FALSE(tt.bitfields_.empty());
  ASSERT_FALSE(tt.route_transport_ranges_.empty());
  ASSERT_FALSE(tt.route_stop_time_ranges_.empty());
  ASSERT_FALSE(tt.locations_.parents_.empty());
  ASSERT_FALSE(tt.locations_.equivalences_.empty());
  ASSERT_FALSE(tt.locations_.children_.empty());

  // One case per input group hashed by compute_tt_fingerprint(). Keep this
  // table in step with that function: an unhashed input is an index that
  // survives a timetable change it should not survive.
  struct mutation_case {
    char const* name_;
    void (*mutate_)(timetable&);
  };
  auto const cases = std::vector<mutation_case>{
      {"n_locations",
       [](timetable& m) {
         // Grow names_ and parents_ together: register_location() keeps them
         // in lockstep and expected_n_locations() asserts on that invariant.
         m.locations_.names_.resize(m.locations_.names_.size() + 1U);
         m.locations_.parents_.resize(m.locations_.parents_.size() + 1U);
       }},
      {"route_location_seq_",
       [](timetable& m) {
         m.route_location_seq_.emplace_back(std::vector<stop::value_type>{});
       }},
      {"transport_traffic_days_",
       [](timetable& m) {
         m.transport_traffic_days_[transport_idx_t{0}] = bitfield_idx_t{999};
       }},
      {"parents_",
       [](timetable& m) {
         m.locations_.parents_[location_idx_t{0}] = location_idx_t{999};
       }},
      {"equivalences_",
       [](timetable& m) {
         m.locations_.equivalences_[location_idx_t{0}].push_back(
             location_idx_t{999});
       }},
      {"children_",
       [](timetable& m) {
         m.locations_.children_[location_idx_t{0}].push_back(
             location_idx_t{999});
       }},
      {"location_routes_",
       [](timetable& m) {
         m.location_routes_.emplace_back(
             std::vector<route_idx_t>{route_idx_t{999}});
       }},
      {"bitfields_",
       [](timetable& m) {
         auto& bf = m.bitfields_[bitfield_idx_t{0}];
         bf.set(0U, !bf.test(0U));
       }},
      {"route_stop_times_",
       [](timetable& m) {
         // Derive the new value from the old one so it cannot coincide with it.
         auto& d = m.route_stop_times_[0U];
         d = delta{static_cast<std::uint16_t>(d.count() + 1)};
       }},
      {"transport_route_",
       [](timetable& m) {
         m.transport_route_[transport_idx_t{0}] = route_idx_t{999};
       }},
      {"date_range_from_",
       [](timetable& m) { m.date_range_.from_ += date::days{1}; }},
      {"date_range_to_",
       [](timetable& m) { m.date_range_.to_ += date::days{1}; }},
      {"footpaths_out_",
       [](timetable& m) {
         m.locations_.footpaths_out_[0].emplace_back(
             std::vector<footpath>{footpath{location_idx_t{0}, duration_t{10}}});
       }},
      {"footpaths_in_",
       [](timetable& m) {
         m.locations_.footpaths_in_[0].emplace_back(
             std::vector<footpath>{footpath{location_idx_t{0}, duration_t{10}}});
       }},
      {"route_transport_ranges_",
       [](timetable& m) {
         m.route_transport_ranges_[route_idx_t{0}].to_ = transport_idx_t{999};
       }},
      {"route_stop_time_ranges_",
       [](timetable& m) {
         m.route_stop_time_ranges_[route_idx_t{0}].to_ += 1U;
       }},
  };

  for (auto const& c : cases) {
    timetable tt_mut = tt;
    c.mutate_(tt_mut);
    EXPECT_FALSE(idx.is_valid_for(tt_mut, profile_idx_t{0}, 15))
        << "mutating " << c.name_ << " did not invalidate the index";
  }
}

TEST(routing, tad_ultra_index_requires_setup_time_validation) {
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, files_two_routes_multihop(), tt);
  finalize(tt);

  auto const idx = routing::build_tad_ultra_index(tt, 15);
  auto const tmp_path =
      std::filesystem::temp_directory_path() / "test_ultra_index_validation.bin";
  std::filesystem::remove(tmp_path);
  ASSERT_TRUE(idx.write(tmp_path));

  auto read_idx = routing::tad_transfer_index::read(tmp_path);
  ASSERT_TRUE(read_idx.has_value());

  // A deserialized index carries no proof that it matches this timetable, so
  // the query path must reject it until setup has validated it once.
  EXPECT_TRUE(read_idx->has_shortcuts_for(profile_idx_t{0}));
  EXPECT_FALSE(read_idx->is_valid_for_query(profile_idx_t{0}));

  EXPECT_FALSE(read_idx->validate_for_timetable(tt, 30));
  EXPECT_FALSE(read_idx->is_valid_for_query(profile_idx_t{0}));

  EXPECT_TRUE(read_idx->validate_for_timetable(tt, 15));
  EXPECT_TRUE(read_idx->is_valid_for_query(profile_idx_t{0}));

  // Non-default transfer time settings still disable the shortcuts per query.
  routing::transfer_time_settings non_default_tts;
  non_default_tts.default_ = false;
  EXPECT_FALSE(read_idx->is_valid_for_query(profile_idx_t{0}, non_default_tts));

  std::filesystem::remove(tmp_path);
}

TEST(routing, tad_query_does_not_recompute_timetable_fingerprint) {
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, files_abc(), tt);
  finalize(tt);

  auto const ultra_idx = routing::build_tad_ultra_index(tt, 15);
  ASSERT_TRUE(ultra_idx.is_valid_for_query(profile_idx_t{0}));

  auto const start_loc = tt.locations_.location_id_to_idx_.at({"0000001", src});
  auto const dest_loc = tt.locations_.location_id_to_idx_.at({"0000003", src});

  routing::query q;
  q.start_time_ = unixtime_t{sys_days{2020_y / March / 30}} + 2_hours;
  q.start_.push_back(routing::offset{start_loc, 0_minutes, 0});
  q.destination_.push_back(routing::offset{dest_loc, 0_minutes, 0});

  // Hashing the timetable is O(timetable) and belongs to index setup only.
  // Doing it per query silently turns every ULTRA measurement into a
  // measurement of the validation itself.
  auto const before =
      routing::tad_transfer_index::n_fingerprint_computations_.load();

  routing::search_state s_state;
  routing::tad_state t_state;
  auto const res = routing::tad_route(tt, nullptr, s_state, t_state, q,
                                      direction::kForward, std::nullopt,
                                      &ultra_idx);
  ASSERT_EQ(1ULL, res.algo_stats_.at("tad_using_shortcut_provider"));

  EXPECT_EQ(before,
            routing::tad_transfer_index::n_fingerprint_computations_.load());
}







// Endpoint-only walks must not depend on an unrelated second transit service
// inducing an intermediate shortcut. A witness budget is not a walking cap.
TEST(routing, tad_ultra_endpoint_and_long_walk_parity) {
  constexpr auto src = source_idx_t{0U};
  auto const first = std::string{R"(
*Z 01337 80____       048 030                             %
*A VE 0000001 0000002 000005                              %
*G RE  0000001 0000002                                    %
0000001 A                            00200                %
0000002 B                     00230                       %
)"};
  auto const second = std::string{R"(
*Z 07331 80____       048 030                             %
*A VE 0000004 0000005 000005                              %
*G RE  0000004 0000005                                    %
0000004 C                            00315                %
0000005 D                     00345                       %
)"};
  struct scenario {
    char const* name_;
    std::string services_;
    char const* from_;
    char const* to_;
    duration_t walk_;
  };
  for (auto const& c : std::vector<scenario>{
           {"terminal_egress", first, "0000001", "0000004", 7_minutes},
           {"terminal_access", second, "0000002", "0000005", 7_minutes},
           {"direct_walk", first, "0000002", "0000004", 7_minutes},
           {"long_intermediate", first + second, "0000001", "0000005", 16_minutes}}) {
    SCOPED_TRACE(c.name_);
    timetable tt;
    tt.date_range_ = full_period();
    load_timetable(src, loader::hrd::hrd_5_20_26, custom_hrd(c.services_), tt);
    finalize(tt);
    auto const loc = [&](char const* id) {
      return tt.locations_.location_id_to_idx_.at({id, src});
    };
    auto const b = loc("0000002");
    auto const c_stop = loc("0000004");
    tt.locations_.footpaths_out_[0].resize(tt.n_locations());
    tt.locations_.footpaths_in_[0].resize(tt.n_locations());
    tt.locations_.footpaths_out_[0][b].push_back(footpath{c_stop, c.walk_});
    tt.locations_.footpaths_in_[0][c_stop].push_back(footpath{b, c.walk_});
    auto const idx = routing::build_tad_ultra_index(tt, 15);
    for (auto const dir : {direction::kForward, direction::kBackward}) {
      SCOPED_TRACE(dir == direction::kForward ? "forward" : "backward");
      routing::query q;
      q.start_time_ = unixtime_t{sys_days{2020_y / March / 30}} +
                      (dir == direction::kForward ? 0_hours : 6_hours);
      q.start_.push_back(routing::offset{
          loc(dir == direction::kForward ? c.from_ : c.to_), 0_minutes, 0});
      q.destination_.push_back(routing::offset{
          loc(dir == direction::kForward ? c.to_ : c.from_), 0_minutes, 0});
      routing::search_state s_base, s_idx;
      routing::tad_state t_base, t_idx;
      auto const base_result = routing::tad_route(
          tt, nullptr, s_base, t_base, q, dir);
      auto const indexed_result = routing::tad_route(
          tt, nullptr, s_idx, t_idx, q, dir, std::nullopt, &idx);
      EXPECT_FALSE(base_result.journeys_->empty());
      if (base_result.journeys_->empty()) continue;
      EXPECT_EQ(base_result.journeys_->size(), indexed_result.journeys_->size());
      if (!indexed_result.journeys_->empty()) {
        EXPECT_EQ(base_result.journeys_->begin()->dest_time_,
                  indexed_result.journeys_->begin()->dest_time_);
        EXPECT_EQ(base_result.journeys_->begin()->start_time_,
                  indexed_result.journeys_->begin()->start_time_);
      }
    }
  }
}
