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
  prf.out_offsets_.resize(11, 0);
  prf.in_offsets_.resize(11, 0);
  prf.out_offsets_[1] = 0;
  for (std::size_t i = 2; i <= 10; ++i) {
    prf.out_offsets_[i] = 1;
  }
  prf.out_edges_.push_back(routing::tad_transfer_edge{location_idx_t{2}, duration_t{5}});

  EXPECT_TRUE(idx.is_valid_for(profile_idx_t{0}));
  EXPECT_FALSE(idx.is_valid_for(profile_idx_t{1}));

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
  EXPECT_FALSE(idx.is_valid_for(profile_idx_t{0}));
}

TEST(routing, tad_shortcut_scalar_parity) {
  constexpr auto const src = source_idx_t{0U};
  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, files_abc(), tt);
  finalize(tt);

  auto const transfer_idx = routing::build_tad_transfer_index(tt);
  EXPECT_TRUE(transfer_idx.is_valid_for(profile_idx_t{0}));

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
  EXPECT_TRUE(idx.is_valid_for(profile_idx_t{0}));
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
  EXPECT_TRUE(ultra_idx.is_valid_for(profile_idx_t{0}));

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

  ASSERT_EQ(res_default.journeys_->size(), res_shortcut.journeys_->size());
  if (!res_default.journeys_->empty()) {
    EXPECT_EQ(res_default.journeys_->begin()->dest_time_,
              res_shortcut.journeys_->begin()->dest_time_);
    EXPECT_EQ(res_default.journeys_->begin()->legs_,
              res_shortcut.journeys_->begin()->legs_);
  }
}



