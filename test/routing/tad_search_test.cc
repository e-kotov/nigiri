#include "gtest/gtest.h"

#include "nigiri/loader/hrd/load_timetable.h"
#include "nigiri/loader/init_finish.h"

#include "../loader/hrd/hrd_timetable.h"

#include "nigiri/routing/tad/tad_search.h"
#include "nigiri/routing/tad/tad_state.h"
#include "nigiri/routing/tad/tad_reconstruct.h"
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
  routing::tad_search search(tt, state, tts);
  
  auto const base = tt.internal_interval_days().from_;
  auto const start_delta = unix_to_delta(base, start_time);
  
  search.execute(start_loc, start_delta);
  
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
      routing::tad_reconstruct_journey(tt, tts, state, start_loc, dest_loc, base, tad_journey);
      
      routing::journey const* best_raptor = nullptr;
      for (auto const& j : raptor_results) {
          auto arr = unix_to_delta(base, j.legs_.back().arr_time_);
          if (arr == raptor_arrival) {
              best_raptor = &j;
              break;
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
  routing::tad_search search(tt, state, tts);
  auto const start_time = unixtime_t{sys_days{2020_y / March / 30}};
  auto const base = tt.internal_interval_days().from_;
  
  auto const f_meta_it = tt.locations_.location_id_to_idx_.find({"0000010", src});
  ASSERT_NE(f_meta_it, tt.locations_.location_id_to_idx_.end());
  auto const f_meta_loc = f_meta_it->second;
  
  search.execute(start_loc, unix_to_delta(base, start_time));
  
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
  routing::tad_search search(tt, state, tts);
  auto const start_time = unixtime_t{sys_days{2020_y / March / 30}};
  auto const base = tt.internal_interval_days().from_;
  search.execute(start_loc, unix_to_delta(base, start_time));
  
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
