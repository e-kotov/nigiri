#include "gtest/gtest.h"

#include "nigiri/loader/hrd/load_timetable.h"
#include "nigiri/loader/init_finish.h"

#include "../loader/hrd/hrd_timetable.h"

#include "nigiri/routing/tad/tad_search.h"
#include "nigiri/routing/tad/tad_state.h"
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
  auto const start_loc = tt.locations_.location_id_to_idx_.find({start_id, src})->second;
  auto const dest_loc = tt.locations_.location_id_to_idx_.find({dest_id, src})->second;

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
