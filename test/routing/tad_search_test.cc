#include "gtest/gtest.h"

#include "nigiri/loader/hrd/load_timetable.h"
#include "nigiri/loader/init_finish.h"

#include "../loader/hrd/hrd_timetable.h"

#include "nigiri/routing/tad/tad_search.h"
#include "nigiri/routing/tad/tad_state.h"
#include "nigiri/routing/transfer_time_settings.h"
#include "nigiri/timetable.h"

using namespace date;
using namespace nigiri;
using namespace nigiri::loader;
using namespace nigiri::test_data::hrd_timetable;

TEST(routing, tad_forward) {
  constexpr auto const src = source_idx_t{0U};

  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, files_abc(), tt);
  finalize(tt);

  // Initialize TAD state and search
  routing::transfer_time_settings tts;
  routing::tad_state state;
  routing::tad_search search(tt, state, tts);

  // Map location "A" ("0000001")
  auto const start_loc_it = tt.locations_.location_id_to_idx_.find({ "0000001", src });
  ASSERT_NE(start_loc_it, tt.locations_.location_id_to_idx_.end());
  auto const start_loc = start_loc_it->second;
  
  // Set start time: 2020-03-30 05:00
  auto const base = tt.internal_interval_days().from_;
  auto const start_time = unixtime_t{sys_days{2020_y / March / 30}} + 5_hours;
  auto const start_delta = unix_to_delta(base, start_time);
  
  search.execute(start_loc, start_delta);

  // Check arrival at "C" ("0000003")
  auto const dest_loc_it = tt.locations_.location_id_to_idx_.find({ "0000003", src });
  ASSERT_NE(dest_loc_it, tt.locations_.location_id_to_idx_.end());
  auto const dest_loc = dest_loc_it->second;
  
  auto const dest_label = state.node_labels_[to_idx(dest_loc)];
  ASSERT_EQ(dest_label.time_stamp_, state.current_time_stamp_);
  
  auto const arrival = dest_label.arrival_time_;
  
  // MVP implementation verified valid path discovery
  EXPECT_EQ(arrival, 10545);
}
