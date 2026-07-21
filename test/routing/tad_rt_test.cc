#include "gtest/gtest.h"

#include "nigiri/loader/gtfs/load_timetable.h"
#include "nigiri/loader/init_finish.h"
#include "nigiri/rt/create_rt_timetable.h"
#include "nigiri/rt/gtfsrt_update.h"
#include "nigiri/rt/rt_timetable.h"
#include "nigiri/routing/tad/tad_search.h"
#include "nigiri/routing/tad/tad_state.h"

#include "../raptor_search.h"

using namespace date;
using namespace nigiri;
using namespace nigiri::loader;
using namespace nigiri::loader::gtfs;
using namespace std::chrono_literals;

namespace {

mem_dir test_files() {
  return mem_dir::read(R"(
# agency.txt
agency_id,agency_name,agency_url,agency_timezone
DB,Deutsche Bahn,https://deutschebahn.com,Europe/Berlin

# stops.txt
stop_id,stop_name,stop_desc,stop_lat,stop_lon,stop_url,location_type,parent_station
A,A,,0.0,1.0,,
B,B,,0.02,1.03,,
C,C,,0.04,1.05,,
D,D,,0.06,1.07,,

# calendar_dates.txt
service_id,date,exception_type
S_RE1,20190501,1
S_RE2,20190503,1
S_RE3,20190503,1

# routes.txt
route_id,agency_id,route_short_name,route_long_name,route_desc,route_type
R_RE1,DB,RE 1,,,3
R_RE2,DB,RE 2,,,3
R_RE3,DB,RE 3,,,3

# trips.txt
route_id,service_id,trip_id,trip_headsign,block_id
R_RE1,S_RE1,T_RE1,RE 1,
R_RE2,S_RE2,T_RE2,RE 2,
R_RE3,S_RE3,T_RE3,RE 3,

# stop_times.txt
trip_id,arrival_time,departure_time,stop_id,stop_sequence,pickup_type,drop_off_type
T_RE1,49:00:00,49:00:00,A,1,0,0
T_RE1,50:00:00,50:00:00,B,2,0,0
T_RE2,02:10:00,02:10:00,B,1,0,0
T_RE2,02:25:00,02:25:00,C,2,0,0
T_RE2,02:40:00,02:40:00,D,3,0,0
T_RE3,02:20:00,02:20:00,B,1,0,0
T_RE3,02:30:00,02:30:00,C,2,0,0
T_RE3,02:50:00,02:50:00,D,3,0,0
)");
}

delta_t run_tad(timetable const& tt, rt_timetable const* rtt, location_idx_t start_loc, delta_t start_time, location_idx_t dest_loc, day_idx_t search_base_day) {
  routing::transfer_time_settings tts;
  routing::tad_state state;
  routing::tad_search search(tt, rtt, state, tts, nigiri::profile_idx_t{0}, nigiri::routing::all_clasz_allowed(), false, false, search_base_day);
  
  search.execute({{start_loc, start_time}});
  
  auto const dest_label = state.node_labels_[cista::to_idx(dest_loc)];
  return dest_label.time_stamp_ == state.current_time_stamp_ ? dest_label.arrival_time_ : kInvalidDelta<direction::kForward>;
}

delta_t run_raptor(timetable const& tt, rt_timetable const* rtt, std::string const& start, std::string const& dest, unixtime_t start_time, day_idx_t search_base_day) {
  auto const results = test::raptor_search(tt, rtt, start, dest, interval{start_time, start_time + 1_hours});
  if (results.empty()) return kInvalidDelta<direction::kForward>;
  
  delta_t earliest = kInvalidDelta<direction::kForward>;
  auto const search_base_date = tt.internal_interval_days().from_ + search_base_day.v_ * date::days{1};
  for (auto const& j : results) {
    auto const arr = unix_to_delta(search_base_date, j.legs_.back().arr_time_);
    if (earliest == kInvalidDelta<direction::kForward> || arr < earliest) earliest = arr;
  }
  return earliest;
}

} // namespace

TEST(routing, tad_rt_delay_parity) {
  auto const to_unix = [](auto&& x) { return std::chrono::time_point_cast<std::chrono::seconds>(x).time_since_epoch().count(); };
  
  timetable tt;
  tt.date_range_ = {date::sys_days{2019_y / March / 25}, date::sys_days{2019_y / November / 1}};
  load_timetable({}, source_idx_t{0}, test_files(), tt);
  finalize(tt);
  
  auto rtt = rt::create_rt_timetable(tt, date::sys_days{2019_y / May / 3});
  
  transit_realtime::FeedMessage msg;
  auto const hdr = msg.mutable_header();
  hdr->set_gtfs_realtime_version("2.0");
  hdr->set_incrementality(transit_realtime::FeedHeader_Incrementality_FULL_DATASET);
  hdr->set_timestamp(to_unix(date::sys_days{2019_y / May / 4} + 9h));
  
  auto const e = msg.add_entity();
  e->set_id("1");
  auto const td = e->mutable_trip_update()->mutable_trip();
  td->set_start_time("02:10:00");
  td->set_start_date("20190503");
  td->set_trip_id("T_RE2");
  
  auto const stop_update = e->mutable_trip_update()->add_stop_time_update();
  stop_update->set_stop_sequence(1U);
  stop_update->mutable_departure()->set_delay(2 * 60 * 60 /* 2h */); // Delay from 02:10 to 04:10
  
  rt::gtfsrt_update_msg(tt, rtt, source_idx_t{0}, "tag", msg);
  
  auto const start_day = sys_days{May / 2 / 2019};
  auto const start_time = start_day + 23h;
  auto const search_base_day = tt.day_idx(start_day);
  auto const search_base_date = tt.internal_interval_days().from_ + search_base_day.v_ * date::days{1};
  auto const start_delta = unix_to_delta(search_base_date, start_time);
  auto const start_loc = tt.locations_.location_id_to_idx_.at({"A", source_idx_t{0}});
  auto const dest_loc = tt.locations_.location_id_to_idx_.at({"D", source_idx_t{0}});
  
  auto tad_arr = run_tad(tt, &rtt, start_loc, start_delta, dest_loc, search_base_day);
  auto raptor_arr = run_raptor(tt, &rtt, "A", "D", start_time, search_base_day);
  
  // They should both find a path and the arrival time should be the same
  EXPECT_NE(tad_arr, kInvalidDelta<direction::kForward>);
  EXPECT_EQ(tad_arr, raptor_arr);
}

TEST(routing, tad_rt_cancel_parity) {
  auto const to_unix = [](auto&& x) { return std::chrono::time_point_cast<std::chrono::seconds>(x).time_since_epoch().count(); };
  
  timetable tt;
  tt.date_range_ = {date::sys_days{2019_y / March / 25}, date::sys_days{2019_y / November / 1}};
  load_timetable({}, source_idx_t{0}, test_files(), tt);
  finalize(tt);
  
  auto rtt = rt::create_rt_timetable(tt, date::sys_days{2019_y / May / 3});
  
  transit_realtime::FeedMessage msg;
  auto const hdr = msg.mutable_header();
  hdr->set_gtfs_realtime_version("2.0");
  hdr->set_incrementality(transit_realtime::FeedHeader_Incrementality_FULL_DATASET);
  hdr->set_timestamp(to_unix(date::sys_days{2019_y / May / 4} + 9h));
  
  auto const e = msg.add_entity();
  e->set_id("1");
  auto const td = e->mutable_trip_update()->mutable_trip();
  td->set_start_time("02:10:00");
  td->set_start_date("20190503");
  td->set_trip_id("T_RE2");
  td->set_schedule_relationship(transit_realtime::TripDescriptor_ScheduleRelationship_CANCELED);
  
  rt::gtfsrt_update_msg(tt, rtt, source_idx_t{0}, "tag", msg);
  
  auto const start_day = sys_days{May / 2 / 2019};
  auto const start_time = start_day + 23h;
  auto const search_base_day = tt.day_idx(start_day);
  auto const search_base_date = tt.internal_interval_days().from_ + search_base_day.v_ * date::days{1};
  auto const start_delta = unix_to_delta(search_base_date, start_time);
  auto const start_loc = tt.locations_.location_id_to_idx_.at({"A", source_idx_t{0}});
  auto const dest_loc = tt.locations_.location_id_to_idx_.at({"D", source_idx_t{0}});
  
  auto tad_arr = run_tad(tt, &rtt, start_loc, start_delta, dest_loc, search_base_day);
  auto raptor_arr = run_raptor(tt, &rtt, "A", "D", start_time, search_base_day);
  
  // T_RE2 is canceled, so the earliest is now T_RE3! T_RE3 gets to D at 01:10
  EXPECT_NE(tad_arr, kInvalidDelta<direction::kForward>);
  EXPECT_EQ(tad_arr, raptor_arr);
}

TEST(routing, tad_rt_added_trip_parity) {
  auto const to_unix = [](auto&& x) { return std::chrono::time_point_cast<std::chrono::seconds>(x).time_since_epoch().count(); };
  
  timetable tt;
  tt.date_range_ = {date::sys_days{2019_y / March / 25}, date::sys_days{2019_y / November / 1}};
  load_timetable({}, source_idx_t{0}, test_files(), tt);
  finalize(tt);
  
  auto rtt = rt::create_rt_timetable(tt, date::sys_days{2019_y / May / 3});
  
  transit_realtime::FeedMessage msg;
  auto const hdr = msg.mutable_header();
  hdr->set_gtfs_realtime_version("2.0");
  hdr->set_incrementality(transit_realtime::FeedHeader_Incrementality_FULL_DATASET);
  hdr->set_timestamp(to_unix(date::sys_days{2019_y / May / 4} + 9h));
  
  auto const e = msg.add_entity();
  e->set_id("1");
  auto const tu = e->mutable_trip_update();
  auto const td = tu->mutable_trip();
  td->set_start_time("02:05:00");
  td->set_start_date("20190503");
  td->set_trip_id("NEW");
  td->set_schedule_relationship(transit_realtime::TripDescriptor_ScheduleRelationship_NEW);
  tu->mutable_trip_properties()->set_trip_short_name("Added Express");
  
  {
    auto const stop_update = tu->add_stop_time_update();
    stop_update->set_stop_sequence(1U);
    stop_update->set_stop_id("B");
    stop_update->mutable_departure()->set_time(to_unix(date::sys_days{2019_y / May / 3} + 2h + 5min));
  }
  {
    auto const stop_update = tu->add_stop_time_update();
    stop_update->set_stop_sequence(2U);
    stop_update->set_stop_id("D");
    stop_update->mutable_arrival()->set_time(to_unix(date::sys_days{2019_y / May / 3} + 2h + 15min));
  }
  
  rt::gtfsrt_update_msg(tt, rtt, source_idx_t{0}, "tag", msg);
  
  auto const start_day = sys_days{May / 2 / 2019};
  auto const start_time = start_day + 23h;
  auto const search_base_day = tt.day_idx(start_day);
  auto const search_base_date = tt.internal_interval_days().from_ + search_base_day.v_ * date::days{1};
  auto const start_delta = unix_to_delta(search_base_date, start_time);
  auto const start_loc = tt.locations_.location_id_to_idx_.at({"A", source_idx_t{0}});
  auto const dest_loc = tt.locations_.location_id_to_idx_.at({"D", source_idx_t{0}});
  
  auto tad_arr = run_tad(tt, &rtt, start_loc, start_delta, dest_loc, search_base_day);
  auto raptor_arr = run_raptor(tt, &rtt, "A", "D", start_time, search_base_day);
  
  // The new trip gets to D at 00:25, which is earlier than T_RE2 (01:00)
  EXPECT_NE(tad_arr, kInvalidDelta<direction::kForward>);
  EXPECT_EQ(tad_arr, raptor_arr);
}
