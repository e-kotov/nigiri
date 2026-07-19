#include "nigiri/routing/tad/tad_route.h"
#include "nigiri/routing/tad/tad_search.h"
#include "nigiri/routing/tad/tad_reconstruct.h"
#include "nigiri/routing/journey.h"
#include "nigiri/special_stations.h"
#include "utl/overloaded.h"
#include "date/date.h"

namespace nigiri::routing {

routing_result tad_route(
    timetable const& tt,
    rt_timetable const*,
    search_state& s_state,
    tad_state& t_state,
    query q,
    direction search_dir,
    std::optional<std::chrono::seconds>) {
    
    if (search_dir != direction::kForward) {
        throw std::runtime_error("TAD MVP only supports forward search");
    }

    s_state.results_.clear();

    unixtime_t start_time;
    if (std::holds_alternative<interval<unixtime_t>>(q.start_time_)) {
        start_time = std::get<interval<unixtime_t>>(q.start_time_).from_;
    } else {
        start_time = std::get<unixtime_t>(q.start_time_);
    }

    auto const base = tt.internal_interval_days().from_;
    auto const start_delta = unix_to_delta(base, start_time);
    
    std::vector<std::pair<location_idx_t, delta_t>> starts;
    for (auto const& l : q.start_) {
        starts.emplace_back(l.target(), start_delta + static_cast<delta_t>(l.duration().count()));
    }

    auto processing_start_time = std::chrono::steady_clock::now();

    routing::tad_search search(tt, t_state, q.transfer_time_settings_);
    auto stats = search.execute(starts);

    auto execution_duration = std::chrono::steady_clock::now() - processing_start_time;

    search_stats search_stats_out;
    search_stats_out.execute_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(execution_duration);
    
    std::map<std::string, std::uint64_t> algo_stats_out = {
        {"tad_settled_stop_labels", stats.settled_stop_labels_},
        {"tad_scanned_routes", stats.scanned_routes_},
        {"tad_scanned_stop_events", stats.scanned_stop_events_},
        {"tad_relaxed_walking_edges", stats.relaxed_walking_edges_},
        {"tad_pruned_scans", stats.pruned_scans_},
        {"tad_queue_updates", stats.queue_updates_},
        {"tad_engine_used", 1ULL}
    };

    for (auto const& dest_offset : q.destination_) {
        auto const dest_loc = dest_offset.target();
        if (cista::to_idx(dest_loc) >= t_state.node_labels_.size()) {
            continue;
        }
        auto const& dest_label = t_state.node_labels_[cista::to_idx(dest_loc)];
        
        if (dest_label.time_stamp_ == t_state.current_time_stamp_ && 
            dest_label.arrival_time_ != kInvalidDelta<direction::kForward>) {
            
            routing::journey j;
            auto curr = dest_loc;
            while (true) {
                auto const& label = t_state.node_labels_[cista::to_idx(curr)];
                if (label.parent_ == curr || label.parent_ == location_idx_t::invalid()) {
                    break;
                }
                curr = label.parent_;
            }
            auto best_start = curr;
            
            routing::tad_reconstruct_journey(tt, q.transfer_time_settings_, t_state, best_start, dest_loc, base, j);
            
            offset start_match = offset{location_idx_t::invalid(), duration_t{0}, 0};
            bool has_start_match = false;
            for (auto const& start_offset : q.start_) {
                if (start_offset.target() == best_start) {
                    start_match = start_offset;
                    has_start_match = true;
                    break;
                }
            }
            
            if (has_start_match && q.start_match_mode_ == location_match_mode::kIntermodal) {
                j.legs_.insert(j.legs_.begin(), journey::leg{
                    direction::kForward,
                    get_special_station(special_station::kStart),
                    best_start,
                    start_time,
                    start_time + std::chrono::minutes{start_match.duration().count()},
                    start_match
                });
            }
            
            auto const core_arrival = start_time + std::chrono::minutes{dest_label.arrival_time_ - start_delta};
            auto const final_arrival = core_arrival + std::chrono::minutes{dest_offset.duration().count()};
            
            if (q.dest_match_mode_ == location_match_mode::kIntermodal) {
                j.legs_.push_back(journey::leg{
                    direction::kForward,
                    dest_loc,
                    get_special_station(special_station::kEnd),
                    core_arrival,
                    final_arrival,
                    dest_offset
                });
            }
            
            j.start_time_ = start_time;
            j.dest_time_ = final_arrival;
            j.transfers_ = j.legs_.empty() ? 0 : static_cast<std::uint8_t>(j.legs_.size() - 1);
            
            s_state.results_.add(std::move(j));
        }
    }

    interval<unixtime_t> out_interval{start_time, start_time};

    return {&s_state.results_, out_interval, search_stats_out, algo_stats_out};
}

}  // namespace nigiri::routing
