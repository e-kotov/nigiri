#include "nigiri/routing/tad/tad_route.h"
#include "nigiri/routing/tad/tad_search.h"
#include "nigiri/routing/tad/tad_reconstruct.h"
#include "nigiri/routing/journey.h"
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
        {"tad_queue_updates", stats.queue_updates_}
    };

    delta_t best_arr = kInvalidDelta<direction::kForward>;
    location_idx_t best_dest = location_idx_t::invalid();

    for (auto const& dest_offset : q.destination_) {
        auto const dest_loc = dest_offset.target();
        auto const& dest_label = t_state.node_labels_[cista::to_idx(dest_loc)];
        
        if (dest_label.time_stamp_ == t_state.current_time_stamp_) {
            auto arr = dest_label.arrival_time_ + static_cast<delta_t>(dest_offset.duration().count());
            if (best_arr == kInvalidDelta<direction::kForward> || arr < best_arr) {
                best_arr = arr;
                best_dest = dest_loc;
            }
        }
    }

    if (best_dest != location_idx_t::invalid()) {
        routing::journey j;
        
        auto curr = best_dest;
        while (true) {
            auto const& label = t_state.node_labels_[cista::to_idx(curr)];
            if (label.parent_ == curr || label.parent_ == location_idx_t::invalid()) {
                break;
            }
            curr = label.parent_;
        }
        auto best_start = curr;
        
        routing::tad_reconstruct_journey(tt, q.transfer_time_settings_, t_state, best_start, best_dest, base, j);
        
        j.start_time_ = start_time;
        j.dest_time_ = start_time + std::chrono::minutes(best_arr - start_delta);
        j.transfers_ = j.legs_.empty() ? 0 : static_cast<std::uint8_t>(j.legs_.size() - 1);
        
        s_state.results_.add(std::move(j));
    }

    interval<unixtime_t> out_interval{start_time, start_time};

    return {&s_state.results_, out_interval, search_stats_out, algo_stats_out};
}

}  // namespace nigiri::routing
