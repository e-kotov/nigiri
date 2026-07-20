#include "nigiri/routing/tad/tad_route.h"
#include "nigiri/routing/tad/tad_search.h"
#include "nigiri/routing/tad/tad_reconstruct.h"
#include "nigiri/routing/journey.h"
#include "nigiri/special_stations.h"
#include "nigiri/td_footpath.h"
#include "utl/overloaded.h"
#include "date/date.h"
#include "nigiri/routing/raptor/reconstruct.h"
#include "date/date.h"

namespace nigiri::routing {

routing_result tad_route(
    timetable const& tt,
    rt_timetable const* rtt,
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
    for (auto const& [loc, td_offsets] : q.td_start_) {
        auto const dur_opt = get_td_duration(direction::kForward, std::span{td_offsets}, start_time);
        if (dur_opt.has_value()) {
            starts.emplace_back(loc, start_delta + static_cast<delta_t>(dur_opt->first.count()));
        }
    }

    auto processing_start_time = std::chrono::steady_clock::now();

    routing::tad_search search(tt, t_state, q.transfer_time_settings_,
                               q.prf_idx_, q.allowed_claszes_,
                               q.require_bike_transport_, q.require_car_transport_);
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

    auto const add_journey = [&](location_idx_t const dest_loc, offset const dest_offset) {
        if (cista::to_idx(dest_loc) >= t_state.node_labels_.size()) return;
        auto const& dest_label = t_state.node_labels_[cista::to_idx(dest_loc)];
        if (dest_label.time_stamp_ != t_state.current_time_stamp_ || 
            dest_label.arrival_time_ == kInvalidDelta<direction::kForward>) return;
            
        routing::journey j;
        auto curr = dest_loc;
        while (true) {
            auto const& label = t_state.node_labels_[cista::to_idx(curr)];
            if (label.parent_ == curr || label.parent_ == location_idx_t::invalid()) break;
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
        if (!has_start_match) {
            auto it = q.td_start_.find(best_start);
            if (it != q.td_start_.end()) {
                auto const dur_opt = get_td_duration(direction::kForward, std::span{it->second}, start_time);
                if (dur_opt.has_value()) {
                    start_match = offset{best_start, dur_opt->first, dur_opt->second.transport_mode_id_};
                    has_start_match = true;
                }
            }
        }
        
        if (has_start_match && q.start_match_mode_ == location_match_mode::kIntermodal) {
            j.legs_.insert(j.legs_.begin(), journey::leg{
                direction::kForward, get_special_station(special_station::kStart), best_start,
                start_time, start_time + std::chrono::minutes{start_match.duration().count()}, start_match
            });
        }
        
        auto const core_arrival = start_time + std::chrono::minutes{dest_label.arrival_time_ - start_delta};
        auto const final_arrival = core_arrival + std::chrono::minutes{dest_offset.duration().count()};
        
        if (q.dest_match_mode_ == location_match_mode::kIntermodal) {
            j.legs_.push_back(journey::leg{
                direction::kForward, dest_loc, get_special_station(special_station::kEnd),
                core_arrival, final_arrival, dest_offset
            });
        }
        
        j.start_time_ = start_time;
        j.dest_time_ = final_arrival;
        
        optimize_footpaths<direction::kForward>(tt, rtt, q, j);
        
        // Ensure transfers reflects the final legs array
        j.transfers_ = j.legs_.empty() ? 0 : static_cast<std::uint8_t>(j.legs_.size() - 1);
        j.transfers_ = j.legs_.empty() ? 0 : static_cast<std::uint8_t>(j.legs_.size() - 1);
        s_state.results_.add(std::move(j));
    };

    for (auto const& dest_offset : q.destination_) {
        add_journey(dest_offset.target(), dest_offset);
    }
    for (auto const& [dest_loc, td_offsets] : q.td_dest_) {
        if (cista::to_idx(dest_loc) >= t_state.node_labels_.size()) continue;
        auto const& dest_label = t_state.node_labels_[cista::to_idx(dest_loc)];
        if (dest_label.time_stamp_ != t_state.current_time_stamp_ || 
            dest_label.arrival_time_ == kInvalidDelta<direction::kForward>) continue;
            
        auto const core_arrival = start_time + std::chrono::minutes{dest_label.arrival_time_ - start_delta};
        auto const dur_opt = get_td_duration(direction::kForward, std::span{td_offsets}, core_arrival);
        if (dur_opt.has_value()) {
            add_journey(dest_loc, offset{dest_loc, dur_opt->first, dur_opt->second.transport_mode_id_});
        }
    }

    interval<unixtime_t> out_interval{start_time, start_time};

    return {&s_state.results_, out_interval, search_stats_out, algo_stats_out};
}

}  // namespace nigiri::routing
