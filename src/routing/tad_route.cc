#include "nigiri/routing/tad/tad_route.h"
#include "nigiri/routing/raptor_search.h"
#include "nigiri/routing/tad/tad_search.h"
#include "nigiri/routing/tad/tad_reconstruct.h"
#include "nigiri/routing/journey.h"
#include "nigiri/routing/limits.h"
#include "nigiri/special_stations.h"
#include "nigiri/td_footpath.h"
#include "nigiri/for_each_meta.h"
#include "utl/overloaded.h"
#include "utl/verify.h"
#include "date/date.h"
#include "nigiri/routing/raptor/reconstruct.h"
#include "nigiri/routing/start_times.h"
#include "utl/equal_ranges_linear.h"

namespace nigiri::routing {

routing_result tad_route(
    timetable const& tt,
    rt_timetable const* rtt,
    search_state& s_state,
    tad_state& t_state,
    query q,
    direction search_dir,
    std::optional<std::chrono::seconds> timeout,
    tad_transfer_index const* transfer_index) {
    // These contracts are implemented by upstream RAPTOR. Scalar TAD has no
    // reservation/vehicle-wheelchair filter or RT-only journey reconstruction.
    if (q.no_compulsory_reservation_ || q.prf_idx_ == kWheelchairProfile ||
        (rtt != nullptr && rtt->n_rt_transports() != 0U)) {
      raptor_state state;
      auto result = raptor_search(tt, rtt, s_state, state, q, search_dir, timeout);
      result.algo_stats_["tad_fallback_raptor"] = 1U;
      return result;
    }
    s_state.results_.clear();

    q.sanitize(tt);
    utl::verify(q.via_stops_.size() <= kMaxVias,
                "too many via stops: {}, limit: {}", q.via_stops_.size(),
                kMaxVias);

    auto const fwd = search_dir == direction::kForward;
    auto const invalid_delta = fwd ? kInvalidDelta<direction::kForward> : kInvalidDelta<direction::kBackward>;

    bool const is_interval_query = std::holds_alternative<nigiri::interval<unixtime_t>>(q.start_time_);

    unixtime_t query_time;
    if (is_interval_query) {
        auto const start_interval = std::get<nigiri::interval<unixtime_t>>(q.start_time_);
        query_time = fwd ? start_interval.from_ : start_interval.to_ - duration_t{1};
    } else {
        query_time = std::get<unixtime_t>(q.start_time_);
    }

    auto const search_base_day = tt.day_idx(std::chrono::time_point_cast<date::days>(query_time));
    auto const search_base_date = tt.internal_interval_days().from_ + search_base_day.v_ * date::days{1};
    auto const query_delta = unix_to_delta(search_base_date, query_time);
    
    // Endpoint walking must not depend on intermediate transit witnesses.
    // Non-default adjustments remain per-edge on the fallback path.
    std::unique_ptr<tad_transfer_provider> transfer_provider;
    auto const using_shortcuts = transfer_index != nullptr &&
        transfer_index->is_valid_for_query(q.prf_idx_, q.transfer_time_settings_);
    if (using_shortcuts) {
        transfer_provider = std::make_unique<endpoint_transfer_provider>(tt, q.prf_idx_);
    } else {
        if (transfer_index != nullptr) {
            warn_transfer_index_unusable(*transfer_index, q.prf_idx_, q.transfer_time_settings_);
        }
        transfer_provider = std::make_unique<footpath_transfer_provider>(tt, q.prf_idx_);
    }

    std::vector<std::pair<location_idx_t, delta_t>> starts;

    auto const add_start_location = [&](location_idx_t const loc, duration_t const dur) {
        if (fwd) {
            starts.emplace_back(loc, clamp(query_delta + dur.count()));
        } else {
            starts.emplace_back(loc, clamp(query_delta - dur.count()));
        }
    };

    std::vector<tad_transfer_edge> transfer_edges_buf;

    struct expanded_start_info {
        location_idx_t root_loc_;
        duration_t fp_dur_;
        duration_t combined_dur_;
    };
    hash_map<location_idx_t, expanded_start_info> expanded_start_footpaths;

    for (auto const& o : q.start_) {
        if (o.duration() > q.max_start_offset_) continue;
        for_each_meta(tt, q.start_match_mode_, o.target(), [&](location_idx_t const l) {
            add_start_location(l, o.duration());
            if (q.use_start_footpaths_) {
                transfer_edges_buf.clear();
                transfer_provider->stop_transfers(l, fwd, transfer_edges_buf);
                for (auto const& fp : transfer_edges_buf) {
                    if (fp.target_ == l) continue;
                    auto const transfer_dur = adjusted_transfer_time(q.transfer_time_settings_, fp.duration_);
                    auto const combined_dur = o.duration() + transfer_dur;
                    if (combined_dur <= q.max_start_offset_) {
                        add_start_location(fp.target_, combined_dur);
                        auto const it = expanded_start_footpaths.find(fp.target_);
                        if (it == expanded_start_footpaths.end() || combined_dur < it->second.combined_dur_) {
                            expanded_start_footpaths[fp.target_] = expanded_start_info{l, transfer_dur, combined_dur};
                        }
                    }
                }
            }
        });
    }
    for (auto const& [loc, td_offsets] : q.td_start_) {
        auto const dur_opt = get_td_duration(search_dir, std::span{td_offsets}, query_time);
        if (dur_opt.has_value() && duration_t{dur_opt->first} <= q.max_start_offset_) {
            add_start_location(loc, duration_t{dur_opt->first});
        }
    }

    auto const K = q.via_stops_.size();
    std::vector<tad_state> states;
    if (K > 0) {
        states.resize(K + 1);
    }

    auto processing_start_time = std::chrono::steady_clock::now();
    tad_stats total_stats{};

    int interval_length_in_days = 0;
    if (is_interval_query) {
        auto const start_interval = std::get<nigiri::interval<unixtime_t>>(q.start_time_);
        interval_length_in_days = std::max(0, static_cast<int>((start_interval.to_ - start_interval.from_).count() / 1440));
    }
    auto const min_day = std::max(0, static_cast<int>(search_base_day.v_) - 3);
    auto const max_day = std::min(static_cast<int>(kMaxDays - 1), static_cast<int>(search_base_day.v_) + 3 + interval_length_in_days);

    bitfield search_mask;
    for (int d = min_day; d <= max_day; ++d) {
        search_mask.set(d);
    }

    std::vector<uint8_t> route_active(tt.n_routes(), 0);

    std::function<void(
        std::vector<std::pair<location_idx_t, delta_t>> const&,
        unixtime_t const,
        day_idx_t const,
        date::sys_days const,
        delta_t const)> run_search_and_reconstruct = [&](
        std::vector<std::pair<location_idx_t, delta_t>> const& initial_starts,
        unixtime_t const cur_query_time,
        day_idx_t const cur_search_base_day,
        date::sys_days const cur_search_base_date,
        delta_t const cur_query_delta) {

        auto current_starts = initial_starts;

        struct expanded_egress_info {
            location_idx_t target_stop_;
            location_idx_t final_dest_loc_;
            duration_t transfer_dur_;
        };
        std::vector<expanded_egress_info> expanded_egress_targets;

        for (std::size_t leg_idx = 0; leg_idx <= K; ++leg_idx) {
            auto& current_state = (K == 0) ? t_state : states[leg_idx];
            bool const is_first_leg = (leg_idx == 0);

            std::vector<std::pair<location_idx_t, duration_t>> current_targets;
            bool enable_pruning = (leg_idx == K) && (is_interval_query || q.min_connection_count_ <= 1);
            if (enable_pruning) {
                auto const add_target = [&](location_idx_t const loc, duration_t const dur) {
                    auto const idx = cista::to_idx(loc);
                    for (auto& [t_loc, t_dur] : current_targets) {
                        if (cista::to_idx(t_loc) == idx) {
                            t_dur = std::min(t_dur, dur);
                            return;
                        }
                    }
                    current_targets.emplace_back(loc, dur);
                };

                for (auto const& dest_offset : q.destination_) {
                    for_each_meta(tt, q.dest_match_mode_, dest_offset.target(), [&](location_idx_t const dest_loc) {
                        add_target(dest_loc, dest_offset.duration());
                        transfer_edges_buf.clear();
                        transfer_provider->stop_transfers(dest_loc, !fwd, transfer_edges_buf);
                        for (auto const& fp : transfer_edges_buf) {
                            if (fp.target_ == dest_loc) continue;
                            auto const transfer_dur = adjusted_transfer_time(q.transfer_time_settings_, fp.duration_);
                            auto const combined_dur = dest_offset.duration() + transfer_dur;
                            add_target(fp.target_, combined_dur);
                            expanded_egress_targets.push_back(expanded_egress_info{fp.target_, dest_loc, transfer_dur});
                        }
                    });
                }
                for (auto const& [dest_loc, td_offsets] : q.td_dest_) {
                    duration_t min_dur{std::numeric_limits<duration_t::rep>::max()};
                    for (auto const& offset : td_offsets) {
                        min_dur = std::min(min_dur, offset.duration());
                    }
                    if (min_dur.count() != std::numeric_limits<duration_t::rep>::max()) {
                        add_target(dest_loc, min_dur);
                        transfer_edges_buf.clear();
                        transfer_provider->stop_transfers(dest_loc, !fwd, transfer_edges_buf);
                        for (auto const& fp : transfer_edges_buf) {
                            if (fp.target_ == dest_loc) continue;
                            auto const transfer_dur = adjusted_transfer_time(q.transfer_time_settings_, fp.duration_);
                            auto const combined_dur = min_dur + transfer_dur;
                            add_target(fp.target_, combined_dur);
                            expanded_egress_targets.push_back(expanded_egress_info{fp.target_, dest_loc, transfer_dur});
                        }
                    }
                }
            }

            routing::tad_search search(tt, rtt, current_state, q.transfer_time_settings_,
                                       q.prf_idx_, q.allowed_claszes_,
                                       q.require_bike_transport_, q.require_car_transport_,
                                       cur_search_base_day, search_dir, is_first_leg,
                                       cur_query_delta, q.max_travel_time_, q.max_transfers_,
                                       &route_active, search_mask, transfer_index);
            auto const stats = search.execute(current_starts, current_targets);

            total_stats.settled_stop_labels_ += stats.settled_stop_labels_;
            total_stats.scanned_routes_ += stats.scanned_routes_;
            total_stats.scanned_stop_events_ += stats.scanned_stop_events_;
            total_stats.relaxed_walking_edges_ += stats.relaxed_walking_edges_;
            total_stats.relaxed_transfers_ += stats.relaxed_transfers_;
            total_stats.pruned_scans_ += stats.pruned_scans_;
            total_stats.queue_updates_ += stats.queue_updates_;
            total_stats.time_pq_pop_us_ += stats.time_pq_pop_us_;
            total_stats.time_footpaths_us_ += stats.time_footpaths_us_;
            total_stats.time_routes_us_ += stats.time_routes_us_;
            total_stats.time_rt_transports_us_ += stats.time_rt_transports_us_;

            if (leg_idx < K) {
                std::vector<std::pair<location_idx_t, delta_t>> next_starts;
                auto const& via = q.via_stops_[leg_idx];
                for_each_meta(tt, location_match_mode::kEquivalent, via.location_, [&](location_idx_t const l) {
                    auto const l_idx = cista::to_idx(l);
                    if (l_idx < current_state.node_labels_.size()) {
                        auto const& lbl = current_state.node_labels_[l_idx];
                        if (lbl.time_stamp_ == current_state.current_time_stamp_ &&
                            lbl.arrival_time_ != invalid_delta) {
                            auto const arr_time = lbl.arrival_time_;
                            auto const dep_time = clamp(
                                static_cast<int>(arr_time) +
                                (fwd ? via.stay_.count() : -via.stay_.count()));
                            next_starts.emplace_back(l, dep_time);
                        }
                    }
                });
                if (next_starts.empty()) {
                    return;
                }
                current_starts = std::move(next_starts);
            }
        }

        auto& state_final = (K == 0) ? t_state : states[K];

        for (auto const& egress_info : expanded_egress_targets) {
            auto const s_idx = cista::to_idx(egress_info.target_stop_);
            if (s_idx >= state_final.node_labels_.size()) continue;
            auto const& s_lbl = state_final.node_labels_[s_idx];
            if (s_lbl.time_stamp_ != state_final.current_time_stamp_ || s_lbl.arrival_time_ == invalid_delta) continue;

            auto const arr_dest = fwd ? clamp(s_lbl.arrival_time_ + egress_info.transfer_dur_.count())
                                      : clamp(s_lbl.arrival_time_ - egress_info.transfer_dur_.count());

            auto const dest_idx = cista::to_idx(egress_info.final_dest_loc_);
            if (dest_idx >= state_final.node_labels_.size()) continue;
            auto& dest_lbl = state_final.node_labels_[dest_idx];

            bool update = false;
            if (dest_lbl.time_stamp_ != state_final.current_time_stamp_ || dest_lbl.arrival_time_ == invalid_delta) {
                update = true;
            } else {
                if (fwd ? arr_dest < dest_lbl.arrival_time_ : arr_dest > dest_lbl.arrival_time_) {
                    update = true;
                }
            }

            if (update) {
                dest_lbl.arrival_time_ = arr_dest;
                dest_lbl.parent_ = egress_info.target_stop_;
                dest_lbl.reached_by_walking_ = true;
                dest_lbl.parent_transport_ = transport_idx_t::invalid();
                dest_lbl.parent_day_ = day_idx_t::invalid();
                dest_lbl.time_stamp_ = state_final.current_time_stamp_;
            }
        }

        auto const find_root = [&](tad_state const& state, location_idx_t loc) {
            for (auto i = 0U; i != state.node_labels_.size(); ++i) {
                auto const& label = state.node_labels_[cista::to_idx(loc)];
                if (label.parent_ == loc || label.parent_ == location_idx_t::invalid()) {
                    return loc;
                }
                loc = label.parent_;
            }
            return location_idx_t::invalid();
        };

        std::function<void(tad_state const&, location_idx_t const, location_idx_t const, journey&)> reconstruct_path =
            [&](tad_state const& state,
                location_idx_t const reach_loc,
                location_idx_t const root_loc,
                journey& j) {
            if (fwd) {
                routing::tad_reconstruct_journey(tt, q.transfer_time_settings_, state, root_loc, reach_loc, q.prf_idx_, cur_search_base_date, cur_search_base_day, j);
            } else {
                auto curr_l = reach_loc;
                bool const is_wheelchair = q.prf_idx_ == kWheelchairProfile;
                for (auto i = 0U; i != state.node_labels_.size() && curr_l != root_loc &&
                     curr_l != location_idx_t::invalid(); ++i) {
                    auto const& lbl = state.node_labels_[cista::to_idx(curr_l)];
                    if (lbl.time_stamp_ != state.current_time_stamp_) {
                        throw std::runtime_error("TAD Reconstruct: broken path (timestamp mismatch)");
                    }
                    auto const parent_l = lbl.parent_;
                    if (parent_l == location_idx_t::invalid()) {
                        throw std::runtime_error("TAD Reconstruct: broken path (invalid parent)");
                    }
                    auto const& parent_lbl = state.node_labels_[cista::to_idx(parent_l)];
                    auto const dep_time = lbl.arrival_time_;
                    auto const arr_time = parent_lbl.arrival_time_;

                    if (lbl.reached_by_walking_) {
                        j.add(journey::leg{direction::kForward,
                                           curr_l,
                                           parent_l,
                                           delta_to_unix(cur_search_base_date, dep_time),
                                           delta_to_unix(cur_search_base_date, arr_time),
                                           footpath{parent_l, duration_t{static_cast<std::int16_t>(arr_time - dep_time)}}});
                    } else {
                        auto const t = lbl.parent_transport_;
                        if (t == transport_idx_t::invalid()) {
                            throw std::runtime_error("TAD Reconstruct: invalid parent transport");
                        }
                        auto const start_day = lbl.parent_day_;
                        if (start_day == day_idx_t::invalid()) {
                            throw std::runtime_error("TAD Reconstruct: invalid parent day");
                        }
                        auto const r = tt.transport_route_[t];
                        auto const seq = tt.route_location_seq_[r];
                        stop_idx_t board_stop_idx = 0;
                        delta_t actual_board_time = kInvalidDelta<direction::kForward>;
                        for (unsigned i = 0; i < seq.size(); ++i) {
                            if (stop{seq[i]}.location_idx() == curr_l && stop{seq[i]}.can_start<direction::kForward>(is_wheelchair)) {
                                auto const dep_mam = tt.event_mam(r, t, i, event_type::kDep);
                                auto const event_time = tt_to_delta(cur_search_base_day, start_day, minutes_after_midnight_t{dep_mam.count()});
                                if (event_time <= dep_time) {
                                    board_stop_idx = static_cast<stop_idx_t>(i);
                                    actual_board_time = event_time;
                                    break;
                                }
                            }
                        }
                        stop_idx_t alight_stop_idx = 0;
                        delta_t actual_arr_time = kInvalidDelta<direction::kForward>;
                        for (unsigned i = board_stop_idx + 1; i < seq.size(); ++i) {
                            if (stop{seq[i]}.location_idx() == parent_l && stop{seq[i]}.can_finish<direction::kForward>(is_wheelchair)) {
                                auto const arr_mam = tt.event_mam(r, t, i, event_type::kArr);
                                actual_arr_time = tt_to_delta(cur_search_base_day, start_day, minutes_after_midnight_t{arr_mam.count()});
                                alight_stop_idx = static_cast<stop_idx_t>(i);
                                break;
                            }
                        }
                        if (actual_board_time == kInvalidDelta<direction::kForward> ||
                            actual_arr_time == kInvalidDelta<direction::kForward>) {
                            throw std::runtime_error("TAD Reconstruct: unresolved board or alight time");
                        }
                        rt::run r_run{.t_ = transport{t, start_day},
                                      .stop_range_ = interval<stop_idx_t>{0, static_cast<stop_idx_t>(seq.size())}};
                        j.add(journey::leg{direction::kForward,
                                           curr_l,
                                           parent_l,
                                           delta_to_unix(cur_search_base_date, actual_board_time),
                                           delta_to_unix(cur_search_base_date, actual_arr_time),
                                           journey::run_enter_exit{r_run, board_stop_idx, alight_stop_idx}});
                                           
                        if (parent_l != root_loc && !parent_lbl.reached_by_walking_) {
                            auto const min_transfer = tt.locations_.transfer_time_[parent_l].count();
                            auto const transfer_dur = adjusted_transfer_time(q.transfer_time_settings_, min_transfer);
                            if (transfer_dur > 0) {
                                j.add(journey::leg{direction::kForward,
                                                   parent_l,
                                                   parent_l,
                                                   delta_to_unix(cur_search_base_date, actual_arr_time),
                                                   delta_to_unix(cur_search_base_date, static_cast<delta_t>(actual_arr_time + transfer_dur)),
                                                   footpath{parent_l, duration_t{static_cast<std::int16_t>(transfer_dur)}}});
                            }
                        }
                    }
                    curr_l = parent_l;
                }
                if (curr_l != root_loc) {
                    throw std::runtime_error("TAD Reconstruct: parent cycle detected");
                }
            }
        };
        auto const add_journey_for_dest = [&](location_idx_t const dest_loc, offset const dest_offset) {
            if (cista::to_idx(dest_loc) >= state_final.node_labels_.size()) return;
            auto const& dest_label = state_final.node_labels_[cista::to_idx(dest_loc)];
            if (dest_label.time_stamp_ != state_final.current_time_stamp_ || 
                dest_label.arrival_time_ == invalid_delta) return;
                
            std::vector<location_idx_t> via_locs(K + 2);
            via_locs[K + 1] = dest_loc;
            
            bool chain_valid = true;
            for (int leg = static_cast<int>(K); leg >= 0; --leg) {
                auto const leg_u = static_cast<std::size_t>(leg);
                auto const& st = (K == 0) ? t_state : states[leg_u];
                auto const root = find_root(st, via_locs[leg_u + 1]);
                if (root == location_idx_t::invalid()) {
                    chain_valid = false;
                    break;
                }
                if (K > 0 && leg > 0) {
                    auto const via_idx = fwd ? leg_u - 1 : K - leg_u;
                    if (!matches(tt, location_match_mode::kEquivalent, q.via_stops_[via_idx].location_, root)) {
                        chain_valid = false;
                        break;
                    }
                }
                via_locs[leg_u] = root;
            }
            if (!chain_valid) return;
            
            routing::journey j;
            
            int const start_leg = fwd ? 0 : static_cast<int>(K);
            int const end_leg = fwd ? static_cast<int>(K + 1) : -1;
            int const step = fwd ? 1 : -1;
            
            for (int leg = start_leg; leg != end_leg; leg += step) {
                auto const leg_u = static_cast<std::size_t>(leg);
                auto const& st = (K == 0) ? t_state : states[leg_u];
                auto const reach_l = via_locs[leg_u + 1];
                auto const root_l = via_locs[leg_u];
                
                routing::journey leg_j;
                reconstruct_path(st, reach_l, root_l, leg_j);
                
                for (auto&& leg_item : leg_j.legs_) {
                    j.add(std::move(leg_item));
                }
            }
            
            location_idx_t const start_loc = fwd ? via_locs[0] : dest_loc;
            location_idx_t const end_loc = fwd ? dest_loc : via_locs[0];

            offset start_match = offset{location_idx_t::invalid(), duration_t{0}, 0};
            bool has_start_match = false;
            for (auto const& start_offset : q.start_) {
                if (matches(tt, q.start_match_mode_, start_offset.target(), start_loc)) {
                    start_match = start_offset;
                    has_start_match = true;
                    break;
                }
            }
            if (!has_start_match) {
                auto it = q.td_start_.find(start_loc);
                if (it != q.td_start_.end()) {
                    auto const dur_opt = get_td_duration(search_dir, std::span{it->second}, cur_query_time);
                    if (dur_opt.has_value()) {
                        start_match = offset{start_loc, duration_t{dur_opt->first}, dur_opt->second.mode()};
                        has_start_match = true;
                    }
                }
            }
            
            bool use_expanded_start = false;
            auto const start_fp_it = fwd ? expanded_start_footpaths.find(start_loc) : expanded_start_footpaths.end();
            if (start_fp_it != expanded_start_footpaths.end()) {
                if (!has_start_match || start_fp_it->second.combined_dur_ <= start_match.duration()) {
                    use_expanded_start = true;
                }
            }

            if (use_expanded_start) {
                auto const& info = start_fp_it->second;
                auto const core_dep = j.legs_.empty() ? cur_query_time : j.legs_.front().dep_time_;
                auto const walk_dur = std::chrono::minutes{info.fp_dur_.count()};
                auto const walk_dep = core_dep - walk_dur;
                j.legs_.insert(j.legs_.begin(), journey::leg{
                    direction::kForward, info.root_loc_, start_loc,
                    walk_dep, core_dep, footpath{start_loc, info.fp_dur_}
                });
            } else if (has_start_match) {
                bool is_q_start = utl::any_of(q.start_, [&](offset const& o) { return matches(tt, q.start_match_mode_, o.target(), start_loc); });
                if (q.start_match_mode_ == location_match_mode::kIntermodal || (!is_q_start && start_match.duration().count() > 0)) {
                    auto const dep_time = j.legs_.empty() ? cur_query_time : j.legs_.front().dep_time_;
                    auto const core_dep = dep_time;
                    auto const start_dep = core_dep - std::chrono::minutes{start_match.duration().count()};
                    j.legs_.insert(j.legs_.begin(), journey::leg{
                        direction::kForward, get_special_station(special_station::kStart), start_loc,
                        start_dep, core_dep, start_match
                    });
                }
            }
            
            bool is_q_dest = utl::any_of(q.destination_, [&](offset const& o) { return matches(tt, q.dest_match_mode_, o.target(), end_loc); });
            
            if (q.dest_match_mode_ == location_match_mode::kIntermodal || (!is_q_dest && dest_offset.duration().count() > 0)) {
                auto const core_arr = j.legs_.empty() ? cur_query_time : j.legs_.back().arr_time_;
                auto const final_arr = core_arr + std::chrono::minutes{dest_offset.duration().count()};
                j.legs_.push_back(journey::leg{
                    direction::kForward, end_loc, get_special_station(special_station::kEnd),
                    core_arr, final_arr, dest_offset
                });
            }
            
            if (fwd) {
                j.start_time_ = j.legs_.empty() ? cur_query_time : j.legs_.front().dep_time_;
                j.dest_time_ = j.legs_.empty() ? cur_query_time : j.legs_.back().arr_time_;
                optimize_footpaths(tt, rtt, q, j);
            } else {
                j.start_time_ = j.legs_.empty() ? cur_query_time : j.legs_.back().arr_time_;
                j.dest_time_ = j.legs_.empty() ? cur_query_time : j.legs_.front().dep_time_;
                optimize_footpaths(tt, rtt, q, j);
            }
            std::uint8_t num_routes = 0;
            for (auto const& leg : j.legs_) {
                if (std::holds_alternative<journey::run_enter_exit>(leg.uses_)) {
                    num_routes++;
                }
            }
            j.transfers_ = num_routes > 0 ? num_routes - 1 : 0;
            
            if (j.transfers_ <= q.max_transfers_) {
                s_state.results_.add(std::move(j));
            }
        };

        delta_t min_dest_arr = invalid_delta;
        if (!is_interval_query) {
            for (auto const& dest_offset : q.destination_) {
                for_each_meta(tt, q.dest_match_mode_, dest_offset.target(), [&](location_idx_t const dest_loc) {
                    if (cista::to_idx(dest_loc) < state_final.node_labels_.size()) {
                        auto const& lbl = state_final.node_labels_[cista::to_idx(dest_loc)];
                        if (lbl.time_stamp_ == state_final.current_time_stamp_ && lbl.arrival_time_ != invalid_delta) {
                            auto const tot_arr = clamp(lbl.arrival_time_ + (fwd ? dest_offset.duration().count() : -dest_offset.duration().count()));
                            min_dest_arr = (min_dest_arr == invalid_delta)
                                ? tot_arr
                                : (fwd ? std::min(min_dest_arr, tot_arr) : std::max(min_dest_arr, tot_arr));
                        }
                    }
                });
            }
            for (auto const& [dest_loc, td_offsets] : q.td_dest_) {
                if (cista::to_idx(dest_loc) < state_final.node_labels_.size()) {
                    auto const& lbl = state_final.node_labels_[cista::to_idx(dest_loc)];
                    if (lbl.time_stamp_ == state_final.current_time_stamp_ && lbl.arrival_time_ != invalid_delta) {
                        auto const core_arrival = cur_query_time + std::chrono::minutes{lbl.arrival_time_ - cur_query_delta};
                        auto const dur_opt = get_td_duration(fwd ? direction::kBackward : direction::kForward, std::span{td_offsets}, core_arrival);
                        if (dur_opt.has_value()) {
                            auto const tot_arr = clamp(lbl.arrival_time_ + (fwd ? dur_opt->first.count() : -dur_opt->first.count()));
                            min_dest_arr = (min_dest_arr == invalid_delta)
                                ? tot_arr
                                : (fwd ? std::min(min_dest_arr, tot_arr) : std::max(min_dest_arr, tot_arr));
                        }
                    }
                }
            }
        }

        auto const should_add_dest = [&](location_idx_t const dest_loc, duration_t const offset_dur) {
            if (cista::to_idx(dest_loc) >= state_final.node_labels_.size()) return false;
            auto const& lbl = state_final.node_labels_[cista::to_idx(dest_loc)];
            if (lbl.time_stamp_ != state_final.current_time_stamp_ || lbl.arrival_time_ == invalid_delta) return false;

            if (lbl.reached_by_walking_) {
                auto const parent_l = lbl.parent_;
                if (parent_l != location_idx_t::invalid() && cista::to_idx(parent_l) < state_final.node_labels_.size()) {
                    auto const& parent_lbl = state_final.node_labels_[cista::to_idx(parent_l)];
                    if (parent_lbl.time_stamp_ == state_final.current_time_stamp_ && parent_lbl.arrival_time_ == lbl.arrival_time_) {
                        return false;
                    }
                }
            }

            if (!is_interval_query && min_dest_arr != invalid_delta) {
                auto const tot_arr = clamp(lbl.arrival_time_ + (fwd ? offset_dur.count() : -offset_dur.count()));
                if (fwd && tot_arr > min_dest_arr) return false;
                if (!fwd && tot_arr < min_dest_arr) return false;
            }
            return true;
        };

        for (auto const& dest_offset : q.destination_) {
            for_each_meta(tt, q.dest_match_mode_, dest_offset.target(), [&](location_idx_t const dest_loc) {
                if (should_add_dest(dest_loc, dest_offset.duration())) {
                    add_journey_for_dest(dest_loc, dest_offset);
                }
            });
        }
        for (auto const& [dest_loc, td_offsets] : q.td_dest_) {
            if (cista::to_idx(dest_loc) < state_final.node_labels_.size()) {
                auto const& dest_label = state_final.node_labels_[cista::to_idx(dest_loc)];
                if (dest_label.time_stamp_ == state_final.current_time_stamp_ && dest_label.arrival_time_ != invalid_delta) {
                    auto const core_arrival = cur_query_time + std::chrono::minutes{dest_label.arrival_time_ - cur_query_delta};
                    auto const dur_opt = get_td_duration(fwd ? direction::kBackward : direction::kForward, std::span{td_offsets}, core_arrival);
                    if (dur_opt.has_value() && should_add_dest(dest_loc, duration_t{dur_opt->first})) {
                        add_journey_for_dest(dest_loc, offset{dest_loc, dur_opt->first, dur_opt->second.mode()});
                    }
                }
            }
        }
    };

    if (is_interval_query) {
        s_state.starts_.clear();
        get_starts(search_dir, tt, rtt, q.start_time_, q.start_, q.td_start_,
                   q.via_stops_, q.max_start_offset_, q.start_match_mode_,
                   q.use_start_footpaths_, s_state.starts_, true, q.prf_idx_,
                   q.transfer_time_settings_);

        if (s_state.starts_.empty()) {
            run_search_and_reconstruct(starts, query_time, search_base_day, search_base_date, query_delta);
        } else {
            std::sort(begin(s_state.starts_), end(s_state.starts_),
                      [&](start const& a, start const& b) { return fwd ? a < b : b < a; });

            auto const target_connections = std::max(1U, q.min_connection_count_);

            bool stop_interval_search = false;
            utl::equal_ranges_linear(
                s_state.starts_,
                [](start const& a, start const& b) {
                  return a.time_at_start_ == b.time_at_start_;
                },
                [&](auto&& from_it, auto&& to_it) {
                    if (stop_interval_search) {
                        return;
                    }
                    if (!s_state.results_.empty() && (!is_interval_query || s_state.results_.size() >= target_connections)) {
                        stop_interval_search = true;
                        return;
                    }
                    auto const batch_query_time = from_it->time_at_start_;
                    auto const batch_search_base_day = tt.day_idx(std::chrono::time_point_cast<date::days>(batch_query_time));
                    auto const batch_search_base_date = tt.internal_interval_days().from_ + batch_search_base_day.v_ * date::days{1};
                    auto const batch_query_delta = unix_to_delta(batch_search_base_date, batch_query_time);

                    std::vector<std::pair<location_idx_t, delta_t>> batch_starts;
                    for (auto const& s : it_range{from_it, to_it}) {
                        batch_starts.emplace_back(s.stop_, unix_to_delta(batch_search_base_date, s.time_at_stop_));
                    }
                    run_search_and_reconstruct(batch_starts, batch_query_time, batch_search_base_day, batch_search_base_date, batch_query_delta);
                });
        }
    } else {
        run_search_and_reconstruct(starts, query_time, search_base_day, search_base_date, query_delta);
    }

    auto const effective_target = (is_interval_query && q.min_connection_count_ > 1) ? q.min_connection_count_ : 1U;
    if (s_state.results_.size() > effective_target) {
        auto min_arr = std::min_element(begin(s_state.results_.els_), end(s_state.results_.els_),
            [](journey const& a, journey const& b) { return a.arrival_time() < b.arrival_time(); })->arrival_time();
        utl::erase_if(s_state.results_.els_, [&](journey const& j) { return j.arrival_time() > min_arr; });
        if (s_state.results_.els_.size() > effective_target) {
            s_state.results_.els_.resize(effective_target);
        }
    }

    utl::erase_duplicates(
        s_state.results_.els_,
        [](journey const& a, journey const& b) {
            return std::tie(a.start_time_, a.dest_time_, a.transfers_, a.legs_) <
                   std::tie(b.start_time_, b.dest_time_, b.transfers_, b.legs_);
        },
        [](journey const& a, journey const& b) {
            return a.start_time_ == b.start_time_ &&
                   a.dest_time_ == b.dest_time_ &&
                   a.transfers_ == b.transfers_ &&
                   a.legs_ == b.legs_;
        });

    auto execution_duration = std::chrono::steady_clock::now() - processing_start_time;

    search_stats search_stats_out;
    search_stats_out.execute_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(execution_duration);
    
    bool const using_shortcut_provider = using_shortcuts;

    std::map<std::string, std::uint64_t> algo_stats_out = {
        {"tad_settled_stop_labels", total_stats.settled_stop_labels_},
        {"tad_scanned_routes", total_stats.scanned_routes_},
        {"tad_scanned_stop_events", total_stats.scanned_stop_events_},
        {"tad_relaxed_walking_edges", total_stats.relaxed_walking_edges_},
        {"tad_relaxed_transfers", total_stats.relaxed_transfers_},
        {"tad_pruned_scans", total_stats.pruned_scans_},
        {"tad_queue_updates", total_stats.queue_updates_},
        {"tad_time_pq_pop_us", total_stats.time_pq_pop_us_},
        {"tad_time_footpaths_us", total_stats.time_footpaths_us_},
        {"tad_time_routes_us", total_stats.time_routes_us_},
        {"tad_time_rt_transports_us", total_stats.time_rt_transports_us_},
        {"tad_using_shortcut_provider", using_shortcut_provider ? 1ULL : 0ULL},
        {"tad_engine_used", 1ULL}
    };

    interval<unixtime_t> out_interval{query_time, query_time};

    return {&s_state.results_, out_interval, search_stats_out, algo_stats_out};
}

}  // namespace nigiri::routing
