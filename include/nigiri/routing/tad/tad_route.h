#pragma once

#include "nigiri/routing/search.h"
#include "nigiri/routing/tad/tad_state.h"
#include "nigiri/routing/query.h"
#include "nigiri/timetable.h"
#include <chrono>
#include <optional>

namespace nigiri::routing {

routing_result tad_route(
    timetable const& tt,
    rt_timetable const* rtt,
    search_state& s_state,
    tad_state& t_state,
    query q,
    direction search_dir,
    std::optional<std::chrono::seconds> timeout = std::nullopt);

}  // namespace nigiri::routing
