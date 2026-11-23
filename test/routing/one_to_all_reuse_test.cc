#include <algorithm>
#include <cstdint>
#include <string_view>

#include "gtest/gtest.h"

#include "nigiri/routing/limits.h"
#include "nigiri/routing/one_to_all.h"
#include "nigiri/routing/query.h"
#include "nigiri/routing/raptor/raptor_state.h"

#include "../transfer_rules_util.h"

using namespace nigiri;
using namespace nigiri::routing;
using namespace std::chrono_literals;

namespace {

// One state reused across queries with changing transfer limits (also down
// and up again) yields exactly what a fresh state yields, in every row; rows
// above the query's last round hold no arrival.
template <direction Dir>
void expect_reused_equals_fresh(timetable const& tt) {
  constexpr auto kInvalid = kInvalidDelta<Dir>;
  auto reused = raptor_state{};
  auto n_queries = 0U;
  auto n_arrivals = std::array<std::size_t, kMaxTransfers + 2U>{};
  for (auto const max_transfers :
       {std::uint8_t{14U}, std::uint8_t{2U}, std::uint8_t{0U},
        std::uint8_t{14U}, std::uint8_t{1U}}) {
    for (auto const hhmm : {"09:50", "10:00", "10:20", "10:40", "11:10"}) {
      for (auto l = 0U; l != tt.n_locations(); ++l) {
        auto const q = query{.start_time_ = test::at(hhmm),
                             .start_ = {{location_idx_t{l}, 0_minutes, 0U}},
                             .max_transfers_ = max_transfers};
        auto const fresh = one_to_all<Dir>(tt, nullptr, q);
        one_to_all<Dir>(tt, nullptr, q, reused);
        ++n_queries;

        auto const& a = fresh.one_to_all_arrivals_;
        auto const& b = reused.one_to_all_arrivals_;
        ASSERT_EQ(a.n_rows_, kMaxTransfers + 2U);
        ASSERT_EQ(a.n_rows_, b.n_rows_);
        ASSERT_EQ(a.n_columns_, b.n_columns_);
        EXPECT_TRUE(std::equal(begin(a.entries_), end(a.entries_),
                               begin(b.entries_), end(b.entries_)))
            << "start=" << l << " at " << hhmm
            << " max_transfers=" << static_cast<int>(max_transfers);
        auto const best_a = fresh.template get_best<0U>();
        auto const best_b = reused.template get_best<0U>();
        for (auto x = 0U; x != tt.n_locations(); ++x) {
          EXPECT_EQ(best_a[x][0], best_b[x][0]);
        }
        for (auto k = 0U; k != a.n_rows_; ++k) {
          for (auto x = 0U; x != a.n_columns_; ++x) {
            if (k >= max_transfers + 2U) {
              EXPECT_EQ(kInvalid, a[k][x]);
              EXPECT_EQ(kInvalid, b[k][x]);
            } else if (a[k][x] != kInvalid) {
              ++n_arrivals[k];
            }
          }
        }
      }
    }
  }
  EXPECT_LT(0U, n_queries);
  // The networks reach stops with and without changing trains.
  EXPECT_LT(0U, n_arrivals[1]);
  EXPECT_LT(0U, n_arrivals[2]);
}

}  // namespace

TEST(one_to_all, reused_state_equals_fresh_state) {
  auto const tt = test::load_network("S1,S1,3,,,,,\nS1,S1,2,60,RF,RH,,");
  ASSERT_LT(0U, test::n_virts(tt));
  expect_reused_equals_fresh<direction::kForward>(tt);
  expect_reused_equals_fresh<direction::kBackward>(tt);
}

TEST(one_to_all, reused_state_equals_fresh_state_without_rules) {
  auto const tt = test::load_network("");
  expect_reused_equals_fresh<direction::kForward>(tt);
  expect_reused_equals_fresh<direction::kBackward>(tt);
}
