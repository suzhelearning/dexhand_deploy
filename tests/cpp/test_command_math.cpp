#include "../../native/control/command_math.hpp"
#include <cassert>
#include <limits>

int main() {
  using namespace tianji_control;
  Joints7 zero{}, q{}, lo{}, hi{};
  q.fill(.3); lo.fill(-1); hi.fill(1);
  CommandConfig cfg{.1,.1,200.,.2};
  assert(validate_command(q,zero,lo,hi,cfg,100,100,std::nullopt,false,true).error == CommandError::maximum_step);
  assert(validate_command(q,zero,lo,hi,cfg,100,100,std::nullopt,true,true).error == CommandError::ok);
  assert(validate_command(q,zero,lo,hi,cfg,100,100,std::nullopt,true,false).error == CommandError::tracking_hold_real);
  auto end = home_command(q,zero,10.,.5,.7);
  assert(end == zero);
  auto clipped = track_command(q,zero,.1,true,false);
  assert(clipped[6] == .1);
  const auto max = std::numeric_limits<std::int64_t>::max();
  assert(validate_command(zero,zero,lo,hi,cfg,max,0,max,false,true).error == CommandError::timestamp_rollback);
  assert(validate_command(zero,zero,lo,hi,cfg,0,max,0,false,true).error == CommandError::source_stale);
}
