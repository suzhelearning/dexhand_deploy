#include "../../native/control/execution_guard.hpp"
#include <cassert>
#include <limits>

int main() {
  using tianji_control::ExecutionGuard;
  tianji_control::Positions q{};
  ExecutionGuard accepted(100,2);
  accepted.register_tick(1,100,q);
  assert(accepted.check(120));
  assert(accepted.observe(1,true,"accepted",q));
  assert(accepted.in_flight()==0);
  assert(!accepted.observe(1,true,"duplicate",q));

  ExecutionGuard timeout(100,2);
  timeout.register_tick(1,100,q);
  timeout.register_tick(2,101,q);
  assert(timeout.check(200));
  assert(!timeout.check(201));
  assert(timeout.reason()=="coordinator receipt timeout");
  assert(timeout.in_flight()==0);
  timeout.pause("later fault");
  assert(timeout.reason()=="coordinator receipt timeout");

  ExecutionGuard changed(100,2);
  changed.register_tick(1,100,q);
  q[13]=.001;
  assert(!changed.observe(1,true,"accepted",q));
  assert(changed.reason()=="reference_direct command was modified downstream");

  const auto maximum=std::numeric_limits<std::int64_t>::max();
  ExecutionGuard boundary(maximum,1);
  boundary.register_tick(1,1,q);
  assert(boundary.check(maximum));
  assert(!boundary.check(1));
  assert(boundary.reason()=="execution clock rollback");
  bool threw=false;
  try { boundary.validate_tick(maximum); }
  catch (const std::invalid_argument&) { threw=true; }
  assert(threw);
}
