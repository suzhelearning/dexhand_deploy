// Offline facts/reducer test protocol only: this executable cannot publish commands.
#include "../../native/control/session_machine.hpp"
#include <iomanip>
#include <iostream>

int main(int argc, char** argv) {
  if (argc != 4) return 2;
  tianji_control::SessionMachine machine(std::stoll(argv[1]), std::stoll(argv[2]), std::stoi(argv[3]) != 0);
  std::string op, arg;
  std::int64_t now;
  unsigned flags;
  std::uint64_t revision;
  while (std::cin >> op >> now >> flags >> revision >> std::quoted(arg)) {
    tianji_control::SessionFacts facts;
    facts.source_ready = flags & 1;
    facts.producer_ready = flags & 2;
    facts.executor_ready = flags & 4;
    facts.arm_fresh = flags & 8;
    facts.arm_home = flags & 16;
    facts.hand_producer_ready = flags & 32;
    facts.hand_zero = flags & 64;
    facts.hand_tracking = flags & 128;
    facts.command_home = flags & 256;
    facts.proposal_stale = flags & 512;
    facts.arm_exact_home = flags & 1024;
    facts.input_revision = revision;
    tianji_control::IntentOutcome outcome;
    if (op == "tick") machine.tick(now, facts);
    else if (op == "fault") machine.fault(now, arg);
    else if (op == "rearm") outcome = machine.rearm(now, std::stoll(arg), facts, true);
    else outcome = machine.intent(now, op, arg, true, facts);
    const auto& state = machine.state();
    std::cout << outcome.accepted << ' ' << state.phase << ' ' << state.epoch << ' '
              << state.at_home << ' ' << state.return_complete << ' ' << state.shutdown_complete << ' '
              << std::quoted(state.reason) << ' ' << std::quoted(outcome.reason) << '\n';
  }
}
