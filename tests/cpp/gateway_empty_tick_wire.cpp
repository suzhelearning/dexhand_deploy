#include "../../native/control/session_gateway.hpp"
#include <iostream>

int main() {
  using namespace tianji_control;
  for (std::uint64_t index = 1; index <= 3; ++index) {
    SessionCycleSnapshot cycle;
    cycle.timestamp_ns = 10000 + index;
    cycle.snapshot.state.phase = "teleop";
    cycle.snapshot.state.epoch = 1;
    cycle.snapshot.ticks = index;
    WorkerTick request;
    request.id = index;
    request.now_ns = cycle.timestamp_ns;
    if (index != 2) {
      request.packet = {1, 2, 3};
      request.received_ns = 9000 + index;
      request.generation = 1;
      request.source_sequence = index;
    }
    cycle.request = request;
    const auto frame = GatewayWireCodec::encode_cycle(cycle);
    std::cout.write(reinterpret_cast<const char*>(frame.data()), frame.size());
  }
}
