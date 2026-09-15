#include "../../native/hand/scheduler.hpp"
#include "../../native/hand/scheduler_wire.hpp"
#include <future>
#include <iostream>
using namespace tianji_hand;
#define CHECK(x) do {if(!(x)) throw std::runtime_error(#x);} while(false)
int main() {
 try {
  std::promise<void> entered,release;auto released=release.get_future().share();int resets=0;
  auto solve=[](const double*,std::uint64_t,double*) {};
  HandScheduler scheduler({},solve,solve,[&]{++resets;entered.set_value();released.wait();},[&]{++resets;});
  CHECK(scheduler.submit_session({1,1,10,HandPhase::idle}));
  const HandSession request{2,2,20,HandPhase::idle};
  CHECK(scheduler.submit_session(request));
  auto tick=std::async(std::launch::async,[&]{return scheduler.tick(20);});
  entered.get_future().wait();
  const auto early=scheduler.wait_reset_ack(request,std::chrono::milliseconds(0));
  release.set_value();tick.get();CHECK(!early);
  const auto ack=scheduler.wait_reset_ack(request,std::chrono::milliseconds(1));
  CHECK(ack && resets==2 && ack->epoch==2 && ack->sequence==2 && ack->requested_ns==20 && ack->completed_ns>=20);
  const auto frame=scheduler_wire::encode_reset_ack(*ack);
  const auto decoded=scheduler_wire::decode_reset_ack(frame.data(),frame.size());
  CHECK(decoded.epoch==2 && decoded.sequence==2 && decoded.completed_ns==ack->completed_ns);
  const auto wire=scheduler_wire::encode_reset_request(request);
  CHECK(scheduler_wire::decode_reset_request(wire.data(),wire.size()).sequence==2);
  auto malformed=frame;malformed[5]=1;
  bool rejected=false;try {scheduler_wire::decode_reset_ack(malformed.data(),malformed.size());} catch(const std::exception&) {rejected=true;}
  CHECK(rejected);
  CHECK(!scheduler.wait_reset_ack({2,3,20,HandPhase::idle},std::chrono::milliseconds(0)));
  HandScheduler failed({},solve,solve,[]{throw std::runtime_error("reset failure");},[]{});
  CHECK(failed.submit_session(request));CHECK(!failed.tick(20));
  CHECK(!failed.wait_reset_ack(request,std::chrono::milliseconds(1)));
  scheduler.request_stop();CHECK(!scheduler.wait_reset_ack(request,std::chrono::milliseconds(1)));
  // The reader can admit a frame after the owner's tick clock was sampled,
  // including while reset callbacks run without the state mutex.
  for(const auto phase:{HandPhase::idle,HandPhase::teleop}) {
    int solves=0;
    auto count=[&](const double*,std::uint64_t,double*){++solves;};
    HandScheduler delayed({},count,count);
    CHECK(delayed.submit_session({1,1,100,phase}));
    HandInput input;input.sequence=1;input.timestamp_ns=101;input.generation=1;input.flags=3;
    CHECK(delayed.submit_input(input));
    CHECK(!delayed.tick(100));
    HandCommand output;CHECK(!delayed.pop(output));CHECK(solves==0);
    CHECK(delayed.tick(101));CHECK(delayed.pop(output));CHECK(solves==2);
    CHECK(output.input_sequence==1 && output.scheduler_timestamp_ns>=output.input_timestamp_ns);
    CHECK(output.status==(phase==HandPhase::idle ? HandOutputStatus::processed : HandOutputStatus::command));
  }
  std::cout<<"hand reset ACK barriers passed\n";
 } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
