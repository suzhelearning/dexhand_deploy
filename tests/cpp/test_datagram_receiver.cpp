#include "../../native/control/datagram_receiver.hpp"
#include <cassert>
#include <condition_variable>
#include <mutex>
#include <sys/socket.h>
#include <fcntl.h>
using namespace tianji_control;
template<class F> void until(F predicate) {
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(!predicate()) {
    assert(std::chrono::steady_clock::now()<deadline);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
int main() {
  int pair[2]; assert(socketpair(AF_UNIX,SOCK_DGRAM|SOCK_CLOEXEC,0,pair)==0);
  std::mutex mutex;
  std::vector<ReceivedDatagram> frames;
  DatagramReceiver receiver(OwnedDescriptor(pair[0]),[&](ReceivedDatagram frame) {
    std::lock_guard<std::mutex> lock(mutex); frames.push_back(std::move(frame)); return true;
  });
  receiver.start();
  bool invalid=false;
  try { receiver.start(); } catch(const std::logic_error&) { invalid=true; }
  assert(invalid);
  const std::vector<std::uint8_t> packet(656,42),oversized(657,43),large(8192,44);
  for(const auto* data:{&packet,&oversized,&large})
    assert(send(pair[1],data->data(),data->size(),0)==static_cast<ssize_t>(data->size()));
  assert(send(pair[1],nullptr,0,0)==0);
  assert(send(pair[1],"end",3,0)==3);
  until([&]{const auto s=receiver.stats(); return s.datagrams==5 && s.delivered==2;});
  receiver.stop(); receiver.stop();
  const auto stats=receiver.stats();
  assert(stats.delivered==2 && stats.rejected_size==3 && stats.failure.empty() && !stats.running);
  assert(frames.size()==2 && frames[0].bytes==packet && frames[1].bytes==std::vector<std::uint8_t>({'e','n','d'}));
  assert(frames[0].received_ns>0 && frames[1].received_ns>=frames[0].received_ns);
  // The receiver releases only its transferred descriptor; never the sender.
  assert(fcntl(pair[0],F_GETFD)==-1 && errno==EBADF);
  assert(fcntl(pair[1],F_GETFD)>=0); close(pair[1]);

  for(bool throws:{false,true}) {
    assert(socketpair(AF_UNIX,SOCK_DGRAM|SOCK_CLOEXEC,0,pair)==0);
    std::string reported;
    DatagramReceiver failed(OwnedDescriptor(pair[0]),[throws](ReceivedDatagram) -> bool {
      if(throws) throw std::runtime_error("sink failed");
      return false;
    },[&](const std::string& error){reported=error;});
    failed.start(); assert(send(pair[1],"x",1,0)==1);
    until([&]{return !failed.stats().failure.empty();});
    failed.stop();
    assert(failed.stats().delivered==0 && !failed.stats().running);
    assert(reported==failed.stats().failure && !failed.stats().notification_failed);
    close(pair[1]);
  }
  assert(socketpair(AF_UNIX,SOCK_DGRAM|SOCK_CLOEXEC,0,pair)==0);
  DatagramReceiver idle(OwnedDescriptor(pair[0]),[](ReceivedDatagram){return true;});
  idle.start();
  const auto start=std::chrono::steady_clock::now(); idle.stop();
  assert(std::chrono::steady_clock::now()-start<std::chrono::seconds(1)); close(pair[1]);
  assert(socketpair(AF_UNIX,SOCK_STREAM|SOCK_CLOEXEC,0,pair)==0);
  invalid=false;
  try { DatagramReceiver bad(OwnedDescriptor(pair[0]),[](ReceivedDatagram){return true;}); }
  catch(const std::invalid_argument&) { invalid=true; }
  assert(invalid && fcntl(pair[0],F_GETFD)==-1); close(pair[1]);
  assert(socketpair(AF_UNIX,SOCK_DGRAM|SOCK_CLOEXEC,0,pair)==0);
  DatagramReceiver broken_notifier(OwnedDescriptor(pair[0]),[](ReceivedDatagram){return false;},
    [](const std::string&){throw std::runtime_error("report failed");});
  broken_notifier.start(); assert(send(pair[1],"x",1,0)==1);
  until([&]{return broken_notifier.stats().notification_failed;});
  broken_notifier.stop();
  assert(broken_notifier.stats().failure=="datagram sink rejected admission"); close(pair[1]);
  assert(socketpair(AF_UNIX,SOCK_DGRAM|SOCK_CLOEXEC,0,pair)==0);
  DatagramReceiver unopened(OwnedDescriptor(pair[0]),[](ReceivedDatagram){return true;});
  unopened.stop(); invalid=false;
  try {unopened.start();} catch(const std::logic_error&) {invalid=true;}
  assert(invalid && fcntl(pair[0],F_GETFD)==-1); close(pair[1]);
}
