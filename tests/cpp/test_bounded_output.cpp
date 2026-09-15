#include "../../native/control/bounded_output.hpp"
#include <cassert>
#include <condition_variable>
#include <vector>
using namespace tianji_control;
template<class F> void until(F predicate) {
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(!predicate()) {
    assert(std::chrono::steady_clock::now()<deadline);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
int main() {
  std::vector<int> values; bool finalized=false;
  BoundedOutput<int> output(32,[&](const int& value){values.push_back(value);},
    [&]{finalized=true;},[]{});
  for(int i=0;i<20;++i) assert(output.submit(i));
  output.finish(); output.finish();
  assert(finalized && values.size()==20);
  for(int i=0;i<20;++i) assert(values[i]==i);
  auto stats=output.stats();
  assert(stats.complete && stats.accepted==20 && stats.processed==20 && !stats.pending);
  assert(!output.submit(30));

  // A blocked consumer cannot block admission or silently drop overflow.
  std::mutex mutex; std::condition_variable wake; bool release=false,entered=false;
  auto cancel=[&]{std::lock_guard<std::mutex> lock(mutex); release=true; wake.notify_all();};
  finalized=false;
  BoundedOutput<int> slow(1,[&](const int&) {
    std::unique_lock<std::mutex> lock(mutex); entered=true; wake.notify_all();
    wake.wait(lock,[&]{return release;});
  },[&]{finalized=true;},cancel);
  assert(slow.submit(1));
  { std::unique_lock<std::mutex> lock(mutex); wake.wait(lock,[&]{return entered;}); }
  assert(slow.submit(2)); assert(!slow.submit(3));
  assert(!slow.stats().failure.empty());
  slow.abort();
  assert(!finalized && !slow.stats().complete);
  assert(slow.stats().accepted==2 && slow.stats().processed<=1);

  for(bool finish_failure:{false,true}) {
    BoundedOutput<int> failed(4,[finish_failure](const int&) {
      if(!finish_failure) throw std::runtime_error("write failed");
    },[finish_failure]{if(finish_failure) throw std::runtime_error("close failed");},[]{});
    assert(failed.submit(1)); failed.finish();
    assert(!failed.stats().complete && !failed.stats().failure.empty());
    assert(!failed.submit(2));
  }
  finalized=false;
  { BoundedOutput<int> abandoned(4,[](const int&){},[&]{finalized=true;},[]{}); }
  assert(!finalized); // Destructor is abort, not a successful recording close.
  // Abort must cancel a writer even when another thread is waiting in finish.
  release=false; entered=false;
  BoundedOutput<int> concurrent(4,[&](const int&) {
    std::unique_lock<std::mutex> lock(mutex); entered=true; wake.notify_all();
    wake.wait(lock,[&]{return release;});
  },[]{},cancel);
  assert(concurrent.submit(1));
  {std::unique_lock<std::mutex> lock(mutex); wake.wait(lock,[&]{return entered;});}
  std::thread finisher([&]{concurrent.finish();});
  concurrent.abort(); finisher.join();
  assert(!concurrent.stats().complete);
  bool invalid=false;
  try { BoundedOutput<int> bad(0,[](const int&){},[]{},[]{}); }
  catch(const std::invalid_argument&) {invalid=true;}
  assert(invalid);
}
