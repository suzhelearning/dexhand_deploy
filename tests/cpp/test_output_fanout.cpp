#include "../../native/control/output_fanout.hpp"
#include <cassert>
#include <condition_variable>
#include <chrono>
#include <thread>
#include <atomic>
using namespace tianji_control;
int main() {
  std::mutex mutex;std::condition_variable wake;
  bool entered=false,released=false;
  std::atomic<int> fast{0},slow{0},failures{0};
  auto release=[&]{std::lock_guard<std::mutex> lock(mutex);released=true;wake.notify_all();};
  auto block=[&](const int&) {
    std::unique_lock<std::mutex> lock(mutex);entered=true;wake.notify_all();
    wake.wait(lock,[&]{return released;});++slow;
  };
  {
    OutputFanout<int> out(4,{{"slow",block,release},
                            {"fast",[&](const int&){++fast;wake.notify_all();},[]{}}},
                           [&](const std::string&){++failures;});
    assert(out.submit(1));
    std::unique_lock<std::mutex> lock(mutex);
    assert(wake.wait_for(lock,std::chrono::seconds(2),[&]{return entered && fast==1;}));
    assert(slow==0);lock.unlock();
    release();assert(out.finish());
    assert(slow==1 && fast==1 && failures==0);
    for(const auto& row:out.stats()) assert(row.second.complete && row.second.accepted==row.second.processed);
    assert(!out.submit(2));
  }
  entered=false;released=false;
  {
    OutputFanout<int> out(1,{{"blocked",block,release}},[&](const std::string&){++failures;});
    assert(out.submit(1));
    std::unique_lock<std::mutex> lock(mutex);
    assert(wake.wait_for(lock,std::chrono::seconds(2),[&]{return entered;}));lock.unlock();
    assert(out.submit(2));assert(!out.submit(3));
    assert(out.failure().find("blocked")!=std::string::npos);
    out.abort();assert(!out.finish());
    assert(!out.stats()[0].second.complete);
  }
  {
    OutputFanout<int> out(2,{{"failed",[](const int&){throw std::runtime_error("last item failed");},[]{}}},
                           [&](const std::string&){++failures;});
    assert(out.submit(1));assert(!out.finish());
    assert(out.failure().find("last item failed")!=std::string::npos);
  }
  assert(failures==2);
}
