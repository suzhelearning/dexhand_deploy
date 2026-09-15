#include "../../native/control/owned_mujoco.hpp"
#include "../../native/control/mujoco_endpoint.hpp"
#include <cstdlib>
#include <iostream>
#include <new>
static thread_local bool counting=false;
static thread_local std::size_t allocations=0;
// Keep the paired malloc/free test instrumentation out of allocator inlining;
// GCC otherwise diagnoses the replacement delete as mismatched with new.
__attribute__((noinline)) void* operator new(std::size_t n) {
  if(counting) ++allocations;
  if(auto* p=std::malloc(n?n:1)) return p;
  throw std::bad_alloc();
}
__attribute__((noinline)) void operator delete(void* p) noexcept { std::free(p); }
__attribute__((noinline)) void operator delete(void* p,std::size_t) noexcept { std::free(p); }
int main(int argc,char** argv) {
  if(argc!=2) return 2;
  try {
    std::vector<std::vector<std::string>> names(2);
    for(int s=0;s<2;++s) for(int j=1;j<=7;++j) names[s].push_back("Joint"+std::to_string(j)+(s?"_R":"_L"));
    tianji_control::OwnedMujoco sim(argv[1],names);
    sim.apply({std::vector<double>(7,.1),std::vector<double>(7,-.2)});
    const auto expected=sim.snapshot();
    volatile double sum=0.;
    counting=true;
    for(int i=0;i<100;++i) { auto value=sim.snapshot(); sum+=value.groups[0][i%7]+value.groups[1][i%7]; }
    counting=false; const auto legacy=allocations; allocations=0;
    counting=true;
    for(int i=0;i<100;++i) {
      const auto left=sim.group_positions<7>(0),right=sim.group_positions<7>(1);
      sum+=left[i%7]+right[i%7];
      for(int j=0;j<7;++j) if(left[j]!=expected.groups[0][j] || right[j]!=expected.groups[1][j]) return 3;
    }
    counting=false; const auto fixed=allocations;
    if(!legacy || fixed || sum==0.) return 4;
    int rejected=0;
    try { sim.group_positions<7>(2); } catch(const std::invalid_argument&) { ++rejected; }
    try { sim.group_positions<6>(0); } catch(const std::invalid_argument&) { ++rejected; }
    std::thread other([&] { try { sim.group_positions<7>(0); } catch(const std::logic_error&) { ++rejected; } });
    other.join();
    if(rejected!=3 || sim.snapshot().qpos!=expected.qpos) return 5;
    tianji_control::MujocoEndpoint endpoint(argv[1]);
    tianji_control::ArmPair q{}; q[0].fill(.1); q[1].fill(-.2);
    allocations=0; counting=true;
    for(int i=0;i<100;++i) { endpoint.apply(q); if(endpoint.feedback()!=q) return 6; }
    counting=false; const auto adapter=allocations;
    if(adapter) { std::cerr<<"adapter allocations="<<adapter<<'\n'; return 7; }
    auto invalid=q; invalid[0][0]=.3; invalid[1][6]=std::numeric_limits<double>::quiet_NaN();
    rejected=0;
    try { endpoint.apply(invalid); } catch(const std::invalid_argument&) { ++rejected; }
    std::thread wrong_owner([&] { try { endpoint.apply(q); } catch(const std::logic_error&) { ++rejected; } });
    wrong_owner.join();
    if(rejected!=2 || endpoint.feedback()!=q) return 8;
    tianji_control::MujocoEndpoint with_hands(argv[1],true);
    tianji_control::HandUpdates hands;
    hands[0]=tianji_control::HandJoints{}; hands[0]->fill(.3);
    hands[1]=tianji_control::HandJoints{}; hands[1]->fill(.4);
    with_hands.apply_frame(q,hands); // Warm model internals before measured loop.
    allocations=0; counting=true;
    for(int i=0;i<100;++i) {
      with_hands.apply_frame(q,hands);
      const auto feedback=with_hands.hand_feedback();
      if(!feedback || (*feedback)[0]!=*hands[0] || (*feedback)[1]!=*hands[1]) return 9;
      if(with_hands.feedback()!=q) return 10;
    }
    counting=false; const auto mixed=allocations;
    if(mixed) { std::cerr<<"mixed allocations="<<mixed<<'\n'; return 11; }
    std::cout<<"legacy_allocations="<<legacy<<" fixed_allocations="<<fixed<<" adapter_allocations="<<adapter
      <<" mixed_allocations="<<mixed<<'\n';
  } catch(const std::exception& e) { counting=false; std::cerr<<e.what()<<'\n'; return 1; }
}
