#include "../../native/control/mujoco_endpoint.hpp"
#include <nlohmann/json.hpp>
#include <iostream>
#include <limits>
using namespace tianji_control;
int main(int argc,char** argv) {
  if(argc!=2) return 2;
  try {
    MujocoEndpoint endpoint(argv[1],true);
    ArmPair arms{}; arms[0].fill(.1); arms[1].fill(-.2);
    HandUpdates hands;
    hands[0]=HandJoints{}; hands[1]=HandJoints{};
    for(int s=0;s<2;++s) for(int j=0;j<20;++j) (*hands[s])[j]=.01*(1+j+20*s);
    SimulationEndpoint& owned=endpoint;
    owned.apply_frame(arms,hands);
    const auto expected=owned.hand_feedback();
    const auto height=owned.height_reference();
    MujocoEndpoint reference(argv[1]);
    if(height!=reference.height_reference() || owned.hand_feedback()!=expected) return 11;
    if(!expected || (*expected)[0]!=*hands[0] || (*expected)[1]!=*hands[1]) return 3;
    auto bad=hands; (*bad[1])[19]=std::numeric_limits<double>::quiet_NaN();
    auto changed=arms; changed[0][0]=.4;
    int rejected=0;
    try { owned.apply_frame(changed,bad); } catch(const std::invalid_argument&) { ++rejected; }
    if(owned.feedback()!=arms || owned.hand_feedback()!=expected) return 4;
    auto bad_arms=changed; bad_arms[1][6]=std::numeric_limits<double>::infinity();
    try { owned.apply_frame(bad_arms,hands); return 12; } catch(const std::invalid_argument&) {}
    if(owned.feedback()!=arms || owned.hand_feedback()!=expected) return 13;
    std::thread other([&] {
      try { owned.apply_frame(changed,hands); } catch(const std::logic_error&) { ++rejected; }
      try { owned.hand_feedback(); } catch(const std::logic_error&) { ++rejected; }
    }); other.join();
    if(rejected!=3 || owned.feedback()!=arms || owned.hand_feedback()!=expected) return 5;
    HandUpdates left_only; left_only[0]=HandJoints{}; left_only[0]->fill(.6);
    owned.apply_frame(changed,left_only);
    auto after=owned.hand_feedback();
    if(!after || (*after)[0]!=*left_only[0] || (*after)[1]!=(*expected)[1]) return 6;
    owned.apply(arms); // Existing arm-only caller holds hand state.
    if(owned.hand_feedback()!=after || owned.feedback()!=arms) return 7;
    MujocoEndpoint disabled(argv[1]);
    SimulationEndpoint& old=disabled;
    const auto before=old.feedback();
    try { old.apply_frame(changed,hands); return 8; } catch(const std::invalid_argument&) {}
    if(old.hand_feedback() || old.feedback()!=before) return 9;
    old.apply_frame(arms,{});
    if(old.feedback()!=arms) return 10;
    std::cout<<nlohmann::json{{"arms",owned.feedback()},{"hands",*after}}<<'\n';
  } catch(const std::exception& error) { std::cerr<<error.what()<<'\n';return 1; }
}
