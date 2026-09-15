#include "../../native/control/height_calibration.hpp"
#include <cassert>
#include <cmath>
using namespace tianji_control;
int main() {
  HeightCalibration common({1.1,1.2},std::array<double,2>{.73,.70},true);
  common.begin(1000000000);
  for(int i=0;i<=100;++i) {
    RawProgress p;p.accepted=true;p.skeleton_valid=true;p.rotations_valid=true;
    p.epoch=1;p.sequence=i+1;p.palms={{{.5,.2,1.4},{.55,-.2,1.45}}};
    auto now=1000000000LL+i*20000000LL;common.add(p,now,now);common.tick(now);
  }
  assert(common.candidate_x());
  assert(std::abs(.5+(*common.candidate_x())[0]-.70)<1e-10);
  assert(std::abs(.55+(*common.candidate_x())[1]-.70)<1e-10);
  HeightCalibration c({1.121,1.121},std::array<double,2>{.7325,.7325});
  c.begin(1000000000);
  for(int i=0;i<=100;++i) {
    RawProgress p; p.accepted=true;p.skeleton_valid=true;p.rotations_valid=true;
    p.epoch=1;p.generation=1;p.sequence=i+1;
    p.palms={{{.50,.2,1.4},{.55,-.2,1.45}}};
    auto now=1000000000LL+i*20000000LL;c.add(p,now,now);c.tick(now);
  }
  assert(c.candidate() && c.candidate_x());
  assert(std::abs((*c.candidate_x())[0]-.2325)<1e-10);
  assert(std::abs((*c.candidate())[1]+.329)<1e-10);
  c.commit();assert(c.ready() && c.status().x_offsets);
  c.begin(4000000000LL);c.tick(4300000000LL);
  assert(c.status().state=="failed" && c.status().x_offsets && c.ready());
}
