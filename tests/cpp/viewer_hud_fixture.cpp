#include "../../native/control/viewer_hud.hpp"
#include <stdexcept>
using namespace tianji_control;
int main() {
  SessionSnapshot s; s.height=HeightCalibration::Status{};
  auto require=[&](const char* text){if(viewer_hud(s,true).find(text)==std::string::npos) throw std::runtime_error(text);};
  require("Press C");
  s.height->state="collecting";s.height->count={50,48};require("Do not press S");require("50 / 48");
  s.height->state="sampled";s.reset_pending=true;require("Waiting for reset");
  s.reset_pending=false;s.height->state="calibrated";s.height->offsets=std::array<double,2>{-.1,-.2};
  require("SUCCESS");require("Press S");
  s.height->state="failed";s.height->error="tracking identity/epoch changed during calibration";
  require("FAILED");require("epoch changed");require("Previous calibration retained");
  s.height->offsets.reset();require("Press C");
  s.state.phase="teleop";require("H: Home");
  s.state.phase="fault";s.state.reason="injected fault";require("injected fault");require("Restart");
}
