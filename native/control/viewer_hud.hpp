#pragma once
#include "session_runtime.hpp"
#include <sstream>
#include <iomanip>

namespace tianji_control {
// Display only: never grants start authority or changes calibration state.
inline std::string viewer_hud(const SessionSnapshot& s,bool xz) {
  std::ostringstream out;
  out<<"Session: "<<s.state.phase<<" | S: start  H: Home  R: rearm  Q: exit\n";
  if(s.state.phase=="fault") {out<<"FAULT: "<<s.state.reason<<"\nRestart required";return out.str();}
  if(s.state.phase=="teleop") {out<<"TELEOP ACTIVE | H: Home before recalibration";return out.str();}
  if(s.reset_pending) {out<<"Waiting for reset / calibration ACK. Do not press S.";return out.str();}
  if(s.state.phase!="idle") {out<<s.state.reason<<"\nWait for Home.";return out.str();}
  if(!s.height) {out<<"Press S when input is fresh.";return out.str();}
  const auto& h=*s.height;
  out<<"Calibration: "<<(xz?"X/Z (reference J2=-90 deg)":"Z-only")<<"\n";
  if(h.state=="collecting") {
    out<<"SAMPLING: hold arms "<<(xz?"forward and horizontal":"horizontal")<<" for 2 seconds.\n"
       <<"Samples L / R: "<<h.count[0]<<" / "<<h.count[1]<<" | Do not press S.";
  } else if(h.state=="sampled") out<<"Waiting for reset / calibration ACK. Do not press S.";
  else {
    if(h.state=="failed") out<<"FAILED: "<<h.error<<"\n";
    if(h.offsets) {
      out<<(h.state=="failed"?"Previous calibration retained.":"SUCCESS.")
         <<" Press S when input is fresh; C to recalibrate.\n";
      out<<std::fixed<<std::setprecision(3)<<"Offset Z L/R (m): "<<(*h.offsets)[0]<<" / "<<(*h.offsets)[1];
      if(h.x_offsets) out<<" | X: "<<(*h.x_offsets)[0]<<" / "<<(*h.x_offsets)[1];
    } else out<<"Press C, hold arms "<<(xz?"forward and horizontal":"horizontal")<<" steadily. Then wait for SUCCESS.";
  }
  return out.str();
}
}
