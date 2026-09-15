#include "../../native/control/viewer_window.hpp"
#include <cassert>
using namespace tianji_control;
int main(int argc,char** argv) {
  ViewerState display(true);
  display.ingest(SessionReply{7,{false,"complete height calibration before start"},"start",1});
  assert(display.snapshot().operator_report.find("REJECTED")!=std::string::npos);
  assert(display.snapshot().operator_report.find("complete height calibration")!=std::string::npos);
  display.ingest(SessionReply{8,{true,"accepted"},"height_commit",2});
  assert(display.snapshot().operator_report.find("height_commit OK")!=std::string::npos);
  assert(viewer_key('S',1,false)==GatewayAction::start);
  assert(viewer_key('H',1,false)==GatewayAction::return_home);
  assert(viewer_key('R',1,false)==GatewayAction::rearm);
  assert(viewer_key('Q',1,false)==GatewayAction::shutdown);
  assert(!viewer_key('C',1,false));
  assert(viewer_key('C',1,true)==GatewayAction::calibrate);
  assert(!viewer_key('S',0,true) && !viewer_key('S',2,true));
  assert(!viewer_key('X',1,true));
  if(argc==2) {
    bool failed=false;
    try{ViewerWindow window(argv[1],false,[](auto){});}catch(const std::runtime_error&){failed=true;}
    assert(failed); // This fixture runs with DISPLAY removed, never opens a live window.
  }
}
