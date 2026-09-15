#include "../../native/control/session_runtime.hpp"
#include "../../native/control/mujoco_endpoint.hpp"
#include "../../native/control/worker_ik_endpoint.hpp"
#include "../../native/control/tjvr_input.hpp"
#include <iostream>
#include <atomic>
#include <cstdlib>
using namespace tianji_control;
static std::int64_t time_ns() { return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
int main(int argc,char** argv) {
  if(argc<8) return 2;
  try {
    std::vector<std::string> command;
    for(int i=4;i<argc;++i) command.emplace_back(argv[i]);
    std::vector<std::vector<std::uint8_t>> packets;
    for(std::string hex;std::cin>>hex;) {
      std::vector<std::uint8_t> bytes;
      for(std::size_t i=0;i<hex.size();i+=2) bytes.push_back(std::stoul(hex.substr(i,2),nullptr,16));
      packets.push_back(std::move(bytes));
    }
    if(packets.empty()) return 2;
    SessionCommandConfig c; c.run=c.router="offline"; c.producer="ik"; c.instance="worker";
    c.home={Joints7{1.1,-1.52,-1.52,-1.1,0,0,0},Joints7{-1.1,-1.52,1.52,-1.1,0,0,0}};
    for(auto& q:c.lower) q.fill(-3.2);
    for(auto& q:c.upper) q.fill(3.2);
    c.math.maximum_step=10.; c.math.time_window=0.; c.home_duration=.05; c.home_speed=100.;
    SessionHealthConfig h; h.home=c.home; h.age=10000000000LL;
    h.authorities={Authority{"src","source","offline"},Authority{"ik","worker","offline"},Authority{"sim","executor","offline"}};
    const auto caller=std::this_thread::get_id();
    const char* env=std::getenv("NATIVE_IK_TEST_MODE"); const std::string mode=env?env:"";
    const bool height_mode=std::getenv("NATIVE_IK_HEIGHT")!=nullptr;
    const bool auto_home=std::getenv("NATIVE_AUTO_HOME")!=nullptr;
    const bool reset_mode=mode.rfind("reset_",0)==0;
    std::atomic<bool> entered{false},release{false},destroyed{false},reset_entered{false},reset_finished{false},height_entered{false},height_finished{false};
    struct FakeIk:IkEndpoint,ResetEndpoint {
      std::string mode; ArmPair home; std::atomic<bool>& entered; std::atomic<bool>& release; std::atomic<bool>& destroyed;
      std::atomic<bool> cancelled{false}; std::thread::id owner=std::this_thread::get_id();
      std::atomic<bool>& reset_entered; std::atomic<bool>& reset_finished; std::int64_t reset_epoch=1;
      std::atomic<bool>& height_entered; std::atomic<bool>& height_finished;
      FakeIk(std::string m,ArmPair h,std::atomic<bool>& e,std::atomic<bool>& r,std::atomic<bool>& d,
             std::atomic<bool>& re,std::atomic<bool>& rf,std::atomic<bool>& he,std::atomic<bool>& hf):mode(std::move(m)),home(h),entered(e),release(r),destroyed(d),reset_entered(re),reset_finished(rf),height_entered(he),height_finished(hf) {}
      ~FakeIk() { destroyed=std::this_thread::get_id()==owner && (!reset_entered || reset_finished) && (!height_entered || height_finished); }
      void request_stop() noexcept override { cancelled=true; }
      ResetEndpoint* reset_service() noexcept override { return mode.rfind("reset_",0)==0 || supports_height()?this:nullptr; }
      bool supports_height() const noexcept override { return mode.rfind("height_",0)==0; }
      void configure_height(const std::array<double,2>&) override {
        height_entered=true;
        struct Done { std::atomic<bool>& finished; ~Done(){ finished=true; } } done{height_finished};
        if(mode=="height_fail") throw std::runtime_error("injected height ACK failure");
        const auto until=time_ns()+5000000000LL;
        while(!cancelled && time_ns()<until) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        throw std::runtime_error("injected height cancellation");
      }
      std::int64_t epoch() const override { return reset_epoch; }
      void reset(const std::array<double,14>& q,std::int64_t epoch) override {
        for(int s=0;s<2;++s) for(int j=0;j<7;++j)
          if(q[s*7+j]!=home[s][j]) throw std::runtime_error("reset received non-Home");
        reset_entered=true;
        struct Done { std::atomic<bool>& finished; ~Done(){ finished=true; } } done{reset_finished};
        const auto until=time_ns()+5000000000LL;
        while(!supports_height() && !cancelled && !release && time_ns()<until) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if(cancelled || mode=="reset_fail") throw std::runtime_error("injected reset failure/cancellation");
        reset_epoch=epoch+(mode=="reset_wrong_epoch"?1:0);
      }
      WorkerResult step(const WorkerTick& request) override {
        entered=true;
        if(mode=="cancel" || mode=="late_return" || mode=="overflow" || mode=="io_failure") {
          const auto until=time_ns()+5000000000LL;
          while(!cancelled && !release && time_ns()<until) std::this_thread::sleep_for(std::chrono::milliseconds(1));
          if(cancelled) throw std::runtime_error("injected IPC cancellation");
        }
        if(mode=="stale") std::this_thread::sleep_for(std::chrono::milliseconds(250));
        WorkerResult out; out.tick=request.id; out.timestamp=request.now_ns;
        for(int s=0;s<2;++s) out.arms[s].q=home[s];
        out.arms[0].q[0]+=.1;
        if(mode=="wrong_tick") ++out.tick;
        if(mode=="nonfinite") out.arms[0].q[0]=std::numeric_limits<double>::quiet_NaN();
        if(mode=="limit") out.arms[0].q[0]=99.;
        return out;
      }
    };
    SessionRuntime runtime(5000000,100000000,512,false,c,h,nullptr,
      std::make_unique<TjvrInput>(std::string(argv[1])=="mapped_palm",.15,.6),200000000,
      [&] { return std::make_unique<MujocoEndpoint>(command[2]); },
      [&]() -> std::unique_ptr<IkEndpoint> {
        if(std::this_thread::get_id()==caller) throw std::runtime_error("IK factory on caller thread");
        if(!mode.empty()) return std::make_unique<FakeIk>(mode,c.home,entered,release,destroyed,reset_entered,reset_finished,height_entered,height_finished);
        return std::make_unique<WorkerIkEndpoint>(command,argv[1],argv[2],std::stoi(argv[3]),true);
      },height_mode);
    if(auto_home) runtime.enable_auto_home_rearm();
    SessionEvent forged{90,"proposal","",false}; forged.proposal=SessionProposal{};
    if(runtime.submit(forged)) throw std::runtime_error("external IK proposal accepted");
    SessionEvent status{1,"status","",false}; HealthStatus s; s.role=1; s.sequence=1; s.authority=h.authorities[1]; s.ready=s.healthy=s.simulation=true; status.status=s;
    if(runtime.submit(status)) throw std::runtime_error("external producer status accepted");
    runtime.enable_cycle_capture(8192);
    runtime.start();
    auto end=time_ns()+10000000000LL;
    while((!runtime.snapshot().ik_ready || !runtime.snapshot().simulation_ready) && time_ns()<end) {
      if(runtime.snapshot().state.phase=="fault") throw std::runtime_error(runtime.snapshot().state.reason);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    s.role=0; s.authority=h.authorities[0]; status.status=s; runtime.submit(status);
    std::size_t reset_packet=10;
    auto wait_reply=[&](std::uint64_t id,bool accepted) {
      const auto until=time_ns()+2000000000LL;
      auto next_raw=time_ns();
      while(time_ns()<until) {
        if((id==6 || (auto_home && id==0)) && mode.empty() && time_ns()>=next_raw && reset_packet<packets.size()) {
          SessionEvent raw{100+reset_packet,"raw","",false};
          raw.raw=RawDatagram{h.authorities[0],packets[reset_packet++]}; runtime.submit(raw);
          next_raw=time_ns()+10000000;
        }
        SessionReply r; SessionCommandResult cr;
        while(runtime.pop_command_receipt(cr)) if(!cr.receipt->accepted) throw std::runtime_error(cr.receipt->reason);
        while(runtime.pop(r)) if(r.id==id) {
          if(r.outcome.accepted!=accepted) throw std::runtime_error(r.outcome.reason);
          return;
        }
        if(runtime.snapshot().state.phase=="fault") {
          if(mode=="height_fail" && id>=100) return; // The injected ACK failure is checked by the height scenario below.
          throw std::runtime_error(runtime.snapshot().state.reason);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      throw std::runtime_error("reply timeout");
    };
    runtime.submit({2,"start","",true}); wait_reply(2,false);
    if(runtime.snapshot().ik_ticks!=0) throw std::runtime_error("IK ran without input/authorization");
    if(height_mode) {
      std::size_t i=0;
      for(;i<packets.size();++i) {
        SessionEvent e{100+i,"raw","",false}; e.raw=RawDatagram{h.authorities[0],packets[i]};
        runtime.submit(e); wait_reply(100+i,true);
        if(i==0) {
          runtime.submit({3,"start","",true}); wait_reply(3,false);
          runtime.submit({4,"calibrate","",true}); wait_reply(4,true);
          if(mode=="height_gap" || mode=="height_abort") {
            if(mode=="height_abort") runtime.submit({9,"shutdown","",true});
            wait_reply(4,false);
            auto failed=runtime.snapshot();
            if(!failed.height || failed.height->state!="failed" || failed.ik_ticks)
              throw std::runtime_error("gap did not report failed calibration at rest");
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            SessionReply extra;
            while(runtime.pop(extra)) if(extra.id==4) throw std::runtime_error("duplicate calibration failure");
            runtime.stop();std::cout<<"scheduled_height_failure_held\n";return 0;
          }
          runtime.submit({5,"calibrate","",true}); wait_reply(5,false);
          runtime.submit({8,"start","",true}); wait_reply(8,false);
        }
        auto snap=runtime.snapshot();
        if(mode=="height_cancel" && height_entered) {
          auto stopping=time_ns(); runtime.stop(); snap=runtime.snapshot();
          if(time_ns()-stopping>1000000000LL || !destroyed || snap.reset_pending || snap.state.epoch!=1 || snap.height->offsets || snap.state.shutdown_complete)
            throw std::runtime_error("height cancellation committed or leaked");
          std::cout<<"scheduled_height_failure_held\n"; return 0;
        }
        if(mode=="height_fail" && snap.state.phase=="fault") {
          runtime.stop(); snap=runtime.snapshot();
          if(!height_entered || !destroyed || snap.state.epoch!=1 || snap.height->offsets || snap.state.shutdown_complete)
            throw std::runtime_error("height failure committed or leaked");
          std::cout<<"scheduled_height_failure_held\n"; return 0;
        }
        if(snap.ik_ticks || snap.feedback!=std::optional<ArmPair>(c.home)) throw std::runtime_error("calibration moved robot");
        if(snap.height && snap.height->state=="calibrated") break;
        if(snap.height && snap.height->state=="failed") throw std::runtime_error(snap.height->error);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      auto snap=runtime.snapshot();
      if(!snap.height || !snap.height->offsets || snap.state.epoch!=2) throw std::runtime_error("height transaction did not commit");
      for(int n=0;n<10 && ++i<packets.size();++n) {
        SessionEvent e{100+i,"raw","",false}; e.raw=RawDatagram{h.authorities[0],packets[i]};
        runtime.submit(e); wait_reply(100+i,true);
        if(n==0) { runtime.submit({9,"start","",true}); wait_reply(9,true); }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      if(runtime.snapshot().ik_ticks<2) throw std::runtime_error("calibrated IK did not run");
      runtime.submit({10,"shutdown","",true}); wait_reply(10,true);
      while(!runtime.snapshot().state.shutdown_complete && time_ns()<end) std::this_thread::sleep_for(std::chrono::milliseconds(1));
      runtime.stop();
      if(!runtime.snapshot().state.shutdown_complete) throw std::runtime_error("height session did not return Home");
      std::cout<<"scheduled_height_complete\n"; return 0;
    }
    const auto first_batch=mode.empty()?std::min<std::size_t>(10,packets.size()):packets.size();
    for(std::size_t i=0;i<first_batch;++i) {
      SessionEvent event{100+i,"raw","",false}; event.raw=RawDatagram{h.authorities[0],packets[i]}; runtime.submit(event); wait_reply(100+i,true);
      if(reset_mode) {
        runtime.submit({6,"rearm","",true,2,false});
        while(!reset_entered && time_ns()<end) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if(!reset_entered || !runtime.snapshot().reset_pending) throw std::runtime_error("reset task did not start");
        auto before=runtime.snapshot().ticks;
        runtime.submit({67,"start","",true}); wait_reply(67,false);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if(runtime.snapshot().ticks<=before+1 || entered) throw std::runtime_error("reset blocked scheduler or overlapped IK");
        if(mode=="reset_ok") {
          SessionEvent raw{70,"raw","",false}; raw.raw=RawDatagram{h.authorities[0],packets.at(2)};
          runtime.submit(raw); wait_reply(70,true); // A fresh PRE-commit frame is still not a restart token.
        }
        if(mode=="reset_epoch") {
          SessionEvent raw{68,"raw","",false}; raw.raw=RawDatagram{h.authorities[0],packets.at(1)}; runtime.submit(raw);
        }
        if(mode=="reset_overflow") for(int n=0;n<2000;++n)
          if(!runtime.submit({static_cast<std::uint64_t>(1000+n),"invalid","",false})) break;
        if(mode=="reset_io_failure" && !runtime.report_failure("recorder failed during reset"))
          throw std::runtime_error("local failure admission rejected");
        if(mode=="reset_ok" || mode=="reset_wrong_epoch" || mode=="reset_fail") release=true;
        if(mode=="reset_ok") {
          wait_reply(6,true);
          runtime.submit({69,"start","",true}); wait_reply(69,false);
          SessionEvent duplicate{71,"raw","",false}; duplicate.raw=RawDatagram{h.authorities[0],packets.at(2)};
          runtime.submit(duplicate); wait_reply(71,false);
          runtime.submit({72,"start","",true}); wait_reply(72,false);
        } else if(mode!="reset_cancel") {
          while(runtime.snapshot().state.phase!="fault" && time_ns()<end) std::this_thread::sleep_for(std::chrono::milliseconds(1));
          if(runtime.snapshot().state.phase!="fault") throw std::runtime_error("reset failure did not fault");
        }
        auto stopping=time_ns(); runtime.stop(); auto snap=runtime.snapshot();
        if(time_ns()-stopping>1000000000LL || !destroyed || !reset_finished || snap.reset_pending || snap.ik_ready ||
           snap.state.epoch!=(mode=="reset_ok"?2:1) || snap.ik_ticks || snap.state.shutdown_complete || snap.feedback!=std::optional<ArmPair>(c.home))
          throw std::runtime_error("reset cleanup/epoch/Home invariant failed");
        std::cout<<"scheduled_ik_reset_checked\n"; return 0;
      }
      if(i==0) { runtime.submit({3,"start","",true}); wait_reply(3,true); }
      if(!mode.empty()) {
        while(!entered && time_ns()<end) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if(mode=="overflow") {
          for(int n=0;n<513;++n) runtime.submit({static_cast<std::uint64_t>(1000+n),"invalid","",false});
          release=true;
        }
        if(mode=="io_failure") {
          if(!runtime.report_failure("recorder failed during IK")) throw std::runtime_error("local failure admission rejected");
          release=true;
        }
        if(mode=="late_return") {
          runtime.submit({4,"return","",true}); release=true; wait_reply(4,true);
          while(runtime.snapshot().state.phase!="idle" && time_ns()<end) std::this_thread::sleep_for(std::chrono::milliseconds(1));
          if(runtime.snapshot().ik_reference) throw std::runtime_error("late result adopted after return");
        } else if(mode!="cancel") {
          while(runtime.snapshot().state.phase!="fault" && time_ns()<end) std::this_thread::sleep_for(std::chrono::milliseconds(1));
          if(runtime.snapshot().state.phase!="fault") throw std::runtime_error("invalid result did not fault");
          if(mode=="io_failure" && (runtime.snapshot().ik_reference || runtime.snapshot().ik_ticks))
            throw std::runtime_error("late IK result adopted after I/O failure");
        }
        auto stopping=time_ns(); runtime.stop();
        if(time_ns()-stopping>1000000000LL || !destroyed || runtime.snapshot().ik_ready ||
           runtime.snapshot().state.shutdown_complete || runtime.snapshot().feedback!=std::optional<ArmPair>(c.home))
          throw std::runtime_error("failure/cancellation cleanup or hold invalid");
        if(mode=="io_failure" || mode=="late_return") {
          SessionCycleSnapshot cycle;bool saw_rejected=false;
          while(runtime.pop_cycle(cycle)) {
            if(cycle.result) {
              if(!cycle.request || cycle.ik_adopted) throw std::runtime_error("late result falsely recorded as adopted");
              saw_rejected=true;
            }
          }
          if(!saw_rejected) throw std::runtime_error("late result missing from completed-cycle audit");
        }
        std::cout<<"scheduled_ik_failure_held\n"; return 0;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      const auto snap=runtime.snapshot();
      if(snap.state.phase=="fault") throw std::runtime_error(snap.state.reason);
      if(snap.ik_reference && snap.feedback!=snap.ik_reference) throw std::runtime_error("IK/feedback mismatch");
    }
    if(runtime.snapshot().ik_ticks<2) throw std::runtime_error("IK did not execute");
    runtime.submit({4,"return","",true}); wait_reply(4,true);
    const auto ticks=runtime.snapshot().ik_ticks;
    while(runtime.snapshot().state.phase!="idle" && time_ns()<end) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if(runtime.snapshot().feedback!=std::optional<ArmPair>(c.home) || (!auto_home && runtime.snapshot().ik_ticks!=ticks))
      throw std::runtime_error("Home was overwritten by IK");
    if(auto_home) {
      wait_reply(0,true);
      if(runtime.snapshot().state.epoch!=2 || runtime.snapshot().ik_ticks || runtime.snapshot().state.phase!="idle")
        throw std::runtime_error("automatic Home rearm failed or started motion");
      SessionEvent fresh{92,"raw","",false}; fresh.raw=RawDatagram{h.authorities[0],packets.at(reset_packet++)};
      runtime.submit(fresh); wait_reply(92,true);
      runtime.submit({93,"start","",true}); wait_reply(93,true);
      while(!runtime.snapshot().ik_ticks && time_ns()<end) std::this_thread::sleep_for(std::chrono::milliseconds(1));
      if(!runtime.snapshot().ik_ticks) throw std::runtime_error("direct s after automatic rearm failed");
      runtime.submit({94,"return","",true}); wait_reply(94,true);
      wait_reply(0,true);
      if(runtime.snapshot().state.epoch!=3) throw std::runtime_error("second automatic rearm failed");
    }
    if(!auto_home) { runtime.submit({5,"start","",true}); wait_reply(5,false); }
    runtime.submit({60,"rearm","",false,2,true}); wait_reply(60,false);
    runtime.submit({61,"rearm","",true,auto_home?5:3,true}); wait_reply(61,false);
    // The boolean is false deliberately: only the actual worker ACK may commit.
    runtime.submit({6,"rearm","",true,auto_home?4:2,false}); wait_reply(6,true);
    auto reset=runtime.snapshot();
    if(reset.state.epoch!=(auto_home?4:2) || reset.ik_ticks || reset.ik_reference || reset.feedback!=std::optional<ArmPair>(c.home))
      throw std::runtime_error("owned reset did not atomically clear IK and retain measured Home");
    // The real continuously-fed stream can race the ACK with a post-commit frame.
    // The controlled reset_ok case above tests the exact no-new-frame barrier.
    SessionEvent duplicate{63,"raw","",false}; duplicate.raw=RawDatagram{h.authorities[0],packets[reset_packet-1]};
    runtime.submit(duplicate); wait_reply(63,false);
    for(std::size_t i=reset_packet;i<packets.size();++i) {
      SessionEvent event{100+i,"raw","",false}; event.raw=RawDatagram{h.authorities[0],packets[i]};
      runtime.submit(event); wait_reply(100+i,true);
      if(i==reset_packet) { runtime.submit({65,"start","",true}); wait_reply(65,true); }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      auto snap=runtime.snapshot();
      if(snap.state.phase=="fault") throw std::runtime_error(snap.state.reason);
      if(snap.ik_reference && snap.feedback!=snap.ik_reference) throw std::runtime_error("post-reset IK/feedback mismatch");
    }
    if(runtime.snapshot().ik_ticks<2) throw std::runtime_error("post-reset IK did not resume");
    runtime.submit({66,"rearm","",true,3,true}); wait_reply(66,false);
    runtime.submit({7,"shutdown","",true}); wait_reply(7,true);
    while(!runtime.snapshot().state.shutdown_complete && time_ns()<end) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    runtime.stop();
    if(!runtime.snapshot().state.shutdown_complete || runtime.snapshot().ik_ready || runtime.snapshot().simulation_ready) return 1;
    SessionCycleSnapshot cycle;
    std::uint64_t sequence=0,adopted=0;
    bool home_complete=false;
    while(runtime.pop_cycle(cycle)) {
      if(cycle.snapshot.ticks!=++sequence) throw std::runtime_error("cycle sequence gap");
      if(cycle.ik_adopted) {
        ++adopted;
        if(!cycle.request || !cycle.result || cycle.result->wire.empty() ||
           cycle.request->id!=cycle.result->tick || cycle.snapshot.feedback!=cycle.snapshot.ik_reference)
          throw std::runtime_error("cycle IK association or feedback missing");
      }
      home_complete=cycle.snapshot.state.shutdown_complete;
    }
    if(!adopted || !home_complete || sequence!=runtime.snapshot().ticks || runtime.snapshot().cycle_capture_failed)
      throw std::runtime_error("cycle capture incomplete");
    std::cout<<"scheduled_ik_and_simulation_complete\n";
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
