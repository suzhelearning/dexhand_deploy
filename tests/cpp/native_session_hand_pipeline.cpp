#include "../../native/control/session_runtime.hpp"
#include "../../native/control/native_hand_pipeline.hpp"
#include "../../native/control/mujoco_endpoint.hpp"
#include "../../native/control/manus_receiver.hpp"
#include <iostream>
using namespace tianji_control;
class ArmResetFixture final:public ResetEndpoint {
 public:
  void reset(const std::array<double,14>&,std::int64_t e) override {epoch_=e;}
  std::int64_t epoch() const override {return epoch_;}
  void request_stop() noexcept override {}
 private:std::int64_t epoch_=1;
};
class IntegratedArmFixture final:public IkEndpoint,public ResetEndpoint {
 public:
  WorkerResult step(const WorkerTick& request) override {
    WorkerResult r;r.tick=request.id;r.timestamp=request.now_ns;r.input_live=true;
    r.applied_epoch=1;r.applied_sequence=request.id;r.control_executed=true;
    for(auto& arm:r.arms)arm.accepted=true;
    return r;
  }
  ResetEndpoint* reset_service() noexcept override{return this;}
  void reset(const std::array<double,14>&,std::int64_t e) override{epoch_=e;}
  std::int64_t epoch() const override{return epoch_;}
  void request_stop() noexcept override{}
 private:std::int64_t epoch_=1;
};
// Explicit offline arm-input fixture; a status heartbeat must NOT release the
// main session's post-reset new-input gate.
class RawFixture final:public RawInputEndpoint {
 public:
  RawProgress ingest(const std::vector<std::uint8_t>& bytes) override {
    RawProgress p;if(bytes.size()!=1) return p;
    p.accepted=p.decoded=p.skeleton_valid=true;p.epoch=1;
    p.sequence=p.ingress_sequence=bytes[0];return p;
  }
};
static std::uint64_t stamp() {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
int main(int argc,char** argv) {
 if(argc<5) return 2;
 try {
  std::vector<std::string> command;for(int i=3;i<argc;++i) command.emplace_back(argv[i]);
  const std::string mode=argv[2];
  SessionCommandConfig c;c.run=c.router="offline";c.producer="ik";c.instance="worker";
  c.home_duration=.02;c.home_speed=100.;
  for(auto& q:c.lower) q.fill(-3.2);
  for(auto& q:c.upper) q.fill(3.2);
  SessionHealthConfig h;h.home=c.home;h.age=10000000000LL;
  h.authorities={Authority{"src","source","offline"},Authority{"ik","worker","offline"},Authority{"sim","executor","offline"}};
  SessionHandConfig hc;hc.run=c.run;hc.source={"manus","rawviz","offline"};hc.producer={"hand","native","offline"};hc.age=5000000000LL;
  for(auto& q:hc.lower) q.fill(-4.);
  for(auto& q:hc.upper) q.fill(4.);
  for(auto& q:hc.zero_tolerance) q.fill(.01);
  const bool automatic=mode=="auto_home";
  SessionRuntime r(5000000,10000000000LL,128,true,c,h,
    automatic?nullptr:std::unique_ptr<ResetEndpoint>(std::make_unique<ArmResetFixture>()),std::make_unique<RawFixture>(),5000000000LL,
    [&]{return std::make_unique<MujocoEndpoint>(argv[1],true);},
    automatic?IkFactory([]{return std::make_unique<IntegratedArmFixture>();}):IkFactory{},false,hc,
    std::make_unique<NativeHandPipeline>(command,2000,mode=="overflow"?1:16));
  if(automatic)r.enable_auto_home_rearm();
  std::vector<std::string> driver_blocks;
  std::unique_ptr<OwnedDescriptor> driver_writer;
  std::unique_ptr<ManusReceiver> manus;
  std::atomic<unsigned> raw_lines{0};
  r.enable_cycle_capture(128);
  r.enable_manus_capture(mode=="capture_overflow"?1:512);
  unsigned hand_results=0;
  unsigned recorded_lines=0;
  auto drain=[&]{SessionCycleSnapshot cycle;while(r.pop_cycle(cycle)) {
    for(const auto& audit:cycle.hand_results) {
      if(audit.result.run!=hc.run || !(audit.result.producer==hc.producer) ||
         audit.observed_ns<=0 || !audit.result.value.output_sequence || audit.outcome.reason.empty())
        throw std::runtime_error("incomplete native hand result audit");
      ++hand_results;
    }
  }
    ManusIngressRecord raw;while(r.pop_manus(raw)) {
      if(raw.line_sequence!=++recorded_lines) throw std::runtime_error("Manus recording reordered or lost lines");
    }
  };
  if(mode=="rawviz") {
    for(std::string block;std::getline(std::cin,block,'\f');)driver_blocks.push_back(std::move(block));
    int fds[2];if(pipe(fds))throw std::runtime_error("fixture pipe failed");
    driver_writer=std::make_unique<OwnedDescriptor>(fds[1]);
    manus=std::make_unique<ManusReceiver>(OwnedDescriptor(fds[0]),hc.source,1,"","",
      [&](const ManusIngressRecord& record){
        ++raw_lines;
        return r.submit_manus(record);
      },[&](const std::string& why){r.report_failure(why);});
    manus->start();
  }
  std::uint64_t id=0;
  auto wait=[&](auto predicate){const auto until=stamp()+5000000000ULL;while(stamp()<until){
    drain();
    if(predicate()) return;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));}
    throw std::runtime_error("timeout: "+r.snapshot().state.reason);};
  auto status=[&](int role,int sequence){SessionEvent e{++id,"status","",false};e.reply=false;
    if(automatic && role==1)return;
    HealthStatus s;s.role=role;s.sequence=sequence;s.authority=h.authorities[role];s.ready=s.healthy=s.simulation=true;e.status=s;
    if(!r.submit(e)) throw std::runtime_error("status rejected");};
  auto intent=[&](const std::string& action,std::int64_t epoch=0){const auto key=++id;
    if(!r.submit({key,action,"",true,epoch,false})) throw std::runtime_error("intent admission rejected");
    bool ok=false;wait([&]{SessionReply reply;while(r.pop(reply)) if(reply.id==key){ok=reply.outcome.accepted;return true;}return false;});return ok;};
  auto sample=[&](int sequence,int generation=1){
    if(manus) {
      const auto& block=driver_blocks.at(sequence-1);
      for(char c:block)if(write(driver_writer->get(),&c,1)!=1)throw std::runtime_error("fixture write failed");
      return;
    }
    HandSampleEnvelope envelope;envelope.source=hc.source;auto& v=envelope.value;
    v.sequence=sequence;v.generation=generation;v.timestamp_ns=stamp();v.flags=3;
    for(int side=0;side<2;++side){std::size_t k=side*63+3;
      for(int finger=0;finger<5;++finger) for(int joint=0;joint<4;++joint){
        v.points[k++]=.02*(finger-2);v.points[k++]=.02*(joint+1);v.points[k++]=.002*joint;}}
    if(mode=="nonfinite" && sequence==3) v.points[0]=std::numeric_limits<double>::quiet_NaN();
    SessionEvent event{++id,"hand_sample","",false};event.reply=false;event.hand_sample=envelope;
    if(!r.submit(event) && mode!="overflow") throw std::runtime_error("sample rejected");};
  auto raw=[&](std::uint8_t seq){SessionEvent e{++id,"raw","",false};e.reply=false;
    e.raw=RawDatagram{h.authorities[0],{seq},static_cast<std::int64_t>(stamp())};
    if(!r.submit(e)) throw std::runtime_error("raw admission rejected");};
  status(0,1);status(1,1);r.start();wait([&]{return r.snapshot().simulation_ready;});
  SessionEvent forged{++id,"hand_result","",false};forged.hand_result=HandResultEnvelope{};
  if(r.submit(forged)) throw std::runtime_error("external result bypassed owned producer");
  if(intent("start")) throw std::runtime_error("started without hand input");
  if(mode=="capture_overflow") {
    ManusIngressRecord first;first.line_sequence=1;first.received_ns=stamp();first.raw_line="one";
    if(!r.submit_manus(first)) throw std::runtime_error("first Manus record refused");
    auto second=first;second.line_sequence=2;
    if(r.submit_manus(second)) throw std::runtime_error("Manus raw overflow silently accepted");
    wait([&]{return r.snapshot().state.phase=="fault";});r.stop();
    if(recorded_lines!=1 || r.snapshot().state.shutdown_complete)
      throw std::runtime_error("Manus raw overflow lost admitted data or completed");
    return 0;
  }
  if(mode=="stall" || mode=="overflow" || mode=="reset_cancel") {
    raw(1);sample(1);
    const auto ticks=r.snapshot().ticks;
    wait([&]{return r.snapshot().ticks>=ticks+4;});
    if(intent("start")) throw std::runtime_error("missing result granted readiness");
    if(mode=="overflow") {
      for(int i=2;i<10;++i) sample(i);
      wait([&]{return r.snapshot().state.phase=="fault";});
      // Pending samples coalesce now; a stalled child still faults on its IPC
      // deadline, rather than overflowing a FIFO of obsolete samples.
      if(r.snapshot().state.reason.find("timeout")==std::string::npos) throw std::runtime_error("wrong stalled-worker failure: "+r.snapshot().state.reason);
    }
    if(mode=="reset_cancel") {
      r.submit({++id,"rearm","",true,2,false});
      wait([&]{return r.snapshot().reset_pending;});
    }
    const auto before=stamp();r.stop();
    if(stamp()-before>2000000000ULL || r.snapshot().state.epoch!=1 || r.snapshot().state.shutdown_complete)
      throw std::runtime_error("blocked hand I/O shutdown failed");
    std::cout<<"hand pipeline bounded failure passed\n";return 0;
  }
  raw(1);sample(1);wait([&]{return intent("start");});
  sample(2);wait([&]{const auto s=r.snapshot();return s.hand_feedback && *s.hand_feedback!=hc.zero;});
  if(mode=="generation" || mode=="nonfinite") {
    sample(3,mode=="generation"?2:1);wait([&]{return r.snapshot().state.phase=="fault";});r.stop();
    if(r.snapshot().state.shutdown_complete) throw std::runtime_error("fault marked complete");
  } else {
    if(!intent("return")) throw std::runtime_error("return rejected");
    wait([&]{auto s=r.snapshot();return s.state.phase=="idle" && s.hand_feedback && *s.hand_feedback==hc.zero;});
    if(automatic)wait([&]{return r.snapshot().state.epoch==2 && !r.snapshot().reset_pending;});
    if(!intent("rearm",automatic?3:2)) throw std::runtime_error("same-worker manual rearm failed");
    if(intent("start")) throw std::runtime_error("old hand cache bypassed reset");
    status(0,2);status(1,2);raw(2);sample(3);wait([&]{return intent("start");});
    sample(4);wait([&]{const auto s=r.snapshot();return s.hand_feedback && *s.hand_feedback!=hc.zero;});
    if(!intent("shutdown")) throw std::runtime_error("shutdown rejected");
    wait([&]{return r.snapshot().state.shutdown_complete;});r.stop();
    if(*r.snapshot().hand_feedback!=hc.zero) throw std::runtime_error("hand Home incomplete");
  }
  if(manus){manus->stop();if(!manus->failure().empty() || !raw_lines)throw std::runtime_error("rawviz ingress failed");}
  drain();
  if(manus && recorded_lines!=raw_lines) throw std::runtime_error("raw Manus queue did not drain");
  if((mode=="home" || mode=="rawviz" || automatic) && hand_results<4)
    throw std::runtime_error("native hand result audit lost worker outputs");
  std::cout<<"native sample/solve/session/MuJoCo pipeline passed\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
