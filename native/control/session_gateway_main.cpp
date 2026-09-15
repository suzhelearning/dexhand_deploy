#include "mujoco_endpoint.hpp"
#include "session_gateway.hpp"
#include "viewer_hud.hpp"
#include "tjvr_input.hpp"
#include "worker_ik_endpoint.hpp"
#include "session_zenoh_publication.hpp"
#include "session_recording_sink.hpp"
#include "viewer_state.hpp"
#include "hand_launch_config.hpp"
#include "native_hand_pipeline.hpp"
#include "manus_receiver.hpp"
#ifdef TIANJI_WITH_GLFW
#include "viewer_window.hpp"
#endif

#include <nlohmann/json.hpp>

#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <netdb.h>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

namespace {
using json=nlohmann::json;
using namespace tianji_control;

const json& required(const json& object,const char* key) {
  if(!object.is_object() || !object.contains(key))
    throw std::invalid_argument(std::string("native gateway manifest missing ")+key);
  return object.at(key);
}

std::string text(const json& object,const char* key,std::size_t maximum=4096) {
  const auto& value=required(object,key);
  if(!value.is_string()) throw std::invalid_argument(std::string("native gateway ")+key+" must be text");
  const auto result=value.get<std::string>();
  if(result.empty() || result.size()>maximum || result.find('\0')!=std::string::npos)
    throw std::invalid_argument(std::string("invalid native gateway ")+key);
  return result;
}

std::int64_t integer(const json& object,const char* key,std::int64_t minimum=0) {
  const auto& value=required(object,key);
  if(!value.is_number_integer()) throw std::invalid_argument(std::string("native gateway ")+key+" must be integer");
  const auto result=value.get<std::int64_t>();
  if(result<minimum) throw std::invalid_argument(std::string("invalid native gateway ")+key);
  return result;
}

double real(const json& object,const char* key,double minimum=0.) {
  const auto& value=required(object,key);
  if(!value.is_number()) throw std::invalid_argument(std::string("native gateway ")+key+" must be number");
  const auto result=value.get<double>();
  if(!std::isfinite(result) || result<minimum)
    throw std::invalid_argument(std::string("invalid native gateway ")+key);
  return result;
}

bool boolean(const json& object,const char* key) {
  const auto& value=required(object,key);
  if(!value.is_boolean()) throw std::invalid_argument(std::string("native gateway ")+key+" must be boolean");
  return value.get<bool>();
}

Authority authority(const json& object,const char* key) {
  const auto& value=required(object,key);
  if(!value.is_object()) throw std::invalid_argument(std::string("native gateway ")+key+" authority must be object");
  return {text(value,"logical",256),text(value,"instance",256),text(value,"router",256)};
}

Joints7 joints(const json& value,const std::string& field) {
  if(!value.is_array() || value.size()!=7)
    throw std::invalid_argument("native gateway "+field+" must have seven joints");
  Joints7 result{};
  for(std::size_t i=0;i<result.size();++i) {
    if(!value[i].is_number()) throw std::invalid_argument("native gateway "+field+" contains non-number");
    result[i]=value[i].get<double>();
    if(!std::isfinite(result[i])) throw std::invalid_argument("native gateway "+field+" contains nonfinite joint");
  }
  return result;
}

ArmPair paired_joints(const json& object,const char* key) {
  const auto& value=required(object,key);
  if(!value.is_array() || value.size()!=2)
    throw std::invalid_argument(std::string("native gateway ")+key+" must contain two sides");
  return {joints(value[0],std::string(key)+"[0]"),joints(value[1],std::string(key)+"[1]")};
}

std::vector<std::string> command(const json& object) {
  const auto& value=required(object,"worker_command");
  if(!value.is_array() || value.empty() || value.size()>64)
    throw std::invalid_argument("native gateway worker_command must be a nonempty array");
  std::vector<std::string> result;
  result.reserve(value.size());
  for(const auto& item:value) {
    if(!item.is_string()) throw std::invalid_argument("native gateway worker argv must be text");
    auto arg=item.get<std::string>();
    if(arg.empty() || arg.size()>4096 || arg.find('\0')!=std::string::npos)
      throw std::invalid_argument("invalid native gateway worker argv");
    result.push_back(std::move(arg));
  }
  if(result.front().front()!='/') throw std::invalid_argument("native gateway worker must be absolute");
  return result;
}

void require_file(const std::string& path,const char* label) {
  const std::filesystem::path value(path);
  if(!value.is_absolute() || !std::filesystem::is_regular_file(value))
    throw std::invalid_argument(std::string("native gateway missing ")+label+": "+path);
}

int bind_udp(const std::string& host,std::int64_t port) {
  if(port<0 || port>65535) throw std::invalid_argument("native gateway UDP port out of range");
  addrinfo hints{}; hints.ai_family=AF_INET; hints.ai_socktype=SOCK_DGRAM;
  hints.ai_flags=AI_NUMERICHOST;
  addrinfo* addresses=nullptr;
  const auto port_text=std::to_string(port);
  const int result=getaddrinfo(host.c_str(),port_text.c_str(),&hints,&addresses);
  if(result!=0) throw std::invalid_argument(std::string("native gateway invalid UDP bind: ")+gai_strerror(result));
  int fd=-1;
  for(auto* address=addresses;address;address=address->ai_next) {
    fd=socket(address->ai_family,address->ai_socktype|SOCK_CLOEXEC,address->ai_protocol);
    if(fd<0) continue;
    // Deliberately do not set SO_REUSEADDR: two active sessions must not share
    // a TJVR port and receive an ambiguous stream.
    if(bind(fd,address->ai_addr,address->ai_addrlen)==0) break;
    close(fd); fd=-1;
  }
  freeaddrinfo(addresses);
  if(fd<0) throw std::runtime_error(std::string("native gateway UDP bind failed: ")+std::strerror(errno));
  return fd;
}

json load_manifest(const std::filesystem::path& path) {
  if(!path.is_absolute() || !std::filesystem::is_regular_file(path))
    throw std::invalid_argument("native gateway manifest must be an existing absolute file");
  std::ifstream input(path);
  if(!input) throw std::runtime_error("native gateway manifest open failed");
  json value=json::parse(input,nullptr,true,true);
  if(!value.is_object()) throw std::invalid_argument("native gateway manifest must be an object");
  return value;
}

int run(const std::filesystem::path& manifest_path,int control_fd) {
  if(control_fd<3) throw std::invalid_argument("native gateway control fd must be >= 3");
  const auto manifest=load_manifest(manifest_path);
  const auto hand_launch=parse_hand_launch(manifest,control_fd);
  const auto run_id=text(manifest,"run_id",256);
  const auto router=text(manifest,"router_zid",256);
  const auto source=authority(manifest,"source_authority");
  const auto producer=authority(manifest,"producer_authority");
  const auto executor=authority(manifest,"executor_authority");
  if(source.router!=router || producer.router!=router || executor.router!=router)
    throw std::invalid_argument("native gateway authority router mismatch");
  const auto worker_prefix=text(manifest,"worker_prefix",32);
  if(worker_prefix!="spark" && worker_prefix!="mapped_palm")
    throw std::invalid_argument("native gateway worker_prefix must be spark or mapped_palm");
  const auto algorithm=text(manifest,"algorithm",256);
  const auto model=text(manifest,"model",4096);
  require_file(model,"MuJoCo model");
  const auto worker_command=command(manifest);
  require_file(worker_command.front(),"IK worker");

  SessionCommandConfig commands;
  commands.run=run_id; commands.producer=producer.logical;
  commands.instance=producer.instance; commands.router=router;
  commands.home=paired_joints(manifest,"home");
  commands.lower=paired_joints(manifest,"lower");
  commands.upper=paired_joints(manifest,"upper");
  commands.math.maximum_step=real(manifest,"maximum_step",0.0000001);
  commands.math.time_window=real(manifest,"time_window",0.);
  commands.math.rate=real(manifest,"rate_hz",0.0000001);
  commands.math.proposal_timeout=real(manifest,"proposal_timeout_s",0.0000001);
  commands.home_duration=real(manifest,"home_duration_s",0.0000001);
  commands.home_speed=real(manifest,"home_speed_rad_s",0.0000001);
  commands.ingress_age=integer(manifest,"ingress_age_ns",1);
  commands.clipping=boolean(manifest,"command_step_clipping");

  SessionHealthConfig health;
  health.authorities={source,producer,executor}; health.home=commands.home;
  health.age=integer(manifest,"health_age_ns",1);
  health.home_tolerance=real(manifest,"home_tolerance_rad",0.);
  const auto period=integer(manifest,"period_ns",1);
  const auto freshness=integer(manifest,"freshness_ns",1);
  const auto raw_age=integer(manifest,"raw_age_ns",1);
  const auto worker_timeout=static_cast<int>(integer(manifest,"worker_timeout_ms",1));
  if(worker_timeout>60000) throw std::invalid_argument("native gateway worker timeout is too large");
  const auto mapped=boolean(manifest,"mapped_palm");
  if(mapped!=(worker_prefix=="mapped_palm")) throw std::invalid_argument("native gateway mapped/worker mismatch");
  const auto height=boolean(manifest,"height_calibration");
  if(manifest.value("common_x_reference",false) && !manifest.value("xz_calibration",false))
    throw std::invalid_argument("common X reference requires X/Z calibration");
  const auto max_jump=real(manifest,"max_position_jump_m",0.0000001);
  const auto max_rotation=real(manifest,"max_orientation_jump_rad",0.0000001);
  const auto deterministic=boolean(manifest,"deterministic_worker");
  const auto bind_host=text(manifest,"tjvr_bind",256);
  const auto bind_port=integer(manifest,"tjvr_port",0);

  std::optional<SessionHandConfig> hand_config;
  std::unique_ptr<HandResetEndpoint> hand_worker;
  std::optional<OwnedDescriptor> manus_descriptor;
  if(hand_launch) {
    struct stat info{};
    const int flags=fcntl(hand_launch->fd,F_GETFL);
    if(flags<0 || (flags&O_ACCMODE)!=O_RDONLY || fstat(hand_launch->fd,&info)<0 || !S_ISFIFO(info.st_mode))
      throw std::invalid_argument("Manus stdout must be an inherited read-only pipe");
    manus_descriptor.emplace(hand_launch->fd);
    require_file(hand_launch->command.front(),"hand worker");
    hand_config=hand_launch->domain;
    hand_worker=std::make_unique<NativeHandPipeline>(hand_launch->command,hand_launch->timeout_ms,hand_config->capacity);
  }
  auto runtime=std::make_unique<SessionRuntime>(period,freshness,4096,hand_launch.has_value(),commands,health,
      nullptr,std::make_unique<TjvrInput>(mapped,max_jump,max_rotation),raw_age,
      [model,hands=hand_launch.has_value()] { return std::make_unique<MujocoEndpoint>(model,hands); },
      [worker_command,worker_prefix,algorithm,worker_timeout,deterministic] {
        return std::make_unique<WorkerIkEndpoint>(worker_command,worker_prefix,algorithm,
                                                   worker_timeout,deterministic);
      },height,hand_config,std::move(hand_worker));
  runtime->enable_auto_home_rearm();
  if(manifest.contains("xz_calibration") && boolean(manifest,"xz_calibration")) {
    if(!mapped || !height) throw std::invalid_argument("XZ requires mapped height calibration");
    runtime->enable_xz_calibration(manifest.value("common_x_reference",false));
  }
  std::unique_ptr<ManusReceiver> manus;
  NativeInputHooks manus_hooks;
  if(hand_launch) {
    runtime->enable_manus_capture(hand_launch->domain.capacity);
    auto* owner=runtime.get();
    manus=std::make_unique<ManusReceiver>(std::move(*manus_descriptor),hand_launch->domain.source,
      hand_launch->generation,hand_launch->right_glove,hand_launch->left_glove,
      [owner](const ManusIngressRecord& raw){return owner->submit_manus(raw);},
      [owner](const std::string& error){owner->report_failure("Manus receiver failed: "+error);});
    manus_hooks.start=[&]{manus->start();};manus_hooks.stop=[&]{manus->stop();};
  }
  const auto publication_backend=manifest.value("publication_backend",std::string("python"));
  const auto viewer_backend=manifest.value("viewer_backend",std::string("python"));
  if(viewer_backend!="python" && viewer_backend!="cpp") throw std::invalid_argument("invalid viewer backend");
  const bool native_viewer=viewer_backend=="cpp";
#ifndef TIANJI_WITH_GLFW
  if(native_viewer) throw std::runtime_error("native viewer requires gateway rebuilt with GLFW development files");
#endif
  std::unique_ptr<ViewerState> display;
  if(native_viewer) display=std::make_unique<ViewerState>(mapped);
  if(publication_backend!="python" && publication_backend!="cpp")
    throw std::invalid_argument("invalid publication backend");
  std::unique_ptr<SessionZenohPublication> publication;
  if(publication_backend=="cpp") publication=std::make_unique<SessionZenohPublication>(manifest);
  std::unique_ptr<SessionRecordingSink> recording;
  NativeRecordingHooks recording_hooks;
  if(manifest.contains("recording_fd")) {
    const auto fd=integer(manifest,"recording_fd",3);
    if(fd>std::numeric_limits<int>::max() || fd==control_fd)
      throw std::invalid_argument("invalid recording descriptor");
    recording=std::make_unique<SessionRecordingSink>(OwnedDescriptor(static_cast<int>(fd)),manifest,
      integer(manifest,"recording_origin_ns",1),worker_timeout);
    recording_hooks.write=[&](const auto& item){recording->write(item);};
    recording_hooks.cancel=[&]{recording->request_stop();};
    recording_hooks.finish=[&](bool complete,const auto& state){
      const auto timestamp=std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
      recording->finish(complete,timestamp,state.state.reason);
    };
  }
  auto datagram=OwnedDescriptor(bind_udp(bind_host,bind_port));
  const auto transport=manifest.value("diagnostic_transport",std::string("full"));
  if(transport!="full" && transport!="summary") throw std::invalid_argument("invalid diagnostic transport");
  const bool summaries=transport=="summary";
  if(summaries && (!publication || !native_viewer))
    throw std::invalid_argument("summary transport requires native publication and viewer");
  auto control=OwnedDescriptor(control_fd);
  NativeSessionGateway gateway(std::move(runtime),std::move(datagram),std::move(control),source,
                               4096,worker_timeout,std::chrono::milliseconds(100),
                               publication?std::function<void(const SessionCycleSnapshot&)>(
                                 [&](const auto& cycle){publication->publish(cycle);}):nullptr,
                               []{},std::move(recording_hooks),display?std::function<void(const SessionOutputItem&)>(
                                 [&](const auto& item){display->ingest(item);}):nullptr,summaries,std::move(manus_hooks));
#ifdef TIANJI_WITH_GLFW
  std::unique_ptr<ViewerWindow> window;
  if(native_viewer) window=std::make_unique<ViewerWindow>(model,height,[&](auto action){
    if(!gateway.request_viewer_action(action)) throw std::runtime_error("viewer action could not be submitted");
  });
#endif
  std::cout<<"native_session_gateway_ready"<<(publication?" publication=cpp":"")
           <<(recording?" recording=cpp":"")<<(native_viewer?" viewer=cpp":"")
           <<(summaries?" diagnostics=summary":"")<<(hand_launch?" hands=cpp":"")<<'\n'<<std::flush;
  gateway.start();
#ifdef TIANJI_WITH_GLFW
  if(window) {
    const bool overlay=manifest.value("spark_overlay",false);
    while(true) {
      const auto state=gateway.snapshot();
      if(state.state.phase=="fault" || state.state.shutdown_complete || (!state.running && state.ticks>0)) break;
      const auto now=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
      const auto snapshot=display->snapshot();
      window->frame(snapshot,overlay?display->geometry(snapshot,now):ViewerGeometry{},now,
                    viewer_hud(state,manifest.value("xz_calibration",false))+
                    (manifest.value("common_x_reference",false)?"\nX reference: shared minimum robot TCP X":""));
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  } else
#endif
    gateway.wait();
  gateway.finish();
  const auto final=gateway.snapshot();
  if(!final.state.shutdown_complete || !gateway.output_complete())
    std::cerr<<"native_session_incomplete: state="<<final.state.phase
             <<" reason="<<final.state.reason<<" output_complete="<<gateway.output_complete()<<'\n';
  return final.state.shutdown_complete && gateway.output_complete()?0:1;
}
} // namespace

int main(int argc,char** argv) {
  try {
    if(argc!=5 || std::string(argv[1])!="--manifest" || std::string(argv[3])!="--control-fd") {
      std::cerr<<"usage: tianji_native_session_gateway --manifest FILE --control-fd FD\n";
      return 2;
    }
    const auto fd=std::stoll(argv[4]);
    if(fd<3 || fd>std::numeric_limits<int>::max()) throw std::invalid_argument("invalid control fd");
    return run(std::filesystem::absolute(argv[2]),static_cast<int>(fd));
  } catch(const std::exception& error) {
    std::cerr<<"native_session_gateway_error: "<<error.what()<<'\n';
    return 1;
  }
}
