#pragma once
#include "session_output_bridge.hpp"
#include "tianji_mapped_palm/pico_teleop_protocol.hpp"
#include <array>
#include <mutex>

namespace tianji_control {
struct ViewerMarker {
  std::string label;
  std::array<double,3> position{};
  std::array<double,4> color{};
  std::optional<std::array<double,9>> rotation;
};
struct ViewerGeometry {
  std::vector<ViewerMarker> markers;
  std::vector<std::array<std::array<double,3>,2>> bones;
};
// Latest-only DISPLAY storage. Input/recording queues are never dropped here.
// No rendering while holding the mutex, and never called on the control thread.
class ViewerState {
 public:
  struct Snapshot {
    std::optional<ArmPair> feedback;
    std::optional<HandPair> hand_feedback;
    std::optional<WorkerResult> native;
    std::optional<tianji_mapped_palm::PicoTeleopFrame> raw;
    std::int64_t raw_ns=0,epoch=0;
    std::uint64_t raw_sequence=0;
    bool active=false;
    std::string operator_report;
  };
  explicit ViewerState(bool mapped):mapped_(mapped) {}
  void ingest(const SessionOutputItem& item) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(const auto* reply=std::get_if<SessionReply>(&item)) {
      state_.operator_report="Last action: "+reply->action+(reply->outcome.accepted?" OK: ":" REJECTED: ")+
                             reply->outcome.reason.substr(0,512);
      return;
    }
    if(const auto* raw=std::get_if<SessionRawSnapshot>(&item)) {
      if(mapped_ || raw->sequence<=state_.raw_sequence) return;
      auto decoded=tianji_mapped_palm::decodePicoTeleopPacket(raw->bytes.data(),raw->bytes.size());
      if(decoded.frame) {state_.raw=std::move(decoded.frame);state_.raw_ns=raw->received_ns;state_.raw_sequence=raw->sequence;}
      return;
    }
    const auto* c=std::get_if<SessionCycleSnapshot>(&item);
    if(!c || c->snapshot.state.epoch<state_.epoch) return;
    state_.feedback=c->snapshot.feedback;
    state_.hand_feedback=c->snapshot.hand_feedback;
    if(c->result) {
      const auto& result=*c->result;
      bool valid=result.tick>0 && result.timestamp>0;
      for(const auto& arm:result.arms) {
        double norm=0.;
        for(double v:arm.target_position) valid=valid && std::isfinite(v);
        for(double v:arm.target_quaternion) {valid=valid && std::isfinite(v);norm+=v*v;}
        valid=valid && std::abs(std::sqrt(norm)-1.)<=1e-3;
      }
      if(mapped_ && result.height_present)
        for(double v:result.height_offsets) valid=valid && std::isfinite(v) && std::abs(v)<=1.;
      if(mapped_ && result.x_present)
        for(double v:result.x_offsets) valid=valid && std::isfinite(v) && std::abs(v)<=1.;
      // Invalid diagnostics never replace a previously valid target.
      if(!valid) return;
    }
    const bool changed=c->snapshot.state.epoch>state_.epoch;
    // A duplicate result must not clear an applied skeleton or change its
    // diagnostic activity. Match MappedPalmOverlay's whole-cycle rejection.
    if(mapped_ && c->result && !changed && state_.native &&
       c->result->tick<=state_.native->tick) return;
    if(changed && mapped_) state_.raw.reset();
    if(changed) state_.native.reset();
    state_.epoch=c->snapshot.state.epoch;state_.active=c->snapshot.state.phase=="teleop";
    if(c->result && c->result->tick>0 && c->result->timestamp>0 &&
       (!state_.native || c->result->tick>state_.native->tick)) state_.native=c->result;
    if(!mapped_) return;
    if(!state_.active) {state_.raw.reset();state_.native.reset();return;}
    if(c->result && c->request && !c->request->packet.empty()) {
      auto raw=tianji_mapped_palm::decodePicoTeleopPacket(c->request->packet.data(),c->request->packet.size());
      if(raw.frame && raw.frame->sequence==c->result->applied_sequence && raw.frame->tracking_epoch==c->result->applied_epoch) {
        state_.raw=std::move(raw.frame);state_.raw_ns=c->request->received_ns;
      }
    }
    if(state_.native && state_.raw && (state_.raw->sequence!=state_.native->applied_sequence ||
       state_.raw->tracking_epoch!=state_.native->applied_epoch)) state_.raw.reset();
  }
  Snapshot snapshot() const {std::lock_guard<std::mutex> lock(mutex_);return state_;}
  ViewerGeometry geometry(std::int64_t now) const {return geometry(snapshot(),now);}
  ViewerGeometry geometry(const Snapshot& s,std::int64_t now) const {
    ViewerGeometry out;
    if(mapped_ && !s.active) return out;
    auto fresh=[&](std::uint64_t stamp,std::uint64_t age){return now>=0 && stamp<=std::uint64_t(now) && std::uint64_t(now)-stamp<=age;};
    const bool raw_fresh=s.raw && s.raw_ns>=0 && fresh(s.raw_ns,mapped_?50000000:200000000);
    const bool applied=raw_fresh && (!mapped_ || (s.native && s.native->input_live && fresh(s.native->timestamp,50000000)));
    auto point=[](const Eigen::Vector3d& v){return std::array<double,3>{v[0],v[1],v[2]};};
    auto rotation=[](const Eigen::Matrix3d& m){std::array<double,9> out{};for(int i=0;i<3;++i)for(int j=0;j<3;++j)out[i*3+j]=m(i,j);return out;};
    if(applied) {
      const auto& raw=*s.raw;
      const auto& skeleton=raw.upper_limb_skeleton;
      if(skeleton.valid) {
        for(const auto edge:std::array<std::array<int,2>,7>{{{0,4},{0,1},{1,2},{2,3},{4,5},{5,6},{6,7}}})
          out.bones.push_back({point(skeleton.points[edge[0]]),point(skeleton.points[edge[1]])});
        const char* names[]={"shoulder","elbow","wrist","palm"};
        for(int i=0;i<8;++i) {
          ViewerMarker m{std::string(mapped_?"Applied corrected ":"TJVR corrected ")+names[i%4]+(i<4?" left":" right"),
            point(skeleton.points[i]),{.1,.8,1.,.8},std::nullopt};
          if(!mapped_ && i%4==3 && skeleton.rotations_valid) m.rotation=rotation(skeleton.rotations[i].toRotationMatrix());
          out.markers.push_back(std::move(m));
        }
      }
      if(!mapped_) for(int side=0;side<2;++side) {
        const auto& pose=side?raw.right:raw.left;
        out.markers.push_back({std::string("TJVR packet target ")+(side?"right":"left"),point(pose.position),
          {.9,.2,.9,.8},rotation(pose.rotation)});
      }
    }
    if(s.native) for(int side=0;side<2;++side) {
      const auto& a=s.native->arms[side];const auto& q=a.target_quaternion;
      const bool stale=!fresh(s.native->timestamp,200000000);
      out.markers.push_back({std::string(mapped_?"Mapped-palm":"SPARK")+" IK target "+(side?"right":"left")+(stale?" [stale]":""),
        a.target_position,stale?std::array<double,4>{.5,.5,.5,1.}:std::array<double,4>{1.,.75,.05,1.},
        rotation(Eigen::Quaterniond(q[3],q[0],q[1],q[2]).normalized().toRotationMatrix())});
    }
    if(mapped_ && applied && s.native->height_present) for(int side=0;side<2;++side) {
      auto p=point(s.raw->upper_limb_skeleton.points[side?7:3]);p[2]+=s.native->height_offsets[side];
      if(s.native->x_present) p[0]+=s.native->x_offsets[side];
      out.markers.push_back({std::string("Z calibrated palm position ")+(side?"right":"left"),p,{.9,.2,.9,1.},std::nullopt});
    }
    return out;
  }
 private:
  const bool mapped_;
  mutable std::mutex mutex_;
  Snapshot state_;
};
} // namespace tianji_control
