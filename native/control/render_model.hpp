#pragma once
#include "session_health.hpp"
#include "worker_result.hpp"
#include "simulation_endpoint.hpp"
#include "hand_joint_names.hpp"
#include <mujoco/mujoco.h>
#include <algorithm>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace tianji_control {
// Display-only model/data. Never shares storage with the control executor and
// never emits commands. All MuJoCo access stays on the constructing thread.
class RenderModel {
 public:
  struct Snapshot {
    std::vector<double> qpos;
    std::array<std::array<double,3>,2> positions;
    std::array<std::array<double,4>,2> quaternions;
  };
  explicit RenderModel(const std::string& path) {
    if(mj_version()!=mjVERSION_HEADER) throw std::runtime_error("render MuJoCo ABI mismatch");
    if(path.empty() || path.find('\0')!=std::string::npos) throw std::invalid_argument("invalid render model path");
    char error[1024]{};
    model_.reset(mj_loadXML(path.c_str(),nullptr,error,sizeof(error)));
    if(!model_) throw std::runtime_error(std::string("render model load failed: ")+error);
    const auto names=arm_names();
    for(std::size_t i=0;i<names.size();++i) {
      const int id=lookup(mjOBJ_JOINT,names[i]);
      if(model_->jnt_type[id]!=mjJNT_HINGE && model_->jnt_type[id]!=mjJNT_SLIDE)
        throw std::invalid_argument("render arm requires scalar joints");
      addresses_[i]=model_->jnt_qposadr[id];
    }
    for(int s=0;s<2;++s) {
      const std::string suffix=s?"R":"L";
      targets_[s]=model_->body_mocapid[lookup(mjOBJ_BODY,"target_"+suffix)];
      sites_[s]=lookup(mjOBJ_SITE,"hand_tcp_frame_"+suffix);
      if(targets_[s]<0) throw std::invalid_argument("render target must be mocap body");
    }
    data_.reset(mj_makeData(model_.get()));
    if(!data_) throw std::runtime_error("render data allocation failed");
    mj_forward(model_.get(),data_.get());
  }
  void update(const ArmPair& q,const std::optional<WorkerResult>& native,bool active,std::int64_t now_ns,
              const std::optional<HandPair>& hands=std::nullopt) {
    check_owner();
    if(now_ns<0) throw std::invalid_argument("invalid render clock");
    for(const auto& side:q) for(double v:side) if(!std::isfinite(v))
      throw std::invalid_argument("nonfinite render joints");
    if(hands) {
      for(const auto& side:*hands) for(double v:side) if(!std::isfinite(v))
        throw std::invalid_argument("nonfinite render hand joints");
      if(!hand_addresses_) {
        std::array<int,40> addresses{};
        for(int s=0;s<2;++s) {
          const auto names=hand_joint_names(s);
          for(int j=0;j<20;++j) {
            auto name=names[j];
            int id=mj_name2id(model_.get(),mjOBJ_JOINT,name.c_str());
            if(id<0 && name.find("thumb_")==std::string::npos) {
              name.insert(name.find('_',2),"_finger");
              id=mj_name2id(model_.get(),mjOBJ_JOINT,name.c_str());
            }
            if(id<0 || (model_->jnt_type[id]!=mjJNT_HINGE && model_->jnt_type[id]!=mjJNT_SLIDE))
              throw std::invalid_argument("render model missing scalar hand joint: "+name);
            addresses[s*20+j]=model_->jnt_qposadr[id];
          }
        }
        hand_addresses_=addresses;
      }
    }
    const bool live=active && native && native->timestamp<=static_cast<std::uint64_t>(now_ns) &&
      static_cast<std::uint64_t>(now_ns)-native->timestamp<=200000000;
    if(live) for(const auto& side:native->arms) {
      for(double v:side.target_position) if(!std::isfinite(v)) throw std::invalid_argument("invalid render target");
      double norm=0;
      for(double v:side.target_quaternion) {
        if(!std::isfinite(v)) throw std::invalid_argument("invalid render orientation");
        norm+=v*v;
      }
      if(std::abs(std::sqrt(norm)-1.)>1e-3) throw std::invalid_argument("render quaternion must be unit length");
    }
    // Validate the complete display update before mutating even render storage.
    for(int s=0;s<2;++s) for(int j=0;j<7;++j) data_->qpos[addresses_[s*7+j]]=q[s][j];
    if(hands) for(int s=0;s<2;++s) for(int j=0;j<20;++j)
      data_->qpos[(*hand_addresses_)[s*20+j]]=(*hands)[s][j];
    mj_forward(model_.get(),data_.get());
    for(int s=0;s<2;++s) {
      auto* position=data_->mocap_pos+targets_[s]*3;
      auto* quaternion=data_->mocap_quat+targets_[s]*4;
      if(live) {
        const auto& a=native->arms[s];
        std::copy(a.target_position.begin(),a.target_position.end(),position);
        quaternion[0]=a.target_quaternion[3];
        for(int j=0;j<3;++j) quaternion[j+1]=a.target_quaternion[j];
      } else {
        std::copy_n(data_->site_xpos+sites_[s]*3,3,position);
        mju_mat2Quat(quaternion,data_->site_xmat+sites_[s]*9);
      }
    }
    mj_forward(model_.get(),data_.get());
  }
  Snapshot snapshot() const {
    check_owner();Snapshot out;
    out.qpos.assign(data_->qpos,data_->qpos+model_->nq);
    for(int s=0;s<2;++s) {
      std::copy_n(data_->mocap_pos+targets_[s]*3,3,out.positions[s].begin());
      std::copy_n(data_->mocap_quat+targets_[s]*4,4,out.quaternions[s].begin());
    }
    return out;
  }
  // Scene objects are renderer-owned; no mutable model/data pointer escapes.
  // Call make_scene on a default-initialized scene and free it with mjv_freeScene.
  void make_scene(mjvScene& scene,int capacity) const {
    check_owner();
    if(capacity<=0 || capacity>100000 || scene.maxgeom!=0)
      throw std::invalid_argument("render scene must be empty and bounded");
    mjv_makeScene(model_.get(),&scene,capacity);
  }
  void update_scene(mjvScene& scene,const mjvOption& options,mjvCamera& camera) {
    check_owner();
    if(scene.maxgeom<=0) throw std::invalid_argument("render scene is not initialized");
    mjv_updateScene(model_.get(),data_.get(),&options,nullptr,&camera,mjCAT_ALL,&scene);
  }
  void make_context(mjrContext& context) const {
    check_owner();mjr_makeContext(model_.get(),&context,mjFONTSCALE_100);
  }
  void default_camera(mjvCamera& camera) const {
    check_owner();mjv_defaultFreeCamera(model_.get(),&camera);
  }
  void move_camera(mjtMouse action,double dx,double dy,mjvScene& scene,mjvCamera& camera) const {
    check_owner();mjv_moveCamera(model_.get(),action,dx,dy,&scene,&camera);
  }
 private:
  struct ModelDelete {void operator()(mjModel* value) const {mj_deleteModel(value);}};
  struct DataDelete {void operator()(mjData* value) const {mj_deleteData(value);}};
  int lookup(mjtObj kind,const std::string& name) const {
    const int id=mj_name2id(model_.get(),kind,name.c_str());
    if(id<0) throw std::invalid_argument("render model missing: "+name);
    return id;
  }
  void check_owner() const {
    if(std::this_thread::get_id()!=owner_) throw std::logic_error("render accessed outside owner thread");
  }
  const std::thread::id owner_=std::this_thread::get_id();
  std::unique_ptr<mjModel,ModelDelete> model_;
  std::unique_ptr<mjData,DataDelete> data_;
  std::array<int,14> addresses_{};
  std::optional<std::array<int,40>> hand_addresses_;
  std::array<int,2> targets_{},sites_{};
};
} // namespace tianji_control
