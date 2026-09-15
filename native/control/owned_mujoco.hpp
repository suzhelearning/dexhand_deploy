#pragma once
#include <mujoco/mujoco.h>
#include <array>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace tianji_control {
// Simulation storage/FK only. The coordinator and execution guard must authorize
// commands before apply(). No networking, dynamics step, renderer or Python owner.
class OwnedMujoco {
 public:
  using Batch=std::vector<std::optional<std::vector<double>>>;
  struct BodyPose { std::array<double,3> position{}; std::array<double,4> quaternion{}; };
  struct Snapshot { std::vector<double> qpos; std::vector<std::vector<double>> groups; std::vector<BodyPose> bodies; };
  OwnedMujoco(const std::string& xml,const std::vector<std::vector<std::string>>& groups,
              const std::vector<std::string>& bodies={},
              const std::unordered_map<std::string,std::string>& joint_aliases={}) {
    if(mj_version()!=mjVERSION_HEADER) throw std::runtime_error("MuJoCo runtime/header version mismatch");
    if(xml.empty() || xml.find('\0')!=std::string::npos || groups.size()>4 || bodies.size()>64)
      throw std::invalid_argument("invalid MuJoCo configuration");
    char error[1024]{};
    model_.reset(mj_loadXML(xml.c_str(),nullptr,error,sizeof(error)));
    if(!model_) throw std::runtime_error(std::string("MuJoCo model load failed: ")+error);
    std::vector<bool> used(model_->nq,false);
    for(const auto& group:groups) {
      if(group.size()>static_cast<std::size_t>(model_->njnt)) throw std::invalid_argument("oversized joint group");
      std::vector<int> addresses;
      for(const auto& name:group) {
        int id;
        // Match the reference executor: canonical first, alias only if absent.
        const auto alias=joint_aliases.find(name);
        if(alias!=joint_aliases.end() && !name.empty() && name.size()<=256 && name.find('\0')==std::string::npos &&
           mj_name2id(model_.get(),mjOBJ_JOINT,name.c_str())<0)
          id=lookup(mjOBJ_JOINT,alias->second);
        else id=lookup(mjOBJ_JOINT,name);
        if(model_->jnt_type[id]!=mjJNT_HINGE && model_->jnt_type[id]!=mjJNT_SLIDE)
          throw std::invalid_argument("scalar joint required");
        const int address=model_->jnt_qposadr[id];
        if(address<0 || address>=model_->nq || used[address]) throw std::invalid_argument("duplicate or invalid joint address");
        used[address]=true; addresses.push_back(address);
      }
      groups_.push_back(std::move(addresses));
    }
    for(const auto& name:bodies) bodies_.push_back(lookup(mjOBJ_BODY,name));
    data_.reset(mj_makeData(model_.get()));
    if(!data_) throw std::runtime_error("MuJoCo data allocation failed");
    mj_forward(model_.get(),data_.get());
  }
  OwnedMujoco(const OwnedMujoco&)=delete;
  OwnedMujoco& operator=(const OwnedMujoco&)=delete;
  template<std::size_t G,std::size_t J>
  void apply(const std::array<std::array<double,J>,G>& batch) {
    check_owner();
    if(groups_.size()!=G) throw std::invalid_argument("joint batch count mismatch");
    for(std::size_t g=0;g<G;++g) {
      if(groups_[g].size()!=J) throw std::invalid_argument("joint group length mismatch");
      for(double q:batch[g]) if(!std::isfinite(q)) throw std::invalid_argument("nonfinite joint position");
    }
    for(std::size_t g=0;g<G;++g) for(std::size_t j=0;j<J;++j) data_->qpos[groups_[g][j]]=batch[g][j];
    mj_forward(model_.get(),data_.get());
  }
  void apply(const Batch& batch) {
    check_owner();
    if(batch.size()!=groups_.size()) throw std::invalid_argument("joint batch count mismatch");
    // Entire bilateral/hand batch is validated before any write. Null groups hold.
    for(std::size_t g=0;g<batch.size();++g) if(batch[g]) {
      if(batch[g]->size()!=groups_[g].size()) throw std::invalid_argument("joint group length mismatch");
      for(double q:*batch[g]) if(!std::isfinite(q)) throw std::invalid_argument("nonfinite joint position");
    }
    for(std::size_t g=0;g<batch.size();++g) if(batch[g])
      for(std::size_t i=0;i<groups_[g].size();++i) data_->qpos[groups_[g][i]]=(*batch[g])[i];
    mj_forward(model_.get(),data_.get()); // Preserve reference direct-qpos semantics; no mj_step.
  }
  template<std::size_t A,std::size_t H>
  void apply_mixed(const std::array<std::array<double,A>,2>& arms,
                   const std::array<std::optional<std::array<double,H>>,2>& hands) {
    check_owner();
    if(groups_.size()!=4) throw std::invalid_argument("arm/hand group count mismatch");
    for(std::size_t s=0;s<2;++s) {
      if(groups_[s].size()!=A || groups_[s+2].size()!=H)
        throw std::invalid_argument("arm/hand group length mismatch");
      for(double q:arms[s]) if(!std::isfinite(q)) throw std::invalid_argument("nonfinite arm position");
      if(hands[s]) for(double q:*hands[s]) if(!std::isfinite(q))
        throw std::invalid_argument("nonfinite hand position");
    }
    // One owner, complete validation before mutation, one FK for all four groups.
    // Fixed arrays avoid per-tick vector construction in the mixed-DOF path.
    for(std::size_t s=0;s<2;++s) {
      for(std::size_t j=0;j<A;++j) data_->qpos[groups_[s][j]]=arms[s][j];
      if(hands[s]) for(std::size_t j=0;j<H;++j) data_->qpos[groups_[s+2][j]]=(*hands[s])[j];
    }
    mj_forward(model_.get(),data_.get());
  }
  Snapshot snapshot() const {
    check_owner();
    Snapshot out;
    out.qpos.assign(data_->qpos,data_->qpos+model_->nq);
    for(const auto& group:groups_) {
      std::vector<double> q; q.reserve(group.size());
      for(int address:group) q.push_back(data_->qpos[address]);
      out.groups.push_back(std::move(q));
    }
    for(int body:bodies_) {
      BodyPose p;
      for(int i=0;i<3;++i) p.position[i]=data_->xpos[body*3+i];
      for(int i=0;i<4;++i) p.quaternion[i]=data_->xquat[body*4+i];
      out.bodies.push_back(p);
    }
    return out; // No mutable model/data pointers cross the owner boundary.
  }
  template<std::size_t N> std::array<double,N> group_positions(std::size_t group) const {
    check_owner();
    if(group>=groups_.size() || groups_[group].size()!=N) throw std::invalid_argument("fixed joint group size/index mismatch");
    std::array<double,N> out{};
    for(std::size_t j=0;j<N;++j) out[j]=data_->qpos[groups_[group][j]];
    return out; // Feedback hot path: no full qpos/body snapshot or temporary vectors.
  }
  std::array<double,2> zero_group_site_heights(const std::array<std::string,2>& sites) const {
    check_owner();
    std::unique_ptr<mjData,DataDelete> reference(mj_makeData(model_.get()));
    if(!reference) throw std::runtime_error("MuJoCo reference allocation failed");
    for(const auto& group:groups_) for(int address:group) reference->qpos[address]=0.;
    mj_forward(model_.get(),reference.get());
    std::array<double,2> result{};
    for(int s=0;s<2;++s) {
      result[s]=reference->site_xpos[lookup(mjOBJ_SITE,sites[s])*3+2];
      if(!std::isfinite(result[s])) throw std::runtime_error("nonfinite horizontal site height");
    }
    return result; // Separate mjData: never overwrite current simulation command/FK.
  }
  std::array<std::array<double,2>,2> forward_site_xz(const std::array<std::string,2>& sites) const {
    check_owner();
    std::unique_ptr<mjData,DataDelete> reference(mj_makeData(model_.get()));
    if(!reference) throw std::runtime_error("MuJoCo reference allocation failed");
    for(const auto& group:groups_) for(int address:group) reference->qpos[address]=0.;
    for(int s=0;s<2;++s) reference->qpos[groups_.at(s).at(1)]=-std::acos(-1.)/2.;
    mj_forward(model_.get(),reference.get());
    std::array<std::array<double,2>,2> out{};
    for(int s=0;s<2;++s) {
      const auto* p=reference->site_xpos+lookup(mjOBJ_SITE,sites[s])*3;
      out[s]={p[0],p[2]};
      for(double v:out[s]) if(!std::isfinite(v)) throw std::runtime_error("nonfinite forward TCP");
    }
    return out;
  }
 private:
  struct ModelDelete { void operator()(mjModel* m) const { mj_deleteModel(m); } };
  struct DataDelete { void operator()(mjData* d) const { mj_deleteData(d); } };
  int lookup(mjtObj type,const std::string& name) const {
    if(name.empty() || name.size()>256 || name.find('\0')!=std::string::npos) throw std::invalid_argument("invalid MuJoCo object name");
    const int id=mj_name2id(model_.get(),type,name.c_str());
    if(id<0) throw std::invalid_argument("missing MuJoCo object: "+name);
    return id;
  }
  void check_owner() const {
    if(std::this_thread::get_id()!=owner_) throw std::logic_error("MuJoCo accessed outside owner thread");
  }
  const std::thread::id owner_=std::this_thread::get_id();
  std::unique_ptr<mjModel,ModelDelete> model_;
  std::unique_ptr<mjData,DataDelete> data_; // Destroy data before its model, including constructor failures.
  std::vector<std::vector<int>> groups_;
  std::vector<int> bodies_;
};
} // namespace tianji_control
