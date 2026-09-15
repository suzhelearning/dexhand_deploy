#pragma once
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include "render_model.hpp"
#include "viewer_state.hpp"
#include "session_gateway_wire.hpp"

namespace tianji_control {
inline std::optional<GatewayAction> viewer_key(int key,int action,bool height) {
  if(action!=GLFW_PRESS) return std::nullopt;
  switch(key) {
    case GLFW_KEY_S:return GatewayAction::start;
    case GLFW_KEY_H:return GatewayAction::return_home;
    case GLFW_KEY_R:return GatewayAction::rearm;
    case GLFW_KEY_Q:return GatewayAction::shutdown;
    case GLFW_KEY_C:if(height) return GatewayAction::calibrate;break;
  }
  return std::nullopt;
}
// The native executable's MAIN thread owns GLFW/OpenGL and this entire object.
// Operator callbacks only submit intents; they never write joints or clear faults.
class ViewerWindow {
 public:
  ViewerWindow(const std::string& path,bool height,std::function<void(GatewayAction)> action)
      :height_(height),action_(std::move(action)) {
    if(!action_) throw std::invalid_argument("viewer operator callback required");
    mjv_defaultScene(&scene_);mjr_defaultContext(&context_);mjv_defaultOption(&options_);
    if(!glfwInit()) throw std::runtime_error("native viewer: GLFW initialization failed (display unavailable)");
    initialized_=true;
    try {
      window_=glfwCreateWindow(1200,900,"Tianji native teleop | s start h Home r rearm c calibrate q exit",nullptr,nullptr);
      if(!window_) throw std::runtime_error("native viewer: window creation failed");
      glfwMakeContextCurrent(window_);glfwSwapInterval(1);
      model_=std::make_unique<RenderModel>(path);
      model_->make_scene(scene_,4096);model_->make_context(context_);model_->default_camera(camera_);
      glfwSetWindowUserPointer(window_,this);
      glfwSetKeyCallback(window_,[](GLFWwindow* w,int key,int,int action,int){
        auto* self=static_cast<ViewerWindow*>(glfwGetWindowUserPointer(w));
        if(auto intent=viewer_key(key,action,self->height_)) self->submit(*intent);
      });
      glfwSetScrollCallback(window_,[](GLFWwindow* w,double,double dy){
        auto* self=static_cast<ViewerWindow*>(glfwGetWindowUserPointer(w));
        self->model_->move_camera(mjMOUSE_ZOOM,0.,-.05*dy,self->scene_,self->camera_);
      });
      glfwGetCursorPos(window_,&mouse_x_,&mouse_y_);
    }catch(...) {cleanup();throw;}
  }
  ~ViewerWindow(){cleanup();}
  ViewerWindow(const ViewerWindow&)=delete;
  ViewerWindow& operator=(const ViewerWindow&)=delete;
  void frame(const ViewerState::Snapshot& state,const ViewerGeometry& overlay,std::int64_t now,
             const std::string& status="") {
    if(!error_.empty()) throw std::runtime_error(error_);
    glfwPollEvents();
    if(glfwWindowShouldClose(window_) && !closing_) {closing_=true;submit(GatewayAction::shutdown);}
    if(!error_.empty()) throw std::runtime_error(error_);
    double x,y;glfwGetCursorPos(window_,&x,&y);
    int width,height;glfwGetFramebufferSize(window_,&width,&height);
    if(height>0 && (glfwGetMouseButton(window_,GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS ||
                    glfwGetMouseButton(window_,GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS)) {
      const auto mode=glfwGetMouseButton(window_,GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS?mjMOUSE_MOVE_V:mjMOUSE_ROTATE_V;
      model_->move_camera(mode,(x-mouse_x_)/height,(y-mouse_y_)/height,scene_,camera_);
    }
    mouse_x_=x;mouse_y_=y;
    if(state.feedback) model_->update(*state.feedback,state.native,state.active,now,state.hand_feedback);
    model_->update_scene(scene_,options_,camera_);
    append_overlay(overlay);
    if(width>0 && height>0) {
      const mjrRect viewport{0,0,width,height};
      mjr_render(viewport,&scene_,&context_);
      if(!status.empty()) {
        const auto text=status+"\n"+state.operator_report;
        mjr_overlay(mjFONT_NORMAL,mjGRID_TOPLEFT,viewport,text.c_str(),nullptr,&context_);
      }
      glfwSwapBuffers(window_);
    }
  }
 private:
  void submit(GatewayAction action) noexcept {
    try{action_(action);}catch(const std::exception& e){error_=e.what();}
    catch(...){error_="native viewer operator submission failed";}
  }
  mjvGeom* geom(mjtGeom kind,const std::array<double,3>& p,const std::array<double,4>& c,const std::string& label="") {
    if(scene_.ngeom>=scene_.maxgeom) return nullptr;
    auto* g=&scene_.geoms[scene_.ngeom++];
    const mjtNum size[3]={.013,.013,.013},identity[9]={1,0,0,0,1,0,0,0,1};
    float color[4];for(int i=0;i<4;++i)color[i]=static_cast<float>(c[i]);
    mjv_initGeom(g,kind,size,p.data(),identity,color);
    std::strncpy(g->label,label.c_str(),sizeof(g->label)-1);g->label[sizeof(g->label)-1]='\0';return g;
  }
  void append_overlay(const ViewerGeometry& overlay) {
    for(const auto& bone:overlay.bones) {
      auto* g=geom(mjGEOM_CAPSULE,bone[0],{.1,.8,1.,.65});if(!g)return;
      mjv_connector(g,mjGEOM_CAPSULE,.004,bone[0].data(),bone[1].data());
    }
    for(const auto& marker:overlay.markers) {
      if(!geom(mjGEOM_SPHERE,marker.position,marker.color,marker.label))return;
      if(marker.rotation) for(int axis=0;axis<3;++axis) {
        const std::array<std::array<double,4>,3> colors{{{1,0,0,1},{0,1,0,1},{0,.3,1,1}}};
        auto* g=geom(mjGEOM_CAPSULE,marker.position,colors[axis]);if(!g)return;
        auto end=marker.position;for(int j=0;j<3;++j)end[j]+=.1*(*marker.rotation)[j*3+axis];
        mjv_connector(g,mjGEOM_CAPSULE,.003,marker.position.data(),end.data());
      }
    }
  }
  void cleanup() noexcept {
    if(window_)glfwMakeContextCurrent(window_);
    mjr_freeContext(&context_);mjv_freeScene(&scene_);model_.reset();
    if(window_) {glfwDestroyWindow(window_);window_=nullptr;}
    if(initialized_) {glfwTerminate();initialized_=false;}
  }
  bool height_,initialized_=false,closing_=false;
  std::function<void(GatewayAction)> action_;
  std::string error_;
  GLFWwindow* window_=nullptr;
  std::unique_ptr<RenderModel> model_;
  mjvScene scene_{};mjrContext context_{};mjvCamera camera_{};mjvOption options_{};
  double mouse_x_=0,mouse_y_=0;
};
} // namespace tianji_control
