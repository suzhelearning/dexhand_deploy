// Compile this harness against the ORIGINAL project's libtianji_qp_ik.a.
// It intentionally never links the port, and only runs model-state control.
#include "tianji_qp_ik/controller.hpp"
#include "tianji_qp_ik/cartesian_otg.hpp"
#include <Eigen/Geometry>
#include <iostream>
#include <iomanip>

int main(int argc,char**argv) {
  using namespace tianji_qp_ik;
  if(argc<3) return 2;
  const bool combined=argc>3;
  auto config=loadConfig(argv[1]);
  if(combined) {
    config.controller.initial_left_q_rad<<55,-65,-70,-60,60,0,0;
    config.controller.initial_right_q_rad<<-55,-65,70,-60,-60,0,0;
    config.controller.initial_left_q_rad*=M_PI/180;
    config.controller.initial_right_q_rad*=M_PI/180;
  }
  MujocoRobot robot(argv[2]);
  robot.setArmState(ArmSide::kLeft,config.controller.initial_left_q_rad,Vec7::Zero());
  robot.setArmState(ArmSide::kRight,config.controller.initial_right_q_rad,Vec7::Zero());
  robot.forward();
  DualArmController controller(robot,config);
  DualArmTargets home; home.left=robot.tcpPose(ArmSide::kLeft);home.right=robot.tcpPose(ArmSide::kRight);
  TargetManager manager(config,home); manager.setMode(TargetMode::kManual,1.0);
  CartesianReferenceGenerator left(config.cartesian_otg,.005),right(config.cartesian_otg,.005);
  left.reset(home.left);right.reset(home.right);
  std::cout<<std::setprecision(17);
  for(int tick=0;tick<600;++tick) {
    const double t=tick*.005;
    auto targets=home;
    targets.left.position+=Eigen::Vector3d(.025*std::sin(t),.02*std::sin(.7*t),.015*std::sin(1.3*t));
    targets.right.position+=Eigen::Vector3d(-.02*std::sin(t),.015*std::sin(.8*t),.02*std::sin(.9*t));
    targets.left.rotation=Eigen::AngleAxisd(.08*std::sin(t),Eigen::Vector3d(1,.4,.7).normalized()).toRotationMatrix()*home.left.rotation;
    targets.right.rotation=Eigen::AngleAxisd(.06*std::sin(t),Eigen::Vector3d(.3,1,.6).normalized()).toRotationMatrix()*home.right.rotation;
    if(combined) {
      targets.left.position=home.left.position+Eigen::AngleAxisd(-1.5708,Eigen::Vector3d::UnitX())*Eigen::Vector3d(.02,-.02,.02);
      targets.right.position=home.right.position+Eigen::AngleAxisd(1.5708,Eigen::Vector3d::UnitX())*Eigen::Vector3d(.02,-.02,.02);
      targets.left.rotation=home.left.rotation*Eigen::AngleAxisd(8*M_PI/180,Eigen::Vector3d::UnitZ()).toRotationMatrix();
      targets.right.rotation=home.right.rotation*Eigen::AngleAxisd(8*M_PI/180,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    }
    if(tick%2==0) manager.setManualTargets(targets.left,targets.right,1+t,1+t);
    const auto sampled=manager.sample(1+t);
    DualArmReferences refs;
    refs.left=left.update(sampled.left,sampled.left_twist,sampled.left_stale,.005);
    refs.right=right.update(sampled.right,sampled.right_twist,sampled.right_stale,.005);
    auto result=controller.step(refs,.005);
    std::cout<<tick<<' '<<result.left.accepted<<' '<<result.right.accepted;
    for(auto side:{ArmSide::kLeft,ArmSide::kRight}) {
      const auto q=robot.armPosition(side);
      for(int j=0;j<7;++j) std::cout<<' '<<q[j];
    }
    std::cout<<'\n';
  }
}
