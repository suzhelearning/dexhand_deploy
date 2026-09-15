#include "../../native/control/render_model.hpp"
#include <nlohmann/json.hpp>
#include <iostream>
int main(int argc,char** argv) {
  try {
    if(argc!=2) return 2;
    tianji_control::RenderModel model(argv[1]);
    mjvScene scene; mjv_defaultScene(&scene);
    model.make_scene(scene,4096);
    struct Cleanup {mjvScene* scene;~Cleanup(){mjv_freeScene(scene);}} cleanup{&scene};
    mjvOption options;mjv_defaultOption(&options);
    mjvCamera camera;mjv_defaultCamera(&camera);
    std::string line;
    while(std::getline(std::cin,line)) {
      const auto row=nlohmann::json::parse(line);
      const auto q=row.at("q").get<tianji_control::ArmPair>();
      std::optional<tianji_control::WorkerResult> result;
      if(row.at("native").is_object()) {
        result.emplace();result->timestamp=row.at("native").at("timestamp_ns");
        for(int s=0;s<2;++s) {
          result->arms[s].target_position=row.at("native").at(s?"right":"left").at("target_position").get<std::array<double,3>>();
          result->arms[s].target_quaternion=row.at("native").at(s?"right":"left").at("target_quaternion_xyzw").get<std::array<double,4>>();
        }
      }
      std::optional<tianji_control::HandPair> hands;
      if(row.contains("hands")) hands=row.at("hands").get<tianji_control::HandPair>();
      model.update(q,result,row.at("active"),row.at("now_ns"),hands);
      const auto view=model.snapshot();
      model.update_scene(scene,options,camera);
      if(scene.ngeom<=0 || model.snapshot().qpos!=view.qpos)
        throw std::runtime_error("scene generation failed or mutated joint state");
      std::cout<<nlohmann::json({{"qpos",view.qpos},{"positions",view.positions},{"quaternions",view.quaternions}}).dump()<<'\n';
      auto invalid=q;invalid[1][6]=std::numeric_limits<double>::quiet_NaN();
      bool rejected=false;
      try {model.update(invalid,result,true,1001);}catch(const std::invalid_argument&){rejected=true;}
      if(!rejected || model.snapshot().qpos!=view.qpos) throw std::runtime_error("invalid update changed render joints");
      if(hands) {
        auto bad=hands;(*bad)[1][19]=std::numeric_limits<double>::quiet_NaN();
        rejected=false;
        try {model.update(q,result,true,1001,bad);}catch(const std::invalid_argument&){rejected=true;}
        if(!rejected || model.snapshot().qpos!=view.qpos) throw std::runtime_error("invalid hand changed render state");
      }
      if(result) {
        auto bad=result;bad->arms[1].target_quaternion.fill(0.);
        rejected=false;
        try {model.update(q,bad,true,1001);}catch(const std::invalid_argument&){rejected=true;}
        if(!rejected || model.snapshot().positions!=view.positions) throw std::runtime_error("invalid target changed render state");
      }
      rejected=false;
      std::thread wrong([&]{try{model.snapshot();}catch(const std::logic_error&){rejected=true;}});
      wrong.join();
      if(!rejected) throw std::runtime_error("render allowed cross-thread access");
    }
  }catch(const std::exception& e){std::cerr<<e.what();return 1;}
}
