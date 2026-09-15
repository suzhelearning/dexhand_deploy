#include "../../native/control/owned_mujoco.hpp"
#include <nlohmann/json.hpp>
#include <iostream>
#include <thread>
using namespace tianji_control;
int main(int argc,char** argv) {
  if(argc!=2) return 2;
  try {
    std::string line; std::getline(std::cin,line); auto config=nlohmann::json::parse(line);
    OwnedMujoco sim(argv[1],config.at("groups").get<std::vector<std::vector<std::string>>>(),
      config.at("bodies").get<std::vector<std::string>>(),
      config.value("aliases",std::unordered_map<std::string,std::string>{}));
    auto snapshot=[&] {
      auto s=sim.snapshot();
      nlohmann::json out={{"qpos",s.qpos},{"groups",s.groups},{"poses",nlohmann::json::array()}};
      for(const auto& p:s.bodies) out["poses"].push_back({{"position",p.position},{"quaternion_wxyz",p.quaternion}});
      return out;
    };
    std::cout<<snapshot()<<'\n';
    while(std::getline(std::cin,line)) {
      auto value=nlohmann::json::parse(line);
      bool accepted=true;
      try {
        if(value=="wrong_thread") {
          int rejected=0;
          std::thread other([&] {
            try { sim.snapshot(); } catch(const std::logic_error&) { ++rejected; }
            try { sim.apply({}); } catch(const std::logic_error&) { ++rejected; }
          });
          other.join(); if(rejected!=2) throw std::runtime_error("foreign thread accepted");
        } else if(value.is_object() && value.contains("mixed")) {
          const auto& batch=value.at("mixed");
          std::array<std::array<double,7>,2> arms{
            batch.at(0).get<std::array<double,7>>(),batch.at(1).get<std::array<double,7>>()};
          std::array<std::optional<std::array<double,20>>,2> hands;
          for(int s=0;s<2;++s) if(!batch.at(s+2).is_null())
            hands[s]=batch.at(s+2).get<std::array<double,20>>();
          sim.apply_mixed(arms,hands);
        } else {
          OwnedMujoco::Batch batch;
          if(value=="nan") {
            auto s=sim.snapshot();
            for(const auto& g:s.groups) batch.emplace_back(g);
            batch.front()->front()+=.123;
            batch.back()->back()=std::numeric_limits<double>::quiet_NaN();
          } else for(const auto& g:value) {
            if(g.is_null()) batch.emplace_back(std::nullopt);
            else batch.emplace_back(g.get<std::vector<double>>());
          }
          sim.apply(batch);
        }
      } catch(const std::invalid_argument&) { accepted=false; }
      auto out=snapshot(); out["accepted"]=accepted; std::cout<<out<<'\n';
    }
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
