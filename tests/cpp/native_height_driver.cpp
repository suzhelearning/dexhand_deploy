#include "../../native/control/height_calibration.hpp"
#include <nlohmann/json.hpp>
#include <iostream>
using namespace tianji_control;
int main() {
  try {
    HeightCalibration height({1.1,1.1});
    for(std::string line;std::getline(std::cin,line);) {
      auto row=nlohmann::json::parse(line); auto now=row.at("now").get<std::int64_t>();
      if(row.at("op")=="begin") height.begin(now);
      else {
        if(row.contains("seq")) {
          RawProgress p; p.accepted=true; p.sequence=row.at("seq"); p.epoch=row.value("epoch",9);
          p.skeleton_valid=p.rotations_valid=row.value("valid",true);
          p.palms={{{.3,.35,row.value("z",1.2)},{.3,-.35,row.value("z",1.2)+.02}}};
          height.add(p,row.value("stamp",now),now);
        }
        if(height.tick(now)) height.commit();
      }
      const auto s=height.status();
      nlohmann::json out={{"state",s.state},{"count",s.count}};
      out["offsets"]=s.offsets?nlohmann::json(*s.offsets):nlohmann::json(nullptr);
      std::cout<<out.dump()<<'\n';
    }
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
