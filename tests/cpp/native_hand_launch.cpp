#include "../../native/control/hand_launch_config.hpp"
#include <iostream>
int main(){try{
  nlohmann::json input;std::cin>>input;
  const auto c=tianji_control::parse_hand_launch(input,9);
  if(!c){std::cout<<"disabled";return 0;}
  std::cout<<c->fd<<' '<<c->domain.run<<' '<<c->domain.source.instance<<' '<<c->command.size();
}catch(const std::exception& e){std::cerr<<e.what();return 1;}}
