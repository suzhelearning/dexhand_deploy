#include "native/control/worker_result_json.hpp"
#include <iostream>
int main() {
  try {
    std::string line;
    while (std::getline(std::cin,line)) {
      const auto input=nlohmann::json::parse(line);
      const auto bytes=input.at("bytes").get<std::vector<std::uint8_t>>();
      std::cout << tianji_control::worker_result_json(bytes,input.at("mapped").get<bool>()).dump() << '\n';
    }
    return 0;
  } catch(const std::exception& e) { std::cerr << e.what(); return 1; }
}
