#pragma once
#include <array>
#include <string>
#include <vector>
namespace tianji_control {
inline std::vector<std::string> hand_joint_names(int side) {
  constexpr std::array<const char*,20> suffixes{{
    "thumb_cmc_flex","thumb_cmc_abd","thumb_mcp","thumb_ip",
    "index_mcp_flex","index_mcp_abd","index_pip","index_dip",
    "middle_mcp_flex","middle_mcp_abd","middle_pip","middle_dip",
    "ring_mcp_flex","ring_mcp_abd","ring_pip","ring_dip",
    "pinky_mcp_flex","pinky_mcp_abd","pinky_pip","pinky_dip"}};
  std::vector<std::string> out;
  for(const auto* suffix:suffixes) out.push_back(std::string(side?"r_":"l_")+suffix);
  return out;
}
}
