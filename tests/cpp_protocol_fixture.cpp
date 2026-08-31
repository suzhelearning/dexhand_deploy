#include "tianji_teleop/protocol/json_parser.hpp"

#include <cmath>
#include <iostream>
#include <iterator>
#include <string>

namespace {
using tianji_teleop::protocol::field;
using tianji_teleop::protocol::require_exact_fields;
using tianji_teleop::protocol::StrictJsonParser;

void parse_target(const std::string &payload, const std::string &router, const std::string &side) {
  const auto root = StrictJsonParser::parse(payload);
  require_exact_fields(root, {
    "schema_version", "publisher_instance_id", "router_zid", "sequence",
    "timestamp_ns", "source_timestamp_ns", "source", "side", "frame_id",
    "position_m", "orientation_xyzw", "elbow_reference_direction"
  });
  if (field(root, "schema_version").as_uint("schema_version") != 1 ||
      field(root, "router_zid").as_string("router_zid") != router ||
      field(root, "side").as_string("side") != side ||
      field(root, "frame_id").as_string("frame_id") != (side == "left" ? "Base_L" : "Base_R")) {
    throw std::invalid_argument("target identity/frame mismatch");
  }
  const auto quaternion = tianji_teleop::protocol::vector_field(root, "orientation_xyzw", 4);
  const auto elbow = tianji_teleop::protocol::vector_field(root, "elbow_reference_direction", 3);
  (void)tianji_teleop::protocol::vector_field(root, "position_m", 3);
  const auto qnorm = std::sqrt(quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);
  const auto enorm = std::sqrt(elbow[0] * elbow[0] + elbow[1] * elbow[1] + elbow[2] * elbow[2]);
  if (qnorm < 0.999 || qnorm > 1.001 || enorm < 1e-8) throw std::invalid_argument("invalid target geometry");
  (void)field(root, "publisher_instance_id").as_string("publisher_instance_id");
  (void)field(root, "source").as_string("source");
  (void)field(root, "sequence").as_uint("sequence");
  (void)field(root, "timestamp_ns").as_uint("timestamp_ns");
  if (!field(root, "source_timestamp_ns").is_null()) (void)field(root, "source_timestamp_ns").as_uint("source_timestamp_ns");
}

void parse_command(const std::string &payload, const std::string &router, const std::string &side) {
  const auto root = StrictJsonParser::parse(payload);
  require_exact_fields(root, {
    "schema_version", "publisher_instance_id", "router_zid", "sequence",
    "timestamp_ns", "producer", "side", "mode", "proposal_sequence",
    "target_sequence", "names", "position_rad"
  });
  if (field(root, "schema_version").as_uint("schema_version") != 1 ||
      field(root, "router_zid").as_string("router_zid") != router ||
      field(root, "side").as_string("side") != side) {
    throw std::invalid_argument("command identity mismatch");
  }
  const auto mode = field(root, "mode").as_string("mode");
  if (mode != "idle" && mode != "teleop" && mode != "returning") throw std::invalid_argument("invalid command mode");
  const auto names = tianji_teleop::protocol::string_array_field(root, "names", 7);
  for (std::size_t index = 0; index < names.size(); ++index) {
    const auto expected = "Joint" + std::to_string(index + 1) + (side == "left" ? "_L" : "_R");
    if (names[index] != expected) throw std::invalid_argument("command joint order mismatch");
  }
  (void)tianji_teleop::protocol::vector_field(root, "position_rad", 7);
  (void)field(root, "publisher_instance_id").as_string("publisher_instance_id");
  (void)field(root, "producer").as_string("producer");
  (void)field(root, "sequence").as_uint("sequence");
  (void)field(root, "timestamp_ns").as_uint("timestamp_ns");
  if (!field(root, "proposal_sequence").is_null()) (void)field(root, "proposal_sequence").as_uint("proposal_sequence");
  if (!field(root, "target_sequence").is_null()) (void)field(root, "target_sequence").as_uint("target_sequence");
}

std::string proposal() {
  return R"({"schema_version":1,"publisher_instance_id":"ik-1","router_zid":"router-1","sequence":8,"timestamp_ns":9,"producer":"ik","side":"left","target_sequence":7,"names":["Joint1_L","Joint2_L","Joint3_L","Joint4_L","Joint5_L","Joint6_L","Joint7_L"],"position_rad":[0,0,0,0,0,0,0],"diagnostics":{"accepted":true}})";
}
std::string solved() {
  return R"({"schema_version":1,"publisher_instance_id":"ik-1","router_zid":"router-1","sequence":8,"timestamp_ns":9,"producer":"ik","side":"left","frame_id":"Base_L","target_sequence":7,"position_m":[0.1,0.2,0.3],"orientation_xyzw":[0,0,0,1]})";
}
std::string status() {
  return R"({"schema_version":1,"publisher_instance_id":"ik-1","router_zid":"router-1","sequence":8,"timestamp_ns":9,"component_role":"producer_arm","component_id":"ik","phase":"ready","ready":true,"healthy":true,"capabilities":["simulation"],"error":null,"diagnostics":{}})";
}
}  // namespace

int main(int argc, char **argv) {
  try {
    if (argc < 2) throw std::invalid_argument("fixture mode is required");
    const std::string mode(argv[1]);
    if (mode == "emit-proposal") std::cout << proposal();
    else if (mode == "emit-solved") std::cout << solved();
    else if (mode == "emit-status") std::cout << status();
    else {
      if (argc != 4) throw std::invalid_argument("parser mode requires router and side");
      const std::string payload((std::istreambuf_iterator<char>(std::cin)), std::istreambuf_iterator<char>());
      if (mode == "target") parse_target(payload, argv[2], argv[3]);
      else if (mode == "command") parse_command(payload, argv[2], argv[3]);
      else throw std::invalid_argument("unknown fixture mode");
      std::cout << "ok\n";
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
