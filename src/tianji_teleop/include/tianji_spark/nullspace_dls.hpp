#pragma once

#include "tianji_spark/velocity_ik.hpp"

namespace tianji_spark {

class NullspaceDlsIk7 final : public IArmVelocityIk {
 public:
  explicit NullspaceDlsIk7(DlsConfig config);

  ArmIkResult solve(const ArmIkInput& input) override;
  void reset() override {}
  IkAlgorithm algorithm() const noexcept override {
    return IkAlgorithm::kNullspaceDls;
  }

 private:
  DlsConfig config_;
};

}  // namespace tianji_spark
