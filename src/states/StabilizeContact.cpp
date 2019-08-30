#include "StabilizeContact.h"

#include <mc_tasks/MetaTaskLoader.h>

#include "../mc_r_arm_push_unknown_wall.h"

void StabilizeContactState::configure(const mc_rtc::Configuration &config) {
  state_conf_.load(config);
}

void StabilizeContactState::start(mc_control::fsm::Controller &ctlInput) {

  LOG_SUCCESS("Starting StabilizeContactState");

  auto &ctl = static_cast<Controller &>(ctlInput);

  const auto &conf = ctl.config()("hrp4");
  const auto &surfaceName = ctl.config()("states")("Stabilize")("surfaceName");

  rAdmTaskPtr_ = std::make_shared<mc_tasks::force::AdmittanceTask>(
      surfaceName, ctl.robots(), 0, conf("rightEfTaskDynamic")("stiffness"),
      conf("rightEfTaskDynamic")("weight"));

  ctl.solver().addTask(rAdmTaskPtr_);

  // read the current pose of the surface
  rAdmTaskPtr_->targetPose();

  // Read from Json
  std::vector<double> targetForce =
      ctl.config()("states")("Stabilize")("targetForce");
  targetForce_(0) = targetForce[0];
  targetForce_(1) = targetForce[1];
  targetForce_(2) = targetForce[2];

  rAdmTaskPtr_->targetWrench(
      sva::ForceVecd(Eigen::Vector3d::Zero(), targetForce_));
  run(ctlInput);
}

bool StabilizeContactState::run(mc_control::fsm::Controller &ctlInput) {
  auto &ctl = static_cast<Controller &>(ctlInput);

  if (rAdmTaskPtr_->eval().norm()) {
    LOG_INFO("Admittance task achieved.");
    output("finished");
    return true;
  }

  return false;
}

void StabilizeContactState::teardown(mc_control::fsm::Controller &ctl_) {
  LOG_SUCCESS("Completed StabilizeContactState");
  auto &ctl = static_cast<Controller &>(ctl_);
}

EXPORT_SINGLE_STATE("StabilizeContact", StabilizeContactState)
