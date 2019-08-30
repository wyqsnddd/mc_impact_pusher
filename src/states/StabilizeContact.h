#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/AdmittanceTask.h>

struct StabilizeContactState : mc_control::fsm::State {
  void configure(const mc_rtc::Configuration &config) override;

  void start(mc_control::fsm::Controller &) override;

  bool run(mc_control::fsm::Controller &) override;

  void teardown(mc_control::fsm::Controller &) override;

protected:
  // tasks to raise the robot hands
  std::shared_ptr<mc_tasks::force::AdmittanceTask> rAdmTaskPtr_; // right hand
  sva::PTransformd rTransformZero_;
  mc_rtc::Configuration state_conf_;
  Eigen::Vector3d targetForce_ = Eigen::Vector3d::Zero();
  double admThreshold_ = 0.0;
  //  bool removedConstraint_ = false;
};
