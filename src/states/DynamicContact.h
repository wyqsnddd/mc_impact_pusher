#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/PositionTask.h>

// #include "BoundJointTorqueJump.h"
// #include "BoundJointVelocityJump.h"

struct DynamicContactState : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller &) override;

  bool run(mc_control::fsm::Controller &) override;

  void teardown(mc_control::fsm::Controller &) override;

protected:
  // tasks to raise the robot hands
  std::shared_ptr<mc_tasks::PositionTask> rPosTaskPtr_; // right hand
  sva::PTransformd rTransformZero_;
  double contactVelocity_;
  mc_rtc::Configuration state_conf_;

  //  std::unique_ptr<mc_impact::BoundJointTorqueJump> boundTorqueJump_;
  // std::unique_ptr<mc_impact::BoundJointVelocityJump> boundVelocityJump_;
};
