#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/EndEffectorTask.h>

#include <mc_impact_constraints/BoundJointTorqueJump.h>
#include <mc_impact_constraints/BoundJointVelocityJump.h>
#include <mc_impact_constraints/COPInsideContactAreaWithImpulse.h>
#include <mc_impact_constraints/ZeroSlippageWithImpulse.h>

struct DynamicContactState : mc_control::fsm::State {
  void configure(const mc_rtc::Configuration &config) override;

  void start(mc_control::fsm::Controller &) override;

  bool run(mc_control::fsm::Controller &) override;

  void teardown(mc_control::fsm::Controller &) override;

protected:
  // tasks to raise the robot hands
  std::shared_ptr<mc_tasks::EndEffectorTask> rPosTaskPtr_; // right hand
  sva::PTransformd rTransformZero_;
  double contactVelocity_;
  mc_rtc::Configuration state_conf_;

  bool removedConstraint_ = false;

  std::unique_ptr<mc_impact::BoundJointTorqueJump> boundTorqueJump_;
  std::unique_ptr<mc_impact::BoundJointVelocityJump> boundVelocityJump_;
  std::unique_ptr<mc_impact::ZeroSlippageWithImpulse> zeroSlippageLeftFoot_;
  std::unique_ptr<mc_impact::ZeroSlippageWithImpulse> zeroSlippageRightFoot_;
};
