#pragma once
#include <mc_control/fsm/Controller.h>
#include <mc_prediction/mi_impactPredictor.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/Robots.h>

#include "BoundJointTorqueJump.h"
#include "BoundJointVelocityJump.h"
#include "PositiveContactForceWithImpulse.h"
#include "COPInsideContactAreaWithImpulse.h" 

struct Controller : public mc_control::fsm::Controller
{
  Controller(const mc_rbdyn::RobotModulePtr & rm, const double & dt, const mc_rtc::Configuration & conf);

  void reset(const mc_control::ControllerResetData & data) override;

  std::shared_ptr<mc_tasks::MetaTask> comTaskPtr;

  // Force sensor threshold
  double forceThreshold = 3.0;

  bool firstContact = true;

  bool rArmInContact();

  std::unique_ptr<mi_impactPredictor> miPredictorPtr;
  std::unique_ptr<mc_impact::BoundJointTorqueJump> boundTorqueJump_;
  std::unique_ptr<mc_impact::BoundJointVelocityJump> boundVelocityJump_;
  std::unique_ptr<mc_impact::PositiveContactForceWithImpulse> positiveContactForceLeftFoot_;
  std::unique_ptr<mc_impact::PositiveContactForceWithImpulse> positiveContactForceRightFoot_;

  std::unique_ptr<mc_impact::COPInsideContactAreaWithImpulse> COPImpulseLeftFoot_;
  std::unique_ptr<mc_impact::COPInsideContactAreaWithImpulse> COPImpulseRightFoot_;
private:
  const mc_rbdyn::Robot & realRobot() const;

  const mc_rbdyn::Contact & getContact(const std::string & s);

  // mc_rtc::Configuration state_conf_;
};
