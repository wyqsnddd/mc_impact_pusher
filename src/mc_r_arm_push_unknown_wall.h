#pragma once
#include <mc_control/fsm/Controller.h>
#include <mc_prediction/mi_impactPredictor.h>
#include <mc_prediction/mi_lcp.h>
#include <mc_prediction/mi_multiImpactPredictor.h>
#include <mc_prediction/mi_qpEstimator.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/Robots.h>

#include <mc_impact_constraints/BoundJointTorqueJump.h>
#include <mc_impact_constraints/BoundJointVelocityJump.h>
#include <mc_impact_constraints/COPInsideContactAreaWithImpulse.h>
#include <mc_impact_constraints/ZeroSlippageWithImpulse.h>
#include <mc_impact_constraints/copWithImpulse.h>
#include <mc_impact_constraints/frictionWithImpulse.h>
#include <mc_impact_constraints/zmpWithImpulse.h>

struct Controller : public mc_control::fsm::Controller
{
  Controller(const mc_rbdyn::RobotModulePtr & rm, const double & dt, const mc_rtc::Configuration & conf);

  void reset(const mc_control::ControllerResetData & data) override;

  bool run() override;

  std::shared_ptr<mc_tasks::MetaTask> comTaskPtr;

  std::unique_ptr<mc_impact::BoundJointTorqueJump> boundTorqueJump_;
  std::unique_ptr<mc_impact::BoundJointVelocityJump> boundVelocityJump_;
  std::unique_ptr<mc_impact::ZeroSlippageWithImpulse> zeroSlippageLeftFoot_;
  std::unique_ptr<mc_impact::ZeroSlippageWithImpulse> zeroSlippageRightFoot_;
  std::unique_ptr<mc_impact::COPInsideContactAreaWithImpulse> COPImpulseLeftFoot_;
  std::unique_ptr<mc_impact::COPInsideContactAreaWithImpulse> COPImpulseRightFoot_;
  std::unique_ptr<mc_impact::frictionWithImpulse> frictionImpulseRightFoot_;
  std::unique_ptr<mc_impact::frictionWithImpulse> frictionImpulseLeftFoot_;
  std::unique_ptr<mc_impact::copWithImpulse> copImpulseLeftFoot_;
  std::unique_ptr<mc_impact::copWithImpulse> copImpulseRightFoot_;
  std::unique_ptr<mc_impact::zmpWithImpulse> zmpImpulse_;

  std::unique_ptr<mc_impact::BoundJointTorqueJump> left_boundTorqueJump_;
  std::unique_ptr<mc_impact::BoundJointVelocityJump> left_boundVelocityJump_;

  std::unique_ptr<mc_impact::ZeroSlippageWithImpulse> left_zeroSlippageLeftFoot_;
  std::unique_ptr<mc_impact::ZeroSlippageWithImpulse> left_zeroSlippageRightFoot_;
  std::unique_ptr<mc_impact::COPInsideContactAreaWithImpulse> left_COPImpulseLeftFoot_;
  std::unique_ptr<mc_impact::COPInsideContactAreaWithImpulse> left_COPImpulseRightFoot_;
  std::unique_ptr<mc_impact::frictionWithImpulse> left_frictionImpulseRightFoot_;
  std::unique_ptr<mc_impact::frictionWithImpulse> left_frictionImpulseLeftFoot_;
  std::unique_ptr<mc_impact::copWithImpulse> left_copImpulseLeftFoot_;
  std::unique_ptr<mc_impact::copWithImpulse> left_copImpulseRightFoot_;
  std::unique_ptr<mc_impact::zmpWithImpulse> left_zmpImpulse_;

  // Force sensor threshold
  double forceThreshold = 3.0;

  bool firstContact = true;

  bool rArmInContact();

  // std::unique_ptr<mi_impactPredictor> miPredictorPtr;
  std::unique_ptr<mi_multiImpactPredictor> multiImpactPredictorPtr;
  std::unique_ptr<mi_lcp> lcpSolverPtr;
  std::unique_ptr<mi_qpEstimator> qpEstimatorPtr;
  std::unique_ptr<mi_qpEstimator> jsdQpEstimatorPtr;
  std::unique_ptr<mi_qpEstimator> osdQpEstimatorPtr;
  std::unique_ptr<mi_qpEstimator> ecQpEstimatorPtr;
  // std::shared_ptr<mi_osd> miOsdPtr;

  const mc_rbdyn::Contact & getContact(const std::string & s);

private:
  const mc_rbdyn::Robot & realRobot() const;
  double impactIndicator_;

  size_t iter_ = 0;
  // mc_rtc::Configuration state_conf_;
};
