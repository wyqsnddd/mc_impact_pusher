#include "DynamicContact.h"

#include <mc_tasks/MetaTaskLoader.h>

#include "../mc_r_arm_push_unknown_wall.h"

void DynamicContactState::configure(const mc_rtc::Configuration & config)
{
  state_conf_.load(config);
}

void DynamicContactState::start(mc_control::fsm::Controller & ctlInput)
{

  std::cout << "Starting DynamicContactState" << std::endl;

  auto & ctl = static_cast<Controller &>(ctlInput);

  const auto & conf = ctl.config()("hrp4");

  rPosTaskPtr_ = std::make_shared<mc_tasks::PositionTask>(conf("rightEfTaskDynamic")("bodyName"), ctl.robots(), 0,
                                                          conf("rightEfTaskDynamic")("stiffness"),
                                                          conf("rightEfTaskDynamic")("weight"));
  ctl.solver().addTask(rPosTaskPtr_);

  Eigen::Vector3d referenceVelocity, pushDepth, currentPos;
  pushDepth = ctl.config()("states")("Contact")("pushDepth");
  referenceVelocity = ctl.config()("states")("Contact")("contactVelocity");

  currentPos = rPosTaskPtr_->position();
  // Set the target position, which is supposed to be far away.
  rPosTaskPtr_->position(currentPos + pushDepth);
  // Set the reference velocity
  rPosTaskPtr_->refVel(referenceVelocity);

  // Set the damping gain (set it much much higher than the position
  // gain[stiffness] to ensure that velocity is properly tracked
  if(state_conf_.has("rightEfStiffness"))
  {
    rPosTaskPtr_->stiffness(static_cast<double>(state_conf_("rightEfStiffness")));
  }
  if(state_conf_.has("rightEfDamping"))
  {
    rPosTaskPtr_->damping(static_cast<double>(state_conf_("rightEfDamping")));
  }
  ctl.miPredictorPtr->resetDataStructure();
  
  run(ctlInput);
}

bool DynamicContactState::run(mc_control::fsm::Controller & ctlInput)
{
  auto & ctl = static_cast<Controller &>(ctlInput);
  Eigen::Vector3d surfaceNormal;
  surfaceNormal << 1, 0, 0;
  // Convert surfaceNormal to the local frame of the right wrist.
  sva::PTransformd X_0_ee = ctl.robot().bodyPosW("r_wrist");

  ctl.miPredictorPtr->run(X_0_ee.rotation() * surfaceNormal + X_0_ee.translation());

  if(ctl.rArmInContact())
  {
    //output("ImpactDetected");
    output("OK");
    //ctl.solver().removeTask(rPosTaskPtr_);
    
  
    return true;
  }

  return false;
}

void DynamicContactState::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<Controller &>(ctl_);
  ctl.solver().removeTask(rPosTaskPtr_);
  ctl.solver().removeTask(ctl.comTaskPtr);
    if(ctl.config()("impact")("constraints")("zmpWithImpulse")("on") && !removedConstraint_){
      removedConstraint_ = true;
      ctl.solver().removeConstraint(ctl.zmpImpulse_.get());
      ctl.logger().removeLogEntry("ZMP_Constraint_test");
      ctl.logger().removeLogEntry("ZMP_constraint_difference");
      ctl.logger().removeLogEntry("ZMP_perturbation_zmpconstraint");
      ctl.logger().removeLogEntry("ZMP_sensor_zmpconstraint");
      ctl.logger().removeLogEntry("ZMP_prediction_zmpconstraint");
    }
   
  if(ctl.config()("impact")("constraints")("copWithImpulse")("on") )
  {
    ctl.solver().removeConstraint(ctl.copImpulseLeftFoot_.get());
    ctl.solver().removeConstraint(ctl.copImpulseRightFoot_.get());
  }
  
  if(ctl.config()("impact")("constraints")("frictionWithImpulse") ){
    ctl.solver().removeConstraint(ctl.frictionImpulseLeftFoot_.get());
    ctl.solver().removeConstraint(ctl.frictionImpulseRightFoot_.get());
  }

  if(ctl.config()("impact")("constraints")("jointTorque")("on") ){
    ctl.solver().removeConstraint(ctl.boundTorqueJump_.get());
  }

  if(ctl.config()("impact")("constraints")("jointVelocity") ){
    ctl.solver().removeConstraint(ctl.boundVelocityJump_.get());
  }
  ctl.solver().updateConstrSize();
 
}

EXPORT_SINGLE_STATE("DynamicContact", DynamicContactState)
