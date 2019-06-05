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
  /*
    std::cout << "About to create new constriants" << std::endl;

    boundTorqueJump_.reset(new mc_impact::BoundJointTorqueJump(*ctl.miPredictorPtr, ctl.timeStep, ctl.timeStep,
          state_conf_("JumpTorqueMultiplier", 5.0)));
    ctl.solver().addConstraint(boundTorqueJump_.get());

    boundVelocityJump_.reset(new mc_impact::BoundJointVelocityJump(*ctl.miPredictorPtr, ctl.timeStep));

    std::cout << "bound velocity jump constraint is created" << std::endl;
    ctl.solver().addConstraint(boundVelocityJump_.get());
    std::cout << "bound velocity jump constraint is added" << std::endl;

  */

  // ------------------------------------ Positive contact force
  /*
  positiveContactForceLeftFoot_.reset(
      new mc_impact::PositiveContactForceWithImpulse(ctl.solver(), ctl.getContact("LeftFoot"), *ctl.miPredictorPtr));

  positiveContactForceRightFoot_.reset(
      new mc_impact::PositiveContactForceWithImpulse(ctl.solver(), ctl.getContact("RightFoot"), *ctl.miPredictorPtr));

  std::cout << "Positive contact force constraint is created" << std::endl;

  ctl.solver().addConstraint(positiveContactForceLeftFoot_.get());
  ctl.solver().addConstraint(positiveContactForceRightFoot_.get());

  std::cout << "Positive contact force constraint is added" << std::endl;
  */
  // ------------------------------------ Zero slippage
  /*
   COPImpulseLeftFoot_.reset(
      new mc_impact::COPInsideContactAreaWithImpulse(ctl.solver(), ctl.getContact("LeftFoot"), *ctl.miPredictorPtr)
       );

   COPImpulseRightFoot_.reset(
      new mc_impact::COPInsideContactAreaWithImpulse(ctl.solver(), ctl.getContact("RightFoot"), *ctl.miPredictorPtr)
       );


   ctl.solver().addConstraint(COPImpulseLeftFoot_.get());
   ctl.solver().addConstraint(COPImpulseRightFoot_.get());
   std::cout << "Zero slippage constraint is added." <<std::endl;
 */

  // ctl.solver().updateConstrSize();

  /*

    logger().addLogEntry("r_ankle_predict_OSD_QP_force", [this]() {
        Eigen::VectorXd force = miPredictorPtr->getQPForce("r_sole");
        return force;
    });
  */
  run(ctlInput);
}

bool DynamicContactState::run(mc_control::fsm::Controller & ctlInput)
{
  auto & ctl = static_cast<Controller &>(ctlInput);
  std::cout << "DynamicsState: The right ef error is: " << rPosTaskPtr_->eval().norm() << std::endl;
  // std::cout << "DynamicsState: The right sole OSD force is: " << ctl.miPredictorPtr->getOsdForce("l_sole")<<
  // std::endl; std::cout << "DynamicsState: The right sole OSD force is: " <<
  // ctl.miPredictorPtr->getOsdForce("r_sole")<< std::endl;

  Eigen::Vector3d surfaceNormal;
  surfaceNormal << 1, 0, 0;
  // Convert surfaceNormal to the local frame of the right wrist.
  sva::PTransformd X_0_ee = ctl.robot().bodyPosW("r_wrist");

  ctl.miPredictorPtr->run(X_0_ee.rotation() * surfaceNormal + X_0_ee.translation());

  // if(rPosTaskPtr_->eval().norm() <= 0.01)
  if(ctl.rArmInContact())
  {
    // Output the transition signal such that we can move on according to the transitions

    output("ImpactDetected");
    std::cout << "---------------Impact Detected--------------" << std::endl;
    // output("OK");
    ctl.solver().removeTask(rPosTaskPtr_);
    std::cout << "---------------End-effector task removed--------------" << std::endl;
    // ctl.solver().removeConstraint(ctl.copImpulseLeftFoot_);
    // ctl.solver().removeConstraint(ctl.copImpulseRightFoot_);
    return true;
  }

  return false;
}

void DynamicContactState::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<Controller &>(ctl_);
  ctl.solver().removeTask(rPosTaskPtr_);
  ctl.solver().removeTask(ctl.comTaskPtr);
  ctl.solver().removeConstraint(ctl.boundTorqueJump_.get());
  // ctl.solver().removeConstraint(ctl.boundVelocityJump_.get());
  // ctl.solver().removeConstraint(positiveContactForceRightFoot_.get());
  // ctl.solver().removeConstraint(positiveContactForceRightFoot_.get());
}

EXPORT_SINGLE_STATE("DynamicContact", DynamicContactState)
