#include "DynamicContact.h"

#include <mc_tasks/MetaTaskLoader.h>

#include "../mc_r_arm_push_unknown_wall.h"

void DynamicContactState::configure(const mc_rtc::Configuration & config)
{
  state_conf_ = config;
}

void DynamicContactState::start(mc_control::fsm::Controller & ctlInput)
{
  auto & ctl = static_cast<Controller &>(ctlInput);

  const auto & conf = ctl.config()("hrp4");

  rPosTaskPtr_ =
      std::make_shared<mc_tasks::PositionTask>(conf("rightEfTask")("bodyName"), ctl.robots(), 0,
                                               conf("rightEfTask")("stiffness"), conf("rightEfTask")("weight"));
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

  ctl.miPredictorPtr.reset(new mi_impactPredictor(ctl.robot(), ctl.config()("states")("Contact")("impactBodyName"),
                                                  ctl.config()("states")("Contact")("useLinearJacobian"),
                                                  ctl.solver().dt(),
                                                  ctl.config()("states")("Contact")("coeRestitution")));

  std::vector<std::string> eeNameVector = ctl.config()("states")("Contact")("end-effectors");

  for(auto index = eeNameVector.begin(); index != eeNameVector.end(); ++index)
  {

    // std::string eeName = ctl.config()("states")("Contact")("end-effectors")(index);
    std::cout << "Processing end-effector: " << *index << std::endl;

    if(!ctl.miPredictorPtr->addEndeffector(*index))
    {
      throw std::runtime_error("Impact predictor failed to add endeffector!");
    }
    else
    {
      std::cout << "End-effector: " << *index << "is added to the impact-predictor. " << std::endl;
    }
  }
  ctl.miPredictorPtr->initializeDataStructure();
  std::cout << "Operational space dynamics Predictor is created." << std::endl;

  auto * ctlPtr = &ctl;
  ctlPtr->logger().addLogEntry("ee_Vel_impact_jump", [ctlPtr]() {
    Eigen::Vector3d eeVelJump = ctlPtr->miPredictorPtr->getEeVelocityJump();
    return eeVelJump;
  });
  ctlPtr->logger().addLogEntry("ee_Vel", [ctlPtr]() {
    Eigen::Vector3d eeVel = ctlPtr->realRobots()
                                .robot()
                                .mbc()
                                .bodyVelW[ctlPtr->realRobots().robot().mb().bodyIndexByName("r_wrist")]
                                .linear();

    return eeVel;
  });

  ctlPtr->logger().addLogEntry("l_ankle_Vel_impact_jump", [ctlPtr]() {
    Eigen::Vector3d eeVelJump = ctlPtr->miPredictorPtr->getEeVelocityJump("l_ankle");
    return eeVelJump;
  });

  ctlPtr->logger().addLogEntry("r_ankle_Vel_impact_jump", [ctlPtr]() {
    Eigen::Vector3d eeVelJump = ctlPtr->miPredictorPtr->getEeVelocityJump("r_ankle");
    return eeVelJump;
  });

  ctlPtr->logger().addLogEntry("ee_impact_impulse", [ctlPtr]() {
    Eigen::Vector3d eeImpulse = ctlPtr->miPredictorPtr->getImpulsiveForce();
    return eeImpulse;
  });

  ctlPtr->logger().addLogEntry("l_ankle_predict_impact_impulse", [ctlPtr]() {
    Eigen::Vector3d ankleImpulse = ctlPtr->miPredictorPtr->getImpulsiveForce("l_ankle");
    return ankleImpulse;
  });

  ctlPtr->logger().addLogEntry("l_ankle_ee_acc_force", [ctlPtr]() {
    Eigen::Vector3d ankleAccForce = ctlPtr->miPredictorPtr->getEeAccForce("l_ankle");
    return ankleAccForce;
  });

  ctlPtr->logger().addLogEntry("r_ankle_ee_acc_force", [ctlPtr]() {
    Eigen::Vector3d ankleAccForce = ctlPtr->miPredictorPtr->getEeAccForce("r_ankle");
    return ankleAccForce;
  });

  ctlPtr->logger().addLogEntry("r_ankle_predict_impact_impulse", [ctlPtr]() {
    Eigen::Vector3d ankleImpulse = ctlPtr->miPredictorPtr->getImpulsiveForce("r_ankle");
    return ankleImpulse;
  });

  ctlPtr->logger().addLogEntry("q_vel_predict_impact_jump", [ctlPtr]() {
    Eigen::VectorXd qveljump = ctlPtr->miPredictorPtr->getJointVelocityJump();
    return qveljump;
  });
}

bool DynamicContactState::run(mc_control::fsm::Controller & ctlInput)
{
  // std::cout<<"The left ef error is: "<< lEfTaskPtr_->eval().norm() << ", the right ef error is: " <<
  // rEfTaskPtr_->eval().norm() <<std::endl;

  auto & ctl = static_cast<Controller &>(ctlInput);
  ctl.miPredictorPtr->run();

  if(rPosTaskPtr_->eval().norm() <= 0.01)
  {
    // Output the transition signal such that we can move on according to the transitions
    // ctl.solver().removeTask(rPosTaskPtr_);
    output("RightHandFinished");
    return true;
  }

  return false;
}

void DynamicContactState::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<Controller &>(ctl_);
  ctl.solver().removeTask(rPosTaskPtr_);
  ctl.solver().removeTask(ctl.comTaskPtr);
}

EXPORT_SINGLE_STATE("DynamicContact", DynamicContactState)
