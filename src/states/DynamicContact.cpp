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
  ctl.miPredictorPtr->resetDataStructure();

  /*
    ctl.miPredictorPtr.reset(new mi_impactPredictor(ctl.robot(), ctl.config()("states")("Contact")("impactBodyName"),
                                                    ctl.config()("states")("Contact")("useLinearJacobian"),
                                                    ctl.solver().dt(),
                                                    ctl.config()("states")("Contact")("coeRestitution")));

    std::vector<std::string> eeNameVector = ctl.config()("states")("Contact")("end-effectors");

    auto * ctlPtr = &ctl;
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

    ctlPtr->logger().addLogEntry("new_eelee_impact_impulse", [ctlPtr]() {
      Eigen::Vector3d eeleeImpulse = ctlPtr->miPredictorPtr->getNewEeLeeImpulsiveForce();
      return eeleeImpulse;
    });

    ctlPtr->logger().addLogEntry("new_eeRee_impact_impulse", [ctlPtr]() {
      Eigen::Vector3d eereeImpulse = ctlPtr->miPredictorPtr->getNewEeReeImpulsiveForce();
      return eereeImpulse;
    });

    ctlPtr->logger().addLogEntry("new_lee_impact_impulse", [ctlPtr]() {
      Eigen::Vector3d leeImpulse = ctlPtr->miPredictorPtr->getNewLeeImpulsiveForce();
      return leeImpulse;
    });
    ctlPtr->logger().addLogEntry("new_ree_impact_impulse", [ctlPtr]() {
      Eigen::Vector3d reeImpulse = ctlPtr->miPredictorPtr->getNewReeImpulsiveForce();
      return reeImpulse;
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

    ctlPtr->logger().addLogEntry("q_vel", [ctlPtr]() {
      Eigen::VectorXd qvel = rbd::dofToVector(ctlPtr->robot().mb(), ctlPtr->robot().mbc().alpha);
      return qvel;
    });


    ctlPtr->logger().addLogEntry("q_vel_predict_impact_jump", [ctlPtr]() {
      Eigen::VectorXd qveljump = ctlPtr->miPredictorPtr->getJointVelocityJump();
      return qveljump;
    });

    ctlPtr->logger().addLogEntry("q_tau_predict_impact_jump", [ctlPtr]() {
      Eigen::VectorXd tauJump = ctlPtr->miPredictorPtr->getTauJump();
      return tauJump;
    });
  */
}

bool DynamicContactState::run(mc_control::fsm::Controller & ctlInput)
{
  // std::cout<<"The left ef error is: "<< lEfTaskPtr_->eval().norm() << ", the right ef error is: " <<
  // rEfTaskPtr_->eval().norm() <<std::endl;

  auto & ctl = static_cast<Controller &>(ctlInput);

  Eigen::Vector3d surfaceNormal;
  surfaceNormal << -1, 0, 0;

  ctl.miPredictorPtr->run(surfaceNormal);

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
