#include "mc_r_arm_push_unknown_wall.h"

Controller::Controller(const mc_rbdyn::RobotModulePtr & rm, const double & dt, const mc_rtc::Configuration & conf)
: mc_control::fsm::Controller(rm, dt, conf)
{

  forceThreshold = config()("contact_detection")("ForceThreshold");

  std::cout << "Kinematics and Dynamics constraints are created." << std::endl;
  solver().addConstraintSet(kinematicsConstraint);
  solver().addConstraintSet(dynamicsConstraint);

  logger().addLogEntry("CoP_LeftFoot_World", [this]() {
    auto & robot = this->realRobots().robot();
    return robot.copW("LeftFoot");
  });
  logger().addLogEntry("CoP_RightFoot_World", [this]() {
    auto & robot = this->realRobots().robot();
    return robot.copW("RightFoot");
  });

  logger().addLogEntry("ZMP_World", [this]() {
    auto & robot = this->realRobots().robot();
    Eigen::Vector3d zmp;
    try
    {
      zmp = robot.zmp({"LeftFootForceSensor", "RightFootForceSensor"}, Eigen::Vector3d::Zero(),
                      Eigen::Vector3d{0., 0., 1.}, 100);
      zmp.z() = 0;
    }
    catch(...)
    {
      zmp.setZero();
    }
    return zmp;
  });
  logger().addLogEntry("realRobot_posW", [this]() { return realRobot().posW(); });

  gui()->addElement({"Forces"},
                    mc_rtc::gui::Force("LeftCoPForce_real", mc_rtc::gui::ForceConfig(mc_rtc::gui::Color(1., 0.2, 0.)),
                                       [this]() { return realRobot().surfaceWrench("LeftFoot"); },
                                       [this]() {
                                         auto & robot = realRobot();
                                         Eigen::Vector3d cop = robot.copW("LeftFoot");
                                         const sva::PTransformd X_0_s = robot.surface("LeftFoot").X_0_s(robot);
                                         sva::PTransformd surface(X_0_s.rotation(), cop);
                                         return surface;
                                       }),
                    mc_rtc::gui::Force("RightCoPForce_real", mc_rtc::gui::ForceConfig(mc_rtc::gui::Color(1., 0.2, 0.)),
                                       [this]() { return realRobot().surfaceWrench("RightFoot"); },
                                       [this]() {
                                         auto & robot = this->realRobots().robot();
                                         Eigen::Vector3d cop = robot.copW("RightFoot");
                                         const sva::PTransformd X_0_s = robot.surface("RightFoot").X_0_s(robot);
                                         sva::PTransformd surface(X_0_s.rotation(), cop);
                                         return surface;
                                       }),
                    mc_rtc::gui::Point3D("ZMP_real", [this]() {
                      auto & robot = this->realRobots().robot();
                      Eigen::Vector3d zmp;
                      try
                      {
                        zmp = robot.zmp({"LeftFootForceSensor", "RightFootForceSensor"}, Eigen::Vector3d::Zero(),
                                        Eigen::Vector3d{0., 0., 1.}, 100);
                        zmp.z() = 0;
                      }
                      catch(...)
                      {
                        zmp.setZero();
                      }
                      return zmp;
                    }));

  std::string impactBodyString(config()("states")("Contact")("impactBodyName"));

  //auto fd_example = dynamicsConstraint.motionConstr->fd();
  //std::unique_ptr<rbd::ForwardDynamics> fdtempPtr = std::unique_ptr<rbd::ForwardDynamics>(dynamicsConstraint.motionConstr->fd());
  //std::shared_ptr<rbd::ForwardDynamics> fdPtr = std::make_shared<rbd::ForwardDynamics>(dynamicsConstraint.motionConstr->fd());
  //const rbd::ForwardDynamics * fdPtr_temp = dynamicsConstraint.motionConstr->fd();


  //std::shared_ptr<rbd::ForwardDynamics> fdPtr = std::make_shared<rbd::ForwardDynamics>(dynamicsConstraint.motionConstr->fd());
  //miPredictorPtr.reset(new mi_impactPredictor(robots().robot(robots().robotIndex()), 
  miPredictorPtr.reset(new mi_impactPredictor(robot(), 
			  impactBodyString,
			  config()("states")("Contact")("useLinearJacobian"), 
			  //solver().dt(),
			  config()("states")("Contact")("delta_dt"),
			  config()("states")("Contact")("coeRestitution")
			  )
		  );

  std::vector<std::string> eeNameVector = config()("states")("Contact")("end-effectors");

  miPredictorPtr->initializeDataStructure(static_cast<int>(eeNameVector.size()));
  miPredictorPtr->resetDataStructure();

  for(auto index = eeNameVector.begin(); index != eeNameVector.end(); ++index)
  {

    // std::string eeName = ctl.config()("states")("Contact")("end-effectors")(index);
    std::cout << "Processing end-effector: " << *index << std::endl;

    if(!miPredictorPtr->addEndeffector(*index))
    {
      throw std::runtime_error("Impact predictor failed to add endeffector!");
    }
    else
    {
      std::cout << "End-effector: " << *index << "is added to the impact-predictor. " << std::endl;
    }
  }

  std::cout << "Operational space dynamics Predictor is created." << std::endl;

  miPredictorPtr->setContact("l_sole");
  miPredictorPtr->setContact("r_sole");
 
  logger().addLogEntry("ee_Vel_impact_jump", [this]() {
    //Eigen::VectorXd eeVelJump = 
    return miPredictorPtr->getEeVelocityJump();
  });
  logger().addLogEntry("ee_Vel", [this]() {
		  return realRobots().robot().mbc().bodyVelW[realRobots().robot().mb().bodyIndexByName("r_wrist")].linear();
  });

  logger().addLogEntry("ee_dq", [this]() {

    return miPredictorPtr->getBranchJointVelJump("r_wrist"); 
    
  });
  logger().addLogEntry("ee_dtau", [this]() {
    return miPredictorPtr->getBranchTauJump("r_wrist"); 
  });


  logger().addLogEntry("l_ankle_Vel_impact_jump", [this]() {
    return miPredictorPtr->getEeVelocityJump("l_sole");
  });

  logger().addLogEntry("l_ankle_Vel", [this]() {
    return realRobots().robot().mbc().bodyVelW[realRobots().robot().mb().bodyIndexByName("l_sole")].linear();
  });

  logger().addLogEntry("l_ankle_dq", [this]() {

    return miPredictorPtr->getBranchJointVelJump("l_sole"); 
    
  });
  logger().addLogEntry("l_ankle_dtau", [this]() {
    return miPredictorPtr->getBranchTauJump("l_sole"); 
  });



  logger().addLogEntry("r_ankle_Vel_impact_jump", [this]() {
    return miPredictorPtr->getEeVelocityJump("r_sole");
  });

  logger().addLogEntry("r_ankle_Vel", [this]() {
    return  realRobots().robot().mbc().bodyVelW[realRobots().robot().mb().bodyIndexByName("r_sole")].linear();
  });

  logger().addLogEntry("r_ankle_dq", [this]() {

    return miPredictorPtr->getBranchJointVelJump("r_sole"); 
    
  });
  logger().addLogEntry("r_ankle_dtau", [this]() {
    return miPredictorPtr->getBranchTauJump("r_sole"); 
  });


  logger().addLogEntry("ee_impact_impulse", [this]() {
    return miPredictorPtr->getImpulsiveForce();
  });

  logger().addLogEntry("l_ankle_predict_impact_impulse", [this]() {
    return miPredictorPtr->getImpulsiveForce("l_sole");
  });

  logger().addLogEntry("r_ankle_predict_impact_impulse", [this]() {
    return miPredictorPtr->getImpulsiveForce("r_sole");
  });
  logger().addLogEntry("q_position", [this]() {
    return rbd::dofToVector(robot().mb(), robot().mbc().q);
  });


  logger().addLogEntry("q_vel", [this]() {
    return rbd::dofToVector(robot().mb(), robot().mbc().alpha);
  });

  logger().addLogEntry("q_acc", [this]() {
    return rbd::dofToVector(robot().mb(), robot().mbc().alphaD);
  });


  logger().addLogEntry("tau", [this]() {
    return rbd::dofToVector(robot().mb(), robot().mbc().jointTorque);
  });


  logger().addLogEntry("delta_q_vel", [this]() {
    return miPredictorPtr->getJointVelocityJump(); 
  });

  logger().addLogEntry("delta_tau", [this]() {
    return miPredictorPtr->getTauJump();
  });

  logger().addLogEntry("test_delta_tau_norm", [this]() {
   return miPredictorPtr->getTauJump().norm(); 
  });

  logger().addLogEntry("test_delta_tau_impact_norm", [this]() {
   return miPredictorPtr->getBranchTauJump("r_wrist").norm(); 
  });
  logger().addLogEntry("test_delta_tau_l_sole_norm", [this]() {
   return miPredictorPtr->getBranchTauJump("l_sole").norm(); 
  });
logger().addLogEntry("test_delta_tau_r_sole_norm", [this]() {
   return miPredictorPtr->getBranchTauJump("r_sole").norm(); 
  });


  logger().addLogEntry("test_delta_tau_jacobian", [this]() {
		  Eigen::VectorXd temp = (
				  rbd::dofToVector(robot().mb(), robot().mbc().alpha) 
				  + rbd::dofToVector(robot().mb(), robot().mbc().alphaD)*miPredictorPtr->getImpactDuration_()
				  );
    return ((1/miPredictorPtr->getImpactDuration_())*miPredictorPtr->getJacobianDeltaTau()*temp  ).norm();
  });




  logger().addLogEntry("diff_delta_tau", [this]() {
		  Eigen::VectorXd temp = (
				  rbd::dofToVector(robot().mb(), robot().mbc().alpha) 
				  + rbd::dofToVector(robot().mb(), robot().mbc().alphaD)*miPredictorPtr->getImpactDuration_()
				  );
    return ((1/miPredictorPtr->getImpactDuration_())*miPredictorPtr->getJacobianDeltaTau()*temp - rbd::dofToVector(robot().mb(), robot().mbc().jointTorque) ).norm();
  });

logger().addLogEntry("delta_alpha_norm", [this]() {
   return miPredictorPtr->getJointVelocityJump().norm(); 
  });


logger().addLogEntry("diff_delta_alpha", [this]() {
		  Eigen::VectorXd temp = (
				  rbd::dofToVector(robot().mb(), robot().mbc().alpha) 
				  + rbd::dofToVector(robot().mb(), robot().mbc().alphaD)*miPredictorPtr->getImpactDuration_()
				  );
    return  (miPredictorPtr->getJacobianDeltaAlpha()*temp - miPredictorPtr->getJointVelocityJump()).norm();

  });

  logger().addLogEntry("realRobot_posW", [this]() { return realRobot().posW(); });

  gui()->addElement({"Forces"},
                    mc_rtc::gui::Force("LeftCoPForce_real", mc_rtc::gui::ForceConfig(mc_rtc::gui::Color(1., 0.2, 0.)),
                                       [this]() { return realRobot().surfaceWrench("LeftFoot"); },
                                       [this]() {
                                         auto & robot = realRobot();
                                         Eigen::Vector3d cop = robot.copW("LeftFoot");
                                         const sva::PTransformd X_0_s = robot.surface("LeftFoot").X_0_s(robot);
                                         sva::PTransformd surface(X_0_s.rotation(), cop);
                                         return surface;
                                       }),
                    mc_rtc::gui::Force("RightCoPForce_real", mc_rtc::gui::ForceConfig(mc_rtc::gui::Color(1., 0.2, 0.)),
                                       [this]() { return realRobot().surfaceWrench("RightFoot"); },
                                       [this]() {
                                         auto & robot = this->realRobots().robot();
                                         Eigen::Vector3d cop = robot.copW("RightFoot");
                                         const sva::PTransformd X_0_s = robot.surface("RightFoot").X_0_s(robot);
                                         sva::PTransformd surface(X_0_s.rotation(), cop);
                                         return surface;
                                       }),
                    mc_rtc::gui::Point3D("ZMP_real", [this]() {
                      auto & robot = this->realRobots().robot();
                      Eigen::Vector3d zmp;
                      try
                      {
                        zmp = robot.zmp({"LeftFootForceSensor", "RightFootForceSensor"}, Eigen::Vector3d::Zero(),
                                        Eigen::Vector3d{0., 0., 1.}, 100);
                        zmp.z() = 0;
                      }
                      catch(...)
                      {
                        zmp.setZero();
                      }
                      return zmp;
                    }));
  // std::cout << "Operational space dynamics Predictor is about to be created." << std::endl;
  // Add constriants:
}

const mc_rbdyn::Robot & Controller::realRobot() const
{
  return realRobots().robot();
}

bool Controller::rArmInContact()
{

  if(robot().forceSensor("RightHandForceSensor").force().x() > forceThreshold)
  {
    return true;
  }
  else
    return false;
}

void Controller::reset(const mc_control::ControllerResetData & data)
{

  /** First reset to get correct initial position of main robot */
  mc_control::MCController::reset(data);

  // Open the grippers
  for(auto & g : grippers)
  {
    g.second->setTargetOpening(1.0);
  }

  /** Initialize FSM stuff */
  mc_control::fsm::Controller::reset(data);
}

CONTROLLER_CONSTRUCTOR("RArmPushUnknownWall", Controller)
