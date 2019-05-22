#include "mc_r_arm_push_unknown_wall.h"

Controller::Controller(const mc_rbdyn::RobotModulePtr & rm, const double & dt, const mc_rtc::Configuration & conf)
: mc_control::fsm::Controller(rm, dt, conf)
{

  forceThreshold = config()("contact_detection")("ForceThreshold");

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
    Eigen::VectorXd eeVelJump = miPredictorPtr->getEeVelocityJump();
    return eeVelJump;
  });
  logger().addLogEntry("ee_Vel", [this]() {
    Eigen::VectorXd eeVel =
        realRobots().robot().mbc().bodyVelW[realRobots().robot().mb().bodyIndexByName("r_wrist")].vector();

    return eeVel;
  });

  logger().addLogEntry("l_ankle_Vel_impact_jump", [this]() {
    Eigen::VectorXd eeVelJump = miPredictorPtr->getEeVelocityJump("l_sole");
    return eeVelJump;
  });

  logger().addLogEntry("l_ankle_Vel", [this]() {
    Eigen::VectorXd leeVelJump = realRobots().robot().mbc().bodyVelW[realRobots().robot().mb().bodyIndexByName("l_sole")].vector();
    
    return leeVelJump;
  });

  logger().addLogEntry("r_ankle_Vel_impact_jump", [this]() {
    Eigen::VectorXd eeVelJump = miPredictorPtr->getEeVelocityJump("r_sole");
    return eeVelJump;
  });

  logger().addLogEntry("r_ankle_Vel", [this]() {
    Eigen::VectorXd reeVelJump = realRobots().robot().mbc().bodyVelW[realRobots().robot().mb().bodyIndexByName("r_sole")].vector();
    
    return reeVelJump;
  });

  logger().addLogEntry("ee_impact_impulse", [this]() {
    Eigen::VectorXd eeImpulse = miPredictorPtr->getImpulsiveForce();
    return eeImpulse;
  });

  logger().addLogEntry("l_ankle_predict_impact_impulse", [this]() {
    Eigen::VectorXd ankleImpulse = miPredictorPtr->getImpulsiveForce("l_sole");
    return ankleImpulse;
  });

  logger().addLogEntry("r_ankle_predict_impact_impulse", [this]() {
    Eigen::VectorXd ankleImpulse = miPredictorPtr->getImpulsiveForce("r_sole");
    return ankleImpulse;
  });
  logger().addLogEntry("q_position", [this]() {
    Eigen::VectorXd q = rbd::dofToVector(robot().mb(), robot().mbc().q);
    return q;
  });


  logger().addLogEntry("q_vel", [this]() {
    Eigen::VectorXd qvel = rbd::dofToVector(robot().mb(), robot().mbc().alpha);
    return qvel;
  });

  logger().addLogEntry("q_acc", [this]() {
    Eigen::VectorXd qacc = rbd::dofToVector(robot().mb(), robot().mbc().alphaD);
    return qacc;
  });


  logger().addLogEntry("tau", [this]() {
    Eigen::VectorXd tau = rbd::dofToVector(robot().mb(), robot().mbc().jointTorque);
    return tau;
  });


  logger().addLogEntry("delta_q_vel", [this]() {
    Eigen::VectorXd delta_qvel = miPredictorPtr->getJointVelocityJump(); 
    return delta_qvel;
  });

  logger().addLogEntry("delta_tau", [this]() {
    Eigen::VectorXd delta_tau = miPredictorPtr->getTauJump();
    return delta_tau;
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
  std::cout << "Kinematics and Dynamics constraints are created." << std::endl;
  solver().addConstraintSet(kinematicsConstraint);
  solver().addConstraintSet(dynamicsConstraint);
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
