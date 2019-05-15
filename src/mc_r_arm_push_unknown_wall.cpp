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
  /*
  logger().addLogEntry("ZMP_World",
    [this]()
    {
      auto& robot = this->realRobots().robot();
      Eigen::Vector3d zmp;
      try
      {
        zmp = robot.zmp({"LeftFootForceSensor", "RightFootForceSensor"}, Eigen::Vector3d::Zero(),
  Eigen::Vector3d{0.,0.,1.}, 100); zmp.z() = 0; } catch(...)
      {
        zmp.setZero();
      }
      return zmp;
    });
    */
  logger().addLogEntry("realRobot_posW", [this]() { return realRobot().posW(); });

  gui()->addElement({"Forces"},
    mc_rtc::gui::Force("LeftCoPForce_real",
      mc_rtc::gui::ForceConfig(mc_rtc::gui::Color(1., 0.2, 0.)),
      [this]() {
         return realRobot().surfaceWrench("LeftFoot");
      },
      [this]() {
        auto& robot = realRobot();
        Eigen::Vector3d cop = robot.copW("LeftFoot");
        const sva::PTransformd X_0_s = robot.surface("LeftFoot").X_0_s(robot);
        sva::PTransformd surface(X_0_s.rotation(), cop);
        return surface;
      }),
    mc_rtc::gui::Force("RightCoPForce_real",
      mc_rtc::gui::ForceConfig(mc_rtc::gui::Color(1., 0.2, 0.)),
      [this]() {
         return realRobot().surfaceWrench("RightFoot");
      },
      [this]() {
        auto& robot = this->realRobots().robot();
        Eigen::Vector3d cop = robot.copW("RightFoot");
        const sva::PTransformd X_0_s = robot.surface("RightFoot").X_0_s(robot);
        sva::PTransformd surface(X_0_s.rotation(), cop);
        return surface;
      }),
    mc_rtc::gui::Point3D("ZMP_real",
      [this]() {
        auto& robot = this->realRobots().robot();
        Eigen::Vector3d zmp;
        try
        {
          zmp = robot.zmp({"LeftFootForceSensor", "RightFootForceSensor"}, Eigen::Vector3d::Zero(), Eigen::Vector3d{0.,0.,1.}, 100);
          zmp.z() = 0;
        } catch(...)
        {
          zmp.setZero();
        }
        return zmp;
      })
    );
  std::cout << "Operational space dynamics Predictor is about to be created." << std::endl;
  std::string impactBodyName("r_wrist");

  bool useLinearJacobian = true;
   miPredictorPtr.reset(new mi_impactPredictor(robot(), impactBodyName, useLinearJacobian, solver().dt()));
  //logger().log('Operational space dynamics Predictor is created. ');
  std::cout << "Operational space dynamics Predictor is created." << std::endl;

  logger().addLogEntry("ee_Vel_impact_jump", [this]() {
    return this->miPredictorPtr->getEeVelocityJump();
  });
  logger().addLogEntry("ee_Vel", [this]() -> Eigen::Vector3d {
    return this->realRobots().robot().mbc().bodyVelW
	    [
	    realRobots().robot().mb().bodyIndexByName("r_wrist")
	    ].linear();

  });


  logger().addLogEntry("l_ankle_Vel_impact_jump", [this]() {
    return this->miPredictorPtr->getEeVelocityJump("l_ankle");
  });
  logger().addLogEntry("r_ankle_Vel_impact_jump", [this]() {
    return this->miPredictorPtr->getEeVelocityJump("r_ankle");
  });

  logger().addLogEntry("ee_impact_impulse", [this]() {
    return this->miPredictorPtr->getImpulsiveForce();
  });

  logger().addLogEntry("l_ankle_predict_impact_impulse", [this]() {
    return this->miPredictorPtr->getImpulsiveForce("l_ankle");
  });

  logger().addLogEntry("l_ankle_ee_acc_force", [this]() {
    return this->miPredictorPtr->getEeAccForce("l_ankle");
  });

  logger().addLogEntry("r_ankle_ee_acc_force", [this]() {
    return this->miPredictorPtr->getEeAccForce("r_ankle");
  });

  logger().addLogEntry("r_ankle_predict_impact_impulse", [this]() {
    return this->miPredictorPtr->getImpulsiveForce("r_ankle");
  });


  logger().addLogEntry("q_vel_predict_impact_jump", [this]() {
    return this->miPredictorPtr->getJointVelocityJump();
  });



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
