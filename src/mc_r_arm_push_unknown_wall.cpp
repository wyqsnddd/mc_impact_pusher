#include "mc_r_arm_push_unknown_wall.h"


Controller::Controller(const mc_rbdyn::RobotModulePtr & rm, const double & dt, const mc_rtc::Configuration & conf): mc_control::fsm::Controller(rm, dt, conf){

	forceThreshold = config()("contact_detection")("ForceThreshold");

}
bool Controller::rArmInContact(){

	if(robot().forceSensor("RightHandForceSensor").force().x() > forceThreshold){
		return true;	
	}else
		return false;
}


void Controller::reset(const mc_control::ControllerResetData & data ){

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

