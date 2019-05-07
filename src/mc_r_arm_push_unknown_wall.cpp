#include "mc_r_arm_push_unknown_wall.h"


Controller::Controller(const mc_rbdyn::RobotModulePtr & rm, const double & dt, const mc_rtc::Configuration & conf): mc_control::fsm::Controller(rm, dt, conf){

	forceThreshold = config()("contact_detection")("ForceThreshold");
	/*
	std::cout<<"mc_rtc says the robot has "<<this->robot().mb().nrJoints()<<" joints."<<std::endl;
	std::cout<<"mc_rtc says the robot has "<<this->robot().mb().nrDof()<<" DOF."<<std::endl;

	//std::cout<<"mc_rtc says the robot mass matrix has dimension: "<<
	std::cout<<"Ready to load the predictor robot."<<std::endl;

	auto robot_module = mc_rbdyn::RobotLoader::get_robot_module("HRP4GRF");
	//auto robot_module = mc_rbdyn::RobotLoader::get_robot_module("HRP4");

	if(robot_module == nullptr) {
		std::cout<<"null pointer robot_module."<<std::endl;
	}else{
	
		std::cout<<"The predictor robot name is: "<<robot_module->name <<std::endl;
		std::cout<<"The predictor robot path is: "<<robot_module->urdf_path<<std::endl;
	
	}
	std::cout<<"Loaded the predictor robot module."<<std::endl;
	robotsGRF_.load(*robot_module);
	std::cout<<"Loaded the predictor robot."<<std::endl;

	rbd::ForwardDynamics fd(this->robot().mb());
	fd.computeH(robot().mb(), this->robot().mbc());
	fd.computeC(robot().mb(), this->robot().mbc());
	std::cout<<"mc_rtc says: the robot mass matrix row is: "<<fd.H().rows()<<" the column is: "<<fd.H().cols()<<std::endl;
	std::cout<<"Forward kinematics calculated.."<<std::endl;
	auto M1 = fd.H().block(6,6, 50, 50);

	rbd::ForwardDynamics fd_grf(robotsGRF_.robot().mb());
	fd_grf.computeH(robotsGRF_.robot().mb(), robotsGRF_.robot().mbc());
	fd_grf.computeC(robotsGRF_.robot().mb(), robotsGRF_.robot().mbc());

	std::cout<<"mc_rtc says: the GRF robot mass matrix row is: "<<fd.H().rows()<<" the column is: "<<fd.H().cols()<<std::endl;
	auto M2= fd_grf.H().block(12,12, 50, 50);
	std::cout<<"Difference is: "<<std::endl<<M1 - M2<<std::endl;
	*/	
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

