#include "PrepareHand.h"


#include "../mc_r_arm_push_unknown_wall.h"

#include <mc_tasks/MetaTaskLoader.h>


void PrepareHandState::configure(const mc_rtc::Configuration & config){
	config("EfThreshold", efThreshold_);
}


void PrepareHandState::start(mc_control::fsm::Controller& ctlInput){
	
	auto & ctl = static_cast<Controller&>(ctlInput);

	rEfTaskPtr_ = std::make_shared<mc_tasks::EndEffectorTask>(
			ctl.config()("hrp4")("rightEfTask")("bodyName"),
			ctl.robots(),
			0,
			ctl.config()("hrp4")("rightEfTask")("stiffness"),
			ctl.config()("hrp4")("rightEfTask")("weight")
			);
	std::vector<std::string> right_joints = {
		"R_SHOULDER_P",
		"R_SHOULDER_R",
		"R_SHOULDER_Y",
		"R_ELBOW_P",
		"R_WRIST_Y",
		"R_WRIST_P",
		"R_WRIST_R"
	};
	rEfTaskPtr_->selectActiveJoints(ctl.solver(), right_joints);
	ctl.solver().addTask(rEfTaskPtr_);

	ctl.comTaskPtr = mc_tasks::MetaTaskLoader::load(ctl.solver(), ctl.config()("com"));
	ctl.solver().addTask(ctl.comTaskPtr);

	rTransformZero_ = rEfTaskPtr_->get_ef_pose();
	rEfTaskPtr_->set_ef_pose(rTransformZero_);

	Eigen::Vector3d translation_offset;
	translation_offset = ctl.config()("states")("Prepare")("raiseHandOffset");
	// We need to 
	//auto desiredRotation =  sva::RotY(-M_PI*2/3);
	//auto desiredRotation =  sva::RotY(-M_PI/2);
	auto desiredRotation =  sva::RotZ(-M_PI/2);

	//desiredRotation = sva::RotY(-M_PI/2)
	sva::PTransformd right_raise_hand( desiredRotation*rTransformZero_.rotation().inverse(), translation_offset);
	rEfTaskPtr_->add_ef_pose(right_raise_hand);


}

bool PrepareHandState::run(mc_control::fsm::Controller & ctl){
	if( rEfTaskPtr_->eval().norm() <= efThreshold_)
	{
		// Output the transition signal such that we can move on according to the transitions 
		output("RightHandReady");
		return true;
	}

	return false;
}


void PrepareHandState::teardown(mc_control::fsm::Controller & ctl_ )
{
	auto & ctl = static_cast<Controller&>(ctl_);
	ctl.solver().removeTask(rEfTaskPtr_);


}

EXPORT_SINGLE_STATE("PrepareHand", PrepareHandState)

