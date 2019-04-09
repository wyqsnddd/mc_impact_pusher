#include "DynamicContact.h"

#include "../mc_r_arm_push_unknown_wall.h"

#include <mc_tasks/MetaTaskLoader.h>

void DynamicContactState::configure(const mc_rtc::Configuration & config)
{
}


void DynamicContactState::start(mc_control::fsm::Controller & ctlInput)
{
	auto & ctl = static_cast<Controller&>(ctlInput);

	rPosTaskPtr_ = std::make_shared<mc_tasks::PositionTask>(
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
	rPosTaskPtr_->selectActiveJoints(ctl.solver(), right_joints);
	ctl.solver().addTask(rPosTaskPtr_);

	//rTransformZero_ = rPosTaskPtr_->get_ef_pose();
	// rEfTaskPtr_->set_ef_pose(rTransformZero_);

	
	Eigen::Vector3d referenceVelocity, pushDepth, currentPos;
	pushDepth= ctl.config()("states")("Contact")("pushDepth");
	referenceVelocity= ctl.config()("states")("Contact")("contactVelocity");

	currentPos = rPosTaskPtr_->position();
	// Set the target position, which is supposed to be far away. 
	rPosTaskPtr_->position(currentPos + pushDepth);
	// Set the reference velocity 
	rPosTaskPtr_->refVel(referenceVelocity);
}

bool DynamicContactState::run(mc_control::fsm::Controller & ctl)
{
	// std::cout<<"The left ef error is: "<< lEfTaskPtr_->eval().norm() << ", the right ef error is: " << rEfTaskPtr_->eval().norm() <<std::endl;
	if( rPosTaskPtr_->eval().norm() <= 0.01)
	{
		// Output the transition signal such that we can move on according to the transitions 
		// ctl.solver().removeTask(rPosTaskPtr_);
		output("RightHandFinished");
		return true;
	}

	return false;
}

void DynamicContactState::teardown(mc_control::fsm::Controller & ctl_ )
{
	auto & ctl = static_cast<Controller&>(ctl_);
	ctl.solver().removeTask(rPosTaskPtr_);


}


EXPORT_SINGLE_STATE("DynamicContact", DynamicContactState)
