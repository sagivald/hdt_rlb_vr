#include "control_group.h"

/*----------------------------------------------------------------------------
  initiation

  Input: None
  Output: 1
  Operation: This function initiates the control groups,
 *----------------------------------------------------------------------------*/
int ControlGroup::Init(robot_model::RobotModelPtr model, robot_state::RobotState *state, std::string robot_prefix, std::string name, std::string joint_cmd_topic, boost::function< void() > joy_cb) {
	ROS_INFO("ControlGroup::Init");

	 //Setting ROS Handle
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	// copy vars
	group_name = name;
	JoyControlCb = joy_cb;
	
	// publisher
	joint_cmd_pub = nh.advertise<sensor_msgs::JointState>(joint_cmd_topic, 3);

	// copy model
	kinematic_model = model;

	// get kinematic state
	kinematic_state = state;

	// move group
	move_group = new move_group_interface::MoveGroup(group_name);

	// joint model group
	joint_model_group = kinematic_model->getJointModelGroup(group_name);

	// path trajectory
	path_trajectory = new robot_trajectory::RobotTrajectory(kinematic_model, group_name);	

	// get joint names
	const std::vector<std::string> &joint_names = joint_model_group->getActiveJointModelNames();
	int num_joints = joint_names.size();
	ROS_INFO("%s model group has %d joints", group_name.c_str(), num_joints);
	
	// set cmd msg names and values
	joint_cmd_msg.name = joint_names;
	joint_cmd_msg.position.resize(num_joints);
	joint_cmd_msg.velocity.resize(num_joints);
	joint_cmd_msg.effort.resize(num_joints);
	std::fill(joint_cmd_msg.position.begin(), joint_cmd_msg.position.end(), 0.0);
	std::fill(joint_cmd_msg.velocity.begin(), joint_cmd_msg.velocity.end(), MAX_JOINT_VEL);
	std::fill(joint_cmd_msg.effort.begin(), joint_cmd_msg.effort.end(), 0.0);
	
	std::string action_name;
	// joint plan action servers and clients
	action_name = robot_prefix + group_name + JOINT_SPACE_PATH_ACTION;
	joint_space_path_as = new actionlib::SimpleActionServer<hdt_arm_control::JointSpacePathAction>(nh, action_name, boost::bind(&ControlGroup::JointSpacePathCb, this, _1), false);
	joint_space_path_as->start();
	joint_space_path_ac = new actionlib::SimpleActionClient<hdt_arm_control::JointSpacePathAction>(action_name, true);
	
	// task plan action servers and clients
	action_name = robot_prefix + group_name + TASK_SPACE_PATH_ACTION;
	task_space_path_as = new actionlib::SimpleActionServer<hdt_arm_control::TaskSpacePathAction>(nh, action_name, boost::bind(&ControlGroup::TaskSpacePathCb, this, _1), false);
	task_space_path_as->start();
	task_space_path_ac = new actionlib::SimpleActionClient<hdt_arm_control::TaskSpacePathAction>(action_name, true);

	// trajectory plan action servers and clients
	action_name = robot_prefix + group_name + TRAJECTORY_PATH_ACTION;
	trajectory_path_as = new actionlib::SimpleActionServer<hdt_arm_control::TrajectoryPathAction>(nh, action_name, boost::bind(&ControlGroup::TrajectoryPathCb, this, _1), false);
	trajectory_path_as->start();
	trajectory_path_ac = new actionlib::SimpleActionClient<hdt_arm_control::TrajectoryPathAction>(action_name, true);

	// set mode
	mode = MODE_NONE;

	return 1;
}

/*----------------------------------------------------------------------------
   step

  Input: None
  Output: None
  Operation: Switches between the different modes
 *----------------------------------------------------------------------------*/
void ControlGroup::Step(void) {
	// mode switch
	switch(GetMode()) {
		case(MODE_JOINT_SPACE):
			// check for path completion
			if(joint_space_path_ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
				//ROS_INFO("Joint path returned");
				PathStep();
			}
			break;
		case(MODE_TASK_SPACE):
			// check for path completion
			if(task_space_path_ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
				//ROS_INFO("Task path returned");
				PathStep();
			}
			break;
		case(MODE_TRAJECTORY):
			// check for path completion
			if(trajectory_path_ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
				//ROS_INFO("Task path returned");
				PathStep();
			}
			break;
		case(MODE_JOYSTICK):
			JoyControlCb();
			break;
		default:
			break;
	}
}

/*----------------------------------------------------------------------------
   joint space path callback
    
  Input: goal
  Output: None
  Operation: create path plan from current position and goal , joint space
 *----------------------------------------------------------------------------*/
void ControlGroup::JointSpacePathCb(const hdt_arm_control::JointSpacePathGoalConstPtr &goal) {
	hdt_arm_control::JointSpacePathResult result;
	ROS_INFO("ControlGroup::JointSpacePathCb");

	// get position map
	std::map<std::string, double> joint_map;
	joint_model_group->getVariableDefaultPositions(goal->pose, joint_map);

	// set target
	move_group->setJointValueTarget(joint_map);

	// create plan
	if(move_group->plan(path_plan)) {
		joint_space_path_as->setSucceeded(result);
	}
	else {
		ROS_ERROR("Joint space path failure");
		joint_space_path_as->setAborted(result);
	}

	// set path trajectory
	path_trajectory->setRobotTrajectoryMsg(*kinematic_state, path_plan.trajectory_);

	// start path timer
	path_start_time = ros::Time::now();	
}

/*----------------------------------------------------------------------------
   task space path callback

  Input: goal
  Output: None
  Operation: create path plan from current position and goal, task
 *----------------------------------------------------------------------------*/
void ControlGroup::TaskSpacePathCb(const hdt_arm_control::TaskSpacePathGoalConstPtr &goal) {
	hdt_arm_control::TaskSpacePathResult result;
	ROS_INFO("ControlGroup::TaskSpacePathCb");

	// set target
	move_group->clearPoseTargets();
	//move_group->setPoseTarget(goal->pose, fiducial_links[goal->marker]);
	move_group->setPoseTarget(goal->pose, IK_LINK_NAME);

	// create plan
	if(move_group->plan(path_plan)) {
		task_space_path_as->setSucceeded(result);
	}
	else {
		ROS_ERROR("Task space path failure");
		task_space_path_as->setAborted(result);
	}

	// set path trajectory
	path_trajectory->setRobotTrajectoryMsg(*kinematic_state, path_plan.trajectory_);
	
	// start path timer
	path_start_time = ros::Time::now();	
}

/*----------------------------------------------------------------------------
   trajectory path callback

  Input: goal
  Output: None
  Operation: create path plan from current position and goal, trajectory
 *----------------------------------------------------------------------------*/
void ControlGroup::TrajectoryPathCb(const hdt_arm_control::TrajectoryPathGoalConstPtr &goal) {
	hdt_arm_control::TrajectoryPathResult result;
	ROS_INFO("ControlGroup::TrajectoryPathCb");

	// generate plan
	move_group->setEndEffectorLink(IK_LINK_NAME);

	// create plan
	if(move_group->computeCartesianPath(goal->waypoints, 0.01, 0.0, path_plan.trajectory_) >= 1.0) {
		trajectory_path_as->setSucceeded(result);
	}
	else {
		ROS_ERROR("Task space path failure");
		trajectory_path_as->setAborted(result);
	}

	// set path trajectory
	path_trajectory->setRobotTrajectoryMsg(*kinematic_state, path_plan.trajectory_);
	
	// start path timer
	path_start_time = ros::Time::now();	
}

/*----------------------------------------------------------------------------
  path step

  Input: None
  Output: None
  Operation: progresses path execution in 1 step
 *----------------------------------------------------------------------------*/
void ControlGroup::PathStep(void) {
	// interpolate path
	ros::Duration path_time = ros::Time::now() - path_start_time;
	moveit::core::RobotStatePtr path_state(new robot_state::RobotState(kinematic_model));
	if(path_trajectory->getStateAtDurationFromStart(path_time.toSec(), path_state)) {
		// update joint cmd
		std::vector<double> joint_positions;
		path_state->copyJointGroupPositions(joint_model_group, joint_positions);
		joint_cmd_msg.position = joint_positions;

		// update kinematic state to latest cmd
		kinematic_state->setVariableValues(joint_cmd_msg);
	}

	// check for path end
	if(ros::Time::now() >= path_start_time + ros::Duration(path_plan.trajectory_.joint_trajectory.points.back().time_from_start)) {
		ChangeMode(MODE_NONE);
	}
}

/*----------------------------------------------------------------------------
  change mode

  Input: next mode
  Output: true
  Operation: change modes
 *----------------------------------------------------------------------------*/
bool ControlGroup::ChangeMode(ModeType new_mode) {
	int ret = true;
	
	// mode dependent switching
	switch(new_mode) {
		default:
			break;
	}

	// change mode
	mode = new_mode;

	// send feedback
	ROS_INFO("ControlGroup::ChangeMode %d", (int)mode);

	return ret;
}

/*----------------------------------------------------------------------------
  publish

  Input: None
  Output: None
  Operation: publishes the joint command message
 *----------------------------------------------------------------------------*/
void ControlGroup::Publish(void) {
	// publish joint cmd msg
	joint_cmd_msg.header.seq++;
	joint_cmd_msg.header.stamp = ros::Time::now();
	joint_cmd_pub.publish(joint_cmd_msg);

}

