// Header for control_group

// If the header is not defined then define it
#ifndef control_group_h
#define control_group_h

// Including Used Libraries

// ROS Libraries
#include "ros/ros.h"

// Action Libraries
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include "hdt_arm_control/JointSpacePathAction.h"
#include "hdt_arm_control/TaskSpacePathAction.h"
#include "hdt_arm_control/TrajectoryPathAction.h"

// MoveIT Libraries
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// Defining Global Variables

#define MAX_JOINT_VEL				2.0944
#define IK_LINK_NAME				"endpoint_link"
#define JOINT_SPACE_PATH_ACTION		"/joint_space_path"
#define TASK_SPACE_PATH_ACTION		"/task_space_path"
#define TRAJECTORY_PATH_ACTION		"/trajectory_path"

// Control Group Class
class ControlGroup {
public:
	// setup
	ControlGroup() {};
	~ControlGroup() {};
	int Init(robot_model::RobotModelPtr model, robot_state::RobotState *state, std::string robot_prefix, std::string name, std::string joint_cmd_topic, boost::function< void() > joy_cb);
	// Mode Types
	typedef enum ModeType {
		MODE_NONE = 		-1,
		MODE_JOINT_SPACE = 	0,
		MODE_TASK_SPACE = 	1,
		MODE_TRAJECTORY =	2,
		MODE_JOYSTICK = 	3,
		NUM_MODES =			4
	} ModeType;

	 // action servers and clients
	actionlib::SimpleActionServer<hdt_arm_control::JointSpacePathAction> *joint_space_path_as;
	actionlib::SimpleActionClient<hdt_arm_control::JointSpacePathAction> *joint_space_path_ac;
	actionlib::SimpleActionServer<hdt_arm_control::TaskSpacePathAction> *task_space_path_as;
	actionlib::SimpleActionClient<hdt_arm_control::TaskSpacePathAction> *task_space_path_ac;
	actionlib::SimpleActionServer<hdt_arm_control::TrajectoryPathAction> *trajectory_path_as;
	actionlib::SimpleActionClient<hdt_arm_control::TrajectoryPathAction> *trajectory_path_ac;

	// moveit variables
	robot_state::JointModelGroup *joint_model_group;
	moveit::planning_interface::MoveGroup::Plan path_plan;
	robot_trajectory::RobotTrajectory *path_trajectory;
	move_group_interface::MoveGroup *move_group;

	// callbacks
	void JointSpacePathCb(const hdt_arm_control::JointSpacePathGoalConstPtr &goal);
	void TaskSpacePathCb(const hdt_arm_control::TaskSpacePathGoalConstPtr &goal);
	void TrajectoryPathCb(const hdt_arm_control::TrajectoryPathGoalConstPtr &goal);
	
	// mode variables
	bool ChangeMode(ModeType new_mode);
	ModeType GetMode(void) {return (ModeType)mode;};

	// step functions
	void PathStep(void);
	void Step(void);
	void Publish(void);
	
	// msg and publisher variables
	sensor_msgs::JointState joint_cmd_msg;
	ros::Publisher joint_cmd_pub;

	// kinematic state variables
	robot_state::RobotState *kinematic_state;
private:
	std::string group_name;
	
	ModeType mode;
	
	// timer
	ros::Time path_start_time;

	// kinematic model
	robot_model::RobotModelPtr kinematic_model;

	boost::function< void() > JoyControlCb;
};

#endif // control_group_h
