// Header for rlb_control

// If the header is not defined then define it
#ifndef rlb_control_h
#define rlb_control_h

// Including Used Libraries

// ROS Libraries
#include "ros/ros.h"

// Action Libraries
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include "hdt_rlb_control/JointSpacePathAction.h"
#include "hdt_rlb_control/TaskSpacePathAction.h"
#include "hdt_rlb_control/TrajectoryPathAction.h"


// Geometry Messages
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Transform.h"

// Sensor Messages 
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/MultiDOFJointState.h"
#include "sensor_msgs/JointState.h"

// Standard Messages
#include "std_msgs/Float64.h"

// MoveIT Libraries
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/dynamics_solver/dynamics_solver.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/move_group_interface/move_group.h>


// Defining Global Variables

// general
#define ROBOT_PREFIX				"hdt_rlb/"
#define ROS_NAME					"rlb_control"
#define SAMPLE_PERIOD				0.1
#define JOY_TOPIC					"/joy"
#define JOY_NUM_AXES				8
#define JOY_NUM_BUTTONS				11
#define IK_LINK_NAME				"endpoint_link"
#define MAX_JOINT_VEL				2.0944
#define MAX_TRANS_VEL				0.25			// 15 cm/sec
#define MAX_ROT_VEL					0.7845			// 45 deg/sec

// adroit communications
#define COMS_READY_SERVICE			"/hdt_adroit_coms/coms_ready"
#define COMS_READY_TIMEOUT			5
#define JOINT_TELEM_TOPIC			"/hdt_adroit_coms/joint_telem"
#define JOINT_COMMAND_TOPIC			"/hdt_adroit_coms/joint_cmd"

// moveit
#define ARM_GROUP_NAME				"arm"
#define JOINT_SPACE_PATH_ACTION		"hdt_rlb_control/joint_space_path"
#define TASK_SPACE_PATH_ACTION		"hdt_rlb_control/task_space_path"
#define TRAJECTORY_PATH_ACTION		"hdt_rlb_control/trajectory_path"

// gazebo
#define CONTROLLER_SUFFIX			"_position_controller/command"

// Main Class for the Robotic Arm
class rlbControl {
public:
	// setup
	rlbControl() {}
	~rlbControl() {};
	int Init(void);

	// State Machine and Mode Types of the Robotic Arm
	typedef enum StateType {
		STATE_NONE = 		-1,
		STATE_OFF = 		0,
		STATE_ON = 			1,
		STATE_FAULT = 		2,
	} StateType;
	typedef enum ModeType {
		MODE_NONE = 		-1,
		MODE_FREE_SPACE = 	0,
		MODE_APPROACH = 	1,
		MODE_GRIP =			2,
		MODE_HARD_STOP = 	3,
		MODE_DAMPED_STOP = 	4,
		MODE_HOME = 		5,
		MODE_STOWED = 		6,
		MODE_TEACH = 		7,
	} ModeType;

	typedef struct StateModeType {
		StateType state;
		ModeType mode;
	} StateModeType;

	// change state and mode functions
	bool ChangeState(StateType new_state);
	bool ChangeMode(ModeType new_mode);
private:
	int num_joints, num_links;
	StateModeType state_mode;


	sensor_msgs::MultiDOFJointState task_space_fb_msg;
	sensor_msgs::JointState joint_space_fb_msg;

	// get state and mode functions
	StateType GetState(void) {return (StateType)state_mode.state;};
	ModeType GetMode(void) {return (ModeType)state_mode.mode;};

	// CallBacks functions
	void TimerCb(const ros::TimerEvent &event);
	void JoyCallback(const sensor_msgs::Joy& msg);
	void JointSpacePathCb(const hdt_rlb_control::JointSpacePathGoalConstPtr &goal);
	void TaskSpacePathCb(const hdt_rlb_control::TaskSpacePathGoalConstPtr &goal);
	void TrajectoryPathCb(const hdt_rlb_control::TrajectoryPathGoalConstPtr &goal);
	bool CollisionCb(const planning_scene::PlanningScene *planning_scene, robot_state::RobotState *state, const robot_state::JointModelGroup *group, const double *joint_group_variable_values);
	void JointTelemCb(const sensor_msgs::JointState& msg);

	// adroit communications variables
	bool use_adroit_coms;
	sensor_msgs::JointState joint_telem_msg;
	sensor_msgs::JointState joint_cmd_msg;
	ros::Subscriber joint_telem_sub;
	ros::Publisher joint_cmd_pub;

	// gazebo variables
	bool use_gazebo;
	std::vector<std_msgs::Float64> gazebo_cmd_msg;
	std::vector<ros::Publisher> gazebo_cmd_pub;
	sensor_msgs::JointState gazebo_telem_msg;
	ros::Subscriber gazebo_telem_sub;
	
	// steps functions
	void PathStep(void);
	void TeachStep(void);

	// timers variables
	ros::Timer sample_loop_timer;
	ros::Time path_start_time;
	ros::Time state_mode_time;

	// moveit variables
	robot_model_loader::RobotModelLoader *model_loader;
	robot_model::RobotModelPtr kinematic_model;
	robot_state::RobotState *kinematic_state;
	robot_state::JointModelGroup *joint_model_group;
	dynamics_solver::DynamicsSolver *gravity_solver;
	planning_scene::PlanningScene *collision_scene;
	collision_detection::AllowedCollisionMatrix *allowed_collision_matrix;
	std::vector<double> joint_accelerations;
	std::vector<double> gravity_effort;
	std::vector<geometry_msgs::Wrench> link_wrenches;
	moveit::planning_interface::MoveGroup::Plan path_plan;
	robot_trajectory::RobotTrajectory *path_trajectory;
	move_group_interface::MoveGroup *move_group;

	// joy variables
	ros::Subscriber joy_sub;
	sensor_msgs::Joy joy_msg;

 	// action servers and clients
	actionlib::SimpleActionServer<hdt_rlb_control::JointSpacePathAction> *joint_space_path_as;
	actionlib::SimpleActionClient<hdt_rlb_control::JointSpacePathAction> *joint_space_path_ac;
	actionlib::SimpleActionServer<hdt_rlb_control::TaskSpacePathAction> *task_space_path_as;
	actionlib::SimpleActionClient<hdt_rlb_control::TaskSpacePathAction> *task_space_path_ac;
	actionlib::SimpleActionServer<hdt_rlb_control::TrajectoryPathAction> *trajectory_path_as;
	actionlib::SimpleActionClient<hdt_rlb_control::TrajectoryPathAction> *trajectory_path_ac;
};

#endif // rlb_control_h
