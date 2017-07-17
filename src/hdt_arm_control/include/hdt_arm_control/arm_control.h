// Header for arm_control

// If the header is not defined then define it
#ifndef arm_control_h
#define arm_control_h

// Including Used Libraries

// ROS Libraries
#include "ros/ros.h"

// Geometry Messages Libraries
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Transform.h"

// Leap Motion Libraries
#include "math.h"
#include "leap_motion/leapros.h"

// Sensor Messages Libraries
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/MultiDOFJointState.h"
#include "sensor_msgs/JointState.h"

// Standard Messages
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

// MoveIT Libraries
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/dynamics_solver/dynamics_solver.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_matrix.h>
#include "moveit_msgs/MoveGroupActionGoal.h"

// Hocu Driver Header
#include "hdt_hocu_driver/hocu_driver.h"

// Control Groups Header
#include "control_group.h"

// Defining Global Variables

// general
#define ROBOT_PREFIX				"hdt_arm_control/"
#define ROS_NAME				"arm_control"
#define SAMPLE_PERIOD				0.1
#define JOY_TOPIC				"/joy"
#define JOY_NUM_AXES				8
#define JOY_NUM_BUTTONS				11
#define MAX_TRANS_VEL				0.15			// 15 cm/sec
#define MAX_ROT_VEL				0.7845			// 45 deg/sec
#define MAX_ROTATION				0.7854
#define MAX_HAND_VEL				3.1416

// adroit communications
#define COMS_READY_SERVICE			"/hdt_adroit_coms/coms_ready"
#define COMS_READY_TIMEOUT			5
#define JOINT_TELEM_TOPIC			"/hdt_adroit_coms/joint_telem"
#define JOINT_COMMAND_TOPIC			"/hdt_adroit_coms/joint_cmd"

// joint_states
#define JOINT_STATES_TOPIC			"/joint_states"

// path plan
#define PP_TOPIC					"/move_group/goal"
#define PP_EXECUTE					"/move_group/goal/goal/planning_options/plan_only"

// moveit
#define ARM_GROUP_NAME				"arm"
#define HAND_GROUP_NAME				"hand"

// gazebo
#define CONTROLLER_SUFFIX			"_position_controller/command"

// leap motion
#define LEAP_MOTION_TOPIC			"leapmotion/data"
#define PI					3.14159265359
#define START_X					-0.105
#define GRADIENT_X				-0.0064333
#define START_Y					-0.58515154
#define GRADIENT_Y				0.0038484
#define START_Z					-0.688
#define GRADIENT_Z				0.00696
#define K_YAW					1
#define K_PITCH					1.5
#define K_ROLL					1.2
#define K_THETA_INDEX				0.848826363*1.8
#define K_THETA_MIDDLE				0.848826363*2.2
#define K_THETA_THUMB				-1.80180182
#define L_THETA_THUMB				2.15 
#define K_PHI_THUMB				3.272491667
#define L_PHI_THUMB				-4.58148833


// Arm Control Class
class ArmControl {
public:
	// setup
	ArmControl() {}
	~ArmControl() {};
	int Init(void);

	// State Machine of the Robotic Arm
	typedef enum StateType {
		STATE_NONE = 		-1,
		STATE_OFF = 		0,
		STATE_ON = 			1,
		STATE_FAULT = 		2,
		NUM_STATES =		3
	} StateType;

	// change state function
	bool ChangeState(StateType new_state);
private:
	int num_joints;
	std::string model_name;
	StateType state;

	sensor_msgs::MultiDOFJointState task_space_fb_msg;
	sensor_msgs::JointState joint_space_fb_msg;

	// get state function
	StateType GetState(void) {return (StateType)state;};

	// control groups
	ControlGroup arm_group, hand_group;

	// CallBacks functions
	void TimerCb(const ros::TimerEvent &event);
	void JoyCallback(const sensor_msgs::Joy& msg);
	bool CollisionCb(const planning_scene::PlanningScene *planning_scene, robot_state::RobotState *state, const robot_state::JointModelGroup *group, const double *joint_group_variable_values);
	void JointTelemCb(const sensor_msgs::JointState& msg);
	void JointGuiCb(const sensor_msgs::JointState& msg);
	void PPCb(const moveit_msgs::MoveGroupActionGoal& msg);
	void HocuCallback(const sensor_msgs::Joy& msg);
	void LeapCb(const leap_motion::leapros& msg);
	void JoyControlArmCb(void);
	void JoyControlHandCb(void);

	// adroit communications variables
	bool use_adroit_coms;
	sensor_msgs::JointState joint_telem_msg;
	ros::Subscriber joint_telem_sub;

	// gazebo variables
	bool use_gazebo;
	std::vector<std_msgs::Float64> gazebo_cmd_msg;
	std::vector<ros::Publisher> gazebo_cmd_pub;
	sensor_msgs::JointState gazebo_telem_msg;
	ros::Subscriber gazebo_telem_sub;
	
	// step function
	void TeachStep(void);

	// timers variables
	ros::Timer sample_loop_timer;
	ros::Time state_mode_time;

	// moveit variables
	robot_model_loader::RobotModelLoader *model_loader;
	robot_model::RobotModelPtr kinematic_model;
	robot_state::RobotState *kinematic_state;
	dynamics_solver::DynamicsSolver *gravity_solver;
	planning_scene::PlanningScene *collision_scene;
	collision_detection::AllowedCollisionMatrix *allowed_collision_matrix;
	std::vector<double> joint_accelerations;
	std::vector<double> gravity_effort;
	std::vector<geometry_msgs::Wrench> link_wrenches;

	// hocu variables
	bool use_hocu;
	ros::Subscriber hocu_sub;
	sensor_msgs::Joy hocu_msg, hocu_prev;
	Eigen::Affine3d hold_state;

	// joy variables
	ros::Subscriber joy_sub;
	sensor_msgs::Joy joy_msg;

	// gui variables
	bool use_gui;
	ros::Subscriber joint_states_sub;
	sensor_msgs::JointState joint_states_msg;

	// Path Planning variables
	bool use_pp;
	ros::Subscriber pp_sub;
	bool plan_only;
	float pp_goal[6];

	// leap motion variables
	bool use_leap;
	ros::Subscriber leap_sub;
	geometry_msgs::Pose endpoint_pose; 
	double theta_index,theta_middle,theta_thumb,phi_thumb;
	double FingerAngle (geometry_msgs::Point a,geometry_msgs::Point b,geometry_msgs::Vector3 u);
	geometry_msgs::Quaternion RPYtoQuat (double roll,double pitch,double yaw);


};

#endif // arm_control_h
