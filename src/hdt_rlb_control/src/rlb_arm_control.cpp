#include <eigen_conversions/eigen_msg.h>

#include "hdt_adroit_driver/ComsReady.h"

#include "arm_control.h"



geometry_msgs::Pose test_pose;
//std::vector<geometry_msgs::Pose> test_waypoints;

/*----------------------------------------------------------------------------
  init
 *----------------------------------------------------------------------------*/
int ArmControl::Init(void) {
	ROS_INFO("ArmControl::Init");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	// get configuration parameters
	pnh.param<bool>("use_adroit_coms", use_adroit_coms, true);
	pnh.param<bool>("use_gazebo", use_gazebo, false);
	pnh.param<bool>("use_hocu", use_hocu, false);
	pnh.param<bool>("use_gui", use_gui, false);
	pnh.param<bool>("use_pp", use_pp, false);
	pnh.param<bool>("use_leap", use_leap, false);

	if(use_adroit_coms) {
		// wait for adroit coms service
		if(ros::service::waitForService(COMS_READY_SERVICE, ros::Duration(COMS_READY_TIMEOUT))) {
			ros::ServiceClient coms_ready_client = nh.serviceClient<hdt_adroit_driver::ComsReady>(COMS_READY_SERVICE);
			hdt_adroit_driver::ComsReady srv;
		
			// call coms ready service
			if(coms_ready_client.call(srv)) {
				joint_telem_msg = srv.response.telem;
				//ROS_INFO_STREAM(joint_telem_msg);				
			}
			else {
				ROS_ERROR("Failed to call COMS service");
				return -1;
			}
		}
		else {
			ROS_ERROR("No COMS service");
			return -1;
		}
	}

	// adroit coms publishers and subscribers
	if(use_adroit_coms) {
		joint_telem_sub = nh.subscribe(JOINT_TELEM_TOPIC, 3, &ArmControl::JointTelemCb, this);
	}

	if(use_gui){
		joint_states_sub = nh.subscribe(JOINT_STATES_TOPIC, 3, &ArmControl::JointGuiCb, this);
	}

	if(use_pp){
		//plan_only_sub = nh.subscribe(PP_EXECUTE, 3, &ArmControl::isPPeXecuteCb, this);
		ROS_INFO("Path Plan Subscriber Initialized");
		pp_sub = nh.subscribe(PP_TOPIC, 3, &ArmControl::PPCb, this);
	}

	if(use_leap){
		leap_sub = nh.subscribe(LEAP_MOTION_TOPIC, 10, &ArmControl::LeapCb, this);
	}

	// check for robot description
	std::string description;
	if(nh.getParam("robot_description", description)) {
		// load robot model
		model_loader = new robot_model_loader::RobotModelLoader("robot_description");
		kinematic_model = model_loader->getModel();
		model_name = kinematic_model->getName();
		
		// get kinematic state
		kinematic_state = new robot_state::RobotState(kinematic_model);

		// control groups 
		arm_group.Init(kinematic_model, kinematic_state, ROBOT_PREFIX, ARM_GROUP_NAME, JOINT_COMMAND_TOPIC, boost::bind(&ArmControl::JoyControlArmCb, this));
		hand_group.Init(kinematic_model, kinematic_state, ROBOT_PREFIX, HAND_GROUP_NAME, JOINT_COMMAND_TOPIC, boost::bind(&ArmControl::JoyControlHandCb, this));

		// dynamics solver gravity vector
		geometry_msgs::Vector3 gravity_vector;
		gravity_vector.x = 0.0;
		gravity_vector.y = 0.0;
		gravity_vector.z = 9.81;

		// initialize ddynamics solver
		gravity_solver = new dynamics_solver::DynamicsSolver(kinematic_model, ARM_GROUP_NAME, gravity_vector);
	
		// collision planning scene
		collision_scene = new planning_scene::PlanningScene(kinematic_model);
		allowed_collision_matrix = new collision_detection::AllowedCollisionMatrix(collision_scene->getAllowedCollisionMatrix());
		allowed_collision_matrix->setEntry("index_dist", "thumb_dist", true);

		// get joint names
		const std::vector<std::string> &joint_names = arm_group.joint_model_group->getActiveJointModelNames();
		num_joints = joint_names.size();

		// dynamic variables
		joint_accelerations.resize(num_joints);
		gravity_effort.resize(num_joints);
		std::fill(joint_accelerations.begin(), joint_accelerations.end(), 0.0);
		std::fill(gravity_effort.begin(), gravity_effort.end(), 0.0);

		// set initial positions
		if(use_adroit_coms) {
			// copy initial positions from state
			//kinematic_state->setVariableValues(joint_telem_msg);
			//kinematic_state->copyJointGroupPositions(ARM_GROUP_NAME, arm_group.joint_cmd_msg.position);
			arm_group.kinematic_state->setVariableValues(joint_telem_msg);
			arm_group.kinematic_state->copyJointGroupPositions(ARM_GROUP_NAME, arm_group.joint_cmd_msg.position);
			hand_group.kinematic_state->setVariableValues(joint_telem_msg);
			hand_group.kinematic_state->copyJointGroupPositions(HAND_GROUP_NAME, hand_group.joint_cmd_msg.position);
		}
		else if(use_gui){
			arm_group.kinematic_state->setVariableValues(joint_states_msg);
			arm_group.kinematic_state->copyJointGroupPositions(ARM_GROUP_NAME, arm_group.joint_cmd_msg.position);
			hand_group.kinematic_state->setVariableValues(joint_states_msg);
			hand_group.kinematic_state->copyJointGroupPositions(HAND_GROUP_NAME, hand_group.joint_cmd_msg.position);
		}
		else if(use_gazebo) {
		//if(use_gazebo) {
			// copy initial positions from gazebo
			//kinematic_state->setVariableValues(arm_group.joint_cmd_msg);
			arm_group.kinematic_state->setVariableValues(arm_group.joint_cmd_msg);
			hand_group.kinematic_state->setVariableValues(hand_group.joint_cmd_msg);
		}
		else {
			// otherwise set to home position
			std::map<std::string, double> joint_map;
			arm_group.joint_model_group->getVariableDefaultPositions("home", joint_map);
			//kinematic_state->setVariablePositions(joint_map);
			//kinematic_state->copyJointGroupPositions(ARM_GROUP_NAME, arm_group.joint_cmd_msg.position);
			arm_group.kinematic_state->setVariablePositions(joint_map);
			arm_group.kinematic_state->copyJointGroupPositions(ARM_GROUP_NAME, arm_group.joint_cmd_msg.position);

			hand_group.joint_model_group->getVariableDefaultPositions("hand_home", joint_map);
			hand_group.kinematic_state->setVariablePositions(joint_map);
			hand_group.kinematic_state->copyJointGroupPositions(HAND_GROUP_NAME, hand_group.joint_cmd_msg.position);
		}

		if(use_gazebo) {
			// resize gazebo command messages
			gazebo_cmd_msg.resize(num_joints);

			// gazebo cmd pub
			for(int i = 0; i < num_joints; i++) {
				std::string pub_name = model_name + "/" + joint_names[i] + CONTROLLER_SUFFIX;
				//ROS_INFO("%s", pub_name.c_str());
				gazebo_cmd_pub.push_back(nh.advertise<std_msgs::Float64>(pub_name, 3));
				gazebo_cmd_msg[i].data = 0.0;
			}
		}

		// get link names
  		const std::vector<std::string> &link_names = arm_group.joint_model_group->getLinkModelNames();
		int num_links = link_names.size();
		link_wrenches.resize(num_links);
		ROS_INFO("%s model group has %d links", ARM_GROUP_NAME, num_links);
		
		// create zero wrench
		geometry_msgs::Wrench wrench;
		wrench.force.x = wrench.force.y = wrench.force.z = 0;
		wrench.torque.x = wrench.torque.y = wrench.torque.z = 0;

		// initialize link wrenches
		for(int i = 0; i < num_links; i++) {
			link_wrenches[i] = wrench;
		}

		// initialize task space fb message
		geometry_msgs::Transform transform;
		transform.translation.x = transform.translation.y = transform.translation.z = 0.0;
		transform.rotation.x = transform.rotation.y = transform.rotation.z = 0.0;
		transform.rotation.w = 1.0;
	}
	else {
		ROS_ERROR("No model found, exiting");
		return 0;
	}

	// hocu setup
	if(use_hocu) {
		hocu_msg.axes.resize(HOCU_NUM_AXES);
		hocu_msg.buttons.resize(HOCU_NUM_BUTTONS);
		hocu_sub = nh.subscribe(HOCU_TOPIC, 3,  &ArmControl::HocuCallback, this);

		// reset hocu msg
		std::fill(hocu_msg.axes.begin(), hocu_msg.axes.end(), 0.0);
		std::fill(hocu_msg.buttons.begin(), hocu_msg.buttons.end(), 0);
		hocu_prev = hocu_msg;
	}

	// joystick setup
	joy_msg.axes.resize(JOY_NUM_AXES);
	joy_msg.buttons.resize(JOY_NUM_BUTTONS);
	joy_sub = nh.subscribe(JOY_TOPIC, 3,  &ArmControl::JoyCallback, this);

	// reset joy msg
	std::fill(joy_msg.axes.begin(), joy_msg.axes.end(), 0.0);
	std::fill(joy_msg.buttons.begin(), joy_msg.buttons.end(), 0);

	// initialize state and mode
	state = STATE_OFF;

	// create timer
	sample_loop_timer = nh.createTimer(ros::Duration(SAMPLE_PERIOD), &ArmControl::TimerCb, this);
	
	return 1;
}

/*----------------------------------------------------------------------------
  timer callback
 *----------------------------------------------------------------------------*/
void ArmControl::TimerCb(const ros::TimerEvent &event) {
	//ROS_INFO("ArmControl::TimerCb");

	// step groups
	arm_group.Step();
	hand_group.Step();
	
	// state switch
	switch(GetState()) {
		case(STATE_ON): {
			// do gravity compensation
			gravity_solver->getTorques(arm_group.joint_cmd_msg.position, arm_group.joint_cmd_msg.velocity, joint_accelerations, link_wrenches, gravity_effort);
			// reverse effort for command? (no)
			arm_group.joint_cmd_msg.effort = gravity_effort;
			sleep(0.25);
			// publish commands
			arm_group.Publish();
			hand_group.Publish();
			
			// gazebo publishing
			if(use_gazebo) {
				// publish gazebo command
				for(int i = 0; i < num_joints; i++) {
					// set control command message
					gazebo_cmd_msg[i].data = arm_group.joint_cmd_msg.position[i];
		
					// send control message
					gazebo_cmd_pub[i].publish(gazebo_cmd_msg[i]);
				}
			}
			break;
		}
		default:
			if (use_gui){
				arm_group.kinematic_state->setVariableValues(joint_states_msg);
				arm_group.kinematic_state->copyJointGroupPositions(ARM_GROUP_NAME, arm_group.joint_cmd_msg.position);
				hand_group.kinematic_state->setVariableValues(joint_states_msg);
				hand_group.kinematic_state->copyJointGroupPositions(HAND_GROUP_NAME, hand_group.joint_cmd_msg.position);
				// do gravity compensation
				gravity_solver->getTorques(arm_group.joint_cmd_msg.position, arm_group.joint_cmd_msg.velocity, joint_accelerations, link_wrenches, gravity_effort);
				// reverse effort for command? (no)
				arm_group.joint_cmd_msg.effort = gravity_effort;

				// publish commands
				arm_group.Publish();
				hand_group.Publish();
			}
			break;
	}
}

/*----------------------------------------------------------------------------
  joy control arm
 *----------------------------------------------------------------------------*/
void ArmControl::JoyControlArmCb(void) {
	//ROS_INFO("ArmControl::JoyControlArmCb");
	/*ROS_INFO("arm_group.joint_cmd_msg.position[0] = %f ",arm_group.joint_cmd_msg.position[0]);
	ROS_INFO("arm_group.joint_cmd_msg.position[1] = %f ",arm_group.joint_cmd_msg.position[1]);
	ROS_INFO("arm_group.joint_cmd_msg.position[2] = %f ",arm_group.joint_cmd_msg.position[2]);
	if (arm_group.joint_cmd_msg.position[3]>=2.09)
		arm_group.joint_cmd_msg.position[3]=2.09;
	if (arm_group.joint_cmd_msg.position[3]<=-2.09)
		arm_group.joint_cmd_msg.position[3]=-2.09;
	if (arm_group.joint_cmd_msg.position[4]>=1.57)
		arm_group.joint_cmd_msg.position[4]=1.57;
	if (arm_group.joint_cmd_msg.position[4]<=-1.57)
		arm_group.joint_cmd_msg.position[4]=-1.57;
	if (arm_group.joint_cmd_msg.position[5]>=1.57)
		arm_group.joint_cmd_msg.position[5]=1.57;
	if (arm_group.joint_cmd_msg.position[5]<=-1.57)
		arm_group.joint_cmd_msg.position[5]=-1.57;*/

	// get current endpoint state
	Eigen::Affine3d endpoint_state;
	if (!use_leap)	
		endpoint_state = arm_group.kinematic_state->getGlobalLinkTransform(IK_LINK_NAME);
	else
		tf::poseMsgToEigen(endpoint_pose,endpoint_state);	
	
	// get translation and rotation vectors from joy data
	Eigen::Vector3d t;
	Eigen::Matrix3d r, e, rt;

	if(use_hocu) {
		t << (hocu_msg.axes[0] - hocu_prev.axes[0])*SAMPLE_PERIOD*MAX_TRANS_VEL, -(hocu_msg.axes[2] - hocu_prev.axes[2])*SAMPLE_PERIOD*MAX_TRANS_VEL, (hocu_msg.axes[1] - hocu_prev.axes[1])*SAMPLE_PERIOD*MAX_TRANS_VEL;
		//r = Eigen::Matrix3d::Identity();
		r = Eigen::AngleAxisd((hocu_msg.axes[4] - hocu_prev.axes[4])*MAX_ROTATION, Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(-(hocu_msg.axes[5] - hocu_prev.axes[5])*MAX_ROTATION, Eigen::Vector3d::UnitY())*Eigen::AngleAxisd((hocu_msg.axes[3] - hocu_prev.axes[3])*MAX_ROTATION, Eigen::Vector3d::UnitX());
		
		// update translation and rotation
		endpoint_state.pretranslate(t);
		endpoint_state.linear() = hold_state.linear()*r;
	}
	else if(!use_leap){
		t << joy_msg.axes[0]*SAMPLE_PERIOD*MAX_TRANS_VEL, -joy_msg.axes[1]*SAMPLE_PERIOD*MAX_TRANS_VEL, (joy_msg.axes[2] - joy_msg.axes[5])*SAMPLE_PERIOD*MAX_TRANS_VEL; // Translation Vector (X,Y,Z)
		r = Eigen::AngleAxisd(joy_msg.axes[4]*SAMPLE_PERIOD*MAX_ROT_VEL, Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(-(joy_msg.buttons[5] - joy_msg.buttons[4])*SAMPLE_PERIOD*MAX_ROT_VEL, Eigen::Vector3d::UnitY())*Eigen::AngleAxisd((joy_msg.axes[3])*SAMPLE_PERIOD*MAX_ROT_VEL, Eigen::Vector3d::UnitX()); // Rotation Around Each Axis (Z,Y,X)
	
		// rotation of base frame relative to global coordinates
		rt = Eigen::Matrix3d::Identity();

		// translate with respect to ee if button down
		e = endpoint_state.rotation();
		if(joy_msg.buttons[10]) {
			endpoint_state.pretranslate(e*rt*t);
		}
		// rotate with respect to ee if button down
		else if(joy_msg.buttons[9]) {
			endpoint_state.rotate(rt*r*rt.transpose());
		}
		// otherwise translate and rotate with respect to world
		else {
			endpoint_state.pretranslate(t);
			//endpoint_state.rotate(e.transpose()*r*e);
			endpoint_state.rotate(rt*r*rt.transpose());
			/*arm_group.joint_cmd_msg.position[3] += joy_msg.buttons[4]*0.3;
			arm_group.joint_cmd_msg.position[3] -= joy_msg.buttons[5]*0.3;
			arm_group.joint_cmd_msg.position[4] -= joy_msg.axes[4]*0.3;
			arm_group.joint_cmd_msg.position[5] += joy_msg.axes[3]*0.3;
			arm_group.kinematic_state->setVariableValues(arm_group.joint_cmd_msg);*/
		}
	}

	// calculate IK, seed state near home?
	bool found_ik = arm_group.kinematic_state->setFromIK(arm_group.joint_model_group, endpoint_state, IK_LINK_NAME, 1, SAMPLE_PERIOD, boost::bind(&ArmControl::CollisionCb, this, collision_scene, _1, _2, _3));
	//bool found_ik = false;	
	if(found_ik) {
		// update joint cmd
		std::vector<double> joint_positions;
		arm_group.kinematic_state->copyJointGroupPositions(arm_group.joint_model_group, joint_positions);

		// check for feasible joint velocity	
		int num_joints = joint_positions.size();
		double dv[num_joints];
		double max_dv = 0.0;
		for(int i = 0; i < num_joints; i++) {
			dv[i] = (joint_positions[i] - arm_group.joint_cmd_msg.position[i])/SAMPLE_PERIOD;
			max_dv = std::max(max_dv, fabs(dv[i]));
		}

		// check max joint velocity
		if(max_dv > MAX_JOINT_VEL) {
			double vs;
			if (!use_leap)
				vs = MAX_JOINT_VEL/max_dv;
			else
				vs = (1*MAX_JOINT_VEL)/max_dv;
			ROS_INFO("Max joint velocity exceedes limit, scaling by %f", vs);
		
			// scale joint positions
			for(int i = 0; i < num_joints; i++) {
				arm_group.joint_cmd_msg.position[i] += dv[i]*vs*SAMPLE_PERIOD;
			}
		}
		else {
			// set joint positions
			arm_group.joint_cmd_msg.position = joint_positions;
		}
	}
	else {
		ROS_INFO("Did not find IK solution");
		// revert endpoint state to previous cmd
		//kinematic_state->setVariableValues(joint_space_fb_msg);
		//endpoint_state = kinematic_state->getGlobalLinkTransform(ik_link_name);
	}

	// update kinematic state
	arm_group.kinematic_state->setVariableValues(arm_group.joint_cmd_msg);
}

/*----------------------------------------------------------------------------
  joy control hand
 *----------------------------------------------------------------------------*/
void ArmControl::JoyControlHandCb(void) {
	//ROS_INFO("ArmControl::JoyControlHandCb");
  
	//ROS_INFO("Finger1Open = %f" , hand_group.joint_cmd_msg.position[0]);
	//ROS_INFO("Finger2Open = %f" , hand_group.joint_cmd_msg.position[1]);
	//ROS_INFO("ThumbRotate = %f" , hand_group.joint_cmd_msg.position[2]);
	//ROS_INFO("ThumbOpen = %f" , hand_group.joint_cmd_msg.position[3]);
	if (!use_leap){
		if (joy_msg.axes[6]!=0)
			joy_msg.axes[7]=0;
		if (joy_msg.axes[7]!=0)
			joy_msg.axes[6]=0;
		if (hand_group.joint_cmd_msg.position[0]<=0)
		{
			hand_group.joint_cmd_msg.position[0]=0;
			hand_group.joint_cmd_msg.position[1]=0;
			hand_group.joint_cmd_msg.position[3]=0;
		}

		if (hand_group.joint_cmd_msg.position[0]>=1.333)
		{
			hand_group.joint_cmd_msg.position[0]=1.333;
			hand_group.joint_cmd_msg.position[1]=1.333;
			hand_group.joint_cmd_msg.position[3]=1.333;
		}
		if (hand_group.joint_cmd_msg.position[0]>=1.098)
			joy_msg.axes[6]=0;
		if (hand_group.joint_cmd_msg.position[2]>=0)
			hand_group.joint_cmd_msg.position[2]=0;
	
	

		if(use_hocu) {
			// update hand command
			hand_group.joint_cmd_msg.position[0] = hocu_msg.axes[8]*2.3562;
			hand_group.joint_cmd_msg.position[1] = hocu_msg.axes[9]*2.3562;
			hand_group.joint_cmd_msg.position[2] = hocu_msg.axes[7]*1.5708;
			hand_group.joint_cmd_msg.position[3] = hocu_msg.axes[6]*2.0944;
			hand_group.kinematic_state->setVariableValues(hand_group.joint_cmd_msg);
		}
		else {
			// update hand command
			hand_group.joint_cmd_msg.position[0] += joy_msg.axes[7]*MAX_ROT_VEL*0.1;
			hand_group.joint_cmd_msg.position[1] += joy_msg.axes[7]*MAX_ROT_VEL*0.1;
			hand_group.joint_cmd_msg.position[2] -= joy_msg.axes[6]*MAX_ROT_VEL*0.1;
			hand_group.joint_cmd_msg.position[3] += joy_msg.axes[7]*MAX_ROT_VEL*0.1;
			hand_group.kinematic_state->setVariableValues(hand_group.joint_cmd_msg);
		}
	}
	else {
	


		hand_group.joint_cmd_msg.position[0] = K_THETA_INDEX * theta_index;
		hand_group.joint_cmd_msg.position[1] = K_THETA_MIDDLE * theta_middle;
		hand_group.joint_cmd_msg.position[2] = K_PHI_THUMB * phi_thumb + L_PHI_THUMB;
		hand_group.joint_cmd_msg.position[3] = K_THETA_THUMB * theta_thumb + L_THETA_THUMB;
		
		if (joy_msg.axes[6]!=0)
			joy_msg.axes[7]=0;
		if (joy_msg.axes[7]!=0)
			joy_msg.axes[6]=0;

		if (hand_group.joint_cmd_msg.position[0]<=0)
			hand_group.joint_cmd_msg.position[0]=0;
		if (hand_group.joint_cmd_msg.position[1]<=0)
			hand_group.joint_cmd_msg.position[1]=0;
		if (hand_group.joint_cmd_msg.position[3]<=0)
			hand_group.joint_cmd_msg.position[3]=0;

		if (hand_group.joint_cmd_msg.position[0]>=1.333)
			hand_group.joint_cmd_msg.position[0]=1.333;
		if (hand_group.joint_cmd_msg.position[1]>=1.333)
			hand_group.joint_cmd_msg.position[1]=1.333;
		if (hand_group.joint_cmd_msg.position[3]>=1.333)
			hand_group.joint_cmd_msg.position[3]=1.333;

		if (hand_group.joint_cmd_msg.position[0]>=1.098)
			joy_msg.axes[6]=0;
		if (hand_group.joint_cmd_msg.position[2]>=0)
			hand_group.joint_cmd_msg.position[2]=0;

		hand_group.kinematic_state->setVariableValues(hand_group.joint_cmd_msg);
	}
	// enforce bounds
	hand_group.kinematic_state->enforceBounds(hand_group.joint_model_group);
	hand_group.kinematic_state->copyJointGroupPositions(hand_group.joint_model_group, hand_group.joint_cmd_msg.position);
}

/*----------------------------------------------------------------------------
  collision callback
 *----------------------------------------------------------------------------*/
bool ArmControl::CollisionCb(const planning_scene::PlanningScene *planning_scene, robot_state::RobotState *state, const robot_state::JointModelGroup *group, const double *joint_group_variable_values) {
	state->setJointGroupPositions(group, joint_group_variable_values);

	// setup collision request
	collision_detection::CollisionRequest req;
	req.verbose = false;
	req.group_name = group->getName();

	// check for collision
	collision_detection::CollisionResult res;
	planning_scene->checkCollision(req, res, *state, *allowed_collision_matrix);
	if (res.collision) {
		std::vector<std::string> colliding_links;
		planning_scene->getCollidingLinks(colliding_links, *state);
		for(int i = 0; i < colliding_links.size(); i++) ROS_ERROR("%s", colliding_links[i].c_str());
		return false;
	}

	return planning_scene->isStateFeasible(*state, true);
	//return true;
}

/*----------------------------------------------------------------------------
  change state
 *----------------------------------------------------------------------------*/
bool ArmControl::ChangeState(StateType new_state) {
	int ret = true;
	
	// state dependent switching
	switch(new_state) {
		case(STATE_OFF):
		case(STATE_FAULT):
			arm_group.ChangeMode(ControlGroup::MODE_NONE);
			hand_group.ChangeMode(ControlGroup::MODE_NONE);
			break;
		default:
			break;
	}

	// change state
	state = new_state;

	// send feedback
	ROS_INFO("ArmControl::ChangeState %d", (int)state);

	return true;
}

/*----------------------------------------------------------------------------
  hocu callback
 *----------------------------------------------------------------------------*/
void ArmControl::HocuCallback(const sensor_msgs::Joy& msg) {
	//ROS_INFO("ArmControl::HocuCallback()");
	// process deadman button down
	if((msg.buttons[0] == 1) && (hocu_msg.buttons[0] == 0)) {
		// change mode
		if(arm_group.ChangeMode(ControlGroup::MODE_JOYSTICK)) {
			// store hocu data
			hocu_prev = hocu_msg;
			
			// get current endpoint state
			hold_state = arm_group.kinematic_state->getGlobalLinkTransform(IK_LINK_NAME);
		}
	}
	// process deadman button up
	else if((msg.buttons[0] == 0) && (hocu_msg.buttons[0] == 1)) {
		// check mode
		switch(arm_group.GetMode()) {
			case ControlGroup::MODE_JOYSTICK:
				// change mode
				if(arm_group.ChangeMode(ControlGroup::MODE_NONE)) {
				}
				break;
			default:
				break;
		}
	}

	// process A button up
	if((msg.buttons[1] == 0) && (hocu_msg.buttons[1] == 1)) {
		// change state
		switch(GetState()) {
			case(STATE_OFF):
				ChangeState(STATE_ON);
				break;
			default:
				ChangeState(STATE_OFF);
				break;
		}
	}

	// process B button up
	if((msg.buttons[2] == 0) && (hocu_msg.buttons[2] == 1)) {
		// create action
		hdt_arm_control::JointSpacePathGoal goal;
		goal.pose = "home";
		arm_group.joint_space_path_ac->sendGoal(goal);

		// change mode
		if(arm_group.ChangeMode(ControlGroup::MODE_JOINT_SPACE)) {
		}
	}

	// process C button up
	if((msg.buttons[3] == 0) && (hocu_msg.buttons[3] == 1)) {
		// check mode
		switch(hand_group.GetMode()) {
			case ControlGroup::MODE_JOYSTICK:
				// change mode
				if(hand_group.ChangeMode(ControlGroup::MODE_NONE)) {
				}
				break;
			case ControlGroup::MODE_NONE:
				if(hand_group.ChangeMode(ControlGroup::MODE_JOYSTICK)) {
				}
				break;
			default:
				break;
		}
	}

	// process D button up
	if((msg.buttons[4] == 0) && (hocu_msg.buttons[4] == 1)) {
		// create action
		hdt_arm_control::JointSpacePathGoal goal;
		goal.pose = "hand_home";
		hand_group.joint_space_path_ac->sendGoal(goal);

		// change mode
		if(hand_group.ChangeMode(ControlGroup::MODE_JOINT_SPACE)) {
		}
	}

	// update hocu msg
	hocu_msg = msg;
}

/*----------------------------------------------------------------------------
  joy callback
 *----------------------------------------------------------------------------*/
void ArmControl::JoyCallback(const sensor_msgs::Joy& msg) {
	// process A button up
	if((msg.buttons[0] == 0) && (joy_msg.buttons[0] == 1)) {
		ROS_INFO("ArmControl::JoyCallback::A");

		// create action
		hdt_arm_control::JointSpacePathGoal goal;
		goal.pose = "home";
		arm_group.joint_space_path_ac->sendGoal(goal);

		// change mode
		if(arm_group.ChangeMode(ControlGroup::MODE_JOINT_SPACE)) {
		}
	}
	// process B button up
	if((msg.buttons[1] == 0) && (joy_msg.buttons[1] == 1)) {
		ROS_INFO("ArmControl::JoyCallback::B");
		
		hand_group.joint_cmd_msg.position[0]=0;
		hand_group.joint_cmd_msg.position[1]=0;
		hand_group.joint_cmd_msg.position[2]=0;
		hand_group.joint_cmd_msg.position[3]=0;
		sleep(1);
		// create action
		hdt_arm_control::JointSpacePathGoal goal;
		goal.pose = "stowed";
		arm_group.joint_space_path_ac->sendGoal(goal);

		// change mode
		if(arm_group.ChangeMode(ControlGroup::MODE_JOINT_SPACE)) {
		}

	}
	// process X button up
	if((msg.buttons[2] == 0) && (joy_msg.buttons[2] == 1)) {
		ROS_INFO("ArmControl::JoyCallback::X");

		// change mode
		if(arm_group.ChangeMode(ControlGroup::MODE_JOYSTICK)) {
		}

		// change mode
		if(hand_group.ChangeMode(ControlGroup::MODE_JOYSTICK)) {
		}
	}
	// process Y button up
	if((msg.buttons[3] == 0) && (joy_msg.buttons[3] == 1)) {
		ROS_INFO("ArmControl::JoyCallback::Y");

		// change mode
		if(arm_group.ChangeMode(ControlGroup::MODE_NONE)) {
		}

		// change mode
		if(hand_group.ChangeMode(ControlGroup::MODE_NONE)) {
		}
		
		arm_group.joint_cmd_msg.position[0] = pp_goal[0];
		sleep(0.4);
		arm_group.joint_cmd_msg.position[1] = pp_goal[1];
		sleep(0.4);
		arm_group.joint_cmd_msg.position[2] = pp_goal[2];
		sleep(0.4);
		arm_group.joint_cmd_msg.position[3] = pp_goal[3];
		sleep(0.4);
		arm_group.joint_cmd_msg.position[4] = pp_goal[4];
		sleep(0.4);
		arm_group.joint_cmd_msg.position[5] = pp_goal[5];
		sleep(1);
		arm_group.kinematic_state->setVariableValues(arm_group.joint_cmd_msg);
		sleep(1);
		// store pose
		//Transform2Pose(task_space_fb_msg.transforms[0], test_pose);
		//test_waypoints.push_back(test_pose);
		//test_pose.position.x = task_space_fb_msg.transforms[0].translation.x;
		//test_pose.position.y = task_space_fb_msg.transforms[0].translation.y;
		//test_pose.position.z = task_space_fb_msg.transforms[0].translation.z;
		//test_pose.orientation = task_space_fb_msg.transforms[0].rotation;
	}
	// process Start button up
	if((msg.buttons[7] == 0) && (joy_msg.buttons[7] == 1)) {
		ROS_INFO("ArmControl::JoyCallback::Start");

		// change state
		switch(GetState()) {
			case(STATE_OFF):
				ChangeState(STATE_ON);
				break;
			default:
				ChangeState(STATE_OFF);
				break;
		}
	}
	// process Back button up
	if((msg.buttons[6] == 0) && (joy_msg.buttons[6] == 1)) {
		ROS_INFO("ArmControl::JoyCallback::Back");

		// test call action
		//if(ChangeMode(MODE_FREE_SPACE)) {
		//if(ChangeMode(MODE_APPROACH)) {
		//	hdt_arm_control::TaskSpacePathGoal goal;
		//	goal.marker = IK_LINK_NAME;
		//	goal.pose = test_pose;
		//	task_space_path_ac->sendGoal(goal);
			//hdt_arm_control::TrajectoryPathGoal goal;
			//goal.marker = fiducial_links.begin()->first;
			//goal.waypoints = test_waypoints;
			//trajectory_path_ac->sendGoal(goal);
		//}
	}
	// check mode
	switch(hand_group.GetMode()) {
		// don't execute motions in joystick mode
		case ControlGroup::MODE_JOYSTICK:
			break;
		default:
			// process D-pad button up
			if((msg.axes[6] < 0.5) && (joy_msg.axes[7] >= 0.5)) {
				ROS_INFO("ArmControl::JoyCallback::Up");

				// create action
				hdt_arm_control::JointSpacePathGoal goal;
				goal.pose = "hand_home";
				hand_group.joint_space_path_ac->sendGoal(goal);

				// change mode
				if(hand_group.ChangeMode(ControlGroup::MODE_JOINT_SPACE)) {
				}
			}
			// process D-pad button down
			if((msg.axes[6] > -0.5) && (joy_msg.axes[7] <= -0.5)) {
				ROS_INFO("ArmControl::JoyCallback::Down");

				// create action
				hdt_arm_control::JointSpacePathGoal goal;
				goal.pose = "hand_open";
				hand_group.joint_space_path_ac->sendGoal(goal);

				// change mode
				if(hand_group.ChangeMode(ControlGroup::MODE_JOINT_SPACE)) {
				}
			}
			// process D-pad button left
			if((msg.axes[6] < 0.5) && (joy_msg.axes[6] >= 0.5)) {
				ROS_INFO("ArmControl::JoyCallback::Left");

				// create action
				hdt_arm_control::JointSpacePathGoal goal;
				goal.pose = "hand_home";
				hand_group.joint_space_path_ac->sendGoal(goal);

				// change mode
				if(hand_group.ChangeMode(ControlGroup::MODE_JOINT_SPACE)) {
				}
			}
			// process D-pad button right
			if((msg.axes[6] > -0.5) && (joy_msg.axes[6] <= -0.5)) {
				ROS_INFO("ArmControl::JoyCallback::Right");
			}
		break;
	}

	// update joy_msg
	joy_msg = msg;
}

/*----------------------------------------------------------------------------
  joint telem callback
 *----------------------------------------------------------------------------*/
void ArmControl::JointTelemCb(const sensor_msgs::JointState& msg) {
	//ROS_INFO("ArmControl::JointTelemCb");

	// update joint telem
	joint_telem_msg = msg;
}

/*----------------------------------------------------------------------------
  joint gui callback
 *----------------------------------------------------------------------------*/
void ArmControl::JointGuiCb(const sensor_msgs::JointState& msg) {
	//ROS_INFO("ArmControl::JointGuiCb");
	int i;
	for (i=0;i<10;i++)
		ROS_DEBUG("msg.positio[%d] = %f",i,msg.position[i]);
	// update joint states
	joint_states_msg = msg;
}

/*----------------------------------------------------------------------------
  Path Planning callback
 *----------------------------------------------------------------------------*/
void ArmControl::PPCb(const moveit_msgs::MoveGroupActionGoal& msg) {
	ROS_INFO("ArmControl::PPCb");
	if (!msg.goal.planning_options.plan_only){
		int i;
		for (i=0;i<6;i++){
			ROS_DEBUG("msg.positio[%d] = %f",i,msg.goal.request.goal_constraints[0].joint_constraints[i].position);
			pp_goal[i]=msg.goal.request.goal_constraints[0].joint_constraints[i].position;
		}
		// update joint states
		//joint_states_msg = msg;
	}
}

/*----------------------------------------------------------------------------
  Leap Motion callback
 *----------------------------------------------------------------------------*/
void ArmControl::LeapCb(const leap_motion::leapros& msg) {
	//ROS_INFO("ArmControl::LeapCb");
	double x,y,z,yaw,pitch,roll;
	//geometry_msgs::Quaternion q;
	
	x = START_X + GRADIENT_X * msg.palmpos.x;
	y = START_Y + GRADIENT_Y * msg.palmpos.z;
	z = START_Z + GRADIENT_Z * msg.palmpos.y;

	yaw = atan2(msg.normal.y,msg.normal.x) + PI;
	pitch = -atan2(msg.direction.y,-msg.direction.z);
	roll = atan2(msg.direction.z,msg.direction.x) + PI/2;
	yaw *= K_YAW;
	pitch *= K_PITCH;
	roll *= K_ROLL;
	//ROS_INFO("pitch = %f",pitch*(180/PI));
	double lim=0.5;
	endpoint_pose.position.x = x;
	endpoint_pose.position.y = y;
	endpoint_pose.position.z = z;
	if(x>lim)endpoint_pose.position.x=lim;
	if(x<-lim)endpoint_pose.position.x=-lim;
	if(y>lim)endpoint_pose.position.y = lim;
	if(y<-lim)endpoint_pose.position.y = -lim;
	if(z<-lim)endpoint_pose.position.z = -lim;
	if(z>lim)endpoint_pose.position.z = lim;
	
	//roll = 0; pitch = 0; yaw = PI/2;
	//q = RPYtoQuat(roll,pitch,yaw);
	
	/*endpoint_pose.orientation.x = q.x;
	endpoint_pose.orientation.y = q.y;
	endpoint_pose.orientation.z = q.z;
	endpoint_pose.orientation.w = q.w;*/

	endpoint_pose.orientation.w = cos(yaw/2)*cos(pitch/2)*cos(roll/2) - sin(yaw/2)*sin(pitch/2)*sin(roll/2);
	endpoint_pose.orientation.x = sin(yaw/2)*sin(pitch/2)*cos(roll/2) + cos(yaw/2)*cos(pitch/2)*sin(roll/2);
	endpoint_pose.orientation.y = sin(yaw/2)*cos(pitch/2)*cos(roll/2) + cos(yaw/2)*sin(pitch/2)*sin(roll/2);
	endpoint_pose.orientation.z = cos(yaw/2)*sin(pitch/2)*cos(roll/2) - sin(yaw/2)*cos(pitch/2)*sin(roll/2);

	theta_index = FingerAngle (msg.index_metacarpal,msg.index_tip,msg.direction);
	theta_middle = FingerAngle (msg.middle_metacarpal,msg.middle_tip,msg.direction);
	theta_thumb = FingerAngle (msg.thumb_metacarpal,msg.thumb_tip,msg.direction);
	phi_thumb = FingerAngle (msg.thumb_metacarpal,msg.thumb_tip,msg.normal);
	//ROS_INFO("[ %f ]",phi_thumb);
	
}

double ArmControl::FingerAngle (geometry_msgs::Point a,geometry_msgs::Point b,geometry_msgs::Vector3 u){

	double v1,v2,v3,V,theta;

	v1 = b.x - a.x;
	v2 = b.y - a.y;
	v3 = b.z - a.z;
	V = sqrt((v1*v1)+(v2*v2)+(v3*v3));
	v1 /= V;
	v2 /= V;
	v3 /= V;
	theta = acos((v1*u.x) + (v2*u.y) + (v3*u.z));
	return theta;
}


geometry_msgs::Quaternion ArmControl::RPYtoQuat (double roll,double pitch,double yaw){

	geometry_msgs::Quaternion q;
	q.w = cos(yaw/2)*cos(pitch/2)*cos(roll/2) - sin(yaw/2)*sin(pitch/2)*sin(roll/2);
	q.x = sin(yaw/2)*sin(pitch/2)*cos(roll/2) + cos(yaw/2)*cos(pitch/2)*sin(roll/2);
	q.y = sin(yaw/2)*cos(pitch/2)*cos(roll/2) + cos(yaw/2)*sin(pitch/2)*sin(roll/2);
	q.z = cos(yaw/2)*sin(pitch/2)*cos(roll/2) - sin(yaw/2)*cos(pitch/2)*sin(roll/2);	
	return q;
}



/*----------------------------------------------------------------------------
  main function
 *----------------------------------------------------------------------------*/
int main(int argc, char **argv) {
	ros::init(argc, argv, ROS_NAME);
	int ret;

	ROS_INFO("%s starting", ROS_NAME);

	// start spinner
	ros::AsyncSpinner spinner(2);
	spinner.start();

	// arm control class
	ArmControl arm_ctl;
	if(arm_ctl.Init() <= 0) {
		return 0;
	}
	
	// wait for ctl-c
	ros::waitForShutdown();

	return 0;
}
