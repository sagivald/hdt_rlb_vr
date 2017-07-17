#include <tinyxml.h>

#include "hdt_adroit_driver.h"
#include "AdroitApp.h"

/*----------------------------------------------------------------------------
  main function
 *----------------------------------------------------------------------------*/
int AdroitDriver::Init(AdroitInterface *coms_interface) {
	ros::NodeHandle node_handle;

	// get coms interface
	COMS = coms_interface;

	// get params
	ros::NodeHandle private_node_handle("~");
	bool console_enabled;
	private_node_handle.param<bool>("impedance_enabled", impedance_enabled, false);
	private_node_handle.param<bool>("control_enabled", control_enabled, true);	
	private_node_handle.param<double>("damping_ratio", damping_ratio, 1.0);
	private_node_handle.param<double>("stiffness_ratio", stiffness_ratio, 0.5);
	private_node_handle.param<bool>("console_enabled", console_enabled, false);

	// initialize drives
	for(int i = 0; i < MAX_DRIVES; i++) {
		adroit_drives[i] = new AdroitComs::AdroitDrive;
		
		// reset drive
		COMS->reset_drive(adroit_drives[i]);
	}

	// check for robot description
	std::string description;

	// wait for robot_description
	if(!(node_handle.hasParam(ROBOT_DESCRIPTION))) {
		ROS_INFO("No urdf file, checking every %f second(s)", PARAM_WAIT_DURATION);

		// loop until robot_description
		while(!(node_handle.hasParam(ROBOT_DESCRIPTION))) {
			// pause here
			ros::Duration(PARAM_WAIT_DURATION).sleep();
		}
	}
	
	// parse urdf
	if(node_handle.getParam(ROBOT_DESCRIPTION, description)) {

		ROS_INFO("hdt_adroit_coms::main: Found urdf");
		
		// parse urdf
		TiXmlDocument doc;
		if(!(doc.Parse(description.c_str(), 0, TIXML_ENCODING_UTF8))) {
	   		ROS_ERROR("Failed to parse urdf file");
			return -1;
		}
		ROS_INFO("Successfully parsed urdf file");

		// get joints
		int num_drives = 0;
		TiXmlElement *joint;
		joint = doc.FirstChildElement("robot")->FirstChildElement("joint");
		for(joint; joint; joint=joint->NextSiblingElement("joint")) {
			TiXmlElement *node;
			if(node = joint->FirstChildElement("hdt")) {
				// make sure this drive has an id
				if(node->Attribute("id") != NULL) {
					// store name and id
					int addr = -1;
					float kmin = 0, kmax = 0, inertia = 0;
					try {
						addr = atoi(node->Attribute("id"));
						kmin = atof(node->Attribute("kmin"));
						kmax = atof(node->Attribute("kmax"));
						inertia = atof(node->Attribute("inertia"));
					}
					catch(...) {
						ROS_ERROR("Error retreiving joint parameters");
						return -1;
					}
					ROS_INFO("Found id = %d", addr);
					if((addr > 0) && (addr < MAX_DRIVES)) {
						active_drives[joint->Attribute("name")] = addr;

						// store drive information
						adroit_drives[addr]->impedance_params.kmin = kmin;
						adroit_drives[addr]->impedance_params.kmax = kmax;
						adroit_drives[addr]->impedance_params.inertia = inertia;
					}
				}
			}
		}
	
		ROS_INFO("active_drives.size() = %d", (int)active_drives.size());
		
		for(it_type iterator = active_drives.begin(); iterator != active_drives.end(); iterator++) {
		  hdt_debug_telem_topics[iterator->second]="/" + (std::string)ROS_NAME + "/" + iterator->first + "/" +  "hdt_debug_telem";
		  hdt_ls_telem_topics[iterator->second]="/" + (std::string)ROS_NAME + "/" + iterator->first + "/" +  "hdt_ls_telem";
		  hdt_ms_telem_topics[iterator->second]="/" + (std::string)ROS_NAME + "/" + iterator->first + "/" +  "hdt_ms_telem";
		  hdt_hs_telem_topics[iterator->second]="/" + (std::string)ROS_NAME + "/" + iterator->first + "/" +  "hdt_hs_telem";
		  hdt_error_telem_topics[iterator->second]="/" + (std::string)ROS_NAME + "/" + iterator->first + "/" +  "hdt_error_telem";
		  hdt_param_telem_topics[iterator->second]="/" + (std::string)ROS_NAME + "/" + iterator->first + "/" +  "hdt_param_telem";
		  hdt_status_telem_topics[iterator->second]="/" + (std::string)ROS_NAME + "/" + iterator->first + "/" +  "hdt_status_telem";
		  hdt_can_control_cmd_telem_topics[iterator->second]="/" + (std::string)ROS_NAME + "/" + iterator->first + "/" +  "hdt_can_control_cmd_telem";
		  hdt_control_cmd_telem_topics[iterator->second]="/" + (std::string)ROS_NAME + "/" + iterator->first + "/" +  "hdt_control_cmd_telem";

		  std::cout<<hdt_debug_telem_topics[iterator->second]<<std::endl;
		  hdt_debug_telem_pub[iterator->second]=node_handle.advertise<hdt_adroit_driver::HDTDebugTelem>(hdt_debug_telem_topics[iterator->second], DEBUG_MSG_RATE);
		  hdt_debug_telem_msg[iterator->second].hs_var1.resize(16);
		  hdt_debug_telem_msg[iterator->second].hs_var2.resize(16);

		  hdt_ls_telem_pub[iterator->second]=node_handle.advertise<hdt_adroit_driver::HDTLSTelem>(hdt_ls_telem_topics[iterator->second], 3);
		  hdt_ms_telem_pub[iterator->second]=node_handle.advertise<hdt_adroit_driver::HDTMSTelem>(hdt_ms_telem_topics[iterator->second], 3);
		  hdt_hs_telem_pub[iterator->second]=node_handle.advertise<hdt_adroit_driver::HDTHSTelem>(hdt_hs_telem_topics[iterator->second], 3);

		  hdt_error_telem_pub[iterator->second]=node_handle.advertise<hdt_adroit_driver::HDTErrorTelem>(hdt_error_telem_topics[iterator->second], 3);
		  hdt_param_telem_pub[iterator->second]=node_handle.advertise<hdt_adroit_driver::HDTParameterTelem>(hdt_param_telem_topics[iterator->second], 3);
		  hdt_status_telem_pub[iterator->second]=node_handle.advertise<hdt_adroit_driver::HDTStatusTelem>(hdt_status_telem_topics[iterator->second], 3);
		  hdt_can_control_cmd_telem_pub[iterator->second]=node_handle.advertise<hdt_adroit_driver::HDTControlCmdTelem>(hdt_can_control_cmd_telem_topics[iterator->second], 3);
		  hdt_control_cmd_telem_pub[iterator->second]=node_handle.advertise<hdt_adroit_driver::HDTControlCmdTelem>(hdt_control_cmd_telem_topics[iterator->second], 3);
		}


		// resize joint state msg
		joint_telem_msg.name.resize(active_drives.size());
		joint_telem_msg.position.resize(active_drives.size());
		joint_telem_msg.velocity.resize(active_drives.size());
		joint_telem_msg.effort.resize(active_drives.size());
		
		hdt_telem_msg.name.resize(active_drives.size());
		hdt_telem_msg.position.resize(active_drives.size());
		hdt_telem_msg.velocity.resize(active_drives.size());
		hdt_telem_msg.effort.resize(active_drives.size());
		hdt_telem_msg.motor_current.resize(active_drives.size());
		hdt_telem_msg.bus_voltage.resize(active_drives.size());
		hdt_telem_msg.bus_current.resize(active_drives.size());
		hdt_telem_msg.temperature.resize(active_drives.size());

		// initialize drive timers
		drive_timers.resize(active_drives.size());

		// reset drives
		ResetDrives();
  	}
	else {
		ROS_ERROR("Did not find urdf");
		return -1;
	}


	// get adroit params file
	std::string params_file;
	private_node_handle.param<std::string>("params_file", params_file, "adroit_params.xml");
	// get adroit params
	PARAMS = new AdroitParams();
	if(!(PARAMS->load_file(params_file))) {
	  ROS_ERROR("could not load params file: %s", params_file.c_str());
	  return -1;
	}

	// joint telem publisher
	joint_telem_pub = node_handle.advertise<sensor_msgs::JointState>(JOINT_TELEM_TOPIC, 3);
	hdt_telem_pub = node_handle.advertise<hdt_adroit_driver::HDTJointState>(HDT_TELEM_TOPIC, 3);
	//drive_error_pub = node_handle.advertise<hdt_adroit_driver::DriveError>(HDT_TELEM_TOPIC, 3);
	  
	// joint command subscriber
	joint_cmd_sub = node_handle.subscribe(JOINT_CMD_TOPIC, 3, &AdroitDriver::JointCmdCallback, this);

	// HDT joint command subscriber
	hdt_joint_cmd_sub = node_handle.subscribe(HDT_JOINT_CMD_TOPIC, 3, &AdroitDriver::HDTJointCmdCallback, this);

	// coms ready service
	coms_ready = node_handle.advertiseService(COMS_READY_SERVICE, &AdroitDriver::ComsReadyCallback, this);
	
	// write param service
	write_param_srv = node_handle.advertiseService(WRITE_PARAM_SERVICE, &AdroitDriver::WriteParamServiceCallback, this);

	// read param service
	read_param_srv = node_handle.advertiseService(READ_PARAM_SERVICE, &AdroitDriver::ReadParamServiceCallback, this);

	// application load service	
	app_load_srv = node_handle.advertiseService(APP_LOAD_SERVICE, &AdroitDriver::AppLoadServiceCallback, this);

	//state select service
	state_select_srv=node_handle.advertiseService(STATE_SELECT_SERVICE, &AdroitDriver::StateSelectServiceCallback, this);

	// get status service
	get_status_srv=node_handle.advertiseService(GET_STATUS_SERVICE, &AdroitDriver::GetStatusServiceCallback, this);

	// commit service
	commit_srv=node_handle.advertiseService(COMMIT_SERVICE, &AdroitDriver::CommitServiceCallback, this);

	// create sample loop timer
	ros::Timer timer = node_handle.createTimer(ros::Duration(SAMPLE_PERIOD), &AdroitDriver::TimerCallback, this);

	ROS_INFO("%s ready", ROS_NAME);

	// adroit console loop
	if(console_enabled) {
		// initialize console
		adroit_console.Init(COMS, adroit_drives);
	}
	else {

		ros::waitForShutdown();
	}

	return 1;
}

/*----------------------------------------------------------------------------
   reset drives
 *----------------------------------------------------------------------------*/
void AdroitDriver::ResetDrives(void) {
	msg_count=0;

	// reset drives
	AdroitComs::StateCmd state_cmd;
	state_cmd.state = AdroitComs::INIT_STATE;
	COMS->send_state_cmd(BROADCAST_ADDR, &state_cmd);
	COMS->Write();
	joint_cmd_time = ros::Time::now();

	// wait for reset
	ROS_INFO("Resetting drives");
	usleep(INIT_WAIT_TIME);

	// iterate through map, update msg and send to startup
	ProcessMessages();
	int i = 0;
	state_cmd.state = AdroitComs::STARTUP_STATE;
	for(it_type iterator = active_drives.begin(); iterator != active_drives.end(); iterator++) {
		// update telem msg
		joint_telem_msg.name[i] = iterator->first;
		joint_telem_msg.position[i] = 0.0;
		joint_telem_msg.velocity[i] = 0.0;
		joint_telem_msg.effort[i] = 0.0;

		// update hdt telem msg
		hdt_telem_msg.name[i] = iterator->first;
		hdt_telem_msg.position[i] = 0.0;
		hdt_telem_msg.velocity[i] = 0.0;
		hdt_telem_msg.effort[i] = 0.0;
		hdt_telem_msg.motor_current[i] = 0.0;
		hdt_telem_msg.bus_voltage[i] = 0.0;
		hdt_telem_msg.bus_current[i] = 0.0;			
		hdt_telem_msg.temperature[i] = 0.0;	

		// send to startup
		//adroit_drives[i]->msg_update[AdroitComs::STATUS_TELEM] = false;
		COMS->send_state_cmd(iterator->second, &state_cmd);

		// reset hs telem
		adroit_drives[iterator->second]->msg_update[AdroitComs::HIGH_SPEED_TELEM] = false;

		i++;
	}

	COMS->Write();

	// wait for telem
	ROS_INFO("Waiting for drives to start");
	usleep(INIT_WAIT_TIME);

	// update cmd msg
	ProcessMessages();
	for(it_type iterator = active_drives.begin(); iterator != active_drives.end(); iterator++) {
		// check for hs telem
		if(adroit_drives[iterator->second]->msg_update[AdroitComs::HIGH_SPEED_TELEM]) {
			// update cmd position to match telem
			adroit_drives[iterator->second]->control_cmd.position = adroit_drives[iterator->second]->hs_telem.position;
		}
		else {
			ROS_ERROR("Did not receive response from %s", iterator->first.c_str());
			adroit_drives[iterator->second]->control_cmd.position = 0.0;
			//return -1;
		}
		adroit_drives[iterator->second]->control_cmd.velocity = 0.0;
		adroit_drives[iterator->second]->control_cmd.effort = 0.0;
		adroit_drives[iterator->second]->control_cmd.motor_current = 0.0;
	}

	if(impedance_enabled) {
		// send impedance command
		SendImpedanceCommands(damping_ratio, stiffness_ratio);
	}
}

/*----------------------------------------------------------------------------
  process messages
 *----------------------------------------------------------------------------*/
int AdroitDriver::ProcessMessages(void) {
	int num = 0;
	AdroitComs::AdroitMsg msg;

	// process messages
	while(COMS->ReceiveMsg(&msg) == 1) {
		uint8_t id = (uint8_t)((msg.id >> 7) & 0x0F);
		uint8_t addr = (uint8_t)(msg.id & 0x7F);
		AdroitComs::AdroitDrive *drive = adroit_drives[addr];
		num++;

		// take message dependent action
		switch(id) {
			case(AdroitComs::ERROR_TELEM):
				if(COMS->process_error_telem(&drive->error_telem, &msg)) {
					drive->msg_update[id] = true;

					// disable control, set error?
					if(drive->error_telem.error_severity == AdroitComs::ERROR_CRITICAL) {
						ROS_ERROR("Drive %d reported Error Code %d", addr, drive->error_telem.error_code);
						control_enabled = false;
					}
					else {
						ROS_WARN("Drive %d reported Error Code %d", addr, drive->error_telem.error_code);
					}

					hdt_error_telem_msg[addr].error_code=adroit_drives[addr]->error_telem.error_code;
					hdt_error_telem_msg[addr].error_severity=adroit_drives[addr]->error_telem.error_severity;
					hdt_error_telem_msg[addr].error_value=adroit_drives[addr]->error_telem.error_value;

					if(hdt_error_telem_pub[addr])
					  hdt_error_telem_pub[addr].publish(hdt_error_telem_msg[addr]);
					else
					  std::cout<<"empty publisher"<<std::endl;
					
				}
				break;
			case(AdroitComs::HIGH_SPEED_TELEM):
				if(COMS->process_hs_telem(&drive->hs_telem, &msg)) {
					drive->msg_update[id] = true;

					hdt_hs_telem_msg[addr].position=adroit_drives[addr]->hs_telem.position;
					hdt_hs_telem_msg[addr].velocity=adroit_drives[addr]->hs_telem.velocity;
					hdt_hs_telem_msg[addr].effort=adroit_drives[addr]->hs_telem.effort;
					hdt_hs_telem_msg[addr].motor_current=adroit_drives[addr]->hs_telem.motor_current;

					if(hdt_hs_telem_pub[addr])
					  hdt_hs_telem_pub[addr].publish(hdt_hs_telem_msg[addr]);
					else
					  std::cout<<"empty publisher"<<std::endl;
					

				}
				break;
			case(AdroitComs::MEDIUM_SPEED_TELEM):
				if(COMS->process_ms_telem(&drive->ms_telem, &msg)) {
					drive->msg_update[id] = true;

					hdt_ms_telem_msg[addr].var1=adroit_drives[addr]->ms_telem.var1;
					hdt_ms_telem_msg[addr].var2=adroit_drives[addr]->ms_telem.var2;
					hdt_ms_telem_msg[addr].var3=adroit_drives[addr]->ms_telem.var3;
					hdt_ms_telem_msg[addr].var4=adroit_drives[addr]->ms_telem.var4;

					if(hdt_ms_telem_pub[addr])
					  hdt_ms_telem_pub[addr].publish(hdt_ms_telem_msg[addr]);
					else
					  std::cout<<"empty publisher"<<std::endl;

				}
				break;
			case(AdroitComs::LOW_SPEED_TELEM):
				if(COMS->process_ls_telem(&drive->ls_telem, &msg)) {
					drive->msg_update[id] = true;

					hdt_ls_telem_msg[addr].bus_voltage=adroit_drives[addr]->ls_telem.bus_voltage;
					hdt_ls_telem_msg[addr].bus_current=adroit_drives[addr]->ls_telem.bus_current;
					hdt_ls_telem_msg[addr].temperature=adroit_drives[addr]->ls_telem.temperature;

					if(hdt_ls_telem_pub[addr])
					  hdt_ls_telem_pub[addr].publish(hdt_ls_telem_msg[addr]);
					else
					  std::cout<<"empty publisher"<<std::endl;
				}
				break;
			case(AdroitComs::DEBUG_TELEM):
				if(COMS->process_debug_telem(&drive->debug_telem, &msg)) {
					drive->msg_update[id] = true;
					
					hdt_debug_telem_msg[addr].index=adroit_drives[addr]->debug_telem.index;
					hdt_debug_telem_msg[addr].index=adroit_drives[addr]->debug_telem.index;
					hdt_debug_telem_msg[addr].hs_samples=adroit_drives[addr]->debug_telem.hs_samples;
					hdt_debug_telem_msg[addr].ls_var1=adroit_drives[addr]->debug_telem.ls_var1;
					hdt_debug_telem_msg[addr].ls_var2=adroit_drives[addr]->debug_telem.ls_var2;
					hdt_debug_telem_msg[addr].ls_var3=adroit_drives[addr]->debug_telem.ls_var3;

					for(int i=0; i<16; i++){
					  hdt_debug_telem_msg[addr].hs_var1[i]=adroit_drives[addr]->debug_telem.hs_var1[i];
					  hdt_debug_telem_msg[addr].hs_var2[i]=adroit_drives[addr]->debug_telem.hs_var2[i];

					}

					hdt_debug_telem_msg[addr].crc=adroit_drives[addr]->debug_telem.crc;

					if(hdt_debug_telem_pub[addr])
					  hdt_debug_telem_pub[addr].publish(hdt_debug_telem_msg[addr]);
					else
					  std::cout<<"empty publisher"<<std::endl;

					msg_count=msg_count+1;
				}
				break;
			case(AdroitComs::PARAMETER_TELEM): 
				if(COMS->process_parameter_telem(&drive->parameter_telem, &msg)) {
					drive->msg_update[id] = true;
					//ROS_INFO("Message recieved from Drive: %d",addr);
					// post drive semaphore
					sem_post(&drive->semaphore);

					hdt_param_telem_msg[addr].head=adroit_drives[addr]->parameter_telem.head;
					hdt_param_telem_msg[addr].index=adroit_drives[addr]->parameter_telem.index;
					hdt_param_telem_msg[addr].sub_index=adroit_drives[addr]->parameter_telem.sub_index;
					hdt_param_telem_msg[addr].data=adroit_drives[addr]->parameter_telem.data;

					if(hdt_param_telem_pub[addr])
					  hdt_param_telem_pub[addr].publish(hdt_param_telem_msg[addr]);
					else
					  std::cout<<"empty publisher"<<std::endl;
				}
				else{
				  ROS_INFO("Did not recieve a parameter telem message from drive");
				}
				break;
			case(AdroitComs::STATUS_TELEM):

				if(COMS->process_status_telem(&drive->status_telem, &msg)) {
				  
					drive->msg_update[id] = true;

					hdt_status_telem_msg[addr].state=adroit_drives[addr]->status_telem.state;

					if(hdt_status_telem_pub[addr])
					  hdt_status_telem_pub[addr].publish(hdt_status_telem_msg[addr]);
					else
					  std::cout<<"empty publisher"<<std::endl;
					
				}
				else{
				  ROS_INFO("Did not recieve a status telem message from drive");
				}
				break;
			case(AdroitComs::CONTROL_CMD):
				if(COMS->process_control_cmd(&drive->control_cmd, &msg)) {
					drive->msg_update[id] = true;
					hdt_can_control_cmd_telem_msg[addr].position=adroit_drives[addr]->control_cmd.position;
					hdt_can_control_cmd_telem_msg[addr].velocity=adroit_drives[addr]->control_cmd.velocity;
					hdt_can_control_cmd_telem_msg[addr].effort=adroit_drives[addr]->control_cmd.effort;
					hdt_can_control_cmd_telem_msg[addr].motor_current=adroit_drives[addr]->control_cmd.motor_current;

					if(hdt_can_control_cmd_telem_pub[addr])
					  hdt_can_control_cmd_telem_pub[addr].publish(hdt_can_control_cmd_telem_msg[addr]);
					else
					  std::cout<<"empty publisher"<<std::endl;
				}
				else{
				  ROS_INFO("Did not recieve a status telem message from drive");
				}
				break;
			default:
				num--;
				break;
		}
	}

	//std::cout<<msg_count<<std::endl;
	return num;
}

/*----------------------------------------------------------------------------
  timer callback
 *----------------------------------------------------------------------------*/
void AdroitDriver::TimerCallback(const ros::TimerEvent &event) {
	//ROS_INFO("TimerCallback");

	// send control commands to drive
	if(control_enabled) {
		// check joint cmd timer
		if(ros::Time::now() < joint_cmd_time + ros::Duration(JOINT_CMD_TIMEOUT)) {
			for(it_type iterator = active_drives.begin(); iterator != active_drives.end(); iterator++) {
				COMS->send_control_cmd(iterator->second, &adroit_drives[iterator->second]->control_cmd);
					hdt_control_cmd_telem_msg[iterator->second].position=adroit_drives[iterator->second]->control_cmd.position;
					hdt_control_cmd_telem_msg[iterator->second].velocity=adroit_drives[iterator->second]->control_cmd.velocity;
					hdt_control_cmd_telem_msg[iterator->second].effort=adroit_drives[iterator->second]->control_cmd.effort;
					hdt_control_cmd_telem_msg[iterator->second].motor_current=adroit_drives[iterator->second]->control_cmd.motor_current;

					if(hdt_control_cmd_telem_pub[iterator->second])
					  hdt_control_cmd_telem_pub[iterator->second].publish(hdt_control_cmd_telem_msg[iterator->second]);
					else
					  std::cout<<"empty publisher"<<std::endl;
			}
		}
	}

	// send waiting CAN messages
	COMS->Write();

	// process messages
	ProcessMessages();

	// update telem msg
	int i = 0;
	for(it_type iterator = active_drives.begin(); iterator != active_drives.end(); iterator++) {
		joint_telem_msg.position[i] = adroit_drives[iterator->second]->hs_telem.position;
		joint_telem_msg.velocity[i] = adroit_drives[iterator->second]->hs_telem.velocity;
		joint_telem_msg.effort[i] = adroit_drives[iterator->second]->hs_telem.effort;


		hdt_telem_msg.position[i] = adroit_drives[iterator->second]->hs_telem.position;
		hdt_telem_msg.velocity[i] = adroit_drives[iterator->second]->hs_telem.velocity;
		hdt_telem_msg.effort[i] = adroit_drives[iterator->second]->hs_telem.effort;
		hdt_telem_msg.motor_current[i] = adroit_drives[iterator->second]->hs_telem.motor_current;
		hdt_telem_msg.bus_voltage[i] = adroit_drives[iterator->second]->ls_telem.bus_voltage;
		hdt_telem_msg.bus_current[i] = adroit_drives[iterator->second]->ls_telem.bus_current;
		hdt_telem_msg.temperature[i] = adroit_drives[iterator->second]->ls_telem.temperature;

		i++;
	}

	// publish telem msg
	joint_telem_msg.header.stamp = hdt_telem_msg.header.stamp = ros::Time::now();
	joint_telem_pub.publish(joint_telem_msg);
	hdt_telem_pub.publish(hdt_telem_msg);

}

/*----------------------------------------------------------------------------
  joint command callback
 *----------------------------------------------------------------------------*/
void AdroitDriver::JointCmdCallback(const sensor_msgs::JointState& msg) {
	//ROS_INFO("JointCmdCallback");

	//boost::mutex::scoped_lock lock(*mutex);

	// iterate through command
	for(int i = 0; i < msg.name.size(); i++) {
		// update drive
		try {
			int addr = active_drives[msg.name[i]];
			adroit_drives[addr]->control_cmd.position = msg.position[i];
			adroit_drives[addr]->control_cmd.velocity = msg.velocity[i];
			adroit_drives[addr]->control_cmd.effort = msg.effort[i];
			adroit_drives[addr]->control_cmd.motor_current = MAX_DRIVE_CURRENT;
		}
		catch(...) {
			// invalid drive name in cmd message'
			ROS_ERROR("JointCmdCallback invalid joint name");
		}
	}
	
	// update timer
	//joint_cmd_time = msg.header.stamp;
	joint_cmd_time = ros::Time::now();
}



/*----------------------------------------------------------------------------
  joint command callback
 *----------------------------------------------------------------------------*/
void AdroitDriver::HDTJointCmdCallback(const hdt_adroit_driver::HDTJointState& msg) {
	//ROS_INFO("JointCmdCallback");

	//boost::mutex::scoped_lock lock(*mutex);

	// iterate through command
	for(int i = 0; i < msg.name.size(); i++) {
		// update drive
		try {
			int addr = active_drives[msg.name[i]];
			adroit_drives[addr]->control_cmd.position = msg.position[i];
			adroit_drives[addr]->control_cmd.velocity = msg.velocity[i];
			adroit_drives[addr]->control_cmd.effort = msg.effort[i];
			adroit_drives[addr]->control_cmd.motor_current = msg.motor_current[i];
			
		}
		catch(...) {
			// invalid drive name in cmd message'
			ROS_ERROR("JointCmdCallback invalid joint name");
		}
	}
	
	// update timer
	//joint_cmd_time = msg.header.stamp;
	joint_cmd_time = ros::Time::now();
}

/*----------------------------------------------------------------------------
   coms ready callback
 *----------------------------------------------------------------------------*/
bool AdroitDriver::ComsReadyCallback(hdt_adroit_driver::ComsReady::Request& req, hdt_adroit_driver::ComsReady::Response& res) {
	res.telem = joint_telem_msg;
	return true;
}

/*----------------------------------------------------------------------------
  status callback
 *---------------------------------------------------------------------------*/
bool AdroitDriver::GetStatusServiceCallback(hdt_adroit_driver::GetStatus::Request& req, hdt_adroit_driver::GetStatus::Response& res) {
  // reset msg_update
  for(int i = 0; i < MAX_DRIVES; i++) {
    adroit_drives[i]->msg_update[AdroitComs::STATUS_TELEM] = false;
  }

  // send status command and sleep for a moment
  COMS->send_status_cmd(BROADCAST_ADDR);
  usleep(100*1000);
  // check for responses
  for(int i = 0; i < MAX_DRIVES; i++) {
    if(adroit_drives[i]->msg_update[AdroitComs::STATUS_TELEM] == true) {
      res.addr=(uint32_t)i;
      res.state=(uint32_t)adroit_drives[i]->status_telem.state;
    }
    else {
      COMS->reset_drive(adroit_drives[i]);
    }
  }
  
  return true;

}

/*---------------------------------------------------------------------------
  commit callback
 *--------------------------------------------------------------------------*/

bool AdroitDriver::CommitServiceCallback(hdt_adroit_driver::Commit::Request& req, hdt_adroit_driver::Commit::Response& res) {

  //ROS_INFO("entered commit");
  int addr= active_drives[req.name.c_str()];
  // check for valid addr
  if((addr < 0) || (addr >= MAX_DRIVES)) {
    ROS_ERROR("invalid addr...");
    return false;
  }
  
  // commit parameters
  if(COMS->commit_parameters(addr, adroit_drives[addr])) {
    ROS_INFO("commit successful");
    res.ret=0;
  }
  else {
    ROS_ERROR("commit failed");
    res.ret=1;
  }
  return true;
}

/*------------------------------------------------------------------------------
  write param callback
 *----------------------------------------------------------------------------*/
bool AdroitDriver::WriteParamServiceCallback(hdt_adroit_driver::WriteDriveParam::Request& req, hdt_adroit_driver::WriteDriveParam::Response& res) {
  uint32_t data=req.value;
  int addr=active_drives[req.name.c_str()];
  int index=req.index;
  
  try {
    switch(PARAMS->get_type_by_index(index)) {

    case(INT_TYPE): {
      int int_data;
      memcpy(&int_data, &data, sizeof(data));
      break;
    }

    case(FLOAT_TYPE): {
      float float_data;
      memcpy(&float_data, &data, sizeof(data));
      break;
    }

    default:
      ROS_ERROR("invalid input");
      return false;
    }
  }

  catch(...) {
    ROS_ERROR("invalid input...");
    return false;
  }

  if(COMS->write_parameter(addr, index, adroit_drives[addr], &data)) {
    if(COMS->read_parameter(addr, index, adroit_drives[addr])) {
      res.ret=0;
    }
  }
  else {
    res.ret=1;
    ROS_ERROR("parameter write failed");
  }

  return true;
}

/*--------------------------------------------------------------------------
  read param callback
  * -----------------------------------------------------------------------------*/

bool AdroitDriver::ReadParamServiceCallback(hdt_adroit_driver::ReadDriveParam::Request& req, hdt_adroit_driver::ReadDriveParam::Response& res) {


  int addr=active_drives[req.name.c_str()];
  int index=req.index;
  //ROS_INFO("Reading from Drive: %d Index: %d",addr, index);

  if(COMS->read_parameter(addr, index, adroit_drives[addr])) {
    
    uint32_t data=adroit_drives[addr]->parameter_telem.data;
    memcpy(&res.ret, &data, sizeof(data));
    return true;
  }

  else {
    ROS_ERROR("parameter read unsuccessful. Drive: %d. Index: %d.",addr, index);
    return false;
  }
}

/*-----------------------------------------------------------------------------
  appload callback
  *----------------------------------------------------------------------------*/
bool AdroitDriver::AppLoadServiceCallback(hdt_adroit_driver::AppLoad::Request& req, hdt_adroit_driver::AppLoad::Response& res) {
  // load file
  AdroitApp adroit_app;
  int addr=active_drives[req.name.c_str()];

  // check file name extension
  std::string ext = req.file.substr(req.file.find_last_of(".") + 1);
  // load and convert hex file
  if(ext == "hex") {
    if(adroit_app.read_hex_file(req.file)) {
      //ROS_INFO("converting hex file to binary...");
      adroit_app.hex_to_bin();
    }
    else {
      ROS_ERROR("error reading hex file");
      res.ret=0;
      return false;
    }
  }
  // load binary file
  else if(ext == "BIN") {
    ROS_ERROR("no support for binary files yet...");
    res.ret=0;
    return false;
  }

  else {
    ROS_ERROR("invalid file extension...");
    res.ret=0;
    return false;
  }

  // send application image
  if(!(COMS->load_application_image(addr, adroit_app.bin_data, adroit_drives[addr]))) {
    ROS_ERROR("application load failed...");
    res.ret=0;
    return false;
  }
  
  res.ret=1;
  return true;
}

/*---------------------------------------------------------------------
  State Select Service Callback
  *-------------------------------------------------------------------*/
bool AdroitDriver::StateSelectServiceCallback(hdt_adroit_driver::StateSelect::Request& req, hdt_adroit_driver::StateSelect::Response& res) {
  // load file
  AdroitApp adroit_app;
  int addr=active_drives[req.name.c_str()];
  uint8_t state=req.state;
 
	// send state command
		if(state >= 0 && state < AdroitComs::NUM_STATES && addr >= 0 && addr < MAX_DRIVES) {
			AdroitComs::StateCmd cmd;
			cmd.state = state;
			COMS->send_state_cmd(addr, &cmd);
		}
		else {
		        ROS_ERROR("invalid arguments...");
			res.ret=1;
			return false;
		}
 
  res.ret=0;
  return true;
}


/*------------------------------------------------------------------
   send impedance commands
 *----------------------------------------------------------------------------*/
void AdroitDriver::SendImpedanceCommands(double damping_ratio, double stiffness_ratio, bool stiffness_enabled) {
	ROS_INFO("Sending impedance commands");
	for(it_type iterator = active_drives.begin(); iterator != active_drives.end(); iterator++) {
		// calculate impedance parameters
		COMS->set_impedance_cmd(adroit_drives[iterator->second], damping_ratio, stiffness_ratio);
		COMS->send_impedance_cmd(iterator->second, &adroit_drives[iterator->second]->impedance_cmd);
	}
}


