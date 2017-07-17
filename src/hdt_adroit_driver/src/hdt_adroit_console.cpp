#include <fstream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <readline/readline.h>
#include <readline/history.h>

#include "ros/ros.h"

#include "hdt_adroit_console.h"
#include "AdroitApp.h"

/*----------------------------------------------------------------------------
  main ui
 *----------------------------------------------------------------------------*/
int AdroitConsole::Init(AdroitComs *coms, AdroitComs::AdroitDrive **drives) {
	// copy pointers
	COMS = coms;
	Drives = drives;

	// get params file
	ros::NodeHandle private_node_handle("~");
	std::string params_file;
	private_node_handle.param<std::string>("params_file", params_file, "adroit_params.xml");

	// load params
	PARAMS = new AdroitParams();
	if(!(PARAMS->load_file(params_file))) {
		ROS_ERROR("could not load params file: %s", params_file.c_str());
		return -1;
	}

	// start receive thread
	LoopThread = new boost::thread(boost::bind(&AdroitConsole::Loop, this));
	LoopThread->join();
	
	return 0;
}

/*----------------------------------------------------------------------------
  loop
 *----------------------------------------------------------------------------*/
void AdroitConsole::Loop(void) {
	std::string input_line;
	std::vector<std::string> lines;
	char *buf;
	rl_bind_key('\t',rl_abort); // disable auto-complete
	
	ROS_INFO("starting console");

	// main loop
	MainMenu();
	while((buf = readline(">> ")) != NULL) {
		// split lines 
		input_line = buf;
		Split(input_line, lines, ' '); // split everything separated by spaces
		
		// input specific action
		if(lines.size() == 0) {
			continue;
		}
		else if(lines[0] == "q") {
			break;
		}
		else {
			// process command
			MainProcessCommand(lines);

			// add to history
			if (buf[0]!=0) add_history(buf);
		}
	}
	free(buf);
}

/*----------------------------------------------------------------------------
  process command
 *----------------------------------------------------------------------------*/
void AdroitConsole::MainProcessCommand(std::vector<std::string> lines) {
	// process command input
	if(lines[0] == "s") {
		// reset msg_update
		for(int i = 0; i < MAX_DRIVES; i++) {
			Drives[i]->msg_update[AdroitComs::STATUS_TELEM] = false;
		}

		// send status command and sleep for a moment
		COMS->send_status_cmd(BROADCAST_ADDR);
		usleep(100*1000);

		// check for responses
		for(int i = 0; i < MAX_DRIVES; i++) {
			if(Drives[i]->msg_update[AdroitComs::STATUS_TELEM] == true) {
				ROS_INFO("Addr: %3d, State: %3d", i, Drives[i]->status_telem.state);
			}
			else {
				COMS->reset_drive(Drives[i]);
			}
		}

	}
	else if(lines[0] == "cs") {
		int addr, state;

		// check for valid arguments
		if(lines.size() != 3) {
			ROS_ERROR("invalid arguments...");
			return;
		}

		// parse input
		try {
			std::istringstream(lines[1]) >> addr;
			std::istringstream(lines[2]) >> state;
		}
		catch(...) {
			ROS_ERROR("invalid arguments...");
			return;
		}

		// send state command
		if(state >= 0 && state < AdroitComs::NUM_STATES && addr >= 0 && addr < MAX_DRIVES) {
			AdroitComs::StateCmd cmd;
			cmd.state = state;
			COMS->send_state_cmd(addr, &cmd);
		}
		else {
			ROS_ERROR("invalid arguments...");
		}
	}
	else if(lines[0] == "a") {
		int addr;

		// check for valid arguments
		if(lines.size() != 3) {
			ROS_ERROR("invalid arguments...");
			return;
		}

		// parse input
		try {
			std::istringstream(lines[1]) >> addr;
		}
		catch(...) {
			ROS_ERROR("invalid arguments...");
			return;
		}

		// check for valid addr
		if((addr < 0) || (addr >= MAX_DRIVES)) {
			ROS_ERROR("invalid arguments...");
			return;
		}

		// load file
		AdroitApp adroit_app;

		// check file name extension
		std::string ext = lines[2].substr(lines[2].find_last_of(".") + 1);
		// load and convert hex file
		if(ext == "hex") {
			if(adroit_app.read_hex_file(lines[2])) {
				ROS_INFO("converting hex file to binary...");
				adroit_app.hex_to_bin();
			}
			else {
				ROS_ERROR("error reading hex file");
				return;
			}
		}
		// load binary file
		else if(ext == "BIN") {
			ROS_ERROR("no support for binary files yet...");
		}
		else {
			ROS_ERROR("invalid file extension...");
			return;
		}

		// send application image
		if(!(COMS->load_application_image(addr, adroit_app.bin_data, Drives[addr]))) {
			ROS_ERROR("application load failed...");
		}	
	}
	else if(lines[0] == "param") {
		int index, index2;

		// check for valid arguments
		if(lines.size() == 2) {
			// parse input
			try {
				std::istringstream(lines[1]) >> index;
				index2 = index;
			}
			catch(...) {
				ROS_ERROR("invalid input...");
				return;
			}
		}
		else if(lines.size() == 3) {
			// parse input
			try {
				std::istringstream(lines[1]) >> index;
				std::istringstream(lines[1]) >> index2;
			}
			catch(...) {
				ROS_ERROR("invalid input...");
				return;
			}
		}
		else {
			ROS_ERROR("invalid arguments...");
			return;
		}

		// check for valid index
		if(((index < 0) && (index >= NUM_FLASH_PARAMETERS)) || ((index2 < 0) && (index2 >= NUM_FLASH_PARAMETERS))) {
			ROS_ERROR("invalid index...");
			return;
		}

		// display parameter(s)
		for(int i = index; i <= index2; i++) {
			ROS_INFO("Param: %3d, Name: %s", i, PARAMS->get_name_by_index(i).c_str());
		}
	}
	else if(lines[0] == "r") {
		int addr, index, index2;

		// check for valid arguments
		if(lines.size() == 3) {
			// parse input
			try {
				std::istringstream(lines[1]) >> addr;
				std::istringstream(lines[2]) >> index;
				index2 = index;
			}
			catch(...) {
				ROS_ERROR("invalid input...");
				return;
			}
		}
		else if(lines.size() == 4) {
			// parse input
			try {
				std::istringstream(lines[1]) >> addr;
				std::istringstream(lines[2]) >> index;
				std::istringstream(lines[3]) >> index2;
			}
			catch(...) {
				ROS_ERROR("invalid input...");
				return;
			}
		}
		else {
			ROS_ERROR("invalid arguments...");
			return;
		}

		// check for valid addr
		if((addr < 0) || (addr >= MAX_DRIVES)) {
			ROS_ERROR("invalid addr...");
			return;
		}

		// check for valid index
		if(((index < 0) && (index >= NUM_FLASH_PARAMETERS)) || ((index2 < 0) && (index2 >= NUM_FLASH_PARAMETERS))) {
			ROS_ERROR("invalid index...");
			return;
		}

		// read parameter(s)
		for(int i = index; i <= index2; i++) {
			if(COMS->read_parameter(addr, i, Drives[addr])) {
				ROS_INFO("Addr: %3d, Param: %3d, Data: %s", addr, i, param_to_string(&Drives[addr]->parameter_telem.data, PARAMS->get_type_by_index(i)).c_str());
			}
			else {
				ROS_ERROR("parameter read unsuccessful");
				return;
			}
		}
	}
	else if(lines[0] == "ra") {
		int addr;

		// check for valid arguments
		if(lines.size() != 3) {
			ROS_ERROR("invalid arguments...");
			return;
		}

		// parse input
		try {
			std::istringstream(lines[1]) >> addr;
		}
		catch(...) {
			ROS_ERROR("invalid input...");
			return;
		}

		// check for valid addr
		if((addr < 0) || (addr >= MAX_DRIVES)) {
			ROS_ERROR("invalid addr...");
			return;
		}

		// check file name extension
		std::string ext = lines[2].substr(lines[2].find_last_of(".") + 1);
			
		// read all parameters
		if(ext == "xml") {
			std::vector<uint32_t> param_data;

			// read all parameters
			for(int i = 0; i < NUM_FLASH_PARAMETERS; i++) {
				// read parameter
				if(COMS->read_parameter(addr, i, Drives[addr])) {
					param_data.push_back(Drives[addr]->parameter_telem.data);
				}
				else {
					ROS_ERROR("parameter read unsuccessful...");
					return;
				}
			}

			// write params to xml file
			if(param_data.size() == NUM_FLASH_PARAMETERS) {
				PARAMS->params_to_xml(param_data, lines[2]);
			}
			else {
				ROS_ERROR("parameter read unsuccessful...");
			}
		}
		else {
			ROS_ERROR("invalid file extension...");
			return;
		}
	}
	else if(lines[0] == "w") {
		int addr, index;

		// check for valid arguments
		if(lines.size() != 4) {
			ROS_ERROR("invalid arguments...");
			return;
		}

		// parse input
		try {
			std::istringstream(lines[1]) >> addr;
			std::istringstream(lines[2]) >> index;
		}
		catch(...) {
			ROS_ERROR("invalid input...");
			return;
		}

		// check for valid addr
		if((addr < 0) || (addr >= MAX_DRIVES)) {
			ROS_ERROR("invalid addr...");
			return;
		}

		// check for valid index
		if((index < 0) && (index >= NUM_FLASH_PARAMETERS)) {
			ROS_ERROR("invalid index...");
			return;
		}

		// get input data
		uint32_t data;
		try {
			switch(PARAMS->get_type_by_index(index)) {
			case(INT_TYPE): {
				int int_data;
				std::istringstream(lines[3]) >> int_data;
				memcpy(&data, &int_data, sizeof(int_data));
				break;
				}
			case(FLOAT_TYPE): {
				float float_data;
				std::istringstream(lines[3]) >> float_data;
				memcpy(&data, &float_data, sizeof(float_data));
				break;
				}
			default:
				ROS_ERROR("invalid input");
				return;
			}
		}
		catch(...) {
			ROS_ERROR("invalid input...");
			return;
		}

		// write parameter
		if(COMS->write_parameter(addr, index, Drives[addr], &data)) {
			if(COMS->read_parameter(addr, index, Drives[addr])) {
				ROS_INFO("Addr: %3d, Param: %3d, Data: %s", addr, index, param_to_string(&Drives[addr]->parameter_telem.data, PARAMS->get_type_by_index(index)).c_str());
			}
		}
		else {
			ROS_ERROR("parameter write failed");
		}
	}
	else if(lines[0] == "wa") {
		int addr;

		// make sure there is a file name
		if(lines.size() != 3) {
			ROS_ERROR("invalid arguments");
			return;
		}

		// parse input
		try {
			std::istringstream(lines[1]) >> addr;
		}
		catch(...) {
			ROS_ERROR("invalid input...");
			return;
		}

		// check for valid addr
		if((addr < 0) || (addr >= MAX_DRIVES)) {
			ROS_ERROR("invalid addr...");
			return;
		}

		// check file name extension
		std::string ext = lines[2].substr(lines[2].find_last_of(".") + 1);
			
		// read all parameters
		if(ext == "xml") {
			std::vector<uint32_t> param_data;
			PARAMS->xml_to_params(param_data, lines[2]);

			// write all parameters (except 0)
			for(int i = 1; i < NUM_FLASH_PARAMETERS; i++) {
				// write parameter
				if(COMS->write_parameter(addr, i, Drives[addr], &param_data[i])) {
				}
				else {
					ROS_ERROR("parameter write failed");
					return;
				}
			}
		}
	}
	else if(lines[0] == "c") {
		int addr;

		// make sure there is a file name
		if(lines.size() != 2) {
			ROS_ERROR("invalid arguments");
			return;
		}

		// parse input
		try {
			std::istringstream(lines[1]) >> addr;
		}
		catch(...) {
			ROS_ERROR("invalid input...");
			return;
		}

		// check for valid addr
		if((addr < 0) || (addr >= MAX_DRIVES)) {
			ROS_ERROR("invalid addr...");
			return;
		}

		// commit parameters
		if(COMS->commit_parameters(addr, Drives[addr])) {
			ROS_INFO("commit successful");
		}
		else {
			ROS_ERROR("commit failed");
		}
	}
	else {
		MainMenu();
	}
}

/*----------------------------------------------------------------------------
  main menu
 *----------------------------------------------------------------------------*/
void AdroitConsole::MainMenu(void) {
	// print menu
	std::cout << "s  - status" << std::endl;
	std::cout << "cs - change state" << std::endl;
	std::cout << "a  - appload" << std::endl;
	std::cout << "r  - read" << std::endl;
	std::cout << "ra - read all" << std::endl;
	std::cout << "w  - write" << std::endl;
	std::cout << "wa - write all" << std::endl;
	std::cout << "c  - commit" << std::endl;
	std::cout << "q  - quit" << std::endl;
}

/*----------------------------------------------------------------------------
  split function
 *----------------------------------------------------------------------------*/
unsigned int AdroitConsole::Split(const std::string &s, std::vector<std::string> &elems, char delim) {
	elems.clear();
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems.size();
}

