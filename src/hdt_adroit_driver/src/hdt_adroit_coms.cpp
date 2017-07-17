#include "ros/ros.h"

#include "hdt_adroit_coms.h"
#include "hdt_adroit_driver.h"

// coms interfaces
#include "hdt_pcan_interface.h"
#include "hdt_loopback_interface.h"
#include "hdt_tcpcoms_interface.h"
#include "hdt_socketcan_interface.h"

#include <tinyxml.h>

// coms interface
static AdroitInterface *COMS;

// adroit driver
static AdroitDriver adroit_driver;

/*----------------------------------------------------------------------------
  main function
 *----------------------------------------------------------------------------*/
int main(int argc, char **argv) {
	ros::init(argc, argv, ROS_NAME);
	ros::NodeHandle node_handle;
	int ret;

	ROS_INFO("%s starting", ROS_NAME);

	// start spinner
	ros::AsyncSpinner spinner(3);
	spinner.start();

	// get params
	ros::NodeHandle private_node_handle("~");
	std::string coms_interface, baud_rate, params_file, ip_address, device;
	int port;
	private_node_handle.param<std::string>("coms_interface", coms_interface, "loopback");
	private_node_handle.param<std::string>("baud_rate", baud_rate, "CAN_BAUD_1M");
	//private_node_handle.param<std::string>("params_file", params_file, "adroit_params.xml");
	private_node_handle.param<std::string>("ip_address", ip_address, "127.0.0.1");
	private_node_handle.param<int>("port", port, 8765);
	private_node_handle.param<std::string>("device", device, "can0");

	// coms interface
	if(coms_interface == "pcan") {
		COMS = new PcanInterface(baud_rate);
	}
	else if(coms_interface == "loopback") {
		COMS = new LoopbackInterface();
	}
	else if(coms_interface == "tcpcoms") {
		COMS = new TcpComsInterface(ip_address, port);
	}
	else if(coms_interface == "socketcan") {
		COMS = new SocketcanInterface(device);
	}
	else {
		ROS_ERROR("Invalid COMS Interface %s", coms_interface.c_str());
		return -1;
	}

	// initialize coms
	if(COMS->Init() <= 0) {
		ROS_ERROR("Failed to initialize COMS");
		return -1;
	}

	// initialize adroit driver
	if(adroit_driver.Init(COMS) <= 0) {
		ROS_ERROR("Failed to initialize Adroit Driver");
		return -1;
	}
	
	return 0;
}

