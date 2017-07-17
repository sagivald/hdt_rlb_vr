#include <ros/ros.h>

#include <sys/ioctl.h>
#include <net/if.h>
#include <errno.h>

#include "hdt_socketcan_interface.h"

/*----------------------------------------------------------------------------
  constructorW
 *----------------------------------------------------------------------------*/
SocketcanInterface::SocketcanInterface(std::string device) : AdroitInterface() {
	this->device = device;
}

/*----------------------------------------------------------------------------
  destructor
 *----------------------------------------------------------------------------*/
SocketcanInterface::~SocketcanInterface() {
	close(s);
}

/*----------------------------------------------------------------------------
  init
 *----------------------------------------------------------------------------*/
int SocketcanInterface::Init(void) {
	int ret;
    struct sockaddr_can addr;
    struct ifreq ifr;

	// create socket
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

	// ioctl
    strcpy(ifr.ifr_name, device.c_str());
    ioctl(s, SIOCGIFINDEX, &ifr);

	// addr
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

	// bind
    ret = bind(s, (struct sockaddr *)&addr, sizeof(addr));
	if(ret != 0) {
		ROS_ERROR("SocketcanInterface::Init bind failure");
		return -1;
	}

	// start read thread
	TxMutex = new boost::mutex();
	RxMutex = new boost::mutex();
	ReadThread = new boost::thread(boost::bind(&SocketcanInterface::Read, this));

	return 1;
}

/*----------------------------------------------------------------------------
  send msg
 *----------------------------------------------------------------------------*/
int SocketcanInterface::SendMsg(AdroitMsg *adroit_msg) {
	// get mutex lock
	boost::mutex::scoped_lock lock(*TxMutex);
	
	// push msg to list
	TxList.push_back(*adroit_msg);

	return 1;
}

/*----------------------------------------------------------------------------
  transmit all waiting messages
 *----------------------------------------------------------------------------*/
int SocketcanInterface::Write(void) {
	AdroitMsg adroit_msg;
	struct can_frame frame;
	int ret = 1;

	// loop through messages
	while(!TxList.empty()) {
		{
			// get mutex lock
			boost::mutex::scoped_lock lock(*TxMutex);
			
			// get first message in list
			adroit_msg = TxList.front();
			TxList.pop_front();
		}
		
		// create can message
		frame.can_id = adroit_msg.id;
		frame.can_dlc = adroit_msg.dlc;
		//can_msg.is_extended = adroit_msg.ff;
		for(int i=0; i < adroit_msg.dlc; i++) {
			frame.data[i] = adroit_msg.data[i];
		}
		
		// send message
		ssize_t nbytes = write(s, &frame, sizeof(struct can_frame));
		
		// check for write error
		if (nbytes < 0) {
			ROS_ERROR("SocketcanInterface::Write can raw socket write");
			return -1;
		}
		// check for incomplete frame
		else if (nbytes < sizeof(struct can_frame)) {
			ROS_ERROR("SocketcanInterface::Write incomplete CAN frame");
			continue;
		}	
	}

	return ret;
}

/*----------------------------------------------------------------------------
  receive msg
 *----------------------------------------------------------------------------*/
int SocketcanInterface::ReceiveMsg(AdroitMsg *msg) {
	// get first message in list
	if(!RxList.empty()) {
		// get mutex lock
		boost::mutex::scoped_lock lock(*RxMutex);

		*msg = RxList.front();
		RxList.pop_front();
		return 1;
	}
	else return -1;
}

/*----------------------------------------------------------------------------
  read callback
 *----------------------------------------------------------------------------*/
void SocketcanInterface::Read() {
	AdroitMsg adroit_msg;
	struct can_frame frame;
	
	// read loop
	while(1) {
		// read from socket
		ssize_t nbytes = read(s, &frame, sizeof(struct can_frame));
		
		// check for read error
		if (nbytes < 0) {
			ROS_ERROR("SocketcanInterface::Read can raw socket read");
			break;
		}
		// check for incomplete frame
		else if (nbytes < sizeof(struct can_frame)) {
			ROS_ERROR("SocketcanInterface::Read incomplete CAN frame");
			continue;
		}	
		// check for data
		else {
			//ROS_INFO("can msg received");
		
			// move bit 31 of id downto bit 2 of flags
			//can_rx_msg_.flags = (rx_.can_id & (1 << 31)) >> 29;
			//can_rx_msg_.cob = rx_.can_id & 0x1FFFFFFF; // mask out EFF/RTR/ERR flags from id
			adroit_msg.id = frame.can_id & 0x1FFFFFFF;
			adroit_msg.dlc = frame.can_dlc;
			for (int i = 0; i < frame.can_dlc; i++) {
				adroit_msg.data[i] = frame.data[i];
			}
			
			// lock rx mutex
			boost::mutex::scoped_lock lock(*RxMutex);

			// add message to rx list
			RxList.push_back(adroit_msg);
		}
	}	
}

