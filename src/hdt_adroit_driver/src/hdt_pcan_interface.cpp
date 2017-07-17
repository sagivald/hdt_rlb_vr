#include "ros/ros.h"

#include "hdt_pcan_interface.h"

/*----------------------------------------------------------------------------
  constructor
 *----------------------------------------------------------------------------*/
PcanInterface::PcanInterface(std::string baud_rate) : AdroitInterface()  {
	// get baud rate
	if(baud_rate == "CAN_BAUD_1M"){
		BaudRate = CAN_BAUD_1M;
	}
	else if(baud_rate == "CAN_BAUD_500K"){
		BaudRate = CAN_BAUD_500K;
	}
	else if(baud_rate == "CAN_BAUD_250K"){
		BaudRate = CAN_BAUD_250K;
	}
	else if(baud_rate == "CAN_BAUD_125K"){
		BaudRate = CAN_BAUD_250K;
	}
	else if(baud_rate == "CAN_BAUD_100K"){
		BaudRate = CAN_BAUD_100K;
	}
	else if(baud_rate == "CAN_BAUD_50K"){
		BaudRate = CAN_BAUD_50K;
	}
	else if(baud_rate == "CAN_BAUD_20K"){
		BaudRate = CAN_BAUD_20K;
	}
	else if(baud_rate == "CAN_BAUD_10K"){
		BaudRate = CAN_BAUD_10K;
	}
	else if(baud_rate == "CAN_BAUD_5K"){
		BaudRate = CAN_BAUD_5K;
	}
	else{
		ROS_ERROR("Invalid CAN baud rate specified %s, reverting to CAN_BAUD_1M", baud_rate.c_str());
		BaudRate = CAN_BAUD_1M;
	}
}

/*----------------------------------------------------------------------------
  destructor
 *----------------------------------------------------------------------------*/
PcanInterface::~PcanInterface() {
	if(CAN_Close(CanHandle)) {
		ROS_INFO("PcanInterface:CAN_Close");
	}
}

/*----------------------------------------------------------------------------
  init
 *----------------------------------------------------------------------------*/
int PcanInterface::Init(void) {
	int ret;

	// open can handle
	CanHandle = CAN_Open(HW_USB, 0, 0);
	if (!CanHandle) {
		ROS_ERROR("PcanInterface:Init:CAN_Open");
		return -1;
	}

	// init to an user defined bit rate
	ret = CAN_Init(CanHandle, BaudRate, CAN_INIT_TYPE_ST);
	if (ret) {
		ROS_ERROR("PcanInterface:Init:CAN_Init");
		return -1;
	}

	// Start the receive thread
	TxMutex = new boost::mutex();
	RxMutex = new boost::mutex();
	ReceiveThread = new boost::thread(boost::bind(&PcanInterface::Read, this));

	return 1;
}

/*----------------------------------------------------------------------------
  send msg
 *----------------------------------------------------------------------------*/
int PcanInterface::SendMsg(AdroitMsg *adroit_msg) {
	// get mutex lock
	boost::mutex::scoped_lock lock(*TxMutex);
	
	// push msg to list
	TxList.push_back(*adroit_msg);

	return 1;
}

/*----------------------------------------------------------------------------
  transmit all waiting messages
 *----------------------------------------------------------------------------*/
int PcanInterface::Write(void) {
	AdroitMsg adroit_msg;
	int ret;
	TPCANMsg can_msg;

	// get mutex lock
	boost::mutex::scoped_lock lock(*TxMutex);

	// loop through messages
	while(!TxList.empty()) {
		// get first message in list
		adroit_msg = TxList.front();
		TxList.pop_front();
		
		// create can message
		can_msg.ID = adroit_msg.id;
		can_msg.LEN = adroit_msg.dlc;
		//can_msg.m_bFF = adroit_msg->ff;
		can_msg.MSGTYPE = MSGTYPE_STANDARD;
		memcpy(&can_msg.DATA[0], &adroit_msg.data[0], 8);

		ret = LINUX_CAN_Write_Timeout(CanHandle, &can_msg, 0);
		if (ret) {
			ROS_ERROR("PcanInterface:Transmit error");
			return -1;
		}
	}

	return 1;
}

/*----------------------------------------------------------------------------
  receive msg
 *----------------------------------------------------------------------------*/
int PcanInterface::ReceiveMsg(AdroitMsg *msg) {
	// get mutex lock
	boost::mutex::scoped_lock lock(*RxMutex);
	
	// get first message in list
	if(!RxList.empty()) {
		*msg = RxList.front();
		RxList.pop_front();
		return 1;
	}
	else return -1;
}

/*----------------------------------------------------------------------------
  read task
 *----------------------------------------------------------------------------*/
void PcanInterface::Read() {
	AdroitMsg adroit_msg;
	int ret;
	TPCANRdMsg can_msg;

	// read loop
	while(1) {
		// get can message
		ret = LINUX_CAN_Read(CanHandle, &can_msg);
		if(!ret) {
			// get mutex lock
			boost::mutex::scoped_lock lock(*RxMutex);

			// get adroit message
			adroit_msg.id = can_msg.Msg.ID;
			adroit_msg.dlc = can_msg.Msg.LEN;
			adroit_msg.ff = can_msg.Msg.MSGTYPE;
			memcpy(&adroit_msg.data[0], &can_msg.Msg.DATA, 8);

			// add to rx list
			RxList.push_back(adroit_msg);
		}
		else {
			ROS_ERROR("PcanInterface:Task:CAN_Read");
			break;
		}
	}
}

