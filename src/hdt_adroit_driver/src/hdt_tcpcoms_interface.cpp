#include "ros/ros.h"

#include "hdt_tcpcoms_interface.h"

/*----------------------------------------------------------------------------
  constructor
 *----------------------------------------------------------------------------*/
TcpComsInterface::TcpComsInterface(std::string ip_addr, int port) : AdroitInterface() {
	// set internal variables
	IpAddr = ip_addr;
	Port = port;
}

/*----------------------------------------------------------------------------
  destructor
 *----------------------------------------------------------------------------*/
TcpComsInterface::~TcpComsInterface() {
}

/*----------------------------------------------------------------------------
  init
 *----------------------------------------------------------------------------*/
int TcpComsInterface::Init(void) {
	ros::NodeHandle node_handle;
	
	// initialize coms client
	ComsClient = new ComsInterface(new TcpClient((char *)"ComsClient", const_cast<char *>(IpAddr.c_str()), Port, 0), new SlipProtocol());
	ComsClient->Init();

	//ROS_INFO("server = %s, port = %d", IpAddr.c_str(), Port);

	// Start the receive thread
	TxMutex = new boost::mutex();
	RxMutex = new boost::mutex();
	ReceiveThread = new boost::thread(boost::bind(&TcpComsInterface::Read, this));

	// create sample loop timer
	step_timer = node_handle.createTimer(ros::Duration(SAMPLE_PERIOD), &TcpComsInterface::TimerCallback, this);
	ROS_INFO("SAMPLE_PERIOD = %f", SAMPLE_PERIOD);
	return 1;
}

/*----------------------------------------------------------------------------
  timer callback
 *----------------------------------------------------------------------------*/
void TcpComsInterface::TimerCallback(const ros::TimerEvent &event) {
	//ROS_INFO("TcpComsInterface::TimerCallback()");
	
	// step coms
	Step();
}

/*----------------------------------------------------------------------------
  communications step
 *----------------------------------------------------------------------------*/
int TcpComsInterface::Step() {
	// parse incoming data
	ComsClient->Process();

	// receive incoming messages
	ComsInterface::ComsMsg msg;
	while(ComsClient->GetMsg(&msg) > 0) {
		// take message id dependent action
		switch(msg.id) {
			case(TCPCOMS_HEARTBEAT_CMD):
				//ROS_INFO("received heartbeat cmd");
				SendHeartbeatRes();
				break;
			case(TCPCOMS_STATE_RES):
				//ROS_INFO("received state res");
				break;
			case(TCPCOMS_JOINT_RES):
				//ROS_INFO("received joint res");
				break;
			case(TCPCOMS_WORKSPACE_RES):
				//ROS_INFO("received workspace res");
				break;
			case(TCPCOMS_GATEWAY_RES):
				//ROS_INFO("received gateway res");
				ProcessGatewayRes(msg.size, &msg.data[0]);
				break;
			default:
				break;
		}
	}

	return 1;
}

/*----------------------------------------------------------------------------
  process  command
 *----------------------------------------------------------------------------*/
int TcpComsInterface::ProcessGatewayRes(uint16_t size, uint8_t *data) {
	AdroitMsg adroit_msg;

	// check for proper message size
	if(size % GATEWAY_MSG_SIZE > 0) return 0;

	// process messages
	int num = size/GATEWAY_MSG_SIZE;
	for(int i = 0; i < num; i++) {
		int ind = i*GATEWAY_MSG_SIZE;

		// set msg data
		adroit_msg.id = data[ind] + (data[ind + 1] << 8) + (data[ind + 2] << 16) + (data[ind + 3] << 24);
		adroit_msg.dlc = data[ind + 4];
		memcpy(&adroit_msg.data[0], &data[ind + 5], 8);

		// send message
		RxList.push_back(adroit_msg);

		//ROS_INFO("received gateway msg");
	}

	return num;
}

/*----------------------------------------------------------------------------
  send heartbeat cmd
 *----------------------------------------------------------------------------*/
void TcpComsInterface::SendHeartbeatCmd(void) {
	ComsInterface::ComsMsg msg;

	// package message
	msg.id = TCPCOMS_HEARTBEAT_CMD;
	msg.size = 0;

	// send message
	ComsClient->SendMsg(&msg);
}

/*----------------------------------------------------------------------------
  send heartbeat res
 *----------------------------------------------------------------------------*/
void TcpComsInterface::SendHeartbeatRes(void) {
	ComsInterface::ComsMsg msg;;

	// package message
	msg.id = TCPCOMS_HEARTBEAT_RES;
	msg.size = 0;

	// send response (immediately?)
	ComsClient->SendMsg(&msg);
}

/*----------------------------------------------------------------------------
  send msg
 *----------------------------------------------------------------------------*/
int TcpComsInterface::SendMsg(AdroitMsg *adroit_msg) {
	// get mutex lock
	boost::mutex::scoped_lock lock(*TxMutex);
	
	// push msg to list
	TxList.push_back(*adroit_msg);

	// check for maximum list size?

	return 1;
}

/*----------------------------------------------------------------------------
  transmit all waiting messages
 *----------------------------------------------------------------------------*/
int TcpComsInterface::Write(void) {
	AdroitMsg adroit_msg;

	// get mutex lock
	boost::mutex::scoped_lock lock(*TxMutex);

	int num = TxList.size();
	if(num <= 0) return 0;

	//ROS_INFO("%d gateway messages", num);
	
	// setup tcp msg
	ComsInterface::ComsMsg tcp_msg;
	tcp_msg.id = TCPCOMS_GATEWAY_CMD;
	tcp_msg.size = GATEWAY_MSG_SIZE*num;

	// loop through messages
	for(int i = 0; i < num; i++) {
		int ind = i*GATEWAY_MSG_SIZE;
		
		// get first message in list
		adroit_msg = TxList.front();
		TxList.pop_front();
		
		// package as tcp packet
		tcp_msg.data[ind] = adroit_msg.id & 0xFF;
		tcp_msg.data[ind + 1] = (adroit_msg.id >> 8) & 0xFF;
		tcp_msg.data[ind + 2] = (adroit_msg.id >> 16) & 0xFF;
		tcp_msg.data[ind + 3] = (adroit_msg.id >> 24) & 0xFF;
		tcp_msg.data[ind + 4] = adroit_msg.dlc;
		memcpy(&tcp_msg.data[ind + 5], &adroit_msg.data[0], 8);
	}

	// send tcp msg
	ComsClient->SendMsg(&tcp_msg);
	//TcpComs::PrintMsg(&tcp_msg);
	//ROS_INFO("sent gateway cmd");

	return num;
}

/*----------------------------------------------------------------------------
  receive msg
 *----------------------------------------------------------------------------*/
int TcpComsInterface::ReceiveMsg(AdroitMsg *msg) {
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
void TcpComsInterface::Read() {
}

