#include <signal.h>

#include "ros/ros.h"

#include "hdt_common_test.h"
#include "Ekg.h"
#include "Timer.h"

#include "ComsInterface.h"
#include "TcpInterface.h"
//#include "SerialInterface.h"
#include "SlipProtocol.h"
	
#define ROS_NAME			"hdt_common_test"
#define SAMPLE_RATE			100.0
#define SAMPLE_PERIOD 		1.0/SAMPLE_RATE

// global pointers
AperiodicTest *AT;
PeriodicTest *PT;
Ekg *EKG;
Timer *TMR;
ComsInterface *ComsClient, *ComsServer;
//ComsInterface *SerialComs;

unsigned int msgs_sent, msgs_received;

/*----------------------------------------------------------------------------
  timer callback
 *----------------------------------------------------------------------------*/
void TimerCallback(const ros::TimerEvent &event) {
	//ROS_INFO("TimerCallback");
	printf("sent = %d, received = %d\n", msgs_sent, msgs_received);
}

/*----------------------------------------------------------------------------
  main function
 *----------------------------------------------------------------------------*/
int main(int argc, char **argv) {
	ros::init(argc, argv, ROS_NAME);
	ros::NodeHandle n;
	int ret;

	ros::NodeHandle private_node_handle("~");
	int baud;
	std::string device;
	private_node_handle.param<int>("baud", baud, 9600);
	private_node_handle.param<std::string>("device", device, "/dev/ttyUSB0");


	ROS_INFO("Starting %s", ROS_NAME);

	printf("END = %d\n", END);
	printf("ESC = %d\n", ESC);
	printf("ESC_END = %d\n", ESC_END);
	printf("ESC_ESC = %d\n", ESC_ESC);

	// initialize EKG
	//EKG = new Ekg();

	// Setup Timer
	TMR = new Timer();
	TMR->Init(SAMPLE_RATE);

	// coms init
	ComsServer = new ComsInterface(new TcpServer((char *)"ComsServer", 8765, 0), new SlipProtocol());
	ComsServer->Init();

	ComsClient = new ComsInterface(new TcpClient((char *)"ComsClient", (char *)"127.0.0.1", 8765, 0), new SlipProtocol());
	ComsClient->Init();

	//SerialComs = new ComsInterface(new SerialInterface((char *)"SerialComs", (char *)device.c_str(), baud, 0), new SlipProtocol());
	//if(SerialComs->Init() <= 0) {
	//	ROS_ERROR("SerialComs:Init failed");
	//	return -1;
	//}

	// initialize AperiodicTest
	AT = new AperiodicTest();
	ret = AT->Init((char *)"AperiodicTest", 0);
	if(ret <= 0) ROS_ERROR("AT:Init failed, ret = %d", ret);

	// initialize PeriodicTest
	PT = new PeriodicTest();
	ret = PT->Init((char *)"PeriodicTest", SAMPLE_RATE, 0);
	if(ret <= 0) ROS_ERROR("PT:Init failed, ret = %d", ret);

	// start up EKG
	//EKG->Init(SAMPLE_RATE, 0);

	// create sample loop timer
	ros::Timer timer = n.createTimer(ros::Duration(1.0), TimerCallback);

	ros::spin();
	
	return 0;
}

/*----------------------------------------------------------------------------
  AperiodicTest constructor
 *----------------------------------------------------------------------------*/
AperiodicTest::AperiodicTest() {
}

/*----------------------------------------------------------------------------
  AperiodicTest destructor
 *----------------------------------------------------------------------------*/
AperiodicTest::~AperiodicTest() {
}

/*----------------------------------------------------------------------------
  AperiodicTest init
 *----------------------------------------------------------------------------*/
int AperiodicTest::Init(char *name, int priority) {
	return AperiodicTask::Init(name, priority);
}

/*----------------------------------------------------------------------------
  AperiodicTest task
 *----------------------------------------------------------------------------*/
void AperiodicTest::Task(void) {
	// wait for trigger
	while(1) {
		TriggerWait();
		
		//ROS_INFO("AperiodicTest::Task");
	}
}

/*----------------------------------------------------------------------------
  PeriodicTest constructor
 *----------------------------------------------------------------------------*/
PeriodicTest::PeriodicTest() {
}

/*----------------------------------------------------------------------------
  PeriodicTest destructor
 *----------------------------------------------------------------------------*/
PeriodicTest::~PeriodicTest() {
}

/*----------------------------------------------------------------------------
  PeriodicTest init
 *----------------------------------------------------------------------------*/
int PeriodicTest::Init(char *name, double rate, int priority) {
	double actual_rate;
	
	return PeriodicTask::Init(name, rate, &actual_rate, priority);
}

/*----------------------------------------------------------------------------
  PeriodicTest task
 *----------------------------------------------------------------------------*/
void PeriodicTest::Task(void) {
	ComsInterface::ComsMsg msg;
	msg.id = 1;
	msg.size = 9;
	msg.data[0] = 1;
	msg.data[1] = -64;
	msg.data[2] = 1;
	msg.data[3] = -37;
	msg.data[4] = 1;
	msg.data[5] = -36;
	msg.data[6] = 1;
	msg.data[7] = -35;
	msg.data[8] = 1;

	// wait for timer
	while(1) {
		TimerWait();

		// Timer step
		TMR->Process();

		// process EKG
		//EKG->Process();
		
		//ROS_INFO("PeriodicTest::Task");
		
		// send test message
		ComsClient->SendMsg(&msg);
		msgs_sent++;

		ComsServer->SendMsg(&msg);
		msgs_sent++;

		// process coms
		ComsServer->Process();
		while(ComsServer->GetMsg(&msg) > 0) {
			//ComsServer->PrintMsg(&msg);
			msgs_received++;
		}
		ComsClient->Process();
		while(ComsClient->GetMsg(&msg) > 0) {
			//ComsClient->PrintMsg(&msg);
			msgs_received++;
		}

		// send test message
		//SerialComs->SendMsg(&msg);
		//msgs_sent++;

		// process coms
		//SerialComs->Process();
		//while(SerialComs->GetMsg(&msg) > 0) {
		//	SerialComs->PrintMsg(&msg);
		//	msgs_received++;
		//}

		// trigger AperiodicTest task	
		AT->Trigger(0);
	}
}




