#include "ros/ros.h"

#include "hdt_loopback_interface.h"

/*----------------------------------------------------------------------------
  constructor
 *----------------------------------------------------------------------------*/
LoopbackInterface::LoopbackInterface() : AdroitInterface()  {
}

/*----------------------------------------------------------------------------
  destructor
 *----------------------------------------------------------------------------*/
LoopbackInterface::~LoopbackInterface() {
}

/*----------------------------------------------------------------------------
  init
 *----------------------------------------------------------------------------*/
int LoopbackInterface::Init(void) {
	// Start the receive thread
	TxMutex = new boost::mutex();
	RxMutex = new boost::mutex();
	ReceiveThread = new boost::thread(boost::bind(&LoopbackInterface::Read, this));

	return 1;
}

/*----------------------------------------------------------------------------
  send msg
 *----------------------------------------------------------------------------*/
int LoopbackInterface::SendMsg(AdroitMsg *adroit_msg) {
	// get mutex lock
	boost::mutex::scoped_lock lock(*TxMutex);
	
	// push msg to list
	TxList.push_back(*adroit_msg);

	return 1;
}

/*----------------------------------------------------------------------------
  transmit all waiting messages
 *----------------------------------------------------------------------------*/
int LoopbackInterface::Write(void) {
	AdroitMsg adroit_msg;

	// get mutex lock
	boost::mutex::scoped_lock lock(*TxMutex);

	// clear tx list for now
	TxList.clear();

	return 1;
}

/*----------------------------------------------------------------------------
  receive msg
 *----------------------------------------------------------------------------*/
int LoopbackInterface::ReceiveMsg(AdroitMsg *msg) {
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
void LoopbackInterface::Read() {
}

