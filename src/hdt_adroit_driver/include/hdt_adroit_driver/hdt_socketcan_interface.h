#ifndef hdt_socketcan_interface
#define hdt_socketcan_interface

#include "AdroitComs.h"
#include "hdt_adroit_coms.h"

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <linux/can.h>

// CAN CanComsmunication class
class SocketcanInterface: public AdroitInterface {
//class CanComs {
public:
	SocketcanInterface(std::string device);
	~SocketcanInterface();

	int Init(void);
	int SendMsg(AdroitMsg *adroit_msg);
	int ReceiveMsg(AdroitMsg *msg);
	int Write(void);
private:
	std::string device;
	int s;
	
	std::list<AdroitMsg> RxList;
	std::list<AdroitMsg> TxList;
	
	// thread for receiving messages
	boost::thread *ReadThread;
	
	// mutex for receiving messages
	boost::mutex *TxMutex;
	boost::mutex *RxMutex;
	
	// Task function
	void Read(void);
};

#endif // hdt_socketcan_interface
