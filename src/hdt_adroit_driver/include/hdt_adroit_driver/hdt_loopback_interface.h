#ifndef hdt_loopback_interface_h
#define hdt_loopback_interface_h

#include "AdroitComs.h"
#include "hdt_adroit_coms.h"

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

// CAN CanComsmunication class
class LoopbackInterface: public AdroitInterface {
//class CanComs {
public:
	LoopbackInterface();
	~LoopbackInterface();

	int Init(void);
	int SendMsg(AdroitMsg *adroit_msg);
	int ReceiveMsg(AdroitMsg *msg);
	int Write(void);
private:
	std::list<AdroitMsg> RxList;
	std::list<AdroitMsg> TxList;

	// thread for receiving messages
	boost::thread *ReceiveThread;

	// mutex for receiving messages
	boost::mutex *TxMutex;
	boost::mutex *RxMutex;

	// Task function
	void Read(void);
};

#endif // hdt_loopback_interface
