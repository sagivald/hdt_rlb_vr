#ifndef hdt_pcan_interface_h
#define hdt_pcan_interface_h

#include "AdroitComs.h"
#include "hdt_adroit_coms.h"

#include <libpcan.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

// CAN CanComsmunication class
class PcanInterface: public AdroitInterface {
//class CanComs {
public:
	PcanInterface(std::string baud_rate);
	~PcanInterface();

	int Init(void);
	int SendMsg(AdroitMsg *adroit_msg);
	int ReceiveMsg(AdroitMsg *msg);
	int Write(void);
private:
	HANDLE CanHandle;
	int BaudRate;

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

#endif // hdt_pcan_interface
