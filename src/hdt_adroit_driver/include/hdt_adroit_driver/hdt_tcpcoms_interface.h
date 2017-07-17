#ifndef hdt_tcpcoms_interface_h
#define hdt_tcpcoms_interface_h

#include "AdroitComs.h"
#include "AdroitCrc.h"
#include "hdt_adroit_coms.h"

#include "hdt_common/ComsInterface.h"
#include "hdt_common/TcpInterface.h"
#include "hdt_common/SlipProtocol.h"

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#define GATEWAY_MSG_SIZE	13	// 4 for id, 1 for dlc, 8 for data

// CAN CanComsmunication class
class TcpComsInterface: public AdroitInterface {
//class CanComs {
public:
	enum TCPCOMS_MSG_ID {
	  	TCPCOMS_INVALID_ID	            = 0x00,
	  	TCPCOMS_HEARTBEAT_CMD			= 0x01,
	  	TCPCOMS_HEARTBEAT_RES			= 0x02,
	  	TCPCOMS_CONTROL_CMD				= 0x03,
	  	TCPCOMS_CONTROL_RES				= 0x0A,
	  	TCPCOMS_STATE_CMD				= 0x04,
	  	TCPCOMS_STATE_RES				= 0x0B,
	  	TCPCOMS_WORKSPACE_CMD			= 0x05,
	  	TCPCOMS_WORKSPACE_RES			= 0x0C,
	  	TCPCOMS_JOINT_CMD				= 0x06,
	  	TCPCOMS_JOINT_RES				= 0x0D,
	  	TCPCOMS_PARAMETER_CMD			= 0x07,
	  	TCPCOMS_PARAMETER_RES			= 0x0E,
	  	TCPCOMS_GATEWAY_CMD				= 0x08,
	  	TCPCOMS_GATEWAY_RES				= 0x0F,
	};

	TcpComsInterface(std::string ip_addr, int port);
	~TcpComsInterface();

	int Init(void);
	int SendMsg(AdroitMsg *adroit_msg);
	int ReceiveMsg(AdroitMsg *msg);
	int Write(void);
private:
	std::string IpAddr;
	int Port;

	ComsInterface *ComsClient;

	// coms functions
	void TimerCallback(const ros::TimerEvent &event);
	int Step(void);
	int ProcessGatewayRes(uint16_t size, uint8_t *data);
	void SendHeartbeatCmd(void);
	void SendHeartbeatRes(void);
	
	std::list<AdroitMsg> RxList;
	std::list<AdroitMsg> TxList;

	// thread for receiving messages
	boost::thread *ReceiveThread;

	// mutex for receiving messages
	boost::mutex *TxMutex;
	boost::mutex *RxMutex;

	// Task function
	void Read(void);

	ros::Timer step_timer;
};

#endif // hdt_tcpcoms_interface
