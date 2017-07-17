/*----------------------------------------------------------------------------
 * Name:    ComsInterface.h
 * Purpose: communication base class
 * Note(s):
 *----------------------------------------------------------------------------*/

#ifndef ComsInterface_h
#define ComsInterface_h

#include "stdint.h"

#include "BaseInterface.h"
#include "Crc.h"

#define MSG_FIFO_SIZE		PACKET_FIFO_SIZE
#define COMS_HEADER_SIZE	3
#define COMS_CRC_SIZE       4
#define COMS_DATA_SIZE		MAX_PACKET_SIZE - COMS_CRC_SIZE - COMS_HEADER_SIZE
#define COMS_MSG_SIZE		MAX_PACKET_SIZE
#define CRC_INITIAL_VALUE 	0xFFFFFFFF
#define MSG_TIMEOUT			(double)0.5

// coms interface class
class ComsInterface {
public:
	enum ComsState {
		COMS_STANDBY				= 0x00,
		COMS_CONNECTED				= 0x01,
		COMS_READY					= 0x02
	};

	enum ComsMsgId {
	  	COMS_INVALID_ID	            = 0x00,
	  	COMS_HEARTBEAT_CMD			= 0x01,
	  	COMS_HEARTBEAT_RES			= 0x02,
	};

	typedef struct ComsMsg {
		uint8_t		id;
		uint16_t	size;
		uint8_t		data[COMS_DATA_SIZE];
		uint32_t	crc;
	} ComsMsg;

	ComsInterface(BaseInterface *interface, BaseProtocol *protocol);
	~ComsInterface();

	int Init(void);
	void Process(void);
	int ProcessPacket(BaseProtocol::PacketType *packet);
	int SendMsg(ComsMsg *msg);
	int GetMsg(ComsMsg *msg);
	void PrintMsg(ComsMsg *msg);
private:
	FifoQ *MsgFifo;
	BaseInterface *Interface;
	BaseProtocol *Protocol;
	Crc *crc;
	ComsState State;
};

#endif // ComsInterface_h
