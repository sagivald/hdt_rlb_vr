/*----------------------------------------------------------------------------
 * Name:    TcpComs.h
 * Purpose: tcp communication base class
 * Note(s):
 *----------------------------------------------------------------------------*/

#ifndef TcpComs_h
#define TcpComs_h

#define TCPCOMS_CRC_SIZE         	4
#define TCPCOMS_HEADER_SIZE			3
#define TCPCOMS_MAX_DATA_SIZE		512
#define TCPCOMS_MAX_MSG_SIZE		TCPCOMS_HEADER_SIZE + TCPCOMS_MAX_DATA_SIZE
#define TCPCOMS_CRC_INITIAL_VALUE 	0xFFFFFFFF
#define TCPCOMS_SBUF_SIZE			TCPCOMS_MAX_DATA_SIZE
#define TCPCOMS_MAX_BUFFERS			256
#define TCPCOMS_MSG_TIMEOUT			(double)0.5
#define DEBUG

#include "stdint.h"
#include "TcpInterface.h"
#include "FifoQ.h"
//#include "Timer.h"
#include "Crc.h"

class TcpComs;

class TcpComs {
public:
	enum TCPCOMS_STATE {
		TCPCOMS_WAITING					= 0x00,
		TCPCOMS_CONNECTED				= 0x01,
		TCPCOMS_READY					= 0x02
	};
	enum TCPCOMS_MSG_ID{
	  	TCPCOMS_INVALID_ID	            = 0x00,
	  	TCPCOMS_HEARTBEAT_CMD			= 0x01,
	  	TCPCOMS_HEARTBEAT_RES			= 0x02,
	};

	typedef struct TcpComsMsg {
		uint8_t		id;
		uint16_t	size;
		uint8_t		data[TCPCOMS_MAX_DATA_SIZE];
		uint32_t	crc;
	} TcpComsMsg;

	TcpComs() {};
	~TcpComs() {};

	//int Init(TcpInterface *interface, double timeout) ;
	int Init(TcpInterface *interface, Crc *crc) ;
	int Parse(void);
	int SendMsg(TcpComsMsg *msg);
	int GetMsg(TcpComsMsg *msg);
	void PrintMsg(TcpComsMsg *msg);
	int GetState(void) { return (int)State; };
	void SetState(int state) { State = state; };
	int GetSentMsgs(void) { return SentMsgs; };
	int GetReceivedMsgs(void) { return ReceivedMsgs; };

	virtual int Step(void) = 0;
protected:
	TcpInterface *mTcpInterface;
	FifoQ *MsgFifo;
	int State;

	virtual int CheckMsgId(uint8_t id) = 0;

	// timer and state for messaging
	//Timer::TimerData *MsgTimer;

	void SendHeartbeatCmd(void);
	void SendHeartbeatRes(void);

	int SentMsgs, ReceivedMsgs, ParsedBytes;

private:
	char *mName;
	uint8_t *sbuf;

	Crc *mCrc;
};

#endif // TcpComs_h

