/*----------------------------------------------------------------------------
 * Name:    BaseProtocol.h
 * Purpose: base protocol class
 * Note(s):
 *----------------------------------------------------------------------------*/

#ifndef BaseProtocol_h
#define BaseProtocol_h

#include "FifoQ.h"

#define MAX_PACKET_SIZE		512
#define PACKET_FIFO_SIZE	8

class BaseInterface;

// base interface class
class BaseProtocol {
public:
	BaseProtocol() {};
	~BaseProtocol() {};
	
	// packet type
	typedef struct PacketType {
		char data[MAX_PACKET_SIZE];
		int len;
	} PacketType;

	int Init(BaseInterface *interface);
	void PrintPacket(PacketType *packet);
	
	// virtual functions
	virtual int Start(void) = 0;
	virtual int SendPacket(PacketType *packet) = 0;
	virtual int GetPacket(PacketType *packet) = 0;
	virtual void ProcessByte(char data) = 0;
protected:
	FifoQ *PacketFifo;
	BaseInterface *Interface;
};

#endif // BaseProtocol_h
