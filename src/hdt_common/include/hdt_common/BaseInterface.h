/*----------------------------------------------------------------------------
 * Name:    BaseInterface.h
 * Purpose: base interface class
 * Note(s):
 *----------------------------------------------------------------------------*/

#ifndef BaseInterface_h
#define BaseInterface_h

#include "FifoQ.h"
#include "BaseProtocol.h"

#define TX_FIFO_SIZE	4096

// base interface class
class BaseInterface {
public:
	BaseInterface();
	~BaseInterface();
	
	int Init(BaseProtocol *protocol);	
	//int SendPacket(BaseProtocol::PacketType *packet);
	//int GetPacket(BaseProtocol::PacketType *packet);

	// virtual functions
	virtual int Start(void) = 0;
	virtual int SendByte(char data) = 0;
	//virtual int ReadByte(char &data) = 0;
	virtual int IsConnected(void) = 0;
	virtual int Disconnect(void) = 0;
protected:
	BaseProtocol *Protocol;
};

#endif // BaseInterface_h
