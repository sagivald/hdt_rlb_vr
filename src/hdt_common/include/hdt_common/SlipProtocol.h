/*----------------------------------------------------------------------------
 * Name:    SlipProtocol.h
 * Purpose: slip protocol class
 * Note(s):
 *----------------------------------------------------------------------------*/

#ifndef SlipProtocol_h
#define SlipProtocol_h

#include "BaseProtocol.h"

// SLIP special character codes
#define END			(char)0300    // indicates end of packet
#define ESC			(char)0333    // indicates byte stuffing
#define ESC_END		(char)0334    // ESC ESC_END means END data byte
#define ESC_ESC		(char)0335    // ESC ESC_ESC means ESC data byte

// base interface class
class SlipProtocol : public BaseProtocol {
public:
	SlipProtocol() {};
	~SlipProtocol() {};

	typedef enum SlipState {
		STATE_NORMAL =	0,
		STATE_ESC =		1
	} SlipState;
	
	// virtual functions
	int Start(void);
	int SendPacket(PacketType *packet);
	int GetPacket(PacketType *packet);
	void ProcessByte(char data);
private:
	SlipState State;
	PacketType PacketBuffer;

	int PutByte(char data);
	void ResetBuffer(void) {PacketBuffer.len = 0; State = STATE_NORMAL;};
};

#endif // SlipProtocol_h
