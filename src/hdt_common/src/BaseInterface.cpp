/*----------------------------------------------------------------------------
 * Name:    BaseInterface.cpp
 * Purpose: base interface class
 * Note(s):
 *----------------------------------------------------------------------------*/

#include "BaseInterface.h"

/*----------------------------------------------------------------------------
  constructor
 *----------------------------------------------------------------------------*/
BaseInterface::BaseInterface() {
}


/*----------------------------------------------------------------------------
  destructor
 *----------------------------------------------------------------------------*/
BaseInterface::~BaseInterface() {
	delete(Protocol);
}


/*----------------------------------------------------------------------------
  init
 *----------------------------------------------------------------------------*/
int BaseInterface::Init(BaseProtocol *protocol) {
	// initialize protocol class
	Protocol = protocol;
	if(Protocol->Init(this) <= 0) {
		return -1;
	}

	// start interface
	if(Start() <= 0) {
		return -1;
	}

	return 1;	
}

/*----------------------------------------------------------------------------
  send packet
 *----------------------------------------------------------------------------
int BaseInterface::SendPacket(BaseProtocol::PacketType *packet) {
	return Protocol->SendPacket(packet);
}*/

/*----------------------------------------------------------------------------
  get packet
 *----------------------------------------------------------------------------
int BaseInterface::GetPacket(BaseProtocol::PacketType *packet) {
	return Protocol->GetPacket(packet);
}*/


