/*----------------------------------------------------------------------------
 * Name:    BaseProtocol.cpp
 * Purpose: base protocol class
 * Note(s):
 *----------------------------------------------------------------------------*/

#include <stdio.h>

#include "BaseProtocol.h"
#include "BaseInterface.h"

/*----------------------------------------------------------------------------
  init
 *----------------------------------------------------------------------------*/
int BaseProtocol::Init(BaseInterface *interface) {
	// copy interface pointer
	Interface = interface;

	return Start();
}

/*----------------------------------------------------------------------------
  display packet
 *----------------------------------------------------------------------------*/
void BaseProtocol::PrintPacket(PacketType *packet) {
	printf("len:\t%3d\n", packet->len);
	for (int i = 0; i < packet->len; i++) { printf("%2d ", packet->data[i]); }
	printf("\n");
}
