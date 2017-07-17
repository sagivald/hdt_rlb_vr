/*----------------------------------------------------------------------------
 * Name:    ComsInterface.cpp
 * Purpose: communication base class
 * Note(s):
 *----------------------------------------------------------------------------*/

#include <string.h>
#include <stdio.h>

#include "ComsInterface.h"

/*----------------------------------------------------------------------------
  constructor
 *----------------------------------------------------------------------------*/
ComsInterface::ComsInterface(BaseInterface *interface, BaseProtocol *protocol) {
	Interface = interface;
	Protocol = protocol;
}


/*----------------------------------------------------------------------------
  destructor
 *----------------------------------------------------------------------------*/
ComsInterface::~ComsInterface() {
	delete(Interface);
	delete(Protocol);
}

/*----------------------------------------------------------------------------
  init
 *----------------------------------------------------------------------------*/
int ComsInterface::Init(void) {
	// setup message Fifo
	MsgFifo = new FifoQ(sizeof(ComsMsg), MSG_FIFO_SIZE, FifoQ::DISCARD);

	// init crc
	crc = new Crc();

	// set state
	State = COMS_STANDBY;

	// init interface and protocol
	return 	Interface->Init(Protocol);
}

/*----------------------------------------------------------------------------
  get msg
 *----------------------------------------------------------------------------*/
int ComsInterface::SendMsg(ComsMsg *msg) {
	BaseProtocol::PacketType packet;

	// check size
	if(msg->size > COMS_DATA_SIZE) {
		printf("invalid size %d\n", msg->size);
		return -1;
	}

	// populate packet
	packet.len = COMS_HEADER_SIZE + msg->size + COMS_CRC_SIZE;
	packet.data[0] = msg->id;
	packet.data[1] = msg->size & 0xFF;
	packet.data[2] = (msg->size >> 8) & 0xFF;
	memcpy(&packet.data[3], &msg->data[0], msg->size);

	// compute crc
	uint16_t crc_ind = COMS_HEADER_SIZE + msg->size;
	msg->crc = crc->crc32((uint8_t *)&packet.data[0], crc_ind);
	memcpy(&packet.data[crc_ind], &msg->crc, sizeof(uint32_t));

	// send packet
	Protocol->SendPacket(&packet);

	return 1;
}

/*----------------------------------------------------------------------------
  get msg
 *----------------------------------------------------------------------------*/
int ComsInterface::GetMsg(ComsMsg *msg) {
	return MsgFifo->Get(msg);
}

/*----------------------------------------------------------------------------
  display message
 *----------------------------------------------------------------------------*/
void ComsInterface::PrintMsg(ComsMsg *msg) {
	printf("id:\t%2x\n", msg->id);
	printf("size:\t%2x\n", msg->size);
	printf("data:\t");
	for (int i = 0; i < msg->size; i++) { printf("%2x ", msg->data[i]); }
	printf("\n");
}

/*----------------------------------------------------------------------------
  process packets
 *----------------------------------------------------------------------------*/
int ComsInterface::ProcessPacket(BaseProtocol::PacketType *packet) {
	uint16_t size;
	//Protocol->PrintPacket(packet);

	// verify size
	size = (unsigned char)packet->data[1] | ((unsigned char)packet->data[2] << 8);
	if(size != (packet->len - COMS_HEADER_SIZE - COMS_CRC_SIZE)) {
		printf("invalid size %d\n", size);
		Protocol->PrintPacket(packet);
		return -1;
	}
	else {
		uint32_t crc_val, crc_test;
		
		// compute crc
		uint16_t crc_ind = packet->len - COMS_CRC_SIZE;
		crc_val = crc->crc32((uint8_t *)&packet->data[0], crc_ind);
		memcpy(&crc_test, &packet->data[crc_ind], sizeof(uint32_t));

		// verify crc
		if(crc_val != crc_test) {
			printf("invalid crc\n");
			return -1;
		}
		else {
			ComsMsg msg;
		
			// copy message into structure
			msg.id = packet->data[0];
			msg.size = size;
			memcpy(&msg.data[0], &packet->data[COMS_HEADER_SIZE], size);

			// add to fifo
			return MsgFifo->Put(&msg);
		}
	}
}

/*----------------------------------------------------------------------------
  process
 *----------------------------------------------------------------------------*/
void ComsInterface::Process(void) {
	BaseProtocol::PacketType packet;

	// get available packets
	while(Protocol->GetPacket(&packet) > 0) {
		ProcessPacket(&packet);
	}
}

