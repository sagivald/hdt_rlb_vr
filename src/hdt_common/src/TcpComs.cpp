/*----------------------------------------------------------------------------
 * Name:    TcpComs.cpp
 * Purpose: external communication
 * Note(s):
 *----------------------------------------------------------------------------*/

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "ros/ros.h"

#include "TcpComs.h"

/*----------------------------------------------------------------------------
  extcoms init
 *----------------------------------------------------------------------------*/
//int TcpComs::Init(TcpInterface *interface, double timeout) {
int TcpComs::Init(TcpInterface *interface, Crc *crc) {
	// setup message Fifo
	MsgFifo = new FifoQ(sizeof(TcpComsMsg), TCPCOMS_MAX_BUFFERS, FifoQ::DISCARD);

	// setup crc table
	mCrc = crc;

	// initialize tcp interface
	mTcpInterface = interface;

	// buffer for parsing
	sbuf = new uint8_t[TCPCOMS_SBUF_SIZE];

	// for messaging states
	State = TCPCOMS_WAITING;
	//MsgTimer = new Timer::TimerData();
	//TMR->Set(MsgTimer, timeout);

	SentMsgs = 0;
	ReceivedMsgs = 0;
	ParsedBytes = 0;

	return 1;
}

/*----------------------------------------------------------------------------
  get msg from rx fifo
 *----------------------------------------------------------------------------*/
int TcpComs::GetMsg(TcpComsMsg *msg) {
	return MsgFifo->Get(msg);
}

/*----------------------------------------------------------------------------
  send heartbeat cmd
 *----------------------------------------------------------------------------*/
void TcpComs::SendHeartbeatCmd(void) {
	TcpComsMsg msg;

	// package message
	msg.id = TCPCOMS_HEARTBEAT_CMD;
	msg.size = 0;

	// send message
	SendMsg(&msg);
}

/*----------------------------------------------------------------------------
  send heartbeat res
 *----------------------------------------------------------------------------*/
void TcpComs::SendHeartbeatRes(void) {
	TcpComsMsg msg;

	// package message
	msg.id = TCPCOMS_HEARTBEAT_RES;
	msg.size = 0;

	// send message
	SendMsg(&msg);
}

/*----------------------------------------------------------------------------
  display message
 *----------------------------------------------------------------------------*/
void TcpComs::PrintMsg(TcpComsMsg *msg) {
	ROS_INFO("msg_id:\t%2x\n", msg->id);
	ROS_INFO("size:\t%2x\n", msg->size);
	ROS_INFO("data:\t");
	for (int i = 0; i < msg->size; i++) { ROS_INFO("%2x ", msg->data[i]); }
}


/*----------------------------------------------------------------------------
  send message
 *----------------------------------------------------------------------------*/
int TcpComs::SendMsg(TcpComsMsg *msg) {
	unsigned int msg_size = TCPCOMS_HEADER_SIZE + msg->size + TCPCOMS_CRC_SIZE;
	uint8_t buffer[msg_size];

	// populate buffer
	buffer[0] = msg->id;
	buffer[1] = msg->size & 0xFF;
	buffer[2] = (msg->size >> 8) & 0xFF;
	memcpy(&buffer[3], &msg->data[0], msg->size);

	// compute crc
	int crc_ind = TCPCOMS_HEADER_SIZE + msg->size;
	msg->crc = mCrc->crc32(&buffer[0], crc_ind);
	memcpy(&buffer[crc_ind], &msg->crc, sizeof(uint32_t));
//	ROS_DEBUG("crc = %d", msg->crc);

//	// print
//	for(int i = 0; i < msg_size; i++) {
//		ROS_DEBUG("%i\t%2x ", buffer[i]);
//	}

	mTcpInterface->SendData((char *)&buffer[0], msg_size);

	SentMsgs += 1;
	return 1;
}


/*----------------------------------------------------------------------------
  parse data
 *----------------------------------------------------------------------------*/
int TcpComs::Parse(void) {
	int num = 0;
	// check length
	int len = mTcpInterface->RxFifo->Length();
//	ROS_DEBUG("len = %d", len);

	if (len == 0) {
		return num;
	}
	else {
//		ROS_DEBUG("len = %d", len);
		// copy each data byte into buffer
		for (int i = 0; i < len; i++) {
//			char data;
			mTcpInterface->RxFifo->Read(&sbuf[i], i);
//			mTcpInterface->RxFifo->Get(&data);
//			ROS_DEBUG("%3d - %2x", i, sbuf[i]);
		}
	}

	int last_ind = 0;
	// check buffer for potential messages, must be bigger than min size
	for (int i = 0; i <= len - (TCPCOMS_HEADER_SIZE + TCPCOMS_CRC_SIZE); i++) {
		int crc_ind, size;
		uint32_t crc_val, crc_test;

		// check for valid message id
		if(!CheckMsgId(sbuf[i])) {
#ifdef DEBUG
			ROS_ERROR("invalid message id %d", sbuf[i]);
#endif
			continue;
		}

		// calculate potential message size
		size = sbuf[i + 1] + (sbuf[i + 2] << 8);

		// set crc_index for comparison, skip if it's outside of length
		crc_ind = i + TCPCOMS_HEADER_SIZE + size;
		if (crc_ind > (len - TCPCOMS_CRC_SIZE)) {
#ifdef DEBUG
			ROS_ERROR("invalid message size %d", size);
#endif
			continue;
		}

		// calculate crcs
		crc_val = mCrc->crc32(&sbuf[i], TCPCOMS_HEADER_SIZE + size);
		memcpy(&crc_test, &sbuf[crc_ind], sizeof(uint32_t));

		// if crcs match, put message in fifo
		if (crc_test == crc_val) {
			TcpComsMsg *msg = new TcpComsMsg();

			// copy message into structure
			msg->id = sbuf[i];
			msg->size = size;
			memcpy(&msg->data[0], &sbuf[i + TCPCOMS_HEADER_SIZE], size);

			// put in fifo
			int ret = MsgFifo->Put(msg);
#ifdef DEBUG
//			if(ret <= 0) ROS_ERROR("unable to put in msg fifo");
#endif

			delete msg;
			// increment num, jump i to eom
			num++;
			i = crc_ind + (TCPCOMS_CRC_SIZE - 1);
			last_ind = i;
		}
		// crcs don't match
		else {
#ifdef DEBUG
			ROS_ERROR("%d crcs don't match", i);
			ROS_ERROR("crc_ind = %d\tcrc_val = %8x\tcrc_test = %8x", crc_ind, crc_val, crc_test);
#endif
			continue;
		}
	}

	// remove used data from serial rxfifo
	for (int i = 0; i <= last_ind; i++) {
		unsigned char data;
		int get_ret = mTcpInterface->RxFifo->Get(&data);
#ifdef DEBUG
//		if(get_ret <= 0) ROS_ERROR("no data to get!");
//		ParsedBytes++;
#endif
	}

//	if(num > 0) ROS_DEBUG("%d message(s) parsed", num);
#ifdef DEBUG
	ReceivedMsgs += num;
#endif
	return num;
}
