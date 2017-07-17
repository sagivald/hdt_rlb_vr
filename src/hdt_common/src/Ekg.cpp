#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <errno.h>

#include <signal.h>

#include "Ekg.h"
#include "FifoQ.h"

#define DEFAULT_PORT 3000    // TCP/IP port
#define SAMPLE_RATE_DIVISOR	1

Ekg::Ekg() :
	AperiodicTask() {
	mNumSignals = 0;
	mInitialized = 0;
	divisorCount = 0;
}

Ekg::~Ekg() {

}

void Ekg::AddSignal(char *prefix, char *name, double *val) {

	if (!mInitialized && (mNumSignals <= MAX_NUM_SIGNALS)) {
		if (strcmp(prefix, "") != 0) {
			sprintf(label, "%s_%s", prefix, name);
		}
		else {
			sprintf(label, "%s", name);
		}

		if (strlen(label) >= MAX_NAME_LENGTH) {
			strncpy(mNameArr[mNumSignals], label, MAX_NAME_LENGTH);
		} else {
			strcpy(mNameArr[mNumSignals], label);
		}

		// record pointer in array
		mpValPtrArr[mNumSignals].p.dValPtr = val;
		mpValPtrArr[mNumSignals].t = Ekg::DOUBLE;

		mNumSignals++;
	}
}

void Ekg::AddSignal(char *prefix, char *name, int *val) {

	if (!mInitialized && (mNumSignals <= MAX_NUM_SIGNALS)) {
		if (strcmp(prefix, "") != 0) {
			sprintf(label, "%s_%s", prefix, name);
		}

		if (strlen(label) >= MAX_NAME_LENGTH) {
			strncpy(mNameArr[mNumSignals], label, MAX_NAME_LENGTH);
		} else {
			strcpy(mNameArr[mNumSignals], label);
		}

		// record pointer in array
		mpValPtrArr[mNumSignals].p.iValPtr = val;
		mpValPtrArr[mNumSignals].t = Ekg::INTEGER;

		mNumSignals++;
	}
}

void Ekg::AddSignal(char *prefix, char *name, float *val) {

	if (!mInitialized && (mNumSignals <= MAX_NUM_SIGNALS)) {
		if (strcmp(prefix, "") != 0) {
			sprintf(label, "%s_%s", prefix, name);
		}
		else {
			sprintf(label, "%s", name);
		}

		if (strlen(label) >= MAX_NAME_LENGTH) {
			strncpy(mNameArr[mNumSignals], label, MAX_NAME_LENGTH);
		} else {
			strcpy(mNameArr[mNumSignals], label);
		}

		// record pointer in array
		mpValPtrArr[mNumSignals].p.fValPtr = val;
		mpValPtrArr[mNumSignals].t = Ekg::FLOAT;

		mNumSignals++;
	}
}

void Ekg::AddSignal(char *prefix, char *name, int16_t *val){

	if(!mInitialized && (mNumSignals <= MAX_NUM_SIGNALS)){
		sprintf(label, "%s%s", prefix, name);

		if(strlen(label) >= MAX_NAME_LENGTH){
			strncpy(mNameArr[mNumSignals], label, MAX_NAME_LENGTH);
		}
		else{
			strcpy(mNameArr[mNumSignals], label);
		}

		// record pointer in array
		mpValPtrArr[mNumSignals].p.i16ValPtr = val;
		mpValPtrArr[mNumSignals].t = Ekg::INT16;

		mNumSignals++;
	}
}

int Ekg::TcpIpInit() {
	mSocket = socket(AF_INET, SOCK_STREAM, 0);
	if (mSocket < 0) {
		printf("Ekg:tcpIpInit: error opening socket\n");
		return -1;
	}

	int opt = 1;
	if (setsockopt(mSocket, SOL_SOCKET, SO_REUSEADDR, (char *) &opt, sizeof(int)) < 0) {
		printf("Ekg:tcpIpInit: error setting socket option for SO_REUSEADDR using SOL_SOCKET\n");
		return -1;
	}

	mServerAddr.sin_family = AF_INET;
	mServerAddr.sin_addr.s_addr = INADDR_ANY;
	mServerAddr.sin_port = htons(mPort);
	memset(&(mServerAddr.sin_zero), '\0', 8);

	if (bind(mSocket, (struct sockaddr *) &mServerAddr, sizeof(struct sockaddr)) < 0) {
		printf("Ekg:tcpIpInit: error binding socket: %s\n", strerror(errno));
		return -1;
	}

	if (listen(mSocket, 2) == -1) {
		printf("Ekg:tcpIpInit: listen failed\n");
		return -1;
	}

	// register an empty signal handler for SIGPIPE
	// to prevent exiting upon client disconnect
//	struct sigaction act;
//
//	act.sa_handler = &sig_handler;
//	act.sa_flags = 0;
//	sigaction(SIGPIPE, &act, NULL);

	signal(SIGPIPE, SIG_IGN);

	return 1;
}

int Ekg::Init(double rate, int priority) {
	return Init(rate, priority, DEFAULT_PORT);
}

int Ekg::Init(double rate, int priority, int port) {
	mInitialized = 1;
	mSampleRate = rate;
	mPort = port;

	mpValBuf = new double[mNumSignals];
	mpFifo = new FifoQ(sizeof(double) * mNumSignals, 3, FifoQ::OVERWRITE);

	// Initialize TCP/IP
	if (TcpIpInit() == -1) {
		return -1;
	}

	// Start the thread
	AperiodicTask::Init((char *) "Ekg Task", priority);

	return 1;
}

void Ekg::Process() {
	// check to see if connection has been established
	if (validSession && mInitialized) {
		divisorCount++;
		if (divisorCount == SAMPLE_RATE_DIVISOR) {
			divisorCount = 0;
			// copy values into buffer
			for (int i = 0; i < mNumSignals; i++) {
				if (mpValPtrArr[i].t == Ekg::DOUBLE)
					mpValBuf[i] = *mpValPtrArr[i].p.dValPtr;
				else if (mpValPtrArr[i].t == Ekg::INTEGER) {
					mpValBuf[i] = (double)(*mpValPtrArr[i].p.iValPtr);
				}
				else if (mpValPtrArr[i].t == Ekg::FLOAT) {
					mpValBuf[i] = (double)(*mpValPtrArr[i].p.fValPtr);
				}
				else if(mpValPtrArr[i].t == Ekg::INT16){
					mpValBuf[i] = (double)(*mpValPtrArr[i].p.i16ValPtr);
				}
			}

			// put buffer into thread communication fifo
			mpFifo->Put(mpValBuf);

			// trigger the server to read buffer and send data
			Trigger(0);
		}
	}
}

void Ekg::Task() {
	int bytes;

	while (1) {
		validSession = 0;
		mSessionSocket = accept(mSocket, 0, 0);
		if (mSessionSocket == -1) {
			printf("Ekg:Task: error accepting socket\n");
			continue;
		} else {
			int opt = 1;
			if (setsockopt(mSessionSocket, IPPROTO_TCP, TCP_NODELAY, (char *) &opt, sizeof(int)) < 0) {
				printf("Ekg:Task: Error setting socket option for TCP_NODELAY using IPPROTO\n");
				close(mSessionSocket);
				continue;
			}
		}

		// send labels
		bytes = MarshallLabels();
		if (send(mSessionSocket, mBuf, bytes, 0) == -1) {
			printf("Ekg:Task: Unable to send\n");
			close(mSessionSocket);
			continue;
		}

		// send sample rate
		bytes = MarshallSampleRate();
		if (send(mSessionSocket, mBuf, bytes, 0) == -1) {
			printf("Ekg:Task: Unable to send\n");
			close(mSessionSocket);
			continue;
		}

		mpFifo->Reset();

		validSession = 1;

		while (1) {
			// wait for trigger to send signals
			AperiodicTask::TriggerWait();

			// pull signals from fifo
			mpFifo->Get(&mBuf[4]);

			// send
			bytes = MarshallSignals();

			if (send(mSessionSocket, mBuf, bytes, 0) == -1) {
				printf("Ekg:Task: Unable to send\n");
				close(mSessionSocket);
				break;
			}
		}
	}
}

int Ekg::MarshallSignals() {

	mBuf[0] = 0x55;
	mBuf[1] = 1;
	mBuf[2] = (mNumSignals & 0xFF);
	mBuf[3] = ((mNumSignals >> 8) & 0xFF);

	return (mNumSignals * 8 + 4); // account for header bytes

}

int Ekg::MarshallSampleRate() {
	//	int i;
	unsigned char *b;
	double actualRate;

	mBuf[0] = 0x55;
	mBuf[1] = 3;
	mBuf[2] = 1;
	mBuf[3] = 0;

	b = (mBuf + 4);

	actualRate = mSampleRate / SAMPLE_RATE_DIVISOR;

	for (int i = 0; i < 8; i++) {
		b[i] = *((unsigned char *) &(actualRate) + i);
	}

	return (8 + 4); // account for header bytes
}

int Ekg::MarshallLabels() {
	//	unsigned char *b;
	int totalLength = 0;

	mBuf[0] = 0x55;
	mBuf[1] = 2;

	// buf[2] and buf[3] - total number of bytes will be filled in at the end
	totalLength = 4;

	for (int i = 0; i < mNumSignals; i++) {

		memcpy(mBuf + totalLength, mNameArr[i], strlen(mNameArr[i]));
		totalLength += strlen(mNameArr[i]);

		mBuf[totalLength] = ':';
		totalLength += 1;
	}

	totalLength -= 4; // get rid of header
	mBuf[2] = totalLength & 0xFF;
	mBuf[3] = (totalLength >> 8) & 0xFF;

	return (totalLength + 4); // account for header bytes
}


