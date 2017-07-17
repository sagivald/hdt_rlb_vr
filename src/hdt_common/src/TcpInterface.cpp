/*----------------------------------------------------------------------------
 * Name:    TcpInterface.cpp
 * Purpose: tcp interface class
 * Note(s):
 *----------------------------------------------------------------------------*/

#include <termios.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <errno.h>
#include <signal.h>

#include "TcpInterface.h"

/*----------------------------------------------------------------------------
  constructor
 *----------------------------------------------------------------------------*/
TcpInterface::TcpInterface() : BaseInterface(), AperiodicTask() {
}

/*----------------------------------------------------------------------------
  init
 *----------------------------------------------------------------------------*/
int TcpInterface::Start(void) {
	int ret = 1;

	// set variables
	sfd = INVALID_SFD;

	// initialize fifo
	RxFifo = new FifoQ(sizeof(char), MAX_PACKET_SIZE, FifoQ::DISCARD);

	// Initialize tcp
	addr = new struct sockaddr_in();
	if (TcpInit() <= 0) {
		printf("error initializing tcp\n");
		return -1;
	}

	// initialize associated Tcpip writer
	Writer = new TcpWriter(Name, Priority);
	ret = Writer->Start();

	AperiodicTask::Init(Name, Priority);

	return ret;
}

/*----------------------------------------------------------------------------
  check connection state
 *----------------------------------------------------------------------------*/
int TcpInterface::IsConnected(void) {
	if(sfd < 0) return 0;
	else return 1;
}

/*----------------------------------------------------------------------------
  disconnect socket
 *----------------------------------------------------------------------------*/
int TcpInterface::Disconnect(void) {
	if(IsConnected()) {
		close(sfd);
		sfd = INVALID_SFD;
		Writer->SetSocket(sfd);

		return 1;
	}

	return 0;
}

/*----------------------------------------------------------------------------
  send data
 *----------------------------------------------------------------------------*/
int TcpInterface::SendByte(char data) {
	return Writer->SendByte(data);
}

/*----------------------------------------------------------------------------
  read data
 *----------------------------------------------------------------------------*/
int TcpInterface::ReadByte(char &data) {
	return RxFifo->Get(&data);
}

/*----------------------------------------------------------------------------
  constructor
 *----------------------------------------------------------------------------*/
TcpClient::TcpClient(char *name, char *server, int port, int priority) : TcpInterface() {
	Name = name;
	Server = server;
	Port = port;
	Priority = priority;
}

/*----------------------------------------------------------------------------
  connection and read task
 *----------------------------------------------------------------------------*/
void TcpClient::Task() {
	char data[MAX_PACKET_SIZE];

	// loop trying to connect to server
	while(1) {
		// create socket
		sfd = socket(AF_INET, SOCK_STREAM, 0);
		if (sfd < 0) {
			printf("error opening socket\n");
			break;
		}

		// set socket options
		int opt = 1;
		if (setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR, (char *) &opt, sizeof(int)) < 0) {
			printf("error setting socket option for SO_REUSEADDR using SOL_SOCKET\n");
			Disconnect();
			break;
		}
		if (setsockopt(sfd, IPPROTO_TCP, TCP_NODELAY, (char *) &opt, sizeof(int)) < 0) {
			printf("error setting socket option for SO_REUSEADDR using SOL_SOCKET\n");
			Disconnect();
			break;
		}

		// set server address
		memset(addr, 0x00, sizeof(struct sockaddr_in));
		addr->sin_family = AF_INET;
		addr->sin_addr.s_addr = inet_addr(Server);
		addr->sin_port = htons(Port);

		printf("attempting socket connection\n");
		printf("server = %s, port = %d\n", Server, Port);

		// connect to server
		if((connect(sfd, (struct sockaddr*)addr, sizeof(struct sockaddr_in))) < 0) {
			printf("can't connect to server\n");
			Disconnect();
			
			// wait one second before trying to connect again
			usleep(1000*1000);
			continue;
	   }

		// set output socket
		Writer->SetSocket(sfd);
		printf("socket opened %d\n", (int)sfd);

		while (1) {
			int ret;

			// block on read, put data in buffer
			ret = read(sfd, &data, MAX_PACKET_SIZE);

			// put all data into fifo			
			if(ret > 0) {
				for(int i = 0; i < ret; i++) {
					//int rx_ret = RxFifo->Put(&data[i]);
					BaseInterface::Protocol->ProcessByte(data[i]);
				}
			}
			// read error, close socket
			else {
				Disconnect();
				printf("socket closed\n");
				break;
			}
		}
	}
}

/*----------------------------------------------------------------------------
  socket settings
 *----------------------------------------------------------------------------*/
int TcpClient::TcpInit(void) {
	return 1;
}


/*----------------------------------------------------------------------------
  constructor
 *----------------------------------------------------------------------------*/
TcpServer::TcpServer(char *name, int port, int priority) : TcpInterface() {
	Name = name;
	Port = port;
	Priority = priority;
}

/*----------------------------------------------------------------------------
  connection and read task
 *----------------------------------------------------------------------------*/
void TcpServer::Task() {
	char data[MAX_PACKET_SIZE];

	// loop trying to connect to client
	while(1) {
		printf("waiting for socket connection\n");
		printf("port = %d\n", Port);
		
		// accept client connection
		sfd = accept(server_sfd, 0, 0);
		if (sfd < 0) {
			printf("Error accepting socket\n");
			continue;
		}
		else {
			// set socket options
			int opt = 1;
			if (setsockopt(sfd, IPPROTO_TCP, TCP_NODELAY, (char *)&opt, sizeof(int)) < 0) {
				printf("Error setting socket option for TCP_NODELAY using IPPROTO\n");
				Disconnect();
				continue;
			}
		}

		// set writer socket
		Writer->SetSocket(sfd);
		printf("socket opened %d\n", (int)sfd);

		while (1) {
			int ret;
			// block on read, put data in buffer
			ret = read(sfd, &data, MAX_PACKET_SIZE);

			// put all data into fifo
			if(ret > 0) {
				for(int i = 0; i < ret; i++) {
					//int rx_ret = RxFifo->Put(&data[i]);
					BaseInterface::Protocol->ProcessByte(data[i]);
				}
			}
			// read error, close socket
			else {
				Disconnect();
				printf("socket closed\n");
				break;
			}
		}
	}
}

/*----------------------------------------------------------------------------
  socket settings
 *----------------------------------------------------------------------------*/
int TcpServer::TcpInit(void) {
	// reset socket
	server_sfd = INVALID_SFD;
	
	// create server socket
	server_sfd = socket(AF_INET, SOCK_STREAM, 0);
	if (server_sfd < 0) {
		printf("error opening socket\n");
		return -1;
	}

	// set socket options
	int opt = 1;
	if (setsockopt(server_sfd, SOL_SOCKET, SO_REUSEADDR, (char *) &opt, sizeof(int)) < 0) {
		printf("error setting socket option for SO_REUSEADDR using SOL_SOCKET\n");
		return -1;
	}

	// set address
	addr->sin_family = AF_INET;
	addr->sin_addr.s_addr = INADDR_ANY;
	addr->sin_port = htons(Port);
	memset(&(addr->sin_zero), '\0', 8);

	// bind to server socket
	if (bind(server_sfd, (struct sockaddr *) addr, sizeof(struct sockaddr)) < 0) {
		printf("error binding socket: %s\n", strerror(errno));
		return -1;
	}

	// listen for clients
	if (listen(server_sfd, 2) == -1) {
		printf("listen failed\n");
		return -1;
	}

	signal(SIGPIPE, SIG_IGN);

	return 1;
}

/*----------------------------------------------------------------------------
  writer thread constructor
 *----------------------------------------------------------------------------*/
TcpWriter::TcpWriter(char *name, int priority) : AperiodicTask() {
	Name = name;
	Priority = priority;
}

/*----------------------------------------------------------------------------
  writer thread init
 *----------------------------------------------------------------------------*/
int TcpWriter::Start() {
	int ret = 1;

	// set variables
	sfd = INVALID_SFD;

	// initialize fifo, double packet size for now
	TxFifo = new FifoQ(sizeof(char), TX_FIFO_SIZE, FifoQ::DISCARD);

	AperiodicTask::Init(Name, Priority);

	return ret;
}

/*----------------------------------------------------------------------------
  send byte
 *----------------------------------------------------------------------------*/
int TcpWriter::SendByte(char data) {
	// can't send if not connected
	if(sfd < 0) return -1;

	//return write(sfd, &data, 1);

	// check for fifo error
	if(TxFifo->Put(&data) < 0) {
		printf("tx fifo full\n");
		return -1;
	}

	// trigger writer
	Trigger(0);

	return 1;
}

/*----------------------------------------------------------------------------
  writer task
 *----------------------------------------------------------------------------*/
void TcpWriter::Task() {
	char data[TX_FIFO_SIZE];
	int ret, i;

	while (1) {
		TriggerWait();

		// get available data
		for(i = 0; i < TX_FIFO_SIZE; i++) {
			if(TxFifo->Get(&data[i]) <= 0) break;
		}

		// send message if bytes available
		if((i > 0) & (sfd >= 0)) {
			ret = write(sfd, &data, i);
		}
	}
}
