/*----------------------------------------------------------------------------
 * Name:    TcpInterface.h
 * Purpose: tcp interface class
 * Note(s):
 *----------------------------------------------------------------------------*/

#ifndef TcpInterface_h
#define TcpInterface_h

#include <sys/types.h>
#include <netinet/in.h>

#include "BaseInterface.h"
#include "AperiodicTask.h"

#define INVALID_SFD	-1

class TcpWriter;

// base class for tcp client and server
class TcpInterface: public BaseInterface, public AperiodicTask {
public:
	TcpInterface();
	~TcpInterface() {};

	int Start(void);
	int SendByte(char data);
	int ReadByte(char &data);
	int IsConnected(void);
	int Disconnect(void);
protected:
	TcpWriter *Writer;
	FifoQ *RxFifo;

	int sfd;
	sockaddr_in *addr;
	char *Name;
	int Port;
	int Priority;

	virtual void Task() = 0;
	virtual int TcpInit(void) = 0;
};

// tcp client class
class TcpClient: public TcpInterface {
public:
	TcpClient(char *name, char *server, int port, int priority);
	~TcpClient() {};
	int Init(void);
private:
	void Task();
	int TcpInit(void);

	char *Server;
};

// tcp server class
class TcpServer: public TcpInterface {
public:
	TcpServer(char *name, int port, int priority);
	~TcpServer() {};
	int Init(void);
private:
	void Task();
	int TcpInit(void);

	int server_sfd;
};

// class for writing to tcp sockets
class TcpWriter: public AperiodicTask {
public:
	TcpWriter(char *name, int priority);
	~TcpWriter() {};

	int Start(void);
	int SendByte(char data);
	void SetSocket(int socket) { sfd = socket;};
private:
	FifoQ *TxFifo;
	int sfd;

	char *Name;
	int Priority;
	
	void Task();
};

#endif // TcpInterface_h
