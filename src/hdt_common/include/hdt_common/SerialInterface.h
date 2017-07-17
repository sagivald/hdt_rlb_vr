/*----------------------------------------------------------------------------
 * Name:    SerialInterface.h
 * Purpose: serial interface class
 * Note(s):
 *----------------------------------------------------------------------------*/

#ifndef SerialInterface_h
#define SerialInterface_h

#include "BaseInterface.h"
#include "AperiodicTask.h"

int SetupPort(int fd, int baud);
int GetBaudRate(int baud);

class SerialWriter;

// serial interface class
class SerialInterface : public BaseInterface, public AperiodicTask {
public:
	SerialInterface(char *name, char *port, int baud, int priority);
	~SerialInterface() {};
	
	int Start();
	int SendByte(char data);
	int ReadByte(char &data);
	int IsConnected(void) { return (fd == -1); };
	int Disconnect(void) { return 0; };
private:
	SerialWriter *Writer;
	FifoQ *RxFifo;
	int fd;
	
	char *Name;
	char *Port;
	int Baud;
	int Priority;

	void Task(void);
};

// serial writer class
class SerialWriter: public AperiodicTask {
public:
	SerialWriter(char *name, char *port, int baud, int priority);
	~SerialWriter() {};

	int Start();
	int SendByte(char data);
private:
	FifoQ *TxFifo;
	int fd;

	char *Name;
	char *Port;
	int Baud;
	int Priority;

	void Task(void);
};

#endif // SerialInterface_h
