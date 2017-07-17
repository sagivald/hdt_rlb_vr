/*----------------------------------------------------------------------------
 * Name:    SerialInterface.cpp
 * Purpose: serial interface class
 * Note(s):
 *----------------------------------------------------------------------------*/

#include <termios.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "SerialInterface.h"

/*----------------------------------------------------------------------------
  constructor
 *----------------------------------------------------------------------------*/
SerialInterface::SerialInterface(char *name, char *port, int baud, int priority) : BaseInterface(), AperiodicTask() {
	Name = name;
	Port = port;
	Baud = baud;
	Priority = priority;
}

/*----------------------------------------------------------------------------
  init
 *----------------------------------------------------------------------------*/
int SerialInterface::Start() {
	int ret = 1;

	//printf("%s opening port\n", Name);
	// try to open port
	if ((fd = open(Port, O_RDONLY)) == -1) {
		printf("%s can't open port\n", Name);
        return -1;
    }

	//printf("%s port open\n", Name);
	// setup port
	SetupPort(fd, Baud);
	//printf("%s port setup\n", Name);
	
	int rx_size, tx_size;
	ioctl(fd, TIOCINQ, &rx_size);
	ioctl(fd, TIOCOUTQ, &tx_size);
	printf("rx_size = %d, tx_size = %d\n", rx_size, tx_size);

	// initialize fifo
	RxFifo = new FifoQ(sizeof(char), MAX_PACKET_SIZE, FifoQ::DISCARD);

	// initialize associated serial writer
	Writer = new SerialWriter(Name, Port, Baud, Priority);
	ret = Writer->Start();
	//printf("%s writer start\n", Name);

	AperiodicTask::Init(Name, Priority);

	return ret;
}

/*----------------------------------------------------------------------------
  send byte (to writer)
 *----------------------------------------------------------------------------*/
int SerialInterface::SendByte(char data) {
	// trigger writer
	return Writer->SendByte(data);
}

/*----------------------------------------------------------------------------
  read byte
 *----------------------------------------------------------------------------*/
int SerialInterface::ReadByte(char &data) {
	return RxFifo->Get(&data);
}

/*----------------------------------------------------------------------------
  read task
 *----------------------------------------------------------------------------*/
void SerialInterface::Task(void) {
	char data[MAX_PACKET_SIZE];

	while (1) {
		int ret;
		
		// block on read, put data in buffer
		ret = read(fd, &data, MAX_PACKET_SIZE);

		// put all data into fifo
		for(int i = 0; i < ret; i++) {
			//RxFifo->Put(&data[i]);
			Protocol->ProcessByte(data[i]);
		}
	}
}

/*----------------------------------------------------------------------------
  constructor
 *----------------------------------------------------------------------------*/
SerialWriter::SerialWriter(char *name, char *port, int baud, int priority) : AperiodicTask() {
	Name = name;
	Port = port;
	Baud = baud;
	Priority = priority;
}

/*----------------------------------------------------------------------------
  init
 *----------------------------------------------------------------------------*/
int SerialWriter::Start(void) {
	int ret = 1;

	// try to open port
	fd = -1;
	if ((fd = open(Port, O_WRONLY)) == -1) {
		printf("%s can't open port\n", Name);
        return -1;
    }

	// setup port
	SetupPort(fd, Baud);

	// initialize fifo
	TxFifo = new FifoQ(sizeof(char), TX_FIFO_SIZE, FifoQ::DISCARD);

	AperiodicTask::Init(Name, Priority);

	return ret;
}

/*----------------------------------------------------------------------------
  send byte
 *----------------------------------------------------------------------------*/
int SerialWriter::SendByte(char data) {
	//return write(fd, &data, 1);

	// check for fifo error
	if(TxFifo->Put(&data) < 0) {
		return -1;
	}

	// trigger
	Trigger(0);

	return 1;
}

/*----------------------------------------------------------------------------
  write task
 *----------------------------------------------------------------------------*/
void SerialWriter::Task(void) {
	char data[TX_FIFO_SIZE];
	int ret, i;

	while (1) {
		TriggerWait();

		// get available data
		for(i = 0; i < TX_FIFO_SIZE; i++) {
			if(TxFifo->Get(&data[i]) <= 0) break;
		}

		// send message if bytes available
		if((i > 0) & (fd >= 0)) {
			ret = write(fd, &data, i);
			if(ret < 0) printf("write error\n");
			else if(ret < i) printf("not all bytes written\n");
		}
	}
}

/*----------------------------------------------------------------------------
  get baud rate
 *----------------------------------------------------------------------------*/
int GetBaudRate(int baud) {
	if (baud == 300)
		return B300;
	if (baud == 1200)
		return B1200;
	if (baud == 2400)
		return B2400;
	if (baud == 4800)
		return B4800;
	if (baud == 9600)
		return B9600;
	if (baud == 19200)
		return B19200;
	if (baud == 38400)
		return B38400;
	if (baud == 57600)
		return B57600;
	if (baud == 115200)
		return B115200;

	return (B0);
}

/*----------------------------------------------------------------------------
  serial port setup
 *----------------------------------------------------------------------------*/
int SetupPort(int fd, int baud) {
	struct termios termios_p;

	// get port control structure
	tcgetattr(fd, &termios_p);

	// set baud rate
	cfsetispeed(&termios_p, GetBaudRate(baud));
	cfsetospeed(&termios_p, GetBaudRate(baud));

	//INPUT
	// set input mode
	// clear all
	//termios_p.c_iflag &= ~(TC_IPOSIX);
	termios_p.c_iflag &= ~(IGNCR);
	// set no parity
	termios_p.c_iflag |= (IGNBRK | IGNPAR);

	//OUTPUT
	// set output modes
	// clear all
	//termios_p.c_oflag &= ~(TC_OPOSIX);

	//LOCAL
	// set local modes
	// clear all
	//termios_p.c_lflag &= ~(TC_LPOSIX);
	termios_p.c_lflag |= IEXTEN;

	//CONTROL
	// set control modes
	// clear all
	//termios_p.c_cflag &= ~(TC_CPOSIX);
	//termios_p.c_cflag &= ~(IHFLOW | OHFLOW | PARSTK);
	termios_p.c_cflag &= ~(CRTSCTS);
	// set to 8 bits, receiver enabled, no parity
	termios_p.c_cflag |= (CLOCAL | CREAD | CS8);

	// set port control structure values
	tcsetattr(fd, TCSANOW, &termios_p);

	// flush data
	tcflush(fd, TCIOFLUSH);

	return 1;
}
