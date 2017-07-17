///////////////////////////////////////////////////////////////////////////////
// Upon getting a connection request the server sends a list of signal
// names followed by the sample rate

#ifndef Ekg_h
#define Ekg_h

#include <sys/types.h>
#include <netinet/in.h>
#include "AperiodicTask.h"

class FifoQ;
class Ekg;

extern Ekg 	*EKG;

#define MAX_NUM_SIGNALS 256
#define MAX_NAME_LENGTH 25

class Ekg : public AperiodicTask{
    public:
        // Constuctor
        Ekg();
        // Destructor
        ~Ekg();
		  int Init(double rate, int priority);
		  int Init(double rate, int priority, int port);
		  void Process();
		  void AddSignal(char *prefix, char *name, double *val);
		  void AddSignal(char *prefix, char *name, float *val);
		  void AddSignal(char *prefix, char *name, int *val);
		  void AddSignal(char *prefix, char *name, int16_t *val);
    private:
		double mSampleRate;
		FifoQ *mpFifo;
		int mPort;

		char label[100];

		typedef union valPointer_t{
			double *dValPtr;
			int *iValPtr;
			float *fValPtr;
			int16_t *i16ValPtr;
		}valPointer;

		typedef enum{INTEGER, DOUBLE, INT16, FLOAT}valType;

		typedef struct signalPointer_t{
			valPointer p;
			valType t;
		}signalPointer;

		char mNameArr[MAX_NUM_SIGNALS][MAX_NAME_LENGTH+1];
		unsigned char mBuf[MAX_NUM_SIGNALS * (MAX_NAME_LENGTH + 1)];
		signalPointer mpValPtrArr[MAX_NUM_SIGNALS];
		double *mpValBuf;
		int mNumSignals;
		int mInitialized;

		// TCP/IP
		int mSocket;
		int mSessionSocket;
		struct sockaddr_in mServerAddr;
		unsigned char validSession;

		int TcpIpInit();

		// Data marshalling
		int MarshallSignals();
		int MarshallSampleRate();
		int MarshallLabels();

		// Task function
		void Task();

		static void sig_handler(int signo){};

		int divisorCount;

};

#endif // Ekg_h


