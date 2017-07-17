/*----------------------------------------------------------------------------
 * Name:    PeriodicTask.h
 * Purpose: periodic task class
 * Note(s):
 *----------------------------------------------------------------------------*/

#ifndef PeriodicTask_h
#define PeriodicTask_h

#include <semaphore.h>

#include "ThreadTask.h"

class PeriodicTask: public ThreadTask {
public:
	PeriodicTask();
	~PeriodicTask();
	int Init(char *name, double rate, double *actual_rate, int priority);
	
	int mOverRun;
private:
	unsigned char mRunning;
	double mRate;
	sem_t mTimerSemaphore;
	
	pthread_t mTimerThreadId;

	int InitTimer(int priority, double rate, double *actual_rate);
	static void TimerControlThread(sigval arg);
	static void *TimerThread(void *arg);


protected:
	int TimerWait();
};

#endif // PeriodicTask_h
