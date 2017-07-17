/*----------------------------------------------------------------------------
 * Name:    AperiodicTask.h
 * Purpose: aperiodic task class
 * Note(s):
 *----------------------------------------------------------------------------*/

#ifndef AperiodicTask_h
#define AperiodicTask_h

#include <semaphore.h>

#include "ThreadTask.h"

class AperiodicTask: public ThreadTask {
public:
	AperiodicTask();
	~AperiodicTask();
	int Init(char *name, int priority);
	
	int Trigger(int value);
private:
	pthread_t mThreadId;
	
	sem_t mTaskSemaphore;
	
	static void *TaskThread(void *arg);

protected:
	int TriggerWait();
};

#endif // AperiodicTask_h
