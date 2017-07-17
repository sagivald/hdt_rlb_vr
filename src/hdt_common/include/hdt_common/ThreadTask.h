/*----------------------------------------------------------------------------
 * Name:    ThreadTask.h
 * Purpose: thread task base class
 * Note(s):
 *----------------------------------------------------------------------------*/

#ifndef ThreadTask_h
#define ThreadTask_h

#include <unistd.h>
#include <sys/mman.h>
#include <pthread.h>

class ThreadTask {
public:
	ThreadTask();
	~ThreadTask();
	
protected:
	char *mTaskName;
	
	virtual void Task(void) = 0;
	int InitThread(pthread_t *threadId, int prio, void *(*func)(void *), void *param);
};

#endif // ThreadTask_h
