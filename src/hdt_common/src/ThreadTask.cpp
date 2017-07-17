/*----------------------------------------------------------------------------
 * Name:    ThreadTask.cpp
 * Purpose: thread task base class
 * Note(s):
 *----------------------------------------------------------------------------*/

#include <errno.h>

#include "ros/ros.h"

#include "ThreadTask.h"

/*----------------------------------------------------------------------------
 * constructor
 *----------------------------------------------------------------------------*/
ThreadTask::ThreadTask() {
}

/*----------------------------------------------------------------------------
 * destructor
 *----------------------------------------------------------------------------*/
ThreadTask::~ThreadTask() {
}

/*----------------------------------------------------------------------------
 * init thread
 *----------------------------------------------------------------------------*/
int ThreadTask::InitThread(pthread_t *threadId, int prio, void *(*func)(void *), void *param) {	
	pthread_attr_t attr;
	sched_param sched;

	if(pthread_attr_init(&attr) != 0) {
		ROS_ERROR("%s:pthread_attr_init failed, errno = %d", mTaskName, errno);
		return -1;
	}

	if(pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED) != 0) {
		ROS_ERROR("%s:pthread_attr_setdetachstate failed, errno = %d", mTaskName, errno);
		return -1;
	}

	if(pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED) != 0) {
		ROS_ERROR("%s:pthread_attr_setinheritsched failed, errno = %d", mTaskName, errno);
		return -1;
	}
/*
	if(pthread_attr_setschedpolicy(&attr, SCHED_FIFO) != 0) {
		ROS_ERROR("%s:pthread_attr_setschedpolicy failed, errno = %d", mTaskName, errno);
		return -1;
	}

	if(pthread_attr_getschedparam(&attr, &sched) != 0) {
		ROS_ERROR("%s:pthread_attr_getschedparam failed, errno = %d", mTaskName, errno);
		return -1;
	}

	sched.sched_priority = prio;

 	if(pthread_attr_setschedparam(&attr, &sched) != 0) {
		ROS_ERROR("%s:pthread_attr_setschedparam failed, errno = %d", mTaskName, errno);
		return -1;
	}
*/
 	if(pthread_create(threadId, &attr, func, param) != 0) {
		ROS_ERROR("%s:pthread_create failed, errno = %d", mTaskName, errno);
		return -1;
	}

	if(pthread_attr_destroy(&attr) != 0) {
		ROS_ERROR("%s:pthread_attr_destroy failed, errno = %d", mTaskName, errno);
		return -1;
	}
	
	return 1;
}
