/*----------------------------------------------------------------------------
 * Name:    PeriodicTask.cpp
 * Purpose: periodic task class
 * Note(s):
 *----------------------------------------------------------------------------*/

#include <sched.h>
#include <semaphore.h>
#include <time.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/types.h>

#include "ros/ros.h"

#include "PeriodicTask.h"

/*----------------------------------------------------------------------------
 * constructor
 *----------------------------------------------------------------------------*/
PeriodicTask::PeriodicTask(): ThreadTask() {
}

/*----------------------------------------------------------------------------
 * destructor
 *----------------------------------------------------------------------------*/
PeriodicTask::~PeriodicTask() {
}

/*----------------------------------------------------------------------------
 * init
 *----------------------------------------------------------------------------*/
int PeriodicTask::Init(char *name, double rate, double *actual_rate, int priority) {
	mTaskName = name;
	mRate = rate;
	int ret;
	
	// initialize semaphore
	if(sem_init(&mTimerSemaphore, 1, 0) == -1) {
		ROS_ERROR("%s:Init:sem_init failed", mTaskName);
		return -1;
	}

	// init timer
	if(InitTimer(priority, rate, actual_rate) == -1) {
		ROS_ERROR("%s:Init:InitTimer failed", mTaskName);
		return -1;
	}
	
	// start timer thread
	if(InitThread(&mTimerThreadId, priority, &TimerThread, (void *)this) == -1) {
		ROS_ERROR("%s:Init:InitThread:mTimerThreadId failed", mTaskName);
		return -1;
	}
	
	return 1;
}

/*----------------------------------------------------------------------------
 * init timer
 *----------------------------------------------------------------------------*/
int PeriodicTask::InitTimer(int priority, double rate, double *actual_rate) {
	struct sigevent sevp;
	struct itimerspec its;
	static timer_t timerid;
	
	// calculate timer period
	long period_nsec = (long)(1.0/rate * 1.0e9);

	// get the actual ticksize
	*actual_rate = 1.0/((double)period_nsec / 1.0e9);

	// create timer
	sevp.sigev_notify = SIGEV_THREAD;
	sevp.sigev_notify_function = TimerControlThread;
    sevp.sigev_value.sival_ptr = (void *)this;
	sevp.sigev_notify_attributes = NULL;
    if(timer_create(CLOCK_REALTIME, &sevp, &timerid)) {
		ROS_ERROR("%s:TimerControlThread:timer_create failed, errno = %d", mTaskName, errno);
		return -1;
	}

	// set timer value
	its.it_value.tv_sec = 0;
	its.it_value.tv_nsec = period_nsec;
	// nsec can't be bigger than 999999999
    if (its.it_value.tv_nsec >= 1000000000) {
		its.it_value.tv_sec += its.it_value.tv_nsec / 1000000000;
		its.it_value.tv_nsec %= 1000000000;
	}
	its.it_interval = its.it_value;
    if(timer_settime(timerid, 0, &its, NULL)) {
		ROS_ERROR("%s:TimerControlThread:timer_settime failed, errno = %d", mTaskName, errno);
		return -1;
	}
}

/*----------------------------------------------------------------------------
 * timer control thread
 *----------------------------------------------------------------------------*/
void PeriodicTask::TimerControlThread(sigval arg) {
	PeriodicTask *taskInst;
	taskInst = (PeriodicTask *)arg.sival_ptr;

	//ROS_INFO("PeriodicTask::TimerControlThread");

	// unblock task thread
	if(sem_post(&(taskInst->mTimerSemaphore)) == -1) {
		ROS_ERROR("%s:TimerControlThread:sem_post failed", taskInst->mTaskName);
		exit(1);
	}
}

/*----------------------------------------------------------------------------
 * timer wait
 *----------------------------------------------------------------------------*/
int PeriodicTask::TimerWait() {
	mRunning = 0;

	// wait for timer semaphore
	if(sem_wait(&(mTimerSemaphore)) == -1) {
		ROS_ERROR("%s:TimerThread:sem_wait failed", mTaskName);
		return -1;
	}

	mRunning = 1;

	return 1;
}

/*----------------------------------------------------------------------------
 * task thread
 *----------------------------------------------------------------------------*/
void *PeriodicTask::TimerThread(void *arg) {
	PeriodicTask *taskInst;
	taskInst = (PeriodicTask *)arg;
	
	// start task
	taskInst->Task();
}

