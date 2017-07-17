/*----------------------------------------------------------------------------
 * Name:    AperiodicTask.cpp
 * Purpose: aperiodic task class
 * Note(s):
 *----------------------------------------------------------------------------*/

#include <stdlib.h>
#include <stdio.h>

#include "ros/ros.h"

#include "AperiodicTask.h"

/*----------------------------------------------------------------------------
 * constructor
 *----------------------------------------------------------------------------*/
AperiodicTask::AperiodicTask(): ThreadTask() {
}

/*----------------------------------------------------------------------------
 * destructor
 *----------------------------------------------------------------------------*/
AperiodicTask::~AperiodicTask() {
}

/*----------------------------------------------------------------------------
 * init
 *----------------------------------------------------------------------------*/
int AperiodicTask::Init(char *name, int priority) {
	mTaskName = name;
	
	// initialize semaphore
	if(sem_init(&mTaskSemaphore, 1, 0) == -1) {
		ROS_ERROR("%s:Init:sem_init failed", mTaskName);
		return -1;
	}
	
	// start timer thread
	return InitThread(&mThreadId, priority, &TaskThread, (void *)this);
}

/*----------------------------------------------------------------------------
 * trigger task
 *----------------------------------------------------------------------------*/
int AperiodicTask::Trigger(int val) {
	// unblock task thread
	if(sem_post(&(mTaskSemaphore)) == -1) {
		ROS_ERROR("%s:Trigger:sem_post failed", mTaskName);
		exit(1);
	}
}

/*----------------------------------------------------------------------------
 * trigger wait
 *----------------------------------------------------------------------------*/
int AperiodicTask::TriggerWait() {
	// wait for trigger semaphore
	if(sem_wait(&(mTaskSemaphore)) == -1) {
		ROS_ERROR("%s:TriggerWait:sem_wait failed", mTaskName);
		return -1;
	}

	return 1;
}

/*----------------------------------------------------------------------------
 * task thread
 *----------------------------------------------------------------------------*/
void *AperiodicTask::TaskThread(void *arg) {
	AperiodicTask *taskInst;
	taskInst = (AperiodicTask *)arg;
	
	// start task
	taskInst->Task();
}
