#include <stdio.h>
#include "Timer.h"
//#include "Ekg.h"

Timer::Timer(){

}

Timer::~Timer(){

}

int Timer::Init(double rate){
	sampleRate = rate;
	ticks = 0;

	//EKG->AddSignal((char *)"TMR", (char *)"ElapsedTime_Sec", &ElapsedTime_Sec);
	return 1;
}

int Timer::Process(){
	ticks++;
	ElapsedTime_Sec = (double)ticks/sampleRate;
	return 1;
}

int Timer::Set(TimerData *td, double timeInSec){
	td->startTime = ticks;
	td->stopTime = td->startTime + (unsigned long long)(timeInSec * sampleRate);
	td->armed = 1;
	td->timerSec = timeInSec;
	return 1;
}

int Timer::Set(TimerData *td, double timeInSec, StartState state){

	if (state == START_OFF) {
		td->startTime = ticks;
		td->stopTime = td->startTime;
		td->armed = 1;
		td->timerSec = timeInSec;
		return 1;
	}
	else {
		Set(td, timeInSec);
		return 1;
	}
}

int Timer::Armed(TimerData *td){
	return td->armed;
}

int Timer::TimeUp(TimerData *td){
	if(ticks >= td->stopTime && td->armed){
		return 1;
	}
	else{
		return 0;
	}
}

double Timer::Overtime(TimerData *td){
	if(ticks >= td->stopTime && td->armed){
		return (((double)(ticks - td->stopTime)) / sampleRate);
	}
	else{
		return 0.;
	}
}

double Timer::Elapsed(TimerData *td){
	return ((double)(ticks - td->startTime) / sampleRate);
}

int Timer::Disarm(TimerData *td){
	td->armed = 0;
	return 1;
}

int Timer::Reset(TimerData *td) {
	Set(td, td->timerSec);
	return 1;
}
