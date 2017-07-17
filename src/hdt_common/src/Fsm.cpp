#include<stdio.h>
#include "Fsm.h"

//////////////////////////////////////////////////////////////////////////////
// Fsm class functions
//////////////////////////////////////////////////////////////////////////////
Fsm::Fsm(char *name, int debug){
	mpName = name;
	mpInitState = NULL;
	mpCurState = NULL;
	mDebug = debug;
}

void Fsm::Init(){
	mpCurState = mpInitState;
	if(mpCurState == NULL){
		printf("FSM '%s' : Initial State not Set\n", mpName);
		return;
	}
	printf("FSM '%s' : STATE '%s' -> Init()\n", mpName, mpCurState->mpName);
	if(mpCurState->Init != NULL){
		Callback(this, mpCurState->Init);
	}

}

int Fsm::Process(){
  	if(mpCurState == NULL)
		return -1;

	//printf("FSM '%s' : STATE '%s' -> Process()\n", mpName, mpCurState->mpName);
	if(mpCurState->Process != NULL){
		Callback(this, mpCurState->Process);
	}

	Transition *t = mpCurState->GetActiveTransition(this);
	if(t != NULL){
		printf("FSM '%s' : TRANSITION '%s' Active\n", mpName, t->mpName);
		if(mpCurState->Exit != NULL){
			printf("FSM '%s' : STATE '%s' -> Exit()\n", mpName, mpCurState->mpName);
			Callback(this, mpCurState->Exit);
		}
		mpCurState = t->mpTarget;		
		if(mpCurState->Init != NULL){
			printf("FSM '%s' : STATE '%s' -> Init()\n", mpName, mpCurState->mpName);
			Callback(this, mpCurState->Init);
		}
	}

	//printf("\n");

	return 1;
}

void Fsm::SetInitState(State *init){
	mpInitState = init;
	mpCurState = mpInitState;
}

char* Fsm::GetCurState(){
	return mpCurState->mpName;
}
//////////////////////////////////////////////////////////////////////////////
// State class functions
//////////////////////////////////////////////////////////////////////////////
State::State(char *name, FsmMemberFunc init, FsmMemberFunc process, FsmMemberFunc exit){
	mpName = name;
	mTransitionCount = 0;
	Init = init;
	Process = process;
	Exit = exit;
}

State::~State(){
	int i;

	for(i=0; i<mTransitionCount; i++){
		delete mTransitionList[i];
	}
}

int State::AddTransition(Transition *t){
	if(mTransitionCount < FSM_STATE_MAX_TRANSITIONS){
		mTransitionList[mTransitionCount++] = t;
		return 1;
	}
	else{
		return -1;
	}
}

Transition* State::GetActiveTransition(Fsm *fsm){
	int i;

	for(i = 0; i < mTransitionCount; i++){
		if(mTransitionList[i]->Process != NULL){
			if(Callback(fsm, mTransitionList[i]->Process) == 1){
				return mTransitionList[i];
			}
		}
	}
	return NULL;
}

//////////////////////////////////////////////////////////////////////////////
// Transition class functions
//////////////////////////////////////////////////////////////////////////////
Transition::Transition(char *name, State *target, FsmMemberFunc process){
	mpName = name;
	mpTarget = target;
	Process = process;
}

