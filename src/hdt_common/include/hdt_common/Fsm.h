//////////////////////////////////////////////////////////////////////////////
// Finite State Machine
//
// Fsm consists of three types of objects:
// 1. Fsm : the machine itself
// 2. State : the state in a machine
// 3. Transition : the conditional state transition mechanism
//
// The actual tasks in the state machine are implemented via user defined
// callbacks which are provided as function pointers to State and Trasition
// constuctors.  The following user callbacks can be defined:
//
// State's Init() method
// State's Process() method
// State's Exit() method
// Transition's Process method
//
// The callback has to conform to the following type
// int f(void);
// In addition the callback has to be a member function of the Fsm derived class.
// Note that the Transition's Process() callback method has to return 1
// upon sucessfull condition check as the call to this function drives the
// state transitions.  The returns of other user defined callback do not
// have any significance.
//
// The Fsm class is meant to serve as a base class.  The user h
// should be carefull not to overload base class
// functions such as Process(), etc or be sure to call them in the overloaded
// functions (i.e. Fsm::Process()).  Also be sure to call the base class
// constructor!
//////////////////////////////////////////////////////////////////////////////

#ifndef Fsm_h
#define Fsm_h

class Fsm;
class State;
class Transition;


typedef  int (Fsm::*FsmMemberFunc)(void);
#define Callback(obj,func) (obj->*func)()

//////////////////////////////////////////////////////////////////////////////
// Fsm class
//////////////////////////////////////////////////////////////////////////////
class Fsm{
	public:
		// Constructor
		Fsm(char *name, int debug);
		void Init();
		int Process();
		void Exit(){;}
		void SetInitState(State *init);
		char *GetCurState();

	private:
		State *mpCurState;
		State *mpInitState;
		char *mpName;
		int	mDebug;
};

//////////////////////////////////////////////////////////////////////////////
// State class
//////////////////////////////////////////////////////////////////////////////
#define FSM_STATE_MAX_TRANSITIONS	20

class State{
	friend class Fsm;

	public:
		State(char *name, FsmMemberFunc init, FsmMemberFunc process, FsmMemberFunc exit);
		~State();
		int AddTransition(Transition *t);
		Transition* GetActiveTransition(Fsm *fsm);

	private:
		char *mpName;
		int mTransitionCount;
		Transition *mTransitionList[FSM_STATE_MAX_TRANSITIONS];

		// Callbacks
		FsmMemberFunc Init;
		FsmMemberFunc Process;
		FsmMemberFunc Exit;
};

//////////////////////////////////////////////////////////////////////////////
// Transition class
//////////////////////////////////////////////////////////////////////////////
class Transition{
	friend class Fsm;
	friend class State;

	public:
		Transition(char *name, State *target, FsmMemberFunc process);

	private:
		char *mpName;
		State *mpTarget;

		// Callbacks
		FsmMemberFunc Process;
};


#endif // Fsm_h
