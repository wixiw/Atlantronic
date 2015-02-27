#ifndef STATE_MACHINE
#define STATE_MACHINE

typedef void (*StateMachineEntry_t) ();
typedef void (*StateMachineRun_t) ();
typedef void (*StateMachineExit_t) ();
typedef unsigned int (*StateMachineTransition_t) (unsigned int currentState);

struct StateMachineState
{
	const char* name;
	StateMachineEntry_t entry;
	StateMachineRun_t run;
	StateMachineExit_t exit;
	StateMachineTransition_t transition;
};

//Default callbacks :
void no_entry();
void no_run();
void no_exit();
unsigned int no_transition(unsigned int state);

class StateMachine
{
	public:
		StateMachine(StateMachineState* states, unsigned int size);

		int execute();
		inline int getCurrentState()
		{
			return currentStateId;
		}

	protected:
		StateMachineState* states;
		unsigned int size;
		unsigned int currentStateId;
		unsigned int lastStateId;
};

#endif
