#ifndef LED_H
#define LED_H

typedef enum
{
	LED_MODE_WAIT_START_IN,
	LED_MODE_WAIT_AU_UP,
	LED_MODE_WAIT_X86,
	LED_MODE_WAIT_COLOR_SELECTION,
	LED_MODE_COLOR_SELECTION_PREF,
	LED_MODE_COLOR_SELECTION_SYM,
	LED_MODE_COLOR_VALIDATED,
	LED_MODE_WAIT_SELF_TEST,
	LED_MODE_SELF_TESTING,
	LED_MODE_READY_MATCH,
	LED_MODE_WAIT_MATCH,
	LED_MODE_MATCH_RUNNING,
	LED_MODE_MATCH_ENDED,
	LED_MODE_HEARTBEAT_LOST,
	LED_SHUTDOWN
}eLedState;

void led_setState(eLedState state);

#endif
