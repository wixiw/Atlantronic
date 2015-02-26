#ifndef END_H
#define END_H

#ifndef WEAK_END
#include "core/asm/asm_base_func.h"
#define WEAK_END __attribute__((weak, alias("nop_function") ))
#endif

//Configure the end of match duration in milliseconds. If zero, desactivates the end of match
void end_cmd_set_time(uint32_t time) WEAK_END;

//In case you didn't provided a match duration, call this to end the match
void end_quit_match() WEAK_END;

//Retrieve the time until the end of the match in ms
uint32_t end_get_match_time_togo() WEAK_END;

//self explainatory
bool isMatchBegun() WEAK_END;

//self explainatory
bool isMatchEnded() WEAK_END;


#endif
