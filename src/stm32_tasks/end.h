#ifndef END_H
#define END_H

extern volatile int end_match;

//Configure the end of match duration in milliseconds. If zero, desactivates the end of match
void end_cmd_set_time(uint32_t time);

//In case you didn't provided a match duration, call this to end the match
void end_quit_match();

//Retrieve the time until the end of the match in ms
uint32_t end_get_match_time_togo();

#endif
