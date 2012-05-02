
#ifndef WUS_HPP_
#define WUS_HPP_

//ceux qui vont m'invclure auront besoin de linker avec
// signal(SIGXCPU, warn_upon_switch);
//donc ils en auront besoin
#include <signal.h>

// LE fameux WUS
void warn_upon_switch(int sig __attribute__((unused)));

#endif
