
#include "wus.hpp"
#include <execinfo.h>
#include <errno.h>
#include <stdio.h>

// LE fameux WUS
void warn_upon_switch(int sig __attribute__((unused)))
{
    void *bt[32];
    int nentries;

    /* Dump a backtrace to standard error of the frame which caused the switch to
       secondary mode: */
    nentries = backtrace(bt,sizeof(bt) / sizeof(bt[0]));
    backtrace_symbols_fd(bt,nentries,fileno(stderr));
}
