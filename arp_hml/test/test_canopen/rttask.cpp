/*
 * rttask.cpp
 *
 *  Created on: Feb 2, 2014
 *      Author: ard
 */

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <string.h>
#include <iostream>

using namespace std;
int MY_PRIORITY;
#define MAX_SAFE_STACK (8*1024) /* The maximum stack size which is
                                   guaranteed safe to access without
                                   faulting */

#define NSEC_PER_SEC    (1000000000) /* The number of nsecs per sec. */

void stack_prefault(void)
{

    return;
}

int main(int argc, char* argv[])
{
    int c;
    opterr = 0;
    char *pvalue = NULL;

    while ((c = getopt(argc, argv, "p:")) != -1)
        switch (c)
        {
            case 'p':
                pvalue = optarg;
                sscanf(pvalue,"%d", &MY_PRIORITY);
                break;
            case '?':
                if (optopt == 'p')
                    fprintf(stderr, "Option -%c requires an argument.\n", optopt);
                else if (isprint(optopt))
                    fprintf(stderr, "Unknown option `-%c'.\n", optopt);
                else
                    fprintf(stderr, "Unknown option character `\\x%x'.\n", optopt);
                return 1;
            default:
                abort();
        }

    struct sched_param param;

    cout << "Starting an infinite loop at priority " << MY_PRIORITY << " use Ctrl+C to interrupt." << endl;

    /* Declare ourself as a real time task */

    param.sched_priority = MY_PRIORITY;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
    {
        perror("sched_setscheduler failed");
        exit(-1);
    }

    /* Lock memory */

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        perror("mlockall failed");
        exit(-2);
    }

    /* Pre-fault our stack */

    stack_prefault();

    while (1)
    {
        //cout << "poil" << endl;

    }
}

