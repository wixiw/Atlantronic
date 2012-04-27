#include "rtdm/rtcan.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>     /* for NULL */
#include <errno.h>
#include <sys/mman.h>
#include <native/task.h>
#include <native/pipe.h>
#include <rtdm/rtcan.h>


RT_TASK rt_task_desc;

int main()
{
    printf("hello\n");

    struct can_frame frame;
    struct ifreq ifr;
    int socket;
    int ret;

    //open socket
    socket = rt_dev_socket (PF_CAN, SOCK_RAW, CAN_RAW);
    if( socket < 0 )
        printf("Failed to open socket %d", socket);
    else
        printf("Opened socket %d\n",socket);

    //configure device
    strncpy(ifr.ifr_name, "rtcan0", IFNAMSIZ);
    ret = rt_dev_ioctl(socket, SIOCGIFINDEX, &ifr);
    printf("s=%d, ifr_name=%s\n", socket, ifr.ifr_name);
    if (ret < 0)
        printf(stderr, "rt_dev_ioctl GET_IFINDEX: %s\n", strerror(-ret));
    else
        printf("Ioctl succeed\n");

    //send message
    ret = rt_dev_send(socket,&frame,sizeof(frame),0);
    if( ret < 0 )
         printf("Failed to send %d (%s)\n", ret, strerror (-ret));
    else
        printf("Message send\n");

    //shadowing
    mlockall(MCL_CURRENT | MCL_FUTURE);
    ret = rt_task_shadow(&rt_task_desc, "MyTask", 0, 0);
    if (ret)
        printf(stderr, "rt_task_shadow: %s\n", strerror(-ret));
    else
        printf("Shadowing ok\n");

    //send message RT
    ret = rt_dev_send(socket,&frame,sizeof(frame),0);
    if( ret < 0 )
        printf("Failed to send RT %d (%s)\n", ret, strerror (-ret));
    else
        printf("Message RT send\n");

    return 0;
}
