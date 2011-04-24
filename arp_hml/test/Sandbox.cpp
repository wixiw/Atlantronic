//============================================================================
// Name        : can_test.cpp
// Author      : WLA
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <string.h>

//can
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>



using namespace std;


void send_socket()
{
    /* Create the socket */
    int skt = socket( PF_CAN, SOCK_RAW, CAN_RAW );

    /* Locate the interface you wish to use */
    struct ifreq ifr;
    strcpy(ifr.ifr_name, "vcan0");
    ioctl(skt, SIOCGIFINDEX, &ifr); /* ifr.ifr_ifindex gets filled
                                   * with that device's index */

    /* Select that CAN interface, and bind the socket to it. */
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind( skt, (struct sockaddr*)&addr, sizeof(addr) );

    /* Send a message to the CAN bus */
    struct can_frame frame;
    frame.can_id = 0x123;
    frame.data[0] = 0b10101111;
    frame.data[1] = 0;
    frame.data[2] = 0;
    frame.data[3] = 0;
    frame.data[4] = 0;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0b11111111;
    frame.can_dlc = 8;
    int bytes_sent = write( skt, &frame, sizeof(frame) );
    cout << "byte sent : " << bytes_sent << endl;
}


void read_socket()
{
    /* Create the socket */
    int skt = socket( PF_CAN, SOCK_RAW, CAN_RAW );

    /* Locate the interface you wish to use */
    struct ifreq ifr;
    strcpy(ifr.ifr_name, "vcan0");
    ioctl(skt, SIOCGIFINDEX, &ifr); /* ifr.ifr_ifindex gets filled
                                   * with that device's index */

    /* Select that CAN interface, and bind the socket to it. */
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind( skt, (struct sockaddr*)&addr, sizeof(addr) );

    /* Send a message to the CAN bus */
    struct can_frame frame;

    /* Read a message back from the CAN bus */
    int bytes_read = read( skt, &frame, sizeof(frame) );
    cout << "byte read : " << bytes_read << endl;
}


int main()
{
    cout << "!!!Hello World!!!" << endl; // prints !!!Hello World!!!

    send_socket();
    read_socket();


}


