#!/bin/bash
rm -f /tmp/ARP_LOADING
rm -f /tmp/ARP_FAILED
rm -f /tmp/ARP_DEPLOYED
rm -f /tmp/ARP_READY
rm -f /tmp/ARP_RUNNING
rm -f /tmp/ARP_FINISHED

beep -f300 -l300
touch /tmp/ARP_RUNNING
echo Match is running !
