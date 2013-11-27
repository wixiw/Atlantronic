#!/bin/bash
cd `rospack find arp_hml`/src/orocos/can/dictionnary
python `rospack find can_festival`/src/objdictgen/objdictedit.py CanARD.od
