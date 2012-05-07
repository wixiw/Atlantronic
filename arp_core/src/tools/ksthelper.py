# coding=utf-8
f = open("/opt/ros/reports.dat", "r")
firstLine = f.readline()
dd = firstLine.split()
for i, d in enumerate(dd):
  print i+1, d 