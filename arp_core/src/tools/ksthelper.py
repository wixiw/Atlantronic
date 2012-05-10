# coding=utf-8
f = open("/tmp/reports.dat", "r")
firstLine = f.readline()
dd = firstLine.split()
for i, d in enumerate(dd):
  print i+1, d 