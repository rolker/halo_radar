#!/usr/bin/env python3

import socket
import sys

address = sys.argv[1]
port = 6878

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('',port))
s.settimeout(2.0)

while True:
  s.sendto(bytearray.fromhex('01b1'), (address,port))
  data,addr = s.recvfrom(1024)
  print (addr)
  print (data)
