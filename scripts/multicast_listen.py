#!/usr/bin/env python3

import socket
import struct

address = '236.6.7.5'
port = 6878

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((address, port))

mreq = struct.pack("4sl", socket.inet_aton(address), socket.INADDR_ANY)

sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
while True:
  data,addr = sock.recvfrom(10240)
  print(len(data),'bytes from',addr)
