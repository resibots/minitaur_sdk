import socket
import os
import struct
import threading

# from netaddr import IPNetwork,IPAddress
# from ctypes import *

# host to listen on
host   = '169.254.98.1'
port = 15000
# sniffer = socket.socket(socket.AF_PACKET , socket.SOCK_RAW , socket.ntohs(0x0003))
sniffer = socket.socket(socket.AF_PACKET , socket.SOCK_DGRAM)
sniffer.bind((host, port))
try:
	while True:
		raw_buffer = sniffer.recvfrom(65565)[0]
		print raw_buffer
# handle CTRL-C
except KeyboardInterrupt:
    # if we're on Windows turn off promiscuous mode
    if os.name == "nt":
        sniffer.ioctl(socket.SIO_RCVALL, socket.RCVALL_OFF)
