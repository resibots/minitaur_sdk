
import socket
import os
addr = '169.254.98.2'
while True:
	for port in range(14999,15001):
		srx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
		if not hasattr(socket, 'SO_BINDTODEVICE'):
			socket.SO_BINDTODEVICE = 25
		srx.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, 'enxe4b97a92cc38'+'\0')
		try:
			srx.bind((addr,port))
			print 'RX bind to port succeeded.', srx.getsockname()
		except socket.error as msg:
			print 'RX bind to port failed: ' + msg[1]
		srx.settimeout(1)
		try:
			data_received = srx.recv(4096)
		except socket.error, e:
			print e
		srx.close()
