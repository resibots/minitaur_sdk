
import socket
import os
addr = '169.254.98.1'
port=15000
iface = 'enxe4b97a92cc38'
# Open TX socket
stx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
# Enable broadcast
try:
	stx.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
except socket.error as msg:
	print 'TX set broadcast failed: ' + msg[1]
# Bind to network interface
# try:
# 	if not hasattr(socket, 'SO_BINDTODEVICE'):
# 		socket.SO_BINDTODEVICE = 25
# 	stx.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, iface+'\0')
# 	print 'TX bind to ' + iface + ' succeeded.'
# except socket.error as msg:
# 	print 'TX bind to ' + iface + ' failed: ' + msg[1]
result = stx.connect_ex((addr, port))
print result
# stx.bind((addr,port))
stx.settimeout(10)
while True:
	try:
		data_received, addr_received = stx.recvfrom(4096)
	except socket.error, e:
		print "yes"
		print e
