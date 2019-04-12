#!/usr/bin/env python
import socket
import subprocess
import sys
from datetime import datetime
import os

# Clear the screen
subprocess.call('clear', shell=True)

# Ask for input
# remoteServer    = raw_input("Enter a remote host to scan: ")
remoteServerIP  = '169.254.98.1'
OBC_PORT = 15000
MCU_PORT = 15001
MCU_SRC_PORT = 14999

# Print a nice banner with information on which host we are about to scan
print "-" * 60
print "Please wait, scanning remote host", remoteServerIP
print "-" * 60

# Check what time the scan started
t1 = datetime.now()

# Using the range function to specify ports (here it will scans all ports between 1 and 1024)

# We also put in some error handling for catching errors
winner = -1
try:
	for port in range(15000,15001):
		sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
		result = sock.connect_ex((remoteServerIP, port))
		sock.setblocking(1)
		# sock.settimeout(5)
		if result == 0:
			print "Port {}: 	 Open".format(port)
			try:
				data_received = sock.recv(4096)
				winner = port
				print(data_received)
			except socket.error, e:
				err = e.args[0]
				# print "RX error "
				# print os.strerror(err)
		sock.close()
	print winner

except KeyboardInterrupt:
	print "You pressed Ctrl+C"
	sys.exit()

except socket.gaierror:
	print 'Hostname could not be resolved. Exiting'
	sys.exit()

except socket.error:
	print "Couldn't connect to server"
	sys.exit()

# Checking the time again
t2 = datetime.now()

# Calculates the difference of time, to see how long it took to run the script
total =  t2 - t1

# Printing the information to screen
print 'Scanning Completed in: ', total
