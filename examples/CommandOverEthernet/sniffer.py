import socket
stx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
while True:
    print stx.recvfrom(4096)
