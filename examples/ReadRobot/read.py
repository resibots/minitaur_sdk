import struct
import serial

# Open USB port
port = serial.Serial('/dev/tty.usbserial-DN02MSZX', 230400, timeout=1)
print("USB: " + port.name)

while(True):

    # Read alignment word
    ALIGN_SIZE = 4
    bytes = port.read(ALIGN_SIZE)
    if(len(bytes) == ALIGN_SIZE):
        unpacked = struct.unpack( '<I', bytes)
        if(unpacked[0] == 0xFFFFFFFF):

            # Read struct
            STRUCT_SIZE = 8
            bytes = port.read(STRUCT_SIZE)
            if(len(bytes) == STRUCT_SIZE):
                unpacked = struct.unpack( '<IHH', bytes)
                print("%d\t%d\t%d" % (unpacked[0], unpacked[1], unpacked[1]))

# Close
port.close()


