#!/bin/bash
# Authors: Tom Jacobs, Avik De

# Print Help
echo ""
echo "Capture network packets on selected interface, saving to new file every 30 seconds. Run with sudo. Devices:"
echo "Put the index of the favorite interface after -i in the tshark command in this script"
echo "In the future someone could check if `Ethernet` is a substring of the interface name"

unameOut="$(uname -s)"
case "${unameOut}" in
    Linux*)     machine=Linux;;
    Darwin*)    machine=Mac;;
    *)          machine=Win
esac

if [ $machine != Win ]
then 
  # Stop any previously running tshark
  sudo pkill -9 tshark
  sudo pkill -9 dumpcap
fi

# Move old files
mkdir -p old/
mv -f *.pcap old/
chmod 666 old/
chmod 666 old/trial*.pcap

# List devices
tshark -D

# Start capture
echo ""
echo "Starting..."
echo ""
for i in `seq 1 1000`; do

  # Run tshark for 30 seconds
  echo "Trial $i. Press any key and then wait until trial finishes to stop."
  tshark -i 10 -a duration:10 -w trial${i}.pcap
  chmod 666 trial${i}.pcap

  # Read keyboard buffer
  if read -r -n 1 -t 1; then
    break
  fi

done 

# Stop tshark
echo "Stopping tshark."
if [ $machine != Win ]
then 
  sudo pkill -9 tshark
  sudo pkill -9 dumpcap
fi
