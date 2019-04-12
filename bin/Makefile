
PORT=COM39

SDK_LOCATION=..

TARGET=mbm_relay
ROBOT=MINITAUR_E

# ---

BINFILE=$(TARGET)_$(ROBOT).bin

all:
	@python $(SDK_LOCATION)/tools/stm32loader.py -p $(PORT) -b 230400 -y mblc -E 524288 -L 0 -ew $(BINFILE)


