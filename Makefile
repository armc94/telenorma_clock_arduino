BOARD?=arduino:avr:uno
PORT?=/dev/ttyUSB0

.PHONY: default lint all flash clean

default: all flash

all:
	arduino-cli compile --fqbn $(BOARD) ./

flash:
	arduino-cli upload --fqbn $(BOARD) --port $(PORT)

clean:
	rm -r build

monitor:
	sudo python ../mbed_serial/main.py
