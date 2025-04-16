#!/bin/bash

# Use the full path to arduino-cli
/usr/local/bin/arduino-cli compile --fqbn arduino:avr:mega ardu
/usr/local/bin/arduino-cli upload -p --fqbn arduino:avr:mega /dev/ttyACM0 ardu --verbose