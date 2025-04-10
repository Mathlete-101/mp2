#!/bin/bash

arduino_cli compile --fqbn arduino:avr:mega ardu
arduino_cli upload -p --fqbn arduino:avr:mega /dev/ttyACM0 ardu --verbose