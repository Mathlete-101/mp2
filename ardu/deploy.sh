#!/bin/bash

# Try both possible paths to arduino-cli
ARDUINO_CLI_PATHS=(
    "/home/moonpie/bin/arduino-cli"
    "/usr/local/bin/arduino-cli"
)

# Find the first working path
ARDUINO_CLI=""
for path in "${ARDUINO_CLI_PATHS[@]}"; do
    if [ -x "$path" ]; then
        ARDUINO_CLI="$path"
        break
    fi
done

# Check if we found a working path
if [ -z "$ARDUINO_CLI" ]; then
    echo "Error: arduino-cli not found in any of the following locations:"
    for path in "${ARDUINO_CLI_PATHS[@]}"; do
        echo "  $path"
    done
    echo "Please ensure arduino-cli is installed and in your PATH"
    exit 1
fi

echo "Using arduino-cli at: $ARDUINO_CLI"

# Compile and upload
$ARDUINO_CLI compile --fqbn arduino:avr:mega ardu
$ARDUINO_CLI upload -p --fqbn arduino:avr:mega /dev/ttyACM0 ardu --verbose