#! /bin/bash

# Check if arduino-cli is already installed
if command -v arduino-cli &> /dev/null; then
    echo "arduino-cli is already installed"
    arduino-cli version
else
    echo "Installing arduino-cli..."

    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
    echo 'export PATH=$HOME/bin:$PATH" >> ~/.bashrc
    source ~/.bashrc

fi 
arduino-cli config init
arduino-cli core update-index
arduino-cli core install arduino:avr

# install Encoder, "Cytron Motor Driver Library", "ArduinoJson"
arduino-cli lib install Encoder
arduino-cli lib install "Cytron Motor Drivers Library"
arduino-cli lib install ArduinoJson
arduino-cli lib install Streaming

