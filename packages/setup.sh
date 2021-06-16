#!/bin/bash

if [ "$1" = "install" ]; then
    echo "Install the RMT agent"
    sudo apt update
    sudo apt install -y ./*.deb
elif [ "$1" = "uninstall" ]; then
    echo "Remove the RMT agent"
    sudo apt remove rmt_library
    sudo apt remove rmt_agent
else
    echo "Usage: ./agent_install.sh [command]"
    echo "command:"
    echo " - install: Install the RMT agent"
    echo " - uninstall: Remove the RMT agent"
fi
