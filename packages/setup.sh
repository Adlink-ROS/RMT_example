#!/bin/bash

if [ "$1" = "install" ]; then
    echo "Install the RMT agent"
    sudo dpkg -i ./*.deb
elif [ "$1" = "uninstall" ]; then
    echo "Remove the RMT agent"
    sudo dpkg -r rmt_library
    sudo dpkg -r rmt_agent
else
    echo "Usage: ./agent_install.sh [command]"
    echo "command:"
    echo " - install: Install the RMT agent"
    echo " - uninstall: Remove the RMT agent"
fi
