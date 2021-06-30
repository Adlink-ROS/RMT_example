# Introduction

rmt-agent can run on ADLINK ROScube series and communicate to server with RMT library.

# Install necessary packages

```bash
sudo apt install libnm-dev libglib2.0-dev makeself
# If you want to use ROS, please also install ROS 2 foxy
```

# Build

* Clone the repo

```bash
cd ~
git clone https://github.com/Adlink-ROS/rmt-agent.git
cd ~/rmt-agent
```

* Install necessary packages

```bash
# install Neuron Library & RMT Library
cd ~/rmt-agent/agent
sudo apt install ./packages/*.deb
```

* Build Agent

```bash
# If you want to use ROS, run "cmake -Bbuild -H. -DUSE_ROS=ON" instead
cmake -Bbuild -H.
cmake --build build
# If you want to build deb file, run the command and find deb file in build folder
cmake --build build --target package
# Create run file (including rmt_library, rmt_agent, Neuron Library)
cmake --build build --target makeself
```

# Usage

## Run directly

We can run the rmt-agent directly after building.

```bash
cd ~/rmt-agent/agent/build
sudo ./rmt-agent --id <your_id>
```

## Run with installation

We can also install rmt-agent, and it'll run automatically after reboot.

* Install rmt-library

```bash
cd ~/rmt-agent/agent/build
./rmt_agent_<version>_<arch>.run install
```

* Run / stop / show status of rmt-agent

```bash
# Run rmt-agent
systemctl --user start rmt-agent.service
# Stop rmt-agent
systemctl --user stop rmt-agent.service
# Show the status
systemctl --user status rmt-agent.service
```

* Uninstall rmt-library

```bash
cd ~/rmt-agent/agent/build
./rmt_agent_<version>_<arch>.run uninstall
```
