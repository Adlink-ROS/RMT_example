# Introduction

rmt-agent can run on ROScube and be controlled by RMT library.

rmt-agent is also an example of how to use RMT library.

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
# Create run file (including rmt_library, rmt_agent, mraa)
cmake --build build --target makeself
```

* Run

```bash
cd ~/rmt-agent/agent/build
sudo ./rmt-agent --id <your_id>
```

# Usage

* After packaging, we can install with the following commands

```bash
./build/rmt_agent_<version>_<arch>.run install
```

* Enable / Disable rmt-agent

```bash
systemctl --user enable rmt-agent.service
systemctl --user disable rmt-agent.service
```
