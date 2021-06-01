# introduction

The repo contains several components:

* Web Interface of Neuronbot.
* RMT Agent.

# Install necessary packages

```bash
sudo apt install libnm-dev libglib2.0-dev
# If you want to use ROS, please also install ROS 2 foxy
```

# Build

* Clone the repo

```bash
cd ~
git clone https://github.com/Adlink-ROS/RMT_example.git
```

* Install necessary packages: Neuron Library, RMT Library

```bash
cd ~/RMT_example/agent
sudo apt install ./packages/*.deb
```

* Build Agent

```bash
# If you don't want to use ROS, run "cmake -Bbuild -H. -DUSE_ROS=off" instead
cmake -Bbuild -H.
cmake --build build
# If you want to build deb file, run the command and find deb file in build folder
cmake --build build --target package
```

* Run

```bash
cd ~/RMT_example/agent/build
sudo ./agent_example --id <your_id>
```
