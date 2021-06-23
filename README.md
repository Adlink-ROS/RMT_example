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

* Install necessary packages

```bash
sudo apt install libnm-dev makeself
# install Neuron Library & RMT Library
cd ~/RMT_example/agent
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
cd ~/RMT_example/agent/build
sudo ./agent-example --id <your_id>
```
