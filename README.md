# introduction

The repo contains several components:

* Web Interface of Neuronbot.
* RMT Agent.

# Install necessary packages
```
sudo apt install libnm-dev libglib2.0-dev
```

# Build

* Make sure you've already installed rmt_library

* Download

```bash
cd ~
git clone https://github.com/Adlink-ROS/RMT_example.git
```

* Build Agent

```bash
cd ~/RMT_example/agent
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
