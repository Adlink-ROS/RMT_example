# introduction

The repo contains several components:

* Web Interface of Neuronbot.
* RMT Agent.

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
```

* Run

```bash
cd ~/RMT_example/agent/build
sudo ./agent_example --id <your_id>
```
