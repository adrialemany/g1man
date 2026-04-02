# G1 Pilot: Wireless Operation via Zenoh Bridge

This guide documents how to run the g1pilot stack over a wireless network, bypassing the requirement for a physical Ethernet connection to the Unitree G1.

## Overview

The G1 internal architecture uses two PCs. PC1 handles low-level motor control and broadcasts data via Multicast over an internal Ethernet bridge. Since Wi-Fi routers typically block or degrade Multicast traffic, standard ROS 2 discovery fails. 

We solve this by using **Zenoh** as a transparent Unicast tunnel between the robot's accessible PC (PC2) and your local workstation.

## Prerequisites

- **Robot Side:** Zenoh Bridge DDS (aarch64/ARM64 version for the internal Jetson).
- **Workstation Side:** Zenoh Bridge DDS (Official g1pilot docker image recommended).

---

## 1. Robot Configuration (PC2)

Connect to the robot via SSH and run the Zenoh bridge. This instance will act as the "server" (endpoint) on the robot's side.

```bash
# Clean existing DDS configurations
unset CYCLONEDDS_URI

# Download and run the ARM64 standalone bridge
wget https://github.com/eclipse-zenoh/zenoh-plugin-dds/releases/download/1.8.0/zenoh-plugin-dds-1.8.0-aarch64-unknown-linux-gnu-standalone.zip
python3 -m zipfile -e zenoh-plugin-dds-1.8.0-aarch64-unknown-linux-gnu-standalone.zip .
chmod +x zenoh-bridge-dds

# Start the bridge listening on port 7447
./zenoh-bridge-dds -e tcp/0.0.0.0:7447
```

## 2. Workstation Configuration
Run the Zenoh bridge on your laptop. This instance will connect to the robot and inject the remote topics into your local ROS 2 graph.

```bash
# Use the official Docker image to handle dependencies automatically
docker run --init --rm --net host eclipse/zenoh-bridge-dds:1.8.0 --connect tcp/192.168.0.107:7447
```

## 3. Running g1pilot
Once the bridges are connected, you can launch the g1pilot stack inside your local Docker container. The stack will perceive the bridged topics as local traffic.

```bash
# Enter your g1pilot Docker environment
sh run.sh

# Inside the container:
unset CYCLONEDDS_URI
export ROS_DOMAIN_ID=0
export G1_INTERFACE=wlo1  # Set to your laptop's wireless interface

# Launch the bringup
ros2 launch g1pilot bringup_launcher.launch.py
```

---

Still working on how to move the motors from the station. Working on it. Will try Motion 2 or this repo: https://github.com/sharan05032000/Unitree-G1-MoveIt2-Arm-Manipulation/tree/main
