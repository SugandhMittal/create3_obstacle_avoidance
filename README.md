# Create3 Obstacle Avoidance

This repository can be used as a node to enable the Create3 to successfully avoid all obstacles and respond appropriately to encountered obstacles or when it gets stuck. 

## Demonstration


https://github.com/user-attachments/assets/142f4feb-4c23-4438-b3a5-172ff66e6273




## Setup
To start working, simply follow the steps below:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/SugandhMittal/create3_obstacle_avoidance.git
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/local_setup.sh
```
Now, in the terminal just type

```bash
ros2 run create3_obstacle_avoidance reactive_controller
```



**If there are any issues/updates, I would be happy to address them. You can post them in the issues section**


