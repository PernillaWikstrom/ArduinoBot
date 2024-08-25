# ArduinoBot

This is the project developed along the Udemy course Robotics and ROS - Learn by Doing! Manipulators with https://github.com/AntoBrandi/Robotics-and-ROS-Learn-by-Doing-Manipulators.

## Installation

Follow installation guide on ROS official website (https://wiki.ros.org/noetic/Installation/Ubuntu)
and then install the following ROS packages:

```bash
sudo apt-get install ros-noetic-ros-controllers
sudo apt-get install ros-noetic-gazebo-ros-control
sudo apt-get install ros-noetic-rosserial
sudo apt-get install ros-noetic-rosserial-arduino
sudo apt-get install ros-noetic-roboticsgroup-upatras-gazebo-plugins
sudo apt-get install ros-noetic-moveit
sudo apt-get install ros-noetic-actionlib-tools
```

and Python dependencies:

```bash
sudo apt install python3-pip
pip install flask
pip install flask-ask-sdk
pip install ask-sdk
```

Install ngrok (https://ngrok.com/download), create and setup an account

```bash
cd ~/Downloads/ngrok-v3-stable-linux-amd64
./ngrok authtoken <YOUR-TOKEN>
```

## Build

```bash
cd ~/ArduinoBot
catkin_make
```

## Launch the simulation

```bash
source devel/setup.bash
roslaunch arudionobot_bringup sim_complete.launch
```

Start the ngrok web server

```bash
ngrok http <your port>
```

add the link provided under 'Forwarding' in the section endpoint of your Alexa development account.
