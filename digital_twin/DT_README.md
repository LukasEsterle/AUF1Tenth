# BSc-DT-F1-10
This is a digital twin for the f1tenth car project. With simulations in INTO-CPS

## Demo
https://youtu.be/-DEOV0oWJdw

## Installation
On a Ubuntu 20.04 machine the following tings are needed:

- Python 3
- ROS
- RabbitMQ
- INTO-CPS

Then for using the code in this repository the following instructions has to be followed

- Copy the f1tenth_simulator_modified folder into your catkin workspace and build
- Copy the ros_nodes into your catkin workspace. Fx the f1tenth_simulator_modified folder
- Then copy the f1tenth_intoCPS into your into cps project folder

## Usage
For running the digital twin the following steps has to be done

Frist terminal
```bash
…\catkin_ws $ roslaunch f1tenth_simulator simulator.launch
```

Second terminal
```bash
…\ros_nodes $ python3 lidar_sensor.py
```

Third terminal
```bash
…\ros_nodes $ python3 lidar_data_processer.py
```

Fourth terminal
```bash
…\ros_nodes $ python3 odom_sensor.py
```

Fifth terminal
```bash
…\ros_nodes $ python3 moter_actuator.py
```

Then go to INTO CPS and open the f1tenth_intoCPS project. Here it will now be possible to run the digital twin setup from there cosim tab
