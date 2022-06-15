# README

This README will walk the reader through the steps to setup the ROS workspace for this project and how to Launch the Car.
This tutorial is intended for unix-based terminals.

It is assumed that ROS-melodic is installed on the Jetson.

SSH into the Jetson or connect keyboard, mouse and monitor to the Jetson and open a terminal in the the Linux GUI and follow these steps:

## ROS Workspace Setup

1. Clone this repository into a folder on the Jetson

```console
$ cd ~/sandbox (or whatever folder you want to work in)
$ git clone https://github.com/jepp5220/f1tenth.git
```

2. Create workspace

```console
$ mkdir -p f1tenth_ws/src
$ cp -r f1tenth_system f1tenth_ws/src/
```

3. Install additional ROS packages

```console
$ sudo apt-get update
$ sudo apt-get install ros-melodic-driver-base
```

4. Make Python scripts executeable

```console
$ cd f1tenth_ws
$ find . -name “*.py” -exec chmod +x {} \;
```

5. Move to workspace folder and compile the code 

```console
$ catkin_make
```

6. Source directory

```console
$ source devel/setup.bash
```

Now we are ready to launch the car.

# Launching the Car



1. Start roscore

```console
$ roscore
```

2. In another terminal, move to workspace folder and source the bash file as such:

```console
$ source devel/setup.bash
```

3. To launch the car, use the following command:

```console
$ roslaunch racecar drive.launch
```

Now we can activate the different kinds of control.

For external control, run these three Python scripts in seperate terminals:

```console
$ python3 f1tenthv2/f1tenth_ws/src/f1tenth_system/racecar/racecar/
    scripts/lidarMQ.py

$ python3 f1tenthv2/f1tenth_ws/src/f1tenth_system/racecar/racecar/
    scripts/odomMQ.py

$ python3 f1tenthv2/f1tenth_ws/src/f1tenth_system/racecar/racecar/
    scripts/recieveinputMQ.py
```
Here it is noted that the external controller also needs to comsume and send input to actually activate the external control.

For Manual control, run this Python script in a seperate terminal:

```console
$ python3 f1tenthv2/f1tenth_ws/src/f1tenth_system/racecar/racecar/
    scripts/manual_controller.py
```

For Pure Pursuit control, run this Python script in a seperate terminal:

```console
$ python3 f1tenthv2/f1tenth_ws/src/f1tenth_system/racecar/racecar/
    scripts/purepursuit.py
```
