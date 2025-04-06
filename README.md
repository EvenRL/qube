# Qube
This repository is for the Mini-project assignment in subject AIS2105.  The repository contains packages for controlling and simulating a Quanser Qube.

## qube_description
This package is contains urdf and launch files to visualize a qube using Rviz2.

![quberviz](/doc/screenshots/rviz.png)

## qube_bringup
This package contains a launch file for connecting the qube visualization in rviz2 with either a simulation or a real qube so the qube can be controlled with a controller.

The launch file has the following launch parameters:

- `baud_rate` - Baud-rate for serial communication to arduino default: 9600

- `device` - Device name for serial connection, default: COM1

- `simulation` - Enable or disable simulation, default: false

- `joint_state_gui` - Enable or disable the joint_state_publisher gui, default: false


## qube_controller
This package contains a PID controller node for controller the position of a qube. It subcribes to the /joint_states topic to get position and velocity information and publishes commadns to the /velocity_controller/commands topic.

The controllers parameters `Kp`, `Ki`, `Kd` as well as `setpoint` can easily be changed using the command line or from a gui using rqt_reconfigure as shown below. There are also additional parameters, `max_velocity`, to limit the velocity of the qube, and `joint_name`, to change the name of the urdf joint to control.

![rqtreconfigure](/doc/screenshots/rqtreconfiugre.png)

## Dependencies
The package qube_bringup requires the [qube_driver](https://github.com/adamleon/qube_driver) package installed to run the `bringup.launch.py` launch file.
