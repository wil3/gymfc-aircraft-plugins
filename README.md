# Gazebo Aircraft Plugins for GymFC

These plugins are a port from a subset of PX4's
[sitl_gazebo](https://github.com/PX4/sitl_gazebo) project to be compatible with
GymFC v0.2.0+ and are intended to be a more accurate model for multirotor aircraft in
comparison to Gazebo's [Lift and Drag](http://gazebosim.org/tutorials?tut=aerodynamics&cat=plugins) plugin.

Changes from PX4:

* Implements GymFC API
* Allow reset for iterative training/testing

## Pre-requisites
* cmake 3+
* Google protobuf
* Gazebo

## Build

1) From root, `mkdir build`
2) `cd build`
3) `cmake ..`
4) `make`

## Configure

## Additional References

* [Documentation: What is the math behind the Motor Model?](https://github.com/PX4/sitl_gazebo/issues/110)
* [PX4 motor models notes](https://github.com/mvernacc/gazebo_motor_model_docs/blob/master/notes.pdf)
* [SDF Format](http://sdformat.org/spec)
