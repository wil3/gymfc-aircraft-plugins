# This is a fork from Gazebo Aircraft Plugins for GymFC by Wil Koch, including new Distance Sensor Plugin

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

## Current make results
```
cuda@cuda:~/workspace/aircraft-plugins-new/build$ make
[ 62%] Built target sensor_msgs
[ 75%] Built target control_msgs
[ 83%] Built target gazebo_motor_model
[ 91%] Built target gazebo_imu_plugin
[ 95%] Building CXX object CMakeFiles/gazebo_distance_plugin.dir/src/gazebo_distance_plugin.cpp.o
In file included from /home/cuda/workspace/aircraft-plugins-new/src/gazebo_distance_plugin.cpp:21:0:
/home/cuda/workspace/aircraft-plugins-new/src/gazebo_distance_plugin.h: In constructor ‘gazebo::GazeboDistancePlugin::GazeboDistancePlugin()’:
/home/cuda/workspace/aircraft-plugins-new/src/gazebo_distance_plugin.h:63:58: error: no matching function for call to ‘ignition::math::v4::Vector3<double>::Vector3(int)’
       ground_distance_units_(ground_distance_units::METER){
                                                          ^
In file included from /usr/include/ignition/math4/ignition/math/Spline.hh:23:0,
                 from /usr/local/include/gazebo-10/gazebo/common/Animation.hh:22,
                 from /usr/local/include/gazebo-10/gazebo/common/common.hh:5,
                 from /home/cuda/workspace/aircraft-plugins-new/src/gazebo_distance_plugin.h:23,
                 from /home/cuda/workspace/aircraft-plugins-new/src/gazebo_distance_plugin.cpp:21:
/usr/include/ignition/math4/ignition/math/Vector3.hh:77:15: note: candidate: ignition::math::v4::Vector3<T>::Vector3(const ignition::math::v4::Vector3<T>&) [with T = double]
       public: Vector3(const Vector3<T> &_v)
               ^~~~~~~
/usr/include/ignition/math4/ignition/math/Vector3.hh:77:15: note:   no known conversion for argument 1 from ‘int’ to ‘const ignition::math::v4::Vector3<double>&’
/usr/include/ignition/math4/ignition/math/Vector3.hh:68:15: note: candidate: ignition::math::v4::Vector3<T>::Vector3(const T&, const T&, const T&) [with T = double]
       public: Vector3(const T &_x, const T &_y, const T &_z)
               ^~~~~~~
/usr/include/ignition/math4/ignition/math/Vector3.hh:68:15: note:   candidate expects 3 arguments, 1 provided
/usr/include/ignition/math4/ignition/math/Vector3.hh:57:15: note: candidate: ignition::math::v4::Vector3<T>::Vector3() [with T = double]
       public: Vector3()
               ^~~~~~~
/usr/include/ignition/math4/ignition/math/Vector3.hh:57:15: note:   candidate expects 0 arguments, 1 provided
/home/cuda/workspace/aircraft-plugins-new/src/gazebo_distance_plugin.cpp: In member function ‘void gazebo::GazeboDistancePlugin::OnUpdate(const gazebo::common::UpdateInfo&)’:
/home/cuda/workspace/aircraft-plugins-new/src/gazebo_distance_plugin.cpp:145:27: error: missing template arguments before ‘position’
   ignition::math::Vector3 position = T_W_I.Pos();
                           ^~~~~~~~
/home/cuda/workspace/aircraft-plugins-new/src/gazebo_distance_plugin.cpp:147:3: error: ‘height’ was not declared in this scope
   height = set_h(position.Z())
   ^~~~~~
/home/cuda/workspace/aircraft-plugins-new/src/gazebo_distance_plugin.cpp:147:3: note: suggested alternative: ‘ceilh’
   height = set_h(position.Z())
   ^~~~~~
   ceilh
/home/cuda/workspace/aircraft-plugins-new/src/gazebo_distance_plugin.cpp:147:18: error: ‘position’ was not declared in this scope
   height = set_h(position.Z())
                  ^~~~~~~~
/home/cuda/workspace/aircraft-plugins-new/src/gazebo_distance_plugin.cpp:147:18: note: suggested alternative: ‘copyPosition’
   height = set_h(position.Z())
                  ^~~~~~~~
                  copyPosition
/home/cuda/workspace/aircraft-plugins-new/src/gazebo_distance_plugin.cpp:147:12: error: ‘set_h’ was not declared in this scope
   height = set_h(position.Z())
            ^~~~~
/home/cuda/workspace/aircraft-plugins-new/src/gazebo_distance_plugin.cpp:147:12: note: suggested alternative: ‘seq_’
   height = set_h(position.Z())
            ^~~~~
            seq_
/home/cuda/workspace/aircraft-plugins-new/src/gazebo_distance_plugin.cpp:160:21: error: ‘class sensor_msgs::msgs::Distance’ has no member named ‘set_seq’
   distance_message_.set_seq(seq_++);
                     ^~~~~~~
CMakeFiles/gazebo_distance_plugin.dir/build.make:62: recipe for target 'CMakeFiles/gazebo_distance_plugin.dir/src/gazebo_distance_plugin.cpp.o' failed
make[2]: *** [CMakeFiles/gazebo_distance_plugin.dir/src/gazebo_distance_plugin.cpp.o] Error 1
CMakeFiles/Makefile2:218: recipe for target 'CMakeFiles/gazebo_distance_plugin.dir/all' failed
make[1]: *** [CMakeFiles/gazebo_distance_plugin.dir/all] Error 2
Makefile:83: recipe for target 'all' failed
make: *** [all] Error 2
```

