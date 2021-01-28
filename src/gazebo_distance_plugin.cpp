/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "gazebo_distance_plugin.h"
//#include "gazebo_imu_plugin.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <Eigen/Eigen> //test

#include <boost/bind.hpp>

namespace gazebo {

    /*  
GazeboImuPlugin::GazeboImuPlugin()
    : ModelPlugin(),
      ground_distance_W_(0)
{
    
}
*/
GazeboDistancePlugin::~GazeboDistancePlugin() {
  updateConnection_->~Connection();
}

void GazeboDistancePlugin::OnTimeReset()
{
  #if GAZEBO_MAJOR_VERSION >= 9
  last_time_ = world_->SimTime();
  #else
  last_time_ = world_->GetSimTime();
  #endif

}

void GazeboDistancePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  gzdbg << "Loading DISTANCE  sensor\n";
  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();

  // default params
  namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_distance_plugin] Please specify a robotNamespace.\n";
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_distance_plugin] Please specify a linkName.\n";
  // Get the pointer to the link
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_distance_plugin] Couldn't find specified link \"" << link_name_ << "\".");

  frame_id_ = link_name_;

  getSdfParam<std::string>(_sdf, "distanceTopic", distance_topic_, kDefaultDistanceTopic);
  
  getSdfParam<double>(_sdf, "MaxRange",
                      distance_parameters_.Max_Range,
                      distance_parameters_.Max_Range);

  getSdfParam<std::string>(_sdf, "groundDistanceUnit", ground_distance_units_,
                           ground_distance_units_);

  #if GAZEBO_MAJOR_VERSION >= 9
  last_time_ = world_->SimTime();
  #else
  last_time_ = world_->GetSimTime();
  #endif

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboDistancePlugin::OnUpdate, this, _1));

  this->resetEvent_ = 
      event::Events::ConnectTimeReset(
      //   (height);
     boost::bind(&GazeboDistancePlugin::OnTimeReset, this));

  // Specify queue limit and rate, make this configurable
  distance_pub_ = node_handle_->Advertise<sensor_msgs::msgs::Distance>("/aircraft/sensor/distance");







  // Fill imu message.
  // imu_message_.header.frame_id = frame_id_; TODO Add header
  // We assume uncorrelated noise on the 3 channels -> only set diagonal
  // elements. Only the broadband noise component is considered, specified as a
  // continuous-time density (two-sided spectrum); not the true covariance of
  // the measurements.
  // Angular velocity measurement covariance.

}

// This gets called by the world update start event.
void GazeboDistancePlugin::OnUpdate(const common::UpdateInfo& _info) {
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time  = world_->SimTime();
#else
  common::Time current_time  = world_->GetSimTime();
#endif
  double dt = (current_time - last_time_).Double();
  last_time_ = current_time;
  double t = current_time.Double();

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d T_W_I = link_->WorldPose(); //TODO(burrimi): Check tf.
#else
  ignition::math::Pose3d T_W_I = ignitionFromGazeboMath(link_->GetWorldPose()); //TODO(burrimi): Check tf.
#endif

  ignition::math::Vector3d position = T_W_I.Pos();
  
  float height = position.Z();

  // Fill IMU message.
  // ADD HEaders
  // imu_message_.header.stamp.sec = current_time.sec;
  // imu_message_.header.stamp.nsec = current_time.nsec;

  // TODO(burrimi): Add orientation estimator.
  // imu_message_.orientation.w = 1;
  // imu_message_.orientation.x = 0;
  // imu_message_.orientation.y = 0;
  // imu_message_.orientation.z = 0;
  distance_message_.set_time_usec(_info.simTime.sec * 1000000 + _info.simTime.nsec / 1000);
  //distance_message_.set_seq(seq_++);


  distance_message_.set_ground_distance(height);
  //gzdbg << "Publishing IMU message\n";
  distance_pub_->Publish(distance_message_);
}


GZ_REGISTER_MODEL_PLUGIN(GazeboDistancePlugin);
}
