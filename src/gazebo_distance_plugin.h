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

#include <random>

#include "Distance.pb.h"
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "common.h"
#include <ignition/math.hh>

#include <Eigen/Core>


namespace ground_distance_units{
    static const std::string METERS="m";    
}

namespace gazebo {
//typedef const boost::shared_ptr<const sensor_msgs::msgs::Distance> Distanceptr;

// Default values for use with ADIS16448 IMU
static constexpr double kDefaultAdisMaxRange=20.0;

static const std::string kDefaultDistanceTopic = "Distance";

struct DistanceParameters {
  /// Norm of the gravitational acceleration [m/s^2]
  double Max_Range;

  DistanceParameters()
      : 
        Max_Range(kDefaultAdisMaxRange) {}
};

class GazeboDistancePlugin : public ModelPlugin {
 public:

  GazeboDistancePlugin()
      : ModelPlugin(),
      ground_distance_W_(0.0),
      ground_distance_units_(ground_distance_units::METERS){
      }
  ~GazeboDistancePlugin();

  void InitializeParams();
  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  void OnUpdate(const common::UpdateInfo&);

  void OnTimeReset();
 private:
  std::string namespace_;
  std::string distance_topic_;
  transport::NodePtr node_handle_;
  transport::PublisherPtr distance_pub_;
  std::string frame_id_;
  std::string link_name_;
  std::string ground_distance_units_;

  std::default_random_engine random_generator_;
  std::normal_distribution<double> standard_normal_distribution_;

  // Pointer to the world
  physics::WorldPtr world_;
  // Pointer to the model
  physics::ModelPtr model_;
  // Pointer to the link
  physics::LinkPtr link_;
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection_;
  event::ConnectionPtr resetEvent_;

  common::Time last_time_;

  sensor_msgs::msgs::Distance distance_message_;
  ignition::math::Temperature ground_distance_W_;
  DistanceParameters distance_parameters_;
  uint64_t seq_ = 0;
};
}
