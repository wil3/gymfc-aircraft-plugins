#include "gazebo_distance_plugin.h"
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <Eigen/Eigen> //test

#include <boost/bind.hpp>
namespace gazebo
{
  /*  
GazeboImuPlugin::GazeboImuPlugin()
    : ModelPlugin(),
      ground_distance_W_(0)
{
    
}
*/
  GazeboDistancePlugin::~GazeboDistancePlugin()
  {updateConnection_->~Connection();}

void GazeboDistancePlugin::OnTimeReset()
{
  #if GAZEBO_MAJOR_VERSION >= 9
    last_time_ = world_->SimTime();
  #else
    last_time_ = world_->GetSimTime();
  #endif
}

void GazeboDistancePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
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

  getSdfParam<std::string>(_sdf,"distanceTopic", distance_topic_, kDefaultDistanceTopic);

  getSdfParam<double>(_sdf,"MaxRange",
                      distance_parameters_.Max_Range,
                      distance_parameters_.Max_Range);

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
          boost::bind(&GazeboDistancePlugin::OnTimeReset, this));

    
  distance_pub_ = node_handle_->Advertise<sensor_msgs::msgs::Distance>(distance_topic_);
  }

  // This gets called by the world update start event.
void GazeboDistancePlugin::OnUpdate(const common::UpdateInfo &_info){
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = world_->SimTime();
#else
  common::Time current_time = world_->GetSimTime();
#endif

  double dt = (current_time - last_time_).Double();
  last_time_ = current_time;
  double t = current_time.Double();

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d T_W_I = link_->WorldPose(); //TODO(burrimi): Check tf.
#else
  ignition::math::Pose3d T_W_I = ignitionFromGazeboMath(link_->GetWorldPose());//TODO(burrimi): Check tf.
#endif
  ignition::math::Vector3d position = T_W_I.Pos();
  float height = position.Z();
  #if height <= MaxRange
    height = height;
  #else
    height = MaxRange;
  #endif
  distance_message_.set_ground_distance(height);
  distance_pub_->Publish(distance_message_);
}
  GZ_REGISTER_MODEL_PLUGIN(GazeboDistancePlugin);
} // namespace gazebo
