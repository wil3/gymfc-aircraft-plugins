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


namespace ground_distance_units {
    static const std::string METERS="m";
    }
namespace gazebo {
//typedef const boost::shared_ptr<const sensor_msgs::msgs::Distance> Distanceptr;
static constexpr double kDefaultAdisMaxRange=20.0;
static const std::string kDefaultDistanceTopic = "Distance";

struct DistanceParameters {
  double Max_Range;
  DistanceParameters()
      : Max_Range(kDefaultAdisMaxRange) {}
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
