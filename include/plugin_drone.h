#ifndef ARDRONE_SIMPLE_CONTROL_H
#define ARDRONE_SIMPLE_CONTROL_H

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/Events.hh"
#include "ignition/math4/ignition/math.hh"
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

#include <sensor_msgs/Imu.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>

#include <mascot/Gain.h>
#include <mascot/Force.h>

#include "pid_controller.h"

#define LANDED_MODEL        0
#define FLYING_MODEL        1
#define TAKINGOFF_MODEL     2
#define LANDING_MODEL       3

#define EPS 1E-6
namespace gazebo
{
class DroneSimpleController : public ModelPlugin
{
public:
  DroneSimpleController();
  virtual ~DroneSimpleController();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void LoadControllerSettings(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Update();
  void UpdateDynamics(double dt);
  void UpdateState(double dt);
  virtual void Reset();
  double updatepid(double new_input, double x, double dx, double dt);


private:
  double m_timeAfterCmd;
  bool m_posCtrl;
  bool m_consenCtrl;
  bool m_velMode;
  unsigned int navi_state;
  
  
  /// \brief The parent World
  physics::WorldPtr world;

  /// \brief The link referred to by this plugin
  physics::LinkPtr link;

  ros::NodeHandle* node_handle_;
  ros::CallbackQueue callback_queue_;
  ros::Subscriber cmd_subscriber_;
  ros::Subscriber cmd_position_subscriber_;
  ros::Subscriber posctrl_subscriber_;
  ros::Subscriber consenctrl_subscriber_;
  ros::Subscriber cmd_force_subscriber_;
  ros::Subscriber loadPID_subscriber_;
  ros::Subscriber imu_subscriber_;
  
  // extra robot control command
  ros::Subscriber takeoff_subscriber_;
  ros::Subscriber land_subscriber_;
  ros::Subscriber reset_subscriber_;
  ros::Subscriber switch_mode_subscriber_;
  
  ros::Publisher pub_gt_pose_;   //for publishing ground truth pose
  ros::Publisher pub_gt_vec_;   //ground truth velocity in the body frame
  ros::Publisher pub_gt_acc_;   //ground truth acceleration in the body frame


  geometry_msgs::Twist cmd_val;
  mascot::Gain gains;
  mascot::Force cmd_force;
  geometry_msgs::Point cmd_position;
  // callback functions for subscribers
  void CmdCallback(const geometry_msgs::TwistConstPtr&);
  void CmdForceCallback(const mascot::ForceConstPtr&);
  void CmdPositionCallback(const geometry_msgs::PointConstPtr&);
  void PosCtrlCallback(const std_msgs::BoolConstPtr&);
  void ConsenCtrlCallback(const std_msgs::BoolConstPtr&);
  void LoadPIDCallback(const mascot::GainConstPtr&);
  void ImuCallback(const sensor_msgs::ImuConstPtr&);
  void TakeoffCallback(const std_msgs::EmptyConstPtr&);
  void LandCallback(const std_msgs::EmptyConstPtr&);
  void ResetCallback(const std_msgs::EmptyConstPtr&);
  void SwitchModeCallback(const std_msgs::BoolConstPtr&);
 
 
  ros::Time state_stamp;
  ignition::math::Pose3d pose;
  ignition::math::Vector3d euler, velocity, acceleration, angular_velocity, position;

  std::string link_name_;
  std::string cmd_normal_topic_;
  std::string switch_mode_topic_;
  std::string posctrl_topic_;
  std::string consenctrl_topic_;
  std::string cmd_force_topic_;
  std::string cmd_position_topic_;
  std::string imu_topic_;
  std::string loadPID_topic_;
  std::string pubpos_topic_;
  std::string pubvel_topic_;
  std::string pubacc_topic_;
  std::string takeoff_topic_;
  std::string land_topic_;
  std::string reset_topic_;
    
  double max_force_;
  double thrust;
  double set_height=0;
  double motion_small_noise_;
  double motion_drift_noise_;
  double motion_drift_noise_time_;


// Remove this after tunning
  double my_gain_p = 1.1;
  double my_gain_i = 0.0;
  double my_gain_d = 0.3;
  double my_time_constant = 0.0;
  double my_limit = -1.0;

  double my_input;
  double my_dinput;
  double my_output;
  double my_p, my_i, my_d;

  struct Controllers {
    PIDController roll;
    PIDController pitch;
    PIDController yaw;
    PIDController velocity_x;
    PIDController velocity_y;
    PIDController velocity_z;
    PIDController pos_x;
    PIDController pos_y;
    PIDController pos_z;
  } controllers_;

  ignition::math::Vector3d inertia;
  double mass;

  /// \brief save last_time
  common::Time last_time;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;
};

}

#endif // ARDRONE_SIMPLE_CONTROL_H
