#include "plugin_drone.h"

#include <cmath>
#include <stdlib.h>
#include <iostream>

namespace gazebo
{

  DroneSimpleController::DroneSimpleController()
  {
    navi_state = LANDED_MODEL;
    m_posCtrl = false;
    m_consenCtrl = false;
    m_velMode = false;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  DroneSimpleController::~DroneSimpleController()
  {
    // Deprecated since Gazebo 8.
    // event::Events::DisconnectWorldUpdateBegin(updateConnection);

    node_handle_->shutdown();
    delete node_handle_;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Load the controller
  void DroneSimpleController::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {

    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                       << "Load the Gazebo system plugin 'libgazebo_ros_init.so' in the gazebo_ros package)");
    }

    world = _model->GetWorld();
    ROS_INFO("The drone plugin is loading!");

    // load parameters
    cmd_normal_topic_ = "cmd_vel";
    cmd_force_topic_ = "cmd_force";
    cmd_position_topic_ = "cmd_positon";
    takeoff_topic_ = "drone/takeoff";
    land_topic_ = "drone/land";
    reset_topic_ = "drone/reset";
    posctrl_topic_ = "drone/posctrl";
    pubvel_topic_ = "drone/gt_vel";
    pubacc_topic_ = "drone/gt_acc";
    pubpos_topic_ = "drone/gt_pose";
    switch_mode_topic_ = "drone/vel_mode";
    consenctrl_topic_ = "drone/consenctrl";
    loadPID_topic_ = "drone/loadPID";

    if (!_sdf->HasElement("pubposTopic"))
      pubpos_topic_.clear();
    else
      pubpos_topic_ = _sdf->GetElement("pubposTopic")->Get<std::string>();

    if (!_sdf->HasElement("pubvelTopic"))
      pubvel_topic_.clear();
    else
      pubvel_topic_ = _sdf->GetElement("pubvelTopic")->Get<std::string>();

    if (!_sdf->HasElement("pubaccTopic"))
      pubacc_topic_.clear();
    else
      pubacc_topic_ = _sdf->GetElement("pubaccTopic")->Get<std::string>();

    if (!_sdf->HasElement("loadPIDTopic"))
      loadPID_topic_.clear();
    else
      loadPID_topic_ = _sdf->GetElement("loadPIDTopic")->Get<std::string>();

    if (!_sdf->HasElement("resetTopic"))
      reset_topic_.clear();
    else
      reset_topic_ = _sdf->GetElement("resetTopic")->Get<std::string>();

    if (!_sdf->HasElement("consenTopic"))
      consenctrl_topic_.clear();
    else
      consenctrl_topic_ = _sdf->GetElement("consenTopic")->Get<std::string>();

    if (!_sdf->HasElement("imuTopic"))
      imu_topic_.clear();
    else
      imu_topic_ = _sdf->GetElement("imuTopic")->Get<std::string>();

    if (!_sdf->HasElement("takeoffTopic"))
      takeoff_topic_.clear();
    else
      takeoff_topic_ = _sdf->GetElement("takeoffTopic")->Get<std::string>();

    if (!_sdf->HasElement("landTopic"))
      land_topic_.clear();
    else
      land_topic_ = _sdf->GetElement("landTopic")->Get<std::string>();

    if (!_sdf->HasElement("posctrlTopic"))
      posctrl_topic_.clear();
    else
      posctrl_topic_ = _sdf->GetElement("posctrlTopic")->Get<std::string>();

    if (!_sdf->HasElement("cmdvelTopic"))
      cmd_normal_topic_.clear();
    else
      cmd_normal_topic_ = _sdf->GetElement("cmdvelTopic")->Get<std::string>();

    if (!_sdf->HasElement("cmdforceTopic"))
      cmd_force_topic_.clear();
    else
      cmd_force_topic_ = _sdf->GetElement("cmdforceTopic")->Get<std::string>();

    if (!_sdf->HasElement("cmdpositionTopic"))
      cmd_position_topic_.clear();
    else
      cmd_position_topic_ = _sdf->GetElement("cmdpositionTopic")->Get<std::string>();

    if (!_sdf->HasElement("bodyName"))
    {
      link = _model->GetLink();
      link_name_ = link->GetName();
    }
    else
    {
      link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
      link = boost::dynamic_pointer_cast<physics::Link>(world->EntityByName(link_name_));
    }

    if (!link)
    {
      ROS_FATAL("gazebo_ros_baro plugin error: bodyName: %s does not exist\n", link_name_.c_str());
      return;
    }

    if (!_sdf->HasElement("maxForce"))
      max_force_ = -1;
    else
      max_force_ = _sdf->GetElement("maxForce")->Get<double>();

    if (!_sdf->HasElement("motionSmallNoise"))
      motion_small_noise_ = 0;
    else
      motion_small_noise_ = _sdf->GetElement("motionSmallNoise")->Get<double>();

    if (!_sdf->HasElement("motionDriftNoise"))
      motion_drift_noise_ = 0;
    else
      motion_drift_noise_ = _sdf->GetElement("motionDriftNoise")->Get<double>();

    if (!_sdf->HasElement("motionDriftNoiseTime"))
      motion_drift_noise_time_ = 1.0;
    else
      motion_drift_noise_time_ = _sdf->GetElement("motionDriftNoiseTime")->Get<double>();

    // get inertia and mass of quadrotor body
    inertia = link->GetInertial()->PrincipalMoments();
    mass = link->GetInertial()->Mass();

    node_handle_ = new ros::NodeHandle;

    // subscribe command: control command
    if (!cmd_normal_topic_.empty())
    {
      ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::Twist>(
          cmd_normal_topic_, 1,
          boost::bind(&DroneSimpleController::CmdCallback, this, _1),
          ros::VoidPtr(), &callback_queue_);
      cmd_subscriber_ = node_handle_->subscribe(ops);

      if (cmd_subscriber_.getTopic() != "")
        ROS_INFO_NAMED("quadrotor_simple_controller", "Using cmd_topic %s.", cmd_normal_topic_.c_str());
      else
        ROS_INFO("cannot find the command topic!");
    }

    if (!cmd_force_topic_.empty())
    {
      ros::SubscribeOptions ops = ros::SubscribeOptions::create<mascot::Force>(
          cmd_force_topic_, 1,
          boost::bind(&DroneSimpleController::CmdForceCallback, this, _1),
          ros::VoidPtr(), &callback_queue_);
      cmd_force_subscriber_ = node_handle_->subscribe(ops);

      if (cmd_force_subscriber_.getTopic() != "")
        ROS_INFO_NAMED("quadrotor_simple_controller", "Using cmd_force_topic %s.", cmd_force_topic_.c_str());
      else
        ROS_INFO("cannot find the command force topic!");
    }

    if (!cmd_position_topic_.empty())
    {
      ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::Point>(
          cmd_position_topic_, 1,
          boost::bind(&DroneSimpleController::CmdPositionCallback, this, _1),
          ros::VoidPtr(), &callback_queue_);
      cmd_position_subscriber_ = node_handle_->subscribe(ops);

      if (cmd_position_subscriber_.getTopic() != "")
        ROS_INFO_NAMED("quadrotor_simple_controller", "Using cmd_position_topic %s.", cmd_position_topic_.c_str());
      else
        ROS_INFO("cannot find the command position topic!");
    }

    if (!posctrl_topic_.empty())
    {
      ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Bool>(
          posctrl_topic_, 1,
          boost::bind(&DroneSimpleController::PosCtrlCallback, this, _1),
          ros::VoidPtr(), &callback_queue_);
      posctrl_subscriber_ = node_handle_->subscribe(ops);

      if (posctrl_subscriber_.getTopic() != "")
        ROS_INFO("find the position control topic!");
      else
        ROS_INFO("cannot find the position control topic!");
    }

    if (!consenctrl_topic_.empty())
    {
      ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Bool>(
          consenctrl_topic_, 1,
          boost::bind(&DroneSimpleController::ConsenCtrlCallback, this, _1),
          ros::VoidPtr(), &callback_queue_);
      consenctrl_subscriber_ = node_handle_->subscribe(ops);

      if (consenctrl_subscriber_.getTopic() != "")
        ROS_INFO("find the Consensu control topic!");
      else
        ROS_INFO("cannot find the Consensus control topic!");
    }

    if (!loadPID_topic_.empty())
    {
      ros::SubscribeOptions ops = ros::SubscribeOptions::create<mascot::Gain>(
          loadPID_topic_, 1,
          boost::bind(&DroneSimpleController::LoadPIDCallback, this, _1),
          ros::VoidPtr(), &callback_queue_);
      loadPID_subscriber_ = node_handle_->subscribe(ops);

      if (loadPID_subscriber_.getTopic() != "")
        ROS_INFO("find the load PID topic!");
      else
        ROS_INFO("cannot find the load PID topic!");
    }

    // subscribe imu
    if (!imu_topic_.empty())
    {
      ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::Imu>(
          imu_topic_, 1,
          boost::bind(&DroneSimpleController::ImuCallback, this, _1),
          ros::VoidPtr(), &callback_queue_);
      imu_subscriber_ = node_handle_->subscribe(ops);

      if (imu_subscriber_.getTopic() != "")
        ROS_INFO_NAMED("quadrotor_simple_controller", "Using imu information on topic %s as source of orientation and angular velocity.", imu_topic_.c_str());
      else
        ROS_INFO("cannot find the IMU topic!");
    }

    // subscribe command: take off command
    if (!takeoff_topic_.empty())
    {
      ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Empty>(
          takeoff_topic_, 1,
          boost::bind(&DroneSimpleController::TakeoffCallback, this, _1),
          ros::VoidPtr(), &callback_queue_);
      takeoff_subscriber_ = node_handle_->subscribe(ops);
      if (takeoff_subscriber_.getTopic() != "")
        ROS_INFO("find the takeoff topic");
      else
        ROS_INFO("cannot find the takeoff topic!");
    }

    // subscribe command: land command
    if (!land_topic_.empty())
    {
      ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Empty>(
          land_topic_, 1,
          boost::bind(&DroneSimpleController::LandCallback, this, _1),
          ros::VoidPtr(), &callback_queue_);
      land_subscriber_ = node_handle_->subscribe(ops);
    }

    // subscribe command: reset command
    if (!reset_topic_.empty())
    {
      ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Empty>(
          reset_topic_, 1,
          boost::bind(&DroneSimpleController::ResetCallback, this, _1),
          ros::VoidPtr(), &callback_queue_);
      reset_subscriber_ = node_handle_->subscribe(ops);
    }

    if (!pubpos_topic_.empty())
    {
      pub_gt_pose_ = node_handle_->advertise<geometry_msgs::Pose>(pubpos_topic_, 1024);
    }

    pub_gt_vec_ = node_handle_->advertise<geometry_msgs::Twist>(pubvel_topic_, 1024);
    pub_gt_acc_ = node_handle_->advertise<geometry_msgs::Twist>(pubacc_topic_, 1024);

    if (!switch_mode_topic_.empty())
    {
      ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Bool>(
          switch_mode_topic_, 1,
          boost::bind(&DroneSimpleController::SwitchModeCallback, this, _1),
          ros::VoidPtr(), &callback_queue_);
      switch_mode_subscriber_ = node_handle_->subscribe(ops);
    }

    LoadControllerSettings(_model, _sdf);

    Reset();

    // New Mechanism for Updating every World Cycle
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&DroneSimpleController::Update, this));
  }

  void DroneSimpleController::LoadControllerSettings(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    controllers_.roll.Load(_sdf, "rollpitch");
    controllers_.pitch.Load(_sdf, "rollpitch");
    controllers_.yaw.Load(_sdf, "yaw");
    controllers_.velocity_x.Load(_sdf, "velocityXY");
    controllers_.velocity_y.Load(_sdf, "velocityXY");
    controllers_.velocity_z.Load(_sdf, "velocityZ");

    controllers_.pos_x.Load(_sdf, "positionXY");
    controllers_.pos_y.Load(_sdf, "positionXY");
    controllers_.pos_z.Load(_sdf, "positionZ");
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Callbacks
  void DroneSimpleController::CmdCallback(const geometry_msgs::TwistConstPtr &cmd)
  {
    cmd_val = *cmd;

    static common::Time last_sim_time = world->SimTime();
    static double time_counter_for_drift_noise = 0;
    static double drift_noise[4] = {0.0, 0.0, 0.0, 0.0};
    // Get simulator time
    common::Time cur_sim_time = world->SimTime();
    double dt = (cur_sim_time - last_sim_time).Double();
    // save last time stamp
    last_sim_time = cur_sim_time;

    // generate noise
    if (time_counter_for_drift_noise > motion_drift_noise_time_)
    {
      drift_noise[0] = 2 * motion_drift_noise_ * (drand48() - 0.5);
      drift_noise[1] = 2 * motion_drift_noise_ * (drand48() - 0.5);
      drift_noise[2] = 2 * motion_drift_noise_ * (drand48() - 0.5);
      drift_noise[3] = 2 * motion_drift_noise_ * (drand48() - 0.5);
      time_counter_for_drift_noise = 0.0;
    }
    time_counter_for_drift_noise += dt;

    cmd_val.angular.x += drift_noise[0] + 2 * motion_small_noise_ * (drand48() - 0.5);
    cmd_val.angular.y += drift_noise[1] + 2 * motion_small_noise_ * (drand48() - 0.5);
    cmd_val.angular.z += drift_noise[3] + 2 * motion_small_noise_ * (drand48() - 0.5);
    cmd_val.linear.z += drift_noise[2] + 2 * motion_small_noise_ * (drand48() - 0.5);
  }

  void DroneSimpleController::CmdForceCallback(const mascot::ForceConstPtr &msg)
  {
    cmd_force = *msg;
  }

  void DroneSimpleController::CmdPositionCallback(const geometry_msgs::PointConstPtr &msg)
  {
    cmd_position = *msg;
  }

  void DroneSimpleController::PosCtrlCallback(const std_msgs::BoolConstPtr &cmd)
  {
    m_posCtrl = cmd->data;
    if (m_posCtrl)
    {
      m_consenCtrl = false;
      std::cout << "Position Control!" << std::endl;
    }
  }

  void DroneSimpleController::ConsenCtrlCallback(const std_msgs::BoolConstPtr &cmd)
  {
    m_consenCtrl = cmd->data;
    if (m_consenCtrl)
    {
      m_posCtrl = false;
      std::cout << " Control Initiated !" << std::endl;
    }
  }

  void DroneSimpleController::LoadPIDCallback(const mascot::Gain::ConstPtr &msg)
  {
    gains = *msg;
    my_gain_p = gains.k_p;
    my_gain_d = gains.k_d;
    my_gain_i = gains.k_i;
    // controllers_.velocity_x.Load(sdf, "velocityXY");
    // controllers_.velocity_y.Load(sdf, "velocityXY");
    // std::cout << "Loaded New PID Gains " << std::endl;
  }

  void DroneSimpleController::ImuCallback(const sensor_msgs::ImuConstPtr &imu)
  {
    // directly read the quternion from the IMU data
    pose.Rot().Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
    euler = pose.Rot().Euler();
    angular_velocity = pose.Rot().RotateVector(ignition::math::Vector3d(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z));
  }

  void DroneSimpleController::TakeoffCallback(const std_msgs::EmptyConstPtr &msg)
  {
    if (navi_state == LANDED_MODEL)
    {
      navi_state = TAKINGOFF_MODEL;
      m_timeAfterCmd = 0;
      ROS_INFO("%s", "\nQuadrotor takes off!!");
    }
  }

  void DroneSimpleController::LandCallback(const std_msgs::EmptyConstPtr &msg)
  {
    if (navi_state == FLYING_MODEL || navi_state == TAKINGOFF_MODEL)
    {
      navi_state = LANDING_MODEL;
      m_timeAfterCmd = 0;
      ROS_INFO("%s", "\nQuadrotor lands!!");
    }
  }

  void DroneSimpleController::ResetCallback(const std_msgs::EmptyConstPtr &msg)
  {
    ROS_INFO("%s", "\nReset quadrotor!!");
    m_posCtrl = true;
    cmd_val.linear.x = 0;
    cmd_val.linear.y = 0;
    cmd_val.linear.z = 0;

    Reset();
  }

  void DroneSimpleController::SwitchModeCallback(const std_msgs::BoolConstPtr &msg)
  {
    m_velMode = msg->data;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Update the controller
  void DroneSimpleController::Update()
  {

    // Get new commands/state
    callback_queue_.callAvailable();

    // Get simulator time
    common::Time sim_time = world->SimTime();
    double dt = (sim_time - last_time).Double();
    if (dt == 0.0)
      return;

    UpdateState(dt);
    UpdateDynamics(dt);

    // save last time stamp
    last_time = sim_time;
  }

  void DroneSimpleController::UpdateState(double dt)
  {
    if (navi_state == TAKINGOFF_MODEL)
    {
      m_timeAfterCmd += dt;
      if (m_timeAfterCmd > 0.5)
      {
        navi_state = FLYING_MODEL;
        std::cout << "Entering flying model!" << std::endl;
      }
    }
    else if (navi_state == LANDING_MODEL)
    {
      m_timeAfterCmd += dt;
      if (m_timeAfterCmd > 1.0)
      {
        navi_state = LANDED_MODEL;
        std::cout << "Landed!" << std::endl;
      }
    }
    else
      m_timeAfterCmd = 0;
  }

  void DroneSimpleController::UpdateDynamics(double dt)
  {
    ignition::math::Vector3d force, torque;

    // Get Pose/Orientation from Gazebo (if no state subscriber is active)
    //  if (imu_subscriber_.getTopic()=="")
    {
      pose = link->WorldPose();
      angular_velocity = link->WorldAngularVel();
      euler = pose.Rot().Euler();
    }
    // if (state_topic_.empty())
    {
      acceleration = (link->WorldLinearVel() - velocity) / dt;
      velocity = link->WorldLinearVel();
    }

    // publish the ground truth pose of the drone to the ROS topic
    geometry_msgs::Pose gt_pose;
    gt_pose.position.x = pose.Pos().X();
    gt_pose.position.y = pose.Pos().Y();
    gt_pose.position.z = pose.Pos().Z();

    gt_pose.orientation.w = pose.Rot().W();
    gt_pose.orientation.x = pose.Rot().X();
    gt_pose.orientation.y = pose.Rot().Y();
    gt_pose.orientation.z = pose.Rot().Z();
    pub_gt_pose_.publish(gt_pose);

    // convert the acceleration and velocity into the body frame
    // ignition::math::Vector3d body_vel = pose.Rot().RotateVector(velocity);
    // ignition::math::Vector3d body_acc = pose.Rot().RotateVector(acceleration);

    // publish the velocity in earth frame
    geometry_msgs::Twist tw;
    tw.linear.x = velocity.X();
    tw.linear.y = velocity.Y();
    tw.linear.z = velocity.Z();
    pub_gt_vec_.publish(tw);

    // publish the acceleration in earth frame
    tw.linear.x = acceleration.X();
    tw.linear.y = acceleration.Y();
    tw.linear.z = acceleration.Z();
    pub_gt_acc_.publish(tw);

    ignition::math::Vector3d poschange = pose.Pos() - position;
    position = pose.Pos();

    // Get gravity
    ignition::math::Vector3d gravity_body = pose.Rot().RotateVector(world->Gravity());
    double gravity = gravity_body.Length();
    double load_factor = gravity * gravity / world->Gravity().Dot(gravity_body); // Get gravity

    // Rotate vectors to coordinate frames relevant for control
    ignition::math::Quaterniond heading_quaternion(cos(euler.Z() / 2), 0, 0, sin(euler.Z() / 2));
    ignition::math::Vector3d velocity_xy = heading_quaternion.RotateVectorReverse(velocity);
    ignition::math::Vector3d acceleration_xy = heading_quaternion.RotateVectorReverse(acceleration);
    ignition::math::Vector3d angular_velocity_body = pose.Rot().RotateVectorReverse(angular_velocity);

    // update controllers
    force.Set(0.0, 0.0, 0.0);
    torque.Set(0.0, 0.0, 0.0);

    if (m_posCtrl)
    {

      double set_pos_x = cmd_position.x;
      double set_pos_y = cmd_position.y;
      double set_pos_z = cmd_position.z;

      // position control
      if (navi_state == FLYING_MODEL)
      {
        double vx = controllers_.pos_x.update(set_pos_x, position.X(), poschange.X(), dt);
        double vy = controllers_.pos_y.update(set_pos_y, position.Y(), poschange.Y(), dt);
        double vz = controllers_.pos_z.update(set_pos_z, position.Z(), poschange.Z(), dt);
        
        // std::cout << vx << " , "<< position.X()<<" , " << vy << " , "<< position.X()<< std::endl;

        ignition::math::Vector3d vb = heading_quaternion.RotateVectorReverse(ignition::math::Vector3d(vx, vy, vz));

        double pitch_command = controllers_.velocity_x.update(vb.X(), velocity_xy.X(), acceleration_xy.X(), dt) / gravity;
        double roll_command = -controllers_.velocity_y.update(vb.Y(), velocity_xy.Y(), acceleration_xy.Y(), dt) / gravity;

        torque.X() = inertia.X() * controllers_.roll.update(roll_command, euler.X(), angular_velocity_body.X(), dt);
        torque.Y() = inertia.Y() * controllers_.pitch.update(pitch_command, euler.Y(), angular_velocity_body.Y(), dt);
        torque.Z() = inertia.Z() * controllers_.yaw.update(cmd_val.angular.z, euler.Z(), angular_velocity_body.Z(), dt);
        force.Z() = mass * (controllers_.velocity_z.update(vz, velocity.Z(), acceleration.Z(), dt) + load_factor * gravity);

        // std::cout << pitch_command << "," << roll_command << std::endl;
        if (force.Z() >= 1)
          thrust = force.Z();
          
      }
    }
    else if (m_consenCtrl)
    {

      // if (navi_state == FLYING_MODEL)
      // {
      // double force_x = cmd_force.force.x;
      // double force_y = cmd_force.force.y;
      // double force_z = cmd_force.force.z;

      // double pitch_command = atan(force_x / (mass * gravity));
      // double roll_command = -atan(force_y / (mass * gravity));

      // torque.X() = inertia.X() * controllers_.roll.update(roll_command, euler.X(), angular_velocity_body.X(), dt);
      // torque.Y() = inertia.Y() * controllers_.pitch.update(pitch_command, euler.Y(), angular_velocity_body.Y(), dt);
      // torque.Z() = inertia.Z() * controllers_.yaw.update(cmd_val.angular.z, euler.Z(), angular_velocity_body.Z(), dt);

      // double vz = controllers_.pos_z.update(set_height, position.Z(), poschange.Z(), dt);
      // force.Z() = mass * (controllers_.velocity_z.update(vz, velocity.Z(), acceleration.Z(), dt) + load_factor * gravity);
      // }

      double set_pos_x = cmd_position.x;
      double set_pos_y = cmd_position.y;
      double set_pos_z = cmd_position.z;

      // consen control
      if (navi_state == FLYING_MODEL)
      {
        double force_x = cmd_force.force.x;
        double force_y = cmd_force.force.y;
        // double force_x = my_gain_d*(my_gain_p*(set_pos_x - position.X())-velocity_xy.X());
        // double force_y = my_gain_d*(my_gain_p*(set_pos_y - position.Y())-velocity_xy.Y());

        double vz = controllers_.pos_z.update(set_pos_z, position.Z(), poschange.Z(), dt);

        ignition::math::Vector3d accel = heading_quaternion.RotateVectorReverse(ignition::math::Vector3d(force_x,force_y,0)); // converting to body frame

        double pitch_command = (mass/thrust)*accel.X();
        double roll_command = -(mass/thrust)*accel.Y();
        

        // double pitch_command = atan(force_x / (mass * gravity));
        // double roll_command = -atan(force_y / (mass * gravity));

        // ignition::math::Vector3d force_b = heading_quaternion.RotateVectorReverse(ignition::math::Vector3d(force_x,force_y,force_z));
        // double pitch_command = my_i*atan(force_b.X()/(mass*gravity));
        // double roll_command = -my_i*atan(force_b.Y()/(mass*gravity));

        torque.X() = inertia.X() * controllers_.roll.update(roll_command, euler.X(), angular_velocity_body.X(), dt);
        torque.Y() = inertia.Y() * controllers_.pitch.update(pitch_command, euler.Y(), angular_velocity_body.Y(), dt);
        torque.Z() = inertia.Z() * controllers_.yaw.update(cmd_val.angular.z, euler.Z(), angular_velocity_body.Z(), dt);
        force.Z() = mass * (controllers_.velocity_z.update(vz, velocity.Z(), acceleration.Z(), dt) + load_factor * gravity);

        if (force.Z() >= 1)
          thrust = force.Z();

        // std::cout << force_x << "," << force_y << "," << pitch_command << "," << roll_command << "," << (set_pos_x - position.X()) << "," << (set_pos_y - position.Y()) << std::endl;
      }
    }
    else
    {
      // normal control
      if (navi_state == FLYING_MODEL) //&& cmd_val.linear.x >= 0 && cmd_val.linear.y >= 0)
      {
        // hovering
        double pitch_command = controllers_.velocity_x.update(cmd_val.linear.x, velocity_xy.X(), acceleration_xy.X(), dt) / gravity;
        double roll_command = -controllers_.velocity_y.update(cmd_val.linear.y, velocity_xy.Y(), acceleration_xy.Y(), dt) / gravity;
        torque.X() = inertia.X() * controllers_.roll.update(roll_command, euler.X(), angular_velocity_body.X(), dt);
        torque.Y() = inertia.Y() * controllers_.pitch.update(pitch_command, euler.Y(), angular_velocity_body.Y(), dt);
      }
      else
      {
        // control by velocity
        if (m_velMode)
        {
          double pitch_command = controllers_.velocity_x.update(cmd_val.angular.x, velocity_xy.X(), velocity_xy.X(), dt) / gravity;
          double roll_command = -controllers_.velocity_y.update(cmd_val.angular.y, velocity_xy.Y(), velocity_xy.Y(), dt) / gravity;
          torque.X() = inertia.X() * controllers_.roll.update(roll_command, euler.X(), angular_velocity_body.X(), dt);
          torque.Y() = inertia.Y() * controllers_.pitch.update(pitch_command, euler.Y(), angular_velocity_body.Y(), dt);
        }
        else
        {
          // control by tilting
          torque.X() = inertia.X() * controllers_.roll.update(cmd_val.angular.x, euler.X(), angular_velocity_body.X(), dt);
          torque.Y() = inertia.Y() * controllers_.pitch.update(cmd_val.angular.y, euler.Y(), angular_velocity_body.Y(), dt);
        }
      }
      torque.Z() = inertia.Z() * controllers_.yaw.update(cmd_val.angular.z, angular_velocity.Z(), 0, dt);
      force.Z() = mass * (controllers_.velocity_z.update(cmd_val.linear.z, velocity.Z(), acceleration.Z(), dt) + load_factor * gravity);
    }

    if (max_force_ > 0.0 && force.Z() > max_force_)
      force.Z() = max_force_;
    if (force.Z() < 0.0)
      force.Z() = 0.0;
    // process robot state information

    if (navi_state == FLYING_MODEL)
    {
      link->AddRelativeForce(force);
      link->AddRelativeTorque(torque);
    }
    else if (navi_state == TAKINGOFF_MODEL)
    {
      link->AddRelativeForce(force * 1.5);
      link->AddRelativeTorque(torque * 1.5);
    }
    else if (navi_state == LANDING_MODEL)
    {
      link->AddRelativeForce(force * 0.8);
      link->AddRelativeTorque(torque * 0.8);
    }
  }
  ////////////////////////////////////////////////////////////////////////////////
  // Reset the controller
  void DroneSimpleController::Reset()
  {
    std::cout << "Reset is called" << std::endl;
    controllers_.roll.reset();
    controllers_.pitch.reset();
    controllers_.yaw.reset();
    controllers_.velocity_x.reset();
    controllers_.velocity_y.reset();
    controllers_.velocity_z.reset();

    link->SetForce(ignition::math::Vector3d(0, 0, 0));
    link->SetTorque(ignition::math::Vector3d(0, 0, 0));

    // reset state
    pose.Reset();
    velocity.Set();
    angular_velocity.Set();
    acceleration.Set();
    euler.Set();
    state_stamp = ros::Time();
  }

  /// Remmove this after tunning

  double DroneSimpleController::updatepid(double new_input, double x, double dx, double dt)
  {
    // limit command
    if (my_limit > 0.0 && fabs(new_input) > my_limit)
      new_input = (new_input < 0 ? -1.0 : 1.0) * my_limit;

    // filter command
    if (dt + my_time_constant > 0.0)
    {
      my_input = (dt * new_input + my_time_constant * my_input) / (dt + my_time_constant);
      my_dinput = (new_input - my_input) / (dt + my_time_constant);
    }

    // update proportional, differential and integral errors
    my_p = my_input - x;
    my_d = my_dinput - dx;
    my_i = my_i + dt * my_p;

    // update control output
    my_output = my_gain_p * my_p + my_gain_d * my_d + my_gain_i * 0;
    return my_output;
  }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(DroneSimpleController)

} // namespace gazebo
