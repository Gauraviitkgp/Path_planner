#include "rpg_mpc/mpc_controller.h"

#include <ctime>

namespace rpg_mpc 
{

template <typename T>
  MpcController<T>::MpcController(
    const ros::NodeHandle & nh, const ros::NodeHandle & pnh) :
  nh_(nh),
  pnh_(pnh),
  mpc_wrapper_(MpcWrapper<T>()),
  timing_feedback_(T(1e-3)),
  timing_preparation_(T(1e-3)),
  est_state_((Eigen::Matrix<T, kStateSize, 1>() <<
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0).finished()),
  reference_states_(Eigen::Matrix<T, kStateSize, kSamples+1>::Zero()),
  reference_inputs_(Eigen::Matrix<T, kInputSize, kSamples+1>::Zero()),
  predicted_states_(Eigen::Matrix<T, kStateSize, kSamples+1>::Zero()),
  predicted_inputs_(Eigen::Matrix<T, kInputSize, kSamples>::Zero()),
  point_of_interest_(Eigen::Matrix<T, 3, 1>::Zero())
  {
    pub_predicted_trajectory_ =
    nh_.advertise<nav_msgs::Path>("mpc/trajectory_predicted", 1);

    //Not essential
    sub_point_of_interest_ = nh_.subscribe("mpc/point_of_interest", 1,
      &MpcController<T>::pointOfInterestCallback, this);

    //Parameter Loading from mpc_params.h
    if(!params_.loadParameters(pnh_))
    {
      ROS_ERROR("[%s] Could not load parameters.", pnh_.getNamespace().c_str());
      ros::shutdown();
      return;
    }
    //Setting up the params and the object in mpc_controller.h
    setNewParams(params_);

    //start the preparation thread
    preparation_thread_ = std::thread(&MpcWrapper<T>::prepare, mpc_wrapper_);
  }

  //Setting up point of intrest
template <typename T>
  void MpcController<T>::pointOfInterestCallback(
    const geometry_msgs::PointStamped::ConstPtr& msg)
  {
    point_of_interest_(0) = msg->point.x;
    point_of_interest_(1) = msg->point.y;
    point_of_interest_(2) = msg->point.z;
    mpc_wrapper_.setPointOfInterest(point_of_interest_);
  }

  //Maybe setting up thrust to 0
template <typename T>
  quadrotor_common::ControlCommand MpcController<T>::off()
  {
    quadrotor_common::ControlCommand command;

    command.zero();

    return command;
  }

  //most important
  //state estimate and reference trajectory shall be given by hedwig.
template <typename T>
  quadrotor_common::ControlCommand MpcController<T>::run(
    const quadrotor_common::QuadStateEstimate& state_estimate,
    const quadrotor_common::Trajectory& reference_trajectory,
    const MpcParams<T>& params)
  {
    //Starting time
    ros::Time call_time = ros::Time::now();
    const clock_t start = clock();

    //Changing the parameters(we maybe not changing)
    if(params.changed_)
    {
      params_ = params;
      setNewParams(params_);
    }

    //waits for thread to finish
    preparation_thread_.join();

    //gets the state_estimate and trajectory and convert it into eigen format
    // Convert everything into Eigen format.
    setStateEstimate(state_estimate);
    setReference(reference_trajectory);

    static const bool do_preparation_step(false);

    //After running the entire code we are waiting for updates and inputs
    // Get the feedback from MPC.
    mpc_wrapper_.setTrajectory(reference_states_, reference_inputs_);
    mpc_wrapper_.update(est_state_, do_preparation_step);
    mpc_wrapper_.getStates(predicted_states_);
    mpc_wrapper_.getInputs(predicted_inputs_);

    // Publish the predicted trajectory.
    publishPrediction(predicted_states_, predicted_inputs_, call_time);

    // Start a thread to prepare for the next execution.
    preparation_thread_ = std::thread(&MpcController<T>::preparationThread, this);

    // Timing
    const clock_t end = clock();
    timing_feedback_ = 0.9*timing_feedback_ +
    0.1* double(end - start)/CLOCKS_PER_SEC;
    if(params_.print_info_)
      ROS_INFO_THROTTLE(1.0, "MPC Timing: Latency: %1.1f ms  |  Total: %1.1f ms",
        timing_feedback_*1000, (timing_feedback_+timing_preparation_)*1000);

    // Return the input control command.
    return updateControlCommand(predicted_states_.col(0),
      predicted_inputs_.col(0),
      call_time);
  }

  //Setting state estimation from hedwig
template <typename T>
  bool MpcController<T>::setStateEstimate(
    const quadrotor_common::QuadStateEstimate& state_estimate)
  {
    est_state_(kPosX) = state_estimate.position.x();
    est_state_(kPosY) = state_estimate.position.y();
    est_state_(kPosZ) = state_estimate.position.z();
    est_state_(kOriW) = state_estimate.orientation.w();
    est_state_(kOriX) = state_estimate.orientation.x();
    est_state_(kOriY) = state_estimate.orientation.y();
    est_state_(kOriZ) = state_estimate.orientation.z();
    est_state_(kVelX) = state_estimate.velocity.x();
    est_state_(kVelY) = state_estimate.velocity.y();
    est_state_(kVelZ) = state_estimate.velocity.z();
    const bool quaternion_norm_ok = abs(est_state_.segment(kOriW, 4).norm()-1.0)<0.1;
    return quaternion_norm_ok;
  }

  //Setting reference from hedwig
template <typename T>
  bool MpcController<T>::setReference(
    const quadrotor_common::Trajectory& reference_trajectory)
  {
    //initialize by 0
    reference_states_.setZero();
    reference_inputs_.setZero();

    const T dt = mpc_wrapper_.getTimestep();
    Eigen::Matrix<T, 3, 1> acceleration;
    const Eigen::Matrix<T, 3, 1> gravity(0.0, 0.0, -9.81);
    Eigen::Quaternion<T> q_heading;
    Eigen::Quaternion<T> q_orientation;
    bool quaternion_norm_ok(true);
    //if the reference trajectory has just one point(maybe endpoint)
    if(reference_trajectory.points.size() == 1)
    {
      q_heading = Eigen::Quaternion<T>(Eigen::AngleAxis<T>(
        reference_trajectory.points.front().heading,
        Eigen::Matrix<T,3,1>::UnitZ()));
      q_orientation = q_heading *
      reference_trajectory.points.front().orientation.template cast<T>();
      reference_states_ = (Eigen::Matrix<T, kStateSize, 1>() <<
        reference_trajectory.points.front().position.template cast<T>(),
        q_orientation.w(),
        q_orientation.x(),
        q_orientation.y(),
        q_orientation.z(),
        reference_trajectory.points.front().velocity.template cast<T>()
        ).finished().replicate(1, kSamples+1);

      acceleration <<
      reference_trajectory.points.front().acceleration.template cast<T>()
      - gravity;
      reference_inputs_ = (Eigen::Matrix<T, kInputSize, 1>() <<
        acceleration.norm(),
        reference_trajectory.points.front().bodyrates.template cast<T>()
        ).finished().replicate(1, kSamples+1);
    }
    else
    {
      //for more than one points
      //making a list outof trajectory points
      std::list<quadrotor_common::TrajectoryPoint>::const_iterator iterator(
        reference_trajectory.points.begin());
      for(int i=0; i<kSamples+1; i++)
      {
        //need time from start
        while(iterator->time_from_start.toSec() < i*dt &&
          iterator!=reference_trajectory.points.end())
        {
          iterator++;
        }
        //need hedwig ka orientation,position,heading(need to see that),velocity,acceln,bodyrates
        q_heading = Eigen::Quaternion<T>(Eigen::AngleAxis<T>(
          iterator->heading, Eigen::Matrix<T,3,1>::UnitZ()));
        q_orientation = q_heading * iterator->orientation.template cast<T>();
        reference_states_.col(i) << iterator->position.template cast<T>(),
        q_orientation.w(),
        q_orientation.x(),
        q_orientation.y(),
        q_orientation.z(),
        iterator->velocity.template cast<T>();
        if(reference_states_.col(i).segment(kOriW,4).dot(
          est_state_.segment(kOriW,4))<0.0)
          reference_states_.block(kOriW,i,4,1) =
        -reference_states_.block(kOriW,i,4,1);
        acceleration << iterator->acceleration.template cast<T>() - gravity;
        reference_inputs_.col(i) << acceleration.norm(),
        iterator->bodyrates.template cast<T>();
        quaternion_norm_ok &= abs(est_state_.segment(kOriW, 4).norm()-1.0)<0.1;
      }
    }
    return quaternion_norm_ok;
  }


  //updating of contoller commnad
  //output is a control command,state and input is taken as inputs
template <typename T>
  quadrotor_common::ControlCommand MpcController<T>::updateControlCommand(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> input,
    ros::Time& time)
  {
    Eigen::Matrix<T, kInputSize, 1> input_bounded = input.template cast<T>();

    // Bound inputs for sanity.
    //need to pick from parameter files
    input_bounded(INPUT::kThrust) = std::max(params_.min_thrust_,
      std::min(params_.max_thrust_, input_bounded(INPUT::kThrust)));
    input_bounded(INPUT::kRateX) = std::max(-params_.max_bodyrate_xy_,
      std::min(params_.max_bodyrate_xy_, input_bounded(INPUT::kRateX)));
    input_bounded(INPUT::kRateY) = std::max(-params_.max_bodyrate_xy_,
      std::min(params_.max_bodyrate_xy_, input_bounded(INPUT::kRateY)));
    input_bounded(INPUT::kRateZ) = std::max(-params_.max_bodyrate_z_,
      std::min(params_.max_bodyrate_z_, input_bounded(INPUT::kRateZ)));

    //we've to see the class of hedwig command
    quadrotor_common::ControlCommand command;

    command.timestamp = time;
    command.armed = true;
    command.control_mode = quadrotor_common::ControlMode::BODY_RATES;
    command.expected_execution_time = time;
    command.collective_thrust = input_bounded(INPUT::kThrust);
    command.bodyrates.x() = input_bounded(INPUT::kRateX);
    command.bodyrates.y() = input_bounded(INPUT::kRateY);
    command.bodyrates.z() = input_bounded(INPUT::kRateZ);
    command.orientation.w() = state(STATE::kOriW);
    command.orientation.x() = state(STATE::kOriX);
    command.orientation.y() = state(STATE::kOriY);
    command.orientation.z() = state(STATE::kOriZ);
    return command;
  }

  //Prediction publisher
template <typename T>
  bool MpcController<T>::publishPrediction(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples+1>> states,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples>> inputs,
    ros::Time& time)
  {
    nav_msgs::Path path_msg;
    path_msg.header.stamp = time;
    path_msg.header.frame_id = "world";
    geometry_msgs::PoseStamped pose;
    T dt = mpc_wrapper_.getTimestep();

    for(int i=0; i<kSamples; i++)
    {
      pose.header.stamp = time + ros::Duration(i*dt);
      pose.header.seq = i;
      pose.pose.position.x = states(kPosX,i);
      pose.pose.position.y = states(kPosY,i);
      pose.pose.position.z = states(kPosZ,i);
      pose.pose.orientation.w = states(kOriW,i);
      pose.pose.orientation.x = states(kOriX,i);
      pose.pose.orientation.y = states(kOriY,i);
      pose.pose.orientation.z = states(kOriZ,i);
      path_msg.poses.push_back(pose);
    }

    pub_predicted_trajectory_.publish(path_msg);

    return true;
  }

  //Starting the preartion thread. mostly related to time
template <typename T>
  void MpcController<T>::preparationThread()
  {
    const clock_t start = clock();

    mpc_wrapper_.prepare();

  // Timing
    const clock_t end = clock();
    timing_preparation_ = 0.9*timing_preparation_ +
    0.1* double(end - start)/CLOCKS_PER_SEC;
  }

  //Updating the costs, params or camera parameters
template <typename T>
  bool MpcController<T>::setNewParams(MpcParams<T>& params)
  {
    mpc_wrapper_.setCosts(params.Q_, params.R_);
    mpc_wrapper_.setLimits(
      params.min_thrust_, params.max_thrust_,
      params.max_bodyrate_xy_, params.max_bodyrate_z_);
    mpc_wrapper_.setCameraParameters(params.p_B_C_, params.q_B_C_);
    params.changed_ = false;
    return true;
  }


template class MpcController<float>;
template class MpcController<double>;

} // namespace rpg_mpc