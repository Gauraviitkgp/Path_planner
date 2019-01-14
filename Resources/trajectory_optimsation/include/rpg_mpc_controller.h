
#include <thread>

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "hedwig_mpc/mpc_wrapper.h"
#include "rpg_mpc/mpc_params.h"

namespace hedwig_mpc 
{

  enum STATE 
  {
    kPosX = 0,
    kPosY = 1,
    kPosZ = 2,
    kOriW = 3,
    kOriX = 4,
    kOriY = 5,
    kOriZ = 6,
    kVelX = 7,
    kVelY = 8,
    kVelZ = 9
  };

  enum INPUT
  {
    kThrust = 0,
    kRateX = 1,
    kRateY = 2,
    kRateZ = 3
  };

  template <typename T>
  class MpcController 
  {
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static_assert(kStateSize == 10,
      "MpcController: Wrong model size. Number of states does not match.");
    static_assert(kInputSize == 4,
      "MpcController: Wrong model size. Number of inputs does not match.");

    MpcController(const ros::NodeHandle & nh, const ros::NodeHandle & pnh);
    MpcController() : MpcController(ros::NodeHandle(), ros::NodeHandle("~")) {}

    //Not_Required
    quadrotor_common::ControlCommand off();

    //Not required
    quadrotor_common::ControlCommand run(
      const quadrotor_common::QuadStateEstimate& state_estimate,
      const quadrotor_common::Trajectory& reference_trajectory,
      const MpcParams<T>& params);


  private:
    // Internal helper functions.

    //Not Required
    void pointOfInterestCallback(
      const geometry_msgs::PointStamped::ConstPtr& msg);

    //Given by hedwig
    bool setStateEstimate(
      const quadrotor_common::QuadStateEstimate& state_estimate);

    //Given by hedwig
    bool setReference(const quadrotor_common::Trajectory& reference_trajectory);

    //Command which we have to give to ros
    quadrotor_common::ControlCommand updateControlCommand(
      const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
      const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> input,
      ros::Time& time);

    //Still don't understand
    bool publishPrediction(
      const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples+1>> states,
      const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples>> inputs,
      ros::Time& time);

    //Started Preparation_thread
    void preparationThread();

    //A function of set new params, which sets the mpc params from the file. (mpcparams.h)
    bool setNewParams(MpcParams<T>& params);

    // Handles
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // Subscribers and publisher.
    ros::Subscriber sub_point_of_interest_;
    ros::Publisher pub_predicted_trajectory_;

    // Parameters
    MpcParams<T> params_;

    // MPC
    MpcWrapper<T> mpc_wrapper_;

    // Preparation Thread
    std::thread preparation_thread_;

    // Variables
    T timing_feedback_, timing_preparation_;
    Eigen::Matrix<T, kStateSize, 1> est_state_;
    Eigen::Matrix<T, kStateSize, kSamples+1> reference_states_;
    Eigen::Matrix<T, kInputSize, kSamples+1> reference_inputs_;
    Eigen::Matrix<T, kStateSize, kSamples+1> predicted_states_;
    Eigen::Matrix<T, kInputSize, kSamples> predicted_inputs_;
    Eigen::Matrix<T, 3, 1> point_of_interest_;
  };
