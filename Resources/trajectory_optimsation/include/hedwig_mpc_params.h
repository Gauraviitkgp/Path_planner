#include <ros/ros.h>
#include <trajectory_optimsation/mpc_wrapper.h>

namespace hedwig_mpc
{

template <typename T>
	class MpcParams 
	{
	public:

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		T Q_pos_xy,Q_pos_z,Q_perception,Q_attitude,Q_velocity;//errors horizontol postion, height, rejporjection error,attitude and velocity
		T R_thrust, R_pitchroll, R_yaw; //Reading input costs
		bool changed_,print_info_;

		MpcParams() :
		changed_(false),
		print_info_(false),
		state_cost_exponential_(0.0),
		input_cost_exponential_(0.0),
		max_bodyrate_xy_(0.0),
		max_bodyrate_z_(0.0),
		min_thrust_(0.0),
		max_thrust_(0.0),
		p_B_C_(Eigen::Matrix<T, 3, 1>::Zero()),
		q_B_C_(Eigen::Quaternion<T>(1.0, 0.0, 0.0, 0.0)),
		Q_(Eigen::Matrix<T, kCostSize, kCostSize>::Zero()),
		R_(Eigen::Matrix<T, kInputSize, kInputSize>::Zero())
		Q
		{
		}

		~MpcParams()
		{
		}

		bool loadParameters()
		{
			max_bodyrate_xy_= 3;
			max_bodyrate_z_=5;
			min_thrust_=0;
			max_thrust_=1;
			Q_pos_xy=200;
			p_B_C_=(Eigen::Matrix<T, 3, 1>()<<0,0,0);
			q_B_C= Eigen::Quaternion<T>()<<1,0,0,0;
			state_cost_exponential_= 0.0;
			input_cost_exponential_= 0.0;

			Q_pos_xy=200;
			Q_pos_z=500;
			Q_attitude=50;
			Q_velocity=10;
			Q_perception=0;

			R_thrust=1;
			R_pitchroll=1;
			R_yaw=1;
			Q_ = (Eigen::Matrix<T, kCostSize, 1>() <<
				Q_pos_xy, Q_pos_xy, Q_pos_z,
				Q_attitude, Q_attitude, Q_attitude, Q_attitude,
				Q_velocity, Q_velocity, Q_velocity,
				Q_perception, Q_perception).finished().asDiagonal();
			R_ = (Eigen::Matrix<T, kInputSize, 1>() <<
				R_thrust, R_pitchroll, R_pitchroll, R_yaw).finished().asDiagonal();
			return true;
		}

		T state_cost_exponential_;
		T input_cost_exponential_;
		Eigen::Matrix<T, 3, 1> p_B_C_;
		Eigen::Quaternion<T> q_B_C_;
		Eigen::Matrix<T, kCostSize, kCostSize> Q_;
		Eigen::Matrix<T, kInputSize, kInputSize> R_;
	};



} // namespace rpg_mpc
