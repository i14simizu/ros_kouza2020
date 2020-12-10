#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
//#include "geometry_msgs/Twist.h"

#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

namespace gazebo{
	class GAZEBO_VISIBLE KamePlugin : public ModelPlugin{
		event::ConnectionPtr update_conn_;
	public:
		KamePlugin();
		~KamePlugin();

		void Load(physics::ModelPtr model, sdf::ElementPtr _sdf) override;
		void Reset() override;
		void OnUpdate();
		bool GetParam(sdf::ElementPtr sdf);
		void CrankAcallback(const std_msgs::Float64::ConstPtr& a_vel_msg);
		//void PointCallback(const std_msgs::Float64MultiArray::ConstPtr& vel_msg_array);

		void CrankBcallback(const std_msgs::Float64::ConstPtr& b_vel_msg);
		void CrankCcallback(const std_msgs::Float64::ConstPtr& c_vel_msg);
		void CrankDcallback(const std_msgs::Float64::ConstPtr& d_vel_msg);

	private:
		physics::ModelPtr model_;
		physics::WorldPtr world_;
		common::Time last_time_;
		physics::LinkPtr link_;


		ros::NodeHandle nh_;
		ros::Subscriber sub_a_;
		//ros::Publisher pub_;

		ros::Subscriber sub_b_;
		ros::Subscriber sub_c_;
		ros::Subscriber sub_d_;

		std::string topic_name_;
		std::string pub_name_;

		std::string a_joint_;
		//float a_p_;
		float a_target_;


		std::string b_joint_;
		float b_target_;

		std::string c_joint_;
		float c_target_;

		std::string d_joint_;
		float d_target_;


		//std_msgs::Float64 a_current_vel;

		//std_msgs::Float64MultiArray pub_array;

		/*
		std::string yaw_joint_name_;
		std::string pitch_joint_name_;
		float yaw_p_;
		float pitch_p_;
		float yaw_target_;
		float pitch_target_;
		*/
	};
}
