#include "model_plugin.hh"
#include <gazebo/common/Events.hh>
#include <cmath>



namespace gazebo{
	GZ_REGISTER_MODEL_PLUGIN(KamePlugin)

	KamePlugin::KamePlugin():nh_(){}
	KamePlugin::~KamePlugin(){}

	void KamePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
		/*Load() : initialize. When plugin is loaded, this function is called.*/
		/*ModelPtr : pointer to Gazebo model*/
		/*ElementPtr : pointer to SDF parameter*/
		gzmsg << "Load KamePlugin\n";
		GetParam(_sdf);
		model_ = _model;
		world_ = _model->GetWorld();
		link_ = _model->GetLink("link");
		update_conn_ = event::Events::ConnectWorldUpdateBegin(std::bind(&KamePlugin::OnUpdate, this));
		/*register callback function "OnUpdate". callback function is called every 1000 times per sec.*/
		sub_a_ = nh_.subscribe("cmd_velocity/velocity_a", 100, &KamePlugin::CrankAcallback, this);
		sub_b_ = nh_.subscribe("cmd_velocity/velocity_b", 100, &KamePlugin::CrankBcallback, this);
		sub_c_ = nh_.subscribe("cmd_velocity/velocity_c", 100, &KamePlugin::CrankCcallback, this);
		sub_d_ = nh_.subscribe("cmd_velocity/velocity_d", 100, &KamePlugin::CrankDcallback, this);

		//pub_ = nh_.advertise<std_msgs::Float64MultiArray>("Kame/pub_array/current_vel", 100);

		//pub_array.data.resize(4);
	}

	void KamePlugin::OnUpdate(){
		common::Time current_time = world_->SimTime();
		/*
		if((current_time - last_time_).Double() > (1.0/2.0)){
			last_time_ = current_time;
			ignition::math::Pose3d pose = this->link_->WorldPose();
			printf("pos: %f %f %f\n", pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
			printf("rot: %f %f %f\n", pose.Rot().Roll(), pose.Rot().Pitch(), pose.Rot().Yaw());
		}
		*/

		
		auto a = model_->GetJoint(a_joint_);
		//a->SetVelocity(0, -a_p_*(a->Position(0) - a_target_));
		a->SetVelocity(0,a_target_);
		//a->SetParam("fmax",0,100.0);
		//a->SetParam("vel",0,a_target_);

		
		auto b = model_->GetJoint(b_joint_);
		b->SetVelocity(0,b_target_);

		auto c = model_->GetJoint(c_joint_);
		c->SetVelocity(0, c_target_);

		auto d = model_->GetJoint(d_joint_);
		d->SetVelocity(0, d_target_);
		
		

		/*
		auto yaw = model_->GetJoint(yaw_joint_name_);
		//We can get pointer to model's joint
		//attention : please use correct joint name.
		yaw->SetVelocity(0, -yaw_p_ * (yaw->GetAngle(0).Radian() - yaw_target_));
		//joint_ptr->GetAngle(0).Radian() : We can get joint's angle.
		//GenAngle(0) : 0 is joint's axis index.
		//joint_ptr->SetVelocity(value) : We can set joint's velocity
		auto pitch = model_->GetJoint(pitch_joint_name_);
		pitch->SetVelocity(0, -pitch_p_ * (pitch->GetAngle(0).Radian() - pitch_target_));
		*/

		//get current velicity
		//pub_array.data[0] = a->GetVelocity(0);
		//printf("current velocity : %lf\n", a_current_vel);

		//pub_array.data[1] = b->GetVelocity(0);
		//pub_array.data[2] = c->GetVelocity(0);
		//pub_array.data[3] = d->GetVelocity(0);

		//pub_.publish(pub_array);


	}

	
	void KamePlugin::Reset(){
	}


	void KamePlugin::CrankAcallback(const std_msgs::Float64::ConstPtr& a_vel_msg){
	//void KamePlugin::PointCallback(const std_msgs::Float64MultiArray::ConstPtr& vel_msg_array){
		a_target_ = a_vel_msg->data;

		//a_target_ = vel_msg_array->data[0];
		//b_target_ = vel_msg_array->data[1];
		//c_target_ = vel_msg_array->data[2];
		//d_target_ = vel_msg_array->data[3];

		/*
		a_target_ = vel_msg.data[0];
		b_target_ = vel_msg.data[1];
		c_target_ = vel_msg.data[2];
		d_target_ = vel_msg.data[3];
		*/


		/*
		float hlen = sqrt(point_msg.x * point_msg.x + point_msg.y * point_msg.y);
		if(0.1 <= hlen){
			yaw_target_ = atan2(point_msg.y, point_msg.x);
			pitch_target_ = -atan2(point_msg.z, hlen);
		}
		*/
	}

	void KamePlugin::CrankBcallback(const std_msgs::Float64::ConstPtr& b_vel_msg){
		b_target_ = b_vel_msg->data;
	}

	void KamePlugin::CrankCcallback(const std_msgs::Float64::ConstPtr& c_vel_msg){
		c_target_ = c_vel_msg->data;
	}

	void KamePlugin::CrankDcallback(const std_msgs::Float64::ConstPtr& d_vel_msg){
		d_target_ = d_vel_msg->data;
	}

	bool KamePlugin::GetParam(sdf::ElementPtr sdf){
		//Get topic name
		if(!sdf->HasElement("topic_name")){
			gzmsg << "topic_name not set\n";
			return false;
		}
		else{
			topic_name_ = sdf->GetElement("topic_name")->Get<std::string>();
		}

		//Get joint name
		if(!sdf->HasElement("a_joint_name")){
			gzmsg << "a_joint_name not set\n";
			return false;
		}
		else{
			a_joint_ = sdf->GetElement("a_joint_name")->Get<std::string>();
		}
		if(!sdf->HasElement("b_joint_name")){
			gzmsg << "b_joint_name not set\n";
			return false;
		}
		else{
			b_joint_ = sdf->GetElement("b_joint_name")->Get<std::string>();
		}
		if(!sdf->HasElement("c_joint_name")){
			gzmsg << "c_joint_name not set\n";
			return false;
		}
		else{
			c_joint_ = sdf->GetElement("c_joint_name")->Get<std::string>();
		}
		if(!sdf->HasElement("d_joint_name")){
			gzmsg << "d_joint_name not set\n";
			return false;
		}
		else{
			d_joint_ = sdf->GetElement("d_joint_name")->Get<std::string>();
		}


		/*
		//get joint param
		if(!sdf->HasElement("a_p")){
			gzmsg << "a_p has not set\n";
			return false;
		}
		else{
			a_p_ = sdf->GetElement("a_p")->Get<float>();
		}
		*/

		return true;

		/*sdf->HasElement("") : is able to Determine if SDF parameter exists*/
		/*
		if(!sdf->HasElement("topic_name")){
			gzmsg << "topic_name not set\n";
			return false;
		}
		else{
			topic_name_ = sdf->GetElement("topic_name")->Get<std::string>();
		}
		if(!sdf->HasElement("yaw_joint_name")){
			gzmsg << "yaw_joint_name not set\n";
			return false;
		}
		else{
			yaw_joint_name_ = sdf->GetElement("yaw_joint_name")->Get<std::string>();
			//We can get real parameter's value
		}
		if(!sdf->HasElement("yaw_p")){
			gzmsg << "yaw_p not set\n";
			return false;
		}
		else yaw_p_ = sdf->GetElement("yaw_p")->Get<float>();

		if (!sdf->HasElement("pitch_joint_name")){
    		gzmsg << "pitch_joint_name not set\n";
    		return false;
 		}
  		else{
    		pitch_joint_name_ = sdf->GetElement("pitch_joint_name")->Get<std::string>();
  		} 
  		if (!sdf->HasElement("pitch_p")){
   			gzmsg << "pitch_p not set\n";
    		return false;
  		}
  		else pitch_p_ = sdf->GetElement("pitch_p")->Get<float>();

  		return true;
  		*/
	}
}
