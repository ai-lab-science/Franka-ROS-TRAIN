#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "sensor_msgs/Imu.h"

#include <gazebo/gazebo.hh>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <thread>
#include <memory>
#include <map>

namespace gazebo
{
    class set_imu_data : public ModelPlugin
    {
    public:
        set_imu_data() : ModelPlugin(){}
        
        // Set angular vel. and linear acc for the base link
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            this->model = _parent;
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&set_imu_data::OnUpdate, this, _1));
            physics::LinkPtr baseLink = this->model->GetLink("base_link");

            // Initialize ROS, if it has not already bee initialized.
            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "gazebo_client",
					ros::init_options::NoSigintHandler);
            }

            // Create our ROS node. This acts in a similar manner to the Gazebo node
            this->rosNode.reset(new ros::NodeHandle("set_imu_data"));
            
			// Create the subscriber
            ros::SubscribeOptions so = ros::SubscribeOptions::create<sensor_msgs::Imu>("/imu_data",
                                                                            1,
                                                                            boost::bind(&set_imu_data::OnRosMsg, this, _1),
                                                                            ros::VoidPtr(), &this->rosQueue);
            this->rosSub = this->rosNode->subscribe(so);
            
            // Spin up the queue helper thread
            this->rosQueueThread = std::thread(std::bind(&set_imu_data::QueueThread, this));
        }

        // Handle an incoming message from ROS
        void OnRosMsg(const sensor_msgs::Imu::ConstPtr &_msg)
        {
			this->imu_data = *_msg;
            return;
        }

        void OnUpdate(const common::UpdateInfo & _info)

        {
			// Set angular velocity and linear acceleration
			ignition::math::Vector3d lin_accel(imu_data.linear_acceleration.x,
												imu_data.linear_acceleration.y,
												imu_data.linear_acceleration.z);
			ignition::math::Vector3d ang_vel(imu_data.angular_velocity.x,
												imu_data.angular_velocity.y,
												imu_data.angular_velocity.z);
			
			this->model->GetLink("base_link")->SetLinearAccel(lin_accel);
			this->model->GetLink("base_link")->SetAngularVel(ang_vel);
			
            return;
        }
        
        
    private:
        // ROS helper function that processes messages
        void QueueThread()
        {
            static const double timeout = 0.01;
            while (this->rosNode->ok())
            {
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

		// Gazebo model pointer and connection pointer
        physics::ModelPtr model;

        event::ConnectionPtr updateConnection;

        // A node used for ROS transport
        std::unique_ptr<ros::NodeHandle> rosNode;

        // Subscribers
        ros::Subscriber rosSub;
		
		// A ROS callbackqueue that helps process messages
        ros::CallbackQueue rosQueue;
		
		// A thread the keeps running the rosQueue	
        std::thread rosQueueThread;
        
        // Data storage
        sensor_msgs::Imu imu_data;
    };

    GZ_REGISTER_MODEL_PLUGIN(set_imu_data)
}
