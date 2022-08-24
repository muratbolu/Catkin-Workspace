#ifndef _MY_VEHICLE_PLUGIN_HPP_
#define _MY_VEHICLE_PLUGIN_HPP_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

#include <thread>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Twist.h>

namespace gazebo {

    /// \brief A plugin to make a wheelbarrow turn around the origin.
    class WheelbarrowPlugin : public ModelPlugin {

        /// \brief Pointer to the model.
        physics::ModelPtr model;

        /// \brief Pointer to the main link.
        physics::LinkPtr link;

        /// \brief A node use for ROS transport
        std::unique_ptr<ros::NodeHandle> rosNode;

        /// \brief A ROS subscriber
        ros::Subscriber twistSubscriber;

        public:

        /// \brief Constructor
        WheelbarrowPlugin() {}

        /// \brief The load function is called by Gazebo when the plugin is
        /// inserted into simulation
        /// \param[in] _model A pointer to the model that this plugin is
        /// attached to.
        /// \param[in] _sdf A pointer to the plugin's SDF element.
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

            // Store the model pointer for convenience.
            this->model = _model;

            // Create our ROS node.
            this->rosNode.reset(new ros::NodeHandle());

            // Create a topic, and subscribe to it.
            this->twistSubscriber = this->rosNode->subscribe("/" + this->model->GetName() + "/vel_cmd",
                                                             1,
                                                             &WheelbarrowPlugin::OnRosMsg,
                                                             this);

        }

        /// \brief Set the velocity of the vehicle
        /// \param[in] _vel New target velocity
        void SetVelocity(const geometry_msgs::Twist &_vel) {
            // Set the model's target velocity.
            this->model->SetWorldTwist(ignition::math::Vector3<double>(_vel.linear.x,  _vel.linear.y,  _vel.linear.z),
                                       ignition::math::Vector3<double>(_vel.angular.x, _vel.angular.y, _vel.angular.z));
        }

        /// \brief Handle an incoming message from ROS
        /// \param[in] _msg A float value that is used to set the velocity
        /// of the Velodyne.
        void OnRosMsg(const geometry_msgs::Twist::ConstPtr &_msg) {
            this->SetVelocity(*_msg);
        }

    };

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(WheelbarrowPlugin)

}

#endif
