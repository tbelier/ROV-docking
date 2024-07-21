#ifndef MY_PLUGIN_HPP
#define MY_PLUGIN_HPP

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
// #include <math/gzmath.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Angle.hh>
#include <ctime>
#include <math.h>
#include "iostream"


#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <memory>

namespace gazebo
{
    class MyPlugin;
    // Forward declaration of private data class.
    class MyPluginPrivate
    {
        /**
         * @brief Class to hold data members and methods for plugin
         *
         */
    public:
        MyPluginPrivate(MyPlugin* plug);

        physics::ModelPtr model_;
        gazebo::physics::LinkPtr baseLink,frontLink;
        physics::JointPtr middleJoint;
        /**
        gazebo::physics::LinkPtr baseLink,frontLink;
        physics::JointPtr middleJoint;
        */


        // double x1,y1,x2,y2,theta,sigma,V;
        // double dx1,dy1,dx2,dy2,dtheta,dsigma,dV;
        // double k=0.1,K=0;
        // double p1=0.1,p2=0;
        // double h;

        double x,y,z,theta;
        double dx,dy,dz,dthetax,dthetay,dthetaz;
        double k1=0.1, k2=0.1, kp=0.2;
        double dt = 0.1;

        double u0,u1, u2, u3, u4, u5, u6;

        /// Connection to world update event. Callback is called while this is alive.
        gazebo::event::ConnectionPtr update_connection_;

        /// Node for ROS communication.
        gazebo_ros::Node::SharedPtr ros_node_;

        // Callback function to perform task with each iteration in Gazebo
        void OnUpdate();


        MyPlugin* plugin;

    };

class MyPlugin : public gazebo::ModelPlugin
    {
    public:
        /// Constructor
        MyPlugin();

        /// Destructor
        virtual ~MyPlugin();

        /// Gazebo calls this when the plugin is loaded.
        /// \param[in] model Pointer to parent model. Other plugin types will expose different entities,
        /// such as `gazebo::sensors::SensorPtr`, `gazebo::physics::WorldPtr`,
        /// `gazebo::rendering::VisualPtr`, etc.
        /// \param[in] sdf SDF element containing user-defined parameters.
        void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

        void Init() override;

        void get_state();
        void process_derivative();
        void set_vel(geometry_msgs::msg::Twist::SharedPtr msg);
    private:
        /// Recommended PIMPL pattern. This variable should hold all private
        /// data members.
        std::unique_ptr<MyPluginPrivate> impl_;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_motor_command;

        double M=2,L=0.4,P0=1,g=9.81,S=3.14*pow(0.057,2);

    };
}



#endif