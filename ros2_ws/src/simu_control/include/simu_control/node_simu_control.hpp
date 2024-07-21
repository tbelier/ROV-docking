#include <eigen3/Eigen/Dense>
#include <chrono>
#include <functional>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace Eigen;
using namespace std::chrono_literals;

class PilotageNode : public rclcpp::Node {
    public:
        PilotageNode();
        ~PilotageNode();
        void init_interfaces();
        void init_parameters();
        void timer_callback();
        void get_joy(sensor_msgs::msg::Joy joy);
        void get_vision(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    private:
        Matrix<double, 3, 1> L_xyz;
        Matrix<double, 3, 1> L_angles;
        Matrix<double, 3, 1> L_Vxyz;
        Matrix<double, 3, 1> L_Vangles;
        Matrix<double, 3, 1> L_Vxyz_vision;
        Matrix<double, 3, 1> L_Vangles_vision;
        bool teleop;
        bool button_pressed;
        std::chrono::milliseconds loop_dt_;
        rclcpp::TimerBase::SharedPtr timer_; // objet timer
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_joy_; // objet subscriber
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_guidance_; // objet subscriber
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_command_; // objet publisher
};
