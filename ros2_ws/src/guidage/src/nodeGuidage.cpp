#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <chrono>
#include <cmath>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <subsonus_pkg/msg/subsonus_remote_track_packet.hpp>
#include <subsonus_pkg/msg/subsonus_raw_sensors_packet.hpp>

double normalizeAngle360(double angle) {
    // Passer d'un angle entre -180 et 180 à un angle entre 0 et 360
    angle = fmod(angle + 360.0, 360.0);
    return angle;
}

double sawtooth(double angle){
    double diff_angle = (normalizeAngle360(angle+180))-180;  // or equivalently   2*arctan(tan(x/2))
    return diff_angle;
}
    

class NodeGuidage : public rclcpp::Node
{
public:
    NodeGuidage() : Node("nodeGuidage") {
        subscriber_subsonusTrackPacket = this->create_subscription<subsonus_pkg::msg::SubsonusRemoteTrackPacket>("subsonus/track_packet_filtered", 10,std::bind(&NodeGuidage::get_SubsonusTrackPackets, this, std::placeholders::_1));
        subscriber_subsonusROV = this->create_subscription<subsonus_pkg::msg::SubsonusRawSensorsPacket>("subsonus/raw_sensors_packet_ROV", 10,std::bind(&NodeGuidage::get_SubsonusROV, this, std::placeholders::_1));
        subscriber_vision = this->create_subscription<std_msgs::msg::Float64MultiArray>("/vision/pose", 10,std::bind(&NodeGuidage::get_vision, this, std::placeholders::_1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/real/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&NodeGuidage::timer_callback, this));                                                                               
    }

private:
    // Fonction de guidage
    void timer_callback()  
    {
        double vx;
        double vy;
        double vz;
        double vcap;

        // VISION
        double offset_x = 0.4;
        double kx = 0.6;
        double kcap = 0.5; 

        if (vision_dx > offset_x) {vx = kx*(vision_dx-offset_x)/3;}
        else if ((offset_x > vision_dx) and (vision_dx > 0)){vx = kx*(vision_dx-offset_x)/3;}
        else {vx=0.;}

        if (vision_dy == 0){vy=0.;}
        else {vy = vision_dy;}

        if (vision_dz == 0){vz=0.;}
        else {vz = vision_dz;}

        if (vision_cap == 0){vcap = 0.0;}
        else {vcap = -kcap*vision_cap/90;}

        // USBL
        // double kz = 0.2;
        // vx = 0.4;
        // vy = 0;
        // vz = kz*(usbl_dz+1);  // On se place à un mètre au dessus l'USBL de la cage pour voir le robot sous l'eau

        // double cap_desire = normalizeAngle360(cap_surface) + normalizeAngle360(usbl_azimut) - 180;  // Entre 0 et 360°
        // cap_desire = normalizeAngle360(cap_desire);
        // cap_ROV = normalizeAngle360(cap_ROV);
        // double err_cap = sawtooth(cap_desire - cap_ROV);  // Entre -180 et 180
        // vcap = -err_cap/180;
        //vcap = 0.7*tanh(-0.02*err_cap);


        // RCLCPP_INFO(get_logger(), "cap desire : ");
        // RCLCPP_INFO(get_logger(), std::to_string(cap_desire).c_str());
        // RCLCPP_INFO(get_logger(), "erreur_cap : ");
        // RCLCPP_INFO(get_logger(), std::to_string(err_cap).c_str());

        //RCLCPP_INFO(get_logger(), "cap desire : ");
        //RCLCPP_INFO(get_logger(), std::to_string(cap_desire).c_str());
        //RCLCPP_INFO(get_logger(), "erreur_cap : ");
        //RCLCPP_INFO(get_logger(), std::to_string(err_cap).c_str());


        // Publication sur le topic de contrôle
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = vx;  // Axe vers l'avant du robot
        msg.linear.y = vy;  // Axe vers la droite du robot
        msg.linear.z = vz;  // Axe vers le haut
        msg.angular.z = vcap;  // Sens trigonométrique
        publisher_->publish(msg);
    }

    // Fonction de récupération des données par subscribers
    void get_SubsonusTrackPackets(const subsonus_pkg::msg::SubsonusRemoteTrackPacket::SharedPtr msg)
    {
        usbl_range = msg->remote_range;  // en (m)
        usbl_azimut = msg->remote_azimuth;  // Entre -180 et 180° (Angle par rapport à l'axe x de l'UBSL surface)
        usbl_dx = msg->remote_position_x;
        usbl_dy = msg->remote_position_y;
        usbl_dz = msg->remote_position_z;  // dz < 0 si le ROV est moins profond que l'UBSL surface ; dz > 0 si le ROV est plus profond que l'UBSL surface
    }

    void get_SubsonusROV(const subsonus_pkg::msg::SubsonusRawSensorsPacket::SharedPtr msg)
    {
        cap_ROV = msg->cap_calib;  // Entre -180 et 180° (90° Est, -90° Ouest)
    }

    void get_vision(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // data[0] est le nombre d'aruco détectés
        //RCLCPP_INFO(node->get_logger(), msg->data);
        if (msg->data.size() >= 5) {
            vision_dx = msg->data[1];
            vision_dy = msg->data[2];
            vision_dz = msg->data[3];
            vision_cap = msg->data[4];
        }
        else{
            vision_dx = 0.0;
            vision_dy = 0.0;
            vision_dz = 0.0;
            vision_cap = 0.0;
        }
    }

    // Définition des variables
    double usbl_range;
    double usbl_azimut;
    double usbl_dx;
    double usbl_dy;
    double usbl_dz;
    double vision_dx;
    double vision_dy;
    double vision_dz;
    double vision_cap;
    double cap_ROV;
    double cap_surface = -170;  // à ajouter manuellement en fonction de la pos de la cage : Entre -180 et 180° (90° Est, -90° Ouest)

    // Définition des subscribers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<subsonus_pkg::msg::SubsonusRemoteTrackPacket>::SharedPtr subscriber_subsonusTrackPacket;
    rclcpp::Subscription<subsonus_pkg::msg::SubsonusRawSensorsPacket>::SharedPtr subscriber_subsonusROV;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_vision;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<NodeGuidage>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;

}
