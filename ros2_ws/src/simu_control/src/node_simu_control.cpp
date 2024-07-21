#include "simu_control/node_simu_control.hpp"

PilotageNode::PilotageNode() : Node("node_simu_control") {
    init_interfaces();
    init_parameters();
    timer_ = this->create_wall_timer(loop_dt_, std::bind(&PilotageNode::timer_callback, this));
}

PilotageNode::~PilotageNode() {}

void PilotageNode::init_interfaces(){
    publisher_command_ = this->create_publisher<geometry_msgs::msg::Twist>("/simu/cmd_vel", 10);
    subscriber_joy_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&PilotageNode::get_joy, this, std::placeholders::_1));
    subscriber_guidance_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/vision/pose", 10,std::bind(&PilotageNode::get_vision, this, std::placeholders::_1));
}

void PilotageNode::init_parameters() {
    loop_dt_ = 40ms;
    L_xyz << 0, 0, 0;
    L_angles << 0,0,0;
    L_Vxyz << 0, 0, 0;
    L_Vxyz_vision << 0, 0, 0;
    L_Vangles << 0,0,0;
    L_Vangles_vision << 0,0,0;

    teleop = true;
    button_pressed = false;

}

void PilotageNode::get_joy(sensor_msgs::msg::Joy joy) {
    double z = 0.0;
    if (joy.buttons[6] == 1 )
    {
        z = 1.0;
    } else if (joy.buttons[7] == 1 )
    {
        z = -1.0;
    }
    L_Vxyz << joy.axes[1], joy.axes[0], z;
    //L_Vangles << 0,-joy.axes[3],joy.axes[2];
    L_Vangles << 0,0,joy.axes[2];

    if (joy.buttons[9] == 1 && !button_pressed)
    {
        teleop = !teleop;
        button_pressed = true;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Je change de mode !");
    } else if (joy.buttons[9] == 0 && button_pressed)
    {
        button_pressed = false;
    }

}
void PilotageNode::get_vision(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
    // data[0] est le nombre d'aruco détectés
    //RCLCPP_INFO(node->get_logger(), msg->data);
    if (msg->data.size() >= 5) {
        L_Vxyz_vision <<msg->data[1],msg->data[2],msg->data[3];
        L_Vangles_vision << 0,0,0.msg->data[4];
    }
    else{
        L_Vxyz_vision << 0.0, 0.0, 0.0;
        L_Vangles_vision << 0.0, 0.0, 0.0;
    }
}

void PilotageNode::timer_callback(){
    auto message = geometry_msgs::msg::Twist();

    if (teleop)
    {   
        message.linear.x = L_Vxyz(0);
        message.linear.y = L_Vxyz(1);
        message.linear.z = L_Vxyz(2);
        message.angular.x = L_Vangles(0);
        message.angular.y = L_Vangles(1);
        message.angular.z = L_Vangles(2);

    } else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Je suis en mode autonome !");

        double offset_x = 0.4;
        double kx = 0.2;
        double kcap = 2; 

        if (L_Vxyz_vision(0) > offset_x) {vx = kx*(L_Vxyz_vision(0)-offset_x)/3;}
        else if ((offset_x > L_Vxyz_vision(0)) and (L_Vxyz_vision(0) > 0)){message.linear.x = kx*(L_Vxyz_vision(0)-offset_x)/3;}
        else {message.linear.x=0.;}

        if (L_Vxyz_vision(1) == 0){message.linear.y=0.;}
        else {message.linear.y = L_Vxyz(1)/3;}

        if (L_Vxyz_vision(2) == 0){message.linear.z=0.;}
        else {message.linear.z = L_Vxyz(2)/3;}

        if (L_Vangles_vision(2) == 0){message.angular.z = 0.0;}
        else {message.angular.z = -kcap*L_Vangles_vision(2)/90;}

        message.angular.x = 0.0;
        message.angular.y = 0.0;
    }
    // RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "commande : %f %f", u1_, u2_);
    publisher_command_->publish(message);
}



/****************************************
 
    MAIN

*****************************************/

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PilotageNode>());
    rclcpp::shutdown();
    return 0;
}
