#include <custom_gazebo_plugin/my_plugin.hpp>

using namespace std::placeholders;
using namespace std;

namespace gazebo{
MyPluginPrivate::MyPluginPrivate(MyPlugin* plug)
{
	plugin = plug;
}

MyPlugin::MyPlugin(): impl_(std::make_unique<MyPluginPrivate>(this))
{
	printf("Initiated Plugin !\n");
}

MyPlugin::~MyPlugin()
{
}

/**
 * @brief Load the SDF/URDF model of the robot and access the links/joints.
 * 
 * @param model 
 * @param sdf 
 */
void MyPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
    impl_->model_ = model;
	impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

	// Create a connection so the OnUpdate function is called at every simulation iteration. 
	impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
		std::bind(&MyPluginPrivate::OnUpdate, impl_.get()));
	
	
	subscription_motor_command = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
		"/simu/cmd_vel",
		10,
		std::bind(&MyPlugin::set_vel, this,std::placeholders::_1));
}

void MyPlugin::Init()
{
	//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "my_plugin.cpp : Init");
	impl_->baseLink = impl_->model_->GetLink("base_link");
	impl_->frontLink = impl_->model_->GetLink("dummy_joint_link");
	impl_->middleJoint = impl_->model_->GetJoint("middle_joint");
}

void MyPlugin::set_vel(geometry_msgs::msg::Twist::SharedPtr msg){
	//RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"set_vel");
	impl_->u0 = msg->linear.x;
	impl_->u1 = msg->linear.y;
	impl_->u2 = msg->linear.z;
	impl_->u3 = msg->angular.x;
	impl_->u4 = msg->angular.y;
	impl_->u5 = msg->angular.z;
	
}

void MyPlugin::get_state(){
	//RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"my_plugin.cpp : get_state");
	ignition::math::Pose3d pose_base_link = impl_->baseLink->WorldCoGPose();
	ignition::math::Pose3d pose_front_link = impl_->frontLink->WorldCoGPose();
	ignition::math::Pose3d pose_joint = impl_->middleJoint->WorldPose();

	impl_->x = pose_base_link.X();
	impl_->y = pose_base_link.Y();
	impl_->z = pose_base_link.Z();
	impl_->theta = pose_base_link.Yaw();
	//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "theta %f", impl_->theta);
}

void MyPlugin::process_derivative(){
	//RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"process derivative");
	impl_->dx = impl_->u0*cos(impl_->theta)+impl_->u1*sin(impl_->theta);
	impl_->dy = impl_->u0*sin(impl_->theta)+impl_->u1*cos(impl_->theta);
	impl_->dz = impl_->u2;
	impl_->dthetax = impl_->u3;
	impl_->dthetay = impl_->u4;
	impl_->dthetaz = impl_->u5;
}




/**
 * @brief This method is called at every time interval in Gazebo
 * 
 */
void MyPluginPrivate::OnUpdate()
{	
	
	//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "dx : %f", dx);
	plugin->get_state();
	
	plugin->process_derivative();
	//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "après process derivative");
	ignition::math::Vector3d newLinearVelBase(dx, dy,dz);
	ignition::math::Vector3d newAngularVelBase(dthetax,dthetay,dthetaz);
	baseLink->SetLinearVel(newLinearVelBase);
	//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "baseLink avec vitesse lineaire");
	baseLink->SetAngularVel(newAngularVelBase);
	//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "baseLink avec vitesse angulaire");
	baseLink->Update();
	//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "baseLink update");
	middleJoint->SetVelocity(0,0);
	middleJoint->Update();
	//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "middleJoint update");

	frontLink->Update();
	//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "frontlink update");
	model_->Update();
	//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "model update");
	//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "juste avant");
	if (frontLink and baseLink){
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "je suis dedans");

	}
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MyPlugin)

}
//AJOUTER LES LINK DE LA CAMERA ET LES FAIRE BOUGER
//REGARDER SUR INTERNET S'IL DOIVENT TOUS BOUGER POUR BOUGER et CHECKER LE CODE DES AUTRES POUR VOIR CE QUI BOUGE