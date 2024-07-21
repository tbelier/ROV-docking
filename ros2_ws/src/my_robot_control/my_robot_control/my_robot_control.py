import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self.joy_subscriber = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/demo/cmd_vel', 10)

    def joy_callback(self, msg):
        # Vous pouvez implémenter la logique de contrôle du robot ici
        linear_speed = msg.axes[3]  # Supposons que l'axe 1 contrôle la vitesse linéaire
        angular_speed = msg.axes[2]  # Supposons que l'axe 3 contrôle la vitesse angulaire

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_speed
        if linear_speed < 0:
            cmd_vel_msg.angular.z = -angular_speed
        else:
            cmd_vel_msg.angular.z = angular_speed

        self.cmd_vel_publisher.publish(cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

