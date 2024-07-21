import numpy
import time
import os
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32, Bool, Float64, Float64MultiArray
from std_msgs.msg import UInt16, String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy
from sensor_msgs.msg import FluidPressure
import numpy as np
from numpy import sqrt
from numpy.linalg import inv
from subsonus_pkg.msg import SubsonusRemoteTrackPacket
from subsonus_pkg.msg import SubsonusRawSensorsPacket
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle



class Localisation_node(Node):
    def __init__(self):
        # Init Node
        super().__init__('Localisation_node')

        # Subscriber
        self.subscriber1_ = self.create_subscription(Float64, '/sensor/pressure', self.callback_subcriber1, 10)
        self.subscriber2_ = self.create_subscription(SubsonusRemoteTrackPacket, '/subsonus/track_packet_filtered', self.callback_subcriber2, 10)  
        self.subscriber3_ = self.create_subscription(SubsonusRawSensorsPacket, '/subsonus/raw_sensors_packet_ROV', self.callback_subcriber3, 10) 
        self.subscriber4_ = self.create_subscription(Float64MultiArray, '/vision/pose', self.callback_subcriber4, 10)
        self.subscriber5_ = self.create_subscription(Twist, '/kalman/cmd_vel', self.callback_subcriber5, 10)
        self.subscriber6_ = self.create_subscription(Float64, '/sensor/yaw_speed', self.callback_subcriber6, 10)

        # Publisher
        self.publisher1_ = self.create_publisher(Twist, '/kalman/pose', 10)
        self.publisher2_ = self.create_publisher(Twist, '/kalman/std', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.callback_publisher)

        # Variables capteurs
        self.pressure = 1015.8  # (mB)
        self.usbl_range = 0.  # (m)
        self.usbl_azimut = 0. # -180/180
        self.usbl_dx = 0. #
        self.usbl_dy = 0. #
        self.usbl_dz = 0. #  (z vers le bas)
        self.usbl_cap_ROV = 0.
        self.cap_cage = 170. # degrés
        self.vision_dx = 0.
        self.vision_dy = 0.
        self.vision_dz = 0.
        self.vision_cap_ROV = 0.
        self.yaw_speed = 0.

        # ------ Kalman filter ------ # 
        # Boolean for confidence 
        self.aruco_is_detected = False

        # Initial state
        self.X = np.zeros((5,1))
        epsilon = 0.01               # x is in a circle of radius 1/epsilon
        self.Γx = (1/(epsilon**2))*np.eye(5)

        # Command
        self.u1 = 0.
        self.u2 = 0.
        self.u3 = 0.
        self.u4 = 0.

        # affichage 
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(10, 5))


    # Pressure sensor
    def callback_subcriber1(self,msg):
        self.pressure = msg.data
    
    # IMU
    def callback_subcriber6(self,msg):
        self.yaw_speed = msg.data
    
    # USBL msg1
    def callback_subcriber2(self,msg):
        self.usbl_range = msg.remote_range  # en (m)
        self.usbl_azimut = msg.remote_azimuth  # Entre -180 et 180° dans le sens trigo (Angle par rapport à l'axe x de l'UBSL surface)
        self.usbl_dx = msg.remote_position_x
        self.usbl_dy = msg.remote_position_y
        self.usbl_dz = msg.remote_position_z  # dz < 0 si le ROV est moins profond que l'UBSL surface ; dz > 0 si le ROV est plus profond que l'UBSL surface

    # USBL msg2
    def callback_subcriber3(self,msg):
        self.usbl_cap_ROV = msg.cap_calib     # entre -180 et 180 degres
    
    # Camera
    def callback_subcriber4(self,msg):
        if len(msg.data) >= 5 : 
            self.aruco_is_detected = True
            self.vision_dx = msg.data[1]
            self.vision_dy = msg.data[2]
            self.vision_dz = msg.data[3]
            self.vision_cap = msg.data[4]
        
        else :
            self.aruco_is_detected = False
            self.vision_dx = 0.0
            self.vision_dy = 0.0
            self.vision_dz = 0.0
            self.vision_cap = 0.0
    
    # Command
    def callback_subcriber5(self,msg):
        self.u1 = msg.linear.x     
        self.u2 = msg.linear.y 
        self.u3 = msg.linear.z 
        self.u4 = msg.angular.z

    
    # Update and Publish the estimation 
    def callback_publisher(self):

        # ------ Kalman filter ------ #
        # Prediction 
        dt = 0.1
        # Matrice d'évolution
        A = np.eye(5) + dt * np.array([[0,0,0,0,0],     # x
                                       [0,0,0,0,0],     # y 
                                       [0,0,0,0,0],     # z 
                                       [0,0,0,0,1],     # theta      
                                       [0,0,0,0,0]])    # theta_dot
        
        # Real speed in function of cmd_vel
        c_x = 1.   # m/s
        c_y = 0.5  # m/s
        c_z = 0.5    # m/s
        c_w = 120.   # deg/s 
        u = np.array([self.u1 * c_x,self.u2 * c_y ,self.u3 * c_z, 0., self.u4 * c_w])

        # Covariance
        ε_xyz = 1.**2         # (m)

        ε_theta = 20.0**2       # deg
        ε_theta_dot = 45**2     # deg/s
        Γ_alpha = np.array([[ε_xyz,0,0,0,0],
                            [0,ε_xyz,0,0,0],
                            [0,0,ε_xyz,0,0],
                            [0,0,0,ε_theta,0],
                            [0,0,0,0,ε_theta_dot]])

        # Matrice d'observation
        C_k = np.array([[0,0,1,0,0],    # pressure sensor
                        [1,0,0,0,0],    # usbl
                        [0,1,0,0,0],    # usbl
                        [0,0,1,0,0],    # usbl
                        [0,0,0,1,0],    # usbl
                        [1,0,0,0,0],    # camera
                        [0,1,0,0,0],    # camera
                        [0,0,1,0,0],    # camera
                        [0,0,0,1,0],    # camera
                        [0,0,0,0,1]])   # accelerometer
        
        # Confiance dans les mesure
        ε_press2 = 0.1**2       # (m)        pressure
        ε_xyz2 = 0.1**2         # (m)        usbl
        ε_cap2 = 10**2           # (degre)    usbl
        ε_camxyz2 = 100**2      # (m)        camera
        ε_cam_cap2 = 180**2     # (degre)    camera
        ε_acc = 2**2          # (deg/s)    IMU
        # If Rov sees an aruco 
        if self.aruco_is_detected :
            ε_camxyz2 = 0.05**2     # (m)        camera
            ε_cam_cap2 = 5**2       # (degre)    camera

        Γ_beta = np.array([[ε_press2,0,0,0,0,0,0,0,0,0],    # pressure sensor  (peu de confiance)   ~ 10 cm
                           [0,ε_xyz2,0,0,0,0,0,0,0,0],      # usbl             (confiance moyenne)
                           [0,0,ε_xyz2,0,0,0,0,0,0,0],      # usbl             
                           [0,0,0,ε_xyz2,0,0,0,0,0,0],      # usbl
                           [0,0,0,0,ε_cap2,0,0,0,0,0],      # usbl
                           [0,0,0,0,0,ε_camxyz2,0,0,0,0],   # camera           (grande confiance)
                           [0,0,0,0,0,0,ε_camxyz2,0,0,0],   # camera
                           [0,0,0,0,0,0,0,ε_camxyz2,0,0],   # camera 
                           [0,0,0,0,0,0,0,0,ε_cam_cap2,0],  # camera
                           [0,0,0,0,0,0,0,0,0,ε_acc]])      # accelerometer    (confiance moyenne)
        

        # Vecteur d'état X à estimer
        # x,y,z est la position du robot dans le repère NED de l'aruco central
        # theta est le cap du robot (angle par rapport au Nord magnétique)
            
        # Measurement 
        depth_aruco = 1.4    # m (depth of the central aruco)
        offset_usbl = 0.4    # m (distance between usbl and central aruco)
        p0 = 998.0  # mBar
        depth = ((self.pressure - p0)/1000)*10   # m
        y0 = np.array([depth,self.usbl_dx,self.usbl_dy,self.usbl_dz,self.usbl_cap_ROV,self.vision_dx,-self.vision_dy,self.vision_dz,self.vision_cap_ROV,self.yaw_speed]).reshape((10,1))
        y = y0 + np.array([-depth_aruco,0,0,-offset_usbl,0.,0.,0.,0.,0.,0.]).reshape((10,1))
        
        # Kalman equations 
        x_pred = A @ self.X  + u                 # predicted estimation
        Γ_pred = A @ self.Γx @ (A.T) + Γ_alpha   # predicted covariance
        S = C_k @ self.Γx @ (C_k.T) + Γ_beta            # covariance of the innovation 
        K = self.Γx @ (C_k.T) @ inv(S)                  # Kalman gain 
        y_tild = y - C_k @ x_pred                       # innovation
        self.X = x_pred + K @ y_tild                    # corrected estimation 
        #self.Γx = np.sqrt((np.eye(5) - K @ C_k) @ Γ_pred @ (Γ_pred.T) @ ((np.eye(5) - K @ C_k).T))     # corrected covariance (the formaula is there to prevent negative values)
        self.Γx = np.sqrt(np.abs((np.eye(5) - K @ C_k) @ Γ_pred))     # corrected covariance (the formaula is there to prevent negative values)

        ## Publish 
        # Pose
        msg = Twist()
        msg.linear.x = self.X[0,0]
        msg.linear.y = self.X[1,0]
        msg.linear.z = self.X[2,0]
        msg.angular.x = 0.
        msg.angular.y = 0.
        msg.angular.z = self.X[3,0]   # cap

        # Standard deviation
        std = Twist()
        std.linear.x = sqrt(self.Γx[0,0])
        std.linear.y = sqrt(self.Γx[1,1])
        std.linear.z = sqrt(self.Γx[2,2])
        std.angular.x = sqrt(self.Γx[0,0] + self.Γx[1,0])  # rayon du cercle de confiance
        std.angular.y = 0.
        std.angular.z = sqrt(self.Γx[3,3]) 

        # Rejeu de mission
        # self.ax1.clear()
        # xmin,xmax = 0,15
        # ymin,ymax = -15,15
        # self.ax1.set_xlim(xmin, xmax)
        # self.ax1.set_ylim(ymin, ymax)
        # self.ax1.set_xlabel('x')
        # self.ax1.set_ylabel('y')
        # self.ax1.set_title('Aerial view')
        # self.ax1.plot(msg.linear.x,msg.linear.y, marker = "o", color = "blue")
        # theta = ((msg.angular.z)/180*np.pi) + np.pi
        # self.ax1.quiver(msg.linear.x, msg.linear.y, np.cos(theta), np.sin(theta), color='r')
        # plt.grid()

        # self.ax2.clear()
        # self.ax2.set_title('Depth view')
        # xmin,xmax = 0,2
        # ymin,ymax = -3,10
        # self.ax2.set_xlim(xmin, xmax)
        # self.ax2.set_ylim(ymin, ymax)
        # self.ax2.set_xlabel('x')
        # self.ax2.set_ylabel('z')
        # self.ax2.plot(1.,msg.linear.z, marker = "o", color = "blue")

        # self.fig.suptitle('Suivi en temps réel de la mission ')

        # plt.pause(0.01)
        


        
        self.publisher1_.publish(msg)
        self.publisher2_.publish(std)



def main(args=None):

    rclpy.init()
    node = Localisation_node()
    rclpy.spin(node)


    rclpy.shutdown()


if __name__ == "__main__":
    main()






