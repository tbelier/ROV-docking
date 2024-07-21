import numpy
import time
import os
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32, Bool
from std_msgs.msg import UInt16, Float64
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy
from sensor_msgs.msg import FluidPressure
from scipy.spatial.transform import Rotation
from pymavlink import mavutil
import numpy as np

"""
ros2 pkg create my_python_pkg --build-type ament_python
"""

"""
Commander chaque moteur :
message du type : ACTUATOR_OUTPUT_FUNCTION_MOTOR%n%  avec %n% le moteur en quesstion  
"""

"""
Messages de control 
MAV_CMD_NAV_SET_YAW_SPEED
MANUAL_CONTROL
RC_CHANNELS_OVERRIDE
MANUAL_SETPOINT 
SET_ATTITUDE_TARGET
ACTUATOR_CONTROL_TARGET

"""

master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()

#button = [0,0,0,0,0,0,0,0,0,0,0,0]  # [X,A,B,Y,LB,RB,LT,RT,back,start,L3,R3]
#Jaxes = [0,0,0,0,0,0]       # [L_horizontal,L_vertical,R_horizontal,R_vertical,Cross_horizontal,Cross_Vertical]
stable_on=False
stable_cap_on = False
pressionair=1.00265
k = 200  # tunning gain
old_values_buttons = [0,0,0,0,0,0,0,0]
    #                1          2      3       4      5         6       7     8    9   10
    # channels =  [ pitch ,  roll , up/down, yaw , av/rec ,right/left, ?  , cam, lum, ...]
channels =        [1500,1500,1500,1500,1500,1500,1500,1500,1100,1500,1500,1500,1500,1500,1500,1500]
static_channels = [1500,1500,1500,1500,1500,1500,1500,1500,1100,1500,1500,1500,1100,1500,1500,1500]
    #               1    2    3     4    5    6    7    8    9   10   ...
class Transmitter(Node):
    def __init__(self):
        # Init Node
        super().__init__('Transmitter')
        # Subscribers
        self.subscription_transm = self.create_subscription(Joy, '/joy', self.callback_transmitter, 10)
        self.subscription_auto_pilot_vel = self.create_subscription(Twist, '/real/cmd_vel', self.callback_auto_vel, 10)
        # Publishers
        self.publisher_ = self.create_publisher(Twist, '/kalman/cmd_vel', 10)
        self.publisher1_ = self.create_publisher(Float64, '/sensor/pressure', 10)
        self.publisher2_ = self.create_publisher(Float64, '/sensor/yaw_speed', 10)
        # Callback timers
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.callback_publisher)
        dt_sensor = 0.1
        self.timer1 = self.create_timer(dt_sensor, self.callback_publisher1)


        # Class variables
        self.LT = 0  # Elev -
        self.RT = 0  # Elev +
        self.LB = 0  # Gain -
        self.RB = 0  # Gain +
        self.Cross_vertical = 0.           # Camera orientation top+/bottom-
        self.Cross_horizontal = 0.         # Spotlights +/-
        self.JoystickLeft_vertical = 0.    # Unused
        self.JoystickLeft_horizontal = 0.  # Rotate Left-/+Right
        self.JoystickRight_vertical = 0.   # XY plane backward-/+forward
        self.JoystickRight_horizontal = 0. # XY plane left-/+right
        self.X_button = 0  # Switch Manual control  
        self.Y_button = 0  # Switch Stabilize mode 
        self.A_button = 0  # Switch Auto control 
        self.B_button = 0  # Unused
        self.start_button = 0
        self.back_button = 0
        self.frame_id = 0
        self.frame_id_old = 0
        self.use_transmitter = True
        self.armed = False
        self.lum = 0

        # Auto pilot commands 
        self.linear = np.zeros((3,1))
        self.angular = np.zeros((3,1))

        # Sensors
        self.press_msg = Float64()
        self.yaw_sp_msg = Float64()


        # Init with normal flight mode 
        self.set_normal_mode()

    def arming(self,boolean):
        # Arm
        if boolean:
            # master.arducopter_arm() or:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

            # wait until arming confirmed (can manually check with companion.motors_armed())
            #print("Waiting for the vehicle to arm")
            # master.motors_armed_wait()
            #print('Armed!')
            self.get_logger().info('\033[91mWaiting for the vehicle to arm\033[0m')
            master.motors_armed_wait()
            self.get_logger().info('\033[91mArmed !\033[0m')
            # Bleu : 94
            # Rouge: 91
            # Vert: 92
            # Orange: 93

        # Disarm
        else :
            # companion.arducopter_disarm() or:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)

            # wait until disarming confirmed
            # print("Waiting for the vehicle to disarm")
            # master.motors_disarmed_wait()
            # print('Disarmed')
            self.get_logger().info('\033[91mWaiting for the vehicle to disarm\033[0m')
            master.motors_disarmed_wait()
            self.get_logger().info('\033[91mDisarmed \033[0m')
        
    
    def send_mavlink_msg(self, channels):
        rc_channel_values = [1500 for _ in range(18)]
        for i in range(0,len(channels)) :
            channel_id = i
            if channel_id < 0 or channel_id > 17:
                print("Channel " + str(channel_id) + " does not exist.")
                return

            # Mavlink 2 supports up to 18 channels:
            # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
            rc_channel_values[channel_id] = channels[i]
        master.mav.rc_channels_override_send(
            master.target_system,                # target_system
            master.target_component,             # target_component
            *rc_channel_values)                  # RC channel list, in microseconds.

    def set_stabilize_mode(self):
        # Choose a mode
            mode = 'STABILIZE'

            # Check if mode is available
            if mode not in master.mode_mapping():
                print('Unknown mode : {}'.format(mode))
                print('Try:', list(master.mode_mapping().keys()))
                sys.exit(1)

            # Get mode ID
            mode_id = master.mode_mapping()[mode]
            # Set new mode
            master.mav.set_mode_send(
                master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id)
            
    def set_normal_mode(self):
        # Choose a mode
            mode = 'MANUAL'

            # Check if mode is available
            if mode not in master.mode_mapping():
                print('Unknown mode : {}'.format(mode))
                print('Try:', list(master.mode_mapping().keys()))
                sys.exit(1)

            # Get mode ID
            mode_id = master.mode_mapping()[mode]
            # Set new mode
            master.mav.set_mode_send(
                master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id)
            
    def set_depth_hold_mode(self):
        a = 1

    
    def send_autopilot_commands(self):
        k_auto = 200
        channels_auto = static_channels.copy()
        channels_auto[8] = self.lum
        # cha 1 = pitch
        # cha 2 = roll
        # cha 3 = monter / descendre 
        # cha 4 = yaw 
        # cha 5 = av / rec
        # cha 6 = coté 
        # cha 7 = ?
        # cha 8 = camera
        # cha 9 = lights

        channels_auto[4] = int(1500 + k_auto * self.linear[0,0])     # x    (+ vers l'avant)
        channels_auto[5] = int(1500 - k_auto * self.linear[1,0])     # y    (+ vers la gauche)
        channels_auto[2] = int(1500 + k_auto * self.linear[2,0])     # z    (+ vers le haut)
        channels_auto[3] = int(1500 - k_auto * self.angular[2,0])    # yaw  (+ vers la gauche) sens trigo
        

        self.send_mavlink_msg(channels_auto)

        # master.mav.manual_control_send(
        #     master.target_system,
        #     int(self.linear[0,0]*1000),  # x    normalized to the range [-1000,1000]
        #     int(self.linear[1,0]*1000),  # y    normalized to the range [-1000,1000]
        #     int(self.linear[2,0]*1000),  # z    normalized to the range [-1000,1000]
        #     int(self.angular[2,0]*1000), # yaw  normalized to the range [-1000,1000]
        #     0) 


    def callback_transmitter(self,msg):
        button = msg.buttons
        Jaxes = msg.axes

        self.LT = button[6]
        self.RT = button[7]
        self.LB = button[4]
        self.RB = button[5]
        self.Cross_vertical = Jaxes[5]
        self.Cross_horizontal = Jaxes[4]
        self.JoystickLeft_vertical = Jaxes[1]
        self.JoystickLeft_horizontal = Jaxes[0]
        self.JoystickRight_vertical = Jaxes[3]
        self.JoystickRight_horizontal = Jaxes[2]
        self.X_button = button[0]  # Switch Manual control 
        self.Y_button = button[3]  # Set Stabilize mode 
        self.A_button = button[1]  # Switch to Autopilot
        self.B_button = button[2]  # Unused
        self.start_button = button[9]
        self.back_button = button[8]
    
    def callback_auto_vel(self,msg):
        self.linear = np.array([[msg.linear.x],[msg.linear.y],[msg.linear.z]])
        self.angular = np.array([[msg.angular.x],[msg.angular.y],[msg.angular.z]])

    # Sensor values (pressure, yawspeed)
    def callback_publisher1(self):
        m = master.recv_match(type='SCALED_PRESSURE2')
        if m is not None:
            p = m.press_abs            # hPa = mB
            t = m.temperature          # cdegC
            self.press_msg.data  = p
            self.publisher1_.publish(self.press_msg)
        
        m = master.recv_match(type='ATTITUDE')
        if m is not None:
            w = (m.yawspeed/np.pi)*180.   # deg/s
            self.yaw_sp_msg.data = w
            self.publisher2_.publish(self.yaw_sp_msg)


    # Commands from Joystick 
    def callback_publisher(self):
        global k, old_values_buttons, channels
        
        channels = static_channels.copy()

        ################## Manual mode ##################
            # msg = OverrideRC 
            #                 1        2       3       4       5         6          7        8    9   10
            # channels = [  pitch ,  roll , up/down,  yaw , av/rec ,right/left, stab mode , cam, lum, ...]
            # channels = [1500,1500,1500,1500,1500,1500,1500,1500,1100,1500,1500,1500,1500,1500,1500,1500] 
            #              1    2    3     4    5    6    7    8    9   10

        values_buttons = [self.X_button,self.A_button,self.B_button,self.Y_button,self.start_button,self.back_button,self.LT,self.RT]
        start_button = old_values_buttons[4]
        start_button = old_values_buttons[5]
        X_button = old_values_buttons[0]
        A_button = old_values_buttons[1]
        B_button = old_values_buttons[2]
        Y_button = old_values_buttons[3]
        # self.LT = old_values_buttons[6]
        # self.RT = old_values_buttons[7]

        # # Get data
        LB = self.LB  # Gain -
        RB = self.RB  # Gain +
        LT = self.LT  # Elev -
        RT = self.RT  # Elev +
        Cross_vertical = self.Cross_vertical   # Camera orientation top+/bottom-
        Cross_horizontal = self.Cross_horizontal   # Spotlights +/-
        JoystickLeft_vertical = self.JoystickLeft_vertical   # Unused
        JoystickLeft_horizontal = self.JoystickLeft_horizontal   # Rotate Left-/+Right
        JoystickRight_vertical = self.JoystickRight_vertical   # XY plane backward-/+forward
        JoystickRight_horizontal = self.JoystickRight_horizontal   # XY plane left-/+right
        front_montant = False

        # Detection du front montant
        if old_values_buttons != values_buttons:
            front_montant = True
            X_button = self.X_button  # Switch to Manual control
            Y_button = self.Y_button  # Switch to stabilise mode 
            A_button = self.A_button  # Switch to Auto control
            B_button = self.B_button  # Unused
            start_button = self.start_button
            back_button = self.back_button
            old_values_buttons = [self.X_button,self.A_button,self.B_button,self.Y_button,self.start_button,self.back_button,self.LT,self.RT]
        else :
            front_montant = False
            X_button = old_values_buttons[0]  # Switch to Manual control
            Y_button = old_values_buttons[3]  # Switch to Stabilise mode 
            A_button = old_values_buttons[1]  # Switch to Auto control
            B_button = old_values_buttons[2]  # Switch to Normal mode 
            start_button = old_values_buttons[4]
            back_button = old_values_buttons[5]
            

        ### Control ###
        # If start
        if start_button == 1 and front_montant:
            if not(self.armed) :
                self.armed = True
                self.arming(True) # mavros service disarm
        
        # If back
        if back_button == 1 and front_montant:
            if self.armed :
                self.armed = False
                self.arming(False) # mavros service disarm
    
        # If A
        if A_button==1 and front_montant:
            if self.use_transmitter:
                self.use_transmitter = False
                self.set_stabilize_mode()
                #print("Auto mode")
                self.get_logger().info('\033[94mAuto mode\033[0m')
                
        # if X
        if X_button==1 and front_montant:
            if not(self.use_transmitter):
                self.use_transmitter = True
                #print("Manual mode")
                self.get_logger().info('\033[94mManual mode\033[0m')

        # if Y
        if Y_button==1 and front_montant: #and channels[6]==1500:    
            #print("Stabilise mode")
            self.get_logger().info('\033[94mStabilise mode\033[0m')
            #channels[6] = 1400
            self.set_stabilize_mode()

        # if B
        if B_button==1 and front_montant :#and channels[6]==1400:    
            #print("Normal mode")
            self.get_logger().info('\033[94mNormal mode\033[0m')
            #channels[6] = 1500
            self.set_normal_mode()

        # If Elevation > / < --- Channel 3
        if self.LT==1 and self.RT==0:
            channels[2] = 1500 - k
            
        if self.RT == 1 and self.LT == 0:
            channels[2] = 1500 + k

        if (self.RT ==1 and self.LT ==1) or (self.RT ==0 and self.LT ==0):
            channels[2] = 1500

        # If Yaw --- Channel 4
        if JoystickLeft_horizontal != 0:
            channels[3] = int(1500 - JoystickLeft_horizontal*k)

        # If move --- Channel 5 & 6
        if JoystickRight_horizontal !=0 or JoystickRight_vertical !=0 :
            channels[4] = int(1500 + JoystickRight_vertical * k)
            channels[5] = int(1500 - JoystickRight_horizontal * k)

        # If Camera --- Channel 8
        if Cross_vertical !=0 :
            if False :  # pas besoin de bouger la caméra pour l'instant 
                channels[7] = 1500 + 100*int(Cross_vertical/abs(Cross_vertical))

        # If lights --- Channel 9 
        if self.Cross_horizontal !=0 :
            self.lum += -int(50 * self.Cross_horizontal)
            #channels[8] += -int(50 * self.Cross_horizontal)
        channels[8] = self.lum

        
        ### Control - Sending commands ###
        """
        Mavlink message
        """
        if self.use_transmitter:
            kal_msg = Twist()
            # cha 1 = pitch
            # cha 2 = roll
            # cha 3 = monter / descendre 
            # cha 4 = yaw 
            # cha 5 = av / rec
            # cha 6 = coté 
            # cha 7 = ?
            # cha 8 = camera
            # cha 9 = lights
            if self.armed:
                # Kalman filter 
                kal_msg.linear.x = (channels[4]-1500)/k
                kal_msg.linear.y = -(channels[5]-1500)/k
                kal_msg.linear.z = (channels[2]-1500)/k
                kal_msg.angular.z = -(channels[3]-1500)/k
                self.publisher_.publish(kal_msg)

                #test = [ 1500, 1500, 1600, 1500, 1500, 1500, 1500, 1500, 1100, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,1500]
                #self.send_mavlink_msg(test)
                #print(channels)
                self.send_mavlink_msg(channels)
                
                
            else :
                # Kalman filter 
                kal_msg.linear.x = 0.
                kal_msg.linear.y = 0.
                kal_msg.linear.z = 0.
                kal_msg.angular.z = 0.
                self.publisher_.publish(kal_msg)

                self.send_mavlink_msg(static_channels)

            

        

        ################### Auto mode ###################
        else :
            kal_msg = Twist()
            if self.armed:
                # Kalman filter 
                kal_msg.linear.x = self.linear[0,0]
                kal_msg.linear.y = self.linear[1,0]
                kal_msg.linear.z = self.linear[2,0]
                kal_msg.angular.z = self.angular[2,0]
                self.publisher_.publish(kal_msg)

                # Autopilot
                self.send_autopilot_commands()
            else:
                print('Auto mode but not armed')
                # Kalman filter 
                kal_msg.linear.x = 0.
                kal_msg.linear.y = 0.
                kal_msg.linear.z = 0.
                kal_msg.angular.z = 0.
                self.publisher_.publish(kal_msg)


        

def main(args=None):
    rclpy.init(args=args)
    print("Waiting for companion heartbeat")
    master.wait_heartbeat()
    print("Heartbeat detected")

    print('Control ROV par Joystick. Pensez à armer.')
    transmitter = Transmitter()
    rclpy.spin(transmitter)
    


    rclpy.shutdown()


if __name__ == "__main__":
    main()







