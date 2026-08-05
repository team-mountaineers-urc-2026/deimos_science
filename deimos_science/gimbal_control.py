#%%%%%%%%%%%%
#
#   Ros Notes
#   
#   sensor_msgs/JointState should serve our purposes for 2-axis control.
#   An example below.
#   
#   from sensor_msgs.msg import JointState
#   msg = JointState()
#   msg.name = ['motor1', 'motor2']
#   msg.position = [1.57, 3.14] #Radians
#   msg.velocity = []
#   msg.effort = []
#
#
#%%%%%%%%%%%%%

from dynamixel_lib import Dynamixel, U2D2
from dynamixel_lib import XL430W250
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String

from sensor_msgs.msg import JointState #new import

import robot_interfaces


# custom msg types

from robot_interfaces.msg import TargetedFloat
from robot_interfaces.msg import TargetedString

import time
import math

class GimbalControl(Node):

    def __init__(self):
        super().__init__('gimbal_controller')

        ##################
        #   PARAMETERS   #
        ##################

        # U2D2 Related Parameters
        # TODO FIX THIS PARAMETER
        self.declare_parameter("u2d2_port", '/dev/ttyUSB0') # check usb here when testing with "ls /dev" in deimos_science directory
                                                            # changing the port number based on what you see.

        # Motor IDs
        self.declare_parameter("gimbal_pitch_id", 11)
        self.declare_parameter("gimbal_yaw_id", 10) #arbitrary motor for now.
        self.declare_parameter("belly_gim_id", 12)

        # Function Specific Parameters

        # Gimbal
        self.declare_parameter("gimbal_0_pitch", 2048)
        self.declare_parameter("gimbal_0_yaw", 2048)
        
        #Belly Gimbal
        self.declare_parameter("belly_gim_0", 2048)

        # Save the values of the parameters
        self.u2d2_port          = self.get_parameter("u2d2_port").get_parameter_value().string_value
        self.gimbal_pitch_id    = self.get_parameter("gimbal_pitch_id").get_parameter_value().integer_value
        self.gimbal_yaw_id    = self.get_parameter("gimbal_yaw_id").get_parameter_value().integer_value
        self.initial_pitch_pos = self.get_parameter("gimbal_0_pitch").get_parameter_value().integer_value
        self.initial_yaw_pos = self.get_parameter("gimbal_0_yaw").get_parameter_value().integer_value

        self.belly_gim_id    = self.get_parameter("belly_gim_id").get_parameter_value().integer_value
        self.initial_belly_gim_pos = self.get_parameter("belly_gim_0").get_parameter_value().integer_value

        ######################
        #   HARDWARE SETUP   #
        ######################

        # U2D2 Stuff
        self.u2d2 = U2D2(self.u2d2_port, 57600)

        # Define the motors
        self.pitch_motor = Dynamixel(XL430W250, self.gimbal_pitch_id, self.u2d2)
        self.yaw_motor = Dynamixel(XL430W250, self.gimbal_yaw_id, self.u2d2)
        self.belly_gim = Dynamixel(XL430W250, self.belly_gim_id, self.u2d2)

        # Initialize the Gimbal (pitch)
        self.pitch_motor.write(XL430W250.TorqueEnable, 0)

        self.pitch_motor.write(XL430W250.OperatingMode, 3)
        self.pitch_motor.write(XL430W250.ProfileVelocity, 20)
        self.pitch_motor.write(XL430W250.ProfileAcceleration, 1)

        self.pitch_motor.write(XL430W250.TorqueEnable, 1)

        # Initialize the Gimbal (yaw)
        self.yaw_motor.write(XL430W250.TorqueEnable, 0)

        self.yaw_motor.write(XL430W250.OperatingMode, 3)
        self.yaw_motor.write(XL430W250.ProfileVelocity, 20)
        self.yaw_motor.write(XL430W250.ProfileAcceleration, 1)

        self.yaw_motor.write(XL430W250.TorqueEnable, 1)

        # Initialize the Gimbal (belly_gim)
        self.belly_gim.write(XL430W250.TorqueEnable, 0)

        self.belly_gim.write(XL430W250.OperatingMode, 3)
        self.belly_gim.write(XL430W250.ProfileVelocity, 20)
        self.belly_gim.write(XL430W250.ProfileAcceleration, 1)

        self.belly_gim.write(XL430W250.TorqueEnable, 1)

        #######################
        #   CONTROL CIRCUIT   #
        #######################

        # Subscriptions
        #old subscriber self.gimbal_move_pos_sub = self.create_subscription(Float32, '/gimbal_move_pos', self.gimbal_pos, 10)

        self.gimbal_move_pos_sub = self.create_subscription(JointState, '/gimbal_move_pos', self.gimbal_pos, 10)
        self.belly_move_pos_sub = self.create_subscription(Float32, '/belly_move_pos', self.belly_pos, 10)    


        self.reboot_sub     = self.create_subscription(Int32, '/motor_error_status_confirmation_subscriber', self.motor_reboot, 10)    

        #Error Status Publisher
        self.send_status_pub = self.create_publisher(TargetedString, '/motor_error_status_publisher', 10)
    

        # Timer
        self.control_timer = self.create_timer(0.5, self.control_timer_callback)

        # Control Variables
        self.next_pitch_pos = 2048
        self.next_yaw_pos   = 2048
        self.next_belly_pos = 2048

    #########################
    #   MAIN CONTROL LOOP   #
    #########################

    def control_timer_callback(self):
        
        # Gimbal
        self.pitch_motor.write(XL430W250.GoalPosition, self.next_pitch_pos)
        self.yaw_motor.write(XL430W250.GoalPosition, self.next_yaw_pos)
        self.belly_gim.write(XL430W250.GoalPosition, self.next_belly_pos)



    ######################
    #  Status CALLBACKS  #
    ######################

    # might be issues with this section.
    def motor_status(self):

        # Check for a jam with the gimbal motors
        pitch_status_id = Dynamixel.bytes_to_int(self.pitch_motor.read(XL430W250.HardwareErrorStatus)[0])
        yaw_status_id = Dynamixel.bytes_to_int(self.yaw_motor.read(XL430W250.HardwareErrorStatus)[0])
        belly_gim_status_id = Dynamixel.bytes_to_int(self.belly_gim.read(XL430W250.HardwareErrorStatus)[0])

        
        if pitch_status_id != 0:
            self.send_status_pub.publish(pitch_status_id._id)
        else:
            self.send_status_pub.publish(0)

        if yaw_status_id != 0:
            self.send_status_pub.publish(yaw_status_id._id)
        else:
            self.send_status_pub.publish(0)
        
        if belly_gim_status_id != 0:
            self.send_status_pub.publish(belly_gim_status_id._id)
        else:
            self.send_status_pub.publish(0)

    def motor_reboot(self, msg):
        msg_int = int(msg.data)
        #Gimbal reboot
        if msg_int == 10:
            self.pitch_motor.reboot()
            self.yaw_motor.reboot()
            self.belly_gim.reboot()

    ######################
    #  Gimbal CALLBACKS  #
    ######################

    def gimbal_pos(self, msg):
        
        #min max in degrees
        gimbal_yaw_max = 180
        gimbal_yaw_min = -180

        gimbal_pitch_min = -58
        gimbal_pitch_max = 90

        #180 degrees is front facing 
        #2048 is 180 degees in steps
        front_degree = 2048

        #msg_int = int(msg.data)
        msg_yaw = int(msg.position[0])
        msg_pitch = int(msg.position[1])
        

        if (gimbal_pitch_min <= msg_pitch <= gimbal_pitch_max): 
            pitch_Val = front_degree + (msg_pitch * 11.3177)
            self.next_pitch_pos = int(pitch_Val)

        if (gimbal_yaw_min <= msg_yaw <= gimbal_yaw_max):
            yaw_Val   = front_degree + (msg_yaw   * 11.3177)
            self.next_yaw_pos = int(yaw_Val)

    def gimbal_zero(self):
        self.pitch_motor.write(XL430W250.GoalPosition, 2048)
        self.yaw_motor.write(XL430W250.GoalPosition, 2048)

   ######################
    #  Gimbal CALLBACKS  #
    ######################
    def belly_pos(self, msg):
        
        #min max in degrees
        belly_ang_max = 60
        belly_ang_min = -55

        #180 degrees is front facing 
        #2048 is 180 degees in steps
        front_degree = 2048

        #msg_int = int(msg.data)
        msg_pos = int(msg.data)
        

        if (belly_ang_min <= msg_pos <= belly_ang_max): 
            ang_Val = front_degree + (msg_pos * 11.3177)
            self.next_belly_pos = int(ang_Val)

    def belly_zero(self):
        self.belly_gim.write(XL430W250.GoalPosition, 2048)

########################
#   HELPER FUNCTIONS   #
########################

def bytes_to_twos_complement(byte_list):
    return int.from_bytes(byte_list, byteorder='little', signed=True)

def get_cur_pos(motor, type):
    return bytes_to_twos_complement(motor.read(type.PresentPosition)[0])

############
#   MAIN   #
############

def main(args=None):
    rclpy.init(args=args)

    gimbal_controller = GimbalControl()

    rclpy.spin(gimbal_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gimbal_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
