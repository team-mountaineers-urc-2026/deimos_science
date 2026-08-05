from dynamixel_lib import Dynamixel, U2D2
from dynamixel_lib import XL430W250
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String, Bool
import robot_interfaces


# custom msg types
from robot_interfaces.msg import TargetedFloat
from robot_interfaces.msg import TargetedString
from robot_interfaces.msg import TargetedBool

from sensor_msgs.msg import JointState 

import time
import math

class DynamixelInterface(Node):

    def __init__(self):
        super().__init__('dynamixel_interface')

        ##################
        #   PARAMETERS   #
        ##################

        # U2D2 Related Parameters
        self.declare_parameter("u2d2_port", '/dev/ttyUSB0')

        # Motor IDs
        self.declare_parameter("gimbal_pitch_id", 11)
        self.declare_parameter("gimbal_yaw_id", 10) #arbitrary motor for now.
        self.declare_parameter("belly_gim_id", 12)

        self.declare_parameter("pump_one_id", 1)#site one
        self.declare_parameter("pump_two_id", 2)#site two
        self.declare_parameter("cache_one_id", 3)#currently buad at 57360(default needs to change later) also upper cache
        self.declare_parameter("cache_two_id", 4)#lower cache
        self.declare_parameter("carousel_id", 5)
        #scoop drum not declared here due to it being a myactuator motor and not a dynamixel motor


        # Function Specific Parameters

        # Carousel
        self.declare_parameter("cuvette_0_pos", 0)

        # Scoop
        # self.declare_parameter("max_scoop_vel", 100)

        # Cache
        self.declare_parameter("cache_one_open_deg", 186.15)
        self.declare_parameter("cache_one_closed_deg", 272.5)
        self.declare_parameter("cache_two_open_deg", 64.16)
        self.declare_parameter("cache_two_closed_deg", 0.44)

        # Gimbal
        self.declare_parameter("gimbal_0_pitch", 2048)
        self.declare_parameter("gimbal_0_yaw", 2048)
        self.declare_parameter("belly_gim_0", 2048)

        # Save the values of the parameters
        self.gimbal_pitch_id    = self.get_parameter("gimbal_pitch_id").get_parameter_value().integer_value
        self.gimbal_yaw_id    = self.get_parameter("gimbal_yaw_id").get_parameter_value().integer_value
        self.initial_pitch_pos = self.get_parameter("gimbal_0_pitch").get_parameter_value().integer_value
        self.initial_yaw_pos = self.get_parameter("gimbal_0_yaw").get_parameter_value().integer_value
        self.belly_gim_id    = self.get_parameter("belly_gim_id").get_parameter_value().integer_value
        self.initial_belly_gim_pos = self.get_parameter("belly_gim_0").get_parameter_value().integer_value


        # Save the values of the parameters
        self.u2d2_port = self.get_parameter("u2d2_port").get_parameter_value().string_value

        self.carousel_id  = self.get_parameter("carousel_id").get_parameter_value().integer_value
        self.pump_one_id  = self.get_parameter("pump_one_id").get_parameter_value().integer_value
        self.pump_two_id  = self.get_parameter("pump_two_id").get_parameter_value().integer_value

        self.initial_carousel_pos = self.get_parameter("cuvette_0_pos").get_parameter_value().integer_value
        # self.max_scoop_vel = self.get_parameter("max_scoop_vel").get_parameter_value().integer_value

        self.cache_one_id = self.get_parameter("cache_one_id").get_parameter_value().integer_value
        self.cache_two_id = self.get_parameter("cache_two_id").get_parameter_value().integer_value
        
        self.cache_one_open_deg = self.get_parameter("cache_one_open_deg").get_parameter_value().double_value
        self.cache_one_closed_deg = self.get_parameter("cache_one_closed_deg").get_parameter_value().double_value
        self.cache_two_open_deg = self.get_parameter("cache_two_open_deg").get_parameter_value().double_value
        self.cache_two_closed_deg = self.get_parameter("cache_two_closed_deg").get_parameter_value().double_value

        ######################
        #   HARDWARE SETUP   #
        ######################

        # U2D2 Stuff
        self.u2d2 = U2D2(self.u2d2_port, 57600)#number is the baud rate

        # Define the motors
        self.carousel    = Dynamixel(XL430W250, self.carousel_id, self.u2d2)
        self.pump_one    = Dynamixel(XL430W250, self.pump_one_id, self.u2d2)
        self.pump_two    = Dynamixel(XL430W250, self.pump_two_id, self.u2d2)
        self.cache_one   = Dynamixel(XL430W250, self.cache_one_id, self.u2d2)
        self.cache_two   = Dynamixel(XL430W250, self.cache_two_id, self.u2d2)
        self.pitch_motor = Dynamixel(XL430W250, self.gimbal_pitch_id, self.u2d2)
        self.yaw_motor = Dynamixel(XL430W250, self.gimbal_yaw_id, self.u2d2)
        self.belly_gim = Dynamixel(XL430W250, self.belly_gim_id, self.u2d2)

        self.xl_motors = [self.carousel, self.pump_one, self.pump_two, self.cache_one, self.cache_two, self.pitch_motor, self.yaw_motor]


        # Initialize the Motors
        for motor in self.xl_motors: motor.write(XL430W250.TorqueEnable, 0)

        # Carousel
        self.carousel.write(XL430W250.OperatingMode, 3)
        self.carousel.write(XL430W250.ProfileVelocity, 20)
        self.carousel.write(XL430W250.ProfileAcceleration, 1)


        # Pumps
        self.pump_one.write(XL430W250.OperatingMode, 4)
        self.pump_two.write(XL430W250.OperatingMode, 4)

        # Cache
        self.cache_one.write(XL430W250.OperatingMode, 3)
        self.cache_one.write(XL430W250.ProfileVelocity, 20)
        self.cache_one.write(XL430W250.ProfileAcceleration, 1)
        self.cache_two.write(XL430W250.OperatingMode, 3)
        self.cache_two.write(XL430W250.ProfileVelocity, 20)
        self.cache_two.write(XL430W250.ProfileAcceleration, 1)

        # Initialize the Gimbal (pitch)
        self.pitch_motor.write(XL430W250.TorqueEnable, 0)

        self.pitch_motor.write(XL430W250.OperatingMode, 3)
        self.pitch_motor.write(XL430W250.ProfileVelocity, 20)
        self.pitch_motor.write(XL430W250.ProfileAcceleration, 1)

        # Initialize the Gimbal (yaw)
        self.yaw_motor.write(XL430W250.TorqueEnable, 0)

        self.yaw_motor.write(XL430W250.OperatingMode, 3)
        self.yaw_motor.write(XL430W250.ProfileVelocity, 20)
        self.yaw_motor.write(XL430W250.ProfileAcceleration, 1)
    
        # Initialize the Gimbal (belly_gim)
        self.belly_gim.write(XL430W250.TorqueEnable, 0)

        self.belly_gim.write(XL430W250.OperatingMode, 3)
        self.belly_gim.write(XL430W250.ProfileVelocity, 20)
        self.belly_gim.write(XL430W250.ProfileAcceleration, 1)

        self.belly_gim.write(XL430W250.TorqueEnable, 1)

        for motor in self.xl_motors: motor.write(XL430W250.TorqueEnable, 1)

        #######################
        #   CONTROL CIRCUIT   #
        #######################

        # Subscriptions
        self.cuvette_sub = self.create_subscription(Int32, 'carousel/curr_cuvette', self.cuvette_callback, 10)

        #single pump subscriber
        self.pump_sub      = self.create_subscription(TargetedFloat, 'pump/milliliters', self.pump_callback, 10)
        self.cache_sub     = self.create_subscription(TargetedBool, 'cache_closed', self.cache_callback, 10)
        self.gimbal_move_pos_sub = self.create_subscription(JointState, '/gimbal_move_pos', self.gimbal_pos, 10)
        self.belly_move_pos_sub = self.create_subscription(Float32, '/belly_move_pos', self.belly_pos, 10)    


        self.reboot_sub    = self.create_subscription(Int32, '/motor_error_status_confirmation_subscriber', self.motor_reboot, 10) 


        #Error Status Publisher
        self.send_status_pub = self.create_publisher(TargetedString, '/motor_error_status_publisher', 10)

        # Dynamixel Control Variables
        self.next_carousel_pos = self.initial_carousel_pos
        self.next_pump_one_pos = get_cur_pos(self.pump_one, XL430W250)
        self.next_pump_two_pos = get_cur_pos(self.pump_two, XL430W250)

        self.cache_one_state = int(self.cache_one_closed_deg/0.088)
        self.cache_two_state = int(self.cache_two_closed_deg/0.088)

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
        
        # Carousel
        self.carousel.write(XL430W250.GoalPosition, self.next_carousel_pos)

        # Pumps
        self.pump_one.write(XL430W250.GoalPosition, self.next_pump_one_pos)
        self.pump_two.write(XL430W250.GoalPosition, self.next_pump_two_pos)

        # Cache
        self.cache_one.write(XL430W250.GoalPosition, self.cache_one_state)
        self.cache_two.write(XL430W250.GoalPosition, self.cache_two_state)

        # Gimbal
        self.pitch_motor.write(XL430W250.GoalPosition, self.next_pitch_pos)
        self.yaw_motor.write(XL430W250.GoalPosition, self.next_yaw_pos)
        self.belly_gim.write(XL430W250.GoalPosition, self.next_belly_pos)


    ##########################
    #   CAROUSEL CALLBACKS   #
    ##########################

    def cuvette_callback(self, msg):
        
        # Sanitize the Data
        if msg.data < 0 or msg.data > 7:
            self.get_logger().error(f"Invalid Cuvette Selected: {msg.data}")
            return

        # Calculate the position to go to
        cuvette_pos_deg = (msg.data * 45)
        self.next_carousel_pos = 4095 - int(cuvette_pos_deg/0.088) - self.initial_carousel_pos

    #######################
    #   Scoop Callbacks   #
    #######################

    # def front_back_scoop_callback(self, msg):
    #     # Convert the proportion of max speed to a value
    #     motor_speed = int(msg.data * self.max_scoop_vel)

    #     # Cap the value if we are sending one too high
    #     if motor_speed > 1023:
    #         motor_speed = 1023

    #     elif motor_speed < -1023:
    #         motor_speed = -1023

    #     print(msg.target)

    #     if msg.target == 7:
    #         print("7")
    #         self.front_scoop_speed = motor_speed
    #     elif msg.target == 8:
    #         print("8")
    #         self.back_scoop_speed = motor_speed

    ######################
    #   PUMP CALLBACKS   #
    ######################

    def pump_callback(self, msg):
        
        self.get_logger().info(f"{msg.target}")
        self.get_logger().info(f"{msg.data}")

        amt = msg.data
        pulses = int(amt/0.204*4096)    # 0.204 mL per turn [4095 total turns] !!!!! new value found with testing pumped 5mL, got 6mL so multed the old const (0.17) by 1.2 to get the new const (0.204) to be more accurate
        pulses *= -1    # Negative to make sure the pump rotates the correct direction
        self.get_logger().info(f"Amount of turns: {pulses}")

        # Depending on the pump we have, send the command to a different one
        match msg.target:
            
            case 1:
                self.next_pump_one_pos = get_cur_pos(self.pump_one, XL430W250) + pulses

            case 2:
                self.next_pump_two_pos = get_cur_pos(self.pump_two, XL430W250) + pulses

            case _:
                self.get_logger().error(f"Unknown Pump: {msg.target} specified")


    ######################
    #  Status CALLBACKS  #
    ######################

    def motor_status(self):

        #Should check the motors for any error status/jam that happens
        for motor in self.xl_motors:
            motor_status_id = Dynamixel.bytes_to_int(motor.read(XL430W250.HardwareErrorStatus)[0])
            if motor_status_id != 0:
                self.send_status_pub.publish(motor._id)
            else:
                self.send_status_pub.publish(0)

    def motor_reboot(self, msg):
        msg_int = int(msg.data)

        #pumps and carousel
        if 1 <= msg_int <= 5:
            self.xl_motors[(msg_int - 1)].reboot()


    ##########################
    #     CACHE CALLBACKS    #
    ########################## 

    def cache_callback(self, msg : TargetedBool):
        #check target is valid
        if msg.target not in [1, 2]:
            self.get_logger().error(f"Unknown Cache: {msg.target}")
            return
        
        #determine if it wants open or close and set the target position accordingly
        if msg.data:
            self.get_logger().info(f"Closing Cache {msg.target}")
            if msg.target == 1:
                target_position = int(self.cache_one_closed_deg/0.088)
            else:
                target_position = int(self.cache_two_closed_deg/0.088)
        else:
            self.get_logger().info(f"Opening Cache {msg.target}")
            if msg.target == 1:
                target_position = int(self.cache_one_open_deg/0.088)
            else:
                target_position = int(self.cache_two_open_deg/0.088)
                print(target_position)

        #get the targeted cache to move to the desired position
        if msg.target == 1:
            self.cache_one_state = target_position
        elif msg.target == 2:
            self.cache_two_state = target_position


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

    dynamixel_interface = DynamixelInterface()

    rclpy.spin(dynamixel_interface)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dynamixel_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


