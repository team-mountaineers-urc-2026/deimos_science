# 2-axis Gimbal Controller Notes

ROS 2 node for controlling a 2-axis Dynamixel gimbal (yaw / pitch) using
`sensor_msgs/JointState`.

---

## TODO
- Implement error handling for both motors  
  - Currently only the yaw motor has a subscriber; however, communication is shared between both motors on the 2XL bus, so pitch control still functions.
- Add GUI support
- Expand potential proportional control methods.
- Photo of how the Gimbal '0' positions should be.

---

## Requirements
The following repositories must be built and sourced:

- `Dynamixel_lib` -> Trevor's Workspace Branch
- `workspace_deimos`
- `robot_interfaces`
- `deimos_science`

---

## How to Run

### Gimbal '0' Positions
Before starting the gimbal, make sure the 0 positions of the motors start aligned 
like the photos below. This can be checked with dynamixel wizard.
![IMG_8815_cropped](https://github.com/user-attachments/assets/f5a94722-7ef6-4649-84d5-e5406da38864)
<img width="4032" height="3024" alt="Belly_Gimbal" src="https://github.com/user-attachments/assets/ca218381-11ec-4ca9-b4f0-15bb0d277e75" />

### Terminal 1 – Start the Gimbal Controller
```
source /opt/ros/humble/setup.bash
source ~/workspace-newrobot2026/install/setup.bash
source ~/deimos_science/install/setup.bash
ros2 run daedalus_science gimbal_control
```
### Terminal 2 - Send Position Commands
Control Tower Gimbal.
```
ros2 topic pub --once /gimbal_move_pos sensor_msgs/msg/JointState "{name:['yaw','pitch'], position:[0.0,0.0]}"
```
Replace the values inside ```position[]``` with whichever two angles you would like the motor to move to. 
##Joint limits:
  - Yaw: [-180°, 180°]
  - Pitch: [-58°, 90°]
  Note: These limits are currently enforced in the node and may change in the future.

Control Belly Gimbal.
```
ros2 topic pub --once /belly_move_pos std_msgs/msg/Float32 "{data: 0.0}"
```
Replace the value ```data: 0.0``` with whichever angle you would like the motor to move to.
##Joint limits:
  - [-15°, 90°]
Note: Subject to change as current CAD print does not work.

## Joint States Topic / Interface
```
  from sensor_msgs.msg import JointState
  msg = JointState()
  msg.name = ['motor1', 'motor2']
  msg.position = [x, y] 
  msg.velocity = []
  msg.effort = []
```

## Float32 Topic /Interface
```
  from std_msgs.msg import Float32

  msg = Float32()
  msg.data = x
```

## Ros Subscriptions
- '/gimbal_move_pos' -> Motor Position Control Topic
- '/motor_error_status_confirmation_subscriber'
- '/motor_error_status_publisher'

## Hardware Assumptions
  - Motor Model: Dynamixel 2XL430-W250
  - Yaw Motor: 10
  - Pitch Motor: 11
  - Baudrate (both motors): 57600
## Troubleshooting
  - `/dev/ttyUSB*` not found, you will need to manually check what the port the motors are plugged into.
  - Motors unresponsive, either you are inputting past limits or the ROS message didn't reach the motor (common).
