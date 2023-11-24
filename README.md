# whi_motion_teleop
tele-operation for manipulating movable robots under ROS 1. It publishes geometry_msgs/Twist Message with user-defined topic(default: cmd_vel)

## Dependencies
```
git clone https://github.com/xinjuezou-whi/whi_interfaces.git
```

## Manipulating keys definition
![Untitled Diagram drawio](https://user-images.githubusercontent.com/72239958/202851886-e404eafc-dae1-488b-bbb4-356eb4cca441.png)

## Engineering maintenance keys definition
| Key | Function                                         |
|-----|--------------------------------------------------|
| 0   | neutralize all engineering operations             |
| 3   | print the yaw value of IMU                       |
| 4   | reset the IMU                                    |
| 5   | print the value of the encoder                       |
| 6   | reset the value of the encoder                       |
| 7   | calibrate the map between reference and PWM duty |
| 8   | clear calibration results                        |

## Params
```
whi_motion_teleop:
  command_frequency: 20 # Hz
  motion_state_topic: motion_state
  linear: # m/s
    min: 0.08
    max: 0.3
    step: 0.01 
  angular: # rad/s
    min: 0.1
    max: 1.0
    step: 0.1
```
