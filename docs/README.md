# Smart Servo Project

This project implements the Pikes door-servo task with ESP32 firmware, Wokwi,
ROS 2 Control, keyboard control, and RViz visualization. See the
[system diagram](system_diagram.md) for the data flow.

## Requirements covered

| Requirement | Implementation |
|---|---|
| Continuous-rotation servo | Unbounded motor angle and signed speed in ESP32 firmware |
| 40:15 transmission | Door angle is `servo × 15/40` on the ESP32 side |
| Servo simulation | ESP32 and servo in Wokwi; a pure ROS fallback is included |
| ROS gateway | TCP server in firmware and reconnecting `tcp_bridge` node |
| Resource Manager/controller | `SmartServoSystem` hardware plugin and ROS 2 velocity controller |
| Keyboard | Left/right move, up/down stop, `+/-` change speed |
| Random endpoints | New closed/open limits generated on every reset |
| Torque stop | Synthetic torque crosses the threshold at each mechanical stop |
| Restricted motion | Calibrated limits clamp all subsequent movement |
| Visualization | RViz shows both arms and limits; a Wokwi stepper proxy shows continuous 360-degree rotation |

## Prerequisites

- Ubuntu 22.04 with ROS 2 Humble and `ros-humble-ros2-control`,
  `ros-humble-ros2-controllers`, and `ros-humble-rviz2`
- PlatformIO CLI
- Wokwi for VS Code and the Wokwi IoT Gateway for firmware mode

Install missing ROS dependencies and build both ROS packages:

```bash
cd /home/yousef/smart-servo/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Run without Wokwi

This mode exercises the complete ROS 2 Control path with a software copy of the
ESP32 state machine:

```bash
ros2 launch smart_servo_ros smart_servo.launch.py mode:=simulated
```

For a machine without a graphical display, append `rviz:=false`. Set
`keyboard:=false` when running commands from another terminal.

## Run the ESP32 firmware in Wokwi

1. Build the firmware:

   ```bash
   cd /home/yousef/smart-servo/firmware/smart_servo_esp32
   pio run
   ```

2. Open this firmware directory itself as the root folder in VS Code and run
   **Wokwi: Start Simulator**. Its `wokwi.toml` points to the generated binary,
   forwards local port 3333 to the ESP32, and `diagram.json` wires GPIO 5 to
   the virtual servo. GPIO 18/19 also drive an A4988 and stepper used only as
   the unlimited 360-degree visual proxy.
3. Wait for `TCP server started on port 3333` in the Wokwi serial monitor.
4. In a sourced ROS workspace, run:

   ```bash
   ros2 launch smart_servo_ros smart_servo.launch.py \
     mode:=wokwi host:=127.0.0.1 port:=3333
   ```

Wokwi for VS Code's bundled private gateway applies the forwarding rule. The
ROS bridge automatically reconnects if Wokwi restarts.

## Controls and expected behavior

- Left/right arrows (or `a`/`d`): continuous rotation
- Up/down arrows (or `s`): stop
- `+`/`-`: increase/decrease speed from 10 to 180 degrees/second
- `c`: find closed then open endpoint using the fake-torque threshold
- `r`: reset and randomize the hidden endpoints
- `q`: exit the keyboard node

Before calibration, motion is unlimited. Press `c`; the servo searches in both
directions and stores an endpoint span of 40–60 degrees (never more than the
required 60 degrees). RViz then shows both green endpoint markers and prevents
keyboard motion beyond them.

ROS 2 Control owns `servo_joint/velocity`; the Resource Manager reads motor
position/velocity plus geared door position/fake effort. Inspect them with:

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic echo /joint_states
```

## Suggested demonstration video

Record one short continuous take:

1. Show both blue motor and orange geared-door arms moving at different rates.
2. Press `+` and `-` while rotating.
3. Press `r`, then `c`, and wait for both detected limits to appear.
4. Hold left and right to demonstrate that neither detected limit can be crossed.
5. Briefly show `ros2 control list_controllers` to prove the controller is active.
