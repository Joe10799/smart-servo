# Smart Servo Door Controller

ESP32 firmware and ROS 2 Control integration for a continuous-rotation door
servo with automatic endpoint detection.

The servo drives a door through a 40:15 transmission. The firmware maintains
the continuous motor angle, applies the transmission ratio, simulates load
torque at the door stops, detects randomized open/closed endpoints, and
prevents motion beyond calibrated limits.

The system has two execution modes:

- **ROS simulation:** runs the ESP32 state machine as a ROS node.
- **Wokwi firmware:** runs the real ESP32 firmware in Wokwi and connects it to
  ROS through a TCP gateway.

## Requirement coverage

| Requirement | Implementation |
|---|---|
| ESP32 firmware | PlatformIO/Arduino project in `firmware/smart_servo_esp32` |
| Continuous-rotation servo | Unbounded motor angle with signed speed and direction |
| 40:15 transmission | Door angle calculated as `motor angle × 15/40` |
| Servo simulation | ESP32, PWM servo, and 360° visual proxy in Wokwi |
| ROS gateway | TCP server in firmware and reconnecting ROS TCP bridge |
| ROS 2 Control | Resource Manager hardware plugin and velocity controller |
| Keyboard control | Direction, stop, speed, calibration, and reset controls |
| Random endpoints | New closed/open positions generated on reset |
| Endpoint detection | Automatic search using simulated torque |
| Motion restriction | Door motion clamped to detected limits |
| Visualization | Wokwi hardware view and RViz motor/door view |
| Documentation | This README and `docs/system_diagram.md` |

## Mechanical model and assumptions

The implementation assumes a 15-tooth input gear drives a 40-tooth output
gear:

```text
gear scale = 15 / 40 = 0.375
door angle = motor angle × 0.375
```

The door therefore moves 37.5% as far and as fast as the motor. For example:

```text
Motor angle:  120°
Door angle:    45°
```

The transmission layout was not specified, so the model keeps motor and door
angles with the same sign. Reversing the output direction would only require a
negative gear scale.

## Direction convention

Angles follow the ROS right-hand convention, viewed from above the mechanism
(`+Z` toward the origin):

```text
                       +Y
                        ^
                        |
        180°     <------o------>  0° / +X
                        |
                        v
                       -Y

Positive angle: counter-clockwise (CCW)
Negative angle: clockwise (CW)
```

The keyboard uses the following mapping:

| Key | Result |
|---|---|
| Right arrow or `d` | Start positive/CCW rotation |
| Left arrow or `a` | Start negative/CW rotation |
| Up arrow, down arrow, or `s` | Stop and hold the current angle |
| `+` | Increase speed by 10°/s, up to 180°/s |
| `-` | Decrease speed by 10°/s, down to 10°/s |
| `c` | Stop manual motion and start endpoint calibration |
| `r` | Reset state and generate new hidden endpoints |
| `q` | Send stop, then exit the keyboard node |

Direction commands are latched: pressing left or right starts continuous
rotation. The servo continues until a stop command, opposite-direction command,
calibration sequence, or calibrated endpoint stops it.

## Expected behavior

### Before calibration

- The servo is free to rotate continuously in either direction.
- The blue motor arrow in RViz rotates.
- The orange door arrow rotates in the same direction at 37.5% of the motor
  rate.
- No green endpoint markers are visible.
- `calibrated=0` and `detecting=0` are reported in the state.

### During calibration

Press `c` once and do not send other movement commands until calibration ends.

1. Manual velocity is set to zero.
2. The door searches in the negative/CW direction for the closed stop.
3. Simulated torque rises at the stop.
4. The closed position is recorded.
5. The door reverses and searches for the positive/CCW open stop.
6. The open position is recorded.
7. Motion stops and calibrated limits become active.

The randomized endpoint separation is between 40° and 60°. Calibration
normally takes about 10–15 seconds, depending on the generated endpoints and
simulation speed.

### After calibration

- Green closed/open markers appear in RViz.
- Left/right movement remains available inside the calibrated range.
- When the orange door arrow reaches a limit, direction becomes zero.
- Further commands toward that limit cannot move the door beyond it.
- `calibrated=1` and `detecting=0` are reported in the state.

### Reset

Pressing `r`:

- stops motion;
- sets motor and door angles to zero;
- restores the default speed of 40°/s;
- clears detected limits;
- generates a new randomized endpoint pair;
- returns the system to the uncalibrated state.

## Visual guide

### RViz

RViz is the main system visualization.

| Display | Meaning |
|---|---|
| Blue arrow | Continuous motor/servo angle before transmission |
| Orange arrow | Door angle after applying `15/40` |
| Green arrows | Detected closed and open limits |
| White text | Direction, speed, angles, torque, and calibration state |

Both the blue and orange objects are rotating arms; their arrowheads show the
current angular position, not a keyboard instruction.

### Wokwi

Wokwi's standard servo component has a physical visual range of 0–180°, while
the firmware model uses an unlimited continuous angle. The diagram therefore
contains both:

| Component | Purpose |
|---|---|
| Orange standard servo | Actual PWM servo interface on ESP32 GPIO 5 |
| Blue stepper and A4988 | Display-only 360° proxy for the continuous motor angle |

The proxy direction is driven by GPIO 18 and its step signal by GPIO 19. It
mirrors the firmware's `servo_angle`; it does not replace the servo, gearbox,
torque model, endpoint detection, or ROS interface.

## System architecture

```text
Keyboard node
    |
    | /servo_velocity_controller/commands
    v
ROS 2 velocity controller
    |
    v
ROS 2 Control Resource Manager
    |
    v
SmartServoSystem hardware plugin
    |
    | /smart_servo/cmd
    v
+---------------------------+   or   +----------------------------+
| ROS ESP32 simulation node |        | TCP bridge -> Wokwi ESP32 |
+---------------------------+        +----------------------------+
    |                                      |
    +----------- /smart_servo/state -------+
                         |
                         +--> Resource Manager -> /joint_states
                         |
                         +--> RViz visualizer -> /smart_servo/markers
```

The Resource Manager exposes:

| Interface | Type | Description |
|---|---|---|
| `servo_joint/velocity` | Command | Signed motor velocity |
| `servo_joint/position` | State | Continuous motor angle |
| `servo_joint/velocity` | State | Current motor velocity |
| `door_joint/position` | State | Geared door angle |
| `door_joint/effort` | State | Simulated torque |

The component-level diagram is in
[`docs/system_diagram.md`](docs/system_diagram.md).

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- `ros-humble-ros2-control`
- `ros-humble-ros2-controllers`
- `ros-humble-rviz2`
- PlatformIO CLI
- Wokwi for VS Code for firmware simulation

Install missing ROS package dependencies with `rosdep` as shown below.

## Build the ROS workspace

```bash
cd ~/smart-servo/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

Source both ROS and the workspace in every new terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/smart-servo/ros2_ws/install/setup.bash
```

## Run the ROS-only simulation

Terminal 1:

```bash
cd ~/smart-servo/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch smart_servo_ros smart_servo.launch.py \
  mode:=simulated keyboard:=false
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/smart-servo/ros2_ws/install/setup.bash
ros2 run smart_servo_ros keyboard_node
```

Keep Terminal 2 focused while using the controls. Use `Ctrl+C` to stop each
process.

To run without RViz:

```bash
ros2 launch smart_servo_ros smart_servo.launch.py \
  mode:=simulated keyboard:=false rviz:=false
```

## Run the ESP32 firmware with Wokwi and ROS

### 1. Build the firmware

```bash
cd ~/smart-servo/firmware/smart_servo_esp32
pio run
```

Rebuild and restart Wokwi after every firmware change.

### 2. Start Wokwi

Open the firmware directory itself as a separate VS Code workspace:

```bash
code -n ~/smart-servo/firmware/smart_servo_esp32
```

In that window:

1. Install/activate the Wokwi for VS Code extension.
2. Press `F1`.
3. Select **Wokwi: Start Simulator**.
4. Keep the simulator tab visible.
5. Wait for:

   ```text
   WiFi connected. IP=...
   TCP server started on port 3333
   ```

The forwarding rule in `wokwi.toml` maps local port `3333` to port `3333` on
the simulated ESP32.

Confirm the local listener if needed:

```bash
ss -ltn | grep 3333
```

Expected:

```text
LISTEN ... 127.0.0.1:3333 ...
```

### 3. Start ROS in Wokwi mode

Terminal 1:

```bash
cd ~/smart-servo/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch smart_servo_ros smart_servo.launch.py \
  mode:=wokwi host:=127.0.0.1 port:=3333 keyboard:=false
```

Expected bridge message:

```text
Connected to Wokwi ESP32 TCP 127.0.0.1:3333
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/smart-servo/ros2_ws/install/setup.bash
ros2 run smart_servo_ros keyboard_node
```

In this mode, motion, transmission scaling, torque simulation, and endpoint
detection run in the ESP32 firmware. ROS 2 Control sends commands and reads the
reported state through the TCP bridge.

If local port `3333` is occupied, change only the `from` port in
`wokwi.toml`, for example:

```toml
[[net.forward]]
from = "localhost:3334"
to = "target:3333"
```

Then launch ROS with `port:=3334`.

## Verify ROS 2 Control

Run these commands in another sourced terminal:

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states
ros2 topic echo /smart_servo/state
```

Expected active controllers:

```text
joint_state_broadcaster       active
servo_velocity_controller    active
```

Example stopped, uncalibrated state:

```text
STATE servo=0.00 door=0.00 speed=40.00 dir=0 torque=10.00 \
calibrated=0 detecting=0 closed=0.00 open=0.00
```

Useful state fields:

| Field | Meaning |
|---|---|
| `servo` | Continuous motor angle in degrees |
| `door` | Geared door angle in degrees |
| `speed` | Configured positive speed magnitude |
| `dir` | `-1` CW, `0` stopped, `1` CCW |
| `torque` | Simulated door load |
| `calibrated` | Whether detected limits are active |
| `detecting` | Whether calibration is running |
| `closed`, `open` | Detected door limits |

## Troubleshooting

### The servo continues moving

This is expected after a left/right command because direction commands are
latched. Press `s`, up, or down to stop. A stopped state reports `dir=0`.

### The standard Wokwi servo jumps after 180°

That is a limitation of the standard Wokwi servo graphic. Use the blue stepper
proxy or RViz to observe continuous rotation.

### RViz shows no movement

Check that `/smart_servo/state` is updating and that `dir` is not zero:

```bash
ros2 topic echo /smart_servo/state
```

### ROS cannot connect to Wokwi

Check that:

1. Wokwi is still running and its tab is visible.
2. The firmware printed `TCP server started on port 3333`.
3. `ss -ltn | grep 3333` shows a local listener.
4. No previous ROS launch is still connected to the single ESP32 TCP server.

The bridge retries automatically after a disconnection.

## Repository layout

```text
firmware/smart_servo_esp32/
  src/main.cpp                  ESP32 motion, TCP, torque, and calibration
  diagram.json                  Wokwi servo and 360° proxy circuit
  platformio.ini                ESP32 build configuration
  wokwi.toml                    Wokwi binaries and TCP forwarding

ros2_ws/src/smart_servo_hardware/
  src/smart_servo_system.cpp    ROS 2 Control hardware plugin
  smart_servo_hardware.xml      Plugin registration

ros2_ws/src/smart_servo_ros/
  smart_servo_ros/              Keyboard, transport, simulation, visualization
  config/controllers.yaml       Controller configuration
  description/smart_servo.urdf  Hardware interfaces
  launch/                       System launch and RViz configuration
  tests/                        Keyboard and state-parser tests
```

Generated directories (`.pio`, `build`, `install`, `log`, caches) are excluded
from source control and can be recreated from the commands above.

## Demonstration video sequence

A short continuous recording should show:

1. Wokwi running the ESP32 firmware and ROS connected over TCP.
2. `ros2 control list_controllers` with both controllers active.
3. Right/left continuous rotation and stop.
4. Speed changes using `+` and `-`.
5. The Wokwi blue 360° proxy rotating beyond 180°.
6. Different motor and geared-door rates in RViz.
7. Reset followed by endpoint calibration.
8. Torque rising and both endpoint markers appearing.
9. Movement stopping at both calibrated limits.
