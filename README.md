# ak-c-drivers

C driver library for CubeMars AK series motors over CAN bus.
Supports both MIT mode and Servo mode communication protocols.

---

## Overview

* Provides message generation and decoding for CubeMars AK brushless motors
* Intended for robotics engineers and embedded developers using Linux with SocketCAN
* Fills the gap between the manufacturer PDF and working code — no dependencies, no bloat

---

## Features

* MIT mode: position/velocity/torque control with configurable gains (kp, kd, t_ff)
* Servo mode: duty cycle, current, RPM, position, and position+speed control
* CAN feedback decoding (position, speed, current, temperature, error)
* Supports 10 motor models out of the box (AK10-9, AK40-10, AK60-6, AK70-10, AK80-6/8/9/64, AK45-10/36)
* Interactive ncurses TUI demo with live CAN monitoring

---

## Tech Stack

* Language: C17
* Infrastructure: Linux SocketCAN
* Other tools: CMake 3.28+, ncurses (demo only)

---

## Installation

```bash
# Build library and demo
cmake -B build
cmake --build build
```

The static library `libcubemars-drivers.a` and the `demo` binary will be placed in `build/`.

To link the library in your own project:

```cmake
add_subdirectory(ak-c-drivers)
target_link_libraries(your_target PRIVATE cubemars-drivers)
```

---

## Configuration

No environment variables are required. The CAN interface name and motor ID are set at runtime (either in code or via the demo TUI).

To bring up a CAN interface before use:

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

---

## Usage

### Servo mode — send a position command

```c
#include "ak_servo.h"
#include <linux/can.h>

uint8_t motor_id = 1;
struct can_frame frame;
AKMotorServoMessage msg;

msg = generate_position_message(motor_id, 90.0f);  // 90 degrees
frame.can_id  = msg.id | CAN_EFF_FLAG;
frame.can_dlc = 8;
memcpy(frame.data, msg.data, 8);
// write(can_socket, &frame, sizeof(frame));
```

### MIT mode — torque-controlled command

```c
#include "ak_mit.h"
#include <linux/can.h>

uint8_t motor_id = 1;
MotorModel model = AK80_9;   // pick your motor
struct can_frame frame;
AKMotorMITMessage msg;

// Enter MIT mode first
msg = generate_mit_enter_message(motor_id);
// ... write to socket ...

// Send control command: hold position 0 with soft gains
msg = generate_mit_command_message(motor_id, model,
        /*p_des=*/0.0f, /*v_des=*/0.0f,
        /*kp=*/5.0f,    /*kd=*/0.5f,
        /*t_ff=*/0.0f);
frame.can_id  = msg.id;
frame.can_dlc = 8;
memcpy(frame.data, msg.data, 8);
// write(can_socket, &frame, sizeof(frame));
```

### Decoding feedback (Servo mode)

```c
#include "ak_servo.h"

struct can_frame rx;
// read(can_socket, &rx, sizeof(rx));

ServoCANFeedback fb = decode_servo_can_feedback(rx.data);
printf("pos=%.2f deg  speed=%d ERPM  current=%.2f A  temp=%d°C\n",
       fb.position, fb.speed, fb.current, fb.temperature);
```

---

## ROS 2

The `ros/` directory contains three ROS 2 packages:

| Package | Description |
|---------|-------------|
| `cubemars_msgs` | Message and service definitions |
| `cubemars_controller` | Node that bridges ROS topics/services to the CAN bus |
| `cubemars_rviz_panel` | RViz 2 panel for interactive motor control and live feedback |

### Prerequisites

Bring up the CAN interface before launching (requires root):

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

### Build

```bash
cd ros
colcon build
source install/setup.bash
```

### Launch

```bash
ros2 launch cubemars_controller cubemars_controller.launch.py
```

Motor model and CAN interface are configured in `ros/cubemars_controller/config/config.yaml`.

### Services

**Enter MIT mode** (must be called before sending commands):

```bash
ros2 service call /cubemars_controller_node/enter_mit cubemars_msgs/srv/EnterMIT "{motor_id: 1}"
```

**Exit MIT mode:**

```bash
ros2 service call /cubemars_controller_node/exit_mit cubemars_msgs/srv/ExitMIT "{motor_id: 1}"
```

**Set zero position** (saves current position as the new zero):

```bash
ros2 service call /cubemars_controller_node/set_zero cubemars_msgs/srv/SetZero "{motor_id: 1}"
```

### Topics

**Send an MIT command** (publishes to the subscriber):

```bash
ros2 topic pub /cubemars_controller_node/mit_command cubemars_msgs/msg/MITCommand \
  "{motor_id: 1, p_des: 0.0, v_des: 0.0, kp: 10.0, kd: 1.0, t_ff: 0.0}"
```

**Monitor motor feedback** (only published when the motor replies to a command):

```bash
ros2 topic echo /cubemars_controller_node/mit_feedback
```

### RViz Panel

`cubemars_rviz_panel` is a pluginlib-based RViz 2 panel that lets you control and monitor motors interactively without writing any code.

**Features**
- Add any number of motors by ID (1–127)
- Per-motor MIT command inputs: position, velocity, kp, kd, torque feed-forward, send rate
- Send a single command or start/stop continuous publishing at a configurable rate
- Per-motor service buttons: Enter MIT, Exit MIT, Set Zero
- Live feedback display: position, velocity, torque (from `/cubemars_controller_node/mit_feedback`)
- Global "Enter MIT (all)" / "Exit MIT (all)" buttons
- Panel state (which motor IDs are open) is saved and restored with the RViz config

**Installation**

The panel requires `rviz2` and Qt 5. Build with:

```bash
colcon build --packages-up-to cubemars_rviz_panel
source install/setup.bash
```

(Running `colcon build` from `ros/` without arguments builds all three packages including the panel.)

**Opening the panel in RViz 2**

1. Start RViz 2 (the `cubemars_controller` node must already be running so the panel can reach its topics and services).
2. In the menu bar: **Panels → Add New Panel**.
3. Select `cubemars_rviz_panel / CubemarsPanel` and click **OK**.
4. Click **Add Motor**, enter a motor ID, and the widget appears in the scrollable list.

To persist the panel across sessions, save the RViz config after adding motors (**File → Save Config**).

---

