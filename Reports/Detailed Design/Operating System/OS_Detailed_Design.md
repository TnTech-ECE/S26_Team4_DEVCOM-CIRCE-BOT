# CIRCE Operating System Subsystem: Detailed Design

## Function of the Subsystem

The Operating System (OS) subsystem serves as the central coordination and integration layer for all major functional blocks of the CIRCE system. It is responsible for managing system-wide communication, synchronizing data exchange between the Raspberry Pi 5 and Teensy 4.1, initiating startup procedures in the correct sequence, and overseeing subsystem interdependencies required for autonomous navigation and cable dispensing. The OS also manages a custom Python launch sequence that initializes nodes in the corresponding order. CIRCE's OS architecture is divided into two layers:

1. **High-Level Processing on RPi 5** : Responsible for operating the ROS 2 Jazzy middleware, including node lifecycle management, sensor fusion, generating an environmental map, and navigation.

2. **Low-Level Processing on Teensy 4.1** : Responsible for operating bare-metal firmware in C++ to control the stepper motor of the cable dispenser, reading the Battery Management System (BMS), and reading ultrasonic sensors.

---

## Specifications

- The OS subsystem shall initiate all CIRCE's subsystems during boot, including navigation, motor control, sensor fusion, and serial communications.
- The OS shall manage bidirectional serial communication between the Raspberry Pi 5 and Teensy 4.1, receiving battery state, cable length remaining, and ultrasonic sensor readings.
- The OS shall relay telemetry data via a WebSocket at a minimum of 10 Hz, including battery state, cable length remaining, velocity, and position.
- The OS shall support modular task scheduling through ROS 2 launch configurations, enabling future node additions or subsystem upgrades without requiring a complete system rewrite.
- The Teensy 4.1 shall control the stepper motor of the cable dispensing mechanism via PWM output.
- The Teensy 4.1 shall poll the BMS over I2C and report battery state of charge and health back to the Raspberry Pi 5.
- The Teensy 4.1 shall read ultrasonic sensor data and transmit readings to the Raspberry Pi 5 at a minimum of 10 Hz.
- The Teensy 4.1 shall execute all tasks deterministically within a single cooperative bare-metal control loop.

---

## Constraints

**Raspberry Pi 5:**

- The Pi's SoC will throttle CPU if the processor exceeds 85°C. Running multiple ROS 2 nodes under sustained load in an enclosed chassis can push the processor toward this threshold.
- The Pi 5 requires a steady 5V/5A supply to reliably process ROS 2 nodes while simultaneously handling USB data from LiDAR, Depth Camera, and Teensy 4.1.
- ROS 2 nodes must launch in a corresponding sequence for stable operation (ROS 2 Core → Drivers → Serial Communication → Autonomy Stack).
- The Pi 5 boots and logs from a microSD card, which has limited write cycles and is vulnerable to unexpected power loss that can cause data loss.
- The LiDAR, Depth Camera, Teensy 4.1, and Dual FSESC Controller all share the Pi's USB bus. The Pi 5 must reliably handle its USB buffer to prevent any loss of data.

**Teensy 4.1:**

- The Teensy 4.1 operates at 3.3V logic on its GPIO pins. Any 5V sensors or peripherals require logic level shifting or are connected through interfaces that handle the conversion natively.
- The bare-metal firmware must service all tasks, stepper PWM, I2C polling, ultrasonic reads, and USB serial, within a single control loop iteration without blocking on any one task long enough to violate the 10 Hz telemetry requirement.
- USB serial communication is dependent on the Raspberry Pi 5 maintaining an active connection. The Teensy firmware must handle the case where the Pi has not yet initialized or has temporarily dropped the connection without halting execution.
- The I2C bus speed must be compatible with the selected BMS module's communication requirements.

---

## Overview of Proposed Solution

This system includes the following devices:

1. **Raspberry Pi 5 (8GB model)** : Single Board Computer running ROS 2 Jazzy and coordinating the system.
2. **Teensy 4.1** : Low-level real-time controller for cable dispensing, battery monitoring, and ultrasonic readings.
3. **Intel RealSense D456** : Depth camera with integrated IMU (Gyro/Accel).
4. **RPLIDAR A1** : 2D LiDAR sensor for obstacle detection and mapping.
5. **Dual FSESC Motor Controller** : Controlled directly from the Pi 5 using the `roverrobotics_driver` package.
6. **Powered USB Hub** : Supplies stable power and data connectivity to USB peripherals, linked to the Pi 5.

The Teensy 4.1 runs bare-metal C++ firmware compiled using the Arduino framework for Teensy (Teensyduino). It executes a cooperative control loop that cycles through each of its hardware responsibilities in sequence: stepping the cable spool motor, polling the BMS via I2C, reading the ultrasonic sensors, and pushing a telemetry packet to the Raspberry Pi 5 over USB serial. The Pi sends cable dispense commands to the Teensy, and the Teensy responds with telemetry including battery state of charge, cable remaining, and ultrasonic readings. These values are then published by the Pi onto the appropriate ROS 2 topics for consumption by the navigation and communication nodes.

---

## Interface with Other Subsystems

| Subsystem | Interface Type | Devices | Communication to Pi | Data Exchanged |
|---|---|---|---|---|
| Navigation | ROS 2 Topics | RPLIDAR A1, RealSense Camera w/ IMU | RPLIDAR → Pi 5; Camera → Pi 5; Both via USB | LiDAR publishes 2D scan data on `/scan`. Camera publishes depth point clouds on `/camera/depth`, IMU data on `/imu`, and RGB streams on `/camera/image_raw`. `nav2`, `slam_toolbox`, and `robot_localization` consume these topics for path planning, obstacle detection, and localization. |
| Motor Control | ROS 2 Topics | Dual FSESC + Hall Effect Encoders | Pi 5 → Motor Controller via USB | The `roverrobotics_driver` node subscribes to `/cmd_vel` from nav2 and translates velocity commands into motor output. Encoder feedback is published on `/odom` for use by `robot_localization` and the navigation stack. |
| Cable Dispenser | USB + PWM | Teensy 4.1 + Stepper Motor | Teensy 4.1 → Pi 5 via USB; Teensy 4.1 → Motor via PWM pins | The Pi sends cable dispense start/stop commands to the Teensy. The Teensy controls the stepper motor and reports cable length remaining to the Pi. Pi publishes this on `/cable_remaining`. |
| Power Management | USB + I2C | Teensy 4.1 + BMS | BMS → Teensy 4.1 via I2C | The Teensy polls the BMS for battery state of charge and transmits it to the Pi. The Pi publishes this on `/battery_state`. |
| Short-Range Obstacle Detection | GPIO + USB Serial | Ultrasonic Sensors + Teensy 4.1 | Sensors → Teensy 4.1 via GPIO; Teensy 4.1 → Pi 5 via USB | Teensy reads ultrasonic echo return times and transmits distance readings to the Pi, which publishes them for consumption by the navigation stack. |

---

## 3D Model of Physical Components

**Raspberry Pi 5**

![Raspberry Pi 5](Raspberry%20Pi%205.png)

**Teensy 4.1**

![Teensy 4.1](Teensy%204.1.png)

---

## Signal Flow Chart

![ROS2 Flow Chart](ROS2%20Flow%20Chart.png)

---

## Bill of Materials (BOM)

| Item | Quantity | Part Number | Description | Store Link | Unit Price ($) |
|---|---|---|---|---|---|
| Raspberry Pi 5 | 1 | SC1432 | Single Board Computer, 8 GB RAM | [View on DigiKey](https://www.digikey.com/en/products/detail/raspberry-pi/SC1432/21658257) | 172.72 (Currently Have) |
| Teensy 4.1 | 1 | N/A | Microcontroller Unit, ARM Cortex-M7 @ 600 MHz | [View on SparkFun](https://www.sparkfun.com/products/16771) | 31.50 |
| USB Hub | 1 | USBG-4U2ML | USB Hub 4-Port 480 Mbps | [View on DigiKey](https://www.digikey.com/en/products/detail/coolgear/USBG-4U2ML/12318669) | 111.14 (Currently Have) |

---

## Analysis of Design Choices

### ROS Middleware

The Raspberry Pi 5 runs CIRCE's entire autonomy stack through the ROS 2 Jazzy middleware framework on Ubuntu 24.04 LTS. ROS 2 operates on a decentralized publish-subscribe architecture powered by DDS, where each functional component runs as an independent node that communicates by publishing and subscribing to named topics. There is no central master process. Nodes discover each other through DDS peer-to-peer discovery over the Pi's loopback interface and shared memory transport. This architecture enables CIRCE to operate autonomously in GPS-denied and RF-denied environments.

CIRCE's node graph consists of six to eight nodes, each responsible for a distinct function. This includes sensor driver nodes (`rplidar_ros` for RPLIDAR A1, `realsense2_camera` for the Intel RealSense D456), the `roverrobotics_driver` for FSESC motor control, a custom serial bridge node for Teensy 4.1 communication, and the full navigation stack (`slam_toolbox`, `robot_localization`, and `nav2`).

These nodes execute concurrently as separate processes scheduled by the Linux kernel. ROS 2 Jazzy supports both single-threaded and multi-threaded executors, allowing nodes with high callback frequency, such as the sensor drivers publishing at a rate of at least 10 Hz. The Pi 5's quad-core ARM Cortex-A76 processor at 2.4 GHz distributes this workload across four cores, and the DDS shared memory transport minimizes inter-node communication latency.

### Custom Python Launch Sequence

CIRCE's startup is managed by a custom Python launch file built using the ROS 2 `launch` library. Rather than launching all nodes simultaneously, the script initializes subsystems in a defined sequence. Hardware drivers first, then serial communication with the Teensy 4.1, then the full autonomy stack, to prevent nodes from attempting to subscribe to topics that have not yet been published. The pseudocode below outlines the intended launch logic:

```python
# circe_launch.py : CIRCE ROS 2 Startup Sequence (Pseudocode)
# Uses ROS 2 launch library: from launch import LaunchDescription
# and launch_ros.actions import Node

def generate_launch_description():

    # -------------------------------------------------------
    # STAGE 1: Hardware Drivers
    # Launch sensor and motor driver nodes first so that
    # all hardware interfaces are active before any
    # autonomy nodes attempt to subscribe to their topics.
    # -------------------------------------------------------

    rplidar_node        # rplidar_ros : publishes /scan
    realsense_node      # realsense2_camera : publishes /camera/depth, /imu, /camera/image_raw
    roverrobotics_node  # roverrobotics_driver : subscribes to /cmd_vel, publishes /odom

    # -------------------------------------------------------
    # STAGE 2: Serial Communication (Teensy 4.1 Bridge)
    # Wait for hardware drivers to be ready, then open the
    # Teensy's virtual COM port and begin exchanging telemetry.
    # Bridge publishes /battery_state, /cable_remaining,
    # and ultrasonic distance data received from the Teensy.
    # Pi sends cable dispense commands back to Teensy.
    # -------------------------------------------------------

    wait_for_drivers_ready()    # confirm /scan and /odom are being published

    teensy_serial_bridge_node   # custom node : opens /dev/ttyACM0 (Teensy USB CDC)
                                # publishes: /battery_state, /cable_remaining, /ultrasonic
                                # subscribes: /cable_dispense_cmd

    # -------------------------------------------------------
    # STAGE 3: Autonomy Stack
    # Launch SLAM, localization, and navigation only after
    # sensor data and serial comms are confirmed active.
    # -------------------------------------------------------

    wait_for_serial_ready()     # confirm /battery_state is being published

    slam_toolbox_node           # builds occupancy map from /scan, publishes /map
    robot_localization_node     # fuses /odom + /imu → publishes /odometry/filtered, /tf
    nav2_node                   # path planning : subscribes /map, /odometry/filtered
                                # publishes /cmd_vel to roverrobotics_driver

    # -------------------------------------------------------
    # STAGE 4: Communication Interface
    # Launch WebSocket bridge last: all telemetry topics
    # must be live before CirceSoft can connect.
    # -------------------------------------------------------

    websocket_bridge_node       # serializes telemetry into CirceBot2CirceSoft.proto
                                # relays CirceSoft2CirceBot.proto waypoints to nav2
                                # minimum publish rate: 10 Hz

    return LaunchDescription([
        rplidar_node,
        realsense_node,
        roverrobotics_node,
        teensy_serial_bridge_node,
        slam_toolbox_node,
        robot_localization_node,
        nav2_node,
        websocket_bridge_node,
    ])
```

---

### Teensy 4.1: Low-Level Controller

#### Bare-Metal Firmware Over RTOS

The CIRCE conceptual design originally designated Zephyr RTOS for the Teensy 4.1 to provide deterministic preemptive scheduling across concurrent tasks. During detailed design, the scope of the Teensy's responsibilities was narrowed significantly from the original conception. The Teensy is no longer responsible for drivetrain motor control, which is handled directly by the Rover Zero 3 platform's existing FSESC motor controller interfaced through the Raspberry Pi 5 via the `roverrobotics_driver` ROS 2 package. With drivetrain control removed, the Teensy's remaining workload consists of four well-bounded tasks: stepper PWM generation, I2C BMS polling, ultrasonic sensor reads, and USB serial communication.

At 600 MHz, the Teensy 4.1's ARM Cortex-M7 processor can complete each of these operations in microseconds. Stepper pulse generation for a standard stepper driver requires toggling a GPIO pin at a fixed frequency, which is handled by a hardware timer peripheral and does not consume meaningful CPU cycles. I2C transactions to the BMS are initiated at a low polling rate, as battery state does not change fast enough to require high-frequency reads. Ultrasonic sensor reads involve triggering a pulse and measuring the echo return time, a process that takes on the order of single-digit milliseconds for nearby obstacles. USB serial transmission of a compact telemetry struct at 10 Hz is well within the bandwidth of USB Full Speed.

The combined execution time of one full loop iteration is well under the 100 ms budget imposed by the 10 Hz telemetry floor. A preemptive RTOS scheduler would add memory overhead, a device tree configuration burden, and a steeper bring-up cost with no meaningful benefit given that none of the Teensy's tasks need to preempt one another. The bare-metal approach results in a simpler, more auditable firmware codebase that is easier to debug during integration and testing.

#### USB Serial Communication to the Raspberry Pi 5

The Teensy 4.1 connects to the Raspberry Pi 5 over USB, appearing as a CDC serial device. This interface was selected for several reasons. First, it requires no additional wiring beyond a single USB cable, which simplifies physical integration inside the chassis. Second, the Raspberry Pi 5's USB host controller provides stable power to the Teensy, eliminating the need for a separate regulated supply for the microcontroller. Third, USB serial is natively supported by both the Teensyduino framework and the Linux kernel on the Pi, requiring no custom driver development. The ROS 2 serial bridge node on the Pi opens the Teensy's virtual COM port and handles the framing and parsing of telemetry packets, publishing the received values onto the `/battery_state` and `/cable_remaining` topics consumed by the rest of the autonomy stack.

#### I2C for BMS Communication

The BMS module is polled by the Teensy over I2C rather than through a direct analog voltage measurement or a dedicated SPI interface. I2C was selected because it requires only two signal lines shared across the bus, preserving GPIO availability on the Teensy for stepper control and ultrasonic sensor trigger and echo pins. Standard BMS modules designed for lithium-ion polymer battery packs commonly expose an I2C interface with registers for state of charge, cell voltages, temperature, and fault flags. Polling over I2C at a low rate is entirely sufficient given that battery state of charge changes slowly relative to the robot's operational timescales.

#### Stepper Motor Control for Cable Dispensing

The cable spool mechanism designed by the ME team uses a stepper motor for controlled cable dispensing. The Teensy generates step and direction pulses to the stepper motor driver via GPIO. Using a stepper rather than a brushed DC motor provides open-loop position control without requiring encoder feedback, which simplifies the ME team's mechanical design and the firmware required to manage it. The Teensy controls dispense rate by adjusting the step pulse frequency in response to commands received from the Raspberry Pi 5, which can modulate cable payout speed based on the robot's current velocity reported by the drivetrain odometry.

---

## References

[1] FS, "ANSI/TIA-568.2-D," *FS.com Glossary*, [Online]. Available: https://www.fs.com/glossary/ansitia5682d-g73.html. [Accessed: Apr. 22, 2026].

[2] IEEE, "IEEE 1872.2-2021 — Standard for Autonomous Robotics (AuR) Ontology," *IEEE Standards Association*, [Online]. Available: https://standards.ieee.org/ieee/1872.2/7094/. [Accessed: Apr. 22, 2026].

[3] IEEE, "IEEE 1012-2016 — Standard for System, Software, and Hardware Verification and Validation," *The ANSI Blog*, [Online]. Available: https://blog.ansi.org/ansi/ieee-1012-2016-verification-validation-vv/. [Accessed: Apr. 22, 2026].

[4] ATIS Protection Engineers Group, "National Electric Safety Code (NESC) Update," [Online]. Available: https://peg.atis.org/wp-content/uploads/2019/03/NESC_UpdateTBowmer.pdf. [Accessed: Apr. 22, 2026].

[5] Open Robotics, "REP-2000: ROS 2 Releases and Target Platforms," *Robotics Enhancement Proposals*, [Online]. Available: https://reps.openrobotics.org/rep-2000/. [Accessed: Apr. 22, 2026].

[6] XDA Developers, "Raspberry Pi 5 vs. Jetson Nano: General Purpose or AI-Focused," *XDA Developers*, [Online]. Available: https://www.xda-developers.com/raspberry-pi-5-vs-jetson-nano/. [Accessed: Apr. 22, 2026].

[7] ThinkRobotics, "Jetson Nano vs. Raspberry Pi 5 for AI: The Ultimate Performance and Value Comparison," *ThinkRobotics*, [Online]. Available: https://thinkrobotics.com/blogs/learn/jetson-nano-vs-raspberry-pi-5-for-ai-the-ultimate-performance-and-value-comparison. [Accessed: Apr. 22, 2026].

[8] ROS, "ROS 2 Releases and Target Platforms," *ROS.org*, [Online]. Available: https://www.ros.org/reps/rep-2000.html. [Accessed: Apr. 22, 2026].

[9] PJRC, "Teensy 4.1," *PJRC.com*, [Online]. Available: https://www.pjrc.com/store/teensy41.html. [Accessed: Apr. 22, 2026].

[10] PJRC, "Teensyduino: Using the Teensy with the Arduino IDE," *PJRC.com*, [Online]. Available: https://www.pjrc.com/teensy/teensyduino.html. [Accessed: Apr. 22, 2026].

[11] ARM, "Cortex-M7 Processor," *ARM Developer*, [Online]. Available: https://developer.arm.com/Processors/Cortex-M7. [Accessed: Apr. 22, 2026].
