## CIRCE Autonomous Navigation Subsystem Design

### Function of the Subsystem

The navigation subsystem for CIRCE serves as the brain of its autonomous movement. Its primary functions include determining the robot’s position and orientation in space, planning safe and efficient paths toward specified goals when pre defined paths are impossible to fulfill, and execute motion commands to reach those destinations while avoiding obstacles. This system integrates real-time localization, environment mapping, sensor fusion, and precise motor control, enabling CIRCE to autonomously explore and navigate a maximum distance of up to 100 meters from its start location in both GPS denied and GPS allowed environments.  

---

### Specifications and Constraints

- CirceBot shall carry up to 100 meters (approximately 10 lbs.) of Ethernet cable.  
	- ANSI/TIA-568.2-D: This standard specifies the maximum allowable length for twisted-pair Ethernet cables in standard networking applications. The maximum length is defined as 109 yards (328 feet or approximately 100 meters), beyond which signal attenuation, latency, and packet loss can degrade network performance. To extend connectivity beyond this limit, additional hardware such as repeaters, switches, or fiber optic solutions are required. For this project it is intended to stick with the initial 100 yards.
- CirceBot shall report error codes via self diagnosis.
	- The customer, DEVCOM, has specified that error codes shall be reported using a self diagnosis system. 
	- ISO 
- CirceBot shall receive Next Position waypoints and navigate to the next waypoint. 
	- The customer, DEVCOM, has specified that navigation shall be conducted using a waypoint-to-waypoint system.  
- CirceBot shall transmit real-time data, including current position, current velocity, heading, and error codes if any occur.  
	- The customer, DEVCOM, has specified the required data to be transmitted and displayed for the current operation.  
- CirceBot shall transmit data at a minimum speed of 10 Hz.
	- The customer, DEVCOM, has specified the data should be transmitted at a speed no lower than 10 Hz.
- CirceBot shall use minor obstacle avoidance to avoid collisions. This will allow the robot to go around obstacles and avoid any collisions that could possibly damage the system.  
	- ANSI/RIA R15.08: This standard establishes safety requirements for autonomous mobile robots (AMRs), including obstacle detection and avoidance. It defines  
      acceptable sensor technologies, response times, and stopping distances to ensure safe operation in dynamic environments.  

**Constraints include:**  

● CirceBot shall use approximately 70–85% of the Raspberry Pi 4’s CPU and 1.2–1.8 GB of RAM during operation, with the depth camera posing the highest processing demand (balanced via ROS optimization).  

● CirceBot shall synchronize all sensor data inputs (LiDAR, depth camera, wheel encoders, and inertial measurement unit (IMU)) with a maximum time skew of ±20 milliseconds. 

● CirceBot shall complete the full navigation processing loop within 100 milliseconds to maintain a consistent 10 Hz update rate.

● CirceBot shall maintain a positional accuracy within ±2% of the total distance traveled over time.

● CirceBot shall not accumulate drift exceeding 10 cm per meter traveled under nominal terrain and sensor conditions.

---

### Overview of Proposed Solution

The system uses the following integrated components:  

● Raspberry Pi 5: Executes Simultaneous Localization and Mapping (SLAM), positioning, minor path planning, and environmental mapping using Robot Operating System (ROS). It collects sensor data from the LiDAR and RealSense camera to build a real-time occupancy map and fuses this with inertial and odometric feedback.  The bulk of the path planning shall be determined by CirceSoft at C2, however CIRCE should be capable of adjusting to the environment if CirceBot comes across an impassable obstacle making waypoint unachievable. 

● Teensy 4.0/4.1: Handles Battery Management Systems (BMS) monitoring, internal temperature monitoring, cable spool deployment velocity, cable stapling frequency, and ultrasonic sensor interfacing and signal processing.  The microcontroller will perform light signal processing and assign a confidence value to the ultrasonic sensor data

● SLAM and Path Planning: Navigation is guided using A* algorithm. These methods will rely on the LiDAR and RealSeanse depth camera for accuracy and functionality. The ultrasonic sensors will serve as a last line of defense to prevent the robot from colliding with an impassable obstacle.  To mitigate the risk of false positive, a median-filter will be applied to the ultrasonic sensor data points on the Teensy before being sent to the RPi for further filtering with the LiDAR and depth camera sensors.  

- Coordinate System Integration:  
	- Earth-Centered Earth-Fixed (ECEF): Maps real-world 3D positions relative to  
      the Earth’s surface. Useful when interfacing CIRCE with global positioning  
      systems.  
	- Local Cartesian Frame: CIRCE builds a local 2D/3D occupancy map in Cartesian space where all obstacle detection and path planning take place. This is  the robot's immediate operational frame.  

These coordinate systems are converted via transformation matrices inside the Pi’s software stack (e.g., using ROS tf package). ECEF supports geographic referencing, and the Cartesian map offers immediate spatial decision-making.  

● Wheel Encoders with Hall Sensors: Each DC motor is equipped with a Hall-effect-based quadrature encoder. These generate two out-of-phase digital signals, interpreted by the Teensy to determine both speed and direction of each wheel. Combined with the IMU, they provide dead-reckoning odometry, crucial for short-term localization and motor control feedback.  

---

### Interface with Other Subsystems

● Power System: Supplies 5V and 12V to Raspberry Pi, Teensy, sensors, and motors. A regulated buck converter ensures voltage consistency.  

● Motor Controls: UART over serial allows for the Teensy to send linear and angular velocity commands to the motor controller for precise movement. 

● Operating System: This subsystem is almost directly linked to the operating subsystem, since the operating system will control the linking between the Raspberry Pi and the Teensy. 

---

### Buildable Schematic

● Navigation Wiring Schematic:


---

### Flowchart

![Navigation Flow Chart](https://github.com/TnTech-ECE/S25_Team1_MyCapstoneProject/blob/DD-Navigation/Detail%20Design/Navigation/CIRCE%20Navigation%20Flowchart%20(1).jpg)

---

### Bill of Materials

| Product                   | Manufacturer | Part Number | Distributor | Distributor Part Number | Quantity | Price (USD) | Purchasing URL                                      |
| ------------------------- | ------------ | ----------- | ----------- | ----------------------- | -------- | ----------- | --------------------------------------------------- |
| A02YYUW Ultrasonic Sensor | DF Robot     | SEN0311     | DFRobot     | SEN0311                 | 3        | $47.70      | [[https://www.dfrobot.com/product-1935.html\|Link]] |
| Total                     |              |             |             |                         |          | $47.70      |                                                     |

---

### Analysis

The CIRCE autonomous navigation subsystem has been thoughtfully designed to meet the constraints and fulfill the intended function of guiding a mobile robot up to 100 yards with full autonomy. It integrates a suite of complementary technologies, SLAM, real-time path planning, and obstacle avoidance using RPLIDAR A1 and the Intel RealSense D456. These enable CIRCE to operate within dynamic environments while avoiding collisions, adhering to standards. Real-time sensor fusion—combining LiDAR, depth imaging, Hall-effect wheel encoders, and IMU data—ensures robust localization and reliable obstacle detection even in GPS-denied or cluttered environments.

To achieve CIRCE’s goal of autonomous outdoor navigation with reliable obstacle avoidance and path planning over distances of up to 100 meters, a combination of LiDAR, stereo depth camera, and inertial sensing is employed. The RPLIDAR A1 offers 360° scanning with an angular resolution of approximately 1° and a range of up to 12 meters, providing sufficient accuracy (±2–3 cm at 1.5 m) for 2D SLAM and obstacle detection. This unit will also refresh at a rate of 10 Hz (10 full scans per second) at a proper optimization of the D456 and sensor fusion. The Intel RealSense D456 depth camera complements this with high-resolution 3D depth perception, delivering depth accuracy within ±2% at 2 meters and a maximum effective range of 5–10 meters. Together, these sensors enable CIRCE to detect and navigate around objects with a positional accuracy requirement of ±10 cm. However, IMUs inherently drift over time, particularly in yaw, and must be corrected through sensor fusion techniques. This drift is mitigated by fusing IMU data with wheel encoder feedback, LiDAR scans, and stereo vision via algorithms such as the SLAM frameworks, ensuring robust and consistent localization throughout autonomous operation. To mitigate vibration noise affecting CIRCE’s IMU, the IMU should be isolated from the robot's motors and other vibrating components using vibration-dampening materials such as rubber mounts or foam pads. Additionally, a Kalman filter should be implemented to fuse the IMU data with other sensor inputs, smoothing out high-frequency vibration noise and providing more accurate estimates of the robot’s orientation and position. To mitigate adverse effects of false positives from the ultrasonic sensors, median filters may be applied to the ultrasonic sensor data prior to being fused in the Kalman filter with the rest of the sensors. Regular sensor calibration should also be performed to further reduce residual noise and improve accuracy.

Furthermore, CIRCE satisfies critical constraints such as limited power and processing capacity through modular hardware selection and software optimization within ROS. The navigation system draws anywhere from 8-10.5 W and transmits 10 Hz telemetry, including all specified metrics—position, velocity, cable length remaining, heading, and fault states—as required by the customer (DEVCOM). By delegating high-level planning to the Raspberry Pi 4 and real-time control to the Teensy 4.1, CIRCE achieves efficient task partitioning that maintains system responsiveness under computational limits.

Designed for modular upgrades and future scalability, the CIRCE subsystem can accommodate expanded capabilities such as GPS integration or advanced motion planning modules. The design reflects a balance of cutting-edge navigation techniques with practical embedded systems engineering. Its ability to maintain accuracy over time, respond to real-world uncertainties, and deliver transparent diagnostics confirms that CIRCE is a reliable and field-ready autonomous solution.  

---

### References
[1] "CIRCE_Autonomous_Navigation_Subsystem_Design", Spring 2025 DEVCOM CirceBot Team, https://github.com/TnTech-ECE/S25_Team1_MyCapstoneProject/blob/main/Detail%20Design/Navigation/CIRCE_Autonomous_Navigation_Subsystem_Design.md (accessed April 23, 2026)
[2] "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems", 2nd Ed., P. Groves, accessed April, 2026
[3] "Kalman Filters", M. Budge Jr., accessed April, 2026

