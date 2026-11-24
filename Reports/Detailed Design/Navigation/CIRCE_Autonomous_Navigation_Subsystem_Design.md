Link to Detailed Block Diagram: https://lucid.app/lucidchart/a8185fd5-f90d-4644-bcab-a16ba6563003/edit?beaconFlowId=3A3960EEFA645D94&invitationId=inv_8c0ef14b-ae71-4fa5-bc44-34a20c63f79f&page=0_0#
## CIRCE Autonomous Navigation Subsystem Design

### Function of the Subsystem

The navigation subsystem for CIRCE serves as the brains and eyes of its autonomous movement. Its primary functions include determining the robot’s position and orientation in space, planning safe and efficient paths toward specified goals when pre-defined paths are unfeasible, and executing motion commands to reach those destinations while avoiding obstacles. This system integrates real-time localization, environment mapping, sensor fusion, and precise motor control, enabling CIRCE to autonomously explore and navigate a maximum distance of 100 meters from its start location in both GPS-denied and GPS-available environments.  

---

### Specifications and Constraints

- CirceBot shall carry up to 100 meters (approximately 10 lbs.) of Ethernet cable.  
	- ANSI/TIA-568.2-D: This standard specifies the maximum allowable length for twisted-pair Ethernet cables in standard networking applications. The maximum length is defined as 109 yards (328 feet or approximately 100 meters), beyond which signal attenuation, latency, and packet loss can degrade network performance. To extend connectivity beyond this limit, additional hardware such as repeaters, switches, or fiber optic solutions are required. For this project it is intended to stick with the initial 100 yards.
- CirceBot shall report error codes via self diagnosis.
	- The customer, DEVCOM, has specified that error codes shall be reported using a self diagnosis system. 
- CirceBot shall receive Next Position waypoints and navigate to the next waypoint. 
	- The customer, DEVCOM, has specified that navigation shall be conducted using a waypoint-to-waypoint system.  
- CirceBot shall transmit real-time data, including current position, current velocity, heading, and error codes if any occur. 
	- The customer, DEVCOM, has specified the required data to be transmitted and displayed for the current operation.  
- CirceBot shall transmit data at a minimum speed of 10 Hz.
	- The customer, DEVCOM, has specified the data should be transmitted at a speed no lower than 10 Hz.
- CirceBot shall use minor obstacle avoidance to avoid collisions. This will allow the robot to go around obstacles and avoid any collisions that could possibly damage the system.  
	- ANSI/RIA R15.08: This standard establishes safety requirements for autonomous mobile robots (AMRs), including obstacle detection and avoidance. It defines acceptable sensor technologies, response times, and stopping distances to ensure safe operation in dynamic environments.  

**Constraints include:**  

● CirceBot shall use approximately 70–85% of the Raspberry Pi 4’s CPU and 1.2–1.8 GB of RAM during operation, with the depth camera posing the highest processing demand (balanced via ROS optimization).  

● CirceBot shall synchronize all sensor data inputs (LiDAR, depth camera, wheel encoders, and inertial measurement unit (IMU)) with a maximum time skew of ±20 milliseconds. 

● CirceBot shall complete the full navigation processing loop within 100 milliseconds to maintain a consistent 10 Hz update rate.

● CirceBot shall maintain a positional accuracy within ±2% of the total distance traveled over time.

● CirceBot shall not accumulate drift exceeding 10 cm per meter traveled under nominal terrain and sensor conditions.

---

### Overview of Proposed Solution

The system uses the following integrated components:  

- Raspberry Pi 5: Executes Simultaneous Localization and Mapping (SLAM), positioning, minor path planning, and environmental mapping using Robot Operating System (ROS) and nav2. It collects sensor data from the LiDAR and RealSense camera to build a real-time occupancy map and fuses this with inertial and odometric feedback.  The bulk of the path planning shall be determined by CirceSoft at C2, however CIRCE should be capable of adjusting to the environment if CirceBot comes across an impassable obstacle making waypoint unachievable. 

- Teensy 4.0/4.1: Handles Battery Management Systems (BMS) monitoring, internal temperature monitoring, cable spool deployment velocity, cable stapling frequency, and ultrasonic sensor interfacing and signal processing.  The microcontroller will perform light signal processing and assign a confidence value to the ultrasonic sensor data

- SLAM and Path Planning: Navigation is guided using the A* algorithm. These methods will rely on the LiDAR and RealSense depth camera for accuracy and functionality. 
	- To ensure CirceBot is compliant with ANSI/RIA R15.08 standards, sensor placement and orientation is governed by the stopping distance of CirceBot on wet grass with no braking assistance from the drivetrain.  Assuming CirceBot is braking on flat ground (i.e. not attempting to brake while travelling downhill) from the manufacturer specified top speed of $5.6~mph$ ($2.5~m/s$), CirceBot is expected to come to a complete stop in $2.98~ft$ ($0.910~meters$). 

- Sensors
	- The LiDAR sensor will be mounted on the immediate top of the robot chassis to detect obstacles taller than 8 inches including but not limited to buildings, trees, and tall impassable shrubbery. As this sensor is not in the exact center of the robot, a ROS2 transform will be applied to account for the sensor's position and orientation relative to the origin of the base frame of CIRCEBot.  This, paired with the depth camera, will provide updates to global and local cost maps, should minor path corrections be necessary.   
	- Paired with the LiDAR sensor, the depth camera will be installed above the LiDAR camera. Positioning the depth camera above the LiDAR will allow for greater negative pitch angle on the depth camera with the intention of improving hole detection for CIRCEBot.  Using 70% of the tire diameter as a reference for a hole depth deemed impassable, we estimate a negative pitch angle of $20^\cdot$ placed above the LiDAR sensor will suffice in allowing for CIRCEBot to detect impassable holes while still being able to detect above ground obstacles. In a similar fashion to the LiDAR, due to the depth camera being offset from the origin of the base frame for CIRCEBot, a ROS2 transform will be applied to the depth camera data to account for the sensor's orientation and position.  
	- Ultrasonic will serve as a last line of defense to prevent the robot from colliding with an impassable obstacle.  To mitigate the risk of false positives, a moving average filter will be applied to the ultrasonic sensor data points on the Teensy. To integrate the ultrasonic sensors into the system from the Teensy to the RPi while meeting the 100ms time constraint, the sensors will run on a separated loop from the main processing loop and run at a slower clock speed. Within this slower loop, data from all three sensors and the IMU will be filtered and processed to determine whether the robot is in immediate danger to hitting an obstacle. Should the sensors indicate a high probability or hitting an obstacle a hardware semaphore will be raised, triggering a movement sequence within the faster loop to pause current drive commands, rotate the robot 90 in the opposite direction of the obstacle, travel a minimum of 1 length of the robot, and resume to the current waypoint.  Should the robot detect an obstacle directly in the center of its current path, the robot will be programmed to rotate in a default direction.  
	- Wheel Encoders with Hall Sensors: Each DC motor is equipped with a Hall-effect-based quadrature encoder. These generate two out-of-phase digital signals, interpreted by the Teensy to determine both speed and direction of each wheel. Combined with the IMU[^1] via ROS2 nodes and topics, they provide dead-reckoning odometry, crucial for short-term localization and motor control feedback.  

Below: Side view of the approximate vertical field of views (FOVs) of each sensor.
![[Detailed_Design_Sensor_FOV_Side_View.png|800]]
- Coordinate System Integration:  
	- Earth-Centered Earth-Fixed (ECEF): Maps real-world 3D positions relative to  
      the Earth’s surface. Useful when interfacing CIRCE with global positioning  
      systems.  
	- Local Cartesian Frame: CIRCE builds a local 2D/3D occupancy map in Cartesian space where all obstacle detection and  any small path planning modifications take place. This is  the robot's immediate operational frame.  
	- The aforementioned coordinate systems are converted via transformation matrices inside the Pi’s software stack (e.g., using ROS tf package). ECEF supports geographic referencing, and the Cartesian map offers immediate spatial decision-making.  
- Under GPS-denied conditions,  CIRCEBot will rely solely on the IMU, wheel encoders, and other onboard sensors for dead reckoning localization. When configured to operate in GPS-enabled conditions, CIRCE will include the GPS module using its associated ROS2 node and have the GPS feed into the EKF should the GPS receive inaccurate data from minimal satellite connections.  


---

### Interface with Other Subsystems

● Power System: Supplies 5V and 12V to Raspberry Pi, Teensy, sensors, and motors. A regulated buck converter ensures voltage consistency.  

● Motor Controls: UART over serial allows for the Teensy to send linear and angular velocity commands to the motor controller for precise movement. 

● Operating System: This subsystem is almost directly linked to the operating subsystem, since the operating system will control the linking between the Raspberry Pi and the Teensy. 

Integrating navigation, drivetrain, and OS subsystems together will primarily be done through ROS2 node architecture.  A simplified example of the navigation and drivetrain communication is shown below.  This abstract representation condenses mapping and path planning into the 'Waypoint Publisher' block and omits the emergency stop system entirely to focus on the ROS2 architecture.  For a detailed view of what is encompassed under 'Waypoint Publisher' block, see the [[#Flowchart]].  The navigation subsystem will operate under the pretense that the waypoints will be determined by CIRCESoft using an A* algorithm. Any onboard sensor will be used to both locate CIRCEBot and provide feedback should CIRCEBot need to make corrections for unforeseen obstacles.  

When operating in GPS-allowed scenarios, a GPS publisher node will be included and feed the `robot_localization` node, providing more accurate localization albeit at a slower rate compared to the odometer and IMU nodes. 
![[Navigation Drivetrain ROS2 Nodes.png]]

---

### Buildable Schematic

● Navigation Wiring Schematic:
![Navigation_Wiring_Diagram_V1](https://github.com/TnTech-ECE/S26_Team4_DEVCOM-CIRCE-BOT/blob/troubadour-tardigrade-patch-2/Reports/Detailed%20Design/Navigation/Navigation_Wiring_Diagram_V1.pdf)

---

### Flowcharts

![Navigation Flow Chart](https://github.com/TnTech-ECE/S25_Team1_MyCapstoneProject/blob/DD-Navigation/Detail%20Design/Navigation/CIRCE%20Navigation%20Flowchart%20(1).jpg)
Within the navigation system, a close range collision (CRC) software watchdog will be implemented to allow the ultrasonic sensors adequate sampling periods to produce reliable and confident data.  The planned flow chart is shown below. 
#### Close Range Collision module
The close range collision (CRC) module:
![[CRC Watchdog Loop.png]]

### Navigation and Drivetrain ROS2 integration
![[Navigation Drivetrain ROS2 Nodes.png]]

### Navigation and Operating System ROS2/Nav2 Integration
![[Navigation Nav2 Flowchart.png]]


---

### Bill of Materials

| Product                   | Manufacturer | Part Number    | Distributor | Distributor Part Number | Quantity | Price (USD) | Purchasing URL                                                                                           |
| ------------------------- | ------------ | -------------- | ----------- | ----------------------- | -------- | ----------- | -------------------------------------------------------------------------------------------------------- |
| A02YYUW Ultrasonic Sensor | DF Robot     | SEN0311        | DFRobot     | SEN0311                 | 3        | $51.42      | [[https://www.dfrobot.com/product-1935.html\|Link]]                                                      |
| PETG 3D Printer Filament  | OVERTURE     | OVPETG175      | Amazon      | OVPETG175               | 1        | $14.99      | [[<br>\|https://www.amazon.com/OVERTURE-Filament-Consumables-Dimensional-Accuracy/dp/B07PDV9RC8\|]]      |
| Threaded Insert Kit M2-M6 | Okxiri       | XMKJ1210-LMTQJ | Amazon      | XMKJ1210-LMTQJ          | 1        | $6.99       | https://www.amazon.com/Threaded-Inserts-Assortment-Printing-Components/dp/B0FVS6W7RF/ref=sr_1_12?sr=8-12 |
| Total                     |              |                |             |                         |          | $73.40      |                                                                                                          |

---

### Analysis

The CIRCE autonomous navigation subsystem has been thoughtfully designed to meet the constraints and fulfill the intended function of guiding a mobile robot up to 100 yards with full autonomy. It integrates a suite of complementary technologies, SLAM, real-time path planning, and obstacle avoidance using RPLIDAR A1 and the Intel RealSense D456. These enable CIRCE to operate within dynamic environments while avoiding collisions, adhering to standards. Real-time sensor fusion—combining LiDAR, depth imaging, Hall-effect wheel encoders, and IMU data—ensures robust localization and reliable obstacle detection even in GPS-denied or cluttered environments.

To achieve CIRCE’s goal of autonomous outdoor navigation with reliable obstacle avoidance and path planning over distances of up to 100 meters, a combination of LiDAR, stereo depth camera, and inertial sensing is employed. The RPLIDAR A1 offers 360° scanning with an angular resolution of approximately 1° and a range of up to 12 meters, providing sufficient accuracy (±2–3 cm at 1.5 m) for 2D SLAM and obstacle detection. This unit will also refresh at a rate of 10 Hz (10 full scans per second) at a proper optimization of the D456 and sensor fusion. The Intel RealSense D456 depth camera complements this with high-resolution 3D depth perception, delivering depth accuracy within ±2% at 2 meters and a maximum effective range of 5–10 meters. Together, these sensors enable CIRCE to detect and navigate around objects with a positional accuracy requirement of ±10 cm. However, IMUs inherently drift over time, particularly in yaw, and must be corrected through sensor fusion techniques. This drift is mitigated by fusing IMU data with wheel encoder feedback, LiDAR scans, and stereo vision via algorithms such as the SLAM frameworks, ensuring robust and consistent localization throughout autonomous operation. To mitigate vibration noise affecting CIRCE’s IMU, the IMU should be isolated from the robot's motors and other vibrating components using vibration-dampening materials such as rubber mounts or foam pads. Additionally, a Kalman filter should be implemented to fuse the IMU data with other sensor inputs, smoothing out high-frequency vibration noise and providing more accurate estimates of the robot’s orientation and position. To mitigate adverse effects of false positives from the ultrasonic sensors, median filters may be applied to the ultrasonic sensor data. Regular sensor calibration should also be performed to further reduce residual noise and improve accuracy.

Furthermore, CIRCE satisfies critical constraints such as limited power and processing capacity through modular hardware selection and software optimization within ROS. The navigation system draws anywhere from 8-10.5 W and transmits 10 Hz telemetry, including all specified metrics—position, velocity, cable length remaining, heading, and fault states—as required by the customer (DEVCOM). By delegating high-level planning to the Raspberry Pi 4 and real-time control to the Teensy 4.1, CIRCE achieves efficient task partitioning that maintains system responsiveness under computational limits.

Designed for modular upgrades and future scalability, the CIRCE subsystem can accommodate expanded capabilities such as GPS integration or advanced motion planning modules. The design reflects a balance of cutting-edge navigation techniques with practical embedded systems engineering. Its ability to maintain accuracy over time, respond to real-world uncertainties, and deliver transparent diagnostics confirms that CIRCE is a reliable and field-ready autonomous solution.  

---

### References
[1] "CIRCE_Autonomous_Navigation_Subsystem_Design", Spring 2025 DEVCOM CirceBot Team, https://github.com/TnTech-ECE/S25_Team1_MyCapstoneProject/blob/main/Detail%20Design/Navigation/CIRCE_Autonomous_Navigation_Subsystem_Design.md (accessed April 23, 2026)
[2] "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems", 2nd Ed., P. Groves, accessed April, 2026
[3] "Kalman Filters", M. Budge Jr., accessed April, 2026

[^1]: IMU: Inertial Measurement Unit
