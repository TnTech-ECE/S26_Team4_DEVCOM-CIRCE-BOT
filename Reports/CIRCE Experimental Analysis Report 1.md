# Navigation system analysis

## Experiment 1: Navigation Accuracy

                Navigation accuracy was tested to quantify CIRCEBot’s ability to follow given waypoints. At the time of testing, CIRCEBot was undergoing final assembly and troubleshooting. While CIRCEBot was unavailable for long distance real world full system testing, simulation testing was done in the Gazebo physics simulator which interfaced with our ROS2 architecture.  This setup therefore introduces more uncertainty as there is no real-world twin to validate the simulation.

### Procedure:

                 Despite testing in simulation after extensive real-world testing became unrealistic, the following procedures were followed to gather data to determine the probable error of CIRCEBot’s navigation system. 

1.      Start Gazebo simulation window and RVIZ window.

a.      Only for simulation: Ensure the Gazebo world has a grid of white squares over the entire face with larger red squares. This grid will be used to determine CIRCEBot’s position when away from the origin.

2.      Record CIRCEBot’s starting location on an (x,y) cartesian plane.

3.      Send CIRCEBot a command via the terminal to travel some distance in one axis and record that distance sent.

4.      Wait for CIRCEBot to stop moving and to return “Goal Status: SUCCEEDED” in the command pose terminal. 

5.      Using the grid on the map, determine CIRCEBot’s new position on the (x,y) cartesian coordinate frame. Record this new position.

6.      Repeat steps 2-5 ensuring CIRCEBot travels in the same axis for 5 runs.

7.      Repeat steps 2-5 again with CIRCEBot travelling in the axis perpendicular to the previous axis. 

8.      Repeat steps 2-5 again with commanded paths requiring travel in both axes.

### Data:

After running 4 to 5 trials of strictly X axis, strictly Y axis and XY travel, we have the following values for error between recorded final position and commanded position.


| Strictly X axis travel (“Down range”) error        | 2.8 ± 2.4 m                                                    |
| -------------------------------------------------- | -------------------------------------------------------------- |
| Strictly Y axis travel (“Cross range”) error       | 12 ± 8.9 m                                                     |
| X and Y axis travel (“Down and Cross range”) error | Down Range: 5.125 ± 0.78 m<br><br>Cross Range: -2.375 ± 0.31 m |

### Analysis:

                Initial testing indicates significant errors in strictly Y axis travel. Results of this test conclude an elliptical error probable of 0.57 meters.   One observation made during testing is that the robot appears to believe it is turning more than it is in the Gazebo simulations.  Reviewing CIRCEBot’s IMU and odometry data on angular motion about the z-axis during cross range testing indicates full 90-degree rotation towards parallel with y-axis while Gazebo simulation shows the robot rotating less than 90 degrees.  This however contradicts small scale real world around the Capstone lab where CIRCEBot successfully completed multiple differential drive turn maneuvers to rotate 90 degrees. Such a discrepancy may lead towards the conclusion of the physics simulator missing key factors. 

                Moving forward from initial testing, more tuning of the MPPI controller critics is necessary on top of completing and clearing CIRCEBot for real world extensive testing.  Using simulation data as a starting point for tuning, the Goal critic and PathFollow critics will be the first to be tuned.  When tuning values, the starting critic value, and new critic values will be recorded before running the steps outlined in the procedures section of the experiment again.  By collecting the old and new critic values, we can gain better understanding of how critic values affect error and tune with more insight.

## Inventory
| Item                              | Last Known Location       | Condition                                       |
| --------------------------------- | ------------------------- | ----------------------------------------------- |
| Intel Realsense D456 Depth Camera | On CIRCEBot; Capstone Lab | LIke new                                        |
| RPLiDAR A1                        | On CIRCEBot; Capstone Lab | LIke new                                        |
| Threaded Insert Assortment        | Capstone Lab              | Missing Inserts (Some inserts used in CIRCEBot) |
| White 1.75mm PETG Filament        | Capstone Lab              | <200g remaining                                 |
| M2.5-M5 Screw Assortment          | Capstone Lab              | Missing screws (Screws used in CIRCEBot)        |
| Ultrasonic Sensors                | On CIRCEBot; Capstone Lab | JST connectors cut off                          |
| GPS Module                        | On CIRCEBot; Capstone Lab | JST connectors cut off                          |

# **OS Subsystem Experimental Analysis**

**Purpose and Justification:**

The purpose of the operating system subsystem is to implement a middleware called “Robot Operating System (ROS)” that enables CIRCE’s capabilities to utilize sensor fusion which feeds to a network where it can be used to compute its behavior.  

**Setup:**

1.       Install:          

a.        Install ROS2 Jazzy and Python:
- ros-jazzy-rmw-cyclonedds-cpp \
- ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
- ros-jazzy-slam-toolbox ros-jazzy-robot-localization \
- ros-jazzy-realsense2-camera ros-jazzy-realsense2-description \
- ros-jazzy-laser-filters \
- ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher \
- ros-jazzy-diagnostic-updater ros-jazzy-diagnostic-aggregator \
- ros-jazzy-rosbag2-storage-mcap ros-jazzy-tf2-tools \
- ros-jazzy-joy ros-jazzy-teleop-twist-joy ros-jazzy-teleop-twist-keyboard \
- ros-jazzy-rviz-imu-plugin ros-jazzy-rqt-robot-monitor ros-jazzy-rqt-graph \
- ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
- sudo apt install -y python3-numpy python3-scipy python3-serial python3-psutil \
- python3-websockets python3-protobuf protobuf-compiler libprotobuf-dev setserial python3-pip python3-venv

b.       Clone the repo:

git clone --recurse-submodules [https://github.com/TnTech-ECE/S26_Team4_DEVCOM-CIRCE-BOT.git](https://github.com/TnTech-ECE/S26_Team4_DEVCOM-CIRCE-BOT.git) ~/S26_Team4_DEVCOM-CIRCE-BOT

ln -s ~/S26_Team4_DEVCOM-CIRCE-BOT/Software/circe_ws ~/circe_ws

c.        Teensy 4.0 firmware (PlatformIO):

curl -fsSL -o /tmp/get-platformio.py [https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py](https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py)

python3 /tmp/get-platformio.py

cd ~/circe_ws/src/circe_teensy && ~/.platformio/penv/bin/pio run -t upload

d.       Build and add ROS environment:

                                                                           i.      Build the workspace:

colcon build --symlink-install

                                                                         ii.      Open ~/.bashrc in the terminal:

nano ~/.bashrc

                                                                       iii.      Add this at the end of the file and save:

source /opt/ros/jazzy/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

export CYCLONEDDS_URI=file:///etc/cyclonedds/rover.xml

export ROS_DOMAIN_ID=42

source ~/circe_ws/install/setup.bash

2.       Setup laptop:

a.        ~/circe_ws/laptop/setup.sh

3.       Connect to Pi5 via SSH:

a.        ssh [devcom@10.104.132.133](mailto:devcom@10.104.132.133)

b.       Password: circe2026

4.       Launch circe_bringup and RVIZ:

a.        cd ~/circe_ws              

b.       On ethernet:

                                                                           i.      On pi:

ros2 launch circe_bringup circe_bringup.launch.py

##  wait for "CIRCE IS UP"

ip -br addr   # confirm 192.168.50.1 is on the wired port

                                                                         ii.      On laptop:

CIRCE_CABLE=1 ~/circe_ws/laptop/view_robot.sh

c.        On wifi:

                                                                           i.      On pi:

export CYCLONEDDS_URI=file://$HOME/cyclonedds_wifi.xml

ros2 launch circe_bringup circe_bringup.launch.py

source ~/circe_ws/laptop/env.sh

export CYCLONEDDS_URI=file://$HOME/circe_ws/laptop/cyclonedds_wifi.xml

                                                                         ii.      On laptop:

rviz2 -d ~/circe_ws/src/circe_nav/rviz/nav2_view.rviz

d.        The Pi5 has Tailscale SSH which allows to connect to Pi5 over a different network (go to: [https://tailscale.com/](https://tailscale.com/) to set up connection between laptop and pi5):

                                                                           i.      On pi:

CYCLONEDDS_URI=file://$HOME/circe_ws/pi/cyclonedds_tailscale.xml ros2 launch circe_bringup circe_bringup.launch.py

                                                                         ii.      On laptop:

CIRCE_TAILSCALE=1 ~/circe_ws/laptop/view_robot.sh

**Results & Expectations:**

**Launch**

When launching circe_bringup, every node is expected to launch in a sequential order. Before the next node initialization, the current node must be verified that it has actually launched. This ensures stability across topics so that each node can subscribe or publish to topics. The launch order follows as: drivers -> sensors -> MCU -> autonomy -> WebSocket. Additionally, the bringup also includes _rosbags,_ records everything that is happening on the network. This can include topics, QoS profiles, topic rates, subscribers/publishers, etc.  

CIRCE utilizes the Raspberry Pi 5 as its single board computer. The pi5 includes a 4-core ARM-A76 CPU @2.4GHz with 8GB of RAM. The pi CPU is enough to process ROS2 for CIRCE’s task, though it still has its limitations.

To determine the stability of the launch script, the script ran ten times. Timing each launch to see how fast it can boot up CIRCE and check for any launch errors.  Each node has a timeout and a respawn=true that allows the script to relaunch the node if it fails during the bringup.

|   |   |   |   |   |   |   |   |   |
|---|---|---|---|---|---|---|---|---|
|**Run**|**Drivers**|**LiDAR**|**Camera**|**Sensors**|**EKF**|**Nav2**|**SLAM**|**Total time (s)**|
|**1**|5.5|9.5|19.1|19.7|21.8|24.9|28.0|41.6|
|**2**|5.3|7.4|16.7|17.3|19.9|22.5|25.6|39.8|
|**3**|5.1|7.2|16.5|17.0|19.0|21.2|24.0|35.7|
|**4**|5.0|7.1|16.3|16.8|18.8|21.2|23.9|36.8|
|**5**|4.9|7.1|16.4|16.9|18.8|21.3|24.1|37.0|
|**6**|5.0|FAIL|-|-|-|-|-|failed|
|**7**|4.9|11.2|20.4|20.9|22.7|25.4|28.0|40.7|
|**8**|4.8|7.0|16.2|16.7|18.6|21.1|23.8|36.7|
|**9**|4.9|7.0|16.2|16.7|18.9|21.4|24.2|35.5|
|**10**|4.8|7.0|16.2|16.8|18.8|21.1|23.7|36.0|

Table 1: Timing of each node launch ten times.

In run #6, the LiDAR launch failed. The lidar answered its health check and the driver then started the motor and asked for a scan. The lidar replied with a code that was invalid that caused the bringup to stop. The code was likely a stale byte that was left in the buffer from the previous launch. Waiting a moment before restarting the bringupcan prevent this from happening.

**Data Flow Integrity**

**Purpose**

SLAM, EKF, and Nav2 require topics that stream at reasonably good rate for CIRCEBot to keep up with its autonomy This experiment checks that every sensor and Teensy data stream arrives on the Pi at its designated rate.

**Procedure**

1.       Start the full stack.

2.       From _rosbag,_ check each topic: rate, jitter, message age (time received minus timestamp in message)

3.       Check the Teensy serial link for lost frames. The Teensy sends a heartbeat counter to the pi to check liveliness.

**Expectation**

1.       Each topic streams at its configured rate.

|   |   |   |   |   |   |   |
|---|---|---|---|---|---|---|
|**Stream**|**Configured**|**Measured**|**Jitter**|**Longest gap**|**Message Age**|**Verdict**|
|**Lidar /scan_raw → /scan → /scan_filtered**|10 Hz|6.35 Hz|7–11 ms|0.24 s|158 / 170 ms|Limited by the A1 itself; fails the 10 Hz design line|
|**D456 depth image**|5 Hz|5.00 Hz|9 ms|0.28 s|24 / 39 ms|OK|
|**D456 IMU (raw)**|200 Hz|199.5 Hz|2.5 ms|0.17 s|1.5 / 9 ms|OK|
|**/imu/data (fused)**|100 Hz|99.9 Hz|3.5 ms|0.17 s|2 / 11 ms|OK|
|**/imu/attitude**|10 Hz|10.0 Hz|6 ms|0.22 s|3 / 11 ms|OK|
|**Wheel odometry**|15 Hz|15.0 Hz (trial 2)|–|0.2 s (trial 2)|1 / 4 ms|OK|
|**EKF /odometry/filtered**|30 Hz|30.0 Hz|4 ms|0.18 s|10 / 21 ms|OK|
|**Terrain grid**|5 Hz|5.0 Hz|13 ms|0.29 s|12 / 26 ms|OK|
|**Teensy ultrasonic ×2**|≥10 Hz|10.5 Hz|8 ms|0.22 s|1 / 6 ms|OK|
|**Teensy spool status**|5 Hz|5.0 Hz|7 ms|0.36 s|1 / 5 ms|OK|
|**Teensy BMS ×2, GPS, heartbeat**|1 Hz|1.0 Hz|6–10 ms|≤ 1.13 s|≤ 8 ms|OK|
|**Local / global costmap**|5 / 1 Hz|3.2 / 0.62 Hz|–|0.44 / 2.15 s|–|Nav2 scheduling, not data loss|

Table 2: Topic publishing frequency.

·         **RPLiDAR A1:** The driver stamps each scan with the time the revolution started, so about 155 ms of the 158 ms is the revolution itself. Actual delivery delay is a few ms.

·         **Costmaps:** Publishes at about 63& of their configured rate. It’s a Nav2 timing issue that skips some cycles; every map that is sent is complete.

**QoS Configuration and Stability**

Every ROS2 topic has QoS (Quality of Service) profile that sets how data is being delivered.

·         **Reliability:** _Reliable_ retries lost packets for guaranteed delivery; _Best Effort_ skips retrying to send lost packets and send the next newest message.

·         **Durability:** _Transient Local_ retains recent messages for late-joining subscribers; _Volatile_ only delivers messages to active subscribers.

Topics with _Reliable_ profile can send to both _Best Effort_ and _Reliable._ While _Best Effort_ can only send to another _Best Effort._ This can cause publishers and subscribers to disagree and silently drop messages. This experiment checks every connection in CIRCE matches, and that each profile behaves as intended.

**Procedure:**

1.       Start the full stack.

2.       Run for t in $(ros2 topic list); do echo "=== $t"; ros2 topic info -v $t; done to list active publishers and subscribers along with their QoS policies.

3.       Check for negative control when a topic subscribes to a _Best Effort_ (e.g. /imu/data) with a _Reliable_ request.

a.        # Should be refused: prints the "incompatible QoS ... RELIABILITY" warning, then no data

timeout 5 ros2 topic echo /imu/data --qos-reliability reliable

##  Should work: messages arrive

timeout 5 ros2 topic echo /imu/data --qos-reliability best_effort

4.       Check if late-joiners receive data

a.        # Transient local: the stored value prints immediately

ros2 topic echo --once /teensy/connected --qos-reliability reliable --qos-durability transient_local

## Volatile: nothing arrives, because the value is only sent when it changes

timeout 5 ros2 topic echo --once /teensy/connected --qos-durability volatile

5.       Load test that sends 2,000 numbered messages at 200 Hz while the stack is running. Runs six times.

a.        Terminal 1:

## Count what arrives (one CSV line per message)

ros2 topic echo /e3_load std_msgs/msg/UInt32 --csv \

--qos-reliability reliable --qos-history keep_last --qos-depth 10 | wc -l

b.       Terminal 2:

## Send 2,000 messages at 200 Hz with the matching profile

ros2 topic pub -r 200 -t 2000 /e3_load std_msgs/msg/UInt32 "{data: 1}" \

--qos-reliability reliable --qos-history keep_last --qos-depth 10

|                                                      |                                                                                                                                  |
| ---------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------- |
| **Test**                                             | **Result**                                                                                                                       |
| Topics and pub/sub pairs checked                     | about 145 topics, 1,417–1,499 pairs per run                                                                                      |
| Incompatible pairs                                   | 0 in all 6 runs                                                                                                                  |
| QoS mix (216 publishers)                             | 125 reliable/volatile, 83 reliable/transient local, 8 best effort (D456 depth and IMU, /imu/data, /imu/attitude, /scan_filtered) |
| "Incompatible QoS" in bringup logs                   | 0.                                                                                                                               |
| Negative control (Reliable on best-effort /imu/data) | Refused 3/3: 1 RELIABILITY event, 0 messages. Best-effort request: about 500 messages in 5 s                                     |
| Late joiner, transient local                         | Value received 9/9 times, within 4–31 ms                                                                                         |
| Late joiner, volatile                                | 0 messages, 9/9 times                                                                                                            |



# Hardwired Communication
**Experiment 1: Establish communication**

Purpose and Justification:

The purpose of this experiment was to determine whether a connection between the Raspberry Pi 5 and a laptop was able to be established. This experiment tested whether the laptop and Raspberry Pi 5 could send and receive simulated commands and telemetry data using proto formatting. This test was run with simulated python data and actual data using ROS nodes.  Achieving a connection is essential for the Hardwired Communication subsystem, as the subsystem’s purpose is to transmit telemetry data from CirceBot and receive commands from the user with CirceSoft. This experiment verified whether CirceBot could successfully communicate to and from CirceSoft on a laptop using a WebSocket and TCP connection. This test will ensure CirceBot can run on its own with an Ethernet connection.

Detailed Procedure:

The following procedure was used during the experiment:

·         Set up

o   Power on the Raspberry Pi 5 (RPi) and laptop

o   Connect the laptop and Raspberry Pi 5 using a CAT5e ethernet cable and an ethernet adapter if need be

o   Temporarily connect the Raspberry Pi 5 to a mouse, keyboard, and monitor

o   Download Ubuntu on Raspberry Pi 5 if needed

·         Initialize

o   Connect the Raspberry Pi 5 and the laptop to the internet and download the files needed to run the programs on both

o   Navigate to the folder with the program on the Raspberry Pi 5 and laptop and open a terminal on each

o   Download the required objects on RPi

§  sudo apt install python3-pip

§  python3 -m venv venv && source venv/bin/activate

§  pip install “websockets>=13” protobuf

o   If objects need to be updated run this code instead

§  sudo apt update

§  sudo apt install python3-websockets

o   Ensure python3 and the required objects are downloaded on the laptop in the CirceSoft folder

o   Using the terminal, locate the IP address on the Raspberry Pi 5 and laptop

§  For the RPi run hostname -I to find the IP address

§  For the laptop navigate to settings to find IP address

o   Use ping on both RPi and Laptop to ensure ethernet is connected

o   In the event that the laptop is not able to ping the RPi try the following:

§  Set a temporary IP address for the Pi

·         sudo ip addr add \<IP address> dev eth0

·         sudo ip link set eth0 up

·         ip addr show eth0

§  Check your firewall and check what connections have access to your laptop

§  Enable communication to/from port 8000 on laptop firewall

o   Ensure all the files needed are in their respective folders on the RPi and laptop

§  The RPi and laptop need both proto files to run

§  To test the connection, the actual robot data is needed using ROS to properly test the connection (part of the navigation and O.S. subsystems)

§  A script with fake data under the same variable names may be used as an alternative method of communication for the RPi

·         Test connection

o   Once the ping is tested and the required objects are downloaded run the given code

§  For the RPi run: python circebot_comm.py

§  For the laptop terminal run: python circesoft_comm.py

o   WebSocket’s/TCP connection should establish, and the proto files should be sent

Expected Results:

Hypothesis:

·         The TCP connection and WebSocket connection will be established.

·         CirceBot shall send telemetry data using the proto format.

·         CirceSoft shall send commands using the proto format.

·         CirceBot and CirceSoft will communicate bidirectionally using the proto format over the ethernet cable connection.

Actual Results:

|   |   |   |   |
|---|---|---|---|
|Trial type|Function Tested|Result|Conclusion|
|Python Simulation|Websocket/TCP connection established|Successful|Connection can be established and recognized by the Raspberry Pi and the laptop.|
|Python Simulation|CirceBot sends telemetry|Successful|CirceBot can send simulated data to CirceSoft.|
|Python Simulation|CirceSoft sends  commands|Successful|CirceSoft can send simulated commands to the Raspberry Pi.|
|Python Simulation|Bidirectional communication established|Successful|CirceSoft and CirceBot can send and receive data/commands simultaneously.|
|Actual ROS Data|Websocket/TCP connection established|Successful|The connection can be established and recognized by the Raspberry Pi and the laptop.|
|Actual ROS Data|CirceBot sends telemetry|Successful|CirceBot can send simulated data to CirceSoft.|
|Actual ROS Data|CirceSoft sends  commands|Successful|CirceSoft can send simulated commands to the Raspberry Pi.|
|Actual ROS Data|Bidirectional communication established|Successful|CirceSoft and CirceBot can send and receive data/commands simultaneously.|

Discussion:

This experiment demonstrates that the Raspberry Pi 5 can successfully establish an ethernet connection over the WebSocket and TCP connection. The functionality of the WebSocket’s and TCP connection worked as expected. The user is able to receive telemetry data and send commands without the use of other means. This experiment confirms that the Hardwired Communication Subsystem is working as intended. Both simulated telemetry data (using a python script) and data from the actual robot (using ROS) were tested. Both methods were successful in establishing a WebSocket/TCP connection and sending the proto files bidirectionally.

Conclusion:

                The experiment met all expectations and specifications tested. The WebSocket and TCP connection was able to be established between the Raspberry Pi 5 and laptop through ethernet connection. Both simulated and actual data and commands were able to be sent and received using the proto formatting. Overall, this experiment was a success.

  

**Experiment 2: Communication Specifications**

Purpose and Justification:

                The purpose of this experiment was to determine whether a connection between the Raspberry Pi 5 (RPi) and a laptop was able to be maintained during the expected time of operation. This experiment tested whether the laptop and RPi could send and receive simulated commands and telemetry data using proto formatting for a 20-minute minimum operation time. The experiment tested the latency, packet loss, and frequency to determine whether the minimum specifications were met for the hardwired communication subsystem. The hardwired communication subsystem is expected to run at a minimum frequency of 10 Hz, maintain an average latency of less than 20 ms, and keep the packet loss under 1% of the total messages sent during the operation time. Establishing and retaining a fast, strong communication subsystem is necessary for critical missions where information needs to be sent at a constant rate. This experiment was run with actual data using the ROS nodes.

Detailed Procedure:

·         Set up and test the IP address using the ping command for CirceSoft and CirceBot in the same manner as the previous experiment

·         Once the ping is tested and the required items are downloaded run the given code

o   For the RPi run: python circebot_comm.py

o   For the laptop terminal run: python circesoft_comm.py

·         The circesoft_comm.py will display the frequency, packet loss, RTT, and average latency

·         Record data points at regular intervals manually or with an automatic program such as ping_logger.py in the terminal

·         Reset and run the experiment for a minimum of 3 times for accurate data

Expected Results:

Hypothesis:

·         The connection will be under the maximum allowable latency (20 ms) during the whole 20-minute operation

·         The connection will be under the maximum allowable packet loss (1%) during the whole 20-minute operation

·         The connection will not drop below the 10 Hz minimum frequency during the whole 20-minute operation

Actual Results:

|   |   |   |   |   |   |   |
|---|---|---|---|---|---|---|
|Trial Number|Function Tested|Statistical model or method|Sample size or duration|Average Value|Result|Conclusion|
|1|Latency|Histogram and Bar Chart|20 minutes|5.21 ms|Success|The average latency measured was below the 20 ms maximum.|
|1|Packet Loss|Scatter Plot|20 minutes|0 %|Success|There was no packet loss, which is below the 1 % maximum.|
|1|Frequency|Histogram|20 minutes|19.998 Hz|Success|The average frequency was well above the 10 Hz minimum.|

![](data:image/png;base64,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)

Figure 1: Box and Whisker Chart of latency.

![](data:image/png;base64,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)

Figure 2: Histogram of latency values measured.

![](data:image/png;base64,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)

Figure 3: Box and Whisker Chart of frequency measured.

![](data:image/png;base64,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)

Figure 4: Scatter plot of measured packet loss.

Discussion:

                Based on the experiments run on the hardwired communication subsystem, the communication is expected to deliver stable and fast messages during the time of operation. The experiment verified that CirceBot can maintain a connection above the minimum frequency and below the maximum allowable packet loss and latency. The average latency was 5.21 ms, where the maximum latency was 14.685 ms and the minimum latency was 1.88 ms.  The standard deviation of the latency was 3.06 ms, while the median was 4.55 ms. The average frequency was 19.998 Hz, where the minimum frequency was 18 Hz and the maximum frequency was 21 Hz. The standard deviation of the frequency was 0.55 Hz, and the median of the frequency was 19.9 Hz.  There was no loss of message packets during the whole 20-minute operation. All values were well within the specifications listed for the hardwired communication subsystem.

Conclusion:

                The experiment met all expectations and specifications for the hardwired communication subsystem. The data and commands are able to be sent at a fast rate while maintaining a secure connection. Each value tested was well within the specifications. Thus, CirceBot should be able to run in its expected environment (Tennessee) at its expected operation time (20 minutes).

Inventory check:

|   |   |   |   |   |   |   |   |   |
|---|---|---|---|---|---|---|---|---|
|Part #|Description|Quantity|Vendor|Order #|Storage Location|Date Acquired|Condition|Notes|
|U3-GE-1P|Ethernet to USB adapter|1|Amazon||Capstone Lab||||
|731433618240|Rotatable Ethernet Extender|1|Amazon||Capstone Lab||||
|YS-52QY-IR7P|Crimping Kit|1|Amazon||Capstone Lab||||
|1016-Amazon|Ethernet Cable|1|Amazon||Capstone Lab||||

Hardwired Communication Success Criteria:

Success Criterion: Establish and maintain a reliable and fast ethernet connection for communication between CirceBot and CirceSoft capable of transmitting telemetry data and user commands in the specified format.

Results:   Fully met

Evidence:

·         The Raspberry Pi 5 can establish and send message packets in the .proto format to and from a laptop using the WebSocket/TCP connection

·         The latency was under the 20 ms maximum latency as stated in the specifications

·         The packet loss was under the 1% maximum as stated in the specifications

·         The frequency was maintained at a rate above the minimum set in the specifications

Reason for Incomplete Success: none

Future Improvements and Recommended Next Steps:  

·         Optimize the frequency and latency rates for faster communication

·         Expand upon CirceSoft

o   Add advanced security features

o   Develop the CirceSoft terminal application


# Drivetrain analysis
**Drivetrain Subsystem: Experimental Analysis**

**Introduction**

The purpose of the drivetrain experimental analysis is to evaluate the functionality and performance of the Rover Robotics 4WD Rover Zero 3 platform selected for the CIRCE system. The drivetrain is responsible for converting navigation commands into physical movement while supporting the weight of the Ethernet cable and additional subsystems.

Experimentation focuses on motor control, wheel odometry, waypoint navigation, payload capacity, and terrain traversal. These experiments are intended to determine whether the drivetrain satisfies the specifications and constraints established in the Conceptual Design and Detailed Design reports.

The results of each experiment are compared against the original design requirements to identify limitations, verify functionality, and determine whether further modifications or testing are necessary.

**Critical Success Criteria**

- The drivetrain shall respond correctly to velocity and directional commands.
- The drivetrain shall provide wheel encoder feedback to support odometry calculations.
- The drivetrain shall support integration with ROS 2.
- The drivetrain shall navigate to specified waypoints and stop upon reaching its destination.
- The drivetrain shall support the weight of the Ethernet cable and additional subsystems.
- The drivetrain shall operate on uneven outdoor terrain without becoming immobilized.

**Experiment 1: Motor Control and Drivetrain Functionality**

**Purpose and Justification**

The purpose of this experiment was to verify that the drivetrain subsystem could successfully receive and execute movement commands through ROS 2. This experiment directly addresses the drivetrain design requirement to convert navigation data into motor outputs that control the robot's speed and direction.

The experiment also evaluates communication between ROS 2 and the Rover Zero motor controller. Establishing reliable motor control is necessary before implementing autonomous waypoint navigation and integrating the remaining CIRCE subsystems.

**Detailed Procedure**

**Equipment Used:**

- Rover Robotics 4WD Rover Zero
- Dual FSESC 4.20 motor controller
- Integrated brushless motors
- Integrated Hall effect encoders
- Rover battery
- USB communication cable
- Computer running Ubuntu 22.04 through WSL2
- ROS 2 Humble

**Experimental Procedure:**

1. Power on the Rover Zero and establish a USB connection between the Rover and the computer running ROS 2 Humble.
2. Launch the Rover Robotics driver to establish communication between ROS 2 and the drivetrain.
3. Publish a positive linear velocity command to the /cmd_vel topic and observe the forward movement of the Rover.
4. Publish a negative linear velocity command and observe the reverse movement of the Rover.
5. Publish angular velocity commands and observe the rotational movement of the Rover.
6. Publish a zero-velocity command to verify that the Rover stops.
7. Launch keyboard teleoperation to verify that the Rover responds to manual directional commands.
8. Monitor the /odometry/wheels topic to verify that the wheel odometry updates in response to physical movement.
9. Record whether the Rover successfully executes each movement command and identify any unexpected behavior.

**Data Collection:**

Testing was conducted indoors. The physical response of the Rover was visually observed following each movement command. Position and orientation feedback were monitored through the /odometry/wheels topic. Each tested function was evaluated on a pass/fail basis.

**Trials:**

Movement commands were tested during initial drivetrain development. However, the exact number of repetitions was not formally recorded.

**Potential Sources of Error:**

- Wheel slippage during movement and rotation.
- Communication delays between ROS 2 and the motor controller.
- Accumulated wheel odometry error.
- Subjectivity associated with visually evaluating physical movement.

**Expected Results**

The Rover is expected to successfully execute forward, reverse, rotational, and stop commands received through ROS 2. The drivetrain should also respond correctly to keyboard teleoperation commands.

Additionally, the wheel odometry data is expected to update as the Rover changes its position and orientation, demonstrating that the drivetrain provides the necessary feedback for autonomous navigation.

**Actual Results**

The Rover successfully responded to linear and angular velocity commands published through the ROS 2 /cmd_vel topic. Forward movement, reverse movement, rotational movement, and keyboard teleoperation were successfully demonstrated on the physical Rover.

Wheel odometry data was monitored through the /odometry/wheels topic. Changes in the Rover's estimated position and orientation were observed during physical movement.

**Table 1: Drivetrain Functional Testing Results**

|   |   |   |   |
|---|---|---|---|
|**Test**|**Expected Result**|**Actual Result**|**Status**|
|Forward Movement|Rover moves forward|Rover moved forward|Pass|
|Reverse Movement|Rover moves backward|Rover moved backward|Pass|
|Rotational Movement|Rover changes heading|Rover successfully rotated|Pass|
|Stop Command|Rover stops moving|Rover stopped|Pass|
|Keyboard Teleoperation|Rover responds to keyboard inputs|Rover responded correctly|Pass|
|Position Feedback|Odometry position updates|Position updated|Pass|
|Orientation Feedback|Odometry orientation updates|Orientation updated|Pass|

**Interpretation and Conclusions**

The experimental results demonstrate that the drivetrain successfully receives and executes movement commands through ROS 2. The Rover Robotics driver provides the necessary interface between the software and the physical drivetrain.

The wheel odometry feedback also provides the position and orientation information necessary for implementing autonomous waypoint navigation.

The results matched the expected functional behavior of the drivetrain. However, actual velocity, positioning accuracy, and trial counts were not formally recorded during this experiment. Therefore, the results verify drivetrain functionality rather than quantitative performance.

The drivetrain satisfies the tested movement control requirements established in the Detailed Design and provides a foundation for further autonomous navigation and subsystem integration.

**Experiment 2: Wheel Odometry and Waypoint Navigation**

**Purpose and Justification**

The purpose of this experiment was to evaluate the drivetrain's ability to utilize wheel odometry feedback to autonomously navigate to specified waypoints. This experiment directly addresses the drivetrain design requirements to move toward a specified location, provide position and heading feedback, and stop upon reaching its destination.

Additionally, the experiment evaluates the drivetrain's response to a loss of odometry feedback to prevent continued autonomous movement when positioning information becomes unavailable.

Successful implementation of wheel odometry-based navigation is particularly important for the CIRCE system because the robot must be capable of operating in environments where GPS information is unavailable.

**Detailed Procedure**

**Equipment Used:**

- Rover Robotics 4WD Rover Zero
- Dual FSESC 4.20 motor controller
- Integrated brushless motors
- Integrated Hall effect encoders
- Rover battery
- USB communication cable
- Computer running Ubuntu 22.04 through WSL2
- ROS 2 Humble
- CIRCE waypoint controller
- Python and Matplotlib waypoint visualization software

**Experimental Conditions:**

|   |   |
|---|---|
|**Parameter**|**Value**|
|Testing Environment|Indoors|
|Surface|Tile|
|Additional Payload|None|
|Positioning Method|Wheel Odometry|
|Commanded Linear Velocity|0.10 m/s|
|Maximum Commanded Angular Velocity|0.30 rad/s|
|Initial Waypoint Tolerance|0.15 m|
|Final Waypoint Tolerance|0.05 m|
|Random Waypoint Generation Area|3 m × 3 m|
|Number of Random Waypoints|5|
|Odometry Timeout Threshold|1.0 s|
|GPS Assistance|None|
|LiDAR Assistance|None|

**Experimental Procedure:**

1. Position the Rover on a flat indoor tile surface with sufficient space for autonomous movement.
2. Power on the Rover and establish communication with the computer running ROS 2 Humble.
3. Launch the Rover Robotics driver to enable motor control and wheel odometry feedback.
4. Launch the CIRCE waypoint controller and waypoint visualization software.
5. Verify that wheel odometry data is being received through the /odometry/wheels topic.
6. Configure a single waypoint at coordinates (0.50 m, 0.00 m) with an acceptance tolerance of 0.15 m.
7. Start the autonomous navigation mission and observe the Rover's movement toward the specified waypoint.
8. Record the final position reported by the wheel odometry and calculate the remaining distance to the target.
9. Configure four waypoints forming a 0.50 m × 0.50 m square.
10. Start the square navigation mission and observe the Rover's ability to transition between consecutive waypoints and return toward its starting position.
11. Record the final estimated position and calculate the remaining distance to the final waypoint.
12. Reduce the waypoint acceptance tolerance from 0.15 m to 0.05 m.
13. Generate five random waypoints within a 3 m × 3 m coordinate area.
14. Record the generated waypoint coordinates.
15. Start the random waypoint navigation mission and observe the Rover's ability to reach all five waypoints without manual intervention.
16. Monitor the estimated position and traveled path of the Rover using the waypoint visualization software.
17. Verify that the Rover stops upon reaching the final waypoint and that the waypoint controller reports successful mission completion.
18. Record the final estimated position and calculate the remaining distance to the final waypoint.
19. Record a video of the random waypoint navigation experiment.
20. During a separate autonomous navigation test, terminate the Rover Robotics driver to simulate a loss of odometry feedback.
21. Record the time required for the waypoint controller to detect the loss of feedback.
22. Restart the Rover Robotics driver and verify that autonomous movement does not automatically resume without a new start command.

**Data Collection:**

Position and orientation data were obtained through the /odometry/wheels topic. Position was recorded in meters, while heading information was obtained from the orientation data provided by the wheel odometry.

The waypoint visualization software was used to monitor the estimated position of the Rover relative to the specified waypoints. The software displayed the target waypoints and the estimated path traveled by the Rover throughout the navigation mission.

The ROS 2 terminal output was monitored to determine when individual waypoints were reached and when the overall navigation mission was completed.

The random waypoint experiment was also recorded on video to document the physical movement of the Rover, real-time waypoint visualization, and terminal output.

The remaining distance between the estimated Rover position and the target waypoint was calculated using the Euclidean distance equation:  
  

![](data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAMcAAAAaCAMAAAA0e7gFAAAAAXNSR0IArs4c6QAAALpQTFRFAAAAAAAAAAA6AABmADpmADqQAGa2OgAAOgA6OgBmOjo6OjqQOmaQOma2OpDbZgAAZgA6ZgBmZjoAZjo6ZjpmZjqQZmYAZmZmZmaQZpBmZpDbZrbbZrb/kDoAkDo6kDpmkGY6kGaQkJBmkLb/kNvbkNv/tmYAtmY6tmZmtmaQtpA6tpCQttv/tv//25A625Bm25C227Zm27aQ27a229uQ2/+22//b2////7Zm/7aQ/9uQ/9u2//+2///bZtaD0QAAAAF0Uk5TAEDm2GYAAAAJcEhZcwAADsQAAA7EAZUrDhsAAAAZdEVYdFNvZnR3YXJlAE1pY3Jvc29mdCBPZmZpY2V/7TVxAAACbUlEQVRYR+1XiVLCMBBtEawXqOAt9YSiCMXbmjb//1sm2SakTbKUzjgyI51xxjFvX/Z6u9Hz1t86A3+YARr6q/xVzgzZ/aqMXWVg0lll76r7NhpWx64wMjuaCu9Ib/l44r2JsH0J/D1Xc9bgtdFG/iaexKQtzuNaKiE7PHimsHS/b7+mFq+FloaOC+S1sTgn21CVZb9kA+zotbKPdb3V5DVpswO8XcABGtYUuzSczbOlx1GX16RdNFXhnGwtLw4oXdzi9rP7eSH1OGrzFmjTrt+4y+Ux33biYvXB1M3rmHYbQ2/EflyfCRCWrBqJKogeR8478pkIMaUaAJ2WBO2vbH9Bw8DUhfDp42fYGV26NWIB8IyTgD0IrHEA73c/ZtEiHW4CdNqIkSDG6S3LZD514T4ekK/HbSliEVDqSAMveYVbMBjFVFBPIXlvCaA3esZHoZoXhunsPGAu5+QqDjkonDUpA0wF6H0lebkvD5gEywCNVvwaIduD1wumrtSHR28QcYhMlgFm4BZ9cD/Ihbtf2UkJoNGSoE+fThB7DriCJOXh06tB2CFMIHLJl69WAHWgKjn/i9aZKq1x84x1rouWN7QAWGnHfnPy4rfdr9mo9XEMpzCvo+aQCYQZcOHaCqkAaXdzxtvStnis+yPhdXbRio7gAIwWqSYJTmXuSu1R2MkWhrf3rUsxsS16KtjKc9iTCK0AYLRYV0batCyslYVv4BjKtXjPib6jY8iXk1YCKtMWo9JfP6/aezc7nD7j/1xFkN/8vYvl6rU3CBuwk1y0VAK86rTYleosCxv465J5VImoCPol2hqerE3+fQZ+ADi/VABPItENAAAAAElFTkSuQmCC)  

Where:

- d = Remaining distance to the target waypoint (m)
- ![](data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAA4AAAAWCAMAAADHE10VAAAAAXNSR0IArs4c6QAAAGBQTFRFAAAAAAAAAAA6AABmADpmAGa2OgAAOma2OpDbZgBmZjoAZjpmZrbbZrb/kDoAkDo6kGY6kNvbkNv/tmYAtmY6tmaQtv//25A625C22//b2////7Zm/9uQ/9u2//+2///bqkoOEwAAAAF0Uk5TAEDm2GYAAAAJcEhZcwAADsQAAA7EAZUrDhsAAAAZdEVYdFNvZnR3YXJlAE1pY3Jvc29mdCBPZmZpY2V/7TVxAAAAYElEQVQoU2NgGCggy8UkxCACxGAgLyrJxyHCg+QYcUYOZKdJM4shceUFYBrBmvkF+ThkgJrluIGKhFmEGMQZ2aUYZNgYGVmRTUQxkEEEajVEhRynmIQUQrEcHxMvaSEDAOhvA7T4oa7jAAAAAElFTkSuQmCC) = Target x-coordinate (m)
- ![](data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABIAAAAWCAMAAAD6gTxzAAAAAXNSR0IArs4c6QAAAGZQTFRFAAAAAAAAAAA6AABmADpmAGa2OgAAOgA6Oma2OpDbZgAAZgA6ZgBmZjpmZmaQZpDbZrbbZrb/kDoAkNv/tmYAtmY6tmZmtpA6tv//25A625Bm25C22////7Zm/7aQ/9uQ//+2///bzueSpgAAAAF0Uk5TAEDm2GYAAAAJcEhZcwAADsQAAA7EAZUrDhsAAAAZdEVYdFNvZnR3YXJlAE1pY3Jvc29mdCBPZmZpY2V/7TVxAAAAa0lEQVQoU81PRxKAIAxcUOwNK9jA/39S1BkFr17cUybZFuCv6EgEtIQ9/VYuPQGdNnZlFTZQhmlBJxy9QzJGTJWvT6VfzIDOxbNfqJGpgFipIz/OMr5J23DN3Z1Q0+rc6ExMxtGpUtNT/wE70VcEadPS0esAAAAASUVORK5CYII=)= Target y-coordinate (m)
- ![](data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAA8AAAAWCAMAAAAo0TYrAAAAAXNSR0IArs4c6QAAAGZQTFRFAAAAAAAAAAA6AABmADpmAGa2OgAAOgA6Oma2OpDbZgBmZjoAZrbbZrb/kDoAkDo6kGY6kNvbkNv/tmYAtmY6tmaQtv//25A625Bm27a229uQ2//b2////7Zm/9uQ/9u2//+2///bSzRjAQAAAAF0Uk5TAEDm2GYAAAAJcEhZcwAADsQAAA7EAZUrDhsAAAAZdEVYdFNvZnR3YXJlAE1pY3Jvc29mdCBPZmZpY2V/7TVxAAAAXUlEQVQoU2NgGGggz80kxCACxBCgKCrNxynCg+wqcUZOFEfKMosh8xUF4JrBBvAL8nHKwQ0QZhFiEGfkkGGQ52aVYEMySFKKnUcW2WBxVtSgEOZF4StwodhKZLABABNZA7Az+L8sAAAAAElFTkSuQmCC) = Estimated Rover x-coordinate (m)
- ![](data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABIAAAAWCAMAAAD6gTxzAAAAAXNSR0IArs4c6QAAAGxQTFRFAAAAAAAAAAA6AABmADpmAGa2OgAAOgA6Oma2OpDbZgAAZgA6ZgBmZjoAZmaQZpDbZrbbZrb/kDoAkNv/tmYAtmY6tmZmtpA6tv//25A625Bm27a229uQ2////7Zm/7aQ/9uQ/9u2//+2///brXkMKQAAAAF0Uk5TAEDm2GYAAAAJcEhZcwAADsQAAA7EAZUrDhsAAAAZdEVYdFNvZnR3YXJlAE1pY3Jvc29mdCBPZmZpY2V/7TVxAAAAa0lEQVQoU2NgGKxAlJGDgUGEkRXhPnlBSWYJBiVuYWQnK7ALMygAVSIBJS5BBjEURUCDWBX40XwqycIniyYkxwTUpsjLKsXGCZOREgSypGXYBeSgQsriEIYk3GVCTAIQ1SIgtahO4ZGgNHwB0QUEVKTVwr8AAAAASUVORK5CYII=)= Estimated Rover y-coordinate (m)

The waypoint controller considers a waypoint reached when the calculated distance is within the specified acceptance tolerance.

The waypoint completion rate was calculated using the following equation:

![](data:image/png;base64,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)

**Trials:**

Testing was performed progressively, beginning with a single waypoint, followed by a square navigation path and a random waypoint navigation path. One recorded trial of the five-waypoint random navigation experiment was conducted.

Each navigation experiment evaluated a different level of complexity. The single-waypoint experiment evaluated basic autonomous movement, the square-path experiment evaluated consecutive waypoint transitions, and the random waypoint experiment evaluated navigation through a less predictable path with a reduced acceptance tolerance.

The number of repetitions was insufficient to establish statistical repeatability.

**Potential Sources of Error:**

- Wheel slippage on the tile surface.
- Accumulated wheel odometry error over extended distances.
- Differences between commanded and actual motor velocities.
- Communication delays between ROS 2 and the drivetrain.
- Differences between the estimated and actual physical position of the Rover.
- Rounding of waypoint coordinates displayed in the ROS 2 terminal.
- Limited number of experimental trials.
- Lack of an independent physical positioning measurement system.

**Expected Results**

The Rover is expected to autonomously navigate toward each specified waypoint using wheel odometry feedback. The waypoint controller should continuously calculate the remaining distance and heading relative to the target waypoint and generate the appropriate linear and angular velocity commands.

Upon reaching the specified acceptance tolerance, the Rover should proceed to the next waypoint. After reaching the final waypoint, the Rover should stop without requiring manual intervention.

For the single-waypoint and square-path experiments, the Rover is expected to reach each specified waypoint within an estimated distance of 0.15 m.

For the random waypoint experiment, the Rover is expected to successfully navigate through all five randomly generated waypoints using a reduced acceptance tolerance of 0.05 m. The expected waypoint completion rate is 100%.

The waypoint visualization software is expected to display the estimated position and traveled path of the Rover throughout the navigation mission.

During a loss of odometry feedback, the waypoint controller is expected to detect the interruption within approximately one second, suspend autonomous navigation, and issue a zero-velocity command. Autonomous navigation should not resume automatically when communication is restored.

**Actual Results**

The Rover successfully completed the single-waypoint, square-path, and random-waypoint navigation experiments.

**Single Waypoint Navigation:**

The Rover was commanded to navigate toward a waypoint located at (0.50 m, 0.00 m) with an acceptance tolerance of 0.15 m.

The final position reported by the wheel odometry was approximately (0.3953 m, -0.0052 m).

The remaining distance to the target waypoint was calculated as:

![](data:image/png;base64,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)  
![](data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAFwAAAAWCAMAAAB68gtgAAAAAXNSR0IArs4c6QAAAJNQTFRFAAAAAAAAAAA6AABmADo6ADpmADqQAGaQAGa2OgAAOgA6OgBmOjpmOjqQOmaQOma2OpCQOpDbZgAAZgA6ZjoAZjo6ZmYAZpC2ZpDbZrbbZrb/kDoAkDo6kGY6kJBmkNv/tmYAtmY6tpA6tpCQttu2ttv/tv//25A625Bm27Zm2/+22////7Zm/9uQ/9u2//+2///bJScOqQAAAAF0Uk5TAEDm2GYAAAAJcEhZcwAADsQAAA7EAZUrDhsAAAAZdEVYdFNvZnR3YXJlAE1pY3Jvc29mdCBPZmZpY2V/7TVxAAABVklEQVRIS+2Ty1qDMBCFM1Sw8YZitQJWsFouJoG8/9M5maSCQrpTN2TF9yc5Z+ZkYGxZ/5RACevfc9Z55hNvOQTpcVPv7LkRVNwgvQOAGzEr0t8XHnHFU1YHbhc1SXwEdW6Qzs8F65L59tX1vCljZSRY724124+ExEfw9dagnniNZ3+sbgPBi/PEOtxy53R+58TolhUZQRlbZOw6/JKwet9AJBoeGKx4LPrESMwtu2OujsQHqK4q57cH+zQy2ArFw0LTHWrRG/lpcf1cmGbqjNWYq+JYiFxVtjOTERmry8pWNonFxj2p3EHpMsxc5ij8TVxd4CSU/ik3ujbjIXMyO0JbHc1jPRHnmX57XKsnT+gSUtbiKFLP7kHx2Sz88tN5dLCjOFSOTmwP4aGF2DeKrOEQoowRVw8Yw5kpw0GyAxp0/InwKfEYjkqJCBOLPPUueEngrxP4BHxMKQ8joQ4KAAAAAElFTkSuQmCC)

The estimated remaining distance was approximately 10.48 cm, which was within the specified acceptance tolerance of 15 cm.

The Rover successfully completed the single-waypoint navigation experiment and stopped without requiring manual intervention.

**Square Waypoint Navigation:**

The Rover was commanded to navigate through four waypoints forming a 0.50 m × 0.50 m square.

The waypoint coordinates are provided in the following table.

|   |   |   |   |
|---|---|---|---|
|**Waypoint**|**X-Coordinate (m)**|**Y-Coordinate (m)**|**Result**|
|1|0.50|0.00|Reached|
|2|0.50|0.50|Reached|
|3|0.00|0.50|Reached|
|4|0.00|0.00|Reached|

The Rover successfully navigated through all four waypoints. The final position reported by the wheel odometry was approximately (0.0182 m, 0.1077 m).

The remaining distance to the final waypoint was calculated as:

![](data:image/png;base64,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)

![](data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAE4AAAAWCAMAAABZqVq1AAAAAXNSR0IArs4c6QAAAIdQTFRFAAAAAAAAAAA6ADo6ADpmADqQAGa2OgAAOgA6OgBmOjqQOmaQOma2OpC2OpDbZgAAZgA6ZjoAZjo6ZmYAZpC2ZpDbZrbbZrb/kDoAkGY6kJBmkNv/tmYAtmY6tpA6tpBmtpCQttv/tv//25A625Bm27Zm2/+22////7Zm/9uQ/9u2//+2///bq5EOvwAAAAF0Uk5TAEDm2GYAAAAJcEhZcwAADsQAAA7EAZUrDhsAAAAZdEVYdFNvZnR3YXJlAE1pY3Jvc29mdCBPZmZpY2V/7TVxAAABJklEQVQ4T+2SW1vCMAyG24lUZDo8MKpOFMoscf3/v88c2qqwesOl5GrPl+Rt8i1KneNEBzo9PZHwqz3YVQnXG121KRmepC6J4VVLNrxh2TKVDYuXAg5Mq1wVs0hhXBa7S696zAZ745WTJOWvfQFHDUMjVuwePxruSCJc4TvB3krv0NDH552unqN1weoYSKGQWmqPHYTLIuNUF5vBYBJM7YU7FpI5wGURx/Y4TsQxlgcvWvc3TgFuNreCc2TYQG7AbCujHS0rth1N91NkAtJqMoTXT9uPrEukbLY8LvgsOjZ2jzSo0bpVeH+YQr6ZA+Ret3wKYMjEiEsi7bPmK+LTcFiy1pNNr3nS0dgZPcEGwsE9/vYLejiKSKjmm2+TSj+0xD7r/9WBLwx2HHsBmpkHAAAAAElFTkSuQmCC)m

The estimated remaining distance was approximately 10.92 cm, which was within the specified acceptance tolerance of 15 cm.

The waypoint completion rate was calculated as:

![](data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAQsAAAAnCAYAAAAYTG2rAAAAAXNSR0IArs4c6QAAAAlwSFlzAAAOxAAADsQBlSsOGwAAABl0RVh0U29mdHdhcmUATWljcm9zb2Z0IE9mZmljZX/tNXEAAA0wSURBVHhe7V2/byLXFj7D237TUz6GwqF1pB3SO4wb0vAkGl6zkM6zhavwcGDdEWmHVIE0oUEKlRvDuo9ZKbQIaRlS0sd/gLnvO3cYGH5jL2Yxe0eyZMGde8/57r3fPb+GeZHP50ldCgGFgEJgHQIv1jVQ3ysEFAIKAUZAkYVaBwoBhcBGCCiy2Agm1WgWgVzypYjqFkUagoLtgqYQOnwEFFkc/hxvXcNcLilKUZ1a6Dmy9d5Vh/uKgCKLfZ2ZPZarmUlRPWFTumXtsZRKtG0joMhi24geeH+544GIDrJ0e0aUUVxx4LM9rZ4iiy9quj9NWRmnSA0oextDR81P60zd/ewQUGTx7Kbs8wjsxilSlKjeUrtQ0GK5Y+FJknx5I4rhWxXo/DxTs7NRFVnsDOpnPlCzSFYLIU1dJj7ExcVIH1OjCv5NN565fkr8tQgoslgLkWrACBTaQc1f7ZuDZZHRTCIvddouKKAOHAFFFgc+wUo9hcC2EFBksS0kv6B+PKuC3Q+CG5JuCKEKsw5/ASiyOPw53rqGhUJbC+IBxPEjiMoF2TrG+9jhmCw42t0sFenSqsjKPHkZabKz53TXru1FOa9XOVhPOHRy93iZkscvxbV+Rne1pytTPh5cCFMevf7LoLRdpeAnyL6Pi0jJ9GUgIMnC24RWyyC74VBVd5V3rlNkmjoZtiM+ZXN+TiiTLwci1T2iavmMakj5jXWNhCkffGLJDJsa1VMaweniaekPMttn5X9iifeuezlfqBjtxhenZpMoEkuBld0DDmTcABnPHG7r2nD9SCZlUQWdGEjrnATbU4cjF6Jpl0fkoBKN19AuQPLrXTEpkM8XxqlqHj/5zWCYilU0v94VMzTVbl2bXPKrYSZ1pnl6tyom7s+Px4Hew8DlkfbxzzNOggUkWfRLKaTFQBTOrTxta20PjhNybBK6VaSsKAvOr+8CqO2O0aFW52jcZaEAi+SETeixktsdbqY3PRQaL7Bcrirsuk7WVfMBRDUt/5MKu+XOZbXnVXxu88kDijdoMUzn5djSDcibPKq5z6Ck4/PCuQ+zXeBhNliaIAiu99DNFK9j4VmN69p49SNHVUEi1Cd+5qUbn8RgvMMlnRU7I4ppvXnvTm873uTR8IUWafSoZeoBpxS9D5spze7dDi09F8gXCmJdG0H9e9TNaBO9w8BxOLyIae79ueSw9G1Ye529pzCIgj974YKBpDkYdZFZHjrNkt3tgVEmkyVPu7G7Ms/mbIJfHjUo270kk2mLOR/9V8s6OTgl/J/5Wdy9D5YNFcnr30jbY6tg0VqW7pOvT+k6Vct0Npp4XQ7fAjNarKPI49TwxvFbSxvrRJdw1VpyAa+TbdneM45ga9y5ZDUn/+h0xB6SC3eR/Mt0fkq36lE8EjunxCXIEflVxt3rw//E6rKTmuej2ItTVcSpyCnaBVezaFEL1ls1BkIGnKGzLKUtk6zihIzXtulfU50SVA0hPYyDJJkwRL3XJ8/olAdppEH5HT1ZO6V3YLHe74tn1Hpl0+8xncrSDvgfvbZiZP2MqlqYIYR40to2TmmsN5ALnP7n1X2997dGJoBAMKr/S4refH1NwgR5jKyaF8RgYeWns1id7fnTtlDDJAO5ds3No0v2tiLUcAR/prH/n4KrgvUwFRFvWdhUdpYcEaNQv0RR3SS94hKLU4ZAzQylTJMGc/elqIj7qqKM+5qU4ftoucvQzOhkdmyMcyuZn1lZ16PwoWAlneQ1JwF564mVJuRDdaqOdbIodXRKJyt2ktPvUzKZlFPKpGa10tS4DaEK0r3Jk78BeV08IT9Ox6MV8i/T2X+i+kWauJlrtjw23jZNbd58uduG6EZNusEC4YNh00fba3dBLQjrL0STStF5nS7ISJyOT3wOvMbTJCqdHiXLORHCCccH4ao2MKvnQGl1YalgzbuyYq1jvtuF1ZbotjDeQO9hKfqTZiRM98R3N3Lg+9c0hN5aT4ihLvX+CXovb7NI7w/QG8QBvb8afhuOaNf3MYqVmURdiB6UDXFTZhfY8O7C5g5q7TutahtCvyxhoeWEd1IYCOTxyV0DyAxkwrAItcLSn+RTIJc7x2cV8rO4lCidRQCwLe/jq9xIi4p5hSXDHOozb/CdKw+7TxNfMoTTzDZ0ql/3V25ib4U8Vie+P4uFaY4W1sJtiKcyTfzmw/hiq8cpy3Jp/swNKg/Ijp+O8QzF4pRGTSTP26KQymN0Hrteq1hNCnnHG28Nozzsa97AZRBGRhKGLQaMh+1soTTcoS6bd4lV8mzSBve3usTbxLvY8sud6fIxfP69jhgOtsuLipDWJGRfFL/bHcZ96n7AVlin99o2UOYD9EZXXJTL+r+C3n+KPtwPi76+vqfY+wx9e1oZ3qKNYfceRhbU71EHZnJiYkVLfENh/KrBDOAPW1LTE+WZ6PJT/Qgj1gmWIcHanL6kPC2qjEqQ+csLrw454p4Oa6+n1GnmpJYWjJ7BSeXGf+QCAyUk6VpcDwaiC22iOAmlj75M8G3ovBaU+QbAdSrAtqgLBMcWxrQkYVRt+WM5Fbij+X3KBoXgq8NRumqWqTxyySNZ1/K1CJZWDNYttMpiV8Xw8FxGT9FLX0zkEVDuxy2e3u9/JRH7e1j64wNFsv8mrfkDvRHv6ON37ynzL6IfP0JvDXqH/wuyCJ0Sn/rLgm7uk4bw6vAAEbzo/VB0SopJYHZauN0EMB8CiOdT88JkHvNMV/3GTVGfc9ok5Cz10X2UOg5Gb6LzNkzkZUSwif7j0nAbv4GBmMLNFrNrnsuwSo5VbSSRwXqNoriMmY5ja7eSIDrSCgyBNCqIrgZhSfOKiqdb4mqB1bcNjDfB0mvT6rKVHcbf8h/cXt0mFoDe99FYQJvoDYIIdOjdx19Jf/8D/fY6TpUwuzptEX/dGr7wgjoWYgyLGLN/XadWK0JMtixcBOferInc73WAckKmCLe+RZ0uRvTGn4FTsuO8PMeDGwRKXTdo7QQs6eNpdNLpyKAJMct4EZNdeZyF4o21UuZH6Lw7E3leckkUcEE6I/M957CFoSNW5SDGtcH8LAFjNj4xSYtPxyj8MYxlbfi5lxMEBV0vrQ2j4kZ0uB6GXe1jmpuPji8A6om3K4wLhb9m4hMUgHuOzIY/jtEW0zGMRW041hEM+PV+n7kZdt5VtUpYC5w16B66Te0fGbMInVVJpvQQGLQbVXHqq7PQEfnn+AObzUwE5xyfMGFKOzClxwHOloxjbCMH3bJS8BOrciHlklhoSI1xpsYb/zhCgk+KKgJYBci0UJ4L0EsD7MYRcs9Fmg53jJcgL7qn1sm/oDjabtXdABz/JoRLdm70nfUtZS7dpzhHN83Kv1Ren85rCXJHDbzTtuMroivU7rTbEWHMBsUfKlYMTEAVi4rNU4lfXz4ZC/JFauOu5va2SRv/uLKm4gYBcQThRzH9ObEiYXdtfa7ru+/TpEHvn5smkh/IMELvN61X9O53pJnxo0SEsNMmbab0Rt1G4G1C+/hriFCvuPDnSiRZuFFrR4S5ghODW14vHJBD9sJfwVm7O9EcG6lTxAngWwvwOPbyNgJW7qAGMiHxK5lelY9Bs1noyFy8Ozuxc5sMzpBwBkQGVGm1PKOAp6ljC45Sp7OT/NQ6+cdzN3+drvtcQQqiggnM2SRIJy7YHakiVd0xqevdtED+Xcr7SRsCi7iOtOOshecRRqZYkqS56JBxC6XqpHkxKLgJZBgizUHykcXIFoFj+/CTAWS3VsiTe5M2XlvXXUIYFpbeWCaOmVUQYMfh6MYsDDo6H2e+PwmeRTfP6w3DwTCGg5dV8KJbdFX4KxjoldLDlIl8CNFQf+vqbYXcegi5pzdoM9H7m2Em8FZ797E8rqnIJb8m47cr+rEXG8a0Y8QsjEmA0wu2TcwStysQxZxOnN6ZajfTph3Ma1gg4/s8E23dZ+4NOrWDJ3gc2gvdt8dEIUHAyeSOPYncr5Jn2jx0CWdWPv7soTp5/Swr7uIx8sH57IL7qHeQF7SUpTZlAgNvThWhaAwPZsnvF8m/ibxbX8WP6FCa98HFRzDPYxDmwLLsi/z+5ITy+Ju6fOtqEQ4erv57Zud2URtu3y/BqkM2Lu8nG8hRRWEiH44mH4ywsp+ynmWV3v73gdX+8bsQvG4Kc9GLTdpIvX95i/hElgTcD6+Cs1D7J/C7TcNUWEOVi6v3g1Knj1gv6pYDRWDTeonnpD5bbCgemxN5imxmyOo56bdM1to/J4GfWO+ZtxNOkQ30VmRxCLO9Yx3UqwB2DPieDLdXZLHIPdgTnJQYPgTUqwC+zOWwV2TxZU7B89JavQrgec3XNqVVZLFNNA+8L/UqgAOf4DXqKbL4sud/Y+3VqwA2hupgGyqyONip3bJi6lUAWwb0+XWnyOL5zdlnkVi9CuCzwL5Xgyqy2KvpUMIoBPYXAUUW+zs3eyuZehXA3k7NkwqmyOJJ4T3MztWrAA5zXtdppchiHULqe4WAQkAioMhCLQSFgEJgIwT+D2t8OiNrEgxpAAAAAElFTkSuQmCC)The Rover successfully transitioned between consecutive waypoints and stopped upon completing the square navigation mission.

**Random Waypoint Navigation:**

The random waypoint navigation experiment was conducted on September 28, 2026. Five random waypoints were generated within a 3 m × 3 m coordinate area. The waypoint acceptance tolerance was reduced from 0.15 m to 0.05 m to evaluate navigation using a more restrictive positioning criterion.

The following waypoint coordinates were obtained from the ROS 2 terminal output displayed in the recorded experiment.

|   |   |   |   |
|---|---|---|---|
|**Waypoint**|**X-Coordinate (m)**|**Y-Coordinate (m)**|**Result**|
|1|0.17|1.34|Reached|
|2|2.86|1.13|Reached|
|3|1.97|2.98|Reached|
|4|0.10|1.59|Reached|
|5|1.13|3.00|Reached|

The Rover began near the origin of its odometry coordinate system and autonomously navigated through all five waypoints using wheel odometry feedback. The estimated position and traveled path of the Rover were displayed in real time using the waypoint visualization software.

Throughout the experiment, the waypoint controller generated linear and angular velocity commands based on the estimated position and orientation of the Rover relative to the current target waypoint.

The final position reported by the wheel odometry was approximately (1.1331 m, 2.9868 m).

Using the final waypoint coordinates displayed in the terminal, the estimated remaining distance to the final waypoint was calculated as:

![](data:image/png;base64,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)

![](data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAE4AAAAWCAMAAABZqVq1AAAAAXNSR0IArs4c6QAAAJNQTFRFAAAAAAAAAAA6AABmADo6ADpmADqQAGaQAGa2OgAAOgA6OgBmOjoAOjqQOmaQOma2OpC2OpDbZgAAZgA6ZjoAZjo6ZmYAZpC2ZpDbZrbbZrb/kDoAkGY6kJBmkNv/tmYAtmY6tpA6tpCQttv/tv//25A625Bm27Zm27aQ29u22/+22////7Zm/9uQ/9u2//+2///bnXAzeAAAAAF0Uk5TAEDm2GYAAAAJcEhZcwAADsQAAA7EAZUrDhsAAAAZdEVYdFNvZnR3YXJlAE1pY3Jvc29mdCBPZmZpY2V/7TVxAAABNUlEQVQ4T+2S3VaDMBCEE2pp419bbK00VVRKUSEk7/90TrIhyNGcXvTS5oIzsOTLzmQZu6wzEyj4/EzCaLuRuxiuFjzJfXHQZu82mFdUl65q6tvJwf+nH14iOCVyViVUHTS4FmfksmGVk3p9XwaEumsiuCJtmM4oiqCP268s2FEC0sg+rW7Nk2f/YiT3CxRnQa48Z6x1wH1ubBRVb1SJRaMzu+mvRRXbFvyMNHXXcj5FEkbO9lDIzjmIRncKB1KBZHWWlqyDYde1uvF38sssxdZ391OH7OyJdGo7YeoavRbxqbMkCpCoXg/ZORTdRJXi8nfm/XGuniLhtTxnNewosUJQpF2ONCg4x42RfXS2xTc+LWu+iA0KOwoXtsX1mqkN7v8KHXxA2CowmOdtFBLp9fL5vybwDV5cH8GXOVLPAAAAAElFTkSuQmCC)m

The estimated remaining distance of approximately 1.36 cm was within the specified acceptance tolerance of 5 cm. This calculated distance is approximate because the displayed target coordinates were rounded.

The ROS 2 terminal confirmed successful completion of the navigation mission by displaying the following messages:

WAYPOINT 5 REACHED!

MISSION COMPLETE - All waypoints reached!

The waypoint completion rate was calculated as:

![](data:image/png;base64,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)The Rover successfully reached all five waypoints without manual intervention during the recorded mission, resulting in a waypoint completion rate of 100% for this trial. The Rover automatically transitioned between consecutive waypoints and stopped upon completing the navigation mission.

A video recording approximately 64 seconds in length documents the physical movement of the Rover, real-time waypoint visualization, wheel odometry feedback, and successful mission completion.

[Random Waypoint Navigation Experiment Video](https://www.youtube.com/watch?v=j0UbDJDEi-8)

**Odometry Loss Detection:**

During a separate autonomous navigation experiment, the Rover Robotics driver was terminated to simulate a loss of odometry feedback. The waypoint controller was configured with an odometry timeout threshold of 1.0 second.

The waypoint controller detected the loss of feedback after approximately 1.04 seconds.

The difference between the configured timeout threshold and the observed detection time was calculated as:

![](data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAKIAAAAWCAMAAAB0d7dKAAAAAXNSR0IArs4c6QAAAJ9QTFRFAAAAAAAAAAA6AABmADpmADqQAGa2OgAAOgA6OgBmOjoAOjo6OjpmOjqQOmZmOmaQOma2OpC2OpDbZgAAZgA6ZgBmZjo6ZjqQZmY6ZmZmZpBmZpC2ZpDbZrb/kDoAkDo6kDpmkGYAkJDbkLb/kNv/tmYAtmY6tpBmtv//25A627Zm27aQ27a229uQ2//b2////7Zm/9uQ/9u2//+2///b3+Pq7AAAAAF0Uk5TAEDm2GYAAAAJcEhZcwAADsQAAA7EAZUrDhsAAAAZdEVYdFNvZnR3YXJlAE1pY3Jvc29mdCBPZmZpY2V/7TVxAAAByklEQVRIS+1UXVODMBAMtR9aaEWqtdUC1QISq0Ik//+3eXehlakQDH1wnCFP5C63u9k7wli/egd6B0wciEeZyfE/OCsmg4hoxVXaRK9JmSjuCsNvFh7yyKTRTU3KRGFXmOI2ok4XC8uyLshHGcAnLSW6kjJRdHq2M0w+ZuISOy0Dv0mAJmWiuSMMlslgjKNIQmuXJqWTeNqNjjBiBr3k2OD8OIqn0JWUiWk/zn4zGMHE+KuICfQ4RivrVyVFV2pZccPEHGFqQapB4RIFAalnADstA0++7GrJq6kzJCqY4i5lRhJ5+etCp7k12DTYo1KJbbtMOCt7lhWhbUcY2DAxDx/DCFlpC0Fn2uAiwsgnaxgByHSUQeVGYRCcCpYs4KIGqKmL8OuD82gA93I3+3yGgNymwklZ7jHuq606pZmEg4sJaIJKVYRwiJxEJYuLGT1QDQeHqeUeAkH1fulEfLhew+UhUNx/bFO1xVO/kljelooIDoFiv2RxgcpcYtXF/Q46+wogbxnNVfIA16Zt6+UPLiqJqojg3kniWS6WUzIPcRaXc9vHwLWSSC8ebSWM0ErXaJbALJIadTksIjgVVCxF6LYDtT0sff5fOfAF4/FCW/k7nIYAAAAASUVORK5CYII=)

![](data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAHQAAAAWCAMAAAAmeEhEAAAAAXNSR0IArs4c6QAAAH5QTFRFAAAAAAAAAAA6AABmADo6ADpmADqQAGa2OgAAOgA6OgBmOjpmOjqQOma2OpCQOpC2OpDbZgAAZgA6Zjo6ZpC2ZpDbZrbbZrb/kDoAkDo6kGY6kLb/kNv/tmYAtmY6tpBmtv//25A625Bm27Zm2/+22////7Zm/9uQ//+2///bTsx4lgAAAAF0Uk5TAEDm2GYAAAAJcEhZcwAADsQAAA7EAZUrDhsAAAAZdEVYdFNvZnR3YXJlAE1pY3Jvc29mdCBPZmZpY2V/7TVxAAABNUlEQVRIS+2TyULDIBBAIVaLdWvUxqhJLV1Y/v8HZQZCaALEg0e4BXizvCGElFUM/JeB/u6yGEp/fQx3ToxWjf2QzO9OI8SAK7LqbIjNIZXcXB/CS9YQbgnd+t0pGAMCkvCXeosh9smOj+/nekgKXlS9BmL/7HcnSaNAQKrXDv2qmlJ6g72aDtzyZaghvG6hQiTEk9+dG5oDAUnEmsh7sKXb5ICgJHeoUAsklQ+HvyT1wEhiKt2CLZs6seZJ9WcHpXAoJqcmllQ+GlEctArvMh8DCjSdCjeBhJ6gSgfYh4CD6cEWPv0eH8dSp0jZ8QTSMzMdAU/a3wT8mkD65zufVLItEbQhJ/eTLc70CvAkd5KMX06rXSKnfDPXVjtjxHR4ZPTWTh8efNxuHBjJjNNyVAwUAxMDv/VPI9F86cxiAAAAAElFTkSuQmCC)

![](data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAEMAAAAWCAMAAACsV9EFAAAAAXNSR0IArs4c6QAAAG9QTFRFAAAAAAAAAAA6AABmADpmADqQAGa2OgAAOgA6OgBmOjpmOjqQOma2OpCQOpC2OpDbZgAAZgA6ZpDbZrbbZrb/kDoAkDo6kLb/kNv/tmYAtmY6tpBmtv//25A625Bm27Zm2////7Zm/9uQ//+2///b9RUY2wAAAAF0Uk5TAEDm2GYAAAAJcEhZcwAADsQAAA7EAZUrDhsAAAAZdEVYdFNvZnR3YXJlAE1pY3Jvc29mdCBPZmZpY2V/7TVxAAAA80lEQVQ4T+2SyRKCMBBEM7jEXRRRQKJm+f9vdCYzUUqhPHmwyjl1heSlu4NS//lKA/Xk9ol71ZAVsqmjnT7wotNZxWLeDrCcLpRJu546lCAMs8m3dDY0g37Iqc+n8YaObtY5M/yuimF8DgCj6AT5MkwNJV3CkTvarrww7FS5GYUJpRh7C+SjUWY8tVu0wqCToSSbTOqbXkY4Vgg8GLzYLZFuKIN91PGahatIPkRbiYuMmmzGN6q5s76h41wEo5LmLPyeFAbXw/nUz7BQqCu+rdNblTTtZIYRRxjGQLYfMnLRMMa2iKFERwSkH2Qwwv/DLzdwB7DEFGnevezuAAAAAElFTkSuQmCC)s

The observed detection time exceeded the configured timeout threshold by approximately 40 milliseconds.

The controller suspended autonomous navigation and published a zero-velocity command. The Rover stopped moving during the interruption. However, because the Rover Robotics driver had been terminated, delivery of the zero-velocity command to the physical motor controller could not be independently verified.

After the Rover Robotics driver was restarted, autonomous navigation did not automatically resume. A new start command was required before autonomous navigation could continue.

**Table 2: Waypoint Navigation Experimental Results**

|   |   |   |   |
|---|---|---|---|
|**Test**|**Expected Result**|**Actual Result**|**Status**|
|Single Waypoint Navigation|Reach specified waypoint|1 of 1 waypoints completed|Pass|
|Single Waypoint Final Position|Remaining distance within 0.15 m|0.1048 m estimated remaining distance|Pass|
|Square Waypoint Navigation|Complete all four waypoints|4 of 4 waypoints completed|Pass|
|Square Path Final Position|Remaining distance within 0.15 m|0.1092 m estimated remaining distance|Pass|
|Square Waypoint Completion Rate|100%|100%|Pass|
|Random Waypoint Navigation|Complete all five waypoints|5 of 5 waypoints completed|Pass|
|Reduced Waypoint Tolerance|Waypoints accepted within 0.05 m|All five waypoints accepted by controller|Pass|
|Random Waypoint Final Position|Remaining distance within 0.05 m|Approximately 0.0136 m estimated remaining distance|Pass|
|Random Waypoint Completion Rate|100%|100%|Pass|
|Autonomous Movement|No manual intervention|No manual intervention required|Pass|
|Destination Stop|Stop after final waypoint|Rover stopped|Pass|
|Real-Time Path Visualization|Display estimated position and traveled path|Successfully displayed|Pass|
|Odometry Loss Detection|Detect loss after approximately 1.0 s|Detected after 1.04 s|Pass|
|Communication Recovery|Prevent automatic mission resumption|Manual start command required|Pass|

**Table 3: Comparison of Navigation Experiment Results**

|   |   |   |   |
|---|---|---|---|
|**Parameter**|**Single Waypoint**|**Square Path**|**Random Waypoints**|
|Total Waypoints|1|4|5|
|Waypoints Reached|1|4|5|
|Completion Rate|100%|100%|100%|
|Acceptance Tolerance (m)|0.15|0.15|0.05|
|Final Estimated Remaining Distance (m)|0.1048|0.1092|0.0136|
|Manual Intervention Required|No|No|No|
|Mission Completed|Yes|Yes|Yes|

**Interpretation and Conclusions**

The experimental results demonstrate that the drivetrain can utilize wheel odometry feedback to autonomously navigate through multiple specified waypoints without manual intervention.

The single-waypoint and square-path experiments demonstrated successful navigation within the initial acceptance tolerance of 0.15 m. The random waypoint experiment further demonstrated that the waypoint controller could complete a more complex navigation mission using a reduced acceptance tolerance of 0.05 m.

The reduction in waypoint acceptance tolerance was calculated as:

![](data:image/png;base64,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)

![](data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAANcAAAAWCAMAAABkF/mWAAAAAXNSR0IArs4c6QAAAKtQTFRFAAAAAAAAAAA6AABmADo6ADpmADqQAGaQAGa2OgAAOgA6OgBmOjoAOjqQOmaQOma2OpC2OpDbZgAAZgA6ZgBmZjoAZmY6ZpC2ZpDbZrbbZrb/kDoAkDo6kDpmkGY6kGZmkLbbkNu2kNv/tmYAtmY6tpA6tpBmttvbttv/tv//25A625Bm27Zm27aQ29u229vb29v/2/+22////7Zm/9uQ/9u2/9vb//+2///bD7pWIgAAAAF0Uk5TAEDm2GYAAAAJcEhZcwAADsQAAA7EAZUrDhsAAAAZdEVYdFNvZnR3YXJlAE1pY3Jvc29mdCBPZmZpY2V/7TVxAAAC1klEQVRYR+1W23baMBCUoVDcpOHSpm3iNkkxNGCgbsAU//+XdWa1km0wOSFJH3KO/YBX0mpXMztaY0zzNAw0DPw3BqJAnqEmyCNnnZQxZozWt/09Se9olNX4pAQnO/9Nf0eDNF2/EFf3Pk2nwfVe+npci4u1yaPjkA8x5Mvz9pzT3ihsoTTork12GfTps4EtvuUSPbdejHV41npciSZ+cg12lxczcfZGYec/sLQbXZv8+ziPQKz8FLgWYdC6UpD5JAg60Ek8mLTnHHAFo2kYgGqT4/3uDm9145rFBQ27Sb77056yFqM6ui2n8nsmpt41aRG6Fqrnq0xchcQE1cw+rk2CoJ5LOU7SnplVCNQc8JTL1tjErcEfDGYy4Gidheory+omuKBDkFDai4DLsIyLKSSQ1Iu4XNIitOXZXnkRFx+eumqUJ53iBFfPZGfqLFBYSEkog09IToO581+ojCwLwyBeBlj2bpzmMVhiN2kDwtsKu7StwOWT+tBHyvX+BsHnCKWGQClssyFwAkqGUOEmtF2QmbMPbE9Yl0EobKlWzOr2yzk7gsNlfRHIuTkdihp00jqBP4/LbStw+aSP49qNoJgt+ow3ELls2xRgiaC6D59nWxamBpce3NYr6tylD+V6eVzOzeEi/9XFZ+Ha1+FuxGODc28ICp3kEsWNVxj0IZcNrnJMsRzqUDQ0sTqUg9br0Lk5XLzKVluToWrsdXTIT4LcETXk0N4udxAcWnARdKVv4Cbg+mkT4fJudGW2N2UdyjIbgL/22g+lIfi97RkaIHO079MJRO23gXq05mrf0DtYe79Mglxb1scZ7F5+0pVLSoYDVOpVtNwsxJr2X4G9DIPOT34T3P1CK7F93rdph0sE4T4YuOvs8/jm4I+I9Hm3DULb7/OP4jL4rrS+sjeqIV3ZTzK2faRpZWd6v+pJeouz9tO1CA//0L1FNM2ZGwYaBhoGXoWBfxPscq724x1RAAAAAElFTkSuQmCC)

Despite the 66.67% reduction in the allowable estimated distance from each target waypoint, the Rover successfully completed all five randomly generated waypoints without manual intervention.

The random waypoint experiment resulted in a final estimated remaining distance of approximately 0.0136 m, which was below the specified acceptance tolerance of 0.05 m. All three navigation experiments achieved a waypoint completion rate of 100%.

The recorded random waypoint experiment provides visual evidence of the Rover's autonomous movement, real-time odometry feedback, waypoint transitions, and successful mission completion. The experiment also demonstrates that basic autonomous waypoint navigation can be achieved without assistance.

The odometry loss experiment demonstrated that the waypoint controller could detect an interruption in positioning feedback and suspend autonomous navigation. The observed detection time of 1.04 seconds was approximately 40 milliseconds greater than the configured timeout threshold. Requiring a manual start command following communication recovery prevents unintended automatic mission resumption.

However, the position measurements were obtained directly from wheel odometry rather than an independent positioning system. Therefore, the calculated remaining distances represent estimated positioning errors rather than independently verified physical positioning errors. The reduced estimated remaining distance observed during the random waypoint experiment does not independently establish an improvement in physical navigation accuracy.

The drivetrain satisfies the tested functional requirements for autonomous waypoint navigation, position feedback, and destination stopping. However, further integration and experimentation are necessary to verify autonomous navigation accuracy, repeatability, and reliability under the complete CIRCE system operating conditions.

**Overall Drivetrain Conclusion**

The drivetrain experimental analysis was conducted to evaluate the functionality and performance of the Rover Robotics 4WD Rover Zero platform selected for the CIRCE system. The completed experiments focused on motor control, ROS 2 integration, wheel odometry feedback, autonomous waypoint navigation, and the response to a loss of odometry feedback.

The motor control experiment demonstrated that the drivetrain successfully receives and executes linear and angular velocity commands through ROS 2. Forward movement, reverse movement, rotational movement, stopping, and keyboard teleoperation were successfully demonstrated. Wheel odometry feedback was also verified, providing the necessary position and orientation information for autonomous navigation.

The waypoint navigation experiment demonstrated that the Rover could autonomously navigate through single, square, and randomly generated waypoint configurations without manual intervention. All three navigation experiments achieved a waypoint completion rate of 100%. The random waypoint experiment demonstrated successful navigation through five randomly generated waypoints using an acceptance tolerance of 0.05 m, with a final estimated remaining distance of approximately 0.0136 m.

Additionally, the odometry loss detection experiment demonstrated that the waypoint controller could detect a loss of positioning feedback after approximately 1.04 seconds, suspend autonomous navigation, and prevent automatic mission resumption following communication recovery.

**Summary of Drivetrain Design Requirement Verification**

Although the completed experiments demonstrated successful drivetrain functionality, several limitations remain. Testing was conducted indoors on a tile surface without additional payload. Furthermore, positioning measurements were obtained using wheel odometry rather than an independent positioning system. Consequently, the actual physical positioning accuracy and long-term reliability of the drivetrain have not yet been established.

Additional experimentation is necessary to evaluate payload capacity, outdoor terrain traversal, operational endurance, positioning accuracy, and integration with the remaining CIRCE subsystems.

Based on the currently available experimental results, the Rover Zero platform remains a suitable candidate for the CIRCE drivetrain subsystem. The completed experiments demonstrate that the drivetrain satisfies several of the original functional design requirements. However, complete verification of the drivetrain subsystem cannot be established until the remaining experimental requirements have been evaluated.


 
