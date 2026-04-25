# Hardwired Communication
##### Subsystem designed by: Summer Morris

## Subsystem Integration with Project:
  The hardwired communication subsystem serves as the main mode of transportation for important data between CirceBot and CirceSoft via Ethernet cable. The subsystem uses a WebSocket protocol to support bidirectional communication for real time data transmission. The hardwired communication subsystem is crucial for relaying telemetry data such as GPS coordinates, velocity, position, cable length, battery status, and error codes. Effective communication is needed for this project to ensure the robotic tasks can be performed semi-autonomously and with exceptional precision. Due to the nature of the environment being radio frequency contested, continuous communication through the Ethernet cable is necessary to ensure the robot can properly execute the tasks it is given. 

## Specifications:
### Communication:
* CirceBot shall communicate with CirceSoft using a standard WebSocket Protocol (RFC 6455) that is implemented over a Transmission Control Protocol (TCP) [1]
* CirceBot shall send and receive commands at a minimum of 10 Hz
### Data formatting:
* CirceSoft shall send commands in the format supplied in CirceSoft2CirceBot.proto spec to CirceBot
* CirceBot shall transmit telemetry data in the format supplied in the CirceBot2CirceSoft.proto spec to CirceSoft
### Telemetry data: 
* The telemetry data shall include the remaining cable length, last known GPS coordinates, current velocity, current position, commanded velocity, commanded position, battery status, and error codes
### Ethernet cable:
* CirceBot shall communicate with CirceSoft using the Ethernet cable being deployed by CirceBot
* CirceBot shall carry 100 meters of Ethernet cable
### Environment:
* CirceBot shall operate in tactically contested (GPS denied) environments which require the use of a hardwired Ethernet connection between CirceBot and CirceSoft
### Operational requirements:
* CirceBot shall operate for a minimum of 20 minutes on an independent power source
* CirceBot shall be able to handle and deploy an Ethernet cable during operation by maintaining proper tension and avoiding unnecessary damage due to excessive bending or tangling (specifications for the M.E. team design)
## Constraints:
### IEEE 802.3 Ethernet Standards [2]
* IEEE 802.3 are standards for communication over Ethernet
* They standardize the physical layer including signal encoding, cable types, and transmission speeds
* They standardize the data link layer including aspects such as framing, addressing, and error detection
* These standards specify the allowable data rates, signaling methods, and electrical characteristics used to determine the selection of Ethernet Cable for CirceBot
  - Cat5e Ethernet cable was chosen for CirceBot
* These standards will ensure the bandwidth requirements are met for CirceBot
* CAT5e and CAT6 Ethernet cables have speeds up to 1 Gbps at cable lengths up to 100 meters
* These standards will define the hardware and communication protocols to maintain a steady data flow rate in the frequency-contested environment
* These standards will ensure the safety and reliability of the Ethernet cable
### ANSI/TIA 568 Standard [3]
* ANSI/TIA 568 is the standard for wiring schemes regarding twisted-pair cables for consistent connectivity
  - These standards will be used to ensure the connectors are properly attached to the Ethernet cable
* These standards set specifications for the performance of copper cabling categories
* They specify the maximum length of twisted-pair Ethernet cables to be 100 meters, which is the specification for CirceBot
### RFC 6455 WebSocket Protocol [1]
* RFC 6455 defines the specifications for 2-way communication over WebSocket protocol using a TCP connection
  - RFC 6455 will be used to ensure CirceBot can communicate quickly with CirceSoft
* It defines specifications for establishing and maintaining connections over WebSocket protocol
### Stakeholder Constraints: 
* This subsystem shall operate in a radio frequency denied and GPS denied environment
* This subsystem shall operate in a Tennessee environment
* This subsystem shall transmit and receive data at a 10 Hz minimum

## Synopsis of Suggested Solution:
  Communication to and from CirceSoft will be conducted through Cat5e Ethernet cable over WebSocket Protocols. The Cat5e Ethernet cable will ensure the 10 Hz frequency minimum is met while communicating between CirceBot and CirceSoft. The Raspberry Pi 5 connected to CirceBot will receive commands and data from CirceSoft. The Raspberry Pi will then communicate with other subsystems to provide the necessary data to semi-autonomously navigate through the frequency contested environment. The Raspberry Pi helps execute the navigation logic, via sensor fusion, as well as provide adequate motor control [4]. A second microcontroller will be connected to the Raspberry Pi and will likely be a Teensy 4.1. The second microcontroller will be used to communicate with parts such as the Battery Management System (BMS). The navigation will be accomplished with devices such as LiDAR (Light Detection and Ranging) and a depth camera. The commands sent to the motors from the microcontrollers will be accomplished over a USB (Universal Serial Bus) connection. While CirceSoft is sending commands and data to CirceBot, the Raspberry Pi on CirceBot will simultaneously gather and send telemetry data from each subsection back to CirceSoft using the same Ethernet cable being laid. CirceSoft and CirceBot will use the CirceSoft2CirceBot.proto and CirceBot2CirceSoft.proto files respectively when transmitting the telemetry data and commands over the Ethernet cable. The M.E. team plans to design a cable laying system that will alleviate excess tension on the Ethernet cable due to navigation or potential obstacles, which will ensure the connection over the cable is not heavily affected by CirceBot’s movement. The M.E. team’s design includes a contraption that will reduce tension on the cable by staking the Ethernet cable to the ground as CirceBot moves. A motor will be used to feed the cable from CirceBot, as CirceBot moves, to further relieve tension. 

## Interfaces to Other Subsystems:
  The hardwired communication subsystem establishes the main communication between CirceBot and CirceSoft. The subsystem shall exchange telemetry data and commands through an Ethernet connection using a WebSocket Protocol for bidirectional communication. Reliance on the Ethernet cable as the main communication method will enable operation in frequency contested environments, as is required for the project’s constraints set by the stakeholders. 

### Inputs to CirceBot:
Waypoints and Commands sent by CirceSoft:
* The format for data transmission is defined in CirceSoft2CirceBot.proto spec
* Data is sent to the Raspberry Pi over Cat5e Ethernet cable using a WebSocket Protocol
* Data is received by the Raspberry Pi connected to CirceBot
* The Raspberry Pi will transmit the data given by CirceSoft to every subsection as needed (over USB)
* Telemetry data received includes the position, velocity, cable length, battery life, and error codes
* Data is received at a minimum frequency of 10 Hz

### Outputs from CirceBot:
Telemetry data sent to CirceSoft
* The format for data transmission is defined in CirceBot2CirceSoft.proto spec
* Data is sent from the Raspberry Pi over Cat5e Ethernet cable using a WebSocket Protocol
* Data is received by CirceSoft on a computer
* The telemetry data received includes the position, velocity, cable length, battery life, and error codes
* Data is sent at a minimum frequency of 10 Hz

### CirceSoft:
CirceSoft consists of 4 main parts that are interconnected with one another
* The backend controls all API endpoints and WebSocket data
* The frontend consists of the user interface (UI)
  - The user shall interact with CirceSoft via the user interface (UI) to trigger commands manually
  - Commands triggered by the user through CirceSoft are transmitted to the Raspberry Pi connected to CirceBot over Ethernet cable using a WebSocket Protocol
  - The UI will be used to set the waypoints and obstacles that CirceBot will need to navigate
* The A* path-finding algorithm finds the shortest path based on the waypoints and given obstacles
* The simulation simulates a robot’s progress along a path and sends the data via a WebSocket
  - The simulation was designed to simulate CirceBot and is not required to run CirceBot [5]


## Operational Flowchart:
 ![Flowchart](https://github.com/TnTech-ECE/S26_Team4_DEVCOM-CIRCE-BOT/blob/main/Reports/Detailed%20Design/Hardwired%20Communication/HardwiredCommunicationFlowChart.png)
 
## Hardwired Communication Diagram:
![Diagram](https://github.com/TnTech-ECE/S26_Team4_DEVCOM-CIRCE-BOT/blob/main/Reports/Detailed%20Design/Hardwired%20Communication/HardwiredCommunicationDiagram.png)
  
  This is a simplified diagram of the hardwired communication subsystem. The Cat5e Ethernet cable is connected to both CirceSoft on a computer and CirceBot through the Raspberry Pi. The Ethernet cable will allow bidirectional data transmission between CirceBot and CirceSoft through WebSocket Protocols. The Ethernet cable goes through the cable laying system, which will be designed by the M.E. team, before connecting to the Raspberry Pi. The Raspberry Pi is connected to the other subsystems, such as navigation, power, the Robot Operating System (ROS), and the drive train. The Raspberry Pi will take the data or commands given from CirceSoft and give them to their respective subsystems while simultaneously collecting and sending telemetry data from the subsystems back to CirceSoft. 

## Comprehensive Bill of Materials:

| Name | Purpose | Manufacturer | Part # | Distributor| Distributer Part # |Qty| Cost | URL |
|------|---------|--------------|--------|------------|--------------------|---|------|-----|
|Ethernet to USB adapter|Used to connect the microcontroller to the computer |Amazon |U3-GE-1P |Amazon |B00M77HMU0 |1|$12|https://www.amazon.com/AmazonBasics-1000-Gigabit-Ethernet-Adapter/dp/B00M77HMU0/ref=sr_1_4?crid=C9QFSSJ7YZLH&dib=eyJ2IjoiMSJ9.K27jUa-yk0kiU3qmPPA4Xt8WZrKKatqPaLOKXzWoML6fvYhE_5lnnJh-zjydaBPc0eEbrvh0yfvrX19ws-juUVE1QqrzmRl_mCcGGliLJaeqlSdalcDnPTQqJF5cZJYbIGmgwtJuC5x7y7iGms1664pbUvTQ0VNdbcXHCjaQIAEIalrhIhWf_zXNY0_PS_KO7VZ8hqPtbdalsaHbmPgvC6Q5m_l8hQysLUvDJO6FvXk.s4hHGSSWJQyVa7IjINbcVP8ig6U9hxOp-iTZMAXhC4A&dib_tag=se&keywords=ethernet%2Bto%2Busb%2Badapter&qid=1774156703&sprefix=Ethernet%2Bto%2BUSB%2Ba%2Caps%2C150&sr=8-4&th=1| 
|Rotatable Ethernet Extender (already have) |Just in case- minimizes tension on Ethernet connections due to motion |JUXIN|731433618240|Amazon|B0BNMJT39G|1|$10 | https://www.amazon.com/Ethernet-Extenders-Extension-Connector-RightAngle/dp/B0BNMJT39G/ref=sr_1_4?crid=15Q9TO7BG8553&dib=eyJ2IjoiMSJ9.T-xCiC-cPk7tvbMJlWBOeEM4BULx7LMtQ3r1V6O7at80T2NowN_f5sfurzVoL8KjeLOXsDEen-KQcM5afnqZPTJ7XfgcY0M6db4GdIZSWjwYkHLXBjT_N7OYxmGyVaaThDbOOWPd4dP5Rs4NC6Ml9QzFsrPf6LBv69UiNrtp6yYy7-09DK_yjoDzOtTFpI54BNJkc9EwWlFbbceZoImOSji8IWYitWMGCOBfdTZar7I.tHtyDHHorpOzek3Yjx2f4RTmx8Yz4LQLpp2LkCTgIaY&dib_tag=se&keywords=ethernet%2BRJ45%2Bswivel&qid=1774157843&sprefix=ethernet%2Brj45%2Bswivel%2Caps%2C137&sr=8-4&th=1|
|Crimping Kit|Used to add connectors to the Ethernet cable|WEETOTUNG|YS-52QY-IR7P|Amazon|B0B9KH4RSY|1|$20 (estimated max $28)|https://www.amazon.com/WEETOTUNG-Crimper-including-Connecter-product/dp/B0B9KH4RSY/ref=sr_1_1_sspa?dib=eyJ2IjoiMSJ9.vpKLuOzv38MUDHQAVjRFBzKHcW5WSPG8GykqVjK3TQcc5-MjaSit2opJNu_NJ_OrBMrPRIRxfOm1hqRcMwhy3kL7zFWCi9Ct6elAfOgpQQj9ST1gjuTbZxyFi_i6v2nonMUzMR5ZLLxnt0ORKxbf8YHl5quW8fV5mCQ0X2g7s9ucANJeUv4AXIMNo30Y8Pbe62ZXooAOY2BwmrujKKGLFca7mEA26dxstFAWBBnNN55Km2zYEsoq0gWT0deQpWeo4QdXqtBNRff4t0iN4c70n114vVXYk6p4YuQTHHnKgnQ.E3uhpBIckjfMhC6DdQaJoyqhMmxPDPDC3ir4OgVZCZw&dib_tag=se&keywords=RJ45%2Bcrimping%2Btool&qid=1774719026&sr=8-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&th=1|
|Ethernet Cable|For communication|Cmple|1016-Amazon|Amazon|B00AM15FIE|1|$110 (price fluctuates so estimate of $120)| https://www.amazon.com/dp/B00AM15FIE?niid=nl_cl_lst_a_2_1&ref_=nl_cl_lst_a_2_1&nrid=TB0781X0071ESNBP7THH&th=1|
|||||||Total Cost:|$152 estimate $170||

  The total cost is estimated to be $170 to account for any fluctuations in pricing that may occur. The cost of each item is what was listed at the time of making the bill of materials and is subject to change. The rotatable Ethernet extender is listed because the part is already bought (by the previous group) and may be used in extenuating circumstances. In the event that this part is needed, testing will be done on the rotatable Ethernet extender to ensure the minimum transmission speeds are met. The rotatable Ethernet extender, if used, will help ease the tension on the Ethernet port due to the cable laying system and movement by CirceBot. All materials were found on Amazon to allow for easier ordering. The 1000 foot CAT5e Ethernet cable was chosen in order to make a few 100 meter Ethernet cables. Several Ethernet cables are required to test CirceBot’s ability to replace the Ethernet cable in under 2 minutes, and as a backup in the event an Ethernet cable breaks. The crimping kit will be used to safely add the connectors to the Ethernet cable to ensure a proper and secure connection. The Ethernet to USB adaptor will be used to connect the Ethernet cable to a laptop/computer. 

## Analysis of Crucial Design Decisions:
The hardwired communication subsystem is designed to meet all constraints and accomplish its intended function. This subsystem is designed such that bidirectional communication is enabled between CirceBot and CirceSoft in frequency-contested and GPS-denied Tennessee environments.  The subsystem shall send and receive commands at 10 Hz and have a run time of at least 20 minutes. 

### Standards:
The standards regarding the hardwired communication subsystem were chosen to ensure all constraints are met for CirceBot. IEEE 802.3 Ethernet standards ensure the communication over Ethernet cable follows industry standards. These standards will allow for compatibility and reliability regarding different Ethernet equipment. IEEE 802.3 standards define the data rates for CAT5e Ethernet cables of up to 1 Gbps for cable lengths up to 100 meters. Those data rates will ensure the CAT5e ethernet cable can transmit and receive telemetry data of at least 10 Hz. Following the IEEE 802.3 standards shall also help meet the need for quick reloading of Ethernet cable, while the robot is deployed in warzones, due to standardization [2]. ANSI/TIA 568 standards will ensure that any wiring done on the Ethernet cable follows standards set by the industry. This will ensure the connectivity of the Ethernet cable for twisted-pair cabling types is consistent, even when CirceBot is reloaded for reuse [3]. The RFC 6455 WebSocket Protocol will standardize the bidirectional communication accomplished with the Ethernet cable between CirceBot and CirceSoft. Using this protocol will guarantee that the connection to CirceBot is able to be established in a timely manner and is able to be maintained for long periods of time [1]. 

### Communication in frequency contested environments:
  The use of Ethernet cable shall ensure that communication can be established between CirceSoft and CirceBot in frequency contested environments. Communication through the Ethernet cable will give CirceBot a greater chance of succeeding at its goal of installing Ethernet cable, in an active war zone, due to the lack of reliance on radio frequencies. Ethernet cable will give CirceBot a more secure connection with CirceSoft which will allow for the fast transmission of important data. 

### GPS switch:
  A switch will be used to ensure CirceBot can semi-autonomously navigate with and without the help of GPS. The switch will turn off the GPS module connected to the Raspberry Pi. The switch is necessary to simulate a real-world scenario in which GPS may or may not be available. 

### Environmental:
  The Tennessee environment is best characterized as a humid subtropical climate, with varying degrees of humidity and temperature. Tennessee is home to a diverse landscape ranging from forests, mountains, or cities. The M.E. team plans to design a cable laying device to alleviate tension in the cable and guarantee the connectivity is able to remain established during important missions. The use of CAT5e Ethernet cable shall provide sufficient flexibility, due to the lack of shielding, to allow for more movement when avoiding obstacles without the concern of losing the connection [6].

### Data transmission:
  The hardwired communication subsystem is designed such that the telemetry data and commands will be transmitted at a minimum of 10 Hz when communicating through the Ethernet cable. Cat5e cable was selected for its cost-effectiveness and adequate data throughput capabilities. The suggested Cat5e cable chosen in the bill of materials supports data rates up to 1 Gbps, as well as a 350 MHz bandwidth, which is well within the requirements for CirceBot. Given that CirceBot requires a minimum telemetry stream of 10 Hz, the expected telemetry data throughput is estimated to require less than 100 kbps, which is well within the 1 Gbps rating. Choosing a cable that supports large data rates will ensure information and commands are passed on in a timely manner, which is beneficial when obstacles need to be quickly avoided. 

### WebSocket Protocol:
  The WebSocket protocol shall be used to establish and maintain bidirectional communication over Ethernet cable. This will ensure all telemetry data can be sent from CirceBot to CirceSoft, as well as any commands from CirceSoft to CirceBot using the .proto spec formats. The WebSocket Protocol that is implemented over a Transmission Control Protocol (TCP) will ensure the users have access to the latest data and can issue commands in a timely manner [1]. Although TCP is not traditionally chosen for real-time communication systems, due to the added latency, the low data rate requirements for CirceBot make TCP an acceptable option to use [7]. 

### Error codes:
  The hardwired communication subsystem shall send error codes alongside the telemetry data to and from CirceSoft in the event of a fault. Example errors that could potentially be used include a cable dispense error, CirceBot is stuck error, or CirceBot is overheating error. These examples demonstrate the types of faults CirceBot may need to detect.  CirceSoft will record the data given by CirceBot, such as the position, in JSON formatted files that can be used for diagnosing issues. The use of error codes and adequate data collection will minimize the need for intervention by humans in the warzone, as well as establish a well-organized record of important data [5]. 

## References:

[1]	I. Fette and A. Melnikov, “The WebSocket Protocol,” RFC, Dec. 2011, doi: https://doi.org/10.17487/rfc6455.

[2]	“Standard for Ethernet,” IEEE Standards Association, 2018. https://standards.ieee.org/ieee/802.3/12400/.

[3]	D. Schultz, “T568a vs T568b, Which to Use,” trueCABLE, Dec. 29, 2022. https://www.truecable.com/blogs/cable-academy/t568a-vs-t568b 

[4]	Raspberry Pi, “Raspberry Pi Documentation,” www.raspberrypi.com. https://www.raspberrypi.com/documentation/ 

[5]	ttu-bburchfield, “CirceSoft,” GitHub, 2025. https://github.com/ttu-bburchfield/CirceSoft (accessed Apr. 19, 2026).

[6]	Cable Matters, “Cat5e vs. Cat6 | Cable Matters Blog,” Cablematters.com, 2020. https://www.cablematters.com/blog/Networking/cat5e-vs-cat6 

[7]	amar.thodupunoori, “How Do TCP and UDP Differ in Real-Time Application Use Cases?,” DevOps Training Institute, Jul. 30, 2025. https://www.devopstraininginstitute.com/blog/how-do-tcp-and-udp-differ-in-real-time-application-use-cases (accessed Apr. 24, 2026). 
