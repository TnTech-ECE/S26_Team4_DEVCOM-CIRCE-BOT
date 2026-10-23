#  Drivetrain Subsystem: Detailed Design

---

## Function of the Drivetrain Subsystem

  The drivetrain subsystem is responsible for moving CirceBot. Its main function is to take navigation data from the navigation subsystem and convert it into motor outputs that control the robot’s speed and direction. This allows the robot to travel to a specified location.
  
  The drivetrain also serves as the base platform for the rest of the system, supporting all other subsystems and payload during operation.

## Specifications and Constraints

---
- Specifications
  - CirceBot shall operate in Tennessee environments
  - CirceBot shall be able to carry 100 yards of Ethernet cable. (approximately 10 lbs.)
  - CirceBot shall carry the weight of both the payload as well as the other subsystems.
  - CirceBot shall move once receiving a new specified location that is not its current location
  - CirceBot shall stop once specified destination has been reached.
  - CirceBot shall navigate around detected obstacles.
  - CirceBot shall transmit current position, current velocity, meters of cable left, heading, battery life as a percentage, and an error code if one occurs.
## Overview of Proposed Solutions

---
The following device will be used as the basis for the drivetrain of the CirceBot:
- 4WD Rover Zero 3 [1]:
  - The platform has a height of 25.4 cm (10"), a width of 39 cm (15.4"), a length of 62 cm (24.4"), and a ground clearance of 7.87 cm (3.1").
  - The platform weighs 11 kg.
  - The platform has a maximum speed of 9.01 km/hr (5.6 mph).
  - The platform is driven by two 2050 KV inrunner Brush-less Motors with a max power output of 115 W/motor.
  - The platform is capable carring a max weight of 50 kg.
  - The platform is capable of climbing hills, traversing sidewalks, and traversing gravel if the ground is dry.
  - The platform has a drive time of 1 to 2 hours and an idle time of 5 hours.
  - The platform costs $4,000 before tax.
  - The microcontroller will communicate to the motors the speed and torque needed to follow the designated path.
## Interface with Other Subsystems

---
- Power Subsystem:
The drivetrain is powered by an LiPo battery (98Wh), which provides the required voltage and current for motor operation.  

- Navigation Subsystem:
The drivetrain receives waypoint and movement data from CirceSoft and executes the required motion to follow the planned path. 


## 3D Model of Custom Mechanical Components

Full Rover:

![Full_rover](https://github.com/TnTech-ECE/S26_Team4_DEVCOM-CIRCE-BOT/blob/main/Reports/Detailed%20Design/Drivetrain/Robot.png) [1]
![Rover_Schematic](https://github.com/TnTech-ECE/S26_Team4_DEVCOM-CIRCE-BOT/blob/main/Reports/Detailed%20Design/Drivetrain/Robot%202.png) [1]
Motor:

![Motor](https://github.com/TnTech-ECE/S26_Team4_DEVCOM-CIRCE-BOT/blob/main/Reports/Detailed%20Design/Drivetrain/Motor.png) [2]
![Motor_Schematic](https://github.com/TnTech-ECE/S26_Team4_DEVCOM-CIRCE-BOT/blob/main/Reports/Detailed%20Design/Drivetrain/Motor%202.png) [2]
Anti-spark Switch:

![Anti-spark Switch](https://github.com/TnTech-ECE/S26_Team4_DEVCOM-CIRCE-BOT/blob/main/Reports/Detailed%20Design/Drivetrain/Anti-spark%20Switch.png)[3]
Motor Controller:

![Motor Controller](https://github.com/TnTech-ECE/S26_Team4_DEVCOM-CIRCE-BOT/blob/main/Reports/Detailed%20Design/Drivetrain/Motor%20Controller.png)[4]

## Electrical Schemattic

---
![Electrical_schematic](https://github.com/TnTech-ECE/S26_Team4_DEVCOM-CIRCE-BOT/blob/main/Reports/Detailed%20Design/Drivetrain/Robot%20Diagram.png)

![Motor_Connections](https://github.com/TnTech-ECE/S26_Team4_DEVCOM-CIRCE-BOT/blob/main/Reports/Detailed%20Design/Drivetrain/Motor%20Controller%20to%20Motor.png)

How to connect motor[2]:
   1) Three power wires need to be connected to the motor, and they often differ in colors: Phase wire A is Blue, Phase wire B is
Yellow and Phase wire C is Orange. Please note the ESC mark while connecting ESC output wires to motor power wires and
ensure connections are: A-A, B-B and C-C.
   2) If you are using a sensored ESC, please insure Hall-sensor wires are clean and undamaged; then connect them in the correct
direction to the sensor ports of the motor & the ESC respectively. Warning: In such a case, the wire sequence of the ESC and
the motor must strictly follow the rules of A-A, B-B and C-C. Do not change the wires sequence.
   3) While if the ESC is a sensorless one, then connect the motor and the ESC according to the above way may cause the motor
to rotate in the opposite direction, because definitions of phase (#A / #B / #C) are different among manufacturers, at this time
you only need to swap any of two wire connections.


## Flow Diagram

![Flow_Chart](https://github.com/TnTech-ECE/S26_Team4_DEVCOM-CIRCE-BOT/blob/main/Reports/Detailed%20Design/Drivetrain/FlOW.png)

## Design Analysis

---

The drivetrain design meets the required specifications for mobility, payload capacity, and operation time. The Rover Zero 3 platform provides enough power and stability to carry the required cable and subsystems while operating in outdoor environments.

The use of brushless motors improves efficiency and reliability, which is important for continuous operation. The system is capable of responding to navigation commands, avoiding obstacles, and stopping at the correct location.

Overall, the drivetrain provides a practical and reliable solution that supports the goals of the CIRCE system without unnecessary complexity.


### References

[1] Rover Robotics, Inc. “Rover Zero UGV - Mobile Research Robot From Rover Robotics.” Rover Robotics, Inc., roverrobotics.com/products/4wd-rover-zero-unmanned-ground-vehicle.

[2] America, Hobbywing North. “JUSTOCK 3650SD G2.1 Brushless Motor.” HOBBYWING North America, www.hobbywingdirect.com/products/justock-3650sd-g2-1-brushless-motor?srsltid=AfmBOoq0r92gtzdq2ndY9y4VhTjJVl2GHVZ_qYih_iiaVfPHbzuYj1f3&variant=14796323258483.

[3] AliExpress, “Anti Spark Switch Pro V3.0 60V 600A for Electric Skateboard,” [Online]. Available: https://www.aliexpress.us/item/3256808290308473.html. Accessed: Apr. 24, 2026.

[4] Flipsky, “Dual FSESC4.20 Electronic Speed Controller (VESC-Based),” [Online]. Available: https://flipsky.net. Accessed: Apr. 24, 2026.