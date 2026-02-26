# Attendees
- [x] Gerardo Ramirez
- [x] Nick Romsdal
- [x] Sharif Zahra
- [x] Brady Harkleroad
- [x] Daniel Davis
- [x] Summer Morris
## Additional Attendees
DEVCOM
- Griffin Chase
- Joze Lindquist
- Elias Brady
- Dedra Moore
TTU
- Dr. Storm

Any DEVCOM communication should include all DEVCOM advisors. Someone will inevitably answer.  

# Q1: Should we expect a human at the final waypoint to unload the spool or should the rover autonomously remove the spool and return to C2?
A1: No specifications from client. Open ended from specifications, allowing team to create solution with justification.

# Q2: Along with Q1, does the cable laying system need to be located inside the robot or externally implemented?
A2: Open ended, team preference. "Dealers Choice". Main thing with the project is working without GPS. 

# Q3: Is the robot laying 100 yards or 200 yards of ethernet?
A3:  Robot is intended to run 100 meters of cable.  Ethernet is chosen over fiber optic for cost saving.[^2]

# Q4: What Ethernet is the robot laying? Cat5 or Cat6?
A4: Whichever is cheaper. Cable without shielding may be easier to unspool.· CirceBot shall operate in Tennessee environments -> Minimal mud conditions, fields, etc. Not expected to traverse rock scrambles over 3 days.  Minimum runtime is 20 minutes.

# Q5: How will waypoints be provided? Preplanned or real time?
A5: Both software packages are suppose to talk to protospec files. Question is whether we will have a CS counterpart.  Previous CS team turned in their own side of the project.  CirceBot will ping CirceSoft for next waypoint.  Unsure on what the CS team implemented for waypoint planning.  
Purpose is to give the robot waypoints to travel to via CirceBot2CirceSoft. protospec. Therefore it is not necessary to integrate CirceBot and CirceSoft if necessary.[^1] 


# Q8: Are there any restrictions on components we can use?
A8: No real component restrictions from customer, be weary of budget.
A8 (Dr. Storm): Try to reuse already purchased components whenever possible. There is a set budget for  EE Capstone projects.  The ME capstone project will have their own budget for their subsystems.

# Q10: Tension sensing for spool?
A10: Ethernet must be deployed in "best practice for deploying ethernet cable." Biggest roadblock will be getting rover from one point to the other. Get it across the field first then worry about adding things to the rover.  

# Q11: Any documents from DEVCOM you can provide?
A11: DEVCOM may be able to send document to team

# Q12: Is there a desired velocity for the robot or weight limitations?
A12:  Dealer's choice, faster the rover, more inaccuracy in the modelling. 

# Q13: Mechanical Engineering (ME) Team. Will they have their own customer or should they have their own customer?
A13: Talk to the ME team frequently. We can work to combine them.  May be good to bring ME team onto EE DEVCOM call.  Both past ME/EE cycles done with this have seen breakdown in communication, look for near daily communication between ME and EE team. 

# Q14: Weight or dimension limitation?  
A14: Dealer's choice. Payload has a maximum weight of 20lbs.  

Contacting with DEVCOM team can be done if answer need to be made quickly outside of biweekly meeting.



# DEVCOM Questions for team
## Q1: Have we divided roles? 
A1: Nick is Secretary, Summer and Nick are ME team liaisons.  

Rebuttal: Quad charts?  Each person should create a quad chart to have updates for biweekly meetings.  Quick few sentences covering past issues, planned goals for future to provide update.

If questions come up in between meetings, shoot email to DEVCOM contacts.  Someone will answer eventually.

# Do Outs:
Look at quad charts and individual roles.

Elias from DEVCOM will look into sending quad chart and sending textbook resources from last group.

"Old Man Advice": Communicate, Communicate, Communicate.  Communication will likely be first link to break.  Own your sub system. Ask questions if you get stuck.  Those in Mechatronics: look into mechatronics club for potential resources; a lot of good resources for starting points.  

[^1]: Should we create our own GUI for testing CirceBot? Could use a Python file that shoots data on WebSocket to CirceBot via protospec files.  
[^2]: 100 meters is approximately 109 yards which is near the maximum length Ethernet can reliably run without signal degredation.