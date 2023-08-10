# About
This is the start of my exploration with PROS. When I first started this project, I intended on using the integrated PROS library, Okapilib. But since of my lack of an inertial sensor, I ditched it and ended up writing most of it myself from scratch. The only thing I really used Okapi for was to get tank drive for opcontrol. The main branch features a drive positional PD w/ slew & custom exit conditions, as well as an autonomous selector GUI ([credit to this repo](https://github.com/kunwarsahni01/Vex-Autonomous-Selector)). The inertial branch features a heading correction PID and an angular PD. Although not much, I think this is a good improvement from the first drive PD I wrote in March, as the code itself is alot cleaner and provides a good template on how we should write our code once the season actually starts. Overall, I think the switch from VEXCODE V5 PRO to PROS was a good decision. 

---
# To-dos
- ~Create another branch for if we have an inertial sensor~
    - ~Angular PD~
    - ~Heading correction~
- Create an overheating warning (?)
- Create PID with OOP (?)