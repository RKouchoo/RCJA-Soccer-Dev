# RCJA Soccer Dev

@Team Circut Breaker Dev

  First created: 3 / 7 / 2015
  Latest update: 25 / 8 / 2016

  Interesting Link: http://dlnmh9ip6v2uc.cloudfront.net/ - I love this link

  details about the compass sensor can be found on this data sheet http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/Magneto/HMC5883L-FDS.pdf

  ===============
  CURRENT STATE:
  ===============
    
    +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    - MAKE THE ROBOT FUCKING WORK YOU ASS YOU HAVE 1 DAY !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  WORKING ON CURRENTLY: Hard wiring the new light sensors and lasers to robot. Also working on a new check for system, with switches.

  ==============

  KICKER NOTICE:
  currently have not implemented methods for use of a soleniod+transformer/boost converter as i cannot mount it to either of the robots & and dangerous as a high current might get us disqualified (5 AMPS+ OUTPUT)
  
=====================================================  
TODAYS AGENDA:
To do list in day order: Current agenda for today
Big Robot:
Now:
-	MOUNT LIGHT SENSORS AND TEST 
After school:
-	Unmount motor driver and wires 
-	Remount and reconnect wires


Small Robot:
Now:
-	Cut back m4 wires and connect to motor controller
-	Mount arduino and connect all wires to it
After school:
-	When receive sensor mount and connect it


Recess:
-	Testing which motor controllers is dead 
-	Mount light sensors 

========================================================

  TODO LIST:

Small Robot:

  - Set up light sensor hardware in robot

  - create a startup config program for the light sensors


  - write code for the compass to auto align when not much is happening (started compass intialisation)

  - create a heading compass system, later when we have two robots we NOTE: i have pre setup the variables
    can use the heading system for robots to return to original location

Big Robot:

- Figure out which motor driver isnt working 

-  Unmount motor driver that is not working and disconnect all wires and put new motor drivers.
  
  - The fake data plays with the booleans and conditionals making the robot go in different directions.
    currently commented out that current part of the code which makes the robot change direction becuase of those.

  - create a heading compass system, later when we have two robots we NOTE: i have pre setup the variables
    can use the heading system for robots to return to original location


After State:
  - Put compass on both robots
  
  - work on a random defence timer, robot going back into its half and then strafe defending. (Once compass is known to work, ill have to write stuff - cannot getting working til main movement code decides to work.
    to do with the headings....  oh noes.. requires advanced trig :/).

  - write code for the compass to auto align when not much is happening (started compass intialisation)

  - Dress robot make look pretty (Leds, Lasers, Protection etc)

  CHANGELOG:

  - FIXED: "Issue where the robot will see different bars  behind it, so where 9 would be theres 2 or 9 and would make the robot spam left-right
    facing the wrong way"

    SOLUTION: the bars issue is still happening but has been resolved by changing directions in which the motors move. apparently the motor
    controllers do not like some movements and will go backwards !? kinda problematic.. will investigate after comp.
    
  - ADDED: speed control support for the motor controllers, so we dont tear apart the mat. 


  DROP LIST:

  - Not necessary: Re-Writing Main setup code in a seperate file, debunk the code issue and decide if its a hardware issue
  
  - Too Much overhead and takes CPU cycles: get serial screen working, outputting what would be in the console


