# RCJA Soccer Dev

@Team Circut Breaker Dev

  First created: 3 / 7 / 2015
  Latest update: 23 / 8 / 2016

  Interesting Link: http://dlnmh9ip6v2uc.cloudfront.net/ - I love this link

  details about the compass sensor can be found on this data sheet http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/Magneto/HMC5883L-FDS.pdf

  ===============
  CURREENT STATE:
  ===============
    
    +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    - MAKE THE ROBOT FUCKING WORK YOU ASS YOU HAVE 1 DAY !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  WORKING ON CURRENTLY: Hard wiring the new light sensors and lasers to robot. Also working on a new check for system, with switches.

  ==============

  KICKER NOTICE:
  currently have not implemented methods for use of a soleniod+transformer/boost converter as i cannot mount it to either of the robots
  & and dangerous as a high current might get us disqualified (5 AMPS+ OUTPUT)


  TODO LIST:

  - Set up light sensor hardware in robot

  - create a startup config program for the light sensors

  - wire up compass

  - write code for the compass to auto align when not much is happening

  - create a heading compass system, later when we have two robots we NOTE: i have pre setup the variables
    can use the heading system for robots to return to original location

  - figure out why robot goes spazz when the ightsensors are not reporting any data (mystery data gets reported for some reason on A0-A9).
    The fake data plays with the booleans and conditionals making the robot go in different directions.
    currently commented out that current part of the code which makes the robot change direction becuase of those.

  - dress the robot and make it look pretty LEDS, LEDS, LEDS, AND MORE.

  - work on a random defence timer, robot going back into its half and then strafe defending. (Once compass is known to work, ill have to write stuff
    to do with the headings....  oh noes.. requires advanced trig :/).


  CHANGELOG:

  - FIXED: "Issue where the robot will see different bars  behind it, so where 9 would be theres 2 or 9 and would make the robot spam left-right
    facing the wrong way"

    SOLUTION: the bars issue is still happening but has been resolved by changing directions in which the motors move. apparently the motor
    controllers do not like some movements and will go backwards !? kinda problematic.. will investigate after comp.
    
  - ADDED: speed control support for the motor controllers, so we dont tear apart the mat. 


  DROP LIST:

  - Not necessary: Re-Writing Main setup code in a seperate file, debunk the code issue and decide if its a hardware issue
  
  - Too Much overhead and takes CPU cycles: get serial screen working, outputting what would be in the console


