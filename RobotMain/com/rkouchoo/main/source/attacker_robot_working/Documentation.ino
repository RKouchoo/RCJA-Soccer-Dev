/*
 _____           _       ______        ______                         _   __                 _                 
/  __ \         | |      | ___ \       | ___ \                       | | / /                | |                
| /  \/ ___   __| | ___  | |_/ /_   _  | |_/ /___  __ _  __ _ _ __   | |/ /  ___  _   _  ___| |__   ___   ___  
| |    / _ \ / _` |/ _ \ | ___ \ | | | |    // _ \/ _` |/ _` | '_ \  |    \ / _ \| | | |/ __| '_ \ / _ \ / _ \ 
| \__/\ (_) | (_| |  __/ | |_/ / |_| | | |\ \  __/ (_| | (_| | | | | | |\  \ (_) | |_| | (__| | | | (_) | (_) |
 \____/\___/ \__,_|\___| \____/ \__, | \_| \_\___|\__, |\__,_|_| |_| \_| \_/\___/ \__,_|\___|_| |_|\___/ \___/ 
                                 __/ |             __/ |                                                       
                                |___/             |___/                                                        
  
  @Team Circut Breaker Dev
  
  Will eventually be released into the public domain *cough*, ummm.. UPDATE: Slightly older version on git (~10 days).

  First created: 3 / 7 / 2015
  Latest update: 22 / 8 / 2016
   
  Interesting Link: http://dlnmh9ip6v2uc.cloudfront.net/ - I love this link
 
  details about the compass sensor can be found on this data sheet http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/Magneto/HMC5883L-FDS.pdf
 
  ===============
  CURREENT STATE:
  ===============  

  WORKING ON CURRENTLY: Hard wiring the new light sensors and lasers to robot. Also working on a new check for system. with switches.
  
  ==============

  KICKER NOTICE:
  currently have not implemented methods for use of a soleniod+transformer/boost converter as i cannot mount it to either of the robots 
  & and dangerous as a high current might get us disqualified (5 AMPS+ OUTPUT)

  
  TODO LIST:
  
  - Set up light sensor hardware in robot
  
  - create a startup config program for the light sensors 
  
  - wire up compass
  
  - get serial screen working, outputting what would be in the console
  
  - write code for the compass to auto align when not much is happening
  
  - create a heading compass system, later when we have two robots we NOTE: i have pre setup the variables
    can use the heading system for robots to return to original location
    
  - figure out why robot goes spazz when the ightsensors are not reporting any data (mystery data gets reported for some reason on A0-A9).
    The fake data plays with the booleans and conditionals making the robot go in different directions.
    currently commented out that current part of the code which makes the robot change direction becuase of those.
    
  - dress the robot and make it look pretty LEDS, LEDS, LEDS, AND MORE.
  
  - work on a random defence timer, robot going back into its half and then strafe defending. (Once compass is known to work, ill have to write stuff
    to do with the headings....  oh noes.. requires advanced trig :/).


  CHANGE LOG:

  - FIXED: "Issue where the robot will see different bars  behind it, so where 9 would be theres 2 or 9 and would make the robot spam left-right 
    facing the wrong way"
    
    SOLUTION: the bars issue is still happening but has been resolved by changing directions in which the motors move. apparently the motor 
    controllers do not like some movements and will go backwards !? kinda problematic.. will investigate after comp.
    

  DROP LIST:

  - Not necessary: Re-Writing Main setup code in a seperate file, debunk the code issue and decide if its a hardware issue
  
  */
