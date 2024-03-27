oject_detection - detects and integrated
obj_detect_test1 - integrated and drone taking off with blocks and not from mobile
obj_detect_joy -  second logic of keeping drone in air and hover
not worked now using joystick control for takeoff
joystick working properly 
obj test 3 - take only takeoff comand from joystick
running succesfully


obj test 3:
testing on drone 
takeoff through joystick - done
object tracking
roll right - right movement is less
roll left - done
land -done
actual flight 
roll working properly
land working properly
forward and backward working properly.
task is to increase takeoff time this can be achieved by using throttle speed class from drone.py

changes made in pluto
forward -1600 to 1400
backward- 1300 to 1000
left - 1200 to 1000
right - 1600 to 1400
not working after changing values

obj_test 4 -making some changes in obj_test 3
making use of joystick axis button
changes are made in joystick a whole new code of jostick.py is added in this file now drone is working properly.
every command tested and working but speed is too much.
obj_test5
now make drone to roll less
-change drone conditions such that object detcted visits any of one direction once in a cycle.
-read the command only once and not again and again.
-read the command only for 0.1 seconds.


task
-red color for takeoff
-green color for land
