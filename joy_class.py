# class is prepapred for joystick direct use

import sys
import os
from Joystick_control import XboxController
from Pluto import pluto

class DroneController:
    def __init__(self):
        # Initialize Xbox controller and Pluto drone objects
        self.joy = XboxController()
        self.me = pluto()

    # Function to map input range to output range
    def mapping(self, x, inMin, inMax, outMin, outMax):
        x = (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin
        if x < outMin:
            return int(outMin)
        elif x > outMax:
            return int(outMax)
        else:
            return int(x)

    # Main loop for continuously reading and processing controller input
    def control_loop(self):
        while True:
            # Read input from the Xbox controller
            [x, y, a, b, A, B, X, Y, rb, lb, rt, lt, ld, rd, ud, dd] = self.joy.read()

            # Map controller input to drone controls
            self.me.rcThrottle = self.mapping(y, 1, -1, 1000, 2000)
            self.me.rcYaw = self.mapping(x, -1, 1, 1000, 2000)
            self.me.rcPitch = self.mapping(b, 1, -1, 1000, 2000)
            self.me.rcRoll = self.mapping(a, -1, 1, 1000, 2000)

            # Check button states for drone actions
            if A:
                self.me.arm()  # Arm the drone
                print("arming", A)
            elif B:
                self.me.disarm()  # Disarm the drone
                print("disarming", B)
            elif Y:
                self.me.take_off()  # Take off the drone
                print("taken off", X)
            elif X:
                self.me.land()  # Land the drone
                self.me.disarm()
                print("landing", Y)



# from joy_class import DroneController

# # Create an instance of the DroneController class
# drone_controller = DroneController()

# # Start the control loop
# drone_controller.control_loop()

