import pygame
from Pluto import pluto
import time

# Initialize Pluto drone
drone = pluto()

# Initialize pygame
pygame.init()
pygame.joystick.init()

# Check for joysticks
if pygame.joystick.get_count() == 0:
    print("No joystick detected.")
    quit()

# Initialize joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

try:
    while True:
        # Get joystick events
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                # Axis motion event handling
                if event.axis == 0:  # X-axis (left-right)
                    if event.value < -0.5:
                        drone.left()
                    elif event.value > 0.5:
                        drone.right()
                    else:
                        drone.reset()
                elif event.axis == 1:  # Y-axis (up-down)
                    if event.value < -0.5:
                        drone.forward()
                    elif event.value > 0.5:
                        drone.backward()
                    else:
                        drone.reset()
                elif event.axis == 2:  # Throttle (up-down)
                    if event.value < -0.5:
                        drone.increase_height()
                    elif event.value > 0.5:
                        drone.decrease_height()
                    else:
                        drone.reset()
                elif event.axis == 3:  # Yaw (left-right)
                    if event.value < -0.5:
                        drone.left_yaw()
                    elif event.value > 0.5:
                        drone.right_yaw()
                    else:
                        drone.reset()

            elif event.type == pygame.JOYBUTTONDOWN:
                # Button press event handling
                if event.button == 0:  # Button A (arm)
                    drone.arm()
                elif event.button == 1:  # Button B (disarm)
                    drone.disarm()
                elif event.button == 7:  # Button Start (take off)
                    drone.take_off()
                elif event.button == 6:  # Button Back (land)
                    drone.land()

            elif event.type == pygame.JOYBUTTONUP:
                # Button release event handling
                if event.button in [0, 1, 6, 7]:  # Arm, disarm, take off, land
                    drone.reset()

        # Limit loop frequency
        time.sleep(0.1)

finally:
    # Clean up
    joystick.quit()
    pygame.quit()
