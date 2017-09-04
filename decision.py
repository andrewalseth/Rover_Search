import numpy as np
from supporting_functions import is_rock
# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):


    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Counter to determine if robot is stuck - zero velocity but forward command
    if Rover.vel < 0.1 and Rover.mode == 'forward':
        Rover.stuck_count +=1

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        #determine the angle dictated by the perception function
        nav_direction = np.mean(Rover.nav_angles * 180/np.pi)

        #determine if rocks are preseent in image, and
        rock, rock_angles_scrub = is_rock(Rover.rock_angles)
        rock_direction= np.mean(rock_angles_scrub * 180/np.pi)

        # Check for Rover.mode status
        if Rover.mode == 'forward':

            #check if rover is near a rock - stop if that is the case
            if Rover.near_sample == 1:
                Rover.mode = 'stop'

            elif Rover.stuck_count >= 20:
                Rover.mode = 'stop'
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0

            # Check the extent of navigable terrain OR if rock is in view
            elif len(Rover.nav_angles) >= Rover.stop_forward or rock == 1:
                # If mode is forward, navigable terrain looks good
                # and velocity is below max, then throttle
                if Rover.vel < (Rover.max_vel * (1- rock*2/3) ):
                    # Set throttle value to throttle setting - unless rock is visible
                    Rover.throttle = Rover.throttle_set

                    Rover.steer = np.clip((1-rock) * (nav_direction + 5) + rock * rock_direction, -15, 15)
                else: # Else coast
                    Rover.throttle = 0
                    Rover.brake = 0

                    # Set steer to mean angle - preferrential steering towards rocks
                    Rover.steer = np.clip((1-rock) * (nav_direction + 5) + rock * rock_direction, -15, 15)

            # If there's a lack of navigable terrain pixels then go to 'stop' mode - also no rock
            elif len(Rover.nav_angles) < Rover.stop_forward and rock == 0 :
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':

            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:

                # if a sample is nearby, send rover to pickup mode
                if Rover.near_sample == 1:
                    Rover.mode = 'pickup'

                elif Rover.stuck_count >= 20 and abs(nav_direction) < 3:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0

                    Rover.stuck_count = 0
                    # Set steer to mean angle - preferrential steering towards rocks
                    Rover.steer = np.clip((1-rock) * (nav_direction ) + rock * rock_direction, -15, 15)
                # Now we're stopped and we have vision data to see if there's a path forward
                elif len(Rover.nav_angles) < Rover.go_forward or Rover.stuck_count >= 20:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = 15 # Could be more clever here about which way to turn

                # If we're stopped but see sufficient navigable terrain in front then go!
                elif len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0

                    # Set steer to mean angle - preferrential steering towards rocks
                    Rover.steer = np.clip((1-rock) * (nav_direction) + rock * rock_direction, -15, 15)

                    Rover.mode = 'forward'


        elif Rover.mode == 'pickup':

            # if rover in pickup mode and near sample, send pickup signal
            if Rover.picking_up == 0 and Rover.near_sample == 1:
                Rover.send_pickup = True

            # if rover is in pickup mode and not near sample, revert to forward
            elif Rover.near_sample == 0:
                Rover.mode = 'forward'
                Rover.brake = 0
                Rover.throttle = Rover.throttle_set
            # return to forward movement
            else:
                Rover.mode = 'forward'
                Rover.brake = 0
                Rover.throttle = Rover.throttle_set

    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    # If in a state where want to pickup a rock send pickup command


    return Rover
