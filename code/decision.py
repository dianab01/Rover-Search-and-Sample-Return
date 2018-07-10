import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
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
            # only check threshold for stoping or cointinuing going forward if no rock to pick up is in the Rover's field of view
            elif Rover.vel <= 0.2 and not Rover.rock_in_fov:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.brake = 0
                    Rover.throttle = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                    
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
        # State to be added in order to return samples to starting point
        #elif Rover.mode == 'return_samples':
            
        elif Rover.mode == 'collect_rock':
            # First slowly steer in the direction of the rock
            Rover.brake = 0
            Rover.throttle = Rover.throttle_set
            Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)
            # Brake to allow pick up if close to rock
            if Rover.near_sample:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # When a new sample has been collected, or the Rover has just passed nearby the rock without collencting it
            # return to mapping the environment
            if (Rover.samples_collected - Rover.last_samples_collected) > 0 or not Rover.rock_in_fov:
                Rover.mode = 'stop'
                
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
    
    # Try to collect the rock if in sight and store the current collected samples count
    if Rover.rock_in_fov and not Rover.near_sample and not Rover.picking_up:
        Rover.mode = 'collect_rock'
        last_samples_collected = Rover.samples_collected
    
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.last_samples_collected = Rover.samples_collected
        Rover.send_pickup = True
        Rover.samples_located = Rover.samples_located + 1
        
    #if Rover.samples_collected == Rover.samples_to_find:
        #Rover.mode = 'return_samples'
        
    print(Rover.mode)
    #print(np.count_nonzero(np.select(Rover.nav_angles > np.mean(Rover.nav_angles), Rover.nav_angles)))
    
    
    return Rover
