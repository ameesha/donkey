"""
drive.py

Script to run on the Raspberry PI to start your vehicle's drive loop. The
drive loop will use post requests to the server specified in the remote
argument. Run the serve.py script on a different computer to start the remote
server.

Usage:
    drive.py

"""

import sys
from docopt import docopt

import donkey as dk

# Get args.
args = docopt(__doc__)

if __name__ == '__main__':

    # load config file
    cfg = dk.config.parse_config('~/mydonkey/vehicle.ini')

    #load the actuators (default is the adafruit servo hat)
    mythrottlecontroller = dk.actuators.PCA9685_Controller(cfg['throttle_actuator_channel'])
    mysteeringcontroller = dk.actuators.PCA9685_Controller(cfg['steering_actuator_channel'])

    #set the PWM ranges
    mythrottle = dk.actuators.PWMThrottleActuator(controller=mythrottlecontroller,
                                                  min_pulse=cfg['throttle_actuator_min_pulse'],
                                                  max_pulse=cfg['throttle_actuator_max_pulse'],
                                                  zero_pulse=cfg['throttle_actuator_zero_pulse'])

    mysteering = dk.actuators.PWMSteeringActuator(controller=mysteeringcontroller,
                                                  left_pulse=cfg['steering_actuator_min_pulse'],
                                                  right_pulse=cfg['steering_actuator_max_pulse'])

    #abstract class to combine actuators
    mymixer = dk.mixers.AckermannSteeringMixer(mysteering, mythrottle)

    #Create your car
    car = dk.vehicles.BaseVehicle(drive_loop_delay=cfg['vehicle_loop_delay'],
                                  actuator_mixer=mymixer)

    #Start the drive loop
    car.start()
