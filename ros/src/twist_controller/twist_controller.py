import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
BRAKE_ = 400 # N*m , 400 for simulator, 700 for Carla

class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, 
                       wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        # Init lateral controller
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        # Init longitudinal controller
        kp = 0.3	# propotional param
        ki = 0.1	# integral param
        kd = 0.		# differential param
        mn = 0.     # minimum throttle value
        mx = 0.3    # maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        tau = 0.5   # cutoff frequency 1/(2pi*tau)
        ts = 0.02   # sample time

        # The LowPassFilter was created because the velocity
        # taht's coming in over the messages is kind of noisy.
        # Filtering out all of the high-frequency noise in the velocity
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass  = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    # Created instance of this controller class up above and then we call control, that's 50 hertz basically
    def control(self, dbw_enabled, current_vel, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        # We can turn the drive by wire on and off in the car, as we're sitting,
        # waiting for a traffic light, I might turn the DBW off or if I'm fixing something else,
        # DBW might be off while the code is running.
        # And if you're using a PID controller and you have an integral term
        # then if you don't turn the PID controller off
        # you'll be accumulating error and the car might do something really erratic
        if not dbw_enabled:
            self.throttle_controller.reset()    # It resets the controller
            return 0., 0., 0.

        current_vel = self.vel_lpf.filt(current_vel)

        #rospy.logwarn("in control:")
        #rospy.logwarn("Current velocity: {0}".format(current_vel))
        #rospy.logwarn("Target  velocity: {0}".format(linear_vel))       
        #rospy.logwarn("Filterd velocity: {0}".format(self.vel_lpf.get()))        
        #rospy.logwarn("Target Angular vel: {0}".format(angular_vel))

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        # This is a simple control of the car error
        # The first check we're doing is if our linear velocity, that's our target,
        # linear velocity is zero, and we are going very slow,
        # Current velocity < 0.1 => probably be trying to stop,
        # what we're gonna do is set the throttle equal to zero and apply a lot of break
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time  = current_time - self.last_time
        self.last_time = current_time
        
        throttle = self.throttle_controller.step(vel_error, sample_time)       
        brake = 0

        # If our velocity is negative, which means that we're going faster
        # than we want to be, faster than our target velocity
        # Brake or acclerate only 1 actor can work.
        if linear_vel == 0. and current_vel < 0.1:
            throttle = 0
            brake = BRAKE_  # N*m , 400 for simulator, 700 for Carla - to hold the car in place if vehicle stopped at a light 
        elif throttle < 0.1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit) 
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius # Torque N*m  
 
        return throttle, brake, steering
