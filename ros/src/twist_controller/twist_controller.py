from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, decel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement

        # controller for yaw
        min_speed = 0.1
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        # controller for throttle
        Kp = 0.3
        Ki = 0.1
        Kd = 0.0
        min_throttle = 0
        max_throttle = 0.2
        self.throttle_controller = PID(Kp, Ki, Kd, min_throttle, max_throttle)

        #low pass filter for velocity
        tau = 0.5
        ts = 0.02 #50Hz
        self.vel_lpf = LowPassFilter(tau,ts)
        tau_s = 0.5
        ts_s = 0.02 #50Hz
        self.steer_lpf = LowPassFilter(tau_s,ts_s)

        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        self.decel_limit = decel_limit

        self.last_time = rospy.get_time()
        self.last_steering = 0
        


    def control(self, linear_velocity, angular_velocity, current_velocity, current_angular_velocity, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.,0.,0.
        
        # steering control
        current_velocity = self.vel_lpf.filt(current_velocity)
        current_angular_velocity = self.steer_lpf.filt(current_angular_velocity)
        
        # if abs(current_angular_velocity -angular_velocity)<0.2:
        
        steering = self.yaw_controller.get_steering(linear_velocity,angular_velocity,current_velocity)
        # rospy.logwarn("steering requested = {}".format(steering))
        
        self.last_steering = steering

        # Throttle control
        vel_error = linear_velocity - current_velocity
        self.last_velocity = current_velocity
        
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error,sample_time)

        # Brake control
        brake = 0
        if linear_velocity == 0. and current_velocity<0.1:
            throttle = 0
            brake = 700
        elif throttle<0.1 and vel_error<0:
            throttle=0
            decel = max(vel_error,self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius
        
        return throttle, brake, steering
