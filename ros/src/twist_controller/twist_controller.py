from pid import PID
import rospy
from lowpass import LowPassFilter
from yaw_controller import YawController
GAS_DENSITY = 2.858
ONE_MPH = 0.44704




class Controller(object):
    def __init__(self
               ,vehicle_mass
               ,fuel_capacity
               ,brake_deadband
               ,decel_limit
               ,accel_limit
               ,wheel_radius
               ,wheel_base
               ,steer_ratio
               ,max_lat_accel
	       ,max_steer_angle):
	# init class specific variables
	self.vehicle_mass = vehicle_mass
	self.fuel_capacity = fuel_capacity
	self.brake_deadband = brake_deadband
	self.decel_limit = decel_limit
	self.accel_limit = accel_limit
	self.wheel_radius = wheel_radius
	self.wheel_base = wheel_base
	self.steer_ratio = steer_ratio
	self.max_lat_accel = max_lat_accel
	self.max_steer_angle = max_steer_angle

	self.min_vel = 0.
	self.max_vel = 15.
	#init yaw controller
	self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel,max_steer_angle)
	
	kp = 0.3 #2.16 #3.5   #.16
	ki = 0.1 #0.1 #5
	kd = 0. #1.7   # 1.7
	min_throttle = 0.
	max_throttle = 0.2 
    	
        self.throttle_controller = PID(kp,ki,kd,min_throttle, max_throttle)	

	tau = 0.5 # 0.5 cutoff frequency
	ts = 0.02 # 0.02 sampling time
	self.vel_lpf = LowPassFilter(tau,ts)
	
	self.last_time = rospy.get_time() #timestep to pass



    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel, time_step):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enabled:  # if drive by wire is disabled return zero values so can be manually piloted
	    self.throttle_controller.reset()
            return 0., 0., 0.
        
	current_vel = self.vel_lpf.filt(current_vel)
	
	# setting steering angle yaw_controller arglist :
	#  def get_steering(self, linear_velocity, angular_velocity, current_velocity):
	steering_angle = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

	vel_error = linear_vel - current_vel
	self.last_vel = current_vel


        
	#timing calcs for sample_time
	current_time = rospy.get_time()
	sample_time = current_time - self.last_time
	self.last_time = current_time
	
	#setting throttle step from PID
	
	#arglist def step(self, error, sample_time):
	throttle = self.throttle_controller.step(vel_error, sample_time) #time_step)
	brake = 0.

	# to completely stop and stay stopped at the red stoplight
	if linear_vel == 0. and current_vel < 0.1:
 	    throttle = 0.
	    brake = 400 #N*m automatic transmission car  
	
	# indicating vehicle needs to slow down more.
	elif throttle < .01 and vel_error < 0:
	    throttle = 0.
	    decel = max(vel_error, self.decel_limit)
	    # break force needed is decel factor and weight*wheel radius to apply torque force in newton meters.	
	    brake = abs(decel)*(self.vehicle_mass+self.fuel_capacity*GAS_DENSITY) *self.wheel_radius
	return throttle, brake, steering_angle     	




