from lowpass import LowPassFilter
from pid import PID
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704 #1 mph = 0.44704 m/s


class Controller(object):
    def __init__(self, vehicle_mass,fuel_capacity,brake_deadband,decel_limit,accel_limit,wheel_radius,wheel_base,steer_ratio,max_lat_accel,max_steer_angle):
        # TODO: Implement
        Kp = 0.3
		Ki = 0.1
		Kd = 0.0
		Min = 0.0	#min throttle value 
		Max = 0.2	#maximum throttle value
		self.pid = PID(Kp,Ki,Kd,Min,Max)
		
		self.vehicle_mass = vehicle_mass
		self.fuel_capacity = fuel_capacity
		self.brake_deadband = brake_deadband
		self.decel_limit = decel_limit
		self.accel_limit = accel_limit
		self.wheel_radius = wheel_radius
		self.wheel_base = wheel_base
		self.steer_ratio = steer_ration
		self.max_lat_accel = max_lat_accel
		self.max_steer_angle = max_steer_angle
		self.yawcontroller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
		
		self.tau = 0.5 #cutoff frequency = 1/(2*PI*tau)
		self.ts = 0.02
		self.lpf = LowPassFilter(self.tau, self.ts)
		
		self.throttle = 0.0
		self.brake = 0.0
		self.steer = 0.0
		
		self.last_time = rospy.get_time()
		
    def control(self,linear_vel,angular_vel,cur_linear_vel,dbw_enable):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
		if not dbw_enable:
			self.pid.reset()
			return 0.0, 0.0, 0.0
		
		current_vel = self.lpf.filt(cur_linear_vel)
		
		steering = self.yawcontroller.get_steering(linear_vel,angular_vel,current_vel)
		
		vel_error = linear_vel - current_vel
		last_vel = current_vel
		
		current_time = rospy.get_time()
		sample_time = current_time - self.last_time
		self.last_time = current_time
		
		throttle = self.pid.step(vel_error, sample_time)
		brake = 0.0
		
		if linear_vel == 0 and current_vel < 0.1:
			throttle = 0
			brake = 400
		elif throttle < 0.1 and vel_error < 0:
			throttle = 0
			decel = max(vel_error,self.decel_limit)
			brake = abs(decel)*self.vehicle_mass*self.wheel_radius
		
        return throttle, brake, steering
