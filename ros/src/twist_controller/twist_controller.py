import rospy
from pid import *
from lowpass import LowPassFilter
from yaw_controller import *

GAS_DENSITY = 2.858
ONE_MPH = 0.44704



class Controller(object):
    def __init__(self, vehicle_mass,fuel_capacity,brake_deadband,decel_limit,accel_limit,wheel_radius,wheel_base,steer_ratio,max_lat_accel,max_steer_angle):

        self.yaw_controller=YawController(wheel_base, steer_ratio, ONE_MPH, max_lat_accel, max_steer_angle)
        throttle_kp=0.3
        throttle_ki=0.1
        throttle_kd=0
        throttle_mn=0 #min throttle
        throttle_mx=0.5 #max throttle

        accel_kp=0.5
        accel_ki=0.1
        accel_kd=0
        accel_tau=0.5
        accel_ts=0.02

        steer_kp=0.8
        steer_ki=0.1
        steer_kd=0.3

        self.vel_lpf = LowPassFilter(accel_tau, accel_ts)
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.last_time = rospy.get_time()
        self.max_steer_angle=max_steer_angle

        self.throttle_controller=PID(throttle_kp,throttle_ki,throttle_kd,throttle_mn,throttle_mx)
        self.steering_pid=PID(steer_kp,steer_ki,steer_kd,-abs(max_steer_angle),max_steer_angle)
        tau=0.5
        ts=0.02


    def control(self,current_vel,linear_vel,angular_vel,dbw_enabled,cte):
        # TODO: Change the arg, kwarg list to suit your needs
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0,0,0

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        current_vel=self.vel_lpf.filt(current_vel)
        #steering=self.yaw_controller.get_steering(linear_vel,angular_vel,current_vel) + self.steering_pid.step(cte, sample_time)
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        #steering= max(-abs(self.max_steer_angle), min(abs(self.max_steer_angle), steering))
        vel_error=linear_vel - current_vel
        #str="Linear Vel={} Current Vel={} vel_error={}".format(linear_vel,current_vel,vel_error)
        #rospy.logdebug(str)
        self.last_vel=current_vel
        throttle=self.throttle_controller.step(vel_error,sample_time)
        rospy.logdebug("Steering=%s", steering)
        brake=0
        '''
        if linear_vel==0 and current_vel<0.1:
            throttle=0
            brake=400 #Nm to hold the car in place if we are stopped
        elif throttle<0.1 and vel_error<0:
            throttle=0
            decel=max(vel_error,self.decel_limit)
            brake=abs(decel)*self.vehicle_mass*self.wheel_radius # braking in newton-meter  
        '''
        return throttle,brake,steering

        #return 1., 0., 0.
