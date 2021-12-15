#!/usr/bin/env python

import rospy
import numpy as np
import math
import sys

import utils

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseArray, Pose

SCAN_TOPIC = '/car/scan' # The topic to subscribe to for laser scans
CMD_TOPIC = '/car/mux/ackermann_cmd_mux/input/navigation' # The topic to publish controls to
POSE_TOPIC = '/car/car_pose' # The topic to subscribe to for current pose of the car
                                  # NOTE THAT THIS IS ONLY NECESSARY FOR VIZUALIZATION
VIZ_TOPIC = '/laser_wanderer/rollouts' # The topic to publish to for vizualizing
                                       # the computed rollouts. Publish a PoseArray.

MAX_PENALTY = 10000 # The penalty to apply when a configuration in a rollout
                    # goes beyond the corresponding laser scan
CARPOSE = [0.0,0.0,0.0]                    

'''
Wanders around using minimum (steering angle) control effort while avoiding crashing
based off of laser scans. 
'''
class LaserWanderer:

  '''
  Initializes the LaserWanderer
    rollouts: An NxTx3 numpy array that contains N rolled out trajectories, each
              containing T poses. For each trajectory, the t-th element represents
              the [x,y,theta] pose of the car at time t+1
    deltas: An N dimensional array containing the possible steering angles. The n-th
            element of this array is the steering angle that would result in the 
            n-th trajectory in rollouts
    speed: The speed at which the car should travel
    compute_time: The amount of time (in seconds) we can spend computing the cost
    laser_offset: How much to shorten the laser measurements
  '''
  def __init__(self,pose_distL, pose_distR, laser_idxL, laser_idxR, rollouts, deltas, min_delta, max_delta, delta_incr, dt, T, car_length, speed, compute_time, laser_offset):
    # Store the params for later
    self.laser_idxL = laser_idxL
    self.laser_idxR = laser_idxR
    self.pose_distL = pose_distL
    self.pose_distR = pose_distR
    self.rollouts = rollouts
    self.deltas = deltas
    self.min_delta = min_delta
    self.max_delta = max_delta
    self.delta_incr = delta_incr
    self.dt = dt
    self.T = T
    self.car_length = car_length
    self.speed = speed
    self.compute_time = compute_time
    self.laser_offset = laser_offset
    
    # YOUR CODE HERE
    self.cmd_pub = rospy.Publisher(CMD_TOPIC, AckermannDriveStamped, queue_size=10) # Create a publisher for sending controls
    self.laser_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, self.wander_cb) # Create a subscriber to laser scans that uses the self.wander_cb callback
    self.viz_pub = rospy.Publisher(VIZ_TOPIC, PoseArray, queue_size=10) # Create a publisher for vizualizing trajectories. Will publish PoseArrays
    self.viz_sub = rospy.Subscriber(POSE_TOPIC, PoseStamped, self.update_CARPOSE) # Create a subscriber to the current position of the car
    # NOTE THAT THIS VIZUALIZATION WILL ONLY WORK IN SIMULATION. Why?
    
  '''
  get the pose of the car currently and set it to the global var to be accessed by the functions that need it
  '''
  def update_CARPOSE(self, msg):
     global CARPOSE
     CARPOSE = [msg.pose.position.x, msg.pose.position.y, utils.quaternion_to_angle(msg.pose.orientation)]

  '''
  Vizualize the rollouts. Transforms the rollouts to be in the frame of the world.
  Only display the last pose of each rollout to prevent lagginess
    msg: A PoseStamped representing the current pose of the car
  '''  
  def viz_sub_cb(self, msg):
    # Create the PoseArray to publish. Will contain N poses, where the n-th pose
    # represents the last pose in the n-th trajectory
    pa = PoseArray()
    pa.header.frame_id = '/map'
    pa.header.stamp = rospy.Time.now()

    t = len(msg[0]) - 1    

    for i in range(len(msg)):
	pose = Pose()	
	pose.position.x = msg[i,t,0]
	pose.position.y = msg[i,t,1]
	pose.position.z = 0
	pose.orientation = utils.angle_to_quaternion(msg[i,t,2])

	pa.poses.append(pose)

    self.viz_pub.publish(pa)
    
  '''
  Controls the steering angle in response to the received laser scan. Uses approximately
  self.compute_time amount of time to compute the control
    msg: A LaserScan
  '''
  def wander_cb(self, msg):
    start = rospy.Time.now().to_sec() # Get the time at which this function started
    
    # A N dimensional matrix that should be populated with the costs of each
    # trajectory up to time t <= T
    delta_costs = np.zeros(self.deltas.shape[0], dtype=np.float) 
    
    
    laser_ranges = np.array(msg.ranges)
    laser_ranges = np.where(np.isnan(laser_ranges), 1000, laser_ranges)
    laser_distL = laser_ranges[self.laser_idxL] - self.laser_offset
    laser_distR = laser_ranges[self.laser_idxR] - self.laser_offset
    delta_costs = abs(self.deltas) * self.T +np.sum(np.where(self.pose_distL > laser_distL, MAX_PENALTY, 0), axis=1) + np.sum(np.where(self.pose_distR > laser_distR, MAX_PENALTY, 0), axis=1) 
    
    # Find the delta that has the smallest cost and execute it by publishing
    smallest_index = 0
    for i in range(1,len(self.deltas)):
       if delta_costs[i] < delta_costs[smallest_index]:
          smallest_index = i

    #uncomment for the simulation to visualize the rollouts
    #self.viz_sub_cb(self.rollouts)

    ads = AckermannDriveStamped()
    ads.header.frame_id = '/map'
    ads.header.stamp = rospy.Time.now()
    ads.drive.steering_angle = self.deltas[smallest_index]
    ads.drive.speed = self.speed
    self.cmd_pub.publish(ads)
    
'''
Repeatedly apply the kinematic model to produce a trajectory for the car
  init_pose: The initial pose of the robot [x,y,theta]
  controls: A Tx3 numpy matrix where each row is of the form [v,delta,dt]
  car_length: The length of the car
Returns a Tx3 matrix where the t-th row corresponds to the robot's pose at time t+1
'''
def generate_rollout(init_pose, controls, car_length):
  rollout = np.zeros((len(controls),3), dtype=np.float) #create a Tx3 array  

  # dTHETA = speed/length * tan(steering angle)
  # dx = speed * cos(dTHETA)
  # dy = speed * sin(dTHETA)

  temp_x = init_pose[0] #get initial values for these from the initial pose of car
  temp_y = init_pose[1]
  temp_theta = init_pose[2]

  for i in range(len(controls)):
     dTHETA = (controls[i,0] / car_length) * np.tan(controls[i,1]) #calculate dtheta
     rollout[i,2] = temp_theta + dTHETA * controls[i][2] 
     dx = controls[i,0] * np.cos(rollout[i,2]) #use theta to calculate dx 
     dy = controls[i,0] * np.sin(rollout[i,2]) #use theta to calculate dy
     rollout[i,0] = temp_x + dx * controls[i][2]
     rollout[i,1] = temp_y + dy * controls[i][2]
     temp_x = rollout[i,0]
     temp_y = rollout[i,1]
     temp_theta = rollout[i,2]
  
  return rollout
   
'''
Helper function to generate a number of kinematic car rollouts
    speed: The speed at which the car should travel
    min_delta: The minimum allowed steering angle (radians)
    max_delta: The maximum allowed steering angle (radians)
    delta_incr: The difference (in radians) between subsequent possible steering angles
    dt: The amount of time to apply a control for
    T: The number of time steps to rollout for
    car_length: The length of the car
Returns a NxTx3 numpy array that contains N rolled out trajectories, each
containing T poses. For each trajectory, the t-th element represents the [x,y,theta]
pose of the car at time t+1
'''
def generate_mpc_rollouts(speed, min_delta, max_delta, delta_incr, dt, T, car_length):

  deltas = np.arange(min_delta, max_delta, delta_incr)
  N = deltas.shape[0]
  
  init_pose = np.array([0.0,0.0,0.0], dtype=np.float) 
  
  rollouts = np.zeros((N,T,3), dtype=np.float)
  for i in xrange(N):
    controls = np.zeros((T,3), dtype=np.float)
    controls[:,0] = speed
    controls[:,1] = deltas[i]
    controls[:,2] = dt
    rollouts[i,:,:] = generate_rollout(init_pose, controls, car_length)
    
  return rollouts, deltas


def main():

  rospy.init_node('laser_wanderer', anonymous=True)

  # Load these parameters from launch file
  # We provide suggested starting values of params, but you should
  # tune them to get the best performance for your system
  # Look at constructor of LaserWanderer class for description of each var
  # 'Default' values are ones that probably don't need to be changed (but you could for fun)
  # 'Starting' values are ones you should consider tuning for your system  

  speed = 0.5 # Default val: 1.0
  min_delta = -0.34 # Default val: -0.34
  max_delta = 0.341 # Default val: 0.341
  delta_incr = 0.34/5 # Starting val: 0.34/3 (consider changing the denominator) 
  dt = 0.01 # Default val: 0.01
  T = 500 # Starting val: 300
  compute_time = 0.09 # Default val: 0.09
  laser_offset = .283 # Starting val: 1.0
  w = 0.35
  laser_incr = 0.00872664991766
  
  # DO NOT ADD THIS TO YOUR LAUNCH FILE, car_length is already provided by teleop.launch
  car_length = rospy.get_param("/car/vesc/chassis_length", 0.33) 
  
  # Generate the rollouts
  rollouts, deltas = generate_mpc_rollouts(speed, min_delta, max_delta,
                                           delta_incr, dt, T, car_length)
  rollouts_left = np.zeros(rollouts.shape)
  rollouts_left[:,:,0] = rollouts[:,:,0] - (w/2) * np.sin(rollouts[:,:,2]) 
  rollouts_left[:,:,1] = rollouts[:,:,1] + (w/2) * np.cos(rollouts[:,:,2]) 
  rollouts_left[:,:,2] = rollouts[:,:,2] 
  rollouts_right = np.zeros(rollouts.shape)
  rollouts_right[:,:,0] = rollouts[:,:,0] + (w/2) * np.sin(rollouts[:,:,2]) 
  rollouts_right[:,:,1] = rollouts[:,:,1] - (w/2) * np.cos(rollouts[:,:,2]) 
  rollouts_right[:,:,2] = rollouts[:,:,2]

  pose_distL = np.linalg.norm(rollouts_left[:,:,:2], axis=2)
  pose_distR = np.linalg.norm(rollouts_right[:,:,:2], axis=2)

  phiL = np.arctan2((rollouts_left[:,:,1]), (rollouts_left[:,:,0]))
  phiL = np.where(phiL > math.pi, phiL - 2*math.pi, phiL)
  phiL = np.where(phiL < -math.pi, phiL + 2*math.pi, phiL)
  laser_idxL = (phiL-math.pi) / laser_incr 
  laser_idxL = laser_idxL.astype(int)

  phiR = np.arctan2((rollouts_right[:,:,1]), (rollouts_right[:,:,0]))
  phiR = np.where(phiR > math.pi, phiR - 2*math.pi, phiR)
  phiR = np.where(phiR < -math.pi, phiR + 2*math.pi, phiR)
  laser_idxR = (phiR-math.pi) / laser_incr
  laser_idxR = laser_idxR.astype(int)
  
  # Create the LaserWanderer                                         
  lw = LaserWanderer(pose_distL, pose_distR, laser_idxL, laser_idxR, rollouts, deltas, min_delta, max_delta, delta_incr, dt, T, car_length, speed, compute_time, laser_offset)
  
  # Keep the node alive
  rospy.spin()
  

if __name__ == '__main__':
  main()
