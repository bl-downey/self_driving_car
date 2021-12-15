#!/usr/bin/env python

import collections
import sys

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped

import utils

# The topic to publish control commands to
PUB_TOPIC = '/car/mux/ackermann_cmd_mux/input/navigation'

'''
Follows a given plan using constant velocity and PID control of the steering angle
'''
class LineFollower:

  '''
  Initializes the line follower
    plan: A list of length T that represents the path that the robot should follow
          Each element of the list is a 3-element numpy array of the form [x,y,theta]
    pose_topic: The topic that provides the current pose of the robot as a PoseStamped msg
    plan_lookahead: If the robot is currently closest to the i-th pose in the plan,
                    then it should navigate towards the (i+plan_lookahead)-th pose in the plan
    translation_weight: How much the error in translation should be weighted in relation
                        to the error in rotation
    rotation_weight: How much the error in rotation should be weighted in relation
                     to the error in translation
    kp: The proportional PID parameter
    ki: The integral PID parameter
    kd: The derivative PID parameter
    error_buff_length: The length of the buffer that is storing past error values
    speed: The speed at which the robot should travel
  '''
  def __init__(self, plan, pose_topic, plan_lookahead, translation_weight,
               rotation_weight, kp, ki, kd, error_buff_length, speed):
    # Store the passed parameters
    self.plan = plan
    self.plan_lookahead = plan_lookahead
    # Normalize translation and rotation weights
    self.translation_weight = translation_weight / (translation_weight+rotation_weight)
    self.rotation_weight = rotation_weight / (translation_weight+rotation_weight)
    self.kp = kp
    self.ki = ki
    self.kd = kd
    # The error buff stores the error_buff_length most recent errors and the
    # times at which they were received. That is, each element is of the form
    # LMAO THATS NOT TRUE. yall store it [E,now] - [time_stamp (seconds), error]. For more info about the data struct itself, visit
    # https://docs.python.org/2/library/collections.html#collections.deque
    self.error_buff = collections.deque(maxlen=error_buff_length)
    self.speed = speed
    
    # YOUR CODE HERE
    self.cmd_pub = rospy.Publisher(PUB_TOPIC, AckermannDriveStamped, queue_size = 10) # Create a publisher to PUB_TOPIC
    self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self.pose_cb)# Create a subscriber to pose_topic, with callback 'self.pose_cb'
  
  '''
  Computes the error based on the current pose of the car
    cur_pose: The current pose of the car, represented as a numpy array [x,y,theta]
  Returns: (False, 0.0) if the end of the plan has been reached. Otherwise, returns
           (True, E) - where E is the computed error
'''  
  def do_the_thing(self, cur_pose, goal_xy):
     th = cur_pose[2]
     rot_mat = np.matrix([[np.cos(th), -np.sin(th), cur_pose[0]], [np.sin(th), np.cos(th), cur_pose[1]], [0,0,1]])

     inv_rot = np.linalg.inv(rot_mat)     

     wrt_robot = np.matmul(inv_rot, goal_xy)

     return wrt_robot

  def compute_error(self, cur_pose):
    #print(cur_pose)
    # Find the first element of the plan that is in front of the robot, and remove
    # any elements that are behind the robot. To do this:
    # Loop over the plan (starting at the beginning) For each configuration in the plan
        # If the configuration is behind the robot, remove it from the plan
        #   Will want to perform a coordinate transformation to determine if 
        #   the configuration is in front or behind the robot
        # If the configuration is in front of the robot, break out of the loop
    while len(self.plan) > 0:
       goal_xy = np.matrix([[self.plan[0][0]], [self.plan[0][1]], [1]]) 
       wrt_robot = self.do_the_thing(cur_pose, goal_xy)
       
       if wrt_robot[0] < 0:
	  #print(wrt_robot)
          #print("POP")
	  self.plan.pop(0)
       else: 
	  #print(wrt_robot)
	  #print("NO popPY")
	  break
                  
    #print(len(self.plan))
    # Check if the plan is empty. If so, return (False, 0.0)
    if len(self.plan) == 0:
       return False, 0.0
    
    # At this point, we have removed configurations from the plan that are behind
    # the robot. Therefore, element 0 is the first configuration in the plan that is in 
    # front of the robot. To allow the robot to have some amount of 'look ahead',
    # we choose to have the robot head towards the configuration at index 0 + self.plan_lookahead
    # We call this index the goal_index -- okie sounds gud
    goal_idx = min(0+self.plan_lookahead, len(self.plan)-1)
    #print(cur_pose)
    #print(self.plan[goal_idx])
    # Compute the translation error between the robot and the configuration at goal_idx in the plan
    # YOUR CODE HERE
    
    THETA_ref = self.plan[goal_idx][2]
    translation_error = -np.sin(THETA_ref)*(cur_pose[0] - self.plan[goal_idx][0]) + np.cos(THETA_ref) * (cur_pose[1] - self.plan[goal_idx][1])

    rotation_error = (cur_pose[2] - THETA_ref)

	#z = x/sin(theta) where x is translation of x and theta is the angle 
    #translation_error = axis_change / np.cos(rotation_error)
    
    # Compute the total error
    # Translation error was computed above
    # Rotation error is the difference in yaw between the robot and goal configuration
    #   Be carefult about the sign of the rotation error
    
    error = self.translation_weight * translation_error + self.rotation_weight * rotation_error

    return True, error
    
    
  '''
  Uses a PID control policy to generate a steering angle from the passed error
    error: The current error
  Returns: The steering angle that should be executed
  '''
  def compute_steering_angle(self, error):
    now = rospy.Time.now().to_sec() # Get the current time
    
    # Compute the derivative error using the passed error, the current time,
    # the most recent error stored in self.error_buff, and the most recent time
    # stored in self.error_buff
    # YOUR CODE HERE
	#try-except block since the error_buff is empty upon start and we need the value from there after it has been filled
    try:
       past = self.error_buff.pop()
       deriv_error = (error - past[0])/(now - past[1])
       self.error_buff.append((past[0], past[1])) #only append if popped off deque
    except:
       #past = [0,0]
       deriv_error = 0 #(error - past[0])/(now - past[1])

    # Add the current error to the buffer
    self.error_buff.append((error, now))
    
    # Compute the integral error by applying rectangular integration to the elements
    # of self.error_buff: https://chemicalstatistician.wordpress.com/2014/01/20/rectangular-integration-a-k-a-the-midpoint-rule/
    # YOUR CODE HERE
	#try-except again for first instance when deque empty
    integ_error = 0
    time = now
    for err in self.error_buff:
       integ_error += err[0] * (time-err[1])
       time = err[1]

    #print(integ_error)

    # Compute the steering angle as the sum of the pid errors
    return self.kp*error + self.ki*integ_error + self.kd * deriv_error
    
  '''
  Callback for the current pose of the car
    msg: A PoseStamped representing the current pose of the car
    This is the exact callback that we used in our solution, but feel free to change it
  '''  
  def pose_cb(self, msg):
#    print(msg.pose.position)
    cur_pose = np.array([msg.pose.position.x,
                         msg.pose.position.y,
                         utils.quaternion_to_angle(msg.pose.orientation)])
#    print("test: ", cur_pose)
    success, error = self.compute_error(cur_pose)
    print(error)
    if not success:
      # We have reached our goal
      self.pose_sub = None # Kill the subscriber
      self.speed = 0.0 # Set speed to zero so car stops
      
    delta = self.compute_steering_angle(error)
    #print(delta)
    
    # Setup the control message -- yall did the easy stuff already :(
    ads = AckermannDriveStamped()
    ads.header.frame_id = '/map'
    ads.header.stamp = rospy.Time.now()
    ads.drive.steering_angle = delta
    ads.drive.speed = self.speed
    
    # Send the control message
    self.cmd_pub.publish(ads)

def main():
  rospy.init_node('line_follower', anonymous=True) # Initialize the node
  
  # Load these parameters from launch file
  # We provide suggested starting values of params, but you should
  # tune them to get the best performance for your system
  # Look at constructor of LineFollower class for description of each var
  # 'Default' values are ones that probably don't need to be changed (but you could for fun)
  # 'Starting' values are ones you should consider tuning for your system
  # YOUR CODE HERE
  plan_topic = '/planner_node/car_plan' # Default val: '/planner_node/car_plan'
  pose_topic = '/car/car_pose' # Default val: '/car/pose'
  plan_lookahead = 5 # Starting val: 5
  translation_weight = 0.9 # Starting val: 1.0
  rotation_weight = 0.1 # Starting val: 0.0
  kp = -3.0 # Startinig val: 1.0
  ki = 0.4 # Starting val: 0.0
  kd = -2.6 # Starting val: 0.0
  error_buff_length = 10 # Starting val: 10
  speed = 1.0 # Default val: 1.0

  raw_input("Press Enter when plan available...")  # Waits for ENTER key press
  
  # Use rospy.wait_for_message to get the plan msg
  # Convert the plan msg to a list of 3-element numpy arrays
  #     Each array is of the form [x,y,theta]
  # Create a LineFollower object
  # YOUR CODE HERE

  pose_array = rospy.wait_for_message(plan_topic, PoseArray) #wait until there is a PoseArray... but I dont think this is important since its regulated by when the user clicks enter. presumably the user will click the enter key AFTER they map a plan...
#  print(pose_array) #visualize
  
  #loop iterates the length of pose_array and stores the x,y,quat2angle(orientation) aka theta in a list for each pose in pose_array
  plan = [[]]*len(pose_array.poses) #create a list of lists that is of length x where x is the number of poses in the pose_array
  i = 0 #counter var
  for pose in pose_array.poses:
     plan[i] = [pose.position.x,pose.position.y,utils.quaternion_to_angle(pose.orientation)] #load index of plan with pose and angle info
     i += 1 #increment counter
  #for j in range(i): #for loop to check the correct value is being stored in the correct location.
     #print(plan[j][0], plan[j][1], plan[j][2])
#  test = plan.pop(0)
#  for j in range(i-1): #for loop to check the correct value is being stored in the correct location.
#     print(plan[j][0], plan[j][1], plan[j][2])
  
  #instance of the line follower class
  lf = LineFollower(plan, pose_topic, plan_lookahead, translation_weight, rotation_weight, kp, ki, kd, error_buff_length, speed)

  rospy.spin() # Prevents node from shutting down

if __name__ == '__main__':
  main()
