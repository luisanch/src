#!/usr/bin/env python

import stopp
import numpy as np
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float64, Empty


class GenerateTraj:
  def __init__(self):
    # rospy.Subscriber("pid/depth/state", Float64, self.state_cb)
    rospy.Subscriber("enable_depth", Empty, self.start_trigger_cb)
    self.set_point_pub = rospy.Publisher("pid/depth/setpoint", Float64, queue_size=1)
    self.curr_depth = 0
    self.desired_depth = 0
    # self.got_state = False
    self.trigger = False
  
  def state_cb(self, msg):
    self.curr_depth = msg.data
    # self.got_state = True
  
  def start_trigger_cb(self, msg):
    self.trigger = True
  
def generate_traj(start, end, j_max=1, a_max=0.1, v_max=0.05, dt=0.004):
  robot_path = np.array([start, end])
  # print(robot_path)
  my_robot = stopp.Robot(
      n_joints=1, j_max=j_max, a_max=a_max, v_max=v_max)
  trajectory = my_robot.TimeParameterizePath(robot_path, interp_time_step=dt)

  traj = trajectory[0]

  t = traj.t
  pos = traj.pos
  vel = traj.vel
  acc = traj.acc
  
  plt.figure(1)
  plt.plot(t, pos)
  # plt.figure(2)
  plt.plot(t, vel)
  # plt.figure(3)
  # plt.plot(t, acc)
  plt.show()
  return t, pos, vel, acc 

if __name__ == "__main__":
  rospy.init_node("generate_traj")
  gen_traj = GenerateTraj()
  dt = 0.1
  total_time = 20.0
  desired_depth = 0.5
  j_max = 0.005
  a_max = 0.007
  v_max = 10
  i = 0
  start_pub = False
  while not rospy.is_shutdown():
    start = 0
    if gen_traj.trigger:
      t, pos, vel, acc = generate_traj(start, desired_depth, j_max=j_max, a_max=a_max,
                                      v_max=v_max, dt=dt)
      print(len(t))
      gen_traj.trigger = False
      start_pub = True
      i = 0
    if start_pub:
      set_point_msg = Float64(data=pos[i])
      print(set_point_msg)
      gen_traj.set_point_pub(set_point_msg)
      rospy.sleep(dt)
      i += 1
      i = min(i, len(pos) - 1)
