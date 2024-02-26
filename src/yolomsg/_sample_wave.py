"""Generate a sine wave value with added Gaussian noise."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from math import sin

from rospy import ROSInitException, init_node
from std_msgs.msg import Float64
import rospy
import os

os.environ['ROS_MASTER_URI'] = 'http://localhost:11311'


def main() -> None:
  """LOL"""
  publisher = rospy.Publisher('cunt', Float64,
                              queue_size=10)
  rate = rospy.Rate(50)  # In hertz, not waiting time
  c = 0
  F = [1, 4, 9, 16, 25, ]
  R = [1, 1 / 2, 1 / 6, 1 / 24, 1 / 120]
  while not rospy.is_shutdown():
    t = rospy.get_time()
    val = sum([r * sin(f * t) for (f, r) in zip(F, R)])
    rospy.loginfo(val)
    msg = Float64()
    msg.data = val
    publisher.publish(msg)
    rate.sleep()


if __name__ == '__main__':
  main()
