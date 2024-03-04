"""Generate a sine wave value with added Gaussian noise."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from math import sin
import os
import sys
from icecream import ic

here = os.getcwd()
root = os.path.join(here, '..', )
root = os.path.normpath(root)
sys.path.append(root)

ic.configureOutput(includeContext=True)
ic(root)
from rospy import ROSInitException, init_node
from std_msgs.msg import Float64
import rospy

from ezros.rosutils import validateInitialized

os.environ['ROS_MASTER_URI'] = 'http://localhost:11311'


def main() -> None:
  """LOL"""
  nodeName = 'Publisher'
  init_node(nodeName, anonymous=False)
  publisher = rospy.Publisher('topic', Float64,
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
