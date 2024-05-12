#!/usr/bin/env python
"""LMAO"""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import rospy
import msgs.msg as msg
from std_msgs.msg import Float64
from msgs.msg import Float32Stamped
import std_msgs
from math import sin
import time


def imagType():
  """LMAO"""
  rospy.init_node('float32_stamped_publisher', anonymous=True)
  # Ensure the topic name and message type match what subscribers expect
  pub = rospy.Publisher('/tool/pump_current', Float32Stamped, queue_size=10)
  rate = rospy.Rate(10)  # 10 Hz

  while not rospy.is_shutdown():
    # Create a new message instance
    data = Float32Stamped()
    data.header = std_msgs.mgs.Header()
    data.header.stamp = rospy.Time.now()
    data.header.frame_id = 'lmao'
    data.data = sin(float(time.time()))
    pub.publish(data)
    print('Saying: %.3E' % data.data)
    rate.sleep()


if __name__ == '__main__':
  try:
    imagType()
  except rospy.ROSInterruptException:
    pass
