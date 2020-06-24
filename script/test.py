#!/usr/bin/env python

import rospy
from common_config.self_collect_config import say_it_works

if __name__ == '__main__':
	rospy.init_node('test_node')
	say_it_works()
