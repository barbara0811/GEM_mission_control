#!/usr/bin/env python

"""
TODO: write node summary.
"""

__author__ = 'barbanas'

import sys
import os
import rospy
import rospkg
from gem_mission_control_lib.utilities import my_logger
from gem_mission_control_lib.taems import taems

rospack = rospkg.RosPack()
logger = my_logger.CustomLogger()


class MissionAssessor():

    def __init__(self):
        pack = rospy.get_param('~data_pack')
        data_dir = rospy.get_param('~data_dir')
        self.data_dir = os.path.join(rospack.get_path(pack), data_dir)

    def prepare_mission(self, mission_id):
        logger.warn(
            "Default assess mission method. Implement mission-specific task assessment.")
        # return empty tree
        tree = taems.TaemsTree()
        return tree
