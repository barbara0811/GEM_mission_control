#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Initializes ROS node for GEM mission coordinator.

Summary:

    Publications:
         * /peer_discovery [gem_mission_control/HelloMsg]
         * /namespace/mission_enabled_tasks [gem_mission_control/MissionEnabledTasks]
         * /mission_control [gem_mission_control/MissionCtrlMsg]
         * /namespace/mission_control [gem_mission_control/MissionCtrlMsg]

    Subscriptions:
         * /gazebo/model_states [unknown type] ###TODO -- unified topic named /namespace/position
                                                that is the same for simulation and real system
         * /namespace/task_structure_update [gem_mission_control/TaskStructureUpdate]
         * /peer_discovery [gem_mission_control/HelloMsg]
         * /mission_control [gem_mission_control/MissionCtrlMsg]

    Services:
         * /namespace/task_info
         * /namespace/signal_abort
         * /namespace/schedule_ok
         * /namespace/mission_status
         * /namespace/register_executed_task
         * /namespace/resolve_redundant
         * /namespace/mission_info
         * /namespace/register_commitment
         * /namespace/register_feedback_request
         * /namespace/get_position

   Parameters:
         param name="pack" -- name of a package which contains mission specific data
         param name="label" -- list of agent labels in taems structure

    Arguments:
         arg name="name"
"""
__author__ = 'barbanas'

import rospy
import rospkg
import json
import time
import os
import sys
import signal
import numpy as np
from copy import deepcopy

from DTC_scheduler import Criteria
from gem_mission_control_lib.utilities import helper_functions, my_logger
from gem_mission_control_lib.modules.mission_assessor import MissionAssessor
from gem_mission_control.msg import HelloMsg, MissionCtrlMsg
from gem_mission_control.srv import MissionInfo, MissionStatus
from std_msgs.msg import Empty


logger = my_logger.CustomLogger()
rospack = rospkg.RosPack()

# def signal_handler(signal, frame):
#     print("\nprogram exiting gracefully")
#     sys.exit(0)

# signal.signal(signal.SIGINT, signal_handler)


class MissionCoordinator(object):

    def __init__(self):
        # Set the name for the custom logger.
        logger.name = rospy.get_name()

        self.serviceTimeout = 0.2
        self.mission_state = "init"
        self.assessor = MissionAssessor()

        # class is taems structure agent label => agent type
        self.classes = rospy.get_param('~class').split(',')
        self.ns = rospy.get_namespace()[1:-1]

        # Mission control publishers, subscribers and services.
        rospy.Subscriber("/mission_control", MissionCtrlMsg,
                         self.msg_mission_ctrl_callback)
        self.mission_ctrl_pub = rospy.Publisher(
            "/mission_control", MissionCtrlMsg, queue_size=1)
        self.mission_ctrl_local_pub = rospy.Publisher(
            "mission_control", MissionCtrlMsg, queue_size=2)

        # Coordination publishers, subscribers and services.
        self.broadcastHelloPub = rospy.Publisher(
            "/peer_discovery", HelloMsg, queue_size=5)
        rospy.Subscriber("/peer_discovery", HelloMsg, self.msg_hello_callback)

        rospy.Subscriber("done_planning", Empty, self.done_planning_cb)

        self.mission_state = "no_mission"
        self.init_mission_structures()

    def spin(self):
        self.coordinator_state_machine()

    def init_mission_structures(self):
        """Initialize important mission structures."""
        self.agents = []  # agent namespaces
        self.agent_classes = set()  # all agent classes
        self.mission_status = []  # mission status list for all agents
        self.mission_label = ""  # mission id -- initially no mission
        self.criteria = ""  # mission criteria filename

        self.done_planning = False

    def coordinator_state_machine(self):
        """ A state machine of the mission coordinator.
        """
        logger.info("initializing state machine")

        while not rospy.is_shutdown():

            if self.mission_state == "no_mission":
                logger.info("no mission")
                rospy.Rate(1.0).sleep()

            elif self.mission_state == "new_mission":
                # mission assessment and peer discovery phase
                logger.info(
                    '***NEW MISSION***:\n{id: %s, criteria: %s}\n', self.mission_label, self.criteria)
                # Detect neighborhood -> broadcast Hello messages.
                msg = HelloMsg()
                msg.ag_addr = self.ns
                msg.mission_label = self.mission_label
                msg.classes = self.classes
                self.broadcastHelloPub.publish(msg)
                rospy.sleep(1)

                # Perform initial mission assessment.
                logger.info('Performing initial mission assessment.')
                self.assessor.prepare_mission(self.mission_label, self.agent_classes)
                self.mission_state = "start_planning"

            elif self.mission_state == "start_planning":
                logger.info('Planning mission.')
                self.plan_mission()
                self.mission_state = "planning"

            elif self.mission_state == "planning":
                if self.done_planning:
                    self.mission_state = "finished"
                rospy.Rate(1.0).sleep()

            elif self.mission_state == "finished":
                logger.info(
                    '***FINISHED MISSION***:\n{id: %s, criteria: %s}\n', self.mission_label, self.criteria)
                self.init_mission_structures()
                print "OK............."
                self.mission_state = "no_mission"
            else:
                logger.warn("unknown mission state")
                rospy.Rate(1.0).sleep()

    # # ************************** #
    # # CALLBACK FUNCTIONS SECTION #
    # # ************************** #

    def msg_mission_ctrl_callback(self, msg):
        """
        Callback function for "/mission_control" topic.

        Args:
            msg (MissionCtrlMsg): Incoming message specifying mission data
        """
        while self.mission_state == "init":
            rospy.sleep(0.1)
        if msg.type == "NewMission":
            if self.mission_state == "no_mission":
                self.mission_state = "new_mission"
                self.mission_label = msg.root_task
                self.criteria = msg.criteria

        # if msg.type == "Abort":
        #     if msg.ag_addr == self.ns:
        #         return

        #     if mission_sign in self.missions.keys():
        #         logger.info('Got ABORT request from %s.', msg.ag_addr)
        #         logger.info('Aborting mission: %s', mission_sign)
        #         self.mission_status[mission_sign] = "abort"

        # if msg.type == "Restart":
        #     if msg.ag_addr == self.ns:
        #         return

        #     if mission_sign in self.missions.keys():
        #         logger.info('Got RESTART request from %s.', msg.ag_addr)
        #         logger.info('Restarting mission: %s', mission_sign)
        #         self.mission_status[mission_sign] = "restart"

        if msg.type == "Completed":

            if msg.ag_addr == self.ns:
                self.mission_status[mission_sign] = "completed"
                logger.info(
                    '***FINISHED MISSION***: %s[%s]\n', msg.root_task, msg.mission_id)

    def msg_hello_callback(self, msg):
        """
        Callback function for "/peer_discovery" topic.

        Args:
            msg (HelloMsg): Incoming message specifying agent's properties.
        """
        while msg.mission_label != self.mission_label:
            # Maybe the mission wasn't registered yet - wait a little bit.
            rospy.sleep(0.1)

        # register agent
        self.agents.append(msg.ag_addr)
        self.agent_classes.update(msg.classes)
        print self.agents

