#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'barbanas'

import rospy
import rospkg
import json
import math
import os
from sys import maxint

from geometry_msgs.msg import Point, TwistStamped, Pose

import loader
import taems
from gpgp_agent.msg import TaskStructureUpdate, MissionCtrlMsg
from gpgp_agent.srv import AssessMissionTasks, ReassessMissionTasks, ClosestNeighbor, GetPosition, RemoveTask

rospack = rospkg.RosPack()


class TaskAssessor(object):

    def __init__(self):

        self.loader = loader.Loader()
        self.treeDict = {}
        self.label = rospy.get_param('~label').split(',')
        self.tasks = {}  # key : missionID, value : dictionary {key : task label (type), value : Task class instance}
        self.resources = {}  # TODO -- not implemented yet
        self.neighborLabels = {}  # key : missionID, value : dictionary {key : neighbor label (type), value : a list of neighbor addresses}
        self.neighbors = {}  # key : missionID, value : dictionary {key : address, value : [label(s), position - Point(), distance]}
        self.closestNeighbor = {}  # key : missionID, value : [a list of closest neighbors, a list of neighbor positions, index of neighbor with closest mutual bond]
        self.serviceTimeout = 0.2

        self.dataLocationPackage = rospy.get_param('~pack')

        rospy.Service("assessor_remove_task", RemoveTask, self.remove_task_srv)
        rospy.Service("assess_mission_tasks", AssessMissionTasks, self.assess_mission_srv)
        rospy.Service("reassess_mission_tasks", ReassessMissionTasks, self.reassess_mission_srv)
        rospy.Service("preprocess_mission_structure", ReassessMissionTasks, self.preprocess_mission_structure_srv)
        rospy.Service("closest_neighbor", ClosestNeighbor, self.closest_neighbor_srv)

        self.structureUpdatePub = rospy.Publisher('task_structure_update', TaskStructureUpdate, queue_size=1)

        rospy.Subscriber("mission_control", MissionCtrlMsg, self.msg_mission_ctrl_callback)

    def remove_task_srv(self, req):
        self.treeDict[req.mission_id].removeTaskAndSubtasks(req.task_label)
        return []

    def msg_mission_ctrl_callback(self, msg):
        """
        Callback function for "mission_control" topic.

        Args:
            msg (MissionCtrlMsg): Incoming message.
        """
        mission_sign = msg.root_task + "[" + str(msg.mission_id) + "]"

        if msg.type == "NewMission":
            if msg.mission_id not in self.treeDict.keys():
                err = self.load_tree(msg.mission_id, msg.root_task)
                if not err:
                    self.neighbors[mission_sign] = {}
                    self.neighborLabels[mission_sign] = {}
                    self.tasks[mission_sign] = {}
                    self.resources[mission_sign] = {}
                    rospy.loginfo('I got a new mission: %s', mission_sign)

        if msg.type == "Abort" or msg.type == "Restart" or msg.type == "Completed":
            if mission_sign in self.treeDict.keys():
                self.neighbors.pop(mission_sign)
                self.neighborLabels.pop(mission_sign)
                if msg.mission_id in self.closestNeighbor.keys():
                    self.closestNeighbor.pop(mission_sign)
                self.tasks.pop(mission_sign)
                self.resources.pop(mission_sign)
                self.treeDict.pop(mission_sign)

    def load_tree(self, mission_id, task_label):
        """Load taems tree structure for mission."""
        mission_sign = task_label + "[" + str(mission_id) + "]"

        if mission_sign not in self.treeDict.keys():
            dir = rospack.get_path(self.dataLocationPackage) + '/data/Missions/' + rospy.get_namespace()[1:]
            filename = mission_sign + ".taems"

            if os.path.isfile(dir + filename):
                tree = taems.TaemsTree()
                self.loader.parse(dir, filename, self.label, tree)
                if mission_sign not in self.treeDict:
                    self.treeDict[mission_sign] = tree
            else:
                return 1
        return 0

    def assess_mission_srv(self, req):
        """
        Mission assessment service.

        Performs mission task assessment and publishes results to 'task_structure_update' topic.
        """
        while req.mission_id not in self.treeDict.keys():
            rospy.sleep(0.1)

        # testme
        # for (label, address) in izip(req.neighbor_label, req.neighbor_address):
        #     # Fill out or add to neighborLabels {agent_label(type): [addresses of agents with that label]}
        #     if label not in self.neighborLabels[req.mission_id]:
        #         self.neighborLabels[req.mission_id][label] = [address]
        #     else:
        #         self.neighborLabels[req.mission_id][label].append(address)
        #     # Fill out or add to neighbors {address: [label(s), position - Point(), distance]}
        #     if address not in self.neighbors[req.mission_id]:
        #         self.neighbors[req.mission_id][address] = [[label], Point(), maxint] # TODO consider using named tuple
        #     else:
        #         self.neighbors[req.mission_id][address][0].append(label)

        for i in range(len(req.neighbor_label)):
            if req.neighbor_label[i] not in self.neighborLabels[req.mission_id]:
                self.neighborLabels[req.mission_id][req.neighbor_label[i]] = [req.neighbor_address[i]]
            else:
                self.neighborLabels[req.mission_id][req.neighbor_label[i]].append(req.neighbor_address[i])

            if req.neighbor_address[i] not in self.neighbors[req.mission_id]:
                self.neighbors[req.mission_id][req.neighbor_address[i]] = [[req.neighbor_label[i]], Point(), maxint]
            else:
                self.neighbors[req.mission_id][req.neighbor_address[i]][0].append(req.neighbor_label[i])

        # self.determine_resource_state(req.mission_id)

        # Assess given mission.
        # This call will assign values to self.tasks[req.mission_id][task label] (which is task_assessor.Task())
        # and self.treeDict[req.mission_id].tasks[task label] using properties of mutable Python objects.
        error = self.assess_mission(req.mission_id)
        if error != 0:
            return False

        task_outcomes = []
        start = []
        end = []
        target = []
        mass_class = []
        action_class = []

        # Fill out lists that are going to be used in TaskStructureUpdate message.
        for task in self.treeDict[req.mission_id].methodLabels:
            task_outcomes.append(self.treeDict[req.mission_id].tasks[task].outcome)
            if self.tasks[req.mission_id][task].start is None:
                start.append(Pose())
            else:
                start.append(self.tasks[req.mission_id][task].start)
            if self.tasks[req.mission_id][task].end is None:
                end.append(Pose())
            else:
                end.append(self.tasks[req.mission_id][task].end)
            if self.tasks[req.mission_id][task].movingTarget is None:
                target.append("")
            else:
                target.append(self.tasks[req.mission_id][task].movingTarget)
            # TODO -- euroc eval -- remove
            mass_class.append(self.tasks[req.mission_id][task].mass_class)
            action_class.append(self.tasks[req.mission_id][task].action_class)

        # Make a new TaskStructureUpdate message and fill out its fields.
        msg = TaskStructureUpdate()
        msg.mission_id = req.mission_id
        msg.root_task = self.treeDict[req.mission_id].rootTask[0]
        msg.task_label = self.treeDict[req.mission_id].methodLabels
        msg.json_outcome = json.dumps(task_outcomes)
        msg.start = start
        msg.end = end
        msg.target = target
        # TODO -- euroc eval -- remove
        msg.mass_class = mass_class
        msg.action_class = action_class

        resources = []
        resource_state = []
        for item in self.resources[req.mission_id]:
            resources.append(item[0])
            resource_state.append(item[1])

        msg.resource = resources
        msg.resource_state = resource_state

        # Finally, publish the message and return True to indicate success.
        self.structureUpdatePub.publish(msg)
        return True

    def reassess_mission_srv(self, req):
        """
        Mission reassessment service. This is service is used by coordinator after resolving complex redundancy.

        Returns:
            [no error flag, a list of tasks to be removed from agent's structure]
        """

        taskLabels = json.loads(req.neighbor_task_labels)
        neighborTaskLabels = {}
        for i in range(len(req.neighbor_address)):
            neighborTaskLabels[req.neighbor_address[i]] = taskLabels[i]

        error = self.reassess_mission(req.mission_id, neighborTaskLabels)
        # error[0] - completed flag, error[1] - a list of tasks to remove
        if error[0] != 0:
            return [False, []]
        else:
            for task in error[1]:
                self.treeDict[req.mission_id].removeTaskAndSubtasks(task)
            return [True, error[1]]

    def preprocess_mission_structure_srv(self, req):
        # FIXME gdje je definirana ova funkcija
        error = self.preprocess_task_structure(req.mission_id)
        # error[0] - completed flag, error[1] - a list of tasks to remove
        rospy.loginfo('Preprocess mission structure: error is %s', error)
        raw_input("")
        if error[0] != 0:
            return [False, []]
        else:
            for task in error[1]:
                self.treeDict[req.mission_id].removeTaskAndSubtasks(task)
            return [True, error[1]]

    def calc_neighbor_distances(self, missionID, neighborLabels):
        """
        Calculates distance between agent and every neighbor. And sorts them in ascending order.
        """

        neighborOrder = []  # ordered list of closest neighbors
        neighborDistances = []  # list of distances from neighbors in neighborOrder

        # TODO -- this position will be based on agent's location at the start of each mission
        rospy.wait_for_service('get_position')
        try:
            get_position = rospy.ServiceProxy('get_position', GetPosition)
            result = get_position()
            positionEstimate = result.position
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s", e)

        for label in neighborLabels:
            if label in self.neighborLabels[missionID].keys():
                # go through all neighbors with this label
                for i in range(len(self.neighborLabels[missionID][label])):
                    neighborAddress = self.neighborLabels[missionID][label][i]
                    neighbor = self.neighbors[missionID][neighborAddress]

                    if neighborAddress in neighborOrder:
                        continue
                    try:
                        rospy.wait_for_service(neighborAddress + 'get_position', 0.2)
                        get_position = rospy.ServiceProxy(neighborAddress + 'get_position', GetPosition)
                        result = get_position()

                        neighbor[1] = result.position
                        endPoint = Point(neighbor[1].x, neighbor[1].y, neighbor[1].z)
                        distance = self.assess_path(positionEstimate, endPoint)
                        self.neighbors[missionID][neighborAddress][2] = distance

                        j = 0
                        while True:
                            if j == len(neighborOrder):
                                neighborOrder.append(neighborAddress)
                                neighborDistances.append(distance)
                                break
                            if neighborDistances[j] > distance:
                                neighborOrder.insert(j, neighborAddress)
                                neighborDistances.insert(j, distance)
                                break
                            j += 1

                    # TODO -> what if neighbor service failed ?
                    except rospy.ServiceException, e:
                        rospy.logerr('Service call failed: %s', e)
                    except rospy.ROSException, e:
                        rospy.logerr('Service call failed: %s', e)

        self.closestNeighbor[missionID] = [neighborOrder, -1, 0]
        neighborPositions = []
        for item in neighborOrder:
            neighborPositions.append(self.neighbors[missionID][item][1])

        return [neighborOrder, neighborDistances, neighborPositions]

    def establish_closest_neighbor(self, missionID):
        """
        Goes through a list of neighbor agents and sends them ClosestNeighbor relationship request until one
        of them responds positively or end of the list is reached.
        """

        neighborOrder = self.closestNeighbor[missionID][0]
        i = self.closestNeighbor[missionID][2]
        while i < len(neighborOrder):
            # try to establish neighborhood
            rospy.wait_for_service(neighborOrder[i] + 'closest_neighbor')
            try:
                closest_neighbor = rospy.ServiceProxy(neighborOrder[i] + 'closest_neighbor', ClosestNeighbor)
                result = closest_neighbor(missionID, rospy.get_namespace(), "establish")
                if result.done is True:
                    self.closestNeighbor[missionID][1] = i
                    self.closestNeighbor[missionID][2] = i + 1
                    break
                else:
                    self.closestNeighbor[missionID][2] = i + 1
            except rospy.ServiceException, e:
                rospy.logerr('Service call failed: %s', e)

            i += 1

        # wait for the process to complete on all agents
        rospy.sleep(0.5)

    def closest_neighbor_srv(self, req):
        """
        A service that handles ClosestNeighbor relationship establishment or cancellation request.
        """

        while req.mission_id not in self.closestNeighbor.keys():
            pass

        # index of current closest neighbor
        index = self.closestNeighbor[req.mission_id][1]
        # index of next possible closest neighbor
        nextPossibleClosest = self.closestNeighbor[req.mission_id][2]

        if req.type == "establish":
            # if agent doesn't have closest neighbor yet
            if index == -1:
                if nextPossibleClosest < len(self.closestNeighbor[req.mission_id][0]):
                    # if agent is next possible closest neighbor
                    if req.agent_from == self.closestNeighbor[req.mission_id][0][nextPossibleClosest]:
                        self.closestNeighbor[req.mission_id][1] = nextPossibleClosest
                        return True
                    else:
                        return False
                else:
                    # if agent already sent out requests to all other agents
                    self.closestNeighbor[req.mission_id][1] = self.closestNeighbor[req.mission_id][0].index(req.agent_from)
                    return True

            # the connection is already established
            if self.closestNeighbor[req.mission_id][0][index] == req.agent_from:
                return True

            if self.closestNeighbor[req.mission_id][0].index(req.agent_from) < index:
                # if agent is closest than current closest neighbor
                currentClosestNeighbor = self.closestNeighbor[req.mission_id][0][index]
                # new closest neighbor
                self.closestNeighbor[req.mission_id][1] = index
                # cancel neighborhood with previous closest neighbor
                rospy.wait_for_service(currentClosestNeighbor + 'closest_neighbor_srv')
                try:
                    closest_neighbor = rospy.ServiceProxy(currentClosestNeighbor + 'closest_neighbor_srv', ClosestNeighbor)
                    closest_neighbor(req.mission_id, rospy.get_namespace(), "cancel")
                except rospy.ServiceException, e:
                    rospy.logerr('Service call failed: %s', e)
                return True
            else:
                return False

        elif req.type == "cancel":
            # remove closest neighbor and continue sending requests to remaining neighbors
            self.closestNeighbor[req.mission_id][1] = -1
            self.establish_closest_neighbor(req.mission_id)
            return True

    def assess_path(self, start, end):
        """
        Euclidean distance.
        """
        return math.sqrt(math.pow(end.x - start.x, 2) + math.pow(end.y - start.y, 2) + math.pow(end.z - start.z, 2))

    def reassess_mission(self, *args, **kwargs):
        raise NotImplementedError('Must override method reassess_mission!')

    def assess_mission(self, *args, **kwargs):
        raise NotImplementedError('Must override method assess_mission!')


class Task(object):

    def __init__(self, name, agent):
        self.name = name
        self.agent = agent
        self.start = Pose()  # start position
        self.end = Pose()  # end position
        self.movingTarget = None  # moving target can be added -> in such case, this attribute presents target's address (for calling get_position service)
        self.connectedTask = None  # agent which performs this task is this task's movingTarget
        self.obstacleHeight = -1
        self.pathLen = -1
        # TODO for euroc evaluation -- remove when not needed anymore
        self.mass_class = ""
        self.action_class = ""

    def __str__(self):
        str_buffer = []
        str_buffer.append('name: {}'.format(self.name))
        str_buffer.append('agent: {}'.format(self.agent))
        str_buffer.append('start:\n{}'.format(self.start))
        str_buffer.append('end:\n{}'.format(self.end))
        return '\n'.join(str_buffer)
