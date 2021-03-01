#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Initializes ROS node for UMMC mission coordinator.

Summary:

    Publications:
         * /peer_discovery [ummc_agent/HelloMsg]
         * /namespace/mission_enabled_tasks [ummc_agent/MissionEnabledTasks]
         * /mission_control [ummc_agent/MissionCtrlMsg]
         * /namespace/mission_control [ummc_agent/MissionCtrlMsg]

    Subscriptions:
         * /gazebo/model_states [unknown type] ###TODO -- unified topic named /namespace/position
                                                that is the same for simulation and real system
         * /namespace/task_structure_update [ummc_agent/TaskStructureUpdate]
         * /peer_discovery [ummc_agent/HelloMsg]
         * /mission_control [ummc_agent/MissionCtrlMsg]

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
import numpy as np
from copy import deepcopy

from ummc_agent_lib.taems import taems
import loader
import simulator
from DTC_scheduler import Criteria
from utilities import helper_functions, my_logger
from branch_and_bound import BranchAndBoundOptimizer
from ummc_agent.msg import HelloMsg, MissionCtrlMsg, MissionEnabledTasks, TaskStructureUpdate
from ummc_agent.srv import AddCommitmentLocal, AddCommitmentNonLocal, AddNonLocalTask, AdjustScheduleTimes, \
    AssessMissionTasks, ExecuteTask, GetPose, GetPosition, MissionEarliestStart, MissionInfo, MissionStatus, \
    ReassessMissionTasks, RedundancySolution, RegisterFeedbackRequest, RemoveTask, Reschedule, RescheduleResponse, \
    ScheduleOK, SignalMissionAbort, TaskInfo, TaskInfoRequest, TaskOutcomeEV

logger = my_logger.CustomLogger()
rospack = rospkg.RosPack()

# noinspection PyMissingOrEmptyDocstring
class MissionCoordinator(object):

    def __init__(self):
        # Set the name for the custom logger.
        logger.name = rospy.get_name()

        self.serviceTimeout = 0.2

        self.label = rospy.get_param('~label').split(',')  # label is taems structure agent label => agent type
        self.ns = rospy.get_namespace()
        self.missionCaller = {}
        self.missionType = {}
        self.pose_estimate = None

        self.startTime = {}

        # Task structure.
        self.treePathDict = None
        self.dataLocationPackage = rospy.get_param('~pack')
        self.treeDict = {}  # key : mission mission ID, value : TaemsTree instance
        self.treeDictScheduled = {}  # key : mission mission ID, value : TaemsTree instance, IR effects of schedule are applied to this structure
        self.taskOutcome = {}  # outcome for scheduled tasks, key: mission ID, value: dictionary {key: task label, value: [quality_distribution, duration_distribution, cost_distribution}
        self.taskOutcomeEVNonSched = {}  # expected value for q, d, c of non-scheduled tasks, key: mission ID, value: dictionary {key: task label, value: [qEV, dEV, cEV]}
        self.missions = {}  # key: mission ID, value : task label
        self.criteria = {}  # key: mission ID, value : Criteria object
        self.criteriaFilename = {}  # key: mission mission ID, value : criteria filename
        self.waitTaskAssessmentFlag = {}
        self.waitMissionHandleRedundancy = {}

        # Schedules.
        self.bestSchedule = {}  # key : mission ID, value : schedule for this mission - schedule: [[method1, start_time1, end_time1], [method2, start_time2, end_time2], ... ]
        self.completedTasks = {}  # key : mission ID, value : list of completed tasks in a mission
        self.scheduledMethods = {}  # key : mission ID, value : list of methods in schedule
        self.waitRedundant = {}  # key: mission ID, value: number of redundant tasks to be resolved
        self.taskEndTime = {}  # # key: mission ID, value: dictionary {key: task label, value : task end time in mission schedule}
        self.completedNonLocal = {}

        # Coordination.
        self.neighbors = {}  # key : mission ID, value: dictionary {key: neighbor address (namespace), value: Agent class instance}
        self.parentChildCRs = {}  # key: mission ID, value: dictionary {key: non local task, value: [neighbor label, neighbor address]}
        self.commitmentsLocal = {}  # key: mission ID, value: LocalCommitment object
        self.commitmentsNonLocal = {}  # key: mission ID, value: NonLocalCommitment object
        self.redundantTasks = {}  # key: mission ID, value: dictionary {key: task label, value: neighbor address}
        self.responsibleFor = {}  # key: mission ID, value: task label -> redundant task whose resolution this agent is responsible for
        self.missionStatus = {}  # key: mission ID, value: mission status, string
        self.scheduleIsCoordinated = {}  # key: mission ID, value: boolean, true if the schedule is coordinated
        self.hardConstrainedNeighbors = {}  # key: mission ID, value: a list of neighbors with hard constrained coordination relationships
        self.waitForScheduleOK = {}  # key: mission ID, value: a list of neighbors whose schedule_ok signal agent is waiting for
        self.waitComplexRedundancy = {}
        self.feedbackRequestList = {}

        # Execution.
        self.simulator = {}

        # Task structure publishers, subscribers and services.
        self.loader = loader.Loader()
        rospy.Subscriber("task_structure_update", TaskStructureUpdate, self.update_task_structure_callback)

        # Mission control publishers, subscribers and services.
        rospy.Subscriber("/mission_control", MissionCtrlMsg, self.msg_mission_ctrl_callback)
        self.missionCtrlMsgPub = rospy.Publisher("/mission_control", MissionCtrlMsg, queue_size=1)
        self.missionCtrlMsgLocalPub = rospy.Publisher("mission_control", MissionCtrlMsg, queue_size=2)

        # Coordination publishers, subscribers and services.
        self.broadcastHelloPub = rospy.Publisher("/peer_discovery", HelloMsg, queue_size=5)
        rospy.Subscriber("/peer_discovery", HelloMsg, self.msg_hello_callback)
        rospy.Service("task_info", TaskInfo, self.task_info_srv)
        rospy.Service("mission_info", MissionInfo, self.mission_info_srv)
        rospy.Service("mission_status", MissionStatus, self.mission_status_srv)
        rospy.Service("register_feedback_request", RegisterFeedbackRequest, self.register_feedback_request_srv)

        rospy.Service("resolve_redundant", RedundancySolution, self.resolve_redundant_srv)
        rospy.Service("register_commitment", AddCommitmentNonLocal, self.add_commitment_non_local_srv)
        rospy.Service("schedule_ok", ScheduleOK, self.register_schedule_ok_srv)

        rospy.Service("signal_abort", SignalMissionAbort, self.register_abort_srv)
        rospy.Service("get_position", GetPosition, self.get_position_srv)
        rospy.Service("get_pose", GetPose, self.get_pose_srv)

        # Execution publishers, subscribers and services.
        self.missionEnabledTasksPub = rospy.Publisher("mission_enabled_tasks", MissionEnabledTasks, queue_size=1)
        rospy.Service("register_executed_task", ExecuteTask, self.register_executed_task_srv)
        rospy.spin()  #TODO remove this

    def load_tree(self, mission_id, task_label):
        """
        Load taems structure for the mission.

        Args:
            mission_id (int): ID of the mission.
            task_label (str): Name of the mission (should be the same as root task).

        Returns:
            1 for success, 0 for failure.
        """
        mission_sign = task_label + "[" + str(mission_id) + "]"

        if mission_sign not in self.treeDict.keys():
            directory = rospack.get_path(self.dataLocationPackage) + '/data/Missions/' + self.ns[1:]
            filename = mission_sign + ".taems"

            rospy.loginfo('Loading taems file: %s', directory + filename)
            if os.path.isfile(directory + filename):
                tree = taems.TaemsTree()
                self.loader.parse(directory, filename, self.label, tree)

                # todo - check when this can happen??
                # if mission_sign not in self.treeDict.keys():
                self.treeDict[mission_sign] = tree
                # else:
                #    self.treeDict[mission_sign] = taems.TaemsTree.merge(self.treeDict[mission_id], tree)
            else:
                return 0
        return 1

    def load_criteria(self, mission_sign, criteria_filename):
        """
        Load criteria file for the mission.

        Args:
            mission_sign (str): Combination of mission name and ID.
            criteria_filename (str): Name of the file with criteria spec.
        """
        self.criteria[mission_sign] = Criteria()
        criteria_path = rospack.get_path(self.dataLocationPackage) + '/data/Criteria/' + criteria_filename
        rospy.loginfo('Loading criteria file: %s', criteria_path)
        if os.path.isfile(criteria_path):
            self.criteria[mission_sign].load(criteria_path)
            return 1
        else:
            return 0

    def init_mission_structures(self, mission_sign):
        """Initialize important mission structures."""

        self.missionStatus[mission_sign] = "start"
        self.neighbors[mission_sign] = {}

        self.parentChildCRs[mission_sign] = {}
        self.commitmentsLocal[mission_sign] = {}
        self.commitmentsNonLocal[mission_sign] = {}
        self.redundantTasks[mission_sign] = {}
        self.responsibleFor[mission_sign] = set()
        self.feedbackRequestList[mission_sign] = {}

        self.waitRedundant[mission_sign] = set()
        self.hardConstrainedNeighbors[mission_sign] = [set(), set()]  # [outgoing, incoming]
        self.waitForScheduleOK[mission_sign] = set()
        self.waitComplexRedundancy[mission_sign] = False
        self.scheduleIsCoordinated[mission_sign] = False

        self.simulator[mission_sign] = None

    def abort_mission(self, mission_sign, restart):
        """Reset mission structures in case of an abort."""

        self.missionStatus[mission_sign] = "no_mission"
        msg = MissionCtrlMsg()
        if restart:
            msg.type = "Restart"
        else:
            msg.type = "Abort"
        msg.ag_addr = self.ns
        msg.mission_id = int(mission_sign.split("[")[1][:-1])
        msg.root_task = ""
        msg.criteria = ""

        self.missionCtrlMsgLocalPub.publish(msg)

        self.missions.pop(mission_sign)

        self.neighbors.pop(mission_sign)
        self.parentChildCRs.pop(mission_sign)
        self.commitmentsLocal.pop(mission_sign)
        self.commitmentsNonLocal.pop(mission_sign)
        self.redundantTasks.pop(mission_sign)
        self.responsibleFor.pop(mission_sign)
        self.feedbackRequestList.pop(mission_sign)

        if restart is False:
            if mission_sign in self.treeDict.keys():
                self.treeDict.pop(mission_sign)
        if mission_sign in self.treeDictScheduled.keys():
            self.treeDictScheduled.pop(mission_sign)
        if mission_sign in self.criteria.keys():
            self.criteria.pop(mission_sign)

        if mission_sign in self.bestSchedule.keys():
            self.bestSchedule.pop(mission_sign)
        if mission_sign in self.completedTasks.keys():
            self.completedTasks.pop(mission_sign)
        if mission_sign in self.completedNonLocal:
            self.completedNonLocal.pop(mission_sign)
        if mission_sign in self.scheduledMethods.keys():
            self.scheduledMethods.pop(mission_sign)

        if mission_sign in self.taskOutcomeEVNonSched.keys():
            self.taskOutcomeEVNonSched.pop(mission_sign)
        if mission_sign in self.taskEndTime.keys():
            self.taskEndTime.pop(mission_sign)

        self.hardConstrainedNeighbors.pop(mission_sign)
        self.waitForScheduleOK.pop(mission_sign)
        self.scheduleIsCoordinated.pop(mission_sign)
        self.waitComplexRedundancy.pop(mission_sign)

        self.simulator.pop(mission_sign)

        if restart:
            rospy.loginfo('***RESTARTED MISSION***: %s\n', mission_sign)
        else:
            rospy.loginfo('***CANCELED MISSION***: %s\n', mission_sign)

    def start_new_mission(self, mission_id, root_task, criteria):
        """
        Start a new mission with given root task and criteria.

        Args:
            mission_id (int): ID of the mission.
            root_task (str): Label of the root task.
            criteria (str): Name of the file with criteria spec.
        """
        mission_sign = root_task + "[" + str(mission_id) + "]"
        self.startTime[mission_sign] = rospy.get_time()
        print "..."
        rospy.loginfo('***NEW MISSION***:\n{id: %s, task: %s, criteria: %s}\n', mission_sign, root_task, criteria)
        self.init_mission_structures(mission_sign)

        raw_input("##")
        return
        # Send mission control message to agent's nodes.
        msg = MissionCtrlMsg()
        msg.type = "StartMission"
        msg.ag_addr = self.missionCaller[mission_sign]
        msg.mission_id = mission_id
        msg.root_task = root_task
        msg.criteria = criteria
        self.missionCtrlMsgLocalPub.publish(msg)

        self.missions[mission_sign] = root_task
        self.criteriaFilename[mission_sign] = criteria

        # Load taems tree.
        if not self.load_tree(mission_id, root_task):
            rospy.logerr('The agent does not have the taems tree structure for task %s.', root_task)
            self.missionStatus[mission_sign] = 'no_schedule'
            self.abort_mission(mission_sign, restart=False)
            return

        # Load criteria.
        if not self.load_criteria(mission_sign, criteria):
            rospy.logerr('Criteria specification file \"%s\" does not exist. Aborting.', criteria)
            self.missionStatus[mission_sign] = 'no_schedule'
            self.abort_mission(mission_sign, restart=False)
            return

        # Detect neighborhood -> broadcast Hello messages.
        helloMsg = HelloMsg()
        helloMsg.ag_name = self.label
        helloMsg.ag_addr = self.ns
        helloMsg.mission_sign = mission_sign
        helloMsg.task_labels = self.treeDict[mission_sign].tasks.keys()
        helloMsg.neighbors = self.neighbors[mission_sign].keys()
        self.broadcastHelloPub.publish(helloMsg)

        # Wait for incoming Hello messages from other agents.
        rospy.sleep(2.0)

        # Perform initial mission assessment.
        rospy.loginfo('Performing initial mission assessment.')
        self.waitTaskAssessmentFlag[mission_sign] = True
        result = self.mission_assess_request(mission_sign)
        if result is False:
            self.missionStatus[mission_sign] = "abort"
            self.abort_mission(mission_sign, False)
            return
        while self.waitTaskAssessmentFlag[mission_sign]:
            rospy.sleep(0.1)
        if self.check_failed(mission_sign, set()):
            return

        # Get unscheduled task outcomes from scheduler.
        self.get_unscheduled_task_outcome_ev(mission_sign)

        # STARTED COORDINATION - Agents exchange their view of current mission structure.
        # --------------------
        rospy.loginfo('Mission status: ***STARTED COORDINATION***')
        self.missionStatus[mission_sign] = "started_coordination"
        self.wait_for_neighborhood_state(mission_sign, "started_coordination")
        if self.check_failed(mission_sign, set()):
            return

        # WUT? Sto je s ovim
        # rospy.wait_for_service('preprocess_mission_structure')
        # try:
        #     preprocess = rospy.ServiceProxy('preprocess_mission_structure', ReassessMissionTasks)
        #     result = preprocess(missionID, [], "")
        #     if result.done == 0:
        #         print "to remove"
        #         print result.to_remove
        #         for task in result.to_remove:
        #             self.remove_task(missionID, task)
        # except rospy.ServiceException, e:
        #     rospy.logerr('Service call failed: %s', e)
        # raw_input("..")

        rospy.sleep(0.2)

        # Detect parent-child coordination relationships.
        self.detect_parent_children_CRs(self.treeDict[mission_sign].rootTask[0], mission_sign)

        # Assess outcome of all non local tasks.
        missingNonLocalTasks = self.assess_non_local_tasks(mission_sign)
        if self.check_failed(mission_sign, set()):
            return

        if len(missingNonLocalTasks) > 0:
            rospy.logerr('Can\'t schedule mission! Agents performing tasks %s are missing!\n',
                         list(missingNonLocalTasks))
            # TODO -- abort mission in this case??
            self.initialize_mission_rescheduling(mission_sign)
            return

        # INITIAL TASK ASSESSMENT - Resolve complex redundancy.
        # -----------------------
        rospy.loginfo('Mission status: ***INITIAL TASK ASSESSMENT***')
        self.missionStatus[mission_sign] = "initial_task_assessment"
        self.wait_for_neighborhood_state(mission_sign, "initial_task_assessment")
        if self.check_failed(mission_sign, set()):
            return

        rospy.sleep(0.2)

        # TODO: replace with only one code block and function
        # Check for complex mission. Resolve complex redundancy
        if "q_max_all" in self.treeDict[mission_sign].tasks[self.treeDict[mission_sign].rootTask[0]].qaf_local and \
           self.treeDict[mission_sign].tasks[self.treeDict[mission_sign].rootTask[0]].qaf in ["q_sum_all", "q_sum"]:
            self.waitComplexRedundancy[mission_sign] = True
            agents = self.handle_mission_redundancy(mission_sign, True)
            if agents == -1:
                self.missionStatus[mission_sign] = "abort"
                self.abort_mission(mission_sign, False)
                return
            self.waitComplexRedundancy[mission_sign] = False

        # Check for complex redundancy within tasks and resolve.
        for task in self.treeDict[mission_sign].tasks.values():
            if type(task) is taems.TaskGroup:
                if task.qaf in ['q_sum_all', 'q_sum'] and task.qaf_local == 'q_max':
                    logger.info('Found task with complex redundancy: %s', task)
                    self.waitComplexRedundancy[mission_sign] = True
                    agents = self.handle_complex_redundancy(mission_sign, task.label, True)
                    if agents == -1:
                        self.missionStatus[mission_sign] = "abort"
                        self.abort_mission(mission_sign, False)
                        return
        self.waitComplexRedundancy[mission_sign] = False

        # While resolving complex redundancy, some agents remove tasks from their mission structure.
        # We need to update each agent's view of the world.
        self.update_neighborhood_info(mission_sign)

        # Reassess mission following complex redundancy resolving.
        result = self.mission_reassess_request(mission_sign)
        if result is False:
            self.missionStatus[mission_sign] = "abort"
            self.abort_mission(mission_sign, False)
            return

        # Get unscheduled task outcomes from scheduler.
        self.get_unscheduled_task_outcome_ev(mission_sign)
        if self.check_failed(mission_sign, set()):
            return

        # RESOLVED_COMPLEX_REDUNDANCY - Update the world state so everyone is on the same page.
        # ---------------------------
        rospy.loginfo('Mission status: ***RESOLVED COMPLEX REDUNDANCY***')
        self.missionStatus[mission_sign] = "resolved_complex_redundancy"
        self.wait_for_neighborhood_state(mission_sign, "resolved_complex_redundancy")
        if self.check_failed(mission_sign, set()):
            return

        rospy.sleep(0.2)

        # Update neighborhood status - some agents may have removed tasks from their mission structure.
        self.update_neighborhood_info(mission_sign)

        # Get unscheduled task outcomes from scheduler.
        self.get_unscheduled_task_outcome_ev(mission_sign)
        if self.check_failed(mission_sign, set()):
            return

        # READY_TO_SCHEDULE - Agents are ready for making the first schedule. This is a job for DTC scheduler.
        # -----------------
        rospy.loginfo('Mission status: ***READY TO SCHEDULE***')
        self.missionStatus[mission_sign] = "ready_to_schedule"
        self.wait_for_neighborhood_state(mission_sign, "ready_to_schedule")

        # Update parent-child coordination relationships.
        self.parentChildCRs[mission_sign] = {}
        self.detect_parent_children_CRs(self.treeDict[mission_sign].rootTask[0], mission_sign)

        # Assess outcome of all non local tasks.
        missingNonLocalTasks = self.assess_non_local_tasks(mission_sign)
        if self.check_failed(mission_sign, set()):
            return
        if len(missingNonLocalTasks) > 0:
            rospy.logerr('Can\'t schedule mission, agents performing tasks %s are missing!\n',
                         list(missingNonLocalTasks))
            self.initialize_mission_rescheduling(mission_sign)
            return

        # Get the first schedule from DTC scheduler.
        rospy.sleep(0.1)
        result = self.reschedule_request(mission_sign)
        if result == RescheduleResponse.ERROR_1:
            self.missionStatus[mission_sign] = "abort"
            self.abort_mission(mission_sign, False)
            return
        if result == RescheduleResponse.ERROR_2:
            self.missionStatus[mission_sign] = "abort"
            self.abort_mission(mission_sign, False)
            return
        if result == RescheduleResponse.NO_SCHEDULE:
            self.missionStatus[mission_sign] = "no_schedule"
            self.abort_mission(mission_sign, False)
            return

        # INITIAL_SCHEDULE - Update the view of the world with schedules of other agents and resolve simple redundancy.
        # ----------------
        rospy.loginfo('Mission status: ***INITIAL SCHEDULE***')
        self.missionStatus[mission_sign] = "initial_schedule"
        self.wait_for_neighborhood_state(mission_sign, "initial_schedule")
        if self.check_failed(mission_sign, set()):
            return

        # Update neighborhood status - we need updated view of the world to resolve redundancy.
        self.update_neighborhood_info(mission_sign)
        if self.check_failed(mission_sign, set()):
            return

        # Detect redundant tasks.
        self.waitRedundant[mission_sign] = []
        self.detect_redundant_tasks(self.treeDictScheduled[mission_sign].rootTask[0], mission_sign)
        rospy.sleep(0.2)
        rospy.loginfo('Detected redundant tasks: %s', self.redundantTasks[mission_sign])
        logger.debug('Waiting for other agents to resolve: %s', self.waitRedundant[mission_sign])

        missingNonLocalTasks = []
        for cr in self.parentChildCRs[mission_sign].values():
            if cr.count == 0 and cr.isMandatory:
                missingNonLocalTasks.append(cr.taskTo)
        if len(missingNonLocalTasks) > 0:
            rospy.logerr('Can\'t schedule mission, agents performing tasks %s are missing!\n',
                         list(missingNonLocalTasks))
            self.initialize_mission_rescheduling(mission_sign)
            return

        # Share information about redundancy between agents.
        failed = self.communicate_redundancy(mission_sign)
        if self.check_failed(mission_sign, failed):
            return

        recheck = False
        timeout = time.time() + 0.5
        while len(self.waitRedundant[mission_sign]) != 0:
            if time.time() > timeout:
                recheck = True
            if recheck:
                recheck = False
                failed = self.update_neighborhood_info(mission_sign)
                if self.check_failed(mission_sign, failed & set(self.waitRedundant[mission_sign])):
                    return
                timeout = time.time() + 0.5

        # If agent aborted mission.
        if self.check_failed(mission_sign, set()):
            return

        # REMOVED_REDUNDANT - Reassess all non local tasks and make a new schedule with updated view of the world.
        # -----------------
        rospy.loginfo('Mission status: ***REMOVED REDUNDANT***')
        self.missionStatus[mission_sign] = "removed_redundant"
        self.wait_for_neighborhood_state(mission_sign, "removed_redundant")

        # Update neighborhood status - agents have removed redundant tasks or dismissed mission.
        self.update_neighborhood_info(mission_sign)
        if self.check_failed(mission_sign, set()):
            return

        rospy.sleep(0.1)

        # Update parent-child coordination relationships.
        self.parentChildCRs[mission_sign] = {}
        self.detect_parent_children_CRs(self.treeDict[mission_sign].rootTask[0], mission_sign)

        # Assess outcome of all non local tasks.
        missingNonLocalTasks = self.assess_non_local_tasks(mission_sign)
        if self.check_failed(mission_sign, set()):
            return
        if len(missingNonLocalTasks) > 0:
            rospy.logerr('Can\'t schedule mission, agents performing tasks %s are missing!\n',
                         list(missingNonLocalTasks))
            self.initialize_mission_rescheduling(mission_sign)
            return

        # Send a request for rescheduling to DTC scheduler.
        result = self.reschedule_request(mission_sign)
        if result == RescheduleResponse.ERROR_1:
            self.missionStatus[mission_sign] = "abort"
            self.abort_mission(mission_sign, False)
            return
        if result == RescheduleResponse.ERROR_2:
            self.missionStatus[mission_sign] = "abort"
            self.initialize_mission_rescheduling(mission_sign)
            return
        if result == RescheduleResponse.NO_SCHEDULE:
            self.missionStatus[mission_sign] = "no_schedule"
            self.abort_mission(mission_sign, False)
            return
        if self.check_failed(mission_sign, set()):
            return

        # UNCOORDINATED - Each agent has its local schedule. Schedules are coordinated and hard constraints are removed.
        # -------------
        rospy.loginfo('Mission status: ***UNCOORDINATED***')
        self.missionStatus[mission_sign] = "uncoordinated"
        failed = self.wait_for_neighborhood_state(mission_sign, "uncoordinated")
        if self.check_failed(mission_sign, set()):
            return

        rospy.sleep(0.1)

        # Update neighborhood status - agents have their uncoordinated local schedules.
        self.update_neighborhood_info(mission_sign)
        if self.check_failed(mission_sign, set()):
            return

        uncoordinated = [[x[0], round(x[1], 3), round(x[2], 3)] for x in self.bestSchedule[mission_sign]]
        rospy.loginfo('Uncoordinated schedule:\n%s\n%s\n', uncoordinated, '-' * 20)

        # TODO: remove?
        subcultron = False
        if subcultron:
            self.schedPub.publish(msg)
        else:
            # Override this part for subcultron simulation.
            self.commitmentsLocal[mission_sign] = {}
            self.commitmentsNonLocal[mission_sign] = {}
            self.detect_precedence_constraint_CRs(mission_sign)

            # Coordinate hard constraints.
            failed = self.coordinate_hard_constrained_tasks(mission_sign)
            if self.check_failed(mission_sign, failed):
                return

            for task in self.treeDict[mission_sign].tasks.values():
                if task.subtasks is None:
                    if task.nonLocal:
                        for neighbor in self.neighbors[mission_sign].keys():
                            if task.label in self.neighbors[mission_sign][neighbor].scheduledMethods or task.label in \
                                    self.neighbors[mission_sign][neighbor].completedTasks:
                                try:
                                    rospy.wait_for_service(neighbor + 'register_feedback_request', self.serviceTimeout)
                                    request_feedback = rospy.ServiceProxy(neighbor + 'register_feedback_request',
                                                                          RegisterFeedbackRequest)
                                    request_feedback(mission_sign, rospy.get_namespace(), task.label)
                                except (rospy.ServiceException, rospy.ROSException) as e:
                                    rospy.logerr('Service call failed: %s', e)

            self.hardConstrainedNeighbors[mission_sign][1] = set()

            # READY_FOR_MISSION_EXECUTION - All agents have final versions of their schedules. Wait for execution.
            # ---------------------------
            rospy.loginfo('Mission status: ***READY FOR MISSION EXECUTION***')
            self.missionStatus[mission_sign] = "ready_for_mission_execution"
            error = self.check_neighborhood_state(mission_sign, "ready_for_mission_execution")
            while error == -1:
                if self.scheduleIsCoordinated[mission_sign] is False:
                    failed = self.coordinate_hard_constrained_tasks(mission_sign)
                    if self.check_failed(mission_sign, failed):
                        return
                    self.hardConstrainedNeighbors[mission_sign][1] = set()
                    self.missionStatus[mission_sign] = "ready_for_mission_execution"
                rospy.sleep(0.1)
                error = self.check_neighborhood_state(mission_sign, "ready_for_mission_execution")

            rospy.sleep(0.2)

            # TODO: replace numbers with something more meaningful
            if error == -2:
                self.missionStatus[mission_sign] = "restart"
            if self.check_failed(mission_sign, failed):
                return

            finalSchedule = [[x[0], round(x[1], 3), round(x[2], 3)] for x in self.bestSchedule[mission_sign]]
            rospy.loginfo('Final schedule:\n%s\n%s\n', finalSchedule, '-' * 20)

            # Initialize simulator.
            self.simulator[mission_sign] = simulator.LightSimulator(self.treeDict[mission_sign])
            tasksToComplete = deepcopy(self.completedTasks[mission_sign])
            tasksToComplete.extend(self.scheduledMethods[mission_sign])
            tasksToComplete.extend(self.completedNonLocal[mission_sign])

            # TODO: vidi TODO u simulator init_disablemants.
            self.simulator[mission_sign].init_disablements(self.scheduledMethods[mission_sign], tasksToComplete)

        # EXECUTING - All agents start mission execution.
        # ---------
        if self.send_schedule_ok_execute(mission_sign):
            self.missionStatus[mission_sign] = "executing"
        else:
            self.initialize_mission_rescheduling(mission_sign)
            return

        # Total time elapsed from planning start.
        rospy.loginfo('Total time elapsed: %s\n', rospy.get_time() - self.startTime[mission_sign])

    def coordinate_hard_constrained_tasks(self, mission_sign):

        commitList = self.commitmentsLocal[mission_sign].keys()
        sendOKTo = set()

        while True:
            [failed, communicatedTo] = self.communicate_precedence_constraint_CRs(mission_sign, commitList)
            if len(failed) > 0:
                return failed

            rospy.sleep(0.3)

            hardConstrained = communicatedTo | self.hardConstrainedNeighbors[mission_sign][1]
            sendOKTo = sendOKTo | hardConstrained

            self.missionStatus[mission_sign] = "committed"
            failed = self.wait_for_neighborhood_state(mission_sign, "committed", list(hardConstrained))
            if len(failed) > 0:
                return failed

            rospy.sleep(0.3)

            result = self.reschedule_request(mission_sign)
            if result is False:
                self.initialize_mission_rescheduling(mission_sign)
                return
            if self.check_failed(mission_sign, set()):
                return

            commitList = self.check_local_commitments(mission_sign)
            if len(commitList) == 0:
                self.scheduleIsCoordinated[mission_sign] = True
            self.waitForScheduleOK[mission_sign] = deepcopy(hardConstrained)
            self.hardConstrainedNeighbors[mission_sign][1] = set()
            self.missionStatus[mission_sign] = "scheduled"
            failed = self.wait_for_neighborhood_state(mission_sign, "scheduled", list(hardConstrained))

            if len(failed) > 0:
                return failed

            rospy.sleep(0.3)

            if self.scheduleIsCoordinated[mission_sign]:
                rospy.sleep(0.2)
                self.missionStatus[mission_sign] = "coordinated"
                failed = set()
                logger.debug('Coordinated hard constrained tasks. Waiting for %s', self.waitForScheduleOK[mission_sign])
                temp = list(sendOKTo)
                for neighbor in temp:
                    try:
                        rospy.wait_for_service(neighbor + 'schedule_ok', self.serviceTimeout)
                        sched_ok = rospy.ServiceProxy(neighbor + 'schedule_ok', ScheduleOK)
                        sched_ok(mission_sign, rospy.get_namespace())
                        sendOKTo.remove(neighbor)
                    except (rospy.ServiceException, rospy.ROSException) as e:
                        rospy.logerr('Service call failed: %s', e)
                        failed.add(neighbor)
                        self.remove_neighbor(mission_sign, neighbor)

                if len(failed) > 0:
                    return failed

                recheck_interval = 0.5
                timeout = time.time() + recheck_interval
                while len(self.waitForScheduleOK[mission_sign]) > 0:
                    if self.scheduleIsCoordinated[mission_sign] is False:
                        break

                    if time.time() > timeout:
                        failed = self.update_neighborhood_info(mission_sign)
                        if len(failed) > 0:
                            return failed
                        timeout = time.time() + recheck_interval

                if self.scheduleIsCoordinated[mission_sign] is False:
                    continue

                return set()

    def check_failed(self, mission_sign, failed):
        """
        Check for mission failure.

        Check whether the mission status was set to "abort" or "restart" during
        execution. In those cases, abort the mission. If set of agents that
        failed to communicate their status is given and is not empty,
        initialize mission rescheduling.

        Args:
            mission_sign (str): Name and ID of the mission.
            failed (set[str]): Set of failed agents.

        Returns:
            True if the check confirmed mission failure.
        """

        if self.missionStatus[mission_sign] == "abort":
            self.abort_mission(mission_sign, False)
            return True

        if self.missionStatus[mission_sign] == "restart":
            self.abort_mission(mission_sign, True)
            return True

        if failed:
            self.missionStatus[mission_sign] = "failed"
            rospy.logerr('Neighbours %s are not accessible', list(failed))
            self.initialize_mission_rescheduling(mission_sign)
            return True

    def initialize_mission_rescheduling(self, mission_sign):
        """
        Initialize mission rescheduling.

        Args:
            mission_sign (str): Name and ID of the mission.
        """

        task = self.missions[mission_sign]
        criteria = self.criteriaFilename[mission_sign]

        rospy.loginfo('Initializing mission rescheduling...\n')
        msg = MissionCtrlMsg()
        msg.type = "Restart"
        msg.ag_addr = self.ns
        msg.mission_id = int(mission_sign.split('[')[1][:-1])
        msg.root_task = ""
        msg.criteria = ""

        self.missionCtrlMsgPub.publish(msg)

        # Simulate mission abort and wait for all other agents to abort mission.
        self.missionStatus[mission_sign] = "no_mission"
        self.wait_for_neighborhood_state(mission_sign, "no_mission")
        self.abort_mission(mission_sign, True)

        msg = MissionCtrlMsg()
        msg.type = "StartMission"
        msg.ag_addr = self.missionCaller[mission_sign]
        msg.mission_id = int(mission_sign.split('[')[1][:-1])
        msg.root_task = task
        msg.criteria = criteria  # Reschedule with the same criteria.

        self.missionCtrlMsgPub.publish(msg)

    def wait_for_neighborhood_state(self, mission_sign, status, neighborList=None):
        """
        Wait until all agents get to the same mission state.

        Args:
            mission_sign (str): Name and ID of the mission.
            status (str): Mission status.
            neighborList (list[str]): List of name of neighboring agents.

        Returns:
            Set of agents which failed to respond.
        """
        if neighborList is None:
            neighbors = deepcopy(self.neighbors[mission_sign].keys())
        else:
            neighbors = deepcopy(neighborList)

        failed = set()

        logger.debug('Waiting for neighborhood state. Status: %s, neighbors: %s', status, neighbors)

        while len(neighbors) > 0:
            # If the mission was aborted or restarted.
            if self.missionStatus[mission_sign] != status:
                return set()

            for neighbor in neighbors:
                try:
                    rospy.wait_for_service(neighbor + 'mission_status', self.serviceTimeout)

                    mission_status = rospy.ServiceProxy(neighbor + 'mission_status', MissionStatus)
                    response = mission_status(mission_sign)

                    if response.mission_status == "no_mission" and status != "no_mission":
                        failed.add(neighbor)
                        self.remove_neighbor(mission_sign, neighbor)
                        neighbors.remove(neighbor)
                    if response.mission_status == "executing":
                        rospy.logwarn('Mission is already executing on some agents!')
                        self.missionStatus[mission_sign] = "abort"
                    else:
                        if response.mission_status == status:
                            neighbors.remove(neighbor)

                except (rospy.ServiceException, rospy.ROSException) as e:
                    rospy.logerr('Service call failed: %s', e)
                    failed.add(neighbor)
                    self.remove_neighbor(mission_sign, neighbor)
                    neighbors.remove(neighbor)

        return failed

    def check_neighborhood_state(self, mission_sign, status):
        """
        Check neighborhood mission state.

        Args:
            mission_sign (str): Name and ID of the mission.
            status (str): Mission status.

        Returns:
            0 for success, -1 if neighbors have mission state different from 'status', -2 if the mission should be aborted.
        """
        # WUT? Koja je razlika izmedu check_neighborhood i wait_for_neighborhood?
        # If the mission was aborted or restarted.
        if self.missionStatus[mission_sign] != status:
            return -1

        logger.debug('Checking neighborhood state. Status: %s, neighbors: %s', status,
                     self.neighbors[mission_sign].keys())

        for neighbor in self.neighbors[mission_sign].keys():
            try:
                rospy.wait_for_service(neighbor + 'mission_status', self.serviceTimeout)

                mission_status = rospy.ServiceProxy(neighbor + 'mission_status', MissionStatus)
                response = mission_status(mission_sign)

                if response.mission_status == "no_mission" and status != "no_mission":
                    self.remove_neighbor(mission_sign, neighbor)
                    return -2
                if response.mission_status == "executing":
                    rospy.logwarn('Mission is already executing on some agents!')
                    self.missionStatus[mission_sign] = "abort"
                else:
                    if response.mission_status != status:
                        return -1

            except (rospy.ServiceException, rospy.ROSException) as e:
                rospy.logerr('Service call failed: %s', e)
                self.remove_neighbor(mission_sign, neighbor)
                return -2

        return 0

    def update_neighborhood_info(self, mission_sign, neighbors=None):

        failed = set()
        if neighbors is None:
            neighbors = self.neighbors[mission_sign].keys()

        for neighbor in neighbors:
            if self.missionStatus[mission_sign] == "abort":
                return set()

            try:
                rospy.wait_for_service(neighbor + 'mission_info', self.serviceTimeout)

                mission_info = rospy.ServiceProxy(neighbor + 'mission_info', MissionInfo)
                response = mission_info(mission_sign)

                if response.my_mission is False:
                    self.remove_neighbor(mission_sign, neighbor)
                    failed.add(neighbor)
                else:
                    self.neighbors[mission_sign][neighbor].completedTasks = response.completed_tasks
                    self.neighbors[mission_sign][neighbor].scheduledMethods = response.scheduled_methods
                    self.neighbors[mission_sign][neighbor].taskLabels = response.task_labels

            except (rospy.ServiceException, rospy.ROSException) as e:
                rospy.logerr('Service call failed: %s', e)
                failed.add(neighbor)
                self.remove_neighbor(mission_sign, neighbor)

        return failed

    def mission_assess_request(self, mission_sign):

        while self.pose_estimate is None:
            rospy.loginfo('Waiting for position information')
            rospy.sleep(0.5)

        neighbor_address_temp = self.neighbors[mission_sign].keys()
        neighbor_address = []   # List of agents' addresses corresponding to labels below
        neighbor_label = []     # List of all neighboring agents' labels (types)

        # testme
        # for key, value in self.neighbors[mission_sign].items():
        #     neighbor_label.extend(value.label)
        #     neighbor_address.extend([key] * len(value.label))

        for item in neighbor_address_temp:
            neighbor_label.extend(self.neighbors[mission_sign][item].label)
            neighbor_address.extend([item] * len(self.neighbors[mission_sign][item].label))

        rospy.wait_for_service('assess_mission_tasks')
        try:
            logger.debug('Calling \"assess_mission_tasks\" service.')
            mission_assess = rospy.ServiceProxy('assess_mission_tasks', AssessMissionTasks)
            result = mission_assess(mission_sign, neighbor_label, neighbor_address)
            # If result.done is False, there was an error and mission should be canceled.
            return result.done
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s', e)

    def mission_reassess_request(self, mission_sign):

        while self.pose_estimate is None:
            rospy.loginfo('Waiting for position information')
            rospy.sleep(0.5)

        neighbor_address = self.neighbors[mission_sign].keys()
        neighbor_task_labels = []
        for neighbor in neighbor_address:
            neighbor_task_labels.append(self.neighbors[mission_sign][neighbor].taskLabels)

        rospy.wait_for_service('reassess_mission_tasks')
        try:
            logger.debug('Calling \"reassess_mission_tasks\" service.')
            mission_reassess = rospy.ServiceProxy('reassess_mission_tasks', ReassessMissionTasks)
            # TODO should be positionEstimate[mission_sign] -> position at the beginning of each mission
            result = mission_reassess(mission_sign, neighbor_address, json.dumps(neighbor_task_labels))
            # If result.done is False, there was an error and mission should be canceled
            if result.done is True:
                for task in result.to_remove:
                    self.remove_task(mission_sign, task)
            return result.done

        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s', e)

    def get_unscheduled_task_outcome_ev(self, mission_sign):
        """
        Gets task outcome from scheduler. Outcome is calculated as average value of quality,
        duration and cost for every possible alternative that completes the task.
        """
        rospy.wait_for_service('task_outcome_ev')
        try:
            logger.debug('Calling \"task_outcome_ev\" service.')
            task_outcome = rospy.ServiceProxy('task_outcome_ev', TaskOutcomeEV)
            result = task_outcome(mission_sign, self.missions[mission_sign], self.criteriaFilename[mission_sign])

            ev = json.loads(result.json_task_outcome_ev)
            self.taskOutcomeEVNonSched[mission_sign] = {}
            for i in range(len(result.all_tasks)):
                task = result.all_tasks[i]
                self.taskOutcomeEVNonSched[mission_sign][task] = [ev[i][0], ev[i][1], ev[i][2]]

        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s', e)

    def reschedule_request(self, mission_sign):

        startTime = 0
        rospy.wait_for_service('mission_earliest_start')
        try:
            logger.debug('Calling \"mission_earliest_start\" service.')
            earliest_start = rospy.ServiceProxy('mission_earliest_start', MissionEarliestStart)
            result = earliest_start(mission_sign)
            startTime = result.mission_earliest_start
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s', e)

        rospy.wait_for_service('reschedule')
        try:
            logger.debug('Calling \"reschedule\" service.')
            reschedule = rospy.ServiceProxy('reschedule', Reschedule)
            result = reschedule(mission_sign, self.missions[mission_sign], self.criteriaFilename[mission_sign],
                                startTime)
            # check for errors
            if result.done != 0:
                return result.done
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s', e)
            return RescheduleResponse.NO_SCHEDULE

        sched = []
        self.taskEndTime[mission_sign] = {}

        for i in range(len(result.task_labels)):
            sched.append([])
            sched[-1].append(result.task_labels[i])
            sched[-1].append(result.start_times[i])
            sched[-1].append(result.end_times[i])

        self.bestSchedule[mission_sign] = sched
        self.completedTasks[mission_sign] = result.completed_tasks

        for item in self.treeDict[mission_sign].homogeneousTasks:
            if set(self.treeDict[mission_sign].tasks[item].subtasks).issubset(set(result.completed_tasks)):
                self.completedTasks[mission_sign].append(item)

        self.completedNonLocal[mission_sign] = result.non_local_tasks
        self.scheduledMethods[mission_sign] = result.task_labels

        while "slack" in self.scheduledMethods[mission_sign]:
            self.scheduledMethods[mission_sign].remove("slack")

        for task in result.non_local_tasks:
            self.parentChildCRs[mission_sign][task].isMandatory = True

        # Simulation of schedule updates treeDictScheduled[missionID], taskOutcome[missionID]
        # and bestSchedule[missionID] - potentially different duration.
        self.simulate_schedule(mission_sign, result.non_local_tasks)

        # Get new task schedule times from executor (with respect to task ESTs).
        self.adjust_schedule_times_request(mission_sign)

        self.calc_task_end_times(mission_sign)

        bestSchedule = [[x[0], round(x[1], 3), round(x[2], 3)] for x in self.bestSchedule[mission_sign]]
        logger.info('Best schedule:\n%s\n%s\n', bestSchedule, '-' * 20)
        return RescheduleResponse.SUCCESS

    def simulate_schedule(self, missionID, nonLocal):
        """
        Simulate schedule execution to activate soft IR effects. Simulator also updates task outcomes.
        """
        mySim = simulator.Simulator(self.treeDict[missionID])

        self.bestSchedule[missionID] = mySim.execute_schedule(self.bestSchedule[missionID], nonLocal,
                                                              self.bestSchedule[missionID][0][1])

        self.treeDictScheduled[missionID] = mySim.taemsTree
        self.taskOutcome[missionID] = mySim.taskOutcome

        # Tasks coupled for agent homogeneity.
        for item in self.treeDict[missionID].homogeneousTasks:
            mySim.completedTasks.append(item)
            self.taskOutcome[item] = mySim.calc_QDC(item)

    def adjust_schedule_times_request(self, missionID):
        """
        Requests schedule times adjustment from executor -- based on mission schedule, executor inserts it into agent's
        execution schedule and returns new task start and end times.
        """
        tasks = []
        start = []
        end = []
        for item in self.bestSchedule[missionID]:
            if item[0] == "slack":
                continue
            tasks.append(item[0])
            start.append(item[1])
            end.append(item[2])

        constrained = []
        EST = []
        if len(self.commitmentsNonLocal[missionID]) > 0:
            for item in self.commitmentsNonLocal[missionID].values():
                constrained.extend(item.constrainedMethods)
                EST.extend([item.time] * len(item.constrainedMethods))

        rospy.wait_for_service('adjust_schedule_times')
        try:
            adjust_sched = rospy.ServiceProxy('adjust_schedule_times', AdjustScheduleTimes)

            result = adjust_sched(missionID, tasks, start, end, constrained, EST)

            schedule = []
            for i in range(len(result.task_labels)):
                schedule.append([result.task_labels[i], result.start_times[i], result.end_times[i]])
            self.bestSchedule[missionID] = schedule

        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s', e)

    def calc_task_end_times(self, missionID):

        mySim = simulator.Simulator(self.treeDict[missionID])

        for task in self.completedNonLocal[missionID]:
            mySim.execute_non_local_task(task)

        mySim.calc_task_end_times(self.bestSchedule[missionID])

        self.taskEndTime[missionID] = mySim.taskEndTime

        # Tasks coupled for agent homogeneity.
        for item in self.treeDict[missionID].homogeneousTasks:
            if item in self.completedTasks[missionID]:
                time = 0
                for subtask in self.treeDict[missionID].tasks[item].subtasks:
                    if time == 0:
                        time = self.taskEndTime[missionID][subtask]
                    if self.treeDict[missionID].tasks[item].qaf in ['q_max']:
                        time = min(time, self.taskEndTime[missionID][subtask])
                    else:
                        time = max(time, self.taskEndTime[missionID][subtask])
                self.taskEndTime[missionID][item] = maxTime

    def remove_task(self, missionID, task):

        rospy.loginfo('Removing task %s', task)
        if task == self.missions[missionID]:
            self.missionStatus[missionID] = "abort"
            return

        if task not in self.treeDict[missionID].tasks.keys():
            return

        # DTC service.
        rospy.wait_for_service('remove_task')
        try:
            remove_task = rospy.ServiceProxy('remove_task', RemoveTask)
            remove_task(missionID, task)
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s', e)
        # task_assessor service.
        rospy.wait_for_service('assessor_remove_task')
        try:
            remove_task = rospy.ServiceProxy('assessor_remove_task', RemoveTask)
            remove_task(missionID, task)
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s', e)
        self.treeDict[missionID].removeTaskAndSubtasks(task)
        if missionID in self.treeDictScheduled.keys():
            self.treeDictScheduled[missionID].removeTaskAndSubtasks(task)
        '''
        if self.treeDictScheduled[missionID].tasks[task].subtasks is None:
            if task in self.scheduledMethods[missionID]:
                self.scheduledMethods[missionID].remove(task)
            return
        else:
            for subtask in self.treeDictScheduled[missionID].tasks[task].subtasks:
                self.remove_task(missionID, subtask)
        '''

    def handle_mission_redundancy(self, missionID, allMustBeScheduled):
        """
        Handles complex mission redundancy. Problem is represented as operation research assignment problem and
        solved using branch and bound algorithm.
        """
        subMissions = self.treeDict[missionID].tasks[self.missions[missionID]].subtasks
        redundantMissions = {}
        agents = [self.ns]

        # TODO -> for now it is hardcoded -- change to be flexible
        if "UGV_P" in self.label or "UGV_O" in self.label:
            labels = ["UGV_P", "UGV_O"]
        else:
            labels = ["UAV_Qc", "UAV_Qh"]

        for mission in subMissions:
            for agent in self.neighbors[missionID].values():
                if len(set(agent.label) - set(labels)) > 0:
                    continue
                if mission in agent.taskLabels:
                    if mission not in redundantMissions.keys():
                        redundantMissions[mission] = [agent.address]
                    else:
                        redundantMissions[mission].append(agent.address)
                    if agent.address not in agents:
                        agents.append(agent.address)
            if mission in self.treeDict[missionID].tasks.keys():
                if self.treeDict[missionID].tasks[mission].subtasks is None:
                    if self.treeDict[missionID].tasks[mission].nonLocal:
                        continue
                if mission not in redundantMissions.keys():
                    redundantMissions[mission] = [self.ns]
                else:
                    redundantMissions[mission].append(self.ns)
            if mission not in redundantMissions.keys():
                rospy.logerr('Mission is not possible - not enough agents for at least one submission!')
                rospy.logerr('Error detected at %s submission.', mission)
                return -1
        agents.sort()

        if len(agents) < len(subMissions) and allMustBeScheduled:
            rospy.logerr('Mission is not possible - not enough agents for at least one submission!')
            rospy.logerr('Number of agents: %s, number of submissions: %s', len(agents), len(subMissions))
            return -1

        matrix = np.matrix([[0] * len(subMissions)] * len(agents), dtype=np.float64)
        q = []
        d = []
        c = []
        qEV = []
        dEV = []
        cEV = []
        endTime = []
        for i in range(len(subMissions)):
            task = subMissions[i]
            for j in range(len(agents)):
                agent = agents[j]
                # check if agent was removed
                '''if agent in toRemove:
                    subMissions[task].remove(agent)'''
                if agent not in redundantMissions[task]:
                    continue
                matrix[j, i] = 1

                if agent == self.ns:
                    qEV.append(self.taskOutcomeEVNonSched[missionID][task][0])
                    dEV.append(self.taskOutcomeEVNonSched[missionID][task][1])
                    cEV.append(self.taskOutcomeEVNonSched[missionID][task][2])
                    missionEndTime = self.startTime[missionID] + dEV[-1]

                    q.append({qEV[-1]: 1.0})
                    d.append({dEV[-1]: 1.0})
                    c.append({cEV[-1]: 1.0})
                    endTime.append(missionEndTime)
                else:
                    try:
                        rospy.wait_for_service(agent + 'task_info', self.serviceTimeout)

                        task_info = rospy.ServiceProxy(agent + 'task_info', TaskInfo)
                        response = task_info(missionID, "non-sched", self.ns, task)

                        if response.my_mission is False:
                            matrix[j, i] = 0
                        else:
                            qEV.append(response.outcome_ev[0])
                            dEV.append(response.outcome_ev[1])
                            cEV.append(response.outcome_ev[2])
                            missionEndTime = response.end_time

                            q.append({qEV[-1]: 1.0})
                            d.append({dEV[-1]: 1.0})
                            c.append({cEV[-1]: 1.0})

                            endTime.append(missionEndTime)

                    except (rospy.ServiceException, rospy.ROSException) as e:
                        rospy.logerr('Service call failed: %s', e)
                        self.remove_neighbor(missionID, agent)

        ratings = self.criteria[missionID].evaluate(q, d, c, qEV, dEV, cEV)
        # Rating based on quality of task's execution.
        maxRating = max(ratings)
        minRating = min(ratings)
        if maxRating == minRating:
            maxRating = 1
            minRating = 0
        r1 = []
        for i in range(len(ratings)):
            r1.append((ratings[i] - minRating) / (maxRating - minRating))

        # Rating based on time of task's execution.
        maxTime = max(endTime)
        minTime = min(endTime)
        if maxTime == minTime:
            maxTime = 1
            minTime = 0
        r2 = []
        for i in range(len(endTime)):
            r2.append((maxTime - endTime[i]) / (maxTime - minTime))

        totalRating = []
        for i in range(len(ratings)):
            totalRating.append(r1[i] * 0.7 + r2[i] * 0.3)
            # TODO: fixed coefficients?

        k = 0
        for i in range(len(subMissions)):
            for j in range(len(agents)):
                if matrix.item(j, i) == 1:
                    matrix[j, i] = totalRating[k]
                    k += 1

        missionAssignment = BranchAndBoundOptimizer.optimize(matrix)
        myMission = missionAssignment[agents.index(self.ns)]
        # If a mission is assigned to this agent...
        if myMission < len(subMissions):
            # ...remove all other missions, their subtasks and assigned mission subtasks.
            toRemove = deepcopy(subMissions)
            toRemove.remove(subMissions[myMission])
            for mission in toRemove:
                self.remove_task(missionID, mission)
        else:
            return -1

        agents.remove(self.ns)
        return agents

    def handle_complex_redundancy(self, mission_sign, task, allMustBeScheduled):
        """
        Handle complex redundancy between tasks.
        """
        subTasks = self.treeDict[mission_sign].tasks[task].subtasks
        cRedundantTaks = {}  # {'Label of the task with complex redundancy': [list of responsible agents]}
        agents = [self.ns]  # List of agents that have this task in their tree.

        # Subtasks of the given task are complexly redundant, so check all of them.
        for task in subTasks:
            # First, check if neighboring agents have this task in their tree.
            for agent in self.neighbors[mission_sign].values():
                if task in agent.taskLabels:
                    if task not in cRedundantTaks.keys():
                        cRedundantTaks[task] = [agent.address]
                    else:
                        cRedundantTaks[task].append(agent.address)
                    if agent.address not in agents:
                        agents.append(agent.address)
            # Then, add yourself to the list of responsible agents as well.
            if task in self.treeDict[mission_sign].tasks.keys():
                # If task is a method and it's nonLocal, don't add yourself to responsible agents.
                if self.treeDict[mission_sign].tasks[task].subtasks is None:
                    if self.treeDict[mission_sign].tasks[task].nonLocal:
                        continue
                if task not in cRedundantTaks.keys():
                    cRedundantTaks[task] = [self.ns]
                else:
                    cRedundantTaks[task].append(self.ns)
            # If task is missing from dict of redundant tasks, that means that there are no agents which can do it.
            if task not in cRedundantTaks.keys():
                rospy.logerr('Mission is not possible - not enough agents for at least one subtask!')
                rospy.logerr('Error detected at %s subtask.', task)
                return -1
        agents.sort()

        # If there are not enough agents to complete all of the tasks and that is required, signal a failure.
        if len(agents) < len(subTasks) and allMustBeScheduled:
            rospy.logerr('Mission is not possible - not enough agents for at least one subtask!')
            rospy.logerr('Number of agents: %s, number of subtasks: %s', len(agents), len(subTasks))
            return -1

        matrix = np.matrix([[0] * len(subTasks)] * len(agents), dtype=np.float64)
        q = []
        d = []
        c = []
        qEV = []
        dEV = []
        cEV = []
        endTime = []
        for i in range(len(subTasks)):
            task = subTasks[i]
            for j in range(len(agents)):
                agent = agents[j]
                # check if agent was removed
                '''if agent in toRemove:
                    subTasks[task].remove(agent)'''
                if agent not in cRedundantTaks[task]:
                    continue
                matrix[j, i] = 1

                if agent == self.ns:
                    qEV.append(self.taskOutcomeEVNonSched[mission_sign][task][0])
                    dEV.append(self.taskOutcomeEVNonSched[mission_sign][task][1])
                    cEV.append(self.taskOutcomeEVNonSched[mission_sign][task][2])
                    missionEndTime = self.startTime[mission_sign] + dEV[-1]

                    q.append({qEV[-1]: 1.0})
                    d.append({dEV[-1]: 1.0})
                    c.append({cEV[-1]: 1.0})
                    endTime.append(missionEndTime)
                else:
                    try:
                        rospy.wait_for_service(agent + 'task_info', self.serviceTimeout)

                        task_info = rospy.ServiceProxy(agent + 'task_info', TaskInfo)
                        response = task_info(mission_sign, "non-sched", self.ns, task)

                        if response.my_mission is False:
                            matrix[j, i] = 0
                        else:
                            qEV.append(response.outcome_ev[0])
                            dEV.append(response.outcome_ev[1])
                            cEV.append(response.outcome_ev[2])
                            missionEndTime = response.end_time

                            q.append({qEV[-1]: 1.0})
                            d.append({dEV[-1]: 1.0})
                            c.append({cEV[-1]: 1.0})

                            endTime.append(missionEndTime)

                    except (rospy.ServiceException, rospy.ROSException) as e:
                        rospy.logerr('Service call failed: %s', e)
                        self.remove_neighbor(mission_sign, agent)

        ratings = self.criteria[mission_sign].evaluate(q, d, c, qEV, dEV, cEV)

        # Rating based on quality of task's execution.
        maxRating = max(ratings)
        minRating = min(ratings)
        if maxRating == minRating:
            maxRating = 1
            minRating = 0
        r1 = []
        for i in range(len(ratings)):
            r1.append((ratings[i] - minRating) / (maxRating - minRating))

        # Rating based on time of task's execution.
        maxTime = max(endTime)
        minTime = min(endTime)
        if maxTime == minTime:
            maxTime = 1
            minTime = 0
        r2 = []
        for i in range(len(endTime)):
            r2.append((maxTime - endTime[i]) / (maxTime - minTime))

        totalRating = []
        for i in range(len(ratings)):
            totalRating.append(r1[i] * 0.7 + r2[i] * 0.3)
            # TODO: fixed coefficients?

        k = 0
        for i in range(len(subTasks)):
            for j in range(len(agents)):
                if matrix.item(j, i) == 1:
                    matrix[j, i] = totalRating[k]
                    k += 1

        taskAssignment = BranchAndBoundOptimizer.optimize(matrix)
        myTask = taskAssignment[agents.index(self.ns)]
        # If a task is assigned to this agent...
        if myTask < len(subTasks):
            # ...remove all other tasks, their subtasks and assigned mission subtasks.
            toRemove = deepcopy(subTasks)
            toRemove.remove(subTasks[myTask])
            for mission in toRemove:
                self.remove_task(mission_sign, mission)
        else:
            return -1

        agents.remove(self.ns)
        return agents

    def detect_redundant_tasks(self, task, mission_sign):

        tree = self.treeDictScheduled[mission_sign]
        if task not in tree.tasks.keys():  # or task not in self.completedTasks[missionID]:
            return
        if tree.tasks[task].subtasks is None:
            if tree.tasks[task].nonLocal:
                return

        if tree.tasks[task].type == "homogeneous":
            metaTask = None

            # WUT? Sto je meta task?
            for item in self.treeDict[mission_sign].homogeneousTasks:
                if task in self.treeDict[mission_sign].tasks[item].subtasks and \
                   item in self.completedTasks[mission_sign]:
                    if self.treeDict[mission_sign].tasks[item].subtasks[0] != task:
                        # To ensure that each meta task is detected as redundant only once.
                        return
                    metaTask = item
                    break

            if metaTask is not None:
                task = metaTask
            responsibleAgent = self.ns
            redundant = False
            for neighbor in self.neighbors[mission_sign].keys():
                # if len(set(self.neighbors[mission_sign][neighbor].label) & set(self.label)) == 0:
                #    continue
                # if a neighbor scheduled the same method
                if task in self.neighbors[mission_sign][neighbor].completedTasks:
                    redundant = True
                    if task not in self.redundantTasks[mission_sign].keys():
                        self.redundantTasks[mission_sign][task] = []
                    self.redundantTasks[mission_sign][task].append(neighbor)
                    if neighbor < responsibleAgent:
                        responsibleAgent = neighbor
            if not redundant:
                return

            self.waitRedundant[mission_sign].append(responsibleAgent)
            if responsibleAgent == self.ns:
                self.responsibleFor[mission_sign].add(task)

            return

        elif tree.tasks[task].type == "heterogeneous":
            for subtask in tree.tasks[task].subtasks:
                self.detect_redundant_tasks(subtask, mission_sign)

    def communicate_redundancy(self, missionID):

        logger.debug('Communicating redundancy. I am responsible for: %s\n', self.responsibleFor[missionID])

        for task in self.responsibleFor[missionID]:
            logger.debug('Looking at task %s.', task)

            failed = True
            while failed:
                failed = set()
                agents = []
                q = []
                d = []
                c = []
                qEV = []
                dEV = []
                cEV = []
                endTime = []
                logger.debug('Neighbors in redundant tasks: %s', self.redundantTasks[missionID][task])
                for neighbor in self.redundantTasks[missionID][task]:
                    try:
                        rospy.wait_for_service(neighbor + 'task_info', self.serviceTimeout)

                        redundant_serv = rospy.ServiceProxy(neighbor + 'task_info', TaskInfo)
                        response = redundant_serv(missionID, "sched", self.ns, task)

                        if response.my_mission is False:
                            self.remove_neighbor(missionID, neighbor)
                        elif response.my_task:
                            agents.append(neighbor)

                            # FIXME: expected value from json.loads is a dict with floats for keys, but json.dumps
                            #  converts them to unicode strings which may mess up further calculations. This should be
                            #  confirmed. For now, a workaround is to to create a new dictionary with float keys.
                            q.append({float(key): value for key, value in json.loads(response.json_outcome_q).items()})
                            d.append({float(key): value for key, value in json.loads(response.json_outcome_d).items()})
                            c.append({float(key): value for key, value in json.loads(response.json_outcome_c).items()})

                            qEV.append(response.outcome_ev[0])
                            dEV.append(response.outcome_ev[1])
                            cEV.append(response.outcome_ev[2])

                            endTime.append(response.end_time)

                    except (rospy.ServiceException, rospy.ROSException) as e:
                        rospy.logerr('Service call failed: %s', e)
                        self.remove_neighbor(missionID, neighbor)

                agents.append(self.ns)
                q.append(self.taskOutcome[missionID][task][0])
                d.append(self.taskOutcome[missionID][task][1])
                c.append(self.taskOutcome[missionID][task][2])

                qEV.append(helper_functions.calcExpectedValue(self.taskOutcome[missionID][task][0]))
                dEV.append(helper_functions.calcExpectedValue(self.taskOutcome[missionID][task][1]))
                cEV.append(helper_functions.calcExpectedValue(self.taskOutcome[missionID][task][2]))

                endTime.append(self.taskEndTime[missionID][task])
                ratings = self.criteria[missionID].evaluate(q, d, c, qEV, dEV, cEV)

                logger.debug('Ratings: %s. End times: %s', ratings, endTime)

                # Rating based on quality of task's execution.
                maxRating = max(ratings)
                minRating = min(ratings)
                if maxRating == minRating:
                    r1 = [0.0] * len(ratings)
                else:
                    r1 = []
                    for thisRating in ratings:
                        r1.append((thisRating - minRating) / (maxRating - minRating))

                # Rating based on time of task's execution.
                maxTime = max(endTime)
                minTime = min(endTime)
                if maxTime == minTime:
                    r2 = [0.0] * len(ratings)
                else:
                    r2 = []
                    for thisTime in endTime:
                        # If relative improvement from maxTime is less than 5%, we can say agents are equally good.
                        if (maxTime - thisTime) / maxTime < 0.05:
                            r2.append(0.0)
                        else:
                            r2.append((maxTime - thisTime) / (maxTime - minTime))

                totalRating = []
                for i in range(len(ratings)):
                    totalRating.append(r1[i] * 0.7 + r2[i] * 0.3)
                # TODO: set parametric coefficients or tune them

                logger.debug('Total: %s', totalRating)

                bestAgent = agents[totalRating.index(max(totalRating))]
                logger.debug('Communicating redundancy... Task: %s, best agent: %s\n', task, bestAgent)

                # send
                for neighbor in agents:
                    try:
                        rospy.wait_for_service(neighbor + 'resolve_redundant', self.serviceTimeout)

                        solve_redundancy = rospy.ServiceProxy(neighbor + 'resolve_redundant', RedundancySolution)
                        solve_redundancy(self.ns, missionID, task, bestAgent)
                    except (rospy.ServiceException, rospy.ROSException) as e:
                        rospy.logerr('Service call failed: %s', e)
                        if neighbor == bestAgent:
                            failed.add(neighbor)
                        self.remove_neighbor(missionID, neighbor)

        return set()

    def detect_parent_children_CRs(self, task, missionID):
        """
        Recursively detect children-parent relationships.

        Args:
            task (str): Label of the root task.
            missionID (str): Mission identifier.
        """
        tree = self.treeDict[missionID]

        # Recursion return condition: current task is a method.
        if tree.tasks[task].subtasks is None:
            return

        for subtask in tree.tasks[task].subtasks:
            createCR = subtask not in tree.tasks.keys()
            if subtask in tree.tasks.keys():
                if type(tree.tasks[subtask]) is taems.Method:
                    if tree.tasks[subtask].nonLocal:
                        createCR = True
            if createCR:
                self.parentChildCRs[missionID][subtask] = CoordinationRelationshipParentChild(task, subtask, False)
                for neighbor in self.neighbors[missionID]:
                    if subtask in self.neighbors[missionID][neighbor].taskLabels:
                        self.parentChildCRs[missionID][subtask]\
                            .add_agent_to(neighbor, self.neighbors[missionID][neighbor].address)
            else:
                self.detect_parent_children_CRs(subtask, missionID)

    def assess_non_local_tasks(self, missionID):

        missingNonLocalTasks = set()

        for task in self.parentChildCRs[missionID].keys():
            qEV = []
            dEV = []
            cEV = []
            taskIsMandatory = self.parentChildCRs[missionID][task].isMandatory
            i = 0

            while i < self.parentChildCRs[missionID][task].count:
                try:
                    rospy.wait_for_service(self.parentChildCRs[missionID][task].agentToAddr[i] + 'task_info',
                                           self.serviceTimeout)

                    task_info = rospy.ServiceProxy(self.parentChildCRs[missionID][task].agentToAddr[i] + 'task_info',
                                                   TaskInfo)
                    response = task_info(missionID, "non-sched", self.ns, task)

                    if response.my_mission is False:
                        self.remove_neighbor(missionID, self.parentChildCRs[missionID][task].agentToAddr[i])
                        i -= 1
                    elif response.my_task is False:
                        self.parentChildCRs[missionID][task].remove_agent_to(
                            self.parentChildCRs[missionID][task].agentToAddr[i])
                        i -= 1
                    else:
                        qEV.append(response.outcome_ev[0])
                        dEV.append(response.outcome_ev[1])
                        cEV.append(response.outcome_ev[2])

                except (rospy.ServiceException, rospy.ROSException) as e:
                    rospy.logerr('Service call failed: %s', e)
                    self.remove_neighbor(missionID, self.parentChildCRs[missionID][task].agentToAddr[i])
                    i -= 1
                i += 1

            if len(qEV) > 0:
                self.add_non_local_task(missionID, task, self.parentChildCRs[missionID][task].taskFrom,
                                        [sum(qEV) / len(qEV), sum(dEV) / len(dEV), sum(cEV) / len(cEV)])
            else:
                if task in self.treeDict[missionID].tasks.keys():
                    self.remove_task(missionID, task)
                if taskIsMandatory:
                    # No need to assess other tasks.
                    break

        for cr in self.parentChildCRs[missionID].values():
            if cr.count == 0 and cr.isMandatory:
                missingNonLocalTasks.add(cr.taskTo)

        return missingNonLocalTasks

    def add_non_local_task(self, missionID, taskLabel, supertask, outcomeEV):

        task = taems.Method()
        task.label = taskLabel
        task.supertasks = [supertask]
        q = {outcomeEV[0]: 1.0}
        d = {outcomeEV[1]: 1.0}
        c = {outcomeEV[2]: 1.0}
        task.outcome = [q, d, c]
        # Used for execution, has to be local for simulator to count it into parent tasks's completion.
        task.nonLocal = True

        self.treeDict[missionID].tasks[taskLabel] = task

        rospy.wait_for_service('add_non_local_task')
        try:
            add_non_local = rospy.ServiceProxy('add_non_local_task', AddNonLocalTask)
            add_non_local(missionID, taskLabel, supertask, outcomeEV)

        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s', e)

    def remove_non_local_task(self, missionID, task):

        if task in self.treeDict[missionID].tasks.keys():
            if self.treeDict[missionID].tasks[task].subtasks is None:
                if self.treeDict[missionID].tasks[task].nonLocal:
                    logger.info('Removing non local task: %s', task)
                    self.treeDict[missionID].tasks.pop(task)
                    # TODO --> use remove_task function instead
                    # DTC service
                    rospy.wait_for_service('remove_task')
                    try:
                        remove_non_local = rospy.ServiceProxy('remove_task', RemoveTask)
                        remove_non_local(missionID, task)

                    except rospy.ServiceException as e:
                        rospy.logerr('Service call failed: %s', e)

    def detect_precedence_constraint_CRs(self, missionID):

        for ir in self.treeDictScheduled[missionID].IRs.values():
            # Enables.
            if ir.type == taems.Interrelationship.IR_types['enables']:
                if ir.From in self.completedTasks[missionID] or ir.From in self.scheduledMethods[missionID]:
                    if ir.From not in self.taskEndTime[missionID].keys():
                        continue
                    for neighbor in self.neighbors[missionID]:
                        if ir.To in self.neighbors[missionID][neighbor].completedTasks or ir.To in \
                                self.neighbors[missionID][neighbor].scheduledMethods:
                            self.hardConstrainedNeighbors[missionID][0].add(neighbor)
                            if ir.From not in self.commitmentsLocal[missionID].keys():
                                self.commitmentsLocal[missionID][ir.From] = LocalCommitment(self.ns, ir.From, neighbor, ir.To)
                            else:
                                self.commitmentsLocal[missionID][ir.From].add_task_to(neighbor, ir.To)
            # Disables.
            if ir.type == taems.Interrelationship.IR_types['disables']:
                if ir.To in self.completedTasks[missionID] or ir.To in self.scheduledMethods[missionID]:
                    if ir.To not in self.taskEndTime[missionID].keys():
                        continue
                    for neighbor in self.neighbors[missionID]:
                        if ir.From in self.neighbors[missionID][neighbor].completedTasks or ir.From in \
                                self.neighbors[missionID][neighbor].scheduledMethods:
                            self.hardConstrainedNeighbors[missionID][0].add(neighbor)
                            if ir.To not in self.commitmentsLocal[missionID].keys():
                                self.commitmentsLocal[missionID][ir.To] = LocalCommitment(self.ns, ir.To, neighbor, ir.From)
                            else:
                                self.commitmentsLocal[missionID][ir.To].add_task_to(neighbor, ir.From)

    def communicate_precedence_constraint_CRs(self, missionID, tasks):
        logger.debug('Communicate precedence constraints. Tasks: %s', str(tasks))
        failed = set()
        communicatedTo = set()

        for task in tasks:

            if task not in self.commitmentsLocal[missionID].keys():
                continue

            time = self.taskEndTime[missionID][task]
            result = True
            self.commitmentsLocal[missionID][task].time = time

            if result is True:
                i = 0
                while i < self.commitmentsLocal[missionID][task].count:
                    logger.debug('Task: %s. To agent: %s. To task: %s. Time: %s', task,
                                 self.commitmentsLocal[missionID][task].agentToAddr[i],
                                 self.commitmentsLocal[missionID][task].taskTo[i], time)
                    try:
                        rospy.wait_for_service(
                            self.commitmentsLocal[missionID][task].agentToAddr[i] + 'register_commitment',
                            self.serviceTimeout)
                        register_commitment = rospy.ServiceProxy(
                            self.commitmentsLocal[missionID][task].agentToAddr[i] + 'register_commitment',
                            AddCommitmentNonLocal)
                        register_commitment(missionID, self.ns, task, self.commitmentsLocal[missionID][task].taskTo[i], time)
                        communicatedTo.add(self.commitmentsLocal[missionID][task].agentToAddr[i])
                    except (rospy.ServiceException, rospy.ROSException) as e:
                        rospy.logerr('Service call failed: %s', e)
                        failed.add(self.commitmentsLocal[missionID][task].agentToAddr[i])
                        self.remove_neighbor(missionID, self.commitmentsLocal[missionID][task].agentToAddr[i])
                        i -= 1
                    i += 1
                    if task not in self.commitmentsLocal[missionID].keys():
                        break
            else:
                rospy.logwarn('Could not add local commitment for task %s', task)
        logger.debug('Communicate precedence constraints done.')
        return [failed, communicatedTo]

    def check_local_commitments(self, missionID):

        toRecommit = []

        for task in self.commitmentsLocal[missionID].keys():
            if self.taskEndTime[missionID][task] > self.commitmentsLocal[missionID][task].time:
                toRecommit.append(task)

        return toRecommit

    def send_commitment_to_scheduler(self, missionID, commType, task, time, breakTime):

        rospy.wait_for_service('add_commitment')
        try:
            add_commitment = rospy.ServiceProxy('add_commitment', AddCommitmentLocal)
            result = add_commitment(missionID, commType, task, time, breakTime)

            return result.done

        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s', e)

    def remove_neighbor(self, missionID, neighbor):

        if neighbor in self.neighbors[missionID].keys():
            self.neighbors[missionID].pop(neighbor)

        for task in self.parentChildCRs[missionID].keys():
            self.parentChildCRs[missionID][task].remove_agent_to(neighbor)
            if self.parentChildCRs[missionID][task].count == 0:
                self.remove_non_local_task(missionID, task)
                # TODO check this part -> could it be that there are other agents performing this task

        for commitment in self.commitmentsLocal[missionID].values():
            commitment.remove_agent_to(neighbor)
            if commitment.count == 0:
                self.commitmentsLocal[missionID].pop(commitment.taskFrom[0])

        for commitment in self.commitmentsNonLocal[missionID].values():
            commitment.remove_agent_from(neighbor)
            if commitment.count == 0:
                self.commitmentsNonLocal[missionID].pop(commitment.taskTo[0])

        for task in self.redundantTasks[missionID].keys():
            for item in self.redundantTasks[missionID][task]:
                if item == neighbor:
                    self.redundantTasks[missionID][task].remove(item)
                    break

    def send_schedule_ok_execute(self, missionID):

        rospy.wait_for_service('schedule_ok_execute')
        try:
            send_ok = rospy.ServiceProxy('schedule_ok_execute', ScheduleOK)
            result = send_ok(missionID, "")

            return result.done

        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s', e)

    # ************************** #
    # CALLBACK FUNCTIONS SECTION #
    # ************************** #

    def msg_mission_ctrl_callback(self, msg):
        """
        Callback function for "/mission_control" topic.

        Args:
            msg (MissionCtrlMsg): Incoming message specifying mission data
        """

        mission_sign = msg.root_task + "[" + str(msg.mission_id) + "]"
        if msg.type == "NewMission":
            print rospy.get_namespace()
            if msg.ag_addr == rospy.get_namespace():
                print "."
                if mission_sign not in self.missions.keys():
                    print "ya"
                    if mission_sign in self.missionStatus.keys():
                        if self.missionStatus[mission_sign] == "no_schedule":
                            return
                    self.missionCaller[mission_sign] = msg.ag_addr
                    self.missionType[mission_sign] = msg.mission_type
                    self.start_new_mission(msg.mission_id, msg.root_task, msg.criteria)

        if msg.type == "Abort":
            if msg.ag_addr == self.ns:
                return

            if mission_sign in self.missions.keys():
                rospy.loginfo('Got ABORT request from %s.', msg.ag_addr)
                rospy.loginfo('Aborting mission: %s', mission_sign)
                self.missionStatus[mission_sign] = "abort"

        if msg.type == "Restart":
            if msg.ag_addr == self.ns:
                return

            if mission_sign in self.missions.keys():
                rospy.loginfo('Got RESTART request from %s.', msg.ag_addr)
                rospy.loginfo('Restarting mission: %s', mission_sign)
                self.missionStatus[mission_sign] = "restart"

        if msg.type == "Completed":
            if msg.ag_addr == self.ns:
                self.missionStatus[mission_sign] = "completed"
                rospy.loginfo('***FINISHED MISSION***: %s[%s]\n', msg.root_task, msg.mission_id)

            if mission_sign in self.missions.keys():
                # Send message control to agent's nodes.
                msgNew = MissionCtrlMsg()
                msgNew.type = "Completed"
                msgNew.mission_id = msg.mission_id
                msgNew.root_task = msg.root_task
                msgNew.ag_addr = msg.ag_addr
                self.missionCtrlMsgLocalPub.publish(msgNew)

    def msg_hello_callback(self, msg):
        """
        Callback function for "/peer_discovery" topic.

        Args:
            msg (HelloMsg): Incoming message specifying agent's properties.
        """

        # If this agent is the sender - do nothing.
        if msg.ag_addr == self.ns:
            return

        if msg.mission_sign not in self.missions.keys():
            # Maybe the mission wasn't registered yet - wait a little bit.
            rospy.sleep(1.0)

        if msg.mission_sign in self.missions.keys():
            if self.missionStatus[msg.mission_sign] == "executing":
                try:
                    rospy.wait_for_service(msg.ag_addr + 'signal_abort')
                    signal_abort = rospy.ServiceProxy(msg.ag_addr + 'signal_abort', SignalMissionAbort)
                    signal_abort(msg.mission_sign, self.ns)
                except (rospy.ServiceException, rospy.ROSException) as e:
                    rospy.logerr('Service call failed: %s', e)

                return

            if msg.ag_addr not in self.neighbors.keys():
                self.neighbors[msg.mission_sign][msg.ag_addr] = Agent(msg.ag_addr, msg.ag_name)
                self.neighbors[msg.mission_sign][msg.ag_addr].taskLabels = msg.task_labels

            if self.missionStatus[msg.mission_sign] != "start" and \
               self.missionStatus[msg.mission_sign] != "started_coordination":
                self.initialize_mission_rescheduling(msg.mission_sign)

    def update_task_structure_callback(self, msg):

        if msg.mission_id not in self.treeDict.keys():
            return

        outcomes = json.loads(msg.json_outcome)

        output = ['{:^29}|{:^49}'.format('Task', 'Quality, Duration, Cost'), '-' * 80]
        for i in range(len(msg.task_label)):
            task = msg.task_label[i]
            outcome = [{}, {}, {}]
            for item in outcomes[i][0].keys():
                outcome[0][round(float(item), 3)] = outcomes[i][0][item]
            for item in outcomes[i][1].keys():
                outcome[1][round(float(item), 3)] = outcomes[i][1][item]
            for item in outcomes[i][2].keys():
                outcome[2][round(float(item), 3)] = outcomes[i][2][item]
            self.treeDict[msg.mission_id].tasks[task].outcome = outcome

            self.treeDict[msg.mission_id].tasks[task].DurationEV = helper_functions.calcExpectedValue(outcome[1])
            self.treeDict[msg.mission_id].tasks[task].CostEV = helper_functions.calcExpectedValue(outcome[2])
            output.append('{:^29} {:^49}'.format(task, outcome))
        output.append('-' * 80)
        logger.debug('Update task structure callback.\n%s', '\n'.join(output))

        if msg.mission_id in self.treeDictScheduled.keys():
            self.simulate_schedule(msg.mission_id)

        self.waitTaskAssessmentFlag[msg.mission_id] = False

    # ************************* #
    # SERVICE FUNCTIONS SECTION #
    # ************************* #

    def register_abort_srv(self, req):

        rospy.loginfo('Mission %s is already executing on agent %s', req.mission_id, req.ag_addr)
        if req.mission_id in self.missions.keys():
            self.missionStatus[req.mission_id] = "abort"
        return []

    def task_info_srv(self, req):
        """
        Return information about specified task.

        Args:
            req (TaskInfoRequest): Service request data with task label.

        Returns:
            Outcomes, expected values and end time of the task packed in a list.
            return[0] (bool): Agent has the specified mission.
            return[1] (bool): Agent has the specified task.
            return[2:4] (str): Quality, duration, cost outcomes.
            return[5] (list[str]): Quality, duration, cost expected values.
            return[6] (float): End time of the task.
        """

        missionID = req.mission_id
        taskLabel = req.task_label

        # TODO: It's actually mission_sign, not mission_id

        while self.missionStatus[missionID] == "start":
            pass

        if self.missionStatus[missionID] in ["abort", "complete", "no_schedule", "no_mission"]:
            return [False, False, "", "", "", [], 0]

        if taskLabel not in self.treeDict[missionID].tasks.keys():
            logger.debug('Task info: taskLabel not in tree')
            return [True, False, "", "", "", [], 0]
        if self.treeDict[missionID].tasks[taskLabel].subtasks is None:
            if self.treeDict[missionID].tasks[taskLabel].nonLocal:
                return [True, False, "", "", "", [], 0]

        if req.type == "sched":
            # FIXME: taskOutcome keys are floats, but json.dumps converts them to unicode strings which can mess things
            #  up on the receiving side. For now, this problem is solved with a workaround on the receiving end.
            logger.debug('RAW OUTCOME: %s', self.taskOutcome[missionID][taskLabel])
            outcome = [json.dumps(self.taskOutcome[missionID][taskLabel][0]),
                       json.dumps(self.taskOutcome[missionID][taskLabel][1]),
                       json.dumps(self.taskOutcome[missionID][taskLabel][2])]
            logger.debug('JSON OUTCOME: %s', outcome)

            qualityEV = helper_functions.calcExpectedValue(self.taskOutcome[missionID][taskLabel][0])
            durationEV = helper_functions.calcExpectedValue(self.taskOutcome[missionID][taskLabel][1])
            costEV = helper_functions.calcExpectedValue(self.taskOutcome[missionID][taskLabel][2])

            return [True, True, outcome[0], outcome[1], outcome[2], [qualityEV, durationEV, costEV],
                    self.taskEndTime[missionID][taskLabel]]
        else:
            if taskLabel in self.taskOutcomeEVNonSched[missionID].keys():
                outcome_ev = self.taskOutcomeEVNonSched[missionID][taskLabel]
                return [True, True, "", "", "", [outcome_ev[0], outcome_ev[1], outcome_ev[2]],
                        self.startTime[missionID] + outcome_ev[1]]
            else:
                logger.debug('Task info: taskLabel not in EVNonSched')
                return [True, False, "", "", "", [], 0]

    def register_feedback_request_srv(self, req):

        if req.task_label not in self.feedbackRequestList[req.mission_id].keys():
            self.feedbackRequestList[req.mission_id][req.task_label] = LocalCommitment("", req.task_label, req.agent, "")
        else:
            if req.agent not in self.feedbackRequestList[req.mission_id][req.task_label].agentToAddr:
                self.feedbackRequestList[req.mission_id][req.task_label].add_task_to(req.agent, req.task_label)

        return []

    def mission_info_srv(self, req):

        missionID = req.mission_id

        if missionID in self.missions.keys() and self.missionStatus[missionID] not in ["abort", "no_schedule", "no_mission"]:
            while missionID in self.waitComplexRedundancy.keys():
                if self.waitComplexRedundancy[missionID]:
                    rospy.sleep(0.1)
                else:
                    break
            if missionID in self.missions.keys():
                if missionID not in self.completedTasks.keys():
                    return [True, [], [], self.treeDict[missionID].tasks.keys()]
                else:
                    tasks = deepcopy(self.treeDict[missionID].tasks.keys())
                    for task in self.treeDict[missionID].tasks.keys():
                        if self.treeDict[missionID].tasks[task].subtasks is None:
                            if self.treeDict[missionID].tasks[task].nonLocal:
                                tasks.remove(task)
                    return [True, self.completedTasks[missionID], self.scheduledMethods[missionID],
                            self.treeDict[missionID].tasks.keys()]

        return [False, [], [], []]

    def mission_status_srv(self, req):

        if req.mission_id in self.missions.keys():
            return self.missionStatus[req.mission_id]

        return "no_mission"

    def resolve_redundant_srv(self, req):

        if req.mission_id not in self.missions.keys():
            return False

        if req.best_ag_addr != self.ns:
            subtasks = self.treeDictScheduled[req.mission_id].get_all_subtasks(req.task_label)
            self.remove_task(req.mission_id, req.task_label)
            # check if there are any items left in schedule
            remainingTasks = len(self.bestSchedule[req.mission_id])
            for i in range(len(self.bestSchedule[req.mission_id])):
                if self.bestSchedule[req.mission_id][i][0] in subtasks:
                    remainingTasks -= 1
            if remainingTasks == 0:
                self.missionStatus[req.mission_id] = "abort"

        try:
            self.waitRedundant[req.mission_id].remove(req.ag_addr)
            logger.debug('Removed %s from waitRedundant. Current list: %s', req.ag_addr, self.waitRedundant[req.mission_id])
        except ValueError:
            rospy.logerr('Can\'t remove %s from waitRedundant. Item not found.', req.ag_addr)

        return True

    def add_commitment_non_local_srv(self, req):

        if req.ag_addr not in self.neighbors[req.mission_id]:
            return False

        if req.task_label_to not in self.commitmentsNonLocal[req.mission_id].keys():
            non_local = NonLocalCommitment(req.ag_addr, req.task_label_from, self.ns, req.task_label_to, req.time)
            self.commitmentsNonLocal[req.mission_id][req.task_label_to] = non_local

            constrained_methods = self.get_constrained_methods(req.mission_id, req.task_label_to)
            self.commitmentsNonLocal[req.mission_id][req.task_label_to].constrainedMethods = constrained_methods
        else:
            self.commitmentsNonLocal[req.mission_id][req.task_label_to].add_task_from(req.ag_addr,
                                                                                      req.task_label_from,
                                                                                      req.time)
        self.hardConstrainedNeighbors[req.mission_id][1].add(req.ag_addr)
        result = self.send_commitment_to_scheduler(req.mission_id, "earliest_start_time",
                                                   req.task_label_to, req.time, False)

        if self.scheduleIsCoordinated[req.mission_id] is True:
            self.scheduleIsCoordinated[req.mission_id] = False
        return result

    def get_constrained_methods(self, missionID, task):

        if task not in self.treeDict[missionID].tasks.keys():
            return []

        if self.treeDict[missionID].tasks[task].subtasks is None:
            return [task]

        methods = []
        for subtask in self.treeDict[missionID].tasks[task].subtasks:
            methods.extend(self.get_constrained_methods(missionID, subtask))

        return methods

    def register_schedule_ok_srv(self, req):

        if req.mission_id not in self.missions.keys():
            return False

        if req.mission_id in self.waitForScheduleOK.keys():
            if req.ag_addr in self.waitForScheduleOK[req.mission_id]:
                self.waitForScheduleOK[req.mission_id].remove(req.ag_addr)

        return True

    def register_executed_task_srv(self, req):
        logger.debug('!Registering executed task: %s', req.task)

        # if self.missionStatus[req.mission_id] == "completed":
        #    return
        tasksToComplete = deepcopy(self.completedTasks[req.mission_id])  # This is more like "will be completed".
        tasksToComplete.extend(self.scheduledMethods[req.mission_id])
        tasksToComplete.extend(self.completedNonLocal[req.mission_id])

        logger.debug(' \nCompleted: %s\nScheduled: %s\nNon-local: %s', self.completedTasks[req.mission_id],
                     self.scheduledMethods[req.mission_id], self.completedNonLocal[req.mission_id])

        while self.simulator[req.mission_id] is None:
            rospy.sleep(0.2)
        # if task exists, register its execution
        if req.task != "" and req.task != "slack":
            completedBefore = set(self.simulator[req.mission_id].completedTasks)
            logger.debug('Completed before: %s', completedBefore)

            if req.task in self.scheduledMethods[req.mission_id]:
                del self.bestSchedule[req.mission_id][0]

            self.simulator[req.mission_id].execute_task(req.task, tasksToComplete)
            completedAfter = set(self.simulator[req.mission_id].completedTasks)
            completedNew = completedAfter - completedBefore
            logger.debug('Completed new: %s', completedNew)

            logger.debug('Local commitments agents: %s', self.commitmentsLocal[req.mission_id].items())
            logger.debug('Feedback request list: %s', self.feedbackRequestList[req.mission_id].items())

            # communicate to other agents
            for task in completedNew:
                communicateTo = set()
                if task in self.commitmentsLocal[req.mission_id].keys():
                    for i in range(self.commitmentsLocal[req.mission_id][task].count):
                        communicateTo.add(self.commitmentsLocal[req.mission_id][task].agentToAddr[i])
                    self.commitmentsLocal[req.mission_id].pop(task)
                if task in self.feedbackRequestList[req.mission_id]:
                    communicateTo |= set(self.feedbackRequestList[req.mission_id][task].agentToAddr)

                logger.debug('Task: %s', task)
                logger.debug('Communicate to: %s', communicateTo)

                for neighbor in communicateTo:
                    try:
                        rospy.wait_for_service(neighbor + 'register_executed_task', self.serviceTimeout)
                        logger.debug('Calling \"%sregister_executed_task\"', neighbor)
                        register_executed_task = rospy.ServiceProxy(neighbor + 'register_executed_task', ExecuteTask)
                        register_executed_task(req.mission_id, task, req.delay)
                    except (rospy.ServiceException, rospy.ROSException) as e:
                        rospy.logerr('Service call failed: %s', e)

            logger.debug('Nonlocal commitments agents: %s', self.commitmentsNonLocal[req.mission_id].items())
            if req.task not in self.scheduledMethods[req.mission_id]:
                toRemove = set()
                for commitment in self.commitmentsNonLocal[req.mission_id].values():
                    if req.task in commitment.taskFrom:
                        commitment.remove_task_from(req.task)
                        if commitment.count == 0:
                            toRemove.add(commitment.taskTo[0])
                for task in toRemove:
                    self.commitmentsNonLocal[req.mission_id].pop(task)

        '''if req.delay != 0 and len(self.bestSchedule[req.mission_id]) > 0:
            while self.waitTaskAssessmentFlag[req.mission_id]:
                rospy.sleep(0.1)
            self.waitTaskAssessmentFlag[req.mission_id] = True
            schedStart = self.bestSchedule[req.mission_id][0][1]

            self.adjust_schedule_times_request(req.mission_id)
            delay = self.bestSchedule[req.mission_id][0][1] - schedStart
            
            for commitment in self.commitmentsLocal[req.mission_id].values():
                commitment.time += (delay + 0.1)
           
            rospy.wait_for_service('delay_schedule')
            try:
                delay_schedule = rospy.ServiceProxy('delay_schedule', DelaySchedule)
                if delay == 0:
                    delay_schedule(0)
                else:
                    delay_schedule(delay + 0.1)
                    
            except rospy.ServiceException, e:
                rospy.logerr('Service call failed: %s', e)
                     
            self.waitTaskAssessmentFlag[req.mission_id] = False'''

        enabled = self.simulator[req.mission_id].get_enabled_tasks(self.scheduledMethods[req.mission_id])
        logger.debug('Enabled tasks: %s', enabled)

        disabledTasks = []
        for commitment in self.commitmentsNonLocal[req.mission_id].values():
            disabledTasks.extend(commitment.constrainedMethods)
            disabledTasks.extend(commitment.taskTo)
        logger.debug('Disabled tasks: %s', disabledTasks)
        disabledTasks = set(disabledTasks)
        enabled -= disabledTasks
        msg = MissionEnabledTasks()
        msg.mission_id = req.mission_id
        msg.task_labels = enabled

        self.missionEnabledTasksPub.publish(msg)
        return True

    def get_position_srv(self, srv):

        while self.pose_estimate is None:
            rospy.loginfo('Waiting for agent position information')
            rospy.sleep(0.5)

        return self.pose_estimate.position

    def get_pose_srv(self, srv):

        while self.pose_estimate is None:
            rospy.loginfo('Waiting for agent position information')
            rospy.sleep(0.5)

        return self.pose_estimate


class Agent(object):
    """
    A class that represents peer agents included in coordination process.

    Attributes:
        address (str): Agent's namespace.
        label (list[str]): Agent's labels (types) (in taems structure).
        completedTasks (list)
        scheduledMethods (list)
        taskLabels (list[str]): Labels of all agent's tasks.
    """

    def __init__(self, address, label):
        self.address = address
        self.label = label
        self.completedTasks = []
        self.scheduledMethods = []
        self.taskLabels = []


# noinspection PyMissingOrEmptyDocstring
class CoordinationRelationshipParentChild(object):

    def __init__(self, fromTask, toTask, isMandatory):
        self.taskFrom = fromTask
        self.taskTo = toTask
        self.agentToAddr = []
        self.agentToType = set()
        self.count = 0
        self.isMandatory = isMandatory

    def add_agent_to(self, toAgent, toAgentType):
        self.agentToAddr.append(toAgent)
        self.agentToType.add(toAgentType)
        self.count += 1

    def remove_agent_to(self, toAgent):
        while toAgent in self.agentToAddr:
            self.agentToAddr.remove(toAgent)
            self.count -= 1

    def remove_item_at(self, i):
        del self.agentToAddr[i]
        self.count -= 1


# noinspection PyMissingOrEmptyDocstring
class Commitment(object):

    def __init__(self, fromAgent, fromTask, toAgent, toTask):
        self.taskFrom = [fromTask]
        self.taskTo = [toTask]
        self.agentToAddr = [toAgent]
        self.agentFromAddr = [fromAgent]
        self.count = 1
        self.time = -1

    def __str__(self):
        return str([self.taskFrom, self.taskTo, self.agentFromAddr, self.agentToAddr])

    def __repr__(self):
        return self.__str__()


# noinspection PyMissingOrEmptyDocstring
class LocalCommitment(Commitment):

    def __init__(self, fromAgent, fromTask, toAgent, toTask):
        super(LocalCommitment, self).__init__(fromAgent, fromTask, toAgent, toTask)

    def add_task_to(self, toAgent, toTask):
        self.agentToAddr.append(toAgent)
        self.taskTo.append(toTask)
        self.count += 1

    def set_time(self, time):
        self.time = time

    def remove_agent_to(self, toAgent):
        while toAgent in self.agentToAddr:
            index = self.agentToAddr.index(toAgent)
            del self.agentToAddr[index]
            del self.taskTo[index]
            self.count -= 1


# noinspection PyMissingOrEmptyDocstring
class NonLocalCommitment(Commitment):

    def __init__(self, fromAgent, fromTask, toAgent, toTask, time):
        super(NonLocalCommitment, self).__init__(fromAgent, fromTask, toAgent, toTask)
        self.time = time
        self.constrainedMethods = []

    def contains(self, fromTask, fromAgent):
        for i in range(len(self.taskFrom)):
            if fromTask != self.taskFrom[i]:
                continue
            if fromAgent == self.agentFromAddr[i]:
                return True
        return False

    def add_task_from(self, fromAgent, fromTask, time):
        # check for existing commitment
        if self.contains(fromTask, fromAgent):
            self.set_time(time)
        else:
            self.agentFromAddr.append(fromAgent)
            self.taskFrom.append(fromTask)
            self.set_time(time)
            self.count += 1

    def remove_task_from(self, fromTask):
        index = self.taskFrom.index(fromTask)
        del self.agentFromAddr[index]
        del self.taskFrom[index]
        self.count -= 1

    def set_time(self, time):
        # earliest start time for a task is always the latest time of all ESTs
        if time == -1 or time > self.time:
            self.time = time

    def remove_agent_from(self, fromAgent):
        while fromAgent in self.agentFromAddr:
            index = self.agentFromAddr.index(fromAgent)
            del self.agentFromAddr[index]
            del self.taskFrom[index]
            self.count -= 1


if __name__ == "__main__":

    rospy.init_node("coordinator")

    try:
        coordination = MissionCoordinator()
    except rospy.ROSInterruptException:
        pass
