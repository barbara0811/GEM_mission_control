#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Initializes ROS node for generating schedule.

Summary:

    Publications:

    Subscriptions:
         * /namespace/mission_control [gpgp_agent/MissionCtrlMsg]
         * /namespace/task_structure_update [gpgp_agent/TaskStructureUpdate]

    Services:
         * /namespace/reschedule
         * /namespace/add_non_local_task
         * /namespace/remove_task
         * /namespace/add_commitment
         * /namespace/execute_method
         * /namespace/task_outcome_ev

    Parameters:
         param name="pack" -- name of a package which contains mission specific data
         param name="label" -- list of agent labels in taems structure

    Arguments:
         arg name="name"
"""
__author__ = 'barbanas'

import os
import copy
import json
import rospy
import rospkg
from sys import maxint
from itertools import izip
from datetime import datetime

from gem_mission_control_lib.taems import taems
import loader
import genetic_algorithm
from gem_mission_control_lib.utilities import helper_functions, my_logger

from gem_mission_control.msg import TaskStructureUpdate, MissionCtrlMsg
from gem_mission_control.srv import Reschedule, AddNonLocalTask, RemoveTask, AddCommitmentLocal, TaskOutcomeEV, ExecuteTask, \
    RescheduleResponse

logger = my_logger.CustomLogger()
rospack = rospkg.RosPack()


class DTCScheduler(object):
    """
    """

    def __init__(self, maxAlternativeNumber, maxAlternativeNumberRoot):

        logger.name = rospy.get_name()

        self.maxAlternativeNumber = maxAlternativeNumber
        self.maxAlternativeNumberRoot = maxAlternativeNumberRoot
        self.criteria = Criteria()
        self.plans = {}
        self.loader = loader.Loader()
        self.treeDict = {}  # key: mission id, value: taems tree instance
        self.missionStatus = {}
        self.completedMethods = {}

        self.label = rospy.get_param('~label').split(',')
        self.dataLocationPackage = rospy.get_param('~pack')

        rospy.Subscriber("task_structure_update", TaskStructureUpdate, self.update_task_structure)
        rospy.Subscriber("mission_control", MissionCtrlMsg, self.msg_mission_ctrl_callback)

        rospy.Service("reschedule", Reschedule, self.create_schedule_srv)
        rospy.Service("add_non_local_task", AddNonLocalTask, self.add_non_local_task_srv)
        rospy.Service("remove_task", RemoveTask, self.remove_task_srv)
        rospy.Service("add_commitment", AddCommitmentLocal, self.add_commitment_srv)
        rospy.Service("task_outcome_ev", TaskOutcomeEV, self.calc_task_outcome_ev_srv)
        rospy.Service("execute_method", ExecuteTask, self.execute_method_srv)

        rospy.spin()

    def msg_mission_ctrl_callback(self, msg):

        mission_sign = msg.root_task + "[" + str(msg.mission_id) + "]"

        if msg.type == "NewMission":
            self.missionStatus[mission_sign] = "wait"
            if mission_sign not in self.treeDict.keys():
                err = self.load_tree(msg.mission_id, msg.root_task)
                if err:
                    rospy.logerr('No tree structure!')
                    return
            self.completedMethods[mission_sign] = []
            self.missionStatus[mission_sign] = "loaded_tree"

        if msg.type == "Abort":
            if mission_sign in self.treeDict.keys():
                self.missionStatus[mission_sign] = "wait"
                self.treeDict.pop(mission_sign)
                self.completedMethods.pop(mission_sign)
                self.missionStatus.pop(mission_sign)

        if msg.type == "Completed":
            if msg.ag_addr == rospy.get_namespace():
                self.missionStatus[mission_sign] = "wait"
                self.treeDict.pop(mission_sign)
                self.completedMethods.pop(mission_sign)
                self.missionStatus.pop(mission_sign)

        if msg.type == "Restart":
            if mission_sign in self.treeDict.keys():
                self.missionStatus[mission_sign] = "wait"
                for task in self.treeDict[mission_sign].tasks.keys():
                    if self.treeDict[mission_sign].tasks[task].subtasks is None:
                        if self.treeDict[mission_sign].tasks[task].nonLocal:
                            self.treeDict[mission_sign].tasks.pop(task)

    def update_task_structure(self, msg):

        while msg.mission_id not in self.missionStatus.keys():
            pass

        while self.missionStatus[msg.mission_id] not in ["loaded_tree", "ok"]:
            rospy.sleep(0.1)

        self.missionStatus[msg.mission_id] = "wait"

        if msg.mission_id not in self.treeDict.keys():
            err = self.load_tree(msg.mission_id, msg.root_task)
            if err:
                rospy.logerr('No tree structure!')
                return

        outcomes = json.loads(msg.json_outcome)

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

        for i in range(len(msg.resource)):
            self.treeDict[msg.mission_id].resources[msg.resource[i]].state = msg.resource_state[i]

        self.missionStatus[msg.mission_id] = "ok"

    def load_tree(self, mission_id, task_label):
        """ Load taems structure. """
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

    def add_non_local_task_srv(self, req):
        while req.mission_id not in self.missionStatus.keys():
            pass

        while self.missionStatus[req.mission_id] != "ok":
            pass

        task = taems.Method()
        task.label = req.task_label
        task.supertasks = [req.supertask_label]
        q = {req.outcome_ev[0]: 1.0}
        d = {req.outcome_ev[1]: 1.0}
        c = {req.outcome_ev[2]: 1.0}
        task.outcome = [q, d, c]
        task.nonLocal = True

        self.treeDict[req.mission_id].tasks[req.task_label] = task

        return True

    def remove_task_srv(self, req):

        self.treeDict[req.mission_id].removeTaskAndSubtasks(req.task_label)
        return []

    def add_commitment_srv(self, req):

        while req.mission_id not in self.missionStatus.keys():
            pass

        while self.missionStatus[req.mission_id] != "ok":
            pass
        self.add_task_commitment(req.mission_id, req.type, req.task_label, req.time, req.break_time)
        return True

    def add_task_commitment(self, missionID, commType, task, time, breakTime):

        while missionID not in self.missionStatus.keys():
            pass

        if commType == "deadline":
            if self.treeDict[missionID].tasks[task].deadline is None or breakTime:
                self.treeDict[missionID].tasks[task].deadline = time
            elif self.treeDict[missionID].tasks[task].deadline > time:
                self.treeDict[missionID].tasks[task].deadline = time
        elif commType == "earliest_start_time":
            if self.treeDict[missionID].tasks[task].earliestStartTime is None or breakTime:
                self.treeDict[missionID].tasks[task].earliestStartTime = time
            elif self.treeDict[missionID].tasks[task].earliestStartTime < time:
                self.treeDict[missionID].tasks[task].earliestStartTime = time

        if self.treeDict[missionID].tasks[task].subtasks is None:
            return

        for subtask in self.treeDict[missionID].tasks[task].subtasks:
            if subtask not in self.treeDict[missionID].tasks.keys():
                continue
            if self.treeDict[missionID].tasks[subtask].subtasks is None:
                if self.treeDict[missionID].tasks[subtask].nonLocal is True:
                    continue
            self.add_task_commitment(missionID, commType, subtask, time, breakTime)

    def execute_method_srv(self, req):

        if req.task in self.completedMethods[req.mission_id]:
            return True

        self.completedMethods[req.mission_id].append(req.task)
        parent = self.treeDict[req.mission_id].tasks[req.task].supertasks[0]
        self.treeDict[req.mission_id].tasks[parent].subtasks.remove(req.task)
        self.treeDict[req.mission_id].tasks.pop(req.task)

        return True

    def get_disjunct_task_sets(self, tree):
        sets = self._get_disjunct_task_sets(tree, tree.rootTask[0])
        return sets

    def _get_disjunct_task_sets(self, tree, taskLabel):

        if type(tree.tasks[taskLabel]) is taems.Method:
            return [[taskLabel]]
        else:
            s = []
            for subtask in tree.tasks[taskLabel].subtasks:
                if subtask in tree.tasks.keys():
                    s.extend(self._get_disjunct_task_sets(tree, subtask))
                else:
                    s.extend([[subtask]])
            if 'seq' not in tree.tasks[taskLabel].qaf:
                ret = [[taskLabel] for i in range(len(s))]

                for i in range(len(s)):
                    ret[i].extend(s[i])
                return ret
            else:
                ret = [taskLabel]
                for x in s:
                    ret.extend(x)
                return [ret]

    def create_schedule_srv(self, req):
        """
        Create a schedule for the specified mission.

        Args:
            req (RescheduleRequest): Service request data with mission ID.

        Returns:
            Success indicator, tasks schedule and completed tasks.
            return[0] (int): Success (error) indicator.
            return[1] (list): Scheduled tasks.
            return[2] (list): Start times of scheduled tasks.
            return[3] (list): End times of scheduled tasks.
            return[4] (list): Completed tasks.
            return[5] (list): Non-local completed tasks.
        """

        while req.mission_id not in self.missionStatus.keys():
            pass

        while self.missionStatus[req.mission_id] != "ok":
            pass

        tree = self.treeDict[req.mission_id]

        self.criteria.load(rospack.get_path(self.dataLocationPackage) + '/data/Criteria/' + req.criteria)

        # Step 1: create alternatives.
        self.plans = {}
        self.missingTasks = []
        disjunctTaskSets = self.get_disjunct_task_sets(tree)
        rootPlan = self.create_alternatives(tree, tree.tasks[tree.rootTask[0]], False, disjunctTaskSets)

        if len(rootPlan.alternatives) == 0:
            rospy.logerr('Can\'t create schedule, agents performing tasks %s are missing!', self.missingTasks)
            return [RescheduleResponse.ERROR_2, [], [], [], [], []]

        sumQEV = 0
        for item in rootPlan.qualityEV:
            sumQEV += item


        # Step 2: pick best alternatives to schedule.
        rating = self.criteria.evaluate_plan(rootPlan)

        sched = None
        genAlg = genetic_algorithm.GeneticAlgorithm(30, 0.3, 0.4)
        sortedPlan = DTCScheduler.sort_by_rating(rootPlan, rating)

        # Print out top 5 plans.
        output = []
        for index, plan in izip(range(1, 6), sortedPlan):
            output.append('{}. {}'.format(index, plan))
        logger.info('Top 5 plans:\n%s%s\n', '\n'.join(output), '-' * 20)

        # TODO non local task is not in tree!? <----
        while len(sortedPlan) > 0:
            nonLocal = []
            toSchedule = []
            for method in sortedPlan[0].alternatives[0]:
                if self.treeDict[req.mission_id].tasks[method].nonLocal:
                    nonLocal.append(method)
                    continue
                toSchedule.append(method)

            if len(toSchedule) == 0:
                if len(nonLocal) > 0:
                    return [RescheduleResponse.ERROR_1, [], [], [], [], []]
                else:
                    return [RescheduleResponse.ERROR_2, [], [], [], [], []]

            sched = genAlg.optimize(toSchedule, nonLocal, tree, 100, req.mission_start_time)
            if sched is None:
                # schedule next best alternative
                del sortedPlan[0]
            else:
                break

        if sched is None:
            rospy.loginfo('Quality of root task is 0! Schedule won\'t be created...')
            logger.debug('Resources: ')
            for resource in tree.resources.values():
                logger.debug('    label %s', resource.label)
                logger.debug('    state after scheduling %s', resource.state)
                logger.debug('    allowed range <%s,%s>', resource.depleted_at, resource.overloaded_at)
                logger.debug('---\n')
            return [RescheduleResponse.NO_SCHEDULE, [], [], [], [], []]

        taskLabels = []
        startTimes = []
        endTimes = []

        for x in sched:
            taskLabels.append(x[0])
            startTimes.append(x[1])
            endTimes.append(x[2])

        return [RescheduleResponse.SUCCESS, taskLabels, startTimes, endTimes, sortedPlan[0].completedTasks[0], nonLocal]

    def calc_task_outcome_ev_srv(self, req):
        """
        Calculates task outcome as average outcome of all alternatives (plans) that complete task.
        """
        while req.mission_id not in self.missionStatus.keys():
            pass

        logger.debug('Waiting for task assessor %s...', req.mission_id)
        while self.missionStatus[req.mission_id] != "ok":
            pass
        logger.debug('... OK')

        tree = self.treeDict[req.mission_id]
        self.criteria.load(rospack.get_path(self.dataLocationPackage) + '/data/Criteria/' + req.criteria)
        logger.debug('Loaded criteria.')

        # Step 1: create alternatives.
        self.plans = {}
        self.missingTasks = []
        disjunctTaskSets = self.get_disjunct_task_sets(tree)
        self.create_alternatives(tree, tree.tasks[tree.rootTask[0]], True, disjunctTaskSets)
        tasks = self.plans.keys()
        averageEV = []
        for task in tasks:
            q = self.plans[task].qualityEV
            d = self.plans[task].durationEV
            c = self.plans[task].costEV

            if len(q) == 0:
                averageEV.append([0, 0, 0])
                output = []
                output.append(str(task))
                output.append(str(q))
                output.append(str(self.plans[task].alternatives))
                output.append(str(self.treeDict[req.mission_id].tasks[task].subtasks))
                output.append(str(self.treeDict[req.mission_id].tasks[task].supertasks))
                output.append(str(self.treeDict[req.mission_id].tasks[task].qaf))
                rospy.logerr('taems is not defined correctly! More info:\n%s', '\n'.join(output))
                return [[], json.dumps([])]

            toRemove = []
            for i in range(len(q)):
                if q[i] == 0 or d[i] > pow(10, 5) or c[i] > pow(10, 5):
                    toRemove.append(i)
            removed = 0
            for i in toRemove:
                del q[i - removed]
                del d[i - removed]
                del c[i - removed]
                removed += 1

            self.plans[task].qualityEV = q
            self.plans[task].durationEV = d
            self.plans[task].costEV = c

            if len(q) == 0:
                averageEV.append([0, maxint, maxint])
            else:
                averageEV.append([sum(q) / len(q), sum(d) / len(d), sum(c) / len(c)])

        logger.debug('Calculated task outcome EV.\n')
        return [tasks, json.dumps(averageEV)]

    @staticmethod
    def sort_by_rating(plan, rating):
        """
        Sort plan alternatives by their rating.

        Args:
            plan (Plan): Plan to sort.
            rating (list[float]): Ratings for each alternative.

        Returns:
            Sorted plan.
        """
        temp = copy.deepcopy(rating)
        sortedPlan = []

        for i in range(len(rating)):
            index = temp.index(max(temp))

            p = Plan(plan.task)
            p.alternatives = [plan.alternatives[index]]
            p.cost = [plan.cost[index]]
            p.duration = [plan.duration[index]]
            p.quality = [plan.quality[index]]
            p.costEV = [plan.costEV[index]]
            p.durationEV = [plan.durationEV[index]]
            p.qualityEV = [plan.qualityEV[index]]
            p.subtasks = plan.subtasks
            p.methods = plan.methods
            p.completedTasks = [plan.completedTasks[index]]
            p.childrenAlternativeIndex = [plan.childrenAlternativeIndex[index]]
            p.rating = [temp[index]]

            sortedPlan.append(p)
            temp[index] = -1

        return sortedPlan

    def create_alternatives(self, tree, root, ignoreNonLocal, disjunctTaskSets):
        """
        Method for creating alternatives "bottom - up" -> recursively.

        Args:
            tree - TaemsTree
            root - TaskGroup / Method
        """
        # stopping condition
        if type(root) is taems.Method:
            myPlan = Plan(root.label)
            myPlan.quality = [tree.tasks[root.label].outcome[0]]
            myPlan.duration = [tree.tasks[root.label].outcome[1]]
            myPlan.cost = [tree.tasks[root.label].outcome[2]]
            myPlan.alternatives = [[root.label]]
            myPlan.methods.append(root.label)
            myPlan.completedTasks.append([])
            myPlan.qualityEV = [helper_functions.calcExpectedValue(myPlan.quality[0])]
            myPlan.costEV = [helper_functions.calcExpectedValue(myPlan.cost[0])]
            myPlan.durationEV = [helper_functions.calcExpectedValue(myPlan.duration[0])]

            self.plans[root.label] = myPlan
            return

        # create a plan instance
        myPlan = Plan(root.label)

        index = 0
        # go through every child:
        for i in range(len(root.subtasks)):
            if root.subtasks[i] in tree.tasks.keys():
                myPlan.subtasks.append(root.subtasks[i])
                self.create_alternatives(tree, tree.tasks[root.subtasks[i]], ignoreNonLocal, disjunctTaskSets)
                myPlan.methods.extend(self.plans[root.subtasks[i]].methods)
                # childrenPlansMethods.append(childrenPlans[index].alternatives)
                index += 1
            else:
                if ignoreNonLocal is False:
                    if root.qaf == "q_min" or "_all" in root.qaf:
                        self.missingTasks.append(root.label)
                        myPlan = Plan(root.label)
                        self.plans[root.label] = myPlan
                        return myPlan

        for i in range(len(myPlan.subtasks)):
            plan = self.plans[myPlan.subtasks[i]]
            myPlan.childrenAlternativeIndex.append([])
            for j in range(len(plan.alternatives)):
                myPlan.childrenAlternativeIndex[-1].append([i, j])

        if root.qaf == 'q_sum_all' or root.qaf == 'q_min' or root.qaf == 'q_seq_sum_all':
            # all children's plans MUST be included in parent's plan
            myPlan.childrenAlternativeIndex = helper_functions.cartesianProduct(myPlan.childrenAlternativeIndex,
                                                                                removeDuplicates=False, sort=False)

        elif root.qaf == 'q_max':
            # only one children's plan MUST be included
            temp = []
            for item in myPlan.childrenAlternativeIndex:
                if type(item) is list:
                    for element in item:
                        temp.append([element])
                else:
                    temp.append([item])

            myPlan.childrenAlternativeIndex = temp
            # myPlan.childrenAlternativeIndex = helper_functions.OR(myPlan.childrenAlternativeIndex)
            pass

        elif root.qaf == 'q_sum':
            # power set of children's plans
            temp = []
            for x in helper_functions.powerSet(myPlan.childrenAlternativeIndex):
                temp.append(x)

            # get rid of excess of lists crated by function powerSet
            for i in range(len(temp)):
                for j in range(len(temp[i])):
                    temp[i][j] = temp[i][j][0]

            myPlan.childrenAlternativeIndex = temp

        if len(myPlan.childrenAlternativeIndex) > 0:
            myPlan.evaluate(root.qaf, self.plans, disjunctTaskSets)

            if len(root.supertasks) > 0:
                self.criteria.trim_alternatives(myPlan, self.maxAlternativeNumber)
            else:
                self.criteria.trim_alternatives(myPlan, self.maxAlternativeNumberRoot)

            # add actual methods into alternatives
            for alternative in myPlan.childrenAlternativeIndex:
                myPlan.alternatives.append([])
                myPlan.completedTasks.append([])
                myPlan.completedTasks[-1].append(root.label)  # myPlan.subtasks[method[0]])
                for method in alternative:
                    alt = self.plans[myPlan.subtasks[method[0]]].alternatives[method[1]]
                    myPlan.alternatives[-1].extend(alt)
                    myPlan.completedTasks[-1].extend(self.plans[myPlan.subtasks[method[0]]].completedTasks[method[1]])

            if len(myPlan.alternatives) > 0:
                toRemove = []
                for i in range(len(myPlan.alternatives)):
                    if i in toRemove:
                        continue
                    for item in tree.mutuallyExclusiveTasks:
                        tasks = set(myPlan.completedTasks[i]) | set(myPlan.alternatives[i])
                        if len(set(item) & tasks) > 1:
                            toRemove.append(i)
                            break

                removed = 0
                for i in toRemove:
                    myPlan.remove_alternative(i - removed)
                    removed += 1

        self.plans[root.label] = myPlan
        return myPlan

    @staticmethod
    def print_to_file(listOfPlans):
        fileName = os.getcwd() + "\Output\Output_plans " + str(datetime.now())[:-7].replace(':', '-')
        file1 = open(fileName, 'w')
        file2 = open(os.getcwd() + "\Outcome_lastOutput.txt", 'w')

        file1.write(str(listOfPlans[0].qualityEV) + "\t")
        file1.write(str(listOfPlans[0].durationEV) + "\t")
        file1.write(str(listOfPlans[0].costEV) + "\n")

        for plan in listOfPlans:
            file1.write(str(plan.qualityEV) + "\t")
            file1.write(str(plan.durationEV) + "\t")
            file1.write(str(plan.costEV) + "\n")

            file2.write(str(plan.qualityEV) + "\t")
            file2.write(str(plan.durationEV) + "\t")
            file2.write(str(plan.costEV) + "\n")

        file1.close()
        file2.close()


class Criteria(object):
    """Criteria models criteria sliders to calculate goodness of each alternative.
        For more info, please consult manual on DTC criteria sliders.
    """

    def __init__(self):
        self.rawGoodnessQ = 0
        self.rawGoodnessC = 0
        self.rawGoodnessD = 0
        self.thresholdQ = [0, 0]
        self.limitC = [0, 0]
        self.limitD = [0, 0]
        self.certaintyQ = 0
        self.certaintyC = 0
        self.certaintyD = 0
        self.certaintyThresholdQ = [0, 0]
        self.certaintyThresholdC = [0, 0]
        self.certaintyThresholdD = [0, 0]
        self.metaRawGoodness = 0
        self.metaLimitThreshold = 0
        self.metaCertainty = 0
        self.metaCertaintyThresholds = 0

    def load(self, filename):
        """Loads criteria from file given with filename. A structure of criteria
            specification file and an example can be found in documentation.

            Args:
                filename: A string

            Returns:
                -
        """
        with open(filename, 'r') as stream:
            lines = stream.readlines()

        index = Criteria.skip_lines(lines, 0)
        self.rawGoodnessQ = float(lines[index].strip())

        index = Criteria.skip_lines(lines, index + 1)
        self.rawGoodnessC = float(lines[index].strip())

        index = Criteria.skip_lines(lines, index + 1)
        self.rawGoodnessD = float(lines[index].strip())

        index = Criteria.skip_lines(lines, index + 1)

        if lines[index].strip()[0] != '0':
            array = lines[index].strip().split(' ')
            self.thresholdQ[0] = float(array[0])
            self.thresholdQ[1] = float(array[1])

        index = Criteria.skip_lines(lines, index + 1)
        if lines[index].strip()[0] != '0':
            array = lines[index].strip().split(' ')
            self.limitC[0] = float(array[0])
            self.limitC[1] = float(array[1])

        index = Criteria.skip_lines(lines, index + 1)
        if lines[index].strip()[0] != '0':
            array = lines[index].strip().split(' ')
            self.limitD[0] = float(array[0])
            self.limitD[1] = float(array[1])

        index = Criteria.skip_lines(lines, index + 1)
        self.certaintyQ = float(lines[index].strip())

        index = Criteria.skip_lines(lines, index + 1)
        self.certaintyC = float(lines[index].strip())

        index = Criteria.skip_lines(lines, index + 1)
        self.certaintyD = float(lines[index].strip())

        index = Criteria.skip_lines(lines, index + 1)
        if lines[index].strip()[0] != '0':
            array = lines[index].strip().split(' ')
            self.certaintyThresholdQ[0] = float(array[0])
            self.certaintyThresholdQ[1] = float(array[1])

        index = Criteria.skip_lines(lines, index + 1)
        if lines[index].strip()[0] != '0':
            array = lines[index].strip().split(' ')
            self.certaintyThresholdC[0] = float(array[0])
            self.certaintyThresholdC[1] = float(array[1])

        index = Criteria.skip_lines(lines, index + 1)
        if lines[index].strip()[0] != '0':
            array = lines[index].strip().split(' ')
            self.certaintyThresholdD[0] = float(array[0])
            self.certaintyThresholdD[1] = float(array[1])

        index = Criteria.skip_lines(lines, index + 1)
        self.metaRawGoodness = float(lines[index].strip())

        index = Criteria.skip_lines(lines, index + 1)
        self.metaLimitThreshold = float(lines[index].strip())

        index = Criteria.skip_lines(lines, index + 1)
        self.metaCertainty = float(lines[index].strip())

        index = Criteria.skip_lines(lines, index + 1)
        self.metaCertaintyThresholds = float(lines[index].strip())

    def dump_to_file(self, filename):
        line = '#' + ' -' * 11
        with open(filename, 'w') as dump_file:
            header = '\n'.join([line, '# 1. Raw Goodness [%]', line])
            values = '# Quality\n{s.rawGoodnessQ}\n' \
                     '# Cost\n{s.rawGoodnessC}\n' \
                     '# Duration\n{s.rawGoodnessD}\n'.format(s=self)
            dump_file.write(header + '\n' + values + '\n')

            header = '\n'.join([line, '# 2. Threshold & Limits [% value]', line])
            values = '# Quality threshold\n{s.thresholdQ[0]} {s.thresholdQ[1]}\n' \
                     '# Cost limit\n{s.limitC[0]} {s.limitC[1]}\n' \
                     '# Duration limit\n{s.limitD[0]} {s.limitD[1]}\n'.format(s=self)
            dump_file.write(header + '\n' + values + '\n')

            header = '\n'.join([line, '# 3. Certainty [%]', line])
            values = '# Quality\n{s.certaintyQ}\n' \
                     '# Cost\n{s.certaintyC}\n' \
                     '# Duration\n{s.certaintyD}\n'.format(s=self)
            dump_file.write(header + '\n' + values + '\n')

            header = '\n'.join([line, '# 4. Certainty Thresholds [% value]', line])
            values = '# Quality threshold\n{s.certaintyThresholdQ[0]} {s.certaintyThresholdQ[1]}\n' \
                     '# Cost limit\n{s.certaintyThresholdC[0]} {s.certaintyThresholdC[1]}\n' \
                     '# Duration limit\n{s.certaintyThresholdD[0]} {s.certaintyThresholdD[1]}\n'.format(s=self)
            dump_file.write(header + '\n' + values + '\n')

            header = '\n'.join([line, '# 5. Meta [%]', line])
            values = '# Raw goodness\n{s.metaRawGoodness}\n' \
                     '# Threshold & Limits\n{s.metaLimitThreshold}\n' \
                     '# Certainty\n{s.metaCertainty}\n' \
                     '# Certainty Thresholds\n{s.metaCertaintyThresholds}'.format(s=self)
            dump_file.write(header + '\n' + values + '\n')

    @staticmethod
    def skip_lines(lines, index):

        while index < len(lines):
            if len(lines[index].strip()) == 0:
                index += 1
            elif lines[index][0] == "#":
                index += 1
            else:
                return index
        return index

    def evaluate_plan(self, plan):
        """Evaluates a plan by criteria values.

            Args:
                plan - A plan object to evaluate

            Returns:
                totalRating - A list of float values, ratings for every alternative in the given plan
        """
        rating = self.evaluate(plan.quality, plan.duration, plan.cost, plan.qualityEV, plan.durationEV, plan.costEV)
        plan.rating = rating
        return rating

    def evaluate(self, quality, duration, cost, qualityEV, durationEV, costEV):
        """
        """
        # 1. Find min and max expected values for quality, cost and duration:
        minQ = min(qualityEV)
        minC = min(costEV)
        minD = min(durationEV)

        maxQ = max(qualityEV)
        maxC = max(costEV)
        maxD = max(durationEV)

        # 2. Calculate raw goodness:
        sumRG = self.rawGoodnessC + self.rawGoodnessD + self.rawGoodnessQ
        ratingRG = []
        if sumRG > 0:
            ratingRGQ = []
            ratingRGC = []
            ratingRGD = []

            testQ = (maxQ == minQ)
            testC = (maxC == minC)
            testD = (maxD == minD)

            for i in range(len(qualityEV)):
                ratingRGQ.append((1 if testQ else (qualityEV[i] - minQ) / (maxQ - minQ)) * self.rawGoodnessQ / sumRG)
                ratingRGC.append((1 if testC else (maxC - costEV[i]) / (maxC - minC)) * self.rawGoodnessC / sumRG)
                ratingRGD.append((1 if testD else (maxD - durationEV[i]) / (maxD - minD)) * self.rawGoodnessD / sumRG)

                ratingRG.append(ratingRGQ[i] + ratingRGC[i] + ratingRGD[i])

        # 3. Calculate thresholds and limits:
        sumTL = self.thresholdQ[0] + self.limitC[0] + self.limitD[0]
        ratingTL = []
        if sumTL > 0:
            ratingTLQ = []
            ratingTLC = []
            ratingTLD = []

            for i in range(len(qualityEV)):
                ratingTLQ.append(self.thresholdQ[0] / sumTL if qualityEV[i] > self.thresholdQ[1] else 0)
                ratingTLC.append(self.limitC[0] / sumTL if costEV[i] < self.limitC[1] else 0)
                ratingTLD.append(self.limitD[0] / sumTL if durationEV[i] < self.limitD[1] else 0)

                ratingTL.append(ratingTLQ[i] + ratingTLC[i] + ratingTLD[i])

        # 4. Calculate certainty:
        sumCE = self.certaintyC + self.certaintyD + self.certaintyQ
        ratingCE = []
        if sumCE > 0:
            probQ = []
            probC = []
            probD = []

            for i in range(len(qualityEV)):
                probQ.append(helper_functions.probXGreaterThanVal(quality[i], qualityEV[i]))
                probC.append(helper_functions.probXSmallerThanVal(cost[i], costEV[i]))
                probD.append(helper_functions.probXSmallerThanVal(duration[i], durationEV[i]))

            minProbQ = min(probQ)
            minProbC = min(probC)
            minProbD = min(probD)

            maxProbQ = max(probQ)
            maxProbC = max(probC)
            maxProbD = max(probD)

            ratingCEQ = []
            ratingCEC = []
            ratingCED = []

            testQ = (maxProbQ == minProbQ)
            testC = (maxProbC == minProbC)
            testD = (maxProbD == minProbD)

            for i in range(len(qualityEV)):
                if testQ:
                    ratingCEQ.append(1 * self.certaintyQ / sumCE)
                else:
                    ratingCEQ.append(((probQ[i] - minProbQ) / (maxProbQ - minProbQ)) * (self.certaintyQ / sumCE))
                if testC:
                    ratingCEC.append(1 * self.certaintyC / sumCE)
                else:
                    ratingCEC.append(((probC[i] - minProbC) / (maxProbC - minProbC)) * (self.certaintyC / sumCE))
                if testD:
                    ratingCED.append(1 * self.certaintyD / sumCE)
                else:
                    ratingCED.append(((probD[i] - minProbD) / (maxProbD - minProbD)) * (self.certaintyD / sumCE))

                ratingCE.append(ratingCEQ[i] + ratingCEC[i] + ratingCED[i])

        # 5. Calculate certainty threshold:
        sumCET = self.certaintyThresholdC[0] + self.certaintyThresholdD[0] + self.certaintyThresholdQ[0]
        ratingCET = []
        if sumCET > 0 and sumCE > 0:
            ratingCETQ = []
            ratingCETC = []
            ratingCETD = []

            for i in range(len(qualityEV)):
                if probQ[i] > self.certaintyThresholdQ[1]:
                    ratingCETQ.append(self.certaintyThresholdQ[0] / sumTL)
                else:
                    ratingCETQ.append(0)

                if probC[i] < self.certaintyThresholdC[1]:
                    ratingCETC.append(self.certaintyThresholdC[0] / sumTL)
                else:
                    ratingCETC.append(0)

                if probD[i] < self.certaintyThresholdD[1]:
                    ratingCETD.append(self.certaintyThresholdD[0] / sumTL)
                else:
                    ratingCETD.append(0)

                ratingCET.append(ratingCETQ[i] + ratingCETC[i] + ratingCETD[i])

        # 6. Calculate total rating:
        totalRating = []
        for i in range(len(qualityEV)):
            finalRG = ratingRG[i] if ratingRG else 0
            finalTL = ratingTL[i] if ratingTL else 0
            finalCE = ratingCE[i] if ratingCE else 0
            finalCET = ratingCET[i] if ratingCET else 0

            totalRating.append (finalRG * self.metaRawGoodness
                                + finalTL * self.metaLimitThreshold
                                + finalCE * self.metaCertainty
                                + finalCET * self.metaCertaintyThresholds)

        return totalRating

    def trim_alternatives(self, plan, n):
        """Trims worst alternatives (the ones with lowest rating) so that plan consists of
            maximum n alternatives.

            Args:
                plan - A plan object
                n - An integer
        """
        if n == -1:
            return

        ratings = self.evaluate_plan(plan)

        while len(plan.alternatives) > n:
            i = ratings.index(min(ratings))
            plan.remove_alternative(i)
            del ratings[i]


class Plan(object):
    """
    Container for alternatives (unordered lists of methods) that lead to completion of the task.

    Attributes:
        alternatives (list[list]): Alternatives -> [['method1','m2', ...], ['m3','m4', ...], ['m2','m3','m4', ...], ...]
        quality (list[dict]): Quality probability distributions.
        cost (list[dict]): Cost probability distributions.
        duration (list[dict]): Duration probability distributions.
        qualityEV (list[float]): Expected values of quality for every alternative.
        costEV (list[float]) Expected values of cost for every alternative.
        durationEV (list[float]) Expected values of duration for every alternative.
    """

    def __init__(self, task):
        self.task = task
        self.childrenAlternativeIndex = []
        self.alternatives = []  # [['method1','m2', ...], ['m3','m4', ...], ['m2','m3','m4', ...], ...]
        self.quality = []
        self.cost = []
        self.duration = []
        self.qualityEV = []
        self.costEV = []
        self.durationEV = []
        self.subtasks = []
        self.methods = []
        self.completedTasks = []
        self.rating = []

    def __str__(self):
        output = []
        rating = self.rating if self.rating else ['N/A'] * len(self.alternatives)
        for plan in izip(self.alternatives, self.qualityEV, self.durationEV, self.costEV, rating):
            output.append('{}\nQuality: {}\nDuration: {}\nCost: {}\nRating: {}\n'.format(*plan))
        return '\n'.join(output)

    def evaluate(self, qaf, plans, disjunctTaskSets):
        """Calculates quality, cost, duration (probability distribution) for every
                alternative (list of methods) in self.alternatives -> by calling calc_QCD method.
            Calculates quality, cost and duration expected values for every alternative.

            Args:
                qaf - string, predefined set of values for quality accumulation function of a task
                plans - dictionary of plans for every task
        """
        self.quality = []
        self.cost = []
        self.duration = []

        self.qualityEV = []
        self.costEV = []
        self.durationEV = []

        print len(self.alternatives)
        print qaf
        for i in range(len(self.alternatives)):
            # print len(self.childrenAlternativeIndex[i])
            # raw_input("")
            alternative = self.childrenAlternativeIndex[i]
            temp = self.calc_QCD(qaf, alternative, plans, disjunctTaskSets)

            self.quality.append(temp[0])
            self.duration.append(temp[1])
            self.cost.append(temp[2])

            self.qualityEV.append(helper_functions.calcExpectedValue(self.quality[i]))
            self.costEV.append(helper_functions.calcExpectedValue(self.cost[i]))
            self.durationEV.append(helper_functions.calcExpectedValue(self.duration[i]))

    def calc_QCD(self, qaf, alternative, plans, disjunctTaskSets):
        """
        Calculates quality , cost, duration (probability distribution) for
        alternative (list of methods) with respect of given qaf.

        Args:
            qaf (str): Predefined set of values for quality accumulation function of a task.
            alternative (list[str]): List of method labels.
            plans (dict): Plans for every task.

        Returns:
            Quality, cost and duration distributions.
        """
        qualityDistributions = []
        costDistributions = []
        durationDistributions = []

        childWithZeroQ = False
        # d = [[] for i in range(len(disjunctTaskSets))]
        # durDistr = [[] for i in range(len(disjunctTaskSets))]
        for i in alternative:
            child = i[0]
            ind = -1
            # st = rospy.get_time()
            # for j in range(len(disjunctTaskSets)):
            #     print j
            #     if self.subtasks[child] in disjunctTaskSets[j]:
            #         d[j].append(child)
            #         ind = j
            # print rospy.get_time() - st
            childsAlternative = i[1]
            qualityDistributions.append(plans[self.subtasks[child]].quality[childsAlternative])
            costDistributions.append(plans[self.subtasks[child]].cost[childsAlternative])
            durationDistributions.append(plans[self.subtasks[child]].duration[childsAlternative])
            #durDistr[ind].append(plans[self.subtasks[child]].duration[childsAlternative])
            if plans[self.subtasks[child]].qualityEV == 0:
                childWithZeroQ = True
        
        # TODO: what if qaf is q_exactly_one?
        fun = ""
        if qaf == "q_min":
            fun = "min"
        elif qaf == "q_max":
            fun = "max"
        elif qaf == 'q_sum':
            fun = "sum"
        elif qaf == 'q_sum_all' or qaf == "q_seq_sum_all":
            fun = "sum"
            if childWithZeroQ:
                print "zero q child"
                return [{0: 1.0}, {maxint: 1.0}, {maxint: 1.0}]
        else:
            rospy.logerr('Case for qaf \"%s\" is not defined.', qaf)
            return [{0: 1.0}, {maxint: 1.0}, {maxint: 1.0}]

        Q = helper_functions.cartesianProductOfDistributions(qualityDistributions, fun)
        C = helper_functions.cartesianProductOfDistributions(costDistributions, "sum")
        # Ds = []
        # # print durDistr
        # for i in range(len(durDistr)):
        #     if len(durDistr[i]) > 1:
        #         Ds.append(helper_functions.cartesianProductOfDistributions(durDistr[i], "sum"))
        #     elif len(durDistr[i]) == 1:
        #         Ds.append(durDistr[i][0])

        # print Ds
        # raw_input()
        # if len(Ds) > 1:
        #     D = helper_functions.cartesianProductOfDistributions(Ds, "max")
        # elif len(Ds) == 1:
        #     D = Ds[0]
        # else:
        #     D = {0: 1}
        D = helper_functions.cartesianProductOfDistributions(durationDistributions, "sum") #TODO !!! duration is NOT a sum !!!

        return [Q, D, C]

    def remove_alternative(self, i):
        """Removes an alternative at index i from a plan.
        """
        del self.quality[i]
        del self.cost[i]
        del self.duration[i]
        del self.qualityEV[i]
        del self.costEV[i]
        del self.durationEV[i]
        del self.completedTasks[i]

        if len(self.alternatives) > 0:
            del self.alternatives[i]
        del self.childrenAlternativeIndex[i]


if __name__ == "__main__":

    rospy.init_node("scheduler")

    try:
        dtc_scheduler = DTCScheduler(-1, -1)
    except rospy.ROSInterruptException:
        pass
