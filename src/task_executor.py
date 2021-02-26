#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = "barbanas"

import rospy

import actionlib
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import String

from gpgp_agent.msg import MissionCtrlMsg, MissionEnabledTasks, TaskStructureUpdate
from gpgp_agent.srv import ScheduleOK, AdjustScheduleTimes, MissionEarliestStart, ExecuteTask, DelaySchedule, GetPose
from utilities import my_logger

logger = my_logger.CustomLogger()


class TaskExecutor(object):

    def __init__(self):
        logger.name = rospy.get_name()

        self.label = rospy.get_param('~label')

        self.initTime = rospy.get_time()
        self.time = 0.0
        self.feedback = -1
        self.executionSchedule = []  # [method label, start time, predicted end time, mission id]
        self.executableMission = []
        self.missionOrder = []
        self.missions = {}  # key: mission id, value = [mission earliest start time, mission end time]
        self.tasks = {}  # key : mission id, value : dictionary {key : method label, value : [start - Point(), end : Point(), if moving target -- address]}
        self.enabledTasks = {}  # key : mission id, value : list of enabled tasks

        self.bestSchedule = {}  # [[task labels], [start times], [end times]]
        self.taskEST = {}  # key : mission id, value : dictionary {key: task label, value: earliest start time}
        self.missionStatus = {}
        self.missionType = {}

        self.waitScheduleDelay = False

        self.waitUAVGoTo = [False, None]  # [wait(T/F), last uav go to label]

        rospy.Subscriber("mission_control", MissionCtrlMsg, self.msg_mission_ctrl_callback)
        self.missionCtrlMsgPub = rospy.Publisher("/mission_control", MissionCtrlMsg, queue_size=1)
        rospy.Subscriber('mission_enabled_tasks', MissionEnabledTasks, self.mission_enabled_tasks_cb)
        rospy.Subscriber("task_structure_update", TaskStructureUpdate, self.update_task_structure_callback)

        rospy.Service("adjust_schedule_times", AdjustScheduleTimes, self.adjust_schedule_times_srv)
        rospy.Service("schedule_ok_execute", ScheduleOK, self.register_schedule_ok_srv)
        rospy.Service("mission_earliest_start", MissionEarliestStart, self.mission_earliest_start_srv)
        rospy.Service("delay_schedule", DelaySchedule, self.delay_schedule_srv)

        self.executionCtrlMsgPub = rospy.Publisher("execution_control", String, queue_size=2)
        rospy.Subscriber("execution_control", String, self.msg_mission_execution_callback)

    def update_time(self):
        self.time = rospy.get_time()

    def adjust_schedule_times(self, missionID, tasks, startTimes, endTimes, constrained, changeExecSchedule):

        if changeExecSchedule:
            sched = self.executionSchedule
        else:
            # sched = deepcopy(self.executionSchedule)
            sched = []  # TODO temporary solution -- only one mission at a time

        index = 0
        for m in range(len(tasks)):
            if tasks[m] == "slack":
                continue

            est = -1
            if tasks[m] in constrained.keys():
                est = constrained[tasks[m]]

            taskEarliestStart = -1
            if m == 0:
                taskEarliestStart = max([self.missions[missionID][0], est])
            elif est != -1:
                taskEarliestStart = est

            duration = endTimes[m] - startTimes[m]
            slackDuration = 0

            if self.missionType[missionID] == "Homogeneous":
                index = len(sched)
            elif len(sched) > 0 and index < len(sched):
                while True:
                    if sched[index][3] == "_slack_" and sched[index][1] > taskEarliestStart:
                        slackDuration = sched[index][2] - sched[index][1]
                        if slackDuration >= duration:
                            break
                    index += 1
                    if index == len(sched):
                        break

            # insert task
            if index == len(sched):
                # tasks are put into schedule in order, there are no gaps except those who are scheduled
                if len(sched) == 0:
                    startTime = 0
                else:
                    startTime = sched[-1][2]

                slack = 0
                if taskEarliestStart > startTime:
                    slack = taskEarliestStart - startTime
                    if self.missionType[missionID] == "Homogeneous":
                        sched.append(["slack", startTime, taskEarliestStart, missionID])
                        index += 1
                sched.append([tasks[m], startTime + slack, startTime + slack + duration, missionID])
                index += 1
            else:
                # index points to "_slack_" -> Heterogeneous
                sched.append([tasks[m], sched[index - 1][2], sched[index - 1][2] + duration, missionID])
                index += 1

                slackDuration -= duration
                sched.append(["slack", sched[index - 1][2], sched[index - 1][2] + slackDuration, "_slack_"])
                index += 1

        # garbage collection of "_slack_" blocks
        if changeExecSchedule:
            j = 0
            mission = ""
            while True:
                if j == len(sched) - 1:
                    break
                if mission != sched[j + 1][3]:
                    mission = sched[j + 1][3]
                if sched[j][2] != sched[j + 1][1]:
                    start = sched[j][2]
                    end = sched[j + 1][1]
                    sched.insert(j + 1, ["slack", start, end, mission])
                j += 1
            executionSchedule = [[x[0], round(x[1], 3), round(x[2], 3), x[3]] for x in self.executionSchedule]
            rospy.loginfo('New executor schedule:\n%s\n%s\n', executionSchedule, '-' * 20)
        return sched

    def extract_mission_schedule(self, schedule, missionID):

        tasks = []
        startTimes = []
        endTimes = []

        for item in schedule:
            if item[3] == missionID:
                tasks.append(item[0])
                startTimes.append(item[1])
                endTimes.append(item[2])

        i = 0
        while i < len(tasks) - 1:
            if endTimes[i] != startTimes[i + 1]:
                tasks.insert(i + 1, "slack")
                startTimes.insert(i + 1, endTimes[i])
                endTimes.insert(i + 1, startTimes[i + 1])
                i += 1
            i += 1

        return [tasks, startTimes, endTimes]

    def init_task_execution(self, *args, **kwargs):
        raise NotImplementedError('Must override method init_task_execution!')

    def init_slack_execution(self, *args, **kwargs):
        pass

    def finish_mission(self):
        pass

    # ************************** #
    # CALLBACK FUNCTIONS SECTION #
    # ************************** #

    def msg_mission_ctrl_callback(self, msg):
        """
        Callback function for "mission_control" topic.

        Args:
            msg (MissionCtrlMsg): Incoming message.
        """
        mission_sign = msg.root_task + "[" + str(msg.mission_id) + "]"

        if msg.type == "StartMission":
            if msg.ag_addr == rospy.get_namespace():
                if mission_sign not in self.missions.keys():
                    # self.load_tree(msg.mission_id, msg.root_task)
                    self.update_time()
                    self.missions[mission_sign] = [self.time + 10.0, -1]  # buffer for mission coordination
                    self.missionOrder.append(mission_sign)
                    self.missionStatus[mission_sign] = "waiting_for_schedule"
                    self.tasks[mission_sign] = {}
                    self.missionType[mission_sign] = msg.mission_type
                    self.enabledTasks[mission_sign] = []

        elif msg.type == "Abort":
            if mission_sign in self.missions.keys():
                self.missions.pop(mission_sign)
                self.missionOrder.remove(mission_sign)
                self.tasks.pop(mission_sign)
                self.missionType.pop(mission_sign)
                self.enabledTasks.pop(mission_sign)

                if self.missionStatus[mission_sign] == "executing":
                    # TODO What should happen when abort is received during execution?
                    pass
                self.missionStatus[mission_sign] = "no_mission"

        elif msg.type == "Completed":
            if msg.ag_addr == rospy.get_namespace():
                self.missions.pop(mission_sign)
                self.missionOrder.remove(mission_sign)
                self.tasks.pop(mission_sign)
                self.missionType.pop(mission_sign)
                self.enabledTasks.pop(mission_sign)

                if self.missionStatus[mission_sign] == "executing":
                    # TODO
                    pass
                self.missionStatus[mission_sign] = "no_mission"

    def update_task_structure_callback(self, msg):

        while msg.mission_id not in self.tasks.keys():
            pass

        for i in range(len(msg.task_label)):
            self.tasks[msg.mission_id][msg.task_label[i]] = [msg.start[i], msg.end[i]]

            self.tasks[msg.mission_id][msg.task_label[i]].append(msg.target[i])
            # TODO -- euroc eval -- remove
            self.tasks[msg.mission_id][msg.task_label[i]].append(msg.mass_class[i])
            self.tasks[msg.mission_id][msg.task_label[i]].append(msg.action_class[i])

    def mission_enabled_tasks_cb(self, msg):

        if msg.mission_id not in self.missions.keys():
            return

        logger.debug('Enabled tasks: %s', str(msg.task_labels))
        self.enabledTasks[msg.mission_id] = msg.task_labels

    def msg_mission_execution_callback(self, msg):

        if msg.data != rospy.get_namespace():
            return

        rospy.loginfo('***STARTING EXECUTION***\n')

        # if self.client is None:
        #    print "ERROR -- no action client"
        #
        # Waits until the action server has started up and started
        # listening for goals.
        # TODO -- uncomment this
        # self.client.wait_for_server()
        #
        # print ' Connected to server'

        mission = ""
        self.update_time()
        executedTask = ["", 0, 0]  # task label, time of task completion, scheduled time of completion

        while len(self.executionSchedule) > 0:
            self.update_time()
            item = self.executionSchedule[0]  # TODO: named tuple or something similar instead of list
            logger.debug('Current execution item: %s', item)
            # check for new mission
            if item[3] != mission:
                if mission != "" and mission != "_slack_" and item[3] != "_slack_":
                    rospy.loginfo('Mission %s has finished executing.', mission)
                mission = item[3]
                executedTask = ["", 0, 0]

            # mission specific for UAV robots!!!
            # TODO: ukloniti?
            if "UAV_go_to_position[" in item[0]:
                if self.waitUAVGoTo[0] is False:
                    i = 1
                    while "UAV_go_to_position[" in self.executionSchedule[i][0]:
                        self.waitUAVGoTo[1] = self.executionSchedule[i][0]
                        i += 1
                    if self.waitUAVGoTo[1] is not None:
                        self.waitUAVGoTo[0] = True
            else:
                self.waitUAVGoTo = [False, None]

            rospy.wait_for_service('register_executed_task')
            try:
                logger.debug('Calling 1st \"register_executed_task\" service with task: %s', executedTask)
                register_executed_task = rospy.ServiceProxy('register_executed_task', ExecuteTask)
                self.update_time()
                # Commented schedule delay -- something to be done later on (if needed)
                # if executedTask[1] - executedTask[2] > 0:
                #    self.waitScheduleDelay = True
                register_executed_task(mission, executedTask[0], executedTask[1] - executedTask[2])
                # while self.waitScheduleDelay:
                #    rospy.sleep(0.1)
            except rospy.ServiceException, e:
                rospy.logerr('Service call failed: %s', e)

            if item[0] != "slack":
                self.feedback = -1
                self.update_time()
                rospy.loginfo('[{:.3f}] Next task: {} in {} seconds.'.format(self.time, item[0], item[1] - self.time))
                while item[1] - self.time > 0 or item[0] not in self.enabledTasks[mission]:
                    # if the mission was canceled or is getting rescheduled -> TODO callback for mission control message Abort
                    # will other missions be rescheduled if the time is freed up ?? implement that behavior
                    if item[3] not in self.executableMission:
                        return
                    timeDiff = item[1] - self.time
                    if abs(timeDiff) - 0.1 > 1:
                        rospy.loginfo('[{:.3f}] Next task: {} in {} seconds.'.format(self.time, item[0], item[1] - self.time))
                        rospy.sleep(1)
                    self.update_time()

                rospy.loginfo('[{:.3f}] Starting execution... {} ({})'.format(self.time, item[0], item[3]))

                # check for moving target
                if len(self.tasks[mission][item[0]][2]) != 0:
                    target = self.tasks[mission][item[0]][2]
                    try:
                        rospy.wait_for_service(target + 'get_pose', 0.2)
                        get_pose = rospy.ServiceProxy(target + 'get_pose', GetPose)
                        result = get_pose()
                        self.tasks[mission][item[0]][1].position.x = result.pose.position.x
                        self.tasks[mission][item[0]][1].position.y = result.pose.position.y
                        self.tasks[mission][item[0]][1].orientation = result.pose.orientation
                        # TODO --> see what to do with z
                        rospy.loginfo('Moving target: {}; Position: {}'.format(target, result.pose))
                    except (rospy.ServiceException, rospy.ROSException) as e:
                        rospy.logerr('Service call failed: %s', e)

                # start task
                if self.waitUAVGoTo[0] is False:
                    err = self.init_task_execution(mission, item[0])
                elif item[0] == self.waitUAVGoTo[1]:
                    err = self.init_task_execution(mission, item[0])
                else:
                    rospy.loginfo('merging task %s', item[0])
                    self.executionSchedule[1][1] = item[1]
                    err = 0

                if err != 0:
                    # TODO abort mission execution
                    return
                self.update_time()
                executedTask = [item[0], self.time, item[2]]
            else:
                self.init_slack_execution(mission, item)
                self.update_time()
                executedTask = ["", 0, 0]
            # remove the completed task from schedule
            del self.executionSchedule[0]

            # DTC service
            if executedTask[0] != "":
                rospy.wait_for_service('execute_method')
                try:
                    execute_method = rospy.ServiceProxy('execute_method', ExecuteTask)
                    execute_method(mission, executedTask[0], 0)

                except rospy.ServiceException as e:
                    rospy.logerr('Service call failed: %s', e)

        # End of while loop. No more tasks to execute.
        rospy.loginfo('Mission %s has finished executing.', mission)

        self.executableMission.remove(mission)
        rospy.wait_for_service('register_executed_task')
        try:
            logger.debug('Calling 2nd \"register_executed_task\" service with task: %s', executedTask)
            register_executed_task = rospy.ServiceProxy('register_executed_task', ExecuteTask)
            self.update_time()
            # if executedTask[1] - executedTask[2] > 0:
            #    self.waitScheduleDelay = True
            register_executed_task(mission, executedTask[0], executedTask[1] - executedTask[2])
            # while self.waitScheduleDelay:
            #    pass
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s', e)

        # DTC service
        if executedTask[0] != "":
            rospy.wait_for_service('execute_method')
            try:
                execute_method = rospy.ServiceProxy('execute_method', ExecuteTask)
                execute_method(mission, executedTask[0], 0)

            except rospy.ServiceException as e:
                rospy.logerr('Service call failed: %s', e)

        if mission != "" and mission != "_slack_":
            mission_id = int(mission.split("[")[1][:-1])
            task = mission.split("[")[0]

            msg = MissionCtrlMsg()
            msg.type = "Completed"
            msg.ag_addr = rospy.get_namespace()
            msg.mission_id = mission_id
            msg.root_task = task

            logger.debug('Mission completed message: \n%s', msg)

            self.missionCtrlMsgPub.publish(msg)

            self.finish_mission()

    # ************************** #
    # SERVICE FUNCTIONS SECTION #
    # ************************** #

    def mission_earliest_start_srv(self, req):

        return self.missions[req.mission_id][0]

    def adjust_schedule_times_srv(self, req):

        if req.mission_id not in self.missionOrder:
            return [[], [], []]

        constrained = {}
        for i in range(len(req.constrained_tasks)):
            if req.constrained_tasks[i] not in constrained.keys():
                constrained[req.constrained_tasks[i]] = req.task_est[i]
            else:
                if req.task_est[i] > constrained[req.constrained_tasks[i]]:
                    constrained[req.constrained_tasks[i]] = req.task_est[i]

        self.taskEST[req.mission_id] = constrained
        self.bestSchedule[req.mission_id] = [req.task_labels, req.start_times, req.end_times]

        sched = self.adjust_schedule_times(req.mission_id, req.task_labels, req.start_times,
                                           req.end_times, constrained, False)
        [tasks, startTimes, endTimes] = self.extract_mission_schedule(sched, req.mission_id)

        return [tasks, startTimes, endTimes]

    def delay_schedule_srv(self, req):

        if len(self.executionSchedule) == 0:
            return []

        if req.time != 0:
            rospy.loginfo('Delaying schedule %s', str(req.time))
            if self.executionSchedule[0][0] == "slack":
                self.executionSchedule[0][2] += req.time
            else:
                self.executionSchedule[0][1] += req.time
                self.executionSchedule[0][2] += req.time

            for item in self.executionSchedule[1:]:
                item[1] += req.time
                item[2] += req.time

        self.waitScheduleDelay = False
        return []

    def register_schedule_ok_srv(self, req):

        sched = self.bestSchedule[req.mission_id]
        self.adjust_schedule_times(req.mission_id, sched[0], sched[1], sched[2], self.taskEST[req.mission_id], True)
        self.missions[req.mission_id][1] = sched[2][-1]  # mission end time

        triggerExecution = False
        logger.debug('Executable mission: %s', self.executableMission)
        if len(self.executableMission) == 0:
            triggerExecution = True

        self.executableMission.append(req.mission_id)
        logger.debug('Trigger execution: %s', triggerExecution)
        if triggerExecution:
            msg = String()
            msg.data = rospy.get_namespace()
            self.executionCtrlMsgPub.publish(msg)

        return True

