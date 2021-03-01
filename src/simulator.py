#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = "barbanas"

import copy
from collections import Counter

from utilities import helper_functions, my_logger
from ummc_agent_lib.taems.taems import TaemsTree, TaskGroup, Method
from ummc_agent_lib.taems.interrelationships import Interrelationship

logger = my_logger.CustomLogger('SIM')

class Simulator(object):
    '''
    Simulates the execution of a schedule.
    Applies the effects of method execution - interrelationships, resources...
    Calculates the quality, duration and cost probability distributions of the schedule.

    Attributes:
        taemsTree
        numberOfCompletedSubtasks - dictionary {task label : number of its completed subtasks}
        completedTasks - a set of completed tasks
        disabledMethods - a set of currently disabled methods
        taskHeuristicRating - dictionary {method label : rating (float number)}
    '''
    def __init__(self, tree):

        self.taemsTree = copy.deepcopy(tree)

        self.numberOfCompletedSubtasks = {}
        for task in self.taemsTree.tasks.values():
            if type(task) is TaskGroup:
                self.numberOfCompletedSubtasks[task.label] = 0
                for subtask in task.subtasks:
                    if subtask not in self.taemsTree.tasks.keys():
                        continue
                    #if self.taemsTree.tasks[subtask].subtasks is None:
                        #if self.taemsTree.tasks[subtask].nonLocal:
                        #    continue
                        #else:
                            #self.numberOfCompletedSubtasks[task.label] += 1
                    #else:
                    self.numberOfCompletedSubtasks[task.label] += 1

        #print self.numberOfCompletedSubtasks
        #raw_input("\n###\n")
        self.completedTasks = []
        self.taskOutcome = {}
        self.taskEndTime = {}

    def clone(self):
        s = Simulator(self.taemsTree)
        s.numberOfCompletedSubtasks = copy.deepcopy(self.numberOfCompletedSubtasks)
        s.completedTasks = copy.deepcopy(self.completedTasks)
        s.taskOutcome = copy.deepcopy(self.taskOutcome)
        s.taskEndTime = copy.deepcopy(self.taskEndTime)
        return s

    def execute_schedule(self, schedule, nonLocal, time):
        newSchedule = []
        methodList = [i[0] for i in schedule]

        for item in nonLocal:
            self.execute_non_local_task(item)

        for item in schedule:
            if item[0] == "slack":
                if len(newSchedule) == 0:
                    duration = item[2]
                else:
                    duration = item[2] - newSchedule[-1][2]
            else:
                duration = self.taemsTree.tasks[item[0]].DurationEV
                self.execute_task(item[0], methodList, time + duration, True)
            newSchedule.append([item[0], time, time + duration])
            time += duration

        self.evaluate_schedule()
        return newSchedule

    def calc_task_end_times(self, schedule):

        for item in schedule:
            if item[0] == "slack":
                continue
            else:
                self.execute_task(item[0], [], item[2], False)

    def execute_non_local_task(self, task):
        #print ".."
        #print task

        # if the task is Method
        if self.taemsTree.tasks[task].subtasks is None:
            if self.taemsTree.tasks[task].nonLocal:
                self.completedTasks.append(task)

        # if root task -> return
        if len(self.taemsTree.tasks[task].supertasks) == 0:
            return

        parentLabel = self.taemsTree.tasks[task].supertasks[0]
        self.numberOfCompletedSubtasks[parentLabel] -= 1
        #print self.numberOfCompletedSubtasks[parentLabel]
        if parentLabel in self.completedTasks:
            return

        qaf = self.taemsTree.tasks[parentLabel].qaf
        # if execution of only one subtask grants the execution of the subtask
        if qaf == 'q_max' or qaf == 'q_sum' or self.numberOfCompletedSubtasks[parentLabel] == 0:
            self.execute_non_local_task(parentLabel)
        #raw_input(",,")

    def execute_task(self, task, alternative, endTime, activateIR):
        '''
        Description.

        Args:
            mathodLabel - string
            time - float, task start time
        '''
        #print ".."
        #print task
        if task in self.completedTasks:
            return

        # activate interrelationships
        if activateIR:
            self.activate_IR(task, alternative, endTime)

        # update the list of completed tasks
        self.completedTasks.append(task)

        # method
        if self.taemsTree.tasks[task].subtasks is None:
            self.taskEndTime[task] = endTime

        # if root task -> return
        if len(self.taemsTree.tasks[task].supertasks) == 0:
            return

        parentLabel = self.taemsTree.tasks[task].supertasks[0]
        #print "parent   " + parentLabel
        self.numberOfCompletedSubtasks[parentLabel] -= 1
        #print self.numberOfCompletedSubtasks[parentLabel]
        if parentLabel in self.completedTasks:
            return

        qaf = self.taemsTree.tasks[parentLabel].qaf
        # if execution of only one subtask grants the execution of the subtask
        if  qaf == 'q_max' or qaf == 'q_sum' or self.numberOfCompletedSubtasks[parentLabel] == 0:
            if parentLabel in self.taskEndTime.keys():
                if qaf == 'q_max':
                    self.taskEndTime[parentLabel] = min(self.taskEndTime[parentLabel], self.taskEndTime[task]) #TODO adjust this part! -- if multiple children are executed -- end time = max(..)
                else:
                    self.taskEndTime[parentLabel] = max(self.taskEndTime[parentLabel], self.taskEndTime[task])
            else:
                self.taskEndTime[parentLabel] = self.taskEndTime[task]

            self.execute_task(parentLabel, alternative, endTime, activateIR)

    def activate_IR(self, task, alternative, taskEndTime):
        '''
        Activates the effects of interrelationship and modifies soft heuristic ratings.

        Args:
            task - string, a task label
            time - float number, start of ir.From task's execution
        '''
        insufficientResource = []
        for ir in self.taemsTree.IRs.values():
            if ir.From != task:
                continue

            # hard and soft IR (0 - enables, 1 - disables, 2 - facilitates, 3 - hinders)
            if ir.type < 4:
                # if ir.To is not my task -> continue
                if ir.To in alternative and ir.To not in self.completedTasks:
                    ir.activate(self.taemsTree, taskEndTime)

            # consumes
            if ir.type == 5:
                if ir.To in self.taemsTree.resources.keys():
                    ir.activate(self.taemsTree)
                    self.taemsTree.resources[ir.To].checkSufficiency()
                    if self.taemsTree.resources[ir.To].isSufficient == False:
                        insufficientResource.append(ir.To)

        if len(insufficientResource) > 0:
            for resource in insufficientResource:
                irList = []
                for IR in self.taemsTree.IRs.values():
                    if resource == IR.From and ir.To in self.taemsTree.tasks.keys():
                        irList.append(IR)

                if len(irList) == 0:
                    continue

                for ir in irList:
                    ir.activate(self.taemsTree, 0)

    def evaluate_schedule(self):
        '''
        Evaluates a schedule. Calculates quality, duration and cost probability distributions and
        quality, duration and cost expected values.

        Returns:
            A list [quality distribution, duration distr., cost distr., quality EV, duration EV, cost EV]
        '''
        result = self.calc_QDC(self.taemsTree.rootTask[0])
        # WUT? sto je s Q, D, C?
        if result is not None:
            [Q, D, C] = result
        else:
            logger.error('calc_QDC in Simulator returned None -> Root task not in completed tasks!')

    def calc_QDC(self, task):
        '''
        Recursive function. Calculates task's quality, duration and cost probability distributions using
        subtask's outcomes and quality accumulation function.

        Args:
            task - string, task's label

        Returns:
            list - [quality distribution, duration distr., cost distr.]
        '''

        if task not in self.completedTasks:
            return None

        if type(self.taemsTree.tasks[task]) is Method:
            # self.taemsTree.tasks[task].calcExpectations()
            q = self.taemsTree.tasks[task].outcome[0]
            d = self.taemsTree.tasks[task].outcome[1]
            c = self.taemsTree.tasks[task].outcome[2]
            self.taskOutcome[task] = [q, d, c]
            return [q, d, c]

        qaf = self.taemsTree.tasks[task].qaf

        qualityDistributions = []
        durationDistributions = []
        costDistributions = []

        for child in self.taemsTree.tasks[task].subtasks:
            if child in self.taskOutcome.keys():
                childQDC = self.taskOutcome[child]
            else:
                childQDC = self.calc_QDC(child)

            if childQDC is None:
                continue

            qualityDistributions.append(childQDC[0])
            durationDistributions.append(childQDC[1])
            costDistributions.append(childQDC[2])

        fun = ""
        if qaf == "q_min":
            fun = "min"
        elif qaf == "q_max":
            fun = "max"
        elif qaf == 'q_sum_all' or qaf == 'q_sum' or qaf == 'q_seq_sum_all':
            fun = "sum"

        Q = helper_functions.cartesianProductOfDistributions(qualityDistributions, fun)
        C = helper_functions.cartesianProductOfDistributions(costDistributions, "sum")
        D = helper_functions.cartesianProductOfDistributions(durationDistributions, "sum")

        self.taskOutcome[task] = [Q, D, C]
        return [Q, D, C]


class LightSimulator(object):
    """
    Simulate the execution of a schedule while ignoring duration of the tasks.

    LightSimulator is needed during scheduling because it simulates the precedence constraints between tasks. It
    applies the effects of method execution, but ONLY HARD IRs. Effects of soft IRs are simulated in Simulator.

    Attributes:
        taemsTree (TaemsTree): Taems tree used for simulation.
        numberOfCompletedSubtasks (dict[str, int]): {task label: number of its completed subtasks}.
        disablements (Counter): {task label: number of times it has been disabled by other tasks}
        completedTasks (list): List of completed tasks.
        disabledMethods (set): Set of currently disabled methods.
        hardConstrainedTasks (set): Set of tasks that are a source or a target of hard task interrelationship.
    """

    def __init__(self, tree):
        """Initialize instance variables by calling self.setup method."""
        self.taemsTree = tree  # WUT? je li bi se mogla staviti kopija stabla ili je dizajnirano bas ovako?
        self.setup()

    def setup(self):
        """Reset or set up the simulator for new simulation."""
        self.numberOfCompletedSubtasks = {}
        for task in self.taemsTree.tasks.values():
            if type(task) is TaskGroup:
                self.numberOfCompletedSubtasks[task.label] = 0
                for subtask in task.subtasks:
                    if subtask not in self.taemsTree.tasks.keys():
                        continue
                    # TODO: remove
                    # if self.taemsTree.tasks[subtask].subtasks is None:
                    #     if self.taemsTree.tasks[subtask].nonLocal:
                    #         continue
                    #     else:
                    #         self.numberOfCompletedSubtasks[task.label] += 1
                    # else:
                    self.numberOfCompletedSubtasks[task.label] += 1

        self.disablements = Counter()
        self.completedTasks = []
        self.disabledMethods = set()
        self.hardConstrainedTasks = set()

        # Reset all resources for new simulation run
        # logger.debug('Setup')
        for resource in self.taemsTree.resources.values():
            resource.state = copy.deepcopy(resource.defaultState)
            resource.checkSufficiency()
            # logger.debug(repr(resource))

    def init_disablements(self, alternative, tasksToComplete):
        """
        Disable methods in IREnables.To and IRDisables.From to enforce hard precedence constraints.

        Args:
            alternative (list): Unordered list of methods.
            tasksToComplete: ???
        """
        # logger.debug('Init disablements')
        # logger.debug('alternative:\n%s', alternative)
        # logger.debug('tasksToComplete:\n%s', tasksToComplete)
        # TODO: u tasksToComplete su samo grupe zadataka ako se izvrsavaju na drugim agentima, ne i njihove metode.
        # To znaci da ako neka nelokalna metoda ima IR s nekom lokalnom metodom, to se nece pravilno inicijalizirati.
        # Prije resursa se to nije manifestiralo kao problem jer se u koordinatoru pazilo da se ne enableaju metode
        # koje ne smiju. To se radi na kraju register_executed_task_srv gdje se u disabled metode dodaju one koje
        # su ogranicene nekom obavezom od drugog agenta. Executoru se onda salje lista omogucenih metoda bez tih koje
        # su onemogucene obavezom.

        # Problem se dogada u vrlo specificnom slucaju: npr. UGV postavlja ciglu 1.1, a UAV cigle 1.2 i 2.1.
        # Dogada se to da su u simulatoru prve metode svakog agenta disableane 0 puta. Kada se registrira pick cigle
        # 1.1, go_to_pile 1.2 i 2.1 su disableane -1 puta. Tada krece izvodenje go_to_pile 1.2 i zauzima resurs pa disablea
        # sljedece metode, tj. vraca na go_to_pile 1.2 i 2.1 na 0. Kada se izvrsi place 1.2, go_to_pile 1.2 i 2.1 se opet
        # enableaju na -1, ali se to ne registrira dobro jer se gleda 0 i izvodenje se zaustavlja.

        # Trenutni fix je ograniciti broj disablea da bude 0 ili vise. Razmisliti o tome da se ovdje pravilno inicijaliziraju
        # vrijednosti disablea sto bi popravilo ovakve slucajeve.
        for ir in self.taemsTree.IRs.values():
            if ir.To in tasksToComplete and ir.From in tasksToComplete:
                # logger.debug(repr(ir))
                if ir.type == Interrelationship.IR_types['enables']:
                    # TODO: remove
                    # if self.taemsTree.tasks[ir.From].subtasks is None:
                    #     if self.taemsTree.tasks[ir.From].nonLocal:
                    #         continue
                    self.disable_task(ir.To)
                    self.add_hard_constrained_methods(ir.To, alternative)
                    self.add_hard_constrained_methods(ir.From, alternative)

                elif ir.type == Interrelationship.IR_types['disables']:
                    # TODO: remove
                    # if self.taemsTree.tasks[ir.To].subtasks is None:
                    #     if self.taemsTree.tasks[ir.To].nonLocal:
                    #         continue
                    self.disable_task(ir.From)
                    self.add_hard_constrained_methods(ir.To, alternative)
                    self.add_hard_constrained_methods(ir.From, alternative)

            if ir.type == Interrelationship.IR_types['limits']:
                if ir.From in self.taemsTree.resources and ir.To in tasksToComplete:
                    # logger.debug(repr(ir))
                    self.taemsTree.resources[ir.From].checkSufficiency()
                    if not self.taemsTree.resources[ir.From].isSufficient:
                        # logger.debug('Insufficient: %s', repr(ir))
                        ir.activate(self.taemsTree, 0)
                        # If activation of this IR has a 100% chance to reduce the quality of connected task by 100%
                        # (which means that new quality will be 0), add the task to disabled tasks. If 1.0 is not in
                        # quality_power dictionary or has a lower probability, do nothing.
                        if ir.quality_power.get(1.0, 0) == 1.0:
                            self.disable_task(ir.To)
        #logger.debug('\n'.join(['{}: {}'.format(x[0], x[1]) for x in self.disablements.items()]))

    def add_hard_constrained_methods(self, task, alternative):
        """
        Update a set of hard constrained methods.

        If task is a Method, add it to the set. If it's a TaskGroup, add all downstream methods to the set.

        Args:
            task (str): Task label.
            alternative: ???
        """

        if task not in self.taemsTree.tasks:
            return

        # If task is a Method.
        if self.taemsTree.tasks[task].subtasks is None:
            # TODO: remove
            # if task in alternative:
            #     self.hardConstrainedTasks.add(task)
            self.hardConstrainedTasks.add(task)
            return

        # If task is a TaskGroup.
        for subtask in self.taemsTree.tasks[task].subtasks:
            self.add_hard_constrained_methods(subtask, alternative)

    def disable_task(self, task):
        """
        Disable a task and all of its subtasks (in simulation).

        Args:
            task (str): Task label.
        """
        if task not in self.taemsTree.tasks:
            return

        logger.debugv('Disabled: %s', task)
        # If the task is a Method.
        if self.taemsTree.tasks[task].subtasks is None:
            # TODO: remove
            # if self.taemsTree.tasks[task].nonLocal:
            #     return
            self.disablements[task] += 1
            logger.debugv('Task is disabled %s times', self.disablements[task])
            self.disabledMethods.add(task)
        else:
            for subtask in self.taemsTree.tasks[task].subtasks:
                self.disable_task(subtask)
        logger.debugv('Set: %s\n', self.disabledMethods)

    def enable_task(self, task):
        """
        Enable a task and all of its subtasks (in simulation).

        Args:
            task (str): Task label.
        """
        if task not in self.taemsTree.tasks:
            return

        logger.debugv('Enabled: %s', task)
        # If the task is a Method.
        if self.taemsTree.tasks[task].subtasks is None:
            # TODO: remove
            # if self.taemsTree.tasks[task].nonLocal:
            #     return
            # TODO: vidi TODO u init_disablements
            # Poredak je promjenjen i usporeduje se s 1 da ne bi doslo do uklanjanja iz seta u situaciji
            # 0 -> -1 -> 0 kada tog elementa nema u setu.
            if self.disablements[task] == 1:
                self.disabledMethods.remove(task)
            self.disablements[task] = max(self.disablements[task] - 1, 0)
            logger.debugv('Task is disabled %s times', self.disablements[task])
        else:
            for subtask in self.taemsTree.tasks[task].subtasks:
                self.enable_task(subtask)
        logger.debugv('Set: %s\n', self.disabledMethods)

    def get_enabled_tasks(self, alternative):
        """
        Return a set of enabled tasks in list of methods.

        Args:
            alternative (list): List of all possible methods.
        """
        logger.debugv('Disabled methods %s', self.disabledMethods)
        return set(alternative) - self.disabledMethods

    def execute_alternative(self, alternative, tasksToComplete):
        """
        Simulate the execution of all methods in alternative.

        Args:
            alternative (Union[set,list]): Methods in simulated execution.
            tasksToComplete: ???
        """
        for task in alternative:
            self.execute_task(task, tasksToComplete)

    def execute_task(self, task, tasksToComplete):
        """
        Simulate the effects of method execution (only hard precedence constraints and completed task list).

        Args:
            task (str): Label of the task to execute.
            tasksToComplete: ???
        """
        if task in self.completedTasks:
            return

        # Activate hard IRs.
        self.activate_IR(task, tasksToComplete)

        # Update the list of completed tasks.
        self.completedTasks.append(task)

        # If task is non local.
        if task not in self.taemsTree.tasks:
            logger.debugv('!Task %s is non local.', task)
            return

        # If task is the root task.
        if len(self.taemsTree.tasks[task].supertasks) == 0:
            return

        parentLabel = self.taemsTree.tasks[task].supertasks[0]

        self.numberOfCompletedSubtasks[parentLabel] -= 1

        if parentLabel in self.completedTasks:
            return

        qaf = self.taemsTree.tasks[parentLabel].qaf
        # If execution of only one subtask grants the execution of the subtask.
        if qaf == 'q_max' or qaf == 'q_sum' or self.numberOfCompletedSubtasks[parentLabel] == 0:
            self.execute_task(parentLabel, tasksToComplete)
            # TODO: remove
            # if len(self.taemsTree.tasks[parentLabel].supertasks) > 0:
            #     parentLabel = self.taemsTree.tasks[parentLabel].supertasks[0]
            #     taskCompleted = True

    def activate_IR(self, task, tasksToComplete):
        """
        A task that creates the effects of hard precedence constraints - enables and disables methods.

        Args:
            task - a label of task that is being executed
        """
        insufficientResources = []
        nowSufficientResources = []
        # logger.debug('Activate IR for task: %s', task)
        for ir in self.taemsTree.IRs.values():
            if ir.From == task:
                # logger.debug(str(ir))
                if ir.To in tasksToComplete and ir.To not in self.completedTasks:
                    if ir.type == Interrelationship.IR_types['enables']:
                        self.enable_task(ir.To)
                    elif ir.type == Interrelationship.IR_types['disables']:
                        self.disable_task(ir.To)

                # APPLY RESOURCE RELATED IRs ONLY TASK IS LOCAL
                # TODO: fix this scenario
                if task in self.taemsTree.tasks:
                    if ir.type == Interrelationship.IR_types['consumes']:
                        if ir.To in self.taemsTree.resources:
                            ir.activate(self.taemsTree)
                            logger.debugv(repr(ir))
                            self.taemsTree.resources[ir.To].checkSufficiency()
                            #logger.debug(repr(self.taemsTree.resources[ir.To]))
                            if not self.taemsTree.resources[ir.To].isSufficient:
                                insufficientResources.append(ir.To)

                    elif ir.type == Interrelationship.IR_types['produces']:
                        if ir.To in self.taemsTree.resources:
                            ir.activate(self.taemsTree)
                            logger.debugv(repr(ir))
                            self.taemsTree.resources[ir.To].checkSufficiency()
                            #logger.debug(repr(self.taemsTree.resources[ir.To]))
                            if self.taemsTree.resources[ir.To].isSufficient:
                                nowSufficientResources.append(ir.To)

            elif ir.To == task:
                if ir.type == Interrelationship.IR_types['disables']:
                    # The From task in IRDisables was initially disabled, so it has to be enabled.
                    self.enable_task(ir.From)

        # Go through the list of all insufficient resources.
        # TESTME: this is possibly unnecessary because resources activate all limits when they become insufficient.
        for resource in insufficientResources:
            irList = []
            # Find all IRLimits that have the current insufficient resource in their From field
            # and one of the tasksToComplete in their To field.
            for IR in self.taemsTree.IRs.values():
                if resource == IR.From and IR.To in tasksToComplete:
                    irList.append(IR)

            # Activate all found IRLimits.
            for ir in irList:
                ir.activate(self.taemsTree, time=0)
                logger.debugv(repr(ir))
                # If activation of this IR has a 100% chance to reduce the quality of connected task by 100%
                # (which means that new quality will be 0), add the task to disabled tasks. If 1.0 is not in
                # quality_power dictionary or has a lower probability, do nothing.
                if ir.quality_power.get(1.0, 0) == 1.0:
                    self.disable_task(ir.To)

        # Go through the list of all newly sufficient resources.
        # TESTME: this is possibly unnecessary because resources deactivate all limits when they become sufficient.
        for resource in nowSufficientResources:
            irList = []
            # Find all IRLimits that have the current newly sufficient resource in their From field
            # and one of the tasksToComplete in their To field.
            for IR in self.taemsTree.IRs.values():
                if resource == IR.From and IR.To in tasksToComplete:
                    irList.append(IR)

            # Deactivate all found IRLimits.
            # logger.debug('Found IR limits: %s', irList)
            for ir in irList:
                ir.deactivate(self.taemsTree)
                logger.debugv(repr(ir))
                # If activation of this IR has a 100% chance to reduce the quality of connected task by 100%
                # (which means that new quality will be 0), disabling it means that the task can be safely reanabled.
                # If 1.0 is not in quality_power dictionary or has a lower probability, do nothing.
                if ir.quality_power.get(1.0, 0) == 1.0:
                    self.enable_task(ir.To)
                    # logger.debug(self.disablements)
