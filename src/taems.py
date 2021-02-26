#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = "barbanas"
import re
from sys import maxint
from copy import deepcopy
from collections import OrderedDict

from utilities import helper_functions


class TaemsTree(object):
    """
    Class that represents the model of taems tree structure.

    Holds information about all elements that define a task structure.

    Attributes:
        agent (str): Label of agent which owns the structure.
        agentLabels (list[str]): Labels of all agents included in the execution of methods in taems tree.
        tasks (dict[Task]): Tasks in the tree in form of: {task/method label: TaskGroup/Method object}
        rootTask (list[str]): A list with one! element, label of root task.
        methodLabels (list[str]): Labels of atomic methods.
        resources (dict): {Resource label: Resource object}.
        IRs (dict): Interrelationships in the tree in form of {IR label: Interrelationship object}.
        activeIR (list): Active interrelationships' labels.
    """

    def __init__(self):
        self.agent = ""
        self.agentLabels = []
        self.tasks = OrderedDict()
        self.rootTask = []
        self.methodLabels = []
        self.resources = OrderedDict()
        self.IRs = OrderedDict()
        self.mutuallyExclusiveTasks = []  # WUT? Ne koristi se
        self.homogeneousTasks = []  # WUT? Ne koristi se
        # self.IRsKeyMethodFrom = {}
        # self.activeIR = []
        # self.softIR = []
        # self.hardIR = []

    @classmethod
    def init_with_values(cls, agentLabels, root, tasks, IRs, resources):
        """
        Initialize TaemsTree object by manually specifying its values.

        Args:
            agentLabels (list[str]): Labels of all agents included in the execution of methods in taems tree.
            root (str): Label of root task.
            tasks (dict[str, Any]): Tasks in the tree in form of: {task/method label: TaskGroup/Method object}
            IRs (dict[str, Any]): Interrelationships in the tree in form of {IR label: Interrelationship object}.
            resources (dict[str, Any]): Resources in the tree in the form of: {resource label: Resource object}.

        Returns:
            TaemsTree object.
        """
        tree = cls()
        tree.agentLabels = agentLabels
        tree.tasks = tasks
        tree.rootTask = [root]
        tree.methodLabels = [key for key, value in tasks.items() if type(value).__name__ == 'Method']
        tree.IRs = IRs
        tree.resources = resources
        return tree

    @staticmethod
    def load_from_file(filename):
        """
        Load taems tree from file.

        Args:
            filename (str): Path to file.

        Returns:
            Loaded taems tree as a TaemsTree object.
        """
        with open(filename, 'r') as in_file:
            type_node = None
            root = None
            values = {}
            nodes = OrderedDict()
            IRs = OrderedDict()
            resources = OrderedDict()
            agent_labels = set()
            excludes = []

            multi_line_name = []
            multi_line_values = {}

            for line in in_file:
                line = line.strip()

                # Line is empty or comment.
                if not line or line.startswith('%'):
                    continue

                if type_node is None:
                    type_node = line[6:].strip()

                if type_node in ['method', 'task_group', 'enables', 'disables', 'facilitates', 'hinders', 'produces',
                                 'consumes', 'limits', 'excludes', 'consumable_resource', 'non_consumable_resource']:

                    if line.startswith('(') and line.endswith(')'):
                        parts = line[1:-1].split(' ', 1)

                        if len(parts) != 2:
                            raise ValueError("Illegal input format for single line")

                        if len(multi_line_name) > 0:
                            if multi_line_name[-1] not in multi_line_values.keys():
                                multi_line_values[multi_line_name[-1]] = {}
                            multi_line_values[multi_line_name[-1]][parts[0]] = parts[1]
                        else:
                            values[parts[0]] = parts[1]
                    elif line.startswith('('):
                        multi_line_name.append(line[1:])
                    elif line.endswith(')'):

                        if len(multi_line_name) > 0:
                            values[multi_line_name[-1]] = multi_line_values[multi_line_name[-1]]

                            multi_line_values.pop(multi_line_name[-1])
                            del multi_line_name[-1]

                        if len(multi_line_name) == 0:

                            if type_node in ['method', 'task_group']:
                                node_info = values.get('spec_' + type_node)

                                supertasks = node_info.get('supertasks')
                                supertasks = [supertasks] if supertasks is not None else None
                                subtasks = node_info.get('subtasks', '').split(', ')
                                label = node_info.get('label')
                                task_type = node_info.get('type', 'homogeneous')
                                agent = node_info.get('agent').replace(' ', '').split(',')
                                qaf = node_info.get('qaf')
                                outcome = values.get('outcome')
                                # Outcome is loaded as dict of dicts with {'distribution': list(value, probability...)}
                                # Convert outcome to be list of dicts with {value: probability}
                                if outcome is not None:
                                    outcome_list = []
                                    for distribution in [x + '_distribution' for x in ['quality', 'duration', 'cost']]:
                                        outcome_iter = iter(outcome[distribution].split(' '))
                                        distribution_dict = {}
                                        for elem in outcome_iter:
                                            key = float(elem)
                                            value = float(next(outcome_iter))
                                            distribution_dict[key] = value
                                        outcome_list.append(distribution_dict)
                                qaf_local = node_info.get('qaf_local', '')

                                # Pack all common arguments in a dictionary.
                                kwargs = {'label': label,
                                          'agents': agent,
                                          'supertasks': supertasks,
                                          'type': task_type}

                                # Add a Method or a TaskGroup to the dictionary of all nodes.
                                if type_node == 'method':
                                    kwargs.update({'outcome': outcome_list})
                                    nodes[label] = Method.init_with_values(**kwargs)
                                elif type_node == 'task_group':
                                    kwargs.update({'subtasks': subtasks, 'qaf': qaf, 'qaf_local': qaf_local})
                                    nodes[label] = TaskGroup.init_with_values(**kwargs)

                                # Update the set of agent labels in the tree.
                                agent_labels |= set(agent)

                                # If a task doesn't have supertasks, it is the root task.
                                if supertasks is None:
                                    root = nodes[label]

                            elif type_node in Interrelationship.IR_types.keys():
                                node_info = values.get('spec_' + type_node)
                                label = node_info.get('label')

                                # Pack all arguments in a dictionary.
                                kwargs = {'label': label,
                                          'agents': node_info.get('agent').replace(' ', '').split(','),
                                          'From': node_info.get('from'),
                                          'To': node_info.get('to'),
                                          'delay': node_info.get('delay')}

                                # Add specific IR to the dictionary of all IRs.
                                if type_node == 'enables':
                                    IRs[label] = IREnables.init_with_values(**kwargs)
                                elif type_node == 'disables':
                                    IRs[label] = IRDisables.init_with_values(**kwargs)
                                elif type_node == 'limits':
                                    # powers are loaded as 'key value key value key value...'.
                                    # IR must be initialized with a dict with those keys and values.
                                    read_list = node_info.get('quality_power').split(' ')
                                    quality_power = dict(zip(read_list[0::2], read_list[1::2]))
                                    read_list = node_info.get('duration_power').split(' ')
                                    duration_power = dict(zip(read_list[0::2], read_list[1::2]))
                                    read_list = node_info.get('cost_power').split(' ')
                                    cost_power = dict(zip(read_list[0::2], read_list[1::2]))
                                    power = {'quality': quality_power, 'duration': duration_power, 'cost': cost_power}
                                    kwargs.update({'power': power, 'model': node_info.get('model', 'time_independent')})
                                    IRs[label] = IRLimits.init_with_values(**kwargs)
                                elif type_node == 'produces':
                                    # produces is loaded as 'key value key value key value...'.
                                    # IR must be initialized with a dict with those keys and values.
                                    produces_list = node_info.get('produces').split(' ')
                                    produces_dict = dict(zip(produces_list[0::2], produces_list[1::2]))
                                    kwargs.update({'produces': produces_dict,
                                                   'model': node_info.get('model', 'time_independent')})
                                    IRs[label] = IRProduces.init_with_values(**kwargs)
                                elif type_node == 'consumes':
                                    # consumes is loaded as 'key value key value key value...'.
                                    # IR must be initialized with a dict with those keys and values.
                                    consumes_list = node_info.get('consumes').split(' ')
                                    consumes_dict = dict(zip(consumes_list[0::2], consumes_list[1::2]))
                                    kwargs.update({'consumes': consumes_dict,
                                                   'model': node_info.get('model', 'time_independent')})
                                    IRs[label] = IRConsumes.init_with_values(**kwargs)

                            elif type_node in ['consumable_resource', 'non_consumable_resource']:
                                node_info = values.get('spec_' + type_node)
                                label = node_info.get('label')
                                agent = node_info.get('agent')
                                if agent is not None:
                                    agent = agent.replace(' ', '').split(',')

                                # Pack all arguments in a dictionary.
                                kwargs = {'label': label,
                                          'agents': agent,
                                          'state': node_info.get('state'),
                                          'depleted_at': node_info.get('depleted_at'),
                                          'overloaded_at': node_info.get('overloaded_at')}

                                if type_node == 'consumable_resource':
                                    resources[label] = ConsumableResource.init_from_values(**kwargs)
                                elif type_node == 'non_consumable_resource':
                                    resources[label] = NonConsumableResource.init_from_values(**kwargs)

                            type_node = None
                            values = {}

            return TaemsTree.init_with_values(list(agent_labels), root.label, nodes, IRs, resources)

    def copy(self):
        """
        Return a copy of the tree.
        """
        new_tree = TaemsTree.init_with_values(deepcopy(self.agentLabels),
                                              deepcopy(self.rootTask[0]),
                                              deepcopy(self.tasks),
                                              deepcopy(self.IRs),
                                              deepcopy(self.resources))
        return new_tree

    def instantiate(self, taems_id):
        """
        Replace template placeholders in tree with actual mission IDs.

        Args:
            taems_id (int): Mission ID.
        """
        pattern = r'(\[[^\]]*)X([^[]*\])'
        replacement = r'\g<1>%d\g<2>' % taems_id

        # Replace placeholders in tasks
        new_tasks = OrderedDict()
        for key, value in self.tasks.items():
            value.label = re.sub(pattern, replacement, value.label)
            if value.subtasks is not None:
                value.subtasks = [re.sub(pattern, replacement, x) for x in value.subtasks]
            value.supertasks = [re.sub(pattern, replacement, x) for x in value.supertasks]
            new_key = re.sub(pattern, replacement, key)
            new_tasks[new_key] = value
        self.tasks = new_tasks

        # Replace placeholder in root task
        self.rootTask = [re.sub(pattern, replacement, self.rootTask[0])]

        # Replace placeholders in method labels
        self.methodLabels = [re.sub(pattern, replacement, x) for x in self.methodLabels]

        # Replace placeholders in resources
        # Not yet implemented

        # Replace placeholders in IRs
        new_IRs = OrderedDict()
        for key, value in self.IRs.items():
            value.label = re.sub(pattern, replacement, value.label)
            value.From = re.sub(pattern, replacement, value.From)
            value.To = re.sub(pattern, replacement, value.To)
            new_key = re.sub(pattern, replacement, key)
            new_IRs[new_key] = value
        self.IRs = new_IRs

    def dump_to_file(self, filename, mode='w'):
        """
        Dump taems tree structure to the file with given filename.

        Args:
            filename (str): Name of the file.
            mode (str): 'w' for overwriting, 'a' for appending.
        """
        with open(filename, mode) as dump_file:
            if len(self.rootTask) == 0:
                dump_file.write('% This is an empty taems file\n')
            else:
                # First, write root task.
                dump_file.write(str(self.tasks[self.rootTask[0]]))
                dump_file.write('\n\n')
                # Then, write all other tasks.
                for label, task in self.tasks.items():
                    if label != self.rootTask[0]:
                        dump_file.write(str(task))
                        dump_file.write('\n\n')
                # Next, write IRs.
                for IR in self.IRs.values():
                    dump_file.write(str(IR))
                    dump_file.write('\n\n')
                # Finally, write resources
                for res in self.resources.values():
                    dump_file.write(str(res))
                    dump_file.write('\n\n')

    def __str__(self):
        if len(self.rootTask) == 0:
            return '% This is an empty taems file\n'
        else:
            str_buffer = []
            # First, write root task.
            str_buffer.append(str(self.tasks[self.rootTask[0]]))
            # Then, write all other tasks.
            for label, task in self.tasks.items():
                if label != self.rootTask[0]:
                    str_buffer.append(str(task))
            # Next, write IRs.
            for IR in self.IRs.values():
                str_buffer.append(str(IR))
            # Finally, write resources
            for res in self.resources.values():
                str_buffer.append(str(res))

            return '\n\n'.join(str_buffer)

    def dump_to_dot(self, filename, graph_name='', **kwargs):
        """
        Dump taems tree structure to .dot file for easy visualization.

        Args:
            filename (str): Name of the file.
            graph_name (str): Name of the graph.
            **kwargs: Various possible options for formatting the output.
        """

        with open(filename, 'w') as dot_file:
            dot_file.write('digraph {} {{\n'.format(graph_name))

            for task in self.tasks.values():
                dot_file.write(task.to_dot(**kwargs))

            for IR in self.IRs.values():
                dot_file.write(IR.to_dot(**kwargs))

            dot_file.write('}\n')

    def get_all_subtasks(self, task):
        """
        Return all subtasks of given task.

        Args:
            task (str): Label of the task.

        Returns:
            List of all subtask labels.
        """

        if task not in self.tasks.keys():
            return []

        if type(self.tasks[task]) is Method:
            return []

        allSubtasks = []
        for subtask in self.tasks[task].subtasks:
            if subtask in self.tasks.keys():
                allSubtasks.append(subtask)
            allSubtasks.extend(self.get_all_subtasks(subtask))

        return allSubtasks

    def removeTaskAndSubtasks(self, task):
        """
        Remove given task and its subtasks from the tree structure.

        Args:
            task (str): Label of the task.
        """
        toRemove = self.get_all_subtasks(task)

        for item in toRemove:
            if task in self.tasks.keys():
                self.tasks.pop(item)

        if task in self.tasks.keys():
            self.tasks.pop(task)

    def filter_agent(self, agent_labels):
        """
        Remove all nodes that don't have agent type specified in agent_labels.

        If a node has multiple agents specified, remove references to all agents
        not specified in agent_labels. If a node doesn't have agents specified
        in agent_labels in its structure, remove it completely. Also, update
        tree's methodLabels list.

        Args:
            agent_labels (list[str]): Agent labels to leave in the tree.
        """
        root_task = self.tasks[self.rootTask[0]]

        # If root task doesn't have at least one of the specified agents,
        # result is an empty tree.
        intersection = set(agent_labels) & set(root_task.agent)
        if len(intersection) == 0:
            self.rootTask = None
        else:
            # Find tasks that need to be removed.
            to_remove = []
            for task in self.tasks.values():
                intersection = set(agent_labels) & set(task.agent)
                if len(intersection) == 0:
                    to_remove.append(task.label)
                else:
                    task.agent = list(intersection)
            # Remove tasks and methods.
            for key in to_remove:
                del self.tasks[key]
                if key in self.methodLabels:
                    self.methodLabels.remove(key)
            # Remove IRs
            to_remove = [IR.label for IR in self.IRs.values() if len(set(agent_labels) & set(IR.agent)) == 0]
            for IR_label in to_remove:
                del self.IRs[IR_label]

    @staticmethod
    def merge(first, second):
        """
        Merge two taems trees.

        Args:
            first (TaemsTree): First tree.
            second (TaemsTree): Second tree.

        Returns:
            Merged tree.
        """
        result = TaemsTree()

        result.agentLabels = deepcopy(first.agentLabels)
        result.agentLabels.extend(second.agentLabels)

        result.methodLabels = deepcopy(first.methodLabels)
        result.methodLabels.extend(second.methodLabels)

        result.rootTask = deepcopy(first.rootTask)

        result.resources = deepcopy(first.resources)
        result.resources.update(second.resources)

        result.IRs = deepcopy(first.IRs)
        result.IRs.update(second.IRs)

        result.tasks = deepcopy(first.tasks)
        for task in second.tasks.keys():
            if task in result.tasks.keys():
                if type(result.tasks[task]) is not Method:
                    result.tasks[task].subtasks = list(
                        set(result.tasks[task].subtasks) | set(second.tasks[task].subtasks))
                if result.tasks[task].supertasks is not None:
                    result.tasks[task].supertasks = list(
                        set(result.tasks[task].supertasks) | set(second.tasks[task].supertasks))
                result.tasks[task].earliestStartTime = max(
                    [result.tasks[task].earliestStartTime, second.tasks[task].earliestStartTime])
                result.tasks[task].deadline = min([result.tasks[task].deadline, second.tasks[task].deadline])
            else:
                result.tasks[task] = second.tasks[task]

        return result


class Agent(object):
    """
    Class that models an agent which can execute methods in taems tree.

    Attributes:
        label (str): Agent's label, unique name of the agent.
    """

    def __init__(self):
        self.label = ""


class Node(object):
    """
    Base class for an element of taems tree.

    Classes that inherit this class are: Task and Resource.

    Attributes:
        label (str): Element's label, has to be unique among the elements of the same class.
        agent (List[str]): Labels (types) of the agents who 'own' the element (are responsible for it).
    """

    def __init__(self):
        self.label = ''
        self.agent = []


class Task(Node):
    """
    Class that represents a task (taskgroup and method) in taems tree structure.

    This class inherits attributes from Node class.
    Classes that inherit this class are: TaskGroup and Method.

    Attributes:
        subtasks (list[str]): Labels of Task's subtasks - empty for Method.
        supertasks (list[str]): Labels of Task's supertasks - optional for TaskGroup / empty if root.
        earliestStartTime (float): Earliest execution start time.
        deadline (float): Deadline for finishing execution.
        qaf (str): Quality function identifier.
        qaf_local (str): Local quality function identifier.
        type (str): 'homogeneous'
    """
    # WUT? Koliko vidim, Task se bas i ne koristi, svugdje je TaskGroup
    def __init__(self):
        super(Task, self).__init__()
        self.subtasks = []  # doesn't exist for method
        self.supertasks = []  # optional for TaskGroup (if root task)
        self.earliestStartTime = None
        self.deadline = None
        self.qaf = ''
        self.qaf_local = ''
        self.type = 'homogeneous'

    def __str__(self):
        str_buffer = []
        str_buffer.append('(spec_task')
        str_buffer.append('\t(label {})'.format(self.label))
        str_buffer.append('\t(agent {})'.format(', '.join(self.agent)))
        str_buffer.append('\t(supertasks {})'.format(', '.join(self.supertasks)))
        str_buffer.append('\t(subtasks {})'.format(', '.join(self.subtasks)))
        str_buffer.append('\t(qaf {})'.format(self.qaf))
        if self.qaf_local:
            str_buffer.append('\t(qaf_local {})'.format(self.qaf_local))
        str_buffer.append(('\t(type {})'.format(self.type)))
        str_buffer.append(')')
        return '\n'.join(str_buffer)


class TaskGroup(Task):
    """
    Class that represents a task group in taems tree structure.

    Task group is a task that is not executable.
    This class inherits all of its attributes from Task class.
    """

    def __init__(self):
        super(TaskGroup, self).__init__()

    def __str__(self):
        str_buffer = []
        str_buffer.append('(spec_task_group')
        str_buffer.append('\t(label {})'.format(self.label))
        str_buffer.append('\t(agent {})'.format(', '.join(self.agent)))
        if self.supertasks:
            str_buffer.append('\t(supertasks {})'.format(', '.join(self.supertasks)))
        str_buffer.append('\t(subtasks {})'.format(', '.join(self.subtasks)))
        str_buffer.append('\t(qaf {})'.format(self.qaf))
        if self.qaf_local:
            str_buffer.append('\t(qaf_local {})'.format(self.qaf_local))
        str_buffer.append(('\t(type {})'.format(self.type)))
        str_buffer.append(')')
        return '\n'.join(str_buffer)

    def to_dot(self, **kwargs):
        """
        Return .dot representation of this task and connections to its subtasks.

       Args:
            **kwargs: Various possible options for formatting the output.
        """
        node_label = '\t"{0}" [label="{0}\\n{1}"];'.format(self.label, ', '.join(self.agent))
        dot_buffer = [node_label]

        for (index, sub_label) in enumerate(self.subtasks):
            if index == (len(self.subtasks) - 1) / 2:
                dot_buffer.append('"{0}" -> "{1}" [label="{2}"];'.format(self.label, sub_label, self.qaf))
            else:
                dot_buffer.append('"{0}" -> "{1}";'.format(self.label, sub_label))

        return '\n\t'.join(dot_buffer) + '\n'

    @classmethod
    def init_with_values(cls, label, agents, subtasks, qaf, supertasks=None, qaf_local='', type='homogeneous'):
        """
        Initialize Task object by manually specifying its values.

        Args:
            label (str): Label of the task, its unique identifier.
            agents (list[str]): Agent types that participate in this task.
            subtasks (list[str]): Labels of task's subtasks
            qaf (str): Quality function identifier.
            supertasks (list[str]): Labels of task's supertasks - optional for TaskGroup / empty if root.
            qaf_local (str): Local quality function identifier.
            type (str): 'homogeneous'

        Returns:
            Task object.
        """
        task = cls()
        task.label = label
        task.agent = agents
        task.subtasks = subtasks
        task.supertasks = supertasks if supertasks is not None else []
        task.qaf = qaf
        task.qaf_local = qaf_local
        task.type = type
        return task


class Method(Task):
    """
    Class that represents a method in taems tree structure.

    Method is a task that is executable.
    This class inherits attributes from Task class.

    Attributes:
        outcome (list[dict]): Distributions of method outcome -
                - [0] quality_distribution: {value: probability}
                - [1] duration_distribution: {value: probability}
                - [2] cost_distribution: {value: probability}
        QualityEV (float): Expected value for quality of method's execution.
        CostEV (float): Expected value for cost of method's execution.
        DurationEV (float): Expected value for duration of method's execution.
        startTime (float): Start of method's execution.
        endTime (float): End of method's execution.
        accruedTime (float): Time elapsed since the start of method's execution.
        nonLocal (bool): True if method has to be executed by other agent.
        isDisabled (bool): True if method is disabled (can't be executed).
    """

    def __init__(self):
        super(Method, self).__init__()
        self.subtasks = None
        self.outcome = [{}, {}, {}]
        self.original_outcome = [{}, {}, {}]  # Used to reset the outcome when deactivating IRs.
        self.QualityEV = 0
        self.CostEV = 0
        self.DurationEV = 0
        # self.ProbQualityGreaterThanEV = 0
        # self.ProbCostLowerThanEV = 0
        # self.ProbDurationShorterThanEV = 0
        self.startTime = None
        self.endTime = None
        self.accruedTime = None
        self.nonLocal = False
        self.isDisabled = 0

    @classmethod
    def init_with_values(cls, label, agents, supertasks, outcome, type='homogeneous'):
        """
        Initialize Method object by manually specifying its values.

        Args:
            label (str): Label of the method, its unique identifier.
            agents (list[str]): Agent types that participate in this method.
            supertasks (list[str]): Labels of method's supertasks
            outcome (list[dict]): Distributions of method outcome -
                    - [0] quality_distribution: {value: probability}
                    - [1] duration_distribution: {value: probability}
                    - [2] cost_distribution: {value: probability}
            type (str): 'homogeneous'

        Returns:
            Method object.
        """
        method = cls()
        method.label = label
        method.agent = agents
        method.supertasks = supertasks
        method.outcome = outcome
        method.original_outcome = deepcopy(outcome)
        method.type = type
        return method

    def __str__(self):
        str_buffer = []
        str_buffer.append('(spec_method')
        str_buffer.append('\t(label {})'.format(self.label))
        str_buffer.append('\t(agent {})'.format(', '.join(self.agent)))
        str_buffer.append('\t(supertasks {})'.format(', '.join(self.supertasks)))

        str_buffer.append('\t(outcome')
        quality_distribution = ' '.join(['{} {}'.format(key, value) for key, value in self.outcome[0].items()])
        str_buffer.append(('\t\t(quality_distribution {})'.format(quality_distribution)))
        duration_distribution = ' '.join(['{} {}'.format(key, value) for key, value in self.outcome[1].items()])
        str_buffer.append('\t\t(duration_distribution {})'.format(duration_distribution))
        cost_distribution = ' '.join(['{} {}'.format(key, value) for key, value in self.outcome[2].items()])
        str_buffer.append(('\t\t(cost_distribution {})'.format(cost_distribution)))
        str_buffer.append('\t)')

        str_buffer.append(('\t(type {})'.format(self.type)))
        str_buffer.append(')')
        return '\n'.join(str_buffer)

    def to_dot(self, **kwargs):
        """
        Return .dot representation of this method.

        Args:
            **kwargs: Various possible options for formatting the output.
        """
        node_label = '\t"{0}" [label="{0}\\n{1}" shape=box];\n'.format(self.label, ', '.join(self.agent))

        return node_label

    def calcExpectations(self):
        """
        Calculate expected values for quality, duration and cost value distributions.

        Uses a helper function which calculates expected values of probability
        distributions defined as: dictionary {value: probability}.
        """
        self.QualityEV = helper_functions.calcExpectedValue(self.outcome[0])
        self.DurationEV = helper_functions.calcExpectedValue(self.outcome[1])
        self.CostEV = helper_functions.calcExpectedValue(self.outcome[2])


class Resource(Node):
    """
    An interface that represents resources in taems task structure.

    Attributes:
        state (float): Current state of resource (quantity of available resource).
        depleted_at (float): Minimum state of resource.
        overloaded_at (float): Maximum state of resource.
        isSufficient (bool): True if depleted_at < state < overloaded_at.
        type (int): 0 - consumable, 1 - non-consumable.
    """

    resource_types = {'none': -1, 'consumable': 0, 'non_consumable': 1}
    resource_names = {value: key for key, value in resource_types.items()}

    def __init__(self):
        super(Resource, self).__init__()
        self.state = 0
        self.defaultState = 0
        self.depleted_at = 0
        self.overloaded_at = 0
        self.isSufficient = None
        self.type = self.resource_types['none']

    @classmethod
    def init_from_values(cls, label, depleted_at, overloaded_at, state=None, agents=None):
        """
        Initialize Resource object by manually specifying its values.

        Args:
            label (str): Label of the resource, its unique identifier.
            depleted_at (float): Minimum state of resource.
            overloaded_at (float): Maximum state of resource.
            state (float): Current state of resource (quantity of available resource).
            agents (list[str]): Agent types that participate in this method.

        Returns:
            Resource object.
        """
        resource = cls()
        resource.label = label
        resource.agent = agents if agents is not None else []
        resource.state = state if state is not None else overloaded_at
        resource.defaultState = deepcopy(resource.state)
        resource.depleted_at = depleted_at
        resource.overloaded_at = overloaded_at
        return resource

    def __str__(self):
        str_buffer = []
        str_buffer.append('(spec_{}_resource'.format(self.resource_names[self.type]))
        str_buffer.append('\t(label {})'.format(self.label))
        if self.agent:
            str_buffer.append('\t(agent {})'.format(', '.join(self.agent)))
        str_buffer.append('\t(state {})'.format(self.state))
        str_buffer.append('\t(depleted_at {})'.format(self.depleted_at))
        str_buffer.append('\t(overloaded_at {})'.format(self.overloaded_at))
        str_buffer.append(')')
        return '\n'.join(str_buffer)

    def __repr__(self):
        return 'Resource({s.label}, {s.agent}, <{s.depleted_at}, {s.state}, {s.overloaded_at}>)'.format(s=self)

    def produce(self, amount, tree):
        """
        Produce the given amount of resource and change the sufficiency flag if needed.

        Args:
            amount (float): Amount of resource to produce.
            tree (TaemsTree): Taems tree which resource belongs to.
        """
        self.checkSufficiency()
        wasInsuficcient = not self.isSufficient
        self.state += amount

        self.checkSufficiency()

        if not self.isSufficient:
            self.activateLimits(tree)
        if self.isSufficient and wasInsuficcient:
            self.deactivateLimits(tree)

    def consume(self, amount, tree):
        """
        Consume the given amount of resource and change the sufficiency flag if needed.

        Args:
            amount (float): Amount of resource to consume.
            tree (TaemsTree): Taems tree which resource belongs to.
        """
        self.checkSufficiency()
        wasInsuficcient = not self.isSufficient
        self.state -= amount

        self.checkSufficiency()

        if not self.isSufficient:
            self.activateLimits(tree)
        if self.isSufficient and wasInsuficcient:
            self.deactivateLimits(tree)

    def checkSufficiency(self):
        """
        Check the current state of resource and set sufficiency flag.

        Flag is set to True if depleted_at < state < overloaded_at and
        False otherwise.
        """
        if self.depleted_at < self.state < self.overloaded_at:
            self.isSufficient = True
        else:
            self.isSufficient = False

    def activateLimits(self, tree):
        """
        Activate all limits interrelationships that have the insufficient
        resource as source (from field).

        Goes through all IR's of type limits (code 6) and checks to see if
        this resource is source. It then activates IRs that match criteria.

        Args:
            tree (TaemsTree): Taems tree structure with IR's and resources.
        """

        for ir in tree.IRs.values():
            if ir.type == Interrelationship.IR_types['limits']:
                if ir.From == self.label:
                    ir.activate(tree, 0)

    def deactivateLimits(self, tree):
        """
        Deactivate all limits interrelationships that now have sufficient amount
        of resource (from field).

        Goes through all IR's of type limits (code 6) and checks to see if
        this resource is source. It then deactivates IRs that match criteria.

        Args:
            tree (TaemsTree): Taems tree structure with IR's and resources.
        """

        for ir in tree.IRs.values():
            if ir.type == Interrelationship.IR_types['limits']:
                if ir.From == self.label and ir.To in tree.tasks:  # TODO: with agent related resources, ir.To should always be in tree
                    ir.deactivate(tree)


class ConsumableResource(Resource):
    """
    An interface that represents consumable resources in taems task structure.
    """

    def __init__(self):
        super(ConsumableResource, self).__init__()
        self.type = self.resource_types['consumable']


class NonConsumableResource(Resource):
    """
    An interface  that represents non-consumable resources in taems task structure.

    It has initial state to which it returns each time the action that changes
    its state finishes execution.
    """

    def __init__(self):
        super(NonConsumableResource, self).__init__()
        self.initialState = 0
        self.type = self.resource_types['non_consumable']

    def setInitalValue(self):
        """Set the resource's state to its initial value."""
        self.state = self.initialState


class Interrelationship(object):
    """
    Class that represents interrelationships in taems tree structure.

    Attributes:
        label (str): IR's label, unique identifier.
        agent (str): Label of agent who "owns" the IR.
        From (str): Label of the node that is the source of IR.
        To (str): Label of the node that is affected by IR.
        delay (float): Value of time delayed before the effects of IR take place.
        active (bool): True if IR is active
        type (int): Marks the type of IR. Can be: 0, 1, 2, 3, 4, 5 or 6.
                    See class static variable IR_types for details.
    """
    IR_types = {'none': -1,
                'enables': 0,
                'disables': 1,
                'facilitates': 2,
                'hinders': 3,
                'produces': 4,
                'consumes': 5,
                'limits': 6,
                'child_of': 7}

    IR_names = {value: key for key, value in IR_types.items()}

    def __init__(self):
        self.label = ""
        self.agent = ""
        self.From = ""
        self.To = ""
        self.delay = 0
        self.active = False
        self.type = self.IR_types['none']

    def __repr__(self):
        return '{}_{}_{}{}'.format(self.IR_names[self.type], self.From, self.To, '_active' if self.active else '')

    def to_dot(self, **kwargs):
        """
        Return .dot representation of this IR.

        Args:
            **kwargs: Various possible options for formatting the output.
        """
        options = 'style=dashed color=grey fontcolor=grey fontsize=0 constraint=false'
        edge = '\t"{0}" -> "{1}" [{2}  label={3}];\n'.format(self.From, self.To, options, self.IR_names[self.type])

        return edge

    @classmethod
    def init_with_values(cls, label, agents, From, To, delay=0):
        """
        Initialize Interrelationship object by manually specifying its values.

        Args:
            label (str): Label of the IR, its unique identifier.
            agents (list[str]): Agent types that participate in this IR.
            From (str): Label of the node that is the source of IR.
            To (str): Label of the node that is affected by IR.
            delay (float): Value of time delayed before the effects of IR take place.

        Returns:
            Interrelationship object.
        """
        IR = cls()
        IR.label = label
        IR.agent = agents
        IR.From = From
        IR.To = To
        IR.delay = delay
        return IR

    def buffer_common_attributes(self):
        """
        Append common attributes of Interrelationship class to a buffer.

        Buffer is used in __str__ methods of each subclass.
        """
        str_buffer = []
        str_buffer.append('(spec_{}'.format(self.IR_names[self.type]))
        str_buffer.append('\t(label {})'.format(self.label))
        str_buffer.append('\t(agent {})'.format(', '.join(self.agent)))
        str_buffer.append('\t(from {})'.format(self.From))
        str_buffer.append('\t(to {})'.format(self.To))
        if self.delay:
            str_buffer.append('\t(delay {})'.format(self.delay))
        return str_buffer


class IREnables(Interrelationship):
    """
    Class that represents enables interrelationship.

    This class inherits attributes from Interrelationship class.
    It overrides the `type` attribute.
    """

    def __init__(self):
        super(IREnables, self).__init__()
        self.type = self.IR_types['enables']

    def __str__(self):
        str_buffer = self.buffer_common_attributes()
        str_buffer.append(')')
        return '\n'.join(str_buffer)

    def activate(self, tree, time):
        """
        Activate IR enables.

        Set the destination node isDisabled flag to false, modify the destination's
        earliest start time if needed, change IR's state to active.

        Args:
            tree (TaemsTree): A taems tree.
            time (float): Current execution time.
        """
        tree.tasks[self.To].isDisabled -= 1
        # If activation is delayed, set methods earliest start time.
        if self.delay > 0:
            if (tree.tasks[self.To].earliestStartTime < (time + self.delay) or
                    tree.tasks[self.To].earliestStartTime is None):
                tree.tasks[self.To].earliestStartTime = time + self.delay

        self.active = True
        # tree.activeIR.append(self)


class IRDisables(Interrelationship):
    """
    Class that represents disables interrelationship.

    This class inherits attributes from Interrelationship class.
    It overrides the `type` attribute.
    """

    def __init__(self):
        super(IRDisables, self).__init__()
        self.type = self.IR_types['disables']

    def __str__(self):
        str_buffer = self.buffer_common_attributes()
        str_buffer.append(')')
        return '\n'.join(str_buffer)

    def activate(self, tree, time):
        """
        Activate IR disables.

        Set the destination node isDisabled flag to true, modify the destination's
        deadline if needed, change IR's state to active.

        Args:
            tree (TaemsTree): A taems tree.
            time (float): Current execution time.
        """
        tree.tasks[self.To].isDisabled += 1
        # If activation is delayed, set methods deadline.
        if self.delay > 0:
            if tree.tasks[self.To].deadline > (time + self.delay) or tree.tasks[self.To].deadline is None:
                tree.tasks[self.To].deadline = time + self.delay

        self.active = True
        # tree.activeIR.append(self)


class IRFacilitates(Interrelationship):
    """
    Class that represents facilitates interrelationship.

    This class inherits attributes from Interrelationship class.
    It overrides the `type` attribute.

    Attributes:
        quality_power (dict): Probability distribution of quality value which affects the destination node.
        cost_power (dict): Probability distribution of cost value which affects the destination node.
        duration_power (dict): Probability distribution of duration value which affects the destination node.
        startTime (float): IR's start time.
        q_powerEV (float): Expected value for quality.
        d_powerEV (float): Expected value for duration.
        c_powerEV (float): Expected value for cost.
  """

    def __init__(self):
        super(IRFacilitates, self).__init__()
        self.type = self.IR_types['facilitates']
        self.quality_power = {}
        self.cost_power = {}
        self.duration_power = {}
        self.startTime = None
        self.q_powerEV = -1
        self.d_powerEV = -1
        self.c_powerEV = -1

    def __str__(self):
        str_buffer = self.buffer_common_attributes()
        quality_power = ' '.join(['{} {}'.format(key, value) for key, value in self.quality_power.items()])
        str_buffer.append('\t(quality_power {}'.format(quality_power))
        duration_power = ' '.join(['{} {}'.format(key, value) for key, value in self.duration_power.items()])
        str_buffer.append('\t(duration_power {}'.format(duration_power))
        cost_power = ' '.join(['{} {}'.format(key, value) for key, value in self.cost_power.items()])
        str_buffer.append('\t(cost_power {}'.format(cost_power))
        str_buffer.append(')')
        return '\n'.join(str_buffer)

    def calcPowerEV(self):
        """Calculate expected values of power distributions."""
        self.q_powerEV = helper_functions.calcExpectedValue(self.quality_power)
        self.d_powerEV = helper_functions.calcExpectedValue(self.duration_power)
        self.c_powerEV = helper_functions.calcExpectedValue(self.cost_power)

    def activate(self, tree, time):
        """
        Activate IR facilitates.

        Modify the IR's start time, calculate quality, cost and duration
        expected values, modify the destination's outcome.

        Args:
            tree (TaemsTree): A taems tree.
            time (float): Current execution time.
        """
        if self.delay > 0:
            if self.startTime is None or self.startTime < time + self.delay:
                self.startTime = time + self.delay
        else:
            if self.startTime is None or self.startTime > time:
                self.startTime = time

        helper_functions.mutiplyDistribution(tree.tasks[self.To].outcome[0], 1 + self.q_powerEV)
        helper_functions.mutiplyDistribution(tree.tasks[self.To].outcome[1], 1 - self.d_powerEV)
        helper_functions.mutiplyDistribution(tree.tasks[self.To].outcome[2], 1 - self.c_powerEV)

        self.active = True
        # tree.activeIR.append(self)


class IRHinders(Interrelationship):
    """
    Class that represents hinders interrelationship.

    This class inherits attributes from Interrelationship class.
    It overrides the `type` attribute.

    Attributes:
        quality_power (dict): Probability distribution of quality value which affects the destination node.
        cost_power (dict): Probability distribution of cost value which affects the destination node.
        duration_power (dict): Probability distribution of duration value which affects the destination node.
        startTime (float): IR's start time.
        q_powerEV (float): Expected value for quality.
        d_powerEV (float): Expected value for duration.
        c_powerEV (float): Expected value for cost.
    """

    def __init__(self):
        super(IRHinders, self).__init__()
        self.type = self.IR_types['hinders']
        self.quality_power = {}
        self.cost_power = {}
        self.duration_power = {}
        self.startTime = None
        self.q_powerEV = -1
        self.d_powerEV = -1
        self.c_powerEV = -1

    def __str__(self):
        str_buffer = self.buffer_common_attributes()
        quality_power = ' '.join(['{} {}'.format(key, value) for key, value in self.quality_power.items()])
        str_buffer.append('\t(quality_power {}'.format(quality_power))
        duration_power = ' '.join(['{} {}'.format(key, value) for key, value in self.duration_power.items()])
        str_buffer.append('\t(duration_power {}'.format(duration_power))
        cost_power = ' '.join(['{} {}'.format(key, value) for key, value in self.cost_power.items()])
        str_buffer.append('\t(cost_power {}'.format(cost_power))
        str_buffer.append(')')
        return '\n'.join(str_buffer)

    def calcPowerEV(self):
        """Calculate expected values of power distributions."""
        self.q_powerEV = helper_functions.calcExpectedValue(self.quality_power)
        self.d_powerEV = helper_functions.calcExpectedValue(self.duration_power)
        self.c_powerEV = helper_functions.calcExpectedValue(self.cost_power)

    def activate(self, tree, time):
        """
        Activate IR facilitates.

        Modify the IR's start time, calculate quality, cost and duration
        expected values, modify the destination's outcome, activate the IR.

        Args:
            tree (TaemsTree): A taems tree.
            time (float): Current execution time.
        """
        if self.delay > 0:
            if self.startTime is None or self.startTime < time + self.delay:
                self.startTime = time + self.delay
        else:
            if self.startTime is None or self.startTime > time:
                self.startTime = time

        helper_functions.mutiplyDistribution(tree.tasks[self.To].outcome[0], 1 - self.q_powerEV)
        helper_functions.mutiplyDistribution(tree.tasks[self.To].outcome[1], 1 + self.d_powerEV)
        helper_functions.mutiplyDistribution(tree.tasks[self.To].outcome[2], 1 + self.c_powerEV)

        self.active = True
        # tree.activeIR.append(self)


class IRProduces(Interrelationship):
    """
    Class that represents produces interrelationship.

    This class inherits attributes from Interrelationship class.
    It overrides the `type` attribute.

    Attributes:
        model (str): The way the resources are produced: "per_time_unit" or "duration_independent".
        produces (dict): Probability distribution of quantity of resource IR produces.
    """

    def __init__(self):
        super(IRProduces, self).__init__()
        self.type = self.IR_types['produces']
        self.model = ""
        self.produces = {}

    @classmethod
    def init_with_values(cls, label, agents, From, To, produces, model='duration_independent', delay=0):
        """
        Initialize IRProduces object by manually specifying its values.

        Args:
            label (str): Label of the IR, its unique identifier.
            agents (list[str]): Agent types that participate in this IR.
            From (str): Label of the node that is the source of IR.
            To (str): Label of the node that is affected by IR.
            produces (dict): Probability distribution of quantity of resource IR produces.
            model (str): How resources are produced - 'duration_independent' or 'per_time_unit'.
            delay (float): Value of time delayed before the effects of IR take place.

        Returns:
            IRProduces object.
        """
        IR = cls()
        IR.label = label
        IR.agent = agents
        IR.From = From
        IR.To = To
        IR.produces = produces
        IR.model = model
        IR.delay = delay
        return IR

    def __str__(self):
        str_buffer = self.buffer_common_attributes()
        str_buffer.append('\t(model {})'.format(self.model))
        produces = ' '.join(['{} {}'.format(key, value) for key, value in self.produces.items()])
        str_buffer.append('\t(produces {})'.format(produces))
        str_buffer.append(')')
        return '\n'.join(str_buffer)

    def activate(self, tree):
        """
        Activate IR produces.

        Calculate the expected value of produced resource, produce the calculated
        amount of resource, activate the IR.

        Args:
            tree (TaemsTree): A taems tree.
        """
        EVproduced = helper_functions.calcExpectedValue(self.produces)

        resource = tree.resources[self.To]
        if self.model == "per_time_unit":
            resource.produce(EVproduced * tree.tasks[self.From].DurationEV, tree)

        elif self.model == "duration_independent":
            resource.produce(EVproduced, tree)

        self.active = True
        # tree.activeIR.append(self)


class IRConsumes(Interrelationship):
    """
    Class that represents consumes interrelationship.

    This class inherits attributes from Interrelationship class.
    It overrides the `type` attribute.

    Attributes:
        model (str): The way the resources are consumed: "per_time_unit" or "duration_independent".
        consumes (dict): Probability distribution of quantity of resource IR consumes.
    """

    def __init__(self):
        super(IRConsumes, self).__init__()
        self.type = self.IR_types['consumes']
        self.model = ""
        self.consumes = {}

    @classmethod
    def init_with_values(cls, label, agents, From, To, consumes, model='duration_independent', delay=0):
        """
        Initialize IRConsumes object by manually specifying its values.

        Args:
            label (str): Label of the IR, its unique identifier.
            agents (list[str]): Agent types that participate in this IR.
            From (str): Label of the node that is the source of IR.
            To (str): Label of the node that is affected by IR.
            consumes (dict): Probability distribution of quantity of resource IR consumes.
            model (str): How resources are produced - 'duration_independent' or 'per_time_unit'.
            delay (float): Value of time delayed before the effects of IR take place.

        Returns:
            IRConsumes object.
        """
        IR = cls()
        IR.label = label
        IR.agent = agents
        IR.From = From
        IR.To = To
        IR.consumes = consumes
        IR.model = model
        IR.delay = delay
        return IR

    def __str__(self):
        str_buffer = self.buffer_common_attributes()
        str_buffer.append('\t(model {})'.format(self.model))
        consumes = ' '.join(['{} {}'.format(key, value) for key, value in self.consumes.items()])
        str_buffer.append('\t(consumes {})'.format(consumes))
        str_buffer.append(')')
        return '\n'.join(str_buffer)

    def activate(self, tree):
        """
        Activate IR consumes.

        Calculate the expected value of consumed resource, consume the calculated
        amount of resource, activate the IR.

        Args:
            tree (TaemsTree): A taems tree.
    """
        EVconsumed = helper_functions.calcExpectedValue(self.consumes)

        resource = tree.resources[self.To]
        if self.model == "per_time_unit":
            resource.consume(EVconsumed * tree.tasks[self.From].DurationEV, tree)

        elif self.model == "duration_independent":
            resource.consume(EVconsumed, tree)

        self.active = True
        # tree.activeIR.append(self)


class IRLimits(Interrelationship):
    """
    Class that represents limits interrelationship.

    This class inherits attributes from Interrelationship class.
    It overrides the `type` attribute.

    Attributes:
        model (str): The way the limit affects the task: "per_time_click" or "duration_independent".
                  -> in this version, only duration independent mode is implemented
        quality_power (dict): Probability distribution of quality value which affects the destination node
        cost_power (dict): Probability distribution of cost value which affects the destination node
        duration_power (dict): Probability distribution of duration value which affects the destination node
        q_powerEV (float): Expected value for quality.
        d_powerEV (float): Expected value for duration.
        c_powerEV (float): Expected value for cost.
        startTime (float): IR's start time.
    """

    def __init__(self):
        super(IRLimits, self).__init__()
        self.type = self.IR_types['limits']
        self.model = ""
        self.quality_power = {}
        self.cost_power = {}
        self.duration_power = {}
        self.q_powerEV = -1
        self.d_powerEV = -1
        self.c_powerEV = -1
        self.startTime = None

    @classmethod
    def init_with_values(cls, label, agents, From, To, power, model='duration_independent', delay=0):
        """
        Initialize IRProduces object by manually specifying its values.

        Args:
            label (str): Label of the IR, its unique identifier.
            agents (list[str]): Agent types that participate in this IR.
            From (str): Label of the node that is the source of IR.
            To (str): Label of the node that is affected by IR.
            power (dict): Probability distributions of quality, cost and duration values which affect destination node.
            model (str): How resources are produced - 'duration_independent' or 'per_time_unit'.
            delay (float): Value of time delayed before the effects of IR take place.

        Returns:
            IRProduces object.
        """
        IR = cls()
        IR.label = label
        IR.agent = agents
        IR.From = From
        IR.To = To
        IR.quality_power = power['quality']
        IR.cost_power = power['cost']
        IR.duration_power = power['duration']
        IR.model = model
        IR.delay = delay
        return IR

    def __str__(self):
        str_buffer = self.buffer_common_attributes()
        quality_power = ' '.join(['{} {}'.format(key, value) for key, value in self.quality_power.items()])
        str_buffer.append('\t(quality_power {})'.format(quality_power))
        duration_power = ' '.join(['{} {}'.format(key, value) for key, value in self.duration_power.items()])
        str_buffer.append('\t(duration_power {})'.format(duration_power))
        cost_power = ' '.join(['{} {}'.format(key, value) for key, value in self.cost_power.items()])
        str_buffer.append('\t(cost_power {})'.format(cost_power))
        str_buffer.append('\t(model {})'.format(self.model))
        str_buffer.append(')')
        return '\n'.join(str_buffer)

    def activate(self, tree, time):
        """
        Activate IR limits.

        Modify the IR's start time, calculate quality, cost and duration
        expected values if needed, modify the destination's outcome,
        activate the IR.

        Args:
            tree (TaemsTree): A taems tree.
            time (float): Current execution time.
        """

        # Calculate EV only once.
        if self.q_powerEV == -1:
            self.q_powerEV = helper_functions.calcExpectedValue(self.quality_power)
            self.d_powerEV = helper_functions.calcExpectedValue(self.duration_power)
            self.c_powerEV = helper_functions.calcExpectedValue(self.cost_power)

        self.apply_ir_effects(self.To, tree, time)

        self.active = True
        # tree.activeIR.append(self)

    def apply_ir_effects(self, task, tree, time):
        """
        Apply IR effects.

        Args:
            task (str): Label of the task to which IR is applied.
            tree (TaemsTree): A taems tree.
            time (float): Current execution time.
        """

        if task not in tree.tasks.keys():
            return

        # If the task is a method.
        if tree.tasks[task].subtasks is None:
            if tree.tasks[task].nonLocal:
                return

            if self.delay > 0:
                if self.startTime is None or self.startTime < time + self.delay:
                    self.startTime = time + self.delay

            helper_functions.mutiplyDistribution(tree.tasks[task].outcome[0], 1 - self.q_powerEV)
            if self.d_powerEV == -1:
                helper_functions.mutiplyDistribution(tree.tasks[task].outcome[1], maxint)
            else:
                helper_functions.mutiplyDistribution(tree.tasks[task].outcome[1], 1 + self.d_powerEV)
            if self.c_powerEV == -1:
                helper_functions.mutiplyDistribution(tree.tasks[task].outcome[2], maxint)
            else:
                helper_functions.mutiplyDistribution(tree.tasks[task].outcome[2], 1 + self.c_powerEV)

        if tree.tasks[task].subtasks is not None:
            for subtask in tree.tasks[task].subtasks:
                self.apply_ir_effects(subtask, tree, time)

    def deactivate(self, tree):
        """
        Deactivate IR limits.

        Restore the destination's outcome, deactivate the IR.

        Args:
            tree (TaemsTree): A taems tree.
        """
        # Calculate EV only once.
        if self.q_powerEV == -1:
            self.q_powerEV = helper_functions.calcExpectedValue(self.quality_power)
            self.d_powerEV = helper_functions.calcExpectedValue(self.duration_power)
            self.c_powerEV = helper_functions.calcExpectedValue(self.cost_power)

        # If quality power is 1 (100%), when activating the IR, quality of the To task will drop to 0.
        # There is no way to return the original value when deactivating using math, so original value has to be copied.
        # TODO: check for better solution
        if self.q_powerEV == 1:
            tree.tasks[self.To].outcome[0] = deepcopy(tree.tasks[self.To].original_outcome[0])
        else:
            helper_functions.mutiplyDistribution(tree.tasks[self.To].outcome[0], 1 / (1 - self.q_powerEV))
        helper_functions.mutiplyDistribution(tree.tasks[self.To].outcome[1], 1 / (1 + self.d_powerEV))
        helper_functions.mutiplyDistribution(tree.tasks[self.To].outcome[2], 1 / (1 + self.c_powerEV))

        self.active = False
        # tree.activeIR.remove(self.label)


class IRChildOf(Interrelationship):
    """
    Class that represents child of interrelationship.

    This class inherits attributes from Interrelationship class.
    It overrides the `type` attribute.

    Attributes:
        From (str): Parent node's label.
        To (str): Child node's label.
    """

    def __init__(self, From, To, agent):
        """
        Initialize class.

        Args:
            From (str): Parent node's label.
            To (str): Child node's label.
            agent (str): Agent's label.
        """
        super(IRChildOf, self).__init__()
        self.type = self.IR_types['child_of']
        self.From = From
        self.To = To
        self.agent = agent

    def __str__(self):
        str_buffer = self.buffer_common_attributes()
        str_buffer.append(')')
        return '\n'.join(str_buffer)


class Commitment(object):

    def __init__(self):
        self.label = ""
        self.type = ""
        self.From = ""
        self.To = ""
        self.task = ""


class LocalCommitment(Commitment):

    def __init__(self):
        super(LocalCommitment, self).__init__()
        self.importance = 0
        self.min_quality = 0
        self.earliest_start_time = 0
        self.deadline = 0
        self.dont_interval_start = 0
        self.dont_interval_end = 0
        self.time_satisfied = 0


class NonLocalCommitment(Commitment):
    def __init__(self):
        super(NonLocalCommitment, self).__init__()
        self.quality_distribution = {}
        self.time_distribution = {}
