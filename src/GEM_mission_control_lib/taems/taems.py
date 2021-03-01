#!/usr/bin/env python
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
        agent_classes (list[str]): Labels of all agents included in the execution of methods in taems tree.
        tasks (dict[Task]): Tasks in the tree in form of: {task/method label: TaskGroup/Method object}
        rootTask (list[str]): A list with one! element, label of root task.
        methodLabels (list[str]): Labels of atomic methods.
        resources (dict): {Resource label: Resource object}.
        IRs (dict): Interrelationships in the tree in form of {IR label: Interrelationship object}.
        activeIR (list): Active interrelationships' labels.
    """

    def __init__(self):
        self.agent_classes = []
        self.tasks = OrderedDict()
        self.rootTask = []
        self.methodLabels = []
        self.resources = OrderedDict()
        self.IRs = OrderedDict()
        self.mutuallyExclusiveTasks = []
        self.homogeneousTasks = []

    @classmethod
    def init_with_values(cls, agent_classes, root, tasks, IRs, resources):
        """
        Initialize TaemsTree object by manually specifying its values.

        Args:
            agent_classes (list[str]): Labels of all agents included in the execution of methods in taems tree.
            root (str): Label of root task.
            tasks (dict[str, Any]): Tasks in the tree in form of: {task/method label: TaskGroup/Method object}
            IRs (dict[str, Any]): Interrelationships in the tree in form of {IR label: Interrelationship object}.
            resources (dict[str, Any]): Resources in the tree in the form of: {resource label: Resource object}.

        Returns:
            TaemsTree object.
        """
        tree = cls()
        tree.agent_classes = agent_classes
        tree.tasks = tasks
        tree.rootTask = [root]
        tree.methodLabels = [key for key, value in tasks.items() if type(
            value).__name__ == 'Method']
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
                            raise ValueError(
                                "Illegal input format for single line")

                        if len(multi_line_name) > 0:
                            if multi_line_name[-1] not in multi_line_values.keys():
                                multi_line_values[multi_line_name[-1]] = {}
                            multi_line_values[multi_line_name[-1]
                                              ][parts[0]] = parts[1]
                        else:
                            values[parts[0]] = parts[1]
                    elif line.startswith('('):
                        multi_line_name.append(line[1:])
                    elif line.endswith(')'):

                        if len(multi_line_name) > 0:
                            values[multi_line_name[-1]
                                   ] = multi_line_values[multi_line_name[-1]]

                            multi_line_values.pop(multi_line_name[-1])
                            del multi_line_name[-1]

                        if len(multi_line_name) == 0:

                            if type_node in ['method', 'task_group']:
                                node_info = values.get('spec_' + type_node)

                                supertasks = node_info.get('supertasks')
                                supertasks = [
                                    supertasks] if supertasks is not None else None
                                subtasks = node_info.get(
                                    'subtasks', '').split(', ')
                                label = node_info.get('label')
                                task_type = node_info.get(
                                    'type', 'homogeneous')
                                agent = node_info.get('agent').replace(
                                    ' ', '').split(',')
                                qaf = node_info.get('qaf')
                                outcome = values.get('outcome')
                                # Outcome is loaded as dict of dicts with {'distribution': list(value, probability...)}
                                # Convert outcome to be list of dicts with {value: probability}
                                if outcome is not None:
                                    outcome_list = []
                                    for distribution in [x + '_distribution' for x in ['quality', 'duration', 'cost']]:
                                        outcome_iter = iter(
                                            outcome[distribution].split(' '))
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
                                    nodes[label] = Method.init_with_values(
                                        **kwargs)
                                elif type_node == 'task_group':
                                    kwargs.update(
                                        {'subtasks': subtasks, 'qaf': qaf, 'qaf_local': qaf_local})
                                    nodes[label] = TaskGroup.init_with_values(
                                        **kwargs)

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
                                    IRs[label] = IREnables.init_with_values(
                                        **kwargs)
                                elif type_node == 'disables':
                                    IRs[label] = IRDisables.init_with_values(
                                        **kwargs)
                                elif type_node == 'limits':
                                    # powers are loaded as 'key value key value key value...'.
                                    # IR must be initialized with a dict with those keys and values.
                                    read_list = node_info.get(
                                        'quality_power').split(' ')
                                    quality_power = dict(
                                        zip(read_list[0::2], read_list[1::2]))
                                    read_list = node_info.get(
                                        'duration_power').split(' ')
                                    duration_power = dict(
                                        zip(read_list[0::2], read_list[1::2]))
                                    read_list = node_info.get(
                                        'cost_power').split(' ')
                                    cost_power = dict(
                                        zip(read_list[0::2], read_list[1::2]))
                                    power = {
                                        'quality': quality_power, 'duration': duration_power, 'cost': cost_power}
                                    kwargs.update(
                                        {'power': power, 'model': node_info.get('model', 'time_independent')})
                                    IRs[label] = IRLimits.init_with_values(
                                        **kwargs)
                                elif type_node == 'produces':
                                    # produces is loaded as 'key value key value key value...'.
                                    # IR must be initialized with a dict with those keys and values.
                                    produces_list = node_info.get(
                                        'produces').split(' ')
                                    produces_dict = dict(
                                        zip(produces_list[0::2], produces_list[1::2]))
                                    kwargs.update({'produces': produces_dict,
                                                   'model': node_info.get('model', 'time_independent')})
                                    IRs[label] = IRProduces.init_with_values(
                                        **kwargs)
                                elif type_node == 'consumes':
                                    # consumes is loaded as 'key value key value key value...'.
                                    # IR must be initialized with a dict with those keys and values.
                                    consumes_list = node_info.get(
                                        'consumes').split(' ')
                                    consumes_dict = dict(
                                        zip(consumes_list[0::2], consumes_list[1::2]))
                                    kwargs.update({'consumes': consumes_dict,
                                                   'model': node_info.get('model', 'time_independent')})
                                    IRs[label] = IRConsumes.init_with_values(
                                        **kwargs)

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
                                    resources[label] = ConsumableResource.init_from_values(
                                        **kwargs)
                                elif type_node == 'non_consumable_resource':
                                    resources[label] = NonConsumableResource.init_from_values(
                                        **kwargs)

                            type_node = None
                            values = {}

            return TaemsTree.init_with_values(list(agent_labels), root.label, nodes, IRs, resources)

    def copy(self):
        """
        Return a copy of the tree.
        """
        new_tree = TaemsTree.init_with_values(deepcopy(self.agent_classes),
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
                value.subtasks = [re.sub(pattern, replacement, x)
                                  for x in value.subtasks]
            value.supertasks = [re.sub(pattern, replacement, x)
                                for x in value.supertasks]
            new_key = re.sub(pattern, replacement, key)
            new_tasks[new_key] = value
        self.tasks = new_tasks

        # Replace placeholder in root task
        self.rootTask = [re.sub(pattern, replacement, self.rootTask[0])]

        # Replace placeholders in method labels
        self.methodLabels = [re.sub(pattern, replacement, x)
                             for x in self.methodLabels]

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
            to_remove = [IR.label for IR in self.IRs.values() if len(
                set(agent_labels) & set(IR.agent)) == 0]
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

        result.agent_classes = deepcopy(first.agent_classes)
        result.agent_classes.extend(second.agent_classes)

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
                result.tasks[task].deadline = min(
                    [result.tasks[task].deadline, second.tasks[task].deadline])
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
        str_buffer.append('\t(supertasks {})'.format(
            ', '.join(self.supertasks)))
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
            str_buffer.append('\t(supertasks {})'.format(
                ', '.join(self.supertasks)))
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
        node_label = '\t"{0}" [label="{0}\\n{1}"];'.format(
            self.label, ', '.join(self.agent))
        dot_buffer = [node_label]

        for (index, sub_label) in enumerate(self.subtasks):
            if index == (len(self.subtasks) - 1) / 2:
                dot_buffer.append(
                    '"{0}" -> "{1}" [label="{2}"];'.format(self.label, sub_label, self.qaf))
            else:
                dot_buffer.append(
                    '"{0}" -> "{1}";'.format(self.label, sub_label))

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
        # Used to reset the outcome when deactivating IRs.
        self.original_outcome = [{}, {}, {}]
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
        str_buffer.append('\t(supertasks {})'.format(
            ', '.join(self.supertasks)))

        str_buffer.append('\t(outcome')
        quality_distribution = ' '.join(
            ['{} {}'.format(key, value) for key, value in self.outcome[0].items()])
        str_buffer.append(
            ('\t\t(quality_distribution {})'.format(quality_distribution)))
        duration_distribution = ' '.join(
            ['{} {}'.format(key, value) for key, value in self.outcome[1].items()])
        str_buffer.append(
            '\t\t(duration_distribution {})'.format(duration_distribution))
        cost_distribution = ' '.join(
            ['{} {}'.format(key, value) for key, value in self.outcome[2].items()])
        str_buffer.append(
            ('\t\t(cost_distribution {})'.format(cost_distribution)))
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
        node_label = '\t"{0}" [label="{0}\\n{1}" shape=box];\n'.format(
            self.label, ', '.join(self.agent))

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
