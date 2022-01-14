#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'barbanas'

import os
import random
import string
from copy import deepcopy

from gem_mission_control_lib.taems.taems import Method, TaemsTree, TaskGroup
from gem_mission_control_lib.taems.resources import ConsumableResource, NonConsumableResource
from gem_mission_control_lib.taems.interrelationships import IRConsumes, IRDisables, IREnables, IRFacilitates, IRHinders, IRLimits, IRProduces


class Loader(object):
    """Loader is a module that loads and parses .ttaems file (for TTAEMS file structure see documentation)

  Class interface:
      parse(self, filename, tree)

  Attributes:
      public:
          -
      private:
          lines: A list of strings, lines of loaded file
          index: An integer, index of line currently parsed
          tree: An instance of TaemsTree class that is being filled with data from file
  """

    def __init__(self):
        self.lines = []
        self.index = 0
        self.tree = None
        self.directory = ""
        self.prefix = ""
        self.prefixList = []

    def parse(self, directory, filename, labels, tree):
        """
        Parse the .taems file and load data into TaemsTree.

        The method reads the file (filename) line by line and searches for
        specifications of TAEMS elements - specification line starts with '('.
        It then determines what is the element specified based on the keyword
        (e.g. "spec_agent", "spec_task_group", ...) and calls the method that
        handles creation and loading of the specified element.

        Args:
            directory (str): Directory where .taems file is located.
            filename (str): Name of the file with TAEMS specification. Must have .ttaems extension.
            labels (list[str]): Labels (types) of agents in the tree structure.
            tree (TaemsTree): TaemsTree object that will be filled with data from file.
        """
        if directory is not None:
            self.directory = directory

        oldLines = self.lines
        oldIndex = self.index
        self.lines = []
        self.index = 0
        self.tree = tree

        if len(self.tree.agentLabels) == 0:
            self.tree.agentLabels = labels
        f = open(os.path.abspath(os.path.join(self.directory, filename)), 'r')
        self.lines = f.readlines()

        while self.index < len(self.lines):
            # l is one line of text file
            l = self.lines[self.index].strip()

            if self.skip_line(l):
                continue

            if l[0] == '(':
                '''if l[1:] == 'spec_agent':
            self.spec_agent()'''

                if l[1:] == 'spec_task_group':
                    self.spec_task_group()

                if l[1:] == 'spec_task':
                    self.spec_task_group()

                if l[1:] == 'spec_method':
                    self.spec_method()

                if l[1:] == 'spec_consumable_resource':
                    self.spec_resource(0)

                if l[1:] == 'spec_non_consumable_resource':
                    self.spec_resource(1)

                if l[1:] == 'spec_enables':
                    self.spec_IR(0)

                if l[1:] == 'spec_disables':
                    self.spec_IR(1)

                if l[1:] == 'spec_facilitates':
                    self.spec_IR(2)

                if l[1:] == 'spec_hinders':
                    self.spec_IR(3)

                if l[1:] == 'spec_produces':
                    self.spec_IR(4)

                if l[1:] == 'spec_consumes':
                    self.spec_IR(5)

                if l[1:] == 'spec_limits':
                    self.spec_IR(6)

                if l[1:] == 'spec_excludes':
                    self.spec_excludes()

            self.index = self.index + 1

        self.lines = oldLines
        self.index = oldIndex
        f.close()
        return self.tree

    def load_filenames(self, agent, namespace, filename):
        '''Parses the .ttaems file and loads data into TaemsTree.

    The method reads the file (filename) line by line and searches for
    specifications of TAEMS elements - specification line starts with '('. It then
    determines what is the element specified based on the keyword (e.g. "spec_agent",
    "spec_task_group", ...) and calls the method that handles creation and loading
    of the specified element.

    Args:
        filename: A string, name of the file with TAEMS specification
                    - filename should be given with .ttaems file extension (e.g. "my_tree.ttaems")
        tree:       TaemsTree object that should be filled with data from spec. file

    Returns:
        - none -
    '''

        print
        " ... loading  " + filename + "  ..."
        print

        self.lines = []
        self.index = 0
        dictionary = {}

        f = open(filename, 'r')
        self.lines = f.readlines()

        while self.index < len(self.lines):
            # l is one line of text file
            l = self.lines[self.index].strip()

            if self.skip_line(l):
                continue

            if l[0] == '(':
                if l[1:] == 'spec_taems_tree_file':
                    self.spec_taems_tree_file(agent, namespace, dictionary)

                self.index = self.index + 1

        return dictionary

    def add_agents(self, agent_list):

        for ag in agent_list:
            if ag not in self.tree.agentLabels:
                self.tree.agentLabels.append(ag)

    def skip_line(self, l):
        '''Checks to see if the given line should be skipped in parsing.

    Args:
        l: An string, line in file
    Returns:
        0/1: 1 - skip line, 0 - don't skip
    '''

        # if the line is empty
        if len(l) == 0:
            self.index = self.index + 1
            return 1

        # symbol ; at the begining of a line denotes comment
        elif l[0] == ';':
            self.index = self.index + 1
            return 1

        else:
            return 0

    ''' SPECIFICATION FUNCTIONS:
  '''

    def spec_taems_tree_file(self, agent, namespace, dictionary):
        '''Loads taems tree file specification data.

    fields in specification:
        non-optional fields:
            -> filename, task
    '''

        self.index = self.index + 1

        element = ["agent_label", "filename", "task_label", "type"]
        loaded = 0

        while 1:
            l = self.lines[self.index].strip()

            if self.skip_line(l):
                continue

            # end of task group specification
            if l[0] == ')':
                break

            if l[0] == '(':

                l = l[1:-1]
                space = l.find(' ')
                field = l[:space]
                l = l[space + 1:]

                if field == 'agent':
                    element[0] = l.strip()
                    loaded += 1

                if field == 'filename':
                    element[1] = l.strip()
                    loaded += 1

                if field == 'task':
                    element[2] = l.strip()
                    loaded += 1

                if field == 'type':
                    element[3] = l.strip()

            self.index = self.index + 1

        if loaded != 3:
            print
            "Wrong definition of taems tree file specification"
            return -1

        if element[0] != agent:
            return

        if element[3] == "dynamic":
            element[1] = namespace[1:] + element[1]
        dictionary[element[2]] = [element[1]]

    def spec_agent(self):
        '''Loads agent specification data. Creates Agent objects for all specified agents and
        loads them in self.tree.

    fields in specification:
        (label agent_name1)
        (label agent_name2)
        ...
    '''

        self.index = self.index + 1

        while 1:
            l = self.lines[self.index].strip()

            if self.skip_line(l):
                continue

            # end of agent specification
            if l[0] == ')':
                return

            if l[0] == '(':
                l = l[1:-1].replace("label ", "")
                # now l contains only agent's label
                self.tree.agentLabels.append(l)
                if self.tree.agent is not "":
                    raise Exception("[ERROR] Can't define more than one agent in taems!")
                self.tree.agent = l

            self.index = self.index + 1

    def spec_task_group(self):
        '''Loads task group specification data.

    fields in specification:
        non-optional fields:
            -> label, agent(s), subtask(s), qaf
        optional fields:
            -> supertask(s) -- no supertasks denotes the task group is root task
    '''

        self.index = self.index + 1

        taskGroup = TaskGroup()

        while 1:
            l = self.lines[self.index].strip()

            if self.skip_line(l):
                continue

            # end of task group specification
            if l[0] == ')':
                break

            if l[0] == '(':

                l = l[1:-1]
                space = l.find(' ')
                field = l[:space]
                l = l[space + 1:]

                if field == 'label':
                    taskGroup.label = l.strip()

                if field == 'agent':
                    array = l.replace(' ', '').split(',')
                    taskGroup.agent = array
                    if len(set(self.tree.agentLabels) & set(taskGroup.agent)) == 0:
                        return
                    # self.add_agents(array)

                if field == 'subtasks':
                    array = l.replace(' ', '').split(',')
                    taskGroup.subtasks = array
                    if len(self.prefix) > 0:
                        for i in range(len(taskGroup.subtasks)):
                            taskGroup.subtasks[i] = self.prefix + taskGroup.subtasks[i]

                if field == 'supertasks':
                    array = l.replace(' ', '').split(',')
                    taskGroup.supertasks = array
                    if len(self.prefix) > 0:
                        for i in range(len(taskGroup.supertasks)):
                            taskGroup.supertasks[i] = self.prefix + taskGroup.supertasks[i]
                    if len(array) == 1:
                        if array[0] == "Meta":
                            self.tree.homogeneousTasks.append(taskGroup.label)

                if field == 'qaf':
                    taskGroup.qaf = l.strip()

                if field == 'qaf_local':
                    taskGroup.qaf_local = l.strip()

                if field == 'type':
                    taskGroup.type = l.strip()

                if field == 'prefix':
                    self.prefixList.append(l.strip())
                    self.prefix = ""
                    for item in self.prefixList:
                        self.prefix = self.prefix + item

                if field == 'filename':
                    array = l.split(' ')

                    for i in range(len(array) / 2):
                        if array[i * 2] in self.tree.agentLabels:
                            filename = array[i * 2 + 1]
                            self.parse(None, filename, [], self.tree)
                            self.tree.tasks[self.prefix + taskGroup.label].supertasks = taskGroup.supertasks
                            del self.prefixList[-1]
                            self.prefix = ""
                            for item in self.prefixList:
                                self.prefix = self.prefix + item
                            return

            self.index = self.index + 1

        taskGroup.label = self.prefix + taskGroup.label

        if taskGroup.label not in self.tree.tasks.keys():
            self.tree.tasks[taskGroup.label] = taskGroup

        if len(taskGroup.supertasks) == 0:
            self.tree.rootTask.append(taskGroup.label)

        if taskGroup.qaf == 'q_seq_sum' or taskGroup.qaf == 'q_seq_sum_all':
            for i in range(len(taskGroup.subtasks) - 1):
                for j in range(i + 1, len(taskGroup.subtasks)):
                    ir = IREnables()
                    ir.agent = taskGroup.agent
                    ir.label = ''.join(random.choice(string.ascii_uppercase + string.digits) for _ in range(10))
                    ir.From = taskGroup.subtasks[i]
                    ir.To = taskGroup.subtasks[j]
                    self.tree.IRs[ir.label] = ir

    def spec_method(self):
        '''Loads method specification data.

    fields in specification:
        non-optional fields:
            -> label, agent(s), supertask(s), outcome
        optional fields:
            ->
    '''

        self.index = self.index + 1

        method = Method()

        while 1:
            l = self.lines[self.index].strip()

            if self.skip_line(l):
                continue

            # end of method specification
            if l[0] == ')':
                break

            if l[0] == '(' and ')' in l:

                l = l[1:-1]
                space = l.find(' ')
                field = l[:space]
                l = l[space + 1:]

                if field == 'label':
                    method.label = self.prefix + l.strip()

                if field == 'agent':
                    array = l.replace(' ', '').split(',')
                    method.agent = array
                    if len(set(self.tree.agentLabels) & set(method.agent)) == 0:
                        return
                    # self.add_agents(array)

                if field == 'supertasks':
                    array = l.replace(' ', '').split(',')
                    method.supertasks = array
                    if len(self.prefix) > 0:
                        for i in range(len(method.supertasks)):
                            method.supertasks[i] = self.prefix + method.supertasks[i]

            # outcome:

            elif l[0] == '(':
                l = l[1:]
                if not (l == 'outcome'):
                    print
                    'pogresno definiran .taems!!'

                self.index = self.index + 1
                while 1:
                    l = self.lines[self.index].strip()

                    if self.skip_line(l):
                        continue

                    # end of outcome specification
                    if l[0] == ')':
                        break

                    if l[0] == '(':
                        l = l[1:-1]
                        space = l.find(' ')
                        field = l[:space]
                        l = l[space + 1:]

                        # indeks 0
                        if field == 'quality_distribution':
                            array = l.split(' ')

                            for i in range(len(array) / 2):
                                method.outcome[0][float(array[2 * i])] = float(array[2 * i + 1])
                                method.original_outcome[0] = deepcopy(method.outcome[0])

                        # indeks 1
                        elif field == 'duration_distribution':
                            array = l.split(' ')

                            for i in range(len(array) / 2):
                                method.outcome[1][float(array[2 * i])] = float(array[2 * i + 1])
                                method.original_outcome[1] = deepcopy(method.outcome[1])

                        # indeks 2
                        elif field == 'cost_distribution':
                            array = l.split(' ')

                            for i in range(len(array) / 2):
                                method.outcome[2][float(array[2 * i])] = float(array[2 * i + 1])
                                method.original_outcome[2] = deepcopy(method.outcome[2])

                    self.index = self.index + 1

            self.index = self.index + 1

        method.calcExpectations()
        # method.calcProb()

        self.tree.tasks[method.label] = method
        self.tree.methodLabels.append(method.label)

    def spec_resource(self, res_type):
        '''Loads resource specification data.

    fields in specification:
        non-optional fields:
            -> label, state, depleted_at, overloaded_at
        optional fields:
            -> agent
    '''

        self.index = self.index + 1

        if (res_type == 0):
            resource = ConsumableResource()
        elif (res_type == 1):
            resource = NonConsumableResource()

        while 1:
            l = self.lines[self.index].strip()

            if self.skip_line(l):
                continue

            # end of resource specification
            if l[0] == ')':
                break

            if l[0] == '(':

                l = l[1:-1]
                space = l.find(' ')
                field = l[:space]
                l = l[space + 1:]

                if field == 'label':
                    resource.label = l.strip()

                if field == 'agent':
                    resource.agent = l.strip()
                    if len(set(self.tree.agentLabels) & set(resource.agent)) == 0:
                        return
                    # self.add_agents([resource.agent])

                if field == 'state':
                    resource.state = float(l.strip())
                    resource.defaultState = deepcopy(resource.state)
                    if (res_type == 1):
                        resource.initialState = deepcopy(resource.state)

                if field == 'depleted_at':
                    resource.depleted_at = float(l.strip())

                if field == 'overloaded_at':
                    resource.overloaded_at = float(l.strip())

            self.index = self.index + 1

        self.tree.resources[resource.label] = resource

    def spec_IR(self, IR_type):
        '''Loads interrelationship specification data.

    fields in specification:
        non-optional fields:
            -> label, agent, from, to
        optional fields:
            -> from_outcome, delay
    '''

        self.index = self.index + 1

        """
    IR types:
        0 - enables
        1 - disables
        2 - facilitates
        3 - hinders
        4 - produces
        5 - consumes
        6 - limits
        7 - excludes
    """

        if (IR_type == 0):
            IR = IREnables()
        elif (IR_type == 1):
            IR = IRDisables()
        elif (IR_type == 2):
            IR = IRFacilitates()
        elif (IR_type == 3):
            IR = IRHinders()
        elif (IR_type == 4):
            IR = IRProduces()
        elif (IR_type == 5):
            IR = IRConsumes()
        elif (IR_type == 6):
            IR = IRLimits()

        while 1:
            l = self.lines[self.index].strip()

            if self.skip_line(l):
                continue

            # end of IR specification
            if l[0] == ')':
                break

            if l[0] == '(':

                l = l[1:-1]
                space = l.find(' ')
                field = l[:space]
                l = l[space + 1:]

                if field == 'label':
                    IR.label = l.strip()

                if field == 'agent':
                    array = l.replace(' ', '').split(',')
                    IR.agent = array
                    if len(set(self.tree.agentLabels) & set(IR.agent)) == 0:
                        return
                    # self.add_agents(array)

                if field == 'from':
                    IR.From = l.strip()

                if field == 'to':
                    IR.To = l.strip()

                if field == 'delay':
                    IR.delay = float(l.strip())

                if field == 'quality_power':
                    if (IR_type != 2 and IR_type != 3 and IR_type != 6):
                        print
                        "pogresno definiran .taems!!!"

                    array = l.split(' ')

                    for i in range(len(array) / 2):
                        IR.quality_power[float(array[2 * i])] = float(array[2 * i + 1])

                if field == 'cost_power':
                    if (IR_type != 2 and IR_type != 3 and IR_type != 6):
                        print
                        "pogresno definiran .taems!!!"

                    array = l.split(' ')

                    for i in range(len(array) / 2):
                        IR.cost_power[float(array[2 * i])] = float(array[2 * i + 1])

                if field == 'duration_power':
                    if (IR_type != 2 and IR_type != 3 and IR_type != 6):
                        print
                        "pogresno definiran .taems!!!"

                    array = l.split(' ')

                    for i in range(len(array) / 2):
                        IR.duration_power[float(array[2 * i])] = float(array[2 * i + 1])

                if field == 'model':
                    if (IR_type != 4 and IR_type != 5 and IR_type != 6):
                        print
                        "pogresno definiran .taems!!!"

                    IR.model = l.strip()

                if field == 'produces':
                    if (IR_type != 4):
                        print
                        "pogresno definiran .taems!!!"

                    array = l.split(' ')

                    for i in range(len(array) / 2):
                        IR.produces[float(array[2 * i])] = float(array[2 * i + 1])

                if field == 'consumes':
                    if (IR_type != 5):
                        print
                        "pogresno definiran .taems!!!"

                    array = l.split(' ')

                    for i in range(len(array) / 2):
                        IR.consumes[float(array[2 * i])] = float(array[2 * i + 1])

            self.index = self.index + 1

        '''if IR.type < 2:
        self.tree.hardIR.append(IR.label)'''

        if IR.type == 2 or IR.type == 3:
            IR.calcPowerEV()
            # self.tree.softIR.append(IR.label)

        '''if IR.From not in self.tree.IRsKeyMethodFrom.keys():
        self.tree.IRsKeyMethodFrom[IR.From] = [IR]
    else:
        self.tree.IRsKeyMethodFrom[IR.From].append(IR)'''

        self.tree.IRs[IR.label] = IR

    def spec_excludes(self):

        self.index = self.index + 1

        while 1:
            l = self.lines[self.index].strip()

            if self.skip_line(l):
                continue

            # end of excludes specification
            if l[0] == ')':
                break

            if l[0] == '(':

                l = l[1:-1]
                space = l.find(' ')
                field = l[:space]
                l = l[space + 1:]

                if field == 'tasks':
                    array = l.replace(' ', '').split(',')

            self.index = self.index + 1

        self.tree.mutuallyExclusiveTasks.append(array)
