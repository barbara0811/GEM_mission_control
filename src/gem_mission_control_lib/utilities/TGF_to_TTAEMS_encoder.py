#! /usr/bin/env python

__author__ = 'barbanas'

import sys
from gem_mission_control_lib.taems import taems

class Task(object):

    def __init__(self, label):
        self.label = label
        self.subtasks = []
        self.supertasks = []

    def add_subtask(self, subtask):
        self.subtasks.append(subtask)

    def add_supertask(self, supertask):
        self.supertasks.append(supertask)


def output_taems(file, tree, taskLabel):

    task = tree.tasks[taskLabel]

    if type(task) is taems.Method:
        file.write("(spec_method\n")
        file.write("   (label " + task.label + ")\n")
        file.write("   (agent )\n")
        if len(task.supertasks) > 0:
            file.write("   (supertasks " + list_to_str(task.supertasks) + ")\n")
        file.write("   (outcome\n")
        file.write("         (quality_distribution 1 1.0)\n")
        file.write("         (duration_distribution 0 1.0)\n")
        file.write("         (cost_distribution 0 1.0)\n")
        file.write("   )\n")
        file.write(")\n\n")

    else:
        file.write("(spec_task_group\n")
        file.write("   (label " + task.label + ")\n")
        file.write("   (agent )\n")
        if len(task.supertasks) > 0:
            file.write("   (supertasks " + list_to_str(task.supertasks) + ")\n")
        file.write("   (subtasks " + list_to_str(task.subtasks) + ")\n")
        file.write("   (qaf )\n")
        file.write(")\n\n")

        for subtask in task.subtasks:
            output_taems(file, tree, subtask)

def list_to_str(array):

    outStr = ""
    for i in range(len(array) - 1):
        outStr = outStr + str(array[i]) + ", "
    outStr = outStr + str(array[-1])

    return outStr

if __name__ == "__main__":

    if len(sys.argv) < 2:
        print "\n USAGE: python TGF_to_TTAEMS.py filename.tgf [filename.taems]"
        sys.exit(0)

    print "\n READING " + sys.argv[1] + " ..."
    try:
        inFile = open(sys.argv[1], "r")
    except IOError as e:
        print "\n I/O error({0}): {1}".format(e.errno, e.strerror)

    tree = taems.TaemsTree()
    tasks = {}  # {task id: Task object}

    lines = inFile.readlines()
    if len(lines) == 0:
        print "\n ERROR: empty input file"
    i = 0

    # read node definitions
    while lines[i][0] != "#":
        array = lines[i].replace('\r', '').replace('\n', '').split(' ')
        tasks[array[0]] = Task(array[1])
        i += 1

    # read subtask relation definitions
    i += 1
    while i < len(lines):
        array = lines[i].replace('\r', '').replace('\n', '').split(' ')
        tasks[array[0]].add_subtask(array[1])
        tasks[array[1]].add_supertask(array[0])
        i += 1

    inFile.close()

    # put data in taems tree structure
    rootTask = []
    for task in tasks.values():
        # extract supertasks
        supertasks = []
        if len(task.supertasks) > 0:
            for item in task.supertasks:
                supertasks.append(tasks[item].label)

        if len(task.subtasks) == 0:
            # method
            method = taems.Method()
            method.label = task.label
            method.supertasks = supertasks

            tree.tasks[task.label] = method
        else:
            # task group
            taskGroup = taems.TaskGroup()
            taskGroup.label = task.label
            taskGroup.supertasks = supertasks
            # root task
            if len(supertasks) == 0:
                rootTask.append(task.label)
            for item in task.subtasks:
                taskGroup.subtasks.append(tasks[item].label)

            tree.tasks[task.label] = taskGroup

    if len(rootTask) != 1:
        print "ERROR: input tree has more than one root task"
        sys.exit(0)

    if len(sys.argv) == 3:
        filename = sys.argv[2]
    else:
        filename = sys.argv[1].replace(".tgf", ".taems")

    try:
        outFile = open(filename, "w")
    except IOError as e:
        print "\n I/O error({0}): {1}".format(e.errno, e.strerror)

    print "\n WRITING to " + filename + " ..."

    output_taems(outFile, tree, rootTask[0])

    outFile.close()



