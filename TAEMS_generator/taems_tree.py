from copy import deepcopy


class TAEMSNode(object):
    def __init__(self, node_type, label, agent, qaf, outcome, super_tasks, children=None, task_type=None,
                 qafLocal=None):
        self.node_type = node_type
        self.label = label
        self.agent = agent  # This is actually agent type, e.g. UAV_scout
        self.qaf = qaf
        self.qafLocal = qafLocal
        self.outcome = outcome
        self.super_tasks = super_tasks
        self.children = []  # list of actual children (type: TAEMSNode)
        if len(children) == 1:
            if len(children[0]) == 0:
                children = None
        self.childrenLabels = children if children is not None else []  # only labels of all children
        self.type = task_type if task_type is not None else 'homogeneous'

    def __str__(self):
        buffer = []

        buffer.append('(spec_%s' % self.node_type)
        buffer.append('\t(label %s)' % self.label)
        buffer.append('\t(agent %s)' % ','.join(self.agent))

        if self.super_tasks is not None:
            buffer.append('\t(supertasks %s)' % self.super_tasks)

        if len(self.childrenLabels) > 0:
            buffer.append('\t(subtasks %s)' % ', '.join(self.childrenLabels))

        if self.outcome is not None:
            buffer.append('\t(outcome')

            for key, value in self.outcome.items():
                buffer.append('\t\t(%s %s)' % (key, value))

            buffer.append('\t)')

        if self.qaf is not None:
            buffer.append('\t(qaf %s)' % self.qaf)

        if self.qafLocal is not None:
            buffer.append('\t(qaf_local %s)' % self.qafLocal)

        buffer.append('\t(type %s)' % self.type)

        buffer.append(')')

        return '\n'.join(buffer)

    def copy(self):
        children_copy = [child.copy() for child in self.children]
        outcome_copy = None

        if self.outcome is not None:
            outcome_copy = self.outcome.copy()

        n = TAEMSNode(self.node_type, self.label, self.agent, self.qaf, outcome_copy, self.super_tasks,
                      deepcopy(self.childrenLabels), self.type, self.qafLocal)
        n.children = children_copy
        return n


class TAEMSIR(object):
    def __init__(self, type, label, agent, fromNode, toNode, delay=None, qPower=None, dPower=None, cPower=None,
                 model=None, produces=None, consumes=None):
        self.type = type
        self.label = label
        self.agent = agent
        self.fromNode = fromNode
        self.toNode = toNode
        self.qPower = qPower
        self.dPower = dPower
        self.cPower = cPower
        self.model = model
        self.produces = produces
        self.consumes = consumes
        self.delay = delay

    def __str__(self):
        buffer = []

        buffer.append('(spec_%s' % self.type)
        buffer.append('\t(label %s)' % self.label)
        buffer.append('\t(agent %s)' % ','.join(self.agent))
        buffer.append('\t(from %s)' % self.fromNode)
        buffer.append('\t(to %s)' % self.toNode)
        if self.qPower is not None:
            buffer.append('\t(quality_power %s)' % self.qPower)
        if self.dPower is not None:
            buffer.append('\t(duration_power %s)' % self.dPower)
        if self.cPower is not None:
            buffer.append('\t(cost_power %s)' % self.cPower)
        if self.model is not None:
            buffer.append('\t(model %s)' % self.model)
        if self.produces is not None:
            buffer.append('\t(produces %s)' % self.produces)
        if self.consumes is not None:
            buffer.append('\t(consumes %s)' % self.consumes)
        if self.delay is not None:
            buffer.append('\t(delay %s)' % self.delay)
        buffer.append(')')

        return '\n'.join(buffer)

    def copy(self):

        return TAEMSIR(self.type, self.label, self.agent, self.fromNode, self.toNode, self.delay, self.qPower, \
                       self.dPower, self.cPower, self.model, self.produces, self.consumes)


class TAEMSExcludes(object):
    def __init__(self, fromNode, toNode):
        self.fromNode = fromNode
        self.toNode = toNode

    def __str__(self):
        buffer = []
        buffer.append('(spec_excludes')
        buffer.append('\t(tasks %s, %s)' % (self.fromNode, self.toNode,))
        buffer.append(')')

        return '\n'.join(buffer)

    def copy(self):
        return TAEMSExcludes(self.fromNode, self.toNode)


class TAEMSTree(object):
    def __init__(self, root, IRs, excludes=None, taskLabels=[]):
        self.root = root
        self.taskLabels = taskLabels  # List of all task labels in a tree
        self.IRs = IRs
        self.excludes = excludes

    def copy(self):

        if self.IRs is None:
            return TAEMSTree(self.root.copy(), None, deepcopy(self.excludes), deepcopy(self.taskLabels))
        else:
            return TAEMSTree(self.root.copy(), self.IRs.copy(), deepcopy(self.excludes), deepcopy(self.taskLabels))

    def filter_agent(self, agent_id):

        intersection = set(agent_id) & set(self.root.agent)

        if len(intersection) == 0:
            self.root = None
        else:
            self.root.agent = list(intersection)
            self._filter_agent(self.root, agent_id)

            for ir in self.IRs.values():
                if len(set(agent_id) & set(ir.agent)) == 0:
                    self.IRs.pop(ir.label)

        return self

    def _filter_agent(self, node, agent_id):

        for child in node.children:
            intersection = set(agent_id) & set(child.agent)
            if len(intersection) == 0:
                self.taskLabels.remove(child.label)
            else:
                child.agent = list(intersection)
                self._filter_agent(child, agent_id)

    def remove_tasks(self, tasks):
        self._remove_tasks(self.root, tasks)

    def _remove_tasks(self, node, tasks):

        if len(node.children) == 0:
            return

        to_remove = set()
        for i in range(len(node.children)):
            if node.children[i].label in tasks:
                to_remove.add(i)
            else:
                self._remove_tasks(node.children[i], tasks)

        removed = 0
        for i in to_remove:
            del node.children[i - removed]
            removed += 1

    def dump_to_file(self, filename, mode='w'):
        with open(filename, mode) as file:
            if self.root is None:
                file.write('% this is an empty taems file\n')
            else:
                self._dump_to_file(self.root, file)

                for ir in self.IRs:
                    file.write(str(ir))

                for e in self.excludes:
                    file.write(str(e))

    def _dump_to_file(self, node, file):

        if node.label not in self.taskLabels:
            return

        file.write(str(node))
        file.write('\n\n')

        for child in node.children:
            self._dump_to_file(child, file)

    @staticmethod
    def load_from_file(filename):

        with open(filename, 'r') as file:
            type_node = None
            root = None
            values = {}
            taskLabels = []
            nodes = {}
            IRs = {}
            excludes = []

            multi_line_name = []
            multi_line_values = {}

            for line in file:
                line = line.strip()

                if not line or line.startswith('%'):
                    continue

                if type_node is None:
                    type_node = line[6:].strip()

                if type_node in ['method', 'task_group', 'enables', 'disables', 'facilitates', \
                                 'hinders', 'produces', 'consumes', 'limits', 'excludes']:

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
                                subtasks = node_info.get('subtasks', '').split(', ')
                                label = node_info.get('label')
                                task_type = node_info.get('type')

                                nodes[label] = TAEMSNode(type_node, label,
                                                         node_info.get('agent').replace(' ', '').split(','),
                                                         node_info.get('qaf'),
                                                         values.get('outcome'), supertasks, subtasks, task_type,
                                                         node_info.get('qaf_local'))
                                if len(label) > 0:
                                    taskLabels.append(label)

                                if supertasks is None:
                                    root = nodes[label]

                            elif type_node in ['enables', 'disables', 'facilitates', 'hinders', 'produces', 'consumes',
                                               'limits']:

                                node_info = values.get('spec_' + type_node)

                                label = node_info.get('label')

                                IRs[label] = TAEMSIR(type_node, label,
                                                     node_info.get('agent').replace(' ', '').split(','),node_info.get('from'), node_info.get('to'),
                                                     node_info.get('delay'), node_info.get('quality_power'),
                                                     node_info.get('duration_power'),
                                                     node_info.get('cost_power'), node_info.get('model'),
                                                     node_info.get('produces'),
                                                     node_info.get('consumes'), )

                            elif type_node in ['excludes']:

                                node_info = values.get('spec_' + type_node)

                                tasks = node_info.get('tasks').split(', ')

                                excludes.append(TAEMSExcludes(tasks[0], tasks[1]))

                            type_node = None
                            values = {}

            for node in nodes.values():
                node.children = [nodes[child] for child in node.childrenLabels if child in nodes]

            return TAEMSTree(root, IRs, excludes, taskLabels)
