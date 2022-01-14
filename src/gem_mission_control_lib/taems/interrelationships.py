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
        return '{}_{}->{}{}'.format(self.IR_names[self.type], self.From, self.To, '_active' if self.active else '')

    def to_dot(self, **kwargs):
        """
        Return .dot representation of this IR.

        Args:
            **kwargs: Various possible options for formatting the output.
        """
        options = 'style=dashed color=grey fontcolor=grey fontsize=0 constraint=false'
        edge = '\t"{0}" -> "{1}" [{2}  label={3}];\n'.format(
            self.From, self.To, options, self.IR_names[self.type])

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
        quality_power = ' '.join(['{} {}'.format(key, value)
                                  for key, value in self.quality_power.items()])
        str_buffer.append('\t(quality_power {}'.format(quality_power))
        duration_power = ' '.join(['{} {}'.format(key, value)
                                   for key, value in self.duration_power.items()])
        str_buffer.append('\t(duration_power {}'.format(duration_power))
        cost_power = ' '.join(['{} {}'.format(key, value)
                               for key, value in self.cost_power.items()])
        str_buffer.append('\t(cost_power {}'.format(cost_power))
        str_buffer.append(')')
        return '\n'.join(str_buffer)

    def calcPowerEV(self):
        """Calculate expected values of power distributions."""
        self.q_powerEV = helper_functions.calcExpectedValue(self.quality_power)
        self.d_powerEV = helper_functions.calcExpectedValue(
            self.duration_power)
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

        helper_functions.mutiplyDistribution(
            tree.tasks[self.To].outcome[0], 1 + self.q_powerEV)
        helper_functions.mutiplyDistribution(
            tree.tasks[self.To].outcome[1], 1 - self.d_powerEV)
        helper_functions.mutiplyDistribution(
            tree.tasks[self.To].outcome[2], 1 - self.c_powerEV)

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
        quality_power = ' '.join(['{} {}'.format(key, value)
                                  for key, value in self.quality_power.items()])
        str_buffer.append('\t(quality_power {}'.format(quality_power))
        duration_power = ' '.join(['{} {}'.format(key, value)
                                   for key, value in self.duration_power.items()])
        str_buffer.append('\t(duration_power {}'.format(duration_power))
        cost_power = ' '.join(['{} {}'.format(key, value)
                               for key, value in self.cost_power.items()])
        str_buffer.append('\t(cost_power {}'.format(cost_power))
        str_buffer.append(')')
        return '\n'.join(str_buffer)

    def calcPowerEV(self):
        """Calculate expected values of power distributions."""
        self.q_powerEV = helper_functions.calcExpectedValue(self.quality_power)
        self.d_powerEV = helper_functions.calcExpectedValue(
            self.duration_power)
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

        helper_functions.mutiplyDistribution(
            tree.tasks[self.To].outcome[0], 1 - self.q_powerEV)
        helper_functions.mutiplyDistribution(
            tree.tasks[self.To].outcome[1], 1 + self.d_powerEV)
        helper_functions.mutiplyDistribution(
            tree.tasks[self.To].outcome[2], 1 + self.c_powerEV)

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
        produces = ' '.join(['{} {}'.format(key, value)
                             for key, value in self.produces.items()])
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
            resource.produce(
                EVproduced * tree.tasks[self.From].DurationEV, tree)

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
        consumes = ' '.join(['{} {}'.format(key, value)
                             for key, value in self.consumes.items()])
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
            resource.consume(
                EVconsumed * tree.tasks[self.From].DurationEV, tree)

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
        quality_power = ' '.join(['{} {}'.format(key, value)
                                  for key, value in self.quality_power.items()])
        str_buffer.append('\t(quality_power {})'.format(quality_power))
        duration_power = ' '.join(['{} {}'.format(key, value)
                                   for key, value in self.duration_power.items()])
        str_buffer.append('\t(duration_power {})'.format(duration_power))
        cost_power = ' '.join(['{} {}'.format(key, value)
                               for key, value in self.cost_power.items()])
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
            self.q_powerEV = helper_functions.calcExpectedValue(
                self.quality_power)
            self.d_powerEV = helper_functions.calcExpectedValue(
                self.duration_power)
            self.c_powerEV = helper_functions.calcExpectedValue(
                self.cost_power)

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

            helper_functions.mutiplyDistribution(
                tree.tasks[task].outcome[0], 1 - self.q_powerEV)
            if self.d_powerEV == -1:
                helper_functions.mutiplyDistribution(
                    tree.tasks[task].outcome[1], maxint)
            else:
                helper_functions.mutiplyDistribution(
                    tree.tasks[task].outcome[1], 1 + self.d_powerEV)
            if self.c_powerEV == -1:
                helper_functions.mutiplyDistribution(
                    tree.tasks[task].outcome[2], maxint)
            else:
                helper_functions.mutiplyDistribution(
                    tree.tasks[task].outcome[2], 1 + self.c_powerEV)

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
            self.q_powerEV = helper_functions.calcExpectedValue(
                self.quality_power)
            self.d_powerEV = helper_functions.calcExpectedValue(
                self.duration_power)
            self.c_powerEV = helper_functions.calcExpectedValue(
                self.cost_power)

        # If quality power is 1 (100%), when activating the IR, quality of the To task will drop to 0.
        # There is no way to return the original value when deactivating using math, so original value has to be copied.
        # TODO: check for better solution
        if self.q_powerEV == 1:
            tree.tasks[self.To].outcome[0] = deepcopy(
                tree.tasks[self.To].original_outcome[0])
        else:
            helper_functions.mutiplyDistribution(
                tree.tasks[self.To].outcome[0], 1 / (1 - self.q_powerEV))
        helper_functions.mutiplyDistribution(
            tree.tasks[self.To].outcome[1], 1 / (1 + self.d_powerEV))
        helper_functions.mutiplyDistribution(
            tree.tasks[self.To].outcome[2], 1 / (1 + self.c_powerEV))

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