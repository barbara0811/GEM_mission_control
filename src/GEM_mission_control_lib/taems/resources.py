from ummc_agent_lib.taems.taems import Node

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
        str_buffer.append(
            '(spec_{}_resource'.format(self.resource_names[self.type]))
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
                # TODO: with agent related resources, ir.To should always be in tree
                if ir.From == self.label and ir.To in tree.tasks:
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