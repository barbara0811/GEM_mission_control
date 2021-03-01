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
