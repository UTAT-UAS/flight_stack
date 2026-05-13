from flight_stack.flight_stack import FlightPlanner

class STATUS:
    IDLE = 0
    RUNNING = 1
    SUCCESS = 2
    FAILURE = 3

class BTNode:
    """
    Assumes will not be ticked if status is not RUNNING.
    """
    def __init__(self, name:str):
        self.name = name
        self.status = STATUS.IDLE
        self.blackboard = {}
        self.fp = None

    def setup(self, blackboard:dict, fp:FlightPlanner):
        self.blackboard = blackboard
        self.fp = fp

    def initialize(self):
        self.status = STATUS.RUNNING

    def reset(self):
        self.status = STATUS.IDLE

    def tick(self):
        return self.status
