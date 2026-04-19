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
    
    def setup(self, blackboard:dict):
        self.blackboard = blackboard

    def initialize(self):
        self.status = STATUS.RUNNING
    
    def reset(self):
        self.status = STATUS.IDLE

    def tick(self):
        pass