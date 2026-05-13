import time

from flight_stack.flight_stack import FlightPlanner
from .utils import BTNode, STATUS

class DecoratorNode(BTNode):
    def __init__(self, name, child:BTNode=None):
        super().__init__(name)
        self.child = child

    def setup(self, blackboard:dict, fp:FlightPlanner):
        super().setup(blackboard, fp)
        self.child.setup(blackboard, fp)

    def initialize(self):
        super().initialize()
        self.child.initialize()

    def reset(self):
        self.child.reset()
        super().reset()

    def tick(self):
        pass


class RemapStatus(DecoratorNode):
    def __init__(self, name, child:BTNode=None, remap={STATUS.FAILURE: STATUS.SUCCESS}):
        super().__init__(name, child)
        self.remap = remap

    def tick(self):
        child_status = self.child.tick()
        self.status = self.remap.get(child_status, child_status)
        return self.status


class Retry(DecoratorNode):
    """
    While child fails, will retry a specified number of times before returning failure.
    """
    def __init__(self, name, child:BTNode=None, retries=3):
        super().__init__(name, child)
        self.retries = retries
        self.attempts = 0

    def reset(self):
        super().reset()
        self.attempts = 0

    def tick(self):
        child_status = self.child.tick()
        if child_status == STATUS.SUCCESS:
            self.status = STATUS.SUCCESS
            return self.status
        elif child_status == STATUS.FAILURE:
            self.attempts += 1
            if self.attempts >= self.retries:
                self.status = STATUS.FAILURE
            else:
                self.child.reset()
                self.child.initialize()
            return self.status
        else:
            # running
            return self.status


class Timeout(DecoratorNode):
    """
    If child does not succeed within a specified time limit, returns failure.
    """
    def __init__(self, name, child:BTNode=None, timeout=5):
        super().__init__(name, child)
        self.timeout = timeout
        self.start_time = None

    def initialize(self):
        super().initialize()
        self.start_time = time.time()

    def reset(self):
        super().reset()
        self.start_time = None

    def tick(self):
        if time.time() - self.start_time > self.timeout:
            self.status = STATUS.FAILURE
            self.child.status = STATUS.FAILURE  # don't leave in running state
            return self.status

        child_status = self.child.tick()
        self.status = child_status
        return self.status
