from flight_stack.flight_stack import FlightPlanner
from .utils import BTNode, STATUS

class ControlNode(BTNode):
    def __init__(self, name, children:list[BTNode]=[]):
        super().__init__(name)
        self.children = children

    def setup(self, blackboard:dict, fp:FlightPlanner):
        super().setup(blackboard, fp)
        for child in self.children:
            child.setup(blackboard, fp)

    def initialize(self):
        super().initialize()
        for child in self.children:
            child.initialize()

    def reset(self):
        for child in self.children:
            child.reset()
        super().reset()

    def tick(self):
        pass

class Sequence(ControlNode):
    """
    Ticks its children in order. Fails if any child fails. Succeeds once all children succeed.
    """
    def __init__(self, name, children:list[BTNode]=[]):
        super().__init__(name, children)
        self.index = 0

    def reset(self):
        super().reset()
        self.index = 0

    def tick(self):
        child_status = self.children[self.index].tick()
        if child_status == STATUS.SUCCESS:
            print(f"{self.name}: child {self.index} succeeded")
            self.index += 1
            if self.index == len(self.children):
                self.status = STATUS.SUCCESS
                return self.status
            else:
                self.children[self.index].initialize()
        elif child_status == STATUS.FAILURE:
            self.status = STATUS.FAILURE
            return self.status
        else:
            return self.status


class Parallel(ControlNode):
    """Ticks all children each tick, sequentially. Fails if any child fails (default).
    
    Args:
        best_attempt (bool): alternative behavior to continue ticking until all children are finished
    """
    def __init__(self, name, children:list[BTNode]=[], best_attempt=False):
        super().__init__(name, children)
        self.best_attempt = best_attempt
        self.failed = False

    def reset(self):
        super().reset()
        self.failed = False

    def tick(self):
        completed = True
        for child in self.children:
            if child.status == STATUS.RUNNING:
                status = child.tick()
                if status == STATUS.FAILURE:
                    self.failed = True
                    print(f"{self.name}: child {child.name} failed")
                elif status == STATUS.RUNNING:
                    completed = False # checking here allows Parallel to return success on same tick
                else:
                    print(f"{self.name}: child {child.name} succeeded")

        if completed:
            self.status = STATUS.FAILURE if self.failed else STATUS.SUCCESS
        if self.failed and not self.best_attempt:
            self.status = STATUS.FAILURE
        return self.status


class Fallback(ControlNode):
    """
    Ticks its children in order. Succeeds if any child succeeds. Fails once all children fail.
    """
    def __init__(self, name, children:list[BTNode]=[]):
        super().__init__(name, children)
        self.index = 0

    def reset(self):
        super().reset()
        self.index = 0

    def tick(self):
        child_status = self.children[self.index].tick()
        if child_status == STATUS.FAILURE:
            print(f"{self.name}: child {self.index} failed")
            self.index += 1
            if self.index == len(self.children):
                self.status = STATUS.FAILURE
                return self.status
            else:
                self.children[self.index].initialize()
        elif child_status == STATUS.SUCCESS:
            self.status = STATUS.SUCCESS
            print(f"{self.name}: child {self.index} succeeded")
            return self.status
        else:
            return self.status
