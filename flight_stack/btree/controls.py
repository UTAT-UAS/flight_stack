from .utils import BTNode, STATUS

class ControlNode(BTNode):
    def __init__(self, name, children:list[BTNode]=[]):
        super().__init__(name)
        self.children = children

    def setup(self):
        #super().setup()
        for child in self.children:
            child.setup()

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
            print("child", self.index, "succeeded")
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
            self.status = child_status
            return self.status