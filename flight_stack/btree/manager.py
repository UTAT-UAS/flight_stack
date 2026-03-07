from .utils import BTNode, STATUS

class BehaviorTree:
    """
    dont know if even necessary rn
    provides extra tick and completion handling. could provide blackboard interfacing?
    """

    def __init__(self, name, root:BTNode=None):
        self.name = name
        self.root = root
        self.status = STATUS.IDLE
    
    def setroot(self, root:BTNode):
        self.root = root
    
    def setup(self):
        if self.root is None:
            raise Exception("Root node not set")
        self.root.setup()  # should be recursive

    def initialize(self):
        if self.root is None:
            raise Exception("Root node not set")
        self.root.initialize()
        self.status = STATUS.RUNNING

    def tick(self):
        if self.root.status == STATUS.RUNNING:
            self.root.tick()
        elif self.root.status == STATUS.IDLE:
            raise Exception("Root node not initialized")
        else:
            print(f"Behavior tree {self.name} already completed with status {self.root.status}")
        self.status = self.root.status
        return self.status

    def reset(self):
        if self.root is None:
            raise Exception("Root node not set")
        self.root.reset()
        self.status = STATUS.IDLE