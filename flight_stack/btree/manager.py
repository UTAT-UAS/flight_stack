from px4_msgs.msg import TrajectorySetpoint, GotoSetpoint
from flight_stack.flight_stack import FlightPlanner

from .utils import BTNode, STATUS

class BehaviorTree:
    """
    Provides blackboard interfacing and ros topics
    """

    def __init__(self, name, root:BTNode=None):
        self.name = name
        self.root = root
        self.blackboard = {}
        self.status = STATUS.IDLE
        self.fp = None

    def setroot(self, root:BTNode):
        self.root = root

    def setup(self, fp:FlightPlanner):
        self.fp = fp
        if self.root is None:
            raise Exception("Root node not set")
        self.root.setup(self.blackboard, fp)  # should be recursive

    def initialize(self):
        if self.root is None:
            raise Exception("Root node not set")
        
        # Publishing
        self.blackboard["traj_sp_pub_req"] = False
        self.blackboard["traj_sp"] = TrajectorySetpoint()
        self.blackboard["goto_sp_pub_req"] = False
        self.blackboard["goto_sp"] = GotoSetpoint()

        self.root.initialize()
        self.status = STATUS.RUNNING

    def tick(self):
        if self.root.status == STATUS.RUNNING:
            self.root.tick()
        elif self.root.status == STATUS.IDLE:
            raise Exception("Root node not initialized")
        else:
            print(f"Behavior tree {self.name} already completed with status {self.root.status}")

        # publish
        if self.blackboard["traj_sp_pub_req"]:
            self.blackboard["traj_sp_pub_req"] = False
            # can perform sanity checks here if desired
            self.fp._traj_publisher.publish(self.blackboard["traj_sp"])
        if self.blackboard["goto_sp_pub_req"]:
            self.blackboard["goto_sp_pub_req"] = False
            self.fp._goto_publisher.publish(self.blackboard["goto_sp"])

        self.status = self.root.status
        return self.status

    def reset(self):
        if self.root is None:
            raise Exception("Root node not set")
        self.root.reset()
        self.status = STATUS.IDLE
