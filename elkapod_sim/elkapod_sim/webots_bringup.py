import rclpy
from rclpy.executors import MultiThreadedExecutor
from .elkapod_simulation_comm_server import ElkapodSimulationCommServer


class MyRobotDriver:
    def init(self, webots_node, properties):
        self._robot = webots_node.robot

        rclpy.init(args=None)
        self._node = ElkapodSimulationCommServer(self._robot)
        self._executor = MultiThreadedExecutor(num_threads=6)

    def step(self):
        rclpy.spin_once(self._node, timeout_sec=0, executor=self._executor)
