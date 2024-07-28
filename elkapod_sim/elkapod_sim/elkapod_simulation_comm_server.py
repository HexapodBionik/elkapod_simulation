import numpy as np
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from elkapod_msgs.msg import LegFrames, TouchSensorArrayStamped


class Leg:
    """! Leg of Elkapod robot.

    Provides interface for MegaLynx robot.
    """

    def __init__(self, s1, s2, s3, transform):
        """
        @param s1: Servo 1 instance
        @param s2: Servo 2 instance
        @param s3: Servo 3 instance
        @param transform: Coordinate system transform to be applied
        """
        self._s1 = s1
        self._s2 = s2
        self._s3 = s3

        self.transform = transform

    def set_angles(self, q: np.ndarray) -> None:
        q = self.transform(q)
        q = np.deg2rad(q)

        self._s1.setPosition(q[0])
        self._s2.setPosition(q[1])
        self._s3.setPosition(q[2])


class ElkapodSimulationCommServer(Node):
    def __init__(self, robot):
        super().__init__("elkapod_comm_server")

        # Touch Sensors refresh rate [Hz]
        self._ts_refresh_rate = 100

        # Setup elkapod simulation model
        self._robot = robot
        servo11 = self._robot.getDevice("Servo11")
        servo12 = self._robot.getDevice("Servo12")
        servo13 = self._robot.getDevice("Servo13")
        servo21 = self._robot.getDevice("Servo21")
        servo22 = self._robot.getDevice("Servo22")
        servo23 = self._robot.getDevice("Servo23")
        servo31 = self._robot.getDevice("Servo31")
        servo32 = self._robot.getDevice("Servo32")
        servo33 = self._robot.getDevice("Servo33")
        servo41 = self._robot.getDevice("Servo41")
        servo42 = self._robot.getDevice("Servo42")
        servo43 = self._robot.getDevice("Servo43")
        servo51 = self._robot.getDevice("Servo51")
        servo52 = self._robot.getDevice("Servo52")
        servo53 = self._robot.getDevice("Servo53")
        servo61 = self._robot.getDevice("Servo61")
        servo62 = self._robot.getDevice("Servo62")
        servo63 = self._robot.getDevice("Servo63")

        ts1 = self._robot.getDevice("l1_touch_sensor")
        ts2 = self._robot.getDevice("l2_touch_sensor")
        ts3 = self._robot.getDevice("l3_touch_sensor")
        ts4 = self._robot.getDevice("l4_touch_sensor")
        ts5 = self._robot.getDevice("l5_touch_sensor")
        ts6 = self._robot.getDevice("l6_touch_sensor")
        self._ts_list = [ts1, ts2, ts3, ts4, ts5, ts6]
        for ts in self._ts_list:
            ts.enable(1)

        # Create leg objects (for ElkapodSimplified.proto)
        leg1 = Leg(servo11, servo12, servo13, lambda q: q * np.array([-1.0, 1.0, 1.0]))
        leg2 = Leg(servo21, servo22, servo23, lambda q: q * np.array([-1.0, 1.0, 1.0]))
        leg3 = Leg(servo31, servo32, servo33, lambda q: q * np.array([-1.0, 1.0, 1.0]))
        leg4 = Leg(servo41, servo42, servo43, lambda q: q + np.array([180, 0.0, 0.0]))
        leg5 = Leg(servo51, servo52, servo53, lambda q: q + np.array([180, 0.0, 0.0]))
        leg6 = Leg(servo61, servo62, servo63, lambda q: q + np.array([180, 0.0, 0.0]))

        # Create leg objects (for ElkapodV1.proto)
        # leg1 = Leg(servo11, servo12, servo13,
        #            lambda q: q + np.array([0.0, 17.5, 40.7]))
        # leg2 = Leg(servo21, servo22, servo23,
        #            lambda q: q + np.array([0.0, 17.5, 40.7]))
        # leg3 = Leg(servo31, servo32, servo33,
        #            lambda q: q + np.array([0.0, 17.5, 40.7]))
        # leg4 = Leg(servo41, servo42, servo43,
        #            lambda q: q + np.array([0.0, 17.5, 40.7]))
        # leg5 = Leg(servo51, servo52, servo53,
        #            lambda q: q + np.array([0.0, 17.5, 40.7]))
        # leg6 = Leg(servo61, servo62, servo63,
        #            lambda q: q + np.array([0.0, 17.5, 40.7]))

        self._legs = [leg1, leg2, leg3, leg4, leg5, leg6]

        self._elkapod_leg_subscription = self.create_subscription(
            LegFrames,
            "elkapod_comm_server_leg_frames",
            self._leg_frame_callback,
            10,
            callback_group=ReentrantCallbackGroup(),
        )

        self._pub = self.create_publisher(
            TouchSensorArrayStamped, "/elkapod_touch_sensors", 10
        )
        self._t = self.create_timer(
            1 / self._ts_refresh_rate,
            self._touch_sensor_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def _touch_sensor_callback(self):
        msg = TouchSensorArrayStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        for i, ts in enumerate(self._ts_list, start=1):
            msg.sensor_state[i - 1] = bool(ts.getValue())
        self._pub.publish(msg)

    def _leg_frame_callback(self, msg):
        leg_frames = msg.leg_frames
        for leg, frame in zip(self._legs, leg_frames):
            angles = frame.servo_angles.tolist()
            leg.set_angles(np.array([*angles]))
