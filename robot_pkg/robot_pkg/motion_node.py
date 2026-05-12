import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from geometry_msgs.msg import PointStamped
from pymoveit2 import MoveIt2

JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]


class MotionNode(Node):
    """
    Wraps MoveIt2 and exposes simple ROS2 services for high-level motion commands.

    Services (all std_srvs/Trigger):
        /robot/move_home      — move to the home joint configuration
        /robot/move_overview  — move to the overview (photo) position
        /robot/move_to_red    — approach the last known red cube position
        /robot/move_to_yellow — approach the last known yellow cube position
        /robot/move_to_blue   — approach the last known blue cube position
        /robot/move_to_search_N — move to search position N (N = 0, 1, 2, ...)

    Subscriptions:
        vision/red_position_3d, vision/yellow_position_3d, vision/blue_position_3d
    """

    def __init__(self):
        super().__init__('motion_node')

        self.declare_parameter('home_joints',     [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0])
        self.declare_parameter('overview_joints', [0.0, -1.2,    1.0, -1.35,  -1.5708, 0.0])
        self.declare_parameter('search_count', 3)
        self.declare_parameter('search_joints_0', [0.3,  -1.2, 1.0, -1.35, -1.5708, 0.0])
        self.declare_parameter('search_joints_1', [-0.3, -1.2, 1.0, -1.35, -1.5708, 0.0])
        self.declare_parameter('search_joints_2', [0.0,  -1.0, 1.2, -1.5,  -1.5708, 0.0])
        self.declare_parameter('approach_height', 0.10)
        self.declare_parameter('approach_quat',   [1.0, 0.0, 0.0, 0.0])
        self.declare_parameter('group_name',   'ur_manipulator')
        self.declare_parameter('base_frame',   'base_link')
        self.declare_parameter('end_effector', 'tool0')

        self._home_joints     = list(self.get_parameter('home_joints').value)
        self._overview_joints = list(self.get_parameter('overview_joints').value)
        self._approach_height = self.get_parameter('approach_height').value
        self._approach_quat   = list(self.get_parameter('approach_quat').value)
        search_count          = self.get_parameter('search_count').value
        self._search_joints   = [
            list(self.get_parameter(f'search_joints_{i}').value)
            for i in range(search_count)
        ]

        self._cube_pos: dict[str, PointStamped | None] = {
            'red': None, 'yellow': None, 'blue': None
        }

        cb = ReentrantCallbackGroup()

        self._moveit2 = MoveIt2(
            node=self,
            joint_names=JOINT_NAMES,
            base_link_name=self.get_parameter('base_frame').value,
            end_effector_name=self.get_parameter('end_effector').value,
            group_name=self.get_parameter('group_name').value,
            callback_group=cb,
        )

        self.create_service(Trigger, 'robot/move_home',     self._svc_home,     callback_group=cb)
        self.create_service(Trigger, 'robot/move_overview', self._svc_overview,  callback_group=cb)
        self.create_service(Trigger, 'robot/move_to_red',    self._svc_red,      callback_group=cb)
        self.create_service(Trigger, 'robot/move_to_yellow', self._svc_yellow,   callback_group=cb)
        self.create_service(Trigger, 'robot/move_to_blue',   self._svc_blue,     callback_group=cb)

        for i in range(len(self._search_joints)):
            self.create_service(
                Trigger, f'robot/move_to_search_{i}',
                lambda req, res, idx=i: self._svc_search(req, res, idx),
                callback_group=cb,
            )

        for color in ('red', 'yellow', 'blue'):
            self.create_subscription(
                PointStamped,
                f'vision/{color}_position_3d',
                lambda msg, c=color: self._position_cb(msg, c),
                10,
            )

        self.get_logger().info('Motion node ready')

    # ── position cache ────────────────────────────────────────────────────────

    def _position_cb(self, msg: PointStamped, color: str):
        self._cube_pos[color] = msg

    # ── helpers ───────────────────────────────────────────────────────────────

    def _execute_joints(self, joints: list[float]) -> bool:
        self._moveit2.move_to_configuration(joints)
        self._moveit2.wait_until_executed()
        return True

    def _execute_pose(self, x: float, y: float, z: float) -> bool:
        self._moveit2.move_to_pose(
            position=[x, y, z],
            quat_xyzw=self._approach_quat,
        )
        self._moveit2.wait_until_executed()
        return True

    def _approach_cube(self, color: str, res: Trigger.Response) -> Trigger.Response:
        pos = self._cube_pos[color]
        if pos is None:
            res.success = False
            res.message = f'No 3D position available for {color} — cube not detected'
            self.get_logger().warn(res.message)
            return res
        x = pos.point.x
        y = pos.point.y
        z = pos.point.z + self._approach_height
        self.get_logger().info(f'Approaching {color} cube at ({x:.3f}, {y:.3f}, {z:.3f})')
        self._execute_pose(x, y, z)
        res.success = True
        res.message = f'Approached {color} cube'
        return res

    # ── service callbacks ─────────────────────────────────────────────────────

    def _svc_home(self, req, res):
        self.get_logger().info('Moving to home')
        self._execute_joints(self._home_joints)
        res.success = True
        res.message = 'At home'
        return res

    def _svc_overview(self, req, res):
        self.get_logger().info('Moving to overview')
        self._execute_joints(self._overview_joints)
        res.success = True
        res.message = 'At overview'
        return res

    def _svc_red(self, req, res):
        return self._approach_cube('red', res)

    def _svc_yellow(self, req, res):
        return self._approach_cube('yellow', res)

    def _svc_blue(self, req, res):
        return self._approach_cube('blue', res)

    def _svc_search(self, req, res, idx: int):
        self.get_logger().info(f'Moving to search position {idx}')
        self._execute_joints(self._search_joints[idx])
        res.success = True
        res.message = f'At search position {idx}'
        return res


def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
