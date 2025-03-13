#!/usr/bin/env python3

import math
from math import pi
from threading import Thread

# ROS imports
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# MoveIt2 and UR-specific imports
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur

# Messages / Services
from geometry_msgs.msg import Pose, Quaternion, Point
from ur_msgs.srv import SetIO
from ur_commander.srv import VisualizePoses

# Utils
from scipy.spatial.transform import Rotation as R


class URController(Node):
    """
    Example node that sets up a MoveIt2 environment for a UR robot and
    demonstrates moving from home to a target pose.
    """

    def __init__(self, node_name="ur_controller"):
        super().__init__(node_name)
        self.callback_group = ReentrantCallbackGroup()

        # Create MoveIt2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=ur.joint_names(),
            base_link_name=ur.base_link_name(),
            end_effector_name=ur.end_effector_name(),
            group_name=ur.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

        # Executor config
        self.moveit2.max_velocity = 0.1
        self.moveit2.max_acceleration = 0.05
        self.moveit2.planner_id = "RRTconnectkConfigDefault"

        # Collision environment setup
        self._add_collision_objects()

    def _add_collision_objects(self):
        """
        Add all collision boxes used in the environment.
        """
        self.get_logger().info("Adding collision objects to planning scene...")

        husky_size = {"x": 0.99, "y": 0.67, "z": 0.39}
        boxes = {"x": 0.325, "y": 0.49, "z": 0.40}
        ground_height = -husky_size["z"] - 0.01/2 - 0.05

        # Husky
        self.moveit2.add_collision_box(
            id="husky",
            size=[husky_size["x"] + 0.05, husky_size["y"] + 0.05, husky_size["z"] + 0.05],
            position=[-0.3, 0.0, -husky_size["z"]/2 - 0.05],
            quat_xyzw=[0.0, 0.0, 0.0, 0.0],
        )

        # Controller
        self.moveit2.add_collision_box(
            id="controller",
            size=[0.45, 0.65, 0.7],
            position=[-0.7, 0.0, 0.34],
            quat_xyzw=[0.0, 0.0, 0.0, 0.0],
        )

        # Floor
        self.moveit2.add_collision_box(
            id="floor",
            size=[4.0, 4.0, 0.01],
            position=[0.0, 0.0, ground_height],
            quat_xyzw=[0.0, 0.0, 0.0, 0.0],
        )

    def display_poses(self, poses: list[Pose], frame_id: str = "ur_base_link"):
        """
        Utility function to display target poses in RViz (if you have a service listening).
        """
        client = self.create_client(VisualizePoses, "/visualize_poses")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /visualize_poses service...")
        req = VisualizePoses.Request(poses=poses, frame_id=frame_id)
        client.call(req)

    def move_to_home(self):
        """
        Plan and move to home configuration.
        """
        home_position = [0.0, -pi / 2, pi / 2, 0.0, 0.0, 0.0]
        self.get_logger().info("Planning to home position...")

        # Plan
        traj = self.moveit2.move_to_configuration(home_position)
        if traj is None:
            self.get_logger().error("Failed to plan to home position")
            return

        # Execute
        self.get_logger().info("Executing home trajectory...")
        self.moveit2.execute(traj)
        success = self.moveit2.wait_until_executed()
        if not success:
            self.get_logger().error("Failed to execute home trajectory")
        else:
            self.get_logger().info("Moved to home position")

    def move_to_target_pose(self, target_pose: Pose):
        """
        Plan and move to a target Pose.
        """
        # Display the target in RViz if you like
        self.display_poses([target_pose])

        # Plan
        self.get_logger().info(f"Planning to target: {target_pose}")
        position = [target_pose.position.x, target_pose.position.y, target_pose.position.z]
        quat_xyzw = [
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w,
        ]

        traj = self.moveit2.plan(
            position=position,
            quat_xyzw=quat_xyzw,
            cartesian=True,
            cartesian_fraction_threshold=0.0,
        )

        if traj is None:
            self.get_logger().error("Failed to plan to target pose")
            return

        # Execute
        self.get_logger().info("Executing plan to target pose...")
        self.moveit2.execute(traj)
        success = self.moveit2.wait_until_executed()
        if not success:
            self.get_logger().error("Failed to execute trajectory")
        else:
            self.get_logger().info("Successfully reached target pose")


def main():
    rclpy.init()

    # Create your main node
    ur_controller = URController()

    # Use a multi-threaded executor to spin the node
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(ur_controller)

    # Start spinning in a background thread so that the node is responsive
    spin_thread = Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Sleep for a moment to give time for init
    ur_controller.create_rate(1.0).sleep()

    # Now do your routine:
    # 1. Move to home
    ur_controller.move_to_home()

    # 2. Define a target pose
    #    Here we define the same pose you used in the notebook.
    target_pose = Pose()
    target_pose.position.x = 0.5
    target_pose.position.y = 0.0
    target_pose.position.z = -0.1

    # Convert Euler angles to quaternion (roll=0, pitch=90°, yaw=90°)
    roll = 0.0
    pitch = math.radians(90)
    yaw = math.radians(90)
    r = R.from_euler('xyz', [roll, pitch, yaw])
    q = r.as_quat()  # [x, y, z, w]
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]

    # 3. Move to the target pose
    ur_controller.move_to_target_pose(target_pose)

    # Cleanup
    ur_controller.get_logger().info("Shutting down...")
    executor.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
