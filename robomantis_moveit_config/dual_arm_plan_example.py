#!/usr/bin/env python3
"""
RoboMantis 双臂协同规划执行示例
================================
左臂 = 原 geni_craner  (RS06) → left_joint_trajectory_controller
右臂 = 原 geni_craner1 (RS01) → right_joint_trajectory_controller

演示两臂独立规划、同时执行。实际项目中用 MoveIt 规划出轨迹后替换目标点。
运行：ros2 run robomantis_moveit_config dual_arm_plan_example
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time


class DualArmController(Node):
    def __init__(self):
        super().__init__("robomantis_dual_arm_controller")

        self.left_client = ActionClient(
            self, FollowJointTrajectory,
            "/left_joint_trajectory_controller/follow_joint_trajectory"
        )
        self.right_client = ActionClient(
            self, FollowJointTrajectory,
            "/right_joint_trajectory_controller/follow_joint_trajectory"
        )

        self.get_logger().info("等待左臂(geni_craner/RS06)控制器...")
        self.left_client.wait_for_server()
        self.get_logger().info("等待右臂(geni_craner1/RS01)控制器...")
        self.right_client.wait_for_server()
        self.get_logger().info("双臂控制器就绪")

    def send_trajectory(self, client, joint_names, positions, duration=5.0):
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        traj.points = [point]
        goal.trajectory = traj
        return client.send_goal_async(goal)

    def move_both_simultaneously(self, left_pos, right_pos, duration=5.0):
        """两臂同时执行各自目标关节角"""
        left_joints = [f"left_joint{i}" for i in range(1, 8)]
        right_joints = [f"right_joint{i}" for i in range(1, 8)]

        self.get_logger().info(f"左臂(geni_craner)目标: {left_pos}")
        self.get_logger().info(f"右臂(geni_craner1)目标: {right_pos}")

        future_left = self.send_trajectory(self.left_client, left_joints, left_pos, duration)
        future_right = self.send_trajectory(self.right_client, right_joints, right_pos, duration)

        rclpy.spin_until_future_complete(self, future_left)
        rclpy.spin_until_future_complete(self, future_right)

        left_handle = future_left.result()
        right_handle = future_right.result()

        if left_handle and left_handle.accepted:
            self.get_logger().info("左臂轨迹已接受")
            rclpy.spin_until_future_complete(self, left_handle.get_result_async())
        if right_handle and right_handle.accepted:
            self.get_logger().info("右臂轨迹已接受")
            rclpy.spin_until_future_complete(self, right_handle.get_result_async())

        self.get_logger().info("双臂运动完成")


def main():
    rclpy.init()
    arm = DualArmController()
    try:
        # 示例姿态（rad）
        left_target = [0.0, -0.5, 0.0, 0.8, 0.0, 0.5, 0.0]
        right_target = [0.0, -0.5, 0.0, 0.8, 0.0, 0.5, 0.0]

        arm.get_logger().info("=== 双臂同时执行 ===")
        arm.move_both_simultaneously(left_target, right_target, duration=6.0)

        time.sleep(2)
        arm.get_logger().info("回到零位...")
        arm.move_both_simultaneously([0.0]*7, [0.0]*7, duration=4.0)
    except KeyboardInterrupt:
        pass
    finally:
        arm.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
