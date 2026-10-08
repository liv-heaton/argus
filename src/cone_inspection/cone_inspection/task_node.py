"""
Steps the planned trajectory out onto /joint_states, so robot_state_publisher
turns it into TF and RViz draws the arm actually moving.

Run alongside your existing launch file (robot_state_publisher + rviz2),
but WITHOUT joint_state_publisher_gui -- this node replaces it as the
source of /joint_states.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from cone_inspection.planning import load_robot, build_inspection_plan
from spatialmath import SE3

# Must match the <joint name="..."> order in ur_macro.xacro for a UR3
JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]


class TaskNode(Node):
    def __init__(self):
        super().__init__("task_node")

        self.declare_parameter("urdf_path", "/tmp/ur3_cell.urdf")
        self.declare_parameter("publish_rate_hz", 50.0)
        urdf_path = self.get_parameter("urdf_path").value
        rate = self.get_parameter("publish_rate_hz").value

        self.get_logger().info(f"Loading robot from {urdf_path}")
        robot = load_robot(urdf_path)

        # TODO: replace these placeholders with real values once your
        # gripper/grasp pose and scene_node's cone pose exist.
        q_home = [0, -1.57, 1.57, -1.57, -1.57, 0]
        cone_pos = [0.5, 0.0, 0.0]
        grasp_pose_tool0 = SE3(0.3, -0.2, 0.05) * SE3.RPY([180, 0, 0], unit="deg")

        self.get_logger().info("Building trajectory...")
        self.traj = build_inspection_plan(robot, q_home, cone_pos, grasp_pose_tool0)
        self.get_logger().info(f"Trajectory ready: {self.traj.shape[0]} samples")

        self.i = 0
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def tick(self):
        if self.i >= self.traj.shape[0]:
            self.i = 0  # loop the demo; remove this to stop after one pass

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = self.traj[self.i].tolist()
        self.pub.publish(msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = TaskNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()