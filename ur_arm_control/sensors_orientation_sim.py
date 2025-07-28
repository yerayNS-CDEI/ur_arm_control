import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from scipy.spatial.transform import Rotation as R

from robotic_arm_planner.planner_lib.closed_form_algorithm import closed_form_algorithm

class SensorsOrientation(Node):

    def __init__(self):
        super().__init__('sensors_orientation')

        # Node Variables
        self.end_effector_pose = None
        self.min_distance = 19.0    # cm
        self.max_distance = 23.0    # cm
        self.ideal_distance = 5.0  # cm
        self.toggle = 1
        # 3 sensors: A left, B right, C top 
        # Positions on the plate 
        # Define the positions of the sensors
        xA, yA = -10.0, -1.0 # Position of sensor A 
        xB, yB = 10.0, -1.0 # Position of sensor B 
        xC, yC = 0.0, 10.0 # Position of sensor C 
        self.pA= np.array([xA, yA, 0.0])
        self.pB= np.array([xB, yB, 0.0])
        self.pC= np.array([xC, yC, 0.0])

        self.publisher_ = self.create_publisher(String, 'topic', 10)
        # self.publisher_goal_pose = self.create_publisher(Pose, '/goal_pose', 10)
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/planned_trajectory', 10)

        self.subscriptor_ = self.create_subscription(Pose, '/end_effector_pose', self.end_effector_pose_callback, 10)
        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)

        self.timer = self.create_timer(1.0, self.timer_callback)

    def end_effector_pose_callback(self, msg):
        self.end_effector_pose = msg

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def timer_callback(self):

        if not self.end_effector_pose:
            self.get_logger().info("No end effector pose retrieved yet")
            return

        # Distance readings from sensors
        dA = np.random.uniform(self.min_distance, self.max_distance)
        dB = np.random.uniform(self.min_distance, self.max_distance)
        dC = np.random.uniform(self.min_distance, self.max_distance)
        # dB = dA
        # dC = dA

        # normal vector to plate
        nV = np.array([0.0, 0.0, 1.0])

        # Wall intersection points
        wA = self.pA + dA * nV
        wB = self.pB + dB * nV
        wC = self.pC + dC * nV

        # Plane vectors
        v1= wB - wA
        v2= wC - wA

        #Wall normal
        nW0= np.cross(v1,v2)
        nw= nW0/np.linalg.norm(nW0)

        # Distance from center (0,0,0) to wall plane
        distance1 = abs(np.dot(nw, wA))
        distance2 = abs(np.dot(nw, wB))
        distance3 = abs(np.dot(nw, wC))
        distance = np.mean([distance1, distance2, distance3])
        self.get_logger().info(f"Distance to wall from center: {distance:.2f} cm")

        # Pitch and yaw error angles 
        yaw = np.arctan2(nw[0],nw[2])
        pitch = np.arctan2(nw[1],nw[2])
        yaw_deg = np.rad2deg(yaw)
        pitch_deg = np.rad2deg(pitch)

        self.get_logger().info(f"Pitch: {pitch_deg:.2f} degrees")
        self.get_logger().info(f"Yaw: {yaw_deg:.2f} degrees")

        rot = self.end_effector_pose.orientation
        euler = R.from_quat([rot.x, rot.y, rot.z, rot.w]).as_euler('xyz')
        self.get_logger().info(f"Current Orientation (rpy): roll={euler[0]:.2f}, pitch={euler[1]:.2f}, yaw={euler[2]:.2f}")
        goal_orn = [euler[0], euler[1] + pitch, euler[2] + yaw]
        self.get_logger().info(f"Demanded Orientation (rpy): roll={goal_orn[0]:.2f}, pitch={goal_orn[1]:.2f}, yaw={goal_orn[2]:.2f}")
        goal_orn = R.from_euler('xyz', goal_orn, degrees=False)
        q = goal_orn.as_quat()

        # Convert orientation quaternion to rotation matrix
        rot = self.end_effector_pose.orientation
        r = R.from_quat([rot.x, rot.y, rot.z, rot.w])
        nw_global = r.apply(nw)

        # Current end effector position
        pos = self.end_effector_pose.position
        p_current = np.array([pos.x, pos.y, pos.z])

        # Compute corrected position
        self.get_logger().warn(f"Toggle Value: {self.toggle}")
        p_new = p_current - (self.toggle)*abs(distance - self.ideal_distance) * nw_global / 100
        self.toggle *= -1

        # Rotation matrix and transform matrix
        T = np.eye(4)
        T[:3, :3] = R.from_quat(q).as_matrix()
        T[:3, 3] = p_new

        q_current = np.array([self.current_joint_state.position[-1], self.current_joint_state.position[0], self.current_joint_state.position[1], self.current_joint_state.position[2], self.current_joint_state.position[3], self.current_joint_state.position[4]])
        joint_values = closed_form_algorithm(T, q_current, type=0)
        if np.any(np.isnan(joint_values)):
            self.get_logger().error("IK solution contains NaN. Aborting.")
            return

        # Publish GoalPose
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        time_from_start = 0.5
        goal_pose = JointTrajectoryPoint()
        goal_pose.positions = joint_values.tolist()
        goal_pose.time_from_start.sec = int(time_from_start)
        goal_pose.time_from_start.nanosec = int((time_from_start % 1.0) * 1e9)
        traj_msg.points.append(goal_pose)
        self.trajectory_pub.publish(traj_msg)

        # goal_pose = Pose()
        # goal_pose.position.x = p_new[0]
        # goal_pose.position.y = p_new[1]
        # goal_pose.position.z = p_new[2]
        # goal_pose.orientation.x = q[0]
        # goal_pose.orientation.y = q[1]
        # goal_pose.orientation.z = q[2]
        # goal_pose.orientation.w = q[3]
        # self.publisher_goal_pose.publish(goal_pose)

def main(args=None):
    rclpy.init(args=args)

    sensors_orientation = SensorsOrientation()

    rclpy.spin(sensors_orientation)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sensors_orientation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()