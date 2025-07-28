import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ur_msgs.msg import IOStates, ToolDataMsg

from scipy.spatial.transform import Rotation as R
from robotic_arm_planner.planner_lib.closed_form_algorithm import closed_form_algorithm

class SensorsOrientation(Node):

    def __init__(self):
        super().__init__('sensors_distance_orientation')

        # Node Variables
        self.end_effector_pose = None
        self.ideal_distance = 15.0  # cm
        self.toggle = 1
        # 3 sensors: A left, B right, C top 
        # Positions on the plate 
        # Define the positions of the sensors
        xA, yA = -15, -15 # Position of sensor A 
        xB, yB = 15, -15 # Position of sensor B 
        xC, yC = 0.0, 15 # Position of sensor C 
        self.pA= np.array([xA, yA, 0.0])
        self.pB= np.array([xB, yB, 0.0])
        self.pC= np.array([xC, yC, 0.0])
        self.dA = 0
        self.dB = 0
        self.dC = 0

        self.last_x_ee = None

        self.publisher_ = self.create_publisher(String, 'topic', 10)
        # self.publisher_goal_pose = self.create_publisher(Pose, '/goal_pose', 10)
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/planned_trajectory', 10)

        self.subscriptor_ = self.create_subscription(Pose, '/end_effector_pose', self.end_effector_pose_callback, 10)
        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        self.create_subscription(Float32MultiArray, "/distance_sensors", self.listener_distance_callback, 10)

        # self.timer = self.create_timer(5.0, self.timer_callback)

    def end_effector_pose_callback(self, msg):
        self.end_effector_pose = msg

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def listener_distance_callback(self, msg):
        if len(msg.data) == 6:
            ultra1, ultra2, ultra3 = msg.data[0:3]
            s1, s2, s3 = msg.data[3:6]
            # self.dA = s2*100
            # self.dB = s3*100
            # self.dC = s1*100
            self.dA = ultra3*100
            self.dB = ultra1*100
            self.dC = ultra2*100
            # self.dB = self.dA
            # self.dC = self.dA

            self.get_logger().info(
                f"Ultrasound: [{ultra1:.2f}, {ultra2:.2f}, {ultra3:.2f}] m | "
                f"VL6180X: [{s1:.3f}, {s2:.3f}, {s3:.3f}] m"
            )

            # normal vector to plate
            nV = np.array([0.0, 0.0, 1.0])

            # Wall intersection points
            wA = self.pA + self.dA * nV
            wB = self.pB + self.dB * nV
            wC = self.pC + self.dC * nV
            self.get_logger().info(f"Projected points: {wA}, {wB}, {wC}")


            # Plane vectors
            v1= wB - wA
            v2= wC - wA

            #Wall normal
            nW0= np.cross(v1,v2)
            self.get_logger().info(f"Wall normal vector: {nW0}")
            nw= nW0/np.linalg.norm(nW0)

            # Distance from center (0,0,0) to wall plane
            distance1 = abs(np.dot(nw, wA))
            distance2 = abs(np.dot(nw, wB))
            distance3 = abs(np.dot(nw, wC))
            distance = np.mean([distance1, distance2, distance3])
            self.get_logger().info(f"Distance to wall from center: {distance:.2f} cm")

            # Pitch and yaw error angles 
            yaw = -np.arctan2(nw[1],nw[2])
            pitch = np.arctan2(nw[0],nw[2])
            yaw_deg = np.rad2deg(yaw)
            pitch_deg = np.rad2deg(pitch)

            self.get_logger().info(f"Pitch: {pitch_deg:.2f} degrees")
            self.get_logger().info(f"Yaw: {yaw_deg:.2f} degrees")

            ### ORIGINAL

            # rot = self.end_effector_pose.orientation
            # euler = R.from_quat([rot.x, rot.y, rot.z, rot.w]).as_euler('xyz')
            # self.get_logger().info(f"Current Orientation (rpy): roll={euler[0]:.2f}, pitch={euler[1]:.2f}, yaw={euler[2]:.2f}")
            # goal_orn = [euler[0], euler[1] + pitch, euler[2] + yaw]
            # self.get_logger().info(f"Demanded Orientation (rpy): roll={goal_orn[0]:.2f}, pitch={goal_orn[1]:.2f}, yaw={goal_orn[2]:.2f}")
            # goal_orn = R.from_euler('xyz', goal_orn, degrees=False)
            # q = goal_orn.as_quat()

            # # Convert orientation quaternion to rotation matrix
            # rot = self.end_effector_pose.orientation
            # curr_orn_rot = R.from_quat([rot.x, rot.y, rot.z, rot.w])
            # curr_orn = R.from_quat([rot.x, rot.y, rot.z, rot.w]).as_euler('ZYX')    # ZYX: Roll-Pitch-Yaw intrinsic rotation
            # self.get_logger().info(f"Current Orientation (rpy): roll={curr_orn[0]:.2f}, pitch={curr_orn[1]:.2f}, yaw={curr_orn[2]:.2f}")
            # q = curr_orn_rot.apply([0, pitch, yaw])
            # goal_orn = [curr_orn[0] + q[0], curr_orn[1] + q[1], curr_orn[2] + q[2]]
            # self.get_logger().info(f"Demanded Orientation (rpy): roll={goal_orn[0]:.2f}, pitch={goal_orn[1]:.2f}, yaw={goal_orn[2]:.2f}")
            # goal_orn = R.from_euler('ZYX', goal_orn, degrees=False)
            # self.get_logger().info(f"Goal orientation matrix: {goal_orn.as_matrix()}")
            # q = goal_orn.as_quat()

            ### OPTION 1

            # Current rotation of the EE w.r.t. base frame
            rot = self.end_effector_pose.orientation
            curr_orn_rot = R.from_quat([rot.x, rot.y, rot.z, rot.w])
            curr_orn = R.from_quat([rot.x, rot.y, rot.z, rot.w]).as_euler('ZYX', degrees=True)
            self.get_logger().info(f"Current Orientation (rpy w.r.t. base frame): Roll={curr_orn[0]:.2f}, Pitch={curr_orn[1]:.2f}, Yaw={curr_orn[2]:.2f}")

            # 2. Incremental rotation in EE frame Rotación (order ZYX = Intrinsic roll-pitch-yaw)
            increment_rot_ee = R.from_euler('ZYX', [0, pitch, yaw], degrees=False)

            # 3. Rotation Composition: R_goal = R_base * R_increment(EE_frame)
            goal_rot = curr_orn_rot * increment_rot_ee
            # goal_rot_euler = goal_rot.as_euler('ZYX', degrees=True)
            # self.get_logger().info(f"Demanded Orientation (rpy w.r.t. base frame): Roll={goal_rot_euler[0]:.2f}, Pitch={goal_rot_euler[1]:.2f}, Yaw={goal_rot_euler[2]:.2f}")

            ## Roll Compensation
            # Paso 1: Obtaining Z_EE in base frame
            z_ee = goal_rot.apply([0, 0, 1])
            # Paso 2: Obtaining ideal Y axis (-Z on base frame)
            y_desired = np.array([0.0, 0.0, 1.0])  # vertical vector in the sensors setup
            # Paso 3: Project y_desired on the sensors plane
            y_proj = y_desired - np.dot(y_desired, z_ee) * z_ee
            norm = np.linalg.norm(y_proj)
            if norm < 1e-6:
                self.get_logger().warn("y_proj is nearly zero: y_desired is aligned with Z_EE.")
            else:
                y_proj /= norm
                y_proj /= np.linalg.norm(y_proj)
                # Paso 4: Reconstruct ortonormal base (X = Y × Z)
                x_ee = np.cross(y_proj, z_ee)
                x_ee /= np.linalg.norm(x_ee)
                y_ee = np.cross(z_ee, x_ee)
                # Paso 5: New rotation with fixed Z_EE and corrected Y_EE
                R_corrected = np.column_stack((x_ee, y_ee, z_ee))
                goal_rot = R.from_matrix(R_corrected)

            # roll_comp = R.from_euler('ZYX', [0, 0, 0], degrees=True)

            # goal_rot = R_correct * roll_comp

            # ## Roll Compensation 2
            # # Dirección del eje vertical de la plancha en el frame base (montado a 45° entre X_EE e Y_EE)
            # v_plancha_local = np.array([-1, 1, 0]) / np.sqrt(2)
            # v_plancha_base = curr_orn_rot.apply(v_plancha_local)
            # z_ee = goal_rot.apply([0, 0, 1])

            # # Proyectar la dirección deseada sobre el plano ortogonal a Z_EE
            # v_proj = v_plancha_base - np.dot(v_plancha_base, z_ee) * z_ee
            # norm = np.linalg.norm(v_proj)
            # if np.dot(np.cross(v_proj, z_ee), [1, 0, 0]) < 0:
            #     v_proj = -v_proj  # invierte para asegurar dirección coherente
            # if norm < 1e-6:
            #     self.get_logger().warn("Proyección nula: y_desired está alineado con z_ee")
            # else:
            #     v_proj /= norm
            #     # Base ortonormal con nuevo eje de referencia (la plancha)
            #     x_ee = np.cross(v_proj, z_ee)
            #     x_ee /= np.linalg.norm(x_ee)
            #     y_ee = np.cross(z_ee, x_ee)

            #     if self.last_x_ee is not None:
            #         if np.dot(x_ee, self.last_x_ee) < 0:
            #             x_ee = -x_ee
            #             y_ee = -y_ee
            #     self.last_x_ee = x_ee.copy()

            #     R_corrected = np.column_stack((x_ee, y_ee, z_ee))
            #     goal_rot = R.from_matrix(R_corrected)

            # 4. Final quaternion
            q = goal_rot.as_quat()
            final_euler = goal_rot.as_euler('ZYX', degrees=True)
            self.get_logger().info(f"Final Corrected Orientation (rpy w.r.t. base frame): Roll={final_euler[0]:.2f}, Pitch={final_euler[1]:.2f}, Yaw={final_euler[2]:.2f}")

            ### OPTION 2

            # # 1. Rotación actual del EE en el marco base
            # rot = self.end_effector_pose.orientation
            # R_curr = R.from_quat([rot.x, rot.y, rot.z, rot.w])

            # # 2. Ejes locales del EE expresados en el marco base
            # # Estos serán los ejes alrededor de los cuales quieres rotar en el mundo
            # y_axis_base = R_curr.apply([0, 1, 0])  # Eje Y del EE en el marco base
            # z_axis_base = R_curr.apply([0, 0, 1])  # Eje Z del EE en el marco base

            # # 3. Rotaciones sobre esos ejes (en el marco base)
            # R_pitch = R.from_rotvec(pitch * y_axis_base)
            # R_yaw   = R.from_rotvec(yaw * z_axis_base)

            # # 4. Composición en marco base: primero pitch, luego yaw
            # # R_goal = R_yaw * R_pitch * R_curr
            # R_goal = R_curr * R_pitch * R_yaw

            # # 5. Obtener el nuevo quaternion
            # q = R_goal.as_quat()

            ###

            # Current end effector position
            pos = self.end_effector_pose.position
            p_current = np.array([pos.x, pos.y, pos.z])

            # Compute corrected position
            nw_global = curr_orn_rot.apply(nw)
            self.get_logger().warn(f"Toggle Value: {self.toggle}")
            p_new = p_current + (self.toggle)*(distance - self.ideal_distance) * nw_global / 100     # in meters!!
            # self.toggle *= -1

            # Rotation matrix and transform matrix
            T = np.eye(4)
            T[:3, :3] = R.from_quat(q).as_matrix()
            T[:3, 3] = p_new

            q_current = np.array([self.current_joint_state.position[-1], self.current_joint_state.position[0], self.current_joint_state.position[1], self.current_joint_state.position[2], self.current_joint_state.position[3], self.current_joint_state.position[4]])
            joint_values = closed_form_algorithm(T, q_current, type=0)
            if np.any(np.isnan(joint_values)):
                self.get_logger().error("IK solution contains NaN. Aborting.")
                return
            # if {
            #     (joint_values[0] < -1.57 or joint_values[0] > 1.57) or 
            #     (joint_values[1] < -1.57 or joint_values[1] > 1.57) or 
            #     (joint_values[2] < -1.57 or joint_values[2] > 1.57) or 
            #     (joint_values[3] < -1.57 or joint_values[3] > 1.57) or 
            #     (joint_values[4] < -1.57 or joint_values[4] > 1.57) or 
            #     (joint_values[5] < -1.57 or joint_values[5] > 1.57)
            #     }:
            #     self.get_logger().error("Selected solution with possible collision. Joint values outside of safety margins. Aborting.")
            #     return
            # joint_values[5] = 0.0   # NEEDS TO BE MODIFIED IN CASE ANOTHER INITIAL SENSORS POSITION IS USED!!!
            
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
            # goal_pose.positions[5] += -0.7854       # commented for simulation, uncommented for real robot
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
        else:
            self.get_logger().warn("Message recevied was incomplete.")

        if not self.end_effector_pose:
            self.get_logger().info("No end effector pose retrieved yet")
            return

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
