########################################################

#### Node using an action client to send goals

########################################################

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from copy import deepcopy
from std_srvs.srv import Trigger

class PublisherJointTrajectoryActionClient(Node):

    def __init__(self):
        super().__init__("publisher_joint_trajectory_action_client")

        self.declare_parameter("controller_name", "scaled_joint_trajectory_controller")
        self.declare_parameter("joints", ["shoulder_pan_joint", "shoulder_lift_joint",
                                          "elbow_joint", "wrist_1_joint",
                                          "wrist_2_joint", "wrist_3_joint"])
        self.declare_parameter("check_starting_point", False)

        controller_name = self.get_parameter("controller_name").value
        self.joints = self.get_parameter("joints").value
        self.check_starting_point = self.get_parameter("check_starting_point").value

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is required')
        
        self.starting_point = {}
        if self.check_starting_point:
            for name in self.joints:
                param = "starting_point_limits." + name
                self.declare_parameter(param, [-2 * 3.14159, 2 * 3.14159])
                self.starting_point[name] = self.get_parameter(param).value

            for name in self.joints:
                if len(self.starting_point[name]) != 2:
                    raise Exception('"starting_point" parameter is not set correctly!')
                
        self.joint_state_sub = self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        self.trajectory_sub = self.create_subscription(JointTrajectory, "/planned_trajectory", self.trajectory_callback, 10)
        self.status_pub = self.create_publisher(Bool, "/execution_status", 10)
        self.emergency_sub = self.create_subscription(Bool, "/emergency_stop", self.emergency_callback, 10)

        self.emergency_srv = self.create_service(Trigger, "/emergency_stop", self.handle_emergency_service)

        action_topic = f"{controller_name}/follow_joint_trajectory"
        self._action_client = ActionClient(self, FollowJointTrajectory, action_topic)

        self.current_joint_state = None
        self.starting_point_ok = not self.check_starting_point
        self.planned_trajectory = None
        self.trajectory_received = False
        self.execution_complete = True
        self.current_goal_handle = None
        self.prev_status = True

        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server already available.")
        self.timer = self.create_timer(0.1, self.timer_callback)

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

        if self.check_starting_point:
            limit_exceeded = False
            for idx, name in enumerate(msg.name):
                if name in self.starting_point:
                    pos = msg.position[idx]
                    low, high = self.starting_point[name]
                    if not (low <= pos <= high):
                        self.get_logger().warn(f"Joint {name} position {pos:.3f} out of limits {low:.3f}, {high:.3f}")
                        limit_exceeded = True
            self.starting_point_ok = not limit_exceeded
            self.check_starting_point = False       # to just check once at the start

    def trajectory_callback(self, msg):
        if not self.starting_point_ok:
            self.get_logger().warn("Received trajectory but robot not in valid starting configuration.")
            return
        self.planned_trajectory = msg
        self.trajectory_received = True
        self.get_logger().info("Trajectory received and stored.")

    def emergency_callback(self, msg):
        if msg.data and self.current_goal_handle:
            self.get_logger().warn("Emergency stop received! Cancelling active trajectory...")
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda f: self.get_logger().info("Goal cancel requested."))
            self.execution_complete = True

    def handle_emergency_service(self, request, response):
        if self.current_goal_handle:
            self.get_logger().warn("Emergency stop requested via service! Cancelling trajectory...")
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda f: self.get_logger().info("Goal cancel requested."))
            self.execution_complete = True
            response.success = True
            response.message = "Trajectory cancelled successfully."
        else:
            response.success = False
            response.message = "No active trajectory to cancel."
        return response

    def timer_callback(self):
        status_msg = Bool()
        status_msg.data = self.execution_complete
        if self.prev_status != self.execution_complete:
            self.status_pub.publish(status_msg)
            self.prev_status = self.execution_complete

        if self.trajectory_received and self.starting_point_ok:
            self.send_trajectory_goal()
            self.trajectory_received = False
            self.execution_complete = False

    def send_trajectory_goal(self):
        goal_msg = FollowJointTrajectory.Goal()

        trajectory = deepcopy(self.planned_trajectory)
        duration_between_points = 1.0

        for i, point in enumerate(trajectory.points):
            total_sec = duration_between_points * (i + 1)
            secs = int(total_sec)
            nsecs = int((total_sec - secs) * 1e9)
            point.time_from_start = Duration(sec=secs, nanosec=nsecs)
            # if not point.velocities or len(point.velocities) != len(point.positions):
            #     point.velocities = [0.0] * len(point.positions)
            if i > 0:
                prev = trajectory.points[i - 1]
                dt = duration_between_points
                point.velocities = [
                    (p2 - p1) / dt for p1, p2 in zip(prev.positions, point.positions)
                ]
            else:
                point.velocities = [0.0] * len(point.positions)

        goal_msg.trajectory = trajectory
        goal_msg.goal_time_tolerance = Duration(sec=0, nanosec=200_000_000)
        goal_msg.goal_tolerance = [
            JointTolerance(position=0.01, velocity=0.01, name=name) for name in self.joints
        ]

        self.get_logger().info("Sending trajectory goal with added times and velocities...")
        self._action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            # raise RuntimeError("Goal rejected :(")
            self.execution_complete = True
            return

        self.get_logger().info("Goal accepted. Waiting for result...")
        self.current_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f"Done with result: {self.status_to_str(status)}")
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Trajectory execution succeeded.")
        else:
            if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                self.get_logger().error(f"Done with result: {self.error_code_to_str(result.error_code)}")
            # raise RuntimeError("Executing trajectory failed. " + result.error_string)   # To avoid node shutdown can be commented out
            self.get_logger().error(f"Executing trajectory failed. {result.error_string}")
        self.execution_complete = True

    @staticmethod
    def error_code_to_str(error_code):
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            return "SUCCESSFUL"
        if error_code == FollowJointTrajectory.Result.INVALID_GOAL:
            return "INVALID_GOAL"
        if error_code == FollowJointTrajectory.Result.INVALID_JOINTS:
            return "INVALID_JOINTS"
        if error_code == FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP:
            return "OLD_HEADER_TIMESTAMP"
        if error_code == FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED:
            return "PATH_TOLERANCE_VIOLATED"
        if error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
            return "GOAL_TOLERANCE_VIOLATED"

    @staticmethod
    def status_to_str(error_code):
        if error_code == GoalStatus.STATUS_UNKNOWN:
            return "UNKNOWN"
        if error_code == GoalStatus.STATUS_ACCEPTED:
            return "ACCEPTED"
        if error_code == GoalStatus.STATUS_EXECUTING:
            return "EXECUTING"
        if error_code == GoalStatus.STATUS_CANCELING:
            return "CANCELING"
        if error_code == GoalStatus.STATUS_SUCCEEDED:
            return "SUCCEEDED"
        if error_code == GoalStatus.STATUS_CANCELED:
            return "CANCELED"
        if error_code == GoalStatus.STATUS_ABORTED:
            return "ABORTED"

def main(args=None):
    rclpy.init(args=args)
    node = PublisherJointTrajectoryActionClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as err:
        node.get_logger().error(str(err))
    except SystemExit:
        rclpy.logging.get_logger("jtc_client").info("Done")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()



# ########################################################

# #### Node using a topic publisher to send goals 

# ########################################################

# # Copyright 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
# #
# # Licensed under the Apache License, Version 2.0 (the "License");
# # you may not use this file except in compliance with the License.
# # You may obtain a copy of the License at
# #
# #     http://www.apache.org/licenses/LICENSE-2.0
# #
# # Unless required by applicable law or agreed to in writing, software
# # distributed under the License is distributed on an "AS IS" BASIS,
# # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # See the License for the specific language governing permissions and
# # limitations under the License.
# #
# # Authors: Denis Štogl, Lovro Ivanov
# #

# import rclpy
# from rclpy.node import Node
# from builtin_interfaces.msg import Duration
# from rcl_interfaces.msg import ParameterDescriptor

# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Bool



# class PublisherJointTrajectory(Node):
#     ## Subscripcion a un topic que de la posicion actual del robot


#     def __init__(self):
#         super().__init__("publisher_joint_trajectory_planned")
#         # Declare all parameters
#         self.declare_parameter("controller_name", "position_trajectory_controller")
#         self.declare_parameter("wait_sec_between_publish", 1)
#         self.declare_parameter("joints", [""])
#         self.declare_parameter("check_starting_point", True)

#         # Read parameters
#         controller_name = self.get_parameter("controller_name").value
#         wait_sec_between_publish = self.get_parameter("wait_sec_between_publish").value
#         self.joints = self.get_parameter("joints").value
#         self.check_starting_point = self.get_parameter("check_starting_point").value
#         self.starting_point = {}

#         if self.joints is None or len(self.joints) == 0:
#             raise Exception('"joints" parameter is not set!')

#         # starting point stuff
#         if self.check_starting_point:
#             # declare nested params
#             for name in self.joints:
#                 param_name_tmp = "starting_point_limits" + "." + name
#                 self.declare_parameter(param_name_tmp, [-2 * 3.14159, 2 * 3.14159])
#                 self.starting_point[name] = self.get_parameter(param_name_tmp).value

#             for name in self.joints:
#                 if len(self.starting_point[name]) != 2:
#                     raise Exception('"starting_point" parameter is not set correctly!')
#             self.joint_state_sub = self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
#             self.create_subscription(JointTrajectory, "/planned_trajectory", self.joint_path_callback, 10)
#             self.create_subscription(Bool, "/emergency_stop", self.emergency_stop_callback, 10)

#         # initialize starting point status
#         self.starting_point_ok = not self.check_starting_point

#         self.joint_state_msg_received = False
#         self.trajectory_received = False
#         self.planned_trajectory = None
#         self.stop_trajectory = False 
#         self.execution_complete = True
#         self.current_joint_state = None
#         self.joint_names = [
#             'shoulder_pan_joint',
#             'shoulder_lift_joint',
#             'elbow_joint',
#             'wrist_1_joint',
#             'wrist_2_joint',
#             'wrist_3_joint'
#         ]

#         publish_topic = "/" + controller_name + "/" + "joint_trajectory"

#         self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)
#         self.execution_status_pub = self.create_publisher(Bool, "/execution_status", 10)
#         self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)

#     def emergency_stop_callback(self, msg):
#         """Función que activa la parada de emergencia"""
#         self.emergency_stop = msg
#         self.stop_trajectory = True
#         self.execution_complete = False
#         self.get_logger().warn("Emergency stop triggered! Stopping the robot.")
#         # Aquí puedes mandar comandos para detener la trayectoria
#         self.stop_joint_trajectory()        

#     def stop_joint_trajectory(self):
#         """Comando para detener el robot"""
#         # Publica un mensaje vacío o comando de parada al controlador de la trayectoria
#         stop_msg = JointTrajectory()
#         stop_msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
#         stop_msg.points = []  # Vacío o "detener movimiento"
#         self.publisher_.publish(stop_msg)
#         self.get_logger().info("Sent stop command to the robot.")
#         self.execution_complete = True

#     def timer_callback(self):
#         self.monitor_execution()

#         if self.stop_trajectory:
#             self.get_logger().warn("Trajectory stop triggered, halting movement.")
#             return
        
#         if not self.starting_point_ok:
#             self.get_logger().warn("Start configuration is not within configured limits!")
#             return
        
#         execution_msg = Bool()
#         execution_msg.data = self.execution_complete
#         self.execution_status_pub.publish(execution_msg)       

#         if self.trajectory_received and self.planned_trajectory:
#             self.get_logger().info("Publishing received trajectory to controller...")
#             self.publisher_.publish(self.planned_trajectory)
#             self.trajectory_received = False  # evitar repetir envío
#             self.execution_complete = False  # Marca la flag como falsa cuando una nueva trayectoria empieza
#         else:
#             self.get_logger().info("No planned trajectory received yet.")

#     def joint_state_callback(self, msg):
#         self.current_joint_state = msg

#         if not self.joint_state_msg_received:   # Si se elimina esta linea, se evalua en cada punto

#             # check start state
#             limit_exceeded = [False] * len(msg.name)
#             for idx, enum in enumerate(msg.name):
#                 if (msg.position[idx] < self.starting_point[enum][0]) or (
#                     msg.position[idx] > self.starting_point[enum][1]
#                 ):
#                     # self.get_logger().warn(f"Starting point limits exceeded for joint {enum} !")
#                     self.get_logger().warn(f"Starting point limits exceeded for joint {enum} !")
#                     self.get_logger().warn(f"Value stp0: {self.starting_point[enum][0]}, Value stp0: {self.starting_point[enum][1]}, Value pos: {msg.position[idx]}")
#                     limit_exceeded[idx] = True

#             if any(limit_exceeded):
#                 self.starting_point_ok = False
#             else:
#                 self.starting_point_ok = True

#             self.joint_state_msg_received = True
#         else:
#             return
        
#     def joint_path_callback(self, msg):
#         if not self.starting_point_ok:
#             self.get_logger().warn("Received trajectory, but starting point is not valid!")
#             return

#         self.planned_trajectory = msg
#         self.trajectory_received = True
#         # self.execution_complete = False
#         self.get_logger().info("Planned trajectory received and stored.")
#         return
    
#     def monitor_execution(self):
#         """Verifica si el robot ha alcanzado la posición final"""
#         # Simula una verificación de que el robot ha alcanzado su meta.
#         # Este es un lugar donde puedes agregar un mecanismo real como un ActionServer
#         # o simplemente verificar la posición en el JointState, por ejemplo:

#         self.get_logger().info("Monitoring execution.")
        
#         if self.current_joint_state:  # Solo lo haces si el JointState está disponible
#             self.get_logger().info("Current joint state received.")
#             # self.get_logger().info(f"Current joint state: {self.current_joint_state}")
#             goal_reached = True
#             if self.planned_trajectory is not None:
#                 # self.get_logger().info(f"Planned trajectory: {self.planned_trajectory}")
#                 # self.get_logger().info(f"Current joint state position 5: {self.current_joint_state.position[3:5]}")
#                 current_joint_values = [self.current_joint_state.position[-1], self.current_joint_state.position[0], self.current_joint_state.position[1], self.current_joint_state.position[2], self.current_joint_state.position[3], self.current_joint_state.position[4]]
#                 # self.get_logger().info(f"Current joint values: {current_joint_values}")
#                 for idx, goal_position in enumerate(self.planned_trajectory.points[-1].positions):
#                     # self.get_logger().info(f"Goal positons: {goal_position}")
#                     if abs(current_joint_values[idx] - goal_position) > 0.01:  # Tolerancia
#                         goal_reached = False
#                         break
            
#             if goal_reached:
#                 self.execution_complete = True
#                 self.get_logger().info("Goal reached! Execution complete.")
#             else:
#                 self.get_logger().info("Robot has not yet reached goal. Continuing monitoring.")

# def main(args=None):
#     rclpy.init(args=args)

#     publisher_joint_trajectory = PublisherJointTrajectory()

#     try:
#         rclpy.spin(publisher_joint_trajectory)
#     except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
#         print("Keyboard interrupt received. Shutting down node.")
#     except Exception as e:
#         print(f"Unhandled exception: {e}")


# if __name__ == "__main__":
#     main()



########################################################

#### Node using an action client to send goals (basic original structure)

########################################################

# import rclpy
# from rclpy.node import Node
# from control_msgs.action import FollowJointTrajectory
# from rclpy.action import ActionClient
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from std_msgs.msg import Bool
# from geometry_msgs.msg import Pose
# import time

# class ActionClientNode(Node):
#     def __init__(self):
#         super().__init__('publisher_joint_trajectory_planned')

#         # Crear Action Client
#         self._action_client = ActionClient(self, FollowJointTrajectory, 'follow_joint_trajectory')

#         # Crear el publisher para la ejecución de la trayectoria
#         self.execution_status_pub = self.create_publisher(Bool, '/execution_status', 10)

#         self.get_logger().info("Action client node initialized.")

#     def goal_callback(self, msg: Pose):
#         """Callback para recibir goals desde el topic y enviarlos al Action Server"""
#         self.send_goal(msg)

#     def send_goal(self, goal_pose: Pose):
#         """Enviar una meta al Action Server"""
#         goal_msg = FollowJointTrajectory.Goal()

#         joint_trajectory = JointTrajectory()
#         joint_trajectory.joint_names = [
#             'shoulder_pan_joint',
#             'shoulder_lift_joint',
#             'elbow_joint',
#             'wrist_1_joint',
#             'wrist_2_joint',
#             'wrist_3_joint'
#         ]

#         target_position = [goal_pose.position.x, goal_pose.position.y, goal_pose.position.z]
#         target_orientation = [goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w]

#         point = JointTrajectoryPoint()
#         point.positions = target_position + target_orientation
#         point.time_from_start = rclpy.duration.Duration(seconds=5).to_msg()
#         joint_trajectory.points.append(point)

#         goal_msg.trajectory = joint_trajectory

#         self._action_client.wait_for_server()
#         self.get_logger().info('Sending goal...')
#         self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

#     def feedback_callback(self, feedback_msg):
#         self.get_logger().info(f"Received feedback: {feedback_msg.feedback}")

#     def result_callback(self, result):
#         self.get_logger().info(f"Execution result: {result}")

#     def execute(self, goal_pose: Pose):
#         self.send_goal(goal_pose)

#         # Publicar el estado de la ejecución
#         execution_msg = Bool()
#         execution_msg.data = True  # Se puede ajustar si se desea mantener el estado
#         self.execution_status_pub.publish(execution_msg)

# def main(args=None):
#     rclpy.init(args=args)

#     action_client_node = ActionClientNode()

#     goal_pose = Pose()
#     goal_pose.position.x = 1.0
#     goal_pose.position.y = 0.5
#     goal_pose.position.z = 0.0
#     goal_pose.orientation.x = 0.0
#     goal_pose.orientation.y = 0.0
#     goal_pose.orientation.z = 0.0
#     goal_pose.orientation.w = 1.0

#     action_client_node.execute(goal_pose)

#     rclpy.spin(action_client_node)

# if __name__ == '__main__':
#     main()




