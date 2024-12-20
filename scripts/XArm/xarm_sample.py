from omni.isaac.examples.base_sample import BaseSample
from .xarm_follow_target import XArmFollowTarget
from .xarm_rmpflow_controller import XArmRMPFlowController
from .xarm_socket import XArmSocket
import numpy as np
import time
import carb

import omni.kit.pipapi
omni.kit.pipapi.install("pyquaternion")
from pyquaternion import Quaternion

class XArmSample(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self._controller = None
        self._articulation_controller = None
        self._xarm_version = None

        # sending position data to arm
        self.xarm_socket = XArmSocket()
        self.stream_joints = False

        self._max_range = None
        self._min_range = None
        self._min_height = None

        self._last_face_seen_timeout = 1
        self._last_face_seen_time = 0 

        self.rand_target_enabled = True

        self._last_rand_target_timeout = 5
        self._last_rand_target_time = 0 

        self.min_detection_range = None
        self.max_detection_range = None

        self._safe_zone = [
            (0.3, -0.3, 0.3), # back bottom right 
            (0.6, 0.3, 0.625) # top front left
                           ]

    def set_xarm_version(self, xarm_version):
        self._xarm_version = xarm_version
        if self._xarm_version == 5:
            self._max_range = 0.7
            self._min_range = 0.3
            self._min_height = 0.1
        elif self._xarm_version == 7:
            self._max_range = 0.7
            self._min_range = 0.3
            self._min_height = 0.1           

    def setup_scene(self):
        world = self.get_world()
        world.add_task(XArmFollowTarget(self._xarm_version))
        return

    async def setup_pre_reset(self):
        world = self.get_world()
        if world.physics_callback_exists("sim_step"):
            world.remove_physics_callback("sim_step")
        self._controller.reset()
        return

    def world_cleanup(self):
        self._controller = None
        self.xarm_socket.shut_down_socket()
        return

    async def setup_post_load(self):
        self._xarm_task = list(self._world.get_current_tasks().values())[0]
        self._task_params = self._xarm_task.get_params()
        self._xarm = self._world.scene.get_object(self._task_params["robot_name"]["value"])
        self._controller = XArmRMPFlowController(
            name="target_follower_controller", 
            robot_articulation=self._xarm,
            xarm_version=self._xarm_version
        )
        self._articulation_controller = self._xarm.get_articulation_controller()

        self.xarm_socket.start_txsocket()
        self.xarm_socket.start_rxsocket()

        return

    async def _on_follow_target_event_async(self, val):
        world = self.get_world()
        if val:
            await world.play_async()
            world.add_physics_callback("sim_step", self._on_follow_target_simulation_step)
        else:
            world.remove_physics_callback("sim_step")
        return
    
    async def _on_stream_joints_event_async(self, val):
        if val:
            # clicked "Start" stream joints
            self.stream_joints = True
        else:
            # clicked "Stop"
            self.stream_joints = False
        return

    def _on_follow_target_simulation_step(self, step_size):
        observations = self._world.get_observations()
        actions = self._controller.forward(
            target_end_effector_position=observations[self._task_params["target_name"]["value"]]["position"],
            target_end_effector_orientation=observations[self._task_params["target_name"]["value"]]["orientation"],
        )
        
        self._articulation_controller.set_gains(
            1e15*np.ones(self._xarm_version), 
            1e14*np.ones(self._xarm_version)
        ) # Solution from Nvidia Live Session 1:23:00
        self._articulation_controller.apply_action(actions)

        if self.xarm_socket.txconn and self.stream_joints:
            try: 
                sendData = str(self._xarm.get_joint_positions().tolist())
                #print("joints:", sendData)
                res = self.xarm_socket.txconn.send(sendData.encode())
                if res == 0:
                    print("channel is closed...")
            except Exception as e:
                print(e)
                # if sending failed, recreate the socket and reconnect
                print("sending failed... closing connection")
                self.xarm_socket.txconn.close()
                self.xarm_socket.txconn = None

        # face_in_range = False
        # if self.xarm_socket.cam_to_nose and self.xarm_socket.face_direction:
        #     cam_position, cam_orientation = self._xarm.end_effector.get_world_pose()
            
        #     nose_distance_from_camera = np.linalg.norm(self.xarm_socket.cam_to_nose)
        #     carb.log_error(str(self.min_detection_range) + " " + str(self.max_detection_range) + " " + str(nose_distance_from_camera))
        #     face_in_range = nose_distance_from_camera >= self.min_detection_range and nose_distance_from_camera <= self.max_detection_range

        current_time = time.time()
        # if face_in_range:
        #     carb.log_error("here")
        #     cam_orientation = Quaternion(cam_orientation)
        #     nose_position = cam_orientation.rotate(self.xarm_socket.cam_to_nose) + cam_position
        #     nose_direction = cam_orientation.rotate(self.xarm_socket.face_direction)
        #     nose_direction /= np.linalg.norm(nose_direction)

        #     # update position of target from camera feed
        #     cube = self._world.scene.get_object("target")

        #     if nose_distance_from_camera < self.min_detection_range:
        #         newpose = nose_position + nose_direction * self.min_detection_range
        #     elif nose_distance_from_camera > self.max_detection_range:
        #         newpose = nose_position + nose_direction * self.max_detection_range
        #     else:
        #         newpose = nose_position + nose_direction * nose_distance_from_camera

        #     range = np.linalg.norm(newpose)
        #     if range < self._min_range:
        #         newpose = newpose / np.linalg.norm(newpose) * self._min_range
        #     elif range > self._max_range:
        #         newpose = newpose / np.linalg.norm(newpose) * self._max_range

        #     newpose = [newpose[0], newpose[1], max(newpose[2], self._min_height)]

        #     updated_quaternion = self._get_new_target_orientation(newpose)
            
        #     cube.set_world_pose(np.array(newpose), updated_quaternion)
        #     self.xarm_socket.dx = None
        #     self.xarm_socket.dy = None

        #     self._last_face_seen_time = current_time
        if self.xarm_socket.face_direction:
            current_time = time.time()
            cube = self._world.scene.get_object("target")
            pos, qrot = cube.get_world_pose()
            print(qrot)
            newpose = [ pos[0]+self.xarm_socket.z, pos[1]+self.xarm_socket.dx, pos[2]+self.xarm_socket.dy]

            newpose[0] = np.clip(newpose[0], self._safe_zone[0][0], self._safe_zone[1][0])
            newpose[1] = np.clip(newpose[1], self._safe_zone[0][1], self._safe_zone[1][1])
            newpose[2] = np.clip(newpose[2], self._safe_zone[0][2], self._safe_zone[1][2])
            print("pose", pos, "->", newpose, end="")
            cube.set_world_pose(np.array(newpose))
            print("set.")

            self.xarm_socket.dx = None
            self.xarm_socket.dy = None

            self._last_face_seen_time = current_time

        elif self.rand_target_enabled and ( \
                self._xarm_task.task_achieved or \
                current_time > self._last_rand_target_time + self._last_rand_target_timeout \
            ) and current_time > self._last_face_seen_time + self._last_face_seen_timeout:
                # set random location
                cube = self._world.scene.get_object("target")
                randpos = [
                    np.random.uniform(-1, 1), 
                    np.random.uniform(-1, 1),
                    np.random.uniform(0, 1)
                ]
                range = np.linalg.norm(randpos)
                if range < self._min_range:
                    randpos = randpos / np.linalg.norm(randpos) * self._min_range
                elif range > self._max_range:
                    randpos = randpos / np.linalg.norm(randpos) * self._max_range

                randpos = [randpos[0], randpos[1], max(randpos[2], self._min_height)]

                updated_quaternion = self._get_new_target_orientation(randpos)

                print("Setting new target pos:"+str(randpos))
                cube.set_world_pose(np.array(randpos), updated_quaternion)

                self._last_rand_target_time = time.time()

        self.xarm_socket.cam_to_nose=None
        self.xarm_socket.face_direction=None
        return

    def _on_add_obstacle_event(self):
        world = self.get_world()
        current_task = list(world.get_current_tasks().values())[0]
        cube = current_task.add_obstacle()
        self._controller.add_obstacle(cube)
        return

    def _on_remove_obstacle_event(self):
        world = self.get_world()
        current_task = list(world.get_current_tasks().values())[0]
        obstacle_to_delete = current_task.get_obstacle_to_delete()
        self._controller.remove_obstacle(obstacle_to_delete)
        current_task.remove_obstacle()
        return

    def _on_logging_event(self, val):
        world = self.get_world()
        data_logger = world.get_data_logger()
        if not world.get_data_logger().is_started():
            robot_name = self._task_params["robot_name"]["value"]
            target_name = self._task_params["target_name"]["value"]

            def frame_logging_func(tasks, scene):
                return {
                    "joint_positions": scene.get_object(robot_name).get_joint_positions().tolist(),
                    "applied_joint_positions": scene.get_object(robot_name)
                    .get_applied_action()
                    .joint_positions.tolist(),
                    "target_position": scene.get_object(target_name).get_world_pose()[0].tolist(),
                }

            data_logger.add_data_frame_logging_func(frame_logging_func)
        if val:
            data_logger.start()
        else:
            data_logger.pause()
        return

    def _on_save_data_event(self, log_path):
        world = self.get_world()
        data_logger = world.get_data_logger()
        data_logger.save(log_path=log_path)
        data_logger.reset()
        return
    
    def _get_new_target_orientation(self, position):
        direction_vector = np.array([0, 0, 0.3]) - position
        normalized_direction = direction_vector / np.linalg.norm(direction_vector)

        rotation_axis = np.cross(np.array([0, 0, -1]), normalized_direction)
        rotation_axis /= np.linalg.norm(rotation_axis)

        rotation_angle = np.arccos(np.dot(np.array([0, 0, -1]), normalized_direction))
        half_angle = 0.5 * rotation_angle

        sin_half_angle = np.sin(half_angle)
        cos_half_angle = np.cos(half_angle)

        quaternion = np.array([
            cos_half_angle,
            sin_half_angle * rotation_axis[0],
            sin_half_angle * rotation_axis[1],
            sin_half_angle * rotation_axis[2]
        ])
        return quaternion
