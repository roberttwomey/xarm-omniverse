import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.examples")

from omni.isaac.core import World
from XArm.xarm_follow_target import XArmFollowTarget
from XArm.xarm_rmpflow_controller import XArmRMPFlowController
from XArm.xarm_socket import XArmSocket
import numpy as np
import time


def get_new_target_orientation(position):
    direction_vector = np.array([0, 0, 0]) - position
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

def main():
    xarm_version = 7
    world = World(stage_units_in_meters=1.0)
    xarm_task = XArmFollowTarget(xarm_version=xarm_version)
    world.add_task(xarm_task)
    world.reset()

    task_params = world.get_task("xarm_follow_target_task").get_params()
    xarm_name = task_params["robot_name"]["value"]
    target_name = task_params["target_name"]["value"]

    xarm = world.scene.get_object(xarm_name)
    cube = world.scene.get_object(target_name)

    xarm_controller = XArmRMPFlowController(
        name="target_follower_controller", 
        robot_articulation=xarm,
        xarm_version=xarm_version
    )

    articulation_controller = xarm.get_articulation_controller()

    xarm_socket = XArmSocket()
    stream_joints = False

    xarm_socket.start_txsocket()
    xarm_socket.start_rxsocket()

    _safe_zone = [
        (0.3, -0.3, 0.3), # back bottom right 
        (0.6, 0.3, 0.625) # top front left
                        ]
    max_range = 0.7
    min_range = 0.3
    min_height = 0.1

    last_face_seen_timeout = 1
    last_face_seen_time = 0 

    last_rand_target_timeout = 5
    last_rand_target_time = 0 

    start_time = time.time()
    wait_time = 1
    while simulation_app.is_running() and time.time() < start_time + wait_time:
        world.step(render=True)
        if world.is_playing():
            if world.current_time_step_index == 0:
                world.reset()
                xarm_controller.reset()
            
    while simulation_app.is_running():
        world.step(render=True)
        if world.is_playing():            
            observations = world.get_observations()
            actions = xarm_controller.forward(
                target_end_effector_position=observations[task_params["target_name"]["value"]]["position"],
                target_end_effector_orientation=observations[task_params["target_name"]["value"]]["orientation"],
            )
            
            gains = 1e15*np.ones(xarm_version), 1e14*np.ones(xarm_version)
            articulation_controller.set_gains(gains[0], gains[1]) # Solution from Nvidia Live Session 1:23:00
            articulation_controller.apply_action(actions)

            # if xarm_socket.txconn:
            #     try: 
            #         sendData = str(xarm.get_joint_positions().tolist())
            #         res = xarm_socket.txconn.send(sendData.encode())
            #         if res == 0:
            #             print("channel is closed...")
            #     except:
            #         # if sending failed, recreate the socket and reconnect
            #         print("sending failed... closing connection")
            #         xarm_socket.txconn.close()
            #         xarm_socket.txconn = None

            if xarm_socket.txconn and stream_joints:
                try: 
                    sendData = str(_xarm.get_joint_positions().tolist())
                    #print("joints:", sendData)
                    res = xarm_socket.txconn.send(sendData.encode())
                    if res == 0:
                        print("channel is closed...")
                except Exception as e:
                    print(e)
                    # if sending failed, recreate the socket and reconnect
                    print("sending failed... closing connection")
                    xarm_socket.txconn.close()
                    xarm_socket.txconn = None

            current_time = time.time()

            if xarm_socket.face_direction:
                    # update position of target from camera feed            
                cube = _world.scene.get_object("target")
                pos, qrot = cube.get_world_pose()
                print(qrot)
                newpose = [ pos[0]+xarm_socket.z, pos[1]+xarm_socket.dx, pos[2]+xarm_socket.dy]

                newpose[0] = np.clip(newpose[0], _safe_zone[0][0], _safe_zone[1][0])
                newpose[1] = np.clip(newpose[1], _safe_zone[0][1], _safe_zone[1][1])
                newpose[2] = np.clip(newpose[2], _safe_zone[0][2], _safe_zone[1][2])
                print("pose", pos, "->", newpose, end="")
                cube.set_world_pose(np.array(newpose))
                print("set.")

                xarm_socket.dx = None
                xarm_socket.dy = None

                _last_face_seen_time = current_time

            elif rand_target_enabled and ( \
                _xarm_task.task_achieved or \
                current_time > _last_rand_target_time + _last_rand_target_timeout \
                ) and current_time > _last_face_seen_time + _last_face_seen_timeout:
                # set random location
                cube = _world.scene.get_object("target")
                randpos = [
                    np.random.uniform(-1, 1), 
                    np.random.uniform(-1, 1),
                    np.random.uniform(0, 1)
                ]
                range = np.linalg.norm(randpos)
                if range < _min_range:
                    randpos = randpos / np.linalg.norm(randpos) * _min_range
                elif range > _max_range:
                    randpos = randpos / np.linalg.norm(randpos) * _max_range

                randpos = [randpos[0], randpos[1], max(randpos[2], _min_height)]

                updated_quaternion = _get_new_target_orientation(randpos)

                print("Setting new target pos:"+str(randpos))
                cube.set_world_pose(np.array(randpos), updated_quaternion)

                _last_rand_target_time = time.time()

        xarm_socket.cam_to_nose=None
        xarm_socket.face_direction=None

    xarm_socket.shut_down_socket()
    simulation_app.close()


if __name__ == '__main__':
    main()