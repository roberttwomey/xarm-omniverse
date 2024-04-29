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
import math
import time

import omni
from omni.isaac.core.objects import FixedCuboid


def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.
    
    Input
        :param roll: The roll (rotation around x-axis) angle in radians.
        :param pitch: The pitch (rotation around y-axis) angle in radians.
        :param yaw: The yaw (rotation around z-axis) angle in radians.
    
    Output
        :return qw, qx, qy, qz: The orientation in quaternion [w,x,y,z] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
    return [qw, qx, qy, qz]


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

def get_new_target_orientation2(position):
    x, y, z = position
    # extension = np.linalg.norm(position)
    rotation = math.atan(y/x)
    elevation = math.atan(z/x) 

    roll = 0
    pitch = elevation*-1.0 +(0.5*math.pi) # pitch of target is 90deg - how high target is above ground plane
    yaw = rotation # yaw of target is rotation around vertical axis. front is 0.

    return get_quaternion_from_euler(roll, pitch, yaw)

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
    # print(cube.get_world_pose())

    face_cube = world.scene.add(
        FixedCuboid(
            prim_path="/World/XArm7/link7/face_cube", # The prim path of the cube in the USD stage
            name="face_cube", # The unique name used to retrieve the object from the scene later on
            position=np.array([0.0, 0, 0.4]), # Using the current stage units which is in meters by default.
            scale=np.array([0.06, 0.06, 0.06]), # most arguments accept mainly numpy arrays.
            color=np.array([0, 0, 1.0]), # RGB channels, going from 0-1
        ))
    
    face_cube.set_collision_enabled(False)
    face_cube.set_default_state(position=np.array([0.0, 0, 0.4]))
    face_cube.set_local_pose(translation=np.array([0.0, 0, 0.4]))
    
    xarm_controller = XArmRMPFlowController(
        name="target_follower_controller", 
        robot_articulation=xarm,
        xarm_version=xarm_version
    )

    articulation_controller = xarm.get_articulation_controller()

    xarm_socket = XArmSocket()
    stream_joints = True
    rand_target_enabled = False

    xarm_socket.start_txsocket()
    xarm_socket.start_rxsocket()

    safe_zone = [
        (-0.6, -0.3, 0.1), # back left bottom 
        (0.6, 0.3, 1.0) # front right top
                        ]

    relax_back = True

    max_range = 0.7
    min_range = 0.3
    min_height = 0.1

    # set maximum angle movement here
    last_face_seen_timeout = 0.5 # 1
    last_face_seen_time = 0 

    last_nudge = 0
    nudge_timeout = 2.0                     
    idle = [
        np.random.uniform(-0.2, 0.2), 
        np.random.uniform(-0.2, 0.2),
        np.random.uniform(-0.2, 0.2)
    ]
 
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
                    sendData = str(xarm.get_joint_positions().tolist())
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
                # cube = world.scene.get_object("target")
                pos, qrot = cube.get_world_pose()
                # print(qrot)
                
                # end_pose = omni.usd.get_world_transform_matrix(xarm.end_effector)
                # print("end effector pose", end_pose)

                # rx_rad = np.deg2rad(xarm_socket.rx+90)
                rx_rad = np.deg2rad(xarm_socket.rx)
                ry_rad = np.deg2rad(xarm_socket.ry)
                rz_rad = np.deg2rad(xarm_socket.rz)

                # local_face_rot = get_quaternion_from_euler(rz_rad, rx_rad, ry_rad)
                local_face_rot = get_quaternion_from_euler(ry_rad, rx_rad, rz_rad)
                local_face_pos = [xarm_socket.dy, xarm_socket.dx, xarm_socket.dz]
                face_cube.set_local_pose(translation=np.array(local_face_pos))
                face_cube.set_local_pose(orientation=np.array(local_face_rot))
                
                face_pose, face_rot = face_cube.get_world_pose()

                a = 0.9
                b = 1.0-a
                
                newpose = [ 
                    a*pos[0]+b*face_pose[0], 
                    a*pos[1]+b*face_pose[1],
                    a*pos[2]+b*face_pose[2]
                    ]

                # lerp between current pose and face pose                
                # newrot = [
                #     a*qrot[0]+b*face_rot[0],
                #     a*qrot[1]+b*face_rot[1],
                #     a*qrot[2]+b*face_rot[2],
                #     a*qrot[3]+b*face_rot[3],
                # ]

                # recenter calculated pose to be 0.3m off of the table
                newpose_r = [newpose[0], newpose[1], newpose[2]-0.3]
                
                # get target orientation based off of position relative to center
                newrot = get_new_target_orientation2(newpose_r)
                # newrot = get_new_target_orientation(newpose_r)

                # limits based on minimum and maximum range and elevation
                range = np.linalg.norm(newpose_r)
                if range < min_range:
                    newpose_r = newpose_r / np.linalg.norm(newpose_r) * min_range
                elif range > max_range:
                    newpose_r = newpose_r / np.linalg.norm(newpose_r) * max_range

                newpose = [newpose_r[0], newpose_r[1], newpose_r[2]+0.3]

                # limits based on acceptable box
                # newpose[0] = np.clip(newpose[0], safe_zone[0][0], safe_zone[1][0])
                # newpose[1] = np.clip(newpose[1], safe_zone[0][1], safe_zone[1][1])
                # newpose[2] = np.clip(newpose[2], safe_zone[0][2], safe_zone[1][2])

                # print("pose", pos, "->", newpose, end="")

                # cube.set_world_pose(pos, np.array(newrot))
                cube.set_world_pose(np.array(newpose), np.array(newrot))
                
                # print("set.")

                xarm_socket.dx = None
                xarm_socket.dy = None
                xarm_socket.dz = None
                last_face_seen_time = current_time

            # elif rand_target_enabled and ( \
            #     xarm_task.task_achieved or \
            #     current_time > last_rand_target_time + last_rand_target_timeout \
            #     ) and current_time > last_face_seen_time + last_face_seen_timeout:

            elif rand_target_enabled and ( \
                xarm_task.task_achieved or \
                current_time > last_rand_target_time + last_rand_target_timeout \
                ) and current_time > last_face_seen_time + last_face_seen_timeout:

                # set random location
                # cube = world.scene.get_object("target")

                randpos = [
                    np.random.uniform(-1, 1), 
                    np.random.uniform(-1, 1),
                    np.random.uniform(0, 1)
                ]
                range = np.linalg.norm(randpos)
                if range < min_range:
                    randpos = randpos / np.linalg.norm(randpos) * min_range
                elif range > max_range:
                    randpos = randpos / np.linalg.norm(randpos) * max_range

                randpos = [randpos[0], randpos[1], max(randpos[2], min_height)]

                updated_quaternion = get_new_target_orientation(randpos)

                print("Setting new target pos:"+str(randpos))
                cube.set_world_pose(np.array(randpos), updated_quaternion)

                last_rand_target_time = time.time()

            elif current_time > last_face_seen_time + last_face_seen_timeout:
                # relax back to center
                # cube = world.scene.get_object("target")
                pos, qrot = cube.get_world_pose()

                if relax_back:
                    a = 0.99
                else: 
                    a = 1.0 #0.99
                b = 1.0-a

                # idle = [0.01*np.sin(current_time*5), 0.01*np.sin(current_time*4.75), 0]
                if current_time > last_nudge+nudge_timeout:                     
                    # idle = [
                    #     np.random.uniform(-0.05, 0.05), 
                    #     np.random.uniform(-0.1, 0.1),
                    #     np.random.uniform(-0.1, 0.1)
                    # ]

                    idle = [0, 0, 0]
                    
                    last_nudge = time.time()
                    nudge_timeout = np.random.uniform(2.0, 6.0)

                newpose = [ 
                    a*pos[0]+b*(xarm_task.target_start[0]+idle[0]), 
                    a*pos[1]+b*(xarm_task.target_start[1]+idle[1]),
                    a*pos[2]+b*(xarm_task.target_start[2]+idle[2])
                    ]
                
                newrot = [
                    a*qrot[0]+b*xarm_task.target_start_rot[0],
                    a*qrot[1]+b*xarm_task.target_start_rot[1],
                    a*qrot[2]+b*xarm_task.target_start_rot[2],
                    a*qrot[3]+b*xarm_task.target_start_rot[3],
                ]
                
                newpose[0] = np.clip(newpose[0], safe_zone[0][0], safe_zone[1][0])
                newpose[1] = np.clip(newpose[1], safe_zone[0][1], safe_zone[1][1])
                newpose[2] = np.clip(newpose[2], safe_zone[0][2], safe_zone[1][2])

                # recenter calculated pose to be 0.3m off of the table
                # newpose_r = [newpose[0], newpose[1], newpose[2]-0.3]
                # updated_quaternion = get_new_target_orientation2(newpose_r)                
                # cube.set_world_pose(np.array(newpose), np.array(updated_quaternion))

                cube.set_world_pose(np.array(newpose), np.array(newrot))

        xarm_socket.cam_to_nose=None
        xarm_socket.face_direction=None

    print("closing...")
    # print(cube.get_world_pose())

    xarm_socket.shut_down_socket()
    simulation_app.close()


if __name__ == '__main__':
    main()