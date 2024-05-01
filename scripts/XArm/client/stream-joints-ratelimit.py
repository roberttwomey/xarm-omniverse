

"""
# xArm-Python-SDK: https://github.com/xArm-Developer/xArm-Python-SDK
# git clone git@github.com:xArm-Developer/xArm-Python-SDK.git
# cd xArm-Python-SDK
# python setup.py install
"""

import socket
import math
import time
import numpy as np

mysocket = socket.socket()
mysocket.connect(('127.0.0.1',12345))
# mysocket.connect(('192.168.4.5',12345))

try:
    from xarm.tools import utils
except:
    pass
from xarm import version
from xarm.wrapper import XArmAPI

# bSimulate = True

# if (len(sys.argv) > 1):
# 	print(sys.argv)
# 	if sys.argv[1] == "--simulate":
# 		bSimulate = True

arm = XArmAPI('192.168.4.15')
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(0)
time.sleep(0.1)

# omniStartAngle = [112.1, -81.0, -77.8, 33.7, 14.9, -79.3, 90.9]
# frontForwardAngle = [0, 2.5, 0, 37.3, 0, -57.3, -179.0]
# omniStartAngle = [0, 2.5, 0, 37.3, 0, -57.3, -179.0]
omniStartAngle = [-6.838786670630505, -15.210568319335351, 6.733042535674014, 37.296668019488585, 179.41233553276842, 37.60242485277879, 2.225478402831139]
XARM_MAX_JOINT_SPEED = 180.0 #45.0#180.0 # deg/sec

variables = {}
params = {'speed': 100, 'acc': 2000, 'angle_speed':100, 'angle_acc': 1000, 'events': {}, 'variables': variables, 'callback_in_thread': True, 'quit': False}

# Register error/warn changed callback
def error_warn_change_callback(data):#
    if data and data['error_code'] != 0:
        params['quit'] = True
        pprint('err={}, quit'.format(data['error_code']))
        arm.release_error_warn_changed_callback(error_warn_change_callback)
arm.register_error_warn_changed_callback(error_warn_change_callback)


# Register state changed callback
def state_changed_callback(data):
    if data and data['state'] == 4:
        if arm.version_number[0] >= 1 and arm.version_number[1] >= 1 and arm.version_number[2] > 0:
            params['quit'] = True
            pprint('state=4, quit')
            arm.release_state_changed_callback(state_changed_callback)

arm.register_state_changed_callback(state_changed_callback)


# Register counter value changed callback
if hasattr(arm, 'register_count_changed_callback'):
    def count_changed_callback(data):
        if not params['quit']:
            pprint('counter val: {}'.format(data['count']))
    arm.register_count_changed_callback(count_changed_callback)


# Register connect changed callback
def connect_changed_callback(data):
    if data and not data['connected']:
        params['quit'] = True
        pprint('disconnect, connected={}, reported={}, quit'.format(data['connected'], data['reported']))
        arm.release_connect_changed_callback(error_warn_change_callback)

arm.register_connect_changed_callback(connect_changed_callback)


def close_socket(thissocket):
    try:
        thissocket.shutdown(socket.SHUT_RDWR)
        thissocket.close()
        thissocket = None
    except socket.error as e:
        pass
    print("socket is closed")

arm.motion_enable(True)
arm.set_mode(0)
arm.set_state(0)


# move to the omniverse start position
print("moving to start position...", end="")
code = arm.set_servo_angle(servo_id=None, angle=omniStartAngle, speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
# code = arm.set_servo_angle(servo_id=None, angle=omniStartAngle, speed=0.5, mvacc=10.0, wait=True, radius=-1.0, is_radian=False)

print("done.")

if code != 0:
    print("Error moving to start position")
    print('set_servo_angle, code={}'.format(code))
    exit()

arm.set_mode(1)
arm.set_state(0)
time.sleep(0.1)

# start streaming joints from omniverse
count = 0
last_time = time.time()
last_rate = time.time()

try:
    while True and code == 0:
        data = mysocket.recv(1024)
        message = data.decode()
        if message == "Done":
            break
        # print(message)
        try: 
            joints = eval(message)
            # print(joints)
        except:
            continue
        
        # get requested angle in degrees
        
        joints_deg = [math.degrees(joint) for joint in joints]
        
        # get current angle
        curr = arm.get_servo_angle(is_radian=False)    
        joint_diff = np.subtract(joints_deg, curr[1])

        # cap largest joint movement to xarm max angle speed
        
        # should probably do an inner loop with a lerp here instead,
        # stepping to the desired joint position at an appropriate speed.
        max_joint_diff = np.max(joint_diff)
        time_step = last_time-time.time()
        this_max_joint_speed = max_joint_diff/time_step
        # print("max", this_max_joint_speed)

        if abs(this_max_joint_speed) > XARM_MAX_JOINT_SPEED:
            # print(joint_diff)
            joint_diff = np.multiply(joint_diff, (XARM_MAX_JOINT_SPEED/abs(this_max_joint_speed)))
            # print("limited: ", joint_diff)
        
        # joint_diff = [np.clip(a - b, -0.1, 0.1) for a, b in zip(joints_deg, curr[1])]

        new_joints_deg = [a+b for a, b in zip(joint_diff, joints_deg)]
        new_joints = [math.radians(joint) for joint in new_joints_deg]
        # print(joint_diff)

        if arm.connected and arm.state != 4:
            # code = arm.set_servo_angle_j(joints, is_radian=True)
            code = arm.set_servo_angle_j(new_joints, is_radian=True)
            
        count = count + 1
        if count %10 == 0:
            print("moved to", new_joints)
            rate = (time.time()-last_rate)/10.0
            print("----> ",1.0/rate, "hertz")
            last_rate=time.time()

        last_time = time.time()
        
except KeyboardInterrupt:
    print("closing socket...")
    close_socket(mysocket)

print("Isaac Sim Connection Stopped")

print("Go to start pos...", end="")

arm.set_mode(0)
arm.set_state(0)
time.sleep(0.1)

code = arm.set_servo_angle(angle=omniStartAngle, speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)

if code != 0:
    print("Error moving to start position")
    print('set_servo_angle, code={}'.format(code))

print("done.")

if hasattr(arm, 'release_count_changed_callback'):
    arm.release_count_changed_callback(count_changed_callback)
arm.release_error_warn_changed_callback(state_changed_callback)
arm.release_state_changed_callback(state_changed_callback)
arm.release_connect_changed_callback(error_warn_change_callback)
