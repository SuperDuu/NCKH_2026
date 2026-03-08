import numpy as np
import xml.etree.ElementTree as ET

tree = ET.parse('/home/du/Desktop/NCKH_2026/Software/Ros2_WS/src/ros2_pkg/urdf/urdf/Robot.urdf')
root = tree.getroot()

joints = {}
for j in root.findall('joint'):
    name = j.get('name')
    origin = j.find('origin')
    xyz = np.array([float(v) for v in origin.get('xyz').split()])
    rpy = np.array([float(v) for v in origin.get('rpy').split()])
    joints[name] = {'xyz': xyz, 'rpy': rpy}

def rot_matrix(rpy):
    r, p, y = rpy
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

def get_link_transform(chain):
    pos = np.zeros(3)
    rot = np.eye(3)
    for jname in chain:
        j = joints[jname]
        pos = pos + rot @ j['xyz']
        rot = rot @ rot_matrix(j['rpy'])
    return pos, rot

left_chain = ['base_hip_left_joint', 'hip_hip_left_joint', 'hip_knee_left_joint', 'knee_ankle_left_joint', 'ankle_ankle_left_joint']
right_chain = ['base_hip_right_joint', 'hip_hip_right_joint', 'hip_knee_right_joint', 'knee_ankle_right_joint', 'ankle_ankle_right_joint']

p_l, r_l = get_link_transform(left_chain)
p_r, r_r = get_link_transform(right_chain)

print("Left Ankle Origin (relative to base):", p_l)
print("Right Ankle Origin (relative to base):", p_r)

# Foot bottom relative to ankle is known from STL bounds
foot_bottom_l_local = np.array([-0.0007, -0.0065, -0.0720])
foot_bottom_r_local = np.array([-0.0007, -0.0065, -0.0732])

p_foot_l = p_l + r_l @ foot_bottom_l_local
p_foot_r = p_r + r_r @ foot_bottom_r_local

print("Left Foot Bottom (relative to base):", p_foot_l)
print("Right Foot Bottom (relative to base):", p_foot_r)
print("Difference in Z:", p_foot_l[2] - p_foot_r[2])

