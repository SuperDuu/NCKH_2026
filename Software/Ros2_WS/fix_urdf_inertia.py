import xml.etree.ElementTree as ET
import re

fname = '/home/du/Desktop/NCKH_2026/Software/Ros2_WS/src/ros2_pkg/urdf/urdf/Robot.urdf'
tree = ET.parse(fname)
root = tree.getroot()

changed = False
for link in root.findall('link'):
    inertial = link.find('inertial')
    if inertial is not None:
        inertia = inertial.find('inertia')
        
        # Zero out off-diagonals
        inertia.set('ixy', '0.0')
        inertia.set('ixz', '0.0')
        inertia.set('iyz', '0.0')
        
        # Ensure diagonals are at least 1e-4 for stability
        ixx = float(inertia.get('ixx')); ixx = max(ixx, 1e-4)
        iyy = float(inertia.get('iyy')); iyy = max(iyy, 1e-4)
        izz = float(inertia.get('izz')); izz = max(izz, 1e-4)
        
        inertia.set('ixx', f"{ixx:.6f}")
        inertia.set('iyy', f"{iyy:.6f}")
        inertia.set('izz', f"{izz:.6f}")
        changed = True

if changed:
    tree.write(fname)
    print("Inertias fixed!")
