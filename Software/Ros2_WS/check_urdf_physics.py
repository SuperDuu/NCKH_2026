import xml.etree.ElementTree as ET
import numpy as np

tree = ET.parse('/home/du/Desktop/NCKH_2026/Software/Ros2_WS/src/ros2_pkg/urdf/urdf/Robot.urdf')
root = tree.getroot()

masses = {}
for link in root.findall('link'):
    name = link.get('name')
    inertial = link.find('inertial')
    if inertial is not None:
        mass = float(inertial.find('mass').get('value'))
        inertia_elem = inertial.find('inertia')
        ixx = float(inertia_elem.get('ixx'))
        ixy = float(inertia_elem.get('ixy'))
        ixz = float(inertia_elem.get('ixz'))
        iyy = float(inertia_elem.get('iyy'))
        iyz = float(inertia_elem.get('iyz'))
        izz = float(inertia_elem.get('izz'))
        
        masses[name] = mass
        
        I = np.array([
            [ixx, ixy, ixz],
            [ixy, iyy, iyz],
            [ixz, iyz, izz]
        ])
        
        eigvals, _ = np.linalg.eigh(I)
        
        issue = ""
        if any(eigvals <= 0):
            issue += "NOT POSITIVE DEFINITE! "
        if ixx + iyy < izz or ixx + izz < iyy or iyy + izz < ixx:
            issue += "TRIANGLE INEQUALITY VIOLATION! "
            
        print(f"Link: {name}")
        print(f"  Mass: {mass:.4f}")
        print(f"  Inertia matrix eigenvalues: {eigvals}")
        if issue:
            print(f"  *** ERRORS: {issue} ***")

mass_vals = list(masses.values())
if mass_vals:
    min_m = min(mass_vals)
    max_m = max(mass_vals)
    print(f"\nMass range: {min_m:.4f} to {max_m:.4f} (Ratio: {max_m/min_m:.2f})")
