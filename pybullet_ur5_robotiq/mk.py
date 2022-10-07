import pybullet as p
import time
import pybullet_data
from grippers import Suction
import os
import numpy as np

ASSETS_ROOT = os.path.abspath("urdf")
obj_ids = {'fixed': [], 'rigid': [], 'deformable': []}
homej = np.array([-1, -0.5, 0.5, -0.5, -0.5, 0]) * np.pi

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.81)

planeId = p.loadURDF(os.path.join(ASSETS_ROOT, 'plane/plane.urdf'))

origin = np.zeros(3)
noRotation = p.getQuaternionFromEuler([0,0,0])

ur5 = p.loadURDF(os.path.join(ASSETS_ROOT, 'ur5/ur5.urdf'), origin, noRotation, 
                useFixedBase=True)
suc = Suction(ASSETS_ROOT, ur5, 9, obj_ids)
n_joints = p.getNumJoints(ur5)
joints = [p.getJointInfo(ur5, i) for i in range(n_joints)]
joints = [j[0] for j in joints if j[2] == p.JOINT_REVOLUTE]

for j, hj in zip(joints, homej):
    p.resetJointState(ur5, j, hj)

shapes = ['cylinder', 'cube', 'cuboid', 'thin_cuboid', 'thinner_cuboid']
valid_rots = {
    'cylinder': [],
    'cube': [],
    'cuboid': [],
    'thinner_cuboid': [],
    'thinner_cuboid': [],
}
obj_num = 3
objects = [0]*obj_num

objPos = np.array([0.5,0,0.2])
for i in range(obj_num):
    # obj_type = np.random.choice(shapes)
    obj_type = shapes[i%len(shapes)]
    pos = objPos+np.random.uniform(-0.05, 0.05, 3)
    ori = p.getQuaternionFromEuler(np.random.uniform(0, 0.01, 3))

    for i in range (100):
        p.stepSimulation()
        time.sleep(1./240.)

    objects.append(
        (p.loadURDF(os.path.join(ASSETS_ROOT, f'objects/{obj_type}.urdf'), pos, ori),
        obj_type)
    )
    obj_ids['rigid'].append(objects[-1][0])

#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
# cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)

# print(cubePos,cubeOrn)
# p.disconnect()