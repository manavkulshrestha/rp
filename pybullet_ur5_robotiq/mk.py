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

'''
(1024, 768, (0.0069799963384866714, -0.30568817257881165, 0.9521061778068542, 0.0, 0.9999756813049316, 0.0021337540820240974, -0.006645859219133854, 0.0, -0.0, 0.9521294236183167, 0.3056955933570862, 0.0, -0.004907201509922743, 0.03555785119533539, -0.906664252281189, 1.0), (0.7499999403953552, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0000200271606445, -1.0, 0.0, 0.0, -0.02000020071864128, 0.0), (0.0, 0.0, 1.0), (-0.9521061778068542, 0.006645859219133854, -0.3056955933570862), (186.13323974609375, 26666.017578125, -0.0), (-6113.7626953125, 42.675079345703125, 19042.5859375),
89.6000747680664, yaw
-17.800016403198242, pitch
1.0, dist
(-0.07796166092157364, 0.005451506469398737, -0.06238798052072525)) target
'''
target = (-0.07796166092157364, 0.005451506469398737, -0.06238798052072525)
dist = 1.0
yaw = 89.6000747680664
pitch = -17.800016403198242
p.resetDebugVisualizerCamera(dist,yaw,pitch,target)

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
shapes = ['cube', 'cuboid', 'thin_cuboid', 'thinner_cuboid']
valid_rots = {
    # 'cylinder': ([0], [0], [0]),
    'cube': ([0], [0], [0]),
    'cuboid': ([0, np.pi/2], [0, np.pi/2], [0, np.pi/2]),
    'thin_cuboid': ([0, np.pi/2], [0, np.pi/2], [0, np.pi/2]),
    'thinner_cuboid': ([0, np.pi/2], [0, np.pi/2], [0, np.pi/2]),
}
obj_num = 3
objects = [0]*obj_num

while True:
    objPos = np.array([0.5,0,0.1])
    for i in range(obj_num):
        obj_type = np.random.choice(shapes)
        # obj_type = shapes[i%len(shapes)]
        pos = objPos+np.random.uniform(-0.02, 0.02, 3)

        possible_rots = valid_rots[obj_type]
        ori = p.getQuaternionFromEuler([np.random.choice(pr) for pr in possible_rots])

        objects.append(
            (p.loadURDF(os.path.join(ASSETS_ROOT, f'objects/{obj_type}.urdf'), pos, ori),
            obj_type)
        )
        obj_ids['rigid'].append(objects[-1][0])

        for i in range (100):
            p.stepSimulation()
            time.sleep(1./240.)
    
    p.removeBody(obj_ids['rigid'][-1])
    p.removeBody(obj_ids['rigid'][-2])
    p.removeBody(obj_ids['rigid'][-3])

# add collision detection to see valid structures

for i in range (100000):
    p.stepSimulation()
    time.sleep(1./240.)

#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)

# cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)

# print(cubePos,cubeOrn)
# p.disconnect()