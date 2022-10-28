import pybullet as p
import time
import pybullet_data
from grippers import Suction
import os
import numpy as np
import jsonpickle

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

# only static dimension shapes
shapes = ['cube', 'cuboid', 'thin_cuboid', 'thinner_cuboid']
valid_rots = {
    'cube': ([0], [0], [0]),
    'cuboid': ([0, np.pi/2], [0, np.pi/2], [0, np.pi/2]),
    'thin_cuboid': ([0, np.pi/2], [0, np.pi/2], [0, np.pi/2]),
    'thinner_cuboid': ([0, np.pi/2], [0, np.pi/2], [0, np.pi/2]),
}

dims = {
    'cube': np.array([0.03, 0.03, 0.03]),
    'cuboid': np.array([0.03, 0.025, 0.06]),
    'thin_cuboid': np.array([0.06, 0.01, 0.025]),
    'thinner_cuboid': np.array([0.03, 0.03, 0.01])
}

obj_num = 3

objPos = np.array([0.5,0,0.1])

while True:
    objects = np.zeros(obj_num, dtype=int)
    obj_types = np.zeros(obj_num, dtype=object)
    pos = np.zeros((obj_num, 3))
    ori = np.zeros((obj_num, 4))

    for i in range(obj_num):
        obj_types[i] = np.random.choice(shapes)
        pos[i] = objPos + np.random.uniform(-.02, .02, 3)
        # eu = [np.random.choice(vr) for vr in valid_rots[obj_types[i]]]
        # print(eu)
        # ori[i] = p.getQuaternionFromEuler(eu)
        ori[i] = noRotation
        
        objects[i] = p.loadURDF(os.path.join(ASSETS_ROOT, f'objects/{obj_types[i]}.urdf'), pos[i], noRotation)

        for i in range (130):
            p.stepSimulation()
            time.sleep(1./240.)

    o01 = 1 if p.getContactPoints(objects[0], objects[1]) else 0
    o12 = 1 if p.getContactPoints(objects[1], objects[2]) else 0
    o02 = 1 if p.getContactPoints(objects[0], objects[2]) else 0

    print(o01, o02, o12, end=' ')

    for i, obj in enumerate(objects):
        _, cori = p.getBasePositionAndOrientation(obj)

        nr = np.array(p.getEulerFromQuaternion(noRotation))
        co = np.array(p.getEulerFromQuaternion(cori))

        diff = abs(nr-co)
        print(np.around(diff, 4), end=' ')
        if (diff > 1e2).any():
            print('NOPE', end=' ')
            continue

    if o01+o02+o12 > 1:
        print('ACCEPTED', end=' ')
        with open('data_points.txt', 'a') as f:
            np.savetxt(f, np.hstack(np.concatenate([pos[i], dims[obj_types[i]]]) for i in range(3)), newline=' ')
            f.write('\n')

        with open('data_setup.txt', 'a') as f:
            # frozen = jsonpickle.encode([obj_types, pos, [p.getEulerFromQuaternion(orr) for orr in ori]])
            frozen = jsonpickle.encode([obj_types, pos])
            f.write(frozen+'\n')

    print()

    for ob in objects:
        p.removeBody(ob)

#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)

# cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)

# print(cubePos,cubeOrn)
# p.disconnect()
