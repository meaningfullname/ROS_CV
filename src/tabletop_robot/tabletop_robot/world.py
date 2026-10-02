import math
import random
import time
import numpy as np
import pybullet as p
import pybullet_data


from .config import TARGETS


class MotionStopped(RuntimeError):
    pass


class TabletopWorld:
    def __init__(self, gui=False, seed=42):
        self.client = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1 / 240)
        p.loadURDF('plane.urdf')
        self.robot = p.loadURDF('franka_panda/panda.urdf', useFixedBase=True)
        self.orientation = p.getQuaternionFromEuler([math.pi, 0, 0])
        self.blocks = []
        self.constraint = None
        self.seed = seed
        self.view = p.computeViewMatrix([0.45, 0, 1.2], [0.45, 0, 0], [0, 1, 0])
        self.projection = p.computeProjectionMatrixFOV(60, 1, 0.05, 2)
        for position in TARGETS.values():
            visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.06, 0.045, 0.001],
                                         rgbaColor=[0.5, 0.5, 0.5, 1])
            p.createMultiBody(baseVisualShapeIndex=visual,
                              basePosition=[position[0], position[1], 0.002])
        self.home_joints = [0, -0.6, 0, -2.2, 0, 1.6, 0.8]
        self.reset()

    def reset(self):
        self.release()
        for block in self.blocks:
            p.removeBody(block)
        self.blocks.clear()
        for index, value in enumerate(self.home_joints):
            p.resetJointState(self.robot, index, value)
        p.setJointMotorControlArray(self.robot, range(7), p.POSITION_CONTROL,
                                    targetPositions=self.home_joints, forces=[100] * 7)
        rng = random.Random(self.seed)
        positions = []
        for color in ([0.9, 0.05, 0.05, 1], [0.05, 0.9, 0.05, 1], [0.05, 0.05, 0.9, 1]):
            while True:
                point = (rng.uniform(0.36, 0.57), rng.uniform(-0.13, 0.13), 0.025)
                if all(math.dist(point, other) > 0.085 for other in positions):
                    break
            positions.append(point)
            collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.025] * 3)
            visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.025] * 3, rgbaColor=color)
            self.blocks.append(p.createMultiBody(0.05, collision, visual, point))
        for _ in range(120):
            p.stepSimulation()

    def camera(self, size=320):
        image = p.getCameraImage(size, size, self.view, self.projection,
                                 renderer=p.ER_TINY_RENDERER)
        rgb = np.asarray(image[2], dtype=np.uint8).reshape(size, size, 4)[:, :, :3].copy()
        buffer = np.asarray(image[3]).reshape(size, size)
        depth = (2 * 0.05 / (2 - (2 - 0.05) * buffer)).astype(np.float32)
        return rgb, depth, size / (2 * math.tan(math.pi / 6))

    def move(self, position, stop=lambda: False, realtime=False):
        target = p.calculateInverseKinematics(self.robot, 11, position, self.orientation,
                                               maxNumIterations=200, residualThreshold=0.001)
        for step in range(480):
            if stop():
                raise MotionStopped('Canceled or task deadline exceeded')
            p.setJointMotorControlArray(self.robot, range(7), p.POSITION_CONTROL,
                                        targetPositions=target[:7], forces=[100] * 7)
            p.stepSimulation()
            if realtime:
                time.sleep(1 / 240)
            actual = p.getLinkState(self.robot, 11)[4]
            if step > 60 and math.dist(actual, position) < 0.008:
                return
        raise RuntimeError('End effector did not reach requested position')

    def release(self):
        if self.constraint is not None:
            p.removeConstraint(self.constraint)
            self.constraint = None

    def pick_place(self, pick, place, stop=lambda: False, feedback=lambda stage: None, realtime=False):
        for point in (pick, place):
            if not all(math.isfinite(value) for value in point):
                raise ValueError('Non-finite coordinate')
            if not (0.30 <= point[0] <= 0.65 and abs(point[1]) <= 0.30 and 0.01 <= point[2] <= 0.08):
                raise ValueError('Position outside tabletop workspace')
        block = min(self.blocks, key=lambda body: math.dist(p.getBasePositionAndOrientation(body)[0], pick))
        if math.dist(p.getBasePositionAndOrientation(block)[0], pick) > 0.045:
            raise ValueError('No object at pick position')
        try:
            for stage, point in [('approach', (pick[0], pick[1], 0.22)),
                                 ('descend', (pick[0], pick[1], 0.065))]:
                feedback(stage)
                self.move(point, stop, realtime)
            feedback('grasp')
            link_position, link_orientation = p.getLinkState(self.robot, 11)[4:6]
            inverse = p.invertTransform(link_position, link_orientation)
            object_position, object_orientation = p.getBasePositionAndOrientation(block)
            offset, rotation = p.multiplyTransforms(*inverse, object_position, object_orientation)
            self.constraint = p.createConstraint(self.robot, 11, block, -1, p.JOINT_FIXED,
                                                  [0, 0, 0], offset, [0, 0, 0],
                                                  parentFrameOrientation=rotation)
            for stage, point in [('lift', (pick[0], pick[1], 0.22)),
                                 ('transfer', (place[0], place[1], 0.22)),
                                 ('lower', (place[0], place[1], 0.065))]:
                feedback(stage)
                self.move(point, stop, realtime)
            self.release()
            feedback('retreat')
            self.move((place[0], place[1], 0.3), stop, realtime)
            feedback('park')
            self.move((0.25, 0, 0.45), stop, realtime)
            for _ in range(120):
                p.stepSimulation()
            final = p.getBasePositionAndOrientation(block)[0]
            return math.dist(final[:2], place[:2]) < 0.055 and final[2] < 0.06
        finally:
            self.release()

    def close(self):
        p.disconnect(self.client)
