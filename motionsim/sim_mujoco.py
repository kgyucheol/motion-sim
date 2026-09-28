"""MuJoCo reference backend for the Decoupled WBC controller.

Validation only: it runs the policy's own training/sim2mujoco model (`g1_gear_wbc.xml`) so the
controller port can be compared with Motion Creator, and Isaac results can be compared with MuJoCo.
Editing and the physics verdict use Isaac Sim.
"""
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .wbc import ARMS, RobotState, asset_path, load_parameters


class MujocoBackend:
    def __init__(self, parameters=None):
        self.p = parameters or load_parameters()
        path = asset_path('g1_gear_wbc.xml')
        root = ET.parse(path).getroot()
        root.find('compiler').set('meshdir', str(path.parent / 'meshes'))
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        if self.model.nq != 36 or self.model.nu != 29:
            raise ValueError('Unexpected g1_gear_wbc.xml layout')
        self.model.opt.timestep = self.p['simulation_dt']
        self.data = mujoco.MjData(self.model)

    def reset(self, joint_positions):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = self.model.qpos0
        self.data.qpos[7:36] = joint_positions
        mujoco.mj_forward(self.model, self.data)

    def state(self):
        return RobotState(self.data.qpos[:36].copy(), self.data.qvel[3:6].copy(), self.data.qvel[6:35].copy())

    def arm_bias(self):
        return self.data.qfrc_bias[6:35][ARMS].copy()

    def step(self, torques):
        self.data.ctrl[:29] = torques
        mujoco.mj_step(self.model, self.data)


def rollout(controller, backend, seconds):
    """Run the controller from its initial pose. Returns (recording, fell)."""
    backend.reset(controller.initial_joint_positions())
    controller.reset(backend.state())
    for _ in range(int(round(seconds / controller.p['simulation_dt']))):
        backend.step(controller.compute_torques(backend.state(), backend.arm_bias()))
        if not controller.after_step(backend.state()):
            return controller.recording, True
    return controller.recording, False
