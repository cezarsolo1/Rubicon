"""Open the dog + RoArm-M2 scene in the MuJoCo viewer, arm at its home pose.

Usage: .venv/bin/python hardware/view_dog_arm.py
"""
from pathlib import Path

import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path(str(Path(__file__).with_name("dog_with_arm.xml")))
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, 0)
mujoco.viewer.launch(model, data)
