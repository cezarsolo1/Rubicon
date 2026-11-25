"""Open the RoArm-M2 model in the MuJoCo viewer, starting at the home pose.

Usage: .venv/bin/python hardware/ro-arm/view.py
"""
from pathlib import Path

import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path(str(Path(__file__).with_name("ro-arm.xml")))
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
mujoco.viewer.launch(model, data)
