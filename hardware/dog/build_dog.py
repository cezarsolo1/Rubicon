"""Build an articulated MuJoCo model of the dog from hardware/Full_dog.stl.

The CAD export keeps every part as a separate connected shell, so the script:
  1. splits the mesh into parts (connected components),
  2. recognises the frame, motors, hip brackets, thighs and shins by size/shape,
  3. groups parts into rigid bodies (torso + hip/thigh/shin per leg),
  4. puts a hinge at each motor axis (abduction, hip pitch, knee),
  5. writes meshes/ and dog.xml (dog alone) and ../dog_with_arm.xml (RoArm on the front plate).

CAD frame (mm): x = lateral (x<0 is left), y = long axis (+y is front), z = up.
Full_dog.stl is no longer in the working tree; restore it first with:
  git checkout 72e3c33 -- hardware/Full_dog.stl
Run from the repo root:  .venv/bin/python hardware/dog/build_dog.py
"""
from pathlib import Path

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

HERE = Path(__file__).resolve().parent
SRC = HERE.parent / "Full_dog.stl"
MESH_DIR = HERE / "meshes"
MAX_FACES = 199_000  # MuJoCo's STL limit is 200k faces

STL_DT = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])

# Masses in kg: estimates, replace with real values when known.
MASS = {"torso_frame": 1.2, "motor": 0.35, "hip_bracket": 0.10, "thigh": 0.12, "shin": 0.10}

JOINT = {  # range (rad), position-actuator gain, torque limit (N m)
    "abduct": dict(range=(-0.6, 0.6), kp=30, force=12),
    "hip": dict(range=(-3.14, 3.14), kp=30, force=12),
    "knee": dict(range=(-2.6, 2.6), kp=30, force=12),
}


def load_stl(path):
    with open(path, "rb") as fh:
        fh.read(80)
        n = np.frombuffer(fh.read(4), "<u4")[0]
        return np.frombuffer(fh.read(), STL_DT, count=n)["v"].astype(np.float64)  # (n,3,3) mm


def split(tris):
    v = tris.reshape(-1, 3)
    _, vid = np.unique(np.round(v, 4), axis=0, return_inverse=True)
    f = vid.ravel().reshape(-1, 3)
    r = np.concatenate([f[:, 0], f[:, 1], f[:, 2]])
    c = np.concatenate([f[:, 1], f[:, 2], f[:, 0]])
    nv = f.max() + 1
    _, lab = connected_components(coo_matrix((np.ones(len(r)), (r, c)), shape=(nv, nv)), directed=False)
    tl = lab[f[:, 0]]
    order = np.argsort(tl, kind="stable")
    bounds = np.flatnonzero(np.diff(tl[order])) + 1
    return [tris[idx] for idx in np.split(order, bounds)]


def decimate(tris, max_faces):
    """Vertex-clustering decimation; grows the cell until under max_faces."""
    if len(tris) <= max_faces:
        return tris
    v = tris.reshape(-1, 3)
    cell = 0.3
    while True:
        q = np.floor(v / cell).astype(np.int64)
        _, inv = np.unique(q, axis=0, return_inverse=True)
        inv = inv.ravel()
        rep = np.zeros((inv.max() + 1, 3))
        np.add.at(rep, inv, v)
        rep /= np.bincount(inv)[:, None]
        f = inv.reshape(-1, 3)
        f = f[(f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])]
        _, keep = np.unique(np.sort(f, 1), axis=0, return_index=True)
        f = f[np.sort(keep)]
        if len(f) <= max_faces:
            return rep[f]
        cell *= 1.25


def write_stl(path, tris_m):
    t = tris_m.astype("<f4")
    nrm = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-12
    out = np.zeros(len(t), STL_DT)
    out["n"], out["v"] = nrm, t
    with open(path, "wb") as fh:
        fh.write(b"rubicon dog part".ljust(80, b" "))
        fh.write(np.uint32(len(t)).tobytes())
        fh.write(out.tobytes())


def classify(parts):
    """Return list of (kind, tris, bbox_min, bbox_max)."""
    out = []
    for p in parts:
        v = p.reshape(-1, 3)
        mn, mx = v.min(0), v.max(0)
        s = mx - mn
        n = len(p)
        if n > 50_000 and abs(s[0] - 57) < 2 and abs(s[1] - 52) < 2:
            kind = "motor_abduct"  # axis along y
        elif n > 50_000 and abs(s[0] - 52) < 2:
            kind = "motor_knee" if (mn[2] + mx[2]) / 2 < -50 else "motor_hip"  # axis along x
        elif s[0] > 150:
            kind = "frame"
        elif abs(s[2] - 170) < 3 and s[0] < 12:
            kind = "thigh"
        elif abs(s[2] - 158.5) < 3 and s[0] < 12:
            kind = "shin"
        elif abs(s[0] - 63.4) < 2 and abs(s[1] - 72.7) < 2:
            kind = "hip_bracket"
        else:
            kind = "small"  # screws, output discs, labels
        out.append((kind, p, mn, mx))
    return out


def aabb_dist(p, mn, mx):
    return np.linalg.norm(np.maximum(0, np.maximum(mn - p, p - mx)))


def main():
    tris = load_stl(SRC)
    parts = classify(split(tris))
    counts = {}
    for k, *_ in parts:
        counts[k] = counts.get(k, 0) + 1
    print("parts:", counts)
    assert counts.get("motor_abduct") == 4 and counts.get("motor_hip") == 4 and counts.get("motor_knee") == 4
    assert counts.get("thigh") == 4 and counts.get("shin") == 4 and counts.get("hip_bracket") == 4

    y_mid = (tris[..., 1].min() + tris[..., 1].max()) / 2
    z_foot = tris[..., 2].min()

    def corner(mn, mx):
        c = (mn + mx) / 2
        return ("F" if c[1] > y_mid else "B") + ("L" if c[0] < 0 else "R")

    # body name for each major part
    body_of = {"frame": "torso", "motor_abduct": "torso", "hip_bracket": "hip", "motor_hip": "hip",
               "thigh": "thigh", "motor_knee": "thigh", "shin": "shin"}
    groups = {}  # body name -> list of tris arrays
    majors = []
    for kind, p, mn, mx in parts:
        if kind == "small":
            continue
        b = body_of[kind]
        name = b if b == "torso" else f"{corner(mn, mx)}_{b}"
        groups.setdefault(name, []).append(p)
        majors.append((name, mn, mx))
    # small parts go to the body of the nearest major part
    for kind, p, mn, mx in parts:
        if kind != "small":
            continue
        c = (mn + mx) / 2
        name = min(majors, key=lambda m: aabb_dist(c, m[1], m[2]))[0]
        groups.setdefault(name, []).append(p)

    # joint axes from the motor bodies
    axes = {}
    for kind, p, mn, mx in parts:
        c = (mn + mx) / 2
        if kind == "motor_abduct":
            axes[f"{corner(mn, mx)}_abduct"] = (np.array([c[0], 0.0, c[2]]), np.array([0, 1, 0]))
        elif kind == "motor_hip":
            axes[f"{corner(mn, mx)}_hip"] = (c, np.array([1, 0, 0]))
        elif kind == "motor_knee":
            axes[f"{corner(mn, mx)}_knee"] = (c, np.array([1, 0, 0]))
    # abduction axis point: put it at the hip-pitch y so the hip body frame sits on the leg
    for leg in ("FL", "FR", "BL", "BR"):
        a = axes[f"{leg}_abduct"][0]
        axes[f"{leg}_abduct"] = (np.array([a[0], axes[f"{leg}_hip"][0][1], a[2]]), np.array([0, 1, 0]))

    torso_origin = np.array([0.0, y_mid, 0.0])  # mm, CAD
    origins = {"torso": torso_origin}
    for leg in ("FL", "FR", "BL", "BR"):
        origins[f"{leg}_hip"] = axes[f"{leg}_abduct"][0]
        origins[f"{leg}_thigh"] = axes[f"{leg}_hip"][0]
        origins[f"{leg}_shin"] = axes[f"{leg}_knee"][0]

    MESH_DIR.mkdir(exist_ok=True)
    for name, plist in groups.items():
        t = decimate(np.concatenate(plist), MAX_FACES)
        write_stl(MESH_DIR / f"{name}.stl", (t - origins[name]) / 1000.0)
        print(f"  {name:10s} {len(t):7d} faces")

    def mass(name):
        if name == "torso":
            return MASS["torso_frame"] + 4 * MASS["motor"]
        kind = name.split("_")[1]
        return {"hip": MASS["hip_bracket"] + MASS["motor"], "thigh": MASS["thigh"] + MASS["motor"],
                "shin": MASS["shin"]}[kind]

    def m(v):
        return " ".join(f"{x / 1000:.5g}" for x in v)

    def leg_xml(leg, ind):
        p = lambda b: origins[f"{leg}_{b}"]
        j = lambda n: JOINT[n]
        rng = lambda n: f"{j(n)['range'][0]} {j(n)['range'][1]}"
        i = " " * ind
        return f"""{i}<body name="{leg}_hip" pos="{m(p('hip') - torso_origin)}">
{i}  <joint name="{leg}_abduct" axis="0 1 0" range="{rng('abduct')}"/>
{i}  <geom mesh="{leg}_hip" mass="{mass(leg + '_hip'):.3f}"/>
{i}  <body name="{leg}_thigh" pos="{m(p('thigh') - p('hip'))}">
{i}    <joint name="{leg}_hip" axis="1 0 0" range="{rng('hip')}"/>
{i}    <geom mesh="{leg}_thigh" mass="{mass(leg + '_thigh'):.3f}"/>
{i}    <body name="{leg}_shin" pos="{m(p('shin') - p('thigh'))}">
{i}      <joint name="{leg}_knee" axis="1 0 0" range="{rng('knee')}"/>
{i}      <geom mesh="{leg}_shin" mass="{mass(leg + '_shin'):.3f}"/>
{i}    </body>
{i}  </body>
{i}</body>"""

    legs = ("FL", "FR", "BL", "BR")
    stand_z = (torso_origin[2] - z_foot) / 1000 + 0.005
    mount = np.array([0.0, 904.5, 28.5]) - torso_origin  # top edge of the front plate

    def model_xml(name, with_arm):
        meshes = "\n".join(f'    <mesh name="{n}" file="{n}.stl"/>' for n in sorted(groups))
        arm_asset = '    <model name="roarm" file="../ro-arm/ro-arm.xml"/>\n' if with_arm else ""
        arm_body = (f'      <frame name="arm_mount" pos="{m(mount)}" euler="0 0 1.5708">\n'
                    f'        <attach model="roarm" body="base_link" prefix="arm_"/>\n'
                    f'      </frame>\n') if with_arm else ""
        acts = "\n".join(
            f'    <position name="{leg}_{jn}" joint="{leg}_{jn}" kp="{JOINT[jn]["kp"]}" '
            f'ctrlrange="{JOINT[jn]["range"][0]} {JOINT[jn]["range"][1]}" '
            f'forcerange="-{JOINT[jn]["force"]} {JOINT[jn]["force"]}"/>'
            for leg in legs for jn in ("abduct", "hip", "knee"))
        nq_arm = " 0 0 1.5708 0" if with_arm else ""
        home_q = f"0 0 {stand_z:.4f} 1 0 0 0 " + " ".join(["0"] * 12) + nq_arm
        home_ctrl = " ".join(["0"] * 12) + nq_arm
        return f"""<!-- Generated by hardware/dog/build_dog.py from hardware/Full_dog.stl. Do not edit by hand. -->
<mujoco model="{name}">
  <compiler angle="radian" meshdir="meshes"/>
  <option timestep="0.002"/>

  <default>
    <joint damping="0.5" armature="0.005"/>
    <!-- dog parts touch the floor but not each other -->
    <geom type="mesh" contype="1" conaffinity="0" rgba="0.75 0.75 0.78 1" friction="1 0.01 0.001"/>
  </default>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3"/>
    <global azimuth="60" elevation="-20" offwidth="1280" offheight="960"/>
  </visual>

  <asset>
{arm_asset}{meshes}
    <texture name="grid" type="2d" builtin="checker" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance="0.1"/>
    <texture type="skybox" builtin="gradient" rgb1="0.4 0.5 0.6" rgb2="0 0 0" width="512" height="512"/>
  </asset>

  <worldbody>
    <light pos="0.5 0.5 1.5" dir="-0.3 -0.3 -1" directional="true"/>
    <geom name="floor" type="plane" size="3 3 0.05" material="grid" contype="0" conaffinity="1"/>
    <body name="torso" pos="0 0 {stand_z:.4f}">
      <freejoint name="root"/>
      <geom mesh="torso" mass="{mass('torso'):.3f}"/>
{arm_body}{chr(10).join(leg_xml(leg, 6) for leg in legs)}
    </body>
  </worldbody>

  <actuator>
{acts}
  </actuator>

  <keyframe>
    <key name="home" qpos="{home_q}" ctrl="{home_ctrl}"/>
  </keyframe>
</mujoco>
"""

    (HERE / "dog.xml").write_text(model_xml("rubicon_dog", False))
    (HERE.parent / "dog_with_arm.xml").write_text(
        model_xml("dog_with_arm", True).replace('meshdir="meshes"', 'meshdir="dog/meshes"')
        .replace('file="../ro-arm/ro-arm.xml"', 'file="ro-arm/ro-arm.xml"'))
    print("wrote dog.xml and ../dog_with_arm.xml")


if __name__ == "__main__":
    main()
