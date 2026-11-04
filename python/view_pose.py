#!/usr/bin/env python3
"""Show a pose (or a sequence of poses) saved by the MATLAB apps in the MuJoCo viewer.

Kinematics only: for every sample the script sets qpos and calls mj_forward (no
dynamics), just to see what the robot does. The CSV is written by the apps'
"Export CSV for MuJoCo" button (columns: t, pb_x, pb_y, pb_z, qw, qx, qy, qz, then
one column per joint, named as in the URDF).

Usage (from the repository folder):
    python3 python/view_pose.py                              # exports/pose.csv
    python3 python/view_pose.py --csv exports/pose_ik.csv
    python3 python/view_pose.py --csv my_sequence.csv --speed 0.5 --once

Requirements: pip install mujoco numpy
"""
import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

ROOT = Path(__file__).resolve().parent.parent


def find_xml(xml=None):
    """MuJoCo model of the G1: --xml, otherwise urdf/g1_description/g1_29dof.xml."""
    if xml:
        p = Path(xml).expanduser().resolve()
        if not p.is_file():
            raise FileNotFoundError(p)
        return p
    candidates = [ROOT / "urdf" / "g1_description" / "g1_29dof.xml"]
    candidates += sorted(ROOT.rglob("g1_29dof*.xml"))
    for p in candidates:
        if p.is_file():
            return p
    raise FileNotFoundError("g1_29dof.xml not found: copy it into urdf/g1_description or use --xml")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", default=str(ROOT / "exports" / "pose.csv"))
    ap.add_argument("--xml", default=None, help="MuJoCo model (default: urdf/g1_description/g1_29dof.xml)")
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed factor")
    ap.add_argument("--once", action="store_true", help="do not loop the sequence")
    args = ap.parse_args()

    xml = find_xml(args.xml)
    m = mujoco.MjModel.from_xml_path(str(xml))
    d = mujoco.MjData(m)
    floating = m.njnt > 0 and m.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE

    with open(args.csv) as fh:
        header = fh.readline().strip().split(",")
    data = np.loadtxt(args.csv, delimiter=",", skiprows=1, ndmin=2)
    qadr = []
    for name in header[8:]:
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
        if j < 0:
            raise KeyError(f"joint {name} not found in the MuJoCo model")
        qadr.append(m.jnt_qposadr[j])
    qadr = np.array(qadr)
    t = data[:, 0]
    print(f"{args.csv}: {len(t)} poses, {t[-1] - t[0]:.2f} s; model {xml}")

    def set_pose(i):
        d.qpos[:] = 0
        if floating:
            d.qpos[0:3] = data[i, 1:4]
            d.qpos[3:7] = data[i, 4:8]
        d.qpos[qadr] = data[i, 8:]
        mujoco.mj_forward(m, d)

    with mujoco.viewer.launch_passive(m, d) as viewer:
        while viewer.is_running():
            t0 = time.time()
            for i in range(len(t)):
                if not viewer.is_running():
                    break
                set_pose(i)
                viewer.sync()
                wait = (t[i] - t[0]) / args.speed - (time.time() - t0)
                if wait > 0:
                    time.sleep(wait)
            if args.once:
                time.sleep(1.0)
                break


if __name__ == "__main__":
    main()
