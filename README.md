# G1 kinematics app

Interactive **forward and inverse kinematics** for the Unitree G1 humanoid (29 joints + floating base = 35 DoF), in MATLAB, with the CAD meshes.

* **Forward kinematics** – one slider (or a typed angle) per joint, live 3D view, poses of pelvis, torso, head, hands and feet, centre of mass, collision check.
* **Inverse kinematics** – drag the target of a hand, foot, pelvis or torso and read the 29 joint angles (with their limits). Feet-on-ground or fixed-pelvis mode, centre of mass over the feet, optional self-collision avoidance.
* **Standalone** – no toolboxes, no absolute paths, nothing to compile. The kinematic model is built from the URDF the first time you start the app.
* Save / load poses (`.json`, `.csv`), save the 3D view as a PNG, replay a pose in MuJoCo.

The kinematic core (recursive forward kinematics, Jacobians, prioritised inverse kinematics with selective damping and joint limits, capsule collision model) was validated against the MATLAB Robotics System Toolbox, Pinocchio, MuJoCo and finite differences in the full workspace this app was extracted from.

## Quick start

Requirements: **MATLAB R2022a or newer** (developed and tested on R2025b, Ubuntu 22.04). No toolboxes are needed.

```bash
git clone https://github.com/fedene97/g1_kinematics_app.git
cd g1_kinematics_app
```

```matlab
% in MATLAB, with the repository folder as current folder (or addpath it first)
g1_app          % launcher with the two apps
g1_app fk       % forward kinematics directly
g1_app ik       % inverse kinematics directly
g1_app test     % 5-second self-test of the non-graphical parts
```

The first start takes a few seconds (the model and the CAD meshes are loaded). The URDF, the MuJoCo model and the meshes live in `urdf/g1_description/` (see [`urdf/README.md`](urdf/README.md)).

## The apps

**Forward kinematics (`g1_gui`)** – tabs for the left leg, right leg, waist, left arm, right arm. Every joint has a slider and a numeric field (degrees, limited to the joint range). The table shows position and roll-pitch-yaw of the TCPs (tool center points) and the centre of mass; the feet are kept on the ground. Buttons: zero pose, bent knees, save pose, load pose, export for MuJoCo.

**Inverse kinematics (`g1_gui_ik`)** – choose the TCP to move (hands, feet, pelvis, torso) and move its target with six sliders (dx, dy, dz, droll, dpitch, dyaw, relative to its starting pose) or type the values. Options:

| Control | Meaning |
|---|---|
| Mode: *Feet on ground* | floating base; both feet are fixed on the ground (highest priority) |
| Mode: *Fixed pelvis* | pelvis fixed; arms, legs and waist follow the active targets |
| Active | the TCP follows its target (moving a slider activates it) |
| Control orientation | track the full pose of the TCP, not only its position |
| CoM over feet | keep the centre of mass above the midpoint between the feet (feet mode) |
| Avoid collisions | add the capsule self-collision task at priority 1 |

The joint table on the right turns red where a joint is at its limit; the status box reports the tracking error of every active TCP, whether the solver converged, and the collision state.

**Display tab (both apps)** – show or hide the CAD meshes, skeleton, collision capsules, TCP frames, and the legs / trunk / arms separately; mesh opacity; camera presets (Front, Side, Top, 3D); *Save image (PNG)*.

**Pose files** – *Save pose* writes the 29 joint angles as `.json` (`{"unit": "deg", "joints": {"left_hip_pitch_joint": 0, ...}}`) or `.csv` (`joint,deg,rad`); *Load pose* reads them back (joints missing from the file stay at zero, values are clipped to the limits). They are plain text, easy to edit or generate from other tools. Files go to `exports/` by default.

The last result of each app is also published in the MATLAB base workspace: `g1_pose` (the output of `g1_fk`) and `g1_ik` (solution, joint angles and names).

## Use the model from your own scripts

```matlab
addpath(genpath('src'));                    % from the repository folder (or run g1_app once)

% forward kinematics: joint angles -> poses
q = struct('left_knee_joint', 60, 'right_knee_joint', 60, ...
           'left_hip_pitch_joint', -30, 'right_hip_pitch_joint', -30, ...
           'left_ankle_pitch_joint', -30, 'right_ankle_pitch_joint', -30);
out = g1_fk(q, 'degrees', true, 'plot', true);
out.right_hand.p                            % position [m] in the world frame
out.left_foot.R                             % orientation
out.com                                     % centre of mass

% inverse kinematics: hold both feet, move the left hand
[mod, C] = g1_model();                      % kinematic model + collision capsules
x  = stato_cin(mod, 'pb', [0; 0; 0.8], 'qj', postura_casa(mod));
K  = cinematica_diretta(mod, x.Rb, x.pb, x.qj);
[Rl, pl] = posa_frame(mod, K, 'left_sole');
[Rr, pr] = posa_frame(mod, K, 'right_sole');
[~,  ph] = posa_frame(mod, K, 'left_wrist_yaw_link');
comp = [compito('frame', 'nome', 'left_sole',  'R', Rl, 'p', pl, 'priorita', 1), ...
        compito('frame', 'nome', 'right_sole', 'R', Rr, 'p', pr, 'priorita', 1), ...
        compito('frame', 'nome', 'left_wrist_yaw_link', 'p', ph + [0.15; 0; 0.1], 'priorita', 3), ...
        compito('postura', 'q', postura_casa(mod), 'peso', 0.02, 'priorita', 5)];
[x2, info] = ik_risolvi(mod, x, comp, struct('flottante', true));
info.convergenza, info.errori'              % converged?  [position; orientation] errors
g1_fk(x2.qj, 'pb', x2.pb, 'Rb', x2.Rb);     % look at the result
```

`help g1_fk`, `help compito` and `help ik_risolvi` document the options. More in [`docs/FUNCTIONS.md`](docs/FUNCTIONS.md).

## Replay a pose in MuJoCo (optional)

*Export CSV for MuJoCo* in either app writes `exports/pose.csv` (or `exports/pose_ik.csv`). To see the robot in that pose:

```bash
pip install mujoco numpy
python3 python/view_pose.py --csv exports/pose_ik.csv
```

This is kinematics only (`qpos` + `mj_forward`, no dynamics), to check what the pose looks like.

## Conventions

* Joint order = URDF order: left leg (hip pitch, roll, yaw, knee, ankle pitch, roll), right leg, waist (yaw, roll, pitch), left arm (shoulder pitch, roll, yaw, elbow, wrist roll, pitch, yaw), right arm.
* Configuration `q = (p_b, R_b, q_j)`: pelvis position, orientation, 29 joint angles. Velocity `ν = [v_b; ω_b; q̇_j]` (35 values) with the base velocities in the base frame (Pinocchio's *local* convention).
* Configuration update: `p_b += R_b dv`, `R_b = R_b exp([dω]×)`, `q_j += dq`.
* World frame: z up. TCP of the feet = centre of the sole; TCP of the hands = the hand frame of the URDF (the wrist yaw link if the rubber-hand frame is absent).
* Units: metres, radians in the code, degrees in the apps and in the pose files.

## Repository layout

```
g1_app.m            launcher
tests/              self-test (g1_app test)
src/app/            apps (g1_gui, g1_gui_ik), g1_fk, g1_model, g1_setup, 3D view, pose I/O
src/model/          URDF reader and kinematic model
src/kinematics/     recursive forward kinematics, Jacobians, centre of mass
src/ik/             task definition and prioritised inverse kinematics
src/collision/      capsule collision model fitted to the meshes, distances
src/utils/          SO(3) helpers, CSV writer, ...
urdf/g1_description/  URDF, MuJoCo model and meshes (Unitree)
python/view_pose.py   MuJoCo viewer for exported poses
exports/            files written by the apps (not tracked by git)
```

The names of most internal functions are Italian (they come from the original workspace): `cinematica_diretta` = forward kinematics, `posa_frame` = pose of a frame, `compito` = task, `postura_casa` = home posture, `baricentro` = centre of mass. Apps, documentation and the user-facing functions (`g1_*`) are in English; the glossary in [`docs/FUNCTIONS.md`](docs/FUNCTIONS.md) maps the rest.

## Limitations

* Kinematics only: no dynamics, no contact forces. The collision model is a set of capsules fitted to the meshes (an approximation: it agrees with the mesh-based MuJoCo contacts on about 90 % of random poses).
* The ankle is modelled by the pitch and roll joints of the URDF. The real G1 drives them with two motors through a linkage, which this app does not model.
* Joint limits are those of the URDF; check them against your robot before using the angles on hardware.

## License

The code is released under the MIT license (see [`LICENSE`](LICENSE)). The URDF, the MuJoCo model and the meshes in `urdf/g1_description/` are by Unitree Robotics and keep their own license, see [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md).
