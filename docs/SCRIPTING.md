# Using the model from MATLAB scripts

Start from the repository folder (or add it to the path and run `g1_app` once):

```matlab
addpath(genpath('src'));
```

## Forward kinematics

```matlab
q = struct('left_knee_joint', 60, 'right_knee_joint', 60, ...
           'left_hip_pitch_joint', -30, 'right_hip_pitch_joint', -30, ...
           'left_ankle_pitch_joint', -30, 'right_ankle_pitch_joint', -30);
out = g1_fk(q, 'degrees', true, 'plot', true);
out.right_hand.p                            % position [m] in the world frame
out.left_foot.R                             % orientation
out.com                                     % centre of mass
```

`help g1_fk` lists the options and the output fields (pelvis, torso, head, hands, feet, joints, com, collisions).

## Inverse kinematics: hold both feet, move the left hand

```matlab
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

`help compito` and `help ik_risolvi` document the tasks and the solver options.

The last result of each app is also in the MATLAB base workspace: `g1_pose` (the output of `g1_fk`) and `g1_ik` (solution, joint angles and names).

## Replay a pose in MuJoCo (optional)

*Export CSV for MuJoCo* in either app writes `exports/pose.csv` (forward kinematics) or `exports/pose_ik.csv` (inverse kinematics). To see the robot in that pose:

```bash
pip install mujoco numpy
python3 python/view_pose.py --csv exports/pose_ik.csv
```

Kinematics only (`qpos` + `mj_forward`, no dynamics). On macOS use `mjpython`.

## Conventions

* Joint order = URDF order: left leg (hip pitch, roll, yaw, knee, ankle pitch, roll), right leg, waist (yaw, roll, pitch), left arm (shoulder pitch, roll, yaw, elbow, wrist roll, pitch, yaw), right arm.
* Configuration `q = (p_b, R_b, q_j)`: pelvis position, orientation, 29 joint angles. Velocity `ν = [v_b; ω_b; q̇_j]` (35 values) with the base velocities in the base frame (Pinocchio's *local* convention).
* Configuration update: `p_b += R_b dv`, `R_b = R_b exp([dω]×)`, `q_j += dq`.
* World frame: z up. TCP (tool center point) of the feet = centre of the sole; of the hands = the hand frame of the URDF (the wrist yaw link if the rubber-hand frame is absent).
* Units: metres and radians in the code; degrees in the apps and in the pose files.

## Repository layout

```
g1_app.m              launcher
tests/                self-test (g1_app test)
src/app/              apps (g1_gui, g1_gui_ik), g1_fk, g1_model, g1_setup, 3D view, pose I/O
src/model/            URDF reader and kinematic model
src/kinematics/       recursive forward kinematics, Jacobians, centre of mass
src/ik/               task definition and prioritised inverse kinematics
src/collision/        capsule collision model fitted to the meshes, distances
src/utils/            SO(3) helpers, CSV writer, ...
urdf/g1_description/  Unitree package: URDF, MuJoCo model, meshes (the app uses g1_29dof.urdf, g1_29dof.xml, meshes/)
python/view_pose.py   MuJoCo viewer for exported poses
exports/              files written by the apps (not tracked by git)
```

The kinematic core (recursive forward kinematics, Jacobians, prioritised inverse kinematics with selective damping and joint limits, capsule collision model) was validated against the MATLAB Robotics System Toolbox, Pinocchio, MuJoCo and finite differences in the workspace this app was extracted from.
